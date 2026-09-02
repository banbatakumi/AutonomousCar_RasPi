"""RL 訓練用の軽量シム環境 — `VehicleModel` + `Course` + `VirtualLidar` + `ScanAssembler` を
直結するだけで、`SimLink`/`VirtualStm32`（UART フレーミングの模擬）は一切通さない。

`sim/bench.py` は「実機と同じ変換コードを通す」ことを設計の核にしているが、あれは
**評価対象の planner が実機と同じ点群の癖を見られること**が目的で、UART のバイト列
往復そのものに価値があるわけではない。RL の訓練は毎エピソード・毎ステップ数百万回
回るので、そこにシリアライズのオーバーヘッドを持ち込む理由が無い。

一方で `VirtualLidar`（欠測・飽和・鏡像反転のある「意地悪な」点群）と `ScanAssembler`
（実機と全く同じ変換コード）はそのまま使う。**瞬間レイキャストをそのまま観測にすると、
訓練時に見る点群と推論時（`sim.bench`・実機）に見る点群の癖がずれる**ため。

`gymnasium` には依存しない（`sim/` を軽量に保つ方針）。`ml_lidar/env.py` 側で
`gymnasium.Env` としてラップする。
"""

from __future__ import annotations

import math
import threading
from dataclasses import replace
from typing import Callable

import numpy as np

from raspi.auto.base import scan_window
from raspi.msgs import LIDAR_C_SATURATED_M, Scan, ScanAssembler

from .course import Course
from .lidar import VirtualLidar
from .params import SimParams
from .raceline import compute_raceline_offsets, compute_speed_profile
from .vehicle import DriveInput, VehicleModel, VehicleSpec

__all__ = ["SimE2EEnv", "SCAN_DIM", "OBS_DIM"]

NS = 1_000_000_000
_STEP_NS = int(0.1 * NS)          #: LiDAR は 10Hz 回転なので、1ステップ=1回転に揃える
_PRIME_TRIES = 5                  #: reset() 直後にスキャンが1周完成するまで待つ最大回数
#: `vehicle.step()` を呼ぶ際のサブステップ幅 [s]。`VehicleModel._pop_delayed`
#: （`sim/vehicle.py`）の操舵むだ時間キューは「1step=1エントリ」方式のため、
#: `_STEP_NS`(=100ms)を1回で積分すると`dead_time_s`（ドメインランダム化で
#: 0.015〜0.095s）の値によらず実効遅延が常に100msに量子化されてしまう
#: （2026-09-02、コード読解＋手計算シミュレーションで確認）。10ms刻みに分割し、
#: 遅延の解像度を上げる。**衝突判定・LiDAR生成はこのサブステップ化の対象外**
#: （従来通り`_STEP_NS`境界でのみ行う。ここで変えるのは操舵むだ時間の精度だけ）
_DYNAMICS_SUBSTEP_S = 0.01

#: `telemetry_node._cmd_pump`（`raspi/nodes/telemetry_node.py` `CMD_PUB_HZ=50`）+
#: `io_node`の`COMMAND`送信タイマ（`raspi/nodes/io_node.py` `COMMAND_HZ=100`）による
#: 平均待ち時間の見積もり（各周期の半分ずつを合算）。**この環境は`VehicleModel`を
#: 直結しており、`telemetry_node`/`io_node`を一切経由しない**（モジュールdocstring
#: 参照）ため、`config/vehicle.toml`の`dead_time_s`がサーボ単体の値（`tools/sysid`を
#: `steer_cmd_echo`基準に直した後の値）になると、実運用（`raspi/auto/e2e_lidar.py`は
#: 他のplanner同様`planning_node`経由でこのパイプライン遅延を実際に経験する）より
#: 速い応答を前提に訓練してしまう。既知のコード定数から導出した固定値なので、
#: 実車計測は不要
_CMD_PUB_HZ = 50
_COMMAND_HZ = 100
PIPELINE_DEAD_TIME_S = 0.5 / _CMD_PUB_HZ + 0.5 / _COMMAND_HZ   # ≈0.015s

#: `scan_window(scan, 360.0, ...)` が返す点数（-180°〜+180°の361点、fov=360固定の不変量）
SCAN_DIM = 361
#: 観測ベクトルの次元 = 点群 + 自車速度1個 + 現在の平滑化後ステア角1個。
#: **`ml_lidar/env.py`・`export_onnx_rl.py`・`raspi/auto/e2e_lidar.py`と揃えること**
#: （2026-09-02追加、ステア角: 方策が「今どれだけ舵を切っている状態か」を知らずに
#: 次の指令を出していた設計の穴への対応。`raspi/auto/e2e_lidar.py`の`plan()`も
#: 同じ`self._steer`——`steer_tau`一次遅れフィルタ後の実ステア角——を持っており、
#: 学習側の`self._steer`（下記`step()`参照）と同じ量なので新規state無しで追加できる）
OBS_DIM = SCAN_DIM + 2


def _stm_us(t_ns: int) -> int:
    """`VirtualLidar.poll()` が要求する ns→us 変換。訓練では STM32 時計のドリフトは
    どうでもよいので単純な単位変換で済ませる。"""
    return t_ns // 1_000


class _CenterlineProgress:
    """真値の中心線 `(N,3)` への最近傍点から、弧長方向の進捗と横偏差を求める。

    `raspi/nav/centerline.py` の `Centerline`（法線・道幅の最適化用データ構造。
    SLAM で作った地図が前提）とは別物。ここではコース生成時点で分かっている
    真値の中心線に対する素朴な最近傍射影だけを行う。**「最近傍点までの距離」を
    横偏差の近似として使う**（真の垂線ではなく最近傍点までの直線距離）——
    中心線の点間隔が細かければ（`sim/random_course.py`・`sim/track.py` とも
    数cm〜10cm間隔）両者の差は無視できる。

    :param width: `Course.width`（スカラーまたは`centerline_xyyaw`と同じ長さの
        配列）。配列の場合は最近傍点の位置ごとに道幅が変わる（`narrow`/`obstacle`
        アーキタイプ、`sim/random_course.py`参照）ので、`update()`が最近傍点での
        値を返す。`None`は既定1.0m相当として扱う（旧来の挙動と同じ）
    """

    def __init__(self, centerline_xyyaw: np.ndarray,
                width: float | np.ndarray | None = None) -> None:
        self._xy = centerline_xyyaw[:, :2]
        loop = np.vstack([self._xy, self._xy[:1]])
        seg_len = np.hypot(*np.diff(loop, axis=0).T)
        self._arc = np.concatenate([[0.0], np.cumsum(seg_len)])[:-1]
        self._total = float(seg_len.sum())
        self._prev = 0.0
        #: 直近の最近傍点インデックス（窓探索の起点。`reset()`で全探索により初期化）
        self._prev_i = 0
        if isinstance(width, np.ndarray):
            self._half_w = width / 2.0
        else:
            self._half_w = (width if width is not None else 1.0) / 2.0

    def _nearest_full(self, x: float, y: float) -> tuple[int, float]:
        """全点 O(N) 探索。`_nearest()` の初回・フォールバック用。"""
        d2 = (self._xy[:, 0] - x) ** 2 + (self._xy[:, 1] - y) ** 2
        i = int(np.argmin(d2))
        return i, float(d2[i])

    def _nearest(self, x: float, y: float) -> tuple[float, float, int]:
        """`_prev_i` 近傍の窓だけを探す（進捗はほぼ単調なので前回位置の近くに
        あるはず）。RL訓練で毎stepの呼び出しがO(N)のままだと数百万回分の
        コストになるため、O(1)アモータイズに落とす。**窓の端に最近傍が
        張り付いた場合は全探索にフォールバック**——コース位置のワープ
        （エピソード開始・急激な補正）で窓の外に真の最近傍があるケースの
        安全策。
        """
        n = len(self._xy)
        half_window = max(4, n // 20)
        idx = (self._prev_i + np.arange(-half_window, half_window + 1)) % n
        d2 = (self._xy[idx, 0] - x) ** 2 + (self._xy[idx, 1] - y) ** 2
        j = int(np.argmin(d2))
        if j == 0 or j == len(idx) - 1:
            i, d2_i = self._nearest_full(x, y)
        else:
            i, d2_i = int(idx[j]), float(d2[j])
        self._prev_i = i
        return float(self._arc[i]), float(math.sqrt(d2_i)), i

    def reset(self, x: float, y: float) -> None:
        i, _ = self._nearest_full(x, y)
        self._prev_i = i
        self._prev = float(self._arc[i])

    def update(self, x: float, y: float) -> tuple[float, float, float, int]:
        """`(弧長方向の進捗 [m], 横偏差 [m], 最近傍点での道幅の半分 [m], 最近傍点の
        インデックス)`。進捗は1周のラップアラウンドを展開する。インデックスは
        `_RacelineProgress`が理想ラインの参照点を引くのに流用する（同じ弧長
        パラメータ化・同じ点間隔の配列なので、2回目のO(N)最近傍探索をせずに済む）。"""
        p, cross, i = self._nearest(x, y)
        d = p - self._prev
        if self._total > 0:
            if d > self._total / 2:
                d -= self._total
            elif d < -self._total / 2:
                d += self._total
        self._prev = p
        half_w = float(self._half_w[i]) if isinstance(self._half_w, np.ndarray) else self._half_w
        return d, cross, half_w, i


class _RacelineProgress:
    """理想ライン(centerlineからのオフセット+目標速度)への追従度。

    `_CenterlineProgress.update()`が返す最近傍点インデックスをそのまま流用する
    （centerlineとracelineは同じ弧長パラメータ化・同じ点間隔の配列なので、
    `_CenterlineProgress`と同じ「最近傍点流用」の近似で2回目のO(N)最近傍探索を
    避ける。急コーナーでオフセットが道幅いっぱいに振れる場合はこの近似の誤差が
    無視できなくなりうる——`raceline_cross`の分布に異常値が出ないか学習評価で
    確認すること）。"""

    def __init__(self, centerline_xyyaw: np.ndarray, offsets: np.ndarray,
                target_speed: np.ndarray) -> None:
        yaw = centerline_xyyaw[:, 2]
        normal = np.column_stack((-np.sin(yaw), np.cos(yaw)))
        self.xy = centerline_xyyaw[:, :2] + offsets[:, None] * normal
        self.target_speed = target_speed

    def at(self, i: int) -> tuple[np.ndarray, float]:
        return self.xy[i], float(self.target_speed[i])


class SimE2EEnv:
    """1エピソード=1コースの周回試行。`reset()`/`step()` は gymnasium と同じ形の
    戻り値にしてあるが、このクラス自体は `gymnasium.Env` を継承しない。

    観測は `(OBS_DIM,)` = 点群`SCAN_DIM`点 + 自車速度1個 + 現在の平滑化後ステア角1個
    （末尾、順に[m/s]・[rad]、生値）。速度を混ぜているのは、点群だけでは「今どれくらい
    の速さで走っているか」が分からず、速度依存の減速判断（例: 速いほど早めに舵を戻す）
    を学びにくいため（2026-08-28、バンビの指摘で追加）。ステア角を混ぜているのは、
    方策が「自分が今どれだけ舵を切っている状態か」を知らないまま`steer_tau`フィルタ
    越しの応答に対して次の指令を出していた設計の穴への対応（2026-09-02追加）。

    ## 横偏差ペナルティは「中心線からの距離」ではなく「道幅の余白を使い切った分」

    以前は`cross_track_weight * abs(cross_track)`で常に中心線への張り付きを要求
    していたため、コーナーの外側から入って内側を突くレーシングラインが**構造的に
    損**になっていた（2026-08-28、バンビの指摘）。`cross_track_margin_frac`だけ
    道幅の半分を「自由に使ってよい余白」として与え、そこを超えた分だけ罰する
    ことで、コース内のどこを通ってもよい自由度を残しつつ壁への接近だけ抑える。
    """

    #: カリキュラム学習（`set_curriculum_progress`）の`progress=0`（学習序盤）で
    #: 使う`mu_range`。実測中心値(0.454)の近傍だけに絞り、極端に低いグリップでの
    #: 「避けようのない滑走」を序盤は経験させない（2026-09-02追加）
    _CURRICULUM_EASY_MU_RANGE = (0.40, 0.55)

    def __init__(self, courses: list[Course] | None = None, *,
                course_fn: Callable[[np.random.Generator], Course] | None = None,
                spec: VehicleSpec | None = None,
                max_steps: int = 2000, max_range: float = LIDAR_C_SATURATED_M,
                max_speed: float = 1.5,
                collision_penalty: float = -5.0, progress_weight: float = 1.0,
                cross_track_weight: float = 0.5, cross_track_margin_frac: float = 0.5,
                speed_weight: float = 0.1,
                raceline_weight: float = 0.3, raceline_tolerance_m: float = 0.08,
                speed_match_weight: float = 0.3,
                steer_tau: float = 0.10, steer_rate_weight: float = 0.2,
                slip_weight: float = 0.2,
                start_jitter_m: float = 0.03,
                start_jitter_rad: float = math.radians(5), sim_params: SimParams | None = None,
                randomize_lidar: bool = True,
                lidar_noise_sigma_range: tuple[float, float] = (0.0, 0.03),
                lidar_noise_rel_range: tuple[float, float] = (0.0, 0.015),
                lidar_drop_rate_range: tuple[float, float] = (0.0, 0.06),
                lidar_sector_drop_rate_range: tuple[float, float] = (0.0, 0.015),
                randomize_dynamics: bool = True,
                mu_range: tuple[float, float] = (0.28, 0.65),
                tau_steer_s_range: tuple[float, float] = (0.35, 0.75),
                dead_time_s_range: tuple[float, float] = (0.0, 0.08),
                tau_speed_s_range: tuple[float, float] = (0.08, 0.30),
                rolling_resistance_range: tuple[float, float] = (0.15, 0.6),
                seed: int = 0) -> None:
        """:param courses: 固定のコース群から毎エピソードランダムに選ぶ（`ml_lidar/watch.py`
            のように同じコースを繰り返し見せたいときはこちら）。
        :param course_fn: 指定すると、`courses` の代わりに**毎エピソード呼んで新しい
            コースを作る**（`ml_lidar/train_rl.py`。固定プールへの過学習を避けるため、
            訓練では実質無限のコース多様性を持たせたい）。`courses`と排他ではないが
            両方指定した場合は`course_fn`を優先する。**必ずこの環境自身の`self.rng`を
            引数で受け取って使うこと**（`generate_random_course`がそのままこの形）。
            外部で持った別のRNGを閉じ込めると、`reset(seed=...)`で同じseedを渡しても
            同じコースにならず、gymnasiumの決定性チェック
            （`check_env`の`check_step_determinism`）に落ちる
        :param randomize_lidar: `True`なら毎エピソード`sim_params`のLiDARノイズ/欠損率を
            `*_range`からランダムに引き直す（訓練用。センサ条件のドメインランダム化）。
            `False`なら`sim_params`（既定`SimParams()`）を固定で使う（評価用。条件を
            揃えて比較したいので`ml_lidar/train_rl.py`の`make_eval_env`はこちらを使う）
        :param randomize_dynamics: `True`なら毎エピソード`spec`の`[dynamics]`
            パラメータ（`mu`・`tau_steer_s`・`dead_time_s`・`tau_speed_s`・
            `rolling_resistance`）を`*_range`からランダムに引き直す（2026-08-31追加。
            `randomize_lidar`と同じ考え方——個体差・床面差・バッテリー電圧などの
            条件ゆれに対して頑健にするため、特定の1点には過学習させない）。
            幾何・質量（実測確定済み）は変えない。`False`なら`spec`をそのまま
            固定で使う（評価用。`make_eval_env`は`randomize_lidar`と同様こちらを使う）。
            **2026-09-01: `mu`・`tau_steer_s`・`tau_speed_s`はシステム同定タブでの
            実測が済み（`config/vehicle.toml`参照）、各レンジは実測値を中心に
            ±30〜50%程度取り直した**（`tau_steer_s`実測0.539は旧レンジ(0.05, 0.25)を
            全く含んでおらず、訓練が実車より大幅に速い操舵応答を前提にしていた
            ——旋回時の挙動が実車と乖離する主因の一つだったと見られる）。
            `dead_time_s`・`rolling_resistance`は未実測のまま（`rolling_resistance`は
            この学習ループが常に`armed=True`の速度指令モードで駆動するため測っても
            反映されない）。`mu`の下限0.28は`sim/vehicle.py`の摩擦円連成
            （2026-09-01追加。加減速中は`a_lat_max`がさらに絞られる）も踏まえた
            見積もりで、他レンジ同様に**未検証**。**`tau_steer_s_range`は
            `tools/sysid/fit.py`を`steer_cmd_echo`基準（サーボ単体の遅れ）に
            直す前の、Piパイプライン遅延混入込みの実測0.539sを中心に取った
            レンジのまま——`fit.py`修正後に実車を録り直したら、新しい実測値を
            中心に再度取り直すこと**。`dead_time_s_range`は`PIPELINE_DEAD_TIME_S`
            を別途加算するため、こちらはサーボ単体のむだ時間だけを表すレンジ
            （0を含めているのはそのため）
        :param cross_track_margin_frac: 道幅の半分のうち、ペナルティ無しで自由に
            使ってよい割合。既定0.5＝道幅1.0mのコースなら中心線から±0.25mは
            ノーペナルティ、そこから壁（±0.5m）までの残り±0.25mだけ
            `cross_track_weight`で罰する。0にすると常に中心線への距離を罰する
            旧来の挙動に戻る
        :param speed_weight: 毎ステップ`speed/max_speed`に掛けて加算する速度ボーナス
            （2026-08-30、v5評価でバンビが指摘した「衝突はしないが速度・ライン取りが
            消極的」への対応で追加）。`progress`（弧長方向の移動量）も間接的に速度と
            相関するが、典型的な`progress`報酬（0.1〜0.15/step程度）に対して
            `collision_penalty=-5.0`は30〜50ステップ分に相当し、方策が「速く走る
            リスク」を過大評価して速度を抑える方向に偏りやすい。この項は`progress`
            と独立に速度そのものへ価値を持たせ、相対的に`collision_penalty`の
            重みを弱めて積極的な走行を後押しする狙い。**2026-09-01: 既定値を0.3→0.1に
            下げた**——曲率を一切考慮しない一律の速度ボーナスだったため、コーナー前で
            減速する動機を弱めていた可能性がある（v8の実車評価で「衝突はしないが
            綺麗なライン取りができない」との指摘）。主たる速度整形は曲率考慮済みの
            `speed_match_weight`に委譲し、これは「動きを止めない」ための小さな
            下駄として残す
        :param raceline_weight: 理想ライン（`sim/raceline.py`の
            `compute_raceline_offsets`が道幅内で曲率を最小化した参照軌道）からの
            横偏差のうち、`raceline_tolerance_m`を超えた分に掛ける罰則の重み
            （2026-09-01追加。Trajectory-Aided Learning、Bosello et al.
            arXiv:2306.07003に倣い、学習時の報酬にだけ理想ラインを組み込む——
            推論はこれまで通りLiDAR+速度のみのE2Eのまま）。`cross_track_weight`
            （道幅の余白を使い切った分への罰則、壁への安全弁）とは独立に働く。
            **初期値0.3は未検証**
        :param raceline_tolerance_m: 理想ラインへの追従で許容する誤差 [m]。
            `cross_track_margin_frac`と同じ「許容帯パターン」——理想ラインぴったり
            を要求せず、`_RacelineProgress`の最近傍点流用による近似誤差ぶんの
            余裕を持たせる
        :param speed_match_weight: 理想ライン上の目標速度（`compute_speed_profile`。
            曲率に応じてグリップ限界まで減速・加速する）とのズレの小ささに応じて
            加算するボーナス。`speed_weight`が速度の絶対値だけを見るのに対し、
            こちらは「今の位置の曲率にふさわしい速度か」を見る——コーナー前で
            早めに減速する動機を直接与える。**初期値0.3は未検証**
        :param steer_tau: 舵指令に掛ける一次遅れの時定数 [s]。`raspi/auto/e2e_lidar.py`
            の`steer_tau`ParamSpec（既定0.10）と同じ仕組み・同じ既定値を学習側にも
            適用する（2026-08-29、バンビの「舵が不安定」報告の診断を受けて追加）。
            以前は方策の生出力をそのまま車両へ渡していたため、**推論側だけに付いていた
            平滑化フィルタの分だけ学習時と実行時で応答が食い違っていた**
            （方策は「自分の出力が即座に反映される」前提で学習していたのに、実際は
            なまった値が反映される）。0にすると平滑化なし（旧来の挙動）
        :param steer_rate_weight: 平滑化後の舵角が1ステップで変化した量`|Δsteer|`に
            掛ける罰則の重み。`steer_tau`によるなまりだけでは、方策自身が滑らかな
            出力を選ぶ動機にはならない（急な生出力を出しても、なまった後の見た目は
            滑らかに見えてしまう）ため、実際に車両へ伝わる舵角の変化量そのものを
            直接罰することで「滑らかに操舵した方が得」という圧力を報酬に持たせる
        :param slip_weight: `VehicleModel.slip_frac`（要求向心加速度がグリップ限界
            `mu*g`を超えた比率）に掛ける罰則の重み（2026-08-31、バンビの「高速旋回で
            滑る設計になっているか」という指摘への対応で追加）。グリップ限界の
            クランプ自体（`sim/vehicle.py`）はコーナー前の減速を学ぶ動機を与える
            狙いで既に入っていたが、罰則が無いため衝突するまで方策が気づけなかった。
            **初期値0.2は未検証**——`steer_rate_weight`と同程度のオーダーで見積もった
            だけで、学習結果を見て調整が要る可能性がある

        最大舵角は独立した引数を持たない——**`spec.max_steer`（`config/vehicle.toml`の
        車両物理限界）をそのまま使う**（2026-08-28、バンビの指示。自動運転planner側の
        `max_steer`ParamSpec全廃と同じ方針を訓練環境にも揃えた。テストで別の値を
        試したいときは`spec=VehicleSpec(max_steer=...)`を渡すこと）。
        """
        if courses is None and course_fn is None:
            raise ValueError("courses か course_fn のどちらかは要ります")
        self.courses = courses
        self.course_fn = course_fn
        # ★カリキュラム学習用（2026-09-02追加）。`mu_range`を直接書き換えるので、
        # コンストラクタで渡された値を「最終到達点」として別に覚えておく
        # （`set_curriculum_progress`docstring参照）
        self._mu_range_full = mu_range
        self.spec = spec or VehicleSpec.load()
        self.max_steps = max_steps
        self.max_range = max_range
        self.max_speed = max_speed
        self.cross_track_margin_frac = cross_track_margin_frac
        self.collision_penalty = collision_penalty
        self.progress_weight = progress_weight
        self.cross_track_weight = cross_track_weight
        self.speed_weight = speed_weight
        self.raceline_weight = raceline_weight
        self.raceline_tolerance_m = raceline_tolerance_m
        self.speed_match_weight = speed_match_weight
        self.steer_tau = steer_tau
        self.steer_rate_weight = steer_rate_weight
        self.slip_weight = slip_weight
        self.start_jitter_m = start_jitter_m
        self.start_jitter_rad = start_jitter_rad
        self.sim_params = sim_params or SimParams()
        self.randomize_lidar = randomize_lidar
        self.lidar_noise_sigma_range = lidar_noise_sigma_range
        self.lidar_noise_rel_range = lidar_noise_rel_range
        self.lidar_drop_rate_range = lidar_drop_rate_range
        self.lidar_sector_drop_rate_range = lidar_sector_drop_rate_range
        self.randomize_dynamics = randomize_dynamics
        self.mu_range = mu_range
        self.tau_steer_s_range = tau_steer_s_range
        self.dead_time_s_range = dead_time_s_range
        self.tau_speed_s_range = tau_speed_s_range
        self.rolling_resistance_range = rolling_resistance_range
        self.rng = np.random.default_rng(seed)

        self.course: Course | None = None
        self.vehicle: VehicleModel | None = None
        self.lidar: VirtualLidar | None = None
        self.asm: ScanAssembler | None = None
        self._progress: _CenterlineProgress | None = None
        self._raceline: _RacelineProgress | None = None
        self._body: np.ndarray | None = None
        self._last_scan: Scan | None = None
        self._t_ns = 0
        self._steps = 0
        self._steer = 0.0

        # ★raceline先読み・キャッシュ用の状態（2026-09-02追加。`reset()`docstring
        # 参照）。`_raceline_cache`は固定`courses`（`course_fn is None`）のときだけ
        # 使う——手続き生成コース（`course_fn`）は毎回新しいCourseオブジェクトなので
        # キャッシュは常にミスし、無駄にメモリを積むだけになる
        self._raceline_cache: dict[tuple, tuple[np.ndarray, np.ndarray]] = {}
        self._pending_course: Course | None = None
        self._pending_episode_spec: VehicleSpec | None = None
        self._pending_thread: threading.Thread | None = None
        self._pending_result: tuple[np.ndarray, np.ndarray] | None = None
        self._pending_rng: np.random.Generator | None = None

    # ── エピソード管理 ──

    def _draw_course(self) -> Course:
        if self.course_fn is not None:
            return self.course_fn(self.rng)
        return self.courses[int(self.rng.integers(len(self.courses)))]

    def _compute_raceline(self, course: Course,
                          episode_spec: VehicleSpec) -> tuple[np.ndarray, np.ndarray]:
        """理想ラインのオフセット・目標速度プロファイルを計算する純粋関数
        （`course`・`episode_spec`の値だけで決まり、`self.rng`は一切使わない）。
        L-BFGS最適化(`compute_raceline_offsets`)が実測90〜140ms/回と重いため、
        `reset()`はこれを次エピソード分バックグラウンドスレッドで先読みする
        （`self.rng`に触れない純粋関数だからこそ、メインスレッド外で安全に呼べる）。
        """
        vehicle_half_width_m = max(abs(p[1]) for p in self.spec.footprint)
        offsets = compute_raceline_offsets(course.centerline, course.width,
                                           vehicle_half_width_m=vehicle_half_width_m)
        target_speed = compute_speed_profile(
            course.centerline, offsets, mu=episode_spec.mu, max_speed=self.max_speed,
            drive_accel_m_s2=episode_spec.drive_accel_m_s2,
            brake_decel_m_s2=episode_spec.brake_decel_m_s2)
        return offsets, target_speed

    def _raceline_for(self, course: Course,
                      episode_spec: VehicleSpec) -> tuple[np.ndarray, np.ndarray]:
        """固定`courses`向けのメモ化つき`_compute_raceline`。`make_eval_env`は
        circuit/fujiの2コース×固定dynamics(`randomize_dynamics=False`)を評価
        エピソードのたびに（既定30回）使い回すのに、理想ラインは常に同じ結果に
        なるのに毎回L-BFGSを回し直していた（2026-09-02、v9評価コストの見直しで
        発覚）。`course_fn`使用時（手続き生成、`Course`が毎回一意）はキャッシュが
        意味を持たないのでそのまま計算する。
        """
        if self.course_fn is not None:
            return self._compute_raceline(course, episode_spec)
        key = (id(course), episode_spec.mu, episode_spec.drive_accel_m_s2,
              episode_spec.brake_decel_m_s2)
        cached = self._raceline_cache.get(key)
        if cached is None:
            cached = self._compute_raceline(course, episode_spec)
            self._raceline_cache[key] = cached
        return cached

    def reset(self) -> np.ndarray:
        """:実装ノート: `course_fn`使用時（手続き生成コース、`ml_lidar/train_rl.py`の
        訓練用）は、直前の`reset()`が終わった時点で**次エピソード分の
        `course`・`episode_spec`・理想ラインをバックグラウンドスレッドで
        先読み済み**（`self._pending_*`）のはずなので、それをそのまま使う
        （初回=エピソード0だけ先読みが無く同期計算になる）。`self.rng`は
        メインスレッドの`_draw_course()`/`_episode_spec()`だけが触り、
        バックグラウンドスレッドは`_compute_raceline()`という`course`・
        `episode_spec`の値だけで決まる純粋関数しか呼ばないため、`self.rng`を
        2つのスレッドが同時に触ることはない（競合状態が起きない設計）。

        ★`self._pending_rng`（先読みを開始した時点の`self.rng`オブジェクト自体への
        参照）が今の`self.rng`と一致するときだけ先読み結果を使う。`ml_lidar/env.py`の
        `GymSurgeEnv.reset(seed=...)`は`self._env.rng = np.random.default_rng(seed)`
        で`rng`を丸ごと差し替えてから`reset()`を呼ぶ設計（gymnasiumの決定性契約
        `reset(seed=X)`を2回呼べば同じ結果、を満たすため）なので、差し替え後に
        古い`rng`から先読みした結果をそのまま使うと**差し替え前の乱数列に基づく
        コースが返り、決定性が壊れる**（2026-09-02、実装直後の回帰テストで発覚）。
        不一致なら先読み中のスレッドは`join()`だけして結果は捨て、同期的に引き直す。
        """
        if self._pending_course is not None and self._pending_rng is self.rng:
            self.course = self._pending_course
            episode_spec = self._pending_episode_spec
            self._pending_thread.join()
            offsets, target_speed = self._pending_result
            self._pending_course = None
            self._pending_episode_spec = None
            self._pending_thread = None
            self._pending_result = None
            self._pending_rng = None
        else:
            if self._pending_thread is not None:
                self._pending_thread.join()   # rngが差し替わった等で不要になった先読みの後始末
                self._pending_course = None
                self._pending_episode_spec = None
                self._pending_thread = None
                self._pending_result = None
                self._pending_rng = None
            self.course = self._draw_course()
            episode_spec = self._episode_spec()
            offsets, target_speed = self._raceline_for(self.course, episode_spec)

        # コース上のランダムな地点＋小さな横ずれ/向きジッターを開始姿勢にする
        # （domain randomization。固定スタートより少ないエピソード数で
        # コース各所の局所ジオメトリを経験できる）
        i = int(self.rng.integers(len(self.course.centerline)))
        if self.course.obstacles is not None:
            i = self._start_index_away_from_obstacles(i)
        cx, cy, cyaw = self.course.centerline[i]
        jx, jy = self.rng.normal(0.0, self.start_jitter_m, 2)
        jyaw = self.rng.normal(0.0, self.start_jitter_rad)
        start = (float(cx + jx), float(cy + jy), float(cyaw + jyaw))

        self.vehicle = VehicleModel(episode_spec, start)
        self.lidar = VirtualLidar(self.course, episode_spec, self._episode_sim_params(),
                                  seed=int(self.rng.integers(1 << 31)))
        self.asm = ScanAssembler()
        self._body = self.course.body_samples(self.spec.footprint)
        self._progress = _CenterlineProgress(self.course.centerline, self.course.width)
        self._progress.reset(self.vehicle.x, self.vehicle.y)
        # ★理想ラインは今エピソードの`episode_spec`（ドメインランダム化後の`mu`等）で
        # 計算したもの（上の`_raceline_for`/`_pending_result`）を使う——固定`self.spec`
        # を使うと`randomize_dynamics=True`時に今エピソードのグリップ・応答特性と
        # 目標速度プロファイルがズレる（`raceline_weight`docstring参照）
        self._raceline = _RacelineProgress(self.course.centerline, offsets, target_speed)
        self._t_ns = 0
        self._steps = 0
        self._steer = 0.0

        self._last_scan = self._prime_scan()

        # ★次エピソード分の`course`・`episode_spec`・理想ラインをバックグラウンドで
        # 先読みしておく（`course_fn`使用時=手続き生成コースのときだけ。固定`courses`
        # は`_raceline_for`のキャッシュで十分間に合う）。今エピソードの`step()`が
        # 進む間（最大`max_steps`回）に完了すれば、次の`reset()`はブロックしない
        if self.course_fn is not None:
            self._pending_rng = self.rng
            self._pending_course = self._draw_course()
            self._pending_episode_spec = self._episode_spec()
            pending_course, pending_spec = self._pending_course, self._pending_episode_spec

            def _prefetch_worker() -> None:
                self._pending_result = self._compute_raceline(pending_course, pending_spec)

            self._pending_thread = threading.Thread(target=_prefetch_worker, daemon=True)
            self._pending_thread.start()

        return self._obs(self._last_scan)

    def set_curriculum_progress(self, progress: float) -> None:
        """カリキュラム学習（2026-09-02追加）。`progress`（0.0〜1.0）に応じて
        `mu_range`を`_CURRICULUM_EASY_MU_RANGE`→コンストラクタで渡された
        `mu_range`（`self._mu_range_full`）へ線形補間する。`course_fn`が
        `set_progress()`を持つ場合（`sim.random_course.CurriculumCourseFn`）は
        そちらにも同じ`progress`を伝える——固定`courses`使用時（`course_fn is
        None`、評価・観戦用）は該当メソットが無いので何もしない。

        `ml_lidar/train_rl.py`の`CurriculumCallback`が`VecEnv.env_method()`経由で
        各ワーカープロセスの`GymSurgeEnv.set_curriculum_progress()`（`ml_lidar/env.py`）
        から呼ぶ想定。v9の実測で評価rewardの標準偏差が一部の評価回だけ跳ねる
        （＝低グリップ・難コースの組み合わせでだけ大きく崩れる）傾向が見えたことを
        受け、学習序盤は易しい条件に絞ってから徐々に本来の難度分布へ近づける。
        """
        t = max(0.0, min(1.0, progress))
        lo = self._CURRICULUM_EASY_MU_RANGE[0] + t * (self._mu_range_full[0] - self._CURRICULUM_EASY_MU_RANGE[0])
        hi = self._CURRICULUM_EASY_MU_RANGE[1] + t * (self._mu_range_full[1] - self._CURRICULUM_EASY_MU_RANGE[1])
        self.mu_range = (lo, hi)
        set_progress = getattr(self.course_fn, "set_progress", None)
        if set_progress is not None:
            set_progress(t)

    def _episode_sim_params(self) -> SimParams:
        """このエピソードで使う`SimParams`。`randomize_lidar`なら毎回引き直す
        （ドメインランダム化）。固定`self.sim_params`をその場で書き換えず、
        毎回コピーを作る（他エピソード・他ワーカーに影響させないため）。"""
        if not self.randomize_lidar:
            return self.sim_params
        p = self.sim_params.copy()
        p.lidar_noise_sigma_m = float(self.rng.uniform(*self.lidar_noise_sigma_range))
        p.lidar_noise_rel = float(self.rng.uniform(*self.lidar_noise_rel_range))
        p.lidar_drop_rate = float(self.rng.uniform(*self.lidar_drop_rate_range))
        p.lidar_sector_drop_rate = float(self.rng.uniform(*self.lidar_sector_drop_rate_range))
        return p

    def _episode_spec(self) -> VehicleSpec:
        """このエピソードで使う`VehicleSpec`。`randomize_dynamics`なら`[dynamics]`の
        未実測パラメータ（mu・tau_steer_s・dead_time_s・tau_speed_s・
        rolling_resistance）だけをエピソードごとに引き直す（ドメインランダム化）。
        幾何・質量は実測確定済みなので変更しない。`_episode_sim_params`と同じ理由で
        `self.spec`をその場で書き換えず、毎回コピーを作る。

        どちらの分岐でも`dead_time_s`に`PIPELINE_DEAD_TIME_S`を加算する——この
        環境は`telemetry_node`/`io_node`を経由しないため、実運用が実際に
        経験するPi側パイプライン遅延をここで明示的に補わないと、方策が
        実運用より速い応答を前提に学習してしまう（モジュール定数のdocstring参照）。
        """
        if not self.randomize_dynamics:
            return replace(self.spec, dead_time_s=self.spec.dead_time_s + PIPELINE_DEAD_TIME_S)
        return replace(
            self.spec,
            mu=float(self.rng.uniform(*self.mu_range)),
            tau_steer_s=float(self.rng.uniform(*self.tau_steer_s_range)),
            dead_time_s=float(self.rng.uniform(*self.dead_time_s_range)) + PIPELINE_DEAD_TIME_S,
            tau_speed_s=float(self.rng.uniform(*self.tau_speed_s_range)),
            rolling_resistance=float(self.rng.uniform(*self.rolling_resistance_range)),
        )

    def _start_index_away_from_obstacles(self, i: int, *, max_tries: int = 20,
                                         min_dist_m: float = 0.25) -> int:
        """`obstacle`アーキタイプ（`sim/random_course.py`）のコースで、抽選した
        開始地点`i`が障害物にめり込んでいたら引き直す。障害物は数個しかなく、
        `centerline`との重なりも局所的なので、当たらなくなるまで最大`max_tries`回
        引き直すだけで十分（`_hairpin_polygon_xy`の再試行と同じ考え方）。
        `min_dist_m`は障害物半径に足す余裕（ジッター・車体外形ぶん）。
        """
        obstacles = self.course.obstacles
        for _ in range(max_tries):
            cx, cy = self.course.centerline[i, :2]
            d2 = (obstacles[:, 0] - cx) ** 2 + (obstacles[:, 1] - cy) ** 2
            if np.all(d2 > (obstacles[:, 2] + min_dist_m) ** 2):
                return i
            i = int(self.rng.integers(len(self.course.centerline)))
        return i

    def _prime_scan(self) -> Scan:
        """スキャンが最低1周完成するまで、車両を動かさずにLiDAR時計だけ進める。"""
        scan: Scan | None = None
        for _ in range(_PRIME_TRIES):
            self._t_ns += _STEP_NS
            for gen_ns, pkt in self.lidar.poll(self._t_ns, self.vehicle, _stm_us):
                s = self.asm.feed(pkt, gen_ns)
                if s is not None:
                    scan = s
            if scan is not None:
                break
        return scan if scan is not None else Scan()

    # ── 1ステップ ──

    def step(self, action: np.ndarray) -> tuple[np.ndarray, float, bool, bool, dict]:
        """:param action: `[steer, speed]`（物理単位、呼び出し側でクランプ済み前提でも
            ここでも安全側にもう一度クランプする）。`steer`は方策の生出力——実際に
            車両へ渡す前に`steer_tau`の一次遅れを通す（`raspi/auto/e2e_lidar.py`と
            同じ変換。下の`steer_tau`docstring参照）。"""
        steer_cmd = float(np.clip(action[0], -self.spec.max_steer, self.spec.max_steer))
        speed = float(np.clip(action[1], 0.0, self.max_speed))

        dt = _STEP_NS / NS
        tau = self.steer_tau
        alpha = 1.0 if tau <= 1e-3 else 1.0 - math.exp(-dt / tau)
        prev_steer = self._steer
        self._steer += (steer_cmd - self._steer) * alpha
        steer = self._steer

        self.vehicle.apply(DriveInput(armed=True, target_speed=speed, target_steer=steer))
        n_sub = max(1, round(dt / _DYNAMICS_SUBSTEP_S))
        dt_sub = dt / n_sub
        for _ in range(n_sub):
            self.vehicle.step(dt_sub)
        self._t_ns += _STEP_NS

        hit = self.course.collides(self.vehicle.x, self.vehicle.y, self.vehicle.yaw, self._body)
        self.vehicle.note_collision(hit)

        for gen_ns, pkt in self.lidar.poll(self._t_ns, self.vehicle, _stm_us):
            s = self.asm.feed(pkt, gen_ns)
            if s is not None:
                self._last_scan = s

        progress, cross_track, half_w, nearest_i = self._progress.update(self.vehicle.x, self.vehicle.y)
        margin = half_w * self.cross_track_margin_frac
        cross_excess = max(0.0, abs(cross_track) - margin)
        steer_rate = abs(self._steer - prev_steer)
        speed_frac = float(np.clip(self.vehicle.speed, 0.0, self.max_speed)) / self.max_speed
        slip = self.vehicle.slip_frac

        raceline_xy, target_speed = self._raceline.at(nearest_i)
        raceline_dev = math.hypot(self.vehicle.x - raceline_xy[0], self.vehicle.y - raceline_xy[1])
        raceline_excess = max(0.0, raceline_dev - self.raceline_tolerance_m)
        speed_match = max(0.0, 1.0 - abs(self.vehicle.speed - target_speed) / self.max_speed)

        reward = (self.progress_weight * progress - self.cross_track_weight * cross_excess
                 - self.steer_rate_weight * steer_rate + self.speed_weight * speed_frac
                 - self.slip_weight * slip
                 - self.raceline_weight * raceline_excess + self.speed_match_weight * speed_match)

        terminated = False
        if hit:
            reward += self.collision_penalty
            terminated = True

        self._steps += 1
        truncated = self._steps >= self.max_steps

        info = {"cross_track": cross_track, "collided": hit, "progress": progress, "slip": slip,
               "raceline_cross": raceline_dev, "target_speed": target_speed}
        return self._obs(self._last_scan), reward, terminated, truncated, info

    def _obs(self, scan: Scan) -> np.ndarray:
        w = scan_window(scan, 360.0, self.max_range)
        speed = float(np.clip(self.vehicle.speed, 0.0, self.max_speed))
        # `self._steer`は`steer_tau`一次遅れフィルタ後の実ステア角[rad]
        # （`step()`で更新済み。`raspi/auto/e2e_lidar.py`の`self._steer`と同じ量）
        return np.concatenate([np.asarray(w.dist, dtype=np.float32),
                               np.array([speed, self._steer], dtype=np.float32)])
