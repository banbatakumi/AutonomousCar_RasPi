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
from dataclasses import replace
from typing import Callable

import numpy as np

from raspi.auto.base import scan_window
from raspi.msgs import LIDAR_C_SATURATED_M, Scan, ScanAssembler

from .course import Course
from .lidar import VirtualLidar
from .params import SimParams
from .vehicle import DriveInput, VehicleModel, VehicleSpec

__all__ = ["SimE2EEnv", "SCAN_DIM", "OBS_DIM"]

NS = 1_000_000_000
_STEP_NS = int(0.1 * NS)          #: LiDAR は 10Hz 回転なので、1ステップ=1回転に揃える
_PRIME_TRIES = 5                  #: reset() 直後にスキャンが1周完成するまで待つ最大回数

#: `scan_window(scan, 360.0, ...)` が返す点数（-180°〜+180°の361点、fov=360固定の不変量）
SCAN_DIM = 361
#: 観測ベクトルの次元 = 点群 + 自車速度1個。**`ml_lidar/env.py`・`export_onnx_rl.py`・
#: `raspi/auto/e2e_lidar.py`（暗黙に`scan_window`の出力長+1）と揃えること**
OBS_DIM = SCAN_DIM + 1


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
        if isinstance(width, np.ndarray):
            self._half_w = width / 2.0
        else:
            self._half_w = (width if width is not None else 1.0) / 2.0

    def _nearest(self, x: float, y: float) -> tuple[float, float, int]:
        d2 = (self._xy[:, 0] - x) ** 2 + (self._xy[:, 1] - y) ** 2
        i = int(np.argmin(d2))
        return float(self._arc[i]), float(math.sqrt(d2[i])), i

    def reset(self, x: float, y: float) -> None:
        self._prev, _, _ = self._nearest(x, y)

    def update(self, x: float, y: float) -> tuple[float, float, float]:
        """`(弧長方向の進捗 [m], 横偏差 [m], 最近傍点での道幅の半分 [m])`。
        進捗は1周のラップアラウンドを展開する。"""
        p, cross, i = self._nearest(x, y)
        d = p - self._prev
        if self._total > 0:
            if d > self._total / 2:
                d -= self._total
            elif d < -self._total / 2:
                d += self._total
        self._prev = p
        half_w = float(self._half_w[i]) if isinstance(self._half_w, np.ndarray) else self._half_w
        return d, cross, half_w


class SimE2EEnv:
    """1エピソード=1コースの周回試行。`reset()`/`step()` は gymnasium と同じ形の
    戻り値にしてあるが、このクラス自体は `gymnasium.Env` を継承しない。

    観測は `(OBS_DIM,)` = 点群`SCAN_DIM`点 + 自車速度1個（末尾、[m/s]、生値）。
    速度を混ぜているのは、点群だけでは「今どれくらいの速さで走っているか」が
    分からず、速度依存の減速判断（例: 速いほど早めに舵を戻す）を学びにくいため
    （2026-08-28、バンビの指摘で追加）。

    ## 横偏差ペナルティは「中心線からの距離」ではなく「道幅の余白を使い切った分」

    以前は`cross_track_weight * abs(cross_track)`で常に中心線への張り付きを要求
    していたため、コーナーの外側から入って内側を突くレーシングラインが**構造的に
    損**になっていた（2026-08-28、バンビの指摘）。`cross_track_margin_frac`だけ
    道幅の半分を「自由に使ってよい余白」として与え、そこを超えた分だけ罰する
    ことで、コース内のどこを通ってもよい自由度を残しつつ壁への接近だけ抑える。
    """

    def __init__(self, courses: list[Course] | None = None, *,
                course_fn: Callable[[np.random.Generator], Course] | None = None,
                spec: VehicleSpec | None = None,
                max_steps: int = 2000, max_range: float = LIDAR_C_SATURATED_M,
                max_speed: float = 1.5,
                collision_penalty: float = -5.0, progress_weight: float = 1.0,
                cross_track_weight: float = 0.5, cross_track_margin_frac: float = 0.5,
                speed_weight: float = 0.3,
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
            見積もりで、他レンジ同様に**未検証**
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
            重みを弱めて積極的な走行を後押しする狙い。**初期値0.3は未検証**——
            `progress`の典型値と同程度のオーダーになるよう見積もっただけで、
            v6の学習結果を見て調整が要る可能性がある
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
        self.spec = spec or VehicleSpec.load()
        self.max_steps = max_steps
        self.max_range = max_range
        self.max_speed = max_speed
        self.cross_track_margin_frac = cross_track_margin_frac
        self.collision_penalty = collision_penalty
        self.progress_weight = progress_weight
        self.cross_track_weight = cross_track_weight
        self.speed_weight = speed_weight
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
        self._body: np.ndarray | None = None
        self._last_scan: Scan | None = None
        self._t_ns = 0
        self._steps = 0
        self._steer = 0.0

    # ── エピソード管理 ──

    def reset(self) -> np.ndarray:
        if self.course_fn is not None:
            self.course = self.course_fn(self.rng)
        else:
            self.course = self.courses[int(self.rng.integers(len(self.courses)))]

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

        episode_spec = self._episode_spec()
        self.vehicle = VehicleModel(episode_spec, start)
        self.lidar = VirtualLidar(self.course, episode_spec, self._episode_sim_params(),
                                  seed=int(self.rng.integers(1 << 31)))
        self.asm = ScanAssembler()
        self._body = self.course.body_samples(self.spec.footprint)
        self._progress = _CenterlineProgress(self.course.centerline, self.course.width)
        self._progress.reset(self.vehicle.x, self.vehicle.y)
        self._t_ns = 0
        self._steps = 0
        self._steer = 0.0

        self._last_scan = self._prime_scan()
        return self._obs(self._last_scan)

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
        `self.spec`をその場で書き換えず、毎回コピーを作る。"""
        if not self.randomize_dynamics:
            return self.spec
        return replace(
            self.spec,
            mu=float(self.rng.uniform(*self.mu_range)),
            tau_steer_s=float(self.rng.uniform(*self.tau_steer_s_range)),
            dead_time_s=float(self.rng.uniform(*self.dead_time_s_range)),
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
        self.vehicle.step(dt)
        self._t_ns += _STEP_NS

        hit = self.course.collides(self.vehicle.x, self.vehicle.y, self.vehicle.yaw, self._body)
        self.vehicle.note_collision(hit)

        for gen_ns, pkt in self.lidar.poll(self._t_ns, self.vehicle, _stm_us):
            s = self.asm.feed(pkt, gen_ns)
            if s is not None:
                self._last_scan = s

        progress, cross_track, half_w = self._progress.update(self.vehicle.x, self.vehicle.y)
        margin = half_w * self.cross_track_margin_frac
        cross_excess = max(0.0, abs(cross_track) - margin)
        steer_rate = abs(self._steer - prev_steer)
        speed_frac = float(np.clip(self.vehicle.speed, 0.0, self.max_speed)) / self.max_speed
        slip = self.vehicle.slip_frac
        reward = (self.progress_weight * progress - self.cross_track_weight * cross_excess
                 - self.steer_rate_weight * steer_rate + self.speed_weight * speed_frac
                 - self.slip_weight * slip)

        terminated = False
        if hit:
            reward += self.collision_penalty
            terminated = True

        self._steps += 1
        truncated = self._steps >= self.max_steps

        info = {"cross_track": cross_track, "collided": hit, "progress": progress, "slip": slip}
        return self._obs(self._last_scan), reward, terminated, truncated, info

    def _obs(self, scan: Scan) -> np.ndarray:
        w = scan_window(scan, 360.0, self.max_range)
        speed = float(np.clip(self.vehicle.speed, 0.0, self.max_speed))
        return np.concatenate([np.asarray(w.dist, dtype=np.float32),
                               np.array([speed], dtype=np.float32)])
