"""レーシングライン走行（`slam2d`版）— 地図を作り、アウトインアウトの経路を
引いて周回する。

`raspi/auto/raceline.py`（自作SLAM`raspi/nav/slam.py`ベース）と同じ3段状態
機械（EXPLORE/BUILD/RACE）だが、自己位置推定のエンジンだけを`slam2d/`（車体
非依存の汎用SLAMライブラリ、`_slam2d_nav.Slam2dNav`経由）に差し替えてある。
経路生成（`nav/centerline.py`・`nav/raceline.py`）・追従（`nav/purepursuit.py`）・
障害物検出（`nav/obstacles.py`）はそのまま流用する——これらは`OccGrid`に対して
ダックタイピングで動作する設計になっており、`slam2d.core.grid.OccGrid`の
インスタンスを渡してもそのまま動く。

## `raspi/auto/raceline.py`との違い

- **ループ閉じが無い。** `slam2d`側は`Frontend`単体（ループ閉じ無し）で運用
  する方針（`_slam2d_nav.py`のモジュールdocstring参照）。周回検出はするが、
  `close_loop()`に相当する地図の焼き直しはしない
- **`lidar_only`（scan_to_scanのみで走る実験モード）は未対応。** `slam2d`の
  `MotionModel`抽象は差し替え可能だが、このplannerでは`ExternalTwistModel`
  固定にしてある
- `id`/`name`を分けてあるので、GUIで両方を選んで比較できる
"""

from __future__ import annotations

import math
import time
from concurrent.futures import Future, ThreadPoolExecutor

import numpy as np

from slam2d.core.localize import GlobalLocalizer
from slam2d.core.types import Pose2D

from ..core.vehicle import Vehicle
from ..msgs.types import AutoMap, AutoState, Scan, VehicleState
from ..nav import centerline as cl_mod
from ..nav import obstacles as obs_mod
from ..nav import raceline as rl_mod
from ..nav.grid import pack_trinary
from ..nav.purepursuit import PursuitConfig, follow
from . import mapstore
from ._slam2d_nav import Slam2dNav, occgrid_from_trinary
from .base import ParamSpec, Planner
from .follow_the_gap import FollowTheGap

__all__ = ["Slam2dRaceLine"]

#: `DONE`は「地図と経路ができた。保存済み。人間が『レーシングライン走行』を
#: 押すのを待っている」段。**BUILD完了後、自動ではRACEに進まない**——地図作成
#: 直後にそのまま走り出すと、`request_load()`のLOCATE経由で手動再配置しても
#: 走れるようにした意味が無くなる（人間が地図内の好きな場所へ車を置き直す間
#: を作るため。バンビの指示、2026-09-03）
EXPLORE, BUILD, DONE, LOCATE, RACE = "EXPLORE", "BUILD", "DONE", "LOCATE", "RACE"

#: `raspi/auto/raceline.py`と同じ値（狭い分離帯コースを想定した刻み・広さ）
MAP_RES = 0.025
MAP_SIZE_M = 16.0
MAP_MAX_RANGE = 8.0
OUT_OF_BOUNDS_WARN = 2000
PATH_STEP = 0.10
MAX_TRACK_WIDTH = 3.0

LAP_TURN_RATIO = 330.0 / 360.0
LAP_MIN_DIST = 3.0
LAP_NEAR_M = 0.8
LAP_YAW_DEG = 35.0
LAP_CONFIRM = 3

#: `LOCATE`で候補を絞り込めたとみなす当たり率の差。仕上げ（Gauss-Newton）後の
#: 当たり率は正解が0.9台・不正解が0.6以下に割れるので、0.1で十分に分かれる
LOCATE_INLIER_GAP = 0.10


def _polyline_length(xy: np.ndarray) -> float:
    """閉じた経路（周回）の1周の長さ。`request_load()`が保存済みレーシングライン
    から`RaceLine.length`を復元するのに使う（`rl_mod.optimize()`が返す値と
    同じ定義——`raspi/nav/purepursuit.py`の`follow()`が周回長として読む）。
    """
    xy = np.asarray(xy, dtype=np.float64).reshape(-1, 2)
    if len(xy) < 2:
        return 0.0
    d = np.roll(xy, -1, axis=0) - xy
    return float(np.hypot(d[:, 0], d[:, 1]).sum())

def _heading_err(path: rl_mod.RaceLine, i: int, yaw: float) -> float:
    """経路の `i` 番目の接線と車の向きの差 [rad]（絶対値）。"""
    n = len(path)
    j = (i + 1) % n if path.closed else min(i + 1, n - 1)
    if j == i:
        i = max(0, i - 1)
    t = math.atan2(path.xy[j, 1] - path.xy[i, 1], path.xy[j, 0] - path.xy[i, 0])
    return abs((t - yaw + math.pi) % (2.0 * math.pi) - math.pi)



_MAP_EVERY = 10

#: 最小曲率の中心線へ引き戻す重み（`rl_mod.min_curvature_alpha` の `lam`）。★以前は GUI の
#: スライダ「中心線への寄せ」だったが、最小時間への寄せ（`LINE_TIME_ITERS`）と車体外形の検査を
#: 入れてからは 0.02〜1.0 で見積もりラップが ±0.05s しか変わらず（toyota2）、触る理由が無いので
#: 定数にした。壁との距離は `line_margin` で決める
LINE_LAM = 0.1
#: 「最適化 → 法線と幅を測り直す」の回数（`rl_mod.optimize` の `passes`）。★以前は GUI の
#: スライダだったが、2回で足り（3回は往復して悪くなることがある）触る理由が無いので定数にした
LINE_PASSES = 2
#: 最小時間へ寄せる再重み付けの回数（`rl_mod.optimize` の `time_iters`）。1回で効果の大半が
#: 出て2回以上は変わらない（toyota2 の見積もり 14.08→13.23→13.24s）。同じ理由で定数
LINE_TIME_ITERS = 2

#: GUI の組（`ParamSpec.group`）。地図作成（Follow the Gap）と本番走行を分けて見せる
GROUP_EXPLORE = "地図作成（Follow the Gap）"
GROUP_LINE = "経路生成"
GROUP_SPEED = "本番走行: 速度"
GROUP_STEER = "本番走行: 舵"
GROUP_SAFETY = "本番走行: 安全"

#: 速度プロファイルを決める設定（変わったら `_retime` で作り直す）
SPEED_KEYS = ("v_max", "v_min", "a_lat", "a_accel", "a_brake")


def kappa_max(vehicle) -> float:
    """車の曲がれる限界の曲率 [1/m]（最小旋回半径の逆数）。"""
    return math.tan(vehicle.max_steer) / vehicle.wheelbase


def speed_key(p: dict[str, float]) -> tuple:
    return tuple(float(p[k]) for k in SPEED_KEYS)


def speed_kwargs(p: dict[str, float], vehicle) -> dict:
    """`rl_mod.retime` / `optimize` へ渡す速度の設定。"""
    return dict(v_max=p["v_max"], v_min=p["v_min"], a_lat=p["a_lat"],
                a_accel=p["a_accel"], a_brake=p["a_brake"], kappa_max=kappa_max(vehicle))


def line_kwargs(p: dict[str, float], vehicle) -> dict:
    """`rl_mod.optimize` へ渡す経路と速度の設定（BUILD と地図の読み込みで共通）。"""
    return dict(half_width=vehicle.half_width, margin=p["line_margin"], lam=LINE_LAM,
                passes=LINE_PASSES, max_width=MAX_TRACK_WIDTH,
                # コーナーで車体の角が経路の外へ張り出すぶんを余裕に足す
                # （`nav/raceline.body_allowance`）
                front_overhang=vehicle.front_overhang, rear_overhang=vehicle.rear_overhang,
                # 薄い壁の先端を車体がかすめないか、外形で確かめる（`rl_mod._BodyCheck`）
                footprint=vehicle.footprint or None,
                time_iters=LINE_TIME_ITERS, **speed_kwargs(p, vehicle))


def _optimize_line(trinary: np.ndarray, resolution: float, origin: tuple[float, float],
                   cl: cl_mod.Centerline | np.ndarray, p: dict[str, float],
                   vehicle) -> rl_mod.RaceLine:
    """レーシングラインを引く（**ワーカースレッドで走る**。地図の写しだけを触る）。

    `cl` が中心線でなく点列なら、保存済み地図の中心線として測り直してから引く。
    ★ 最小時間への寄せと車体外形の検査で、Pi では数百ms〜1s かかる。`plan()` の中で
    回すと指令が途切れる（`cmd_deadman_ms` 150ms）。
    """
    grid = occgrid_from_trinary(trinary, resolution=resolution, origin=origin, seq=0)
    if not isinstance(cl, cl_mod.Centerline):
        cl = cl_mod.build(grid, cl, step=PATH_STEP, max_width=MAX_TRACK_WIDTH)
    return rl_mod.optimize(grid, cl, **line_kwargs(p, vehicle))


def pursuit_config(p: dict[str, float], vehicle) -> PursuitConfig:
    return PursuitConfig(
        wheelbase=vehicle.wheelbase, max_steer=vehicle.max_steer,
        lookahead_k=p["look_k"], lookahead_min=p["look_min"], delay_s=p["delay_s"],
        speed_preview_s=p["speed_preview"], ff_gain=p["steer_ff"])


class Slam2dRaceLine(Planner):
    id = "slam2d_raceline"
    name = "レーシングライン(slam2d)"
    description = "slam2d(車体非依存の汎用SLAM)で地図を作り、アウトインアウトの経路で周回する"
    #: GUI が地図パネル・地図ライブラリを出す planner（`registry.catalog`）
    map_ui = True

    params = (
        ParamSpec(group=GROUP_EXPLORE, key="explore_laps", label="地図を作る周回数", min=1, max=4, step=1,
                  default=2, unit="周",
                  note="★1周だと動く物が地図から消えない。2周が既定"),
        ParamSpec(group=GROUP_EXPLORE, key="explore_speed", label="地図作成中の速度", min=0.1, max=1.5,
                  default=0.45, step=0.05, unit="m/s",
                  note="この段は Follow the Gap で走る。速いと点群が歪んで地図が荒れる"),
        ParamSpec(group=GROUP_EXPLORE, key="explore_gap_min", label="通れると見なす距離", min=0.2, max=3.0,
                  default=0.6, step=0.05, unit="m",
                  note="★この距離以上が続く方向を「隙間」と数える。**道幅の半分より"
                       "小さくすること**"),
        ParamSpec(group=GROUP_EXPLORE, key="explore_bubble", label="安全バブル半径", min=0.05, max=0.6,
                  default=0.12, step=0.01, unit="m",
                  note="最近傍の周りを侵入禁止にする半径。**車線の半幅より小さく**"),

        ParamSpec(group=GROUP_LINE, key="line_margin", label="壁からの余裕", min=0.0, max=0.4,
                  default=0.08, step=0.01, unit="m",
                  note="車体半幅に足す安全代。地図の誤差・局在化の誤差・追従の誤差をここで飲む。"
                       "車体外形と地図の壁の距離もこれ以上に保つ。★3m/sでは追従が±10cm振れ、"
                       "0.05では広いオーバル（normal）で衝突した。0.08で0回"),

        ParamSpec(group=GROUP_SPEED, key="v_max", label="最高速度", min=0.2, max=3.0, default=2.0,
                  step=0.05, unit="m/s",
                  note="★io_node の --max-speed を超えても Pi 側で切り捨てられるだけ"),
        ParamSpec(group=GROUP_SPEED, key="v_min", label="最低速度", min=0.1, max=1.0, default=0.35,
                  step=0.05, unit="m/s", note="いちばんきついコーナーでもこれ以下にしない"),
        ParamSpec(group=GROUP_SPEED, key="a_lat", label="横加速度の上限", min=0.5, max=8.0, default=3.5,
                  step=0.1, unit="m/s²",
                  note="コーナーの速度を決めているのはこれ。前後の加減速と摩擦円で分け合う。"
                       "実測のグリップ限界は4.48（vehicle.toml の mu）。シムでは4.0まで衝突0"),
        ParamSpec(group=GROUP_SPEED, key="a_accel", label="加速度の上限", min=0.2, max=5.0, default=2.5,
                  step=0.1, unit="m/s²",
                  note="立ち上がりでどれだけ速度を戻せるか。ファームが目標速度を3.0m/s²で"
                       "ランプさせるので、それより上は効かない"),
        ParamSpec(group=GROUP_SPEED, key="a_brake", label="減速度の上限", min=0.2, max=5.0, default=2.5,
                  step=0.1, unit="m/s²",
                  note="★コーナー手前の減速開始点を決める。減速は速度PI（負のトルク）で、"
                       "ファームのランプ3.0m/s²が上限。MDの制動（ABS）は2.75で頭打ちなので使わない"),

        ParamSpec(group=GROUP_STEER, key="look_k", label="前方注視の速度係数", min=0.0, max=2.0,
                  default=0.45, step=0.05, unit="s",
                  note="Ld = 係数×速度 + 最小値。上げると滑らかだがコーナーで内を切る。"
                       "★既定0.7は狭いコースで内を切りすぎた（2m/sのtoyotaで衝突10回・"
                       "横偏差16cm、0.45にすると衝突0・6.2cmで舵も滑らかになった）"),
        ParamSpec(group=GROUP_STEER, key="look_min", label="前方注視の最小値", min=0.15, max=1.5,
                  default=0.30, step=0.05, unit="m",
                  note="低速時の注視距離。小さすぎると舵が振動する"),
        ParamSpec(group=GROUP_STEER, key="delay_s", label="遅延補償", min=0.0, max=0.4, default=0.15,
                  step=0.01, unit="s",
                  note="★舵が効き始めるまでの時間。実測して合わせること"),
        ParamSpec(group=GROUP_STEER, key="steer_ff", label="曲率フィードフォワード", min=0.0, max=1.0, default=0.8,
                  step=0.1, unit="",
                  note="★Pure Pursuit は注視区間で平均した曲率で曲がるので、S字で内側を切り続ける"
                       "（3m/sで±15cm）。足元の経路の曲率に差し替える重み。0で従来どおり。"
                       "0.5ではチケイン（circuit_chicane_a）で内を切って衝突、0.8で0回"),
        ParamSpec(group=GROUP_SPEED, key="speed_preview", label="速度の先読み", min=0.0, max=1.0, default=0.1,
                  step=0.05, unit="s",
                  note="★遅延補償の位置からこの時間ぶん先の速度を指令する（速度ループの遅れを"
                       "埋める）。以前は舵の注視点の速度で、2m/sで0.75s早く減速していた"),
        ParamSpec(group=GROUP_SAFETY, key="join_speed", label="経路に乗るまでの速度", min=0.2, max=1.5,
                  default=0.5, step=0.05, unit="m/s",
                  note="★走り出し（自己位置の復元直後）は経路から横・向きがずれている。"
                       "乗る（横8cm・向き15°以内）まではこの速度に抑える。抑えないと最高速のまま"
                       "大回りして壁に当たった（toyota/toyota2 の sim.bench で実測、2026-09-29）"),
        ParamSpec(group=GROUP_SAFETY, key="min_score", label="自己位置の信頼下限", min=0.1, max=0.8,
                  default=0.35, step=0.01, unit="",
                  note="★スキャンマッチの一致度がこれを切ったら止まる"),
        ParamSpec(group=GROUP_SAFETY, key="max_cross", label="横偏差の上限", min=0.1, max=1.5, default=0.5,
                  step=0.05, unit="m", note="経路からこれだけ離れたら止まる"),
        ParamSpec(group=GROUP_SAFETY, key="obstacle_stop", label="障害物で止まる距離", min=0.0, max=4.0,
                  default=0.0, step=0.1, unit="m",
                  note="★これから通る帯に動く物が居たら制動する。0で無効"),
        ParamSpec(group=GROUP_SAFETY, key="obstacle_pad", label="壁の近くを無視する幅", min=0.05, max=0.6,
                  default=0.25, step=0.01, unit="m",
                  note="★壁からこの範囲に落ちた点は動的障害物と見なさない"),
    )

    #: 保存済み地図を読んだとき、中心線からラインを引き直すか（`slam2d_route` は自前で作る）
    _reline_on_load = True

    def __init__(self) -> None:
        self._line_pool: ThreadPoolExecutor | None = None
        self._last_p: dict[str, float] | None = None
        self.vehicle = Vehicle.load()
        self.ftg = FollowTheGap()
        self.slam = Slam2dNav(resolution=MAP_RES, size_m=MAP_SIZE_M,
                              lidar_x=self.vehicle.lidar_x, lidar_y=self.vehicle.lidar_y,
                              max_range=MAP_MAX_RANGE)
        self.reset()

    # ── 状態 ──

    def reset(self) -> None:
        self.phase = EXPLORE
        self.slam.reset()
        self.ftg.reset()
        self.laps = 0
        self.centerline: cl_mod.Centerline | None = None
        self.path: rl_mod.RaceLine | None = None
        self._lap_idx: list[int] = [0]
        self._lap_hits = 0
        self._build_step = 0
        self._build_error = ""
        self._freeze_requested = False
        self._hint = -1
        self._obs_pad = 0.15
        #: 経路に乗ったか（乗るまでは `join_speed` に抑える、`_join_cap`）
        self._joined = False
        #: 経路から外れて止まっている時間 [s] と、乗り直した回数（`_off_route`）
        self._off_t = 0.0
        self._rejoins = 0
        self._dt = 0.0
        #: 今の経路の速度プロファイルを作った設定（`_retime`）
        self._speed_key: tuple | None = None
        #: 地図の読み込みで引き直しているライン（`request_load`）
        self._line_job: Future | None = None
        #: BUILD で引いているライン（`_build`）
        self._build_job: Future | None = None
        self._prev_obs: list[obs_mod.Obstacle] = []
        self._obs: list[obs_mod.Obstacle] = []
        self._map: AutoMap | None = None
        self._map_seq = -_MAP_EVERY
        self._map_frozen = False
        self._map_dirty = True
        #: 保存済み地図から読み込んだ中心線（`self.centerline`はEXPLORE→BUILDで
        #: 自前生成するものだけを持つので、読み込み経路はここに置く。
        #: `snapshot()`が両方を出し分ける）
        self._loaded_centerline_xy: np.ndarray | None = None
        #: 進行中のグローバルローカリゼーション（`LOCATE`段でのみ非None）
        self._localizer: GlobalLocalizer | None = None
        #: 地図パネルのクリックによる絞り込みヒント（mapフレーム座標）
        self._loc_hint: tuple[float, float] | None = None
        #: 地図作成直後に走り出すときの「直前の自己位置」（`request_load`参照）
        self._locate_seed: tuple[float, float] | None = None
        self._load_error = ""
        #: BUILD完了時に自動保存した地図の名前（`DONE`段の表示用）
        self._saved_map_name = ""
        self._save_error = ""

    def on_vehicle_state(self, vs: VehicleState) -> None:
        """`planning_node`が`vehicle_state`を受けるたびに呼ぶ（50Hz）。

        点ごとの脱スキュー（`slam2d/core/deskew.py`）に使う twist の履歴を
        SLAM へ流し込むだけ。`plan()`は10Hzでしか呼ばれないので、ここで
        受けておかないと1周100msのあいだの回頭の変化が追えない。
        """
        self.slam.on_vehicle_state(vs)

    def request_freeze(self) -> None:
        self._freeze_requested = True

    def request_clear(self) -> None:
        self.reset()

    def request_load(self, name: str) -> None:
        """保存済み地図（`raspi/auto/mapstore.py`）を読み込み、地図作成
        （EXPLORE/BUILD）を経ずに`LOCATE`から走り始める。

        `reset()`を経由しない——地図を作り直すわけではないので、この呼び出し
        だけで前回の`_loc_hint`（直前に押したクリック）を捨てる必要は無い
        （むしろ「クリック→レーシングライン走行」の順で押した意思を活かす）。
        """
        loaded = mapstore.load_map(name)
        if loaded is None:
            self._load_error = f"地図『{name}』を読み込めない"
            return
        grid = occgrid_from_trinary(
            loaded.trinary, resolution=loaded.resolution,
            origin=(loaded.origin_x, loaded.origin_y), seq=self.slam.next_seq())
        # ★ 作ったばかりの地図で走り出すなら、**今どこに居るかはもう分かっている**。
        #   その姿勢を探索の種にする（車を動かされていたら外れるので、
        #   `_locate()`が失敗したら種を捨てて全域探索に落ちる）。
        #   保存地図は今の格子をそのまま書き出したものなので座標系も一致する
        self._locate_seed = ((self.slam.pose.x, self.slam.pose.y)
                             if name == self._saved_map_name else None)
        self.slam.replace_grid_for_race(grid)
        self.path = rl_mod.RaceLine(
            xy=loaded.raceline_xy, v=loaded.raceline_v,
            kappa=rl_mod.curvature(loaded.raceline_xy),
            alpha=np.zeros(len(loaded.raceline_xy)), s=np.zeros(len(loaded.raceline_xy)),
            length=_polyline_length(loaded.raceline_xy))
        self._loaded_centerline_xy = loaded.centerline_xy
        self.centerline = None
        # ★ 保存したラインの形は地図を作ったときのコードと設定のまま。中心線から今の
        #   設定で引き直す（重いのでワーカーで。できるまでは保存したラインで走る）
        self._line_job = None
        if self._reline_on_load and len(loaded.centerline_xy) >= 20:
            p = self._last_p or {s.key: s.default for s in self.params}
            self._line_job = self._pool().submit(
                _optimize_line, loaded.trinary.copy(), loaded.resolution,
                (loaded.origin_x, loaded.origin_y), np.asarray(loaded.centerline_xy).copy(),
                dict(p), self.vehicle)
        self._speed_key = None            # 速度は今の設定で作り直す（`_retime`）
        self.phase = LOCATE
        self._localizer = None
        self._load_error = ""
        self.laps = 0
        self._hint = -1
        self._map_dirty = True

    def _poll_line(self) -> None:
        """地図の読み込みで引き直したラインができていれば差し替える（`request_load`）。"""
        job = self._line_job
        if job is None or not job.done():
            return
        self._line_job = None
        try:
            self.path = job.result()
        except Exception as e:                      # noqa: BLE001  保存したラインで走り続ける
            self._load_error = f"ラインを引き直せなかった（保存したラインで走る）: {e}"
            return
        self._hint = -1
        self._speed_key = None
        self._map_dirty = True

    def request_locate_hint(self, x: float, y: float) -> None:
        self._loc_hint = (x, y)
        self._localizer = None     # 探索中なら絞り込んでやり直す

    # ── 本体 ──

    def plan(self, scan: Scan, vs: VehicleState | None,
             p: dict[str, float], dt: float) -> AutoState:
        st = AutoState(mode=self.id, planner=self.name, phase=self.phase)
        self._dt = dt
        self._last_p = p
        self._poll_line()

        if vs is None:
            st.reason = "車両状態がまだ届いていない"
            return st

        # ★ 回頭はジャイロ、前進は`speed`（射影済み・ローパス済みの値）。
        #   `wheel_speed[]`は低速でばたつくので渡してはいけない（`raspi/auto/raceline.py`と同じ規約）
        u = self.slam.update(scan, dt, yaw_rate=vs.yaw_rate, speed=vs.speed)
        st.pose_x, st.pose_y, st.pose_yaw = u.pose.x, u.pose.y, u.pose.yaw
        st.match_score = u.score
        st.lap_progress = self.slam.lap_progress()
        st.laps = self.laps

        if self.phase == EXPLORE:
            return self._explore(st, scan, vs, p, dt, u.lost)
        if self.phase == BUILD:
            return self._build(st, p)
        if self.phase == DONE:
            return self._done(st)
        if self.phase == LOCATE:
            return self._locate(st, scan, p)
        return self._race(st, scan, vs, p, u.lost)

    # ── EXPLORE ──

    def _explore(self, st: AutoState, scan: Scan, vs: VehicleState,
                 p: dict[str, float], dt: float, lost: bool) -> AutoState:
        lap_msg = ""
        lap_done, explore_done = self._check_lap(p)
        if lap_done:
            # ★ `raspi/auto/raceline.py`と異なり、ここでは`close_loop()`に
            # 相当する地図の焼き直しをしない（`_slam2d_nav.py`のモジュール
            # docstring参照）。周回検出そのものは変わらず行う
            if explore_done:
                self.slam.freeze()
                self.phase = st.phase = BUILD
                self._build_step = 0
                st.reason = "地図を確定した。経路を作っている"
                return st
            lap_msg = f"{self.laps}周目を検出した"

        fp = FollowTheGap.merged({
            "max_speed": p["explore_speed"],
            "gap_min": p["explore_gap_min"],
            "bubble_m": p["explore_bubble"],
        })
        sub = self.ftg.plan(scan, vs, fp, dt)

        st.ready = sub.ready
        st.brake = sub.brake
        st.target_speed = sub.target_speed
        st.target_steer = sub.target_steer
        st.heading = sub.heading
        st.gap_start_deg = sub.gap_start_deg
        st.gap_end_deg = sub.gap_end_deg
        st.bubble_start_deg = sub.bubble_start_deg
        st.bubble_end_deg = sub.bubble_end_deg
        st.free_ahead = sub.free_ahead
        st.nearest = sub.nearest
        st.nearest_deg = sub.nearest_deg
        st.valid_ratio = sub.valid_ratio

        want = int(p["explore_laps"])
        head = f"地図作成 {self.laps}/{want}周（{st.lap_progress:.2f}周ぶん回頭）"
        if self.slam.grid.out_of_bounds > OUT_OF_BOUNDS_WARN:
            head += f"・★地図からはみ出している（{self.slam.grid.out_of_bounds}本）"
        if lost:
            st.reason = f"{head}・自己位置を見失っている"
        elif lap_msg:
            st.reason = f"{head}・{lap_msg}"
        else:
            st.reason = f"{head}・{sub.reason}"
        return st

    def _check_lap(self, p: dict[str, float]) -> tuple[bool, bool]:
        if self._freeze_requested:
            self._freeze_requested = False
            self._lap_idx.append(len(self.slam.trajectory))
            self.laps = max(1, self.laps)
            return True, True
        if not self.slam.trajectory:
            return False, False

        want = int(p["explore_laps"])
        target = self.laps + 1
        ok = (self.slam.lap_progress() >= LAP_TURN_RATIO * target
              and self.slam.distance >= LAP_MIN_DIST * target
              and self._near_start())
        self._lap_hits = self._lap_hits + 1 if ok else 0
        if self._lap_hits < LAP_CONFIRM:
            return False, False

        self._lap_hits = 0
        self.laps += 1
        self._lap_idx.append(len(self.slam.trajectory))
        return True, self.laps >= want

    def _near_start(self) -> bool:
        sx, sy, syaw = self.slam.trajectory[0]
        x, y, yaw = self.slam.pose
        if math.hypot(x - sx, y - sy) > LAP_NEAR_M:
            return False
        d = (yaw - syaw + math.pi) % (2.0 * math.pi) - math.pi
        return abs(math.degrees(d)) <= LAP_YAW_DEG

    # ── BUILD ──

    def _build(self, st: AutoState, p: dict[str, float]) -> AutoState:
        if self._build_error:
            st.reason = self._build_error
            return st
        try:
            if self._build_step == 0:
                traj = self._last_lap()
                if len(traj) < 20:
                    raise ValueError(f"軌跡が短すぎる（{len(traj)}点）")
                self.centerline = cl_mod.build(
                    self.slam.grid, traj, step=PATH_STEP,
                    max_width=MAX_TRACK_WIDTH)
                self._build_step = 1
                self._map_dirty = True
                st.reason = "経路を作っている 1/2（中心線）"
                return st

            assert self.centerline is not None
            if self._build_job is None:
                g = self.slam.grid
                self._build_job = self._pool().submit(
                    _optimize_line, g.trinary().copy(), g.resolution, tuple(g.origin),
                    self.centerline, dict(p), self.vehicle)
            if not self._build_job.done():
                st.reason = "経路を作っている 2/2（レーシングライン）"
                return st
            job, self._build_job = self._build_job, None
            self.path = job.result()
            self._speed_key = speed_key(p)
        except Exception as e:                      # noqa: BLE001
            self._build_error = f"経路を作れなかった: {e}"
            st.reason = self._build_error
            return st

        self.phase = st.phase = DONE
        self._hint = -1
        self._map_dirty = True
        self._saved_map_name = self._auto_save_map()
        base = (f"経路ができた（{len(self.path)}点・"
                f"1周 {self.path.length:.1f}m・"
                f"{self.path.v.min():.2f}〜{self.path.v.max():.2f} m/s）。")
        st.reason = base + (
            f"地図を『{self._saved_map_name}』として保存した。"
            "レーシングライン走行で開始してください"
            if self._saved_map_name else self._save_error)
        return st

    def _auto_save_map(self) -> str:
        """BUILD完了時に自動で地図を保存する。名前は日時から生成する。

        `mapstore.save_map()`は`slam2d`を知らない純粋なファイルI/O層——
        ここは`Slam2dRaceLine`自身が`OccGrid`/`RaceLine`を直接持っているので、
        `telemetry_node._maps_save()`のような`AutoMap`往復（pack/unpack）を
        経由せず呼べる。失敗しても例外は投げない（保存できなくても地図作成
        自体は完了しているので、`DONE`段の判断は続けられるべき）。
        """
        name = time.strftime("course_%Y%m%d_%H%M%S")
        assert self.path is not None
        try:
            g = self.slam.grid
            mapstore.save_map(
                name, resolution=g.resolution, origin_x=g.origin[0], origin_y=g.origin[1],
                trinary=g.trinary(),
                centerline_xy=(self.centerline.xy if self.centerline is not None
                              else np.zeros((0, 2))),
                raceline_xy=self.path.xy, raceline_v=self.path.v)
        except Exception as e:                      # noqa: BLE001
            self._save_error = f"地図の自動保存に失敗: {e}"
            return ""
        return name

    # ── DONE ──

    def _done(self, st: AutoState) -> AutoState:
        """地図と経路ができて保存済み。**人間が「レーシングライン走行」を
        押すまでここで待つ。** BUILD完了直後に自動でRACEへ進まないのは、
        地図内の好きな場所へ車を動かし直す間を作るため——
        「レーシングライン走行」を押すと`request_load()`が呼ばれ、
        `LOCATE`（自己位置復元）から始まる。
        """
        name = f"『{self._saved_map_name}』" if self._saved_map_name else ""
        st.reason = f"地図{name}を保存した。レーシングライン走行で開始してください（車を動かしてもよい）"
        return st

    def _last_lap(self) -> np.ndarray:
        traj = self.slam.trajectory_array()
        if len(self._lap_idx) >= 2:
            a, b = self._lap_idx[-2], self._lap_idx[-1]
            if b - a >= 20:
                return traj[a:b]
        return traj

    # ── LOCATE ──

    def _locate(self, st: AutoState, scan: Scan, p: dict[str, float]) -> AutoState:
        """保存済み地図のどこに車が居るか分からない状態から自己位置を復元する。

        グローバルローカリゼーション（`slam2d.core.localize.GlobalLocalizer`）は
        1周期に候補角度1つぶんしか進めない（計算コストを1周期に収める設計、
        `localize.py`のモジュールdocstring参照）ので、この段の間は車両を
        `ready=False`で静止させたまま複数周期にわたって`step()`を呼び続ける。
        """
        pts = self.slam.deskew_scan(scan)

        if self._localizer is None:
            hint = self._loc_hint or self._locate_seed
            self._localizer = GlobalLocalizer(self.slam.grid, hint=hint)
            if self._localizer.done:
                st.reason = ("自己位置の探索範囲に空きセルが無い。"
                             "地図の選択かクリックしたヒントを見直してください")
                return st

        if not self._localizer.step(pts):
            st.reason = f"自己位置を探索中（{self._localizer.progress * 100:.0f}%）"
            return st

        cands = self._localizer.candidates(3)
        self._localizer = None
        seeded = self._loc_hint is None and self._locate_seed is not None
        if not cands:
            if seeded:
                self._locate_seed = None      # 種の周りに候補が無い。全域で探し直す
            st.reason = "自己位置の候補が見つからない"
            return st

        # ★ 粗探索（0.2m間隔・15°刻み）の得点だけで決めない。上位候補を
        #   それぞれ総当たり＋Gauss-Newton で仕上げ、**当たり率**で比べる。
        #   粗い尺度では並ぶ候補（オーバルの反対側の直線など）も、
        #   仕上げるとはっきり差がつく（`slam2d/core/localize.py`参照）
        scored = []
        for c in cands:
            ref = self.slam.refine(pts, Pose2D(c.x, c.y, c.yaw))
            if ref is not None:
                scored.append((ref.inlier, ref))
        if not scored:
            if seeded:
                self._locate_seed = None
            st.reason = "自己位置の候補を仕上げられなかった"
            return st
        scored.sort(key=lambda t: -t[0])
        best = scored[0][1]
        if len(scored) >= 2 and scored[0][0] - scored[1][0] < LOCATE_INLIER_GAP:
            if seeded:
                self._locate_seed = None
            st.reason = ("自己位置の候補が複数あり絞り込めない"
                         "（左右対称・繰り返し形状の疑い）。"
                         "地図パネルをタップしておおよその位置を教えてください")
            return st
        if best.inlier < p["min_score"]:
            if seeded:
                self._locate_seed = None      # 種が外れている。全域で探し直す
            st.reason = f"自己位置が見つからない（最良の当たり率 {best.inlier:.2f}）"
            return st
        refined = best
        self.slam.set_pose(*refined.pose)
        self.phase = st.phase = RACE
        self._locate_seed = None
        self._hint = -1
        self._joined = False
        self._off_t = 0.0
        self._rejoins = 0
        self._map_dirty = True
        # ★ `st.match_score`はこの`plan()`冒頭の`self.slam.update()`（`_locate()`
        # 呼び出し前の古い姿勢からの追跡マッチ、当然ながら未知の領域を指して
        # 失敗する）の値のまま残っている。ここで求め直した値に置き換えないと、
        # RACEへ切り替わった瞬間だけ「一致度0」を報告してしまう
        st.match_score = refined.inlier
        st.pose_x, st.pose_y, st.pose_yaw = refined.pose
        st.reason = f"自己位置を復元した（一致度 {refined.inlier:.2f}）。走行を開始する"
        return st

    # ── RACE ──

    def _race(self, st: AutoState, scan: Scan, vs: VehicleState,
              p: dict[str, float], lost: bool) -> AutoState:
        if self.path is None:
            st.reason = "経路がまだ無い"
            return st
        self._retime(p)

        if lost or st.match_score < p["min_score"]:
            st.reason = (f"自己位置が信用できない（一致度 {st.match_score:.2f} < "
                         f"{p['min_score']:.2f}）")
            return st

        cfg = pursuit_config(p, self.vehicle)
        pp = follow(self.path, self.slam.pose, vs.speed, vs.steer_actual,
                    cfg, hint=self._hint)
        self._hint = pp.index
        st.target_steer = pp.steer
        st.heading = pp.steer
        st.cross_track = pp.cross_track
        st.target_x, st.target_y = pp.target

        if self._off_route(st, pp, vs, p):
            return st

        self._obs_pad = p["obstacle_pad"]
        self._update_obstacles(scan)
        st.obstacles = [v for o in self._obs for v in (o.x, o.y, o.r)]
        hit, dist = obs_mod.blocking(
            self._obs, self.path.xy, pp.index,
            half_width=self.vehicle.half_width + p["line_margin"],
            ahead_m=p["obstacle_stop"] + self.vehicle.front_overhang,
            step=self.path.length / len(self.path),
            skip_m=self.vehicle.front_overhang)

        st.ready = True
        st.target_speed = pp.speed
        st.free_ahead = dist if hit is not None else math.inf
        self._join_cap(st, pp, self.path, self.slam.pose, p)

        stop_at = p["obstacle_stop"]
        if (stop_at > 0.0 and hit is not None
                and dist - self.vehicle.front_overhang <= stop_at):
            st.brake = True
            st.target_speed = 0.0
            ang = math.degrees(math.atan2(hit.y - st.pose_y, hit.x - st.pose_x)
                               - st.pose_yaw)
            ang = (ang + 180.0) % 360.0 - 180.0
            st.reason = (f"前方 {max(0.0, dist - self.vehicle.front_overhang):.1f}m "
                         f"（{ang:+.0f}°）に障害物。停止")
            return st

        st.reason = (f"{self.laps}周走行中・速度 {st.target_speed:.2f} m/s・"
                     f"横偏差 {pp.cross_track * 100:+.0f}cm"
                     + ("" if self._joined else "・経路に乗るまで減速"))
        return st

    def _retime(self, p: dict[str, float]) -> None:
        """速度の設定が変わっていたら、経路の速度プロファイルだけを作り直す。

        ★ 保存した地図の速度は地図を作ったときの設定のまま（`rl_mod.retime`）。
        `request_load()` の後はここで必ず1回作り直される。
        """
        key = speed_key(p)
        if key == self._speed_key or self.path is None:
            return
        self.path = rl_mod.retime(self.path, **speed_kwargs(p, self.vehicle))
        self._speed_key = key
        self._map_dirty = True

    # ── 経路に乗る・外れたら乗り直す（`slam2d_route` と共通） ──

    def _join_cap(self, st: AutoState, pp, path, pose, p: dict[str, float]) -> None:
        """経路に乗る（横8cm・向き15°以内）までは `join_speed` に抑える。

        ★ 走り出し（LOCATE 直後）の車は経路から横に10〜30cm・向きに25°ずれている。
        抑えないと最高速のまま大回りして壁に当たった（toyota/toyota2 の sim.bench、
        2026-09-29。このplannerも同じ bench で衝突・61cm 外れて0周だった）。
        """
        if self._joined:
            return
        if abs(pp.cross_track) < 0.08 and _heading_err(path, pp.index, pose[2]) < math.radians(15):
            self._joined = True
            return
        st.target_speed = min(st.target_speed, p["join_speed"])

    def _off_route(self, st: AutoState, pp, vs: VehicleState, p: dict[str, float]) -> bool:
        """経路から外れすぎたら止める（True を返す）。**止まったまま終わらせない**:
        1秒止まったら最寄り点を探し直し、`join_speed` で乗り直す（3回まで）。
        乗り直し中は上限を 1.5 倍に広げる（乗りに行く途中で止まらないように）。
        """
        limit = p["max_cross"] * (1.0 if self._joined else 1.5)
        if abs(pp.cross_track) > limit:
            if abs(vs.speed) < 0.05:
                self._off_t += self._dt
            if self._off_t >= 1.0 and self._rejoins < 3:
                self._off_t = 0.0
                self._rejoins += 1
                self._joined = False
                self._hint = -1
            st.reason = (f"経路から {abs(pp.cross_track) * 100:.0f}cm 離れた"
                         f"（上限 {limit * 100:.0f}cm）。"
                         + ("止まってから乗り直す" if self._rejoins < 3
                            else "乗り直しを3回試したので止まる"))
            return True
        self._off_t = 0.0
        if self._joined and self._rejoins and abs(pp.cross_track) < 0.05:
            self._rejoins = 0                 # 経路に戻れたら数え直す
        return False

    def _update_obstacles(self, scan: Scan) -> None:
        pts = self.slam.deskew_scan(scan)
        now = obs_mod.detect(self.slam.grid, pts, self.slam.pose,
                             wall_pad=self._obs_pad)
        self._obs = obs_mod.confirm(now, self._prev_obs)
        self._prev_obs = now

    def _pool(self) -> ThreadPoolExecutor:
        if self._line_pool is None:
            self._line_pool = ThreadPoolExecutor(max_workers=1, thread_name_prefix="line")
        return self._line_pool

    def close(self) -> None:
        if self._line_pool is not None:
            self._line_pool.shutdown(wait=False, cancel_futures=True)

    # ── GUI へ渡す重い状態 ──

    def snapshot(self) -> AutoMap | None:
        g = self.slam.grid
        fresh = g.seq - self._map_seq
        if (self._map is not None and not self._map_dirty
                and fresh < _MAP_EVERY and g.frozen == self._map_frozen):
            return self._map

        m = AutoMap(map_seq=g.seq, resolution=g.resolution,
                    origin_x=g.origin[0], origin_y=g.origin[1],
                    width=g.width, height=g.height,
                    cells=pack_trinary(g.trinary()))
        if self.centerline is not None:
            m.centerline = self.centerline.xy.reshape(-1).tolist()
        elif self._loaded_centerline_xy is not None:
            # 保存済み地図から読み込んだ場合、`self.centerline`はEXPLORE→BUILDの
            # 自前生成専用なので空のまま（`request_load()`参照）
            m.centerline = np.asarray(self._loaded_centerline_xy).reshape(-1).tolist()
        if self.path is not None:
            m.raceline = self.path.xy.reshape(-1).tolist()
            m.raceline_v = self.path.v.tolist()
        self._map = m
        self._map_seq = g.seq
        self._map_frozen = g.frozen
        self._map_dirty = False
        return m
