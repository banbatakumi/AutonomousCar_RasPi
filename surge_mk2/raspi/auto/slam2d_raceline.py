"""レーシングライン走行（`slam2d`版）— 地図を作り、アウトインアウトの経路を
引いて周回する。

状態機械は6段: `EXPLORE`（地図作成。engage 中は Follow the Gap、していなければ人の
ラジコン）→ `BUILD`（中心線とレーシングライン）→ `DONE`（保存して人を待つ）→
`LOCATE`（保存地図の上で自己位置を復元）→ `RACE`（周回）、障害物で進めないときだけ
`DETOUR`（Hybrid A* で抜ける）。保存済みの地図を読めば `LOCATE` から始まる。

自己位置推定は`slam2d/`（車体非依存の汎用SLAMライブラリ、`_slam2d_nav.Slam2dNav`経由）。
経路生成（`nav/centerline.py`・`nav/raceline.py`）・追従（`nav/purepursuit.py`）・
障害物検出（`nav/obstacles.py`）は`OccGrid`に対してダックタイピングで動くので、
`slam2d.core.grid.OccGrid`のインスタンスをそのまま渡している。

- **ループ閉じは EXPLORE→BUILD の瞬間に1回だけ**（`Slam2dNav.freeze()` が
  `SlamSystem.flush()` を呼ぶ）。走行中（RACE）は`Frontend`直結の追跡専用
- 推測航法は`ExternalTwistModel`固定（ジャイロ＋車速。LiDAR だけで走るモードは無い）
"""

from __future__ import annotations

import math
import time
from concurrent.futures import ThreadPoolExecutor

import numpy as np

from slam2d.core.localize import GlobalLocalizer
from slam2d.core.types import Pose2D

from ..core.vehicle import Vehicle
from ..msgs.types import AutoMap, AutoState, Scan, VehicleState
from ..nav import avoid as av_mod
from ..nav import centerline as cl_mod
from ..nav import obstacles as obs_mod
from ..nav import raceline as rl_mod
from ..nav.grid import pack_trinary
from ..nav.purepursuit import PursuitConfig, follow, nearest_index
from ..nav.route_switch import lap_crossed
from . import mapstore
from ._slam2d_nav import Slam2dNav, occgrid_from_trinary
from .base import ParamSpec, Planner
from .follow_the_gap import FollowTheGap
from .park_to_point import ParkToPoint

__all__ = ["Slam2dRaceLine"]

#: `DONE`は「地図と経路ができた。保存済み。人間が『レーシングライン走行』を
#: 押すのを待っている」段。**BUILD完了後、自動ではRACEに進まない**——地図作成
#: 直後にそのまま走り出すと、`request_load()`のLOCATE経由で手動再配置しても
#: 走れるようにした意味が無くなる（人間が地図内の好きな場所へ車を置き直す間
#: を作るため。バンビの指示、2026-09-03）
EXPLORE, BUILD, DONE, LOCATE, RACE = "EXPLORE", "BUILD", "DONE", "LOCATE", "RACE"
#: 横へ避けられない障害物・スタックを、Hybrid A*（`ParkToPoint`）で後退・切り返しも使って
#: 抜けている段。抜けたら RACE へ戻る
DETOUR = "DETOUR"
#: 同じ所で Hybrid A* を試す回数の上限（走り出せたら数え直す）
DETOUR_TRIES = 2
#: 避ける経路を引くとき、検出した障害物の半径を何倍に扱うか（`_plan_avoid`）
_OBS_GROW = 1.4
#: 同じく半径の下限 [m]
_OBS_MIN_R = 0.12
#: Hybrid A* で抜けるときの到着の許容。経路に戻れればよいので駐車ほど詰めない
#: （既定の駐車の許容だと、目標の手前で前後の切り返しを繰り返して終わらなかった）
_DETOUR_POS_TOL = 0.10
_DETOUR_YAW_TOL_DEG = 12.0
#: Hybrid A* で抜ける時間の上限 [s]（超えたら失敗として止まって待つ）
_DETOUR_MAX_S = 20.0
#: 横へ避けられない障害物の手前で止まる距離（車体前端から）[m]
_STOP_GAP = 0.35

#: 地図と経路の刻み・広さ（狭い分離帯コースを想定）
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


def forward_traj(traj: np.ndarray, min_step: float = 0.02) -> np.ndarray:
    """軌跡 (N, 3) から、直前に残した点より**車の向きに前へ**進んだ点だけを残す。

    人のラジコンで地図を作ると、停止・その場の切り返し・後退が混ざる。そのまま
    中心線や道路グラフの「通ってよい向き」に使うと、折り返しで中心線がよじれ、
    後退の票で向きを逆に固定しかねない。後退してから前進し直した区間は、
    後退を始めた点を追い越すまで捨てられる（同じ道の重複も消える）。
    """
    if len(traj) < 2:
        return traj
    keep = [0]
    for i in range(1, len(traj)):
        px, py = traj[keep[-1], 0], traj[keep[-1], 1]
        x, y, yaw = traj[i]
        dx, dy = x - px, y - py
        if math.hypot(dx, dy) < min_step:
            continue
        if dx * math.cos(yaw) + dy * math.sin(yaw) <= 0.0:
            continue
        keep.append(i)
    return traj[keep]


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
GROUP_AVOID = "本番走行: 障害物の回避"

#: 速度プロファイルを決める設定（変わったら `_retime` で作り直す）
SPEED_KEYS = ("v_max", "v_min", "a_lat", "a_accel", "a_brake")


def speed_key(p: dict[str, float]) -> tuple:
    return tuple(float(p[k]) for k in SPEED_KEYS)


def speed_kwargs(p: dict[str, float], vehicle) -> dict:
    """`rl_mod.retime` / `optimize` へ渡す速度の設定。"""
    return dict(v_max=p["v_max"], v_min=p["v_min"], a_lat=p["a_lat"],
                a_accel=p["a_accel"], a_brake=p["a_brake"], kappa_max=vehicle.kappa_max)


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
        speed_preview_s=p["speed_preview"], ff_gain=p["steer_ff"],
        steer_map=vehicle.steer_map)


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
                  note="★スキャンマッチの一致度がこれを切ったら（見失いが推測航法での続行の"
                       "時間を超えたら）止まる"),
        ParamSpec(group=GROUP_SAFETY, key="coast_s", label="推測航法で続行する時間", min=0.0, max=3.0,
                  default=1.5, step=0.1, unit="s",
                  note="★自己位置を見失っても（壁越しに別の部屋が見えた等）、これ以内は推測航法"
                       "（車速+ジャイロ）の姿勢で経路を追い続ける。0で従来どおり即停止"),
        ParamSpec(group=GROUP_SAFETY, key="coast_speed", label="推測航法中の速度上限", min=0.2, max=2.0,
                  default=0.8, step=0.05, unit="m/s",
                  note="推測航法で続行している間はこの速度に抑える"),
        ParamSpec(group=GROUP_SAFETY, key="max_cross", label="横偏差の上限", min=0.1, max=1.5, default=0.5,
                  step=0.05, unit="m", note="経路からこれだけ離れたら止まる"),
        ParamSpec(group=GROUP_SAFETY, key="obstacle_stop", label="障害物で止まる距離", min=0.0, max=4.0,
                  default=0.0, step=0.1, unit="m",
                  note="★これから通る帯に動く物が居たら制動する。0で無効"),
        ParamSpec(group=GROUP_SAFETY, key="obstacle_pad", label="壁の近くを無視する幅", min=0.05, max=0.6,
                  default=0.25, step=0.01, unit="m",
                  note="★壁からこの範囲に落ちた点は動的障害物と見なさない"),
        ParamSpec(group=GROUP_AVOID, key="obstacle_avoid", label="障害物を避ける", min=0, max=1, step=1,
                  default=0, unit="",
                  note="★1で、経路を塞ぐ障害物を横へ避ける（止まらずに済むなら止まらない）。"
                       "避けられなければ止まり、Hybrid A* で後退・切り返しも使って抜ける。"
                       "0 なら従来どおり（『障害物で止まる距離』だけ）"),
        ParamSpec(group=GROUP_AVOID, key="avoid_margin", label="障害物との余裕", min=0.0, max=0.4,
                  default=0.10, step=0.01, unit="m",
                  note="障害物の縁から車体側面までの余裕。自己位置と障害物の位置の誤差をここで飲む"),
        ParamSpec(group=GROUP_AVOID, key="avoid_clear", label="車体外形の合格線", min=0.0, max=0.2,
                  default=0.05, step=0.01, unit="m",
                  note="避ける経路で車体外形と壁・障害物の距離がこれを切ったら、その避け方は使わない"),
        ParamSpec(group=GROUP_AVOID, key="avoid_speed", label="避ける間の速度上限", min=0.2, max=2.5,
                  default=1.0, step=0.05, unit="m/s",
                  note="横へずれて障害物の横を抜ける区間はこの速度に抑える（手前から減速する）"),
        ParamSpec(group=GROUP_AVOID, key="avoid_len_min", label="避け始める長さの下限", min=0.3, max=3.0,
                  default=0.8, step=0.1, unit="m",
                  note="横へずれ始めてからずれ切るまでの距離の下限。速いほど・ずれが大きいほど長くなる"),
        ParamSpec(group=GROUP_AVOID, key="stuck_s", label="スタックとみなす時間", min=0.5, max=5.0,
                  default=1.5, step=0.1, unit="s",
                  note="進めと言っているのに止まっている時間がこれを超えたら、Hybrid A* で抜ける"),
        ParamSpec(group=GROUP_AVOID, key="detour_ahead", label="抜けた先の目標の距離", min=0.3, max=2.0,
                  default=1.2, step=0.1, unit="m",
                  note="Hybrid A* の目標を、障害物の後ろ（車体後端が抜ける位置）からこれだけ先の経路上に置く"),
        ParamSpec(group=GROUP_AVOID, key="detour_budget_ms", label="Hybrid A* の時間予算", min=20.0,
                  max=200.0, default=60.0, step=10.0, unit="ms",
                  note="★1回の経路探索の上限。Pi では指令が150ms途切れると止まる（デッドマン）"),
    )

    #: 保存済み地図を読んだとき、中心線からラインを引き直すか（`slam2d_route` は自前で作る）
    _reline_on_load = True

    def __init__(self) -> None:
        self._line_pool: ThreadPoolExecutor | None = None
        self._last_p: dict[str, float] | None = None
        #: 自動運転に入っているか（`planning_node` が毎周期 `set_engaged` で渡す）。
        #: **入っていない EXPLORE は人のラジコンでの地図作成**として扱う。
        #: `set_engaged` を呼ばない呼び出し元（ベンチ・テスト）では従来どおり FTG で走る
        self._engaged = True
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
        #: 周回の数え上げ（`_count_lap`）。前の周期の最寄り点の添字と、前回数えてから経路に
        #: 沿って進んだ距離 [m]
        self._prev_idx = -1
        self._lap_s = 0.0
        #: 障害物の候補にしてよいセル（`obs_mod.free_mask`）と、それを作った (地図の版, 幅)
        self._obs_free: np.ndarray | None = None
        self._obs_free_key: tuple | None = None
        #: 経路に乗ったか（乗るまでは `join_speed` に抑える、`_join_cap`）
        self._joined = False
        #: 経路から外れて止まっている時間 [s] と、乗り直した回数（`_off_route`）
        self._off_t = 0.0
        self._rejoins = 0
        #: 自己位置を信用できない状態が続いている時間 [s]（`_localization_ok`）
        self._coast_t = 0.0
        #: 停車中に舵を動かしている時間 [s]（手動の地図作成での据え切りの注意）と直前の舵角
        self._still_steer_t = 0.0
        self._prev_steer = 0.0
        #: 障害物を避けている一時経路（`avoid.plan_offset`）と、その元にした経路
        self._avoid: av_mod.AvoidPlan | None = None
        self._avoid_path: rl_mod.RaceLine | None = None
        self._avoid_base: rl_mod.RaceLine | None = None
        self._avoid_ms = 0.0
        #: 壁と車体外形の余裕を測る距離場（地図の版ごとに作る、`_body_check`）
        self._body: rl_mod._BodyCheck | None = None
        self._body_seq = -1
        #: 塞がれて止まっている時間・進めずに止まっている時間 [s]
        self._blocked_t = 0.0
        self._stuck_t = 0.0
        #: Hybrid A* で抜けている（`DETOUR`）。同じ障害物で試した回数（失敗が続いたら止まって待つ）
        self._detour: ParkToPoint | None = None
        self._detour_error = ""
        self._detour_tries = 0
        self._dt = 0.0
        #: 今の経路の速度プロファイルを作った設定（`_retime`）
        self._speed_key: tuple | None = None
        #: 地図の読み込み・`line_margin` の変更で引き直しているライン（`_submit_line`）と、
        #: 今のライン（引き直し中ならその計算）の `line_margin`。`None` = 引き直さない
        self._drop_job("_line_job")
        self._line_margin: float | None = None
        #: BUILD で引いているライン（`_build`）
        self._drop_job("_build_job")
        self._prev_obs: list[obs_mod.Obstacle] = []
        self._obs: list[obs_mod.Obstacle] = []
        self._map: AutoMap | None = None
        self._map_seq = -_MAP_EVERY
        self._map_frozen = False
        self._map_dirty = True
        #: `snapshot()` で `AutoMap` を詰め直した回数（`AutoMap.route_seq` に載せる。版は戻さない）
        self._map_ver = getattr(self, "_map_ver", 0)
        #: 保存済み地図から読み込んだ中心線（`self.centerline`はEXPLORE→BUILDで
        #: 自前生成するものだけを持つので、読み込み経路はここに置く。
        #: `snapshot()`が両方を出し分ける）
        self._loaded_centerline_xy: np.ndarray | None = None
        #: 進行中のグローバルローカリゼーション（`LOCATE`段でのみ非None）
        self._localizer: GlobalLocalizer | None = None
        #: 粗探索の上位候補のうち、まだ仕上げていないもの（`None` = 粗探索中）と仕上げた結果
        self._loc_cands: list | None = None
        self._loc_scored: list = []
        #: 地図パネルのクリックによる絞り込みヒント（mapフレーム座標）
        self._loc_hint: tuple[float, float] | None = None
        #: 地図作成直後に走り出すときの「直前の自己位置」（`request_load`参照）
        self._locate_seed: tuple[float, float] | None = None
        self._load_error = ""
        #: 地図の読み込みに失敗した（`request_load`）。解くまで指令を出さない
        self._load_failed = False
        #: BUILD完了時に自動保存した地図の名前（`DONE`段の表示用）
        self._saved_map_name = ""
        self._save_error = ""

    def on_vehicle_state(self, vs: VehicleState) -> None:
        """`planning_node`が`vehicle_state`を受けるたびに呼ぶ（100Hz）。

        点ごとの脱スキュー（`slam2d/core/deskew.py`）に使う twist の履歴を
        SLAM へ流し込むだけ。`plan()`は10Hzでしか呼ばれないので、ここで
        受けておかないと1周100msのあいだの回頭の変化が追えない。
        """
        self.slam.on_vehicle_state(vs)

    def set_engaged(self, engaged: bool) -> None:
        self._engaged = bool(engaged)

    def on_disengage(self) -> None:
        """自動運転を解いた（`planning_node` が `reset()` の代わりに呼ぶ）。

        地図作成中なら**地図を残して**人のラジコンでの地図作成へ移る——FTG で走らせて
        いる途中に人が操作を奪っても、それまでの地図を捨てない。それ以外の段は
        従来どおり作り直す（次に engage したとき前回の舵の続きから動き出さないため）。
        """
        self._load_failed = False
        if self.phase == EXPLORE and self.slam.trajectory:
            self.ftg.reset()
            return
        self.reset()

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
        if (loaded is not None and self._reline_on_load and len(loaded.raceline_xy) < 3):
            self._load_error = f"地図『{name}』にはレーシングラインが無い"
            self._load_failed = True
            return
        if loaded is None:
            # ★ 走らせない。GUI の「レーシングライン走行」は読み込みと engage を同じ
            #   メッセージで送るので、黙って今の段（EXPLORE）に残ると、読めない地図を
            #   指定しただけで FTG の地図作成走行が始まった
            self._load_error = f"地図『{name}』を読み込めない"
            self._load_failed = True
            return
        self._load_failed = False
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
        # ラインが無い地図（経路を作れないまま保存した地図）は、経路を自前で作る planner
        # （`slam2d_route`）でだけ走れる
        self.path = None
        if len(loaded.raceline_xy) >= 3:
            v = loaded.raceline_v
            if len(v) != len(loaded.raceline_xy):   # 速度はどのみち `_retime` が作り直す
                v = np.zeros(len(loaded.raceline_xy))
            self.path = rl_mod.RaceLine(
                xy=loaded.raceline_xy, v=v, kappa=rl_mod.curvature(loaded.raceline_xy),
                length=mapstore.polyline_length(loaded.raceline_xy))
        self._loaded_centerline_xy = loaded.centerline_xy
        self.centerline = None
        # ★ 保存したラインの形は地図を作ったときのコードと設定のまま。中心線から今の
        #   設定で引き直す（重いのでワーカーで。できるまでは保存したラインで走る）
        self._drop_job("_line_job")
        self._line_margin = None
        self._submit_line(self._last_p or {s.key: s.default for s in self.params})
        self._speed_key = None            # 速度は今の設定で作り直す（`_retime`）
        self.phase = LOCATE
        self._localizer = None
        self._loc_cands = None
        self._load_error = ""
        self.laps = 0
        self._hint = -1
        self._map_dirty = True

    def _submit_line(self, p: dict[str, float]) -> None:
        """読み込んだ地図の中心線から、今の設定でラインを引き直す（ワーカーへ投げる）。"""
        cl = self._loaded_centerline_xy
        if not self._reline_on_load or cl is None or len(cl) < 20:
            return
        g = self.slam.grid
        self._drop_job("_line_job")
        self._line_job = self._pool().submit(
            _optimize_line, g.trinary().copy(), g.resolution, tuple(g.origin),
            np.asarray(cl).copy(), dict(p), self.vehicle)
        self._line_margin = float(p["line_margin"])

    def _drop_job(self, name: str) -> None:
        """ワーカーの計算の結果を捨てる。まだ始まっていなければ取り消す（ワーカーは1本なので、
        古い計算が列に残っていると新しい計算がそのぶん待たされる）。"""
        job = getattr(self, name, None)
        if job is not None:
            job.cancel()
        setattr(self, name, None)

    def _poll_line(self) -> None:
        """地図の読み込みで引き直したラインができていれば差し替える（`request_load`）。"""
        job = self._line_job
        if job is None or not job.done():
            return
        self._line_job = None
        try:
            self.path = job.result()
        except Exception as e:                      # noqa: BLE001  今のラインで走り続ける
            self._load_error = f"ラインを引き直せなかった（今のラインで走る）: {e}"
            return
        self._load_error = ""
        self._hint = -1
        self._prev_idx = -1
        self._speed_key = None
        self._map_dirty = True

    def request_locate_hint(self, x: float, y: float) -> None:
        self._loc_hint = (x, y)
        self._localizer = None     # 探索中なら絞り込んでやり直す
        self._loc_cands = None

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

        if self.phase == LOCATE and not self._load_failed:
            # ★ 姿勢が分かるまで追跡（`slam.update`）は回さない。回しても毎周期「見失い」で、
            #   3周期目からは探し直しの総当たりまで走って結果は捨てられる（LOCATE の
            #   `plan()` の6割）。姿勢は `_locate` が `set_pose` で入れる
            self.slam.idle(yaw_rate=vs.yaw_rate, speed=vs.speed)
            st.pose_x, st.pose_y, st.pose_yaw = self.slam.pose
            st.laps = self.laps
            return self._locate(st, scan, p)

        # ★ 回頭はジャイロ、前進は`speed`（射影済み・ローパス済みの値）。
        #   `wheel_speed[]`は低速でばたつくので渡してはいけない
        u = self.slam.update(scan, dt, yaw_rate=vs.yaw_rate, speed=vs.speed)
        st.pose_x, st.pose_y, st.pose_yaw = u.pose.x, u.pose.y, u.pose.yaw
        st.match_score = u.score
        st.lap_progress = self.slam.lap_progress()
        st.laps = self.laps

        if self._load_failed:
            st.reason = f"{self._load_error}。地図を選び直すか、自動運転を解除してください"
            return st
        if self.phase == EXPLORE:
            return self._explore(st, scan, vs, p, dt, u.lost)
        if self.phase == BUILD:
            return self._build(st, p)
        if self.phase == DONE:
            return self._done(st)
        return self._race(st, scan, vs, p, u.lost)

    # ── EXPLORE ──

    def _explore(self, st: AutoState, scan: Scan, vs: VehicleState,
                 p: dict[str, float], dt: float, lost: bool) -> AutoState:
        lap_msg = ""
        lap_done, explore_done = self._check_lap(p)
        if lap_done:
            if explore_done:
                self.slam.freeze()
                self.phase = st.phase = BUILD
                self._build_step = 0
                st.reason = "地図を確定した。経路を作っている"
                return st
            lap_msg = f"{self.laps}周目を検出した"

        if not self._engaged:
            return self._explore_manual(st, vs, p, dt, lost, lap_msg)

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

    def _explore_manual(self, st: AutoState, vs: VehicleState, p: dict[str, float], dt: float,
                        lost: bool, lap_msg: str) -> AutoState:
        """人のラジコンで地図を作っている（自動運転に入っていない EXPLORE）。

        指令は出さない（`ready=False`。MANUAL の指令はそのまま STM32 へ行く）。SLAM は
        `plan()` の冒頭で走り方に関係なく更新されているので、ここは案内を出すだけ。
        周回の自動判定もそのまま効き、出発点へ戻れば BUILD へ進む。
        """
        moving = abs(vs.steer_actual - self._prev_steer) > math.radians(1.0)
        self._prev_steer = vs.steer_actual
        if abs(vs.speed) < 0.03 and moving:
            self._still_steer_t += dt
        else:
            self._still_steer_t = max(0.0, self._still_steer_t - dt)

        want = int(p["explore_laps"])
        head = (f"手動で地図作成 {self.laps}/{want}周（{st.lap_progress:.2f}周ぶん回頭）。"
                f"ラジコンで{want}周走って出発点へ戻るか、『地図を確定』を押す")
        notes = []
        if self.slam.grid.out_of_bounds > OUT_OF_BOUNDS_WARN:
            notes.append(f"★地図からはみ出している（{self.slam.grid.out_of_bounds}本）")
        if lost:
            notes.append("自己位置を見失っている（ゆっくり走る）")
        if self._still_steer_t > 2.0:
            notes.append("★停車中の据え切りはステアMDが過熱する。走りながら切る")
        if lap_msg:
            notes.append(lap_msg)
        st.reason = "・".join([head] + notes)
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
        try:
            g = self.slam.grid
            # 経路が無くても地図は保存する（`slam2d_route` が経路を作れなかったとき。
            # ラインが空の地図は `slam2d_route` でだけ読める）
            mapstore.save_map(
                name, resolution=g.resolution, origin_x=g.origin[0], origin_y=g.origin[1],
                trinary=g.trinary(),
                centerline_xy=(self.centerline.xy if self.centerline is not None
                              else np.zeros((0, 2))),
                raceline_xy=self.path.xy if self.path is not None else np.zeros((0, 2)),
                raceline_v=self.path.v if self.path is not None else np.zeros(0))
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
        if not self._saved_map_name:
            st.reason = (f"経路はできたが、{self._save_error or '地図を保存できていない'}。"
                         "地図パネルから手動で保存してください")
            return st
        st.reason = (f"地図『{self._saved_map_name}』を保存した。"
                     "レーシングライン走行で開始してください（車を動かしてもよい）")
        return st

    def _last_lap(self) -> np.ndarray:
        traj = self.slam.trajectory_array()
        if len(self._lap_idx) >= 2:
            a, b = self._lap_idx[-2], self._lap_idx[-1]
            if b - a >= 20:
                return forward_traj(traj[a:b])
        return forward_traj(traj)

    # ── LOCATE ──

    def _locate(self, st: AutoState, scan: Scan, p: dict[str, float]) -> AutoState:
        """保存済み地図のどこに車が居るか分からない状態から自己位置を復元する。

        グローバルローカリゼーション（`slam2d.core.localize.GlobalLocalizer`）は
        1周期に候補角度1つぶんしか進めない（計算コストを1周期に収める設計、
        `localize.py`のモジュールdocstring参照）ので、この段の間は車両を
        `ready=False`で静止させたまま複数周期にわたって`step()`を呼び続ける。
        """
        pts = self.slam.deskew_scan(scan)
        seeded = self._loc_hint is None and self._locate_seed is not None

        if self._loc_cands is None:
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
            if not cands:
                if seeded:
                    self._locate_seed = None      # 種の周りに候補が無い。全域で探し直す
                st.reason = "自己位置の候補が見つからない"
                return st
            self._loc_cands, self._loc_scored = list(cands), []

        # ★ 粗探索（0.2m間隔・15°刻み）の得点だけで決めない。上位候補を
        #   それぞれ総当たり＋Gauss-Newton で仕上げ、**当たり率**で比べる。
        #   粗い尺度では並ぶ候補（オーバルの反対側の直線など）も、
        #   仕上げるとはっきり差がつく（`slam2d/core/localize.py`参照）。
        #   1候補の仕上げは Pi で数十ms かかるので **1周期に1候補**（まとめると
        #   指令が `cmd_deadman_ms` を超えて途切れる。停止中なので点群は変わらない）
        c = self._loc_cands.pop(0)
        ref = self.slam.refine(pts, Pose2D(c.x, c.y, c.yaw))
        if ref is not None:
            self._loc_scored.append((ref.inlier, ref))
        if self._loc_cands:
            st.reason = f"自己位置の候補を仕上げている（残り {len(self._loc_cands)}）"
            return st
        scored, self._loc_cands, self._loc_scored = self._loc_scored, None, []
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
        self.slam.set_pose(*best.pose)
        self.phase = st.phase = RACE
        self._locate_seed = None
        # クリックのヒントは使い切り（残すと、次に読む別の地図の探索までその周りに絞る）
        self._loc_hint = None
        self._prev_idx = -1
        self._lap_s = 0.0
        self._hint = -1
        self._joined = False
        self._off_t = 0.0
        self._rejoins = 0
        self._map_dirty = True
        # ★ `st.match_score`はこの`plan()`冒頭の`self.slam.update()`（`_locate()`
        # 呼び出し前の古い姿勢からの追跡マッチ、当然ながら未知の領域を指して
        # 失敗する）の値のまま残っている。ここで求め直した値に置き換えないと、
        # RACEへ切り替わった瞬間だけ「一致度0」を報告してしまう
        st.match_score = best.inlier
        st.pose_x, st.pose_y, st.pose_yaw = best.pose
        st.reason = f"自己位置を復元した（一致度 {best.inlier:.2f}）。走行を開始する"
        return st

    # ── RACE ──

    def _race(self, st: AutoState, scan: Scan, vs: VehicleState,
              p: dict[str, float], lost: bool) -> AutoState:
        if self.phase == DETOUR:
            return self._detour_step(st, scan, vs, p)
        if self.path is None:
            st.reason = "経路がまだ無い"
            return st
        # ★ `line_margin` はラインの形を決める。走行中に変えたら引き直す（以前は速度だけ
        #   作り直し、線は地図を読んだときの値のまま。障害物の帯と回避の余裕は新しい値を
        #   すぐ使うので、表示・線・判定が食い違った）。できるまでは今のラインで走る
        if (self._line_margin is not None and self._line_job is None
                and float(p["line_margin"]) != self._line_margin):
            self._submit_line(p)
        self._retime(p)

        coasting = self._localization_ok(st, lost, p)
        if coasting is None:
            return st

        pose = self.slam.pose
        pp, hit, dist = self._track(st, scan, vs, p, self.path, pose)
        self._count_lap(self.path, pp.index)
        st.laps = self.laps

        if self._off_route(st, pp, vs, p):
            return st

        st.ready = True
        st.target_speed = pp.speed
        st.free_ahead = dist if hit is not None else math.inf
        self._join_cap(st, pp, self._avoid_path or self.path, pose, p)
        if coasting:
            st.target_speed = min(st.target_speed, p["coast_speed"])

        if self._obstacle_response(st, hit, dist, vs, p, self.path, pp.index):
            return st

        st.reason = (f"{self.laps}周走行中・速度 {st.target_speed:.2f} m/s・"
                     f"横偏差 {pp.cross_track * 100:+.0f}cm"
                     + ("" if self._joined else "・経路に乗るまで減速")
                     + self._avoid_note() + self._coast_note(coasting, p)
                     + (f"・★{self._load_error}" if self._load_error else ""))
        return st

    def _count_lap(self, path: rl_mod.RaceLine, index: int) -> None:
        """閉じた経路の最寄り点の添字から周回を数える（`slam2d_route` と共通）。

        添字が末尾→先頭へ回っても、半周も進んでいなければ数えない（走り出しの位置が
        経路の終端の直前だと、動いた瞬間に「1周」と数えてしまう）。進んだ距離は
        **符号付きの巡回差**で積む——`(−1) % n` のように足すと、最寄り点が1つ戻った
        だけでほぼ1周ぶん進んだことになり、この歯止めが効かなくなる。
        """
        n = len(path)
        if self._prev_idx >= 0:
            di = (index - self._prev_idx + n // 2) % n - n // 2
            self._lap_s += di * path.length / n
        if lap_crossed(self._prev_idx, index, n) and self._lap_s > 0.5 * path.length:
            self.laps += 1
            self._lap_s = 0.0
        self._prev_idx = index

    # ── 障害物: 検出・横へ避ける・止まる・Hybrid A* で抜ける（`slam2d_route` と共通） ──

    def _track(self, st: AutoState, scan: Scan, vs: VehicleState, p: dict[str, float],
               path: rl_mod.RaceLine, pose):
        """経路（障害物を避けている間は一時経路）を追い、前方を塞ぐ障害物を調べる。

        塞がれていて「避ける」が有効なら、その場で横へ避ける一時経路を引いて乗り換える
        （止まらずに済むなら止まらない）。戻り値は `(追従の結果, 塞ぐ障害物, そこまでの距離)`。
        """
        cfg = pursuit_config(p, self.vehicle)
        self._detect_obstacles(st, scan, p)
        if self._avoid is not None and (self._avoid_base is not path
                                        or self._avoid_passed(self._hint)):
            self._clear_avoid()
        followed = self._avoid_path or path
        pp = follow(followed, pose, vs.speed, vs.steer_actual, cfg, hint=self._hint)
        hit, dist = self._blocking(followed, pp.index, vs, p)
        if hit is not None and self._still_clear(hit, pp.index):
            hit, dist = None, math.inf
        if (hit is not None and p["obstacle_avoid"] > 0
                and self._plan_avoid(path, pp.index, hit, vs, p, kappa_frac=0.6)):
            followed = self._avoid_path
            pp = follow(followed, pose, vs.speed, vs.steer_actual, cfg, hint=pp.index)
            hit, dist = self._blocking(followed, pp.index, vs, p)
        self._hint = pp.index
        st.target_steer = pp.steer
        st.heading = pp.steer
        st.cross_track = pp.cross_track
        st.target_x, st.target_y = pp.target
        return pp, hit, dist

    def _detect_obstacles(self, st: AutoState, scan: Scan, p: dict[str, float]) -> None:
        """障害物を検出して `self._obs`（2周期続いたもの）と `st.obstacles` に載せる。"""
        g = self.slam.grid
        pad = p["obstacle_pad"]
        key = (g.seq, pad)
        if key != self._obs_free_key:
            # 凍結地図では変わらない。毎周期作ると格子全体の膨張で数msかかる
            self._obs_free = obs_mod.free_mask(g, pad)
            self._obs_free_key = key
        now = obs_mod.detect(g, self.slam.deskew_scan(scan), self.slam.pose,
                             wall_pad=pad, free=self._obs_free)
        self._obs = obs_mod.confirm(now, self._prev_obs)
        self._prev_obs = now
        st.obstacles = [v for o in self._obs for v in (o.x, o.y, o.r)]

    def _blocking(self, path: rl_mod.RaceLine, index: int, vs: VehicleState,
                  p: dict[str, float]) -> tuple[obs_mod.Obstacle | None, float]:
        """経路の前方を塞ぐ障害物とその距離（base_link から経路に沿って）。

        避けるときは、止まれる距離に加えて**避け始める長さ**ぶん先まで見る
        （`obstacle_stop` の距離だけでは、見つけた時にはもう横へずれる余地が無い）。
        """
        ahead = p["obstacle_stop"]
        if p["obstacle_avoid"] > 0:
            v = max(vs.speed, 0.0)
            ahead = max(ahead, v * v / (2.0 * max(p["a_brake"], 0.1)) + 0.6 * v + 1.5)
        n = len(path)
        return obs_mod.blocking(
            self._obs, path.xy, index,
            half_width=self.vehicle.half_width + p["line_margin"],
            ahead_m=ahead + self.vehicle.front_overhang,
            step=path.length / (n if path.closed else max(1, n - 1)),
            skip_m=self.vehicle.front_overhang, closed=path.closed)

    def _obstacle_response(self, st: AutoState, hit: obs_mod.Obstacle | None, dist: float,
                           vs: VehicleState, p: dict[str, float], path: rl_mod.RaceLine,
                           index: int) -> bool:
        """塞がれていたら止まる。止まっても避けられなければ Hybrid A* で抜ける。止めたら True。

        「避ける」が有効なら、**減速して止まるのは開いた経路（停止点へ向かう経路）でも同じ**。
        横へ避ける・Hybrid A* で抜けるのは閉じた経路だけ（`_plan_avoid`）。★以前は開いた経路で
        両方とも切れていて、`obstacle_stop` が 0（既定）だと障害物に減速も制動もしなかった。
        """
        slow_on = p["obstacle_avoid"] > 0
        avoid_on = slow_on and path.closed
        # ★ 駆動が入っていない間（engage 済みで人が ARM を押す前）の静止は、スタックでも
        #   「止まってから避け直す」待ちでもない。数えると ARM 前に Hybrid A* へ入り、ARM した
        #   瞬間に後退を含む操縦から始まった
        driving = bool(vs.armed)
        if hit is None:
            self._blocked_t = 0.0
            # 進めと言っているのに止まっている（壁に擦った・段差・見えない物）
            if avoid_on and driving and st.target_speed > 0.1 and abs(vs.speed) < 0.03:
                self._stuck_t += self._dt
            else:
                self._stuck_t = 0.0
                if abs(vs.speed) > 0.2:
                    self._detour_tries = 0          # 走り出せたら数え直す
            if avoid_on and self._stuck_t > p["stuck_s"] and self._detour_tries < DETOUR_TRIES:
                self._start_detour(st, path, index, None, 0.0, p, "進めない（スタック）")
                return True
            return False

        self._stuck_t = 0.0
        front = self.vehicle.front_overhang
        gap = max(0.0, dist - front)
        stop_at = p["obstacle_stop"]
        if slow_on:
            # 横へ避けられなかった（避けていればここへは来ない）。障害物の手前
            # `_STOP_GAP` で止まれる速度を上限にして、滑らかに減速する。★制動距離だけで
            # 止め始めると、1.9m/s のまま 0.8m 手前まで来てから急制動した
            a = max(p["a_brake"], 0.1)
            room = max(0.0, gap - _STOP_GAP - max(vs.speed, 0.0) * p["delay_s"])
            st.target_speed = min(st.target_speed, math.sqrt(2.0 * a * room))
            stop_at = max(stop_at, _STOP_GAP + 0.05)
        if stop_at <= 0.0 or gap > stop_at:
            if slow_on:
                st.reason = f"前方 {gap:.1f}m に避けられない障害物。止まれる速度へ減速"
                return True
            return False
        st.brake = True
        st.target_speed = 0.0
        ang = math.degrees(math.atan2(hit.y - st.pose_y, hit.x - st.pose_x) - st.pose_yaw)
        ang = (ang + 180.0) % 360.0 - 180.0
        st.reason = f"前方 {gap:.1f}m（{ang:+.0f}°）に障害物。停止"
        if slow_on and not avoid_on:
            st.reason += "（停止点へ向かう経路では避けない。どくまで待つ）"
        if not avoid_on or not driving or abs(vs.speed) >= 0.05:
            return True
        # 止まってから: 曲がれる限界の近くまで使って横へ避け直す → だめなら Hybrid A*
        self._blocked_t += self._dt
        if self._blocked_t >= 0.5 and self._plan_avoid(path, index, hit, vs, p, kappa_frac=0.9):
            st.reason += "。止まってから避ける経路を引いた"
            self._blocked_t = 0.0
        elif self._blocked_t >= 1.0 and self._detour_tries < DETOUR_TRIES:
            self._start_detour(st, path, index, hit, dist, p, "横へ避けられない")
        elif self._detour_tries >= DETOUR_TRIES:
            st.reason += (f"。Hybrid A* でも{DETOUR_TRIES}回抜けられなかったので待つ"
                          + (f"（{self._detour_error}）" if self._detour_error else ""))
        return True

    def _plan_avoid(self, path: rl_mod.RaceLine, index: int, hit: obs_mod.Obstacle,
                    vs: VehicleState, p: dict[str, float], *, kappa_frac: float) -> bool:
        if not path.closed:
            return False                  # 停止点へ向かう開いた経路では止まるだけ
        t0 = time.perf_counter()
        v = self.vehicle
        cfg = av_mod.AvoidConfig(
            half_width=v.half_width, front=v.front_overhang, rear=v.rear_overhang,
            kappa_max=v.kappa_max, footprint=tuple(v.footprint),
            # 避けた後の経路で `_blocking` が同じ障害物を拾わないよう、経路の余裕より広く取る
            margin=max(p["avoid_margin"], p["line_margin"]) + 0.01,
            clear=p["avoid_clear"], len_min=p["avoid_len_min"], kappa_frac=kappa_frac)

        speed = max(vs.speed, 0.0)
        ahead = speed * speed / (2.0 * max(p["a_brake"], 0.1)) + 0.6 * speed + 3.5
        # ★ 検出した中心は**見えている表面の点の重心**で、実物の中心より手前に寄る。円柱の
        #   半周が見えていると、実物は重心から推定半径の約1.4倍まで広がる。そのぶん大きく
        #   扱わないと、避けている途中で見る角度が変わるたびに「まだ塞いでいる」と判定されて
        #   止まった（sim.bench の toyota、半径10cmの円柱）
        #   遠くで見つけたときは点が少なく半径を大きく過小評価する（10cm の円柱が 3.6cm に
        #   見えた）ので下限を置く
        def grown(o: obs_mod.Obstacle) -> obs_mod.Obstacle:
            return obs_mod.Obstacle(o.x, o.y, max(_OBS_GROW * o.r, _OBS_MIN_R), o.n)
        grow = [grown(o) for o in self._obs]
        target = grown(hit)
        plan = av_mod.plan_offset(path, index, target, grow, self._body_check(), cfg,
                                  v_now=speed, ahead_m=ahead)
        self._avoid_ms = (time.perf_counter() - t0) * 1000.0
        if plan is None:
            return False
        rl = rl_mod.retime(plan.path, **speed_kwargs(p, self.vehicle))
        self._avoid_path = av_mod.cap_speed(rl, plan.window, p["avoid_speed"], p["a_brake"],
                                            p["a_accel"])
        self._avoid, self._avoid_base = plan, path
        return True

    def _blend_back(self, path: rl_mod.RaceLine, p: dict[str, float]) -> None:
        """今の横ずれから滑らかに経路へ戻る一時経路に乗せる（`avoid.blend_in`）。"""
        if not path.closed:
            return
        x, y, _ = self.slam.pose
        j = nearest_index(path, x, y)
        nrm = cl_mod.normals(path.xy)[j]
        d0 = float((x - path.xy[j, 0]) * nrm[0] + (y - path.xy[j, 1]) * nrm[1])
        plan = av_mod.blend_in(path, j, d0, max(1.5, 6.0 * abs(d0)), self._body_check())
        rl = rl_mod.retime(plan.path, **speed_kwargs(p, self.vehicle))
        self._avoid_path = av_mod.cap_speed(rl, plan.window, p["join_speed"], p["a_brake"],
                                            p["a_accel"])
        self._avoid, self._avoid_base = plan, path
        self._hint = j

    def _still_clear(self, hit: obs_mod.Obstacle, index: int) -> bool:
        """避けている障害物が「塞いでいる」と出ても、避ける経路の残りで車体が当たらないなら True。

        ★ 遠くで見つけたときは点が少なく半径を小さく見積もる。近づいて見積もりが大きく
        なると、帯（車体半幅＋余裕＋半径）の判定では「まだ塞いでいる」と出て、避けている
        途中で止まった（sim.bench の toyota で2周目）。当たるかどうかを今の見積もりで直接測る。
        """
        a = self._avoid
        if a is None or a.obstacle is None or self._avoid_path is None:
            return False
        if math.hypot(hit.x - a.obstacle.x, hit.y - a.obstacle.y) > 0.35 + hit.r:
            return False                       # 別の障害物
        n = len(self._avoid_path)
        end = int(a.window[-1])
        k = (end - index) % n
        idx = (index + np.arange(k + 2)) % n
        clr = av_mod.clearance_along(self._avoid_path.xy[idx], self._footprint(), [hit])
        return clr >= 0.02

    def _avoid_passed(self, index: int) -> bool:
        """車が避ける窓の終わりを過ぎたか（元の経路へ戻してよい）。"""
        assert self._avoid is not None and self._avoid_path is not None
        n = len(self._avoid_path)
        d = (index - int(self._avoid.window[-1])) % n
        return 0 < d < n // 2

    def _clear_avoid(self) -> None:
        self._avoid = self._avoid_path = self._avoid_base = None

    def _avoid_note(self) -> str:
        if self._avoid is None:
            return ""
        if self._avoid.obstacle is None:
            return f"・経路へ戻っている（横ずれ {self._avoid.offset * 100:+.0f}cm から）"
        side = "左" if self._avoid.offset > 0 else "右"
        return (f"・★障害物を{side}へ {abs(self._avoid.offset) * 100:.0f}cm 避けている"
                f"（余裕 {self._avoid.clearance * 100:.0f}cm・計算 {self._avoid_ms:.0f}ms）")

    def _body_check(self) -> rl_mod._BodyCheck:
        g = self.slam.grid
        if self._body is None or self._body_seq != g.seq:
            self._body = rl_mod._BodyCheck(g, self._footprint(), closed=False)
            self._body_seq = g.seq
        return self._body

    def _footprint(self) -> tuple:
        v = self.vehicle
        if v.footprint:
            return tuple(v.footprint)
        return ((v.front_overhang, v.half_width), (v.front_overhang, -v.half_width),
                (-v.rear_overhang, -v.half_width), (-v.rear_overhang, v.half_width))

    def _start_detour(self, st: AutoState, path: rl_mod.RaceLine, index: int,
                      hit: obs_mod.Obstacle | None, dist: float, p: dict[str, float],
                      why: str) -> None:
        """止まって、経路上の障害物の先の姿勢を目標に Hybrid A*（`ParkToPoint`）へ引き継ぐ。"""
        order, s_fwd = av_mod.forward_order(path, index)
        ahead = p["detour_ahead"]
        if hit is not None:
            ahead += dist + hit.r + self.vehicle.rear_overhang
        k = min(int(np.searchsorted(s_fwd, ahead)), len(order) - 2)
        j, j2 = int(order[k]), int(order[k + 1])
        gx, gy = float(path.xy[j, 0]), float(path.xy[j, 1])
        gyaw = math.atan2(float(path.xy[j2, 1]) - gy, float(path.xy[j2, 0]) - gx)
        x, y, yaw = self.slam.pose
        c, s = math.cos(-yaw), math.sin(-yaw)
        lx, ly = c * (gx - x) - s * (gy - y), s * (gx - x) + c * (gy - y)
        lyaw = (gyaw - yaw + math.pi) % (2.0 * math.pi) - math.pi
        self._detour = ParkToPoint()
        self._detour.request_park_target(lx, ly, lyaw)
        self._detour_error = ""
        self._detour_tries += 1
        self._clear_avoid()
        self._blocked_t = self._stuck_t = 0.0
        self.phase = st.phase = DETOUR
        st.brake = True
        st.target_speed = 0.0
        st.reason = f"{why}。Hybrid A* で {math.hypot(lx, ly):.1f}m 先の経路へ抜ける"

    def _detour_step(self, st: AutoState, scan: Scan, vs: VehicleState,
                     p: dict[str, float]) -> AutoState:
        assert self._detour is not None
        pp = {s.key: s.default for s in ParkToPoint.params}
        pp["plan_budget_ms"] = p["detour_budget_ms"]
        pp["pos_tol_m"] = _DETOUR_POS_TOL
        pp["yaw_tol_deg"] = _DETOUR_YAW_TOL_DEG
        pp["max_maneuver_s"] = _DETOUR_MAX_S
        t0 = time.perf_counter()
        pst = self._detour.plan(scan, vs, pp, self._dt)
        ms = (time.perf_counter() - t0) * 1000.0
        pst.mode, pst.planner, pst.phase = self.id, self.name, DETOUR
        pst.pose_x, pst.pose_y, pst.pose_yaw = st.pose_x, st.pose_y, st.pose_yaw
        pst.match_score = st.match_score
        pst.laps = self.laps
        self._detect_obstacles(pst, scan, p)
        if self._detour.done or self._detour.failed:
            ok = self._detour.done
            self._detour_error = "" if ok else pst.reason
            self._detour = None
            self.phase = pst.phase = RACE
            self._hint = -1
            self._joined = False
            pst.brake = True
            pst.target_speed = 0.0
            if ok and self.path is not None:
                self._blend_back(self.path, p)
            pst.reason = ("Hybrid A* で抜けた。経路に滑らかに戻る" if ok
                          else f"Hybrid A* で抜けられなかった（{self._detour_error}）。止まって待つ")
            return pst
        pst.reason = f"迂回（Hybrid A*、{ms:.0f}ms）: {pst.reason}"
        return pst

    def _localization_ok(self, st: AutoState, lost: bool, p: dict[str, float]) -> bool | None:
        """自己位置を信用して走れるか。`None` = 止まる、`True` = 推測航法で続行、`False` = 正常。

        見失い（`lost`）や一致度の低下が `coast_s` 以内なら、推測航法の姿勢（見失った周期は
        frontend が予測をそのまま採る）で経路を追い続ける。1周期でも止めると、壁越しに
        別の部屋が一瞬見えただけで急停止する。経路に乗る前（自己位置の復元直後）は続行しない。
        """
        if not (lost or st.match_score < p["min_score"]):
            self._coast_t = 0.0
            return False
        self._coast_t += self._dt
        if self._joined and p["coast_s"] > 0.0 and self._coast_t <= p["coast_s"]:
            return True
        st.reason = (f"自己位置が信用できない（一致度 {st.match_score:.2f} < "
                     f"{p['min_score']:.2f}"
                     + (f"・{self._coast_t:.1f}s 続いた" if p["coast_s"] > 0.0 else "") + "）")
        return None

    def _coast_note(self, coasting: bool, p: dict[str, float]) -> str:
        if not coasting:
            return ""
        return f"・★推測航法で続行 {self._coast_t:.1f}/{p['coast_s']:.1f}s"

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
        # ★ 詰め直したら経路の版を進める。`planning_node`・`telemetry_node` は
        #   `(map_seq, route_seq)` が変わったときだけ流すので、凍結した地図（`map_seq` が
        #   動かない）の上で経路だけ変わると配られなかった——BUILD でできた中心線とライン、
        #   読み込み後に引き直したライン、設定を変えて作り直した速度が GUI に届かず、
        #   DONE での手動保存も「保存できる地図が無い」になった
        self._map_ver += 1
        m.route_seq = self._map_ver
        self._map = m
        self._map_seq = g.seq
        self._map_frozen = g.frozen
        self._map_dirty = False
        return m
