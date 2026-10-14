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

import atexit
import math
import multiprocessing
import os
import sys
import time
from typing import NamedTuple
from concurrent.futures import Executor, ProcessPoolExecutor, ThreadPoolExecutor

import numpy as np

from slam2d.core.localize import GlobalLocalizer
from slam2d.core.types import Pose2D

from ..core.vehicle import Vehicle
from ..msgs.types import AutoMap, AutoState, Scan, VehicleState
from ..nav import avoid as av_mod
from ..nav import centerline as cl_mod
from ..nav import grip as grip_mod
from ..nav import obstacles as obs_mod
from ..nav import raceline as rl_mod
from ..nav.grid import pack_trinary
from ..nav.mpc import MPCConfig, MPCFollower
from ..nav.purepursuit import PursuitConfig, follow, nearest_index
from ..nav import speedctl
from ..nav.signals import SignalLights
from ..proto.generated.packets import MDS_COMM_OK
from . import mapstore
from ._slam2d_nav import WALL_HIT_WEIGHT, Slam2dNav, occgrid_from_trinary
from .base import ParamSpec, Planner
from .follow_the_gap import FollowTheGap
from .park_to_point import ParkToPoint

__all__ = ["Slam2dRaceLine"]

#: `DONE`は「地図と経路ができた。保存済み。人間が『レーシングライン走行』を
#: 押すのを待っている」段。**BUILD完了後、自動ではRACEに進まない**——地図作成
#: 直後にそのまま走り出すと、`request_load()`のLOCATE経由で手動再配置しても
#: 走れるようにした意味が無くなる（人間が地図内の好きな場所へ車を置き直す間
#: を作るため）
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
#: 車速が経路の速度よりこれ以上低ければ「追いつく途中」として扱う [m/s]（`_speed_cmd`）
_CATCH_UP_GAP = 0.3
#: 見通しの中で止まるとき、障害物の手前に残す間隔 [m]（`_sight_cap`）
_SIGHT_GAP = 0.05
#: 見通しの上限で 1 周がこの倍率を超えて遅くなったら reason に出す（`_sight_note`）
_SIGHT_NOTE_RATIO = 1.03
#: 障害物検出で「壁の近くを無視する幅」を経路に合わせて詰めるときの下限 [m]（`_obstacle_pad`。
#: 実機の自己位置のずれは p95 2〜3cm・最大 7cm。壁の点が隣の空きセルへ落ちるぶんは残す）
_OBS_PAD_MIN = 0.15
#: 車体のすぐ前（`blocking` が飛ばす足元の帯）で、車体外形と障害物の余裕がこれを切ったら止まる [m]
_NEAR_CLEAR = 0.02
#: 障害物で制動している途中に検出が途切れても、制動を保つ時間 [s]
_BRAKE_HOLD_S = 0.3
#: Hybrid A* で抜けた後、経路からの横ずれがこれ未満なら戻り経路を作らない [m]
_BLEND_MIN = 0.03
#: 引き直したラインへ乗り移ってよい、今の姿勢とのずれ（横 [m]・向き [rad]、`_try_adopt_line`）
_ADOPT_DIST = 0.04
_ADOPT_YAW = math.radians(6.0)
#: ローンチコントロールを要求する舵角の上限 [rad]（約7°。これより切っている発進は速度PIに任せる）
_LAUNCH_STEER_MAX = 0.12
#: 車速がこれ未満になったら「発進」と見なし、次に経路の速度へ届くまでローンチコントロールを要求する [m/s]
_LAUNCH_ARM_SPEED = 0.3
#: 車速が経路の速度のこれだけ手前に届いたら発進は終わり [m/s]
_LAUNCH_DONE_GAP = 0.1

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

#: 最小曲率の中心線へ引き戻す重み（`rl_mod.min_curvature_alpha` の `lam`）。0.02〜1.0 で
#: 見積もりラップが ±0.05s しか変わらない（toyota2）ので定数。壁との距離は `line_margin` で決める
LINE_LAM = 0.1
#: 「最適化 → 法線と幅を測り直す」の回数（`rl_mod.optimize` の `passes`）。2回で足り
#: （3回は往復して悪くなることがある）ので定数
LINE_PASSES = 2
#: 最小時間へ寄せる再重み付けの回数（`rl_mod.optimize` の `time_iters`）。1回で効果の大半が
#: 出て2回以上は変わらない（toyota2 の見積もり 14.08→13.23→13.24s）。同じ理由で定数
LINE_TIME_ITERS = 2
#: 見積もりのラップ時間を直接下げる仕上げ（`rl_mod.refine_time`）の評価回数の上限。
#: ワーカースレッドで回る。幅の巡回は最大4回で、1巡の縮みが 0.2% を切ったら終わる
#: （`rl_mod.REFINE_STOP`）ので、上限まで使うのは S 字の多いコースだけ。Mac で 2〜4 秒
LINE_REFINE_EVALS = 15000

#: GUI の組（`ParamSpec.group`）。地図作成（Follow the Gap）と本番走行を分けて見せる
GROUP_EXPLORE = "地図作成（Follow the Gap）"
GROUP_LINE = "経路生成"
GROUP_SPEED = "本番走行: 速度"
GROUP_STEER = "本番走行: 舵"
GROUP_SAFETY = "本番走行: 安全"
GROUP_AVOID = "本番走行: 障害物の回避"
GROUP_GRIP = "本番走行: 路面の限界の学習"
#: 学習で上げた速度を反映しない、車の先の距離 [m] と時間 [s]（足元の目標速度を跳ばさない、`_grip_hold`）
_GRIP_AHEAD_M = 1.5
_GRIP_AHEAD_S = 0.6
#: 壁との余裕は、その点からこの距離だけ先までの最小を使う [m]（外へ膨らむのは少し先）
_GRIP_WALL_AHEAD_M = 1.5
#: 学習した値を地図の横へ保存する間隔 [s]
_GRIP_SAVE_S = 10.0
#: 別の線へ乗り換えてから、学習を止めておく時間 [s]
_GRIP_SWITCH_QUIET_S = 1.5

#: 速度プロファイルを決める設定（変わったら `_retime` で作り直す）
SPEED_KEYS = ("v_max", "v_min")
#: 走行中に学習しない（`grip_learn` = 0）ときに、同定した限界の何割を使うか（横・加速・減速）。
#: 学習しないと、同定値が路面を表していなくても気づけないので、余裕を残す。学習するときは
#: 学習した値をそのまま使う（`nav/grip.py`）。以前の設定項目 `lat_use`・`accel_use`・`brake_use` の既定値
FIXED_LAT_USE = 0.9
FIXED_ACCEL_USE = 0.95
FIXED_BRAKE_USE = 0.9
#: 学習するときの減速の出発点（同定値に対する割合）。低めから始める——減速の見積もりが高すぎると
#: 最初のコーナーへ止まれない速度で入り、気づいた時点で打つ手が無い。車速が計画に付いて行けている間、
#: 1 周に 1 段ずつ上がる（`grip.LongLearner`。シムで真値 0.72 倍の路面のとき、0.9 から始めると最初の
#: コーナーで線から 33cm 外れた）
LEARN_BRAKE_INIT = 0.75
#: ラインの形を決める設定（変わったら引き直す）。ラインは見積もりのラップ時間を最小にするので、
#: 速度の設定も形に効く
LINE_KEYS = ("line_margin", "kappa_use") + SPEED_KEYS


def line_key(p: dict[str, float]) -> tuple:
    return tuple(float(p[k]) for k in LINE_KEYS)


def kappa_limit(p: dict[str, float], vehicle) -> float:
    """経路に許す曲率の上限 [1/m] = 車の曲がれる限界 × `kappa_use`。"""
    return vehicle.kappa_max * float(p.get("kappa_use", 1.0))

#: LiDAR の 1 周の時間 [s]（LD06 は 10Hz。障害物は最悪この時間だけ遅れて見える）
SCAN_PERIOD_S = 0.1


class Limits(NamedTuple):
    """走りの上限（`limits`）。走行中に学習した値か、同定値（`config/vehicle.toml`）× `FIXED_*_USE`。"""

    a_lat: float                           #: 横加速度 [m/s²]
    a_accel: float                         #: 加速度 [m/s²]
    a_brake: float                         #: 減速度 [m/s²]（速度指令を下げて出す。`brake` ではない）
    #: 障害物・停止点に対して、気づいてから減速が効き始めるまでの空走時間 [s]
    react_s: float


def limits(p: dict[str, float], vehicle, learned: tuple[float, float, float] | None = None) -> Limits:
    """走りの上限を出す。

    :param learned: 走行中に学習した `(横, 加速, 減速)` [m/s²]（`nav/grip.py`）。`None` なら同定値
        （`mu`・`drive_accel_m_s2`・`speed_decel_m_s2`）× `FIXED_*_USE`。人が決める「使う割合」は無い

    空走時間は、点群の周期（最悪そのぶん遅れて見える）＋指令が STM32 に届くまで（同定の
    `control_latency_s`）＋減速が立ち上がるまで（速度PIの時定数の半分）。
    """
    react = SCAN_PERIOD_S + cmd_latency(vehicle) + 0.5 * vehicle.speed_loop_tau_s
    if learned is not None:
        return Limits(a_lat=learned[0], a_accel=learned[1], a_brake=learned[2], react_s=react)
    return Limits(
        a_lat=FIXED_LAT_USE * vehicle.a_lat_max,
        a_accel=FIXED_ACCEL_USE * vehicle.a_accel_max(0.0),
        a_brake=FIXED_BRAKE_USE * vehicle.a_decel_max,
        react_s=react)


def cmd_latency(vehicle) -> float:
    """指令を決めてから STM32 が受理するまでの遅れ [s]（同定の遅延試験の値。未同定なら 0.02）。"""
    return vehicle.control_latency_s if vehicle.control_latency_s > 1e-4 else 0.02


def speed_key(p: dict[str, float]) -> tuple:
    return tuple(float(p[k]) for k in SPEED_KEYS)


def speed_kwargs(p: dict[str, float], vehicle, grip=None) -> dict:
    """`rl_mod.retime` / `optimize` へ渡す速度の設定。

    :param grip: 走行中に学習した値（`nav/grip.py`）の dict: `a_lat`（求める横加速度）・`a_grip`
        （横の限界＝摩擦円の半径）・`a_accel`・`a_brake` [m/s²]。横の 2 つは経路の点ごとの配列でもよい。
        `None` なら同定値 × `FIXED_*_USE`（`limits`）
    """
    if grip is not None:
        return dict(v_max=p["v_max"], v_min=p["v_min"], **grip)
    lim = limits(p, vehicle)
    return dict(v_max=p["v_max"], v_min=p["v_min"], a_lat=lim.a_lat,
                a_accel=lim.a_accel, a_brake=lim.a_brake,
                # 摩擦円は余裕を掛けた値ではなく車の限界で引く（横を 9 割使っていても、頂点で
                # 前後に使える力が 0 になるわけではない）
                a_grip=vehicle.a_lat_max)


#: 仕上げを始める出発点の数（`rl_mod.optimize` の `refine_starts`）。1 だと出発点の中心線の
#: わずかな違いで別の線に止まることがある（toyota2 の地図で見積もりが 6 通り中 1 通りだけ +7%）。
#: 2 でばらつきが 7.0→2.0%・1.3→0.2%（実機の地図）、3 にしても変わらない。時間は 2 倍
LINE_REFINE_STARTS = 2


def line_kwargs(p: dict[str, float], vehicle, refine: bool = True, grip: dict | None = None) -> dict:
    """`rl_mod.optimize` へ渡す経路と速度の設定（BUILD と地図の読み込みで共通）。

    :param grip: 走行中に学習した全体の値（`speed_kwargs` の `grip`、スカラー）。ラインは見積もりの
        ラップ時間を最小にするので、路面の限界が同定値と違えば線の形も変わる

    :param refine: False なら見積もりラップの仕上げをしない（0.1〜0.6 秒で引ける「まず走れる線」）。
        仕上げ込みは Mac で 6〜8 秒かかるので、**先に仕上げなしの線で走り出し、裏で仕上げて
        できたら差し替える**（`Slam2dRaceLine._adopt_line`、`Slam2dRoute._poll`）
    """
    return dict(half_width=vehicle.half_width, margin=p["line_margin"], lam=LINE_LAM,
                passes=LINE_PASSES, max_width=MAX_TRACK_WIDTH,
                # コーナーで車体の角が経路の外へ張り出すぶんを余裕に足す
                # （`nav/raceline.body_allowance`）
                front_overhang=vehicle.front_overhang, rear_overhang=vehicle.rear_overhang,
                # 薄い壁の先端を車体がかすめないか、外形で確かめる（`rl_mod._BodyCheck`）
                footprint=vehicle.footprint or None,
                time_iters=LINE_TIME_ITERS, kappa_max=kappa_limit(p, vehicle),
                refine_evals=LINE_REFINE_EVALS if refine else 0,
                refine_starts=LINE_REFINE_STARTS,
                **speed_kwargs(p, vehicle, grip))


def _optimize_line(trinary: np.ndarray, resolution: float, origin: tuple[float, float],
                   cl: cl_mod.Centerline | np.ndarray, p: dict[str, float],
                   vehicle, refine: bool = True, grip: dict | None = None) -> rl_mod.RaceLine:
    """レーシングラインを引く（**ワーカースレッドで走る**。地図の写しだけを触る）。

    `cl` が中心線でなく点列なら、保存済み地図の中心線として測り直してから引く。
    ★ 最小時間への寄せと車体外形の検査で、Pi では数百ms〜1s かかる。`plan()` の中で
    回すと指令が途切れる（`cmd_deadman_ms` 150ms）。
    """
    grid = occgrid_from_trinary(trinary, resolution=resolution, origin=origin, seq=0)
    if not isinstance(cl, cl_mod.Centerline):
        cl = cl_mod.build(grid, cl, step=PATH_STEP, max_width=MAX_TRACK_WIDTH)
    return rl_mod.optimize(grid, cl, **line_kwargs(p, vehicle, refine, grip))


def pursuit_config(p: dict[str, float], vehicle) -> PursuitConfig:
    return PursuitConfig(
        wheelbase=vehicle.wheelbase, max_steer=vehicle.max_steer,
        lookahead_k=p["look_k"], lookahead_min=p["look_min"], delay_s=steer_delay(p, vehicle),
        speed_preview_s=p["speed_preview"], ff_gain=p["steer_ff"],
        steer_map=vehicle.steer_map)


def steer_delay(p: dict[str, float], vehicle) -> float:
    """舵の遅延補償の時間 [s] = 同定した舵・ヨーの遅れ ＋ 指令が届くまで ＋ 手の補正 `delay_trim`。

    追従に使う姿勢は車速とジャイロで「今」へ進めてある（`Slam2dNav.pose_now`）ので、
    姿勢の古さの手当ては要らず、指令を出してから曲率に効くまでの遅れだけを見込む。
    """
    return max(0.0, vehicle.steer_delay_s + cmd_latency(vehicle) + p["delay_trim"])


def mpc_config(p: dict[str, float], vehicle) -> MPCConfig:
    return MPCConfig(
        wheelbase=vehicle.wheelbase, max_steer=vehicle.max_steer,
        horizon=int(p["mpc_horizon"]), dt=p["mpc_dt"], delay_s=p["mpc_delay_s"],
        q_y=p["mpc_q_y"], q_psi=p["mpc_q_psi"], r_d=p["mpc_r_d"],
        steer_map=vehicle.steer_map, servo_gain=vehicle.steer_servo_gain,
        speed_preview_s=p["speed_preview"])


# ── 重い計算（ライン・道路グラフ）を回す別プロセス ──
#
# ★ ワーカーはスレッドではなく**別プロセス**。ライン生成は Python のループと numpy の長い1回の
#   呼び出し（距離場など）でできていて、その間 GIL を離さない。スレッドだと `plan()` が待たされ、
#   Mac の sim.bench でも 1 周期の最大が 74ms → 140〜170ms に伸びた。デッドマンは 150ms で、
#   Pi は 2〜3 倍遅い。別プロセスなら GIL を共有しないので `plan()` は止まらない（Pi 5 は 4 コア）。
#   - 起動は `spawn`（`fork` は ZeroMQ のソケットやスレッドを持ったまま複製してしまう）
#   - 子は優先度を下げる（計画と SLAM のコアを奪わない）
#   - プロセスを作れない環境ではスレッドに落ちる（走れないよりは遅い方がよい）
#   - planner ごとに作らず 1 つを使い回す（起動に約 1 秒かかる。同時に走る planner は 1 つ）
_WORKER: Executor | None = None
_WORKER_IS_PROCESS = False


def _worker_init() -> None:
    try:
        os.nice(5)
    except OSError:
        pass


def _shutdown_worker() -> None:
    global _WORKER
    if _WORKER is not None:
        _WORKER.shutdown(wait=False, cancel_futures=True)
        _WORKER = None


def worker_pool() -> Executor:
    """重い計算を投げる先（1 つを使い回す。上のコメント）。"""
    global _WORKER, _WORKER_IS_PROCESS
    if _WORKER is None:
        try:
            if os.environ.get("SURGE_LINE_WORKER", "process") != "process":
                raise OSError("SURGE_LINE_WORKER でスレッドを指定")
            _WORKER = ProcessPoolExecutor(max_workers=1,
                                          mp_context=multiprocessing.get_context("spawn"),
                                          initializer=_worker_init)
            _WORKER_IS_PROCESS = True
        except (OSError, ValueError, ImportError) as e:
            print(f"# ライン生成を別プロセスで回せない（{e}）。スレッドで回す——計算中は plan() が"
                  "待たされる", file=sys.stderr, flush=True)
            _WORKER = ThreadPoolExecutor(max_workers=1, thread_name_prefix="line")
            _WORKER_IS_PROCESS = False
        atexit.register(_shutdown_worker)
    return _WORKER


def submit_job(fn, *args, **kwargs):
    """`fn(*args)` を別プロセスへ投げて `Future` を返す。プロセスが落ちていたら作り直して投げ直す。"""
    global _WORKER
    try:
        return worker_pool().submit(fn, *args, **kwargs)
    except Exception:                               # noqa: BLE001  BrokenProcessPool など
        _shutdown_worker()
        return worker_pool().submit(fn, *args, **kwargs)


#: 自己位置のヒント（地図のクリック）を有効とみなす時間 [s]
_LOC_HINT_TTL_S = 120.0

class Slam2dRaceLine(Planner):
    id = "slam2d_raceline"
    #: GUI のモード選択には出さない（分岐のないコースも `slam2d_route` の自動経路で走れる。こちらは
    #: `slam2d_route` の土台と `sim.bench` 用に残してある）
    listed = False
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
        ParamSpec(group=GROUP_EXPLORE, key="explore_gap_min", hidden=True, label="通れると見なす距離", min=0.2, max=3.0,
                  default=0.6, step=0.05, unit="m",
                  note="★この距離以上が続く方向を「隙間」と数える。**道幅の半分より"
                       "小さくすること**"),
        ParamSpec(group=GROUP_EXPLORE, key="explore_bubble", hidden=True, label="安全バブル半径", min=0.05, max=0.6,
                  default=0.12, step=0.01, unit="m",
                  note="最近傍の周りを侵入禁止にする半径。**車線の半幅より小さく**"),

        ParamSpec(group=GROUP_EXPLORE, key="wall_weight", hidden=True, label="壁の残りやすさ", min=2.0, max=16.0,
                  default=WALL_HIT_WEIGHT, step=1.0,
                  note="壁 = 当たり×この値 ≧ 素通り。**地図の壁が点線になる・経路が壁を抜けるなら上げる**"
                       "（作っている地図にその場で効く）。上げるほど壁が太り道が狭くなる。"
                       "★地図を確定した後と、保存済みの地図には効かない（作り直す）"),

        ParamSpec(group=GROUP_LINE, key="line_margin", label="壁からの余裕", min=0.0, max=0.4,
                  default=0.08, step=0.01, unit="m",
                  note="車体半幅に足す安全代。地図の誤差・局在化の誤差・追従の誤差をここで飲む。"
                       "車体外形と地図の壁の距離もこれ以上に保つ。★3m/sでは追従が±10cm振れ、"
                       "0.05では広いオーバル（normal）で衝突した。0.08で0回"),

        ParamSpec(group=GROUP_LINE, key="kappa_use", hidden=True, label="最小回転半径の余裕", min=0.6, max=1.0,
                  default=0.9, step=0.01, unit="",
                  note="★経路に許す曲率 = 車の曲がれる限界（同定した舵の効きから。現在 半径 約48cm）× この値。"
                       "0.9 なら半径 53cm より小さく曲がる経路を引かない。1.0 だと舵が上限に張り付き、"
                       "経路からのずれを直す舵が残らない。道幅が足りず開けない所は、いちばん緩い線になり、"
                       "走行中の表示に『曲がり切れない区間』と出る"),
        ParamSpec(group=GROUP_SPEED, key="v_max", label="最高速度", min=0.2, max=4.0, default=2.0,
                  step=0.05, unit="m/s",
                  note="★io_node の --max-speed を超えても Pi 側で切り捨てられるだけ。"
                       "★3m/s より上は実機で未検証（駆動の同定は約 2m/s まで・実走の記録は 3.2m/s まで。"
                       "STM32 の上限は 5m/s）。長い直線の無いコースでは上げても縮まない"),
        ParamSpec(group=GROUP_SPEED, key="v_min", label="最低速度", min=0.1, max=1.0, default=0.35,
                  step=0.05, unit="m/s", note="いちばんきついコーナーでもこれ以下にしない"),
        ParamSpec(group=GROUP_SPEED, key="speed_ff", hidden=True, label="速度指令の前倒し", min=0, max=1, step=1,
                  default=1, unit="",
                  note="★1で、STM32 の速度PIの遅れ（同定値から約0.16s）を見込んで目標速度を前倒しする"
                       "（`nav/speedctl.py`）。0 だと経路の速度をそのまま送り、コーナーへ 0.3〜0.5m/s"
                       "速く入る（『速度の先読み』だけが効く）"),
        ParamSpec(group=GROUP_SPEED, key="speed_fb", hidden=True, label="速度のずれの戻し", min=0.0, max=2.0,
                  default=1.0, step=0.1, unit="",
                  note="車速が経路の速度から外れたぶんを、この倍率で目標へ足し戻す（前倒しが 1 のとき）"),

        ParamSpec(group=GROUP_SPEED, key="launch", label="ローンチコントロール", min=0, max=1, step=1,
                  default=0, unit="",
                  note="★1で、発進（止まってから経路の速度に届くまで）を STM32 のローンチコントロールで行う"
                       "（目標速度のランプと速度PIを迂回し、全開＋TCで経路の速度まで加速。PIの立ち上がり"
                       "ぶん約0.2秒早い）。経路に乗っていて、前が空いていて、舵がほぼ真っすぐのときだけ"
                       "要求する。既定は 0（速度PIで発進）——今は手動操作でだけ使う方針（2026-10-10）"),

        ParamSpec(group=GROUP_STEER, key="ctrl_mode", label="追従方式(0=Pure Pursuit,1=MPC)", min=0, max=1,
                  step=1, default=0, unit="",
                  note="★1で舵をMPC（経路座標の線形化自転車モデル＋舵レート制約、`nav/mpc.py`）で決める。"
                       "速度は経路のプロファイルのまま。既定は0（Pure Pursuit）。MPCは実機未検証"),
        ParamSpec(group=GROUP_STEER, key="mpc_delay_s", hidden=True, label="MPC 遅延補償", min=0.0, max=0.3, step=0.01,
                  default=0.06, unit="s",
                  note="★MPC専用（Pure Pursuitの遅延補償とは別）。指令が効くまでの時間。大きすぎると逆に"
                       "振動する（3m/sのtoyota2で0.15は毎周期±全舵の振動で衝突、0.05〜0.07で衝突0。★シムの実効遅れはLiDAR周期＋姿勢の古さで決まる）"),
        ParamSpec(group=GROUP_STEER, key="mpc_horizon", hidden=True, label="MPC 予測ステップ数", min=5, max=40, step=1,
                  default=20, unit="",
                  note="予測区間 = ステップ数×刻み。長いほど先のコーナーに備えるが重い"),
        ParamSpec(group=GROUP_STEER, key="mpc_dt", hidden=True, label="MPC 刻み", min=0.03, max=0.2, step=0.01,
                  default=0.1, unit="s", note="1ステップの時間"),
        ParamSpec(group=GROUP_STEER, key="mpc_q_y", hidden=True, label="MPC 横偏差の重み", min=1, max=1000, step=10,
                  default=100, unit="", note="上げると経路に寄るが舵が忙しくなる"),
        ParamSpec(group=GROUP_STEER, key="mpc_q_psi", hidden=True, label="MPC 向き偏差の重み", min=0, max=200, step=1,
                  default=30, unit="",
                  note="経路の向きからのずれの重み。上げると進入で向きを早く揃える"),
        ParamSpec(group=GROUP_STEER, key="mpc_r_d", hidden=True, label="MPC 舵の変化の重み", min=0.01, max=20, step=0.1,
                  default=5.0, unit="", note="上げると舵が滑らかで遅れる"),
        ParamSpec(group=GROUP_STEER, key="look_k", hidden=True, label="前方注視の速度係数", min=0.0, max=2.0,
                  default=0.45, step=0.05, unit="s",
                  note="Ld = 係数×速度 + 最小値。上げると滑らかだがコーナーで内を切る。"
                       "★既定0.7は狭いコースで内を切りすぎた（2m/sのtoyotaで衝突10回・"
                       "横偏差16cm、0.45にすると衝突0・6.2cmで舵も滑らかになった）"),
        ParamSpec(group=GROUP_STEER, key="look_min", hidden=True, label="前方注視の最小値", min=0.15, max=1.5,
                  default=0.30, step=0.05, unit="m",
                  note="低速時の注視距離。小さすぎると舵が振動する"),
        ParamSpec(group=GROUP_STEER, key="delay_trim", hidden=True, label="遅延補償の補正", min=-0.05, max=0.3,
                  default=0.0, step=0.01, unit="s",
                  note="★舵の遅延補償は同定値（舵のむだ時間＋ヨーの遅れ＋通信、約0.06s）から決まる。"
                       "ここはそれに足す補正。追従に使う姿勢は車速とジャイロで『今』へ進めてあるので、"
                       "以前の『遅延補償』（`delay_s`、0.15 前後。姿勢の古さの手当て込み）とは意味が違うので、"
                       "キーを変えてある（保存済みの `delay_s` は読まない）"),
        ParamSpec(group=GROUP_STEER, key="tick", hidden=True, label="点群の合間も追従を更新", min=0, max=1, step=1,
                  default=1, unit="",
                  note="★1で、点群（10Hz）の合間も 50Hz で舵と速度を計算し直す（姿勢は車速とジャイロで"
                       "進める）。0 だと 10Hz——3m/s では 1 周期に 30cm 進む間、同じ舵のまま"),
        ParamSpec(group=GROUP_STEER, key="steer_ff", hidden=True, label="曲率フィードフォワード", min=0.0, max=1.0, default=0.8,
                  step=0.1, unit="",
                  note="★Pure Pursuit は注視区間で平均した曲率で曲がるので、S字で内側を切り続ける"
                       "（3m/sで±15cm）。足元の経路の曲率に差し替える重み。0で従来どおり。"
                       "0.5ではチケイン（circuit_chicane_a）で内を切って衝突、0.8で0回"),
        ParamSpec(group=GROUP_SPEED, key="speed_preview", hidden=True, label="速度の先読み（前倒し 0 のとき）", min=0.0,
                  max=1.0, default=0.1, step=0.05, unit="s",
                  note="『速度指令の前倒し』が 0 のときだけ効く。遅延補償の位置からこの時間ぶん先の"
                       "速度をそのまま指令する（旧方式）"),
        ParamSpec(group=GROUP_SAFETY, key="join_speed", label="経路に乗るまでの速度", min=0.2, max=1.5,
                  default=0.5, step=0.05, unit="m/s",
                  note="★走り出し（自己位置の復元直後）は経路から横・向きがずれている。"
                       "乗る（横8cm・向き15°以内）まではこの速度に抑える。抑えないと最高速のまま"
                       "大回りして壁に当たった（toyota/toyota2 の sim.bench で実測、2026-09-29）"),
        ParamSpec(group=GROUP_SAFETY, key="min_score", hidden=True, label="自己位置の信頼下限", min=0.1, max=0.8,
                  default=0.35, step=0.01, unit="",
                  note="★スキャンマッチの一致度がこれを切ったら（見失いが推測航法での続行の"
                       "時間を超えたら）止まる"),
        ParamSpec(group=GROUP_SAFETY, key="coast_s", hidden=True, label="推測航法で続行する時間", min=0.0, max=3.0,
                  default=1.5, step=0.1, unit="s",
                  note="★自己位置を見失っても（壁越しに別の部屋が見えた等）、これ以内は推測航法"
                       "（車速+ジャイロ）の姿勢で経路を追い続ける。0で従来どおり即停止"),
        ParamSpec(group=GROUP_SAFETY, key="coast_speed", hidden=True, label="推測航法中の速度上限", min=0.2, max=2.0,
                  default=0.8, step=0.05, unit="m/s",
                  note="推測航法で続行している間はこの速度に抑える"),
        ParamSpec(group=GROUP_SAFETY, key="max_cross", label="横偏差の上限", min=0.1, max=1.5, default=0.5,
                  step=0.05, unit="m", note="経路からこれだけ離れたら止まる"),
        ParamSpec(group=GROUP_SAFETY, key="obstacle_stop", label="障害物で止まる距離", min=0.0, max=4.0,
                  default=0.0, step=0.1, unit="m",
                  note="★これから通る帯に動く物が居たら制動する。0で無効"),
        ParamSpec(group=GROUP_SAFETY, key="obstacle_pad", hidden=True, label="壁の近くを無視する幅", min=0.05, max=0.6,
                  default=0.25, step=0.01, unit="m",
                  note="★壁からこの範囲に落ちた点は動的障害物と見なさない。**経路が壁にいちばん寄る所の"
                       "壁までの距離より大きければ、そこまで自動で詰める**（下限 0.15。経路の上の物が"
                       "見えなくなるため）。この外に1点でも落ちた塊は、10cm 壁寄りの点まで数える"),
        ParamSpec(group=GROUP_AVOID, key="obstacle_avoid", label="障害物を避ける", min=0, max=1, step=1,
                  default=0, unit="",
                  note="★1で、経路を塞ぐ障害物を横へ避ける（止まらずに済むなら止まらない）。"
                       "避けられなければ止まり、Hybrid A* で後退・切り返しも使って抜ける。"
                       "0 なら従来どおり（『障害物で止まる距離』だけ）"),
        ParamSpec(group=GROUP_AVOID, key="sight_use", label="見通しの中で止まれる速度に抑える", min=0.0,
                  max=1.0, default=0.0, step=0.05, unit="",
                  note="★既定は 0（抑えない）。『障害物を避ける』か『障害物で止まる距離』が有効なときだけ効く。1.0 で、経路の先が"
                       "見えている距離の中で止まれる速度に抑える（壁の先端を回るヘアピンは 0.6m 先までしか"
                       "見えない）。費用は 1 周 +10〜25%（シム）。0.5 なら『見えている距離の 2 倍で止まれる』"
                       "速度で +0〜8%——陰に障害物が居れば、減速はするが当たる（シムで 0.6m/s で接触）。"
                       "0 で抑えない。★見ているのは経路の線上だけ。内側の壁に寄った障害物は 1.0 でも"
                       "見えるのが遅れて当たりうる"),
        ParamSpec(group=GROUP_AVOID, key="avoid_margin", hidden=True, label="障害物との余裕", min=0.0, max=0.4,
                  default=0.10, step=0.01, unit="m",
                  note="障害物の縁から車体側面までの余裕。自己位置と障害物の位置の誤差をここで飲む"),
        ParamSpec(group=GROUP_AVOID, key="avoid_clear", hidden=True, label="車体外形の合格線", min=0.0, max=0.2,
                  default=0.05, step=0.01, unit="m",
                  note="避ける経路で車体外形と壁・障害物の距離がこれを切ったら、その避け方は使わない"),
        ParamSpec(group=GROUP_AVOID, key="avoid_speed", hidden=True, label="余裕ぎりぎりで避ける速度", min=0.2, max=2.5,
                  default=1.0, step=0.05, unit="m/s",
                  note="★障害物・壁との余裕が『車体外形の合格線』ちょうどのときの速度。余裕がそれより"
                       "15cm 以上広ければ抑えず（曲率で決まる速度のまま）、その間は余裕に比例して上げる"),
        ParamSpec(group=GROUP_AVOID, key="avoid_len_min", hidden=True, label="避け始める長さの下限", min=0.3, max=3.0,
                  default=0.8, step=0.1, unit="m",
                  note="横へずれ始めてからずれ切るまでの距離の下限。速いほど・ずれが大きいほど長くなる"),
        ParamSpec(group=GROUP_AVOID, key="stuck_s", hidden=True, label="スタックとみなす時間", min=0.5, max=5.0,
                  default=1.5, step=0.1, unit="s",
                  note="進めと言っているのに止まっている時間がこれを超えたら、Hybrid A* で抜ける"),
        ParamSpec(group=GROUP_AVOID, key="detour_ahead", hidden=True, label="抜けた先の目標の距離", min=0.3, max=2.0,
                  default=1.2, step=0.1, unit="m",
                  note="Hybrid A* の目標を、障害物の後ろ（車体後端が抜ける位置）からこれだけ先の経路上に置く"),
        ParamSpec(group=GROUP_AVOID, key="detour_budget_ms", hidden=True, label="Hybrid A* の時間予算", min=20.0,
                  max=150.0, default=60.0, step=10.0, unit="ms",
                  note="★1回の経路探索の上限。plan() 全体が auto_cmd_stale_ms（200ms）を超えると中継側が最大制動に読み替える"),
        ParamSpec(group=GROUP_AVOID, key="detour_speed", hidden=True, label="Hybrid A* で抜ける速度", min=0.15,
                  max=0.8, default=0.5, step=0.05, unit="m/s",
                  note="止まってから Hybrid A* で障害物の先へ抜ける間の巡航速度。上げるほど早く抜けるが、"
                       "壁・障害物すれすれを通るので追従の誤差も増える"),
        ParamSpec(group=GROUP_AVOID, key="avoid_wait", hidden=True, label="止まってから抜け始めるまで", min=0.2,
                  max=3.0, default=0.3, step=0.1, unit="s",
                  note="塞がれて止まってから Hybrid A* に入るまでの待ち。その半分の時点で、横へ避ける"
                       "経路をもう一度引いてみる。短いほど早く動き出すが、どく物（人・他車）も待たずに迂回する"),
        ParamSpec(group=GROUP_SAFETY, key="md_fault_speed", hidden=True, label="駆動MD無応答中の速度上限", min=0.2,
                  max=3.0, default=1.0, step=0.05, unit="m/s",
                  note="★後輪のモータドライバが片方だけ無応答（低電圧など）の間はこの速度に抑えて走り続ける"
                       "（制動が約半分になる。実測 −2.5→−1.4 m/s²）。両方とも無応答なら 0 を指令して戻るのを待つ"),
        ParamSpec(group=GROUP_SAFETY, key="md_fault_s", hidden=True, label="片輪無応答で止まるまで", min=1.0,
                  max=60.0, default=10.0, step=1.0, unit="s",
                  note="片輪の無応答がこれだけ続いたら、止まって戻るのを待つ（戻れば走り出す）"),
        # ── 横グリップの学習（`nav/grip.py`） ──
        ParamSpec(group=GROUP_GRIP, key="grip_learn", label="路面の限界を走りながら決める", min=0, max=1,
                  step=1, default=0, unit="",
                  note="★0 = 固定（同定値に決まった余裕を掛けて使う。今までと同じ）。1 = 走行中の車の反応から"
                       "横・加速・減速の上限を決める。横はコースで 1 つ: 上限を 1 段（8%）上げて 1 周走り、"
                       "計画で縮むはずの時間の 6 割以上ラップが縮んだら採用、縮まなければ戻して終える"
                       "（線から外れる・リアが滑るといった兆候が出た段はすぐやめる）。"
                       "減速もコース全体で 1 つ: 1 周に 1 段上げ、コーナーへ計画より速く入るように"
                       "なったら 1 段戻して止める。加速は上限（目標車速の傾きの上限）で固定（学習しない）。"
                       "値は地図の横（<地図名>.grip.json）に保存する"),
        ParamSpec(group=GROUP_GRIP, key="grip_init", hidden=True, label="初期値（同定値に対する割合）", min=0.5, max=1.2,
                  step=0.05, default=0.85, unit="×",
                  note="学習の出発点 = 同定値 `mu`×g × これ。発進の加速度から初期値が出たとき"
                       "（TC が介入した発進）と、保存した値があるときは、そちらを使う"),
        ParamSpec(group=GROUP_GRIP, key="grip_max", label="上げてよい上限", min=2.0, max=8.0, step=0.1,
                  default=6.0, unit="m/s²",
                  note="学習で横の限界をここより上げない"),
        ParamSpec(group=GROUP_GRIP, key="grip_step", hidden=True, label="1回の上げ幅", min=1.0, max=10.0, step=0.5,
                  default=6.0, unit="%",
                  note="前兆なしでコーナーを1つ抜けるたびに、横の限界をこの割合だけ上げる。"
                       "★既定の 6% はシムで決めた値。実機で初めて試すときは 3% から"),
        ParamSpec(group=GROUP_GRIP, key="grip_by_time", hidden=True, label="横の上限を周回のタイムで決める",
                  min=0, max=1, step=1, default=1, unit="",
                  note="1 = 上限を 1 段上げて 1 周走り、計画で縮むはずの時間の 6 割以上縮んだら採用する"
                       "（`nav/grip.TimeLearner`。前兆は行き過ぎた段をやめる安全装置としてだけ使う）。"
                       "0 = 前兆が出るまで上げる元の方式（`GripLearner`）"),
        ParamSpec(group=GROUP_GRIP, key="grip_reline", hidden=True, label="学習値でラインを引き直す",
                  min=0, max=1, step=1, default=0, unit="",
                  note="★1 = 学習が落ち着いたら、学習した限界でラインを引き直して乗り移る。既定は 0: "
                       "シムで衝突は出なくなった（15 本）が、路面によって遅くなる（μ0.70 で −1〜2%、"
                       "μ0.30 で +2.5%）。`PROGRESS.md` 2026-10-12 夜の節"),
        ParamSpec(group=GROUP_GRIP, key="grip_down_only", label="下げる方向だけ許す", min=0, max=1, step=1,
                  default=0, unit="",
                  note="★1 = 上げない（本番の計測走行向け）。前兆が出たとき・加速や減速が出なかった"
                       "ときに下げる層だけ働く。減速を強く求めて測ることもしない"),
    )

    #: 保存済み地図を読んだとき、中心線からラインを引き直すか（`slam2d_route` は自前で作る）
    _reline_on_load = True

    def __init__(self) -> None:
        self._last_p: dict[str, float] | None = None
        #: 自動運転に入っているか（`planning_node` が毎周期 `set_engaged` で渡す）。
        #: **入っていない EXPLORE は人のラジコンでの地図作成**として扱う。
        #: `set_engaged` を呼ばない呼び出し元（ベンチ・テスト）では従来どおり FTG で走る
        self._engaged = True
        self._mpc = MPCFollower()
        self._sff = speedctl.SpeedFF()
        self._sff_catch = speedctl.SpeedFF()        # 追いつく間の速度指令（経路を毎回作るので別に持つ）
        #: `speedctl.ref_command` に渡すプロファイルの加速度と、それを作った `path.v`
        self._ref_acc: np.ndarray | None = None
        self._ref_acc_src: np.ndarray | None = None
        self.vehicle = Vehicle.load()
        #: 横グリップの学習（`nav/grip.py`、`grip_learn` が 1 の間だけ働く）。舵角から出る曲率に
        #: 掛ける遅れはヨーの応答ぶん（舵そのものの遅れは実舵角に入っている）
        self._grip = self._grip_make(True)
        #: 計画のラップ [s] と、それを出した `path.v`（`TimeLearner` へ渡す）
        self._grip_plan: tuple = (None, 0.0, 0.0, 0.0)
        self._grip_on = False
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
        self._mpc.reset()
        self._sff.reset()
        #: 速度指令を前回計算した時刻 [ns]（`VehicleState.t_capture`）
        self._sff_t = 0
        #: 点群の合間の追従（`tick`）に使う、直近の `plan()` の結果:
        #: (追っている経路, 設定, 速度の上限, 追従の起点の添字)。`None` = 合間は更新しない
        self._tick_ctx: tuple | None = None
        self._tick_raw = math.inf
        self._tick_path: rl_mod.RaceLine | None = None
        #: 周回の数え上げ（`_count_lap`）。前の周期の最寄り点の添字と、前回数えてから経路に
        #: 沿って進んだ距離 [m]
        self._prev_idx = -1
        self._lap_s = 0.0
        #: 周回を数えるゲート（地図座標の点。`None` = 次に数えた経路の先頭の点にする）と、
        #: それを経路の添字に直した値・直した経路（`_count_lap`）
        self._lap_gate: tuple[float, float] | None = None
        self._lap_gate_ref: np.ndarray | None = None
        self._lap_gate_idx = 0
        #: 前の周期にゲートから進んでいた割合 [0, 1)（-1 = まだ無い）
        self._lap_frac = -1.0
        #: 障害物の候補にしてよいセル（`obs_mod.free_mask`）と、それを作った (地図の版, 幅)
        self._obs_free: np.ndarray | None = None
        self._obs_near: np.ndarray | None = None
        self._obs_free_key: tuple | None = None
        #: 実際に使っている無視幅（`_obstacle_pad`）と、それを決めた経路
        self._obs_pad = 0.0
        self._obs_pad_ref: np.ndarray | None = None
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
        #: 灯火（パッシング→ウィンカー）。`_signal_step` が毎周期 `st` へ書く
        self._signals = SignalLights()
        self._hit_now = False             #: この周期、前方を塞ぐ障害物を見ているか（`_track`）
        self._detour_side = 0             #: Hybrid A* の迂回先の向き（+1=左 -1=右 0=不明）
        #: 障害物を避けている一時経路（`avoid.plan_offset`）と、その元にした経路
        self._avoid: av_mod.AvoidPlan | None = None
        self._avoid_path: rl_mod.RaceLine | None = None
        self._avoid_base: rl_mod.RaceLine | None = None
        self._avoid_ms = 0.0
        #: 横へ避ける経路を引けなかったときの (障害物と条件のキー, 車の添字, 経路)。`_plan_avoid` 参照
        self._avoid_fail: tuple | None = None
        #: 直近に横へ避ける経路を引けなかった理由（表示用）
        self._avoid_why = ""
        #: 今の避ける経路の作り方（"横ずらし" = 決まった形、"引き直し" = `avoid.plan_elastic`）
        self._avoid_how = ""
        #: 見通しの距離（`_sight_cap`）と、それを掛けた経路（`_with_sight`）のキャッシュ
        self._sight: np.ndarray | None = None
        self._sight_ref: np.ndarray | None = None
        self._sight_seq = -1
        self._sighted: rl_mod.RaceLine | None = None
        self._sighted_src: rl_mod.RaceLine | None = None
        self._sighted_key: tuple | None = None
        self._sight_msg = ""
        #: 今の避ける経路の元にした経路（見通しの上限をそこから取る）
        self._avoid_room_path: rl_mod.RaceLine | None = None
        #: 別プロセスで引き直している避ける経路（`avoid.plan_elastic`）と、その元の経路
        self._drop_job("_elastic_job")
        self._elastic_base: np.ndarray | None = None
        self._v_now = 0.0                  #: 直近の車速（障害物の「近い」の範囲に使う）
        #: 「曲がり切れない区間」の注記と、それを測った (経路, `kappa_use`)（`_tight_note`）
        self._tight_key: tuple | None = None
        self._tight_ref: np.ndarray | None = None
        self._tight_msg = ""
        #: 経路の各点から左右へずらせる量（`_avoid_room`）と、それを作った (経路, 地図の版, 合格線)
        self._room: tuple | None = None
        self._room_key: tuple | None = None
        self._room_ref: np.ndarray | None = None
        #: 壁と車体外形の余裕を測る距離場（地図の版ごとに作る、`_body_check`）
        self._body: rl_mod._BodyCheck | None = None
        self._body_seq = -1
        #: 塞がれて止まっている時間・進めずに止まっている時間 [s]
        self._blocked_t = 0.0
        self._stuck_t = 0.0
        #: 障害物で制動してから検出が途切れている時間 [s]（`None` = 制動していない）
        self._brake_hold: float | None = None
        #: 駆動 MD が片輪だけ無応答の状態が続いている時間 [s]（`_md_fault`）
        self._md_t = 0.0
        #: 止まってから経路の速度に届くまで（＝発進の間）True（`_launch_ok`）
        self._launch_armed = False
        #: Hybrid A* で抜けている（`DETOUR`）。同じ障害物で試した回数（失敗が続いたら止まって待つ）
        self._detour: ParkToPoint | None = None
        self._detour_error = ""
        self._detour_tries = 0
        self._dt = 0.0
        #: 今の経路の速度プロファイルを作った設定（`_retime`）
        self._speed_key: tuple | None = None
        #: 地図の読み込み・ラインの形を決める設定（`LINE_KEYS`）の変更で引き直しているライン
        #: （`_submit_line`）と、今のライン（引き直し中ならその計算）の設定。`None` = 引き直さない
        self._drop_job("_line_job")
        self._line_built: tuple | None = None
        #: 引き直しが済んで、乗り移れる場所を待っているライン（`_try_adopt_line`）
        self._line_next: rl_mod.RaceLine | None = None
        self._line_next_key: tuple | None = None
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
        self._loc_hint_t = 0.0                   #: `_loc_hint` を受けた時刻（`time.monotonic()`）
        #: 地図作成直後に走り出すときの「直前の自己位置」（`request_load`参照）
        self._locate_seed: tuple[float, float] | None = None
        self._load_error = ""
        #: 地図の読み込みに失敗した（`request_load`）。解くまで指令を出さない
        self._load_failed = False
        #: BUILD完了時に自動保存した地図の名前（`DONE`段の表示用）
        self._saved_map_name = ""
        self._save_error = ""
        self._grip_save()
        self._grip.reset()
        self._grip_on = False
        #: 読み込んだ地図の名前（学習した横グリップの保存先）
        self._grip_name = ""
        #: 今のラインを引いたときの学習値 `(横, 加速, 減速)`（`None` = 同定値で引いた、`_grip_line_stale`）
        self._grip_line: tuple | None = None
        #: 学習を止めている理由（空 = 学習してよい）と、それを決めた時刻 [ns]
        self._grip_gate = "走行していない"
        self._grip_gate_t = 0
        #: 今の場所で計画している前後の加速度 [m/s²]（`_speed_cmd`）
        self._grip_ax = 0.0
        self._grip_catch = False            #: 追いつく途中か（`_speed_cmd`）
        self._grip_vp: float | None = None  #: 同じく、プロファイルの速度 [m/s]
        #: この時刻 [ns] までは学習しない（別の線へ乗り換えた直後。横へ数 cm 跳ぶぶんが前兆に見える）
        self._grip_quiet_ns = 0
        self._grip_quiet_why = ""                #: 上の理由（reason の「停止: …」に出す）
        #: 経路の各点の `(求める横加速度, 限界)`（`_grip_levels`）: [(経路の点, 鍵, 配列の組)]（直近の3本）
        self._grip_cache: list[tuple] = []
        #: 経路の各点での車体と壁の余裕（`_grip_wall`）と、それを測った (経路の点, 地図の版)
        self._grip_wall_arr: np.ndarray | None = None
        self._grip_wall_ref: np.ndarray | None = None
        self._grip_wall_key: tuple | None = None
        #: 曲がり切れない区間の印（`_grip_tight`）
        self._grip_tight_arr: np.ndarray | None = None
        self._grip_tight_ref: np.ndarray | None = None
        self._grip_tight_key: tuple | None = None
        #: 上げた速度を車の先にだけ反映するための上限（`_grip_hold`）:
        #: (経路の点, 点ごとの上限, 窓の始まりの添字, 窓の点の数)
        self._grip_hold_cap: tuple | None = None
        self._grip_plain: tuple | None = None
        #: 速度へ反映済みの学習の版 `(ver, ver_down)` と、速度の作り直しの鍵に入れる通し番号
        self._grip_ver = (-1, -1)
        self._grip_tok = getattr(self, "_grip_tok", 0) + 1
        #: 前兆で落とした速度をこの周期の指令に掛けたか
        self._grip_capped = False
        self._grip_saved: tuple = (0, 0.0)
        self._grip_save_t = 0.0
        self._grip_laps = 0

    def on_vehicle_state(self, vs: VehicleState) -> None:
        """`planning_node`が`vehicle_state`を受けるたびに呼ぶ（100Hz）。

        点ごとの脱スキュー（`slam2d/core/deskew.py`）に使う twist の履歴を
        SLAM へ流し込む。`plan()`は10Hzでしか呼ばれないので、ここで
        受けておかないと1周100msのあいだの回頭の変化が追えない。
        横グリップの学習（`nav/grip.py`）の前兆もここで見る（100Hz）。
        """
        self.slam.on_vehicle_state(vs)
        if self._grip_on and self.phase == RACE:
            self._grip_observe(vs)

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
        self._line_built = None
        self._line_next = None
        self._submit_line(self._last_p or {s.key: s.default for s in self.params})
        self._speed_key = None            # 速度は今の設定で作り直す（`_retime`）
        self._grip_load(name)
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
        kw = self._grip_line_kw()
        self._grip_line = self._grip_learned() if kw is not None else None
        self._line_job = submit_job(
            _optimize_line, g.trinary().copy(), g.resolution, tuple(g.origin),
            np.asarray(cl).copy(), dict(p), self.vehicle, True, kw)
        self._line_built = line_key(p)

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
            self._line_next = job.result()
        except Exception as e:                      # noqa: BLE001  今のラインで走り続ける
            self._load_error = f"ラインを引き直せなかった（今のラインで走る）: {e}"
            return
        if self._grip_line is not None and self._grip_worse(self.path, self._line_next):
            self._line_next = None                  # 学習値で引いた線が曲がり切れない。今の線のまま
            return
        self._load_error = ""
        if self.phase != RACE or self.path is None:
            self._adopt_line()                      # まだ走っていない。その場で差し替える

    def _adopt_line(self) -> None:
        new, self._line_next = self._line_next, None
        if new is None:
            return
        self.path = new
        self._hint = -1
        self._prev_idx = -1
        self._speed_key = None
        self._map_dirty = True
        self._grip_quiet(self._grip_gate_t, "線を乗り換えた直後")
        self._grip.new_line()

    def _try_adopt_line(self, vs: VehicleState, p: dict[str, float]) -> None:
        """裏で引き直したラインへ、**乗り移れる場所に来たら**差し替える（RACE 中）。

        ★ できた瞬間には差し替えない。新旧の線は数十cm 離れている所があり、3m/s で走って
        いる途中に足元の経路が横へ跳ぶ（sim.bench で横偏差の最大 26〜33cm が出ていたのはこれ）。
        新しい線が今の姿勢のすぐそばを同じ向きで通り、そこでの速度が今の車速から無理なく
        つながる所（新旧の線は1周に何度か交わる）まで待つ。止まっていればすぐ差し替える。
        """
        new = self._line_next
        if new is None or self._avoid is not None:
            return
        if abs(vs.speed) > 0.1:
            if new.v is None or self._line_next_key != self._skey(p):
                new = self._line_next = rl_mod.retime(new, **self._spd(p, new))
                self._line_next_key = self._skey(p)
            x, y, yaw = self.slam.pose_now(vs)
            j = nearest_index(new, x, y)
            if (math.hypot(new.xy[j, 0] - x, new.xy[j, 1] - y) > _ADOPT_DIST
                    or _heading_err(new, j, yaw) > _ADOPT_YAW
                    or vs.speed > float(new.v[j]) + 0.3):
                return
            self._adopt_line()
            self._hint = j
            self._retime(p)
            return
        self._adopt_line()

    def request_locate_hint(self, x: float, y: float) -> None:
        self._loc_hint = (x, y)
        self._loc_hint_t = time.monotonic()
        self._localizer = None     # 探索中なら絞り込んでやり直す
        self._loc_cands = None

    # ── 本体 ──

    def plan(self, scan: Scan, vs: VehicleState | None,
             p: dict[str, float], dt: float) -> AutoState:
        st = AutoState(mode=self.id, planner=self.name, phase=self.phase)
        self._dt = dt
        self._last_p = p
        self._tick_ctx = self._tick_path = None
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
        if self.phase == EXPLORE:
            # 確定した後は替えない（ラインと地図が食い違う）
            self.slam.set_wall_weight(p.get("wall_weight", WALL_HIT_WEIGHT))
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
        self._hit_now = False
        st = self._race(st, scan, vs, p, u.lost)
        self._signal_step(st)
        # 速度を追従の値より抑えた（障害物・推測航法など）・制動するなら、目標車速のランプは
        # 指定しない（＝STM32 の上限で動かす。プロファイルの緩い傾きのままだと減速が遅れる）
        if st.brake or not st.ready or st.target_speed < self._tick_raw - 1e-6:
            # 横グリップの前兆で落としている間だけは、落とす速さを指定する（`_grip_cap`）
            g = self._grip_cap() if self._grip_capped and st.ready and not st.brake else None
            st.accel_limit = g[1] if g is not None else 0.0
        self._arm_tick(st, p)
        return st

    # ── 点群の合間の追従 ──

    def _arm_tick(self, st: AutoState, p: dict[str, float]) -> None:
        """この周期の判断が「経路を追っているだけ」なら、点群の合間も追従を更新できるようにする。

        制動・停止・Hybrid A*・経路に乗る前の判断は 10Hz のまま（`_tick_ctx` が `None`）。
        速度を抑えた理由（推測航法・MD の異常・障害物の手前）は、抑えた値を上限として持ち越す。
        """
        path = self._tick_path
        if (path is None or p["tick"] <= 0 or not st.ready or st.brake or st.launch
                or self.phase != RACE):
            return
        # 横グリップの前兆で落とした値は持ち越さない（`tick` が 100Hz の最新の値を直に読む）
        cap = (st.target_speed if st.target_speed < self._tick_raw - 1e-6 and not self._grip_capped
               else math.inf)
        self._tick_ctx = (path, p, cap, self._hint)

    def tick(self, vs: VehicleState | None) -> tuple[float, float | None, float] | None:
        """点群の合間（`planning_node` の 50Hz の定期送信）に舵と速度を計算し直す。

        姿勢は直近の SLAM の結果を車速とジャイロで今へ進めたもの。**追従だけ**をやり直す
        （障害物・自己位置の判断は `plan()` のまま）。戻り値は `(舵角, 目標速度, 目標車速のランプ)`、
        更新しないなら `None`。目標速度が `None` なら速度は `plan()` の値のまま。ランプ 0 は指定なし。
        """
        ctx = self._tick_ctx
        if ctx is None or vs is None:
            return None
        path, p, cap, hint = ctx
        pose = self.slam.pose_now(vs)
        pp = self._follow(path, pose, vs, p, pursuit_config(p, self.vehicle), hint)
        if not path.closed or p["speed_ff"] <= 0:
            return pp.steer, None, 0.0             # 停止点へ向かう経路の速度は `plan()` が決める
        speed, ramp = self._speed_cmd(path, pp.index, pose, vs, p)
        if speed > cap:
            speed, ramp = cap, 0.0                 # 抑えた値へは STM32 の上限で向かう
        g = self._grip_cap()
        if g is not None and g[0] < speed:
            return pp.steer, g[0], g[1]            # 横グリップの前兆（`nav/grip.py`）
        return pp.steer, speed, ramp

    def _speed_cmd(self, path: rl_mod.RaceLine, index: int, pose, vs: VehicleState,
                   p: dict[str, float]) -> tuple[float, float]:
        """`(目標速度 [m/s], 目標車速のランプ [m/s²])`（`nav/speedctl.py`）。ランプ 0 は指定なし。

        STM32 の加速度フィードフォワードが有効（`Vehicle.speed_ff_firmware`、★v0.21）なら、
        プロファイルの速度とその傾きをそのまま渡す（`speedctl.ref_command`）。無効なら、速度PIの
        遅れを Pi 側で見込んで前倒しした目標速度（`speedctl.SpeedFF`、ランプは指定しない）。
        """
        j = nearest_index(path, pose[0], pose[1], index, window=8)
        pos = speedctl.arc_index(path, j, pose[0], pose[1])
        t = int(vs.t_capture)
        dt = (t - self._sff_t) / 1e9 if self._sff_t and t else 0.0
        self._sff_t = t
        cfg = speedctl.config_from_vehicle(self.vehicle, k_fb=p["speed_fb"])
        # 今ここで前へ使ってよい加速度（摩擦円の残り）
        n = len(path)
        kap = float(np.abs(path.kappa[j])) if len(path.kappa) == n else 0.0
        lim = self._lim(p)
        # 横の限界（摩擦円の半径）。学習していればその場所の値、していなければ同定値
        grip = self._grip_levels(path)
        grip = grip[1] if grip is not None else None
        a_grip = float(grip[j]) if grip is not None else self.vehicle.a_lat_max
        if self._grip_on:
            if self._ref_acc_src is not path.v:
                self._ref_acc, self._ref_acc_src = speedctl.path_accel(path), path.v
            self._grip_ax = float(self._ref_acc[j])
            self._grip_vp = speedctl._lerp(path.v, pos, path.closed)
        u = max(vs.speed, 0.0) ** 2 * kap / max(a_grip, 1e-6)
        a_room = lim.a_accel * math.sqrt(max(0.0, 1.0 - u * u))
        self._grip_catch = vs.speed < float(path.v[j]) - _CATCH_UP_GAP
        if self._grip_catch:
            # 車速がプロファイルよりずっと低い（迂回の後・走り出し・減速の後）。今の車速から
            # 摩擦円の中で追いつく速度へ置き換えてから前倒しする（`speedctl.catch_up`）
            local = speedctl.catch_up(path, j, vs.speed, a_accel=lim.a_accel,
                                      a_grip=grip if grip is not None else self.vehicle.a_lat_max)
            if self.vehicle.speed_ff_firmware:
                return speedctl.ref_command(local, pos, vs.speed, lead_s=self._ref_lead())
            return self._sff_catch.command(local, pos, vs.speed, dt, cfg, a_room=a_room), 0.0
        if self.vehicle.speed_ff_firmware:
            if self._ref_acc_src is not path.v:
                self._ref_acc, self._ref_acc_src = speedctl.path_accel(path), path.v
            return speedctl.ref_command(path, pos, vs.speed, lead_s=self._ref_lead(),
                                        acc=self._ref_acc)
        return self._sff.command(path, pos, vs.speed, dt, cfg, a_room=a_room), 0.0

    def _ref_lead(self) -> float:
        """指令を決めてからフィードフォワードのトルクに効くまでの時間 [s]:
        次の指令までの半周期（50Hz）＋通信（同定の遅延試験）＋STM32 が目標車速の傾きに掛ける
        ローパスの時定数。どれも実測か既知の定数で、合わせ込む係数は無い。"""
        return 0.01 + cmd_latency(self.vehicle) + speedctl.FW_FF_TAU_S

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
                self._build_job = submit_job(
                    _optimize_line, g.trinary().copy(), g.resolution, tuple(g.origin),
                    self.centerline, dict(p), self.vehicle, False)   # 仕上げは走り出してから
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
        # ヒントは「クリック→読み込み」の意思なので読み込みでは捨てないが、古いものは使わない
        # （別の地図・別の出発点で、ずっと前のクリックに探索が絞られないように）
        if self._loc_hint is not None and time.monotonic() - self._loc_hint_t > _LOC_HINT_TTL_S:
            self._loc_hint = None
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
        self._lap_gate = None
        self._lap_frac = -1.0
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
        # ★ `line_margin` はラインの形を決める。走行中に変えたら引き直す（速度だけ作り直すと、
        #   障害物の帯と回避の余裕は新しい値をすぐ使うので表示・線・判定が食い違う）。
        #   できるまでは今のラインで走る
        self._grip_begin(vs, p)
        if (self._line_built is not None and self._line_job is None
                and (line_key(p) != self._line_built or self._grip_line_stale())):
            self._submit_line(p)
        self._try_adopt_line(vs, p)
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
        md_note = self._md_fault(st, vs, p)
        if md_note is None:
            return st

        if self._obstacle_response(st, hit, dist, vs, p, self.path, pp.index):
            return st

        st.launch = self._launch_ok(st, vs, hit, coasting, md_note, p)
        grip_note = self._grip_end(st, vs, p, self.path, pp, hit, coasting, md_note)
        st.reason = (f"{self.laps}周走行中・速度 {st.target_speed:.2f} m/s・"
                     f"横偏差 {pp.cross_track * 100:+.0f}cm"
                     + ("" if self._joined else "・経路に乗るまで減速")
                     + self._avoid_note() + self._coast_note(coasting, p) + md_note
                     + self._sight_note(self.path, p) + self._tight_note(self.path, p) + grip_note
                     + (f"・★{self._load_error}" if self._load_error else ""))
        return st

    def _tight_note(self, path: rl_mod.RaceLine, p: dict[str, float]) -> str:
        """経路に「車が曲がり切れない区間」が残っていれば、その注記（経路ごとに1回だけ測る）。

        道幅が足りず、半径を `kappa_use`×限界まで開けなかった所。そこは舵が上限に張り付き、
        出口で外へ膨らむ（実車で確認）。`line_margin` を詰めるか、コースを広げるしかない。
        """
        # ★ 経路は `id()` ではなく**物そのもの**で見分ける（`is`）。`id()` は、差し替えで古い配列が
        #   解放された直後に新しい配列へ使い回されることがあり、古い結果を出してしまう
        key = (p["kappa_use"],)
        if self._tight_ref is not path.xy or self._tight_key != key:
            self._tight_ref = path.xy
            lim = kappa_limit(p, self.vehicle)
            length, ratio = rl_mod.tight_sections(path.xy, path.closed, lim)
            self._tight_key = key
            self._tight_msg = (f"・★曲がり切れない区間 {length:.1f}m（曲率が上限の {ratio * 100:.0f}%、"
                               f"半径 {1.0 / max(ratio * lim, 1e-6) * 100:.0f}cm）" if length > 0.0 else "")
        return self._tight_msg

    def _launch_ok(self, st: AutoState, vs: VehicleState, hit, coasting: bool, md_note: str,
                   p: dict[str, float]) -> bool:
        """ローンチコントロール（`AutoState.launch`）を要求してよいか。

        要求している間、STM32 は `target_speed` が車速より 0.5m/s 以上速い加速を、目標速度のランプと
        速度PIを迂回して全開＋TC で行う（1回の要求で1回。始める・終える判定は STM32）。

        要求するのは**発進だけ**: 止まった（`_LAUNCH_ARM_SPEED` 未満）あと、経路の速度に届くまで。
        周回中のコーナーの立ち上がりでは要求しない。全開で加速してよいのは、`target_speed` が経路の
        速度そのもの（乗り直し・推測航法・障害物・MD の異常で抑えていない）で、舵がほぼ真っすぐの
        ときだけ——TC が保つスリップ率では横グリップが4割しか残らない。走り出しは経路から外れていて
        `join_speed` で乗りに行くので、実際に全開になるのは経路に乗った後（STM32 は車速を問わない）。
        途中で条件が外れれば要求を下ろし、STM32 はその場で速度PIへ戻る。
        """
        if abs(vs.speed) < _LAUNCH_ARM_SPEED:
            self._launch_armed = True
        if not (p["launch"] > 0 and self._joined and not coasting and hit is None
                and self._avoid is None and not md_note):
            return False
        if vs.speed >= st.target_speed - _LAUNCH_DONE_GAP:
            self._launch_armed = False
        return self._launch_armed and abs(st.target_steer) < _LAUNCH_STEER_MAX

    def _count_lap(self, path: rl_mod.RaceLine, index: int) -> None:
        """閉じた経路の最寄り点の添字から周回を数える（`slam2d_route` と共通）。

        数える線（ゲート）は**地図上の 1 点に固定**する: 最初に数えた経路の先頭の点（`_lap_gate`）。
        経路の継ぎ目（添字 0）は経路ごとに違う（経由点のグループは最初の経由点、自動経路は
        地図作成の出発点、引き直したラインは引き直した位置）ので、継ぎ目で数えると経路を
        切り替えるたびに数え過ぎ・数え漏れが出る。経路が替わったら、その経路でゲートに
        いちばん近い添字を数える位置にする。

        添字がゲートの手前→先へ回っても、半周も進んでいなければ数えない（走り出しの位置が
        ゲートの直前だと、動いた瞬間に「1周」と数えてしまう）。進んだ距離は
        **符号付きの巡回差**で積む——`(−1) % n` のように足すと、最寄り点が1つ戻った
        だけでほぼ1周ぶん進んだことになり、この歯止めが効かなくなる。
        """
        n = len(path)
        if self._lap_gate is None:
            self._lap_gate = (float(path.xy[0, 0]), float(path.xy[0, 1]))
        if self._lap_gate_ref is not path.xy:
            self._lap_gate_ref = path.xy
            self._lap_gate_idx = nearest_index(path, *self._lap_gate)
        # ゲートから進んだ割合 [0, 1)。経路を乗り換えても同じ地図の位置はほぼ同じ割合なので、
        # 前の周期の値を持ち越す（乗り換えた周期にゲートをまたいでも取りこぼさない）
        frac = ((index - self._lap_gate_idx) % n) / n
        prev = self._lap_frac
        if prev >= 0.0:
            self._lap_s += ((frac - prev + 0.5) % 1.0 - 0.5) * path.length
        if prev > 0.75 and frac < 0.25 and self._lap_s > 0.5 * path.length:
            self.laps += 1
            self._lap_s = 0.0
        self._lap_frac = frac
        self._prev_idx = index

    # ── 障害物: 検出・横へ避ける・止まる・Hybrid A* で抜ける（`slam2d_route` と共通） ──

    def _track(self, st: AutoState, scan: Scan, vs: VehicleState, p: dict[str, float],
               path: rl_mod.RaceLine, pose):
        """経路（障害物を避けている間は一時経路）を追い、前方を塞ぐ障害物を調べる。

        塞がれていて「避ける」が有効なら、その場で横へ避ける一時経路を引いて乗り換える
        （止まらずに済むなら止まらない）。戻り値は `(追従の結果, 塞ぐ障害物, そこまでの距離)`。
        """
        cfg = pursuit_config(p, self.vehicle)
        self._v_now = float(vs.speed)
        self._detect_obstacles(st, scan, p)
        # ★ 元の経路が替わったかは**形**（`xy`）で見る。速度だけ付け直した経路（学習値の版が進んだ・
        #   速度の設定を変えた。`retime` は `xy` を共有する）は同じ経路で、避けている途中に元の線へ
        #   戻すと障害物の側へ舵を切る
        if self._avoid is not None and (self._avoid_base is None or self._avoid_base.xy is not path.xy
                                        or self._avoid_passed(self._hint)):
            self._clear_avoid()
        elif self._avoid is not None:
            self._avoid_base = path
        self._poll_elastic(path, self._hint, vs, p)
        followed = self._avoid_path or self._with_sight(path, p)
        # ★ 追従に使う姿勢は「今」へ進めたもの（`pose` は点群の基準時刻のもので、計画の時点で
        #   数十ms 古い。3m/s なら 10cm 前後）。障害物の判定は地図の上の話なので `pose` のまま
        now = self.slam.pose_now(vs)
        pp = self._follow(followed, now, vs, p, cfg, self._hint)
        hit, dist = self._blocking(followed, pp.index, vs, p)
        if hit is not None and self._still_clear(hit, pp.index):
            hit, dist = None, math.inf
        self._hit_now = hit is not None
        if (hit is not None and p["obstacle_avoid"] > 0
                and self._plan_avoid(path, pp.index, hit, vs, p, kappa_frac=0.6)):
            followed = self._avoid_path
            pp = self._follow(followed, now, vs, p, cfg, pp.index)
            hit, dist = self._blocking(followed, pp.index, vs, p)
        if p["speed_ff"] > 0 and followed.closed:
            speed, st.accel_limit = self._speed_cmd(followed, pp.index, now, vs, p)
            pp = pp._replace(speed=speed)
        self._tick_path, self._tick_raw = followed, pp.speed
        self._hint = pp.index
        st.target_steer = pp.steer
        st.heading = pp.steer
        st.cross_track = pp.cross_track
        st.target_x, st.target_y = pp.target
        return pp, hit, dist

    def _follow(self, path, pose, vs, p, cfg, hint):
        """舵を決める。`ctrl_mode` 1 なら MPC、0 なら Pure Pursuit。"""
        if p["ctrl_mode"] >= 0.5:
            return self._mpc.follow(path, pose, vs.speed, vs.steer_actual,
                                    mpc_config(p, self.vehicle), hint=hint,
                                    now=vs.t_capture / 1e9 if vs.t_capture else None)
        return follow(path, pose, vs.speed, vs.steer_actual, cfg, hint=hint)

    def _detect_obstacles(self, st: AutoState, scan: Scan, p: dict[str, float]) -> None:
        """障害物を検出して `self._obs`（2周期続いたもの）と `st.obstacles` に載せる。"""
        g = self.slam.grid
        base = self.path.xy if self.path is not None else None
        key = (g.seq, p["obstacle_pad"])
        if key != self._obs_free_key or base is not self._obs_pad_ref:
            # 凍結地図では変わらない。毎周期作ると格子全体の膨張で数msかかる
            self._obs_pad = pad = self._obstacle_pad(p)
            self._obs_free = obs_mod.free_mask(g, pad)
            self._obs_near = obs_mod.free_mask(g, obs_mod.near_pad(pad))
            self._obs_free_key, self._obs_pad_ref = key, base
        pad = self._obs_pad
        now = obs_mod.detect(g, self.slam.deskew_scan(scan), self.slam.pose,
                             wall_pad=pad, free=self._obs_free, near=self._obs_near)
        self._obs = obs_mod.confirm(now, self._prev_obs)
        # ★ すぐ近く（今の速度で止まれる距離＋0.6m 以内）に出た物は、2周期目を待たずに扱う。
        #   見通しの悪いコーナーの先の障害物は、見えた時点でもう 1m も無い。確認を待つ 0.1s の間に
        #   1.8m/s で 18cm 進み、止まり切れずに当たった（sim.bench の toyota）。
        #   近い物は点が多く（`min_points` は距離によらない）、単発のノイズで出ることは少ない
        if p["obstacle_avoid"] > 0 or p["obstacle_stop"] > 0:
            v = abs(self._v_now)
            near = v * v / (2.0 * max(self._lim(p).a_brake, 0.1)) + v * self._lim(p).react_s \
                + self.vehicle.front_overhang + 0.6
            px, py = self.slam.pose.x, self.slam.pose.y
            for o in now:
                if (math.hypot(o.x - px, o.y - py) - o.r <= near
                        and not any(o is c for c in self._obs)):
                    self._obs.append(o)
        self._prev_obs = now
        st.obstacles = [v for o in self._obs for v in (o.x, o.y, o.r)]

    def _obstacle_pad(self, p: dict[str, float]) -> float:
        """実際に使う「壁の近くを無視する幅」[m]: 設定 `obstacle_pad` を、経路が壁にいちばん寄る所の
        壁までの距離まで詰めたもの（`_OBS_PAD_MIN` より下へは詰めない）。

        ★ ラインは頂点で壁から「車体半幅＋`line_margin`」（0.08 なら 17cm）を通る。無視する幅が
        それより大きいと、**経路の真上に置かれた物**の点がすべて無視する帯に落ち、足元の帯へ入る
        まで見つからない。実機（`obstacle_pad` 0.25・経路から壁まで 最小 15cm）で、
        壁から 19cm に立った人の脚が 1.7m 手前から毎周 4〜9 点写っていたのに候補にならず、
        2.3m/s のまま 0.4m 手前まで来た。同じ記録を通し直すと、0.15 なら 1.8m 手前で見つかる
        （障害物の無い 654 周期の誤検出は 0 のまま）。
        """
        pad = float(p["obstacle_pad"])
        if self.path is None or pad <= _OBS_PAD_MIN:
            return pad
        g = self.slam.grid
        col, row = g.to_cell(self.path.xy[:, 0], self.path.xy[:, 1])
        ok = g.inside(col, row)
        if not ok.any():
            return pad
        dist = obs_mod.wall_distance(g)
        return min(pad, max(_OBS_PAD_MIN, float(dist[row[ok], col[ok]].min())))

    def _blocking(self, path: rl_mod.RaceLine, index: int, vs: VehicleState,
                  p: dict[str, float]) -> tuple[obs_mod.Obstacle | None, float]:
        """経路の前方を塞ぐ障害物とその距離（base_link から経路に沿って）。

        避けるときは、止まれる距離に加えて**避け始める長さ**ぶん先まで見る
        （`obstacle_stop` の距離だけでは、見つけた時にはもう横へずれる余地が無い）。
        """
        ahead = p["obstacle_stop"]
        if p["obstacle_avoid"] > 0:
            v = max(vs.speed, 0.0)
            ahead = max(ahead, v * v / (2.0 * max(self._lim(p).a_brake, 0.1)) + 0.6 * v + 1.5)
        n = len(path)
        step = path.length / (n if path.closed else max(1, n - 1))
        hit, dist = obs_mod.blocking(
            self._obs, path.xy, index,
            half_width=self.vehicle.half_width + p["line_margin"],
            ahead_m=ahead + self.vehicle.front_overhang,
            step=step, skip_m=self.vehicle.front_overhang, closed=path.closed)
        if hit is None and self._obs and (p["obstacle_avoid"] > 0 or p["obstacle_stop"] > 0):
            hit = self._near_hit(path, index, step)
            if hit is not None:
                dist = self.vehicle.front_overhang
        return hit, dist

    def _near_hit(self, path: rl_mod.RaceLine, index: int, step: float) -> obs_mod.Obstacle | None:
        """車体のすぐ前（`blocking` が飛ばす足元の帯）で、このまま進むと車体が当たる障害物。

        ★ `blocking` は base_link から前オーバーハングぶんを見ない（真横の壁の誤検出で止まらない
        ため）。そのせいで、制動している途中に障害物がこの帯へ入った周期に「塞いでいない」
        扱いになり、まだ 0.5〜1.8m/s で動いているのに制動を解いて加速指令へ戻っていた
        （実機で確認）。帯（車体半幅＋余裕）ではなく、
        これから通る経路の点に車体外形を置いて当たるかどうかで見る。
        """
        n = len(path)
        k = max(1, int(self.vehicle.front_overhang / max(step, 1e-3))) + 1
        idx = index + np.arange(1, k + 1)
        idx = idx % n if path.closed else idx[idx < n]
        if len(idx) < 2:
            return None
        xy = path.xy[idx]
        for o in self._obs:
            if self._clear_of(xy, o) < _NEAR_CLEAR:
                return o
        return None

    def _obstacle_response(self, st: AutoState, hit: obs_mod.Obstacle | None, dist: float,
                           vs: VehicleState, p: dict[str, float], path: rl_mod.RaceLine,
                           index: int) -> bool:
        """塞がれていたら止まる。止まっても避けられなければ Hybrid A* で抜ける。止めたら True。

        「避ける」が有効なら、**減速して止まるのは開いた経路（停止点へ向かう経路）でも同じ**。
        横へ避ける・Hybrid A* で抜けるのは閉じた経路だけ（`_plan_avoid`）。開いた経路で
        両方とも切ると、`obstacle_stop` が 0（既定）のとき障害物に減速も制動もしない。
        """
        slow_on = p["obstacle_avoid"] > 0
        avoid_on = slow_on and path.closed
        # ★ 駆動が入っていない間（engage 済みで人が ARM を押す前）の静止は、スタックでも
        #   「止まってから避け直す」待ちでもない。数えると ARM 前に Hybrid A* へ入り、ARM した
        #   瞬間に後退を含む操縦から始まった
        driving = bool(vs.armed)
        if hit is None and self._brake_hold is not None:
            # 制動の途中で検出が1〜2周期途切れても解かない（すぐ近くの物は点が安定しない）
            self._brake_hold += self._dt
            if self._brake_hold <= _BRAKE_HOLD_S and abs(vs.speed) >= 0.05:
                st.brake = True
                st.target_speed = 0.0
                st.reason = "障害物で制動中（検出が途切れた。止まるまで保つ）"
                return True
        if hit is None:
            self._brake_hold = None
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
            a = max(self._lim(p).a_brake, 0.1)
            room = max(0.0, gap - _STOP_GAP - max(vs.speed, 0.0) * self._lim(p).react_s)
            st.target_speed = min(st.target_speed, math.sqrt(2.0 * a * room))
            stop_at = max(stop_at, _STOP_GAP + 0.05)
        if stop_at <= 0.0 or gap > stop_at:
            self._brake_hold = None
            if slow_on:
                st.reason = (f"前方 {gap:.1f}m に避けられない障害物。止まれる速度へ減速"
                             + (f"（{self._avoid_why}）" if self._avoid_why else ""))
                return True
            return False
        st.brake = True
        st.target_speed = 0.0
        self._brake_hold = 0.0
        ang = math.degrees(math.atan2(hit.y - st.pose_y, hit.x - st.pose_x) - st.pose_yaw)
        ang = (ang + 180.0) % 360.0 - 180.0
        st.reason = f"前方 {gap:.1f}m（{ang:+.0f}°）に障害物。停止"
        if slow_on and not avoid_on:
            st.reason += "（停止点へ向かう経路では避けない。どくまで待つ）"
        if not avoid_on or not driving or abs(vs.speed) >= 0.05:
            return True
        # 止まってから: 曲がれる限界の近くまで使って横へ避け直す → だめなら Hybrid A*
        self._blocked_t += self._dt
        wait = p["avoid_wait"]
        if (self._blocked_t >= 0.5 * wait
                and self._plan_avoid(path, index, hit, vs, p, kappa_frac=0.9)):
            st.reason += "。止まってから避ける経路を引いた"
            self._blocked_t = 0.0
        elif self._blocked_t >= wait and self._detour_tries < DETOUR_TRIES:
            self._start_detour(st, path, index, hit, dist, p, "横へ避けられない")
        elif self._detour_tries >= DETOUR_TRIES:
            st.reason += (f"。Hybrid A* でも{DETOUR_TRIES}回抜けられなかったので待つ"
                          + (f"（{self._detour_error}）" if self._detour_error else ""))
        return True

    def _plan_avoid(self, path: rl_mod.RaceLine, index: int, hit: obs_mod.Obstacle,
                    vs: VehicleState, p: dict[str, float], *, kappa_frac: float) -> bool:
        if not path.closed:
            return False                  # 停止点へ向かう開いた経路では止まるだけ
        self._avoid_room_path = path
        t0 = time.perf_counter()
        v = self.vehicle
        cfg = av_mod.AvoidConfig(
            half_width=v.half_width, front=v.front_overhang, rear=v.rear_overhang,
            kappa_max=v.kappa_max, footprint=tuple(v.footprint),
            # 避けた後の経路で `_blocking` が同じ障害物を拾わないよう、経路の余裕より広く取る
            margin=max(p["avoid_margin"], p["line_margin"]) + 0.01,
            clear=p["avoid_clear"], len_min=p["avoid_len_min"], kappa_frac=kappa_frac)

        speed = max(vs.speed, 0.0)
        ahead = speed * speed / (2.0 * max(self._lim(p).a_brake, 0.1)) + 0.6 * speed + 3.5
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
        # ★ 引けなかった直後は、車か障害物が動く（または使ってよい曲率が変わる）まで引き直さない。
        #   （見えている間、毎周期 10Hz で同じ計算をやり直さないため）
        key = (round(hit.x, 1), round(hit.y, 1), kappa_frac)
        if (self._avoid_fail is not None and self._avoid_fail[0] == key
                and self._avoid_fail[2] is path.xy):
            n = len(path)
            moved = abs((index - self._avoid_fail[1] + n // 2) % n - n // 2) * path.length / n
            if moved < 0.2:
                return False
        timed: list[tuple[av_mod.AvoidPlan, rl_mod.RaceLine]] = []   # 使える避け方と、その速度つきの経路
        why: list[str] = []

        def lap_time(plan: av_mod.AvoidPlan) -> float:
            """この避け方で1周する時間（速度を付けて測る。速い避け方を選ぶ）。"""
            rl = self._avoid_timed(plan, p)
            # ★ 今の速度から、この避け方の速度まで落とせないなら使わない（1m/s まで落とす避け方を
            #   3m/s・2.6m 手前で採ると、速すぎるまま横へ振って外れる）。止まれる速度へ減速を続け、
            #   遅くなってから引き直す
            if speed > float(rl.v[index]) + 0.3:
                why.append(f"{'左' if plan.offset > 0 else '右'}{abs(plan.offset) * 100:.0f}cm:"
                           f"減速が間に合わない({float(rl.v[index]):.1f}m/s)")
                return math.inf
            timed.append((plan, rl))
            return rl_mod._travel_time(rl.xy, rl.v, rl.closed)

        plan = av_mod.plan_offset(path, index, target, grow, self._body_check(), cfg,
                                  v_now=speed, ahead_m=ahead, score=lap_time, diag=why,
                                  room=self._avoid_room(path, p))
        self._avoid_ms = (time.perf_counter() - t0) * 1000.0
        chosen = next((rl for q, rl in timed if q is plan), None)
        if chosen is None:
            plan = None                    # 通れる避け方はあったが、どれも減速が間に合わない
        if plan is None:
            self._avoid_fail = (key, index, path.xy)
            self._avoid_why = "・".join(why[:6])
            # 決まった形では通らない。窓の中の線を引き直す計算を別プロセスへ投げる（結果は
            # `_poll_elastic` が次の周期以降に受け取る。それまでは止まれる速度へ減速を続ける）
            if self._elastic_job is None:
                self._elastic_base = path.xy
                self._elastic_job = submit_job(
                    av_mod.plan_elastic, path, index, target, grow, self._body_check(), cfg,
                    v_now=speed, room=self._avoid_room(path, p),
                    # 線の候補どうしを比べる速度。候補は点が違うので、学習中は全体の値（スカラー）を渡す
                    speed=speed_kwargs(p, self.vehicle, self._grip_scalar()),
                    v_tight=p["avoid_speed"], ahead_m=ahead)
            return False
        self._avoid_fail = None
        self._avoid_path = chosen
        self._avoid, self._avoid_base = plan, path
        self._avoid_how = "横ずらし"
        return True

    def _poll_elastic(self, path: rl_mod.RaceLine, index: int, vs: VehicleState,
                      p: dict[str, float]) -> None:
        """別プロセスで引き直した避ける経路（`avoid.plan_elastic`）ができていれば、使えるか確かめて乗る。

        計算の間に状況が変わっていたら捨てる: 経路が差し替わった・もう別の避け方に乗っている・
        車がずらし始めの点を過ぎた・今の速度からその経路の速度まで落とせない。
        """
        job = self._elastic_job
        if job is None or not job.done():
            return
        self._elastic_job = None
        try:
            plan = job.result()
        except Exception as e:                      # noqa: BLE001  避けられなかった扱い（止まる側）
            self._avoid_why = f"引き直しに失敗: {e!r}"[:80]
            return
        if plan is None:
            self._avoid_why = (self._avoid_why + "・引き直しても通れない")[:160]
            return
        if self._elastic_base is not path.xy or self._avoid is not None or index < 0:
            return
        n = len(path)
        if (int(plan.window[0]) - index) % n > n // 2:
            return                                  # もうずらし始めを過ぎた
        self._avoid_room_path = path
        rl = self._avoid_timed(plan, p)
        if max(vs.speed, 0.0) > float(rl.v[index]) + 0.3:
            return
        self._avoid_path = rl
        self._avoid, self._avoid_base = plan, path
        self._avoid_fail = None
        self._avoid_how = "引き直し"

    def _avoid_room(self, path: rl_mod.RaceLine, p: dict[str, float]):
        """経路の各点から左・右へずらせる量 [m]（`avoid.plan_offset` の `room`）。経路と地図ごとに1回作る。

        道幅（法線方向のレイ）から、車体半幅・合格線・コーナーで車体の角が張り出すぶんを引く。
        """
        g = self.slam.grid
        key = (g.seq, p["avoid_clear"])
        if self._room_ref is not path.xy or self._room_key != key:
            self._room_ref = path.xy
            nrm = cl_mod.normals(path.xy)
            left, right = cl_mod.measure(g, path.xy, nrm, MAX_TRACK_WIDTH)
            v = self.vehicle
            keep = (v.half_width + p["avoid_clear"]
                    + rl_mod.body_allowance(rl_mod.curvature(path.xy, True), v.front_overhang,
                                            v.rear_overhang))
            self._room = (np.maximum(left - keep, 0.0), np.maximum(right - keep, 0.0))
            self._room_key = key
        return self._room

    def _sight_cap(self, path: rl_mod.RaceLine, p: dict[str, float]) -> np.ndarray | None:
        """見通しで決まる速度の上限（点ごと）。障害物に対応しない設定なら `None`（抑えない）。

        ★ 壁の先端を回るヘアピンでは、頂点の先が 0.6m 手前まで見えない。そこへ 1.6m/s で入ると、
        先に障害物が居ても止まり切れない（sim.bench の toyota で衝突）。「見えている
        距離の中で止まれる速度」を上限にする。障害物に対応しない（`obstacle_avoid`・`obstacle_stop`
        とも 0）なら掛けない——何も居ない前提のコースで遅くする理由が無い。
        """
        use = float(p["sight_use"])
        if use <= 0.0 or (p["obstacle_avoid"] <= 0 and p["obstacle_stop"] <= 0):
            return None
        g = self.slam.grid
        if self._sight_ref is not path.xy or self._sight_seq != g.seq:
            # ★ 経路の点が見えるかだけを見る（`half_width` は渡さない）。車体が通る帯の両端まで
            #   求めると、凸の角では内側の端が必ず壁の陰になり、どの頂点も這う速度になる
            #   （toyota で 1 周 13.2→25.7s）。内側の壁に寄った障害物は見えるのが遅れる——残る危険
            self._sight = rl_mod.sight_distance(g, path.xy, path.closed)
            self._sight_ref, self._sight_seq = path.xy, g.seq
        lim = self._lim(p)
        # `use` < 1 は「見えている距離の 1/use 倍までなら止まれる」＝当たるとしても減速しきる
        # 手前まで、という緩めた保証。車体前端が届く手前 `_SIGHT_GAP` で止まる前提
        # 空走時間には、障害物を確かめる 1 周ぶんを足す。★「近い物は 2 周期目を待たずに扱う
        #   （`_detect_obstacles`）ので二重」と見て外すと、頂点の先の障害物に 0.9m/s で当たった
        #   （sim.bench の toyota。外す前は 0.2m 手前で止まる）。内側の壁の陰の物は
        #   経路の点より遅れて見え、点群 1 周の取得にも 0.1s かかるぶんを、この項が吸っている
        return rl_mod.sight_speed(self._sight / use, a_brake=lim.a_brake,
                                  react_s=lim.react_s + SCAN_PERIOD_S,
                                  reserve=(self.vehicle.front_overhang + _SIGHT_GAP) / use)

    def _with_sight(self, path: rl_mod.RaceLine, p: dict[str, float]) -> rl_mod.RaceLine:
        """経路の速度に見通しの上限を掛けたもの（`_sight_cap`。掛けないなら `path` そのまま）。"""
        cap = self._sight_cap(path, p)
        if cap is None:
            return path
        key = (self._skey(p), p["obstacle_avoid"] > 0, p["obstacle_stop"] > 0, p["sight_use"],
               self.slam.grid.seq)
        if self._sighted_src is not path or self._sighted_key != key:
            hold = self._grip_hold(path)
            if hold is not None:
                cap = np.minimum(cap, hold)
            self._sighted = self._timed(path, p, cap)
            self._sighted_src, self._sighted_key = path, key
            # 追う速度での 1 周と、上限が無いときの 1 周（`_sight_note`）
            t_free = rl_mod._travel_time(path.xy, path.v, path.closed)
            t_cap = rl_mod._travel_time(self._sighted.xy, self._sighted.v, path.closed)
            self._sight_msg = (f"・★見通しの上限で 1周 {t_cap:.1f}s（上限なし {t_free:.1f}s）"
                               if path.closed and t_cap > t_free * _SIGHT_NOTE_RATIO else "")
        return self._sighted

    def _sight_note(self, path: rl_mod.RaceLine, p: dict[str, float]) -> str:
        """見通しの上限で 1 周が目立って遅くなっていれば、その注記（追う速度での見積もり）。

        経路選択の「見積もり」は上限を掛ける前の線のもの。障害物対応が有効だと実際に追う速度は
        それより遅く、実機では表示 6.2s に対して 8.1s の速度を追っていた。
        """
        return self._sight_msg if self._with_sight(path, p) is not path else ""

    def _avoid_timed(self, plan: av_mod.AvoidPlan, p: dict[str, float]) -> rl_mod.RaceLine:
        """避ける経路に速度を付ける: 曲率で決まる速度（`retime`）と、余裕で決まる上限（`speed_cap`）。
        見通しの上限は元の経路のものを使う（避ける経路は元の経路と同じ添字で、形もほぼ同じ）。"""
        base = self._avoid_room_path if self._avoid_room_path is not None else plan.path
        rl = rl_mod.retime(plan.path, v_cap=self._sight_cap(base, p), **self._spd(p, plan.path))
        cap = av_mod.speed_cap(plan.clearance, p["avoid_clear"], p["avoid_speed"], p["v_max"])
        return av_mod.cap_speed(rl, plan.window, cap, self._lim(p).a_brake, self._lim(p).a_accel)

    def _blend_back(self, path: rl_mod.RaceLine, p: dict[str, float]) -> None:
        """今の横ずれから滑らかに経路へ戻る一時経路に乗せる（`avoid.blend_in`）。"""
        if not path.closed:
            return
        x, y, _ = self.slam.pose
        j = nearest_index(path, x, y)
        nrm = cl_mod.normals(path.xy)[j]
        d0 = float((x - path.xy[j, 0]) * nrm[0] + (y - path.xy[j, 1]) * nrm[1])
        self._hint = j
        if abs(d0) < _BLEND_MIN:
            # ★ ほぼ経路の上に抜けたなら戻り経路は要らない（横ずれ 0cm でも 1.5m を
            #   `join_speed` で走ってしまう）
            return
        plan = av_mod.blend_in(path, j, d0, max(0.8, 6.0 * abs(d0)), self._body_check())
        rl = rl_mod.retime(plan.path, **self._spd(p, plan.path))
        self._avoid_path = av_mod.cap_speed(rl, plan.window, p["join_speed"], self._lim(p).a_brake,
                                            self._lim(p).a_accel)
        self._avoid, self._avoid_base = plan, path

    def _still_clear(self, hit: obs_mod.Obstacle, index: int) -> bool:
        """避けている障害物が「塞いでいる」と出ても、避ける経路の残りで車体が当たらないなら True。

        ★ 遠くで見つけたときは点が少なく半径を小さく見積もる。近づいて見積もりが大きく
        なると、帯（車体半幅＋余裕＋半径）の判定では「まだ塞いでいる」と出て、避けている
        途中で止まった（sim.bench の toyota で2周目）。当たるかどうかを今見えている点で直接測る。
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
        return self._clear_of(self._avoid_path.xy[idx], hit) >= _NEAR_CLEAR

    def _clear_of(self, xy: np.ndarray, o: obs_mod.Obstacle) -> float:
        """経路の点列 `xy` に車体を置いたときの、障害物 `o` との余裕 [m]。

        ★ 見えている点があればそれで測る。重心と半径の円で測ると、近づくにつれて重心が
        手前へ約10cm動き、半径（重心からの最大距離）も点のばらつきで膨らんで、横へ避けている
        途中に「当たる」と出て止まった（sim.bench の toyota・半径10cm の円柱で毎周）。
        """
        if o.pts is not None:
            return av_mod.clearance_to_points(xy, self._footprint(), o.pts)
        return av_mod.clearance_along(xy, self._footprint(), [o])

    def _avoid_passed(self, index: int) -> bool:
        """車が避ける窓の終わりを過ぎたか（元の経路へ戻してよい）。"""
        assert self._avoid is not None and self._avoid_path is not None
        n = len(self._avoid_path)
        d = (index - int(self._avoid.window[-1])) % n
        return 0 < d < n // 2

    def _hazard_now(self) -> bool:
        """ハザードを出す状況か。ここでは出さない（駐車を持つ `Slam2dRoute` が上書きする）。"""
        return False

    def _signal_step(self, st: AutoState) -> None:
        """灯火を `st` へ書く。障害物に対応している間は パッシング → 避ける側のウィンカー。"""
        a = self._avoid
        avoiding = a is not None and a.obstacle is not None
        obstacle = (self._hit_now or avoiding or self.phase == DETOUR
                    or self._brake_hold is not None)
        side = 0
        if avoiding:
            side = 1 if a.offset > 0 else -1
        elif self.phase == DETOUR:
            side = self._detour_side
        st.passing, st.winker_left, st.winker_right = self._signals.update(
            self._dt, obstacle=obstacle, side=side, hazard=self._hazard_now())

    def _clear_avoid(self) -> None:
        self._avoid = self._avoid_path = self._avoid_base = None

    def _avoid_note(self) -> str:
        if self._avoid is None:
            return ""
        if self._avoid.obstacle is None:
            return f"・経路へ戻っている（横ずれ {self._avoid.offset * 100:+.0f}cm から）"
        side = "左" if self._avoid.offset > 0 else "右"
        return (f"・★障害物を{side}へ {abs(self._avoid.offset) * 100:.0f}cm 避けている"
                f"（{self._avoid_how}・余裕 {self._avoid.clearance * 100:.0f}cm）")

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
        self._detour_side = 0 if abs(ly) < 0.15 else (1 if ly > 0 else -1)
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
        # `ParkToPoint.merged` を通らないので、上限はここで掛ける（Pi は Mac より 5〜8 倍遅い）
        pp["plan_budget_ms"] = min(float(p["detour_budget_ms"]), 150.0)
        pp["cruise_speed"] = p["detour_speed"]
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
            if self._coast_t > 0.0:
                # 見失いから戻った周期は、自己位置が最大で探し直しの幅（数十cm）跳ぶ。直後の横偏差を
                # 横グリップの前兆（線から外れた）と取り違えない
                self._grip_quiet(self._grip_gate_t, "自己位置が戻った直後")
            self._coast_t = 0.0
            return False
        self._coast_t += self._dt
        if self._joined and p["coast_s"] > 0.0 and self._coast_t <= p["coast_s"]:
            return True
        st.reason = (f"自己位置が信用できない（一致度 {st.match_score:.2f} < "
                     f"{p['min_score']:.2f}"
                     + (f"・{self._coast_t:.1f}s 続いた" if p["coast_s"] > 0.0 else "") + "）")
        return None

    def _md_fault(self, st: AutoState, vs: VehicleState, p: dict[str, float]) -> str | None:
        """後輪のモータドライバが無応答のときの抑え。戻り値は reason に足す注記、止めたら `None`。

        実機で駆動バッテリーが垂れ、加速のたびに右後輪の MD が約5秒落ちた。その間の
        加速は 1.7→0.6 m/s²、制動は −2.5→−1.4 m/s²。**約5秒で戻るので止めずに速度を抑える**
        （DISARM は人が解くまで戻らない。両輪とも落ちると制動も効かないので、0 を指令して
        舵だけ経路を追い、戻るのを待つ）。

        ★ `md_status` が 0（一度も状態が来ていない・テストの既定値）は故障と見なさない。
        """
        down = [bool(s) and not (s & MDS_COMM_OK) for s in vs.md_status[:2]]
        if not any(down) or not vs.armed:
            self._md_t = 0.0
            return ""
        if all(down):
            st.target_speed = 0.0
            st.brake = True
            st.reason = "★後輪の MD が両方とも無応答（駆動も制動も効かない）。戻るのを待つ"
            return None
        self._md_t += self._dt
        side = "左" if down[0] else "右"
        if self._md_t > p["md_fault_s"]:
            st.target_speed = 0.0
            st.brake = True
            st.reason = (f"★{side}後輪の MD が {self._md_t:.0f}s 無応答。止まって戻るのを待つ"
                         "（駆動バッテリーを確かめる）")
            return None
        st.target_speed = min(st.target_speed, p["md_fault_speed"])
        return f"・★{side}後輪 MD 無応答（減速）"

    def _coast_note(self, coasting: bool, p: dict[str, float]) -> str:
        if not coasting:
            return ""
        return f"・★推測航法で続行 {self._coast_t:.1f}/{p['coast_s']:.1f}s"

    def _retime(self, p: dict[str, float]) -> None:
        """速度の設定が変わっていたら、経路の速度プロファイルだけを作り直す。

        ★ 保存した地図の速度は地図を作ったときの設定のまま（`rl_mod.retime`）。
        `request_load()` の後はここで必ず1回作り直される。
        """
        key = self._skey(p)
        if key == self._speed_key or self.path is None:
            return
        self.path = self._timed(self.path, p, self._grip_hold(self.path))
        self._speed_key = key
        self._map_dirty = True

    # ── 横グリップの学習（`nav/grip.py`。`slam2d_route` と共通） ──

    def _skey(self, p: dict[str, float]) -> tuple:
        """速度プロファイルを作り直す鍵。学習中は、学習した値の版（`_grip_tok`）も入れる。"""
        return speed_key(p) + ((self._grip_tok,) if self._grip_on else ())

    def _spd(self, p: dict[str, float], path: rl_mod.RaceLine) -> dict:
        """`path` に速度を付ける設定（`speed_kwargs`）。学習中は点ごとの横の限界と、学習した
        加速・減速の上限を通す。"""
        lv = self._grip_levels(path)
        if lv is None:
            return speed_kwargs(p, self.vehicle)
        m = self._grip.map
        return speed_kwargs(p, self.vehicle, dict(a_lat=lv[0], a_grip=lv[1], a_accel=m.accel,
                                                  a_brake=m.decel))

    def _timed(self, path: rl_mod.RaceLine, p: dict[str, float],
               v_cap: np.ndarray | None = None) -> rl_mod.RaceLine:
        """周回の経路に今の設定と学習値で速度を付ける。"""
        rl = rl_mod.retime(path, v_cap=v_cap, **self._spd(p, path))
        # `_grip_begin` が「足元は前の速度のまま」の上限に使う
        self._grip_plain = (path.xy, rl.v)
        return rl

    def _grip_quiet(self, t_ns: int, why: str) -> None:
        """時刻 `t_ns` から `_GRIP_SWITCH_QUIET_S` のあいだ学習を止める（`_grip_end` のゲート）。"""
        self._grip_quiet_ns = int(t_ns) + int(_GRIP_SWITCH_QUIET_S * 1e9)
        self._grip_quiet_why = why

    def _grip_learned(self) -> tuple[float, float, float] | None:
        """学習した全体の値 `(横, 加速, 減速)` [m/s²]（学習していない・まだ値が無いなら `None`）。"""
        m = self._grip.map
        if not self._grip_on or m.level <= 0.0 or m.accel <= 0.0 or m.decel <= 0.0:
            return None
        return m.level, m.accel, m.decel

    def _grip_scalar(self) -> dict | None:
        """`speed_kwargs` の `grip` に渡す全体の値（回避の線の比較用）。"""
        lv = self._grip_learned()
        if lv is None:
            return None
        return dict(a_lat=lv[0], a_grip=lv[0], a_accel=lv[1], a_brake=lv[2])

    def _grip_line_kw(self) -> dict | None:
        """ラインを引く計算に渡す学習値。`grip_reline` が 0 なら `None`（同定値で引く）。"""
        if (self._last_p or {}).get("grip_reline", 0) <= 0:
            return None
        return self._grip_scalar()

    def _lim(self, p: dict[str, float]) -> Limits:
        """今の走りの上限（学習中は学習した値）。"""
        return limits(p, self.vehicle, self._grip_learned())

    def _grip_worse(self, old: rl_mod.RaceLine | None, new: rl_mod.RaceLine) -> bool:
        """学習値で引き直した線 `new` が、今の線 `old` より「曲がり切れない区間」が長いか。

        ★ 横の限界を高くして引くと、最小時間へ寄せた結果、車の曲がれる限界を超える所が残ることが
        ある（真値 0.70 で 4.9m。そこへ乗り換えて壁に当たった、sim.bench）。そういう線は採らない。
        """
        if old is None:
            return False
        lim = kappa_limit(self._last_p or {}, self.vehicle)
        return (rl_mod.tight_sections(new.xy, new.closed, lim)[0]
                > rl_mod.tight_sections(old.xy, old.closed, lim)[0] + 0.05)

    def _grip_line_stale(self) -> bool:
        """今のラインが、学習した値と違う限界で引いたものか（＝引き直す）。

        学習が落ち着いてから（横が収束し、減速も決まってから。「下げるだけ」のときは保存した
        値を読んだ時点で）、ラインを引いたときの値と 3% 以上違えば引き直す。値が動いている間は
        引き直さない（引き直しは数秒かかり、できた線へは乗れる場所で移る）。
        """
        lv = self._grip_learned()
        if lv is None:
            return self._grip_line is not None  # 学習を切った。同定値の線へ戻す
        if (self._last_p or {}).get("grip_reline", 0) <= 0:
            return False
        g = self._grip
        if not (g.down_only or (g.converged and g.long.dec_done)):
            return False
        built = self._grip_line
        return built is None or any(abs(a - b) > 0.03 * b for a, b in zip(lv, built))

    def _grip_levels(self, path: rl_mod.RaceLine) -> tuple[np.ndarray, np.ndarray] | None:
        """経路の各点の `(求める横加速度, 限界の見積もり)` [m/s²]（学習していなければ `None` = 同定値）。"""
        if not self._grip_on:
            return None
        g = self._grip
        key = g.ver
        for ref, k, arr in self._grip_cache:
            if ref is path.xy and k == key:
                return arr
        arr = g.levels(path.xy, self._grip_wall(path))
        self._grip_cache = [(path.xy, key, arr)] + self._grip_cache[:2]
        return arr

    def _grip_wall(self, path: rl_mod.RaceLine) -> np.ndarray | None:
        """経路の各点から `_GRIP_WALL_AHEAD_M` 先までの、車体外形と壁の余裕の最小 [m]。

        同じ超過でも、耐えられる横偏差は場所で違う。外へ膨らむのは前兆が出た所の少し先なので、
        先の余裕まで見る。閉じた経路だけ（`_body_check` は巡回の接線で測る）。
        """
        if not path.closed or len(path) < 3:
            return None
        key = (self.slam.grid.seq,)
        if self._grip_wall_ref is not path.xy or self._grip_wall_key != key:
            c = self._body_check().clearance(path.xy)
            out = c.copy()
            for k in range(1, int(_GRIP_WALL_AHEAD_M / max(path.length / len(path), 1e-6)) + 1):
                np.minimum(out, np.roll(c, -k), out=out)
            self._grip_wall_ref, self._grip_wall_key, self._grip_wall_arr = path.xy, key, out
        return self._grip_wall_arr

    def _grip_tight(self, path: rl_mod.RaceLine, p: dict[str, float]) -> np.ndarray:
        """経路の各点が「曲がり切れない区間」（の前後 0.4m）にあるか（`_tight_note` と同じ基準）。

        そこは舵が上限に張り付き、速度に関係なく膨らむ。グリップの前兆と取り違えないよう、
        学習から外す。
        """
        key = (p["kappa_use"],)
        if self._grip_tight_ref is not path.xy or self._grip_tight_key != key:
            over = rl_mod.speed_kappa(path.xy, path.closed) > kappa_limit(p, self.vehicle) * 1.01
            out = over.copy()
            for k in range(1, 5):
                if path.closed:
                    out |= np.roll(over, k) | np.roll(over, -k)
                else:
                    out[k:] |= over[:-k]
                    out[:-k] |= over[k:]
            self._grip_tight_ref, self._grip_tight_key, self._grip_tight_arr = path.xy, key, out
        return self._grip_tight_arr

    def _grip_hold(self, path: rl_mod.RaceLine) -> np.ndarray | None:
        """学習で上げた直後、車の先の一定距離は前の速度のままにする上限（点ごと。無ければ `None`）。"""
        h = self._grip_hold_cap
        return h[1] if h is not None and h[0] is path.xy else None

    def _grip_make(self, by_time: bool) -> grip_mod.GripLearner:
        v = self.vehicle
        cls = grip_mod.TimeLearner if by_time else grip_mod.GripLearner
        return cls(yaw_wn=v.yaw_natural_freq_rad_s, yaw_zeta=v.yaw_damping,
                   rolling_resistance=v.rolling_resistance)

    def _grip_begin(self, vs: VehicleState, p: dict[str, float]) -> None:
        """周期の初めに学習の設定を渡し、値が変わっていれば速度の作り直しを予約する。

        ★ 上げた速度は車より十分先の点にだけ反映する。上げただけの変更なら、車の先
        `_GRIP_AHEAD_M`（または `_GRIP_AHEAD_S` 秒ぶん）は今の速度を上限に残し、そこを抜けたら外す。
        下げる変更は全体へすぐ反映する。
        """
        on = p.get("grip_learn", 0) > 0
        if on != self._grip_on:
            self._grip_on = on
            self._grip_hold_cap = None
            self._grip_ver = (-1, -1)
        # この先の判断で止まる・抑える場合に備えて、いったん「学習しない」にしておく（`_grip_end`）
        self._grip_gate = "走行を抑えている"
        self._grip_gate_t = int(vs.t_capture)
        self._grip_capped = False
        if not on:
            return
        by_time = p.get("grip_by_time", 1) > 0
        if isinstance(self._grip, grip_mod.TimeLearner) != by_time:
            # 方式を切り替えた。学習器を作り直し、保存した値から始める（判定の設定は引き継ぐ）
            self._grip_save()
            cfg = self._grip.cfg
            self._grip = self._grip_make(by_time)
            self._grip.cfg = cfg
            self._grip_load(getattr(self, "_grip_name", ""))
        g = self._grip
        if not g.cfg.fixed_step:
            g.cfg.step_up = p["grip_step"] / 100.0
        down_only = p["grip_down_only"] > 0
        v = self.vehicle
        g.configure(init_level=p["grip_init"] * v.a_lat_max, max_level=p["grip_max"],
                    down_only=down_only, lat_use=1.0)
        # 加速は学習しない: 目標車速を動かす傾きの上限（`[safety] cmd_accel_limit_m_s2`）で固定。出なければ
        # STM32 の TC が抑える。減速は低めから始め、同じ上限まで上げてよい
        g.long.configure(acc_init=v.speed_ramp_max_m_s2,
                         dec_init=LEARN_BRAKE_INIT * v.a_decel_max,
                         dec_max=v.speed_ramp_max_m_s2, down_only=down_only)
        path, i = self.path, self._hint
        n = len(path) if path is not None else 0
        hold = self._grip_hold_cap
        ver = (g.ver + g.long.ver, g.ver_down + g.long.ver_down)
        if ver != self._grip_ver:
            raised = self._grip_ver[0] >= 0 and ver[1] == self._grip_ver[1]
            self._grip_ver = ver
            self._grip_tok += 1
            self._grip_hold_cap = None
            if raised and path is not None and path.closed and 0 <= i < n and len(path.v) == n:
                m = int(max(_GRIP_AHEAD_M, abs(vs.speed) * _GRIP_AHEAD_S) / (path.length / n)) + 1
                cap = np.full(n, np.inf)
                idx = (i + np.arange(min(m, n))) % n
                plain = self._grip_plain
                src = plain[1] if plain is not None and plain[0] is path.xy and len(plain[1]) == n \
                    else path.v
                cap[idx] = src[idx]
                self._grip_hold_cap = (path.xy, cap, i, m)
            else:
                g.long.note_change()            # 足元のプロファイルが変わる
        elif hold is not None and (path is None or hold[0] is not path.xy
                                   or (i - hold[2]) % max(n, 1) >= hold[3]):
            self._grip_hold_cap = None          # 窓を抜けた。上限を外して作り直す
            self._grip_tok += 1
            g.long.note_change()

    def _grip_end(self, st: AutoState, vs: VehicleState, p: dict[str, float],
                  path: rl_mod.RaceLine, pp, hit, coasting: bool, md_note: str) -> str:
        """この周期が「通常の周回」なら学習を許し、前兆で落とした速度を指令に掛ける。reason の注記を返す。

        学習を止める: 経路に乗るまで・推測航法（自己位置の一致度が低い）・障害物の回避・MD 無応答・
        舵が上限・曲がり切れない区間・見通しで速度を抑えている区間。止まる／制動する周期は
        `_grip_begin` が入れた「走行を抑えている」のまま。
        """
        if not self._grip_on:
            return ""
        g = self._grip
        i = pp.index
        followed = self._tick_path
        gate = ""
        if not self._joined:
            gate = "経路に乗るまで"
        elif coasting:
            gate = "推測航法"
        elif self._avoid is not None or hit is not None:
            gate = "障害物"
        elif md_note:
            gate = "MD無応答"
        elif abs(st.target_steer) >= 0.97 * self.vehicle.max_steer:
            gate = "舵が上限"
        elif vs.t_capture < self._grip_quiet_ns:
            gate = self._grip_quiet_why or "線を乗り換えた直後"
        elif 0 <= i < len(path) and self._grip_tight(path, p)[i]:
            gate = "曲がり切れない区間"
        elif (followed is not None and followed is not path and len(followed) == len(path)
              and 0 <= i < len(path) and followed.v[i] < path.v[i] - 0.02):
            gate = "見通しの上限"
        self._grip_gate = gate
        g.observe_cross(0.0 if gate else pp.cross_track)
        if path.closed and len(path.v) == len(path):
            # 計画のラップ（減速を試す形を重ねる前の速度で）と、横の上限だけを学習の「比べる相手」へ
            # 戻したときのラップ。加速・減速が変わって縮んだぶんを、横を上げた効果と分ける
            plain = self._grip_plain
            v = plain[1] if plain is not None and plain[0] is path.xy and len(plain[1]) == len(path) \
                else path.v
            ref = g.ref_level
            if self._grip_plan[0] is not v or self._grip_plan[1] != ref:
                lap_ref = 0.0
                if ref > 0.0:
                    kw = dict(self._spd(p, path))
                    kw["a_lat"] = kw["a_grip"] = np.full(len(path), max(ref, g.floor))
                    lap_ref = rl_mod._travel_time(path.xy, rl_mod.retime(path, **kw).v, True)
                self._grip_plan = (v, ref, rl_mod._travel_time(path.xy, v, True), lap_ref)
            g.observe_plan(self._grip_plan[2], self._grip_plan[3])
        t = vs.t_capture / 1e9
        if self.laps != self._grip_laps:
            self._grip_laps = self.laps
            g.lap(t)
        cap = self._grip_cap()
        if cap is not None and cap[0] < st.target_speed:
            st.target_speed, st.accel_limit = cap
            self._grip_capped = True
        if t - self._grip_save_t > _GRIP_SAVE_S:
            self._grip_save_t = t
            self._grip_save()
        m = g.map
        lg = g.long
        # 記録から判定を追えるように（`AutoState.grip`）。実機では真値が無いので、これが唯一の手がかり
        x, y = self.slam.pose[:2]
        st.grip = {
            "lat": round(m.level, 3), "acc": round(m.accel, 3), "dec": round(m.decel, 3),
            "lat_here": round(g.level_at(x, y), 3),
            "v_plan": round(self._grip_vp or 0.0, 3), "a_plan": round(self._grip_ax, 3),
            "excess": round(g.last_excess, 4), "thr": round(g.last_thr, 4),
            "noise": round(g.noise_sigma, 5),
            "mean": round(g.last_mean[0], 4), "mean_thr": round(g.last_mean[1], 4),
            "cross_gain": round(g.last_cross_gain, 4),
            "events": len(g.events), "trials": g.trials, "passes": g.passes,
            "win": lg.windows, "lag_dec": round(float(lg.last), 3),
            **g.diag(),
        }
        last = f"・直近 {g.events[-1]['kind']}" if g.events else ""
        return (f"・学習 横 {m.level:.2f}・加速 {m.accel:.2f}・減速 {m.decel:.2f}m/s²"
                f"（{g.state(gate)}・前兆 {len(g.events)}回{last}）")

    def _grip_cap(self) -> tuple[float, float] | None:
        """前兆で落としている `(目標速度 [m/s], 落とす速さ [m/s²])`。落としていなければ `None`。

        落とす速さを限界の一部に抑えるのは、強く減速するとそのぶん横に使える力が減るため。
        大きく超えているとき（`GripLearner.cap_hard`）は抑えない（0 = STM32 の上限で落とす）。
        """
        g = self._grip
        if not self._grip_on or g.cap is None:
            return None
        return g.cap, 0.0 if g.cap_hard else max(0.5, g.cfg.cap_decel * g.map.level)

    def _grip_observe(self, vs: VehicleState) -> None:
        """車両状態 1 つぶんを学習へ渡す（100Hz）。姿勢は車速とジャイロで今へ進めたもの。"""
        t = int(vs.t_capture)
        gate = self._grip_gate
        if not gate and t - self._grip_gate_t > 300_000_000:
            gate = "計画が止まっている"
        g = self._grip
        a = g.launch.feed(t / 1e9, vs.speed, vs.tc_active, vs.steer_actual)
        if a is not None:
            g.set_launch(a)
        x, y, _ = self.slam.pose_now(vs)
        v = self.vehicle
        kappa = math.tan(v.steer_map.effective(vs.steer_actual)) / v.wheelbase
        # 追いつく途中（`speedctl.catch_up`）は、計画の加速度ではなく今の車速から届く速度を指令して
        # いる。加速・減速の測定には使わない（横は見る）
        g.observe(t / 1e9, x, y, vs.speed, vs.yaw_rate, kappa, a_x=self._grip_ax, gate=gate,
                  tc_active=vs.tc_active, v_plan=self._grip_vp,
                  long_gate="追いつく途中" if self._grip_catch else "")

    def _grip_load(self, name: str) -> None:
        """地図 `name` の横に保存した学習値を読む（無ければ初期値から）。車に固有の比も読む。"""
        self._grip_save()
        g = self._grip
        g.reset()
        self._grip_name = name
        self._grip_cache = []
        self._grip_hold_cap = None
        self._grip_ver = (-1, -1)
        self._grip_tok += 1
        self._grip_saved = (0, 0.0)
        g.launch_ratio = mapstore.load_grip_vehicle()
        text = mapstore.load_grip(name)
        if not text:
            return
        try:
            g.load(grip_mod.GripMap.from_json(text))
        except (ValueError, TypeError, KeyError):
            pass                                # 壊れた・古い形式。初期値から学習し直す

    def _grip_save(self) -> None:
        """学習した値を地図の横へ書く（学習器が保存に値すると言い（`worth_saving`）、前回から
        変わっているときだけ）。"""
        g = getattr(self, "_grip", None)
        name = getattr(self, "_grip_name", "")
        if g is None or not name or not g.worth_saving():
            return
        key = (g.ver + g.long.ver, g.launch_ratio)
        if key == self._grip_saved:
            return
        self._grip_saved = key
        try:
            mapstore.save_grip(name, g.map.to_json())
            if g.launch_ratio > 0.0:
                mapstore.save_grip_vehicle(g.launch_ratio)
        except OSError:
            pass

    # ── 経路に乗る・外れたら乗り直す（`slam2d_route` と共通） ──

    def _join_cap(self, st: AutoState, pp, path, pose, p: dict[str, float]) -> None:
        """経路に乗る（横8cm・向き15°以内）までは `join_speed` に抑える。

        ★ 走り出し（LOCATE 直後）の車は経路から横に10〜30cm・向きに25°ずれている。
        抑えないと最高速のまま大回りして壁に当たった（toyota/toyota2 の sim.bench、
        このplannerも同じ bench で衝突・61cm 外れて0周だった）。
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

    def close(self) -> None:
        """この planner が投げた計算を捨てる（プロセスは使い回すので止めない）。"""
        # `_job`・`_stop_job` は `slam2d_route`
        for name in ("_line_job", "_build_job", "_elastic_job", "_job", "_stop_job"):
            self._drop_job(name)
        self._grip_save()

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
