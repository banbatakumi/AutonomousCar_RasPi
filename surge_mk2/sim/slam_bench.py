"""slam2d（`raspi/auto/_slam2d_nav.Slam2dNav`）を**真値と突き合わせて**評価する専用ベンチ。

    .venv/bin/python -m sim.slam_bench                              # 全コース×既定条件
    .venv/bin/python -m sim.slam_bench --course toyota --seeds 3 --errors harsh
    .venv/bin/python -m sim.slam_bench --course normal --plot

## なぜ `sim.bench` とは別に要るのか

`sim.bench` は planner 全体（EXPLORE の Follow the Gap 走行 → BUILD → RACE）を
実時間で回す。SLAM の精度だけを測りたいときには 3 つの点で向かない:

1. **SLAM が制御ループの中にいる。** 推定が崩れると走り方が変わり、走り方が
   変わると推定の条件が変わる。何が原因で数字が悪いのか切り分けられない
2. **実時間でしか回らない**（`SimLink` が壁時計で進む）。コース×シード×誤差
   条件を振ると何十分もかかる
3. **推測航法に誤差が入らない経路がある**（`VirtualStm32` は `speed` を真値の
   まま送る。倍率誤差が乗るのは `odom_dist` だけ）

ここでは車を**真値の中心線に沿って真値の姿勢で**純追従させ（SLAM を制御から
切り離す）、仮想時間で回す。LiDAR は実機と同じ `VirtualLidar → ScanAssembler`
経路（鏡像・セクタ欠損・ノイズ・遅延・走査ゆがみ）を通し、車速・ヨーレートには
`SensorModel` で誤差を注入する。

## 測るもの（2段）

1. **地図作成（EXPLORE 相当）**: `explore_speed` で `laps` 周。推定軌跡と真値の
   ずれ（ATE）・見失い・1周期の処理時間・最後の `freeze()`（ループ閉じの一括
   最適化）の前後で地図がどれだけ真の壁に乗っているか
2. **凍結地図での追従（RACE 相当）**: 1 で作った地図を凍結したまま `race_speed`
   で `race_laps` 周。**レース中の自己位置精度はここで決まる**

真値との比較は**SLAM の原点（最初の点群の時刻の姿勢）から見た相対姿勢**どうしで
行う。SLAM の姿勢は点群の基準時刻 `t_ref` のものなので、真値もその時刻へ
補間して取り出す（`plan()` を呼んだ時刻の真値と比べると伝送遅延ぶん毎回ずれる）。

**このファイルを実機のコードから import してはいけない。**
"""

from __future__ import annotations

import argparse
import os
import copy
import dataclasses
import json
import math
import statistics
import sys
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Callable

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from raspi.msgs import SPEED_DEADBAND_MPS  # noqa: E402
from slam2d.core.types import Pose2D, between, compose, inverse, wrap_angle  # noqa: E402
from raspi.msgs.convert import ScanAssembler  # noqa: E402
from raspi.nav.body_mask import body_ranges, mask_body  # noqa: E402
from raspi.msgs.types import VehicleState  # noqa: E402

from .course import Course, list_courses  # noqa: E402
from .lidar import VirtualLidar  # noqa: E402
from .vehicle import DriveInput, VehicleModel, VehicleSpec  # noqa: E402

__all__ = ["SensorModel", "RunConfig", "run", "load_course"]

NS = 1_000_000_000
DT = 0.002                     #: 車両の積分刻み [s]
TELEMETRY_HZ = 100             #: `VehicleState` の更新周期（実機の TELEMETRY と同じ）
COURSE_DIR = Path(__file__).resolve().parent / "courses"


# ── センサ誤差 ─────────────────────────────────────────────────────────────

from .sensor_faults import (  # noqa: E402
    SensorModel, _wall_bbox, car_box, inject_boxes, inject_ghost, inject_pillars, mask_fov,
)


def load_course(name_or_path: str, route: str = "") -> Course:
    """コースを読み、PNG コースなら隣の `<name>_centerline.npz` から中心線を補う。

    中心線の無いコース（分岐のある壁モードのコース、`toyota2` など）は、真のコースから
    道路グラフを作って周回を引き、それを中心線の代わりにする（`_route_centerline`）。
    `route` は `<routes.json>:<グループ>`（`sim.bench --routes` と同じ形・シムの世界座標）。
    空なら出発点の姿勢を経由点にした最短の周回。
    """
    p = Path(name_or_path)
    if not p.exists():
        found = [q for q in list_courses() if q.stem == name_or_path]
        if not found:
            raise SystemExit(f"コースが無い: {name_or_path}")
        p = found[0]
    c = Course.load(p)
    if c.centerline is None:
        npz = p.with_name(f"{p.stem}_centerline.npz")
        if npz.exists():
            d = np.load(npz)
            c.centerline = np.asarray(d["centerline"], dtype=np.float64)
            c.width = float(d["width"])
    if c.centerline is None or route:
        c.centerline = _route_centerline(c, route)
    return c


def _route_centerline(c: Course, route: str) -> np.ndarray:
    """真のコースの道路グラフ上の周回（`raspi/nav/route.py`）。**真値の運転手の道**。"""
    from raspi.core.vehicle import Vehicle
    from raspi.nav.roadgraph import build_graph, trinary_from_bool
    from raspi.nav.route import RouteError, Waypoint, plan_loop

    veh = Vehicle.load()
    try:
        g = build_graph(trinary_from_bool(c.grid), resolution=c.resolution, origin=c.origin,
                        keep=veh.half_width + 0.02, seed=c.start[:2])
        if route:
            path, _, group = route.rpartition(":")
            pts = json.loads(Path(path).read_text())["groups"][group]
            wps = [Waypoint(q[0], q[1], q[2] if len(q) > 2 else None) for q in pts]
        else:
            wps = [Waypoint(*c.start)]
        loop = plan_loop(g, wps)
    except (RouteError, ValueError, KeyError, OSError) as e:
        raise SystemExit(f"{c.path.name} の周回を道路グラフから作れない: {e}") from e
    return loop.xy


class TruthDriver:
    """真値の姿勢で中心線を純追従する。**SLAM を制御ループから切り離すための運転手。**"""

    def __init__(self, course: Course, spec: VehicleSpec, *, a_lat: float = 2.0,
                 lookahead: float = 0.45) -> None:
        cl = np.asarray(course.centerline, dtype=np.float64)[:, :2]
        # 閉じた周回として扱う。重複した終点は落とす
        if np.hypot(*(cl[0] - cl[-1])) < 1e-6:
            cl = cl[:-1]
        sx, sy, syaw = course.start
        i0 = int(np.argmin(np.hypot(cl[:, 0] - sx, cl[:, 1] - sy)))
        fwd = cl[(i0 + 3) % len(cl)] - cl[i0]
        if math.cos(math.atan2(fwd[1], fwd[0]) - syaw) < 0:
            cl = cl[::-1].copy()
        self.xy = cl
        seg = np.hypot(*np.diff(np.vstack([cl, cl[:1]]), axis=0).T)
        self.length = float(seg.sum())
        self.step = self.length / len(cl)
        self.spec = spec
        self.a_lat = a_lat
        self.lookahead = lookahead
        self.kappa = _loop_curvature(cl)
        self._idx = int(np.argmin(np.hypot(cl[:, 0] - sx, cl[:, 1] - sy)))
        self.progress = 0.0            #: 走った弧長 [m]（中心線上）

    def command(self, x: float, y: float, yaw: float, v_max: float) -> tuple[float, float]:
        n = len(self.xy)
        # 近傍だけ探す（周回で反対側の区間に飛ばない）
        win = np.arange(self._idx - 5, self._idx + 40) % n
        d = np.hypot(self.xy[win, 0] - x, self.xy[win, 1] - y)
        j = int(win[int(np.argmin(d))])
        adv = (j - self._idx) % n
        if adv < n // 2:
            self.progress += adv * self.step
            self._idx = j
        la = self.lookahead + 0.3 * v_max
        k = int(round(la / self.step))
        tx, ty = self.xy[(self._idx + k) % n]
        c, s = math.cos(yaw), math.sin(yaw)
        lx = c * (tx - x) + s * (ty - y)
        ly = -s * (tx - x) + c * (ty - y)
        ld2 = max(1e-6, lx * lx + ly * ly)
        steer = math.atan(2.0 * self.spec.wheelbase * ly / ld2)
        steer = max(-self.spec.max_steer, min(self.spec.max_steer, steer))
        # 先の曲率で速度を決める（摩擦円の手前で曲がれる速度）
        ahead = np.arange(self._idx, self._idx + int(1.5 / self.step) + 1) % n
        kap = float(np.max(np.abs(self.kappa[ahead])))
        v = min(v_max, math.sqrt(self.a_lat / max(kap, 1e-6)))
        return max(0.25, v), steer


def _loop_curvature(xy: np.ndarray) -> np.ndarray:
    p0 = np.roll(xy, 2, axis=0)
    p1 = xy
    p2 = np.roll(xy, -2, axis=0)
    a = np.hypot(*(p1 - p0).T)
    b = np.hypot(*(p2 - p1).T)
    c = np.hypot(*(p2 - p0).T)
    cross = (p1[:, 0] - p0[:, 0]) * (p2[:, 1] - p0[:, 1]) - (p1[:, 1] - p0[:, 1]) * (p2[:, 0] - p0[:, 0])
    k = 2.0 * cross / np.maximum(a * b * c, 1e-9)
    # 手描きコースの尖りで速度が0に張り付かないよう、少しだけ均す
    return np.convolve(np.pad(k, 3, mode="wrap"), np.ones(7) / 7, mode="valid")


# ── 被験者（SLAM）──────────────────────────────────────────────────────────

class SlamUnderTest:
    """ベンチから見た SLAM の面。**実機と同じ `Slam2dNav` を包む**のが既定。

    `update()` は `Scan` と `VehicleState` を受け取り、姿勢・一致度・見失い・
    その姿勢の基準時刻 `t_ref_ns` を返す。
    """

    def __init__(self, factory: Callable[[], object]) -> None:
        self.nav = factory()

    def update(self, scan, vs: VehicleState, dt: float):
        u = self.nav.update(scan, dt, yaw_rate=vs.yaw_rate, speed=vs.speed)
        return u.pose, u.score, u.lost, self.t_ref_ns()

    def on_vehicle_state(self, vs: VehicleState) -> None:
        fn = getattr(self.nav, "on_vehicle_state", None)
        if fn is not None:
            fn(vs)

    def t_ref_ns(self) -> int:
        fe = getattr(self.nav, "_fe", None)
        return int(getattr(fe, "_t_ref", 0)) if fe is not None else 0

    @property
    def grid(self):
        return self.nav.grid

    def freeze(self) -> None:
        self.nav.freeze()

    def relocs(self) -> int:
        fe = getattr(self.nav, "_fe", None)
        return int(getattr(fe, "relocs", 0)) if fe is not None else 0

    def loop_closures(self) -> int:
        return int(getattr(self.nav, "loop_closures", 0))


def default_factory(**kw) -> Callable[[], object]:
    from raspi.auto._slam2d_nav import Slam2dNav
    from raspi.auto.slam2d_raceline import MAP_MAX_RANGE, MAP_RES, MAP_SIZE_M
    spec = VehicleSpec.load()
    lx, ly, _ = spec.sensor_pose("lidar")
    args = dict(resolution=MAP_RES, size_m=MAP_SIZE_M, lidar_x=lx, lidar_y=ly,
                max_range=MAP_MAX_RANGE)
    args.update(kw)
    return lambda: Slam2dNav(**args)


# ── 1回の走行 ─────────────────────────────────────────────────────────────

@dataclass
class RunConfig:
    course: str = "normal"
    #: 中心線の代わりに走る経路（`<routes.json>:<グループ>`、`load_course` 参照）
    route: str = ""
    seed: int = 0
    errors: str = "real"
    laps: int = 2                  #: 地図作成の周回数
    explore_speed: float = 0.6
    race_laps: int = 2
    race_speed: float = 2.0
    a_lat: float = 2.0
    #: 走り出す前に止まっている時間[s]。**実機と同じにするために要る**——
    #: ARM→engage の操作の間ずっと止まっており、SLAM はその間にジャイロの
    #: ゼロ点ずれを較正する（`slam2d/core/motion.py`の ZUPT）
    still_s: float = 3.0
    #: `SensorModel.ghost`（プリセットに足す）
    ghost: tuple[tuple[float, float, float, float, float, float], ...] = ()
    #: `SensorModel`の上書き（プリセットの後に`dataclasses.replace`で当てる）。
    #: 例 `{"speed_quant": 0.05, "time_offset_ms": 100.0}`
    sensor_overrides: tuple[tuple[str, float], ...] = ()
    #: 凍結地図を保存地図と同じ経路（3値→`occgrid_from_trinary`）で読み直し、RACE 中の
    #: 等間隔の`locate`か所で全域探索（`GlobalLocalizer`→仕上げ→`locate_score`）して、
    #: 真値と比べる回数。0 なら通らない。車は動いている（実機の LOCATE は止まっているので厳しめ）
    locate: int = 0
    #: 走行中に居る他車（小さな障害物）。`(段, 弧長s0[m], 横ずれ[m], 速度[m/s])`の組。段は
    #: 0=地図作成・1=走行・2=両方。速度が0以外なら中心線に沿って動く（負は逆走）
    cars: tuple[tuple[float, float, float, float], ...] = ()
    #: `locate`の各試行で、自車のまわりに置く他車の数（前後・左右に振り分ける）
    locate_cars: int = 0
    #: LiDAR の視野 [deg]。車両の正面を中心にこの幅だけ残し、外は「測れなかった」（dist=0）にする。
    #: 360 で全周。車体の後ろに積んだ箱が後方を遮る状況（MAGP）の再現
    fov_deg: float = 360.0
    #: 車体に立てた支柱 `(車両角[deg], LiDARからの距離[m], 直径[m])`。LiDAR から見て車と一緒に動く
    pillars: tuple[tuple[float, float, float], ...] = ()
    #: 支柱の縁の1本ずつが「支柱と背景の間の距離」になる確率（mixed pixel。0 で無し）
    mixed_pixel: float = 0.0


@dataclass
class PhaseStats:
    pos_err: list[float] = field(default_factory=list)   #: [m]
    lat_err: list[float] = field(default_factory=list)   #: 真値の車体座標で横方向の誤差 [m]
    fwd_err: list[float] = field(default_factory=list)   #: 同 進行方向 [m]
    yaw_err: list[float] = field(default_factory=list)   #: [deg]
    update_ms: list[float] = field(default_factory=list)
    #: 壁の向こうとして外した点の割合（`FrontendConfig.behind_margin`）
    behind: list[float] = field(default_factory=list)
    #: 越境の外乱を入れた周期の数と、その間の位置誤差の最大 [m]
    ghost_updates: int = 0
    ghost_max_err: float = 0.0
    lost: int = 0
    updates: int = 0
    max_lost_streak: int = 0
    collisions: int = 0

    def summary(self) -> dict:
        def pct(v, q):
            return float(np.percentile(v, q)) if v else float("nan")
        pe = np.asarray(self.pos_err)
        return dict(
            ate_cm=float(np.sqrt(np.mean(pe ** 2))) * 100 if pe.size else float("nan"),
            lat_rms_cm=float(np.sqrt(np.mean(np.square(self.lat_err)))) * 100 if self.lat_err else float("nan"),
            lat_p99_cm=float(np.percentile(np.abs(self.lat_err), 99)) * 100 if self.lat_err else float("nan"),
            fwd_rms_cm=float(np.sqrt(np.mean(np.square(self.fwd_err)))) * 100 if self.fwd_err else float("nan"),
            max_cm=float(pe.max()) * 100 if pe.size else float("nan"),
            final_cm=float(pe[-1]) * 100 if pe.size else float("nan"),
            yaw_rms_deg=float(np.sqrt(np.mean(np.square(self.yaw_err)))) if self.yaw_err else float("nan"),
            yaw_max_deg=max(self.yaw_err) if self.yaw_err else float("nan"),
            lost_pct=100.0 * self.lost / max(1, self.updates),
            max_lost_streak=self.max_lost_streak,
            ms_mean=statistics.fmean(self.update_ms) if self.update_ms else float("nan"),
            ms_p99=pct(self.update_ms, 99),
            ms_max=max(self.update_ms) if self.update_ms else float("nan"),
            collisions=self.collisions,
            behind_max=max(self.behind) if self.behind else 0.0,
            ghost_n=self.ghost_updates,
            ghost_max_cm=self.ghost_max_err * 100,
        )


@dataclass
class RunResult:
    cfg: RunConfig
    explore: PhaseStats
    race: PhaseStats
    map_before: dict
    map_after: dict
    freeze_ms: float
    relocs: int
    loop_closures: int
    out_of_bounds: int
    traj_true: np.ndarray          #: (N, 3) SLAM 原点基準の真値（EXPLORE+RACE）
    traj_est: np.ndarray
    phase_split: int               #: traj の何点目から RACE か
    wall_time_s: float
    map_walls: np.ndarray | None = None   #: 凍結地図の壁セル中心（重ね合わせ後の世界座標）
    course_obj: Course | None = None
    #: 保存地図＋LOCATE の結果（`RunConfig.locate`）: n, ok（10cm・3°以内で採用）,
    #: ambiguous（決めずに断った）, wrong（誤った姿勢を採用）, none（候補なし・足りない）, err_cm（採用した姿勢の誤差）
    locate: dict = field(default_factory=dict)

    def row(self) -> dict:
        e = self.explore.summary()
        r = self.race.summary()
        return dict(
            course=self.cfg.course, seed=self.cfg.seed, errors=self.cfg.errors,
            ex_ate=e["ate_cm"], ex_max=e["max_cm"], ex_yaw=e["yaw_rms_deg"],
            ex_final=e["final_cm"],
            ex_lost=e["lost_pct"],
            map_prec_before=self.map_before.get("precision", float("nan")),
            map_prec_after=self.map_after.get("precision", float("nan")),
            map_rec_after=self.map_after.get("recall", float("nan")),
            map_blur_after=self.map_after.get("blur_cm", float("nan")),
            map_rigid_deg=self.map_after.get("rigid_deg", float("nan")),
            freeze_ms=self.freeze_ms,
            rc_ate=r["ate_cm"], rc_max=r["max_cm"], rc_yaw=r["yaw_rms_deg"],
            rc_lat=r["lat_rms_cm"], rc_lat99=r["lat_p99_cm"], rc_fwd=r["fwd_rms_cm"],
            rc_lost=r["lost_pct"], rc_streak=r["max_lost_streak"],
            rc_behind=r["behind_max"], ghost_n=r["ghost_n"],
            ghost_max=r["ghost_max_cm"],
            ms_mean=statistics.fmean(self.explore.update_ms + self.race.update_ms)
            if (self.explore.update_ms or self.race.update_ms) else float("nan"),
            ms_max=max(self.explore.update_ms + self.race.update_ms, default=float("nan")),
            relocs=self.relocs, loops=self.loop_closures, oob=self.out_of_bounds,
            loc_n=self.locate.get("n", 0), loc_ok=self.locate.get("ok", 0),
            loc_amb=self.locate.get("ambiguous", 0), loc_wrong=self.locate.get("wrong", 0), loc_none=self.locate.get("none", 0),
            loc_ms=statistics.median(self.locate["ms"]) if self.locate.get("ms") else float("nan"),
        )


class _TruthLog:
    """真値の時系列。**点群の基準時刻へ補間して**取り出すために持つ。"""

    def __init__(self) -> None:
        self.t: list[int] = []
        self.x: list[float] = []
        self.y: list[float] = []
        self.c: list[float] = []
        self.s: list[float] = []

    def add(self, t_ns: int, x: float, y: float, yaw: float) -> None:
        self.t.append(t_ns)
        self.x.append(x)
        self.y.append(y)
        self.c.append(math.cos(yaw))
        self.s.append(math.sin(yaw))

    def at(self, t_ns: int) -> tuple[float, float, float]:
        t = np.asarray(self.t)
        x = float(np.interp(t_ns, t, self.x))
        y = float(np.interp(t_ns, t, self.y))
        yaw = math.atan2(float(np.interp(t_ns, t, self.s)), float(np.interp(t_ns, t, self.c)))
        return x, y, yaw


def run(cfg: RunConfig, factory: Callable[[], object] | None = None, *,
        verbose: bool = False) -> RunResult:
    return _Run(cfg, factory, verbose).execute()


class _Run:
    """`run` の 1 回の走行。段（真値の運転 → 計測 → LiDAR → 外乱 → SLAM → 採点 → 段の切り替え）を
    メソッドに分けるため、ループの状態をここに持つ（分ける前は 1 関数の局所変数だった）。"""

    def __init__(self, cfg: RunConfig, factory: Callable[[], object] | None,
                 verbose: bool) -> None:
        self.t_wall = time.perf_counter()
        self.cfg = cfg
        self.factory = factory
        self.verbose = verbose
        course = self.course = load_course(cfg.course, cfg.route)
        sensors = SensorModel.preset(cfg.errors, course)
        if cfg.ghost:
            sensors = dataclasses.replace(sensors, ghost=sensors.ghost + tuple(cfg.ghost))
        if cfg.sensor_overrides:
            sensors = dataclasses.replace(sensors, **dict(cfg.sensor_overrides))
        self.sensors = sensors
        spec = VehicleSpec.load()
        self.lidar_x, self.lidar_y, _ = spec.sensor_pose("lidar")
        self.ghost_box = _wall_bbox(course)
        self.ghost_t0: list[int | None] = [None] * len(sensors.ghost)
        self.veh = VehicleModel(spec, course.start)
        self.lidar = VirtualLidar(course, spec, sensors.lidar, seed=cfg.seed)
        self.asm = ScanAssembler()
        self.rng = np.random.default_rng(cfg.seed + 12345)
        self.driver = TruthDriver(course, spec, a_lat=cfg.a_lat)
        self.slam = SlamUnderTest(factory or default_factory())
        self.body = course.body_samples(spec.footprint)
        self.body_rng = body_ranges(spec.footprint, self.lidar_x, self.lidar_y)

        self.truth = _TruthLog()
        self.t_ns = 0
        self.queue: list[tuple[int, object]] = []
        self.speed_meas = 0.0
        self.vs: VehicleState | None = None
        self.next_tlm = 0
        self.bias = math.radians(sensors.gyro_bias_dps)
        self.gsig = math.radians(sensors.gyro_noise_dps)

        self.explore, self.race = PhaseStats(), PhaseStats()
        self.stats = self.explore
        self.phase = "explore"
        self.v_max = cfg.explore_speed
        self.explore_len = cfg.laps * self.driver.length
        self.race_len = cfg.race_laps * self.driver.length
        self.origin = None
        self.traj_true: list[tuple[float, float, float]] = []
        self.traj_est: list[tuple[float, float, float]] = []
        self.phase_split = 0
        self.map_before: dict = {}
        self.map_after: dict = {}
        self.freeze_ms = 0.0
        self.locate: dict = {}
        self.loc_done = 0
        self.lost_streak = 0
        self.was_colliding = False
        self.last_plan_ns = 0
        self.steer_cmd = 0.0
        self.speed_cmd = 0.0
        self.race_start_progress = 0.0
        self.max_t = (self.explore_len / 0.3 + self.race_len / 0.5) + 60.0

    def execute(self) -> RunResult:
        while self.t_ns < self.max_t * NS:
            self._drive()
            self._measure()
            scan = self._collect_scan()
            if scan is None or self.vs is None:
                continue
            scan, ghosted = self._inject(scan)
            tr = self._slam_step(scan, ghosted)
            if self.phase == "explore" and self.driver.progress >= self.explore_len:
                self._end_explore()
            elif self.phase == "race":
                if self._race_step(scan, tr):
                    break
        return self._result()

    # ── 段ごと ──

    def _drive(self) -> None:
        """真値の運転手（100Hz 相当で指令を更新）と車の積分・衝突。"""
        veh, cfg = self.veh, self.cfg
        if self.t_ns < cfg.still_s * NS:
            self.speed_cmd, self.steer_cmd = 0.0, 0.0
        elif (self.t_ns // 1_000_000) % 10 == 0:
            self.speed_cmd, self.steer_cmd = self.driver.command(veh.x, veh.y, veh.yaw, self.v_max)
        veh.apply(DriveInput(armed=True, target_speed=self.speed_cmd, target_steer=self.steer_cmd))
        veh.step(DT)
        self.t_ns += int(DT * NS)
        self.truth.add(self.t_ns, veh.x, veh.y, veh.yaw)
        hit = self.course.collides(veh.x, veh.y, veh.yaw, self.body)
        if hit and not self.was_colliding:
            self.stats.collisions += 1
        self.was_colliding = hit

    def _measure(self) -> None:
        """車速・ヨーレートの計測（誤差を注入）。"""
        veh, sensors, t_ns = self.veh, self.sensors, self.t_ns
        scale = sensors.speed_scale
        for (x0, y0, x1, y1, k) in sensors.slip_zones:
            if x0 <= veh.x <= x1 and y0 <= veh.y <= y1:
                scale *= k
        a = 1.0 if sensors.speed_tau <= 0 else 1.0 - math.exp(-DT / sensors.speed_tau)
        self.speed_meas += a * (veh.speed * scale - self.speed_meas)
        if t_ns >= self.next_tlm:
            self.next_tlm += NS // TELEMETRY_HZ
            rng, gsig = self.rng, self.gsig
            sp = self.speed_meas + (float(rng.normal(0, sensors.speed_noise)) if sensors.speed_noise > 0 else 0.0)
            if sensors.speed_quant > 0.0:
                sp = round(sp / sensors.speed_quant) * sensors.speed_quant
            yr = (veh.yaw_rate * sensors.gyro_scale + self.bias
                  + (float(rng.normal(0, gsig)) if gsig > 0 else 0.0))
            self.vs = VehicleState(speed=sp, yaw_rate=yr, steer_actual=veh.steer_actual,
                                   t_capture=t_ns + int(sensors.time_offset_ms * 1e6), stopped=abs(sp) < SPEED_DEADBAND_MPS)
            self.slam.on_vehicle_state(self.vs)

    def _collect_scan(self):
        """LiDAR（セクタ単位・伝送遅延つき）→ ScanAssembler。組み上がった点群か None。"""
        t_ns = self.t_ns
        for gen_ns, pkt in self.lidar.poll(t_ns, self.veh, lambda t: (t // 1000) & 0xFFFFFFFF):
            self.queue.append((gen_ns, pkt))
        scan = None
        if self.queue:
            keep = []
            for gen_ns, pkt in self.queue:
                if gen_ns <= t_ns:
                    # セクタ先頭の Pi 時刻。時刻同期が収束している前提で、STM32 時刻をそのまま戻す
                    done = self.asm.feed(pkt, int(pkt.t_start_us) * 1000)
                    if done is not None:
                        scan = done
                else:
                    keep.append((gen_ns, pkt))
            self.queue = keep
        return scan

    def _lidar_pose(self) -> tuple[float, float, float]:
        veh = self.veh
        c, s = math.cos(veh.yaw), math.sin(veh.yaw)
        return (veh.x + c * self.lidar_x - s * self.lidar_y,
                veh.y + s * self.lidar_x + c * self.lidar_y, veh.yaw)

    def _inject(self, scan):
        """点群へ外乱（越境・他車・視野・支柱）を入れ、車体の内側を捨てる。`(点群, 越境を入れたか)`。"""
        cfg, sensors, phase, t_ns = self.cfg, self.sensors, self.phase, self.t_ns
        driver = self.driver
        ghosted = False
        for k, (ph, f0, dur, deg0, deg1, off) in enumerate(sensors.ghost):
            if int(ph) != (1 if phase == "race" else 0):
                continue
            if phase == "race":
                done_m, total = driver.progress - self.race_start_progress, self.race_len
            else:
                done_m, total = driver.progress, self.explore_len
            if self.ghost_t0[k] is None and done_m >= f0 * total:
                self.ghost_t0[k] = t_ns
            t_start = self.ghost_t0[k]
            if t_start is None or t_ns - t_start > dur * NS:
                continue
            gb = self.ghost_box
            box = (gb[0] - off, gb[1] - off, gb[2] + off, gb[3] + off)
            inject_ghost(scan, self._lidar_pose(), deg0, deg1, box, sensors.lidar.lidar_max_range_m)
            ghosted = phase == "race"

        if cfg.cars:
            ph = 1.0 if phase == "race" else 0.0
            boxes = [car_box(driver, s0 + v * t_ns / NS, lat, v < 0)
                     for (cp, s0, lat, v) in cfg.cars if cp == 2.0 or cp == ph]
            inject_boxes(scan, self._lidar_pose(), boxes, sensors.lidar.lidar_max_range_m)

        mask_fov(scan, cfg.fov_deg)
        inject_pillars(scan, cfg.pillars, cfg.mixed_pixel, self.rng)
        # 実機の planning_node と同じく、車体の外形の内側の点を捨ててから SLAM に渡す
        scan = mask_body(scan, self.body_rng)
        return scan, ghosted

    def _slam_step(self, scan, ghosted: bool) -> tuple[float, float, float]:
        """SLAM を 1 回進め、真値（SLAM 原点基準）と比べて記録する。真値の姿勢を返す。"""
        stats, veh = self.stats, self.veh
        t_ns = self.t_ns
        dt = (t_ns - self.last_plan_ns) / NS if self.last_plan_ns else 0.1
        self.last_plan_ns = t_ns
        t0 = time.perf_counter()
        pose, score, lost, t_ref = self.slam.update(scan, self.vs, dt)
        stats.update_ms.append((time.perf_counter() - t0) * 1000.0)
        stats.updates += 1
        fe = getattr(self.slam.nav, "_fe", None)
        if fe is not None:
            stats.behind.append(float(getattr(fe, "behind_ratio", 0.0)))
        if lost:
            stats.lost += 1
            self.lost_streak += 1
            stats.max_lost_streak = max(stats.max_lost_streak, self.lost_streak)
        else:
            self.lost_streak = 0
        tp = self.truth.at(t_ref) if t_ref > 0 else (veh.x, veh.y, veh.yaw)
        if self.origin is None:
            # SLAM の原点 = 最初の点群の基準時刻の姿勢。**初回の推定値そのものを
            # 真値に重ねる**（初回に推測航法を少し進める実装でも原点がずれない
            # ように。ずれは「世界座標で一定のオフセット」として全周期に乗り、
            # 進行方向が変わるたびに誤差の見かけを変えてしまう）
            self.origin = tuple(compose(Pose2D(*tp), inverse(Pose2D(*pose))))
        tr = tuple(between(Pose2D(*self.origin), Pose2D(*tp)))
        stats.pos_err.append(math.hypot(pose[0] - tr[0], pose[1] - tr[1]))
        _c, _s = math.cos(tr[2]), math.sin(tr[2])
        _dx, _dy = pose[0] - tr[0], pose[1] - tr[1]
        stats.fwd_err.append(_c * _dx + _s * _dy)
        stats.lat_err.append(-_s * _dx + _c * _dy)
        stats.yaw_err.append(abs(math.degrees(wrap_angle(pose[2] - tr[2]))))
        if ghosted:
            stats.ghost_updates += 1
            stats.ghost_max_err = max(stats.ghost_max_err, stats.pos_err[-1])
        self.traj_true.append(tr)
        self.traj_est.append((pose[0], pose[1], pose[2]))

        if self.verbose and stats.updates % 50 == 0:
            print(f"  [{self.phase}] {self.driver.progress:6.1f}m err {stats.pos_err[-1] * 100:5.1f}cm "
                  f"{stats.yaw_err[-1]:4.1f}deg score {score:.2f}{' LOST' if lost else ''}",
                  flush=True)
        return tr

    def _end_explore(self) -> None:
        """地図作成の終わり: 凍結の前後で地図を採点し、凍結地図での走行へ移る。"""
        slam, course, origin = self.slam, self.course, self.origin
        # ★ 採点の条件を揃える: 前後どちらも真のコースへ剛体で重ねてから採点する
        #   （前だけ重ねないと、地図全体が少し回っているだけで before が悪く見える）
        aligned_before = align_map(slam.grid, course, origin)
        self.map_before = map_quality(slam.grid, course, aligned_before)
        t0 = time.perf_counter()
        slam.freeze()
        self.freeze_ms = (time.perf_counter() - t0) * 1000.0
        aligned = align_map(slam.grid, course, origin)
        map_after = self.map_after = map_quality(slam.grid, course, aligned)
        map_after["rigid_deg"] = abs(math.degrees(wrap_angle(aligned[2] - origin[2])))
        map_after["rigid_cm"] = math.hypot(aligned[0] - origin[0], aligned[1] - origin[1]) * 100
        # ここから先（凍結地図での走行）は、地図を真のコースへ重ねた姿勢を原点にする
        self.origin = aligned
        self.phase = "race"
        self.stats = self.race
        self.v_max = self.cfg.race_speed
        self.phase_split = len(self.traj_true)
        self.race_start_progress = self.driver.progress
        self.lost_streak = 0

    def _race_step(self, scan, tr) -> bool:
        """凍結地図での走行中: 決めた所で LOCATE を試す。走り終えたら True。"""
        cfg = self.cfg
        done_m = self.driver.progress - self.race_start_progress
        if cfg.locate > 0 and self.loc_done < cfg.locate and \
                done_m >= self.race_len * (self.loc_done + 0.5) / cfg.locate:
            self.loc_done += 1
            tscan = scan
            if cfg.locate_cars > 0:
                tscan = copy.deepcopy(scan)
                inject_boxes(tscan, self._lidar_pose(),
                             _trial_cars(self.driver, cfg.locate_cars, self.loc_done),
                             self.sensors.lidar.lidar_max_range_m)
            _locate_trial(self.slam, tscan, tr, self.factory or default_factory(), self.locate,
                          force_rescan=tscan is not scan)
        return done_m >= self.race_len

    def _result(self) -> RunResult:
        slam = self.slam
        return RunResult(
            cfg=self.cfg, explore=self.explore, race=self.race, map_before=self.map_before,
            map_after=self.map_after,
            freeze_ms=self.freeze_ms, relocs=slam.relocs(), loop_closures=slam.loop_closures(),
            out_of_bounds=int(getattr(slam.grid, "out_of_bounds", 0)),
            traj_true=np.asarray(self.traj_true), traj_est=np.asarray(self.traj_est),
            phase_split=self.phase_split, wall_time_s=time.perf_counter() - self.t_wall,
            map_walls=_walls_world(slam.grid, self.origin) if self.origin is not None else None,
            course_obj=self.course, locate=self.locate)


def _trial_cars(driver: "TruthDriver", n: int, k: int) -> list[tuple[float, float, float, float, float]]:
    """`locate`の`k`回目の試行で自車のまわりに置く他車`n`台。自車の中心線上の位置から前後に振り分け、
    左右の横ずれは交互にする（自車の真上には置かない）。"""
    s_ego = driver.progress
    slots = [(0.9, 0.30), (-0.9, -0.30), (1.8, -0.30), (-1.8, 0.30), (2.7, 0.30), (-2.7, -0.30)]
    out = []
    for i in range(n):
        ds, lat = slots[(i + k) % len(slots)]
        out.append(car_box(driver, s_ego + ds, lat, backward=(i + k) % 3 == 0))
    return out


def _locate_trial(slam: "SlamUnderTest", scan, truth_pose, factory, out: dict,
                  force_rescan: bool = False) -> None:
    """保存地図を読み直した状態から、今の点群で全域探索（LOCATE）して真値と比べる。

    実機の流れと同じ部品を通す: 地図は3値に落として`occgrid_from_trinary`で読み直し、候補は
    `GlobalLocalizer`→`refine`→`locate_score`、採否は`LOCATE_SCORE_RATIO`（2位が1位に近ければ断る）。
    """
    from raspi.auto._slam2d_nav import occgrid_from_trinary
    from raspi.auto.slam2d_raceline import (LOCATE_CANDIDATES, locate_score, locate_score_robust,
                                            pick_locate)
    from slam2d.core.localize import GlobalLocalizer

    g = slam.grid
    loaded = occgrid_from_trinary(g.trinary(), resolution=g.resolution, origin=tuple(g.origin),
                                  seq=g.seq + 1)
    nav2 = factory()
    nav2.replace_grid_for_race(loaded)
    out["n"] = out.get("n", 0) + 1
    # ★ `deskew_scan`は直前の`update()`と同じ時刻の走査なら結果を使い回す。他車を差し込んだ走査を
    #   渡しても無視されるので、使い回しを外す
    fe = getattr(slam.nav, "_fe", None)
    if fe is not None and force_rescan:
        fe.last_points = None
    pts = slam.nav.deskew_scan(scan)
    import time as _time
    t0 = _time.perf_counter()
    # `LOC_ENGINE=grid` で旧来の総当たり（`GlobalLocalizer`）。既定は分枝限定法（`globalmatch`。C が無ければ旧来）
    from slam2d.core import globalmatch
    use_bb = os.environ.get("LOC_ENGINE", "bb") != "grid" and globalmatch.available()
    if use_bb:
        cands = globalmatch.GlobalMatcher(loaded).match(pts, LOCATE_CANDIDATES)
        win = {"trans": globalmatch.REFINE_TRANS, "rot": globalmatch.REFINE_ROT}
    else:
        loc = GlobalLocalizer(loaded)
        for _ in range(10_000):
            if loc.step(pts):
                break
        cands = loc.candidates(LOCATE_CANDIDATES)
        win = {}
    from slam2d.core.locscore import LocateScorer
    scorer = LocateScorer(loaded)
    scored = []
    for c in cands:
        ref = nav2.refine(pts, Pose2D(c.x, c.y, c.yaw), **win)
        if ref is not None:
            scored.append((locate_score(ref), locate_score_robust(scorer.fit(pts, ref.pose, nav2.lidar_xy)), ref))
    if os.environ.get("LOC_DEBUG"):
        print("   LOC", " ".join(sorted(f"{a:.2f}/{b:.2f}(e{math.hypot(r.pose.x - truth_pose[0], r.pose.y - truth_pose[1])*100:.0f})"
                                         for a, b, r in scored)[::-1]), flush=True)
    best, why = pick_locate(scored)
    out.setdefault("ms", []).append((_time.perf_counter() - t0) * 1000.0)
    if best is None:
        key = {"ambiguous": "ambiguous"}.get(why, "none")
        out[key] = out.get(key, 0) + 1
        return
    pose = best.pose
    err = math.hypot(pose.x - truth_pose[0], pose.y - truth_pose[1])
    yaw = abs(math.degrees(wrap_angle(pose.yaw - truth_pose[2])))
    out.setdefault("err_cm", []).append(err * 100.0)
    key = "ok" if err <= 0.10 and yaw <= 3.0 else "wrong"
    out[key] = out.get(key, 0) + 1


def _walls_world(grid, origin) -> np.ndarray:
    rows, cols = np.nonzero(grid.wall_mask())
    mx, my = grid.to_world(cols, rows)
    ox, oy, oyaw = origin
    c, s = math.cos(oyaw), math.sin(oyaw)
    return np.stack([ox + mx * c - my * s, oy + mx * s + my * c], axis=1)


# ── 地図の採点 ────────────────────────────────────────────────────────────

def map_quality(grid, course: Course, origin) -> dict:
    """SLAM の地図を**真のコースと重ねて**採点する。

    - precision: 地図の壁セルのうち、真の壁から 5cm 以内にあるものの割合
    - blur_cm: 地図の壁セルから最寄りの真の壁までの距離の 90 パーセンタイル
      （壁が二重・太く焼けていると大きくなる）
    - recall: 真の壁のうち、SLAM が一度でも見た範囲（seen>0）にあるものに対して
      地図の壁が 5cm 以内に立っている割合
    """
    try:
        from scipy.ndimage import distance_transform_edt
    except ImportError:                                   # pragma: no cover
        return {}
    wall = grid.wall_mask()
    if not wall.any():
        return dict(precision=0.0, recall=0.0, blur_cm=float("nan"), cells=0)
    ox, oy, oyaw = origin
    co, so = math.cos(oyaw), math.sin(oyaw)

    # 真の壁までの距離場（コースの格子上）
    dist_true = distance_transform_edt(~course.grid) * course.resolution
    rows, cols = np.nonzero(wall)
    mx, my = grid.to_world(cols, rows)
    wx = ox + mx * co - my * so
    wy = oy + mx * so + my * co
    c = ((wx - course.origin[0]) / course.resolution).astype(np.int64)
    r = ((wy - course.origin[1]) / course.resolution).astype(np.int64)
    ok = (c >= 0) & (c < course.grid.shape[1]) & (r >= 0) & (r < course.grid.shape[0])
    d = np.full(wx.size, 1.0)
    d[ok] = dist_true[r[ok], c[ok]]
    # 真の壁の「表面」は壁セル中心から半セル。地図側の量子化ぶんも見込む
    tol = 0.05
    precision = float((d <= tol).mean())
    blur = float(np.percentile(d, 90)) * 100

    # recall: 真の壁の表面セル（走行可能領域に接する壁）→ SLAM 格子へ
    surf = course.grid & ~_erode(course.grid)
    tr, tc = np.nonzero(surf)
    tx = course.origin[0] + (tc + 0.5) * course.resolution
    ty = course.origin[1] + (tr + 0.5) * course.resolution
    lx = (tx - ox) * co + (ty - oy) * so
    ly = -(tx - ox) * so + (ty - oy) * co
    gc, gr = grid.to_cell(lx, ly)
    inside = grid.inside(gc, gr)
    seen = np.zeros(lx.size, dtype=bool)
    seen[inside] = grid.seen[gr[inside], gc[inside]] > 0
    if not seen.any():
        recall = float("nan")
    else:
        dist_map = distance_transform_edt(~wall) * grid.resolution
        dm = dist_map[gr[inside & seen], gc[inside & seen]]
        recall = float((dm <= tol).mean())
    return dict(precision=precision, recall=recall, blur_cm=blur, cells=int(wall.sum()))


def align_map(grid, course: Course, origin) -> tuple[float, float, float]:
    """地図の壁を真の壁に**剛体で**最もよく重ねる「地図の原点の世界姿勢」を返す。

    ポーズグラフは最初のキーフレームを固定するので、ループ閉じの後の地図は
    全体として少し回っていることがある。planner は地図の中でしか動かないので
    **地図全体の回転・平行移動は走りに無関係**で、効くのは地図の歪みと、
    凍結地図に対する自己位置の誤差だけ。RACE 段の誤差はこの重ね合わせを
    基準に測る（そうしないと「地図が1°回っている」が位置誤差として数十cmに化ける）。
    """
    from scipy.ndimage import distance_transform_edt, map_coordinates
    from scipy.optimize import minimize
    wall = grid.wall_mask()
    if not wall.any():
        return origin
    rows, cols = np.nonzero(wall)
    if rows.size > 4000:
        take = np.linspace(0, rows.size - 1, 4000).astype(np.int64)
        rows, cols = rows[take], cols[take]
    mx, my = grid.to_world(cols, rows)
    dist_true = distance_transform_edt(~course.grid) * course.resolution

    def cost(v):
        ox, oy, oyaw = v
        c, s = math.cos(oyaw), math.sin(oyaw)
        wx = ox + mx * c - my * s
        wy = oy + mx * s + my * c
        fc = (wx - course.origin[0]) / course.resolution - 0.5
        fr = (wy - course.origin[1]) / course.resolution - 0.5
        d = map_coordinates(dist_true, [fr, fc], order=1, mode="nearest")
        return float(np.mean(np.minimum(d, 0.15)))

    best = minimize(cost, np.array(origin, dtype=np.float64), method="Nelder-Mead",
                    options=dict(xatol=1e-4, fatol=1e-6, initial_simplex=np.array(origin)
                                 + np.array([[0, 0, 0], [0.05, 0, 0], [0, 0.05, 0],
                                             [0, 0, math.radians(2.0)]]),
                                 maxiter=600))
    return tuple(float(v) for v in best.x)


def _erode(m: np.ndarray) -> np.ndarray:
    """4近傍で1セル削る（格子の外は削る理由にしない＝cv2.erode の既定の境界）。"""
    import cv2

    return cv2.erode(m.astype(np.uint8), cv2.getStructuringElement(cv2.MORPH_CROSS, (3, 3))).astype(bool)


# ── 出力 ──────────────────────────────────────────────────────────────────

_COLS = [
    ("course", "{:>18s}"), ("seed", "{:>4d}"), ("errors", "{:>5s}"),
    ("ex_ate", "{:7.1f}"), ("ex_max", "{:7.1f}"), ("ex_final", "{:7.1f}"), ("ex_yaw", "{:6.2f}"), ("ex_lost", "{:6.1f}"),
    ("map_prec_before", "{:6.2f}"), ("map_prec_after", "{:6.2f}"), ("map_rec_after", "{:6.2f}"),
    ("map_blur_after", "{:6.1f}"), ("map_rigid_deg", "{:6.2f}"), ("freeze_ms", "{:7.0f}"),
    ("rc_ate", "{:7.1f}"), ("rc_max", "{:7.1f}"), ("rc_lat", "{:6.1f}"), ("rc_lat99", "{:6.1f}"),
    ("rc_fwd", "{:6.1f}"), ("rc_yaw", "{:6.2f}"), ("rc_lost", "{:6.1f}"),
    ("rc_streak", "{:5d}"), ("rc_behind", "{:6.2f}"),
    ("ghost_n", "{:5d}"), ("ghost_max", "{:7.1f}"),
    ("ms_mean", "{:6.1f}"), ("ms_max", "{:7.0f}"),
    ("relocs", "{:4d}"), ("loops", "{:4d}"), ("oob", "{:6d}"),
    ("loc_n", "{:5d}"), ("loc_ok", "{:6d}"), ("loc_amb", "{:7d}"), ("loc_wrong", "{:9d}"), ("loc_none", "{:8d}"),
    ("loc_ms", "{:6.0f}"),
]


def print_table(rows: list[dict]) -> None:
    def w(k):
        return 18 if k == "course" else max(6, len(k))
    print(" ".join(f"{k:>{w(k)}s}" for k, _ in _COLS))
    for r in rows:
        cells = []
        for k, fmt in _COLS:
            v = r[k]
            try:
                txt = fmt.format(v).strip()
            except (ValueError, TypeError):
                txt = str(v)
            cells.append(f"{txt:>{w(k)}s}")
        print(" ".join(cells))


def plot(res: RunResult, out: Path) -> None:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, axes = plt.subplots(1, 3, figsize=(20, 6))
    ax = axes[0]
    tt, te = res.traj_true, res.traj_est
    k = res.phase_split or len(tt)
    ax.plot(tt[:k, 0], tt[:k, 1], "k-", lw=0.8, label="truth")
    ax.plot(te[:k, 0], te[:k, 1], "b-", lw=0.8, label="est explore (online)")
    ax.set_aspect("equal")
    ax.legend(fontsize=8)
    ax.set_title(f"{res.cfg.course} seed={res.cfg.seed} {res.cfg.errors}: explore")
    ax = axes[1]
    c = res.course_obj
    if c is not None:
        rr, cc = np.nonzero(c.grid & ~_erode(c.grid))
        ax.plot(c.origin[0] + (cc + 0.5) * c.resolution, c.origin[1] + (rr + 0.5) * c.resolution,
                ",", color="0.6")
    if res.map_walls is not None and len(res.map_walls):
        ax.plot(res.map_walls[:, 0], res.map_walls[:, 1], ",", color="r")
    ax.set_aspect("equal")
    ax.set_title("frozen map (red) over true walls (grey), rigidly aligned")
    ax = axes[2]
    n = len(res.explore.pos_err)
    ax.plot(np.arange(n), np.asarray(res.explore.pos_err) * 100, "b-", lw=0.8, label="explore pos cm")
    ax.plot(np.arange(n, n + len(res.race.pos_err)), np.asarray(res.race.pos_err) * 100, "r-",
            lw=0.8, label="race pos cm")
    ax.plot(np.arange(n), res.explore.yaw_err, "b:", lw=0.8, label="explore yaw deg")
    ax.plot(np.arange(n, n + len(res.race.yaw_err)), res.race.yaw_err, "r:", lw=0.8,
            label="race yaw deg")
    ax.legend(fontsize=8)
    ax.set_xlabel("scan")
    plt.tight_layout()
    plt.savefig(out, dpi=100)
    plt.close(fig)


DEFAULT_COURSES = ["normal", "toyota", "course1", "course2", "course3",
                   "circuit_chicane_a", "circuit_chicane_b", "circuit_chicane_c"]


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--course", default=",".join(DEFAULT_COURSES))
    ap.add_argument("--seeds", type=int, default=1)
    ap.add_argument("--seed0", type=int, default=0)
    ap.add_argument("--errors", default="real", help="ideal|real|harsh（カンマ区切り可）")
    ap.add_argument("--laps", type=int, default=2)
    ap.add_argument("--explore-speed", type=float, default=0.6)
    ap.add_argument("--race-laps", type=int, default=2)
    ap.add_argument("--race-speed", type=float, default=2.0)
    ap.add_argument("--map-size", type=float, default=None, help="Slam2dNav の size_m を上書き")
    ap.add_argument("--no-loop", action="store_true", help="ループ閉じを切る（切り分け用）")
    ap.add_argument("--route", default="",
                    help="中心線の代わりに走る経路 `<routes.json>:<グループ>`（分岐のあるコース用。"
                         "中心線の無いコースで省略すると出発点からの最短の周回）")
    ap.add_argument("--ghost", action="append", default=[],
                    help="壁越しに別の部屋が見える区間 `段:開始:秒数:deg0:deg1:offset`（繰り返し可）。"
                         "段は explore|race。その段の進捗の割合`開始`から`秒数`のあいだ、"
                         "車両角 deg0〜deg1 の点をコースの外接矩形を offset[m] 広げた壁までの"
                         "距離に差し替える")
    ap.add_argument("--behind-margin", type=float, default=None,
                    help="FrontendConfig.behind_margin を上書き（0で壁の向こうの点の除去を切る）")
    ap.add_argument("--front-margin", type=float, default=None,
                    help="FrontendConfig.front_margin を上書き（0で地図の壁より手前の点の除去を切る）")
    ap.add_argument("--speed-quant", type=float, default=0.0,
                    help="車速の読みの量子化の刻み [m/s]（エンコーダの粗さ。例 0.05）")
    ap.add_argument("--time-offset-ms", type=float, default=0.0,
                    help="テレメトリの時刻の一定のずれ [ms]（時刻同期の残差。例 100）")
    ap.add_argument("--locate", type=int, default=0,
                    help="RACE 中にこの回数だけ、保存地図を読み直して全域探索（LOCATE）を試し、"
                         "真値と比べる（loc_ok / loc_amb=断った / loc_wrong=誤った姿勢を採用）")
    ap.add_argument("--car", action="append", default=[],
                    help="走行中に居る他車 `段:弧長s0:横ずれ:速度`（繰り返し可）。段は explore|race|both。"
                         "速度0なら止まっている・負は逆走。LiDARの走査線を遮る小さな箱として差し込む")
    ap.add_argument("--locate-cars", type=int, default=0,
                    help="--locate の各試行で自車のまわり（前後左右）に置く他車の台数")
    ap.add_argument("--fov", type=float, default=360.0,
                    help="LiDAR の視野 [deg]（車両の正面を中心に残す幅。180 で前方半周だけ）")
    ap.add_argument("--pillar", action="append", default=[],
                    help="車体に立てた支柱 `車両角deg:距離m:直径m`（繰り返し可。例 120:0.05:0.01）")
    ap.add_argument("--mixed-pixel", type=float, default=0.0,
                    help="支柱の縁のレイが支柱と背景の間の距離になる確率（0〜1）")
    ap.add_argument("--plot", action="store_true")
    ap.add_argument("--json", default=None, help="結果の行を JSON で保存")
    ap.add_argument("--verbose", action="store_true")
    args = ap.parse_args()

    kw = {}
    if args.map_size:
        kw["size_m"] = args.map_size
    if args.no_loop:
        kw["loop_closure"] = False
    fk = {}
    if args.behind_margin is not None:
        fk["behind_margin"] = args.behind_margin
    if args.front_margin is not None:
        fk["front_margin"] = args.front_margin
    if fk:
        kw["frontend"] = fk
    ghost = []
    for g in args.ghost:
        f = g.split(":")
        if len(f) != 6 or f[0] not in ("explore", "race"):
            ap.error("--ghost は 段(explore|race):開始:秒数:deg0:deg1:offset")
        ghost.append((1.0 if f[0] == "race" else 0.0, *(float(v) for v in f[1:])))
    ghost = tuple(ghost)
    cars = []
    for c_ in args.car:
        f = c_.split(":")
        if len(f) != 4 or f[0] not in ("explore", "race", "both"):
            ap.error("--car は 段(explore|race|both):弧長:横ずれ:速度")
        cars.append(({"explore": 0.0, "race": 1.0, "both": 2.0}[f[0]], *(float(v) for v in f[1:])))
    ov = []
    if args.speed_quant:
        ov.append(("speed_quant", args.speed_quant))
    if args.time_offset_ms:
        ov.append(("time_offset_ms", args.time_offset_ms))
    rows = []
    for err in args.errors.split(","):
        for name in args.course.split(","):
            for s in range(args.seed0, args.seed0 + args.seeds):
                cfg = RunConfig(course=name, seed=s, errors=err, laps=args.laps,
                                explore_speed=args.explore_speed, race_laps=args.race_laps,
                                race_speed=args.race_speed, route=args.route, ghost=ghost, locate=args.locate,
                                cars=tuple(cars), locate_cars=args.locate_cars,
                                sensor_overrides=tuple(ov), fov_deg=args.fov,
                                pillars=tuple(tuple(float(v) for v in q.split(":")) for q in args.pillar),
                                mixed_pixel=args.mixed_pixel)
                res = run(cfg, default_factory(**kw), verbose=args.verbose)
                row = res.row()
                row["wall_s"] = res.wall_time_s
                rows.append(row)
                print_table([row])
                if args.plot:
                    plot(res, Path(f"slam_bench_{name}_{err}_{s}.png"))
                sys.stdout.flush()
    if args.json:
        Path(args.json).write_text(json.dumps(rows, indent=1, ensure_ascii=False))
    print()
    print_table(rows)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
