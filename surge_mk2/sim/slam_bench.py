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
from raspi.msgs.convert import ScanAssembler  # noqa: E402
from raspi.msgs.types import VehicleState  # noqa: E402

from .course import Course, list_courses  # noqa: E402
from .lidar import VirtualLidar  # noqa: E402
from .params import SimParams  # noqa: E402
from .vehicle import DriveInput, VehicleModel, VehicleSpec  # noqa: E402

__all__ = ["SensorModel", "RunConfig", "run", "load_course"]

NS = 1_000_000_000
DT = 0.002                     #: 車両の積分刻み [s]
TELEMETRY_HZ = 50              #: `VehicleState` の更新周期（実機の TELEMETRY と同じ）
COURSE_DIR = Path(__file__).resolve().parent / "courses"


# ── センサ誤差 ─────────────────────────────────────────────────────────────

@dataclass(frozen=True)
class SensorModel:
    """推測航法を狂わせる誤差と、LiDAR の意地悪さ。

    ## 値の根拠（2026-09-23 の外部調査）

    - **MPU6050**: データシートのゼロ点誤差は±20°/s（較正前）、ノイズ密度
      0.005°/s/√Hz。Allan分散の実測報告でバイアス不安定性 0.0013〜0.0024°/s、
      ノイズ 約0.009°/s/√Hz。**起動時の静止較正が前提**で、残るのは温度ドリフト。
      感度（倍率）誤差はデータシートで±3%
    - **LD06**: 0.3m超で誤差±45mm・標準偏差10mm、角度誤差 最大2°、10Hz回転。
      黒い物体は近赤外を吸収するので欠測が増える（大会コースのトンネルは黒壁）
    - **車速**: 前輪アナログ絶対角エンコーダ。分解能が粗く強いローパス。
      低μ板の上ではスリップして読みが伸びる

    `real` は「静止較正はしたが温度ドリフトが残る」程度、`harsh` は
    「較正が甘く、黒壁で欠測が増え、低μ板で滑る」大会当日の上振れを想定する。
    """

    speed_scale: float = 1.0        #: 車速の倍率誤差（車輪半径の誤差・摩耗）
    speed_noise: float = 0.0        #: 車速の白色ノイズ σ [m/s]
    speed_tau: float = 0.0          #: 車速の計測遅れ（1次遅れ）[s]。強いローパスの再現
    gyro_bias_dps: float = 0.0      #: ジャイロのゼロ点ずれ [°/s]
    gyro_noise_dps: float = 0.0     #: ジャイロの白色ノイズ σ [°/s]
    gyro_scale: float = 1.0         #: ジャイロの倍率誤差
    #: スリップ区間。(x0, y0, x1, y1, 倍率) の矩形の中では車速の読みがこの倍率に
    #: なる（低μ板で後輪が空転・前輪が滑る状況の近似）。世界座標
    slip_zones: tuple[tuple[float, float, float, float, float], ...] = ()
    lidar: SimParams = field(default_factory=SimParams)

    @classmethod
    def preset(cls, name: str, course: Course | None = None) -> "SensorModel":
        if name == "ideal":
            lid = SimParams(lidar_noise_sigma_m=0.0, lidar_noise_rel=0.0,
                            lidar_drop_rate=0.0, lidar_sector_drop_rate=0.0,
                            lidar_delay_jitter_ms=0.0)
            return cls(lidar=lid)
        if name == "real":
            # 静止較正済みのジャイロ（残るのは温度ドリフト）＋ LD06 の実力値
            return cls(speed_scale=1.02, speed_noise=0.02, speed_tau=0.05,
                       gyro_bias_dps=0.5, gyro_noise_dps=0.3, gyro_scale=1.01)
        if name == "harsh":
            # 測距ノイズはデータシート（σ10mm・±45mm）の倍程度に留め、
            # **欠測・スリップ・ジャイロの粗さ**で意地悪をする
            lid = SimParams(lidar_noise_sigma_m=0.015, lidar_noise_rel=0.005,
                            lidar_drop_rate=0.10, lidar_sector_drop_rate=0.03,
                            lidar_delay_jitter_ms=5.0)
            zones: tuple = ()
            if course is not None and course.centerline is not None:
                # 中心線の 1/3 付近に 1.2m 四方の低μ区間を置く（読みが 15% 伸びる）
                cx, cy = course.centerline[len(course.centerline) // 3, :2]
                zones = ((cx - 0.6, cy - 0.6, cx + 0.6, cy + 0.6, 1.15),)
            return cls(speed_scale=0.95, speed_noise=0.05, speed_tau=0.08,
                       gyro_bias_dps=1.5, gyro_noise_dps=1.0, gyro_scale=1.03,
                       slip_zones=zones, lidar=lid)
        raise ValueError(f"unknown preset: {name}")


# ── コースと真値の運転手 ───────────────────────────────────────────────────

def load_course(name_or_path: str) -> Course:
    """コースを読み、PNG コースなら隣の `<name>_centerline.npz` から中心線を補う。"""
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
    if c.centerline is None:
        raise SystemExit(f"{p.name} は中心線を持たない（真値の運転手が走れない）")
    return c


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


@dataclass
class PhaseStats:
    pos_err: list[float] = field(default_factory=list)   #: [m]
    lat_err: list[float] = field(default_factory=list)   #: 真値の車体座標で横方向の誤差 [m]
    fwd_err: list[float] = field(default_factory=list)   #: 同 進行方向 [m]
    yaw_err: list[float] = field(default_factory=list)   #: [deg]
    update_ms: list[float] = field(default_factory=list)
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
            ms_mean=statistics.fmean(self.explore.update_ms + self.race.update_ms)
            if (self.explore.update_ms or self.race.update_ms) else float("nan"),
            ms_max=max(self.explore.update_ms + self.race.update_ms, default=float("nan")),
            relocs=self.relocs, loops=self.loop_closures, oob=self.out_of_bounds,
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


def _rel(origin, pose) -> tuple[float, float, float]:
    ox, oy, oyaw = origin
    c, s = math.cos(-oyaw), math.sin(-oyaw)
    dx, dy = pose[0] - ox, pose[1] - oy
    return (dx * c - dy * s, dx * s + dy * c, _wrap(pose[2] - oyaw))


def _compose(a, b) -> tuple[float, float, float]:
    c, s = math.cos(a[2]), math.sin(a[2])
    return (a[0] + c * b[0] - s * b[1], a[1] + s * b[0] + c * b[1], _wrap(a[2] + b[2]))


def _inverse(a) -> tuple[float, float, float]:
    c, s = math.cos(a[2]), math.sin(a[2])
    return (-(c * a[0] + s * a[1]), -(-s * a[0] + c * a[1]), _wrap(-a[2]))


def _wrap(a: float) -> float:
    return (a + math.pi) % (2 * math.pi) - math.pi


def run(cfg: RunConfig, factory: Callable[[], object] | None = None, *,
        verbose: bool = False) -> RunResult:
    t_wall = time.perf_counter()
    course = load_course(cfg.course)
    sensors = SensorModel.preset(cfg.errors, course)
    spec = VehicleSpec.load()
    veh = VehicleModel(spec, course.start)
    lidar = VirtualLidar(course, spec, sensors.lidar, seed=cfg.seed)
    asm = ScanAssembler()
    rng = np.random.default_rng(cfg.seed + 12345)
    driver = TruthDriver(course, spec, a_lat=cfg.a_lat)
    slam = SlamUnderTest(factory or default_factory())
    body = course.body_samples(spec.footprint)

    truth = _TruthLog()
    t_ns = 0
    queue: list[tuple[int, object]] = []
    speed_meas = 0.0
    vs: VehicleState | None = None
    next_tlm = 0
    bias = math.radians(sensors.gyro_bias_dps)
    gsig = math.radians(sensors.gyro_noise_dps)

    explore, race = PhaseStats(), PhaseStats()
    stats = explore
    phase = "explore"
    v_max = cfg.explore_speed
    explore_len = cfg.laps * driver.length
    race_len = cfg.race_laps * driver.length
    origin = None
    traj_true: list[tuple[float, float, float]] = []
    traj_est: list[tuple[float, float, float]] = []
    phase_split = 0
    map_before: dict = {}
    map_after: dict = {}
    freeze_ms = 0.0
    lost_streak = 0
    was_colliding = False
    last_plan_ns = 0
    steer_cmd = 0.0
    speed_cmd = 0.0
    race_start_progress = 0.0
    max_t = (explore_len / 0.3 + race_len / 0.5) + 60.0

    while t_ns < max_t * NS:
        # ── 真値の運転手（100Hz 相当で指令を更新）──
        if t_ns < cfg.still_s * NS:
            speed_cmd, steer_cmd = 0.0, 0.0
        elif (t_ns // 1_000_000) % 10 == 0:
            speed_cmd, steer_cmd = driver.command(veh.x, veh.y, veh.yaw, v_max)
        veh.apply(DriveInput(armed=True, target_speed=speed_cmd, target_steer=steer_cmd))
        veh.step(DT)
        t_ns += int(DT * NS)
        truth.add(t_ns, veh.x, veh.y, veh.yaw)
        hit = course.collides(veh.x, veh.y, veh.yaw, body)
        if hit and not was_colliding:
            stats.collisions += 1
        was_colliding = hit

        # ── 車速・ヨーレートの計測（誤差を注入）──
        scale = sensors.speed_scale
        for (x0, y0, x1, y1, k) in sensors.slip_zones:
            if x0 <= veh.x <= x1 and y0 <= veh.y <= y1:
                scale *= k
        a = 1.0 if sensors.speed_tau <= 0 else 1.0 - math.exp(-DT / sensors.speed_tau)
        speed_meas += a * (veh.speed * scale - speed_meas)
        if t_ns >= next_tlm:
            next_tlm += NS // TELEMETRY_HZ
            sp = speed_meas + (float(rng.normal(0, sensors.speed_noise)) if sensors.speed_noise > 0 else 0.0)
            yr = (veh.yaw_rate * sensors.gyro_scale + bias
                  + (float(rng.normal(0, gsig)) if gsig > 0 else 0.0))
            vs = VehicleState(speed=sp, yaw_rate=yr, steer_actual=veh.steer_actual,
                              t_capture=t_ns, stopped=abs(sp) < SPEED_DEADBAND_MPS)
            slam.on_vehicle_state(vs)

        # ── LiDAR（セクタ単位・伝送遅延つき）→ ScanAssembler ──
        for gen_ns, pkt in lidar.poll(t_ns, veh, lambda t: (t // 1000) & 0xFFFFFFFF):
            queue.append((gen_ns, pkt))
        scan = None
        if queue:
            keep = []
            for gen_ns, pkt in queue:
                if gen_ns <= t_ns:
                    # セクタ先頭の Pi 時刻。時刻同期が収束している前提で、STM32 時刻をそのまま戻す
                    done = asm.feed(pkt, int(pkt.t_start_us) * 1000)
                    if done is not None:
                        scan = done
                else:
                    keep.append((gen_ns, pkt))
            queue = keep
        if scan is None or vs is None:
            continue

        dt = (t_ns - last_plan_ns) / NS if last_plan_ns else 0.1
        last_plan_ns = t_ns
        t0 = time.perf_counter()
        pose, score, lost, t_ref = slam.update(scan, vs, dt)
        stats.update_ms.append((time.perf_counter() - t0) * 1000.0)
        stats.updates += 1
        if lost:
            stats.lost += 1
            lost_streak += 1
            stats.max_lost_streak = max(stats.max_lost_streak, lost_streak)
        else:
            lost_streak = 0
        tp = truth.at(t_ref) if t_ref > 0 else (veh.x, veh.y, veh.yaw)
        if origin is None:
            # SLAM の原点 = 最初の点群の基準時刻の姿勢。**初回の推定値そのものを
            # 真値に重ねる**（初回に推測航法を少し進める実装でも原点がずれない
            # ように。ずれは「世界座標で一定のオフセット」として全周期に乗り、
            # 進行方向が変わるたびに誤差の見かけを変えてしまう）
            origin = _compose(tp, _inverse(tuple(pose)))
        tr = _rel(origin, tp)
        stats.pos_err.append(math.hypot(pose[0] - tr[0], pose[1] - tr[1]))
        _c, _s = math.cos(tr[2]), math.sin(tr[2])
        _dx, _dy = pose[0] - tr[0], pose[1] - tr[1]
        stats.fwd_err.append(_c * _dx + _s * _dy)
        stats.lat_err.append(-_s * _dx + _c * _dy)
        stats.yaw_err.append(abs(math.degrees(_wrap(pose[2] - tr[2]))))
        traj_true.append(tr)
        traj_est.append((pose[0], pose[1], pose[2]))

        if verbose and stats.updates % 50 == 0:
            print(f"  [{phase}] {driver.progress:6.1f}m err {stats.pos_err[-1] * 100:5.1f}cm "
                  f"{stats.yaw_err[-1]:4.1f}deg score {score:.2f}{' LOST' if lost else ''}",
                  flush=True)

        if phase == "explore" and driver.progress >= explore_len:
            map_before = map_quality(slam.grid, course, origin)
            t0 = time.perf_counter()
            slam.freeze()
            freeze_ms = (time.perf_counter() - t0) * 1000.0
            aligned = align_map(slam.grid, course, origin)
            map_after = map_quality(slam.grid, course, aligned)
            map_after["rigid_deg"] = abs(math.degrees(_wrap(aligned[2] - origin[2])))
            map_after["rigid_cm"] = math.hypot(aligned[0] - origin[0], aligned[1] - origin[1]) * 100
            # ここから先（凍結地図での走行）は、地図を真のコースへ重ねた姿勢を原点にする
            origin = aligned
            phase = "race"
            stats = race
            v_max = cfg.race_speed
            phase_split = len(traj_true)
            race_start_progress = driver.progress
            lost_streak = 0
        elif phase == "race" and driver.progress - race_start_progress >= race_len:
            break

    return RunResult(
        cfg=cfg, explore=explore, race=race, map_before=map_before, map_after=map_after,
        freeze_ms=freeze_ms, relocs=slam.relocs(), loop_closures=slam.loop_closures(),
        out_of_bounds=int(getattr(slam.grid, "out_of_bounds", 0)),
        traj_true=np.asarray(traj_true), traj_est=np.asarray(traj_est),
        phase_split=phase_split, wall_time_s=time.perf_counter() - t_wall,
        map_walls=_walls_world(slam.grid, origin) if origin is not None else None,
        course_obj=course)


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
    out = m.copy()
    out[1:, :] &= m[:-1, :]
    out[:-1, :] &= m[1:, :]
    out[:, 1:] &= m[:, :-1]
    out[:, :-1] &= m[:, 1:]
    return out


# ── 出力 ──────────────────────────────────────────────────────────────────

_COLS = [
    ("course", "{:>18s}"), ("seed", "{:>4d}"), ("errors", "{:>5s}"),
    ("ex_ate", "{:7.1f}"), ("ex_max", "{:7.1f}"), ("ex_final", "{:7.1f}"), ("ex_yaw", "{:6.2f}"), ("ex_lost", "{:6.1f}"),
    ("map_prec_before", "{:6.2f}"), ("map_prec_after", "{:6.2f}"), ("map_rec_after", "{:6.2f}"),
    ("map_blur_after", "{:6.1f}"), ("map_rigid_deg", "{:6.2f}"), ("freeze_ms", "{:7.0f}"),
    ("rc_ate", "{:7.1f}"), ("rc_max", "{:7.1f}"), ("rc_lat", "{:6.1f}"), ("rc_lat99", "{:6.1f}"),
    ("rc_fwd", "{:6.1f}"), ("rc_yaw", "{:6.2f}"), ("rc_lost", "{:6.1f}"),
    ("rc_streak", "{:5d}"), ("ms_mean", "{:6.1f}"), ("ms_max", "{:7.0f}"),
    ("relocs", "{:4d}"), ("loops", "{:4d}"), ("oob", "{:6d}"),
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
    ap.add_argument("--plot", action="store_true")
    ap.add_argument("--json", default=None, help="結果の行を JSON で保存")
    ap.add_argument("--verbose", action="store_true")
    args = ap.parse_args()

    kw = {}
    if args.map_size:
        kw["size_m"] = args.map_size
    if args.no_loop:
        kw["loop_closure"] = False
    rows = []
    for err in args.errors.split(","):
        for name in args.course.split(","):
            for s in range(args.seed0, args.seed0 + args.seeds):
                cfg = RunConfig(course=name, seed=s, errors=err, laps=args.laps,
                                explore_speed=args.explore_speed, race_laps=args.race_laps,
                                race_speed=args.race_speed)
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
