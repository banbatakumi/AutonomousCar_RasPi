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
    """

    def __init__(self, centerline_xyyaw: np.ndarray) -> None:
        self._xy = centerline_xyyaw[:, :2]
        loop = np.vstack([self._xy, self._xy[:1]])
        seg_len = np.hypot(*np.diff(loop, axis=0).T)
        self._arc = np.concatenate([[0.0], np.cumsum(seg_len)])[:-1]
        self._total = float(seg_len.sum())
        self._prev = 0.0

    def _nearest(self, x: float, y: float) -> tuple[float, float]:
        d2 = (self._xy[:, 0] - x) ** 2 + (self._xy[:, 1] - y) ** 2
        i = int(np.argmin(d2))
        return float(self._arc[i]), float(math.sqrt(d2[i]))

    def reset(self, x: float, y: float) -> None:
        self._prev, _ = self._nearest(x, y)

    def update(self, x: float, y: float) -> tuple[float, float]:
        """`(弧長方向の進捗 [m], 横偏差 [m])`。進捗は1周のラップアラウンドを展開する。"""
        p, cross = self._nearest(x, y)
        d = p - self._prev
        if self._total > 0:
            if d > self._total / 2:
                d -= self._total
            elif d < -self._total / 2:
                d += self._total
        self._prev = p
        return d, cross


class SimE2EEnv:
    """1エピソード=1コースの周回試行。`reset()`/`step()` は gymnasium と同じ形の
    戻り値にしてあるが、このクラス自体は `gymnasium.Env` を継承しない。

    観測は `(OBS_DIM,)` = 点群`SCAN_DIM`点 + 自車速度1個（末尾、[m/s]、生値）。
    速度を混ぜているのは、点群だけでは「今どれくらいの速さで走っているか」が
    分からず、速度依存の減速判断（例: 速いほど早めに舵を戻す）を学びにくいため
    （2026-08-28、バンビの指摘で追加）。
    """

    def __init__(self, courses: list[Course] | None = None, *,
                course_fn: Callable[[np.random.Generator], Course] | None = None,
                spec: VehicleSpec | None = None,
                max_steps: int = 2000, max_range: float = LIDAR_C_SATURATED_M,
                max_speed: float = 1.5, max_steer: float = 0.45,
                collision_penalty: float = -5.0, progress_weight: float = 1.0,
                cross_track_weight: float = 0.5, start_jitter_m: float = 0.03,
                start_jitter_rad: float = math.radians(5), sim_params: SimParams | None = None,
                randomize_lidar: bool = True,
                lidar_noise_sigma_range: tuple[float, float] = (0.0, 0.03),
                lidar_noise_rel_range: tuple[float, float] = (0.0, 0.015),
                lidar_drop_rate_range: tuple[float, float] = (0.0, 0.06),
                lidar_sector_drop_rate_range: tuple[float, float] = (0.0, 0.015),
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
        """
        if courses is None and course_fn is None:
            raise ValueError("courses か course_fn のどちらかは要ります")
        self.courses = courses
        self.course_fn = course_fn
        self.spec = spec or VehicleSpec.load()
        self.max_steps = max_steps
        self.max_range = max_range
        self.max_speed = max_speed
        self.max_steer = max_steer
        self.collision_penalty = collision_penalty
        self.progress_weight = progress_weight
        self.cross_track_weight = cross_track_weight
        self.start_jitter_m = start_jitter_m
        self.start_jitter_rad = start_jitter_rad
        self.sim_params = sim_params or SimParams()
        self.randomize_lidar = randomize_lidar
        self.lidar_noise_sigma_range = lidar_noise_sigma_range
        self.lidar_noise_rel_range = lidar_noise_rel_range
        self.lidar_drop_rate_range = lidar_drop_rate_range
        self.lidar_sector_drop_rate_range = lidar_sector_drop_rate_range
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
        cx, cy, cyaw = self.course.centerline[i]
        jx, jy = self.rng.normal(0.0, self.start_jitter_m, 2)
        jyaw = self.rng.normal(0.0, self.start_jitter_rad)
        start = (float(cx + jx), float(cy + jy), float(cyaw + jyaw))

        self.vehicle = VehicleModel(self.spec, start)
        self.lidar = VirtualLidar(self.course, self.spec, self._episode_sim_params(),
                                  seed=int(self.rng.integers(1 << 31)))
        self.asm = ScanAssembler()
        self._body = self.course.body_samples(self.spec.footprint)
        self._progress = _CenterlineProgress(self.course.centerline)
        self._progress.reset(self.vehicle.x, self.vehicle.y)
        self._t_ns = 0
        self._steps = 0

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
            ここでも安全側にもう一度クランプする）。"""
        steer = float(np.clip(action[0], -self.max_steer, self.max_steer))
        speed = float(np.clip(action[1], 0.0, self.max_speed))
        self.vehicle.apply(DriveInput(armed=True, target_speed=speed, target_steer=steer))
        self.vehicle.step(_STEP_NS / NS)
        self._t_ns += _STEP_NS

        hit = self.course.collides(self.vehicle.x, self.vehicle.y, self.vehicle.yaw, self._body)
        self.vehicle.note_collision(hit)

        for gen_ns, pkt in self.lidar.poll(self._t_ns, self.vehicle, _stm_us):
            s = self.asm.feed(pkt, gen_ns)
            if s is not None:
                self._last_scan = s

        progress, cross_track = self._progress.update(self.vehicle.x, self.vehicle.y)
        reward = self.progress_weight * progress - self.cross_track_weight * abs(cross_track)

        terminated = False
        if hit:
            reward += self.collision_penalty
            terminated = True

        self._steps += 1
        truncated = self._steps >= self.max_steps

        info = {"cross_track": cross_track, "collided": hit, "progress": progress}
        return self._obs(self._last_scan), reward, terminated, truncated, info

    def _obs(self, scan: Scan) -> np.ndarray:
        w = scan_window(scan, 360.0, self.max_range)
        speed = float(np.clip(self.vehicle.speed, 0.0, self.max_speed))
        return np.concatenate([np.asarray(w.dist, dtype=np.float32),
                               np.array([speed], dtype=np.float32)])
