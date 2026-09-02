"""`slam2d.core.Frontend`(車体非依存の汎用SLAM)を`raspi/auto/`のplannerから使う
ための薄いブリッジ。

`slam2d/`は`raspi.*`を一切importしない独立ライブラリとして設計してある
（`/Users/banbatakumi/.claude/plans/wiggly-doodling-moth.md`参照）。逆方向
（`raspi/`側が`slam2d`を使う）はこのファイルに閉じ込め、以下の車体固有の
変換・付加機能だけをここに置く:

- `Scan`（raspi固有のLD06セクタ形式）→`RawScan`（slam2d、点ごとの時刻を持つ
  汎用形式）の変換
- `VehicleState`（ジャイロ+速度）→`Twist2D`の橋渡し
- `raspi/nav/slam.py`の`Slam`が持っていた`lap_progress()`（累積回頭÷360°）
  のような、slam2d本体には無い車体固有の付加機能

**ループ閉じ（`slam2d/backend/`・`slam2d/pipeline.SlamSystem`）は使わない。**
Phase 5の実測で、単一〜少数のループ拘束では`Frontend`単体より精度が悪化する
ことがあると確認済みのため（`slam2d/backend/loop_detection.py`のモジュール
docstring「既知の限界」参照）。まずは`Frontend`単体（`slam2d/core/confidence.py`
の動的異方性ブレンドだけ）で運用し、`backend/`はg2opy依存なのでimportすら
しない（Pi実機側の依存を増やさないため）。
"""

from __future__ import annotations

import math

import numpy as np

from slam2d.core.frontend import Frontend, FrontendConfig, FrontendUpdate
from slam2d.core.grid import OccGrid
from slam2d.core.motion import ExternalTwistModel, GyroBiasEstimator
from slam2d.core.types import Pose2D, RawScan, Twist2D, wrap_angle

from ..msgs.types import Scan, VehicleState
from ..nav.deskew import point_times_ns

__all__ = ["Slam2dNav", "scan_to_raw"]


def scan_to_raw(scan: Scan) -> RawScan:
    """`Scan`（raspi固有のLD06セクタ形式）を`RawScan`（slam2dの汎用形式）へ変換する。

    有効判定（`sector_seen`かつ`dist>0`）・飽和判定は`raspi/nav/deskew.py`の
    `deskew()`と同じ規約に合わせてある——ここだけ違う読み方をすると、この
    plannerだけ「空きとして彫ってはいけない場所」を彫ってしまう事故になる
    （`raspi/nav/deskew.py`冒頭docstringの「3種類の点を区別する」参照）。
    """
    deg = np.arange(360)
    dist = np.asarray(scan.dist, dtype=np.float64)
    seen = np.asarray(scan.sector_seen, dtype=bool)
    sector = ((deg - 1) % 360) // 30
    valid = seen[sector] & (dist > 0.0)
    saturated = (np.asarray(scan.saturated, dtype=bool) if scan.saturated is not None
                else np.zeros(360, dtype=bool))
    angles = np.radians(deg.astype(np.float64))
    t_point = point_times_ns(scan)
    return RawScan(angles, dist, valid, saturated, t_point)


class Slam2dNav:
    """`raspi/nav/slam.py`の`Slam`と部分的に互換なインターフェースで
    `slam2d.core.Frontend`を包む。

    `raspi/auto/raceline.py`が期待する最小限の面（`update`/`pose`/`grid`/
    `trajectory`/`trajectory_array`/`lap_progress`/`distance`/`speed`/
    `yaw_rate`/`freeze`/`reset`）だけを実装する。`close_loop()`は実装しない
    （上のモジュールdocstring参照。呼び出し側でループ閉じ相当の処理を省く）。
    """

    def __init__(self, *, resolution: float, size_m: float,
                lidar_x: float = 0.0, lidar_y: float = 0.0,
                max_range: float = 12.0) -> None:
        self._resolution = resolution
        self._size_m = size_m
        self._lidar_x = lidar_x
        self._lidar_y = lidar_y
        self._max_range = max_range
        self._raw_yaw_rate = 0.0
        self._raw_speed = 0.0
        self.reset()

    def reset(self) -> None:
        grid = OccGrid(resolution=self._resolution, size_m=self._size_m,
                       min_hits=3, min_seen=3)
        bias = GyroBiasEstimator()
        motion = ExternalTwistModel(self._current_twist, bias_estimator=bias)
        config = FrontendConfig(mount_x=self._lidar_x, mount_y=self._lidar_y,
                                max_range=self._max_range)
        self._fe = Frontend(grid, motion, config)
        self.heading_total = 0.0

    def _current_twist(self) -> Twist2D:
        return Twist2D(self._raw_speed, 0.0, self._raw_yaw_rate)

    def update(self, scan: Scan, dt: float, *, yaw_rate: float | None,
               speed: float | None) -> FrontendUpdate:
        """1周ぶんの点群を取り込んで姿勢を更新する。

        `raspi/nav/slam.py`の`Slam.update()`と同じ引数（`yaw_rate`/`speed`が
        `None`なら`ExternalTwistModel`に0を渡す——`slam2d`側に等速度モデルへの
        自動フォールバックは無いため、`raspi/auto/raceline.py`の`lidar_only`
        オプションはこのブリッジでは未対応）。
        """
        self._raw_yaw_rate = 0.0 if yaw_rate is None else float(yaw_rate)
        self._raw_speed = 0.0 if speed is None else float(speed)
        raw = scan_to_raw(scan)
        prev_yaw = self._fe.pose.yaw
        u = self._fe.update(raw, dt)
        self.heading_total += wrap_angle(u.pose.yaw - prev_yaw)
        return u

    @property
    def pose(self) -> Pose2D:
        return self._fe.pose

    @property
    def grid(self) -> OccGrid:
        return self._fe.grid

    @property
    def trajectory(self) -> list[Pose2D]:
        return self._fe.trajectory

    def trajectory_array(self) -> np.ndarray:
        """軌跡を`(N, 3)`の配列で。空なら`(0, 3)`（`raspi/nav/slam.py`と同じ約束）。"""
        if not self._fe.trajectory:
            return np.zeros((0, 3), dtype=np.float64)
        return np.asarray(self._fe.trajectory, dtype=np.float64)

    @property
    def distance(self) -> float:
        """累積走行距離[m]（軌跡の点間距離の総和で近似）。"""
        traj = self._fe.trajectory
        if len(traj) < 2:
            return 0.0
        arr = np.asarray(traj, dtype=np.float64)
        d = np.diff(arr[:, :2], axis=0)
        return float(np.hypot(d[:, 0], d[:, 1]).sum())

    def lap_progress(self) -> float:
        """周回の進み具合[周]。累積回頭を360°で割ったもの（`Slam`と同じ定義）。"""
        return abs(self.heading_total) / (2.0 * math.pi)

    @property
    def speed(self) -> float:
        return self._fe.motion.current_twist().vx

    @property
    def yaw_rate(self) -> float:
        return self._fe.motion.current_twist().yaw_rate

    @property
    def max_range(self) -> float:
        return self._max_range

    @property
    def lidar_x(self) -> float:
        return self._lidar_x

    @property
    def lidar_y(self) -> float:
        return self._lidar_y

    def freeze(self) -> None:
        self._fe.grid.freeze()

    def deskew_scan(self, scan: Scan):
        """`scan`を現在保持しているtwistで脱スキューする（障害物検出用）。

        `raspi/auto/raceline.py`の`_update_obstacles()`が使う。`Frontend.update()`
        の内部でも同じ脱スキューを1回行っているが、そちらは非公開の中間状態
        なのでここで独立に呼び直す（`raspi/nav/slam.py`の`Slam`が`speed`/
        `yaw_rate`を露出していたのと同じ立て付け）。
        """
        from slam2d.core.deskew import deskew as slam2d_deskew

        raw = scan_to_raw(scan)
        return slam2d_deskew(raw, self._current_twist(),
                             mount_x=self._lidar_x, mount_y=self._lidar_y,
                             max_range=self._max_range)
