"""SLAMエンジン（Phase3 / 実機相当モード）。

OccupancyGridMapper を中核に、占有格子の構築と自己位置推定を行う。

使い方は2系統：
  1. update(scan, pose) … 既知姿勢マッピング（cheat モードの地図構築用）
  2. process(scan)       … 実機相当 SLAM。LiDAR のみで自己位置推定＋常時マッピング
     - 既知のスタート地点からブートストラップ（最初の数スキャンは姿勢固定で地図を種まき）
     - 以降は SLAMLocalizer でスキャンマッチ → その推定姿勢で地図を更新

本コース（6×4m < LiDAR 12m）は全体が常に見えるため、純 LiDAR-SLAM でも安定する。
"""
from __future__ import annotations

from .interfaces import LidarScan, LocalizationResult, OccupancyGrid
from .localization import SLAMLocalizer
from .occupancy import OccupancyGridMapper


class HectorSLAM:
    def __init__(self, resolution: float = 0.05,
                 bounds: tuple[float, float, float, float] = (-1.0, -1.0, 7.0, 5.0),
                 start_pose: tuple[float, float, float] = (0.0, 0.0, 0.0),
                 bootstrap_scans: int = 5,
                 keyframe_dist: float = 0.08,
                 keyframe_angle: float = 5.0) -> None:
        self.resolution = resolution
        self.mapper = OccupancyGridMapper(resolution=resolution, bounds=bounds)
        self.localizer = SLAMLocalizer(self.mapper, start_pose=start_pose)
        self.start_pose = start_pose
        self.bootstrap_scans = bootstrap_scans
        # キーフレーム・マッピング：一定距離/角度動いた時だけ地図更新（誤差の蓄積増幅を抑制）
        self.keyframe_dist = keyframe_dist
        self.keyframe_angle = keyframe_angle
        self.localization_only = False     # True で地図更新を凍結（自己位置推定のみ）
        self._count = 0
        self._last_kf = None               # 最後に地図更新した姿勢 (x,y,heading)

    # ---- 設定 -------------------------------------------------------------
    def set_bounds(self, bounds: tuple[float, float, float, float]) -> None:
        self.mapper.set_bounds(bounds)

    def set_start_pose(self, pose: tuple[float, float, float]) -> None:
        self.start_pose = pose
        self.localizer.set_pose(*pose)

    # ---- 既知姿勢マッピング（cheat 用） ----------------------------------
    def update(self, scan: LidarScan, pose) -> OccupancyGrid:
        self.mapper.integrate_scan(scan, pose)
        return self.mapper.to_occupancy_grid()

    # ---- 実機相当 SLAM（自己位置推定＋常時マッピング） -------------------
    def process(self, scan: LidarScan) -> LocalizationResult:
        self._count += 1
        if self._count <= self.bootstrap_scans:
            # スタート地点で地図を種まき（全体が見えるのでほぼ完成する）
            seed = LocalizationResult(
                x=self.start_pose[0], y=self.start_pose[1], heading=self.start_pose[2],
                confidence=0.3, source="slam", timestamp=scan.timestamp,
            )
            self.mapper.integrate_scan(scan, seed)
            self.localizer.set_pose(*self.start_pose)
            self._last_kf = self.start_pose
            return seed

        result = self.localizer.update(scan)
        # キーフレーム時のみ、かつ未知セルだけを埋める形で地図更新する。
        # 既知セルを凍結することで、良い地図を壊さず（＝自己位置推定を安定に保ち）、
        # 未探索領域だけを継続的に追加できる。
        if not self.localization_only and self._is_keyframe(result):
            self.mapper.integrate_scan(scan, result, only_unknown=True)
            self._last_kf = (result.x, result.y, result.heading)
        return result

    def _is_keyframe(self, result) -> bool:
        if self._last_kf is None:
            return True
        dx = result.x - self._last_kf[0]
        dy = result.y - self._last_kf[1]
        dth = abs((result.heading - self._last_kf[2] + 180) % 360 - 180)
        return (dx * dx + dy * dy) >= self.keyframe_dist ** 2 or dth >= self.keyframe_angle

    def get_map(self) -> OccupancyGrid:
        return self.mapper.to_occupancy_grid()

    def reset(self) -> None:
        self.mapper.reset()
        self.localizer.set_pose(*self.start_pose)
        self._count = 0

    def save(self, path: str) -> None:
        self.mapper.save(path)

    def load(self, path: str) -> OccupancyGrid:
        self.mapper.load(path)
        return self.mapper.to_occupancy_grid()
