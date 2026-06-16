"""シミュレータ・バックエンド。

AckermannModel（物理）と LidarSimulator（センサ）を統合し、BackendBase を
実装する。制御ロジックからは実機 RealBackend と全く同じインターフェースで
利用できる。
"""

from __future__ import annotations

from core.interfaces import (
    BackendBase,
    ControlCommand,
    LidarScan,
    VehicleState,
)

from .lidar_sim import LidarSimulator
from .physics import AckermannModel


class SimBackend(BackendBase):
    """物理演算＋レイキャストLiDARによるシミュレータバックエンド。"""

    def __init__(self, vehicle_config: dict, sim_config: dict, course: dict):
        self.vehicle_config = vehicle_config
        self.sim_config = sim_config
        noise = float(sim_config["simulation"].get("lidar_noise_sigma", 0.02))

        self._start_pose = tuple(course["start_pose"])
        self.model = AckermannModel(vehicle_config, start_pose=self._start_pose)
        self.lidar = LidarSimulator(course["walls"], noise_sigma=noise)
        self.walls = course["walls"]

    # ------------------------------------------------------------------
    def load_course(self, course: dict) -> None:
        """コースを切り替える（壁・スタート位置を更新しリセット）。"""
        self._start_pose = tuple(course["start_pose"])
        self.walls = course["walls"]
        self.lidar.set_walls(course["walls"])
        self.model.reset(self._start_pose)

    # ------------------------------------------------------------------
    def get_lidar_scan(self) -> LidarScan:
        return self.lidar.scan(self.model.get_state())

    def send_command(self, cmd: ControlCommand) -> None:
        self.model.set_command(cmd.target_speed, cmd.target_steer)

    def get_vehicle_state(self) -> VehicleState:
        return self.model.get_state()

    def step(self, dt: float) -> None:
        self.model.step(dt)

    def reset(self) -> None:
        self.model.reset(self._start_pose)
