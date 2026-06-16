"""シミュレーションバックエンド。

PhysicsModel（アッカーマン物理）と LidarSimulator（レイキャスト）を束ね、
BackendBase を実装する。Controller からは実機 RealBackend と同じ顔に見える。
"""
from __future__ import annotations

import time

from backend.base import BackendBase
from core.interfaces import ConnectionStatus, ControlCommand, LidarScan, VehicleState
from core.shared_state import SharedState

from .lidar_sim import LidarSimulator
from .physics import PhysicsModel


class SimBackend(BackendBase):
    def __init__(self, vehicle_cfg: dict, sim_cfg: dict, shared_state: SharedState,
                 walls: list, start_pose: tuple[float, float, float]) -> None:
        self.shared = shared_state
        self.physics = PhysicsModel(vehicle_cfg, shared_state, start_pose)
        self.lidar = LidarSimulator(
            walls, shared_state,
            noise_sigma=float(sim_cfg.get("lidar_noise_sigma", 0.02)),
        )
        self._cmd: ControlCommand | None = None
        self._last_lidar_ts = time.time()

    def set_course(self, walls: list, start_pose: tuple[float, float, float]) -> None:
        self.lidar.set_walls(walls)
        self.physics.set_start_pose(start_pose)
        self.physics.reset()

    def send_command(self, cmd: ControlCommand) -> None:
        self._cmd = cmd

    def step(self, dt: float) -> None:
        self.physics.step(dt, self._cmd)
        scan = self.lidar.scan(self.physics_state())
        self._last_lidar_ts = scan.timestamp

    def physics_state(self) -> VehicleState:
        return self.shared.get_vehicle()

    def get_vehicle_state(self) -> VehicleState:
        return self.shared.get_vehicle()

    def get_lidar_scan(self) -> LidarScan:
        return self.shared.get_lidar()

    def reset(self) -> None:
        self.physics.reset()
        self._cmd = None

    def get_connection_status(self) -> ConnectionStatus:
        now = time.time()
        return ConnectionStatus(
            websocket_connected=True,
            last_received_at=now,
            uart_connected=False,          # シミュでは UART なし
            lidar_receiving=(now - self._last_lidar_ts) < 0.5,
            stm32_connected=False,         # シミュでは STM32 なし
            latency_ms=0.0,
        )
