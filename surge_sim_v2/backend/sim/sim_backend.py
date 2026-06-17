"""シミュレーションバックエンド。

PhysicsModel（アッカーマン物理）と LidarSimulator（レイキャスト）を束ね、
BackendBase を実装する。Controller からは実機 RealBackend と同じ顔に見える。
"""
from __future__ import annotations

import time
from collections import deque

import numpy as np

from backend.base import BackendBase
from core.interfaces import (
    ConnectionStatus, ControlCommand, ImuReading, LidarScan, VehicleState,
)
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
            config=sim_cfg.get("lidar", {}),
        )
        self._cmd: ControlCommand | None = None
        self._last_lidar_ts = time.time()

        # アクチュエータ遅延：指令が効くまでの遅れ（UART往復＋STM32＋サーボ応答を模擬）。
        # 制御周期 20ms に対し N ステップ遅らせる。実機の遅延は制御安定性に直結する。
        control_dt = 0.02
        latency_ms = float(sim_cfg.get("command_latency_ms", 0.0)) if isinstance(sim_cfg, dict) else 0.0
        self._latency_steps = max(0, int(round(latency_ms / (control_dt * 1000.0))))
        self._cmd_delay = deque(maxlen=self._latency_steps + 1)
        self._latest_cmd: ControlCommand | None = None

        # 動的障害物（人・車）：{x,y,r,vx,vy} のリスト。step で移動。
        self._obstacles: list[dict] = []

        # IMU 模擬（実機の AHRS ヨーを模す）：小さなノイズ＋ゆっくりしたバイアスドリフト
        imu_cfg = sim_cfg.get("imu", {}) if isinstance(sim_cfg, dict) else {}
        self._imu_enabled = bool(imu_cfg.get("enabled", True))
        self._imu_noise = float(imu_cfg.get("noise_deg", 0.3))
        self._imu_bias_walk = float(imu_cfg.get("bias_walk_deg", 0.0005))
        self._imu_bias_max = float(imu_cfg.get("bias_max_deg", 0.5))  # AHRSの残留バイアス上限
        self._imu_bias = 0.0
        self._prev_heading = start_pose[2]
        self._imu: ImuReading | None = None

    # ---- 障害物 ----------------------------------------------------------
    def add_obstacle(self, x: float, y: float, r: float = 0.18,
                     vx: float = 0.0, vy: float = 0.0) -> None:
        self._obstacles.append({"x": x, "y": y, "r": r, "vx": vx, "vy": vy})

    def clear_obstacles(self) -> None:
        self._obstacles = []

    def get_obstacles(self) -> list[dict]:
        return self._obstacles

    def set_course(self, walls: list, start_pose: tuple[float, float, float]) -> None:
        self.lidar.set_walls(walls)
        self.physics.set_start_pose(start_pose)
        self.physics.reset()

    def send_command(self, cmd: ControlCommand) -> None:
        self._latest_cmd = cmd

    def step(self, dt: float) -> None:
        # アクチュエータ遅延：N ステップ前の指令を実際に適用する
        if self._latency_steps > 0:
            self._cmd_delay.append(self._latest_cmd)
            self._cmd = self._cmd_delay[0] if len(self._cmd_delay) > self._latency_steps else None
        else:
            self._cmd = self._latest_cmd
        self.physics.step(dt, self._cmd)
        # 動的障害物を移動させ LiDAR に反映
        if self._obstacles:
            for o in self._obstacles:
                o["x"] += o["vx"] * dt
                o["y"] += o["vy"] * dt
            self.lidar.set_obstacles([(o["x"], o["y"], o["r"]) for o in self._obstacles])
        v = self.physics_state()
        scan = self.lidar.scan(v)
        self._last_lidar_ts = scan.timestamp
        self._update_imu(v, dt)

    def _update_imu(self, v: VehicleState, dt: float) -> None:
        if not self._imu_enabled:
            self._imu = None
            return
        # ゆっくりしたバイアスのランダムウォーク（AHRS の残留バイアスを模擬）
        self._imu_bias += np.random.normal(0.0, self._imu_bias_walk)
        self._imu_bias = float(np.clip(self._imu_bias, -self._imu_bias_max, self._imu_bias_max))
        meas = v.heading + self._imu_bias + np.random.normal(0.0, self._imu_noise)
        yaw_rate = (((v.heading - self._prev_heading + 180) % 360) - 180) / dt if dt > 0 else 0.0
        self._prev_heading = v.heading
        self._imu = ImuReading(heading=meas % 360.0, yaw_rate=yaw_rate, timestamp=time.time())

    def get_imu_reading(self) -> ImuReading | None:
        return self._imu

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
