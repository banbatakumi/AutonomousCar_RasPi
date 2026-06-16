"""Ackermann（アッカーマン）モデル物理演算。

自転車モデルをベースに、アッカーマンジオメトリの旋回半径・角速度を計算する。
速度とステアにはそれぞれ一次遅れを入れて実機の応答を模擬する。

座標系: heading [deg] 0=East、反時計回り正。ステア正 = 左旋回（CCW）。
"""
from __future__ import annotations

import math
import time

import numpy as np

from core.interfaces import ControlCommand, VehicleState
from core.shared_state import SharedState


class PhysicsModel:
    def __init__(self, vehicle_cfg: dict, shared_state: SharedState,
                 start_pose: tuple[float, float, float] = (0.0, 0.0, 0.0)) -> None:
        v = vehicle_cfg
        self.wheelbase = float(v["wheelbase"])        # L [m]
        self.tread = float(v["tread"])                # トレッド幅 [m]
        self.max_steer = float(v["max_steer_angle"])  # [deg]
        self.max_speed = float(v["max_speed"])        # [m/s]
        self.tau_speed = float(v["speed_time_constant"])
        self.tau_steer = float(v["steer_time_constant"])

        self.shared = shared_state
        self._start_pose = start_pose

        # 内部状態
        self.x = 0.0
        self.y = 0.0
        self.heading = 0.0      # [deg]
        self.speed = 0.0        # [m/s] 実速度
        self.steer = 0.0        # [deg] 実ステア角
        self.accel = 0.0        # [m/s^2]
        self._prev_speed = 0.0

        self.reset()

    def set_start_pose(self, pose: tuple[float, float, float]) -> None:
        self._start_pose = pose

    def reset(self) -> None:
        self.x, self.y, self.heading = self._start_pose
        self.speed = 0.0
        self.steer = 0.0
        self.accel = 0.0
        self._prev_speed = 0.0
        self._publish()

    def step(self, dt: float, cmd: ControlCommand | None) -> None:
        if dt <= 0.0:
            return

        # --- 指令値（クランプ） ---
        if cmd is not None:
            target_speed = float(np.clip(cmd.target_speed, -self.max_speed, self.max_speed))
            target_steer = float(np.clip(cmd.target_steer, -self.max_steer, self.max_steer))
        else:
            target_speed = 0.0
            target_steer = 0.0

        # --- 一次遅れ応答 ---
        a_s = dt / max(self.tau_speed, 1e-6)
        a_t = dt / max(self.tau_steer, 1e-6)
        self.speed += (target_speed - self.speed) * min(a_s, 1.0)
        self.steer += (target_steer - self.steer) * min(a_t, 1.0)

        # --- アッカーマン旋回 ---
        steer_rad = math.radians(self.steer)
        heading_rad = math.radians(self.heading)

        if abs(steer_rad) > 1e-6:
            # 後輪中心の旋回半径
            radius = self.wheelbase / math.tan(steer_rad)
            yaw_rate = self.speed / radius          # [rad/s] CCW正
        else:
            yaw_rate = 0.0

        # 後輪中心（車両基準点）の運動
        self.x += self.speed * math.cos(heading_rad) * dt
        self.y += self.speed * math.sin(heading_rad) * dt
        self.heading = (self.heading + math.degrees(yaw_rate * dt)) % 360.0

        # --- 加速度（速度差分） ---
        self.accel = (self.speed - self._prev_speed) / dt
        self._prev_speed = self.speed

        self._publish()

    def get_inner_outer_wheel_speed(self) -> tuple[float, float]:
        """内輪・外輪の速度差を返す（参考値）。(inner, outer) [m/s]。"""
        steer_rad = math.radians(self.steer)
        if abs(steer_rad) < 1e-6:
            return self.speed, self.speed
        radius = abs(self.wheelbase / math.tan(steer_rad))
        omega = self.speed / radius
        inner = omega * (radius - self.tread / 2.0)
        outer = omega * (radius + self.tread / 2.0)
        return inner, outer

    def _publish(self) -> None:
        state = VehicleState(
            x=self.x, y=self.y, heading=self.heading,
            speed=self.speed, acceleration=self.accel,
            steer_angle=self.steer, timestamp=time.time(),
        )
        self.shared.update_vehicle(state)
