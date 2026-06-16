"""アッカーマンジオメトリに基づく車両物理演算。

自転車モデル(bicycle model)をベースに、アッカーマン幾何による内輪・外輪の
舵角差・速度差を計算する。速度・ステアにはそれぞれ一次遅れを与え、実機の
応答特性を近似する。

座標系: heading は [deg] 0=East、反時計回り正。
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from core.interfaces import VehicleState


@dataclass
class WheelStatus:
    """アッカーマン幾何による各輪の舵角・速度（表示・解析用）。"""

    inner_steer: float = 0.0   # [deg] 内輪舵角
    outer_steer: float = 0.0   # [deg] 外輪舵角
    inner_speed: float = 0.0   # [m/s] 内輪速度
    outer_speed: float = 0.0   # [m/s] 外輪速度
    turn_radius: float = float("inf")  # [m] 重心の旋回半径


class AckermannModel:
    """アッカーマン車両モデル。"""

    def __init__(self, config: dict, start_pose: tuple[float, float, float] | None = None):
        v = config["vehicle"]
        self.wheelbase: float = float(v["wheelbase"])
        self.tread: float = float(v["tread"])
        self.max_steer_angle: float = float(v["max_steer_angle"])
        self.max_speed: float = float(v["max_speed"])
        self.wheel_radius: float = float(v["wheel_radius"])
        self.speed_tau: float = float(v["speed_time_constant"])
        self.steer_tau: float = float(v["steer_time_constant"])

        self._start_pose = start_pose if start_pose is not None else (0.0, 0.0, 0.0)

        # 指令値（一次遅れの目標）
        self._target_speed: float = 0.0
        self._target_steer: float = 0.0

        self.state = VehicleState()
        self.wheels = WheelStatus()
        self.reset(self._start_pose)

    # ------------------------------------------------------------------
    def reset(self, start_pose: tuple[float, float, float] | None = None) -> None:
        """状態を初期化する。"""
        if start_pose is not None:
            self._start_pose = start_pose
        x, y, heading = self._start_pose
        self.state = VehicleState(
            x=x, y=y, heading=heading,
            speed=0.0, acceleration=0.0, steer_angle=0.0, timestamp=0.0,
        )
        self._target_speed = 0.0
        self._target_steer = 0.0
        self.wheels = WheelStatus()

    # ------------------------------------------------------------------
    def set_command(self, target_speed: float, target_steer: float) -> None:
        """目標速度[m/s]・目標ステア角[deg]を設定する（範囲制限あり）。"""
        self._target_speed = max(-self.max_speed, min(self.max_speed, target_speed))
        self._target_steer = max(-self.max_steer_angle,
                                 min(self.max_steer_angle, target_steer))

    # ------------------------------------------------------------------
    def step(self, dt: float) -> None:
        """状態を dt 秒進める。"""
        if dt <= 0.0:
            return

        st = self.state
        prev_speed = st.speed

        # 一次遅れ: x += (target - x) * (1 - exp(-dt/tau))
        speed_alpha = 1.0 - math.exp(-dt / self.speed_tau) if self.speed_tau > 0 else 1.0
        steer_alpha = 1.0 - math.exp(-dt / self.steer_tau) if self.steer_tau > 0 else 1.0

        speed = prev_speed + (self._target_speed - prev_speed) * speed_alpha
        steer = st.steer_angle + (self._target_steer - st.steer_angle) * steer_alpha

        # 自転車モデルによる旋回
        heading_rad = math.radians(st.heading)
        steer_rad = math.radians(steer)

        if abs(steer_rad) < 1e-6:
            # 直進
            omega = 0.0
            turn_radius = float("inf")
        else:
            turn_radius = self.wheelbase / math.tan(steer_rad)
            omega = speed / turn_radius  # [rad/s]

        heading_rad += omega * dt
        x = st.x + speed * math.cos(heading_rad) * dt
        y = st.y + speed * math.sin(heading_rad) * dt

        acceleration = (speed - prev_speed) / dt

        self.state = VehicleState(
            x=x, y=y,
            heading=math.degrees(heading_rad) % 360.0,
            speed=speed,
            acceleration=acceleration,
            steer_angle=steer,
            timestamp=st.timestamp + dt,
        )

        self._update_wheels(speed, steer_rad, turn_radius)

    # ------------------------------------------------------------------
    def _update_wheels(self, speed: float, steer_rad: float, turn_radius: float) -> None:
        """アッカーマン幾何で内輪・外輪の舵角と速度を計算する。"""
        if abs(steer_rad) < 1e-6 or not math.isfinite(turn_radius):
            self.wheels = WheelStatus(
                inner_steer=0.0, outer_steer=0.0,
                inner_speed=speed, outer_speed=speed,
                turn_radius=float("inf"),
            )
            return

        L = self.wheelbase
        half_t = self.tread / 2.0
        R = abs(turn_radius)  # 重心(後軸)の旋回半径

        # 内輪は旋回中心に近い側、外輪は遠い側
        r_inner = R - half_t
        r_outer = R + half_t

        # 前輪の舵角（アッカーマン条件）
        inner_steer = math.degrees(math.atan2(L, max(r_inner, 1e-6)))
        outer_steer = math.degrees(math.atan2(L, r_outer))

        # 速度は旋回半径に比例
        inner_speed = speed * (r_inner / R) if R > 0 else speed
        outer_speed = speed * (r_outer / R) if R > 0 else speed

        sign = 1.0 if steer_rad > 0 else -1.0
        self.wheels = WheelStatus(
            inner_steer=inner_steer * sign,
            outer_steer=outer_steer * sign,
            inner_speed=inner_speed,
            outer_speed=outer_speed,
            turn_radius=turn_radius,
        )

    # ------------------------------------------------------------------
    def get_state(self) -> VehicleState:
        return self.state
