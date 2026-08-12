"""車両モデル — 自転車モデル + 操舵のむだ時間 + 1次遅れ。

**運動学と遅れだけ。** タイヤモデル・スリップ・TC/TV は入れていない。
実車の車輪半径・ギア比・ホイールベースが未実測（`architecture.md` §15 #3/#4）で、
合わせ込む相手がまだ無いため。入れるなら実測後。

指令は SI で受ける（整数スケールの解釈は `sim/stm32.py` の仕事）。
"""

from __future__ import annotations

import math
import tomllib
from collections import deque
from dataclasses import dataclass, field
from pathlib import Path

__all__ = ["VehicleSpec", "VehicleModel", "DriveInput", "DEFAULT_SPEC_PATH"]

DEFAULT_SPEC_PATH = Path(__file__).resolve().parents[1] / "config" / "vehicle.toml"


@dataclass
class VehicleSpec:
    """`config/vehicle.toml` の中身。**値はほぼ未実測の暫定値。**"""

    wheelbase: float = 0.25
    track: float = 0.19
    max_steer: float = 1.047
    wheel_radius: float = 0.033
    mass: float = 1.6
    footprint: list = field(default_factory=lambda: [
        [0.310, 0.095], [0.310, -0.095], [-0.030, -0.095], [-0.030, 0.095]])

    tau_steer_s: float = 0.12
    dead_time_s: float = 0.030
    tau_speed_s: float = 0.35
    rolling_resistance: float = 0.35
    drive_ratio: float = 2.0

    sensors: dict = field(default_factory=dict)

    @classmethod
    def load(cls, path: str | Path = DEFAULT_SPEC_PATH) -> "VehicleSpec":
        with open(path, "rb") as fp:
            d = tomllib.load(fp)
        dyn = d.get("dynamics", {})
        return cls(
            wheelbase=d.get("wheelbase", 0.25),
            track=d.get("track", 0.19),
            max_steer=d.get("max_steer", 1.047),
            wheel_radius=d.get("wheel_radius", 0.033),
            mass=d.get("mass", 1.6),
            footprint=d.get("footprint", cls().footprint),
            tau_steer_s=dyn.get("tau_steer_s", 0.12),
            dead_time_s=dyn.get("dead_time_s", 0.030),
            tau_speed_s=dyn.get("tau_speed_s", 0.35),
            rolling_resistance=dyn.get("rolling_resistance", 0.35),
            drive_ratio=dyn.get("drive_ratio", 2.0),
            sensors=d.get("sensors", {}),
        )

    def sensor_pose(self, name: str) -> tuple[float, float, float]:
        """センサの base_link からの (x, y, yaw)。未定義なら原点。"""
        s = self.sensors.get(name, {})
        return float(s.get("x", 0.0)), float(s.get("y", 0.0)), float(s.get("yaw", 0.0))


@dataclass
class DriveInput:
    """`COMMAND` を SI に開いたもの。"""

    armed: bool = False
    brake: bool = False
    torque_mode: bool = False
    target_speed: float = 0.0        # m/s
    target_steer: float = 0.0        # rad（路面舵角）
    accel_limit: float = 0.0         # m/s²。0 = 制限なし
    steer_rate_limit: float = 0.0    # rad/s。0 = 制限なし
    brake_torque: float = 0.0        # N·m（後輪各輪）。0 = 未指定 = 最大制動
    target_torque: float = 0.0       # N·m（駆動トルク直接指令、v0.6）


class VehicleModel:
    """真値の状態を持つ。座標は世界系（x = 東、y = 北、yaw = 反時計回り）。"""

    #: `brake_torque = 0`（未指定）のときに使う最大制動トルク [N·m]。
    #: `convert.py` の MAX_BRAKE_TORQUE_NM と同値
    MAX_BRAKE_TORQUE_NM = 0.125

    def __init__(self, spec: VehicleSpec, start: tuple[float, float, float]) -> None:
        self.spec = spec
        self.reset(start)

    def reset(self, start: tuple[float, float, float]) -> None:
        self.x, self.y, self.yaw = start
        self.speed = 0.0
        self.steer_actual = 0.0
        self.yaw_rate = 0.0
        self.accel_x = 0.0
        self.odom_front = [0.0, 0.0]     # [FL, FR] 累積 [m]（射影なし = 前輪の実距離）
        self.collided = False
        self.collisions = 0
        self.cmd = DriveInput()
        #: 操舵のむだ時間を作る待ち行列。(残り時間, 指令値)
        self._steer_delay: deque[tuple[float, float]] = deque()
        self._steer_delayed = 0.0

    # ── 指令 ──

    def apply(self, cmd: DriveInput) -> None:
        self.cmd = cmd

    # ── 積分 ──

    def step(self, dt: float) -> None:
        sp = self.spec
        cmd = self.cmd

        # ── 操舵: むだ時間 → 1次遅れ → レート制限 ──
        target = max(-sp.max_steer, min(sp.max_steer, cmd.target_steer if cmd.armed else 0.0))
        self._steer_delay.append((sp.dead_time_s, target))
        self._steer_delayed = self._pop_delayed(dt)

        want = self._steer_delayed
        if sp.tau_steer_s > 1e-6:
            want = self.steer_actual + (want - self.steer_actual) * (1.0 - math.exp(-dt / sp.tau_steer_s))
        rate_cap = cmd.steer_rate_limit if cmd.steer_rate_limit > 0 else None
        if rate_cap is not None:
            d = max(-rate_cap * dt, min(rate_cap * dt, want - self.steer_actual))
            self.steer_actual += d
        else:
            self.steer_actual = want

        # ── 速度 ──
        prev_speed = self.speed
        self.speed = self._next_speed(dt)

        # ── 運動（自転車モデル。原点は後輪車軸中心） ──
        self.yaw_rate = self.speed / sp.wheelbase * math.tan(self.steer_actual)
        self.yaw = (self.yaw + self.yaw_rate * dt + math.pi) % (2 * math.pi) - math.pi
        self.x += self.speed * math.cos(self.yaw) * dt
        self.y += self.speed * math.sin(self.yaw) * dt
        self.accel_x = (self.speed - prev_speed) / dt if dt > 0 else 0.0

        # 前輪の走行距離は 1/cos(δ) 倍（`uart_protocol.md` §5.3 の射影の逆）
        d_front = abs(self.speed) * dt / max(0.1, math.cos(self.steer_actual))
        sgn = 1.0 if self.speed >= 0 else -1.0
        self.odom_front[0] += d_front * sgn
        self.odom_front[1] += d_front * sgn

    def _pop_delayed(self, dt: float) -> float:
        """むだ時間ぶん寝かせた指令を取り出す。"""
        out = self._steer_delayed
        q = self._steer_delay
        while q and q[0][0] <= 0.0:
            out = q.popleft()[1]
        for i, (remain, val) in enumerate(q):
            q[i] = (remain - dt, val)
        return out

    def _next_speed(self, dt: float) -> float:
        sp = self.spec
        cmd = self.cmd
        v = self.speed

        if not cmd.armed:
            # ARM されていなければ惰行して止まる（駆動は切れているが慣性はある）
            return _toward_zero(v, sp.rolling_resistance * 4.0 * dt)

        if cmd.brake:
            nm = cmd.brake_torque if cmd.brake_torque > 0 else self.MAX_BRAKE_TORQUE_NM
            decel = nm * sp.drive_ratio / (sp.wheel_radius * sp.mass)
            return _toward_zero(v, decel * dt)

        if cmd.torque_mode:
            # トルク直接指令（v0.6）。**無視すると「スポーティなラジコンモード」で
            # 車が一切動かない**ので、加速度への換算だけは入れてある
            a = cmd.target_torque * sp.drive_ratio / (sp.wheel_radius * sp.mass)
            a -= math.copysign(sp.rolling_resistance, v) if abs(v) > 1e-3 else 0.0
            return v + a * dt

        # 速度指令: 1次遅れで追う。accel_limit があればその範囲で
        target = cmd.target_speed
        if sp.tau_speed_s > 1e-6:
            want = v + (target - v) * (1.0 - math.exp(-dt / sp.tau_speed_s))
        else:
            want = target
        if cmd.accel_limit > 0:
            lim = cmd.accel_limit * dt
            want = v + max(-lim, min(lim, want - v))
        return want

    # ── 衝突 ──

    def note_collision(self, hit: bool) -> None:
        """コース側の判定結果を受けて速度を殺す。**テレポートさせない。**

        壁に押し付けられて止まる挙動にしておくと、「どこでぶつかったか」が
        画面に残る。跳ね返したり戻したりすると原因が見えなくなる。
        """
        if hit and not self.collided:
            self.collisions += 1
        self.collided = hit
        if hit:
            self.speed = 0.0

    # ── 派生量 ──

    @property
    def wheel_speed(self) -> list[float]:
        """[FL, FR, RL, RR]。**射影なし**（前輪は 1/cos(δ) 倍で回る）。"""
        f = self.speed / max(0.1, math.cos(self.steer_actual))
        return [f, f, self.speed, self.speed]

    @property
    def accel_lateral(self) -> float:
        return self.speed * self.yaw_rate


def _toward_zero(v: float, delta: float) -> float:
    if abs(v) <= delta:
        return 0.0
    return v - math.copysign(delta, v)
