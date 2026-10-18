"""システム同定 — 車速PI試験（トルク指令で前進・後退を往復し、車速PIを通さずに車体を測る）。

車速PIのゲイン（`[dynamics]` の `speed_kp`・`speed_ki`）を決めるための試験。PI を迂回してトルクを直接掛ける
（`torque_mode`）ので、測った応答に PI のゲインが混ざらない（② 前後運動試験は PI を通した応答に車体を
当てはめるので、PI のゲインを既知として持ち込む）。解析は `tools/sysid/speed_pi.py`。

## 1回の片道（前進・後退とも同じ。後退はトルクの符号が逆）

1. **加速**: トルク `torque_a`／`torque_b`（サイクルで交互、後輪各輪 [N·m]）で `v_max` まで → 車体の利得
2. **揺さぶり**: 転がり抵抗を打ち消すトルクのまわりに ±`dither_nm` を擬似乱数（M系列）で切り替える
   （`prbs_s` の間、1ビット `prbs_bit_s`）→ トルクから加速度までの遅れ（MD のトルクの立ち上がり・むだ時間）。
   車速は ±数cm/s しか動かない。`plan()` はスキャンごと（10Hz）にしか呼ばれないので、1ビットは 0.1s より
   短くしても意味が無い
3. **惰行**: トルク 0 で `coast_s` → 転がり抵抗（前進・後退で別に）
4. 制動して止まり、`hold_s` 待つ

舵は置いた向きの直線へ戻る弱いフィードバック（`sysid_encoder` と同じ）。

## 必要なスペース

直線 `run_length_m`（2m）＋車長。片道の途中で直線の端に止まりきれなくなったら、その時点で制動する。
"""

from __future__ import annotations

from ..core.vehicle import Vehicle
from ..msgs.types import AutoState, Scan, VehicleState
from ._sysid_common import DeadReckon, SensorGuard, TestGate, abort_state, settle_state, stop_distance
from .base import Planner

__all__ = ["SysIdSpeedPi", "prbs"]

_EPS = 1e-6


def prbs(n_bits: int, seed: int = 0x5A) -> list[int]:
    """7段の M 系列（x⁷+x⁶+1、周期127）の先頭 `n_bits` ビット（0/1）。毎回同じ並び（記録を見比べられる）。"""
    s = seed & 0x7F or 1
    out = []
    for _ in range(n_bits):
        bit = ((s >> 6) ^ (s >> 5)) & 1
        s = ((s << 1) | bit) & 0x7F
        out.append(bit)
    return out


class SysIdSpeedPi(Planner):
    id = "sysid_speed_pi"
    name = "システム同定: ⑤ 車速PI試験（トルク往復）"
    description = ("トルクを直接掛けて直線を往復する（3往復）。車速PIを通さずに車体の利得・転がり抵抗（前後別）・"
                   "トルクの遅れを測り、車速PIのゲインを決める。直線2m＋車長。約35秒")
    category = "sysid"
    stats = ()

    SETTINGS: dict[str, float] = {
        "cycles": 3, "run_length_m": 2.0,
        # 加速のトルク [N·m、後輪各輪]（サイクルで交互）と、加速をやめる車速 [m/s]
        "torque_a": 0.03, "torque_b": 0.06, "v_max": 1.0,
        # 揺さぶり: 転がり抵抗を打ち消すトルク [N·m、各輪] のまわりの振れ幅と長さ・1ビットの長さ [s]
        "hold_nm": 0.0063, "dither_nm": 0.02, "prbs_s": 1.2, "prbs_bit_s": 0.1,
        "coast_s": 0.5,
        "hold_s": 1.0,
        "keep_steer": 0.05,
    }
    params = ()

    _ACCEL, _PRBS, _COAST, _STOP, _HOLD = range(5)
    _LABELS = {_ACCEL: "加速", _PRBS: "揺さぶり", _COAST: "惰行", _STOP: "停止", _HOLD: "静止"}
    _STOPPED_M_S = 0.02
    _STOP_TIMEOUT_S = 3.0
    _LEG_TIMEOUT_S = 10.0

    def __init__(self) -> None:
        self.settings = dict(self.SETTINGS)
        self._gate = TestGate()
        self._guard = SensorGuard()
        self._dr = DeadReckon()
        self.vehicle = Vehicle.load()
        self._bits = prbs(127)
        self._reset_state()

    def _reset_state(self) -> None:
        self._t = 0.0
        self._leg_t = 0.0
        self._leg = 0
        self._phase = self._ACCEL
        self._leg_start: float | None = None
        self._origin_set = False
        self._guard.reset()
        self._dr = DeadReckon()

    def reset(self) -> None:
        self._gate.reset()
        self._reset_state()

    def set_engaged(self, engaged: bool) -> None:
        self._gate.set_engaged(engaged)

    def on_vehicle_state(self, vs: VehicleState) -> None:
        self._guard.update(vs)
        self._dr.update(vs)

    def _enter(self, phase: int) -> None:
        self._phase = phase
        self._t = 0.0

    def plan(self, scan: Scan, vs: VehicleState | None,
             p: dict[str, float], dt: float) -> AutoState:
        p = {**self.settings, **p}
        st = AutoState(mode=self.id, planner=self.name)
        st.ready = True

        engaged, armed, just_started = self._gate.tick(vs)
        if just_started:
            self._reset_state()
        if not engaged:
            st.reason = "試験開始を押してください"
            return st
        if not armed:
            st.reason = "ARM待ち（Enterを押してください）"
            return st
        assert vs is not None
        if self._guard.fault is not None:
            return abort_state(st, self._guard.fault)
        if self._gate.settling(dt):
            self._dr.stationary = True
            return settle_state(st)
        if not self._origin_set:
            self._dr.reset_origin()
            self._origin_set = True

        n_legs = 2 * int(p["cycles"])
        if self._leg >= n_legs:
            self._dr.stationary = True
            st.reason = "完了"
            return st
        odom = (vs.odom_dist[0] + vs.odom_dist[1]) / 2.0
        if self._leg_start is None:
            self._leg_start = odom
            self._leg_t = 0.0

        self._t += dt
        self._leg_t += dt
        forward = self._leg % 2 == 0
        sign = 1.0 if forward else -1.0
        torque = p["torque_a"] if (self._leg // 2) % 2 == 0 else p["torque_b"]
        traveled = sign * (odom - self._leg_start)
        v = abs(vs.speed)
        # 加速中は止まるまでにまだ加速が乗る（トルクの加速度で見積もる）
        accel_now = 3.0 if self._phase == self._ACCEL else 0.0
        out_of_road = traveled + stop_distance(v, True, accel_now) >= p["run_length_m"]

        if self._phase in (self._ACCEL, self._PRBS, self._COAST):
            if out_of_road or self._leg_t >= self._LEG_TIMEOUT_S - _EPS:
                self._enter(self._STOP)
            elif self._phase == self._ACCEL and v >= p["v_max"]:
                self._enter(self._PRBS)
            elif self._phase == self._PRBS and self._t >= p["prbs_s"] - _EPS:
                self._enter(self._COAST)
            elif self._phase == self._COAST and self._t >= p["coast_s"] - _EPS:
                self._enter(self._STOP)
        elif self._phase == self._STOP:
            if v <= self._STOPPED_M_S or self._t >= self._STOP_TIMEOUT_S - _EPS:
                self._enter(self._HOLD)
        elif self._phase == self._HOLD and self._t >= p["hold_s"] - _EPS:
            self._leg += 1
            self._leg_start = None
            self._enter(self._ACCEL)
            if self._leg >= n_legs:
                self._dr.stationary = True
                st.reason = "完了"
                return st
            forward = self._leg % 2 == 0
            sign = 1.0 if forward else -1.0
            torque = p["torque_a"] if (self._leg // 2) % 2 == 0 else p["torque_b"]

        self._dr.stationary = self._phase == self._HOLD
        L = self.vehicle.wheelbase
        if self._phase in (self._ACCEL, self._PRBS, self._COAST):
            st.torque_mode = True
            if self._phase == self._ACCEL:
                nm = torque
            elif self._phase == self._PRBS:
                bit = self._bits[int(self._t / p["prbs_bit_s"]) % len(self._bits)]
                nm = p["hold_nm"] + (p["dither_nm"] if bit else -p["dither_nm"])
            else:
                nm = 0.0
            st.target_torque = sign * nm
            st.target_steer = self._dr.lane_keep_steer(forward, p["keep_steer"], L)
        else:
            st.brake = True
            st.target_steer = 0.0
        st.reason = (f"{self._LABELS[self._phase]}（{'前進' if forward else '後退'} {torque:.2f}N·m）"
                     f" {self._leg // 2 + 1}/{int(p['cycles'])}")
        return st
