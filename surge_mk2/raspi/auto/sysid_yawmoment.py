"""制御の同定 — ヨーモーメント試験（左右の駆動力差→ヨーレートの応答）。

直線を一定速で走りながら、左右の後輪のトルクに差を付けて（ヨーモーメントを入れて）、
ヨーレートがどれだけ・どれくらいの速さで付いてくるかを測る。TV（トルクベクタリング）が
相手にしている車の応答そのもので、TV のゲインを決める車両モデルの材料になる。解析は
`tools/ctrl_tune/fit.py` の `fit_yaw()`、結果は `config/vehicle.toml` の `[control.plant]`。

## 手順（`SPEEDS` の各速度で `cycles_per_speed` 回）

1. 車速指令で目標の速度（0.8・1.1m/s。車速PIの行き過ぎで実際は1割ほど速くなる）まで加速し、`settle_s` だけ定速で走る
2. ヨーモーメントを ＋M → −M → ＋M → −M と `pulse_s` ずつ入れる（左右へ同じだけ振るので、
   向きはほぼ元に戻る。M=0.12N·m で付くヨーレートは 0.1rad/s ほど）
3. 止まって、後退で開始位置へ戻る

速度を2つ変えるのは、応答の大きさも速さも車速で変わるため（速いほど大きく・遅く付く）。

## ファームの設定（試験の間だけ。`AutoState.fw_overrides`）

`tv_test_moment_nm` に M を入れる。0 以外の間、ファームは TV の PI を止めてこのヨーモーメント
だけを出す（PI を動かしたままだと入れたモーメントを打ち消しにいく）。TV 自体は有効にしておく。
"""

from __future__ import annotations

from ..msgs.types import AutoState, Scan, VehicleState
from ._sysid_common import OdomRun, SensorGuard, TestGate, abort_state, settle_state
from .base import Planner

__all__ = ["SysIdYawMoment", "SPEEDS", "MOMENT_NM"]

#: 試験する速度 [m/s]
SPEEDS: tuple[float, ...] = (0.8, 1.1)
#: 入れるヨーモーメント [N·m]（ファームの上限 `tv_max_yaw_moment_nm` 0.15 より小さく）
MOMENT_NM = 0.12
#: 1サイクルで入れる向きの並び
_PULSES: tuple[float, ...] = (1.0, -1.0, 1.0, -1.0)


#: 経過時間の比較の余裕 [s]（0.1s の足し算の丸め誤差で判断が1周期ずれないように）
_EPS = 1e-6


class SysIdYawMoment(Planner):
    id = "sysid_yawmoment"
    name = "制御の同定: ヨーモーメント"
    description = ("直線を0.8・1.1m/sの定速で走りながら、左右の後輪のトルクに差を付けて"
                   "ヨーレートの応答を測る（4往復）。直線2m＋車長が要る。約30秒")
    category = "sysid"
    stats = ()

    SETTINGS: dict[str, float] = {
        "cycles_per_speed": 2, "run_length_m": 2.0, "accel_m_s2": 2.4,
        "settle_s": 0.4, "pulse_s": 0.2, "stop_hold_s": 0.5, "return_speed": 0.6,
    }
    params = ()

    _ACCEL, _SETTLE, _PULSE, _DECEL, _HOLD, _RETURN = range(6)
    _LABELS = {_ACCEL: "加速", _SETTLE: "定速", _PULSE: "ヨーモーメント", _DECEL: "減速", _HOLD: "停止",
               _RETURN: "戻り"}
    _STOPPED_M_S = 0.03
    _PHASE_TIMEOUT_S = 4.0

    def __init__(self) -> None:
        self.settings = dict(self.SETTINGS)
        self.speeds = SPEEDS
        self._gate = TestGate()
        self._guard = SensorGuard()
        self._reset_state()

    def _reset_state(self) -> None:
        self._t = 0.0
        self._cycle = 0
        self._phase = self._ACCEL
        self._run = OdomRun()
        self._guard.reset()

    def reset(self) -> None:
        self._gate.reset()
        self._reset_state()

    def set_engaged(self, engaged: bool) -> None:
        self._gate.set_engaged(engaged)

    def on_vehicle_state(self, vs: VehicleState) -> None:
        self._guard.update(vs)

    def n_cycles(self) -> int:
        return len(self.speeds) * int(self.settings["cycles_per_speed"])

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
        st.fw_overrides = {"tv_enable": 1.0}
        if self._guard.fault is not None:
            return abort_state(st, self._guard.fault)
        if self._gate.settling(dt):
            self._run.start(vs)
            return settle_state(st)
        self._run.start(vs)
        n = self.n_cycles()
        if self._cycle >= n:
            st.reason = "完了"
            return st

        self._t += dt
        v = vs.speed
        target = self.speeds[self._cycle // int(p["cycles_per_speed"])]
        out_of_road = self._run.must_stop(vs, p["run_length_m"], v)
        pulses_s = p["pulse_s"] * len(_PULSES)
        if self._phase == self._ACCEL:
            done = v >= target - 0.05 or self._t >= self._PHASE_TIMEOUT_S - _EPS
        elif self._phase == self._SETTLE:
            done = self._t >= p["settle_s"] - _EPS
        elif self._phase == self._PULSE:
            done = self._t >= pulses_s - _EPS
        elif self._phase == self._DECEL:
            done = abs(v) <= self._STOPPED_M_S or self._t >= self._PHASE_TIMEOUT_S - _EPS
        elif self._phase == self._HOLD:
            done = self._t >= p["stop_hold_s"] - _EPS
            if done:
                self._run.arm_return(vs, p["return_speed"])
        else:
            done = self._run.returned(vs, self._t)
        if out_of_road and self._phase in (self._ACCEL, self._SETTLE, self._PULSE):
            self._phase = self._DECEL
            self._t = 0.0
        elif done:
            self._phase = (self._phase + 1) % 6
            self._t = 0.0
            if self._phase == self._ACCEL:
                self._cycle += 1
                if self._cycle >= n:
                    st.reason = "完了"
                    return st
                target = self.speeds[self._cycle // int(p["cycles_per_speed"])]

        detail = ""
        if self._phase in (self._ACCEL, self._SETTLE, self._PULSE):
            st.target_speed = target
            st.accel_limit = p["accel_m_s2"]
            if self._phase == self._PULSE:
                k = min(int((self._t + _EPS) / p["pulse_s"]), len(_PULSES) - 1)
                st.fw_overrides["tv_test_moment_nm"] = _PULSES[k] * MOMENT_NM
                detail = f"（{_PULSES[k] * MOMENT_NM:+.2f}N·m）"
        elif self._phase in (self._DECEL, self._HOLD):
            st.target_speed = 0.0
            st.brake = self._phase == self._HOLD
        else:
            st.target_speed = -p["return_speed"]
        st.reason = f"{self._LABELS[self._phase]}{detail} {target:.1f}m/s {self._cycle + 1}/{n}"
        return st
