"""制御の同定 — タイヤの前後力試験（スリップ率→駆動力・制動力の曲線）。

直線で、**TC と ABS を切って**後輪をわざと滑らせる。駆動トルクを段々に上げて空転するまで、
続けて制動トルクを段々に上げてロックするまで。トルク・後輪の回り方・車速から、各瞬間の
タイヤの前後力 F = (トルク − 慣性×角加速度)/半径 とスリップ率が出るので、その関係
（グリップの上限・立ち上がりの傾き・滑らせたときに力が落ちるか）を当てはめる。解析は
`tools/ctrl_tune/fit.py` の `fit_tyre()`、結果は `config/vehicle.toml` の `[control.plant]`。
**先に後輪の空転試験（`sysid_wheel`）で車輪の慣性を測っておくこと**（力の計算に使う）。

## 手順（`cycles` 回くり返す）

1. 転がす: 0.4m/s まで車速指令で進む（止まった状態はスリップ率が定まらない）
2. 駆動: トルク直接指令で `drive_start_nm` から `drive_step_nm` ずつ上げる（0.1s ごと）。
   後輪が前輪より `spin_end_m_s` 速くなる・最大トルクに達して少し経つ・`v_max` に達する・
   直線が尽きる、のどれかで終える
3. 制動: `brake_start_nm` から `brake_step_nm` ずつ上げる。後輪がロックしたら（前輪の3割以下）
   弱い制動に落として止まる
4. 後退して開始位置へ戻る

## なぜトルク直接指令か

車速指令は目標を最大 3.0m/s² でランプさせるので、タイヤの限界（約3.4m/s²）に届く前に
頭打ちになる。限界を超えるところまでトルクを上げないと、曲線の肩が見えない。

## ファームの設定（試験の間だけ。`AutoState.fw_overrides`）

TC・ABS・TV を切る。片輪浮き対策は入れたまま（後輪の周速の絶対上限 7.5m/s が歯止めに残る。
左右を同じトルクで滑らせる間は左右差が小さく、働かない想定——働いたら解析が★で知らせる）。

## 安全

- 直線 `run_length_m`（2m）＋車長が要る。止まりきれなくなる地点で制動に移る
- 後輪の空転は 100Hz で見て `spin_end_m_s` で打ち切る
"""

from __future__ import annotations

from ..msgs.types import AutoState, Scan, VehicleState
from ._sysid_common import OdomRun, TestGate, settle_state
from .base import Planner

__all__ = ["SysIdTyre", "OVERRIDES"]

OVERRIDES = {"tc_enable": 0.0, "abs_enable": 0.0, "tv_enable": 0.0}
_MAX_TORQUE_NM = 0.15


#: 経過時間の比較の余裕 [s]（0.1s の足し算の丸め誤差で判断が1周期ずれないように）
_EPS = 1e-6


class SysIdTyre(Planner):
    id = "sysid_tyre"
    name = "制御の同定: タイヤの前後力"
    description = ("直線で TC・ABS を切り、駆動トルクを空転するまで・制動トルクをロックするまで段々に上げる"
                   "（3往復）。スリップ率→駆動力・制動力の曲線を測る。直線2m＋車長が要る。約25秒")
    category = "sysid"
    stats = ()

    SETTINGS: dict[str, float] = {
        "cycles": 3, "run_length_m": 2.0, "v_max": 2.0,
        "roll_speed": 0.4, "roll_s": 0.6,
        "drive_start_nm": 0.06, "drive_step_nm": 0.015, "spin_end_m_s": 1.5, "max_hold_s": 0.2,
        # 制動は 0.07→0.10→0.13→0.15 と 0.3s で最大へ（ゆっくり上げると、ロックする前に車が止まる）
        "brake_start_nm": 0.07, "brake_step_nm": 0.03, "gentle_brake_nm": 0.05,
        "stop_hold_s": 0.5, "return_speed": 0.6,
    }
    params = ()

    _ROLL, _DRIVE, _BRAKE, _HOLD, _RETURN = range(5)
    _LABELS = {_ROLL: "転がす", _DRIVE: "駆動", _BRAKE: "制動", _HOLD: "停止", _RETURN: "戻り"}
    _STOPPED_M_S = 0.03
    _PHASE_TIMEOUT_S = 4.0
    #: 後輪が前輪のこの割合より遅ければロック
    _LOCK_RATIO = 0.3
    _LOCK_MIN_FRONT_M_S = 0.5

    def __init__(self) -> None:
        self.settings = dict(self.SETTINGS)
        self._gate = TestGate()
        self._run = OdomRun()
        self._reset_state()

    def _reset_state(self) -> None:
        self._t = 0.0
        self._cycle = 0
        self._phase = self._ROLL
        self._steps = 0             # この区間で上げた段の数
        self._at_max_s = 0.0
        self._spun = False
        self._locked = False
        self._run = OdomRun()

    def reset(self) -> None:
        self._gate.reset()
        self._reset_state()

    def set_engaged(self, engaged: bool) -> None:
        self._gate.set_engaged(engaged)

    def on_vehicle_state(self, vs: VehicleState) -> None:
        """`planning_node` が全サンプル（100Hz）で呼ぶ。空転・ロックを覚える。"""
        if len(vs.wheel_speed) < 4:
            return
        rear_fast = max(vs.wheel_speed[2], vs.wheel_speed[3])
        rear_slow = min(vs.wheel_speed[2], vs.wheel_speed[3])
        if self._phase == self._DRIVE and rear_fast - vs.speed > self.settings["spin_end_m_s"]:
            self._spun = True
        if (self._phase == self._BRAKE and vs.speed > self._LOCK_MIN_FRONT_M_S
                and rear_slow < self._LOCK_RATIO * vs.speed):
            self._locked = True

    def _enter(self, phase: int) -> None:
        self._phase = phase
        self._t = 0.0
        self._steps = 0
        self._at_max_s = 0.0
        self._spun = False
        self._locked = False

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
        st.fw_overrides = dict(OVERRIDES)
        if self._gate.settling(dt):
            self._run.start(vs)
            return settle_state(st)
        self._run.start(vs)
        n = int(p["cycles"])
        if self._cycle >= n:
            st.reason = "完了"
            return st

        self._t += dt
        v = vs.speed
        out_of_road = self._run.must_stop(vs, p["run_length_m"], v)
        if self._phase == self._ROLL:
            done = self._t >= p["roll_s"] - _EPS or out_of_road
        elif self._phase == self._DRIVE:
            done = (self._spun or self._at_max_s >= p["max_hold_s"] - _EPS or v >= p["v_max"] or out_of_road
                    or self._t >= self._PHASE_TIMEOUT_S - _EPS)
        elif self._phase == self._BRAKE:
            done = abs(v) <= self._STOPPED_M_S or self._t >= self._PHASE_TIMEOUT_S - _EPS
        elif self._phase == self._HOLD:
            done = self._t >= p["stop_hold_s"] - _EPS
            if done:
                self._run.arm_return(vs, p["return_speed"])
        else:
            done = self._run.returned(vs, self._t)
        if done:
            nxt = (self._phase + 1) % 5
            self._enter(nxt)
            if nxt == self._ROLL:
                self._cycle += 1
                if self._cycle >= n:
                    st.reason = "完了"
                    return st

        if self._phase == self._ROLL:
            st.target_speed = p["roll_speed"]
            st.accel_limit = 2.0
        elif self._phase == self._DRIVE:
            torque = min(_MAX_TORQUE_NM, p["drive_start_nm"] + p["drive_step_nm"] * self._steps)
            if torque >= _MAX_TORQUE_NM:
                self._at_max_s += dt
            self._steps += 1
            st.torque_mode = True
            st.target_torque = torque
        elif self._phase == self._BRAKE:
            st.brake = True
            if self._locked:
                st.brake_torque = p["gentle_brake_nm"]
            else:
                st.brake_torque = min(_MAX_TORQUE_NM, p["brake_start_nm"] + p["brake_step_nm"] * self._steps)
                self._steps += 1
        elif self._phase == self._HOLD:
            st.brake = True
        else:
            st.target_speed = -p["return_speed"]
        detail = ""
        if self._phase == self._DRIVE:
            detail = f"（{st.target_torque:.3f}N·m）"
        elif self._phase == self._BRAKE:
            detail = f"（{st.brake_torque:.2f}N·m{'・ロック後' if self._locked else ''}）"
        st.reason = f"{self._LABELS[self._phase]}{detail} {self._cycle + 1}/{n}"
        return st
