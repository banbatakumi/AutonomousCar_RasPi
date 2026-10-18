"""制御の同定 — タイヤの前後力試験（スリップ率→駆動力・制動力の曲線）。

直線で、**TC と ABS を切って**後輪をわざと滑らせる。駆動トルクを段々に上げて空転するまで、
続けて制動トルクを段々に上げてロックするまで。トルク・後輪の回り方・車速から、各瞬間の
タイヤの前後力 F = (トルク − 慣性×角加速度)/半径 とスリップ率が出るので、その関係
（グリップの上限・立ち上がりの傾き・滑らせたときに力が落ちるか）を当てはめる。解析は
`tools/ctrl_tune/fit.py` の `fit_tyre()`、結果は `config/vehicle.toml` の `[control.plant]`。
**先に後輪の空転試験（`sysid_wheel`）で車輪の慣性を測っておくこと**（力の計算に使う）。

## 手順（`cycles` 回の「上げる」往復のあと、`hold_cycles` 回の「保つ」往復）

1. 転がす: 0.4m/s まで車速指令で進む（止まった状態はスリップ率が定まらない）。この区間は後輪の周速の
   個体差（左右のスリップ率のずれ）の校正にも使う
2. 駆動: **上げる往復**はトルク直接指令で `drive_start_nm` から `drive_step_nm` ずつ上げる（0.1s ごと）。
   後輪が前輪より `spin_end_m_s` 速くなる・最大トルクに達して少し経つ・`v_max` に達する・
   直線が尽きる、のどれかで終える。**保つ往復**は限界の下の一定トルクを `hold_s` ずつ、
   `HOLD_DRIVE_FRACTIONS` の割合（`drive_limit_nm` に対して）で保つ
3. 制動: 同様に、上げる往復は `brake_start_nm` から `brake_step_nm` ずつ上げる。後輪がロックしたら
   （前輪の3割以下）弱い制動に落として止まる。保つ往復は `HOLD_BRAKE_FRACTIONS`（`brake_limit_nm`）
4. 後退して開始位置へ戻る

## なぜ「保つ」区間が要るか（スリップ剛性 B を測るため）

TC・ABS のゲインは、トルク→スリップ→タイヤの力というループの利得（＝限界の下での力の立ち上がり、
スリップ剛性 B）で決まる。しかしトルクを上げて空転・ロックさせるだけでは、車輪の慣性が小さい（2.4e-5kg·m²）
ので限界で暴走し、立ち上がり（スリップ率 0〜0.1）を 1〜2 サンプルで通り過ぎる。スリップ率の測定誤差
（約 0.03）が立ち上がりの幅と同じで、B が 6〜80 のどれでも同じ記録に合ってしまった（2026-10-18 の実機）。
限界の下で一定トルクを保てば、暴走せず、スリップ率が定常になるので平均でき（誤差 0.003 前後）、
力との関係から B が決まる。空転・ロックまで上げる往復は、力の上限（μ）と限界の先の形に使う。

## なぜトルク直接指令か

車速指令は目標を最大 3.0m/s² でランプさせるので、タイヤの限界（約3.4m/s²）に届く前に
頭打ちになる。限界を超えるところまでトルクを上げないと、曲線の肩が見えない。

## ファームの設定（試験の間だけ。`AutoState.fw_overrides`）

TC・ABS・TV・片輪浮き対策を切る。片輪浮き対策は、左右の後輪の速度差が `wheel_lift_diff_threshold_m_s`
を超えると空転側のトルクを絞る。左右を同じトルクで滑らせても限界では片方の輪が先に空転するので、
実機（2026-10-17 の記録）では駆動の約4割の区間で働き、限界付近の駆動力を歪めた。素のタイヤ特性を
測るので切る。後輪の周速の絶対上限（7.5m/s）の歯止めも一緒に外れるが、空転は 100Hz で見て
`spin_end_m_s` で打ち切る（駆動は最大 0.13N·m で、判断の遅れ 0.1s の増速は約 1.2m/s）。

## 安全

- 直線 `run_length_m`（2m）＋車長が要る。止まりきれなくなる地点（判断の遅れの間の加速ぶんも
  入れた見積もり）で制動に移る
- 後輪の空転は 100Hz で見て `spin_end_m_s` で打ち切る
"""

from __future__ import annotations

from ..msgs.types import AutoState, Scan, VehicleState
from ._sysid_common import (OdomRun, SensorGuard, TestGate, abort_state, brake_decel_guess, settle_state,
                            stop_distance)
from .base import Planner

__all__ = ["SysIdTyre", "OVERRIDES", "HOLD_DRIVE_FRACTIONS", "HOLD_BRAKE_FRACTIONS"]

OVERRIDES = {"tc_enable": 0.0, "abs_enable": 0.0, "tv_enable": 0.0, "wheel_lift_guard_enable": 0.0}
_MAX_TORQUE_NM = 0.13
#: 保つ往復の段（限界のトルクに対する割合）。往復ごとに駆動3段・制動2段。低い段ほどスリップ率が小さく、
#: B に効く。駆動を3段にするのは、制動に入る車速を上げて、制動の2段目も使える速さ（0.35m/s 超）に
#: 残すため
HOLD_DRIVE_FRACTIONS = ((0.30, 0.50, 0.70), (0.55, 0.78, 0.92), (0.40, 0.60, 0.85), (0.65, 0.85, 0.92))
HOLD_BRAKE_FRACTIONS = ((0.45, 0.65), (0.75, 0.90), (0.55, 0.70), (0.85, 0.92))
#: 止まりきれなくなる地点の見積もりに入れる、判断の遅れの間の加速度 [m/s²]。駆動の区間は TC を
#: 切ったトルク直接指令なので、制動に移ると決めてから効くまでの間もタイヤの限界で加速し続ける
#: （ファームの目標ランプの上限と同じ値。測る対象そのものなので `vehicle.toml` に頼らない）。
#: 転がす区間は `accel_limit` の値
_DRIVE_ACCEL_FOR_STOP_M_S2 = 3.0
_ROLL_ACCEL_M_S2 = 2.0


#: 経過時間の比較の余裕 [s]（0.1s の足し算の丸め誤差で判断が1周期ずれないように）
_EPS = 1e-6


class SysIdTyre(Planner):
    id = "sysid_tyre"
    name = "制御の同定: タイヤの前後力"
    description = ("直線で TC・ABS を切り、駆動トルクを空転するまで・制動トルクをロックするまで段々に上げる"
                   "（2往復）のあと、限界の下の一定トルクを保つ往復（4往復）。スリップ剛性・上限・形を測る。直線2m＋車長が要る。約50秒")
    category = "sysid"
    stats = ()

    SETTINGS: dict[str, float] = {
        "cycles": 2, "hold_cycles": 4, "run_length_m": 2.0, "v_max": 2.0,
        "roll_speed": 0.4, "roll_s": 0.8,
        # 保つ往復: 1段を保つ時間 [s] と、段の割合を掛ける限界のトルク [N·m、1輪]（2026-10-18 の実機の
        # 駆動 約0.078・制動 約0.07。段の置き場所を決めるだけで、同定値は記録から求める）
        "hold_s": 0.3, "drive_limit_nm": 0.075, "brake_limit_nm": 0.07,
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
        #: MD の無応答だけ見る。前輪と後輪の照合はしない（後輪をわざと空転・ロックさせる試験）
        self._guard = SensorGuard(wheel_check=False)
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
        self._guard.reset()

    def reset(self) -> None:
        self._gate.reset()
        self._reset_state()

    def set_engaged(self, engaged: bool) -> None:
        self._gate.set_engaged(engaged)

    def on_vehicle_state(self, vs: VehicleState) -> None:
        """`planning_node` が全サンプル（100Hz）で呼ぶ。空転・ロックを覚える。"""
        self._guard.update(vs)
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
        if self._guard.fault is not None:
            return abort_state(st, self._guard.fault)
        if self._gate.settling(dt):
            self._run.start(vs)
            return settle_state(st)
        self._run.start(vs)
        n_ramp = int(p["cycles"])
        n = n_ramp + int(p["hold_cycles"])
        if self._cycle >= n:
            st.reason = "完了"
            return st

        self._t += dt
        v = vs.speed
        # 加速中の区間は、判断の遅れの間の増速ぶんも止まる距離に入れる（入れないと直線をはみ出す）
        accel_now = {self._ROLL: _ROLL_ACCEL_M_S2, self._DRIVE: _DRIVE_ACCEL_FOR_STOP_M_S2}.get(self._phase, 0.0)
        # ロック後は弱い制動（gentle_brake_nm）で止まるので、その強さの見積もり（保守側）で測る
        decel = (brake_decel_guess(p["gentle_brake_nm"])
                 if self._phase == self._BRAKE and self._locked else None)
        stop_d = (stop_distance(v, True, accel_now) if decel is None
                  else stop_distance(v, True, accel_now, decel))
        out_of_road = self._run.traveled(vs) + stop_d >= p["run_length_m"]
        # 保つ往復: 段を保つ時間（この判断の指令が効いている時間を数えるので、今の判断の dt は含めない）
        hold = self._cycle >= n_ramp
        j = (self._cycle - n_ramp) if hold else 0
        elapsed = self._t - dt
        d_fr, b_fr = HOLD_DRIVE_FRACTIONS[j % len(HOLD_DRIVE_FRACTIONS)], HOLD_BRAKE_FRACTIONS[j % len(HOLD_BRAKE_FRACTIONS)]
        if self._phase == self._ROLL:
            done = self._t >= p["roll_s"] - _EPS or out_of_road
        elif self._phase == self._DRIVE:
            done = (self._spun or v >= p["v_max"] or out_of_road or self._t >= self._PHASE_TIMEOUT_S - _EPS
                    or (elapsed >= len(d_fr) * p["hold_s"] - _EPS if hold
                        else self._at_max_s >= p["max_hold_s"] - _EPS))
        elif self._phase == self._BRAKE:
            # ロック後の弱い制動で直線を出そうなら、止まる局面（HOLD＝最大制動）へ早めに移る
            done = (abs(v) <= self._STOPPED_M_S or self._t >= self._PHASE_TIMEOUT_S - _EPS
                    or (self._locked and out_of_road)
                    or (hold and not self._locked and elapsed >= len(b_fr) * p["hold_s"] - _EPS))
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
            st.accel_limit = _ROLL_ACCEL_M_S2
        elif self._phase == self._DRIVE:
            if hold:
                level = min(int(max(self._t - dt, 0.0) / p["hold_s"] + _EPS), len(d_fr) - 1)
                torque = d_fr[level] * p["drive_limit_nm"]
            else:
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
            elif hold:
                level = min(int(max(self._t - dt, 0.0) / p["hold_s"] + _EPS), len(b_fr) - 1)
                st.brake_torque = b_fr[level] * p["brake_limit_nm"]
                self._steps += 1
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
        st.reason = f"{self._LABELS[self._phase]}{detail}{'（保つ）' if hold and self._phase in (self._DRIVE, self._BRAKE) else ''} {self._cycle + 1}/{n}"
        return st
