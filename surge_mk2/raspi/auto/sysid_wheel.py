"""制御の同定 — 後輪の空転試験（車輪の慣性・摩擦・モータドライバの遅れ）。

**後輪を両方とも浮かせて**（車体を台に載せる。前輪は接地したまま動かないこと）、小さなトルクで
後輪を回しては止める。タイヤが路面に触れていないので、トルクと回り方の関係から車輪
（ホイール＋タイヤ＋ロータ）の慣性と回転の摩擦がそのまま出る。解析は `tools/ctrl_tune/fit.py` の
`fit_wheel()`、結果は `config/vehicle.toml` の `[control.plant]`（TC・ABS のパラメータを決める
車両モデル `tools/ctrl_tune` の材料）。

## 手順

`TORQUES_NM` の各トルクを `spin_s` だけ掛け（トルク直接指令）、弱いブレーキで止め、少し休む。
これを `repeats` 回。小さいトルクから始める——慣性は机上の見積もり（4e-5kg·m²）しか無く、
0.016N·m でも 0.2s で周速 2〜3m/s に達する。

## ファームの設定（試験の間だけ）

- **TC を切る**: TC は前輪の車速を基準にスリップを見るので、止まっている車体で後輪が回れば
  すぐ絞る。TV も切る（左右に同じトルクを掛けたい）
- 片輪浮き対策は**入れたまま**: 左右を同じトルクで回す間は左右差が出ないので働かず、
  後輪の周速の絶対上限（7.5m/s）だけが最後の歯止めとして残る

`AutoState.fw_overrides` に書くと、io_node が試験の指令が届いている間だけ入れ、終われば戻す
（`raspi/core/control_params.py`）。

## 安全

- 前輪が回ったら（車体が動いた＝後輪が接地している）すぐ弱いブレーキで中止する
- 後輪の周速が `max_wheel_m_s` を超えたら、その回はそこでブレーキに移る（100Hz で見る。
  `plan()` は LiDAR の 10Hz でしか呼ばれないので、指令が変わるまで最大 0.1s 遅れる）
"""

from __future__ import annotations

from ..msgs.types import AutoState, Scan, VehicleState
from ._sysid_common import TestGate, abort_state, settle_state
from .base import Planner

__all__ = ["SysIdWheel", "TORQUES_NM", "OVERRIDES"]

#: 掛けるトルク [N·m]（1輪あたり）。小さい順
TORQUES_NM: tuple[float, ...] = (0.008, 0.012, 0.016)
#: 試験の間のファームの設定
OVERRIDES = {"tc_enable": 0.0, "tv_enable": 0.0}


#: 経過時間の比較の余裕 [s]（0.1s の足し算の丸め誤差で判断が1周期ずれないように）
_EPS = 1e-6


class SysIdWheel(Planner):
    id = "sysid_wheel"
    name = "制御の同定: 後輪の空転"
    description = ("★後輪を両方とも浮かせて行う（車体を台に載せる）。小さなトルクで後輪を回しては止めるを"
                   "6回。車輪の慣性・摩擦・モータドライバの遅れを測る。約8秒")
    category = "sysid"
    stats = ()

    SETTINGS: dict[str, float] = {
        "spin_s": 0.2, "repeats": 2, "brake_torque_nm": 0.02, "rest_s": 0.4,
        "max_wheel_m_s": 3.5, "brake_timeout_s": 2.0,
    }
    params = ()

    _SPIN, _BRAKE, _REST = 0, 1, 2
    #: 前輪がこれより速く回ったら車体が動いている [m/s]
    _FRONT_MOVING_M_S = 0.15
    _STOPPED_M_S = 0.05

    def __init__(self) -> None:
        self.settings = dict(self.SETTINGS)
        self.torques = TORQUES_NM
        self._gate = TestGate()
        self._reset_state()

    def _reset_state(self) -> None:
        self._t = 0.0
        self._i = 0
        self._phase = self._REST
        self._overspeed = False
        self._moving = False

    def reset(self) -> None:
        self._gate.reset()
        self._reset_state()

    def set_engaged(self, engaged: bool) -> None:
        self._gate.set_engaged(engaged)

    def on_vehicle_state(self, vs: VehicleState) -> None:
        """`planning_node` が全サンプル（100Hz）で呼ぶ。"""
        if len(vs.wheel_speed) < 4:
            return
        if max(abs(vs.wheel_speed[2]), abs(vs.wheel_speed[3])) > self.settings["max_wheel_m_s"]:
            self._overspeed = True
        if self._phase != self._REST and abs(vs.speed) > self._FRONT_MOVING_M_S:
            self._moving = True

    def n_pulses(self) -> int:
        return len(self.torques) * int(self.settings["repeats"])

    def plan(self, scan: Scan, vs: VehicleState | None,
             p: dict[str, float], dt: float) -> AutoState:
        p = {**self.settings, **p}
        st = AutoState(mode=self.id, planner=self.name)
        st.ready = True

        engaged, armed, just_started = self._gate.tick(vs)
        if just_started:
            self._reset_state()
        if not engaged:
            st.reason = "後輪を浮かせてから試験開始を押してください"
            return st
        if not armed:
            st.reason = "ARM待ち（Enterを押してください）"
            return st
        assert vs is not None
        st.fw_overrides = dict(OVERRIDES)
        if self._moving:
            return abort_state(st, "前輪が回っています（後輪を浮かせ、車体が動かないようにしてください）")
        if self._gate.settling(dt):
            return settle_state(st)
        if self._i >= self.n_pulses():
            st.reason = "完了"
            return st

        self._t += dt
        rear = max(abs(vs.wheel_speed[2]), abs(vs.wheel_speed[3])) if len(vs.wheel_speed) >= 4 else 0.0
        if self._phase == self._SPIN:
            done = self._t >= p["spin_s"] - _EPS or self._overspeed
        elif self._phase == self._BRAKE:
            done = rear <= self._STOPPED_M_S or self._t >= p["brake_timeout_s"] - _EPS
        else:
            done = self._t >= p["rest_s"] - _EPS
        if done:
            self._t = 0.0
            if self._phase == self._REST:
                self._phase = self._SPIN
                self._overspeed = False
            elif self._phase == self._SPIN:
                self._phase = self._BRAKE
            else:
                self._phase = self._REST
                self._i += 1
                if self._i >= self.n_pulses():
                    st.reason = "完了"
                    return st

        torque = self.torques[self._i // int(p["repeats"])]
        if self._phase == self._SPIN:
            st.torque_mode = True
            st.target_torque = torque
            st.reason = f"回す {torque:.3f}N·m {self._i + 1}/{self.n_pulses()}"
        elif self._phase == self._BRAKE:
            st.brake = True
            st.brake_torque = p["brake_torque_nm"]
            st.reason = f"止める {self._i + 1}/{self.n_pulses()}"
        else:
            st.reason = f"休む {self._i + 1}/{self.n_pulses()}"
        return st
