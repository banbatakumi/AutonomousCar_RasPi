"""校正 6: 後輪の空転（車輪の慣性・摩擦・トルクの遅れ）。**後輪を両方とも浮かせて**（車体を台に載せる。
前輪は接地したまま動かないこと）行う。解析は `tools/calib/wheel.py`、設計は `docs/calibration.md` §4.6。

`TORQUES_NM` の各トルクで「回す（`spin`）→ トルク 0 で惰行（`coast`）→ 弱く制動して止める → 休む（トルク 0）」を繰り返す
（前進・後退の向き）。路面の力が無いので

    回す: J·ω' = T − 摩擦      惰行: J·ω' = −摩擦

で、**惰行の減速から摩擦÷J、回す加速から (T − 摩擦)÷J** が出て、慣性 J と摩擦が別々に決まる（回して
すぐ止める旧試験は、加速の傾き 1 本で J と摩擦を分けていた）。トルクの立ち上がりから、トルクの遅れ（MD の
遅れ＋MD が報告する車輪の速度の遅れ）も出る。TC の一巡にはこの両方が入るので、合わせた値でよい。

## ファームの設定（試験の間だけ）

TC・ABS・片輪浮き対策・TV を切る（`fw_overrides`）。TC は前輪の車速を基準にスリップを見るので、止まった
車体で後輪が回るとすぐ絞る。切ると後輪の周速の上限（7.5m/s）の歯止めも外れるので、周速は 100Hz で見て
`max_wheel_m_s` を超えたらその回の「回す」を打ち切る。

## 安全

- 前輪が動いたら（車体が動いた＝後輪が接地している）すぐ止める
- 指令が途絶えれば STM32 が 100ms でフェイルセーフに入り、後輪を最大制動する
"""

from __future__ import annotations

from ..msgs.types import AutoState, Scan, VehicleState
from ._calib_common import TestGate, abort_state, md_down, settle_state
from .base import Planner

__all__ = ["CalibWheel", "TORQUES_NM", "OVERRIDES"]

#: 回すトルク [N·m]（1輪あたり、正 = 前進の向き）。**止まった状態から回り出すには、回っている間の摩擦（約 0.003N·m）より
#: 大きなトルクが要る**（静止摩擦とモータのコギング）。2026-10-19 の実機で 0.004・0.006N·m はほとんど回らず、0.008N·m でも
#: 片方の輪しか回らない回があった。回り出した後は慣性約 2.4e-5kg·m² で、0.014N·m なら周速は約 14m/s² で上がる
TORQUES_NM: tuple[float, ...] = (0.010, 0.012, 0.014, -0.010, -0.012, -0.014)
OVERRIDES = {"tc_enable": 0.0, "abs_enable": 0.0, "wheel_lift_guard_enable": 0.0, "tv_enable": 0.0}


class CalibWheel(Planner):
    id = "calib_wheel"
    name = "校正 6: 後輪の空転（台上）"
    description = ("★後輪を両方とも浮かせて行う。小さいトルクで後輪を回しては惰行させ、車輪の慣性・摩擦・"
                   "トルクの遅れを測る。約30秒")
    category = "sysid"
    stats = ()
    params = ()

    SETTINGS = {
        "repeats": 2,
        "spin_s": 0.3,
        "coast_s": 0.8,
        "rest_s": 0.4,
        #: 回すのをやめる後輪の周速 [m/s]
        "max_wheel_m_s": 3.0,
        #: 惰行をやめる周速 [m/s]
        "coast_min_m_s": 0.15,
        "brake_nm": 0.01,
        #: 前輪がこれ以上動いたら中止 [m]（台に載っていない）
        "front_move_m": 0.02,
    }

    def __init__(self) -> None:
        self.settings = dict(self.SETTINGS)
        self._gate = TestGate()
        self._reset_state()

    def _reset_state(self) -> None:
        self._i = 0
        self._stage = "spin"
        self._stage_t = 0.0
        self._fault: str | None = None
        self._odom0: float | None = None
        self._rim = 0.0
        self._too_fast = False

    def reset(self) -> None:
        self._gate.reset()
        self._reset_state()

    def set_engaged(self, engaged: bool) -> None:
        self._gate.set_engaged(engaged)

    def cycles(self) -> list[float]:
        return [tq for tq in TORQUES_NM for _ in range(int(self.settings["repeats"]))]

    def on_vehicle_state(self, vs: VehicleState) -> None:
        """100Hz。前輪が動いた・MD が黙った・回しすぎを見る。"""
        if not vs.armed:
            return
        odom = 0.5 * (vs.odom_dist[0] + vs.odom_dist[1])
        if self._odom0 is None:
            self._odom0 = odom
        if self._fault is None and abs(odom - self._odom0) > self.settings["front_move_m"]:
            self._fault = "前輪が動いた（後輪が接地している？台に載せて後輪を浮かせる）"
        down = md_down(vs.md_status)
        if self._fault is None and down:
            self._fault = f"{'・'.join(down)}のモータドライバが無応答"
        self._rim = max(abs(vs.wheel_speed[2]), abs(vs.wheel_speed[3])) if len(vs.wheel_speed) >= 4 else 0.0
        if self._stage == "spin" and self._rim >= self.settings["max_wheel_m_s"]:
            self._too_fast = True

    def plan(self, scan: Scan, vs: VehicleState | None, p: dict[str, float], dt: float) -> AutoState:
        s = self.settings
        st = AutoState(mode=self.id, planner=self.name)
        st.ready = True
        engaged, armed, just_started = self._gate.tick(vs)
        if just_started:
            self._reset_state()
        if not engaged:
            st.reason = "試験開始を押してください（★後輪を浮かせておく）"
            return st
        if not armed:
            st.reason = "ARM待ち（Enterを押してください）"
            return st
        if self._fault is not None:
            return abort_state(st, self._fault)
        if self._gate.settling(dt):
            return settle_state(st)
        cyc = self.cycles()
        if self._i >= len(cyc):
            st.reason = "完了"
            st.phase = "done"
            return st
        tq = cyc[self._i]
        self._stage_t += dt
        st.fw_overrides = dict(OVERRIDES)
        if self._stage == "spin" and (self._stage_t >= s["spin_s"] or self._too_fast):
            self._stage, self._stage_t, self._too_fast = "coast", 0.0, False
        if self._stage == "coast" and (self._stage_t >= s["coast_s"] or self._rim < s["coast_min_m_s"]):
            self._stage, self._stage_t = "brake", 0.0
        if self._stage == "brake" and self._rim < 0.03:
            self._stage, self._stage_t = "rest", 0.0
        if self._stage == "rest" and self._stage_t >= s["rest_s"]:
            self._i += 1
            self._stage, self._stage_t = "spin", 0.0
            if self._i >= len(cyc):
                st.fw_overrides = {}
                st.reason = "完了"
                st.phase = "done"
                return st
            tq = cyc[self._i]
        sign = "+" if tq > 0 else "-"
        st.phase = f"{self._stage}/{self._i}/{sign}"
        label = f"{self._i + 1}/{len(cyc)}"
        if self._stage == "spin":
            st.torque_mode, st.target_torque = True, tq
            st.reason = f"回す {tq:+.3f}N·m {label}"
        elif self._stage == "coast":
            st.torque_mode, st.target_torque = True, 0.0
            st.reason = f"惰行 {label}"
        elif self._stage == "brake":
            st.brake, st.brake_torque = True, s["brake_nm"]
            st.reason = f"止める {label}"
        else:
            # 休む間はトルク 0（制動で止めておくと STM32 のブレーキホールド＝位置保持に入り、次の回り出しに混ざる）
            st.torque_mode, st.target_torque = True, 0.0
            st.reason = f"休む {label}"
        return st
