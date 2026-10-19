"""校正 5（確かめる）: 車速制御の閉ループの効き。ふだんの走りと同じ車速指令（STM32 のランプ → PI＋FF）で
直線を往復し、追従の誤差を測る。解析は `tools/calib/speed_check.py`（同定した車体と、記録したときの
ゲインでファームを回した予測と比べる）。設計は `docs/calibration.md` §4.5。

片道ごとに「加速のランプ → 定速 → 段で減速 → 定速 → 減速のランプで停止」。往復ごとに `profiles` を順に使う
（加速度・車速の違う組）。

必要なスペース: 直線 2m＋車長。約 35 秒。
"""

from __future__ import annotations

from ..msgs.types import VehicleState
from ._calib_common import Step, StraightTest

__all__ = ["CalibSpeedCheck", "PROFILES"]

#: (加速度 [m/s²], 1 段目の車速, 2 段目の車速, 減速度 [m/s²])。加速度はグリップの限界より十分下に置く
#: （後退の加速は後輪の荷重が抜けるので限界が低い。シムで 2.0m/s² は TC が絞った）。限界の近くは
#: 制御器ではなくタイヤの評価になる（それは校正 8）
PROFILES: tuple[tuple[float, float, float, float], ...] = (
    (1.0, 1.0, 0.6, 1.5),
    (1.5, 1.3, 0.7, 2.0),
    (0.5, 0.7, 0.3, 1.0),
)


class CalibSpeedCheck(StraightTest):
    id = "calib_speed_check"
    name = "校正 5: 車速制御（確かめる）"
    description = ("車速指令で直線を3往復（加速のランプ→定速→段→定速→停止）し、車速PIとFFの追従を測る。"
                   "直線2m＋車長。約35秒")

    SETTINGS = {
        "profiles": PROFILES,
        #: 加速のランプが目標に届いてから保つ時間 [s]・段の後に保つ時間 [s]
        "hold1_s": 0.4, "hold2_s": 0.5,
        #: 段で減速するときの加速度 [m/s²]（減速もトルクで止める＝TC が絞りうるので限界より下）
        "step_accel": 2.0,
    }

    def _reset_state(self) -> None:
        super()._reset_state()
        self._cur_leg = -1
        self._stage_name = "ramp"
        self._stage_from = 0.0

    def leg_count(self) -> int:
        return 2 * len(self.settings["profiles"])

    def profile(self, leg: int) -> tuple[float, float, float, float]:
        return self.settings["profiles"][(leg // 2) % len(self.settings["profiles"])]

    def leg_step(self, leg: int, t: float, vs: VehicleState, traveled: float) -> Step:
        s = self.settings
        a1, v1, v2, a3 = self.profile(leg)
        if leg != self._cur_leg:
            self._cur_leg, self._stage_name, self._stage_from = leg, "ramp", t
        el = t - self._stage_from
        # ランプが目標に届く時間（STM32 のランプは指令した加速度で動く）
        if self._stage_name == "ramp" and el >= v1 / a1 + s["hold1_s"]:
            self._stage_name, self._stage_from, el = "step", t, 0.0
        if self._stage_name == "step" and el >= (v1 - v2) / s["step_accel"] + s["hold2_s"]:
            self._stage_name, self._stage_from, el = "down", t, 0.0
        if self._stage_name == "down" and el >= v2 / a3 + 0.3:
            return Step(done=True)
        if self._stage_name == "ramp":
            return Step("speed", v1, "ramp", accel_limit=a1, note=f"加速 {a1:.1f}m/s² → {v1:.1f}m/s")
        if self._stage_name == "step":
            return Step("speed", v2, "step", accel_limit=s["step_accel"], note=f"段 → {v2:.1f}m/s")
        return Step("speed", 0.0, "down", accel_limit=a3, note=f"減速 {a3:.1f}m/s² → 停止")
