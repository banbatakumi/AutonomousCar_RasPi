"""校正 5（測る）: 車速制御の車体の同定。トルク直接指令で、車速 PI を通さずに車体を測る。
解析は `tools/calib/speed.py`、設計は `docs/calibration.md` §4.5。

片道ごとに「強いトルク（加速）→ 弱いトルク → トルク 0（惰行）」の 3 段。**1 つの区間の中にトルクの違う段が
あること**が大事: トルクが一定の区間だけでは走行距離から「G·T − 転がり抵抗」しか決まらず、G と転がり抵抗が
分かれない（旧 ⑤ の記録はほとんどの区間がトルク 1 段で、区間の組み方で G が 10% 動いた）。段の変わり目は
MD のトルクの遅れの材料にもなる。

強いトルクは往復ごとに `drive_nm` を順に使う。滑らない強さに抑える（1 輪 0.06N·m で約 1.8m/s²、後輪の
前後力は荷重の約 3 割）。TC は入れたまま（滑ったら TC が絞り、その区間は解析が使わない）。

必要なスペース: 直線 2m＋車長。約 40 秒。
"""

from __future__ import annotations

from ..msgs.types import VehicleState
from ._calib_common import Step, StraightTest

__all__ = ["CalibSpeed"]


class CalibSpeed(StraightTest):
    id = "calib_speed"
    name = "校正 5: 車速制御（測る）"
    description = ("トルク直接指令で直線を4往復（加速→弱いトルク→惰行）し、車体の利得・転がり抵抗・トルクの遅れを測る。"
                   "直線2m＋車長。約40秒")

    SETTINGS = {
        "round_trips": 4,
        #: 強いトルク [N·m]（1 輪あたり。往復ごとに順に）
        "drive_nm": (0.035, 0.05, 0.06, 0.045),
        #: 強いトルクをやめる車速 [m/s]
        "v_top": 1.1,
        #: 弱いトルクの、強いトルクに対する割合と長さ [s]
        "mid_ratio": 0.35, "mid_s": 0.45,
        #: 惰行の長さ [s] と、これより遅くなったらやめる車速 [m/s]
        "coast_s": 0.6, "coast_min_speed": 0.3,
        #: 強いトルクの長さの上限 [s]
        "drive_s_max": 2.0,
    }

    def _reset_state(self) -> None:
        super()._reset_state()
        self._cur_leg = -1
        self._stage_from = 0.0
        self._stage_name = "drive"

    def leg_count(self) -> int:
        return 2 * int(self.settings["round_trips"])

    def leg_torque(self, leg: int) -> float:
        lv = self.settings["drive_nm"]
        return float(lv[(leg // 2) % len(lv)])

    def leg_step(self, leg: int, t: float, vs: VehicleState, traveled: float) -> Step:
        s = self.settings
        if leg != self._cur_leg:
            self._cur_leg, self._stage_name, self._stage_from = leg, "drive", t
        tq = self.leg_torque(leg)
        v = abs(vs.speed)
        el = t - self._stage_from
        if self._stage_name == "drive" and (v >= s["v_top"] or el >= s["drive_s_max"]):
            self._stage_name, self._stage_from, el = "mid", t, 0.0
        if self._stage_name == "mid" and el >= s["mid_s"]:
            self._stage_name, self._stage_from, el = "coast", t, 0.0
        if self._stage_name == "coast" and (el >= s["coast_s"] or v < s["coast_min_speed"]):
            return Step(done=True)
        if self._stage_name == "drive":
            return Step("torque", tq, "drive", note=f"加速 {tq:.3f}N·m")
        if self._stage_name == "mid":
            return Step("torque", tq * s["mid_ratio"], "mid", note=f"弱いトルク {tq * s['mid_ratio']:.3f}N·m")
        return Step("torque", 0.0, "coast", note="惰行")
