"""校正 1: 前輪エンコーダ（出力の範囲・1回転の周期誤差）。解析は `tools/calib/encoder.py`、設計は
`docs/calibration.md` §4.1。

直線を往復し、片道ごとに「加速 → 定速 → **トルク 0 で惰行**」。解析が使うのは惰行の区間だけ
（`coast/…`）。惰行中の車輪の真の回転は転がり抵抗だけで滑らかに減速するので、読みのうち時間の滑らかな
関数で説明できない、角度の関数のぶんが周期誤差になる。速度制御中は PI が周期誤差を含む車速に合わせて
トルクを揺らし、真の回転にも同じうねりが乗るので使わない。

惰行に入る車速は片道ごとに `speeds` を順に使う（速さの違う惰行で係数が一致することを解析が確かめる）。

必要なスペース: 直線 2m＋車長。約 40 秒。
"""

from __future__ import annotations

from ..msgs.types import VehicleState
from ._calib_common import Step, StraightTest

__all__ = ["CalibEncoder"]


class CalibEncoder(StraightTest):
    id = "calib_encoder"
    name = "校正 1: 前輪エンコーダ"
    description = "直線を4往復し、トルク0の惰行で前輪エンコーダの周期誤差・出力の範囲を測る。直線2m＋車長。約40秒"

    SETTINGS = {
        "round_trips": 4,
        #: 惰行に入る車速 [m/s]（往復ごとに順に）
        "speeds": (0.7, 1.1),
        "accel": 1.5,
        #: 定速に乗せてから惰行に入るまで [s]（PI の過渡が収まるまで）
        "settle_s": 0.25,
        #: 惰行の長さの上限 [s] と、これより遅くなったらやめる車速 [m/s]（遅いと 1 回転が長く材料が減る）
        "coast_s": 2.0, "coast_min_speed": 0.2,
    }
    _AT_SPEED_M_S = 0.04

    def _reset_state(self) -> None:
        super()._reset_state()
        self._coast_from: float | None = None
        self._at_speed_t = 0.0
        self._cur_leg = -1

    def leg_count(self) -> int:
        return 2 * int(self.settings["round_trips"])

    def leg_speed(self, leg: int) -> float:
        speeds = self.settings["speeds"]
        return float(speeds[(leg // 2) % len(speeds)])

    def leg_step(self, leg: int, t: float, vs: VehicleState, traveled: float) -> Step:
        s = self.settings
        if leg != self._cur_leg:
            self._cur_leg = leg
            self._coast_from = None
            self._at_speed_t = 0.0
            self._last_t = t
        dt, self._last_t = t - self._last_t, t
        target = self.leg_speed(leg)
        v = abs(vs.speed)
        if self._coast_from is None:
            if v >= target - self._AT_SPEED_M_S:
                self._at_speed_t += dt
            if self._at_speed_t >= s["settle_s"]:
                self._coast_from = t
            else:
                return Step("speed", target, "run", accel_limit=s["accel"], note=f"加速・定速 {target:.1f}m/s")
        if t - self._coast_from >= s["coast_s"] or v < s["coast_min_speed"]:
            return Step(done=True)
        return Step("torque", 0.0, "coast", note=f"惰行（{target:.1f}m/s から）")
