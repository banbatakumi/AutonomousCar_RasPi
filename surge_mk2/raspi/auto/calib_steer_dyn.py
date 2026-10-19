"""校正 3: ステアの動特性（舵の遅れ・ヨーの遅れ）。解析は `tools/calib/steer.py` の `analyze_dynamic`、設計は
`docs/calibration.md` §4.3。

直線を `speed` で前進しながら、舵を ±`amplitude` の方形波（半周期 `half_period_s`）で振る（`zigzag/…`）。
振る順は「＋ − − ＋」の繰り返し（＋ − の繰り返しだと最初の半周期ぶん向きが片側に残る。この順なら 4 つごとに向きと
横位置のずれが打ち消し合う）。後退の片道は低速で戻りながら、置いた直線へ強めに寄せ直す（`return_keep_rad`）。

必要なスペース: 直線 2m＋車長 × 幅 約 0.5m。約 30 秒。
"""

from __future__ import annotations

from ..msgs.types import VehicleState
from ._calib_common import Step, StraightTest

__all__ = ["CalibSteerDyn"]


class CalibSteerDyn(StraightTest):
    id = "calib_steer_dyn"
    name = "校正 3: ステアの遅れ"
    description = "直線を1.0m/sで前進しながら舵を±6°の方形波で振り（3往復）、舵とヨーの遅れを測る。直線2m＋車長。約30秒"

    SETTINGS = {
        "round_trips": 3,
        "speed": 1.0, "accel": 1.5,
        "amplitude": 0.10, "half_period_s": 0.3,
        "return_speed": 0.6,
        #: 戻りの片道で置いた直線へ寄せる舵の上限 [rad]
        "return_keep_rad": 0.12,
    }
    keep_steer_rad = 0.03

    def _reset_state(self) -> None:
        super()._reset_state()
        self._cur_leg = -1
        self._zig_from: float | None = None

    def keep_limit(self, leg: int) -> float:
        return self.settings["return_keep_rad"] if leg % 2 == 1 else self.keep_steer_rad

    def leg_count(self) -> int:
        return 2 * int(self.settings["round_trips"])

    def leg_step(self, leg: int, t: float, vs: VehicleState, traveled: float) -> Step:
        s = self.settings
        if leg % 2 == 1:
            return Step("speed", s["return_speed"], "return", accel_limit=1.0, note="戻る")
        if leg != self._cur_leg:
            self._cur_leg, self._zig_from = leg, None
        if self._zig_from is None:
            if abs(vs.speed) >= s["speed"] - 0.05:
                self._zig_from = t
            else:
                return Step("speed", s["speed"], "run", accel_limit=s["accel"], note="加速")
        k = int((t - self._zig_from) / s["half_period_s"])
        sign = 1.0 if k % 4 in (0, 3) else -1.0
        return Step("speed", s["speed"], "zigzag", accel_limit=s["accel"], steer_offset=sign * s["amplitude"],
                    note=f"舵を振る {sign * s['amplitude'] * 57.3:+.0f}°")
