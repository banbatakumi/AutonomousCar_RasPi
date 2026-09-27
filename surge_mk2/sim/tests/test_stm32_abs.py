"""`sim/stm32.py` の ABS の旗（★v0.15）。

シムは後輪の回転を持たないので、ABS は「制動の減速度がグリップ（`brake_decel_m_s2`）で頭打ちに
なった」ときに旗を立てるだけ（`sim/vehicle.py` の `brake_decel`）。ファーム同様、ABS_ENABLE を
切れば立たず、0.25m/s 未満でも立たない。
"""

import sys
import unittest
from dataclasses import replace
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from raspi.proto import packets  # noqa: E402
from sim.course import Course  # noqa: E402
from sim.params import SimParams  # noqa: E402
from sim.stm32 import VirtualStm32  # noqa: E402
from sim.vehicle import DriveInput, VehicleSpec, brake_decel  # noqa: E402

COURSE = Path(__file__).resolve().parents[1] / "courses" / "circuit_chicane_a.png"
SPEC = replace(VehicleSpec.load(), brake_decel_per_nm=35.0, brake_decel_m_s2=3.7,
               rolling_resistance=0.4)


def flags_after(nm: float, speed: float, abs_enable: float | None = None) -> int:
    sim = VirtualStm32(SPEC, Course.load(COURSE), SimParams())
    t0 = 1_000_000_000
    sim.start(t0)
    if abs_enable is not None:
        sim._config[packets.Param.ABS_ENABLE] = abs_enable
    sim._cmd_ns = t0
    sim._input = DriveInput(armed=True, brake=True, brake_torque=nm)
    sim.vehicle.speed = speed
    sim._substep(0.001)
    return sim._flags(t0)


class TestSimAbsFlag(unittest.TestCase):
    def test_flag_only_when_the_brake_is_grip_limited(self):
        self.assertTrue(flags_after(0.15, 2.0) & packets.FLG_ABS_ACTIVE)    # 35×0.15+0.4 > 3.7
        self.assertFalse(flags_after(0.04, 2.0) & packets.FLG_ABS_ACTIVE)   # 35×0.04+0.4 < 3.7

    def test_disabled_or_slow_gives_no_flag(self):
        self.assertFalse(flags_after(0.15, 2.0, abs_enable=0.0) & packets.FLG_ABS_ACTIVE)
        self.assertFalse(flags_after(0.15, 0.2) & packets.FLG_ABS_ACTIVE)

    def test_brake_decel_matches_the_old_model(self):
        """`next_speed` から切り出した `brake_decel` の値は元の式と同じ（頭打ちの有無だけ足した）。"""
        d, capped = brake_decel(SPEC, DriveInput(armed=True, brake=True, brake_torque=0.15), 2.0)
        self.assertEqual((d, capped), (3.7, True))
        d, capped = brake_decel(SPEC, DriveInput(armed=True, brake=True, brake_torque=0.04), 5.0)
        self.assertAlmostEqual(d, 35.0 * 0.04 * 0.9999 + 0.4, places=2)
        self.assertFalse(capped)


if __name__ == "__main__":
    unittest.main()
