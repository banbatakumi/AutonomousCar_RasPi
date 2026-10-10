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
from raspi.proto.framing import FrameEncoder  # noqa: E402
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


class TestSimTorqueCmd(unittest.TestCase):
    """テレメトリの `torque_cmd` は制動中に負になる（GUI 車体図の制動ラインの入力）。"""

    def torque_after(self, nm: float, brake: bool) -> float:
        sim = VirtualStm32(SPEC, Course.load(COURSE), SimParams())
        t0 = 1_000_000_000
        sim.start(t0)
        sim._cmd_ns = t0
        sim._input = DriveInput(armed=True, brake=brake, brake_torque=nm, target_speed=0.0)
        sim.vehicle.speed = 2.0 if brake else 0.0
        sim._substep(0.001)
        return sim._telemetry(t0).torque_cmd[0] * 0.0001

    def test_brake_is_negative_and_scales_with_torque(self):
        weak, strong = self.torque_after(0.04, True), self.torque_after(0.15, True)
        self.assertLess(strong, weak)
        self.assertLess(weak, -0.02)

    def test_standing_still_is_not_braking(self):
        self.assertGreater(self.torque_after(0.0, False), -0.02)


class TestSimLaunch(unittest.TestCase):
    """ローンチコントロール（★v0.20）: `COMMAND.flags2` の LAUNCH → 発進中は `flags` の LAUNCH_ACTIVE。"""

    def _sim(self) -> VirtualStm32:
        sim = VirtualStm32(SPEC, Course.load(COURSE), SimParams())
        sim.start(1_000_000_000)
        return sim

    def _command(self, sim: VirtualStm32, launch: bool) -> None:
        flags2 = packets.CMD_FLG2_LAUNCH if launch else 0
        sim._on_command(packets.Command(mode=2, flags=packets.CMD_FLG_ARM, target_speed=2000,
                                        flags2=flags2), 0, 1_000_000_000)

    def test_flag_follows_the_launch(self):
        sim = self._sim()
        self._command(sim, launch=True)
        self.assertTrue(sim._input.launch)
        sim._substep(0.001)
        self.assertTrue(sim._flags(1_000_000_000) & packets.FLG_LAUNCH_ACTIVE)
        for _ in range(900):                                           # 目標に届けば終わる（壁に着く前）
            sim._cmd_ns = sim._now                                     # COMMAND は届き続けている
            sim._substep(0.001)
        self.assertFalse(sim._flags(1_000_000_000) & packets.FLG_LAUNCH_ACTIVE)
        self.assertAlmostEqual(sim.vehicle.speed, 2.0, delta=0.15)

    def test_no_flag_without_the_request(self):
        sim = self._sim()
        self._command(sim, launch=False)
        sim._substep(0.001)
        self.assertFalse(sim._flags(1_000_000_000) & packets.FLG_LAUNCH_ACTIVE)

    def test_exit_torque_comes_from_config_set(self):
        sim = self._sim()
        sim.rx_bytes(FrameEncoder().encode(
            packets.ConfigSet(param_id=packets.Param.LAUNCH_EXIT_TORQUE_NM, value=0.009)), 1_000_000_000)
        self.assertAlmostEqual(sim.vehicle._speed_ctl.launch_exit_torque_nm, 0.009)


if __name__ == "__main__":
    unittest.main()
