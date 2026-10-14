"""仮想 STM32 のレビュー指摘の修正（LiDAR の撃つ姿勢・センサ倍率誤差・途絶中の舵・ARM 規則）。"""

import math
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from sim.course import Course  # noqa: E402
from sim.lidar import VirtualLidar  # noqa: E402
from sim.params import SimParams  # noqa: E402
from sim.stm32 import VirtualStm32  # noqa: E402
from sim.vehicle import DriveInput, VehicleSpec  # noqa: E402

COURSE = Path(__file__).resolve().parents[1] / "courses" / "circuit_chicane_a.png"


def make(params=None, lidar=False):
    spec = VehicleSpec.load()
    course = Course.load(COURSE)
    params = params or SimParams()
    ld = VirtualLidar(course, spec, params) if lidar else None
    sim = VirtualStm32(spec, course, params, lidar=ld)
    sim.start(1_000_000_000)
    sim._cmd_ns = 10**15                 # 途絶させない
    return sim, ld


class TestLidarPose(unittest.TestCase):
    def test_sectors_in_one_advance_are_shot_at_their_own_pose(self):
        sim, ld = make(lidar=True)
        sim._input = DriveInput(armed=True, target_speed=2.0)
        sim.vehicle.speed = 2.0
        t = 1_000_000_000
        for _ in range(20):                           # 走り出して、ld を初期化する
            t += 5_000_000
            sim.advance_to(t)
        xs = []
        orig = ld._sector_packet

        def spy(idx, t_end, vehicle, stm_us):
            xs.append((vehicle.x, vehicle.y))
            return orig(idx, t_end, vehicle, stm_us)
        ld._sector_packet = spy
        sim.advance_to(t + 60_000_000)                # 60ms をまとめて進める
        self.assertGreaterEqual(len(xs), 5)
        self.assertGreater(len(set(xs)), len(xs) - 2)   # 同一姿勢ではない
        d = math.hypot(xs[-1][0] - xs[0][0], xs[-1][1] - xs[0][1])
        self.assertGreater(d, 0.05)                     # 走行 ≒ 1.7m/s × 50ms の大半


class TestScaleErrors(unittest.TestCase):
    def test_speed_and_gyro_scale(self):
        a, _ = make(SimParams(odom_noise_m=0, gyro_noise_dps=0, gyro_bias_dps=0))
        b, _ = make(SimParams(odom_noise_m=0, gyro_noise_dps=0, gyro_bias_dps=0,
                              speed_scale_err=0.02, gyro_scale_err=0.01))
        for s in (a, b):
            s.vehicle.speed = 2.0
            s.vehicle.yaw_rate = 0.5
            s.vehicle.speed_measured = 2.0
            s.vehicle.yaw_rate_measured = 0.5
        ta, tb = a._telemetry(1_000_000_000), b._telemetry(1_000_000_000)
        self.assertAlmostEqual(tb.speed / ta.speed, 1.02, places=2)
        self.assertAlmostEqual(tb.yaw_rate / ta.yaw_rate, 1.01, places=2)


from raspi.proto.generated import packets  # noqa: E402


def cmd(sim, t_ns, arm, steer=0, mode=2, speed=0):
    sim._on_command(packets.Command(mode=mode, flags=packets.CMD_FLG_ARM if arm else 0,
                                    target_steer=steer, target_speed=speed), 0, t_ns)


def run(sim, t_ns, sec, keep_cmd):
    """`sec` 秒進める。`keep_cmd` なら COMMAND が届き続けている扱い。"""
    t = t_ns
    for _ in range(int(sec * 100)):
        t += 10_000_000
        if keep_cmd:
            sim._cmd_ns = t
        sim.advance_to(t)
    return t


class TestSteerHold(unittest.TestCase):
    def _armed_with_steer(self):
        sim, _ = make()
        t = 1_000_000_000
        cmd(sim, t, False)
        t += 10_000_000
        cmd(sim, t, True, steer=4000)                 # 0.4rad
        t = run(sim, t, 1.0, True)
        self.assertGreater(sim.vehicle.steer_actual, 0.3)
        return sim, t

    def test_command_loss_holds_steer(self):
        sim, t = self._armed_with_steer()
        s0 = sim.vehicle.steer_actual
        run(sim, t, 2.0, False)                       # 途絶
        self.assertTrue(sim.uart_timeout)
        self.assertAlmostEqual(sim.vehicle.steer_actual, s0, delta=0.01)

    def test_estop_and_disarm_hold_steer(self):
        sim, t = self._armed_with_steer()
        s0 = sim.vehicle.steer_actual
        sim.set_estop(True)
        t = run(sim, t, 1.0, True)
        self.assertAlmostEqual(sim.vehicle.steer_actual, s0, delta=0.01)
        sim2, t2 = self._armed_with_steer()
        cmd(sim2, t2, False, steer=0)                 # DISARM（目標 0 でも舵は動かない）
        run(sim2, t2, 1.0, True)
        self.assertAlmostEqual(sim2.vehicle.steer_actual, s0, delta=0.01)


class TestArmRule(unittest.TestCase):
    def test_boot_needs_a_disarm_first(self):
        sim, _ = make()
        t = 1_000_000_000
        cmd(sim, t, True)
        self.assertFalse(sim._input.armed)
        cmd(sim, t + 10_000_000, False)
        cmd(sim, t + 20_000_000, True)
        self.assertTrue(sim._input.armed)

    def test_after_estop_release(self):
        sim, _ = make()
        t = 1_000_000_000
        cmd(sim, t, False)
        cmd(sim, t + 10_000_000, True)
        self.assertTrue(sim._input.armed)
        sim.set_estop(True)
        sim.set_estop(False)
        cmd(sim, t + 20_000_000, True)                # ARM のまま → 受け付けない
        self.assertFalse(sim._input.armed)
        cmd(sim, t + 30_000_000, False)
        cmd(sim, t + 40_000_000, True)
        self.assertTrue(sim._input.armed)

    def test_after_command_loss_and_log_once(self):
        sim, _ = make()
        sim._cmd_ns = -10**12
        t = 1_000_000_000
        cmd(sim, t, False)
        cmd(sim, t + 10_000_000, True)
        self.assertTrue(sim._input.armed)
        n0 = len(sim._tx)
        t += 300_000_000                              # 途絶
        cmd(sim, t, True)
        cmd(sim, t + 10_000_000, True)
        self.assertFalse(sim._input.armed)
        warn = [f for _, f in sim._tx[n0:] if b"ARM ignored" in f]
        self.assertEqual(len(warn), 1)                # LOG は 1 回だけ
        cmd(sim, t + 20_000_000, False)
        cmd(sim, t + 30_000_000, True)
        self.assertTrue(sim._input.armed)

    def test_reserved_mode_keeps_current_mode(self):
        sim, _ = make()
        t = 1_000_000_000
        cmd(sim, t, False)
        cmd(sim, t + 10_000_000, True, mode=2)
        cmd(sim, t + 20_000_000, True, mode=3)
        self.assertEqual(sim.mode, 2)
        self.assertTrue(sim._input.armed)


if __name__ == "__main__":
    unittest.main()
