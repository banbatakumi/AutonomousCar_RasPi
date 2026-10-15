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

        def spy(idx, t_end, vehicle, stm_us, *rest):
            xs.append((vehicle.x, vehicle.y))
            return orig(idx, t_end, vehicle, stm_us, *rest)
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


class TestAutoStop(unittest.TestCase):
    """ファームの自動停止: 停止距離の矩形に LiDAR 3 点以上＋至近の超音波、LiDAR 不調時だけ超音波が停止距離で見る。"""

    def _sim(self, speed):
        sim, ld = make(lidar=True)
        sim.vehicle.speed = sim.vehicle.speed_measured = speed
        sim._now = 2_000_000_000
        ld.last_emit_ns = sim._now                    # LiDAR は生きている
        ld.last_scan[:] = 0.0
        sim.us_front = sim.us_rear = 0.0
        return sim, ld

    def _put(self, ld, deg, dist_m, n=3):
        for k in range(n):
            ld.last_scan[(deg + k) % 360] = dist_m

    def test_rectangle_not_sector(self):
        sim, ld = self._sim(0.0)
        # 停止中: d_stop = マージン 15cm、LiDAR から 15+23=38cm 以内・幅 ±15cm
        self._put(ld, 0, 0.30)
        self.assertTrue(sim._auto_stop_obstacle(1.0))
        ld.last_scan[:] = 0.0
        self._put(ld, 0, 0.45)                        # 遠い
        self.assertFalse(sim._auto_stop_obstacle(1.0))
        ld.last_scan[:] = 0.0
        self._put(ld, 40, 0.30)                       # 横 0.19m（> 15cm）は扇形なら入るが矩形では入らない
        self.assertFalse(sim._auto_stop_obstacle(1.0))
        ld.last_scan[:] = 0.0
        self._put(ld, 0, 0.30, n=2)                   # 2 点では確定しない
        self.assertFalse(sim._auto_stop_obstacle(1.0))

    def test_distance_grows_with_speed_and_direction_follows_motion(self):
        sim, ld = self._sim(2.0)
        # 2m/s: 0.2 + 0.667 + マージン 0.15 = 1.017m + 0.23
        self._put(ld, 0, 1.2)
        self.assertTrue(sim._auto_stop_obstacle(1.0))
        ld.last_scan[:] = 0.0
        self._put(ld, 180, 0.25)                      # 後方の障害物は前進中は見ない
        self.assertFalse(sim._auto_stop_obstacle(-1.0))
        sim.vehicle.speed_measured = 0.0              # 静止なら指令の向きで見る
        self.assertTrue(sim._auto_stop_obstacle(-1.0))

    def test_ultrasonic_near_always_and_fallback_only_without_lidar(self):
        sim, ld = self._sim(0.0)
        sim.us_front = 0.08                           # 5+4=9cm 未満
        self.assertTrue(sim._auto_stop_obstacle(1.0))
        sim.us_front = 0.15                           # 停止距離（15+4=19cm）以内だが LiDAR が生きている
        self.assertFalse(sim._auto_stop_obstacle(1.0))
        ld.last_emit_ns = 0                           # LiDAR 不調 → 超音波が停止距離で見る
        self.assertTrue(sim._auto_stop_obstacle(1.0))

    def test_latches_until_stopped(self):
        sim, ld = self._sim(1.0)
        sim._cmd_flags = packets.CMD_FLG_AUTO_STOP
        sim.uart_timeout = False
        sim._input = DriveInput(armed=True, target_speed=1.0)
        self._put(ld, 0, 0.5)
        self.assertTrue(sim._auto_stop_should_brake(sim._input))
        ld.last_scan[:] = 0.0                         # 障害物が範囲外になっても、止まるまで解除しない
        sim.auto_stop_active = True
        self.assertTrue(sim._auto_stop_should_brake(sim._input))
        sim.vehicle.speed_measured = 0.0              # 止まったら判定し直す
        for _ in range(5):
            r = sim._auto_stop_should_brake(sim._input)
        self.assertFalse(r)


class TestFailsafePowerOff(unittest.TestCase):
    def test_power_is_cut_after_30s_stopped_and_needs_rearm(self):
        sim, _ = make()
        t = 1_000_000_000
        cmd(sim, t, False)
        cmd(sim, t + 10_000_000, True)
        t = run(sim, t + 10_000_000, 0.5, True)
        self.assertTrue(sim._drive_powered())
        t = run(sim, t, 20.0, False)                  # 途絶して停車（20s）
        self.assertTrue(sim._drive_powered())
        t = run(sim, t, 12.0, False)
        self.assertFalse(sim._drive_powered())
        cmd(sim, t, True)                             # ARM のまま戻っても入れない
        self.assertFalse(sim._drive_powered())
        cmd(sim, t + 10_000_000, False)
        cmd(sim, t + 20_000_000, True)
        self.assertTrue(sim._drive_powered())

    def test_target_speed_is_clamped_to_firmware_limit(self):
        sim, _ = make()
        t = 1_000_000_000
        cmd(sim, t, False)
        cmd(sim, t + 10_000_000, True, speed=32000)   # 32 m/s（u16/0.001 で表せる範囲）
        self.assertLessEqual(sim._input.target_speed, 5.0)


class TestBootGates(unittest.TestCase):
    def test_md_boot_delays_arm_when_enabled(self):
        sim, _ = make(SimParams(md_boot_s=1.0))
        t = 1_000_000_000
        cmd(sim, t, False)
        cmd(sim, t + 10_000_000, True)
        self.assertFalse(sim._input.armed)
        for k in range(2, 111):                       # COMMAND は届き続ける
            cmd(sim, t + k * 10_000_000, True)
        self.assertTrue(sim._input.armed)

    def test_lidar_is_off_until_arm_when_gated(self):
        sim, ld = make(SimParams(lidar_power_gate=1.0), lidar=True)
        t = run(sim, 1_000_000_000, 0.5, True)
        self.assertEqual(ld.last_emit_ns, 0)
        cmd(sim, t, False)
        cmd(sim, t + 10_000_000, True)
        t = run(sim, t + 10_000_000, 0.5, True)
        self.assertGreater(ld.last_emit_ns, 0)


class TestVehicleDerived(unittest.TestCase):
    def test_wheel_speeds_split_in_a_turn(self):
        sim, _ = make()
        v = sim.vehicle
        v.speed = v.speed_measured = 2.0
        v.yaw_rate = 1.0                                # 左旋回: 右輪が速い
        w = v.wheel_speed
        half = 1.0 * v.spec.track / 2
        self.assertAlmostEqual(w[2], 2.0 - half, places=6)
        self.assertAlmostEqual(w[3], 2.0 + half, places=6)
        self.assertLess(w[0], w[1])
        v.yaw_rate = 0.0
        self.assertEqual(v.wheel_speed[2], v.wheel_speed[3])

    def test_raycast_has_no_systematic_bias(self):
        import numpy as np
        from sim.course import Course as C
        res = 0.02
        grid = np.zeros((100, 100), dtype=bool)
        grid[:, 60:] = True                              # 壁の面は x = 1.20m
        c = C(name="t", path=Path("t"), resolution=res, origin=(0.0, 0.0),
              start=(0.1, 0.5, 0.0), grid=grid)
        errs = [float(c.raycast(0.1 + k * 0.0013, 0.5, np.array([0.0]), 3.0)[0]) - (1.2 - 0.1 - k * 0.0013)
                for k in range(16)]
        self.assertLess(abs(sum(errs) / len(errs)), 0.003)      # 平均の偏りが 3mm 未満（半歩戻す前は約 5mm）
        self.assertLess(max(abs(e) for e in errs), res * 0.5 + 1e-6)

    def test_can_back_away_after_a_collision(self):
        sim, _ = make()
        v = sim.vehicle
        v.speed = 1.0
        v.note_collision(True)
        self.assertEqual(v.speed, 0.0)
        v.speed = 0.5                                    # 壁へ向かう動きは殺す
        v.note_collision(True)
        self.assertEqual(v.speed, 0.0)
        v.speed = -0.3                                   # 逆向きは通す
        v.note_collision(True)
        self.assertEqual(v.speed, -0.3)
        self.assertEqual(v.collisions, 1)


if __name__ == "__main__":
    unittest.main()
