"""メッセージ変換のテスト。

**ここで守りたいのは「スケールと射影」**。単位を間違えても値は出てしまい、
「なんとなく動くが 10倍おかしい」という形でしか症状が出ない。
"""

import math
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from raspi.msgs import (  # noqa: E402
    SPEED_DEADBAND_MPS,
    DriveCmd,
    ScanAssembler,
    StateBuilder,
    command_from_cmd,
)
from raspi.msgs.types import type_for_topic  # noqa: E402
from raspi.proto import packets  # noqa: E402


def telem(**kw) -> packets.Telemetry:
    base = dict(t_us=1000, flags=0, md_status=[0x10, 0x10, 0x10])
    base.update(kw)
    return packets.Telemetry(**base)


class TestScales(unittest.TestCase):
    def test_si_conversion(self):
        t = telem(speed=1234, yaw_rate=-500, steer_actual=2000,
                  wheel_speed=[1000, 1100, 1200, 1300],
                  accel_x=10, accel_y=-20, accel_z=9810,
                  motor_current=[1500, -1500, 300],
                  torque_cmd=[1000, -1000],
                  batt_voltage_drive=228, batt_voltage_signal=222,
                  batt_current_drive=40, batt_current_signal=45,
                  us_front=60, us_rear=0, temp=[30, 31, 32, 33])
        st = StateBuilder().build(t, 12345)
        self.assertAlmostEqual(st.speed, 1.234)
        self.assertAlmostEqual(st.yaw_rate, -0.5)
        self.assertAlmostEqual(st.steer_actual, 0.2)
        self.assertAlmostEqual(st.accel[2], 9.81)
        self.assertAlmostEqual(st.motor_current[0], 1.5)
        self.assertAlmostEqual(st.torque_cmd[1], -0.1)
        self.assertAlmostEqual(st.batt_voltage[0], 11.4)
        self.assertAlmostEqual(st.batt_current[0], 2.0)
        self.assertAlmostEqual(st.batt_current[1], 0.9)
        self.assertAlmostEqual(st.us_front, 1.2)
        self.assertEqual(st.t_capture, 12345)

    def test_ultrasonic_zero_is_none_not_zero_metres(self):
        """`0 = 無効`。0.0m のまま流すと「目の前に壁がある」と読める。"""
        st = StateBuilder().build(telem(us_front=0, us_rear=50), 0)
        self.assertIsNone(st.us_front)
        self.assertAlmostEqual(st.us_rear, 1.0)

    def test_temp_is_none_when_md_is_silent(self):
        """`comm_ok=0` の MD の温度は信じない（status が最後の値で固まる）。"""
        t = telem(temp=[80, 81, 82, 83], md_status=[0x10, 0x00, 0x10])
        st = StateBuilder().build(t, 0)
        self.assertEqual(st.temp[0], 80)
        self.assertIsNone(st.temp[1])       # comm_ok が落ちている MD
        self.assertEqual(st.temp[3], 83)    # MCU は MD ではないので常に有効

    def test_flags_are_decoded(self):
        f = (packets.FLG_ARMED | packets.FLG_TC_ACTIVE
             | packets.FLG_DRIVE_POWER_LOCKED
             | packets.FLG_FAULT_DRIVE_UNDERVOLTAGE | packets.Mode.MANUAL)
        st = StateBuilder().build(telem(flags=f), 0)
        self.assertTrue(st.armed)
        self.assertTrue(st.tc_active)
        self.assertTrue(st.drive_power_locked)
        self.assertEqual(st.mode, packets.Mode.MANUAL)
        self.assertEqual(st.faults, ["drive_undervoltage"])

    def test_stopped_flag_keeps_raw_speed(self):
        """デッドバンドは生値を書き換えない。判断だけを別に持つ。"""
        v = int((SPEED_DEADBAND_MPS / 2) * 1000)
        st = StateBuilder().build(telem(speed=v), 0)
        self.assertTrue(st.stopped)
        self.assertAlmostEqual(st.speed, v * 1e-3)   # 0 に潰していない


class TestOdometryProjection(unittest.TestCase):
    """`uart_protocol.md` §5.3 — 射影は累積値ではなく差分に対して行う。"""

    #: 50Hz・1m/s の1周期ぶん = 2cm（0.1mm 単位で 200）。**実機と同じ刻みで検査する**
    STEP = 200

    def drive(self, b, total_ticks, steer=0, start=0):
        """`total_ticks`（0.1mm 単位）を実機と同じ 2cm 刻みで走らせる。"""
        pos = start
        b.build(telem(odom_dist=[pos, pos], steer_actual=steer), 0)
        for _ in range(total_ticks // self.STEP):
            pos += self.STEP
            b.build(telem(odom_dist=[pos, pos], steer_actual=steer), 0)
        return pos

    def test_straight_line_matches_front_wheel_distance(self):
        b = StateBuilder()
        self.drive(b, 10_000)                          # 1.0m 前進
        self.assertAlmostEqual(b.odom_center, 1.0, places=6)

    def test_projection_applies_to_the_increment_not_the_total(self):
        """直進100m のあと大舵角を切っても、**累積距離は縮まない**。

        累積値に cos を掛ける実装だと、ここで一気に半分になる。
        """
        b = StateBuilder()
        pos = self.drive(b, 1_000_000)                 # 100m 直進
        self.assertAlmostEqual(b.odom_center, 100.0, places=4)

        d60 = int(math.radians(60) / 1e-4)
        # 舵を切っただけ（動いていない）。ここで累積が縮んだら実装が誤り
        b.build(telem(odom_dist=[pos, pos], steer_actual=d60), 0)
        self.assertAlmostEqual(b.odom_center, 100.0, places=4)

        self.drive(b, 10_000, steer=d60, start=pos)    # 前輪で 1.0m
        # 前輪 1.0m ぶんの移動 → 中心線では cos(60°) = 0.5m
        self.assertAlmostEqual(b.odom_center, 100.5, places=3)

    def test_first_sample_only_sets_the_baseline(self):
        b = StateBuilder()
        b.build(telem(odom_dist=[5_000_000, 5_000_000]), 0)
        self.assertEqual(b.odom_center, 0.0)

    def test_huge_gap_does_not_move_the_accumulator(self):
        """再接続で累積値が飛んでも、位置推定を巻き込まない。"""
        b = StateBuilder()
        b.build(telem(odom_dist=[0, 0]), 0)
        b.build(telem(odom_dist=[10_000_000, 10_000_000]), 0)    # 1000m 分の飛び
        self.assertEqual(b.odom_center, 0.0)
        self.assertEqual(b.odom_jumps, 1)

    def test_front_slip_is_projected_but_rear_is_not(self):
        """射影を忘れると舵角30°で常時 15.5% のスリップが出て見える。"""
        d30 = int(math.radians(30) / 1e-4)
        # 前輪周速 1.155 m/s は、中心線 1.0 m/s を舵角30°で走ったときの値
        t = telem(speed=1000, steer_actual=d30,
                  wheel_speed=[1155, 1155, 1000, 1000])
        st = StateBuilder().build(t, 0)
        self.assertAlmostEqual(st.slip_front[0], 0.0, places=3)
        self.assertAlmostEqual(st.slip_rear[0], 0.0, places=3)


class TestScanAssembler(unittest.TestCase):
    @staticmethod
    def sector(idx, val=1000, cls=packets.LidarSector, **kw):
        return cls(sector_idx=idx, t_start_us=idx * 8300, duration_us=8300,
                   rot_speed_dps=3594, dist=[val] * 30, **kw)

    def test_one_full_turn(self):
        a = ScanAssembler()
        for i in range(12):
            self.assertIsNone(a.feed(self.sector(i), 1000 + i))
        scan = a.feed(self.sector(0), 2000)      # 番号が戻った＝1周完了
        self.assertIsNotNone(scan)
        self.assertTrue(all(scan.sector_seen))
        self.assertEqual(len(scan.dist), 360)
        self.assertAlmostEqual(scan.dist[0], 1.0)
        self.assertAlmostEqual(scan.rot_speed_dps, 3594.0)
        self.assertEqual(a.sectors_lost, 0)

    def test_a_lost_sector_does_not_stall_the_turn(self):
        """「12個そろったら」だと1個落ちた周が永久に完成せず次と混ざる。"""
        a = ScanAssembler()
        for i in (0, 1, 2, 4, 5, 6, 7, 8, 9, 10, 11):    # 3 が欠ける
            a.feed(self.sector(i), 0)
        scan = a.feed(self.sector(0), 0)
        self.assertIsNotNone(scan)
        self.assertFalse(scan.sector_seen[3])
        self.assertEqual(scan.dist[3 * 30], 0.0)         # 0 = 無効
        self.assertEqual(a.sectors_lost, 1)

    def test_compressed_marks_saturation_instead_of_a_hit(self):
        """255 は「5.10m ちょうど」ではなく「5.10m 以上」。壁として打たない。"""
        a = ScanAssembler()
        for i in range(12):
            a.feed(self.sector(i, val=255, cls=packets.LidarSectorC), 0)
        scan = a.feed(self.sector(0, val=100, cls=packets.LidarSectorC), 0)
        self.assertEqual(scan.lidar_format, 2)
        self.assertTrue(scan.saturated[0])
        self.assertAlmostEqual(scan.dist[0], 5.10)

    def test_intensity_is_carried_through(self):
        a = ScanAssembler()
        for i in range(12):
            a.feed(self.sector(i, cls=packets.LidarSectorI,
                               intensity=[200] * 30), 0)
        scan = a.feed(self.sector(0, cls=packets.LidarSectorI,
                                  intensity=[200] * 30), 0)
        self.assertEqual(scan.lidar_format, 1)
        self.assertEqual(scan.intensity[0], 200)


class TestCommandGate(unittest.TestCase):
    """**モータが回る条件は1箇所に閉じる。** ここが緩むと安全設計が崩れる。"""

    live = DriveCmd(mode=1, arm=True, target_speed=2.0, target_steer=0.5)

    def test_without_allow_arm_everything_becomes_disarm(self):
        c = command_from_cmd(self.live, allow_arm=False)
        self.assertEqual(c.mode, packets.Mode.DISARM)
        self.assertEqual(c.flags, 0)
        self.assertEqual(c.target_speed, 0)
        self.assertEqual(c.target_steer, 0)

    def test_with_allow_arm_the_values_pass_through(self):
        c = command_from_cmd(self.live, allow_arm=True)
        self.assertEqual(c.mode, 1)
        self.assertTrue(c.flags & packets.CMD_FLG_ARM)
        self.assertEqual(c.target_speed, 2000)
        self.assertEqual(c.target_steer, 5000)

    def test_clamps_are_applied(self):
        c = command_from_cmd(self.live, allow_arm=True,
                             max_speed=0.5, max_steer=0.2)
        self.assertEqual(c.target_speed, 500)
        self.assertEqual(c.target_steer, 2000)

    def test_out_of_range_saturates_instead_of_raising(self):
        """クランプ漏れで struct が例外を投げると、送信が止まって
        STM32 が COMMAND 途絶の自動ブレーキに入る。分かりにくい壊れ方をする。"""
        c = command_from_cmd(DriveCmd(mode=1, target_speed=1e6, target_steer=-1e6),
                             allow_arm=True)
        self.assertEqual(c.target_speed, 32767)
        self.assertEqual(c.target_steer, -32768)
        c.encode()                       # 例外が出ないこと

    def test_reserved_mode_3_is_never_sent(self):
        c = command_from_cmd(DriveCmd(mode=3), allow_arm=True)
        self.assertEqual(c.mode, packets.Mode.DISARM)


class TestTopicRegistry(unittest.TestCase):
    def test_prefix_lookup(self):
        from raspi.msgs import Heartbeat, ImageRef, VehicleState

        self.assertIs(type_for_topic("vehicle_state"), VehicleState)
        self.assertIs(type_for_topic("image/rear"), ImageRef)
        self.assertIs(type_for_topic("hb/io"), Heartbeat)

    def test_unknown_topic_raises(self):
        """型が分からないまま dict で流すと、フィールド名の typo が実行時まで残る。"""
        with self.assertRaises(KeyError):
            type_for_topic("nope")


if __name__ == "__main__":
    unittest.main()
