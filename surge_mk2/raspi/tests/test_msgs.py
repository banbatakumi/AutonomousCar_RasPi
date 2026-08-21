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
    MAX_BRAKE_TORQUE_NM,
    MAX_TARGET_TORQUE_NM,
    SPEED_DEADBAND_MPS,
    DriveCmd,
    ScanAssembler,
    StateBuilder,
    command_from_cmd,
    decode_flags,
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
        self.assertFalse(scan.sector_seen[11 - 3])       # 反転後のセクタ番号
        self.assertEqual(scan.dist[270], 0.0)            # センサ 90° = 車両 270°、0 = 無効
        self.assertEqual(a.sectors_lost, 1)

    def test_sensor_angles_are_mirrored_into_the_vehicle_frame(self):
        """LD06 は裏向き実装なので生の角度は左右が鏡像。ここで直さないと GUI が反転する。"""
        a = ScanAssembler()
        for i in range(12):
            d = [0] * 30
            if i == 3:
                d[0] = 2000                              # センサ 90° = 車両 270°（右真横）
            if i == 0:
                d[0], d[1] = 1000, 3000                  # センサ 0°/1° = 車両 0°/359°
            a.feed(packets.LidarSector(sector_idx=i, duration_us=8300,
                                       rot_speed_dps=3594, dist=d), 0)
        scan = a.feed(self.sector(0), 0)
        self.assertAlmostEqual(scan.dist[270], 2.0)
        self.assertEqual(scan.dist[90], 0.0)             # 反転し忘れならここに出る
        self.assertAlmostEqual(scan.dist[0], 1.0)        # 前方は反転軸上なので動かない
        self.assertAlmostEqual(scan.dist[359], 3.0)

    def test_point_times_stay_continuous_across_sector_boundaries(self):
        """歪み補正で引く点ごとの時刻。**車両角のまま `30*s+j` と分解してはいけない。**

        反転で 30° の境界が 1° ずれ、`j=0` の点だけ隣のパケット由来になる。
        1点 0.3ms 間隔になる `duration` と受信間隔を与え、**全周で等間隔に並ぶか**を見る。
        誤った分解だと境界の点だけ1セクタぶん飛ぶ。
        """
        a = ScanAssembler()
        for s in range(12):
            # 距離[mm] にセンサ角を入れて、どの点がどこへ行ったか追えるようにする
            a.feed(packets.LidarSector(sector_idx=s, duration_us=8700, rot_speed_dps=3594,
                                       dist=[s * 30 + i for i in range(30)]),
                   s * 9_000_000)
        scan = a.feed(self.sector(0), 12 * 9_000_000)

        for deg in range(360):
            sensor = (360 - deg) % 360                   # `Scan.sector_dur_us` の式
            s, i = 11 - sensor // 30, sensor % 30
            self.assertEqual(round(scan.dist[deg] * 1000), sensor, f"deg={deg}")
            t = scan.sector_t_ns[s] + scan.sector_dur_us[s] * 1000 * i / 29
            self.assertAlmostEqual(t, 300_000 * sensor, delta=1, msg=f"deg={deg}")

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

    def test_max_accel_clamps_accel_limit(self):
        """★v0.11。`LIMITS.max_accel_m_s2` を超える `accel_limit` は丸める。"""
        c = command_from_cmd(
            DriveCmd(mode=1, arm=True, target_speed=1.0, accel_limit=5.0),
            allow_arm=True, max_accel=3.0)
        self.assertEqual(c.accel_limit, round(3.0 / 1e-3))

    def test_max_accel_does_not_touch_the_stm32_default_sentinel(self):
        """`accel_limit=0` は「STM32 の既定に任せる」の意味。クランプで足してはいけない。"""
        c = command_from_cmd(
            DriveCmd(mode=1, arm=True, target_speed=1.0, accel_limit=0.0),
            allow_arm=True, max_accel=3.0)
        self.assertEqual(c.accel_limit, 0)

    def test_no_max_accel_leaves_accel_limit_unclamped(self):
        """`LIMITS` 未受信（`max_accel=None`）なら従来通りそのまま送る。"""
        c = command_from_cmd(
            DriveCmd(mode=1, arm=True, target_speed=1.0, accel_limit=5.0),
            allow_arm=True, max_accel=None)
        self.assertEqual(c.accel_limit, round(5.0 / 1e-3))


class TestCommandAuxiliaries(unittest.TestCase):
    """v0.5 で増えた灯火・パッシング・制動トルク。**値の意味が変わった箇所**を押さえる。"""

    def _cmd(self, **kw) -> packets.Command:
        return command_from_cmd(DriveCmd(mode=1, arm=True, **kw), allow_arm=True)

    def test_light_mode_occupies_bit3_4(self):
        for mode, want in ((0, 0x00), (1, 0x08), (2, 0x10)):
            with self.subTest(light_mode=mode):
                c = self._cmd(light_mode=mode)
                self.assertEqual(c.flags & packets.CMD_FLG_LIGHT_MASK, want)

    def test_reserved_light_mode_3_is_never_sent(self):
        """3 を受けた STM32 は NORMAL 扱いにする。**予約値に寄りかからない。**"""
        self.assertEqual(self._cmd(light_mode=3).flags & packets.CMD_FLG_LIGHT_MASK, 0)

    def test_passing_is_independent_of_light_mode(self):
        c = self._cmd(light_mode=0, passing=True)
        self.assertTrue(c.flags & packets.CMD_FLG_PASSING)
        self.assertEqual(c.flags & packets.CMD_FLG_LIGHT_MASK, 0)

    def test_brake_torque_scale(self):
        self.assertEqual(self._cmd(brake_torque=0.05).brake_torque, 500)

    def test_brake_torque_zero_means_unspecified(self):
        self.assertEqual(self._cmd(brake_torque=0.0).brake_torque, 0)

    def test_tiny_brake_torque_never_rounds_to_unspecified(self):
        """**0 は最大制動を意味する。** 弱い指定が丸めで 0 になると挙動が正反対になる。"""
        self.assertEqual(self._cmd(brake_torque=1e-9).brake_torque, 1)

    def test_brake_torque_clamped_to_stm32_maximum(self):
        c = self._cmd(brake_torque=10.0)
        self.assertEqual(c.brake_torque, round(MAX_BRAKE_TORQUE_NM / 1e-4))
        c.encode()                       # 例外が出ないこと

    def test_brake_torque_clamped_by_limits_max_torque(self):
        """★v0.11。`LIMITS.max_torque_nm` が `MAX_BRAKE_TORQUE_NM` より小さければそちらを使う。"""
        c = command_from_cmd(DriveCmd(mode=1, arm=True, brake_torque=10.0),
                             allow_arm=True, max_torque=0.05)
        self.assertEqual(c.brake_torque, round(0.05 / 1e-4))

    def test_max_torque_larger_than_default_does_not_relax_the_cap(self):
        """`LIMITS.max_torque_nm` が既定の `MAX_BRAKE_TORQUE_NM` より大きくても、
        既定の方（Pi 側の保守的な値）を超えて緩めてはいけない（小さい方を使う）。"""
        c = command_from_cmd(DriveCmd(mode=1, arm=True, brake_torque=10.0),
                             allow_arm=True, max_torque=999.0)
        self.assertEqual(c.brake_torque, round(MAX_BRAKE_TORQUE_NM / 1e-4))


class TestTorqueMode(unittest.TestCase):
    """v0.6 で追加した駆動トルク直接指令。`brake_torque` と違い 0 に特別な意味はない。"""

    def _cmd(self, **kw) -> packets.Command:
        return command_from_cmd(DriveCmd(mode=1, arm=True, **kw), allow_arm=True)

    def test_torque_mode_occupies_bit6(self):
        self.assertEqual(self._cmd(torque_mode=True).flags & packets.CMD_FLG_TORQUE_MODE,
                          packets.CMD_FLG_TORQUE_MODE)
        self.assertEqual(self._cmd(torque_mode=False).flags & packets.CMD_FLG_TORQUE_MODE, 0)

    def test_target_torque_scale(self):
        self.assertEqual(self._cmd(target_torque=0.05).target_torque, 500)

    def test_target_torque_zero_is_just_zero(self):
        """`brake_torque` の 0 とは違い「未指定」の特別扱いはない。"""
        self.assertEqual(self._cmd(target_torque=0.0).target_torque, 0)

    def test_target_torque_supports_negative_for_reverse(self):
        self.assertEqual(self._cmd(target_torque=-0.05).target_torque, -500)

    def test_target_torque_clamped_to_max_both_directions(self):
        max_raw = round(MAX_TARGET_TORQUE_NM / 1e-4)
        c_pos = self._cmd(target_torque=10.0)
        c_neg = self._cmd(target_torque=-10.0)
        self.assertEqual(c_pos.target_torque, max_raw)
        self.assertEqual(c_neg.target_torque, -max_raw)
        c_pos.encode()                   # 例外が出ないこと
        c_neg.encode()

    def test_target_torque_clamped_by_limits_max_torque(self):
        """★v0.11。`LIMITS.max_torque_nm` が既定より小さければそちらを使う（両方向）。"""
        c = command_from_cmd(DriveCmd(mode=1, arm=True, target_torque=10.0),
                             allow_arm=True, max_torque=0.05)
        self.assertEqual(c.target_torque, round(0.05 / 1e-4))


class TestAutoStop(unittest.TestCase):
    """v0.7 で追加した超音波の自動停止。**Pi は許可を出すだけ**で判定はしない。"""

    def _cmd(self, **kw) -> packets.Command:
        return command_from_cmd(DriveCmd(mode=1, arm=True, **kw), allow_arm=True)

    def test_auto_stop_occupies_bit7(self):
        self.assertEqual(self._cmd(auto_stop=True).flags & packets.CMD_FLG_AUTO_STOP,
                         packets.CMD_FLG_AUTO_STOP)
        self.assertEqual(self._cmd(auto_stop=False).flags & packets.CMD_FLG_AUTO_STOP, 0)

    def test_auto_stop_does_not_disturb_other_flags(self):
        """bit7 まで使い切ったので、隣のビットを壊していないことを毎回見ておく。

        `light_mode=2` は bit4 だけなので全ビット同時には立たない（bit3 が 0 で 0xF7）。
        **light=3 は予約なので送らない**（`command_from_cmd` の該当箇所を参照）。
        """
        c = self._cmd(auto_stop=True, torque_mode=True, brake=True,
                      light_mode=2, passing=True, horn=True)
        self.assertEqual(c.flags, 0xF7)

    def test_auto_stop_does_not_touch_speed_or_torque(self):
        """**制動は STM32 側で完結する。** Pi 側で指令値を落とす二重制御はしない。"""
        c = self._cmd(auto_stop=True, target_speed=0.5)
        self.assertEqual(c.target_speed, 500)

    def test_telemetry_flag_decodes(self):
        self.assertTrue(decode_flags(packets.FLG_AUTO_STOP_ACTIVE)["auto_stop_active"])
        self.assertFalse(decode_flags(0)["auto_stop_active"])


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
