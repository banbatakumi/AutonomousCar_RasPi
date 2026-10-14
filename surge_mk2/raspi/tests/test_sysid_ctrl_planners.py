"""制御の同定の planner（`sysid_wheel`・`sysid_tyre`）の約束。

試験→解析が真値を復元できるかは `tools/ctrl_tune/tests`（車両モデルとの閉ループ）が見る。
ここは車両モデル無しで言える約束だけ。
"""

import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from raspi.auto._sysid_common import SETTLE_S  # noqa: E402
from raspi.auto.registry import PLANNERS  # noqa: E402
from raspi.auto.sysid_tyre import SysIdTyre  # noqa: E402
from raspi.auto.sysid_wheel import SysIdWheel  # noqa: E402
from raspi.core.control_params import OVERRIDABLE  # noqa: E402
from raspi.msgs.types import AutoState, DriveCmd, VehicleState  # noqa: E402

DT = 0.1


def vs(speed=0.0, rear=0.0, odom=0.0, armed=True):
    return VehicleState(speed=speed, wheel_speed=[speed, speed, rear, rear], odom_dist=[odom, odom],
                        armed=armed)


def start(planner, state):
    """試験開始を押し、静止区間を抜けるまで進める。"""
    planner.set_engaged(True)
    st = planner.plan(None, state, {}, DT)
    for _ in range(int(SETTLE_S / DT) + 1):
        st = planner.plan(None, state, {}, DT)
    return st


class TestCommon(unittest.TestCase):
    CLASSES = (SysIdWheel, SysIdTyre)

    def test_registered_as_sysid(self):
        for cls in self.CLASSES:
            self.assertIs(PLANNERS[cls.id], cls)
            self.assertEqual(cls.category, "sysid")

    def test_no_override_before_start(self):
        """試験開始を押すまで、ファームの設定に触らない（TC を切らない）。"""
        for cls in self.CLASSES:
            p = cls()
            p.set_engaged(False)
            st = p.plan(None, vs(), {}, DT)
            self.assertEqual(st.fw_overrides, {}, cls.id)
            self.assertFalse(st.torque_mode)

    def test_override_names_are_known_to_io_node(self):
        for cls in self.CLASSES:
            p = cls()
            st = start(p, vs())
            self.assertTrue(st.fw_overrides, cls.id)
            self.assertTrue(set(st.fw_overrides) <= set(OVERRIDABLE), cls.id)

    def test_fields_survive_the_bus(self):
        """`fw_overrides`・`torque_mode` がバスのメッセージ（msgspec）を往復できる。"""
        import msgspec
        st = AutoState(torque_mode=True, target_torque=0.05, fw_overrides={"tc_enable": 0.0})
        back = msgspec.msgpack.decode(msgspec.msgpack.encode(st), type=AutoState)
        self.assertEqual(back.fw_overrides, {"tc_enable": 0.0})
        cmd = DriveCmd(fw_overrides={"tv_test_moment_nm": 0.1})
        self.assertEqual(msgspec.msgpack.decode(msgspec.msgpack.encode(cmd), type=DriveCmd).fw_overrides,
                         {"tv_test_moment_nm": 0.1})


class TestWheel(unittest.TestCase):
    def _until_spinning(self, p):
        st = start(p, vs())
        for _ in range(20):
            if st.torque_mode:
                return st
            st = p.plan(None, vs(), {}, DT)
        self.fail("回し始めない")

    def test_small_torque_pulses_then_brake(self):
        p = SysIdWheel()
        st = start(p, vs())
        seen_torque, seen_brake = [], False
        for _ in range(200):
            st = p.plan(None, vs(), {}, DT)
            if st.torque_mode:
                seen_torque.append(st.target_torque)
            seen_brake = seen_brake or st.brake
            if st.reason == "完了":
                break
        self.assertEqual(st.reason, "完了")
        self.assertTrue(seen_brake)
        self.assertLessEqual(max(seen_torque), 0.02)       # 浮いた輪はわずかなトルクで吹け上がる
        self.assertEqual(st.fw_overrides.get("tc_enable"), 0.0)
        self.assertNotIn("wheel_lift_guard_enable", st.fw_overrides)   # 周速の上限は残す

    def test_aborts_when_the_car_moves(self):
        """前輪が回った＝後輪が接地している。すぐ止める。"""
        p = SysIdWheel()
        self._until_spinning(p)
        p.on_vehicle_state(vs(speed=0.4, rear=0.5))
        st = p.plan(None, vs(speed=0.4, rear=0.5), {}, DT)
        self.assertTrue(st.brake)
        self.assertTrue(st.reason.startswith("中止"))

    def test_overspeed_cuts_the_pulse_short(self):
        p = SysIdWheel()
        self._until_spinning(p)
        p.on_vehicle_state(vs(rear=4.0))
        st = p.plan(None, vs(rear=4.0), {}, DT)
        self.assertTrue(st.brake)


class TestTyre(unittest.TestCase):
    def test_turns_tc_and_abs_off_but_keeps_wheel_speed_cap(self):
        st = start(SysIdTyre(), vs())
        self.assertEqual(st.fw_overrides["tc_enable"], 0.0)
        self.assertEqual(st.fw_overrides["abs_enable"], 0.0)
        self.assertNotIn("wheel_lift_guard_enable", st.fw_overrides)

    def test_torque_rises_in_steps_and_spin_ends_the_drive(self):
        p = SysIdTyre()
        start(p, vs())
        torques = []
        speed = 0.4
        for k in range(40):
            st = p.plan(None, vs(speed=speed, rear=speed, odom=0.05 * k), {}, DT)
            if st.torque_mode:
                torques.append(st.target_torque)
                if len(torques) == 4:
                    p.on_vehicle_state(vs(speed=1.0, rear=3.0))     # 空転
            elif torques:
                break
        self.assertEqual(len(torques), 4)
        self.assertTrue(all(b > a for a, b in zip(torques, torques[1:])))
        self.assertTrue(st.brake)

    def test_drive_ends_where_it_can_still_stop_counting_the_ongoing_acceleration(self):
        """駆動の打ち切りは、判断の遅れの間も加速し続けるぶんを入れた停止距離で決める。"""
        from raspi.auto._sysid_common import stop_distance
        v = 1.6
        with_accel = stop_distance(v, True, 3.0)
        self.assertGreater(with_accel, stop_distance(v) + 0.15)
        # 加速ぶんを入れない見積もりなら止まれる、入れると止まれない位置
        odom = 2.0 - 0.5 * (with_accel + stop_distance(v))
        p = SysIdTyre()
        start(p, vs())
        p._enter(p._DRIVE)
        st = p.plan(None, vs(speed=v, rear=v, odom=odom), {}, DT)
        self.assertTrue(st.brake, st.reason)
        self.assertFalse(st.torque_mode)

    def test_brake_backs_off_after_lock(self):
        p = SysIdTyre()
        p._gate.set_engaged(True)
        p._gate.tick(vs())
        p._enter(p._BRAKE)
        p.on_vehicle_state(vs(speed=1.5, rear=0.2))
        self.assertTrue(p._locked)

    def test_locked_brake_ends_early_with_a_conservative_decel(self):
        """ロック後の弱い制動は減速度が小さいので、直線を出そうなら早めに最大制動の局面へ移る。"""
        from raspi.auto._sysid_common import brake_decel_guess, stop_distance
        v = 1.5
        gentle = SysIdTyre.SETTINGS["gentle_brake_nm"]
        d_gentle, d_max = stop_distance(v, True, 0.0, brake_decel_guess(gentle)), stop_distance(v)
        self.assertGreater(d_gentle, d_max + 0.2)
        odom = 2.0 - 0.5 * (d_gentle + d_max)       # 最大制動なら止まれる、弱い制動では止まれない位置
        p = SysIdTyre()
        start(p, vs())
        p._enter(p._BRAKE)
        p._locked = True
        p.plan(None, vs(speed=v, rear=0.2, odom=odom), {}, DT)
        self.assertEqual(p._phase, p._HOLD)

    def test_md_loss_aborts_but_wheel_spin_does_not(self):
        p = SysIdTyre()
        start(p, vs())
        p._enter(p._DRIVE)
        for _ in range(50):                          # 後輪が前輪より速いのは試験の中身
            p.on_vehicle_state(vs(speed=0.5, rear=2.0))
        self.assertIsNone(p._guard.fault)
        lost = VehicleState(speed=0.5, armed=True, md_status=[0x31, 0x01, 0x31])
        p.on_vehicle_state(lost)
        st = p.plan(None, lost, {}, DT)
        self.assertIn("中止", st.reason)
        self.assertTrue(st.brake)


if __name__ == "__main__":
    unittest.main()
