"""`sim/vehicle.py` の `SpeedController`（ファームの速度制御）と `SecondOrder` のテスト。

`SpeedController` は STM32 の `drive.c`（目標のランプ→PI→トルク上限）を写したもので、
シム・学習環境・同定の解析（`tools/sysid/fit.py`）が同じ実装を使う。ファームの振る舞いの
うち、同定の結果を左右するもの（ランプの上限・静止での惰行・制動での初期化・積分による
定常偏差の打ち消し）を固定しておく。
"""

import math
import sys
import unittest
from dataclasses import replace
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from sim.vehicle import DriveInput, SecondOrder, SpeedController, VehicleModel, VehicleSpec  # noqa: E402

SPEC = VehicleSpec(speed_kp=0.25, speed_ki=0.5, speed_plant_gain=25.0, rolling_resistance=0.3,
                   speed_filter_s=0.00995)


def _run(ctl: SpeedController, cmd: DriveInput, v: float, secs: float, dt: float = 0.001):
    out = []
    for _ in range(int(secs / dt)):
        v = ctl.step(cmd, v, dt)
        out.append(v)
    return v, out


class TestSpeedController(unittest.TestCase):
    def test_acceleration_is_capped_by_the_firmware_ramp(self):
        """COMMAND の accel_limit が 6.0 でも、ファームは 3.0m/s² で目標をランプさせる。"""
        ctl = SpeedController(SPEC)
        v, out = _run(ctl, DriveInput(armed=True, target_speed=3.0, accel_limit=6.0), 0.0, 0.5)
        # 目標は 3.0m/s² でしか上がらず、車速はそれを追う（PI が追いつく瞬間だけ加速度は3.0を超える）
        self.assertAlmostEqual(ctl.ref, 1.5, delta=0.01)
        self.assertLess(v, 1.5)

    def test_integral_removes_the_rolling_resistance_offset(self):
        ctl = SpeedController(SPEC)
        v, _ = _run(ctl, DriveInput(armed=True, target_speed=1.0, accel_limit=3.0), 0.0, 8.0)
        self.assertAlmostEqual(v, 1.0, delta=0.01)

    def test_standstill_coasts_and_brake_resets(self):
        ctl = SpeedController(SPEC)
        v, _ = _run(ctl, DriveInput(armed=True, target_speed=0.0), 0.0, 1.0)
        self.assertEqual(v, 0.0)
        _run(ctl, DriveInput(armed=True, target_speed=1.0, accel_limit=3.0), 0.0, 1.0)
        _run(ctl, DriveInput(armed=True, brake=True), 1.0, 0.01)
        self.assertEqual((ctl.ref, ctl.integral), (0.0, 0.0))

    def test_launch_bypasses_the_ramp_and_hands_over_at_target(self):
        """ローンチコントロール（v0.20）: 停止からの発進はランプとPIを迂回し、目標の手前でPIへ戻る。"""
        spec = replace(SPEC, speed_torque_max_nm=0.30, drive_accel_m_s2=2.5)
        cmd = DriveInput(armed=True, target_speed=1.5, accel_limit=3.0)
        _, plain = _run(SpeedController(spec), cmd, 0.0, 2.0)
        ctl = SpeedController(spec, launch_exit_torque_nm=0.006)
        v, flags = 0.0, []
        out = []
        for _ in range(2000):
            v = ctl.step(replace(cmd, launch=True), v, 0.001)
            out.append(v)
            flags.append(ctl.launching)
        self.assertTrue(flags[0])
        end = max(i for i, f in enumerate(flags) if f)
        self.assertFalse(any(flags[end + 1:]))                          # 1回で終わる
        t95 = lambda o: next(i for i, x in enumerate(o) if x >= 0.95 * 1.5)   # noqa: E731
        self.assertLess(t95(out), t95(plain) - 100)                     # 0.1s 以上早い
        self.assertLess(max(out), 1.5 + 0.08)
        self.assertGreater(min(out[end:]), 1.5 - 0.08)

    def test_launch_needs_a_large_step_but_not_a_standstill(self):
        ctl = SpeedController(SPEC)
        _run(ctl, DriveInput(armed=True, target_speed=0.4, launch=True), 0.0, 0.2)
        self.assertFalse(ctl.launching)
        ctl = SpeedController(SPEC)                                     # 低速で走っている途中からは始める
        ctl.reset(v=0.5, ref=0.5)
        ctl.step(DriveInput(armed=True, target_speed=2.0, launch=True), 0.5, 0.001)
        self.assertTrue(ctl.launching)
        ctl = SpeedController(SPEC)                                     # 目標と逆向きに動いている間は始めない
        ctl.reset(v=-0.6, ref=-0.6)
        ctl.step(DriveInput(armed=True, target_speed=2.0, launch=True), -0.6, 0.001)
        self.assertFalse(ctl.launching)

    def test_launch_is_rearmed_by_braking(self):
        ctl = SpeedController(SPEC)
        go = DriveInput(armed=True, target_speed=1.0, launch=True)
        v, _ = _run(ctl, go, 0.0, 1.0)
        self.assertFalse(ctl.launching)
        v, _ = _run(ctl, DriveInput(armed=True, brake=True), v, 2.0)
        self.assertLess(abs(v), 0.05)
        ctl.step(go, v, 0.001)
        self.assertTrue(ctl.launching)

    def test_overshoot_is_second_order(self):
        """ファームの PI は2次の応答で行き過ぎる（1次遅れでは表せない。第三者検証の件）。"""
        ctl = SpeedController(replace(SPEC, rolling_resistance=0.0))
        _, out = _run(ctl, DriveInput(armed=True, target_speed=1.2, accel_limit=3.0), 0.0, 3.0)
        self.assertGreater(max(out), 1.2 + 0.01)

    def test_vehicle_model_uses_it_only_when_kp_is_set(self):
        vm = VehicleModel(SPEC, (0, 0, 0))
        self.assertIsNotNone(vm._speed_ctl)
        self.assertIsNone(VehicleModel(VehicleSpec(), (0, 0, 0))._speed_ctl)


class TestSecondOrder(unittest.TestCase):
    def test_step_response(self):
        z = 0.4
        so = SecondOrder(20.0, z)
        ys = [so.step(1.0, 0.001) for _ in range(3000)]
        self.assertAlmostEqual(ys[-1], 1.0, places=4)
        self.assertAlmostEqual(max(ys) - 1.0, math.exp(-math.pi * z / math.sqrt(1 - z * z)), delta=0.005)

    def test_independent_of_step_size(self):
        a, b = SecondOrder(15.0, 0.3), SecondOrder(15.0, 0.3)
        ya = [a.step(1.0, 0.001) for _ in range(500)][-1]
        yb = [b.step(1.0, 0.005) for _ in range(100)][-1]
        self.assertAlmostEqual(ya, yb, places=9)


if __name__ == "__main__":
    unittest.main()
