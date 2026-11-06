"""`sim/surface.py`（場所と時刻で変わる路面）と、それを使う真値の車・仮想 STM32 のテスト。"""

import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from raspi.proto.generated import packets  # noqa: E402
from sim.surface import Surface  # noqa: E402
from sim.vehicle import GRAVITY_MPS2, DriveInput, VehicleModel, VehicleSpec  # noqa: E402


class TestSurface(unittest.TestCase):
    def test_base_disc_and_time(self):
        s = Surface(0.4)
        self.assertEqual(s.scale(0.0, 0.0, 0.0), (1.0, 1.0))
        s.set_base(0.6)
        s.add_disc(2.0, 1.0, 0.5, 0.2)
        s.add_change(10.0, 0.3, 0.5)
        self.assertAlmostEqual(s.scale(0.0, 0.0, 0.0)[0], 1.5)
        self.assertAlmostEqual(s.scale(0.0, 0.0, 0.0)[1], 1.5)     # 前後も同じ割合
        self.assertAlmostEqual(s.scale(2.2, 1.0, 0.0)[0], 0.5)     # 円の中
        lat, lon = s.scale(0.0, 0.0, 11.0)                         # 時刻で変わる・前後は別の値
        self.assertAlmostEqual(lat, 0.75)
        self.assertAlmostEqual(lon, 1.25)
        self.assertAlmostEqual(s.scale(2.2, 1.0, 11.0)[0], 0.5)    # 円の中は時刻で変わらない
        self.assertAlmostEqual(s.mu_at(0.0, 0.0, 11.0), 0.3)


class TestVehicleOnSurface(unittest.TestCase):
    def _run(self, lat: float, lon: float, steer: float, speed: float, seconds: float = 3.0):
        spec = VehicleSpec.load()
        v = VehicleModel(spec, (0.0, 0.0, 0.0))
        v.grip_lat, v.grip_long = lat, lon
        v.apply(DriveInput(armed=True, target_speed=speed, target_steer=steer))
        capped, a_max = False, 0.0
        for _ in range(int(seconds / 0.001)):
            v.step(0.001)
            capped = capped or v.accel_capped
            a_max = max(a_max, v.accel_x)
        return v, capped, a_max

    def test_lateral_limit_scales(self):
        spec = VehicleSpec.load()
        for lat in (0.7, 1.0, 1.4):
            v, _, _ = self._run(lat, 1.0, spec.max_steer, 3.0)
            self.assertLessEqual(abs(v.accel_lateral), spec.mu * lat * GRAVITY_MPS2 * 1.001)
            self.assertGreater(abs(v.accel_lateral), spec.mu * lat * GRAVITY_MPS2 * 0.9)

    def test_drive_accel_limit_scales_and_flags_tc(self):
        spec = VehicleSpec.load()
        if spec.drive_accel_m_s2 <= 0.0 or spec.speed_kp <= 0.0:
            self.skipTest("加速の上限・速度PIが未同定")
        _, capped_lo, a_lo = self._run(1.0, 0.5, 0.0, 3.0, 1.0)
        _, _, a_hi = self._run(1.0, 1.0, 0.0, 3.0, 1.0)
        self.assertTrue(capped_lo)
        self.assertLess(a_lo, a_hi * 0.6)
        self.assertAlmostEqual(a_lo, spec.drive_accel_m_s2 * 0.5 - spec.rolling_resistance, delta=0.2)

    def test_default_scale_changes_nothing(self):
        a, _, _ = self._run(1.0, 1.0, 0.2, 2.0, 1.0)
        spec = VehicleSpec.load()
        b = VehicleModel(spec, (0.0, 0.0, 0.0))
        b.apply(DriveInput(armed=True, target_speed=2.0, target_steer=0.2))
        for _ in range(1000):
            b.step(0.001)
        self.assertEqual((a.x, a.y, a.yaw, a.speed), (b.x, b.y, b.yaw, b.speed))


class TestTcFlag(unittest.TestCase):
    def test_flag_follows_accel_cap(self):
        """★ 作り物の TC の旗: 駆動の加速度がグリップの上限で頭打ちの間だけ立つ。"""
        from sim.course import list_courses
        from sim.link import create_sim_link
        course = next(p for p in list_courses() if p.stem == "toyota2")
        link = create_sim_link(course, with_channel=False, with_lidar=False)
        sim = link.sim
        try:
            sim._input = DriveInput(armed=True)
            self.assertFalse(sim._flags(0) & packets.FLG_TC_ACTIVE)
            sim.vehicle._speed_ctl.accel_capped = True
            self.assertTrue(sim._flags(0) & packets.FLG_TC_ACTIVE)
            sim._input = DriveInput(armed=False)
            self.assertFalse(sim._flags(0) & packets.FLG_TC_ACTIVE)
        finally:
            link.close()


class TestCourseSurface(unittest.TestCase):
    def test_course_json_sets_floor_and_switching_resets_it(self):
        """コースの JSON の `"surface"` が仮想 STM32 の床になり、床の無いコースへ替えたら校正の床へ戻る。"""
        import json
        import tempfile
        from sim.course import Course, list_courses
        from sim.link import create_sim_link
        src = next(p for p in list_courses() if p.stem == "normal")
        meta = json.loads(src.read_text(encoding="utf-8"))
        meta["surface"] = {"mu_scale": 1.25, "mu_long_scale": 1.1}
        with tempfile.TemporaryDirectory() as d:
            p = Path(d) / "grippy.json"
            p.write_text(json.dumps(meta), encoding="utf-8")
            link = create_sim_link(p, with_channel=False, with_lidar=False)
            sim = link.sim
            try:
                self.assertIsNotNone(sim.surface)
                lat, lon = sim.surface.scale(0.0, 0.0, 0.0)
                self.assertAlmostEqual(lat, 1.25)
                self.assertAlmostEqual(lon, 1.1)
                sim.vehicle.grip_lat, sim.vehicle.grip_long = lat, lon
                sim.set_course(Course.load(src))
                self.assertIsNone(sim.surface)
                self.assertEqual((sim.vehicle.grip_lat, sim.vehicle.grip_long), (1.0, 1.0))
            finally:
                link.close()

    def test_no_surface_key_is_none(self):
        self.assertIsNone(Surface.for_course(0.4, object()))
        self.assertIsNone(Surface.for_course(0.4, type("C", (), {"surface": {"mu_scale": 1.0}})()))


if __name__ == "__main__":
    unittest.main()
