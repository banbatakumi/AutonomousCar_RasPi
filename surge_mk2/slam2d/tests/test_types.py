"""`core/types.py`のPose2D代数のテスト。"""

import math
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from slam2d.core.types import Pose2D, between, compose, inverse, wrap_angle  # noqa: E402


class TestWrapAngle(unittest.TestCase):
    def test_stays_in_range(self):
        for a in [0.0, math.pi, -math.pi, 3 * math.pi, -3 * math.pi, 10.0, -10.0]:
            w = wrap_angle(a)
            self.assertGreater(w, -math.pi - 1e-9)
            self.assertLessEqual(w, math.pi + 1e-9)

    def test_equivalent_angle(self):
        self.assertAlmostEqual(wrap_angle(2 * math.pi), 0.0, places=9)
        self.assertAlmostEqual(wrap_angle(math.pi + 0.1), -math.pi + 0.1, places=9)


class TestCompose(unittest.TestCase):
    def test_identity_is_neutral(self):
        a = Pose2D(1.5, -2.0, 0.7)
        ident = Pose2D(0.0, 0.0, 0.0)
        r = compose(a, ident)
        self.assertAlmostEqual(r.x, a.x, places=9)
        self.assertAlmostEqual(r.y, a.y, places=9)
        self.assertAlmostEqual(r.yaw, a.yaw, places=9)

    def test_pure_translation(self):
        a = Pose2D(1.0, 2.0, 0.0)
        b = Pose2D(3.0, 4.0, 0.0)
        r = compose(a, b)
        self.assertAlmostEqual(r.x, 4.0, places=9)
        self.assertAlmostEqual(r.y, 6.0, places=9)

    def test_rotation_then_translation(self):
        # 90度回転した姿勢から見て「前に1m」は、世界座標では「+y方向に1m」になる
        a = Pose2D(0.0, 0.0, math.pi / 2)
        b = Pose2D(1.0, 0.0, 0.0)
        r = compose(a, b)
        self.assertAlmostEqual(r.x, 0.0, places=9)
        self.assertAlmostEqual(r.y, 1.0, places=9)


class TestInverse(unittest.TestCase):
    def test_compose_with_inverse_is_identity(self):
        for a in [Pose2D(1.0, 2.0, 0.3), Pose2D(-5.0, 3.0, -2.1), Pose2D(0.0, 0.0, 0.0)]:
            r = compose(a, inverse(a))
            self.assertAlmostEqual(r.x, 0.0, places=9)
            self.assertAlmostEqual(r.y, 0.0, places=9)
            self.assertAlmostEqual(wrap_angle(r.yaw), 0.0, places=9)


class TestBetween(unittest.TestCase):
    def test_recovers_relative_pose(self):
        a = Pose2D(1.0, 1.0, math.radians(30))
        b = Pose2D(2.0, 3.0, math.radians(80))
        rel = between(a, b)
        back = compose(a, rel)
        self.assertAlmostEqual(back.x, b.x, places=9)
        self.assertAlmostEqual(back.y, b.y, places=9)
        self.assertAlmostEqual(wrap_angle(back.yaw - b.yaw), 0.0, places=9)

    def test_between_self_is_identity(self):
        a = Pose2D(5.0, -3.0, 1.2)
        rel = between(a, a)
        self.assertAlmostEqual(rel.x, 0.0, places=9)
        self.assertAlmostEqual(rel.y, 0.0, places=9)
        self.assertAlmostEqual(wrap_angle(rel.yaw), 0.0, places=9)


if __name__ == "__main__":
    unittest.main()
