"""`core/grid.py`（占有格子）のテスト。`raspi/tests/test_nav.py`の流儀を踏襲。"""

import math
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

import numpy as np  # noqa: E402

from slam2d.core.grid import FREE, OCCUPIED, UNKNOWN, OccGrid, dilate  # noqa: E402
from slam2d.core.types import Pose2D  # noqa: E402
from slam2d.tests.helpers import make_room_points  # noqa: E402


class TestOccGrid(unittest.TestCase):
    def setUp(self):
        self.g = OccGrid(resolution=0.05, size_m=12.0, origin=(-1.0, -1.0))

    def test_walls_need_repeated_hits(self):
        """1回しか当たらないセルは壁にならない（＝動く物を壁にしない）。"""
        pts = make_room_points(3.0, 2.0, 0.0)
        self.g.integrate(pts, Pose2D(3.0, 2.0, 0.0))
        self.assertEqual(int(self.g.wall_mask().sum()), 0)
        for _ in range(2):
            self.g.integrate(pts, Pose2D(3.0, 2.0, 0.0))
        self.assertGreater(int(self.g.wall_mask().sum()), 100)

    def test_trinary_values(self):
        pts = make_room_points(3.0, 2.0, 0.0)
        for _ in range(4):
            self.g.integrate(pts, Pose2D(3.0, 2.0, 0.0))
        t = self.g.trinary()
        self.assertEqual(set(np.unique(t)), {UNKNOWN, FREE, OCCUPIED})

    def test_freeze_stops_updates(self):
        pts = make_room_points(3.0, 2.0, 0.0)
        for _ in range(4):
            self.g.integrate(pts, Pose2D(3.0, 2.0, 0.0))
        self.g.freeze()
        before = self.g.hits.copy()
        self.g.integrate(pts, Pose2D(3.0, 2.0, 0.0))
        self.assertTrue(np.array_equal(before, self.g.hits))

    def test_raycast_measures_room_width(self):
        """壁までの距離が実際の部屋と合う。"""
        for i in range(6):
            x = 3.0 + 0.02 * i
            self.g.integrate(make_room_points(x, 3.0, 0.0), Pose2D(x, 3.0, 0.0))
        d = self.g.raycast(3.05, 3.0, np.array([math.pi / 2, -math.pi / 2]), 4.0)
        self.assertAlmostEqual(float(d[0]), 1.0, delta=0.08)   # 上の壁 y=4
        self.assertAlmostEqual(float(d[1]), 3.0, delta=0.08)   # 下の壁 y=0

    def test_dilate(self):
        m = np.zeros((7, 7), dtype=bool)
        m[3, 3] = True
        self.assertEqual(int(dilate(m, 1).sum()), 5)
        self.assertEqual(int(dilate(m, 0).sum()), 1)

    def test_known_free_distinct_from_unknown(self):
        """まだ見ていない場所は「空き」ではなく「未知」のまま。"""
        pts = make_room_points(3.0, 2.0, 0.0)
        for _ in range(4):
            self.g.integrate(pts, Pose2D(3.0, 2.0, 0.0))
        # 部屋の外（原点付近、レイが一度も通っていない）は未知のまま
        col, row = self.g.to_cell(-0.9, -0.9)
        self.assertFalse(bool(self.g.known_free_mask()[row, col]))
        self.assertFalse(bool(self.g.wall_mask()[row, col]))


if __name__ == "__main__":
    unittest.main()
