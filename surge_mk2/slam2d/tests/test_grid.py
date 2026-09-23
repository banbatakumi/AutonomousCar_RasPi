"""`core/grid.py`（占有格子）のテスト。`raspi/tests/test_nav.py`の流儀を踏襲。"""

import math
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

import numpy as np  # noqa: E402

from slam2d.core.grid import FREE, OCCUPIED, UNKNOWN, OccGrid, dilate  # noqa: E402
from slam2d.core.types import Pose2D, ScanPoints  # noqa: E402
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


class TestThinWalls(unittest.TestCase):
    """車線を仕切る薄い壁が消えないこと（`hit_weight`）。"""

    def _counts(self, hit_weight):
        g = OccGrid(resolution=0.025, size_m=4.0, origin=(-2.0, -2.0), hit_weight=hit_weight)
        # 薄い壁のセルに「当たり」を10回、「素通り」を12回入れる
        # （すれすれの角度で通るレイと反対側の車線からのレイ、実測で当たり33/素通り38）
        col, row = g.to_cell(0.0, 0.0)
        g.hits[row, col] = 10
        g.misses[row, col] = 12
        return bool(g.wall_mask()[row, col])

    def test_equal_weights_lose_the_wall(self):
        self.assertFalse(self._counts(1.0))

    def test_default_weight_keeps_the_wall(self):
        self.assertTrue(self._counts(2.0))

    def test_moving_object_is_still_rejected(self):
        """当たりを重くしても、1〜2回しか見ていないものは壁にしない。"""
        g = OccGrid(resolution=0.025, size_m=4.0, origin=(-2.0, -2.0))
        col, row = g.to_cell(0.0, 0.0)
        g.hits[row, col] = 2                      # min_hits=3 に満たない
        g.misses[row, col] = 0
        self.assertFalse(bool(g.wall_mask()[row, col]))


class TestGrowth(unittest.TestCase):
    def test_grid_grows_toward_the_scan(self):
        g = OccGrid(resolution=0.05, size_m=4.0, origin=(-2.0, -2.0), grow=True,
                    grow_step_m=2.0)
        before = (g.width, g.height, g.origin)
        pts = make_room_points(3.0, 2.0, 0.0)     # 原点から最大8m先まで点がある
        g.integrate(pts, Pose2D(0.0, 0.0, 0.0))
        self.assertGreater(g.width, before[0])
        self.assertGreater(g.height, before[1])
        self.assertEqual(g.out_of_bounds, 0)
        # 世界座標は変わらない（原点セルの角が動いただけ）
        col, row = g.to_cell(0.0, 0.0)
        x, y = g.to_world(col, row)
        self.assertAlmostEqual(x, 0.025, places=6)
        self.assertAlmostEqual(y, 0.025, places=6)

    def test_fixed_grid_counts_out_of_bounds(self):
        g = OccGrid(resolution=0.05, size_m=2.0, origin=(-1.0, -1.0), grow=False)
        pts = make_room_points(3.0, 2.0, 0.0)
        g.integrate(pts, Pose2D(0.0, 0.0, 0.0))
        self.assertGreater(g.out_of_bounds, 0)

    def test_growth_stops_at_max_size(self):
        g = OccGrid(resolution=0.05, size_m=2.0, origin=(-1.0, -1.0), grow=True,
                    grow_step_m=2.0, max_size_m=4.0)
        for _ in range(5):
            g.integrate(make_room_points(3.0, 2.0, 0.0), Pose2D(0.0, 0.0, 0.0))
        self.assertLessEqual(g.width * g.resolution, 4.0 + 1e-9)


class TestHitMean(unittest.TestCase):
    def test_mean_is_subcell(self):
        g = OccGrid(resolution=0.10, size_m=4.0, origin=(-2.0, -2.0))
        x = 0.53                                   # セル中心(0.55)から2cmずれた壁
        pts = ScanPoints(np.array([x]), np.array([0.05]), np.array([True]), 1, True)
        for _ in range(3):
            g.integrate(pts, Pose2D(0.0, 0.0, 0.0))
        mean = g.hit_mean()
        self.assertIsNotNone(mean)
        col, row = g.to_cell(x, 0.05)
        self.assertAlmostEqual(float(mean[0][row, col]), x, places=6)

    def test_none_without_hits(self):
        g = OccGrid(resolution=0.05, size_m=2.0)
        self.assertIsNone(g.hit_mean())


class TestCachedMasksAreReadOnly(unittest.TestCase):
    def test_wall_mask_cannot_be_written(self):
        g = OccGrid(resolution=0.05, size_m=4.0, origin=(-2.0, -2.0))
        for _ in range(4):
            g.integrate(make_room_points(3.0, 2.0, 0.0), Pose2D(0.0, 0.0, 0.0))
        m = g.wall_mask()
        with self.assertRaises(ValueError):
            m[0, 0] = True

    def test_cache_is_invalidated_by_integrate(self):
        g = OccGrid(resolution=0.05, size_m=6.0, origin=(-3.0, -3.0))
        pts = make_room_points(3.0, 2.0, 0.0)
        for _ in range(3):
            g.integrate(pts, Pose2D(0.0, 0.0, 0.0))
        n1 = int(g.wall_mask().sum())
        for _ in range(3):
            g.integrate(make_room_points(3.2, 2.0, 0.0), Pose2D(0.2, 0.0, 0.0))
        self.assertNotEqual(n1, int(g.wall_mask().sum()))
