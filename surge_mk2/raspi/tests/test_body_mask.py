"""車体の内側の LiDAR の点を捨てる（`raspi/nav/body_mask.py`）。

会場の試走（2026-10-31）で、車体の内側（LiDAR から 3〜15cm）の点を障害物と取り、毎走行 2〜9 回止まっていた。
"""

from __future__ import annotations

import math
import sys
import unittest
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from raspi.core.vehicle import Vehicle  # noqa: E402
from raspi.msgs.types import Scan  # noqa: E402
from raspi.nav.body_mask import BODY_MARGIN_M, body_ranges, mask_body  # noqa: E402

FP = ((0.30, 0.09), (0.30, -0.09), (-0.07, -0.09), (-0.07, 0.09))
LX = 0.07


def _ray(x: float, y: float) -> tuple[int, float]:
    """base_link の点 → (車両角 [deg], LiDAR からの距離)。"""
    dx, dy = x - LX, y
    return int(round(math.degrees(math.atan2(dy, dx)))) % 360, math.hypot(dx, dy)


class BodyRangesTest(unittest.TestCase):
    def test_exit_distance_of_the_outline(self) -> None:
        r = body_ranges(FP, LX, 0.0)
        self.assertAlmostEqual(r[0], 0.23 + BODY_MARGIN_M, places=6)     # 前: 0.30 − 0.07
        self.assertAlmostEqual(r[180], 0.14 + BODY_MARGIN_M, places=6)   # 後: 0.07 + 0.07
        self.assertAlmostEqual(r[90], 0.09 + BODY_MARGIN_M, places=6)    # 左
        self.assertAlmostEqual(r[270], 0.09 + BODY_MARGIN_M, places=6)   # 右

    def test_mount_outside_or_no_outline_keeps_everything(self) -> None:
        self.assertFalse(body_ranges(FP, 0.5, 0.0).any())
        self.assertFalse(body_ranges((), LX, 0.0).any())

    def test_matches_the_vehicle_config(self) -> None:
        v = Vehicle.load()
        r = body_ranges(v.footprint, v.lidar_x, v.lidar_y)
        self.assertTrue((r > 0.05).all() and (r < 0.4).all())


class MaskBodyTest(unittest.TestCase):
    def setUp(self) -> None:
        self.r = body_ranges(FP, LX, 0.0)

    def _scan(self, pts: list[tuple[float, float]]) -> Scan:
        dist = [3.0] * 360
        for x, y in pts:
            deg, d = _ray(x, y)
            dist[deg] = d
        return Scan(dist=dist, sector_seen=[True] * 12, saturated=[False] * 360)

    def test_points_seen_in_the_trial_are_dropped(self) -> None:
        # 実機の偽の障害物の位置（車体座標）
        inside = [(0.15, 0.01), (0.13, 0.01), (0.17, 0.01), (-0.04, 0.05)]
        out = mask_body(self._scan(inside), self.r)
        for x, y in inside:
            self.assertEqual(out.dist[_ray(x, y)[0]], 0.0)

    def test_points_outside_the_body_are_kept(self) -> None:
        outside = [(0.36, 0.0), (0.10, 0.15), (-0.13, 0.0), (0.0, -0.14)]
        sc = self._scan(outside)
        out = mask_body(sc, self.r)
        self.assertIs(out, sc)                  # 捨てる点が無ければ写しを作らない
        np.testing.assert_array_equal(out.dist, sc.dist)

    def test_only_the_masked_rays_change(self) -> None:
        sc = self._scan([(0.15, 0.01)])
        sc.saturated[5] = True
        out = mask_body(sc, self.r)
        changed = [i for i in range(360) if out.dist[i] != sc.dist[i]]
        self.assertEqual(changed, [_ray(0.15, 0.01)[0]])
        self.assertTrue(out.saturated[5])
        self.assertEqual(out.sector_seen, sc.sector_seen)


if __name__ == "__main__":
    unittest.main()
