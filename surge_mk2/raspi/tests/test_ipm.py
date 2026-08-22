"""カメラ IPM（`raspi/nav/ipm.py`）のテスト。

**実機もモデルも要らない。** 合成した地面の点をピンホールモデルで画素へ順投影し
（`ground_to_pixel`）、それを画像とみなして `project_mask_to_grid` で逆投影し、
元の地面座標に戻ることを確認する（`test_nav.py` の「合成部屋から答え合わせする」
流儀と同じ）。
"""

import math
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

import numpy as np  # noqa: E402

from raspi.nav.grid import OccGrid  # noqa: E402
from raspi.nav.ipm import (CameraExtrinsics, camera_intrinsics,  # noqa: E402
                          ground_to_pixel, pixel_to_ground,
                          project_mask_to_grid)


class TestRoundTrip(unittest.TestCase):
    """`ground_to_pixel` → `pixel_to_ground` が元の座標に戻ること。"""

    def test_level_camera_front(self):
        intr = camera_intrinsics(1.152, 640, 480, bottom_crop=0.25)
        ext = CameraExtrinsics(x=0.097, y=0.0, height=0.09, pitch=0.0, yaw=0.0)
        for x in (0.3, 0.8, 1.5, 3.0):
            for y in (-0.4, 0.0, 0.4):
                px = ground_to_pixel(x, y, intr, ext)
                self.assertIsNotNone(px, f"({x},{y}) が画角外扱いになった")
                back = pixel_to_ground(px[0], px[1], intr, ext)
                self.assertIsNotNone(back)
                self.assertAlmostEqual(back[0], x, delta=1e-6)
                self.assertAlmostEqual(back[1], y, delta=1e-6)

    def test_pitched_camera(self):
        """取付ピッチが付いていても往復が一致する（俯角・仰角の両方）。"""
        intr = camera_intrinsics(1.152, 640, 480, bottom_crop=0.25)
        for pitch in (0.15, -0.05):
            ext = CameraExtrinsics(x=0.097, y=0.0, height=0.09, pitch=pitch, yaw=0.0)
            for x, y in ((0.5, 0.2), (1.2, -0.3), (2.0, 0.0)):
                px = ground_to_pixel(x, y, intr, ext)
                self.assertIsNotNone(px)
                back = pixel_to_ground(px[0], px[1], intr, ext)
                self.assertIsNotNone(back)
                self.assertAlmostEqual(back[0], x, delta=1e-6)
                self.assertAlmostEqual(back[1], y, delta=1e-6)

    def test_rear_camera_yaw(self):
        """後ろ向き（yaw=π）でも往復が一致する。"""
        intr = camera_intrinsics(1.152, 640, 480, bottom_crop=0.0625)
        ext = CameraExtrinsics(x=0.044, y=0.0, height=0.09, pitch=0.0, yaw=math.pi)
        for x, y in ((-0.5, 0.1), (-1.5, -0.2)):
            px = ground_to_pixel(x, y, intr, ext)
            self.assertIsNotNone(px)
            back = pixel_to_ground(px[0], px[1], intr, ext)
            self.assertIsNotNone(back)
            self.assertAlmostEqual(back[0], x, delta=1e-6)
            self.assertAlmostEqual(back[1], y, delta=1e-6)

    def test_above_horizon_has_no_ground_intersection(self):
        """光軸より上の画素（空）は地面との交点を持たない。"""
        intr = camera_intrinsics(1.152, 640, 480, bottom_crop=0.25)
        ext = CameraExtrinsics(x=0.0, y=0.0, height=0.09, pitch=0.0, yaw=0.0)
        # principal_y より上（v が小さい）＝ 光軸より上方向
        self.assertIsNone(pixel_to_ground(intr.cx, intr.principal_y - 5, intr, ext))


class TestProjectMaskToGrid(unittest.TestCase):
    """合成した「壁」領域が画素→占有格子の投影で復元されること。"""

    def test_wall_rectangle_recovered(self):
        width, height = 480, 360
        intr = camera_intrinsics(1.152, width, height, bottom_crop=0.25)
        ext = CameraExtrinsics(x=0.0, y=0.0, height=0.09, pitch=0.0, yaw=0.0)

        # 地面上の「壁」矩形（車両前方 1.0〜1.1m、左右 ±0.3m）
        x0, x1 = 1.0, 1.1
        y0, y1 = -0.3, 0.3

        drivable = np.ones((height, width), dtype=bool)
        n = 60
        for xi in np.linspace(x0, x1, n):
            for yi in np.linspace(y0, y1, n):
                px = ground_to_pixel(float(xi), float(yi), intr, ext)
                if px is None:
                    continue
                u, v = int(round(px[0])), int(round(px[1]))
                if 0 <= u < width and 0 <= v < height:
                    drivable[v, u] = False          # 走行不可（壁）として焼き込む

        grid = OccGrid(resolution=0.05, size_m=6.0, origin=(-1.0, -3.0))
        occ = project_mask_to_grid(drivable, intr, ext, grid, stride=1)

        # 期待される占有セル（矩形の中心と四隅が属するセル）
        expect_cells = set()
        for xi in np.linspace(x0, x1, 10):
            for yi in np.linspace(y0, y1, 10):
                col, row = grid.to_cell(float(xi), float(yi))
                expect_cells.add((int(row), int(col)))

        hit = sum(1 for r, c in expect_cells if occ[r, c])
        self.assertGreater(hit / len(expect_cells), 0.5,
                           "壁矩形の大半が占有セルとして復元されていない")

        # 矩形から離れた場所（例えば x=3m 付近）は誤って占有にならない
        far_col, far_row = grid.to_cell(3.0, 0.0)
        self.assertFalse(bool(occ[far_row, far_col]),
                         "壁が無い遠方セルが誤って占有になった")

    def test_no_solid_mask_yields_empty_grid(self):
        """全画素が走行可能なら、投影結果は空（占有セル無し）。"""
        width, height = 64, 48
        intr = camera_intrinsics(1.152, width, height, bottom_crop=0.25)
        ext = CameraExtrinsics(x=0.0, y=0.0, height=0.09, pitch=0.0, yaw=0.0)
        grid = OccGrid(resolution=0.05, size_m=4.0, origin=(-1.0, -2.0))
        occ = project_mask_to_grid(np.ones((height, width), dtype=bool), intr, ext, grid)
        self.assertEqual(int(occ.sum()), 0)


if __name__ == "__main__":
    unittest.main()
