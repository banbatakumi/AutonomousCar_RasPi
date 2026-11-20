"""カメラ IPM（`raspi/nav/ipm.py`）のテスト。

**実機もモデルも要らない。** 合成した地面の点をピンホールモデルで画素へ順投影し
（`ground_to_pixel`）、`pixel_to_ground` で逆投影し、
元の地面座標に戻ることを確認する（`test_nav.py` の「合成部屋から答え合わせする」
流儀と同じ）。
"""

import math
import unittest


from raspi.nav.ipm import (CameraExtrinsics, camera_intrinsics,
                          ground_to_pixel, pixel_to_ground)


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
