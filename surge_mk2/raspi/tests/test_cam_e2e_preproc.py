"""`raspi/core/cam_e2e_preproc.py` のテスト。

学習（Mac）と推論（Pi）が同じ関数を通ることが前提なので、ここでは関数自体の
性質——色順の入れ替え・面積平均・出力の形——だけを確かめる。
"""

import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

import numpy as np  # noqa: E402

from raspi.core.cam_e2e_preproc import preprocess, resize_area, to_rgb  # noqa: E402


class TestToRgb(unittest.TestCase):
    def test_bgr_ring_is_swapped_so_red_lands_in_channel_0(self):
        """リングの既定は BGR888（`camera_node.py` の `_MEMORY_ORDER`）。
        赤い物は ch2 に入っているので、RGB に直すと ch0 が最大になる。"""
        frame = np.zeros((4, 6, 3), dtype=np.uint8)
        frame[..., 2] = 250                      # BGR の R
        rgb = to_rgb(frame, "BGR888")
        self.assertEqual(rgb[0, 0].tolist(), [250, 0, 0])

    def test_rgb_ring_is_left_as_is(self):
        frame = np.zeros((4, 6, 3), dtype=np.uint8)
        frame[..., 0] = 250
        self.assertEqual(to_rgb(frame, "RGB888")[0, 0].tolist(), [250, 0, 0])

    def test_extra_channel_is_dropped(self):
        frame = np.zeros((4, 6, 4), dtype=np.uint8)
        self.assertEqual(to_rgb(frame, "RGB888").shape, (4, 6, 3))
        self.assertEqual(to_rgb(frame, "BGR888").shape, (4, 6, 3))


class TestResizeArea(unittest.TestCase):
    def test_output_shape_and_dtype(self):
        src = np.zeros((360, 640, 3), dtype=np.uint8)
        out = resize_area(src, 224, 128)
        self.assertEqual(out.shape, (128, 224, 3))
        self.assertEqual(out.dtype, np.uint8)

    def test_uniform_image_stays_uniform(self):
        for value in (0, 37, 255):
            src = np.full((360, 640, 3), value, dtype=np.uint8)
            out = resize_area(src, 224, 128)
            self.assertTrue((out == value).all(), f"{value} が保たれていない")

    def test_averages_the_block_instead_of_picking_one_pixel(self):
        """市松模様を半分に縮めると灰色になる（最近傍なら白か黒のどちらか）。"""
        src = np.zeros((4, 4, 1), dtype=np.uint8)
        src[::2, ::2] = 200
        src[1::2, 1::2] = 200
        out = resize_area(src, 2, 2)
        self.assertTrue((out == 100).all())

    def test_integer_ratio_matches_block_mean(self):
        rng = np.random.default_rng(0)
        src = rng.integers(0, 256, size=(6, 9, 3), dtype=np.uint8)
        out = resize_area(src, 3, 2)
        want = src.reshape(2, 3, 3, 3, 3).mean(axis=(1, 3))
        self.assertLessEqual(np.abs(out.astype(float) - want).max(), 0.5 + 1e-9)

    def test_same_size_returns_the_same_pixels(self):
        src = np.arange(24, dtype=np.uint8).reshape(2, 4, 3)
        self.assertTrue((resize_area(src, 4, 2) == src).all())

    def test_upscale_does_not_divide_by_zero(self):
        src = np.full((2, 2, 3), 9, dtype=np.uint8)
        out = resize_area(src, 5, 5)
        self.assertEqual(out.shape, (5, 5, 3))
        self.assertTrue((out == 9).all())

    def test_accepts_a_non_contiguous_view(self):
        """`to_rgb()` の戻り値は逆順のビュー。そのまま渡せること。"""
        frame = np.zeros((8, 8, 3), dtype=np.uint8)
        frame[..., 2] = 80
        out = resize_area(to_rgb(frame, "BGR888"), 4, 4)
        self.assertEqual(out[0, 0].tolist(), [80, 0, 0])


class TestPreprocess(unittest.TestCase):
    def test_layout_is_nchw_float32_scaled(self):
        rgb = np.zeros((36, 64, 3), dtype=np.uint8)
        rgb[..., 0] = 255
        x = preprocess(rgb, (16, 8), 0.0, 255.0)
        self.assertEqual(x.shape, (1, 3, 8, 16))
        self.assertEqual(x.dtype, np.float32)
        self.assertAlmostEqual(float(x[0, 0].mean()), 1.0)
        self.assertAlmostEqual(float(x[0, 1].mean()), 0.0)


if __name__ == "__main__":
    unittest.main()
