"""`ml_cam_e2e/photo_aug.py` のテスト。合成した絵で、各外乱が意図どおりに効くことを見る。"""

import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))  # ml_cam_e2e/

import numpy as np  # noqa: E402
import photo_aug as pa  # noqa: E402


def _stripes(h=20, w=64):
    img = np.zeros((h, w, 3), np.uint8)
    img[:, ::8] = 255
    return img


class TestHorizontalBlur(unittest.TestCase):
    def test_keeps_shape_and_mean_and_smears_only_horizontally(self):
        img = _stripes()
        out = pa.horizontal_blur(img, 8)
        self.assertEqual(out.shape, img.shape)
        self.assertEqual(out.dtype, np.uint8)
        # 縦縞を幅8（縞の周期）で平均すると一様になる
        self.assertLess(out[:, 8:-8].std(), 1.0)
        # 中ほどは縞の平均（255/8）。端は端の画素の延長が入るので除く
        self.assertAlmostEqual(float(out[:, 8:-8].mean()), 255 / 8, delta=1.0)
        # 横縞は横ブレで変わらない
        rows = np.zeros((20, 64, 3), np.uint8)
        rows[::4] = 255
        self.assertTrue(np.array_equal(pa.horizontal_blur(rows, 8), rows))

    def test_length_one_or_less_is_identity(self):
        img = _stripes()
        self.assertIs(pa.horizontal_blur(img, 1), img)
        self.assertIs(pa.horizontal_blur(img, 0), img)

    def test_box_average_value(self):
        img = np.zeros((1, 9, 3), np.uint8)
        img[0, 4] = 90
        out = pa.horizontal_blur(img, 3)
        self.assertEqual(out[0, 3:6, 0].tolist(), [30, 30, 30])
        self.assertEqual(out[0, 2, 0], 0)
        self.assertEqual(out[0, 6, 0], 0)


class TestOthers(unittest.TestCase):
    def test_noise_has_requested_sigma(self):
        img = np.full((100, 100, 3), 128, np.uint8)
        out = pa.sensor_noise(img, 6.0, np.random.default_rng(0))
        self.assertAlmostEqual(float(out.astype(float).std()), 6.0, delta=0.3)
        self.assertIs(pa.sensor_noise(img, 0.0, np.random.default_rng(0)), img)

    def test_color_shift_gains_and_saturation(self):
        img = np.full((4, 4, 3), 100, np.uint8)
        out = pa.color_shift(img, (1.1, 1.0, 0.9), 1.0)
        self.assertEqual(out[0, 0].tolist(), [110, 100, 90])
        gray = pa.color_shift(np.array([[[200, 100, 0]]], np.uint8), (1, 1, 1), 0.0)
        self.assertEqual(gray[0, 0].tolist(), [100, 100, 100])

    def test_jpeg_roundtrip_keeps_shape(self):
        img = _stripes(32, 32)
        out = pa.jpeg_roundtrip(img, 40)
        self.assertEqual(out.shape, img.shape)
        self.assertLess(np.abs(out.astype(int) - img).mean(), 40)


class TestApply(unittest.TestCase):
    def test_strength_zero_is_identity(self):
        img = _stripes()
        self.assertIs(pa.apply(img, np.random.default_rng(0), 0.0), img)

    def test_changes_some_images_but_keeps_shape_and_dtype(self):
        img = _stripes()
        rng = np.random.default_rng(1)
        changed = 0
        for _ in range(20):
            out = pa.apply(img, rng)
            self.assertEqual(out.shape, img.shape)
            self.assertEqual(out.dtype, np.uint8)
            changed += not np.array_equal(out, img)
        # 4つの外乱がそれぞれ50%なので、何も掛からないのは 1/16
        self.assertGreater(changed, 12)


if __name__ == "__main__":
    unittest.main()
