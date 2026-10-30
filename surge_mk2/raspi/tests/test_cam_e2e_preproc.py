"""`raspi/core/cam_e2e_preproc.py` のテスト。

学習（Mac）と推論（Pi）が同じ関数を通ることが前提なので、ここでは関数自体の
性質——色順の入れ替え・面積平均・出力の形——だけを確かめる。
"""

import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

import numpy as np  # noqa: E402

from raspi.core.cam_e2e_preproc import (  # noqa: E402
    crop_window,
    preprocess,
    resize_area,
    to_rgb,
)


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


def _resize_area_matmul(rgb: np.ndarray, width: int, height: int) -> np.ndarray:
    """2026-10-30 までの `resize_area`（重み行列との積を BLAS に任せる）。突き合わせ用。"""
    def weights(src, out):
        idx = (np.arange(out, dtype=np.int64) * src) // out
        counts = np.maximum(np.diff(np.append(idx, src)), 1)
        m = np.zeros((out, src), dtype=np.float32)
        for i, (start, n) in enumerate(zip(idx, counts)):
            m[i, start:start + n] = 1.0 / n
        return m
    src_h, src_w = rgb.shape[:2]
    x = np.tensordot(weights(src_h, height), rgb.astype(np.float32), axes=(1, 0))
    x = np.tensordot(x, weights(src_w, width), axes=(1, 1))
    return np.floor(x.transpose(0, 2, 1) + 0.5).astype(np.uint8)


class TestResizeAreaMatchesMatmul(unittest.TestCase):
    """BLAS を使わない実装に替えても、出力が旧実装と画素単位で同じであること。

    学習済みモデル（旧実装の絵で学習）をそのまま使うための条件。四捨五入がちょうど
    .5 になる画素（区画が 2x3 画素のとき起きる）は、足す順と丸め方まで同じでないと
    1ずれる。旧実装は Mac（Accelerate）と Pi（参照 BLAS）で同じ結果だった。
    """

    @staticmethod
    def _images(rng, n):
        for i in range(n):
            if i % 2:
                yield rng.integers(0, 256, (360, 640, 3), dtype=np.uint8)
            else:                                   # 隣どうしが近い絵（.5 の画素が出やすい）
                base = rng.integers(0, 256, (12, 20, 3)).astype(np.float32)
                img = np.kron(base, np.ones((30, 32, 1))) + rng.normal(0, 3, (360, 640, 3))
                yield np.clip(img, 0, 255).astype(np.uint8)

    def test_cam_e2e_input(self):
        """cam_e2e: BGR のリング → RGB → 左右切り落とし → 224x128（v7 の契約）。"""
        rng = np.random.default_rng(0)
        for src in self._images(rng, 12):
            rgb = crop_window(to_rgb(src, "BGR888"), 0.0515625)
            np.testing.assert_array_equal(resize_area(rgb, 224, 128),
                                          _resize_area_matmul(rgb, 224, 128))

    def test_cam_perception_input(self):
        """cam_perception・ml_cam: 全幅 → 224x224（縦は拡大を含まない 360→224）。"""
        rng = np.random.default_rng(1)
        for src in self._images(rng, 6):
            rgb = to_rgb(src, "BGR888")
            np.testing.assert_array_equal(resize_area(rgb, 224, 224),
                                          _resize_area_matmul(rgb, 224, 224))

    def test_other_sizes(self):
        rng = np.random.default_rng(2)
        for (h, w), (oh, ow) in [((480, 640), (128, 224)), ((37, 53), (10, 7)),
                                 ((20, 30), (25, 40)), ((360, 640), (36, 64))]:
            src = rng.integers(0, 256, (h, w, 3), dtype=np.uint8)
            np.testing.assert_array_equal(resize_area(src, ow, oh),
                                          _resize_area_matmul(src, ow, oh))


class TestCropWindow(unittest.TestCase):
    def setUp(self):
        # 列番号をそのまま値にした絵（どの列が残ったか読める）
        self.img = np.tile(np.arange(100, dtype=np.uint8)[None, :, None], (4, 1, 3))

    def test_center_window_drops_both_sides(self):
        out = crop_window(self.img, 0.1)
        self.assertEqual(out.shape[1], 80)
        self.assertEqual(int(out[0, 0, 0]), 10)
        self.assertEqual(int(out[0, -1, 0]), 89)

    def test_shift_moves_the_window_but_keeps_its_width(self):
        out = crop_window(self.img, 0.1, shift_px=-10)
        self.assertEqual(out.shape[1], 80)
        self.assertEqual(int(out[0, 0, 0]), 0)

    def test_shift_beyond_the_margin_is_rejected(self):
        with self.assertRaises(ValueError):
            crop_window(self.img, 0.1, shift_px=11)
        with self.assertRaises(ValueError):
            crop_window(self.img, 0.0, shift_px=1)

    def test_no_crop_is_identity(self):
        self.assertIs(crop_window(self.img, 0.0), self.img)

    def test_preprocess_applies_the_crop_before_resizing(self):
        rgb = np.zeros((8, 40, 3), dtype=np.uint8)
        rgb[:, :10, 0] = 255                   # 左の 1/4 だけ赤
        x = preprocess(rgb, (4, 2), 0.0, 255.0, crop_x=0.25)
        self.assertAlmostEqual(float(x[0, 0].max()), 0.0)


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
