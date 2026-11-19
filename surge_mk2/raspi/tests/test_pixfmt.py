"""YUV420（I420・sYCC）の経路 — 変換・共有メモリのリング・JPEG・ステレオの灰色化。

カメラは YUV420 で撮り、ステレオは Y 面をそのまま、色が要る側だけ RGB/BGR に直す（`core/pixfmt.py`）。
"""

from __future__ import annotations

import io
import unittest

import numpy as np


import cv2

from raspi.bus.shm_ring import FrameRing
from raspi.core import pixfmt
from raspi.core.cam_e2e_preproc import to_rgb as preproc_to_rgb
from raspi.core.jpeg import RingJpeg
from raspi.core.stereo_match import to_gray as stereo_to_gray

H, W = 48, 64


def _scene_bgr() -> np.ndarray:
    yy, xx = np.mgrid[0:H, 0:W]
    img = np.dstack([xx * 3, yy * 4, (xx + yy) * 2]).astype(np.float32)
    return cv2.GaussianBlur(np.clip(img, 0, 255).astype(np.uint8), (0, 0), 1.0)


def _i420_from_bgr(bgr: np.ndarray) -> np.ndarray:
    """BGR → 全範囲 BT.601 の I420（カメラの sYCC と同じ並び）。"""
    ycc = cv2.cvtColor(bgr, cv2.COLOR_BGR2YCrCb)
    y = ycc[..., 0]
    cr = cv2.resize(ycc[..., 1], (W // 2, H // 2), interpolation=cv2.INTER_AREA)
    cb = cv2.resize(ycc[..., 2], (W // 2, H // 2), interpolation=cv2.INTER_AREA)
    return np.concatenate([y.ravel(), cb.ravel(), cr.ravel()]).reshape(H * 3 // 2, W)


class PixfmtTest(unittest.TestCase):
    def setUp(self) -> None:
        self.bgr = _scene_bgr()
        self.i420 = _i420_from_bgr(self.bgr)

    def test_planes(self) -> None:
        y, u, v = pixfmt.yuv420_planes(self.i420)
        self.assertEqual(y.shape, (H, W))
        self.assertEqual(u.shape, (H // 2, W // 2))
        self.assertEqual(v.shape, (H // 2, W // 2))
        self.assertEqual(pixfmt.image_hw(self.i420, "YUV420"), (H, W))

    def test_gray_is_y_plane_view(self) -> None:
        g = pixfmt.to_gray(self.i420, "YUV420")
        self.assertEqual(g.shape, (H, W))
        self.assertTrue(np.shares_memory(g, self.i420))
        # 全範囲の Y は RGB から作った灰色とほぼ同じ
        ref = cv2.cvtColor(self.bgr, cv2.COLOR_BGR2GRAY).astype(int)
        self.assertLessEqual(np.abs(g.astype(int) - ref).max(), 1)
        self.assertEqual(stereo_to_gray(self.i420, "YUV420").shape, (H, W))

    def test_colour_roundtrip_full_range(self) -> None:
        # 制限範囲の変換（OpenCV の I420）を使うと平均で数段ずれる。全範囲で戻せば差は 4:2:0 の分だけ
        bgr = pixfmt.to_bgr(self.i420, "YUV420")
        self.assertEqual(bgr.shape, (H, W, 3))
        self.assertLess(np.abs(bgr.astype(int) - self.bgr).mean(), 2.0)
        rgb = pixfmt.to_rgb(self.i420, "YUV420")
        np.testing.assert_array_equal(rgb, bgr[..., ::-1])
        np.testing.assert_array_equal(preproc_to_rgb(self.i420, "YUV420"), rgb)

    def test_bgr_formats_unchanged(self) -> None:
        np.testing.assert_array_equal(pixfmt.to_rgb(self.bgr, "BGR888"), self.bgr[..., ::-1])
        np.testing.assert_array_equal(pixfmt.to_bgr(self.bgr, "BGR888"), self.bgr)
        np.testing.assert_array_equal(pixfmt.to_bgr(self.bgr[..., ::-1], "RGB888"), self.bgr)


class Yuv420RingTest(unittest.TestCase):
    def setUp(self) -> None:
        self.name = f"surge_test_yuv_{id(self):x}"
        self.ring = FrameRing.create(self.name, W, H, "YUV420", n_slots=3)
        self.i420 = _i420_from_bgr(_scene_bgr())

    def tearDown(self) -> None:
        self.ring.close()
        self.ring.unlink()

    def test_write_and_read(self) -> None:
        self.ring.write(self.i420, t_capture_ns=123)
        ref = self.ring.latest()
        arr = ref.as_array()
        self.assertEqual(arr.shape, (H * 3 // 2, W))
        np.testing.assert_array_equal(arr, self.i420)
        self.assertTrue(ref.still_valid())

    def test_padded_stride_rejected(self) -> None:
        with self.assertRaises(ValueError):
            FrameRing.create(self.name + "p", W, H, "YUV420", stride=W + 64)

    def test_jpeg_from_planes_keeps_colour(self) -> None:
        self.ring.write(self.i420, t_capture_ns=1)
        jp = RingJpeg(quality=95)
        if jp.encode is None:
            self.skipTest("JPEG エンコーダが無い")
        got = jp.encode_latest(self.name)
        self.assertIsNotNone(got)
        from PIL import Image

        dec = np.asarray(Image.open(io.BytesIO(got[0])).convert("RGB"))[..., ::-1].astype(int)
        self.assertLess(np.abs(dec - _scene_bgr()).mean(), 4.0)
        # 撮影ボタン（PNG）用は BGR で返す
        arr, colorspace, _ = jp.read_latest(self.name)
        self.assertEqual((colorspace, arr.shape), ("BGR", (H, W, 3)))
        jp.close()


if __name__ == "__main__":
    unittest.main()
