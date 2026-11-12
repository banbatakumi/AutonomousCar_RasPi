"""`ml_cam_e2e/yaw_aug.py`（向きをずらす拡張）と、それを使う `DriveDataset` のテスト。合成画像だけで検証する。"""

import math
import sys
import tempfile
import unittest
from pathlib import Path
from unittest import mock

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))  # repo root
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))  # ml_cam_e2e/

import dataset as D
import numpy as np
from PIL import Image
from samples import Sample
from yaw_aug import (
    estimate_body_mask,
    front_fx_px,
    margin_for,
    yawed_window,
)


def _columns(w=100, h=6):
    """列番号をそのまま値にした絵（どの列が来たか読める）。"""
    return np.tile(np.arange(w, dtype=np.uint8)[None, :, None], (h, 1, 3))


class TestYawedWindow(unittest.TestCase):
    def test_turning_left_moves_the_scene_right(self):
        """車が左（ψ>0）を向くと、正面にあった物は右へ写る＝窓は左へずれる。"""
        img = _columns()
        fx = 100.0
        psi = 5 / fx                       # 5px
        out = yawed_window(img, 10, psi, fx, None)
        self.assertEqual(out.shape[1], 80)
        self.assertEqual(int(out[0, 0, 0]), 5)          # 中央の窓なら 10
        out = yawed_window(img, 10, -psi, fx, None)
        self.assertEqual(int(out[0, 0, 0]), 15)

    def test_shift_is_clamped_to_the_margin(self):
        out = yawed_window(_columns(), 10, 1.0, 100.0, None)
        self.assertEqual(int(out[0, 0, 0]), 0)

    def test_body_pixels_stay_where_they_were(self):
        img = _columns()
        body = np.zeros((6, 100), dtype=bool)
        body[4:, :] = True                 # 下2行が車体
        out = yawed_window(img, 10, 5 / 100.0, 100.0, body)
        np.testing.assert_array_equal(out[4:], img[4:, 10:90])     # 車体は中央の窓のまま
        self.assertEqual(int(out[0, 0, 0]), 5)                      # 景色はずれる


class TestBodyMask(unittest.TestCase):
    def test_still_pixels_touching_the_bottom_are_the_body(self):
        rng = np.random.default_rng(0)
        with tempfile.TemporaryDirectory() as d:
            paths = []
            for i in range(20):
                arr = rng.integers(0, 256, (40, 60, 3), dtype=np.uint8)   # 動く背景
                arr[30:, 10:50] = 90                                      # 動かない車体
                arr[2:6, 20:30] = 200                                     # 動かない天井の照明（上の方）
                p = Path(d) / f"{i}.png"
                Image.fromarray(arr).save(p)
                paths.append(p)
            mask = estimate_body_mask(paths)
        self.assertTrue(mask[35, 30])
        self.assertTrue(mask[30, 25])
        self.assertFalse(mask[35, 2])          # 動く背景
        self.assertFalse(mask[3, 25])          # 上の方の動かない物は車体にしない


class TestFx(unittest.TestCase):
    def test_fisheye_fx_is_scaled_to_the_image_width(self):
        fe = mock.Mock(fx=300.0, width=640)
        v = mock.Mock(cam_front_fisheye=fe)
        self.assertAlmostEqual(front_fx_px(v, 320), 150.0)

    def test_pinhole_fallback(self):
        v = mock.Mock(cam_front_fisheye=None, cam_front_hfov=math.pi / 2)
        self.assertAlmostEqual(front_fx_px(v, 200), 100.0)

    def test_margin_covers_the_largest_shift(self):
        self.assertEqual(margin_for(6.0, 308.0), math.ceil(308.0 * math.radians(6.0)))


class TestDatasetYawAugment(unittest.TestCase):
    def setUp(self):
        self._tmp = tempfile.TemporaryDirectory()
        self.path = Path(self._tmp.name) / "a.png"
        Image.fromarray(_columns(100, 8)).save(self.path)
        from samples import left_path
        Image.fromarray(_columns(100, 8)).save(left_path(self.path))      # 左も同じ絵
        self.sample = Sample(path=self.path, source_mcap="r.mcap", cam="front", t_ns=0,
                             target_steer=0.1, target_speed=1.0, speed_actual=1.0, brake=False)

    def tearDown(self):
        self._tmp.cleanup()

    def ds(self, **kw):
        return D.DriveDataset([self.sample], size=(80, 8), max_steer=0.5, speed_ref=2.0,
                              crop_x=0.1, flip=False, **kw)

    def test_label_is_corrected_back_toward_the_line(self):
        """左を向いた絵（ψ>0）には右へ戻す（舵を減らす）ラベルが付く。"""
        ds = self.ds(augment=True, yaw_max_deg=6.0, yaw_k=0.8, fx_px=100.0, fx_px_left=100.0)
        psi = 0.05
        with mock.patch.object(D, "sample_psi", return_value=psi), \
                mock.patch.object(D.np.random, "uniform", return_value=0.0):
            img, target = ds[0]
        self.assertAlmostEqual(float(target[0]), (0.1 - 0.8 * psi) / 0.5, places=5)
        # 窓は左へ5pxずれている（明るさの揺らぎは 0 に固定）。左カメラ（ch3〜）も同じ ψ でずれる
        self.assertAlmostEqual(float(img[0, 0, 0]) * 255, 5.0, places=3)
        self.assertAlmostEqual(float(img[3, 0, 0]) * 255, 5.0, places=3)

    def test_without_augmentation_the_center_window_is_used(self):
        img, target = self.ds(augment=False)[0]
        self.assertAlmostEqual(float(img[0, 0, 0]) * 255, 10.0, places=3)
        self.assertAlmostEqual(float(target[0]), 0.2, places=5)

    def test_yaw_augment_requires_a_crop(self):
        with self.assertRaises(ValueError):
            D.DriveDataset([self.sample], size=(80, 8), max_steer=0.5, speed_ref=2.0,
                           yaw_max_deg=6.0, fx_px=100.0)


if __name__ == "__main__":
    unittest.main()
