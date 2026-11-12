"""`ml_cam_e2e/dataset.py` のテスト。実データは要らない——合成した画像で検証する。"""

import sys
import tempfile
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))  # repo root
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))  # ml_cam_e2e/

import numpy as np  # noqa: E402
from PIL import Image  # noqa: E402

from dataset import DriveDataset, load_rgb  # noqa: E402
from raspi.core.cam_e2e_preproc import preprocess  # noqa: E402
from samples import Sample, left_path  # noqa: E402


def _write_frame(path: Path, *, size=(40, 30), left_bright=True) -> None:
    w, h = size
    arr = np.zeros((h, w, 3), dtype=np.uint8)
    if left_bright:
        arr[:, : w // 2] = 200
    else:
        arr[:, w // 2:] = 200
    Image.fromarray(arr, mode="RGB").save(path, quality=95)


def _sample(path: Path, *, steer=0.0, speed=0.5, brake=False) -> Sample:
    return Sample(path=path, source_mcap="run.mcap", cam="front", t_ns=0,
                  target_steer=steer, target_speed=speed, speed_actual=speed, brake=brake)


class TestDriveDataset(unittest.TestCase):
    def setUp(self):
        self._tmp = tempfile.TemporaryDirectory()
        self.frame = Path(self._tmp.name) / "a.jpg"
        _write_frame(self.frame)
        _write_frame(left_path(self.frame))       # 左も同じ絵（左右の並びは TestStereoDataset で見る）

    def tearDown(self):
        self._tmp.cleanup()

    def ds(self, sample, **kw):
        kw.setdefault("size", (32, 24))
        kw.setdefault("max_steer", 0.524)
        kw.setdefault("speed_ref", 2.0)
        return DriveDataset([sample], **kw)

    def test_item_shapes_and_normalization(self):
        img, target = self.ds(_sample(self.frame, steer=0.262, speed=0.5))[0]
        self.assertEqual(img.shape, (6, 24, 32))
        self.assertEqual(target.shape, (2,))
        self.assertAlmostEqual(float(target[0]), 0.5, places=3)
        self.assertAlmostEqual(float(target[1]), 0.25, places=3)

    def test_labels_are_clamped_to_the_model_output_range(self):
        _, target = self.ds(_sample(self.frame, steer=10.0, speed=9.0))[0]
        self.assertAlmostEqual(float(target[0]), 1.0)
        self.assertAlmostEqual(float(target[1]), 1.0)

    def test_brake_means_zero_speed(self):
        _, target = self.ds(_sample(self.frame, speed=1.0, brake=True))[0]
        self.assertEqual(float(target[1]), 0.0)

    def test_image_is_exactly_what_the_vehicle_side_preprocess_produces(self):
        """学習で見る絵＝実車でモデルに入る絵。同じ関数を通っていること"""
        img, _ = self.ds(_sample(self.frame), augment=False)[0]
        one = preprocess(load_rgb(self.frame), (32, 24), 0.0, 255.0)[0]
        self.assertTrue(np.array_equal(img.numpy(), np.concatenate([one, one])))

    def test_augment_flips_image_and_negates_steer_but_not_speed(self):
        """左右反転するなら、操舵ラベルの符号も一緒に反転しないと教師信号が矛盾する。"""
        ds = self.ds(_sample(self.frame, steer=0.2, speed=0.5), size=(40, 30), augment=True)
        saw_flip = False
        for _ in range(40):
            img, target = ds[0]
            left_bright = float(img[:, :, :20].mean()) > float(img[:, :, 20:].mean())
            if left_bright:
                self.assertGreater(float(target[0]), 0.0)
            else:
                self.assertLess(float(target[0]), 0.0)
                saw_flip = True
            self.assertAlmostEqual(float(target[1]), 0.25, places=5)
            self.assertGreaterEqual(float(img.min()), 0.0)
            self.assertLessEqual(float(img.max()), 1.0)
        self.assertTrue(saw_flip, "40回試して一度も反転が起きなかった（乱数か実装を疑う）")

    def test_flip_can_be_turned_off(self):
        ds = self.ds(_sample(self.frame, steer=0.2), size=(40, 30), augment=True, flip=False)
        for _ in range(40):
            img, target = ds[0]
            self.assertGreater(float(target[0]), 0.0)
            self.assertGreater(float(img[:, :, :20].mean()), float(img[:, :, 20:].mean()))

    def test_photo_aug_only_when_augmenting_and_keeps_labels(self):
        """外乱（photo_aug）は舵・速度のラベルを変えない。検証の絵（augment=False）には掛からない"""
        plain, _ = self.ds(_sample(self.frame, steer=0.2), augment=False, photo_strength=1.0)[0]
        one = preprocess(load_rgb(self.frame), (32, 24), 0.0, 255.0)[0]
        self.assertTrue(np.array_equal(plain.numpy(), np.concatenate([one, one])))
        ds = self.ds(_sample(self.frame, steer=0.2, speed=0.5), augment=True, flip=False,
                     photo_strength=1.0)
        for _ in range(10):
            img, target = ds[0]
            self.assertAlmostEqual(float(target[0]), 0.2 / 0.524, places=5)
            self.assertAlmostEqual(float(target[1]), 0.25, places=5)
            self.assertGreaterEqual(float(img.min()), 0.0)
            self.assertLessEqual(float(img.max()), 1.0)


class TestStereoDataset(unittest.TestCase):
    """左右2台: `[右, 左]` の 6ch。反転は左右を入れ替えてから鏡像にする。"""

    def setUp(self):
        self._tmp = tempfile.TemporaryDirectory()
        self.frame = Path(self._tmp.name) / "a.jpg"
        _write_frame(self.frame, left_bright=True)                 # 右カメラ: 左半分が明るい
        _write_frame(left_path(self.frame), left_bright=False)     # 左カメラ: 右半分が明るい

    def tearDown(self):
        self._tmp.cleanup()

    def test_channels_are_right_then_left(self):
        ds = DriveDataset([_sample(self.frame)], size=(32, 24), max_steer=0.524, speed_ref=2.0)
        img, _ = ds[0]
        self.assertEqual(img.shape, (6, 24, 32))
        np.testing.assert_allclose(img[:3].numpy(), preprocess(load_rgb(self.frame), (32, 24))[0], atol=1e-6)
        np.testing.assert_allclose(img[3:].numpy(), preprocess(load_rgb(left_path(self.frame)), (32, 24))[0],
                                   atol=1e-6)

    def test_flip_swaps_the_cameras_and_mirrors_each(self):
        from unittest.mock import patch
        ds = DriveDataset([_sample(self.frame, steer=0.262)], size=(32, 24), max_steer=0.524,
                          speed_ref=2.0, augment=True, flip=True)
        plain = DriveDataset([_sample(self.frame, steer=0.262)], size=(32, 24), max_steer=0.524,
                             speed_ref=2.0)[0][0].numpy()
        # 反転を必ず選び、明るさの揺らぎは無しにする
        with patch("numpy.random.rand", return_value=0.0), patch("numpy.random.uniform", return_value=0.0):
            img, target = ds[0]
        img = img.numpy()
        self.assertAlmostEqual(float(target[0]), -0.5, places=3)
        np.testing.assert_allclose(img[:3], plain[3:, :, ::-1], atol=1e-6)   # 新しい右 = 鏡像の左
        np.testing.assert_allclose(img[3:], plain[:3, :, ::-1], atol=1e-6)   # 新しい左 = 鏡像の右


if __name__ == "__main__":
    unittest.main()
