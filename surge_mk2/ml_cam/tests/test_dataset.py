"""`ml_cam/dataset.py` のテスト。実データは要らない——合成した画像/マスクで検証する。"""

import csv
import sys
import tempfile
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))  # ml_cam/

import numpy as np  # noqa: E402
from PIL import Image  # noqa: E402

from dataset import DrivableDataset, list_labeled_pairs  # noqa: E402


def _write_frame(path: Path, *, size=(40, 30), left_bright=True) -> None:
    """左半分と右半分で明るさが違う画像。左右反転の検証に使う。"""
    w, h = size
    arr = np.zeros((h, w, 3), dtype=np.uint8)
    if left_bright:
        arr[:, : w // 2] = 200
    else:
        arr[:, w // 2:] = 200
    Image.fromarray(arr, mode="RGB").save(path)


def _write_mask(path: Path, *, size=(40, 30), left_drivable=True) -> None:
    w, h = size
    arr = np.zeros((h, w), dtype=np.uint8)
    if left_drivable:
        arr[:, : w // 2] = 255
    else:
        arr[:, w // 2:] = 255
    Image.fromarray(arr, mode="L").save(path)


class TestListLabeledPairs(unittest.TestCase):
    def test_only_frames_with_a_mask_are_returned(self):
        with tempfile.TemporaryDirectory() as d:
            frames_dir = Path(d)
            _write_frame(frames_dir / "a.jpg")
            _write_frame(frames_dir / "b.jpg")
            _write_mask(frames_dir / "a_mask.png")     # b にはマスクを作らない

            with open(frames_dir / "manifest.csv", "w", newline="") as f:
                w = csv.writer(f)
                w.writerow(["file", "source_mcap", "cam", "t_capture_ns"])
                w.writerow(["a.jpg", "run.mcap", "front", "0"])
                w.writerow(["b.jpg", "run.mcap", "front", "1"])

            pairs = list_labeled_pairs(frames_dir)
            self.assertEqual(len(pairs), 1)
            self.assertEqual(pairs[0][0].name, "a.jpg")
            self.assertEqual(pairs[0][1].name, "a_mask.png")

    def test_missing_manifest_returns_empty_when_no_frames_either(self):
        with tempfile.TemporaryDirectory() as d:
            self.assertEqual(list_labeled_pairs(Path(d)), [])

    def test_missing_manifest_falls_back_to_scanning_the_directory(self):
        """GUIの「ログ」タブからZIPでフレームを抽出した場合は `manifest.csv`
        が付いてこない（ブラウザは由来の記録を残さない）。それでも、対応する
        `_mask.png` があるフレームはちゃんと拾えること。"""
        with tempfile.TemporaryDirectory() as d:
            frames_dir = Path(d)
            _write_frame(frames_dir / "a.jpg")
            _write_frame(frames_dir / "b.jpg")
            _write_mask(frames_dir / "a_mask.png")     # b にはマスクを作らない
            # manifest.csv を意図的に作らない

            pairs = list_labeled_pairs(frames_dir)
            self.assertEqual(len(pairs), 1)
            self.assertEqual(pairs[0][0].name, "a.jpg")
            self.assertEqual(pairs[0][1].name, "a_mask.png")


class TestDrivableDataset(unittest.TestCase):
    def test_item_shapes_and_dtypes(self):
        with tempfile.TemporaryDirectory() as d:
            frames_dir = Path(d)
            _write_frame(frames_dir / "a.jpg")
            _write_mask(frames_dir / "a_mask.png")
            ds = DrivableDataset([(frames_dir / "a.jpg", frames_dir / "a_mask.png")],
                                size=(32, 24), augment=False)
            img, msk = ds[0]
            self.assertEqual(img.shape, (3, 24, 32))
            self.assertEqual(msk.shape, (1, 24, 32))
            self.assertTrue(0.0 <= float(img.min()) and float(img.max()) <= 1.0)
            self.assertEqual(set(msk.unique().tolist()) - {0.0, 1.0}, set())

    def test_mask_binarisation_matches_the_drivable_region(self):
        with tempfile.TemporaryDirectory() as d:
            frames_dir = Path(d)
            _write_frame(frames_dir / "a.jpg", size=(40, 30))
            _write_mask(frames_dir / "a_mask.png", size=(40, 30), left_drivable=True)
            ds = DrivableDataset([(frames_dir / "a.jpg", frames_dir / "a_mask.png")],
                                size=(40, 30), augment=False)
            _, msk = ds[0]
            left = msk[0, :, :20]
            right = msk[0, :, 20:]
            self.assertTrue(bool((left == 1.0).all()))
            self.assertTrue(bool((right == 0.0).all()))

    def test_augment_flips_image_and_mask_together(self):
        """左右反転するなら、画像とマスクは**同じ側**が反転しないと対応がズレる。"""
        with tempfile.TemporaryDirectory() as d:
            frames_dir = Path(d)
            _write_frame(frames_dir / "a.jpg", size=(40, 30), left_bright=True)
            _write_mask(frames_dir / "a_mask.png", size=(40, 30), left_drivable=True)
            ds = DrivableDataset([(frames_dir / "a.jpg", frames_dir / "a_mask.png")],
                                size=(40, 30), augment=True)

            saw_flip = False
            for _ in range(40):                       # 確率反転なので複数回試す
                img, msk = ds[0]
                img_left_bright = float(img[:, :, :20].mean()) > float(img[:, :, 20:].mean())
                msk_left_drivable = bool((msk[0, :, :20] == 1.0).all())
                # 反転していてもいなくても、「明るい側」と「走行可能側」は必ず一致する
                self.assertEqual(img_left_bright, msk_left_drivable)
                if not img_left_bright:
                    saw_flip = True
            self.assertTrue(saw_flip, "40回試して一度も反転が起きなかった（乱数か実装を疑う）")


if __name__ == "__main__":
    unittest.main()
