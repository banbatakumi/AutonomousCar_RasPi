"""`ml_cam_e2e/dataset.py` のテスト。実データは要らない——合成した画像で検証する。"""

import csv
import sys
import tempfile
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))  # ml_cam_e2e/

import numpy as np  # noqa: E402
from PIL import Image  # noqa: E402

from dataset import DriveDataset, list_pairs  # noqa: E402


def _write_frame(path: Path, *, size=(40, 30), left_bright=True) -> None:
    w, h = size
    arr = np.zeros((h, w, 3), dtype=np.uint8)
    if left_bright:
        arr[:, : w // 2] = 200
    else:
        arr[:, w // 2:] = 200
    Image.fromarray(arr, mode="RGB").save(path)


class TestListPairs(unittest.TestCase):
    def test_reads_manifest_and_skips_missing_files(self):
        with tempfile.TemporaryDirectory() as d:
            frames_dir = Path(d)
            _write_frame(frames_dir / "a.jpg")
            with open(frames_dir / "manifest.csv", "w", newline="") as f:
                w = csv.writer(f)
                w.writerow(["file", "source_mcap", "cam", "t_capture_ns",
                           "target_steer", "target_speed"])
                w.writerow(["a.jpg", "run.mcap", "front", "0", "0.12", "0.3"])
                w.writerow(["missing.jpg", "run.mcap", "front", "1", "0.0", "0.0"])

            pairs = list_pairs(frames_dir)
            self.assertEqual(len(pairs), 1)
            self.assertEqual(pairs[0][0].name, "a.jpg")
            self.assertAlmostEqual(pairs[0][1], 0.12)

    def test_missing_manifest_returns_empty(self):
        with tempfile.TemporaryDirectory() as d:
            self.assertEqual(list_pairs(Path(d)), [])


class TestDriveDataset(unittest.TestCase):
    def test_item_shapes_and_normalization(self):
        with tempfile.TemporaryDirectory() as d:
            frames_dir = Path(d)
            _write_frame(frames_dir / "a.jpg")
            ds = DriveDataset([(frames_dir / "a.jpg", 0.262)], size=(32, 24),
                              augment=False, max_steer=0.524)
            img, steer = ds[0]
            self.assertEqual(img.shape, (3, 24, 32))
            self.assertEqual(steer.shape, (1,))
            self.assertAlmostEqual(float(steer[0]), 0.5, places=3)

    def test_clamps_out_of_range_steer(self):
        with tempfile.TemporaryDirectory() as d:
            frames_dir = Path(d)
            _write_frame(frames_dir / "a.jpg")
            ds = DriveDataset([(frames_dir / "a.jpg", 10.0)], size=(32, 24),
                              augment=False, max_steer=0.524)
            _, steer = ds[0]
            self.assertAlmostEqual(float(steer[0]), 1.0)

    def test_augment_flips_image_and_negates_steer_label(self):
        """左右反転するなら、操舵ラベルの符号も一緒に反転しないと教師信号が矛盾する。"""
        with tempfile.TemporaryDirectory() as d:
            frames_dir = Path(d)
            _write_frame(frames_dir / "a.jpg", size=(40, 30), left_bright=True)
            ds = DriveDataset([(frames_dir / "a.jpg", 0.2)], size=(40, 30),
                              augment=True, max_steer=0.524)

            saw_flip = False
            for _ in range(40):
                img, steer = ds[0]
                img_left_bright = float(img[:, :, :20].mean()) > float(img[:, :, 20:].mean())
                if img_left_bright:
                    self.assertGreater(float(steer[0]), 0.0)
                else:
                    self.assertLess(float(steer[0]), 0.0)
                    saw_flip = True
            self.assertTrue(saw_flip, "40回試して一度も反転が起きなかった（乱数か実装を疑う）")


if __name__ == "__main__":
    unittest.main()
