"""`extract_frames.py`（本物の記録の書き手で往復）・`prelabel.py`（偽の検出器）・`train.prepare` のテスト。"""

import sys
import tempfile
import unittest
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import cv2  # noqa: E402

from dataset import Box, RunPaths, read_labels, write_classes, write_labels  # noqa: E402
from extract_frames import extract_one  # noqa: E402
from prelabel import prelabel  # noqa: E402
from raspi.rec.mcap_log import McapLog  # noqa: E402
from train import TRAIN_AUG, prepare  # noqa: E402


def _jpeg() -> bytes:
    ok, buf = cv2.imencode(".jpg", np.zeros((48, 64, 3), np.uint8))
    assert ok
    return buf.tobytes()


class TestExtract(unittest.TestCase):
    def test_writes_front_frames_thinned_and_skips_existing(self):
        with tempfile.TemporaryDirectory() as d:
            mcap = Path(d) / "run1.mcap"
            with McapLog(mcap, t0_mono_ns=0, t0_unix_ns=0) as log:
                for i in range(10):                         # 100ms 間隔で 10 枚
                    log.write_viz_image(_jpeg(), "front", t_mono_ns=i * 100_000_000)
                    log.write_viz_image(_jpeg(), "left", t_mono_ns=i * 100_000_000)
            out = Path(d) / "images"
            n, existed = extract_one(mcap, out, min_interval_ms=200)
            self.assertEqual((n, existed), (5, 0))
            files = sorted(out.glob("*.jpg"))
            self.assertTrue(all(p.name.startswith("run1__") for p in files))
            self.assertIsNotNone(cv2.imread(str(files[0])))
            self.assertEqual(extract_one(mcap, out, min_interval_ms=200), (0, 5))


class _FakeDetector:
    names = ["right", "left", "cat"]

    def detect(self, frame):
        return [([0.1, 0.2, 0.3, 0.4], 0.9, 1), ([0.5, 0.5, 0.6, 0.6], 0.8, 2)]


class TestPrelabel(unittest.TestCase):
    def test_drafts_only_unconfirmed_and_remaps_class_names(self):
        with tempfile.TemporaryDirectory() as d:
            paths = RunPaths(Path(d))
            paths.images.mkdir(parents=True)
            for n in ("a__1", "a__2"):
                cv2.imwrite(str(paths.images / f"{n}.jpg"), np.zeros((48, 64, 3), np.uint8))
            write_classes(paths, ["left", "right", "straight"])
            write_labels(paths.label_path(paths.images / "a__1.jpg"), [])     # 確定済み（背景）
            n, empty = prelabel(paths.run_dir, Path("x.onnx"), detector=_FakeDetector())
            self.assertEqual((n, empty), (1, 0))
            self.assertFalse(paths.draft_path(paths.images / "a__1.jpg").exists())
            draft = read_labels(paths.draft_path(paths.images / "a__2.jpg"))
            self.assertEqual([b.cls for b in draft], [0])   # モデルの "left"(1) → classes の 0。"cat" は捨てる


class TestTrainPrepare(unittest.TestCase):
    def test_no_flip(self):
        self.assertEqual(TRAIN_AUG["fliplr"], 0.0)

    def test_prepare_uses_only_confirmed(self):
        with tempfile.TemporaryDirectory() as d:
            paths = RunPaths(Path(d))
            paths.images.mkdir(parents=True)
            for n in ("a__1", "a__2", "b__1"):
                (paths.images / f"{n}.jpg").write_bytes(b"x")
            write_labels(paths.label_path(paths.images / "a__1.jpg"), [Box(0, 0, 0, 0.5, 0.5)])
            write_labels(paths.label_path(paths.images / "b__1.jpg"), [])
            _yaml, n_train, n_val = prepare(paths.run_dir, 0.5)
            self.assertEqual(n_train + n_val, 2)

    def test_prepare_without_labels_stops(self):
        with tempfile.TemporaryDirectory() as d:
            with self.assertRaises(SystemExit):
                prepare(Path(d), 0.2)


if __name__ == "__main__":
    unittest.main()
