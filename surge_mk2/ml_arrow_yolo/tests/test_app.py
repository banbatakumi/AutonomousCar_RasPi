"""`ml_arrow_yolo/app.py` のコマンド組み立て・データの写し、`label_tab.box_at` のテスト（Tk の窓は作らない）。"""

import sys
import tempfile
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

try:
    import tkinter  # noqa: F401
except ModuleNotFoundError:
    raise unittest.SkipTest("tkinter が無い")

import app  # noqa: E402
from dataset import Box, RunPaths, write_classes, write_labels  # noqa: E402
from label_tab import box_at  # noqa: E402


class TestCommands(unittest.TestCase):
    def test_extract(self):
        cmd = app.build_extract_cmd("py", ["a.mcap", "b.mcap"], "out", min_interval_ms=300)
        self.assertEqual(cmd[2:4], ["a.mcap", "b.mcap"])
        self.assertIn("300", cmd)

    def test_train_and_export(self):
        cmd = app.build_train_cmd("py", "runs/v1", epochs=5, imgsz=256, device="mps")
        self.assertEqual(cmd[cmd.index("--imgsz") + 1], "256")
        self.assertEqual(cmd[-2:], ["--device", "mps"])
        self.assertNotIn("--device", app.build_train_cmd("py", "runs/v1"))
        self.assertIn("--imgsz", app.build_export_cmd("py", "runs/v1", "m.onnx", imgsz=256))

    def test_prelabel(self):
        self.assertIn("--overwrite", app.build_prelabel_cmd("py", "r", "m.onnx", overwrite=True))

    def test_models_dir_is_where_the_car_looks(self):
        from raspi.nodes.arrow_signal_node import DEFAULT_MODELS_DIR
        self.assertEqual(app.MODELS_DIR.resolve(), DEFAULT_MODELS_DIR.resolve())


class TestCopyDataset(unittest.TestCase):
    def test_copies_images_labels_classes_without_overwriting(self):
        with tempfile.TemporaryDirectory() as d:
            src, dst = RunPaths(Path(d) / "v1"), RunPaths(Path(d) / "v2")
            src.images.mkdir(parents=True)
            (src.images / "a__1.jpg").write_bytes(b"x")
            write_labels(src.label_path(src.images / "a__1.jpg"), [Box(0, 0, 0, 0.5, 0.5)])
            write_classes(src, ["left", "right"])
            self.assertEqual(app.copy_dataset(src, dst), 1)
            self.assertTrue(dst.label_path(dst.images / "a__1.jpg").is_file())
            self.assertTrue(dst.classes.is_file())
            self.assertEqual(app.copy_dataset(src, dst), 0)
            self.assertIn("確定 1枚", app.describe_status(dst.run_dir))


class TestBoxAt(unittest.TestCase):
    def test_picks_the_smallest_containing_box(self):
        boxes = [Box(0, 0, 0, 1, 1), Box(1, 0.4, 0.4, 0.6, 0.6)]
        self.assertEqual(box_at(boxes, 0.5, 0.5), 1)
        self.assertEqual(box_at(boxes, 0.1, 0.1), 0)
        self.assertIsNone(box_at([], 0.5, 0.5))


if __name__ == "__main__":
    unittest.main()
