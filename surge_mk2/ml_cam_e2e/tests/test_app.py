"""`ml_cam_e2e/app.py` の純粋関数（Tkinterを起動しない部分）のテスト。"""

import sys
import tempfile
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))  # ml_cam_e2e/

from app import (  # noqa: E402
    build_export_cmd,
    build_extract_cmd,
    build_preview_cmd,
    build_train_cmd,
    describe_model_status,
    list_model_names,
    next_model_name,
    parse_epoch_line,
    read_note,
    write_note,
)


class TestModelNames(unittest.TestCase):
    def test_next_model_name_continues_from_existing(self):
        self.assertEqual(next_model_name([]), "v1")
        self.assertEqual(next_model_name(["v1", "v2"]), "v3")
        self.assertEqual(next_model_name(["v1", "custom", "v5"]), "v6")

    def test_list_model_names_returns_sorted_directories(self):
        with tempfile.TemporaryDirectory() as d:
            root = Path(d)
            (root / "v2").mkdir()
            (root / "v1").mkdir()
            (root / "not_a_dir.txt").write_text("")
            self.assertEqual(list_model_names(root), ["v1", "v2"])

    def test_list_model_names_empty_when_missing(self):
        self.assertEqual(list_model_names(Path("/nonexistent")), [])


class TestModelStatus(unittest.TestCase):
    def test_describe_counts_pairs_and_training_state(self):
        with tempfile.TemporaryDirectory() as d:
            model_dir = Path(d)
            frames = model_dir / "frames"
            frames.mkdir()
            (frames / "a.jpg").write_bytes(b"x")
            (frames / "b.jpg").write_bytes(b"x")
            status = describe_model_status(model_dir)
            self.assertIn("2件", status)
            self.assertIn("未学習", status)

            (model_dir / "best.pt").write_bytes(b"x")
            status = describe_model_status(model_dir)
            self.assertIn("✓学習済み", status)


class TestNote(unittest.TestCase):
    def test_round_trips(self):
        with tempfile.TemporaryDirectory() as d:
            model_dir = Path(d) / "v1"
            write_note(model_dir, "テストの備考")
            self.assertEqual(read_note(model_dir), "テストの備考")

    def test_missing_note_returns_empty(self):
        self.assertEqual(read_note(Path("/nonexistent")), "")


class TestCommandBuilders(unittest.TestCase):
    def test_extract_cmd_with_min_interval(self):
        cmd = build_extract_cmd("python3", ["a.mcap", "b.mcap"], "out/frames", "front",
                                100, min_interval_ms=500)
        self.assertIn("extract_pairs.py", cmd[1])
        self.assertIn("a.mcap", cmd)
        self.assertIn("b.mcap", cmd)
        self.assertIn("--max-gap-ms", cmd)
        self.assertIn("100", cmd)
        self.assertIn("--min-interval-ms", cmd)
        self.assertNotIn("--target-count", cmd)

    def test_extract_cmd_prefers_target_count(self):
        cmd = build_extract_cmd("python3", ["a.mcap"], "out/frames", "front", 100,
                                min_interval_ms=500, target_count=1000)
        self.assertIn("--target-count", cmd)
        self.assertNotIn("--min-interval-ms", cmd)

    def test_train_cmd(self):
        cmd = build_train_cmd("python3", "out/frames", "out/v1", 30, 16, "224x224", False)
        self.assertIn("--frames", cmd)
        self.assertIn("out/frames", cmd)
        self.assertIn("--epochs", cmd)
        self.assertIn("30", cmd)
        self.assertNotIn("--no-pretrained", cmd)

    def test_train_cmd_no_pretrained_flag(self):
        cmd = build_train_cmd("python3", "out/frames", "out/v1", 30, 16, "224x224", True)
        self.assertIn("--no-pretrained", cmd)

    def test_export_cmd(self):
        cmd = build_export_cmd("python3", "out/v1/best.pt", "models/v1.onnx", "224x224")
        self.assertIn("out/v1/best.pt", cmd)
        self.assertIn("models/v1.onnx", cmd)

    def test_preview_cmd(self):
        cmd = build_preview_cmd("python3", "out/frames", "models/v1.onnx")
        self.assertIn("out/frames", cmd)
        self.assertIn("models/v1.onnx", cmd)


class TestParseEpochLine(unittest.TestCase):
    def test_parses_a_normal_line(self):
        parsed = parse_epoch_line("epoch   3/30  loss=0.1234  val_mae=0.045  (2.7deg, 12s)")
        self.assertEqual(parsed, (3, 0.1234, 0.045))

    def test_parses_nan_val_mae(self):
        parsed = parse_epoch_line("epoch  30/30  loss=0.0100  val_mae=nan  (nandeg, 300s)")
        self.assertEqual(parsed, (30, 0.0100, None))

    def test_returns_none_for_unrelated_lines(self):
        self.assertIsNone(parse_epoch_line("# device: cpu  学習 100件 / 検証 15件"))


if __name__ == "__main__":
    unittest.main()
