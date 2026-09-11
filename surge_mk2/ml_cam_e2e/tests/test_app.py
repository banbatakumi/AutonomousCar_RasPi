"""`ml_cam_e2e/app.py` のテスト。

純粋関数（コマンド組み立て・モデル名・備考）に加え、ウィンドウクローズ時の
確認ダイアログ（`_CONFIRM_STOP_JOB_KEYS`・`confirm_and_close`との配線）も
検証する。GUI自体の見た目・クリック操作はテストしない（`ml_cam/tests/test_app.py`
と同じ方針）。
"""

import sys
import tempfile
import tkinter as tk
import unittest
from pathlib import Path
from unittest.mock import MagicMock, patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))  # ml_cam_e2e/

from app import (  # noqa: E402
    App,
    _CONFIRM_STOP_JOB_KEYS,
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

from ml_common.close_handler import confirm_and_close  # noqa: E402


def _has_display() -> bool:
    try:
        root = tk.Tk()
        root.withdraw()
        root.destroy()
        return True
    except tk.TclError:
        return False


_HAS_DISPLAY = _has_display()


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


class TestCloseConfirmation(unittest.TestCase):
    """ウィンドウを閉じる際、学習中だけ確認ダイアログを出す
    （ペア抽出・エクスポート・プレビューは既存動作どおり無確認で終了）。
    `ml_cam/tests/test_app.py`のテストと同じ考え方（`JobRunner`のダックタイピング
    しか要求しないため、Tkinterを起動せず軽量なフェイクで検証できる）。
    """

    class _FakeJobs:
        def __init__(self, running_keys, labels=None):
            self._running = list(running_keys)
            self._labels = labels or {}

        def running_job_keys(self):
            return list(self._running)

        def label_of(self, key):
            return self._labels.get(key, key)

        def terminate_all(self):
            stopped, self._running = self._running, []
            return stopped

    def test_confirm_stop_job_keys_is_train_only(self):
        self.assertEqual(_CONFIRM_STOP_JOB_KEYS, frozenset({"train"}))

    def test_train_running_asks_before_closing(self):
        jobs = self._FakeJobs(["train"], {"train": "学習"})
        root = MagicMock()
        with patch("ml_common.close_handler.messagebox.askyesno", return_value=True) as mock_ask:
            confirm_and_close(root, jobs, confirm_job_keys=_CONFIRM_STOP_JOB_KEYS)
        mock_ask.assert_called_once()
        root.destroy.assert_called_once()

    def test_declining_confirmation_keeps_window_open(self):
        jobs = self._FakeJobs(["train"], {"train": "学習"})
        root = MagicMock()
        with patch("ml_common.close_handler.messagebox.askyesno", return_value=False):
            confirm_and_close(root, jobs, confirm_job_keys=_CONFIRM_STOP_JOB_KEYS)
        root.destroy.assert_not_called()

    def test_extract_running_closes_without_asking(self):
        jobs = self._FakeJobs(["extract"], {"extract": "ペア抽出"})
        root = MagicMock()
        with patch("ml_common.close_handler.messagebox.askyesno") as mock_ask:
            confirm_and_close(root, jobs, confirm_job_keys=_CONFIRM_STOP_JOB_KEYS)
        mock_ask.assert_not_called()
        root.destroy.assert_called_once()

    def test_no_job_running_closes_without_asking(self):
        jobs = self._FakeJobs([])
        root = MagicMock()
        with patch("ml_common.close_handler.messagebox.askyesno") as mock_ask:
            confirm_and_close(root, jobs, confirm_job_keys=_CONFIRM_STOP_JOB_KEYS)
        mock_ask.assert_not_called()
        root.destroy.assert_called_once()


@unittest.skipUnless(_HAS_DISPLAY, "ディスプレイが無い環境（ヘッドレスCI等）ではスキップ")
class TestRunJobKeys(unittest.TestCase):
    """各タブの`_run()`呼び出しが、ジョブ種別ごとに異なる`job_key`を
    `JobRunner.start()`へ渡していること。"""

    def test_run_passes_job_key_through_to_job_runner(self):
        root = tk.Tk()
        try:
            root.withdraw()
            app = App(root)
            captured = {}

            def fake_start(job_key, cmd, label, *, on_line=None, on_done=None):
                captured["job_key"] = job_key
                return True

            app.jobs.start = fake_start
            app._run("train", ["true"], "学習")
            self.assertEqual(captured["job_key"], "train")

            app._on_job_done("学習", 0)
            app._run("extract", ["true"], "ペア抽出")
            self.assertEqual(captured["job_key"], "extract")
        finally:
            root.destroy()

    def test_stop_stops_whichever_job_is_currently_active(self):
        root = tk.Tk()
        try:
            root.withdraw()
            app = App(root)
            stopped = {}

            def fake_stop(key):
                stopped["key"] = key
                return True

            app.jobs.active_jobs = lambda: [("train", "学習")]
            app.jobs.stop = fake_stop
            app._stop()
            self.assertEqual(stopped["key"], "train")
        finally:
            root.destroy()


if __name__ == "__main__":
    unittest.main()
