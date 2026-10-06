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
    MODELS_DIR,
    build_eval_cmd,
    build_export_cmd,
    build_extract_cmd,
    build_train_cmd,
    describe_model_status,
    extract_warning,
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

    def test_extract_cmd_passes_label_shift_only_when_set(self):
        cmd = build_extract_cmd("python3", ["a.mcap"], "out/frames", "front", 100)
        self.assertNotIn("--label-shift-ms", cmd)
        cmd = build_extract_cmd("python3", ["a.mcap"], "out/frames", "front", 100,
                                label_shift_ms=150)
        self.assertEqual(cmd[cmd.index("--label-shift-ms") + 1], "150")

    def test_train_cmd(self):
        cmd = build_train_cmd("python3", "out/frames", "out/v1", 30, 16, "224x128", False,
                              balance=0.7, speed_weight=0.25)
        self.assertIn("--frames", cmd)
        self.assertIn("out/frames", cmd)
        self.assertEqual(cmd[cmd.index("--epochs") + 1], "30")
        self.assertEqual(cmd[cmd.index("--size") + 1], "224x128")
        self.assertEqual(cmd[cmd.index("--balance") + 1], "0.70")
        self.assertEqual(cmd[cmd.index("--speed-weight") + 1], "0.25")
        self.assertNotIn("--no-pretrained", cmd)
        self.assertNotIn("--no-flip", cmd)

    def test_train_cmd_flags(self):
        cmd = build_train_cmd("python3", "out/frames", "out/v1", 30, 16, "224x128", True,
                              no_flip=True)
        self.assertIn("--no-pretrained", cmd)
        self.assertIn("--no-flip", cmd)

    def test_export_cmd_does_not_ask_for_a_resolution(self):
        """解像度は学習時の値（train_config.json）を使う。2箇所に入力させない。"""
        cmd = build_export_cmd("python3", "out/v1/best.pt", "models/cam_e2e/v1.onnx")
        self.assertIn("out/v1/best.pt", cmd)
        self.assertIn("models/cam_e2e/v1.onnx", cmd)
        self.assertNotIn("--size", cmd)

    def test_eval_cmd(self):
        cmd = build_eval_cmd("python3", "out/v1/frames", "out/v1", "models/cam_e2e/v1.onnx")
        self.assertIn("eval_model.py", cmd[1])
        self.assertEqual(cmd[cmd.index("--frames") + 1], "out/v1/frames")
        self.assertEqual(cmd[cmd.index("--run") + 1], "out/v1")
        self.assertEqual(cmd[cmd.index("--model") + 1], "models/cam_e2e/v1.onnx")


class TestExtractWarning(unittest.TestCase):
    """抽出が0枚で終わったとき、流れていくログではなくダイアログで理由を見せる。"""

    def test_nothing_to_say_when_pairs_were_written(self):
        self.assertIsNone(extract_warning(["# run.mcap: 12枚\n", "# 合計 12枚 → x に追記\n",
                                           "# 手本にしなかったフレーム: 自動運転中 3枚\n"]))

    def test_auto_only_recording_points_at_the_checkbox(self):
        msg = extract_warning(["# run.mcap: 0枚\n", "# 合計 0枚 → x に追記\n",
                               "# 手本にしなかったフレーム: 自動運転中 1303枚\n"])
        self.assertIn("自動運転中 1303枚", msg)
        self.assertIn("自動運転中の走行も手本にする", msg)

    def test_torque_mode_recording_says_to_rerecord(self):
        msg = extract_warning(["# 合計 0枚 → x に追記\n",
                               "# 手本にしなかったフレーム: トルクモード 40枚\n"])
        self.assertIn("速度モード", msg)
        self.assertNotIn("チェック", msg)

    def test_unreadable_file_is_named(self):
        msg = extract_warning(["# skip: a.mcap（読めない: ZstdError）。…\n",
                               "# 合計 0枚 → x に追記\n"])
        self.assertIn("a.mcap", msg)

    def test_no_images_at_all(self):
        self.assertIn("画像", extract_warning(["# 合計 0枚 → x に追記\n"]))

    def test_include_auto_flag_reaches_the_command(self):
        self.assertNotIn("--include-auto",
                         build_extract_cmd("python3", ["a.mcap"], "o", "front", 100))
        self.assertIn("--include-auto",
                      build_extract_cmd("python3", ["a.mcap"], "o", "front", 100,
                                        include_auto=True))


class TestCommandsMatchTheScripts(unittest.TestCase):
    """組み立てた引数を、実際のスクリプトの引数パーサが受け付けること。

    パネルとスクリプトの引数名がずれると「ボタンを押した瞬間に usage が出て
    終わる」ので、`--help` ではなく本物の引数を渡して確かめる（存在しない
    入力なので各スクリプトは引数を読んだ後の最初の確認で終わる。
    argparse の拒否は終了コード 2 ＋ "unrecognized arguments"/"usage:"）。
    """

    def _assert_args_accepted(self, cmd):
        import subprocess
        proc = subprocess.run(cmd, capture_output=True, text=True, timeout=120)
        self.assertNotIn("usage:", proc.stderr, proc.stderr)
        self.assertNotIn("unrecognized arguments", proc.stderr, proc.stderr)

    def test_all_four_commands(self):
        with tempfile.TemporaryDirectory() as d:
            frames = str(Path(d) / "frames")
            self._assert_args_accepted(build_extract_cmd(
                sys.executable, [str(Path(d) / "none.mcap")], frames, "front", 100,
                min_interval_ms=200, label_shift_ms=100))
            self._assert_args_accepted(build_extract_cmd(
                sys.executable, [str(Path(d) / "none.mcap")], frames, "front", 100,
                target_count=10, include_auto=True))
            self._assert_args_accepted(build_train_cmd(
                sys.executable, frames, d, 1, 4, "64x32", True, balance=0.3,
                speed_weight=0.5, no_flip=True))
            self._assert_args_accepted(build_export_cmd(
                sys.executable, str(Path(d) / "best.pt"), str(Path(d) / "m.onnx")))
            self._assert_args_accepted(build_eval_cmd(
                sys.executable, frames, d, str(Path(d) / "m.onnx")))


class TestModelsDir(unittest.TestCase):
    def test_exports_go_where_the_vehicle_node_looks(self):
        sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
        from raspi.nodes.cam_e2e_node import DEFAULT_MODELS_DIR
        self.assertEqual(MODELS_DIR, DEFAULT_MODELS_DIR)


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

    def test_keys_reach_the_visible_tab_but_not_while_typing(self):
        """メモ欄で "x" を打って除外が走る、ボタン上の space が2回効く、を防ぐ。"""
        class _Widget:
            def __init__(self, cls):
                self._cls = cls

            def winfo_class(self):
                return self._cls

        class _Event:
            def __init__(self, keysym, cls):
                self.keysym, self.state, self.widget = keysym, 0, _Widget(cls)

        root = tk.Tk()
        try:
            root.withdraw()
            app = App(root)
            got = []
            app.review_tab.handle_key = lambda e: got.append(e.keysym) or True

            app._on_key(_Event("x", "Frame"))
            self.assertEqual(got, [], "①タブが表のときは②へ渡さない")

            app.notebook.select(app.review_tab.frame)
            self.assertEqual(app._on_key(_Event("x", "Frame")), "break")
            self.assertIsNone(app._on_key(_Event("x", "TEntry")))
            self.assertIsNone(app._on_key(_Event("space", "TButton")))
            self.assertEqual(app._on_key(_Event("Right", "TButton")), "break")
            self.assertEqual(got, ["x", "Right"])
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
