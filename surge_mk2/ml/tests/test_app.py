"""`ml/app.py`（Tkinter操作パネル）のテスト。

**GUIの見た目・クリック操作はテストしない**（`ml/annotate.py` の cv2 ループを
テストしないのと同じ理由）。コマンド組み立て・ダウンロード処理という
Tkinterを知らない純粋関数だけを厳密にテストし、ウィジェット構築だけ
「例外を出さずに組み立てられるか」というスモークテストに留める
（ディスプレイが無い環境ではスキップする）。
"""

import sys
import tempfile
import tkinter as tk
import unittest
from pathlib import Path
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))  # ml/

from app import (  # noqa: E402
    App,
    ML_DIR,
    REPO_ROOT,
    build_annotate_cmd,
    build_export_cmd,
    build_extract_cmd,
    build_preview_cmd,
    build_train_cmd,
    default_sam_checkpoint_path,
    download_file,
    new_run_dir_str,
    parse_epoch_line,
    rel,
)


def _has_display() -> bool:
    try:
        root = tk.Tk()
        root.withdraw()
        root.destroy()
        return True
    except tk.TclError:
        return False


_HAS_DISPLAY = _has_display()


class TestCommandBuilders(unittest.TestCase):
    def test_extract_cmd(self):
        cmd = build_extract_cmd("python3", ["a.mcap", "b.mcap"], "out_dir", "front")
        self.assertEqual(cmd[0], "python3")
        self.assertEqual(cmd[1], str(ML_DIR / "extract_frames.py"))
        self.assertIn("a.mcap", cmd)
        self.assertIn("b.mcap", cmd)
        self.assertEqual(cmd[-2:], ["--cam", "front"])
        self.assertIn("--out", cmd)
        self.assertEqual(cmd[cmd.index("--out") + 1], "out_dir")
        self.assertNotIn("--min-interval-ms", cmd, "既定は間引きなし")

    def test_extract_cmd_with_min_interval(self):
        cmd = build_extract_cmd("python3", ["a.mcap"], "out_dir", "front", 500)
        self.assertEqual(cmd[cmd.index("--min-interval-ms") + 1], "500")

    def test_extract_cmd_zero_interval_omits_flag(self):
        cmd = build_extract_cmd("python3", ["a.mcap"], "out_dir", "front", 0)
        self.assertNotIn("--min-interval-ms", cmd)

    def test_annotate_cmd_without_skip_labeled(self):
        cmd = build_annotate_cmd("python3", "frames", "ckpt.pth", "vit_b", "cpu", False)
        self.assertNotIn("--skip-labeled", cmd)
        self.assertIn("frames", cmd)
        self.assertIn("--checkpoint", cmd)
        self.assertEqual(cmd[cmd.index("--checkpoint") + 1], "ckpt.pth")
        self.assertEqual(cmd[cmd.index("--model-type") + 1], "vit_b")
        self.assertEqual(cmd[cmd.index("--device") + 1], "cpu")

    def test_annotate_cmd_with_skip_labeled(self):
        cmd = build_annotate_cmd("python3", "frames", "ckpt.pth", "vit_b", "cpu", True)
        self.assertIn("--skip-labeled", cmd)

    def test_annotate_cmd_carries_points_by_default(self):
        cmd = build_annotate_cmd("python3", "frames", "ckpt.pth", "vit_b", "cpu", False)
        self.assertNotIn("--no-carry-points", cmd)

    def test_annotate_cmd_can_disable_carry_points(self):
        cmd = build_annotate_cmd("python3", "frames", "ckpt.pth", "vit_b", "cpu", False,
                                 carry_points=False)
        self.assertIn("--no-carry-points", cmd)

    def test_train_cmd_converts_numbers_to_strings(self):
        cmd = build_train_cmd("python3", "frames", "out", 30, 8, "224x224", False)
        self.assertEqual(cmd[cmd.index("--epochs") + 1], "30")
        self.assertEqual(cmd[cmd.index("--batch-size") + 1], "8")
        self.assertEqual(cmd[cmd.index("--size") + 1], "224x224")
        self.assertNotIn("--no-pretrained", cmd)

    def test_train_cmd_with_no_pretrained(self):
        cmd = build_train_cmd("python3", "frames", "out", 1, 1, "64x64", True)
        self.assertIn("--no-pretrained", cmd)

    def test_export_cmd(self):
        cmd = build_export_cmd("python3", "best.pt", "model.onnx", "224x224")
        self.assertEqual(cmd[cmd.index("--checkpoint") + 1], "best.pt")
        self.assertEqual(cmd[cmd.index("--out") + 1], "model.onnx")
        self.assertEqual(cmd[cmd.index("--size") + 1], "224x224")

    def test_preview_cmd(self):
        cmd = build_preview_cmd("python3", "frames", "model.onnx")
        self.assertEqual(cmd[0], "python3")
        self.assertEqual(cmd[1], str(ML_DIR / "preview.py"))
        self.assertIn("frames", cmd)
        self.assertEqual(cmd[cmd.index("--model") + 1], "model.onnx")


class TestNewRunDirStr(unittest.TestCase):
    def test_is_under_ml_runs_with_a_timestamp_name(self):
        s = new_run_dir_str()
        self.assertTrue(s.startswith("ml/runs/"), s)
        self.assertRegex(s[len("ml/runs/"):], r"^\d{8}_\d{6}$")


class TestParseEpochLine(unittest.TestCase):
    def test_extracts_epoch_loss_and_iou(self):
        parsed = parse_epoch_line("epoch   3/30  loss=0.1234  val_iou=0.567  (12s)")
        self.assertEqual(parsed, (3, 0.1234, 0.567))

    def test_nan_iou_becomes_none(self):
        parsed = parse_epoch_line("epoch   1/1  loss=0.5000  val_iou=nan  (1s)")
        self.assertEqual(parsed, (1, 0.5, None))

    def test_unrelated_line_returns_none(self):
        self.assertIsNone(parse_epoch_line("# device: cpu  学習 40枚 / 検証 7枚"))


class TestPaths(unittest.TestCase):
    def test_default_sam_checkpoint_path_is_under_ml_checkpoints(self):
        p = default_sam_checkpoint_path()
        self.assertEqual(p.name, "sam_vit_b_01ec64.pth")
        self.assertEqual(p.parent.name, "checkpoints")

    def test_rel_shortens_paths_under_repo_root(self):
        p = REPO_ROOT / "ml" / "data" / "frames"
        self.assertEqual(rel(p), "ml/data/frames")

    def test_rel_falls_back_to_absolute_outside_repo(self):
        p = Path("/some/other/place")
        self.assertEqual(rel(p), str(p))


class TestDownloadFile(unittest.TestCase):
    def test_writes_all_bytes_and_reports_progress(self):
        chunks = [b"a" * 10, b"b" * 10, b""]

        class FakeResponse:
            length = 20

            def __enter__(self):
                return self

            def __exit__(self, *exc):
                return False

            def read(self, n):
                return chunks.pop(0)

        seen = []
        with tempfile.TemporaryDirectory() as d:
            dest = Path(d) / "sub" / "model.pth"
            with patch("app.urllib.request.urlopen", return_value=FakeResponse()):
                download_file("http://example.invalid/model.pth", dest,
                              progress_cb=lambda got, total: seen.append((got, total)))
            self.assertEqual(dest.read_bytes(), b"a" * 10 + b"b" * 10)
            self.assertEqual(seen, [(10, 20), (20, 20)])
            # 一時ファイルが残っていないこと
            self.assertFalse((dest.parent / (dest.name + ".part")).exists())

    def test_failure_does_not_leave_a_file_at_the_destination(self):
        class FailingResponse:
            length = None

            def __enter__(self):
                return self

            def __exit__(self, *exc):
                return False

            def read(self, n):
                raise ConnectionError("boom")

        with tempfile.TemporaryDirectory() as d:
            dest = Path(d) / "model.pth"
            with patch("app.urllib.request.urlopen", return_value=FailingResponse()):
                with self.assertRaises(ConnectionError):
                    download_file("http://example.invalid/model.pth", dest)
            self.assertFalse(dest.exists(), "失敗したのに dest にファイルが残っている")


@unittest.skipUnless(_HAS_DISPLAY, "ディスプレイが無い環境（ヘッドレスCI等）ではスキップ")
class TestAppWidgetsSmoke(unittest.TestCase):
    """ウィジェットが例外無く組み立てられることだけを見る。クリック操作は試さない。"""

    def test_app_constructs_without_raising(self):
        root = tk.Tk()
        try:
            root.withdraw()
            app = App(root)
            self.assertIsNotNone(app.log_widget)
            self.assertEqual(app.status_label.cget("text"), "待機中")
        finally:
            root.destroy()


if __name__ == "__main__":
    unittest.main()
