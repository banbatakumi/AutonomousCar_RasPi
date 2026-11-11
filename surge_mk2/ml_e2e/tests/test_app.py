"""`ml_e2e/app.py` のテスト。純粋関数（コマンド組み立て・標本の一覧と取り外し・0枚の警告）と、
ウィンドウを実際に作って今のモデルの内容が各タブに映ることを確かめる（クリック操作はテストしない）。"""

import json
import sys
import tempfile
import tkinter as tk
import unittest
from pathlib import Path
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))  # surge_mk2/

from ml_e2e import app as A  # noqa: E402


def _meta() -> dict:
    def rec(n, maps, **kw):
        return {"file": "x.mcap", "samples": n, "line": n - 10, "future": 10, "auto": n, "manual": 0,
                "maps": maps, **kw}
    return {"config": {"k": 40, "ds": 0.25}, "records": {
        "a1": rec(100, ["A"], path="/tmp/a1.mcap"), "a2": rec(200, ["A"]), "a3": rec(300, ["A"]),
        "b1": rec(50, ["B"]), "b2": rec(60, ["B"]), "m1": rec(40, []),
        "empty": {"file": "e.mcap", "samples": 0, "skipped": "点群が無い"}}}


class TestCommands(unittest.TestCase):
    def test_extract(self):
        cmd = A.build_extract_cmd("py", ["a.mcap", "b.mcap"], "out", include_self=True)
        self.assertEqual(cmd, ["py", "-m", "ml_e2e.extract", "a.mcap", "b.mcap", "--out", "out",
                               "--include-self"])
        self.assertNotIn("--include-self", A.build_extract_cmd("py", ["a.mcap"], "out"))

    def test_train(self):
        cmd = A.build_train_cmd("py", "d", "o", epochs=5, batch_size=32, val=["r1", "r2"], init="x/best.pt",
                                no_flip=True)
        self.assertEqual(cmd[:11], ["py", "-m", "ml_e2e.train", "--data", "d", "--out", "o", "--epochs", "5",
                                    "--batch-size", "32"])
        self.assertEqual(cmd[11:], ["--val", "r1", "r2", "--init", "x/best.pt", "--no-flip"])
        plain = A.build_train_cmd("py", "d", "o", epochs=5, batch_size=32)
        self.assertFalse({"--val", "--init", "--no-flip", "--no-pretrained"} & set(plain))

    def test_check_export_eval(self):
        self.assertEqual(A.build_check_cmd("py", "d", 12), ["py", "-m", "ml_e2e.check_labels", "d", "--draw", "12"])
        self.assertEqual(A.build_export_cmd("py", "run", "v3"),
                         ["py", "-m", "ml_e2e.export_onnx", "--run", "run", "--name", "v3"])
        cmd = A.build_eval_cmd("py", "m.onnx", "d", ["a.mcap"], no_image=True)
        self.assertEqual(cmd, ["py", "-m", "ml_e2e.eval", "--model", "m.onnx", "--data", "d", "a.mcap",
                               "--no-image"])

    def test_commands_match_the_scripts_arguments(self):
        """組み立てた引数を、実際のスクリプトの argparse が受け付ける（名前の食い違いを見つける）。"""
        import importlib

        cases = (("ml_e2e.extract", A.build_extract_cmd("py", ["a.mcap"], "out", include_self=True)),
                 ("ml_e2e.train", A.build_train_cmd("py", "d", "o", epochs=1, batch_size=2, val=["r"], init="i",
                                                    no_flip=True, no_pretrained=True)),
                 ("ml_e2e.export_onnx", A.build_export_cmd("py", "run", "v1")),
                 ("ml_e2e.check_labels", A.build_check_cmd("py", "d", 3)),
                 ("ml_e2e.eval", A.build_eval_cmd("py", "m.onnx", "d", ["a.mcap"], no_image=True, no_lidar=True)))
        import argparse
        for mod_name, cmd in cases:
            mod = importlib.import_module(mod_name)
            seen = {}
            with patch.object(argparse.ArgumentParser, "parse_args",
                              lambda self, *a, **k: seen.setdefault("ns", self.parse_known_args(cmd[3:]))):
                try:
                    mod.main()
                except Exception:                    # noqa: BLE001  引数を読んだ後の処理は見ない
                    pass
            self.assertIn("ns", seen, mod_name)
            self.assertEqual(seen["ns"][1], [], f"{mod_name} が受け付けない引数: {seen['ns'][1]}")


class TestDataset(unittest.TestCase):
    def test_rows_skip_empty_records(self):
        rows = A.record_rows(_meta())
        self.assertEqual([r[0] for r in rows], ["a1", "a2", "a3", "b1", "b2", "m1"])
        self.assertEqual(rows[0], ("a1", 100, 90, 10, 100, 0, "A"))

    def test_default_val_is_the_last_record_of_maps_with_three_or_more(self):
        self.assertEqual(A.default_val_records(_meta()), ["a3"])

    def test_read_dataset_tolerates_missing_and_broken_files(self):
        with tempfile.TemporaryDirectory() as d:
            self.assertEqual(A.read_dataset(Path(d))["records"], {})
            (Path(d) / "dataset.json").write_text("{broken")
            self.assertEqual(A.read_dataset(Path(d))["records"], {})

    def test_remove_record_deletes_samples_and_keeps_the_rest(self):
        with tempfile.TemporaryDirectory() as d:
            data = Path(d)
            (data / "dataset.json").write_text(json.dumps(_meta()))
            for n in ("a1", "a2"):
                (data / f"{n}.npz").write_bytes(b"x")
                (data / "frames" / n).mkdir(parents=True)
                (data / "frames" / n / "1.jpg").write_bytes(b"x")
            A.remove_record(data, "a1")
            self.assertFalse((data / "a1.npz").exists())
            self.assertFalse((data / "frames" / "a1").exists())
            self.assertTrue((data / "a2.npz").exists() and (data / "frames" / "a2" / "1.jpg").exists())
            names = [r[0] for r in A.record_rows(A.read_dataset(data))]
            self.assertNotIn("a1", names)
            self.assertIn("a2", names)
            A.remove_record(data, "nothing")             # 無い名前でも落ちない

    def test_status_counts_records_and_samples(self):
        with tempfile.TemporaryDirectory() as d:
            (Path(d) / "data").mkdir()
            (Path(d) / "data" / "dataset.json").write_text(json.dumps(_meta()))
            self.assertEqual(A.describe_model_status(Path(d)), "記録 6本・標本 750枚・未学習")
            (Path(d) / "best.pt").write_bytes(b"x")
            self.assertIn("✓学習済み", A.describe_model_status(Path(d)))


class TestExtractWarning(unittest.TestCase):
    def test_none_when_samples_were_written(self):
        self.assertIsNone(A.extract_warning(["# a.mcap: 10枚（…）", "# 合計 10枚 → out"]))

    def test_zero_samples_reports_the_reasons(self):
        msg = A.extract_warning(["# 合計 0枚 → out", "# 使わなかったコマ: トルクモード 120枚・DISARM中 3枚"])
        self.assertIn("トルクモード 120枚", msg)
        self.assertIn("速度モードで録ってください", msg)

    def test_zero_samples_without_reasons_suspects_missing_images(self):
        self.assertIn("画像", A.extract_warning(["# 合計 0枚 → out"]))

    def test_epoch_line_of_train_is_parsed(self):
        line = "epoch   3/20  loss=0.9112  val_mae=0.5800  near=0.176m  b=0.34m  v=0.21m/s  (197s)"
        self.assertEqual(A.parse_epoch_line(line), (3, 0.9112, 0.58))


def _has_display() -> bool:
    try:
        root = tk.Tk()
        root.withdraw()
        root.destroy()
        return True
    except tk.TclError:
        return False


@unittest.skipUnless(_has_display(), "画面が無い")
class TestWindow(unittest.TestCase):
    def test_window_shows_the_current_model(self):
        with tempfile.TemporaryDirectory() as d:
            runs = Path(d) / "runs"
            data = runs / "v1" / "data"
            data.mkdir(parents=True)
            (data / "dataset.json").write_text(json.dumps(_meta()))
            with patch.object(A, "RUNS_DIR", runs), patch.object(A, "MODELS_DIR", Path(d) / "models"):
                root = tk.Tk()
                root.withdraw()
                try:
                    app = A.App(root)
                    root.update()
                    self.assertEqual(app.model_name_var.get(), "v1")
                    self.assertEqual(list(app.record_tree.get_children()),
                                     ["a1", "a2", "a3", "b1", "b2", "m1"])
                    self.assertEqual([app.val_list.get(i) for i in app.val_list.curselection()], ["a3"])
                    self.assertIn("標本 750枚", app.model_status_var.get())
                    # 別の名前にすると空になり、戻すと元に戻る
                    app.model_name_var.set("v9")
                    self.assertEqual(list(app.record_tree.get_children()), [])
                    app.model_name_var.set("v1")
                    self.assertEqual(len(app.record_tree.get_children()), 6)
                finally:
                    root.destroy()


if __name__ == "__main__":
    unittest.main()
