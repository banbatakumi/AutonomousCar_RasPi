"""`raspi/nodes/telemetry_node.py`のうち、`self`に依存しない純粋な部分だけを狙って
テストする。`TelemetryServer`本体はソケット類を抱えて重く、軽量に構築する口が
無いので、対象を「`self`を一切使わないメソッド」に絞ってある
（`_e2e_models_list`は`TelemetryServer._e2e_models_list(None)`のように未束縛でも
呼べる）。
"""

import json
import sys
import tempfile
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

import raspi.nodes.telemetry_node as tn  # noqa: E402


class TestAtomicWriteBytes(unittest.TestCase):
    """★ C4: 設定JSONの書き込みが `os.replace()` でアトミックであること。"""

    def test_writes_the_content(self):
        with tempfile.TemporaryDirectory() as td:
            path = Path(td) / "sub" / "auto.json"
            tn._atomic_write_bytes(path, b'{"mode":"ftg"}')
            self.assertEqual(path.read_bytes(), b'{"mode":"ftg"}')

    def test_failure_mid_write_does_not_corrupt_existing_file(self):
        """途中で例外が起きても、既存ファイルの中身は古いまま保たれること。

        `os.fdopen(...).write()` が例外を投げても、`os.replace()` に
        到達しないので元ファイルは触られない——tmpファイルへの直書きだけが
        失敗するアトミック置換の要点。
        """
        from unittest import mock

        with tempfile.TemporaryDirectory() as td:
            path = Path(td) / "auto.json"
            path.write_bytes(b'{"mode":"old"}')

            with mock.patch("os.replace", side_effect=RuntimeError("boom")):
                with self.assertRaises(RuntimeError):
                    tn._atomic_write_bytes(path, b'{"mode":"new"}')

            self.assertEqual(path.read_bytes(), b'{"mode":"old"}')

    def test_no_leftover_tmp_file_on_success(self):
        with tempfile.TemporaryDirectory() as td:
            path = Path(td) / "auto.json"
            tn._atomic_write_bytes(path, b"{}")
            leftovers = [p for p in Path(td).iterdir() if p.name != "auto.json"]
            self.assertEqual(leftovers, [])


class TestE2eModelsListNote(unittest.TestCase):
    """`_e2e_models_list()`が`<名前>.json`の`note`（`ml_lidar/export_onnx_rl.py`が
    書く自由記述の備考。2026-08-29追加）を拾って返すことを確認する。"""

    def setUp(self) -> None:
        self._orig_dir = tn.E2E_MODELS_DIR
        self._tmp = tempfile.TemporaryDirectory()
        tn.E2E_MODELS_DIR = Path(self._tmp.name)
        self.addCleanup(self._cleanup)

    def _cleanup(self) -> None:
        tn.E2E_MODELS_DIR = self._orig_dir
        self._tmp.cleanup()

    def _by_name(self, files: list[dict]) -> dict[str, dict]:
        return {f["name"]: f for f in files}

    def test_reads_note_from_sibling_json(self) -> None:
        d = tn.E2E_MODELS_DIR
        (d / "v1.onnx").write_bytes(b"dummy")
        (d / "v1.json").write_text(json.dumps({"note": "グリップ限界+報酬改善版"}))

        result = tn.TelemetryServer._e2e_models_list(None)
        files = self._by_name(result["e2e_model_files"])
        self.assertEqual(files["v1"]["note"], "グリップ限界+報酬改善版")
        self.assertTrue(files["v1"]["has_config"])

    def test_missing_json_gives_empty_note(self) -> None:
        d = tn.E2E_MODELS_DIR
        (d / "v2.onnx").write_bytes(b"dummy")

        result = tn.TelemetryServer._e2e_models_list(None)
        files = self._by_name(result["e2e_model_files"])
        self.assertEqual(files["v2"]["note"], "")
        self.assertFalse(files["v2"]["has_config"])

    def test_corrupt_json_gives_empty_note_without_crashing(self) -> None:
        d = tn.E2E_MODELS_DIR
        (d / "v3.onnx").write_bytes(b"dummy")
        (d / "v3.json").write_text("{ not json")

        result = tn.TelemetryServer._e2e_models_list(None)
        files = self._by_name(result["e2e_model_files"])
        self.assertEqual(files["v3"]["note"], "")
        self.assertTrue(files["v3"]["has_config"])   # ファイルは在る（壊れているだけ）

    def test_json_without_note_field_gives_empty_note(self) -> None:
        d = tn.E2E_MODELS_DIR
        (d / "v4.onnx").write_bytes(b"dummy")
        (d / "v4.json").write_text(json.dumps({"max_speed": 1.5}))

        result = tn.TelemetryServer._e2e_models_list(None)
        files = self._by_name(result["e2e_model_files"])
        self.assertEqual(files["v4"]["note"], "")


class TestCamModelsListNote(unittest.TestCase):
    """`_cam_models_list()`が`<名前>.json`の`note`（`ml_cam/export_onnx.py`が
    書く自由記述の備考。`TestE2eModelsListNote`と対称、2026-08-29追加）を拾って
    返すことを確認する。"""

    def setUp(self) -> None:
        self._orig_dir = tn.MODELS_DIR
        self._tmp = tempfile.TemporaryDirectory()
        tn.MODELS_DIR = Path(self._tmp.name)
        self.addCleanup(self._cleanup)

    def _cleanup(self) -> None:
        tn.MODELS_DIR = self._orig_dir
        self._tmp.cleanup()

    def _by_name(self, files: list[dict]) -> dict[str, dict]:
        return {f["name"]: f for f in files}

    def test_reads_note_from_sibling_json(self) -> None:
        d = tn.MODELS_DIR
        (d / "v1.onnx").write_bytes(b"dummy")
        (d / "v1.json").write_text(json.dumps({"note": "夜間走行用、露出補正あり"}))

        result = tn.TelemetryServer._cam_models_list(None)
        files = self._by_name(result["cam_model_files"])
        self.assertEqual(files["v1"]["note"], "夜間走行用、露出補正あり")
        self.assertTrue(files["v1"]["has_config"])

    def test_missing_json_gives_empty_note(self) -> None:
        d = tn.MODELS_DIR
        (d / "v2.onnx").write_bytes(b"dummy")

        result = tn.TelemetryServer._cam_models_list(None)
        files = self._by_name(result["cam_model_files"])
        self.assertEqual(files["v2"]["note"], "")
        self.assertFalse(files["v2"]["has_config"])

    def test_corrupt_json_gives_empty_note_without_crashing(self) -> None:
        d = tn.MODELS_DIR
        (d / "v3.onnx").write_bytes(b"dummy")
        (d / "v3.json").write_text("{ not json")

        result = tn.TelemetryServer._cam_models_list(None)
        files = self._by_name(result["cam_model_files"])
        self.assertEqual(files["v3"]["note"], "")
        self.assertTrue(files["v3"]["has_config"])   # ファイルは在る（壊れているだけ）

    def test_json_without_note_field_gives_empty_note(self) -> None:
        d = tn.MODELS_DIR
        (d / "v4.onnx").write_bytes(b"dummy")
        (d / "v4.json").write_text(json.dumps({"input_size": [224, 224]}))

        result = tn.TelemetryServer._cam_models_list(None)
        files = self._by_name(result["cam_model_files"])
        self.assertEqual(files["v4"]["note"], "")


if __name__ == "__main__":
    unittest.main()
