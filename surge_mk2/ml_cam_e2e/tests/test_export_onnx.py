"""`ml_cam_e2e/export_onnx.py` のテスト。実データ・実学習は要らない
（`ml_cam/tests/test_export_onnx.py` と同じ方針）。
"""

import json
import shutil
import sys
import tempfile
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))  # ml_cam_e2e/

import onnxruntime as ort  # noqa: E402
import torch  # noqa: E402

from export_onnx import MEAN, STD, export, verify_parity  # noqa: E402
from model import DriveRegressionModel  # noqa: E402


class TestExportOnnx(unittest.TestCase):
    def test_export_writes_onnx_and_config(self):
        with tempfile.TemporaryDirectory() as d:
            ckpt_path = Path(d) / "model.pt"
            model = DriveRegressionModel(pretrained=False)
            torch.save(model.state_dict(), ckpt_path)

            out_path = Path(d) / "model.onnx"
            export(ckpt_path, out_path, (64, 48), max_steer=0.524)

            self.assertTrue(out_path.exists())
            cfg = json.loads(out_path.with_suffix(".json").read_text())
            self.assertEqual(cfg["input_size"], [64, 48])
            self.assertEqual(cfg["mean"], MEAN)
            self.assertEqual(cfg["std"], STD)
            self.assertEqual(cfg["max_steer"], 0.524)
            self.assertEqual(cfg["note"], "")

    def test_note_is_embedded_in_the_exported_json(self):
        with tempfile.TemporaryDirectory() as d:
            ckpt_path = Path(d) / "model.pt"
            model = DriveRegressionModel(pretrained=False)
            torch.save(model.state_dict(), ckpt_path)

            out_path = Path(d) / "model.onnx"
            export(ckpt_path, out_path, (64, 48), max_steer=0.524, note="夜間走行用")

            cfg = json.loads(out_path.with_suffix(".json").read_text())
            self.assertEqual(cfg["note"], "夜間走行用")

    def test_default_max_steer_comes_from_vehicle_toml(self):
        """`max_steer` を省略し、`train_config.json` も無ければ `Vehicle.load()` の値を使う。"""
        with tempfile.TemporaryDirectory() as d:
            ckpt_path = Path(d) / "model.pt"
            model = DriveRegressionModel(pretrained=False)
            torch.save(model.state_dict(), ckpt_path)

            out_path = Path(d) / "model.onnx"
            export(ckpt_path, out_path, (64, 48))

            cfg = json.loads(out_path.with_suffix(".json").read_text())
            self.assertGreater(cfg["max_steer"], 0.0)

    def test_max_steer_prefers_train_config_over_vehicle_toml(self):
        """`train_config.json` があれば、`config/vehicle.toml` の現在値とは無関係に
        学習時に記録された `max_steer` を使う（レビュー指摘: 後日 vehicle.toml が
        sysid で更新されても、過去に学習した best.pt の再エクスポートは学習時の
        正規化基準を維持しなければならない）。"""
        with tempfile.TemporaryDirectory() as d:
            ckpt_path = Path(d) / "best.pt"
            model = DriveRegressionModel(pretrained=False)
            torch.save(model.state_dict(), ckpt_path)
            # train.py が書くのと同じ形式（train.py:143-146参照）。値は現在の
            # vehicle.toml とは明確に異なるものにして、優先されたことを検証する。
            (Path(d) / "train_config.json").write_text(json.dumps({
                "input_size": [64, 48], "epochs": 1, "best_val_mae": 0.1,
                "max_steer": 0.777, "n_train": 1, "n_val": 1,
            }))

            out_path = Path(d) / "model.onnx"
            export(ckpt_path, out_path, (64, 48))

            cfg = json.loads(out_path.with_suffix(".json").read_text())
            self.assertEqual(cfg["max_steer"], 0.777)

    def test_explicit_max_steer_overrides_train_config(self):
        """呼び出し時に `max_steer` を明示すれば、`train_config.json` があっても
        それが最優先される（CLIの `--max-steer` と対称の経路）。"""
        with tempfile.TemporaryDirectory() as d:
            ckpt_path = Path(d) / "best.pt"
            model = DriveRegressionModel(pretrained=False)
            torch.save(model.state_dict(), ckpt_path)
            (Path(d) / "train_config.json").write_text(json.dumps({"max_steer": 0.777}))

            out_path = Path(d) / "model.onnx"
            export(ckpt_path, out_path, (64, 48), max_steer=0.4)

            cfg = json.loads(out_path.with_suffix(".json").read_text())
            self.assertEqual(cfg["max_steer"], 0.4)

    def test_train_config_without_max_steer_key_falls_back_to_vehicle_toml(self):
        """`train_config.json` はあるが `max_steer` キーが無ければフォールバックする。"""
        with tempfile.TemporaryDirectory() as d:
            ckpt_path = Path(d) / "best.pt"
            model = DriveRegressionModel(pretrained=False)
            torch.save(model.state_dict(), ckpt_path)
            (Path(d) / "train_config.json").write_text(json.dumps({"epochs": 1}))

            out_path = Path(d) / "model.onnx"
            export(ckpt_path, out_path, (64, 48))

            cfg = json.loads(out_path.with_suffix(".json").read_text())
            self.assertGreater(cfg["max_steer"], 0.0)
            self.assertNotEqual(cfg["max_steer"], 0.777)

    def test_onnxruntime_output_matches_pytorch(self):
        with tempfile.TemporaryDirectory() as d:
            ckpt_path = Path(d) / "model.pt"
            model = DriveRegressionModel(pretrained=False)
            torch.save(model.state_dict(), ckpt_path)

            out_path = Path(d) / "model.onnx"
            export(ckpt_path, out_path, (64, 48), max_steer=0.524)

            reloaded = DriveRegressionModel(pretrained=False)
            reloaded.load_state_dict(torch.load(ckpt_path, map_location="cpu"))
            err = verify_parity(out_path, reloaded, (64, 48))
            self.assertLess(err, 1e-3)

    def test_onnx_file_loads_alone_without_its_export_directory(self):
        """`external_data=False` により `.onnx` 単体で読める（`ml_cam/export_onnx.py`
        と同じ回帰検出テスト）。"""
        with tempfile.TemporaryDirectory() as d:
            ckpt_path = Path(d) / "model.pt"
            model = DriveRegressionModel(pretrained=False)
            torch.save(model.state_dict(), ckpt_path)

            out_path = Path(d) / "model.onnx"
            export(ckpt_path, out_path, (64, 48), max_steer=0.524)

            with tempfile.TemporaryDirectory() as lone_dir:
                lone_path = Path(lone_dir) / "renamed.onnx"
                shutil.copy(out_path, lone_path)
                sess = ort.InferenceSession(str(lone_path), providers=["CPUExecutionProvider"])
                sess.run(None, {"input": torch.zeros(1, 3, 48, 64).numpy()})

    def test_mismatched_model_raises(self):
        with tempfile.TemporaryDirectory() as d:
            ckpt_path = Path(d) / "model.pt"
            model = DriveRegressionModel(pretrained=False)
            torch.save(model.state_dict(), ckpt_path)

            out_path = Path(d) / "model.onnx"
            export(ckpt_path, out_path, (64, 48), max_steer=0.524)

            different_model = DriveRegressionModel(pretrained=False)
            with self.assertRaises(ValueError):
                verify_parity(out_path, different_model, (64, 48), atol=1e-6)


if __name__ == "__main__":
    unittest.main()
