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

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))  # repo root

import numpy as np  # noqa: E402
import onnxruntime as ort  # noqa: E402
import torch  # noqa: E402

from export_onnx import MEAN, STD, export, verify_parity  # noqa: E402
from model import DriveRegressionModel  # noqa: E402
from raspi.core.cam_e2e_preproc import PREPROC_VERSION  # noqa: E402
from raspi.nodes.cam_e2e_node import MODEL_OUTPUTS, load_model  # noqa: E402


def _save_checkpoint(d: str, name: str = "best.pt") -> Path:
    ckpt_path = Path(d) / name
    torch.save(DriveRegressionModel(pretrained=False).state_dict(), ckpt_path)
    return ckpt_path


def _write_train_config(d: str, **over) -> None:
    cfg = {"input_size": [64, 48], "max_steer": 0.777, "speed_ref": 1.25}
    cfg.update(over)
    (Path(d) / "train_config.json").write_text(json.dumps(cfg))


class TestExportOnnx(unittest.TestCase):
    def test_export_writes_onnx_and_the_full_contract(self):
        with tempfile.TemporaryDirectory() as d:
            ckpt_path = _save_checkpoint(d)
            out_path = Path(d) / "model.onnx"
            export(ckpt_path, out_path, (64, 48), max_steer=0.524, speed_ref=2.0)

            self.assertTrue(out_path.exists())
            cfg = json.loads(out_path.with_suffix(".json").read_text())
            self.assertEqual(cfg["input_size"], [64, 48])
            self.assertEqual(cfg["mean"], MEAN)
            self.assertEqual(cfg["std"], STD)
            self.assertEqual(cfg["max_steer"], 0.524)
            self.assertEqual(cfg["speed_ref"], 2.0)
            self.assertEqual(cfg["outputs"], list(MODEL_OUTPUTS))
            self.assertEqual(cfg["preproc_version"], PREPROC_VERSION)
            self.assertEqual(cfg["color"], "RGB")
            self.assertEqual(cfg["note"], "")

    def test_exported_model_is_accepted_by_the_vehicle_side_loader(self):
        """書いた契約を、実車が使う `load_model()` がそのまま読めること
        ——キー名や版がどちらかだけ変わると、ここで落ちる。"""
        with tempfile.TemporaryDirectory() as d:
            ckpt_path = _save_checkpoint(d)
            out_path = Path(d) / "model.onnx"
            export(ckpt_path, out_path, (64, 48), max_steer=0.524, speed_ref=2.0)

            model = load_model(out_path)
            self.assertEqual(model.input_size, (64, 48))
            steer, speed = model.infer(np.zeros((36, 64, 3), dtype=np.uint8))
            self.assertTrue(-1.0 <= steer <= 1.0)
            self.assertTrue(0.0 <= speed <= 1.0)

    def test_note_is_embedded_in_the_exported_json(self):
        with tempfile.TemporaryDirectory() as d:
            ckpt_path = _save_checkpoint(d)
            out_path = Path(d) / "model.onnx"
            export(ckpt_path, out_path, (64, 48), max_steer=0.524, speed_ref=2.0,
                   note="夜間走行用")

            cfg = json.loads(out_path.with_suffix(".json").read_text(encoding="utf-8"))
            self.assertEqual(cfg["note"], "夜間走行用")

    def test_everything_comes_from_train_config_by_default(self):
        """解像度も正規化の基準も学習時の値を使う。`config/vehicle.toml` が
        後で同定によって変わっても、過去に学習した best.pt の再エクスポートは
        学習時の基準を保つ。"""
        with tempfile.TemporaryDirectory() as d:
            ckpt_path = _save_checkpoint(d)
            _write_train_config(d)
            out_path = Path(d) / "model.onnx"
            size = export(ckpt_path, out_path)

            self.assertEqual(size, (64, 48))
            cfg = json.loads(out_path.with_suffix(".json").read_text())
            self.assertEqual(cfg["input_size"], [64, 48])
            self.assertEqual(cfg["max_steer"], 0.777)
            self.assertEqual(cfg["speed_ref"], 1.25)

    def test_explicit_values_override_train_config(self):
        with tempfile.TemporaryDirectory() as d:
            ckpt_path = _save_checkpoint(d)
            _write_train_config(d)
            out_path = Path(d) / "model.onnx"
            export(ckpt_path, out_path, (32, 32), max_steer=0.4, speed_ref=0.9)

            cfg = json.loads(out_path.with_suffix(".json").read_text())
            self.assertEqual(cfg["input_size"], [32, 32])
            self.assertEqual(cfg["max_steer"], 0.4)
            self.assertEqual(cfg["speed_ref"], 0.9)

    def test_missing_contract_values_raise_instead_of_guessing(self):
        """基準が分からないまま別の値で書き出さない（黙って違う舵角になるより止める）。"""
        with tempfile.TemporaryDirectory() as d:
            ckpt_path = _save_checkpoint(d)
            out_path = Path(d) / "model.onnx"
            with self.assertRaises(ValueError):
                export(ckpt_path, out_path)                       # train_config.json 無し
            with self.assertRaises(ValueError):
                export(ckpt_path, out_path, (64, 48))             # 基準が無い
            _write_train_config(d, speed_ref=0.0)
            with self.assertRaises(ValueError):
                export(ckpt_path, out_path)
            self.assertFalse(out_path.with_suffix(".json").exists())

    def test_onnxruntime_output_matches_pytorch(self):
        with tempfile.TemporaryDirectory() as d:
            ckpt_path = Path(d) / "model.pt"
            model = DriveRegressionModel(pretrained=False)
            torch.save(model.state_dict(), ckpt_path)

            out_path = Path(d) / "model.onnx"
            export(ckpt_path, out_path, (64, 48), max_steer=0.524, speed_ref=2.0)

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
            export(ckpt_path, out_path, (64, 48), max_steer=0.524, speed_ref=2.0)

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
            export(ckpt_path, out_path, (64, 48), max_steer=0.524, speed_ref=2.0)

            different_model = DriveRegressionModel(pretrained=False)
            with self.assertRaises(ValueError):
                verify_parity(out_path, different_model, (64, 48), atol=1e-6)


if __name__ == "__main__":
    unittest.main()
