"""`ml/export_onnx.py` のテスト。

**実データ・実学習は要らない。** ランダム初期化のモデルをそのままエクスポート
し、PyTorch と ONNXRuntime の出力が一致すること（＝配管が正しいこと）だけを
確認する。推論の「正しさ」（意味のあるセグメンテーションかどうか）は
実データが学習された後でしか確認できない。
"""

import json
import sys
import tempfile
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))  # ml/

import torch  # noqa: E402

from export_onnx import MEAN, STD, THRESHOLD, export, verify_parity  # noqa: E402
from model import DrivableSegModel  # noqa: E402


class TestExportOnnx(unittest.TestCase):
    def test_export_writes_onnx_and_config(self):
        with tempfile.TemporaryDirectory() as d:
            ckpt_path = Path(d) / "model.pt"
            model = DrivableSegModel(pretrained=False)
            torch.save(model.state_dict(), ckpt_path)

            out_path = Path(d) / "model.onnx"
            export(ckpt_path, out_path, (64, 48))

            self.assertTrue(out_path.exists())
            cfg_path = out_path.with_suffix(".json")
            self.assertTrue(cfg_path.exists())
            cfg = json.loads(cfg_path.read_text())
            self.assertEqual(cfg["input_size"], [64, 48])
            self.assertEqual(cfg["mean"], MEAN)
            self.assertEqual(cfg["std"], STD)
            self.assertEqual(cfg["threshold"], THRESHOLD)

    def test_onnxruntime_output_matches_pytorch(self):
        with tempfile.TemporaryDirectory() as d:
            ckpt_path = Path(d) / "model.pt"
            model = DrivableSegModel(pretrained=False)
            torch.save(model.state_dict(), ckpt_path)

            out_path = Path(d) / "model.onnx"
            export(ckpt_path, out_path, (64, 48))

            reloaded = DrivableSegModel(pretrained=False)
            reloaded.load_state_dict(torch.load(ckpt_path, map_location="cpu"))
            err = verify_parity(out_path, reloaded, (64, 48))
            self.assertLess(err, 1e-3)

    def test_mismatched_model_raises(self):
        """**エクスポートできただけでは何も保証されない**——別の重みで検証すると
        検出できることを確認する。"""
        with tempfile.TemporaryDirectory() as d:
            ckpt_path = Path(d) / "model.pt"
            model = DrivableSegModel(pretrained=False)
            torch.save(model.state_dict(), ckpt_path)

            out_path = Path(d) / "model.onnx"
            export(ckpt_path, out_path, (64, 48))

            different_model = DrivableSegModel(pretrained=False)   # 別の乱数初期化
            with self.assertRaises(ValueError):
                verify_parity(out_path, different_model, (64, 48), atol=1e-6)


if __name__ == "__main__":
    unittest.main()
