"""`ml_common/onnx_export.py` のテスト（版の一致・契約の書き出し・書き出しと突き合わせ）。"""

import json
import sys
import tempfile
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))  # surge_mk2/

from ml_common import onnx_export  # noqa: E402


class TestCheckVersion(unittest.TestCase):
    def test_only_the_same_version_passes(self):
        onnx_export.check_version("前処理", 3, 3)
        for trained in (1, 2, None):
            with self.assertRaises(ValueError):
                onnx_export.check_version("前処理", trained, 3)


class TestContractAndConfig(unittest.TestCase):
    def test_contract_is_written_next_to_the_onnx(self):
        with tempfile.TemporaryDirectory() as d:
            path = onnx_export.write_contract(Path(d) / "v1.onnx", {"note": "夜間", "k": 3})
            self.assertEqual(path, Path(d) / "v1.json")
            self.assertEqual(json.loads(path.read_text(encoding="utf-8")), {"note": "夜間", "k": 3})
            self.assertIn("夜間", path.read_text(encoding="utf-8"))

    def test_missing_or_broken_train_config_is_empty(self):
        with tempfile.TemporaryDirectory() as d:
            self.assertEqual(onnx_export.read_train_config(Path(d)), {})
            (Path(d) / "train_config.json").write_text("{")
            self.assertEqual(onnx_export.read_train_config(Path(d)), {})


class TestExportAndParity(unittest.TestCase):
    def test_two_input_model_round_trips_and_a_different_model_is_caught(self):
        import numpy as np
        import torch

        class Net(torch.nn.Module):
            def __init__(self):
                super().__init__()
                self.a = torch.nn.Linear(4, 2)
                self.b = torch.nn.Linear(3, 2)

            def forward(self, x, y):
                return self.a(x) + self.b(y)

        torch.manual_seed(0)
        model = Net().eval()

        def make(rng):
            return rng.random((1, 4), dtype=np.float32), rng.random((1, 3), dtype=np.float32)

        names = ("x", "y")
        with tempfile.TemporaryDirectory() as d:
            out = Path(d) / "sub" / "m.onnx"
            onnx_export.export(model, make(np.random.default_rng(0)), out, input_names=names)
            self.assertTrue(out.exists())
            self.assertFalse(out.with_suffix(".onnx.data").exists())      # 重みを別ファイルに切り出さない
            self.assertLess(onnx_export.verify_parity(out, model, make, input_names=names), 1e-5)
            with self.assertRaises(ValueError):
                onnx_export.verify_parity(out, Net().eval(), make, input_names=names, atol=1e-6)


if __name__ == "__main__":
    unittest.main()
