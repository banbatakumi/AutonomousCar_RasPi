"""`ml/model.py` のテスト。**事前学習重みはダウンロードしない**
（`pretrained=False`）——ネットワーク接続が無い環境でも通ること。
"""

import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))  # ml/

import torch  # noqa: E402

from model import DrivableSegModel  # noqa: E402


class TestDrivableSegModel(unittest.TestCase):
    def test_output_shape_matches_input_resolution(self):
        model = DrivableSegModel(pretrained=False)
        model.eval()
        x = torch.rand(2, 3, 64, 64)
        with torch.no_grad():
            out = model(x)
        self.assertEqual(out.shape, (2, 1, 64, 64))

    def test_output_is_a_probability(self):
        """シグモイド出力なので必ず [0, 1] に収まる。"""
        model = DrivableSegModel(pretrained=False)
        model.eval()
        x = torch.rand(1, 3, 96, 128)
        with torch.no_grad():
            out = model(x)
        self.assertGreaterEqual(float(out.min()), 0.0)
        self.assertLessEqual(float(out.max()), 1.0)

    def test_handles_non_multiple_of_32_resolution(self):
        """デコーダの累積アップサンプルが入力サイズにぴったり戻らない解像度でも、
        最後の `interpolate` で必ず入力と同じ形になること。"""
        model = DrivableSegModel(pretrained=False)
        model.eval()
        x = torch.rand(1, 3, 100, 150)
        with torch.no_grad():
            out = model(x)
        self.assertEqual(out.shape[-2:], (100, 150))

    def test_gradients_flow_for_training(self):
        """学習ループが回る前提条件——勾配がエンコーダまで伝わること。"""
        model = DrivableSegModel(pretrained=False)
        model.train()
        x = torch.rand(1, 3, 64, 64)
        out = model(x)
        loss = out.mean()
        loss.backward()
        grad_norms = [p.grad.abs().sum().item() for p in model.parameters()
                     if p.grad is not None]
        self.assertTrue(grad_norms, "勾配が1つも流れていない")
        self.assertGreater(sum(grad_norms), 0.0)


if __name__ == "__main__":
    unittest.main()
