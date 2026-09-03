"""`ml_cam_e2e/model.py` のテスト。**事前学習重みはダウンロードしない**（`pretrained=False`）。"""

import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))  # ml_cam_e2e/

import torch  # noqa: E402

from model import DriveRegressionModel  # noqa: E402


class TestDriveRegressionModel(unittest.TestCase):
    def test_output_shape_is_one_scalar_per_image(self):
        model = DriveRegressionModel(pretrained=False)
        model.eval()
        x = torch.rand(3, 3, 64, 64)
        with torch.no_grad():
            out = model(x)
        self.assertEqual(out.shape, (3, 1))

    def test_output_is_within_tanh_range(self):
        model = DriveRegressionModel(pretrained=False)
        model.eval()
        x = torch.rand(4, 3, 96, 128)
        with torch.no_grad():
            out = model(x)
        self.assertGreaterEqual(float(out.min()), -1.0)
        self.assertLessEqual(float(out.max()), 1.0)

    def test_handles_non_square_resolution(self):
        model = DriveRegressionModel(pretrained=False)
        model.eval()
        x = torch.rand(1, 3, 100, 150)
        with torch.no_grad():
            out = model(x)
        self.assertEqual(out.shape, (1, 1))

    def test_gradients_flow_for_training(self):
        model = DriveRegressionModel(pretrained=False)
        model.train()
        x = torch.rand(2, 3, 64, 64)
        out = model(x)
        loss = out.mean()
        loss.backward()
        grad_norms = [p.grad.abs().sum().item() for p in model.parameters()
                     if p.grad is not None]
        self.assertTrue(grad_norms, "勾配が1つも流れていない")
        self.assertGreater(sum(grad_norms), 0.0)


if __name__ == "__main__":
    unittest.main()
