"""`ml_cam_e2e/model.py` のテスト。**事前学習重みはダウンロードしない**（`pretrained=False`）。"""

import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))  # ml_cam_e2e/

import torch  # noqa: E402

from model import DriveRegressionModel  # noqa: E402


class TestDriveRegressionModel(unittest.TestCase):
    def test_output_is_steer_and_speed_per_image(self):
        model = DriveRegressionModel(pretrained=False)
        model.eval()
        x = torch.rand(3, 6, 64, 64)
        with torch.no_grad():
            out = model(x)
        self.assertEqual(out.shape, (3, 2))

    def test_steer_is_tanh_and_speed_is_sigmoid(self):
        """操舵は -1..1、速度は 0..1（前進のみ）。ヘッドを大きく振っても範囲を出ない。"""
        model = DriveRegressionModel(pretrained=False)
        model.eval()
        x = torch.rand(4, 6, 96, 128)
        for bias in (-50.0, 0.0, 50.0):
            with torch.no_grad():
                model.head[-1].bias.fill_(bias)
                out = model(x)
            self.assertGreaterEqual(float(out[:, 0].min()), -1.0)
            self.assertLessEqual(float(out[:, 0].max()), 1.0)
            self.assertGreaterEqual(float(out[:, 1].min()), 0.0)
            self.assertLessEqual(float(out[:, 1].max()), 1.0)
        self.assertAlmostEqual(float(out[0, 0]), 1.0, places=4)
        self.assertAlmostEqual(float(out[0, 1]), 1.0, places=4)

    def test_batchnorm_tracks_fast_enough_for_short_fine_tuning(self):
        """torchvision 既定の momentum=0.01 のままだと、数百イテレーションの学習では
        推論用の統計が追いつかず、検証・ONNX で直進しか出さないモデルになる。"""
        for pretrained_layout in (False,):
            model = DriveRegressionModel(pretrained=pretrained_layout)
            bns = [m for m in model.modules() if isinstance(m, torch.nn.BatchNorm2d)]
            self.assertTrue(bns)
            self.assertTrue(all(m.momentum == 0.1 for m in bns))

    def test_handles_non_square_resolution(self):
        model = DriveRegressionModel(pretrained=False)
        model.eval()
        x = torch.rand(1, 6, 100, 150)
        with torch.no_grad():
            out = model(x)
        self.assertEqual(out.shape, (1, 2))

    def test_gradients_flow_for_training(self):
        model = DriveRegressionModel(pretrained=False)
        model.train()
        x = torch.rand(2, 6, 64, 64)
        out = model(x)
        loss = out.mean()
        loss.backward()
        grad_norms = [p.grad.abs().sum().item() for p in model.parameters()
                     if p.grad is not None]
        self.assertTrue(grad_norms, "勾配が1つも流れていない")
        self.assertGreater(sum(grad_norms), 0.0)


class TestRightAndLeft(unittest.TestCase):
    def test_head_gets_right_then_left_features_from_the_shared_encoder(self):
        model = DriveRegressionModel(pretrained=False).eval()
        x = torch.rand(2, 6, 64, 96)
        seen = {}

        def grab(_m, inp, _out):
            seen["feat"] = inp[0]          # None を返す（値を返すとヘッドの出力が置き換わる）

        model.head.register_forward_hook(grab)
        with torch.no_grad():
            out = model(x)
            right = model.pool(model.encoder(x[:, :3])).flatten(1)
            left = model.pool(model.encoder(x[:, 3:])).flatten(1)
        self.assertEqual(out.shape, (2, 2))
        c = right.shape[1]
        self.assertEqual(seen["feat"].shape, (2, 2 * c))
        self.assertTrue(torch.allclose(seen["feat"][:, :c], right, atol=1e-5))
        self.assertTrue(torch.allclose(seen["feat"][:, c:], left, atol=1e-5))


if __name__ == "__main__":
    unittest.main()
