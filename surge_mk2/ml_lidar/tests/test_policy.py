"""`ml_lidar/policy.py` の `ScanCNNExtractor` のテスト。"""

import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))  # surge_mk2/

import torch  # noqa: E402
from gymnasium import spaces  # noqa: E402

from ml_lidar.policy import ScanCNNExtractor  # noqa: E402
from sim.gym_env import SCAN_DIM  # noqa: E402


class TestScanCNNExtractor(unittest.TestCase):
    def _space(self, extra_dim: int) -> spaces.Box:
        n = SCAN_DIM + extra_dim
        return spaces.Box(0.0, 1.0, shape=(n,))

    def test_output_shape_matches_features_dim(self):
        space = self._space(2)
        ext = ScanCNNExtractor(space, features_dim=32)
        obs = torch.zeros(4, space.shape[0])
        out = ext(obs)
        self.assertEqual(out.shape, (4, 32))

    def test_gradient_flows_to_conv_and_linear_layers(self):
        space = self._space(2)
        ext = ScanCNNExtractor(space, features_dim=32)
        obs = torch.rand(2, space.shape[0], requires_grad=True)
        out = ext(obs)
        out.sum().backward()
        self.assertIsNotNone(obs.grad)
        self.assertTrue(torch.any(obs.grad != 0))
        for p in ext.parameters():
            self.assertIsNotNone(p.grad)

    def test_extra_dim_derived_from_observation_space(self):
        """`extra`（速度・ステア角）の次元数は`observation_space.shape[0] - scan_dim`
        から自動で決まる——`sim/gym_env.py`の`OBS_DIM`が変わってもこのクラス自体は
        変更不要という設計を境界値(0個・1個・2個)で確認する。"""
        for extra_dim in (0, 1, 2):
            with self.subTest(extra_dim=extra_dim):
                space = self._space(extra_dim)
                ext = ScanCNNExtractor(space, features_dim=16)
                obs = torch.zeros(1, space.shape[0])
                out = ext(obs)
                self.assertEqual(out.shape, (1, 16))

    def test_extra_dim_smaller_than_scan_dim_raises(self):
        space = spaces.Box(0.0, 1.0, shape=(SCAN_DIM - 1,))
        with self.assertRaises(ValueError):
            ScanCNNExtractor(space, features_dim=16)


if __name__ == "__main__":
    unittest.main()
