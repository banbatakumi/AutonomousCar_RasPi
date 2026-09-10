"""`ml_lidar/features.py` のテスト。

主眼は「Conv1d層がSB3のデフォルトorthogonal初期化から漏れる」落とし穴
（`ML_LIDAR_V2_PROMPT.md`）を実際に踏んでいないことの回帰確認——初期化のたびに
Conv1d重みが本当に(半)直交行列になっているかを検証する。
"""

import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))  # surge_mk2/

import numpy as np  # noqa: E402
import torch as th  # noqa: E402
from gymnasium import spaces  # noqa: E402

from ml_lidar.features import Conv1dExtractor  # noqa: E402


class TestConv1dExtractor(unittest.TestCase):
    def test_forward_output_shape(self) -> None:
        n_scan = 181
        obs_space = spaces.Box(low=-1.0, high=1.0, shape=(n_scan + 2,), dtype=np.float32)
        ext = Conv1dExtractor(obs_space, n_scan=n_scan, features_dim=64)
        batch = th.zeros(4, n_scan + 2)
        out = ext(batch)
        self.assertEqual(tuple(out.shape), (4, 64))

    def test_conv1d_weights_are_orthogonal(self) -> None:
        """Conv1d層の重みが(半)直交であることを直接検証する。

        `nn.init.orthogonal_`は `rows = out_channels`、
        `cols = in_channels * kernel_size` として、
        `rows > cols` なら列が正規直交（`W.T @ W ≈ gain^2 * I`）、
        `rows < cols` なら行が正規直交（`W @ W.T ≈ gain^2 * I`）になる
        （PyTorch公式ドキュメント）。PyTorchの既定初期化（kaiming系）では
        この関係は成り立たないので、これが通れば明示的な直交初期化が
        効いていると確認できる。
        """
        n_scan = 181
        obs_space = spaces.Box(low=-1.0, high=1.0, shape=(n_scan + 2,), dtype=np.float32)
        ext = Conv1dExtractor(obs_space, n_scan=n_scan, features_dim=64)

        conv_layers = [m for m in ext.conv.modules() if isinstance(m, th.nn.Conv1d)]
        self.assertGreaterEqual(len(conv_layers), 1)

        gain_sq = 2.0  # sqrt(2) ** 2
        for layer in conv_layers:
            w = layer.weight.detach()
            out_ch, in_ch, k = w.shape
            flat = w.reshape(out_ch, in_ch * k)
            if out_ch >= in_ch * k:
                gram = flat.T @ flat  # (cols, cols)
                expected = gain_sq * th.eye(in_ch * k)
            else:
                gram = flat @ flat.T  # (rows, rows)
                expected = gain_sq * th.eye(out_ch)
            self.assertTrue(th.allclose(gram, expected, atol=1e-3),
                            f"Conv1d({in_ch}->{out_ch}, k={k}) の重みが直交になっていない")


if __name__ == "__main__":
    unittest.main()
