"""ml_lidar/features.py — SB3向け1D-CNN特徴抽出器。

LiDAR点群（角度方向に隣接する1次元系列）から局所的な壁・コーナー形状を畳み込みで
拾う。RL文献ではMLPが標準だが、模倣学習系（TinyLidarNet等）では1D-CNNが明確に
優位という調査結果（`ML_LIDAR_V2_PROMPT.md`）を踏まえ、1D-CNNから始める。

## Conv1d はSB3のデフォルトorthogonal初期化から漏れる

`stable_baselines3.common.policies.BasePolicy.init_weights`は
`isinstance(module, (nn.Linear, nn.Conv2d))`だけを対象にしており（SB3 2.9.0の
ソースで確認済み）、`nn.Conv1d`は対象外。放置すると特徴抽出器の畳み込み層だけ
PyTorchの既定初期化のまま学習が始まり、`ActorCriticPolicy._build()`が他の層に
揃えている初期スケール（`gain=sqrt(2)`のorthogonal初期化）から外れる。
ここでは`__init__`内で明示的に同じ初期化をConv1d層へ適用する。
"""

from __future__ import annotations

import numpy as np
import torch as th
from gymnasium import spaces
from stable_baselines3.common.torch_layers import BaseFeaturesExtractor
from torch import nn

__all__ = ["Conv1dExtractor"]

#: `ActorCriticPolicy._build()`が特徴抽出器全体に適用するgainと同じ値
_ORTHOGONAL_GAIN = float(np.sqrt(2))


class Conv1dExtractor(BaseFeaturesExtractor):
    """観測 `[LiDAR点群(n_scan), 速度, ステア角]` のうち、点群だけ1D-CNNに通してから
    速度・ステア角と連結する。

    :param n_scan: 観測ベクトル先頭のLiDAR点群の長さ。残りの要素（速度・ステア角の
        2個、`ml_lidar/env.py`の`_to_obs()`参照）は畳み込みを通さずそのまま連結する
    """

    def __init__(self, observation_space: spaces.Box, n_scan: int,
                features_dim: int = 128) -> None:
        super().__init__(observation_space, features_dim)
        obs_dim = int(observation_space.shape[0])
        if not (0 < n_scan <= obs_dim):
            raise ValueError(f"n_scan={n_scan} が観測次元{obs_dim}と整合しない")
        self.n_scan = n_scan
        n_extra = obs_dim - n_scan

        self.conv = nn.Sequential(
            nn.Conv1d(1, 16, kernel_size=5, stride=2, padding=2), nn.ReLU(),
            nn.Conv1d(16, 32, kernel_size=5, stride=2, padding=2), nn.ReLU(),
            nn.Conv1d(32, 32, kernel_size=3, stride=2, padding=1), nn.ReLU(),
            nn.Flatten(),
        )
        with th.no_grad():
            conv_out_dim = int(self.conv(th.zeros(1, 1, n_scan)).shape[1])

        self.head = nn.Sequential(
            nn.Linear(conv_out_dim + n_extra, features_dim), nn.ReLU(),
        )

        for m in self.conv.modules():
            if isinstance(m, nn.Conv1d):
                nn.init.orthogonal_(m.weight, gain=_ORTHOGONAL_GAIN)
                if m.bias is not None:
                    nn.init.zeros_(m.bias)

    def forward(self, observations: th.Tensor) -> th.Tensor:
        scan = observations[:, :self.n_scan].unsqueeze(1)  # (B, 1, n_scan)
        extra = observations[:, self.n_scan:]               # (B, n_extra)
        conv_out = self.conv(scan)
        return self.head(th.cat([conv_out, extra], dim=1))
