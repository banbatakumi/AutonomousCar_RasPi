"""ml_lidar/policy.py — LiDARスキャンの空間相関を活かすカスタム特徴抽出器。

`ml_lidar/train_rl.py`（学習、`--features-extractor cnn`）と`export_onnx_rl.py`
（エクスポート、`PPO.load()`が`policy_kwargs`からクラス参照を復元する際に
importできる必要がある）の両方から参照する。**このモジュールを削除・改名すると
既存の`.zip`チェックポイントが読めなくなる**（SB3は`features_extractor_class`を
モジュールパス込みで保存するため）。

## MLPだけでなくCNNを試す理由

`SCAN_DIM=361`点の距離配列は角度順に並んでおり、隣接ビーム間に強い空間相関を持つ
（壁の角・隙間・障害物の輪郭は連続する複数ビームにまたがって現れる）。従来の
`MlpPolicy`は観測`OBS_DIM`次元をそのまま最初の全結合層に投げ込むため、この空間構造を
明示的には利用できない。1D-CNNを前段に置くことで、局所的な幾何パターンを少ない
パラメータで抽出し、ランダムコースへの汎化を助ける狙い（2026-09-02追加）。
"""

from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))  # surge_mk2/

import torch  # noqa: E402
import torch.nn as nn  # noqa: E402
from gymnasium import spaces  # noqa: E402
from stable_baselines3.common.torch_layers import BaseFeaturesExtractor  # noqa: E402

from sim.gym_env import SCAN_DIM  # noqa: E402

__all__ = ["ScanCNNExtractor"]


class ScanCNNExtractor(BaseFeaturesExtractor):
    """観測を`[scan(SCAN_DIM点), extra(残り)]`に分割し、scan部分だけ1D-CNNで
    圧縮してからextra部分（速度・ステア角）と結合し、最終的に`features_dim`次元へ
    まとめる。`extra`の次元数は`observation_space`から自動で決まる——
    `sim/gym_env.py`の`OBS_DIM`が将来変わってもこのクラス自体は変更不要。
    """

    def __init__(self, observation_space: spaces.Box, features_dim: int = 256,
                scan_dim: int = SCAN_DIM) -> None:
        super().__init__(observation_space, features_dim)
        self.scan_dim = scan_dim
        extra_dim = observation_space.shape[0] - scan_dim
        if extra_dim < 0:
            raise ValueError(f"observation_space次元({observation_space.shape[0]})が"
                             f"scan_dim({scan_dim})より小さい")

        self.conv = nn.Sequential(
            nn.Conv1d(1, 16, kernel_size=5, stride=2, padding=2), nn.ReLU(),
            nn.Conv1d(16, 32, kernel_size=5, stride=2, padding=2), nn.ReLU(),
            nn.Conv1d(32, 32, kernel_size=3, stride=2, padding=1), nn.ReLU(),
            nn.Flatten(),
        )
        with torch.no_grad():
            conv_out_dim = self.conv(torch.zeros(1, 1, scan_dim)).shape[1]
        self.linear = nn.Sequential(
            nn.Linear(conv_out_dim + extra_dim, features_dim), nn.ReLU())

    def forward(self, observations: torch.Tensor) -> torch.Tensor:
        scan = observations[:, :self.scan_dim].unsqueeze(1)   # (B, 1, scan_dim)
        extra = observations[:, self.scan_dim:]                # (B, extra_dim)
        feat = self.conv(scan)
        return self.linear(torch.cat([feat, extra], dim=1))
