"""ml_cam_e2e/model.py — 画像から操舵角（正規化値）を直接回帰する軽量モデル。

`ml_cam/model.py` の `DrivableSegModel` と同じ MobileNetV3-Small エンコーダを
再利用し、デコーダ（セグメンテーション用のアップサンプル列）の代わりに
グローバル平均プーリング→全結合の回帰ヘッドを載せる。**幾何変換（IPM）を
一切経由しない**——これがこのモデルの存在理由そのもの（低いカメラ高さで
IPMの深度誤差が拡大する問題を、画像から直接ステアを学習することで迂回する）。

入力: `(N, 3, H, W)` float32（0〜1に正規化済み・`ml_cam_e2e/dataset.py` と同じ前提）
出力: `(N, 1)` float32（操舵角の正規化値、`tanh` で -1〜1。1 = `max_steer` いっぱい左）
"""

from __future__ import annotations

import torch
import torch.nn as nn
import torchvision

__all__ = ["DriveRegressionModel"]


class DriveRegressionModel(nn.Module):
    """操舵角（正規化 -1..1）の回帰。

    :param pretrained: ImageNet 事前学習重みを使うか。**オフライン環境や
        テストでは `False` にする**（`ml_cam/model.py` と同じ理由）
    :param hidden_dim: プーリング後の全結合の中間層幅
    """

    def __init__(self, *, pretrained: bool = True, hidden_dim: int = 64) -> None:
        super().__init__()
        weights = (torchvision.models.MobileNet_V3_Small_Weights.DEFAULT
                  if pretrained else None)
        backbone = torchvision.models.mobilenet_v3_small(weights=weights)
        self.encoder = backbone.features

        # `ml_cam/model.py` と同じ理由——エンコーダの出力チャネル数は
        # torchvision のバージョンで内部構成が変わりうるのでハードコードしない
        with torch.no_grad():
            probe = self.encoder(torch.zeros(1, 3, 64, 64))
        enc_ch = probe.shape[1]

        self.pool = nn.AdaptiveAvgPool2d(1)
        self.head = nn.Sequential(
            nn.Flatten(),
            nn.Linear(enc_ch, hidden_dim),
            nn.ReLU(inplace=True),
            nn.Linear(hidden_dim, 1),
        )

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        feat = self.encoder(x)
        pooled = self.pool(feat)
        out = self.head(pooled)
        return torch.tanh(out)
