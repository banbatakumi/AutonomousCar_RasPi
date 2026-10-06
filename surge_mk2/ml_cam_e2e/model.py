"""ml_cam_e2e/model.py — 画像から操舵と速度（正規化値）を直接回帰する軽量モデル。

`ml_cam/model.py` の `DrivableSegModel` と同じ MobileNetV3-Small エンコーダを
再利用し、デコーダ（セグメンテーション用のアップサンプル列）の代わりに
グローバル平均プーリング→全結合の回帰ヘッドを載せる。**幾何変換（IPM）を
一切経由しない**——これがこのモデルの存在理由そのもの（低いカメラ高さで
IPMの深度誤差が拡大する問題を、画像から直接操作を学習することで迂回する）。

入力: `(N, 3, H, W)` float32（0〜1に正規化済み・`ml_cam_e2e/dataset.py` と同じ前提）
出力: `(N, 2)` float32
    - `[:, 0]` 操舵の正規化値（`tanh` で -1〜1。1 = `max_steer` いっぱい左）
    - `[:, 1]` 速度の正規化値（`sigmoid` で 0〜1。1 = `speed_ref`。前進のみ）

並びは `raspi/nodes/cam_e2e_node.py` の `MODEL_OUTPUTS` と揃える。
"""

from __future__ import annotations

import torch
import torch.nn as nn
import torchvision

__all__ = ["DriveRegressionModel"]

_BN_MOMENTUM = 0.1


class DriveRegressionModel(nn.Module):
    """操舵（正規化 -1..1）と速度（正規化 0..1）の回帰。

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
        # **BatchNorm の移動平均を速く追従させる。** torchvision の MobileNetV3 は
        # momentum=0.01（ImageNet の長い学習向け）で、数百イテレーションの
        # 微調整では推論用の統計が重みの変化に追いつかない——学習中の損失は
        # 下がるのに、検証・ONNX（どちらも移動平均を使う）では直進しか出さない
        # モデルになる（2026-10-06、合成データで確認。左右を見分けるだけの
        # 課題で検証誤差が 17° のまま動かなかった）。PyTorch 既定の 0.1 に戻す
        for m in self.encoder.modules():
            if isinstance(m, nn.BatchNorm2d):
                m.momentum = _BN_MOMENTUM

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
            nn.Linear(hidden_dim, 2),
        )

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        feat = self.encoder(x)
        pooled = self.pool(feat)
        out = self.head(pooled)
        return torch.cat([torch.tanh(out[:, :1]), torch.sigmoid(out[:, 1:])], dim=1)
