"""ml/model.py — 走行可否セグメンテーションの軽量モデル。

MobileNetV3-Small（torchvision）をエンコーダにした最小限のデコーダ。
**Pi 5 の CPU でリアルタイムに動かすことを最優先**し、境界の精細さより
軽さを取っている（走行可否の2値判定に画素単位の精緻さはそこまで要らない
想定——実データで精度不足が分かれば、デコーダの層を増やすかバックボーンを
差し替える）。

入力: `(N, 3, H, W)` float32（0〜1に正規化済み・`ml/dataset.py` と同じ前提）
出力: `(N, 1, H, W)` float32（走行可能の確率、0〜1。1=走行可能）
"""

from __future__ import annotations

import torch
import torch.nn as nn
import torch.nn.functional as F
import torchvision

__all__ = ["DrivableSegModel"]


class _DecoderBlock(nn.Module):
    """2倍アップサンプル＋畳み込み。**転置畳み込みではなく補間＋通常畳み込み**にして
    あるのは、ONNX へのエクスポートで挙動が安定しているため（`export_onnx.py` の
    往復テストで確認済み）。"""

    def __init__(self, in_ch: int, out_ch: int) -> None:
        super().__init__()
        self.conv = nn.Conv2d(in_ch, out_ch, kernel_size=3, padding=1)
        self.bn = nn.BatchNorm2d(out_ch)
        self.act = nn.ReLU(inplace=True)

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        x = F.interpolate(x, scale_factor=2.0, mode="bilinear", align_corners=False)
        return self.act(self.bn(self.conv(x)))


class DrivableSegModel(nn.Module):
    """走行可能／不可能の2値セグメンテーション。

    :param pretrained: ImageNet 事前学習重みを使うか。**オフライン環境や
        テストでは `False` にする**（重みのダウンロードが要らなくなる）
    :param decoder_channels: デコーダ各段のチャネル数。段数がそのまま
        アップサンプル回数になる（エンコーダの縮小率と合わせて調整すること）
    """

    def __init__(self, *, pretrained: bool = True,
                decoder_channels: tuple[int, ...] = (128, 64, 32, 16)) -> None:
        super().__init__()
        weights = (torchvision.models.MobileNet_V3_Small_Weights.DEFAULT
                  if pretrained else None)
        backbone = torchvision.models.mobilenet_v3_small(weights=weights)
        self.encoder = backbone.features

        # **エンコーダの出力チャネル数はハードコードしない。** torchvision の
        # バージョンで内部構成が変わりうるので、実際に1回forwardして確かめる
        with torch.no_grad():
            probe = self.encoder(torch.zeros(1, 3, 64, 64))
        enc_ch = probe.shape[1]

        blocks = []
        ch = enc_ch
        for out_ch in decoder_channels:
            blocks.append(_DecoderBlock(ch, out_ch))
            ch = out_ch
        self.decoder = nn.Sequential(*blocks)
        self.head = nn.Conv2d(ch, 1, kernel_size=1)

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        h, w = x.shape[-2:]
        feat = self.encoder(x)
        dec = self.decoder(feat)
        out = self.head(dec)
        # デコーダの累積アップサンプルは入力解像度にちょうど戻らないことがある
        # （エンコーダの縮小率とデコーダの段数がぴったり合わない場合）ので、
        # 最後に必ず入力サイズへ合わせる
        out = F.interpolate(out, size=(h, w), mode="bilinear", align_corners=False)
        return torch.sigmoid(out)
