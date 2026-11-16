"""ml_stereo_e2e/model.py — 左右の画像・視差の帯・車両状態から、前方の経路点を出すモデル。

```
image (6,h,w)   ── MobileNetV3-Small（左右で共有）── 平均プール ── 全結合 ─┐
disp  (2,Hd,Wd) ── 2D-CNN ────────────────────────────── 全結合 ─┼─ 連結 → 全結合×2 ── 経路点 K×(x, y, v, b)
state (3,)      ── 全結合 ─────────────────────────────────────┘
```

`ml_e2e/model.py`（E2E 融合）の点群の枝を、視差の帯の 2D-CNN に置き換えたもの。画像の枝・入力を伏せる仕組み（`avail`）・
経路点の不確かさ `b` の出し方は同じ（理由はそちらの docstring）。入出力の並びと正規化は `raspi/core/stereo_e2e_preproc.py`。

視差の帯は小さい（既定 12×68）ので、ストライド 2 の畳み込みを縦横に 2 回・横だけ 1 回掛けて 3×9 まで縮める。
"""

from __future__ import annotations

import torch
import torch.nn as nn
import torchvision

from ml_e2e.model import _BN_MOMENTUM, B_MAX, B_MIN

__all__ = ["StereoE2EModel"]


class StereoE2EModel(nn.Module):
    """:param disp_hw: 視差の帯の (高さ, 幅)
    :param k: 経路点の数
    :param xy_scale: 経路点の正規化の基準 [m]
    :param pretrained: 画像エンコーダに ImageNet の重みを使うか（テスト・オフラインでは False）
    """

    def __init__(self, *, disp_hw: tuple[int, int], k: int, xy_scale: float, pretrained: bool = True,
                 feat: int = 128, hidden: int = 256) -> None:
        super().__init__()
        self.k = k
        self.xy_scale = float(xy_scale)

        weights = torchvision.models.MobileNet_V3_Small_Weights.DEFAULT if pretrained else None
        self.encoder = torchvision.models.mobilenet_v3_small(weights=weights).features
        for m in self.encoder.modules():
            if isinstance(m, nn.BatchNorm2d):
                m.momentum = _BN_MOMENTUM
        with torch.no_grad():
            enc_ch = self.encoder(torch.zeros(1, 3, 64, 64)).shape[1]
        self.img_fc = nn.Sequential(nn.AdaptiveAvgPool2d(1), nn.Flatten(),
                                    nn.Linear(enc_ch * 2, feat), nn.ReLU(inplace=True))

        self.disp_cnn = nn.Sequential(
            nn.Conv2d(2, 32, 3, padding=1), nn.ReLU(inplace=True),
            nn.Conv2d(32, 64, 3, stride=2, padding=1), nn.ReLU(inplace=True),
            nn.Conv2d(64, 64, 3, stride=2, padding=1), nn.ReLU(inplace=True),
            nn.Conv2d(64, 64, 3, stride=(1, 2), padding=1), nn.ReLU(inplace=True),
            nn.Flatten(),
        )
        with torch.no_grad():
            flat = self.disp_cnn(torch.zeros(1, 2, int(disp_hw[0]), int(disp_hw[1]))).shape[1]
        self.disp_fc = nn.Sequential(nn.Linear(flat, feat), nn.ReLU(inplace=True))

        self.state_fc = nn.Sequential(nn.Linear(3, 32), nn.ReLU(inplace=True))

        self.no_image = nn.Parameter(torch.zeros(feat))
        self.no_disp = nn.Parameter(torch.zeros(feat))
        self.no_state = nn.Parameter(torch.zeros(32))

        self.trunk = nn.Sequential(
            nn.Linear(feat * 2 + 32, hidden), nn.ReLU(inplace=True),
            nn.Linear(hidden, hidden), nn.ReLU(inplace=True),
        )
        self.path_head = nn.Linear(hidden, 4 * k)
        with torch.no_grad():
            self.path_head.bias[3 * k:].fill_(-2.75)          # `b` の初期値を約 0.3m に（`ml_e2e/model.py`）

    def _image_feat(self, image: torch.Tensor) -> torch.Tensor:
        n = image.shape[0]
        pooled = self.img_fc[:2](self.encoder(torch.cat([image[:, :3], image[:, 3:]], dim=0)))
        return self.img_fc[2:](torch.cat([pooled[:n], pooled[n:]], dim=1))

    def forward(self, image: torch.Tensor, disp: torch.Tensor, state: torch.Tensor,
                avail: torch.Tensor) -> torch.Tensor:
        """出力は `(N, 4K)`（`stereo_e2e_preproc.decode_output` の並び）。"""
        a_img, a_disp, a_st = avail[:, 0:1], avail[:, 1:2], avail[:, 2:3]
        f_img = self._image_feat(image) * a_img + self.no_image * (1.0 - a_img)
        f_disp = self.disp_fc(self.disp_cnn(disp)) * a_disp + self.no_disp * (1.0 - a_disp)
        f_st = self.state_fc(state) * a_st + self.no_state * (1.0 - a_st)
        z = self.trunk(torch.cat([f_img, f_disp, f_st], dim=1))

        p = self.path_head(z)
        k = self.k
        v = torch.sigmoid(p[:, 2 * k:3 * k])
        b = B_MIN + (B_MAX - B_MIN) * torch.sigmoid(p[:, 3 * k:])
        return torch.cat([p[:, :2 * k], v, b], dim=1)
