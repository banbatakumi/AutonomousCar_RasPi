"""ml_e2e/model.py — 画像・点群・車両状態から、舵・速度・経路点を出すモデル。

```
image (3,h,w) ── MobileNetV3-Small ── 平均プール ── 全結合 ─┐
lidar (H+1,n) ── 1D-CNN ─────────────── 全結合 ─┼─ 連結 → 全結合×2 ─┬─ 舵・速度
state (3,)    ── 全結合 ───────────────────────┘                  └─ 経路点 K×(x, y, v, b)
```

入出力の並びと正規化は `raspi/core/e2e_fusion_preproc.py` が正。

## 入力を伏せられるようにしてある（`avail`）

`avail = (画像, 点群, 車両状態)` の 1/0。0 の入力は特徴を捨て、代わりに「無い」ことを表す学習済みのベクトルを入れる。
学習で一定割合を伏せる（`train.py`）ことで、①楽な方の入力だけを見るモデルにならない ②実車で片方の入力が
途絶えても残りで経路を出せる。

## 経路点の不確かさ `b`

各点の位置の誤差をラプラス分布とみなし、その幅 `b` [m] も出す（損失は `train.py` の `path_nll`）。
分岐の手前や見通しの先のように手本が割れる所で大きくなる。実車は `b` が大きい経路を信用しない。
"""

from __future__ import annotations

import torch
import torch.nn as nn
import torchvision

__all__ = ["FusionModel"]

_BN_MOMENTUM = 0.1
#: `b`（不確かさ [m]）の出力範囲。下限は「位置を 1cm より細かく言い当てた」と主張させないため
B_MIN, B_MAX = 0.01, 5.0


class FusionModel(nn.Module):
    """:param lidar_ch: 点群の行数（`history + 1`）
    :param lidar_n: 点群の点数（`fov_deg + 1`）
    :param k: 経路点の数
    :param xy_scale: 経路点の正規化の基準 [m]（`b` を m で出すのに使う）
    :param pretrained: 画像エンコーダに ImageNet の重みを使うか（テスト・オフラインでは False）
    """

    def __init__(self, *, lidar_ch: int, lidar_n: int, k: int, xy_scale: float, pretrained: bool = True,
                 feat: int = 128, hidden: int = 256) -> None:
        super().__init__()
        self.k = k
        self.xy_scale = float(xy_scale)

        weights = torchvision.models.MobileNet_V3_Small_Weights.DEFAULT if pretrained else None
        self.encoder = torchvision.models.mobilenet_v3_small(weights=weights).features
        # 短い微調整では BatchNorm の移動平均が追いつかない（`ml_cam_e2e/model.py` と同じ対処）
        for m in self.encoder.modules():
            if isinstance(m, nn.BatchNorm2d):
                m.momentum = _BN_MOMENTUM
        with torch.no_grad():
            enc_ch = self.encoder(torch.zeros(1, 3, 64, 64)).shape[1]
        self.img_fc = nn.Sequential(nn.AdaptiveAvgPool2d(1), nn.Flatten(), nn.Linear(enc_ch, feat),
                                    nn.ReLU(inplace=True))

        self.lidar_cnn = nn.Sequential(
            nn.Conv1d(lidar_ch, 32, 7, stride=2, padding=3), nn.ReLU(inplace=True),
            nn.Conv1d(32, 64, 5, stride=2, padding=2), nn.ReLU(inplace=True),
            nn.Conv1d(64, 64, 3, stride=2, padding=1), nn.ReLU(inplace=True),
            nn.Conv1d(64, 64, 3, stride=2, padding=1), nn.ReLU(inplace=True),
            nn.Flatten(),
        )
        with torch.no_grad():
            flat = self.lidar_cnn(torch.zeros(1, lidar_ch, lidar_n)).shape[1]
        self.lidar_fc = nn.Sequential(nn.Linear(flat, feat), nn.ReLU(inplace=True))

        self.state_fc = nn.Sequential(nn.Linear(3, 32), nn.ReLU(inplace=True))

        # 入力が無いときに代わりに入れるベクトル
        self.no_image = nn.Parameter(torch.zeros(feat))
        self.no_lidar = nn.Parameter(torch.zeros(feat))
        self.no_state = nn.Parameter(torch.zeros(32))

        self.trunk = nn.Sequential(
            nn.Linear(feat * 2 + 32, hidden), nn.ReLU(inplace=True),
            nn.Linear(hidden, hidden), nn.ReLU(inplace=True),
        )
        self.ctrl_head = nn.Linear(hidden, 2)
        self.path_head = nn.Linear(hidden, 4 * k)
        # `b` の初期値を約 0.3m にする（既定の 0 だと範囲の真ん中 2.5m から始まり、最初の数百回は
        # 「全部わからない」が損失を下げる楽な解になる）
        with torch.no_grad():
            self.path_head.bias[3 * k:].fill_(-2.75)

    def forward(self, image: torch.Tensor, lidar: torch.Tensor, state: torch.Tensor,
                avail: torch.Tensor) -> torch.Tensor:
        """出力は `(N, 2 + 4K)`（`e2e_fusion_preproc.decode_output` の並び）。"""
        a_img, a_lid, a_st = avail[:, 0:1], avail[:, 1:2], avail[:, 2:3]
        f_img = self.img_fc(self.encoder(image)) * a_img + self.no_image * (1.0 - a_img)
        f_lid = self.lidar_fc(self.lidar_cnn(lidar)) * a_lid + self.no_lidar * (1.0 - a_lid)
        f_st = self.state_fc(state) * a_st + self.no_state * (1.0 - a_st)
        z = self.trunk(torch.cat([f_img, f_lid, f_st], dim=1))

        c = self.ctrl_head(z)
        p = self.path_head(z)
        k = self.k
        xy = p[:, :2 * k]
        v = torch.sigmoid(p[:, 2 * k:3 * k])
        b = B_MIN + (B_MAX - B_MIN) * torch.sigmoid(p[:, 3 * k:])
        return torch.cat([torch.tanh(c[:, :1]), torch.sigmoid(c[:, 1:]), xy, v, b], dim=1)
