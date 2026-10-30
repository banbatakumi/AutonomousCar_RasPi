"""ml_cam_e2e/dataset.py — (画像, 正規化した操舵・速度) のペアを読む PyTorch Dataset。

どの行を学習に使うか・ラベルをどう作るかは `samples.py`（torch を知らない
純粋関数）が決める。ここは画像を読んでテンソルにするだけ。

**前処理は `raspi/core/cam_e2e_preproc.py` を通す。** 実車の推論
（`raspi/nodes/cam_e2e_node.py`）と同じ関数なので、学習した絵と実車で
モデルに入る絵が食い違わない。

向きをずらす拡張（`yaw_max_deg > 0`）は `yaw_aug.py`、ブレ・ノイズ・色味・JPEG の外乱
（`photo_strength > 0`）は `photo_aug.py` 参照。拡張しない絵（検証・評価）は推論と同じ
中央の窓（`crop_window`）になる。
"""

from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))  # repo root

import numpy as np  # noqa: E402
import torch  # noqa: E402
from torch.utils.data import Dataset  # noqa: E402

from raspi.core.cam_e2e_preproc import crop_margin_px, crop_window, normalize, resize_area  # noqa: E402
import photo_aug  # noqa: E402
from samples import Sample, load_rgb, speed_norm, steer_norm  # noqa: E402
from yaw_aug import sample_psi, yawed_window  # noqa: E402

__all__ = ["load_rgb", "DriveDataset"]

#: 明るさ（加算）とコントラスト（乗算）の揺らぎ幅。照明や露出の違いに
#: 引きずられないようにする程度の弱い値
_BRIGHTNESS = 25.0
_CONTRAST = 0.2


class DriveDataset(Dataset):
    """`__getitem__` は `(image[3,H,W] float32, target[2] float32)`。

    `target` は `(steer_norm -1..1, speed_norm 0..1)`。正規化の基準
    （`max_steer`・`speed_ref`）は `export_onnx.py` がモデル同梱の JSON に書き、
    実車がその値で物理量へ戻す——ここだけ違う基準で正規化すると学習と推論でズレる。
    """

    def __init__(self, samples: list[Sample], *, size: tuple[int, int] = (224, 128),
                 max_steer: float, speed_ref: float, augment: bool = False,
                 flip: bool = True, mean: float = 0.0, std: float = 255.0,
                 crop_x: float = 0.0, yaw_max_deg: float = 0.0, yaw_k: float = 0.0,
                 fx_px: float = 0.0, body_mask: np.ndarray | None = None,
                 photo_strength: float = 0.0) -> None:
        self.samples = samples
        self.size = size            # (width, height)
        self.max_steer = max_steer
        self.speed_ref = speed_ref
        self.augment = augment
        self.flip = flip
        self.mean = mean
        self.std = std
        #: 左右の切り落とし（`cam_e2e_preproc.crop_window`）。推論と同じ値
        self.crop_x = crop_x
        #: 向きをずらす拡張の最大角 [deg]。0 で拡張しない（`yaw_aug.py`）
        self.yaw_max_deg = yaw_max_deg
        #: 戻す舵の係数: δ' = δ − k·ψ
        self.yaw_k = yaw_k
        self.fx_px = fx_px
        self.body_mask = body_mask
        #: ブレ・ノイズ・色味・JPEG の外乱の強さの倍率。0 で無効（`photo_aug.py`）
        self.photo_strength = photo_strength
        if yaw_max_deg > 0 and (crop_x <= 0 or fx_px <= 0):
            raise ValueError("向きの拡張には crop_x と fx_px が要る")

    def __len__(self) -> int:
        return len(self.samples)

    def __getitem__(self, idx: int) -> tuple[torch.Tensor, torch.Tensor]:
        sample = self.samples[idx]
        w, h = self.size
        rgb = load_rgb(sample.path)
        steer = steer_norm(sample, self.max_steer)
        speed = speed_norm(sample, self.speed_ref)
        if self.augment and self.photo_strength > 0:
            # 元の解像度で掛ける（縮小の後だとノイズが強すぎ、ブレが細かすぎる）。
            # 乱数は np.random から種を取り、学習の seed で再現できるようにする
            rng = np.random.default_rng(np.random.randint(0, 2**31))
            rgb = photo_aug.apply(rgb, rng, self.photo_strength)
        if self.augment and self.yaw_max_deg > 0:
            psi = sample_psi(self.yaw_max_deg)
            margin = crop_margin_px(rgb.shape[1], self.crop_x)
            rgb = yawed_window(rgb, margin, psi, self.fx_px, self.body_mask)
            # 車が左（ψ>0）を向いていたら右へ戻す。正規化値で補正して出力範囲に収める
            steer = max(-1.0, min(1.0, steer - self.yaw_k * psi / self.max_steer))
        else:
            rgb = crop_window(rgb, self.crop_x)
        small = resize_area(rgb, w, h)

        if self.augment:
            if self.flip and np.random.rand() < 0.5:
                # **左右反転と同時に操舵の符号を反転する。** 画像だけ反転すると
                # 「右カーブの絵に左へ切れという教師信号」になり学習が矛盾する
                # （DAVE-2/DonkeyCar系の定番手法）。速度は左右で変わらない
                small = small[:, ::-1, :]
                steer = -steer
            gain = 1.0 + np.random.uniform(-_CONTRAST, _CONTRAST)
            bias = np.random.uniform(-_BRIGHTNESS, _BRIGHTNESS)
            small = np.clip(small.astype(np.float32) * gain + bias, 0.0, 255.0)

        img_t = torch.from_numpy(normalize(small, self.mean, self.std)[0])
        return img_t, torch.tensor([steer, speed], dtype=torch.float32)
