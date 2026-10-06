"""ml_cam_e2e/dataset.py — (画像, 正規化した操舵・速度) のペアを読む PyTorch Dataset。

どの行を学習に使うか・ラベルをどう作るかは `samples.py`（torch を知らない
純粋関数）が決める。ここは画像を読んでテンソルにするだけ。

**前処理は `raspi/core/cam_e2e_preproc.py` を通す。** 実車の推論
（`raspi/nodes/cam_e2e_node.py`）と同じ関数なので、学習した絵と実車で
モデルに入る絵が食い違わない（以前は PIL の BILINEAR とここだけ別実装だった）。
"""

from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))  # repo root

import numpy as np  # noqa: E402
import torch  # noqa: E402
from torch.utils.data import Dataset  # noqa: E402

from raspi.core.cam_e2e_preproc import normalize, resize_area  # noqa: E402
from samples import Sample, load_rgb, speed_norm, steer_norm  # noqa: E402

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
                 flip: bool = True, mean: float = 0.0, std: float = 255.0) -> None:
        self.samples = samples
        self.size = size            # (width, height)
        self.max_steer = max_steer
        self.speed_ref = speed_ref
        self.augment = augment
        self.flip = flip
        self.mean = mean
        self.std = std

    def __len__(self) -> int:
        return len(self.samples)

    def __getitem__(self, idx: int) -> tuple[torch.Tensor, torch.Tensor]:
        sample = self.samples[idx]
        w, h = self.size
        small = resize_area(load_rgb(sample.path), w, h)
        steer = steer_norm(sample, self.max_steer)
        speed = speed_norm(sample, self.speed_ref)

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
