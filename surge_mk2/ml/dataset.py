"""ml/dataset.py — (フレーム, マスク) のペアを読む PyTorch Dataset。

`ml/annotate.py` が書く `<frame_stem>_mask.png`（0/255 の2値）を前提にする。
`ml/extract_frames.py` の `manifest.csv` を軸に対応関係を作る——**ラベル付けが
終わっていないフレームは黙って除外する**（全部揃うまで学習を待たなくてよい
ようにするため。撮り足すたびに `manifest.csv` は増えるが、学習側は増えた分だけ
拾えばよい）。
"""

from __future__ import annotations

import csv
from pathlib import Path

import numpy as np
import torch
from PIL import Image
from torch.utils.data import Dataset

__all__ = ["DrivableDataset", "list_labeled_pairs"]


def list_labeled_pairs(frames_dir: Path,
                       masks_dir: Path | None = None) -> list[tuple[Path, Path]]:
    """`manifest.csv` に載っているフレームのうち、対応するマスクがあるものだけ集める。"""
    masks_dir = masks_dir or frames_dir
    manifest = frames_dir / "manifest.csv"
    pairs: list[tuple[Path, Path]] = []
    if not manifest.exists():
        return pairs
    with open(manifest) as f:
        for row in csv.DictReader(f):
            frame = frames_dir / row["file"]
            mask = masks_dir / f"{frame.stem}_mask.png"
            if frame.exists() and mask.exists():
                pairs.append((frame, mask))
    return pairs


class DrivableDataset(Dataset):
    """`__getitem__` は `(image[3,H,W] float32 0-1, mask[1,H,W] float32 0/1)`。

    **正規化は 0-1（mean=0, std=255 相当）で固定。** `ml/export_onnx.py` の
    `model.json` に書く前処理定数、`raspi/nodes/cam_perception_node.py` の
    `SegmentationModel` の既定値と揃えてある——ここだけ違う値にすると
    学習と推論で入力の意味がズレる（train/inference skew）。
    """

    def __init__(self, pairs: list[tuple[Path, Path]], *,
                size: tuple[int, int] = (224, 224), augment: bool = False) -> None:
        self.pairs = pairs
        self.size = size            # (width, height)
        self.augment = augment

    def __len__(self) -> int:
        return len(self.pairs)

    def __getitem__(self, idx: int) -> tuple[torch.Tensor, torch.Tensor]:
        frame_path, mask_path = self.pairs[idx]
        w, h = self.size
        img = Image.open(frame_path).convert("RGB").resize((w, h), Image.BILINEAR)
        msk = Image.open(mask_path).convert("L").resize((w, h), Image.NEAREST)

        img_arr = np.asarray(img, dtype=np.float32) / 255.0
        msk_arr = (np.asarray(msk, dtype=np.float32) > 127).astype(np.float32)

        if self.augment and np.random.rand() < 0.5:
            # 左右反転だけ。上下反転は「地面は常に下」という前提を壊すのでやらない
            img_arr = np.ascontiguousarray(img_arr[:, ::-1, :])
            msk_arr = np.ascontiguousarray(msk_arr[:, ::-1])

        img_t = torch.from_numpy(img_arr).permute(2, 0, 1)
        msk_t = torch.from_numpy(msk_arr).unsqueeze(0)
        return img_t, msk_t
