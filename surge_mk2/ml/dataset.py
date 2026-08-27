"""ml/dataset.py — (フレーム, マスク) のペアを読む PyTorch Dataset。

`ml/annotate.py` が書く `<frame_stem>_mask.png`（0/255 の2値）を前提にする。
**ラベル付けが終わっていないフレームは黙って除外する**（全部揃うまで学習を
待たなくてよいようにするため）。

対応関係は `manifest.csv`（`ml/extract_frames.py` が書く）があればそれを使うが、
**無くても動く。** GUI の「ログ」タブから ZIP でフレームを抽出した場合は
`manifest.csv` が付いてこない（ブラウザ側は画像を書き出すだけで由来の記録は
残さない）ため、その場合は `frames_dir` 直下を直接走査する
（`ml/annotate.py` 自身も `*.jpg` を glob するだけなので、対応の取り方を
合わせてある）。
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
    """ラベル付け済み（対応する `_mask.png` がある）フレームだけを集める。

    `manifest.csv` があればそれに載っているフレームだけを見る（`source_mcap`/
    `cam` 等の由来情報を将来使う余地を残すため）。**無ければ `frames_dir` 直下の
    `*.jpg` を直接走査する**——`manifest.csv` はあくまで補助情報であって、
    ペアリングの必須条件にはしない。
    """
    masks_dir = masks_dir or frames_dir
    manifest = frames_dir / "manifest.csv"
    pairs: list[tuple[Path, Path]] = []

    if manifest.exists():
        with open(manifest) as f:
            for row in csv.DictReader(f):
                frame = frames_dir / row["file"]
                mask = masks_dir / f"{frame.stem}_mask.png"
                if frame.exists() and mask.exists():
                    pairs.append((frame, mask))
        return pairs

    for frame in sorted(frames_dir.glob("*.jpg")):
        mask = masks_dir / f"{frame.stem}_mask.png"
        if mask.exists():
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
