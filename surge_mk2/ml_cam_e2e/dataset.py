"""ml_cam_e2e/dataset.py — (画像, 正規化した操舵角) のペアを読む PyTorch Dataset。

`ml_cam_e2e/extract_pairs.py` が書く `manifest.csv`（file, source_mcap, cam,
t_capture_ns, target_steer, target_speed）を前提にする。`ml_cam/dataset.py`
と違い、マスクの手作業アノテーションが無いので**全行がそのまま学習に使える**
（除外判定は既に `extract_pairs.py` 側で済んでいる）。
"""

from __future__ import annotations

import csv
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))  # repo root

import numpy as np
import torch
from PIL import Image
from torch.utils.data import Dataset

from raspi.core.vehicle import Vehicle

__all__ = ["list_pairs", "DriveDataset"]


def list_pairs(frames_dir: Path) -> list[tuple[Path, float]]:
    """`manifest.csv` から `(画像パス, target_steer[rad])` の一覧を作る。

    画像が実際に存在する行だけを返す（`--min-interval-ms`/`--target-count` の
    間引きで捨てられた行は manifest に残らないので、通常はここで弾かれるのは
    手動で消したフレームくらい）。
    """
    manifest = frames_dir / "manifest.csv"
    pairs: list[tuple[Path, float]] = []
    if not manifest.exists():
        return pairs
    with open(manifest) as f:
        for row in csv.DictReader(f):
            frame = frames_dir / row["file"]
            if frame.exists():
                pairs.append((frame, float(row["target_steer"])))
    return pairs


class DriveDataset(Dataset):
    """`__getitem__` は `(image[3,H,W] float32 0-1, steer_norm[1] float32 -1..1)`。

    正規化は `target_steer / vehicle.max_steer`（`config/vehicle.toml`）。
    `ml_cam_e2e/export_onnx.py` の `model.json` に書く `max_steer` 契約と
    揃えてある——ここだけ違う基準で正規化すると学習と推論でズレる。
    """

    def __init__(self, pairs: list[tuple[Path, float]], *,
                size: tuple[int, int] = (224, 224), augment: bool = False,
                max_steer: float | None = None) -> None:
        self.pairs = pairs
        self.size = size            # (width, height)
        self.augment = augment
        self.max_steer = max_steer if max_steer is not None else Vehicle.load().max_steer

    def __len__(self) -> int:
        return len(self.pairs)

    def __getitem__(self, idx: int) -> tuple[torch.Tensor, torch.Tensor]:
        frame_path, target_steer = self.pairs[idx]
        w, h = self.size
        img = Image.open(frame_path).convert("RGB").resize((w, h), Image.BILINEAR)
        img_arr = np.asarray(img, dtype=np.float32) / 255.0

        steer_norm = max(-1.0, min(1.0, target_steer / self.max_steer)) \
            if self.max_steer > 0 else 0.0

        if self.augment and np.random.rand() < 0.5:
            # **左右反転と同時に操舵の符号を反転する。** 画像だけ反転すると
            # 「右カーブの絵に左へ切れという教師信号」になり学習が矛盾する
            # （`ml_cam/dataset.py` のマスク反転と同じ理由で、対応するラベルも
            # 一緒に反転させる必要がある。DAVE-2/DonkeyCar系の定番手法でもある）
            img_arr = np.ascontiguousarray(img_arr[:, ::-1, :])
            steer_norm = -steer_norm

        img_t = torch.from_numpy(img_arr).permute(2, 0, 1)
        steer_t = torch.tensor([steer_norm], dtype=torch.float32)
        return img_t, steer_t
