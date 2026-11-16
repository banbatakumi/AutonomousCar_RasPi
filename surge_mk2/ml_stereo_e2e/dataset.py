"""ml_stereo_e2e/dataset.py — `extract.py` が作った標本を読む PyTorch Dataset。

画像・車両状態・手本は `ml_e2e.dataset.FusionDataset` と同じ作り（継承）。違いは2番目の入力が点群ではなく
**視差の帯**（`disp`、`(2, Hd, Wd)`）であること。照合できなかった標本（`disp_ok=False`）は使わない。

## 拡張

- **向きをずらす**（`yaw_max_deg`）: 画像は `ml_e2e` と同じく窓を横へずらし、視差の帯は同じ角度だけ列をずらす
  （`shift_disp`。車が左へ ψ 向くと物は画像の右へ動く。平行化画像は透視投影なので、1 列あたりの角度は
  `cols_per_rad` の逆数でほぼ一定とみなす）。ずらして空いた列は「照合できなかった」（0）にする
- **左右反転はしない。** 視差の帯は右カメラ基準なので、鏡像にしても左右を入れ替えた組の視差にはならない
- 画像の外乱・明るさは `ml_e2e` と同じ
"""

from __future__ import annotations

import json
import math
import sys
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import torch

_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(_ROOT))

from ml_e2e.dataset import FusionDataset, InputSpec, Record, sample_weights, split_records  # noqa: E402

__all__ = ["StereoSpec", "load_records", "split_records", "StereoDataset", "stereo_sample_weights", "shift_disp"]


@dataclass(frozen=True)
class StereoSpec(InputSpec):
    """`InputSpec`（画像・経路点・正規化）に視差の帯の大きさを足したもの。点群の項目は使わない。"""

    disp_hw: tuple[int, int] = (12, 68)


def load_records(data_dir: Path) -> tuple[list[Record], dict]:
    """`data_dir` の全記録を読む（視差のある記録だけ）。"""
    meta = json.loads((data_dir / "dataset.json").read_text())
    records = []
    for p in sorted(data_dir.glob("*.npz")):
        d = dict(np.load(p))
        if len(d["t"]) == 0 or "disp" not in d or not d["disp_ok"].any():
            continue
        info = meta["records"].get(p.stem, {})
        records.append(Record(name=p.stem, data=d, rows=np.zeros((0, 2, 0), np.float32),
                              maps=tuple(info.get("maps", []))))
    return records, meta


def stereo_sample_weights(records: list[Record], max_steer: float, *, alpha: float = 0.5) -> np.ndarray:
    """`ml_e2e.dataset.sample_weights` を、使う標本（`disp_ok`）だけに絞ったもの（`StereoDataset.index` と同じ並び）。"""
    w = sample_weights(records, max_steer, alpha=alpha)
    ok = np.concatenate([r.data["disp_ok"] for r in records])
    return w[ok]


def shift_disp(arr: np.ndarray, shift_cols: float) -> np.ndarray:
    """視差の帯 `(2, H, W)` を列方向へ `shift_cols` だけずらす（正 = 右へ。1次補間、はみ出しは 0）。"""
    if not shift_cols:
        return arr
    _, _, w = arr.shape
    src = np.arange(w, dtype=np.float64) - shift_cols        # 新しい列 c は元の列 c − shift
    i0 = np.floor(src).astype(int)
    f = (src - i0).astype(np.float32)
    out = np.zeros_like(arr)
    for di, wt in ((0, 1.0 - f), (1, f)):
        j = i0 + di
        ok = (j >= 0) & (j < w)
        out[:, :, ok] += arr[:, :, j[ok]] * wt[ok]
    return out


class StereoDataset(FusionDataset):
    """`__getitem__` は入力と手本の dict（テンソル）。`image` (6,h,w)・`disp` (2,Hd,Wd)・`state` (3,)、
    手本は `FusionDataset` と同じ（`path`・`path_v`・`pt_ok`・`v_ok`）。"""

    def __init__(self, records: list[Record], data_dir: Path, spec: StereoSpec, *, cols_per_rad: float = 0.0,
                 augment: bool = False, yaw_max_deg: float = 0.0, fx_px: float = 0.0,
                 body_mask: np.ndarray | None = None, photo_strength: float = 0.0, load_images: bool = True,
                 fx_px_left: float = 0.0, body_mask_left: np.ndarray | None = None) -> None:
        super().__init__(records, data_dir, spec, augment=augment, flip=False, yaw_max_deg=yaw_max_deg,
                         margin_deg=int(math.ceil(yaw_max_deg)), fx_px=fx_px, body_mask=body_mask,
                         photo_strength=photo_strength, load_images=load_images, fx_px_left=fx_px_left,
                         body_mask_left=body_mask_left)
        if augment and yaw_max_deg > 0 and cols_per_rad <= 0:
            raise ValueError("向きの拡張には視差の帯の cols_per_rad が要る（dataset.json の記録ごとの値）")
        self.cols_per_rad = cols_per_rad
        self.index = [(ri, i) for ri, i in self.index if records[ri].data["disp_ok"][i]]

    def _disp(self, rec: Record, i: int, psi: float) -> np.ndarray:
        arr = rec.data["disp"][i].astype(np.float32)
        if psi:
            # 車が左へ ψ 向くと、物は画像の右（列が増える方）へ動く
            arr = shift_disp(arr, psi * self.cols_per_rad)
        return arr

    def __getitem__(self, n: int) -> dict[str, torch.Tensor]:
        ri, i = self.index[n]
        rec = self.records[ri]
        psi_deg, _ = self._draw_aug()
        psi = math.radians(psi_deg)
        return {
            "image": torch.from_numpy(self._image(rec, i, psi, False)),
            "disp": torch.from_numpy(np.ascontiguousarray(self._disp(rec, i, psi))),
            **self._state_and_target(rec, i, psi, False),
        }
