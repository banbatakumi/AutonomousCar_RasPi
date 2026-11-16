"""ml_stereo_e2e/dataset.py — `extract.py` が作った標本を読む PyTorch Dataset。

画像・車両状態・手本は `ml_e2e.dataset.FusionDataset` と同じ作り（継承）。違いは2番目の入力が点群ではなく
**視差の帯**（`disp`、`(2, Hd, Wd)`）であること。照合できなかった標本（`disp_ok=False`）は使わない。

## 拡張

- **向きをずらす**（`yaw_max_deg`）: 画像は `ml_e2e` と同じく窓を横へずらし、視差の帯は同じ角度だけ透視投影どおりに
  列をずらしてから（`stereo_e2e_preproc.yaw_shift_disp`。車が左へ ψ 向くと物は画像の右へ動く。帯の焦点距離・主点は
  記録ごとの `calib_values`）、**両端 `crop_cols` 列を落とした中央の窓**を渡す（`crop_disp`。推論も同じ窓）。
  `crop_cols` はずらしの最大量より大きいので、ずらしで空いた列（0）は窓に入らない（実車に無い帯を手がかりにさせない）
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
from raspi.core.stereo_e2e_preproc import crop_disp, yaw_shift_disp  # noqa: E402

__all__ = ["StereoSpec", "load_records", "split_records", "StereoDataset", "stereo_sample_weights"]


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


class StereoDataset(FusionDataset):
    """`__getitem__` は入力と手本の dict（テンソル）。`image` (6,h,w)・`disp` (2,Hd,Wd)・`state` (3,)、
    手本は `FusionDataset` と同じ（`path`・`path_v`・`pt_ok`・`v_ok`）。"""

    def __init__(self, records: list[Record], data_dir: Path, spec: StereoSpec, *,
                 band_geom: dict[str, tuple[float, float]] | None = None, crop_cols: int = 0,
                 augment: bool = False, yaw_max_deg: float = 0.0, fx_px: float = 0.0,
                 body_mask: np.ndarray | None = None, photo_strength: float = 0.0, load_images: bool = True,
                 fx_px_left: float = 0.0, body_mask_left: np.ndarray | None = None) -> None:
        super().__init__(records, data_dir, spec, augment=augment, flip=False, yaw_max_deg=yaw_max_deg,
                         margin_deg=int(math.ceil(yaw_max_deg)), fx_px=fx_px, body_mask=body_mask,
                         photo_strength=photo_strength, load_images=load_images, fx_px_left=fx_px_left,
                         body_mask_left=body_mask_left)
        self.band_geom = dict(band_geom or {})
        if augment and yaw_max_deg > 0:
            missing = [r.name for r in records if r.name not in self.band_geom]
            if missing:
                raise ValueError(f"向きの拡張には視差の帯の焦点距離・主点が要る（dataset.json の calib_values）: {missing}")
            if crop_cols <= 0:
                raise ValueError("向きの拡張には視差の帯の窓（crop_cols）が要る")
        self.crop_cols = int(crop_cols)
        self.index = [(ri, i) for ri, i in self.index if records[ri].data["disp_ok"][i]]

    def _disp(self, rec: Record, i: int, psi: float) -> np.ndarray:
        arr = rec.data["disp"][i].astype(np.float32)
        if psi:
            # 車が左へ ψ 向くと、物は画像の右（列が増える方）へ動く
            arr = yaw_shift_disp(arr, psi, *self.band_geom[rec.name])
        return crop_disp(arr, self.crop_cols)

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
