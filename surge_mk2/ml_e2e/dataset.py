"""ml_e2e/dataset.py — `extract.py` が作った標本を読む PyTorch Dataset。

**入力の形は `raspi/core/e2e_fusion_preproc.py`・`cam_e2e_preproc.py` を通す**（実車の推論と同じ関数）。
点群は `raspi.auto.base.scan_window` で読む（欠測・飽和・測距不能の扱いを書き写さない）。

## 拡張（`augment=True`）

- **向きをずらす**（`yaw_max_deg`）: 車が ψ だけ左を向いたことにする。画像は窓を横へずらし
  （`ml_cam_e2e/yaw_aug.py`）、点群は ψ ぶん角度をずらし、経路点は −ψ 回す（どれも同じ ψ）。
  舵の手本は `δ − k·ψ`。点群と経路点は幾何として正確で、画像だけが近似
- **左右反転**: 画像・点群・経路点・舵・ヨーレートを一緒に反転する
- **画像の外乱**（`photo_strength`）: ブレ・ノイズ・色味・JPEG（`ml_cam_e2e/photo_aug.py`）
- 入力を伏せる（どの入力が使えるかの `avail`）のは学習ループ側（`train.py`）
"""

from __future__ import annotations

import json
import math
import sys
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import torch
from PIL import Image
from torch.utils.data import Dataset

_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(_ROOT))
sys.path.insert(0, str(_ROOT / "ml_cam_e2e"))        # yaw_aug・photo_aug（スクリプト形式のモジュール）

import photo_aug  # noqa: E402
from yaw_aug import yawed_window  # noqa: E402

from raspi.auto.base import scan_window  # noqa: E402
from raspi.core.cam_e2e_preproc import crop_margin_px, crop_window, normalize, resize_area  # noqa: E402
from raspi.core.e2e_fusion_preproc import lidar_rows  # noqa: E402
from raspi.msgs.types import Scan  # noqa: E402

__all__ = ["Record", "InputSpec", "load_records", "split_records", "FusionDataset", "sample_weights"]

_BRIGHTNESS = 25.0
_CONTRAST = 0.2


@dataclass(frozen=True)
class InputSpec:
    """モデルの入出力の契約のうち、標本の作り方を決める値（`export_onnx.py` が JSON に書く）。"""

    size: tuple[int, int] = (224, 128)      #: 画像の (幅, 高さ)
    crop_x: float = 0.0                     #: 左右の切り落とし（画像幅に対する割合）
    fov_deg: int = 270
    max_range: float = 8.0
    history: int = 3
    k: int = 32                             #: 経路点の数
    stride: int = 1                         #: 抽出した経路点を何個おきに使うか（`ds` = 抽出の ds × stride）
    max_steer: float = 0.47
    speed_ref: float = 5.0
    yaw_rate_ref: float = 6.0


@dataclass
class Record:
    name: str
    data: dict                              #: `<name>.npz` の配列
    rows: np.ndarray                        #: (M, 2, n_wide) 点群の行（視野は `fov_deg + 2·margin_deg`）
    maps: tuple[str, ...]

    def __len__(self) -> int:
        return len(self.data["t"])


def _rows_for(dist: np.ndarray, seen: np.ndarray, sat: np.ndarray, fov: int, max_range: float) -> np.ndarray:
    out = np.zeros((len(dist), 2, fov + 1), dtype=np.float32)
    has_sat = bool(sat.any())
    for i in range(len(dist)):
        sc = Scan(dist=dist[i].tolist(), sector_seen=seen[i].tolist(),
                  saturated=sat[i].tolist() if has_sat else None)
        w = scan_window(sc, fov, max_range)
        out[i] = lidar_rows(w.dist, w.measured, max_range)
    return out


def load_records(data_dir: Path, spec: InputSpec, *, margin_deg: int = 0) -> tuple[list[Record], dict]:
    """`data_dir` の全記録を読む。`margin_deg` は向きの拡張のために左右へ広げる点群の視野。"""
    meta = json.loads((data_dir / "dataset.json").read_text())
    records = []
    for p in sorted(data_dir.glob("*.npz")):
        d = dict(np.load(p))
        if len(d["t"]) == 0:
            continue
        rows = _rows_for(d["scan_dist"], d["scan_seen"], d["scan_sat"], spec.fov_deg + 2 * margin_deg,
                         spec.max_range)
        info = meta["records"].get(p.stem, {})
        records.append(Record(name=p.stem, data=d, rows=rows, maps=tuple(info.get("maps", []))))
    return records, meta


def split_records(records: list[Record], val_names: list[str] | None = None
                  ) -> tuple[list[Record], list[Record]]:
    """学習用と検証用に**記録単位で**分ける。

    同じ走行の別の区間を検証に回すと、ほぼ同じ絵が学習側にあって実質リークする。`val_names` が無ければ、
    同じ地図の記録が3本以上あるとき、その最後の1本を検証に回す。
    """
    if val_names:
        val = [r for r in records if r.name in val_names]
        return [r for r in records if r.name not in val_names], val
    groups: dict[str, list[Record]] = {}
    for r in records:
        groups.setdefault(r.maps[0] if r.maps else "", []).append(r)
    val = [g[-1] for key, g in groups.items() if key and len(g) >= 3]
    return [r for r in records if r not in val], val


def sample_weights(records: list[Record], max_steer: float, *, n_bins: int = 15, alpha: float = 0.5
                   ) -> np.ndarray:
    """直進過多の補正。実舵角の区間に属する枚数 `n` に対して `1 / n**alpha`。"""
    steer = np.concatenate([r.data["state"][:, 1] for r in records]) / max_steer
    b = np.clip(((steer + 1.0) / 2.0 * n_bins).astype(int), 0, n_bins - 1)
    counts = np.bincount(b, minlength=n_bins).astype(np.float64)
    return 1.0 / np.maximum(counts[b], 1.0) ** alpha


class FusionDataset(Dataset):
    """`__getitem__` は入力と手本の dict（テンソル）。

    | キー | 形 | |
    |---|---|---|
    | `image` | (3, h, w) | |
    | `lidar` | (H+1, n) | |
    | `state` | (3,) | |
    | `ctrl` | (2,) | 舵（−1〜1）・速度（0〜1）の手本 |
    | `ctrl_ok` | (2,) | その手本を損失に使うか |
    | `path` | (K, 2) | 経路点（`xy_scale` で割った値） |
    | `path_v` | (K,) | 各点の速度（0〜1） |
    | `pt_ok` | (K,) | その点が有効か |
    | `v_ok` | () | 各点の速度を損失に使うか |
    """

    def __init__(self, records: list[Record], data_dir: Path, spec: InputSpec, *, augment: bool = False,
                 flip: bool = True, yaw_max_deg: float = 0.0, yaw_k: float = 0.8, margin_deg: int = 0,
                 fx_px: float = 0.0, body_mask: np.ndarray | None = None, photo_strength: float = 0.0,
                 load_images: bool = True) -> None:
        self.records = records
        self.data_dir = data_dir
        self.spec = spec
        self.augment = augment
        self.flip = flip
        self.yaw_max_deg = yaw_max_deg
        self.yaw_k = yaw_k
        self.margin_deg = margin_deg
        self.fx_px = fx_px
        self.body_mask = body_mask
        self.photo_strength = photo_strength
        self.load_images = load_images
        if yaw_max_deg > 0 and (spec.crop_x <= 0 or fx_px <= 0 or margin_deg < yaw_max_deg):
            raise ValueError("向きの拡張には crop_x・fx_px と、点群の余白 margin_deg >= yaw_max_deg が要る")
        self.index = [(ri, i) for ri, r in enumerate(records) for i in range(len(r))]
        self.xy_scale = spec.k * spec.stride * self._ds()

    def _ds(self) -> float:
        meta = json.loads((self.data_dir / "dataset.json").read_text())
        return float(meta["config"]["ds"])

    def __len__(self) -> int:
        return len(self.index)

    def _lidar(self, rec: Record, i: int, psi_deg: int) -> np.ndarray:
        """標本 `i` の点群の入力 `(H+1, n)`。`psi_deg` は車が左へ向いた角度 [deg]（整数）。"""
        sp = self.spec
        idx = rec.data["scan_idx"][i]
        # 車が左へ ψ 向くと、物は −ψ の方向に見える: 新しい角 θ の値は元の角 θ+ψ の値
        lo = self.margin_deg + psi_deg
        sl = slice(lo, lo + sp.fov_deg + 1)
        out = []
        last = rec.rows[idx[0]]
        for h in range(sp.history):
            r = rec.rows[idx[h]] if h < len(idx) and idx[h] >= 0 else last
            out.append(r[0, sl])
            last = r
        out.append(rec.rows[idx[0]][1, sl])
        return np.stack(out)

    def __getitem__(self, n: int) -> dict[str, torch.Tensor]:
        ri, i = self.index[n]
        rec = self.records[ri]
        d = rec.data
        sp = self.spec
        w, h = sp.size

        psi = 0.0
        if self.augment and self.yaw_max_deg > 0:
            # 点群は 1° 刻みなので、ずらす角度を整数度にして3つの入力で厳密に同じ角度を使う
            psi_deg = int(np.random.randint(-int(self.yaw_max_deg), int(self.yaw_max_deg) + 1))
            psi = math.radians(psi_deg)
        else:
            psi_deg = 0
        do_flip = self.augment and self.flip and np.random.rand() < 0.5

        # ── 画像 ──
        if self.load_images:
            rgb = np.asarray(Image.open(self.data_dir / str(d["image"][i])).convert("RGB"))
            if self.augment and self.photo_strength > 0:
                rng = np.random.default_rng(np.random.randint(0, 2**31))
                rgb = photo_aug.apply(rgb, rng, self.photo_strength)
            if self.augment and self.yaw_max_deg > 0:
                rgb = yawed_window(rgb, crop_margin_px(rgb.shape[1], sp.crop_x), psi, self.fx_px,
                                   self.body_mask)
            else:
                rgb = crop_window(rgb, sp.crop_x)
            small = resize_area(rgb, w, h)
            if do_flip:
                small = small[:, ::-1, :]
            if self.augment:
                gain = 1.0 + np.random.uniform(-_CONTRAST, _CONTRAST)
                bias = np.random.uniform(-_BRIGHTNESS, _BRIGHTNESS)
                small = np.clip(small.astype(np.float32) * gain + bias, 0.0, 255.0)
            image = normalize(np.ascontiguousarray(small))[0]
        else:
            image = np.zeros((3, h, w), dtype=np.float32)

        # ── 点群 ──
        lidar = self._lidar(rec, i, psi_deg)
        if do_flip:
            lidar = lidar[:, ::-1]

        # ── 車両状態 ──
        speed, steer_act, yaw_rate = (float(x) for x in d["state"][i])
        sgn = -1.0 if do_flip else 1.0
        state = np.array([speed / sp.speed_ref, sgn * steer_act / sp.max_steer,
                          sgn * yaw_rate / sp.yaw_rate_ref], dtype=np.float32)

        # ── 手本 ──
        sel = slice(sp.stride - 1, sp.stride * sp.k, sp.stride)
        xy = d["path_xy"][i][sel].astype(np.float64)
        if psi:
            c, s = math.cos(-psi), math.sin(-psi)
            xy = np.stack([c * xy[:, 0] - s * xy[:, 1], s * xy[:, 0] + c * xy[:, 1]], axis=1)
        if do_flip:
            xy[:, 1] = -xy[:, 1]
        pt_ok = d["path_valid"][i][sel]
        path_v = np.clip(np.nan_to_num(d["path_v"][i][sel]) / sp.speed_ref, 0.0, 1.0)

        steer_cmd = float(d["steer_cmd"][i])
        steer_ok = math.isfinite(steer_cmd)
        steer_n = 0.0
        if steer_ok:
            steer_n = max(-1.0, min(1.0, sgn * (steer_cmd - self.yaw_k * psi) / sp.max_steer))
        speed_n = max(0.0, min(1.0, float(d["speed_cmd"][i]) / sp.speed_ref))

        return {
            "image": torch.from_numpy(image),
            "lidar": torch.from_numpy(np.ascontiguousarray(lidar, dtype=np.float32)),
            "state": torch.from_numpy(state),
            "ctrl": torch.tensor([steer_n, speed_n], dtype=torch.float32),
            "ctrl_ok": torch.tensor([steer_ok, bool(d["speed_ok"][i])], dtype=torch.float32),
            "path": torch.from_numpy((xy / self.xy_scale).astype(np.float32)),
            "path_v": torch.from_numpy(path_v.astype(np.float32)),
            "pt_ok": torch.from_numpy(pt_ok.astype(np.float32)),
            "v_ok": torch.tensor(float(d["v_ok"][i]), dtype=torch.float32),
        }
