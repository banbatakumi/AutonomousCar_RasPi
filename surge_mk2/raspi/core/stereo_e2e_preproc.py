"""stereo_e2e（前方ステレオの左右の画像＋視差の帯＋車両状態 → 前方の経路点）の入出力の形 —
**学習と推論が同じ関数を通るための唯一の定義。**

Pi の `stereo_node`（視差を縮めて `stereo/disp` に出す）・`stereo_e2e_node`（推論）と、Mac の学習
（`ml_stereo_e2e/`。記録の左右の組を照合し直して同じ関数を通す）がここを import する。numpy だけで書く。
画像の前処理は `cam_e2e_preproc.preprocess_stereo`、車両状態と出力の並びは `e2e_fusion_preproc` のものをそのまま使う。

## 入力

| 名前 | 形 | 中身 |
|---|---|---|
| `image` | `(1, 6, h, w)` | 前方ステレオの左右2枚（`[右, 左]`。`cam_e2e_preproc.preprocess_stereo`） |
| `disp` | `(1, 2, Hd, Wd)` | 視差の帯（`disp_band_input`）。ch0 = 有効な画素の視差の平均 / `num_disp`（近いほど大きい・0〜1）、ch1 = 有効な画素の割合 |
| `state` | `(1, 3)` | 車速・実舵角・ヨーレート（`e2e_fusion_preproc.state_input`） |
| `avail` | `(1, 3)` | 画像・視差・車両状態が使えるか（1/0） |

## 視差の帯（`disp_band_input`）

`stereo_node` が照合する平行化画像の行の帯（既定 150〜260 行・640 幅）の視差 `d`（無効は −1）から、
左端の `num_disp` 列（右の画像に対応が無く照合できない列）を落とし、`(Hd, Wd)` のブロックに分けて平均する。
距離ではなく**視差のまま**にするのは、視差の誤差が画素でほぼ一定（距離の誤差は距離の2乗で増える）で、
遠い所（視差が小さい）が自然に小さな値になるため。照合できなかった所は ch0 = 0・ch1 = 0
（模様の無い壁・暗い所。**空きという意味ではない**のでモデルに割合も渡す）。
"""

from __future__ import annotations

import numpy as np

from raspi.core.e2e_fusion_preproc import FusionOutput, decode_output, output_size, state_input

__all__ = ["PREPROC_VERSION", "DISP_VERSION", "INPUT_NAMES", "DEFAULT_DISP_HW", "FusionOutput",
           "disp_band_input", "disp_to_bytes", "disp_from_bytes", "disp_input", "state_input",
           "output_size", "decode_output"]

#: 入力の並び・正規化・出力の並びを変えたら上げる。モデル同梱の JSON と照合する
PREPROC_VERSION = 1
#: `disp_band_input` の中身を変えたら上げる（`StereoDisp.preproc_version`・モデルの JSON と照合する）
DISP_VERSION = 1
#: ONNX の入力名（この順）
INPUT_NAMES = ("image", "disp", "state", "avail")
#: 視差の帯を縮めた大きさ `(Hd, Wd)`。640 幅・行 150〜260・探索幅 96 で 9×8 画素のブロック
DEFAULT_DISP_HW = (12, 68)


def disp_band_input(d: np.ndarray, num_disp: int, out_hw: tuple[int, int] = DEFAULT_DISP_HW) -> np.ndarray:
    """照合した視差 `d`（帯だけ・縮めた画像の画素・無効は 0 以下）→ `(2, Hd, Wd)` float32。

    左端の `num_disp` 列を落とし、残りを中央寄せで `(Hd, Wd)` のブロックに割り切れる大きさに切ってから平均する。
    """
    d = np.asarray(d, dtype=np.float32)
    if d.ndim != 2:
        raise ValueError(f"視差は2次元のはず: {d.shape}")
    oh, ow = int(out_hw[0]), int(out_hw[1])
    nd = int(num_disp)
    if nd <= 0:
        raise ValueError("num_disp は正のはず")
    band = d[:, nd:] if d.shape[1] > nd else d[:, :0]
    H, W = band.shape
    bh, bw = H // oh, W // ow
    if bh < 1 or bw < 1:
        raise ValueError(f"視差の帯 {band.shape} が出力 {out_hw} より小さい")
    r0, c0 = (H - bh * oh) // 2, (W - bw * ow) // 2
    blk = band[r0:r0 + bh * oh, c0:c0 + bw * ow].reshape(oh, bh, ow, bw)
    valid = blk > 0
    cnt = valid.sum(axis=(1, 3)).astype(np.float32)
    tot = np.where(valid, blk, 0.0).sum(axis=(1, 3), dtype=np.float32)
    mean = np.where(cnt > 0, tot / np.maximum(cnt, 1.0), 0.0) / np.float32(nd)
    out = np.stack([np.clip(mean, 0.0, 1.0), cnt / np.float32(bh * bw)])
    return np.ascontiguousarray(out, dtype=np.float32)


def disp_to_bytes(arr: np.ndarray) -> bytes:
    """`(2, Hd, Wd)` → `StereoDisp.data`（float16・C 順）。"""
    return np.ascontiguousarray(arr, dtype=np.float16).tobytes()


def disp_from_bytes(data: bytes, h: int, w: int) -> np.ndarray:
    """`StereoDisp.data` → `(2, h, w)` float32。長さが合わない・有限でなければ例外。"""
    a = np.frombuffer(data, dtype=np.float16)
    if a.size != 2 * h * w:
        raise ValueError(f"視差の長さ {a.size} が (2, {h}, {w}) と合わない")
    out = a.astype(np.float32).reshape(2, h, w)
    if not np.isfinite(out).all():
        raise ValueError("視差に NaN/Inf がある")
    return out


def disp_input(arr: np.ndarray) -> np.ndarray:
    """`(2, Hd, Wd)` → `(1, 2, Hd, Wd)` float32（ONNX の `disp`）。"""
    return np.ascontiguousarray(np.asarray(arr, dtype=np.float32)[None, ...])
