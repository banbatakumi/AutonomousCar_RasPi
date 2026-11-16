"""stereo_e2e（前方ステレオの左右の画像＋視差の帯＋車両状態 → 前方の経路点）の入出力の形 —
**学習と推論が同じ関数を通るための唯一の定義。**

Pi の `stereo_node`（視差を縮めて `stereo/disp` に出す）・`stereo_e2e_node`（推論）と、Mac の学習
（`ml_stereo_e2e/`。記録の左右の組を照合し直して同じ関数を通す）がここを import する。numpy だけで書く。
画像の前処理は `cam_e2e_preproc.preprocess_stereo`、車両状態と出力の並びは `e2e_fusion_preproc` のものをそのまま使う。

## 入力

| 名前 | 形 | 中身 |
|---|---|---|
| `image` | `(1, 6, h, w)` | 前方ステレオの左右2枚（`[右, 左]`。`cam_e2e_preproc.preprocess_stereo`） |
| `disp` | `(1, 2, Hd, Wd − 2m)` | 視差の帯（`disp_band_input`）の**中央の窓**（`crop_disp`、左右 m = `disp_crop_cols` 列を落とす）。ch0 = 有効な画素の視差の平均 / `num_disp`（近いほど大きい・0〜1）、ch1 = 有効な画素の割合 |
| `state` | `(1, 3)` | 車速・実舵角・ヨーレート（`e2e_fusion_preproc.state_input`） |
| `avail` | `(1, 3)` | 画像・視差・車両状態が使えるか（1/0） |

## 視差の帯（`disp_band_input`）

`stereo_node` が照合する平行化画像の行の帯（既定 150〜260 行・640 幅）の視差 `d`（無効は −1）から、
左端の `num_disp` 列（右の画像に対応が無く照合できない列）を落とし、`(Hd, Wd)` のブロックに分けて平均する。
距離ではなく**視差のまま**にするのは、視差の誤差が画素でほぼ一定（距離の誤差は距離の2乗で増える）で、
遠い所（視差が小さい）が自然に小さな値になるため。照合できなかった所は ch0 = 0・ch1 = 0
（模様の無い壁・暗い所。**空きという意味ではない**のでモデルに割合も渡す）。

## 向きの拡張と窓（`yaw_shift_disp`・`crop_disp`）

学習では車の向きを ψ ずらした標本を作る。視差の帯は、帯全体を**透視投影どおりに**横へずらし（1 列あたりの角度は
一定ではない。100° の平行化では端の 1 列は中央の約 1/2.4 の角度）、**両端 m 列を落とした中央の窓**だけをモデルに
渡す。推論でも同じ窓を切る。m はずらしの最大量より大きく取るので、ずらしで空いた列（照合できなかった＝0）が
窓に入らない（画像の `crop_x` と同じ方式。端の作り物を手がかりにする漏れを防ぐ。レビュー M9）。
"""

from __future__ import annotations

import numpy as np

from raspi.core.e2e_fusion_preproc import FusionOutput, decode_output, output_size, state_input

__all__ = ["PREPROC_VERSION", "DISP_VERSION", "INPUT_NAMES", "DEFAULT_DISP_HW", "FusionOutput",
           "disp_band_input", "disp_to_bytes", "disp_from_bytes", "disp_input", "state_input",
           "output_size", "decode_output", "band_geometry", "yaw_shift_disp", "disp_margin_cols", "crop_disp"]

#: 入力の並び・正規化・出力の並びを変えたら上げる。モデル同梱の JSON と照合する
PREPROC_VERSION = 2          # 2: 視差は中央の窓（`disp_crop_cols`）だけを渡す
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


def band_geometry(f_px: float, cx_px: float, scale: float, num_disp: int, out_w: int,
                  img_w: int = 640) -> tuple[float, float]:
    """平行化画像の焦点距離・主点（`img_w` 幅の画素）→ 視差の帯の列で測った `(焦点距離, 主点)`。

    帯の列 c の中心の向きは tan φ = (c − 主点) / 焦点距離（右が正）。`disp_band_input` の切り方と同じ式。
    """
    ws = int(round(img_w * float(scale))) - int(num_disp)
    bw = ws // int(out_w)
    if bw < 1:
        raise ValueError("視差の帯が出力より狭い")
    c0 = (ws - bw * int(out_w)) // 2
    f_b = float(f_px) * float(scale) / bw
    cx_b = (float(cx_px) * float(scale) - int(num_disp) - c0) / bw - 0.5
    return f_b, cx_b


def yaw_shift_disp(arr: np.ndarray, psi_rad: float, f_b: float, cx_b: float) -> np.ndarray:
    """視差の帯 `(2, H, W)` を、車が左へ `psi_rad` 向いたときの見え方へずらす（物は右へ動く）。

    透視投影どおり: 新しい列 c は、向き atan((c − cx)/f) − ψ にあった元の列から 1 次補間で取る。はみ出しは 0。
    """
    if not psi_rad:
        return arr
    _, _, w = arr.shape
    c = np.arange(w, dtype=np.float64)
    src = cx_b + f_b * np.tan(np.arctan((c - cx_b) / f_b) - float(psi_rad))
    i0 = np.floor(src).astype(int)
    fr = (src - i0).astype(np.float32)
    out = np.zeros_like(arr)
    for di, wt in ((0, 1.0 - fr), (1, fr)):
        j = i0 + di
        ok = (j >= 0) & (j < w)
        out[:, :, ok] += arr[:, :, j[ok]] * wt[ok]
    return out


def disp_margin_cols(yaw_max_rad: float, f_b: float, cx_b: float, w: int) -> int:
    """`yaw_shift_disp` で ±`yaw_max_rad` ずらしても、両端 m 列を落とした窓に空いた列が入らない m。"""
    if yaw_max_rad <= 0:
        return 0
    c = np.arange(w, dtype=np.float64)
    worst = 0.0
    for psi in (-yaw_max_rad, yaw_max_rad):
        src = cx_b + f_b * np.tan(np.arctan((c - cx_b) / f_b) - psi)
        worst = max(worst, float(np.max(np.abs(src - c))))
    m = int(np.ceil(worst)) + 1                       # 1 次補間の隣の列のぶん
    if 2 * m >= w:
        raise ValueError(f"向きの拡張 {np.degrees(yaw_max_rad):.1f}° に視差の帯（{w} 列）が足りない")
    return m


def crop_disp(arr: np.ndarray, m: int) -> np.ndarray:
    """視差の帯 `(2, H, W)` の両端 `m` 列を落とした中央の窓（推論と学習で同じ）。"""
    if m <= 0:
        return arr
    return arr[:, :, m:arr.shape[2] - m]
