"""ml_cam_e2e/photo_aug.py — 舵の正解を変えない外乱（ブレ・ノイズ・色味・JPEG）を絵に足す拡張。

明るさ・コントラストの揺らぎ（`dataset.py`）に加えて、実車で起きているのに学習の絵に
無かった変化を混ぜ、モデルがそれを舵の手がかりにしないようにする。どれも**舵の正解を
変えない**（向きのずれのように正解が変わる変形は `yaw_aug.py` でラベルを補正する）。

| 外乱 | 実車で起きていること |
|---|---|
| 横ブレ | 露出 約33ms（暗い室内で自動露出が上限）。旋回の速さに比例して横に流れる。手本より遅く走ると舵が足りない原因だった（2026-10-30、ブレ 20px を足すと舵が 0.56→0.79 倍に戻った） |
| センサーのノイズ | ゲインがほぼ最大（約10倍）なので、暗いほどざらつく |
| 色味 | 窓の外光と室内灯が混ざり、時間で色が変わる |
| JPEG | 学習の絵は記録用の JPEG を展開したもの、実車の推論は圧縮前の生の絵 |

**元の解像度で、実際に起きる順（ブレ → ノイズ → 色味 → 圧縮）に掛ける。** 縮小の後で
足すとノイズが強すぎ、ブレが細かくなりすぎる。強さは各外乱 50% の確率で一様に選ぶ。
`strength` で全体を倍率で変える（0 で無効）。強くしすぎると舵の予測自体が崩れるので、
既定は実測の範囲に留める。
"""

from __future__ import annotations

import io

import numpy as np
from PIL import Image

__all__ = ["apply", "color_shift", "horizontal_blur", "jpeg_roundtrip", "sensor_noise"]

#: 各外乱を掛ける確率
_P = 0.5
#: 横ブレの最大幅（画像幅に対する割合）。640px で 24px。1.06m/s・半径0.6m・露出33ms で約18px
_BLUR_MAX_FRAC = 24 / 640
#: ノイズの標準偏差の最大（0〜255）
_NOISE_MAX = 6.0
#: 色味: チャンネルごとの倍率の揺らぎ・彩度の揺らぎ
_WB_RANGE = 0.08
_SAT_RANGE = 0.25
#: JPEG の品質の範囲（低いほど強い）。上限 95 は記録用 JPEG とほぼ同じ
_JPEG_Q = (40, 95)


def horizontal_blur(rgb: np.ndarray, length: int) -> np.ndarray:
    """横方向の箱型ブレ（幅 `length` px、端は端の値で延長）。`length <= 1` はそのまま。"""
    if length <= 1:
        return rgb
    x = rgb.astype(np.float32)
    left = length // 2
    right = length - 1 - left
    p = np.pad(x, ((0, 0), (left + 1, right), (0, 0)), mode="edge")
    c = np.cumsum(p, axis=1)
    out = (c[:, length:] - c[:, :-length]) / length
    return np.clip(out + 0.5, 0, 255).astype(np.uint8)


def sensor_noise(rgb: np.ndarray, sigma: float, rng: np.random.Generator) -> np.ndarray:
    """標準偏差 `sigma`（0〜255）のガウスノイズ。"""
    if sigma <= 0:
        return rgb
    noisy = rgb.astype(np.float32) + rng.normal(0.0, sigma, rgb.shape).astype(np.float32)
    return np.clip(noisy + 0.5, 0, 255).astype(np.uint8)


def color_shift(rgb: np.ndarray, gains: tuple[float, float, float],
                saturation: float) -> np.ndarray:
    """チャンネルごとの倍率（ホワイトバランスのずれ）と彩度の倍率。"""
    x = rgb.astype(np.float32) * np.asarray(gains, np.float32)
    gray = x.mean(axis=2, keepdims=True)
    x = gray + (x - gray) * saturation
    return np.clip(x + 0.5, 0, 255).astype(np.uint8)


def jpeg_roundtrip(rgb: np.ndarray, quality: int) -> np.ndarray:
    buf = io.BytesIO()
    Image.fromarray(rgb).save(buf, format="JPEG", quality=int(quality))
    buf.seek(0)
    return np.asarray(Image.open(buf).convert("RGB"))


def apply(rgb: np.ndarray, rng: np.random.Generator, strength: float = 1.0) -> np.ndarray:
    """外乱を確率的に足した絵（`(H, W, 3)` uint8、元の解像度）を返す。"""
    if strength <= 0:
        return rgb
    if rng.random() < _P:
        max_len = _BLUR_MAX_FRAC * rgb.shape[1] * strength
        rgb = horizontal_blur(rgb, round(rng.uniform(0.0, max_len)))
    if rng.random() < _P:
        rgb = sensor_noise(rgb, rng.uniform(0.0, _NOISE_MAX * strength), rng)
    if rng.random() < _P:
        wb = _WB_RANGE * strength
        sat = _SAT_RANGE * strength
        gains = tuple(float(g) for g in rng.uniform(1.0 - wb, 1.0 + wb, 3))
        rgb = color_shift(rgb, gains, float(rng.uniform(1.0 - sat, 1.0 + sat)))
    if rng.random() < _P:
        lo, hi = _JPEG_Q
        lo = max(1, round(hi - (hi - lo) * min(strength, 2.0)))
        rgb = jpeg_roundtrip(rgb, int(rng.integers(lo, hi + 1)))
    return rgb
