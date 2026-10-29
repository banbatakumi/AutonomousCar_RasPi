"""ml_cam_e2e/yaw_aug.py — 「車の向きがずれた絵」と「そこから戻す舵」を作る拡張。

手本（人や slam2d_route）はいつも同じ線を走るので、記録には「ずれた絵とそこで戻す舵」の組が
無い。そのまま学習したモデルはずれを直す舵をほとんど持たず（v4 で dδ/dψ ≈ -0.12、
手本の追従の約1/10）、実車では数mで線から外れて戻れない（模倣学習の分布シフト。
Ross et al. 2011 の DAgger が扱う問題）。

NVIDIA の PilotNet（Bojarski et al. 2016, arXiv:1604.07316）と同じく、記録の絵を
「車が向きを ψ だけ変えた絵」に作り替え、ラベルを「そこから戻る舵」に補正する:

    δ' = δ − k·ψ        （ψ>0 は車が左を向いた。舵も左が正）

## 作り方

- **向きの回転は窓の横ずらしで近似する。** 等距離射影に近い魚眼では、地平線付近の
  ずれは fx·ψ [px] でほぼ一様。上下の端ほど近似が崩れるので ±6° 程度に留める
- **端に作り物の画素を出さない。** 学習でも推論でも左右を `crop_x` ずつ切り落とし
  （`raspi/core/cam_e2e_preproc.crop_window`）、ずらすのは窓の位置だけにする。
  端を引き伸ばして埋めると、その帯がラベルの補正と完全に連動し、モデルが帯を
  手がかりに覚えてしまう（causal confusion）
- **車体（ボンネット・前輪）は動かさない。** カメラは車体に固定なので、車が向きを
  変えても車体は同じ所に写る。走行中に画素がほとんど変わらない領域を記録から
  推定し（`estimate_body_mask`）、そこだけ元の絵を貼り戻す

## 教えられないこと

横方向の位置ずれ（向きは合っているが壁寄り）は、奥行きが無いと作れない。
"""

from __future__ import annotations

import itertools
import math
import random
from pathlib import Path

import numpy as np
from PIL import Image

__all__ = [
    "BODY_MASK_NAME",
    "ensure_body_mask",
    "estimate_body_mask",
    "front_fx_px",
    "load_body_mask",
    "margin_for",
    "save_body_mask",
    "yawed_window",
]

#: 車体マスクのファイル名（抽出したフレームと同じフォルダに置く。白=車体）
BODY_MASK_NAME = "body_mask.png"
#: 時間方向の揺らぎ（0〜255 の標準偏差）がこれ未満の画素を車体とみなす。
#: LED の点滅と暗部の JPEG ノイズで車体でも 20 前後は揺らぐ。背景は走行中なら 50 を大きく超える
_STILL_STD = 35.0
#: 車体はこれより上には写らない（画像の高さに対する割合）。天井の照明など、たまたま
#: 揺らぎの小さい背景を拾わないため
_BODY_TOP_FRAC = 0.6
#: 車体の上端より下で静止している画素の割合の下限（LED の点滅などの抜けを許す）
_BODY_FILL = 0.7
#: 車体の列にはさまれた抜けを補間する最大幅（画像幅に対する割合）
_GAP_FRAC = 0.2
#: 車体の縁を上へ広げる画素数（縁の数画素が景色側に残って二重に写るのを防ぐ）
_EDGE_PX = 3
#: 推定に使う最大枚数（全体から等間隔に選ぶ）
_MASK_FRAMES = 300


def _box_blur(img: np.ndarray, r: int) -> np.ndarray:
    """(H, W) の箱フィルタ（端は端の値で延長）。"""
    k = 2 * r + 1
    p = np.pad(img, r, mode="edge")
    c = np.cumsum(np.cumsum(p, axis=0), axis=1)
    c = np.pad(c, ((1, 0), (1, 0)))
    return (c[k:, k:] - c[:-k, k:] - c[k:, :-k] + c[:-k, :-k]) / (k * k)


def estimate_body_mask(paths: list[Path], *, max_frames: int = _MASK_FRAMES) -> np.ndarray:
    """走行中のフレーム群から車体の写る画素を推定する。`(H, W)` の bool。

    車体はカメラに対して動かないので、時間方向の揺らぎが小さい。各列について**下端から
    続く**揺らぎの小さい区間を車体とする（車体は画像の下端に接している）。
    列ごとの上端は中央値で均し、ところどころの抜けを埋める。
    """
    if not paths:
        raise ValueError("車体マスクを推定するフレームが無い")
    step = max(1, len(paths) // max_frames)
    chosen = paths[::step][:max_frames]
    first = np.asarray(Image.open(chosen[0]).convert("RGB"), dtype=np.float32)
    h, w = first.shape[:2]
    s1 = np.zeros((h, w), np.float64)
    s2 = np.zeros((h, w), np.float64)
    n = 0
    for p in chosen:
        im = np.asarray(Image.open(p).convert("RGB"), dtype=np.float32)
        if im.shape[:2] != (h, w):
            continue
        g = im.mean(axis=2)
        s1 += g
        s2 += g * g
        n += 1
    sd = np.sqrt(np.maximum(s2 / n - (s1 / n) ** 2, 0.0))
    still = _box_blur(sd, 3) < _STILL_STD

    # 列ごとに「その行から下端までの大半（_BODY_FILL 以上）が静止している、最も高い静止画素」を
    # 車体の上端とする。下端の LED は点滅して揺らぐので「下端から途切れずに続く」では切れてしまう
    top_limit = int(h * _BODY_TOP_FRAC)
    tops = np.full(w, h, dtype=np.int64)           # h ＝ その列に車体なし
    for x in range(w):
        col = still[:, x]
        below = np.cumsum(col[::-1])[::-1]          # その行から下端までの静止画素数
        frac = below / (h - np.arange(h))
        ok = np.flatnonzero(col & (frac >= _BODY_FILL) & (np.arange(h) >= top_limit))
        if len(ok):
            tops[x] = ok[0]
    # 車体の列にはさまれた短い抜け（光沢や LED で揺らぐ列）は両側から線形に補間する
    has = np.flatnonzero(tops < h)
    if len(has) >= 2:
        for a, b in itertools.pairwise(has):
            if 1 < b - a <= w * _GAP_FRAC:
                tops[a + 1:b] = np.round(np.linspace(tops[a], tops[b], b - a + 1)[1:-1])
    # 列ごとの上端を中央値で均し（1〜2列だけの欠けを埋める）、縁を少し上へ広げる
    r = 7
    padded = np.pad(tops, r, mode="edge")
    tops = np.array([int(np.median(padded[i:i + 2 * r + 1])) for i in range(w)])
    tops = np.where(tops < h, np.maximum(tops - _EDGE_PX, top_limit), h)
    rows = np.arange(h)[:, None]
    return rows >= tops[None, :]


def save_body_mask(mask: np.ndarray, path: Path) -> None:
    Image.fromarray(mask.astype(np.uint8) * 255, mode="L").save(path)


def load_body_mask(path: Path) -> np.ndarray:
    return np.asarray(Image.open(path).convert("L")) >= 128


def ensure_body_mask(frames_dir: Path, image_paths: list[Path]) -> np.ndarray:
    """`frames_dir/body_mask.png` を読む。無ければ推定して書く（次回から同じものを使う）。"""
    path = frames_dir / BODY_MASK_NAME
    if path.exists():
        return load_body_mask(path)
    mask = estimate_body_mask(image_paths)
    save_body_mask(mask, path)
    return mask


def front_fx_px(vehicle, width: int) -> float:
    """前カメラの水平焦点距離 [px]（幅 `width` の画像の画素）。

    魚眼の校正値があればその fx を幅の比で直す（校正は下端クロップ前・同じ水平画角）。
    無ければ `hfov` のピンホール近似。
    """
    fe = getattr(vehicle, "cam_front_fisheye", None)
    if fe is not None and fe.fx > 0 and fe.width > 0:
        return fe.fx * width / fe.width
    hfov = float(getattr(vehicle, "cam_front_hfov", 0.0) or 0.0)
    if not 0.0 < hfov < math.pi:
        raise ValueError("前カメラの fx が決まらない（魚眼の校正値も hfov も無い）")
    return (width / 2) / math.tan(hfov / 2)


def margin_for(max_deg: float, fx_px: float) -> int:
    """±`max_deg` ずらすのに要る左右の切り落とし幅 [px]。"""
    return math.ceil(fx_px * math.radians(max_deg))


def yawed_window(rgb: np.ndarray, margin_px: int, psi_rad: float, fx_px: float,
                 body: np.ndarray | None) -> np.ndarray:
    """車が `psi_rad` だけ左を向いた絵（左右を `margin_px` ずつ切り落とした窓）。

    車が左を向くと景色は右へ流れる → 窓を左へずらす。`body`（元画像と同じ大きさの
    車体マスク）の画素は元の中央の窓から取る（車体は車と一緒に回るので動かない）。
    """
    h, w = rgb.shape[:2]
    shift = round(fx_px * psi_rad)
    shift = max(-margin_px, min(margin_px, shift))
    center = rgb[:, margin_px:w - margin_px]
    moved = rgb[:, margin_px - shift:w - margin_px - shift]
    if body is None:
        return moved
    if body.shape != (h, w):
        body = np.asarray(Image.fromarray(body.astype(np.uint8) * 255).resize(
            (w, h), Image.NEAREST)) >= 128
    keep = body[:, margin_px:w - margin_px, None]
    return np.where(keep, center, moved)


def sample_psi(max_deg: float, rng: random.Random | None = None) -> float:
    """拡張で使う向きのずれ [rad]。±`max_deg` の一様分布。"""
    u = (rng or random).uniform(-1.0, 1.0)
    return math.radians(max_deg) * u
