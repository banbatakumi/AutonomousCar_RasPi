"""cam_e2e（カメラE2E・模倣学習）の前処理 — **学習と推論が同じ関数を通るための唯一の定義。**

Pi の推論（`raspi/nodes/cam_e2e_node.py`）と Mac の学習・評価
（`ml_cam_e2e/dataset.py`・`eval.py`）の両方がここを import する。
以前は学習が PIL の BILINEAR・RGB、推論が最近傍・BGR（リングの並びのまま）と
食い違っており、学習したとおりの絵が実車のモデルに入っていなかった
（2026-10-06 に発覚。誰も学習していなかったので実害は出ていない）。
**同じ処理を2箇所に書かない**ことでこの種の事故を構造的に防ぐ。

numpy だけで書く——Pi には opencv も Pillow も入れていない
（`raspi/requirements.lock.txt`）。

`to_rgb()`・`resize_area()` は `raspi/nodes/cam_perception_node.py`
（セグメンテーション）と `ml_cam/dataset.py` も使う。あちらにも同じ食い違い
（色順が逆・縮小方式が別）があり、同じ関数に寄せて直した（2026-10-06）。

`PREPROC_VERSION` はモデル同梱の `<name>.json` に書かれ、ノードが読み込み時に
照合する。ここの処理（色順・リサイズ方式・正規化）を変えたら必ず上げること。
古い前処理で学習したモデルを新しい前処理で走らせる事故を、読み込み拒否で止める。
"""

from __future__ import annotations

from functools import lru_cache

import numpy as np

__all__ = ["PREPROC_VERSION", "to_rgb", "resize_area", "normalize", "preprocess"]

PREPROC_VERSION = 1


def to_rgb(frame: np.ndarray, fmt: str) -> np.ndarray:
    """共有メモリの生フレーム → RGB の (H, W, 3)。

    `fmt` は `ImageRef.fmt`（**メモリ上の実際のバイト順**、`camera_node.py` の
    `_MEMORY_ORDER` 参照）。判定は `raspi/core/jpeg.py` と同じ「`BGR` で始まるか」。
    記録の JPEG は jpeg.py が同じ規則で正しい色にしているので、ここが揃っていれば
    学習（JPEG をデコードした RGB）と推論の色が一致する。
    """
    if fmt.startswith("BGR"):
        return frame[..., 2::-1]
    return frame[..., :3]


@lru_cache(maxsize=8)
def _area_weights(src: int, out: int) -> np.ndarray:
    """面積平均の重み行列 `(out, src)`。行 i は出力 i が覆う入力の区画に `1/画素数`。

    拡大方向（out > src）では区画が1画素になる（結果は最近傍と同じ）。
    """
    idx = (np.arange(out, dtype=np.int64) * src) // out
    counts = np.maximum(np.diff(np.append(idx, src)), 1)
    m = np.zeros((out, src), dtype=np.float32)
    for i, (start, n) in enumerate(zip(idx, counts)):
        m[i, start:start + n] = 1.0 / n
    return m


def resize_area(rgb: np.ndarray, width: int, height: int) -> np.ndarray:
    """面積平均で (height, width, C) の uint8 に縮小する。

    640x360 → 224x128 は約2.9倍の縮小。最近傍だと画素を拾い飛ばして
    エイリアシングが出るうえ、JPEG（学習データ）と生フレーム（推論）の差が
    そのまま残る。区画内を平均すればブロックノイズも均されて差が小さくなる。

    縦・横それぞれの重み行列との積で求める（BLAS に任せられるので速い。
    Mac で 640x360→224x128 が約 0.7ms。`np.add.reduceat` と整数除算で
    書くと約 4ms かかった）。
    """
    src_h, src_w = rgb.shape[:2]
    if (src_w, src_h) == (width, height):
        return np.ascontiguousarray(rgb, dtype=np.uint8)
    x = np.tensordot(_area_weights(src_h, height), rgb.astype(np.float32), axes=(1, 0))
    x = np.tensordot(x, _area_weights(src_w, width), axes=(1, 1))      # (h, C, w)
    return np.floor(x.transpose(0, 2, 1) + 0.5).astype(np.uint8)


def normalize(resized: np.ndarray, mean: float = 0.0, std: float = 255.0) -> np.ndarray:
    """縮小済みの (h, w, 3) → モデル入力 (1, 3, h, w) float32。

    学習側（`ml_cam_e2e/dataset.py`）は縮小と正規化の間に明るさの揺らぎを
    挟むので、2段に分けてある。推論は `preprocess()` で一息に通す。
    """
    x = (resized.astype(np.float32) - mean) / std
    return np.ascontiguousarray(np.transpose(x, (2, 0, 1))[None, ...])


def preprocess(rgb: np.ndarray, size: tuple[int, int], mean: float = 0.0,
               std: float = 255.0) -> np.ndarray:
    """RGB の (H, W, 3) uint8 → モデル入力 (1, 3, h, w) float32。`size` は (width, height)。"""
    w, h = size
    return normalize(resize_area(rgb, w, h), mean, std)
