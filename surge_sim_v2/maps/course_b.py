"""複雑コース（シケイン＋スイーパーの流れるような周回路）。

中心線（CENTER_LINE）を定義し、平滑化して角を丸めてから左右に半幅オフセットして
壁を自動生成する。平滑化により最小コーナー半径を確保し、走行路がつぶれない。
座標単位 [m]。
"""
from __future__ import annotations

import math

import numpy as np

from core.path_utils import path_normals, resample_closed

COURSE_NAME = "Complex Circuit"

_WIDTH = 1.0  # コース幅 [m]

# 中心線のウェイポイント（流れる周回路：下ストレート→右スイーパー→シケイン→上→左大カーブ）
_CENTER_PTS = [
    (1.6, 0.9),
    (4.2, 0.8),    # 下ストレート
    (5.8, 1.4),    # 右下スイーパー
    (6.2, 2.5),
    (5.4, 3.1),    # シケイン（S字）
    (6.2, 3.9),
    (5.3, 4.6),    # 右上→上ストレート
    (3.4, 4.9),
    (1.8, 4.4),    # 左上大カーブ
    (1.0, 3.2),
    (1.1, 2.0),    # 左ストレート
    (1.0, 1.3),
]


def _smooth_closed(pts: np.ndarray, k: int = 5, iters: int = 6) -> np.ndarray:
    kernel = np.ones(k) / k
    out = pts.copy()
    for _ in range(iters):
        x = np.convolve(np.r_[out[-k:, 0], out[:, 0], out[:k, 0]], kernel, mode="same")
        y = np.convolve(np.r_[out[-k:, 1], out[:, 1], out[:k, 1]], kernel, mode="same")
        out = np.column_stack([x[k:-k], y[k:-k]])
    return out


# リサンプル → 平滑化（角を丸めて半径確保）→ 再リサンプル
_center = resample_closed(np.array(_CENTER_PTS, dtype=float), 0.12)
_center = _smooth_closed(_center, k=7, iters=8)
_center = resample_closed(_center, 0.12)

_normals = path_normals(_center)
_left = _center + _normals * (_WIDTH / 2.0)
_right = _center - _normals * (_WIDTH / 2.0)


def _loop_edges(pts: np.ndarray):
    n = len(pts)
    return [((float(pts[i, 0]), float(pts[i, 1])),
             (float(pts[(i + 1) % n, 0]), float(pts[(i + 1) % n, 1]))) for i in range(n)]


WALLS = _loop_edges(_left) + _loop_edges(_right)

# スタート：中心線の起点、進行方向に整列
_dx = _center[1, 0] - _center[0, 0]
_dy = _center[1, 1] - _center[0, 1]
START_POSE = (float(_center[0, 0]), float(_center[0, 1]),
              float(math.degrees(math.atan2(_dy, _dx))))

# Phase2（cheat）用カンニング中心線
CENTER_LINE = [(float(p[0]), float(p[1])) for p in _center]
