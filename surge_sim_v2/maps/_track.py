"""中心線から幅一定の周回路（左右壁）を生成する共通ヘルパ。

中心線ウェイポイント → 平滑化（角を丸めて半径確保）→ 法線方向に半幅オフセット
して左右壁を作る。任意形状でも幅一定の有効な走行路になる。
"""
from __future__ import annotations

import math

import numpy as np

from core.path_utils import path_normals, resample_closed


def _smooth_closed(pts: np.ndarray, k: int, iters: int) -> np.ndarray:
    kernel = np.ones(k) / k
    out = pts.copy()
    for _ in range(iters):
        x = np.convolve(np.r_[out[-k:, 0], out[:, 0], out[:k, 0]], kernel, mode="same")
        y = np.convolve(np.r_[out[-k:, 1], out[:, 1], out[:k, 1]], kernel, mode="same")
        out = np.column_stack([x[k:-k], y[k:-k]])
    return out


def _loop_edges(pts: np.ndarray):
    n = len(pts)
    return [((float(pts[i, 0]), float(pts[i, 1])),
             (float(pts[(i + 1) % n, 0]), float(pts[(i + 1) % n, 1]))) for i in range(n)]


def build_loop(center_pts, width: float = 1.0, spacing: float = 0.12,
               smooth_k: int = 7, smooth_iters: int = 8):
    """戻り値: (walls, start_pose, center_line[list of (x,y)])。"""
    center = resample_closed(np.array(center_pts, dtype=float), spacing)
    center = _smooth_closed(center, smooth_k, smooth_iters)
    center = resample_closed(center, spacing)
    n = path_normals(center)
    left = center + n * (width / 2.0)
    right = center - n * (width / 2.0)
    walls = _loop_edges(left) + _loop_edges(right)
    dx, dy = center[1] - center[0]
    start_pose = (float(center[0, 0]), float(center[0, 1]),
                  float(math.degrees(math.atan2(dy, dx))))
    center_line = [(float(p[0]), float(p[1])) for p in center]
    return walls, start_pose, center_line
