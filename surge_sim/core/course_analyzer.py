"""コース境界抽出・中心線生成。

Phase3: 占有格子地図（SLAMで構築）から中心線・左右壁・コース幅を抽出する。
        探索走行の軌跡(seed_path)をシードに、各点で法線方向に地図をレイマーチして
        左右の壁までの距離を測り、その中点列を中心線とする（ループ位相は軌跡が与える）。

Phase2の `build_course_map`（コース真値からのカンニング版）も後方互換で残す。
"""

from __future__ import annotations

import math

import numpy as np

from core.interfaces import CourseMap, OccupancyGrid
from core.path_utils import resample_closed


# ===========================================================================
# Phase2 カンニング版（真値の中心線から）
# ===========================================================================
def build_course_map(course: dict, spacing: float = 0.05) -> CourseMap:
    """コース定義（真値）から CourseMap を生成する（Phase2カンニング版）。"""
    walls = course.get("walls", [])
    pts = []
    for (a, b) in walls:
        pts.append(a)
        pts.append(b)
    wall_pts = np.array(pts, dtype=np.float64) if pts else np.zeros((0, 2))

    cl = course.get("center_line")
    center = resample_closed(cl, spacing) if cl else np.zeros((0, 2))

    return CourseMap(
        left_wall=wall_pts, right_wall=wall_pts,
        center_line=center, racing_line=center.copy(),
        width_profile=np.zeros(len(center)),
    )


# ===========================================================================
# Phase3 占有格子からの抽出
# ===========================================================================
class CourseAnalyzer:
    """SLAM占有格子からコース境界・中心線を抽出する（Phase3）。"""

    def __init__(self, spacing: float = 0.05, max_halfwidth: float = 2.0,
                 smooth_window: int = 9, recenter_iters: int = 2):
        self.spacing = spacing            # 中心線リサンプル間隔 [m]
        self.max_halfwidth = max_halfwidth  # 片側レイマーチ最大距離 [m]
        self.smooth_window = smooth_window  # 平滑化窓（奇数）
        self.recenter_iters = recenter_iters

    # ------------------------------------------------------------------
    def analyze(self, grid: OccupancyGrid, seed_path=None) -> CourseMap:
        """占有格子＋探索軌跡(seed_path) から CourseMap を生成する。"""
        if seed_path is None or len(seed_path) < 3:
            raise ValueError("analyze には探索軌跡 seed_path（周回経路）が必要です")

        # 占有マスクを3x3膨張（壁の1セル隙間を塞いでレイマーチの抜けを防ぐ）
        mask = self._dilate(grid.grid == 1)

        # 1. シード軌跡を等間隔リサンプル＋平滑化（閉ループ）
        path = resample_closed(np.asarray(seed_path, dtype=np.float64), self.spacing)
        path = self._smooth_closed(path, self.smooth_window)

        # 2. 各点を法線方向のレイマーチで中心へ寄せる（数回反復）
        for _ in range(self.recenter_iters):
            normals = self._normals(path)
            new_path = path.copy()
            for i in range(len(path)):
                nx, ny = normals[i]
                d_plus = self._raymarch(mask, grid, path[i, 0], path[i, 1], nx, ny)
                d_minus = self._raymarch(mask, grid, path[i, 0], path[i, 1], -nx, -ny)
                offset = (d_plus - d_minus) / 2.0
                new_path[i, 0] = path[i, 0] + offset * nx
                new_path[i, 1] = path[i, 1] + offset * ny
            path = self._smooth_closed(new_path, self.smooth_window)

        # 3. 仕上げ: 再リサンプル＋幅算出
        center = resample_closed(path, self.spacing)
        center = self._smooth_closed(center, self.smooth_window)
        normals = self._normals(center)
        widths = np.zeros(len(center))
        for i in range(len(center)):
            nx, ny = normals[i]
            d_plus = self._raymarch(mask, grid, center[i, 0], center[i, 1], nx, ny)
            d_minus = self._raymarch(mask, grid, center[i, 0], center[i, 1], -nx, -ny)
            widths[i] = max(d_plus + d_minus, 0.0)

        left = center + normals * (widths[:, None] / 2.0)
        right = center - normals * (widths[:, None] / 2.0)

        return CourseMap(left_wall=left, right_wall=right, center_line=center,
                         racing_line=center.copy(), width_profile=widths)

    # ------------------------------------------------------------------
    def extract_walls(self, grid: OccupancyGrid) -> np.ndarray:
        """占有セルのワールド座標 (N,2) を返す（壁点群）。"""
        occ = np.argwhere(grid.grid == 1)  # (row=cy, col=cx)
        if occ.size == 0:
            return np.zeros((0, 2))
        wx = grid.origin_x + (occ[:, 1] + 0.5) * grid.resolution
        wy = grid.origin_y + (occ[:, 0] + 0.5) * grid.resolution
        return np.stack([wx, wy], axis=1)

    # ------------------------------------------------------------------
    def _raymarch(self, mask: np.ndarray, grid: OccupancyGrid, x: float, y: float,
                  dx: float, dy: float) -> float:
        """(x,y)から(dx,dy)方向へ占有(膨張マスク)までの距離[m]（最大max_halfwidth）。"""
        step = grid.resolution * 0.5
        n = int(self.max_halfwidth / step)
        ox, oy, res = grid.origin_x, grid.origin_y, grid.resolution
        h, w = mask.shape
        for k in range(1, n + 1):
            px = x + dx * step * k
            py = y + dy * step * k
            cx = int((px - ox) / res)
            cy = int((py - oy) / res)
            if cx < 0 or cx >= w or cy < 0 or cy >= h:
                return step * k          # 地図外は壁扱い
            if mask[cy, cx]:
                return step * k
        return self.max_halfwidth

    @staticmethod
    def _dilate(mask: np.ndarray) -> np.ndarray:
        """3x3 二値膨張（隣接シフトのOR）。"""
        m = mask.copy()
        for ax in (0, 1):
            m |= np.roll(mask, 1, axis=ax)
            m |= np.roll(mask, -1, axis=ax)
        # 斜め
        m |= np.roll(np.roll(mask, 1, 0), 1, 1)
        m |= np.roll(np.roll(mask, 1, 0), -1, 1)
        m |= np.roll(np.roll(mask, -1, 0), 1, 1)
        m |= np.roll(np.roll(mask, -1, 0), -1, 1)
        return m

    # ------------------------------------------------------------------
    @staticmethod
    def _normals(path: np.ndarray) -> np.ndarray:
        """閉ループ経路の各点の単位法線（接線を+90度回転）。"""
        nxt = np.roll(path, -1, axis=0)
        prv = np.roll(path, 1, axis=0)
        tang = nxt - prv
        norm = np.hypot(tang[:, 0], tang[:, 1])
        norm[norm < 1e-9] = 1.0
        tx, ty = tang[:, 0] / norm, tang[:, 1] / norm
        return np.stack([-ty, tx], axis=1)

    # ------------------------------------------------------------------
    @staticmethod
    def _smooth_closed(path: np.ndarray, window: int) -> np.ndarray:
        """閉ループの移動平均平滑化。"""
        if window < 3:
            return path
        k = window // 2
        n = len(path)
        out = np.empty_like(path)
        ext = np.vstack([path[-k:], path, path[:k]])
        kernel = np.ones(window) / window
        out[:, 0] = np.convolve(ext[:, 0], kernel, mode="valid")[:n]
        out[:, 1] = np.convolve(ext[:, 1], kernel, mode="valid")[:n]
        return out
