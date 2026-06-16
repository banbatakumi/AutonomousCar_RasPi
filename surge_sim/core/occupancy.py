"""占有格子マッピング（log-odds）。

LiDARスキャンと姿勢から占有格子地図を逐次構築する。実機・SIM共通で、
「既知姿勢でのマッピング」にも「SLAM推定姿勢でのマッピング」にも使える。

更新則: 各ビームについて
  - 始点〜終点手前のセル: free 更新 (log-odds 減算)
  - 終点セル(レンジ内): occupied 更新 (log-odds 加算)
ビームが最大レンジなら終点は occupied 更新しない（壁が無い方向）。

出力 OccupancyGrid: 0=free, 1=occupied, -1=unknown。
"""

from __future__ import annotations

import math

import numpy as np

from core.interfaces import LidarScan, OccupancyGrid


class OccupancyGridMapper:
    """log-odds 占有格子マッパー。"""

    def __init__(self, min_x: float, min_y: float, max_x: float, max_y: float,
                 resolution: float = 0.05, max_range: float = 12.0,
                 l_occ: float = 0.85, l_free: float = 0.4, l_clamp: float = 6.0,
                 occ_thresh: float = 0.5, free_thresh: float = -0.5):
        self.resolution = float(resolution)
        self.origin_x = float(min_x)
        self.origin_y = float(min_y)
        self.max_range = float(max_range)
        self.l_occ = l_occ
        self.l_free = l_free
        self.l_clamp = l_clamp
        self.occ_thresh = occ_thresh
        self.free_thresh = free_thresh

        self.w = max(int(math.ceil((max_x - min_x) / resolution)), 1)
        self.h = max(int(math.ceil((max_y - min_y) / resolution)), 1)
        self.log = np.zeros((self.h, self.w), dtype=np.float32)  # log-odds

    # ------------------------------------------------------------------
    def reset(self) -> None:
        self.log.fill(0.0)

    # ------------------------------------------------------------------
    def world_to_cell(self, x: float, y: float) -> tuple[int, int]:
        cx = int((x - self.origin_x) / self.resolution)
        cy = int((y - self.origin_y) / self.resolution)
        return cx, cy

    def _in_bounds(self, cx: int, cy: int) -> bool:
        return 0 <= cx < self.w and 0 <= cy < self.h

    # ------------------------------------------------------------------
    def integrate_scan(self, scan: LidarScan, pose) -> None:
        """1スキャンを姿勢(pose: x,y,heading[deg])で地図へ統合する。"""
        ox, oy = pose.x, pose.y
        ocx, ocy = self.world_to_cell(ox, oy)
        h_rad = math.radians(pose.heading)

        ang = np.radians(scan.angles) + h_rad
        d = scan.distances
        cos_a = np.cos(ang)
        sin_a = np.sin(ang)

        for i in range(d.shape[0]):
            di = d[i]
            hit = di < (self.max_range - 1e-3)
            end = di if hit else self.max_range
            ex = ox + end * cos_a[i]
            ey = oy + end * sin_a[i]
            ecx, ecy = self.world_to_cell(ex, ey)

            # 始点→終点手前を free 更新（Bresenham）
            for (cx, cy) in self._bresenham(ocx, ocy, ecx, ecy):
                if (cx, cy) == (ecx, ecy):
                    break
                if self._in_bounds(cx, cy):
                    self.log[cy, cx] = max(self.log[cy, cx] - self.l_free, -self.l_clamp)

            # 終点を occupied 更新（レンジ内のみ）
            if hit and self._in_bounds(ecx, ecy):
                self.log[ecy, ecx] = min(self.log[ecy, ecx] + self.l_occ, self.l_clamp)

    # ------------------------------------------------------------------
    @staticmethod
    def _bresenham(x0: int, y0: int, x1: int, y1: int):
        """整数ブレゼンハム直線。(x0,y0)から(x1,y1)のセルを順に返す。"""
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx - dy
        x, y = x0, y0
        while True:
            yield (x, y)
            if x == x1 and y == y1:
                break
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x += sx
            if e2 < dx:
                err += dx
                y += sy

    # ------------------------------------------------------------------
    def to_occupancy_grid(self, timestamp: float = 0.0) -> OccupancyGrid:
        """log-odds を 0/1/-1 の占有格子へ量子化して返す。"""
        grid = np.full((self.h, self.w), -1, dtype=np.int8)
        grid[self.log > self.occ_thresh] = 1
        grid[self.log < self.free_thresh] = 0
        return OccupancyGrid(grid=grid, resolution=self.resolution,
                             origin_x=self.origin_x, origin_y=self.origin_y,
                             timestamp=timestamp)

    # ------------------------------------------------------------------
    def is_occupied_world(self, x: float, y: float) -> bool:
        """ワールド座標が occupied セルか（レイマーチ用、未知は非occupied扱い）。"""
        cx, cy = self.world_to_cell(x, y)
        if not self._in_bounds(cx, cy):
            return True   # 範囲外は壁扱い（安全側）
        return self.log[cy, cx] > self.occ_thresh
