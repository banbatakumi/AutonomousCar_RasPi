"""log-odds 占有格子マッパー（Phase3 マッピングの心臓部）。

LiDAR スキャンと姿勢から占有格子を逐次更新する。
- 各ビーム終点を occupied、ロボット〜終点間を free として log-odds を加減算
- 自由空間レイトレースはレイに沿った等間隔サンプリング（解像度刻み）
- to_occupancy_grid() で 0=free / 100=occupied / -1=unknown の OccupancyGrid に変換

シミュでは CheatLocalizer の真姿勢を使う「既知姿勢マッピング」。
実機では SLAMLocalizer の推定姿勢を使う（[[project-surge-sim-v2]] の方針）。
"""
from __future__ import annotations

import math
import time

import numpy as np

from .interfaces import LidarScan, OccupancyGrid

MAX_RANGE = 12.0  # [m] LD06


class OccupancyGridMapper:
    def __init__(self, resolution: float = 0.05,
                 bounds: tuple[float, float, float, float] = (-1.0, -1.0, 7.0, 5.0),
                 l_occ: float = 0.85, l_free: float = 0.4,
                 l_min: float = -4.0, l_max: float = 4.0,
                 occ_thresh: float = 0.5, free_thresh: float = -0.5,
                 max_range: float = MAX_RANGE) -> None:
        self.resolution = float(resolution)
        self.l_occ = l_occ
        self.l_free = l_free
        self.l_min = l_min
        self.l_max = l_max
        self.occ_thresh = occ_thresh
        self.free_thresh = free_thresh
        self.max_range = max_range
        self.set_bounds(bounds)

    # ---- グリッド寸法 -----------------------------------------------------
    def set_bounds(self, bounds: tuple[float, float, float, float]) -> None:
        xmin, ymin, xmax, ymax = bounds
        self.origin_x = float(xmin)
        self.origin_y = float(ymin)
        self.width = max(1, int(math.ceil((xmax - xmin) / self.resolution)))
        self.height = max(1, int(math.ceil((ymax - ymin) / self.resolution)))
        self.log_odds = np.zeros((self.height, self.width), dtype=np.float32)

    def reset(self) -> None:
        self.log_odds.fill(0.0)

    # ---- 座標変換 ---------------------------------------------------------
    def world_to_cell(self, x, y):
        col = ((np.asarray(x) - self.origin_x) / self.resolution).astype(int)
        row = ((np.asarray(y) - self.origin_y) / self.resolution).astype(int)
        return col, row

    def _in_bounds(self, col, row):
        return (col >= 0) & (col < self.width) & (row >= 0) & (row < self.height)

    # ---- スキャン統合 -----------------------------------------------------
    def integrate_scan(self, scan: LidarScan, pose, only_unknown: bool = False,
                       freeze_thresh: float = 2.0) -> None:
        """LiDAR スキャンを姿勢 pose（x,y,heading[deg]）で地図に反映する。

        only_unknown=True のとき、既に確定済み（|log_odds|>=freeze_thresh）の
        セルは更新しない。これにより一度作った良い地図を壊さず、未知領域だけを
        埋めていける（自己位置推定の地図破損による発散を防ぐ）。
        """
        x, y = float(pose.x), float(pose.y)
        heading = math.radians(float(pose.heading))

        d = np.asarray(scan.distances, dtype=float)
        ang = np.radians(np.asarray(scan.angles, dtype=float)) + heading
        cos_a = np.cos(ang)
        sin_a = np.sin(ang)

        hit = d < (self.max_range - 1e-3)

        # --- 自由空間：各レイをサンプリングして free 更新 ---
        free_cols = []
        free_rows = []
        step = self.resolution
        for i in range(len(d)):
            ray_len = d[i] if hit[i] else self.max_range
            if ray_len <= step:
                continue
            rs = np.arange(0.0, ray_len - step * 0.5, step)
            fx = x + rs * cos_a[i]
            fy = y + rs * sin_a[i]
            c, r = self.world_to_cell(fx, fy)
            free_cols.append(c)
            free_rows.append(r)

        if free_cols:
            fc = np.concatenate(free_cols)
            fr = np.concatenate(free_rows)
            m = self._in_bounds(fc, fr)
            fr, fc = fr[m], fc[m]
            if only_unknown:
                keep = np.abs(self.log_odds[fr, fc]) < freeze_thresh
                fr, fc = fr[keep], fc[keep]
            np.add.at(self.log_odds, (fr, fc), -self.l_free)

        # --- 占有：ヒット終点を occupied 更新 ---
        ex = x + d * cos_a
        ey = y + d * sin_a
        oc, orr = self.world_to_cell(ex[hit], ey[hit])
        m = self._in_bounds(oc, orr)
        orr, oc = orr[m], oc[m]
        if only_unknown:
            keep = np.abs(self.log_odds[orr, oc]) < freeze_thresh
            orr, oc = orr[keep], oc[keep]
        np.add.at(self.log_odds, (orr, oc), self.l_occ)

        np.clip(self.log_odds, self.l_min, self.l_max, out=self.log_odds)

    # ---- 出力 -------------------------------------------------------------
    def to_occupancy_grid(self) -> OccupancyGrid:
        grid = np.full((self.height, self.width), -1, dtype=np.int8)
        grid[self.log_odds > self.occ_thresh] = 100
        grid[self.log_odds < self.free_thresh] = 0
        return OccupancyGrid(
            grid=grid,
            resolution=self.resolution,
            origin_x=self.origin_x,
            origin_y=self.origin_y,
            timestamp=time.time(),
        )

    def occupied_points(self) -> np.ndarray:
        """占有セルのワールド座標 shape(K,2) を返す。"""
        rows, cols = np.where(self.log_odds > self.occ_thresh)
        xs = self.origin_x + (cols + 0.5) * self.resolution
        ys = self.origin_y + (rows + 0.5) * self.resolution
        return np.column_stack([xs, ys])

    # ---- 保存・読込 -------------------------------------------------------
    def save(self, path: str) -> None:
        np.savez_compressed(
            path,
            log_odds=self.log_odds,
            resolution=self.resolution,
            origin_x=self.origin_x,
            origin_y=self.origin_y,
        )

    def load(self, path: str) -> None:
        data = np.load(path)
        self.log_odds = data["log_odds"].astype(np.float32)
        self.resolution = float(data["resolution"])
        self.origin_x = float(data["origin_x"])
        self.origin_y = float(data["origin_y"])
        self.height, self.width = self.log_odds.shape
