"""LD06 LiDARデータの処理・フィルタリング。

実機・シミュレータ共通で利用する。スキャンのノイズ除去、無効点の除外、
車両座標系／ワールド座標系への点群変換を提供する。
"""

from __future__ import annotations

import numpy as np

from core.interfaces import LidarScan, LocalizationResult


class LidarProcessor:
    """LiDARスキャンのフィルタリングと座標変換。"""

    def __init__(self, max_range: float = 12.0, min_range: float = 0.05):
        self.max_range = float(max_range)
        self.min_range = float(min_range)

    # ------------------------------------------------------------------
    def filter(self, scan: LidarScan) -> LidarScan:
        """無効点(範囲外)をmax_rangeへクリップし、メディアンで平滑化する。"""
        d = scan.distances.astype(np.float64).copy()

        # 範囲外・異常値の処理
        invalid = (d < self.min_range) | (d > self.max_range) | ~np.isfinite(d)
        d[invalid] = self.max_range

        # 3点メディアンフィルタ（円環）でスパイク除去
        d_filtered = self._median3_circular(d)

        return LidarScan(distances=d_filtered, angles=scan.angles.copy(),
                         timestamp=scan.timestamp)

    # ------------------------------------------------------------------
    @staticmethod
    def _median3_circular(d: np.ndarray) -> np.ndarray:
        """円環方向の3点メディアンフィルタ。"""
        prev = np.roll(d, 1)
        nxt = np.roll(d, -1)
        stacked = np.stack([prev, d, nxt], axis=0)
        return np.median(stacked, axis=0)

    # ------------------------------------------------------------------
    def to_points_vehicle(self, scan: LidarScan,
                          drop_max_range: bool = True) -> np.ndarray:
        """車両座標系の点群 shape(K,2) を返す（x前方, y左）。

        angles は車両前方基準・反時計回り[deg]。
        """
        ang = np.radians(scan.angles)
        d = scan.distances
        xs = d * np.cos(ang)
        ys = d * np.sin(ang)
        pts = np.stack([xs, ys], axis=1)
        if drop_max_range:
            mask = d < (self.max_range - 1e-6)
            pts = pts[mask]
        return pts

    # ------------------------------------------------------------------
    def to_points_world(self, scan: LidarScan, pose: LocalizationResult,
                        drop_max_range: bool = True) -> np.ndarray:
        """ワールド座標系の点群 shape(K,2) を返す。"""
        ang = np.radians(scan.angles) + np.radians(pose.heading)
        d = scan.distances
        xs = pose.x + d * np.cos(ang)
        ys = pose.y + d * np.sin(ang)
        pts = np.stack([xs, ys], axis=1)
        if drop_max_range:
            mask = d < (self.max_range - 1e-6)
            pts = pts[mask]
        return pts

    # ------------------------------------------------------------------
    @staticmethod
    def min_distance(scan: LidarScan) -> float:
        """最小距離[m]を返す（前方衝突回避などに利用）。"""
        if scan.distances.size == 0:
            return float("inf")
        return float(np.min(scan.distances))
