"""レイキャスト方式 LiDAR シミュレータ（LD06相当）。

コース壁（線分リスト）に対し numpy ベクトル演算でレイキャストを行い、
360度・1度分解能・最大12mのスキャンを生成する。ガウスノイズを付加する。

角度の定義:
    angles[i] = i [deg]、車両 heading を基準とした反時計回りの相対角。
    angle=0 は車両前方(heading方向)。
"""

from __future__ import annotations

import math

import numpy as np

from core.interfaces import LidarScan, VehicleState


class LidarSimulator:
    """線分コースに対するレイキャストLiDAR。"""

    NUM_RAYS = 360
    ANGLE_RESOLUTION = 1.0   # [deg]
    MAX_RANGE = 12.0         # [m]

    def __init__(self, walls: list, noise_sigma: float = 0.02):
        """
        Args:
            walls: 線分リスト [((x1, y1), (x2, y2)), ...]
            noise_sigma: ガウスノイズの標準偏差 [m]
        """
        self.noise_sigma = float(noise_sigma)
        # 相対角度（車両前方基準）: 0..359 deg
        self.rel_angles = np.arange(self.NUM_RAYS, dtype=np.float64) * self.ANGLE_RESOLUTION
        self._rel_angles_rad = np.radians(self.rel_angles)
        self.set_walls(walls)

    # ------------------------------------------------------------------
    def set_walls(self, walls: list) -> None:
        """壁線分を numpy 配列へ変換して保持する。"""
        if not walls:
            self._A = np.zeros((0, 2))
            self._E = np.zeros((0, 2))
            return
        a = np.array([[s[0][0], s[0][1]] for s in walls], dtype=np.float64)  # 始点
        b = np.array([[s[1][0], s[1][1]] for s in walls], dtype=np.float64)  # 終点
        self._A = a            # shape(M,2)
        self._E = b - a        # shape(M,2) 線分ベクトル

    # ------------------------------------------------------------------
    def scan(self, state: VehicleState, timestamp: float | None = None) -> LidarScan:
        """車両状態から1スキャンを生成する。"""
        if timestamp is None:
            timestamp = state.timestamp

        ox, oy = state.x, state.y
        heading_rad = math.radians(state.heading)

        # 各レイのワールド方向（単位ベクトル）  shape(R,2)
        world_ang = self._rel_angles_rad + heading_rad
        dx = np.cos(world_ang)
        dy = np.sin(world_ang)

        distances = np.full(self.NUM_RAYS, self.MAX_RANGE, dtype=np.float64)

        if self._A.shape[0] > 0:
            distances = self._raycast(ox, oy, dx, dy)

        # ガウスノイズ（有効レンジのみ）
        if self.noise_sigma > 0.0:
            noise = np.random.normal(0.0, self.noise_sigma, size=self.NUM_RAYS)
            hit = distances < self.MAX_RANGE
            distances[hit] = np.clip(distances[hit] + noise[hit], 0.0, self.MAX_RANGE)

        return LidarScan(
            distances=distances,
            angles=self.rel_angles.copy(),
            timestamp=timestamp,
        )

    # ------------------------------------------------------------------
    def _raycast(self, ox: float, oy: float,
                 dx: np.ndarray, dy: np.ndarray) -> np.ndarray:
        """全レイ × 全線分のレイキャスト。最近接ヒット距離を返す。

        ray:  P = O + t * d           (t >= 0)
        seg:  Q = A + u * E           (0 <= u <= 1)
        t = cross(A - O, E) / cross(d, E)
        u = cross(A - O, d) / cross(d, E)
        cross(p, q) = p_x*q_y - p_y*q_x
        """
        R = dx.shape[0]
        best = np.full(R, self.MAX_RANGE, dtype=np.float64)

        eps = 1e-12
        for i in range(self._A.shape[0]):
            ax, ay = self._A[i]
            ex, ey = self._E[i]

            denom = dx * ey - dy * ex                # cross(d, E)  shape(R,)
            wx = ax - ox
            wy = ay - oy

            # ゼロ割回避
            safe = np.abs(denom) > eps
            denom_safe = np.where(safe, denom, 1.0)

            t = (wx * ey - wy * ex) / denom_safe     # ray パラメータ（= 距離, dは単位）
            u = (wx * dy - wy * dx) / denom_safe     # seg パラメータ

            valid = safe & (t >= 0.0) & (u >= 0.0) & (u <= 1.0) & (t < best)
            best = np.where(valid, t, best)

        return best
