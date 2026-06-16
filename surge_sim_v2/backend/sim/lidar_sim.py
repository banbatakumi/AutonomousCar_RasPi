"""レイキャスト LiDAR シミュレータ（LD06 模擬）。

コース壁（線分リスト）に対して numpy ベクトル演算でレイキャストする。
LD06 スペック: 360度・角度分解能1度・最大測定距離12m。
車両座標・heading を考慮した座標変換を行い、ガウスノイズを付加する。
"""
from __future__ import annotations

import time

import numpy as np

from core.interfaces import LidarScan, VehicleState
from core.shared_state import SharedState

MAX_RANGE = 12.0    # [m]
N_RAYS = 360
ANGLE_RES = 1.0     # [deg]


class LidarSimulator:
    def __init__(self, walls: list, shared_state: SharedState,
                 noise_sigma: float = 0.02) -> None:
        self.shared = shared_state
        self.noise_sigma = float(noise_sigma)
        self._angles = np.arange(N_RAYS, dtype=float) * ANGLE_RES  # [deg]
        self.set_walls(walls)

    def set_walls(self, walls: list) -> None:
        """壁線分リスト [((x1,y1),(x2,y2)), ...] を numpy 配列に変換。"""
        if walls:
            arr = np.array(walls, dtype=float)        # (M, 2, 2)
            self._p1 = arr[:, 0, :]                    # (M, 2)
            self._p2 = arr[:, 1, :]                    # (M, 2)
        else:
            self._p1 = np.zeros((0, 2))
            self._p2 = np.zeros((0, 2))

    def scan(self, vehicle: VehicleState) -> LidarScan:
        ox, oy = vehicle.x, vehicle.y
        # ワールド座標での各レイの角度（車両 heading + センサ相対角）
        ray_angles = np.radians(self._angles + vehicle.heading)
        dirs = np.stack([np.cos(ray_angles), np.sin(ray_angles)], axis=1)  # (N,2)

        distances = np.full(N_RAYS, MAX_RANGE, dtype=float)

        if self._p1.shape[0] > 0:
            distances = self._raycast(ox, oy, dirs)

        # ガウスノイズ付加（範囲内ヒットのみ）
        if self.noise_sigma > 0:
            hit = distances < MAX_RANGE
            noise = np.random.normal(0.0, self.noise_sigma, size=N_RAYS)
            distances[hit] = np.clip(distances[hit] + noise[hit], 0.0, MAX_RANGE)

        scan = LidarScan(
            distances=distances,
            angles=self._angles.copy(),
            timestamp=time.time(),
        )
        self.shared.update_lidar(scan)
        return scan

    def _raycast(self, ox: float, oy: float, dirs: np.ndarray) -> np.ndarray:
        """全レイ × 全線分のヒット距離を一括計算。

        レイ:  P = O + t * d   (t >= 0)
        線分:  Q = A + u * (B - A)  (0 <= u <= 1)
        を解く。形状 (N_rays, M_walls) でブロードキャスト。
        """
        A = self._p1                       # (M,2)
        B = self._p2
        seg = B - A                        # (M,2)

        d = dirs                           # (N,2)
        N = d.shape[0]
        M = A.shape[0]

        # 分母: cross(d, seg) = dx*segy - dy*segx -> (N,M)
        dx = d[:, 0][:, None]              # (N,1)
        dy = d[:, 1][:, None]
        segx = seg[:, 0][None, :]          # (1,M)
        segy = seg[:, 1][None, :]
        denom = dx * segy - dy * segx      # (N,M)

        ax = A[:, 0][None, :]              # (1,M)
        ay = A[:, 1][None, :]
        oax = ax - ox                      # (1,M) → (A - O).x
        oay = ay - oy

        # 平行レイ（denom=0）はゼロ除算になるが下の valid マスクで除外する
        with np.errstate(divide="ignore", invalid="ignore"):
            # t = cross((A-O), seg) / denom
            t_num = oax * segy - oay * segx    # (1,M)、denom(N,M) とブロードキャスト
            t = t_num / denom
            # u = cross((A-O), d) / denom
            u_num = oax * dy - oay * dx        # (1,M)*(N,1) → (N,M)
            u = u_num / denom

        eps = 1e-9
        valid = (np.abs(denom) > eps) & (t > eps) & (u >= -eps) & (u <= 1.0 + eps)
        t = np.where(valid, t, np.inf)

        min_t = np.min(t, axis=1)          # (N,)
        return np.where(np.isfinite(min_t), np.minimum(min_t, MAX_RANGE), MAX_RANGE)
