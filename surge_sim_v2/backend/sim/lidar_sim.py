"""レイキャスト LiDAR シミュレータ（LD06 模擬）。

コース壁（線分リスト）に対して numpy ベクトル演算でレイキャストする。

LD06 データシート準拠のセンサモデル：
  - 測距範囲: 0.02 m 〜 12 m
  - 測距精度: 約 ±1% クラス（距離依存）→ 1σ = max(noise_floor, noise_pct × 距離)
    （近距離は数 mm〜1cm、遠距離ほど大きくなる）
  - 角度分解能: 既定 1°/360点（LD06 実機は約0.8°/450点。config で変更可）
車両座標・heading を考慮した座標変換を行い、距離依存ガウスノイズを付加する。
"""
from __future__ import annotations

import time

import numpy as np

from core.interfaces import LidarScan, VehicleState
from core.shared_state import SharedState

MAX_RANGE = 12.0    # [m] LD06 最大測距
MIN_RANGE = 0.02    # [m] LD06 最小測距


class LidarSimulator:
    def __init__(self, walls: list, shared_state: SharedState,
                 noise_sigma: float = 0.02,
                 config: dict | None = None) -> None:
        self.shared = shared_state
        cfg = config or {}
        # LD06 データシート由来パラメータ
        self.max_range = float(cfg.get("max_range", MAX_RANGE))
        self.min_range = float(cfg.get("min_range", MIN_RANGE))
        # 距離依存ノイズ: σ(d) = max(noise_floor, noise_pct × d)
        self.noise_floor = float(cfg.get("noise_floor_m", noise_sigma))
        self.noise_pct = float(cfg.get("noise_pct", 0.006))    # ≒ ±1% クラス
        n_rays = int(cfg.get("n_rays", 360))
        self.n_rays = n_rays
        self._angles = np.linspace(0.0, 360.0, n_rays, endpoint=False)  # [deg]
        self._obstacles = np.zeros((0, 3))   # (K,3): x, y, r（人・車などの動的障害物）
        self.set_walls(walls)

    def set_obstacles(self, obstacles) -> None:
        """障害物（円）リスト [(x, y, r), ...] を設定。"""
        self._obstacles = np.array(obstacles, dtype=float).reshape(-1, 3) if len(obstacles) else np.zeros((0, 3))

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

        distances = np.full(self.n_rays, self.max_range, dtype=float)

        if self._p1.shape[0] > 0:
            distances = self._raycast(ox, oy, dirs)

        # 障害物（円）も測距し、より近い方を採用
        if self._obstacles.shape[0] > 0:
            distances = np.minimum(distances, self._raycast_circles(ox, oy, dirs))

        hit = distances < (self.max_range - 1e-3)

        # 距離依存ガウスノイズ（LD06 ≒ ±1% クラス）。ヒットのみに付加。
        sigma = np.maximum(self.noise_floor, self.noise_pct * distances)
        noise = np.random.normal(0.0, 1.0, size=self.n_rays) * sigma
        distances[hit] = distances[hit] + noise[hit]

        # 最小測距未満は無効（範囲外＝max_range 扱い）
        distances[distances < self.min_range] = self.max_range
        distances = np.clip(distances, 0.0, self.max_range)

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
        return np.where(np.isfinite(min_t),
                        np.minimum(min_t, self.max_range), self.max_range)

    def _raycast_circles(self, ox: float, oy: float, dirs: np.ndarray) -> np.ndarray:
        """全レイ × 全円のヒット距離（最近）。レイは単位方向 dirs。"""
        C = self._obstacles[:, :2]             # (K,2)
        r = self._obstacles[:, 2]              # (K,)
        dx = dirs[:, 0][:, None]               # (N,1)
        dy = dirs[:, 1][:, None]
        ocx = ox - C[:, 0][None, :]            # (1,K)
        ocy = oy - C[:, 1][None, :]
        b = 2.0 * (dx * ocx + dy * ocy)        # (N,K)
        c = (ocx ** 2 + ocy ** 2) - r[None, :] ** 2   # (1,K)
        disc = b * b - 4.0 * c                 # (N,K)
        with np.errstate(invalid="ignore"):
            sq = np.sqrt(np.where(disc >= 0, disc, np.nan))
            t = (-b - sq) / 2.0                # 手前の交点
        t = np.where((disc >= 0) & (t > 1e-6), t, np.inf)
        min_t = np.min(t, axis=1)
        return np.where(np.isfinite(min_t), min_t, self.max_range)
