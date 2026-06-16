"""レーシングライン最適化（Phase4）。

Minimum Curvature Path（最小曲率パス）による最適化。
中心線からの横オフセット α を変数に、循環二階差分作用素 D で曲率を表し、
Σ‖D·P‖² を最小化する線形最小二乗で解く（numpy のみ、scipy 不要）。
α はコース幅−安全マージン内にクリップする。

速度プロファイルは横加速度制限から各点の上限速度を求め、
前後（加速・減速）パスで縦加速度制限を満たすよう整形する。
"""
from __future__ import annotations

import numpy as np

from .interfaces import CourseMap
from .path_utils import path_curvature, path_normals


class RacingLineOptimizer:
    def __init__(self, config: dict | None = None) -> None:
        cfg = config or {}
        self.safety_margin = float(cfg.get("safety_margin", 0.15))   # [m] 壁からの余裕
        self.a_lat = float(cfg.get("a_lat", 4.0))                    # [m/s²] 横加速度上限
        self.a_long = float(cfg.get("a_long", 3.0))                  # [m/s²] 縦加速度上限
        self.max_speed = float(cfg.get("max_speed", 3.0))
        self.min_speed = float(cfg.get("min_speed", 0.5))
        self.smooth_iters = int(cfg.get("smooth_iters", 2))
        self.pre_smooth_iters = int(cfg.get("pre_smooth_iters", 3))

    # ---- レーシングライン最適化 ------------------------------------------
    def optimize(self, course_map: CourseMap) -> np.ndarray:
        center = np.asarray(course_map.center_line, dtype=float)
        N = len(center)
        if N < 5:
            return center.copy()

        # SLAM 由来の中心線はノイジーなので、最適化の基準線・法線は平滑化版を使う
        # （ノイジーな法線のまま最適化すると逆に波打つため）
        base = center
        for _ in range(self.pre_smooth_iters):
            base = self._smooth_closed(base)
        center = base

        n = path_normals(center)            # 左向き単位法線 (N,2)
        nx, ny = n[:, 0], n[:, 1]
        Cx, Cy = center[:, 0], center[:, 1]

        # 循環二階差分作用素 D（閉ループ曲率の近似）
        eye = np.eye(N)
        D = -2.0 * eye + np.roll(eye, 1, axis=1) + np.roll(eye, -1, axis=1)

        # P = C + diag(n)·α  →  D·Px = D·Cx + (D·diag(nx))·α
        Ax = D * nx[None, :]                # = D @ diag(nx)
        Ay = D * ny[None, :]
        H = Ax.T @ Ax + Ay.T @ Ay + 1e-6 * eye
        g = Ax.T @ (D @ Cx) + Ay.T @ (D @ Cy)
        alpha = np.linalg.solve(H, -g)

        # コース幅−安全マージンでクリップ
        w = self._width_to_n(course_map.width_profile, N)
        bound = np.maximum(w / 2.0 - self.safety_margin, 0.0)
        alpha = np.clip(alpha, -bound, bound)

        racing = center + n * alpha[:, None]
        for _ in range(self.smooth_iters):
            racing = self._smooth_closed(racing)
        return racing

    # ---- 速度プロファイル ------------------------------------------------
    def compute_speed_profile(self, racing_line: np.ndarray,
                              max_speed: float | None = None) -> np.ndarray:
        mx = self.max_speed if max_speed is None else float(max_speed)
        pts = np.asarray(racing_line, dtype=float)
        N = len(pts)
        if N < 3:
            return np.full(N, self.min_speed)

        kappa = path_curvature(pts)
        ds = np.hypot(*(np.roll(pts, -1, axis=0) - pts).T)   # ds[i]: i→i+1 区間長

        # 横加速度制限: v = sqrt(a_lat / κ)
        v = np.sqrt(self.a_lat / np.maximum(kappa, 1e-3))
        v = np.minimum(v, mx)

        # 前後パス（閉ループ、数回反復で収束）
        for _ in range(2):
            for i in range(N):                  # 前方（加速制限）
                j = (i + 1) % N
                v[j] = min(v[j], np.sqrt(v[i] ** 2 + 2.0 * self.a_long * ds[i]))
            for i in range(N - 1, -1, -1):       # 後方（減速制限）
                j = (i - 1) % N
                v[j] = min(v[j], np.sqrt(v[i] ** 2 + 2.0 * self.a_long * ds[j]))

        return np.maximum(v, self.min_speed)

    # ---- 内部ヘルパ -------------------------------------------------------
    @staticmethod
    def _width_to_n(width_profile, N: int) -> np.ndarray:
        if width_profile is None or len(width_profile) == 0:
            return np.full(N, 1.0)
        w = np.asarray(width_profile, dtype=float)
        if len(w) == N:
            return w
        # 長さが違えば弧パラメータで補間して合わせる
        src = np.linspace(0.0, 1.0, len(w), endpoint=False)
        dst = np.linspace(0.0, 1.0, N, endpoint=False)
        return np.interp(dst, src, w)

    @staticmethod
    def _smooth_closed(pts: np.ndarray, k: int = 5) -> np.ndarray:
        if len(pts) < k:
            return pts
        kernel = np.ones(k) / k
        x = np.convolve(np.r_[pts[-k:, 0], pts[:, 0], pts[:k, 0]], kernel, mode="same")
        y = np.convolve(np.r_[pts[-k:, 1], pts[:, 1], pts[:k, 1]], kernel, mode="same")
        return np.column_stack([x[k:-k], y[k:-k]])
