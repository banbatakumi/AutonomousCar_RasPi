"""レーシングライン最適化（Phase4）。

Minimum Curvature Path: 中心線まわりにコース幅内で横方向オフセット α_i を取り、
経路の離散曲率 Σ|P_{i-1}-2P_i+P_{i+1}|² を最小化する。閉ループの二階差分作用素 D を
用いた線形最小二乗で解き、コース幅（安全マージン込み）で α をクリップする。

速度プロファイル: 各点の曲率から横加速度上限 a_lat で v=√(a_lat/κ) を求め、
前後パス(forward-backward)で縦加速度上限 a_long を満たすよう制限する。
"""

from __future__ import annotations

import numpy as np

from core.interfaces import CourseMap
from core.path_utils import path_curvature


class RacingLineOptimizer:
    """最小曲率レーシングライン最適化。"""

    def __init__(self, safety_margin: float = 0.15,
                 a_lat_max: float = 4.0, a_long_max: float = 3.0,
                 min_speed: float = 0.5):
        self.safety_margin = safety_margin   # 壁からの安全マージン [m]
        self.a_lat_max = a_lat_max           # 横加速度上限 [m/s^2]
        self.a_long_max = a_long_max         # 縦加速度上限 [m/s^2]
        self.min_speed = min_speed

    # ------------------------------------------------------------------
    def optimize(self, course_map: CourseMap) -> np.ndarray:
        """中心線＋コース幅から最小曲率レーシングラインを生成する。"""
        center = np.asarray(course_map.center_line, dtype=np.float64)
        n = len(center)
        if n < 5:
            return center.copy()

        normals = self._normals(center)
        widths = np.asarray(course_map.width_profile, dtype=np.float64)
        if widths.shape[0] != n:
            widths = np.full(n, 1.0)
        amax = np.maximum(widths / 2.0 - self.safety_margin, 0.0)

        # 閉ループ二階差分作用素 D (循環)
        D = (np.diag(np.full(n, -2.0))
             + np.diag(np.ones(n - 1), 1) + np.diag(np.ones(n - 1), -1))
        D[0, -1] = 1.0
        D[-1, 0] = 1.0

        nx, ny = normals[:, 0], normals[:, 1]
        Cx, Cy = center[:, 0], center[:, 1]
        Gx = D * nx[None, :]      # D @ diag(nx)
        Gy = D * ny[None, :]
        A = Gx.T @ Gx + Gy.T @ Gy
        b = -(Gx.T @ (D @ Cx) + Gy.T @ (D @ Cy))
        # 数値安定化（わずかな正則化）
        A += 1e-6 * np.eye(n)

        try:
            alpha = np.linalg.solve(A, b)
        except np.linalg.LinAlgError:
            alpha = np.zeros(n)

        alpha = np.clip(alpha, -amax, amax)
        racing = center + normals * alpha[:, None]
        # クリップ由来の微小キンクを除去（軽い閉ループ平滑化）
        racing = self._smooth_closed(racing, 5)
        return racing

    # ------------------------------------------------------------------
    def compute_speed_profile(self, racing_line: np.ndarray,
                              max_speed: float) -> np.ndarray:
        """レーシングライン各点の許容速度[m/s]を返す（閉ループ前後パス）。"""
        path = np.asarray(racing_line, dtype=np.float64)
        n = len(path)
        if n < 3:
            return np.full(n, self.min_speed)

        kappa = path_curvature(path)
        # 曲率からの上限速度
        with np.errstate(divide="ignore"):
            v_curve = np.sqrt(self.a_lat_max / np.maximum(kappa, 1e-6))
        v = np.minimum(v_curve, max_speed)

        # セグメント長
        seg = np.roll(path, -1, axis=0) - path
        ds = np.hypot(seg[:, 0], seg[:, 1])

        # 前進パス（加速制限）: v_{i+1}² ≤ v_i² + 2 a_long ds_i
        for _ in range(2):  # 閉ループなので2周回して収束
            for i in range(n):
                j = (i + 1) % n
                vmax = np.sqrt(v[i] ** 2 + 2 * self.a_long_max * ds[i])
                if v[j] > vmax:
                    v[j] = vmax
        # 後退パス（減速制限）: v_i² ≤ v_{i+1}² + 2 a_long ds_i
        for _ in range(2):
            for i in range(n - 1, -1, -1):
                j = (i + 1) % n
                vmax = np.sqrt(v[j] ** 2 + 2 * self.a_long_max * ds[i])
                if v[i] > vmax:
                    v[i] = vmax

        return np.maximum(v, self.min_speed)

    # ------------------------------------------------------------------
    @staticmethod
    def _normals(path: np.ndarray) -> np.ndarray:
        nxt = np.roll(path, -1, axis=0)
        prv = np.roll(path, 1, axis=0)
        tang = nxt - prv
        norm = np.hypot(tang[:, 0], tang[:, 1])
        norm[norm < 1e-9] = 1.0
        return np.stack([-tang[:, 1] / norm, tang[:, 0] / norm], axis=1)

    @staticmethod
    def _smooth_closed(path: np.ndarray, window: int) -> np.ndarray:
        if window < 3:
            return path
        k = window // 2
        n = len(path)
        ext = np.vstack([path[-k:], path, path[:k]])
        kernel = np.ones(window) / window
        out = np.empty_like(path)
        out[:, 0] = np.convolve(ext[:, 0], kernel, mode="valid")[:n]
        out[:, 1] = np.convolve(ext[:, 1], kernel, mode="valid")[:n]
        return out
