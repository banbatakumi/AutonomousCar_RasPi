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
        self.safety_margin = float(cfg.get("safety_margin", 0.15))   # [m] 車体端から壁への余裕
        self.vehicle_half_width = float(cfg.get("vehicle_half_width", 0.10))  # [m] 車体半幅
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

        # 許容横オフセット = 半幅 − 安全マージン − 車体半幅（車体端が壁から margin 残る）
        # 幅は最小値フィルタ（収縮）してから使う：局所的に幅が過大推定された点や
        # この先で狭くなる点で、オフセットが壁に寄りすぎる（外れる）のを防ぐ。
        w = self._width_to_n(course_map.width_profile, N)
        w = self._min_filter_closed(w, k=5)
        bound = np.maximum(w / 2.0 - self.safety_margin - self.vehicle_half_width, 0.0)
        alpha = np.clip(alpha, -bound, bound)

        # オフセットを平滑化（急な横移動を抑制）→ 最後に再クリップして必ず境界内に収める
        for _ in range(self.smooth_iters):
            alpha = self._smooth_scalar_closed(alpha)
        alpha = np.clip(alpha, -bound, bound)

        racing = center + n * alpha[:, None]

        # 安全策：中心線が既に最適に近い（滑らかな）コースでは、クリップ等で
        # かえって曲率が増えることがある。中心線より悪ければ中心線を採用する。
        if self._curv_energy(racing) > self._curv_energy(center):
            return center
        return racing

    @staticmethod
    def _curv_energy(pts: np.ndarray) -> float:
        return float(np.sum(path_curvature(pts) ** 2))

    # ---- 速度プロファイル ------------------------------------------------
    def compute_speed_profile(self, racing_line: np.ndarray,
                              max_speed: float | None = None) -> np.ndarray:
        mx = self.max_speed if max_speed is None else float(max_speed)
        pts = np.asarray(racing_line, dtype=float)
        N = len(pts)
        if N < 3:
            return np.full(N, self.min_speed)

        kappa = path_curvature(pts)
        kappa = self._smooth_scalar_closed(kappa, k=5)   # 離散外接円のスパイクを抑制
        ds = np.hypot(*(np.roll(pts, -1, axis=0) - pts).T)   # ds[i]: i→i+1 区間長

        # 横加速度制限: v = sqrt(a_lat / κ)
        v = np.sqrt(self.a_lat / np.maximum(np.abs(kappa), 1e-3))
        v = np.minimum(v, mx)

        # 前後パス（閉ループ、収束するまで反復＝最大 N 回で打ち切り）
        for _ in range(min(N, 8)):
            v_prev = v.copy()
            for i in range(N):                  # 前方（加速制限）
                j = (i + 1) % N
                v[j] = min(v[j], np.sqrt(v[i] ** 2 + 2.0 * self.a_long * ds[i]))
            for i in range(N - 1, -1, -1):       # 後方（減速制限）
                j = (i - 1) % N
                v[j] = min(v[j], np.sqrt(v[i] ** 2 + 2.0 * self.a_long * ds[j]))
            if np.max(np.abs(v - v_prev)) < 1e-3:
                break

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

    @staticmethod
    def _min_filter_closed(a: np.ndarray, k: int = 5) -> np.ndarray:
        """閉ループのスカラー列に最小値フィルタ（収縮）。各点を近傍 ±k//2 の最小値に。"""
        a = np.asarray(a, dtype=float)
        n = len(a)
        if n < k:
            return a
        h = k // 2
        out = a.copy()
        for off in range(-h, h + 1):
            out = np.minimum(out, np.roll(a, off))
        return out

    @staticmethod
    def _smooth_scalar_closed(a: np.ndarray, k: int = 5) -> np.ndarray:
        """閉ループのスカラー列（オフセット α や曲率）を移動平均で平滑化。"""
        a = np.asarray(a, dtype=float)
        if len(a) < k:
            return a
        kernel = np.ones(k) / k
        padded = np.r_[a[-k:], a, a[:k]]
        sm = np.convolve(padded, kernel, mode="same")
        return sm[k:-k]
