"""自己位置推定。

- CheatLocalizer: シミュレータの真値をそのまま使う（デバッグ・比較用）
- SLAMLocalizer:  Hector 風スキャンマッチで占有格子に合わせ込み、LiDAR のみで自己位置推定

SLAMLocalizer は実機相当の自己位置推定。車輪オドメトリは使わず、
- 等速度予測（直前の推定移動量からの外挿）で初期姿勢を与え
- マルチ解像度（粗→細でボカした確率場）の Gauss-Newton で合わせ込む
ことで頑健化している。

本コース（6×4m）は LiDAR 最大測距 12m 内に全体が収まり、どこからでもコーナーが
見えるため前後方向も拘束され、純 LiDAR-SLAM でも退化せず安定して推定できる。
"""
from __future__ import annotations

import math
import time

import numpy as np

from .interfaces import LidarScan, LocalizationResult, OccupancyGrid, VehicleState
from .occupancy import MAX_RANGE


def _wrap180(deg: float) -> float:
    return (deg + 180.0) % 360.0 - 180.0


class CheatLocalizer:
    """シミュレータから真の位置を直接取得（デバッグ・比較用）。"""

    def update(self, true_state: VehicleState) -> LocalizationResult:
        return LocalizationResult(
            x=true_state.x, y=true_state.y, heading=true_state.heading,
            confidence=1.0, source="cheat", timestamp=true_state.timestamp,
        )


class SLAMLocalizer:
    """Hector 風スキャンマッチによる自己位置推定（LiDAR のみ）。

    占有格子の確率場 M(x) に対し、スキャン終点 S_i(ξ) が壁（M=1）に乗るよう
    ξ=(x, y, θ) を Gauss-Newton で最適化する。等速度予測で初期値を与え、
    粗→細のボカし段階で局所解を回避しつつ収束範囲を広げる。
    """

    def __init__(self, mapper,
                 start_pose: tuple[float, float, float] = (0.0, 0.0, 0.0),
                 blur_sigmas_m: tuple[float, ...] = (0.15, 0.075, 0.0),  # [m] 解像度非依存
                 iters: tuple[int, ...] = (6, 6, 6),
                 damping: float = 1e-3,
                 max_step_xy: float = 0.25, max_step_th: float = 0.20,
                 prior_xy: float = 2000.0, prior_th: float = 400.0,
                 prior_th_imu: float = 8000.0) -> None:
        self.mapper = mapper
        self.x, self.y, self.heading = start_pose
        self.px, self.py, self.ph = start_pose       # 直前姿勢（等速度予測用）
        self.blur_sigmas_m = blur_sigmas_m           # [m] 単位（マッチ時にセルへ換算）
        self.iters = iters
        self.damping = damping
        self.max_step_xy = max_step_xy
        self.max_step_th = max_step_th
        # 運動モデル事前分布の重み（縦方向など拘束不足な方向を予測で補う）
        self.prior_xy = prior_xy
        self.prior_th = prior_th
        # IMU ヨーを事前分布中心に使うときの重み（運動予測より強く信頼）
        self.prior_th_imu = prior_th_imu

    def set_pose(self, x: float, y: float, heading: float) -> None:
        self.x, self.y, self.heading = x, y, heading
        self.px, self.py, self.ph = x, y, heading

    def get_map(self) -> OccupancyGrid:
        return self.mapper.to_occupancy_grid()

    def update(self, scan: LidarScan, imu_heading: float | None = None) -> LocalizationResult:
        d = np.asarray(scan.distances, dtype=float)
        ang = np.radians(np.asarray(scan.angles, dtype=float))
        hit = d < (MAX_RANGE - 1e-3)
        bx = (d * np.cos(ang))[hit]    # ロボット座標系の終点
        by = (d * np.sin(ang))[hit]

        # --- 等速度予測（直前の推定移動量を外挿、暴走防止に上限クランプ） ---
        # 予測デルタが無制限だと、一度マッチが飛んだとき指数的に発散するため制限する。
        dxp = float(np.clip(self.x - self.px, -self.max_step_xy, self.max_step_xy))
        dyp = float(np.clip(self.y - self.py, -self.max_step_xy, self.max_step_xy))
        gx = self.x + dxp
        gy = self.y + dyp

        # 方位の事前分布中心：IMU があればそれを信頼、無ければ等速度予測
        if imu_heading is not None:
            gth = float(imu_heading)
            th_weight = self.prior_th_imu
        else:
            dthp = float(np.clip(_wrap180(self.heading - self.ph),
                                 -math.degrees(self.max_step_th), math.degrees(self.max_step_th)))
            gth = self.heading + dthp
            th_weight = self.prior_th

        prior = (gx, gy, gth)
        last_res = 1.0
        if len(bx) >= 10:
            prob = 1.0 - 1.0 / (1.0 + np.exp(np.clip(self.mapper.log_odds, -20, 20)))
            res = self.mapper.resolution
            for sigma_m, nit in zip(self.blur_sigmas_m, self.iters):
                sigma = sigma_m / res                # [m] → セル
                field = self._blur(prob, sigma) if sigma > 0 else prob
                gx, gy, gth, last_res = self._match(
                    bx, by, gx, gy, gth, field, nit, prior, th_weight)

        # --- コミット ---
        self.px, self.py, self.ph = self.x, self.y, self.heading
        self.x, self.y, self.heading = gx, gy, gth % 360.0
        confidence = float(np.clip(1.0 - last_res, 0.0, 1.0))
        return LocalizationResult(
            x=self.x, y=self.y, heading=self.heading,
            confidence=confidence, source="slam", timestamp=time.time(),
        )

    # ---- Gauss-Newton 合わせ込み ----------------------------------------
    def _match(self, bx, by, x, y, th_deg, field, n_iter, prior=None, th_weight=None):
        th = math.radians(th_deg)
        px, py, pth = (prior if prior is not None else (x, y, th_deg))
        pth_r = math.radians(pth)
        res_mean = 1.0
        for _ in range(n_iter):
            ct, st = math.cos(th), math.sin(th)
            sx = x + ct * bx - st * by
            sy = y + st * bx + ct * by
            M, gx, gy = self._sample(field, sx, sy)
            r = 1.0 - M

            dsx = -st * bx - ct * by
            dsy = ct * bx - st * by
            J = np.stack([gx, gy, gx * dsx + gy * dsy], axis=1)   # (K,3)
            H = J.T @ J
            g = J.T @ r

            # 運動モデル事前分布：拘束不足な方向を予測 (px,py,pth) に引き寄せる。
            # scan が強く拘束する方向では H が大きく事前分布は無視される。
            if prior is not None:
                w_th = self.prior_th if th_weight is None else th_weight
                H[0, 0] += self.prior_xy + self.damping
                H[1, 1] += self.prior_xy + self.damping
                H[2, 2] += w_th + self.damping
                g[0] += self.prior_xy * (px - x)
                g[1] += self.prior_xy * (py - y)
                g[2] += w_th * math.radians(_wrap180(pth - math.degrees(th)))
            else:
                H[0, 0] += self.damping
                H[1, 1] += self.damping
                H[2, 2] += self.damping
            try:
                dxi = np.linalg.solve(H, g)
            except np.linalg.LinAlgError:
                break

            dxi[0] = float(np.clip(dxi[0], -self.max_step_xy, self.max_step_xy))
            dxi[1] = float(np.clip(dxi[1], -self.max_step_xy, self.max_step_xy))
            dxi[2] = float(np.clip(dxi[2], -self.max_step_th, self.max_step_th))
            x += dxi[0]
            y += dxi[1]
            th += dxi[2]
            res_mean = float(np.mean(np.abs(r)))
            if abs(dxi[0]) < 1e-4 and abs(dxi[1]) < 1e-4 and abs(dxi[2]) < 1e-4:
                break
        return x, y, math.degrees(th), res_mean

    def _sample(self, field, x, y):
        """確率場 field の双線形補間値と空間勾配を返す（ベクトル化）。"""
        res = self.mapper.resolution
        H, W = field.shape
        # 占有値はセル中心にあるので、補間インデックスを半セルずらして整合させる
        # （-0.5 を入れないと systematic に約半セル分のバイアスが出る）
        cf = (x - self.mapper.origin_x) / res - 0.5
        rf = (y - self.mapper.origin_y) / res - 0.5
        c0 = np.clip(np.floor(cf).astype(int), 0, W - 2)
        r0 = np.clip(np.floor(rf).astype(int), 0, H - 2)
        dx = np.clip(cf - c0, 0.0, 1.0)
        dy = np.clip(rf - r0, 0.0, 1.0)

        P00 = field[r0, c0]
        P10 = field[r0, c0 + 1]
        P01 = field[r0 + 1, c0]
        P11 = field[r0 + 1, c0 + 1]

        M = (P00 * (1 - dx) * (1 - dy) + P10 * dx * (1 - dy)
             + P01 * (1 - dx) * dy + P11 * dx * dy)
        gx = ((P10 - P00) * (1 - dy) + (P11 - P01) * dy) / res
        gy = ((P01 - P00) * (1 - dx) + (P11 - P10) * dx) / res
        return M, gx, gy

    @staticmethod
    def _blur(field: np.ndarray, sigma: float) -> np.ndarray:
        """分離可能ガウシアンで確率場をボカす（収束範囲拡大）。"""
        if sigma <= 0:
            return field
        r = max(1, int(round(3 * sigma)))
        xs = np.arange(-r, r + 1)
        k = np.exp(-(xs ** 2) / (2.0 * sigma * sigma))
        k /= k.sum()
        # 行方向 → 列方向に畳み込み
        f = np.apply_along_axis(lambda m: np.convolve(m, k, mode="same"), 1, field)
        f = np.apply_along_axis(lambda m: np.convolve(m, k, mode="same"), 0, f)
        return f
