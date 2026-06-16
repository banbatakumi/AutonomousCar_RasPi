"""自己位置推定モジュール。

Phase1: CheatLocalizer（シミュレータ真値を直接利用）
Phase3: SLAMLocalizer（Hector SLAMベース、スタブ）
将来:   EKFLocalizer（オドメトリ＋IMU融合、スタブ）
"""

from __future__ import annotations

import math

import numpy as np

from core.interfaces import LidarScan, LocalizationResult, OccupancyGrid, VehicleState
from core.occupancy import OccupancyGridMapper


class CheatLocalizer:
    """シミュレータから真の位置を直接取得する（Phase1）。"""

    def update(self, true_state: VehicleState) -> LocalizationResult:
        return LocalizationResult(
            x=true_state.x,
            y=true_state.y,
            heading=true_state.heading,
            confidence=1.0,
            source="cheat",
            timestamp=true_state.timestamp,
        )


class SLAMLocalizer:
    """Hector SLAM風スキャンマッチング自己位置推定（Phase3）。

    占有確率地図に対し、現スキャンの端点群がよく一致する姿勢を Gauss-Newton 法で
    求める（地図勾配を用いた最適化）。推定姿勢で地図を更新していく。

    使い方:
        slam = SLAMLocalizer(bounds=(minx,miny,maxx,maxy), start_pose=(x,y,deg))
        res = slam.update(scan)          # 各周期
        grid = slam.get_map()            # 占有格子
    """

    def __init__(self, bounds: tuple[float, float, float, float],
                 start_pose: tuple[float, float, float] = (0.0, 0.0, 0.0),
                 resolution: float = 0.05, max_range: float = 12.0,
                 gn_iters: int = 5):
        self.mapper = OccupancyGridMapper(*bounds, resolution=resolution,
                                          max_range=max_range)
        self.res = resolution
        self.max_range = max_range
        self.gn_iters = gn_iters
        self.x, self.y, self.theta = start_pose[0], start_pose[1], math.radians(start_pose[2])
        self._initialized = False
        self._t = 0.0

    # ------------------------------------------------------------------
    def update(self, scan: LidarScan, motion=None) -> LocalizationResult:
        """スキャンから姿勢を推定し地図を更新する。

        motion: 任意の予測移動 (dx, dy, dtheta_deg)。あれば初期推定に加える。
        """
        self._t = scan.timestamp
        # 端点（ロボット座標, x前方）
        ang = np.radians(scan.angles)
        d = scan.distances
        valid = d < (self.max_range - 1e-3)
        px = (d * np.cos(ang))[valid]
        py = (d * np.sin(ang))[valid]

        if not self._initialized:
            # 最初のスキャンで地図を初期化
            self._integrate(scan)
            self._initialized = True
            return self._result(confidence=1.0)

        if motion is not None:
            self.x += motion[0]
            self.y += motion[1]
            self.theta += math.radians(motion[2])

        # Gauss-Newton スキャンマッチング
        for _ in range(self.gn_iters):
            H = np.zeros((3, 3))
            b = np.zeros(3)
            c, s = math.cos(self.theta), math.sin(self.theta)
            # ワールド端点
            wx = self.x + c * px - s * py
            wy = self.y + s * px + c * py
            for i in range(px.shape[0]):
                M, dMdx, dMdy = self._map_value_grad(wx[i], wy[i])
                # ∂P/∂θ
                dpx = -s * px[i] - c * py[i]
                dpy = c * px[i] - s * py[i]
                J = np.array([dMdx, dMdy, dMdx * dpx + dMdy * dpy])
                r = 1.0 - M
                H += np.outer(J, J)
                b += J * r
            # Levenberg-Marquardt風ダンピング（Hの規模に比例）+ ステップ制限で発散防止
            lam = 1e-2 * (np.trace(H) / 3.0 + 1e-6)
            H += lam * np.eye(3)
            try:
                dxi = np.linalg.solve(H, b)
            except np.linalg.LinAlgError:
                break
            # 1反復あたりの移動量を制限（並進0.05m, 回転0.03rad）
            dxi[0] = max(-0.05, min(0.05, dxi[0]))
            dxi[1] = max(-0.05, min(0.05, dxi[1]))
            dxi[2] = max(-0.03, min(0.03, dxi[2]))
            self.x += dxi[0]
            self.y += dxi[1]
            self.theta += dxi[2]
            if abs(dxi[0]) + abs(dxi[1]) + abs(dxi[2]) < 1e-5:
                break

        # 推定姿勢で地図更新
        self._integrate(scan)
        # 一致度を信頼度に（端点が占有確率高い割合）
        conf = self._match_score(px, py)
        return self._result(confidence=conf)

    # ------------------------------------------------------------------
    def _integrate(self, scan: LidarScan) -> None:
        pose = type("P", (), {"x": self.x, "y": self.y,
                              "heading": math.degrees(self.theta)})()
        self.mapper.integrate_scan(scan, pose)

    def _result(self, confidence: float) -> LocalizationResult:
        return LocalizationResult(x=self.x, y=self.y,
                                  heading=math.degrees(self.theta) % 360.0,
                                  confidence=float(confidence), source="slam",
                                  timestamp=self._t)

    # ------------------------------------------------------------------
    def _prob(self, log: float) -> float:
        return 1.0 / (1.0 + math.exp(-log))

    def _map_value_grad(self, x: float, y: float):
        """占有確率の双線形補間値とワールド勾配 (M, dM/dx, dM/dy)。"""
        m = self.mapper
        fx = (x - m.origin_x) / m.resolution
        fy = (y - m.origin_y) / m.resolution
        x0 = int(math.floor(fx)); y0 = int(math.floor(fy))
        if x0 < 0 or x0 >= m.w - 1 or y0 < 0 or y0 >= m.h - 1:
            return 0.0, 0.0, 0.0
        tx = fx - x0; ty = fy - y0
        # 4近傍の占有確率
        p00 = self._prob(m.log[y0, x0]); p10 = self._prob(m.log[y0, x0 + 1])
        p01 = self._prob(m.log[y0 + 1, x0]); p11 = self._prob(m.log[y0 + 1, x0 + 1])
        # 双線形
        M = (p00 * (1 - tx) * (1 - ty) + p10 * tx * (1 - ty)
             + p01 * (1 - tx) * ty + p11 * tx * ty)
        # セル座標勾配 → ワールド勾配(/res)
        dMdfx = (p10 - p00) * (1 - ty) + (p11 - p01) * ty
        dMdfy = (p01 - p00) * (1 - tx) + (p11 - p10) * tx
        return M, dMdfx / m.resolution, dMdfy / m.resolution

    def _match_score(self, px, py) -> float:
        if px.shape[0] == 0:
            return 0.0
        c, s = math.cos(self.theta), math.sin(self.theta)
        wx = self.x + c * px - s * py
        wy = self.y + s * px + c * py
        vals = [self._map_value_grad(wx[i], wy[i])[0] for i in range(px.shape[0])]
        return float(np.clip(np.mean(vals), 0.0, 1.0))

    # ------------------------------------------------------------------
    def get_map(self) -> OccupancyGrid:
        return self.mapper.to_occupancy_grid(timestamp=self._t)


class EKFLocalizer:
    """オドメトリ・IMU融合EKF（将来実装）。"""

    def update(self, scan: LidarScan, state: VehicleState) -> LocalizationResult:
        raise NotImplementedError("将来実装")
