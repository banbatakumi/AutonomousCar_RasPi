"""経路追従制御。

Phase2: PurePursuitPlanner（Pure Pursuit経路追従）
Phase4: MPCPlanner（Model Predictive Control）
"""

from __future__ import annotations

import math
import time

import numpy as np

from core.interfaces import ControlCommand, LocalizationResult
from core.path_utils import lookahead_point, nearest_index, path_curvature


class PurePursuitPlanner:
    """Pure Pursuit経路追従（Phase2）。

    前方の先読み点(lookahead point)を車両座標系に変換し、その点を通る円弧の
    曲率から必要なステア角を求める。先読み距離は速度に応じて可変にできる。

    曲率: kappa = 2 * y_local / Ld^2
    ステア: delta = atan(wheelbase * kappa)
    """

    def __init__(self, wheelbase: float = 0.230, max_steer: float = 40.0,
                 lookahead_distance: float = 0.5,
                 lookahead_gain: float = 0.3,
                 min_lookahead: float = 0.3, max_lookahead: float = 1.5,
                 cruise_speed: float = 1.5, max_speed: float = 3.0,
                 curvature_slowdown: float = 0.6):
        self.wheelbase = wheelbase
        self.max_steer = max_steer
        self.lookahead_distance = lookahead_distance
        self.lookahead_gain = lookahead_gain          # 速度比例ゲイン [s]
        self.min_lookahead = min_lookahead
        self.max_lookahead = max_lookahead
        self.cruise_speed = cruise_speed
        self.max_speed = max_speed
        self.curvature_slowdown = curvature_slowdown  # 曲率による減速の強さ

        # デバッグ・描画用に最後の先読み点を保持
        self.last_target: np.ndarray | None = None
        self.last_nearest_idx: int = 0
        self._kappa_cache_id = None
        self._kappa = None

    # ------------------------------------------------------------------
    def compute_command(self, state: LocalizationResult,
                        path: np.ndarray, current_speed: float = 0.0
                        ) -> ControlCommand:
        ts = time.time()
        if path is None or len(path) < 2:
            return ControlCommand(0.0, 0.0, ts)

        # 速度に応じた可変先読み距離
        ld = self.lookahead_distance + self.lookahead_gain * max(current_speed, 0.0)
        ld = max(self.min_lookahead, min(self.max_lookahead, ld))

        near = nearest_index(path, state.x, state.y)
        target, tgt_idx = lookahead_point(path, state.x, state.y, ld, near)
        self.last_target = target
        self.last_nearest_idx = near

        # 先読み点を車両座標系へ変換（x前方, y左）
        dx = target[0] - state.x
        dy = target[1] - state.y
        h = math.radians(state.heading)
        cos_h, sin_h = math.cos(h), math.sin(h)
        x_local = cos_h * dx + sin_h * dy
        y_local = -sin_h * dx + cos_h * dy

        # Pure Pursuit 曲率 → ステア角
        ld_eff = max(math.hypot(x_local, y_local), 1e-6)
        kappa = 2.0 * y_local / (ld_eff * ld_eff)
        steer = math.degrees(math.atan(self.wheelbase * kappa))
        steer = max(-self.max_steer, min(self.max_steer, steer))

        # 目標速度: コース曲率に応じて減速
        target_speed = self._speed_for_curvature(path, near)

        # 先読み点が後方（鋭角）なら強制減速
        if x_local < 0:
            target_speed *= 0.4

        return ControlCommand(target_speed=target_speed,
                              target_steer=steer, timestamp=ts)

    # ------------------------------------------------------------------
    def _speed_for_curvature(self, path: np.ndarray, idx: int) -> float:
        """前方の経路曲率に基づき目標速度を決める。"""
        if id(path) != self._kappa_cache_id:
            self._kappa = path_curvature(path)
            self._kappa_cache_id = id(path)
        n = len(path)
        # 数点先までの最大曲率を見て減速
        look = max(1, int(0.5 / max(self._segment_spacing(path), 1e-3)))
        window = [self._kappa[(idx + k) % n] for k in range(look)]
        kmax = max(window) if window else 0.0
        # 曲率が大きいほど cruise から減速
        factor = 1.0 / (1.0 + self.curvature_slowdown * kmax)
        return max(0.3, min(self.cruise_speed, self.cruise_speed * factor))

    @staticmethod
    def _segment_spacing(path: np.ndarray) -> float:
        if len(path) < 2:
            return 0.05
        return float(np.hypot(*(path[1] - path[0])))


class MPCPlanner:
    """Model Predictive Control（Phase4で実装）。"""

    def compute_command(self, state: LocalizationResult, path: np.ndarray,
                        speed_profile: np.ndarray) -> ControlCommand:
        raise NotImplementedError("Phase4で実装")
