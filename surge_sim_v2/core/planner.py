"""経路追従制御（Phase2 PurePursuit / Phase4 MPC）。"""
from __future__ import annotations

import math
import time

import numpy as np

from .interfaces import ControlCommand, LocalizationResult
from .path_utils import lookahead_point, nearest_index, path_curvature


class PurePursuitPlanner:
    """Pure Pursuit による経路追従（Phase2）。

    可変先読み距離 Ld = clip(min + gain*speed, min, max)。
    曲率 κ = 2*y_local / Ld^2 から steer = atan(L*κ) を求める。
    速度は前方経路の曲率に応じて減速する。
    """

    def __init__(self, lookahead_distance: float = 0.5,
                 wheelbase: float = 0.230,
                 config: dict | None = None) -> None:
        cfg = config or {}
        self.wheelbase = float(wheelbase)
        self.lookahead_min = float(cfg.get("lookahead_min", lookahead_distance))
        self.lookahead_gain = float(cfg.get("lookahead_gain", 0.3))
        self.lookahead_max = float(cfg.get("lookahead_max", 1.2))
        self.cruise_speed = float(cfg.get("cruise_speed", 1.5))
        self.min_speed = float(cfg.get("min_speed", 0.4))
        self.max_speed = float(cfg.get("max_speed", 3.0))
        self.max_steer = float(cfg.get("max_steer_angle", 40.0))
        self.curvature_slowdown = float(cfg.get("curvature_slowdown", 1.0))

        self._kappa_cache: np.ndarray | None = None
        self._cache_id: int | None = None

    def compute_command(
        self,
        state: LocalizationResult,
        path: np.ndarray,
        speed_cap: float | None = None,
        speed_profile: np.ndarray | None = None,
    ) -> ControlCommand:
        pts = np.asarray(path, dtype=float)
        if len(pts) < 2:
            return ControlCommand(0.0, 0.0, time.time())

        pos = (state.x, state.y)
        heading = math.radians(state.heading)
        speed = abs(getattr(state, "speed", 0.0)) if hasattr(state, "speed") else 0.0

        # --- 可変先読み距離 ---
        ld = self.lookahead_min + self.lookahead_gain * speed
        ld = float(np.clip(ld, self.lookahead_min, self.lookahead_max))

        idx = nearest_index(pts, pos)
        target, _ = lookahead_point(pts, pos, ld, start_idx=idx)

        # --- 車両座標系へ変換（ワールド→ボディは -heading 回転） ---
        dx = target[0] - state.x
        dy = target[1] - state.y
        cos_h = math.cos(-heading)
        sin_h = math.sin(-heading)
        x_local = dx * cos_h - dy * sin_h
        y_local = dx * sin_h + dy * cos_h

        # --- Pure Pursuit 操舵 ---
        ld_eff = max(math.hypot(x_local, y_local), 1e-3)
        kappa = 2.0 * y_local / (ld_eff * ld_eff)
        steer = math.degrees(math.atan(self.wheelbase * kappa))
        steer = float(np.clip(steer, -self.max_steer, self.max_steer))

        # --- 目標速度 ---
        cap = self.max_speed if speed_cap is None else min(self.max_speed, speed_cap)
        if speed_profile is not None and len(speed_profile) == len(pts):
            # Phase4: 事前計算した速度プロファイルを先読み点で参照
            look_idx = nearest_index(pts, (target[0], target[1]))
            target_speed = float(min(speed_profile[idx], speed_profile[look_idx]))
        else:
            # Phase2/3: 曲率ヒューリスティックで減速
            kappas = self._curvatures(pts)
            look_idx = nearest_index(pts, (target[0], target[1]))
            local_kappa = max(abs(kappas[idx]), abs(kappas[look_idx]))
            target_speed = self.cruise_speed / (1.0 + self.curvature_slowdown * local_kappa)
        target_speed = float(np.clip(target_speed, self.min_speed, cap))

        return ControlCommand(
            target_speed=target_speed,
            target_steer=steer,
            timestamp=time.time(),
        )

    def _curvatures(self, pts: np.ndarray) -> np.ndarray:
        """経路曲率をキャッシュ（同一経路なら再計算しない）。"""
        pid = id(pts)
        if self._kappa_cache is None or self._cache_id != pid \
                or len(self._kappa_cache) != len(pts):
            self._kappa_cache = path_curvature(pts)
            self._cache_id = pid
        return self._kappa_cache


class MPCPlanner:
    """Phase4で実装。"""

    def compute_command(
        self,
        state: LocalizationResult,
        path: np.ndarray,
        speed_profile: np.ndarray,
    ) -> ControlCommand:
        raise NotImplementedError("Phase4で実装")
