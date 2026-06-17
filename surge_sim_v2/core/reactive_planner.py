"""リアクティブ走行プランナ（LiDARのみ・地図/自己位置不要）。

前方 FOV（既定90度）を一定の角度幅の窓でスキャンし、各窓の距離の平均が
最も大きい方向（＝最も開けている方向）へ進む。地図も自己位置も使わないので、
未知コースをそのまま周回でき、手動探索の代わりに自動で地図を作れる。

アルゴリズム：
  1. 前方 ±fov/2 の LiDAR 点を取り出す
  2. 角度幅 window_deg の窓を細かくずらしながら各窓の平均距離を計算
  3. 平均距離が最大の窓の中心方向を目標角とする
  4. 目標角へ比例操舵、前方の余裕に応じて速度を決める
"""
from __future__ import annotations

import time

import numpy as np

from .interfaces import ControlCommand, LidarScan
from .occupancy import MAX_RANGE


class ReactivePlanner:
    def __init__(self, config: dict | None = None,
                 max_steer: float = 40.0, max_speed: float = 3.0) -> None:
        cfg = config or {}
        self.fov = float(cfg.get("fov_deg", 90.0))            # 前方探索角（全幅）
        self.window = float(cfg.get("window_deg", 30.0))      # 平均をとる角度窓幅
        self.step = float(cfg.get("step_deg", 3.0))           # 窓をずらす刻み
        self.steer_gain = float(cfg.get("steer_gain", 0.9))   # 目標角→操舵ゲイン
        self.cruise_speed = float(cfg.get("cruise_speed", 1.2))
        self.min_speed = float(cfg.get("min_speed", 0.4))
        self.slow_dist = float(cfg.get("slow_dist", 2.0))     # 前方これ以下で減速
        self.stop_dist = float(cfg.get("stop_dist", 0.35))    # 前方これ以下で停止
        self.safety_bubble = float(cfg.get("safety_bubble", 0.30))  # 近接点の無視半径
        self.max_steer = float(max_steer)
        self.max_speed = float(max_speed)

    def compute_command(self, scan: LidarScan) -> ControlCommand:
        if scan is None:
            return ControlCommand(0.0, 0.0, time.time())
        d = np.asarray(scan.distances, dtype=float)
        ang = np.asarray(scan.angles, dtype=float)
        angw = (ang + 180.0) % 360.0 - 180.0       # [-180,180]、0=前方

        # 前方FOV内の点
        fov_mask = np.abs(angw) <= (self.fov / 2.0)
        if not np.any(fov_mask):
            return ControlCommand(0.0, 0.0, time.time())
        fa = angw[fov_mask]
        fd = d[fov_mask].copy()
        # 至近の点は安全のため距離を縮めて評価（バブル）：壁際へ寄らせない
        fd = np.where(fd < self.safety_bubble, 0.0, fd)

        # 窓をずらしながら平均距離が最大の中心角を探す
        centers = np.arange(-self.fov / 2.0 + self.window / 2.0,
                            self.fov / 2.0 - self.window / 2.0 + 1e-6, self.step)
        if len(centers) == 0:
            centers = np.array([0.0])
        best_c, best_mean = 0.0, -1.0
        half = self.window / 2.0
        for c in centers:
            win = (fa >= c - half) & (fa <= c + half)
            if not np.any(win):
                continue
            mean = float(np.mean(fd[win]))
            if mean > best_mean:
                best_mean, best_c = mean, float(c)

        # 目標角（+ = 左）へ比例操舵
        steer = float(np.clip(self.steer_gain * best_c, -self.max_steer, self.max_steer))

        # 速度：正面の余裕で決める（正面±10度の最小距離）
        front = np.abs(angw) < 10.0
        front_dist = float(np.min(d[front])) if np.any(front) else MAX_RANGE
        if front_dist < self.stop_dist:
            speed = 0.0
        else:
            t = np.clip((front_dist - self.stop_dist) / (self.slow_dist - self.stop_dist), 0.0, 1.0)
            speed = self.min_speed + t * (self.cruise_speed - self.min_speed)
            # 急操舵時は減速
            speed *= 1.0 - 0.4 * (abs(steer) / self.max_steer)
        speed = float(np.clip(speed, 0.0, self.max_speed))
        return ControlCommand(speed, steer, time.time())
