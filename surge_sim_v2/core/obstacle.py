"""動的障害物の検知と回避（占有地図ベース）。

自律走行中、LiDAR スキャンと SLAM 占有地図を突き合わせ、
「地図では自由空間（free）なのに点が返ってきた」ビーム＝静的地図に無い物体
＝動的障害物（人・車）を検知する。壁は地図で occupied なので誤検知しない。

検知した経路前方の障害物に対して：
- 至近 → 緊急停止
- 検知距離内 → 空いている側へ操舵を寄せて減速して回避
"""
from __future__ import annotations

import math
import time

import numpy as np

from .interfaces import ControlCommand, LidarScan, LocalizationResult
from .occupancy import MAX_RANGE


class ObstacleAvoider:
    def __init__(self, config: dict | None = None, max_steer: float = 40.0) -> None:
        cfg = config or {}
        self.detect_range = float(cfg.get("detect_range", 1.8))   # [m] 反応開始
        self.stop_dist = float(cfg.get("stop_dist", 0.45))        # [m] 停止
        self.fov = float(cfg.get("fov_deg", 70.0))                # [deg] 前方探索半角
        self.avoid_gain = float(cfg.get("avoid_gain", 28.0))      # 操舵回避ゲイン[deg]
        self.free_thresh = float(cfg.get("free_thresh", -0.5))    # log-odds 自由空間しきい値
        self.max_steer = float(max_steer)

    def adjust(self, cmd: ControlCommand, loc: LocalizationResult,
               scan: LidarScan, mapper) -> tuple[ControlCommand, dict]:
        """経路追従指令 cmd を障害物に応じて補正して返す。

        mapper: SLAM の OccupancyGridMapper（log_odds で free 判定に使う）。
                None（地図なし）の場合は何もしない。
        """
        info = {"detected": False, "estop": False, "distance": None}
        if scan is None or mapper is None:
            return cmd, info

        d = np.asarray(scan.distances, dtype=float)
        ang = np.asarray(scan.angles, dtype=float)
        angw = (ang + 180.0) % 360.0 - 180.0
        ar = np.radians(ang)
        yf = d * np.sin(ar)                         # 車両左方
        m = (d < self.detect_range) & (d * np.cos(ar) > 0) & (np.abs(angw) < self.fov)
        if not np.any(m):
            return cmd, info

        # ヒット点をワールド変換 → 占有地図のセルを参照
        h = math.radians(loc.heading)
        wx = loc.x + d[m] * np.cos(np.radians(ang[m]) + h)
        wy = loc.y + d[m] * np.sin(np.radians(ang[m]) + h)
        col = ((wx - mapper.origin_x) / mapper.resolution).astype(int)
        row = ((wy - mapper.origin_y) / mapper.resolution).astype(int)
        H, W = mapper.log_odds.shape
        inb = (col >= 0) & (col < W) & (row >= 0) & (row < H)

        # 地図で「自由空間」のセルに当たった点 ＝ 静的地図に無い動的障害物
        is_obstacle = np.zeros(m.sum(), dtype=bool)
        if np.any(inb):
            lo = mapper.log_odds[row[inb], col[inb]]
            is_obstacle[inb] = lo < self.free_thresh
        if not np.any(is_obstacle):
            return cmd, info

        dd = d[m][is_obstacle]
        yy = yf[m][is_obstacle]
        k = int(np.argmin(dd))
        nearest = float(dd[k])
        info.update(detected=True, distance=nearest)

        if nearest < self.stop_dist:
            info["estop"] = True
            return ControlCommand(0.0, cmd.target_steer, time.time()), info

        # 回避：障害物の反対側へ操舵を寄せ、距離に応じて減速
        side = np.sign(yy[k]) if abs(yy[k]) > 0.05 else self._freer_side(d, ar, m)
        severity = 1.0 - (nearest - self.stop_dist) / (self.detect_range - self.stop_dist)
        severity = float(np.clip(severity, 0.0, 1.0))
        steer = float(np.clip(cmd.target_steer - side * self.avoid_gain * severity,
                              -self.max_steer, self.max_steer))
        speed = cmd.target_speed * (1.0 - 0.6 * severity)
        return ControlCommand(speed, steer, time.time()), info

    @staticmethod
    def _freer_side(d, ar, m) -> float:
        yf = d * np.sin(ar)
        xf = d * np.cos(ar)
        fwd = m & (xf > 0)
        left = yf[fwd & (yf > 0)]
        right = yf[fwd & (yf < 0)]
        left_room = np.min(np.abs(left)) if left.size else 1e9
        right_room = np.min(np.abs(right)) if right.size else 1e9
        return 1.0 if left_room >= right_room else -1.0
