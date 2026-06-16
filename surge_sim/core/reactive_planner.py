"""リアクティブ制御（Follow the Gap）。

地図・自己位置推定・SLAMを一切使わず、その瞬間の LidarScan だけから
進路（ステア・速度）を決定する。コースが事前に分からない実機でも、
1周目の安全な探索走行に使える。実機・シミュレータ共通。

アルゴリズム（F1TENTH の Follow-the-Gap 系）:
    1. 前方視野(±fov)のスキャンを切り出す
    2. 最近傍障害物の周囲に安全バブル(半径bubble_m)を張り距離0にする
    3. 開いている(=しきい値以上の)連続区間のうち最大ギャップを探す
    4. ギャップ内の最遠点へ向かう角度をステア指令にする
    5. 前方距離とステア量から速度を決める（壁が近い/急旋回ほど減速）

角度規約: scan.angles[i] は車両前方基準・反時計回り[deg]（0=前方, +90=左）。
"""

from __future__ import annotations

import math
import time

import numpy as np

from core.interfaces import ControlCommand, LidarScan


class ReactivePlanner:
    """Follow the Gap によるリアクティブ経路選択。"""

    def __init__(self, max_steer: float = 40.0, cruise_speed: float = 1.5,
                 fov_deg: float = 90.0, bubble_m: float = 0.30,
                 clip_range: float = 4.0, gap_min_abs: float = 0.7,
                 gap_ratio: float = 0.6, steer_gain: float = 0.9,
                 stop_dist: float = 0.25, slow_dist: float = 1.2):
        self.max_steer = max_steer
        self.cruise_speed = cruise_speed
        self.fov_deg = fov_deg
        self.bubble_m = bubble_m
        self.clip_range = clip_range
        self.gap_min_abs = gap_min_abs      # ギャップ判定の最小絶対距離 [m]
        self.gap_ratio = gap_ratio          # 適応しきい値 = ratio * 前方最大距離
        self.steer_gain = steer_gain
        self.stop_dist = stop_dist          # これ以下でほぼ停止 [m]
        self.slow_dist = slow_dist          # これ以上で巡航速度 [m]

        # 可視化・デバッグ用
        self.best_angle: float = 0.0        # 選択方向 [deg]（車両前方基準）
        self.best_dist: float = 0.0         # 選択方向の距離 [m]
        self.front_dist: float = 0.0        # 前方距離 [m]

    # ------------------------------------------------------------------
    def compute_command(self, scan: LidarScan) -> ControlCommand:
        ts = time.time()
        if scan is None or scan.distances.size == 0:
            return ControlCommand(0.0, 0.0, ts)

        d = scan.distances.astype(np.float64).copy()
        ang = scan.angles.astype(np.float64)
        # -180..180 の符号付き角度（0=前方, +左）
        signed = ((ang + 180.0) % 360.0) - 180.0

        # 前方視野を切り出して角度昇順に並べる
        mask = np.abs(signed) <= self.fov_deg
        fa = signed[mask]
        fd = d[mask]
        order = np.argsort(fa)
        fa = fa[order]
        fd = np.clip(fd[order], 0.0, self.clip_range)

        if fd.size == 0:
            return ControlCommand(0.0, 0.0, ts)

        # 1. 最近傍障害物に安全バブルを張る
        c = int(np.argmin(fd))
        r_min = max(fd[c], 1e-3)
        bubble_deg = math.degrees(math.atan2(self.bubble_m, r_min))
        bubble = np.abs(fa - fa[c]) <= bubble_deg
        fd_proc = fd.copy()
        fd_proc[bubble] = 0.0

        # 2. 適応しきい値で「開いている」点を判定
        thresh = max(self.gap_min_abs, self.gap_ratio * float(fd.max()))
        free = fd_proc >= thresh

        # 3. 最大ギャップ（連続するfreeの最長区間）を探す
        start, end = self._largest_run(free)
        if start is None:
            # ギャップ無し → 最も開けている方向（最遠点）へ
            best = int(np.argmax(fd_proc))
        else:
            # ギャップ内の最遠点へ向かう
            seg = fd_proc[start:end + 1]
            best = start + int(np.argmax(seg))

        self.best_angle = float(fa[best])
        self.best_dist = float(fd[best])

        # 前方狭角(±12°)の最小距離 → 速度決定に使う
        fwd_mask = np.abs(fa) <= 12.0
        self.front_dist = float(np.min(fd[fwd_mask])) if np.any(fwd_mask) else float(fd[best])

        # 4. ステア（+左）
        steer = self.steer_gain * self.best_angle
        steer = max(-self.max_steer, min(self.max_steer, steer))

        # 5. 速度：前方距離で巡航〜停止、急旋回でさらに減速
        ratio = (self.front_dist - self.stop_dist) / max(self.slow_dist - self.stop_dist, 1e-3)
        ratio = max(0.0, min(1.0, ratio))
        speed = self.cruise_speed * ratio
        speed *= (1.0 - 0.4 * abs(steer) / self.max_steer)
        speed = max(0.0, speed)

        return ControlCommand(target_speed=speed, target_steer=steer, timestamp=ts)

    # ------------------------------------------------------------------
    @staticmethod
    def _largest_run(flags: np.ndarray):
        """True が連続する最長区間の (start, end) を返す。無ければ (None, None)。"""
        best_len = 0
        best = (None, None)
        i = 0
        n = flags.size
        while i < n:
            if flags[i]:
                j = i
                while j + 1 < n and flags[j + 1]:
                    j += 1
                if (j - i + 1) > best_len:
                    best_len = j - i + 1
                    best = (i, j)
                i = j + 1
            else:
                i += 1
        return best
