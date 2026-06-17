"""コース境界抽出・中心線生成。

Phase2: コース定義の CENTER_LINE からカンニングで CourseMap を生成する
        （build_course_map）。CheatLocalizer と同じ「真値を使う」思想。
Phase3: SLAM 占有格子から境界・中心線を抽出する analyze() を実装する（スタブ）。
"""
from __future__ import annotations

import math

import numpy as np

from .interfaces import CourseMap, OccupancyGrid
from .path_utils import path_normals, resample_closed


class CourseAnalyzer:
    def __init__(self, course_width: float = 1.0, waypoint_spacing: float = 0.1) -> None:
        self.course_width = course_width
        self.waypoint_spacing = waypoint_spacing

    # ---- Phase2: カンニング中心線から CourseMap 生成 ----------------------
    def build_course_map(self, course: dict) -> CourseMap:
        """コース定義 dict（CENTER_LINE / start_pose）から CourseMap を作る。

        Phase2 ではこの中心線をそのまま追従経路に使う。
        racing_line は Phase4 で最適化するまで中心線と同一にしておく。
        """
        center_raw = course.get("center_line") or []
        if not center_raw:
            raise ValueError("CENTER_LINE が無いコースは Phase2 では追従できません")

        center = resample_closed(np.array(center_raw, dtype=float), self.waypoint_spacing)
        n = len(center)

        normals = path_normals(center)
        half = self.course_width / 2.0
        left_wall = center + normals * half
        right_wall = center - normals * half
        width_profile = np.full(n, self.course_width, dtype=float)

        return CourseMap(
            left_wall=left_wall,
            right_wall=right_wall,
            center_line=center,
            racing_line=center.copy(),
            width_profile=width_profile,
        )

    # ---- Phase3: SLAM マップから抽出 -------------------------------------
    def analyze(self, grid: OccupancyGrid, seed_path: np.ndarray | None = None,
                max_search: float = 2.0) -> CourseMap:
        """占有格子と探索軌跡（seed_path）から中心線・左右壁を抽出する。

        各探索点で法線方向に左右へレイマーチし、最初に当たる占有セルまでの
        距離から中心線（左右の中点）とコース幅を求める。
        seed_path は探索（1周分）の走行軌跡。
        """
        if seed_path is None or len(seed_path) < 3:
            raise ValueError("analyze には探索軌跡 seed_path（1周分）が必要です")

        occ = self._occupied_mask(grid)
        res = grid.resolution
        ox, oy = grid.origin_x, grid.origin_y
        H, W = occ.shape

        def is_occ(x, y) -> bool:
            col = int((x - ox) / res)
            row = int((y - oy) / res)
            if 0 <= col < W and 0 <= row < H:
                return bool(occ[row, col])
            return True  # 範囲外は壁扱い（外には出られない）

        path = resample_closed(np.asarray(seed_path, dtype=float), self.waypoint_spacing)
        normals = path_normals(path)
        steps = np.arange(res, max_search, res)

        # 距離変換（各自由セル→最近傍の壁までの距離[m]）。
        # 中心線はこの距離が法線方向で最大になる点 = medial axis（両壁から最遠）。
        # レイマーチ「最初に当たる壁」方式と違い、コーナーでも破綻しない。
        dist = self._distance_transform(occ, res)

        def dist_at(x, y) -> float:
            col = int((x - ox) / res)
            row = int((y - oy) / res)
            if 0 <= col < W and 0 <= row < H:
                return float(dist[row, col])
            return 0.0

        # 中心点探索は探索軌跡近傍の細い帯に限定する（未探索域はクリアランスが
        # 無限大に見えるため、広く探すとコース外を拾ってしまう）。
        band = max(self.course_width * 0.7, 0.5)
        offsets = np.arange(-band, band + res, res)
        centers = []
        widths = []
        left_pts = []
        right_pts = []
        for p, n in zip(path, normals):
            # 法線上で最もクリアランス（壁から距離）が大きい点を中心とする
            best_off, best_d = 0.0, -1.0
            for off in offsets:
                d = dist_at(p[0] + n[0] * off, p[1] + n[1] * off)
                if d > best_d:
                    best_d, best_off = d, off
            center = p + n * best_off
            left = self._march(center, n, steps, is_occ, max_search)
            right = self._march(center, -n, steps, is_occ, max_search)
            centers.append(center)
            widths.append(left + right)
            left_pts.append(center + n * left)
            right_pts.append(center - n * right)

        center_line = self._smooth_closed(np.array(centers), k=7)
        center_line = resample_closed(center_line, self.waypoint_spacing)

        return CourseMap(
            left_wall=np.array(left_pts),
            right_wall=np.array(right_pts),
            center_line=center_line,
            racing_line=center_line.copy(),   # Phase4 で最適化に置換
            width_profile=np.array(widths),
        )

    def extract_walls(self, grid: OccupancyGrid) -> tuple[np.ndarray, np.ndarray]:
        """占有格子から占有点群（壁）のワールド座標を返す。

        左右の分離は行わず、占有セル全体を返す（境界点群）。
        """
        occ = self._occupied_mask(grid)
        rows, cols = np.where(occ)
        xs = grid.origin_x + (cols + 0.5) * grid.resolution
        ys = grid.origin_y + (rows + 0.5) * grid.resolution
        pts = np.column_stack([xs, ys])
        return pts, pts

    # ---- 内部ヘルパ -------------------------------------------------------
    @staticmethod
    def _occupied_mask(grid: OccupancyGrid) -> np.ndarray:
        occ = (np.asarray(grid.grid) == 100)
        # 3x3 膨張（穴埋め・ノイズ吸収）
        d = occ.copy()
        for dr in (-1, 0, 1):
            for dc in (-1, 0, 1):
                d |= np.roll(np.roll(occ, dr, axis=0), dc, axis=1)
        return d

    @staticmethod
    def _distance_transform(occ: np.ndarray, res: float) -> np.ndarray:
        """占有マスクから各セルの最近傍壁までの距離[m]を返す（2パス chamfer 近似）。"""
        H, W = occ.shape
        INF = 1e9
        d = np.where(occ, 0.0, INF).astype(float)
        a, b = 1.0, math.sqrt(2.0)   # 直交・斜めコスト（セル単位）
        # 前方パス（左上→右下）
        for r in range(H):
            for c in range(W):
                if d[r, c] == 0.0:
                    continue
                best = d[r, c]
                if r > 0:
                    best = min(best, d[r - 1, c] + a)
                    if c > 0:
                        best = min(best, d[r - 1, c - 1] + b)
                    if c < W - 1:
                        best = min(best, d[r - 1, c + 1] + b)
                if c > 0:
                    best = min(best, d[r, c - 1] + a)
                d[r, c] = best
        # 後方パス（右下→左上）
        for r in range(H - 1, -1, -1):
            for c in range(W - 1, -1, -1):
                best = d[r, c]
                if r < H - 1:
                    best = min(best, d[r + 1, c] + a)
                    if c > 0:
                        best = min(best, d[r + 1, c - 1] + b)
                    if c < W - 1:
                        best = min(best, d[r + 1, c + 1] + b)
                if c < W - 1:
                    best = min(best, d[r, c + 1] + a)
                d[r, c] = best
        return np.minimum(d, 1e6) * res

    @staticmethod
    def _march(p, n, steps, is_occ, max_search) -> float:
        for s in steps:
            if is_occ(p[0] + n[0] * s, p[1] + n[1] * s):
                return float(s)
        return float(max_search)

    @staticmethod
    def _smooth_closed(pts: np.ndarray, k: int = 5) -> np.ndarray:
        if len(pts) < k:
            return pts
        kernel = np.ones(k) / k
        x = np.convolve(np.r_[pts[-k:, 0], pts[:, 0], pts[:k, 0]], kernel, mode="same")
        y = np.convolve(np.r_[pts[-k:, 1], pts[:, 1], pts[:k, 1]], kernel, mode="same")
        return np.column_stack([x[k:-k], y[k:-k]])
