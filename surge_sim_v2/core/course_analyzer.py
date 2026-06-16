"""コース境界抽出・中心線生成。

Phase2: コース定義の CENTER_LINE からカンニングで CourseMap を生成する
        （build_course_map）。CheatLocalizer と同じ「真値を使う」思想。
Phase3: SLAM 占有格子から境界・中心線を抽出する analyze() を実装する（スタブ）。
"""
from __future__ import annotations

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

        centers = []
        widths = []
        left_pts = []
        right_pts = []
        for p, n in zip(path, normals):
            left = self._march(p, n, steps, is_occ, max_search)
            right = self._march(p, -n, steps, is_occ, max_search)
            center = p + n * (left - right) / 2.0
            centers.append(center)
            widths.append(left + right)
            left_pts.append(p + n * left)
            right_pts.append(p - n * right)

        center_line = self._smooth_closed(np.array(centers), k=5)
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
