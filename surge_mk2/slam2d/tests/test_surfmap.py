"""`core/surfmap.py`（面要素の地図）のテスト。

位置合わせの土台なので、ここが狂うと上の層は全部おかしくなる。**面の位置と
法線が幾何的に正しいか**と、**壁の端より先の点に対応を付けないか**（曲がり角で
回頭を弱める向きに偏る原因だった）を縛る。
"""

import math
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

import numpy as np  # noqa: E402

from slam2d.core.surfmap import SurfaceMap  # noqa: E402

RES = 0.025


def _wall_points(x0, y0, x1, y1, step=0.01):
    n = max(2, int(math.hypot(x1 - x0, y1 - y0) / step))
    t = np.linspace(0.0, 1.0, n)
    return x0 + (x1 - x0) * t, y0 + (y1 - y0) * t


class TestSurfaceMapGeometry(unittest.TestCase):
    def test_normal_of_a_straight_wall_is_perpendicular(self):
        xs, ys = _wall_points(0.0, 1.0, 2.0, 1.0)
        m = SurfaceMap.from_points(xs, ys, center=(1.0, 1.0), radius=2.0, resolution=RES)
        a = m.associate(np.array([1.0]), np.array([1.10]))
        self.assertTrue(bool(a.valid[0]))
        self.assertTrue(bool(a.planar[0]))
        # 法線は±y方向（壁は x 軸に平行）
        self.assertAlmostEqual(abs(float(a.ny[0])), 1.0, delta=0.05)
        self.assertAlmostEqual(abs(float(a.nx[0])), 0.0, delta=0.05)
        # 点対平面の距離は10cm
        self.assertAlmostEqual(float(a.dist[0]), 0.10, delta=0.01)

    def test_surface_position_is_subcell(self):
        """面の位置はセル中心ではなく**そのセルに落ちた点の平均**。"""
        # セル境界から1/4セルだけずらした壁（セル中心とは 0.6cm ずれる）
        y = 1.0 + RES * 0.25
        xs, ys = _wall_points(0.0, y, 2.0, y)
        m = SurfaceMap.from_points(xs, ys, center=(1.0, 1.0), radius=2.0, resolution=RES)
        a = m.associate(np.array([1.0]), np.array([y + 0.05]))
        self.assertAlmostEqual(float(a.qy[0]), y, delta=0.002)
        self.assertAlmostEqual(float(a.dist[0]), 0.05, delta=0.003)

    def test_round_pole_is_point_to_point(self):
        """柱のように丸い塊は点対点で扱う（平面ではないので法線が決まらない）。"""
        t = np.linspace(0, 2 * math.pi, 60, endpoint=False)
        m = SurfaceMap.from_points(1.0 + 0.05 * np.cos(t), 1.0 + 0.05 * np.sin(t),
                                   center=(1.0, 1.0), radius=1.0, resolution=RES)
        a = m.associate(np.array([1.0]), np.array([1.0]))       # 柱の中心
        self.assertTrue(bool(a.valid[0]))
        self.assertFalse(bool(a.planar[0]))

    def test_unsupported_cell_is_not_used(self):
        """近傍の裏付けが無い孤立セルは対応に使わない（遠くのまばらな点対策）。"""
        m = SurfaceMap.from_points(np.array([1.0]), np.array([1.0]),
                                   center=(1.0, 1.0), radius=1.0, resolution=RES)
        a = m.associate(np.array([1.02]), np.array([1.0]))
        self.assertFalse(bool(a.valid[0]))

    def test_point_beyond_the_end_of_a_wall_is_not_associated(self):
        """壁の端より先に落ちた点は対応させない（直線に延長した面へ引っ張らない）。"""
        xs, ys = _wall_points(0.0, 1.0, 1.0, 1.0)      # x=1.0 で途切れる壁
        m = SurfaceMap.from_points(xs, ys, center=(1.0, 1.0), radius=2.0, resolution=RES)
        # 壁の延長線上（端から30cm先）の点。法線方向の距離はほぼ0だが、対応は付けない
        a = m.associate(np.array([1.30]), np.array([1.0]))
        self.assertFalse(bool(a.valid[0]))
        # 壁の内側（端から手前）の点はちゃんと対応が付く
        b = m.associate(np.array([0.90]), np.array([1.03]))
        self.assertTrue(bool(b.valid[0]))

    def test_far_point_has_no_correspondence(self):
        xs, ys = _wall_points(0.0, 1.0, 2.0, 1.0)
        m = SurfaceMap.from_points(xs, ys, center=(1.0, 1.0), radius=3.0, resolution=RES,
                                   max_dist=0.3)
        a = m.associate(np.array([1.0]), np.array([2.0]))
        self.assertFalse(bool(a.valid[0]))

    def test_unknown_cells_are_not_associated(self):
        occ = np.zeros((40, 40), dtype=bool)
        occ[20, 10:30] = True
        known = np.zeros_like(occ)
        known[:, :20] = True                            # 右半分は「見ていない」
        m = SurfaceMap(occ, resolution=RES, origin=(0.0, 0.0), known=known)
        left = m.associate(np.array([0.30]), np.array([0.53]))
        right = m.associate(np.array([0.65]), np.array([0.53]))
        self.assertTrue(bool(left.valid[0]))
        self.assertFalse(bool(right.valid[0]))

    def test_empty_map(self):
        m = SurfaceMap(np.zeros((10, 10), dtype=bool), resolution=RES, origin=(0.0, 0.0))
        self.assertTrue(m.empty)
        a = m.associate(np.array([0.1]), np.array([0.1]))
        self.assertFalse(bool(a.valid[0]))


class TestNearestLookup(unittest.TestCase):
    def test_lookup_matches_brute_force(self):
        rng = np.random.default_rng(3)
        xs = rng.uniform(0.2, 1.8, 40)
        ys = rng.uniform(0.2, 1.8, 40)
        m = SurfaceMap.from_points(xs, ys, center=(1.0, 1.0), radius=1.0, resolution=0.05,
                                   max_dist=2.0)
        qx = rng.uniform(0.2, 1.8, 50)
        qy = rng.uniform(0.2, 1.8, 50)
        a = m.associate(qx, qy)
        for i in range(qx.size):
            if not a.valid[i] or a.planar[i]:
                continue
            d_true = float(np.min(np.hypot(xs - qx[i], ys - qy[i])))
            # 対応先はセルの平均なので、セル1つぶん（5cm）の誤差までは許す
            self.assertLess(abs(float(a.dist[i]) - d_true), 0.05)


if __name__ == "__main__":
    unittest.main()
