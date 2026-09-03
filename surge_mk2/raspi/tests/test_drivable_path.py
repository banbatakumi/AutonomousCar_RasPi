"""`raspi/nav/drivable_path.py`（`extract_centerline`）のテスト。

**合成した占有格子から答え合わせする。** IPM もモデルも要らない——
`raspi/nav/grid.py` の `OccGrid` に直接壁を描いて、中心線がその形どおりに
出てくることだけを確認する。
"""

import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

import numpy as np  # noqa: E402

from raspi.nav.drivable_path import extract_centerline  # noqa: E402
from raspi.nav.grid import OccGrid  # noqa: E402


def _empty_blocked(grid: OccGrid) -> np.ndarray:
    return np.zeros((grid.height, grid.width), dtype=bool)


class TestExtractCenterline(unittest.TestCase):
    def test_open_straight_corridor_is_centered_at_zero(self):
        """左右どちらにも壁が無ければ、中心は常に y=0（車両の前後軸そのまま）。"""
        grid = OccGrid(resolution=0.05, size_m=6.0)
        blocked = _empty_blocked(grid)

        xs, ys, widths = extract_centerline(grid, blocked, x_min=0.3, x_max=2.0, x_step=0.1)

        self.assertEqual(len(xs), 18)          # (2.0-0.3)/0.1 + 1
        self.assertAlmostEqual(xs[0], 0.3, places=6)
        self.assertAlmostEqual(xs[-1], 2.0, places=6)
        for y in ys:
            # グリッドの離散化（解像度0.05m）による量子化誤差はセル1つぶんまで許容する
            self.assertAlmostEqual(y, 0.0, delta=0.05)
        # 壁が無い側は raycast の探索上限（グリッドの広がり）までしか測れない
        for w in widths:
            self.assertGreater(w, 0.0)

    def test_offset_wall_shifts_center_away_from_it(self):
        """右側だけに壁があれば、中心は左（+y）へ寄る。"""
        grid = OccGrid(resolution=0.05, size_m=6.0)
        blocked = _empty_blocked(grid)
        # x=0.3〜2.0 の範囲、右側 y=-0.3〜-0.2 に壁の帯を置く
        for x in np.arange(0.2, 2.1, 0.02):
            for y in np.arange(-0.35, -0.15, 0.02):
                col, row = grid.to_cell(float(x), float(y))
                if grid.inside(np.array([col]), np.array([row]))[0]:
                    blocked[row, col] = True

        xs, ys, widths = extract_centerline(grid, blocked, x_min=0.5, x_max=1.5, x_step=0.1)

        for y in ys:
            self.assertGreater(y, 0.0, "右に壁があるのに中心が左へ寄っていない")

    def test_narrow_gap_is_reported_with_small_width(self):
        """左右両方に壁があれば、その場所の `width` は壁間の距離に近い値になる。"""
        grid = OccGrid(resolution=0.05, size_m=6.0)
        blocked = _empty_blocked(grid)
        x_target = 1.0
        for x in np.arange(x_target - 0.05, x_target + 0.06, 0.02):
            for y in list(np.arange(-1.0, -0.24, 0.02)) + list(np.arange(0.24, 1.0, 0.02)):
                col, row = grid.to_cell(float(x), float(y))
                if grid.inside(np.array([col]), np.array([row]))[0]:
                    blocked[row, col] = True

        xs, ys, widths = extract_centerline(grid, blocked, x_min=x_target, x_max=x_target,
                                            x_step=0.1)

        self.assertEqual(len(xs), 1)
        # 壁の内側の隙間はおよそ 0.48m（-0.24〜0.24）。太らせ(fill)を含めても
        # 極端に広い/狭い値にはならないはず
        self.assertLess(widths[0], 1.0)
        self.assertGreater(widths[0], 0.2)
        self.assertAlmostEqual(ys[0], 0.0, delta=0.1)

    def test_fully_blocked_reports_zero_width(self):
        """全面が `blocked` なら幅はほぼ0（壁の直中から測るため）。"""
        grid = OccGrid(resolution=0.05, size_m=6.0)
        blocked = np.ones((grid.height, grid.width), dtype=bool)

        xs, ys, widths = extract_centerline(grid, blocked, x_min=0.5, x_max=0.5, x_step=0.1)

        self.assertEqual(len(xs), 1)
        self.assertLess(widths[0], 0.2)

    def test_empty_range_returns_empty_lists(self):
        grid = OccGrid(resolution=0.05, size_m=6.0)
        blocked = _empty_blocked(grid)
        xs, ys, widths = extract_centerline(grid, blocked, x_min=1.0, x_max=0.5, x_step=0.1)
        self.assertEqual((xs, ys, widths), ([], [], []))


if __name__ == "__main__":
    unittest.main()
