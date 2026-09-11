"""`sim/track.py` のテスト。境界クリップ（`_disc_slices`）が無い旧実装では、
`margin` を極端に小さくした手書きJSON等で `rasterize()` が範囲外スライス代入を
起こしクラッシュしていた（`ValueError: operands could not be broadcast together`）。
このテストはその再現条件と、直接の境界演算（`_disc_slices`）の両方を確認する。
"""

import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))  # surge_mk2/

import numpy as np  # noqa: E402

from sim.track import (  # noqa: E402
    _disc_slices,
    add_offset_discs,
    build_from_points,
    rasterize,
)


class TestDiscSlices(unittest.TestCase):
    """円盤クリップの単体テスト。バグ報告の再現コード
    （`np.zeros((100,100),bool)` に r=3, rw=2, c=50 で範囲外スライス代入）を直接確認する。
    """

    def test_top_edge_overflow_does_not_raise_and_clips(self):
        grid = np.zeros((100, 100), dtype=bool)
        r = 3
        rw, c = 2, 50           # 上端からはみ出す（rw - r = -1）
        yy, xx = np.mgrid[-r:r + 1, -r:r + 1]
        disc = (xx * xx + yy * yy) <= r * r

        sl = _disc_slices(*grid.shape, rw, c, r)
        self.assertIsNotNone(sl)
        gr0, gr1, gc0, gc1, sr0, sr1, sc0, sc1 = sl
        # クラッシュしないことが本題。この代入自体が範囲外スライスなら例外になる
        grid[gr0:gr1, gc0:gc1] |= disc[sr0:sr1, sc0:sc1]
        self.assertEqual(gr0, 0)                # 上端でクリップされている
        self.assertTrue(grid[0:rw + r + 1, c - r:c + r + 1].any())

    def test_fully_outside_grid_returns_none(self):
        # 中心が遠く離れていて円盤が完全に格子の外にあるケース
        self.assertIsNone(_disc_slices(100, 100, -1000, -1000, 3))

    def test_fully_inside_grid_is_unclipped(self):
        sl = _disc_slices(100, 100, 50, 50, 3)
        gr0, gr1, gc0, gc1, sr0, sr1, sc0, sc1 = sl
        self.assertEqual((gr0, gr1, gc0, gc1), (47, 54, 47, 54))
        self.assertEqual((sr0, sr1, sc0, sc1), (0, 7, 0, 7))


class TestRasterizeBoundaryMargin(unittest.TestCase):
    """`margin` を極端に小さくした手書きJSONを想定した回帰テスト。"""

    def test_zero_margin_straight_line_does_not_crash(self):
        # half=0.1, res=0.02 -> r=5セル。margin=0だとpadがちょうど半径分しかなく、
        # 丸め次第で境界セルの円盤が格子からわずかにはみ出す（旧実装は
        # ValueError: operands could not be broadcast together でクラッシュしていた）
        pts = np.array([[0.0, 0.0, 0.0], [1.0, 0.0, 0.0]])
        grid, origin = rasterize(pts, width=0.2, resolution=0.02, margin=0.0)
        self.assertEqual(grid.dtype, bool)
        self.assertGreater(grid.size, 0)
        # 中心線上は掘られて壁ではないはず
        x0, y0 = origin
        col = int(round((0.0 - x0) / 0.02))
        row = int(round((0.0 - y0) / 0.02))
        self.assertFalse(grid[row, col])

    def test_negative_margin_does_not_crash(self):
        pts = np.array([[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [1.0, 1.0, np.pi / 2]])
        grid, origin = rasterize(pts, width=0.2, resolution=0.02, margin=-0.05)
        self.assertEqual(grid.dtype, bool)
        self.assertGreater(grid.size, 0)

    def test_zero_margin_with_divider_does_not_crash(self):
        pts = np.array([[0.0, 0.0, 0.0], [1.0, 0.0, 0.0]])
        grid, origin = rasterize(pts, width=0.2, resolution=0.02, margin=0.0,
                                 divider=[[0.0, 1.0]], divider_width=0.06)
        self.assertEqual(grid.dtype, bool)
        self.assertGreater(grid.size, 0)

    def test_build_from_points_with_tiny_margin_does_not_crash(self):
        pts = np.array([[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [1.0, 0.3, np.pi / 2]])
        meta = {"resolution": 0.02, "width": 0.2, "margin": 0.001, "loop": False}
        result = build_from_points(pts, meta)
        self.assertEqual(result["grid"].dtype, bool)

    def test_add_offset_discs_at_grid_edge_does_not_crash(self):
        # add_offset_discs はもともとクリップ済みだが、rasterize と同じ境界条件下で
        # 引き続き壊れていないことを確認する（回帰防止）
        pts = np.array([[0.0, 0.0, 0.0], [1.0, 0.0, 0.0]])
        grid, origin = rasterize(pts, width=0.2, resolution=0.02, margin=0.0)
        # 中心線からさらに外側（格子端の外）へオフセットした円盤を追加してもクラッシュしない
        add_offset_discs(grid, origin, 0.02, pts,
                         spans=[(0.0, 10.0, 5.0, 0.05)])
        self.assertEqual(grid.dtype, bool)


if __name__ == "__main__":
    unittest.main()
