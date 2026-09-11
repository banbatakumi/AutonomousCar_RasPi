"""`sim/wall_track.py` のテスト。矩形リングを手で組み立て、壁のラスタライズ→
骨格化→中心線抽出のパイプラインが壊れていないことを確認する。
"""

import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))  # surge_mk2/

import numpy as np  # noqa: E402

from sim.sketch import Loop  # noqa: E402
from sim.wall_track import (  # noqa: E402
    WallExtractionError,
    build,
    derive_centerline,
    rasterize_walls,
)

_RESOLUTION = 0.02
_THICKNESS = 0.05
_MARGIN = 0.2


def _rect_loop(half: float) -> Loop:
    loop = Loop()
    loop.add_line((-half, -half))
    loop.add_line((half, -half))
    loop.add_line((half, half))
    loop.add_line((-half, half))
    loop.close()
    return loop


def _ring_grid():
    outer = _rect_loop(1.0)
    inner = _rect_loop(0.5)
    grid, origin = rasterize_walls([outer, inner], _THICKNESS, _RESOLUTION, _MARGIN)
    return grid, origin


class TestRasterizeWalls(unittest.TestCase):
    def test_no_loops_raises(self):
        with self.assertRaises(WallExtractionError):
            rasterize_walls([], _THICKNESS, _RESOLUTION, _MARGIN)

    def test_walls_are_thin_not_filled(self):
        grid, origin = _ring_grid()
        # 内側のループの中心(0,0)は壁ではなく、壁で囲まれた自由空間のはず
        h, w = grid.shape
        col = int(round((0.0 - origin[0]) / _RESOLUTION))
        row = int(round((0.0 - origin[1]) / _RESOLUTION))
        self.assertFalse(grid[row, col])
        # 全部埋まっているわけではない(中心線モードの「掘る」方式と違い、壁は薄い)
        self.assertLess(grid.sum(), grid.size * 0.5)

    def test_zero_margin_does_not_crash(self):
        """境界クリップが無い旧実装は、marginを0（またはそれ以下）にすると
        `pad`がちょうど円盤の半径分しかなくなり、丸め次第で境界の壁スタンプが
        格子からわずかにはみ出して`ValueError: operands could not be
        broadcast together`でクラッシュしていた（手書きJSONでmarginを
        0以下にするケースの再現）。"""
        outer = _rect_loop(0.5)
        for margin in (0.0, -0.01, -0.02):
            grid, origin = rasterize_walls([outer], _THICKNESS, _RESOLUTION, margin)
            self.assertEqual(grid.dtype, np.bool_)
            self.assertGreater(grid.size, 0)


class TestDeriveCenterline(unittest.TestCase):
    def test_annulus_start_yields_closed_centerline(self):
        grid, origin = _ring_grid()
        cl = derive_centerline(grid, origin, _RESOLUTION, start_xy=(0.75, 0.0))
        self.assertGreater(len(cl), 10)
        # 中心線は外周(半径1.0)と内周(半径0.5)の間、おおむね真ん中(0.75)付近を通る
        radii = np.maximum(np.abs(cl[:, 0]), np.abs(cl[:, 1]))
        self.assertTrue(np.all(radii > 0.5))
        self.assertTrue(np.all(radii < 1.0))
        self.assertTrue(np.all(np.abs(radii - 0.75) < 0.2))

    def test_start_inside_isolated_island_fails(self):
        """内側のループの中は壁で完全に囲まれた孤立領域（袋小路すらない単純な
        塊）なので、骨格化してもループが取れずエラーになるはず。"""
        grid, origin = _ring_grid()
        with self.assertRaises(WallExtractionError):
            derive_centerline(grid, origin, _RESOLUTION, start_xy=(0.0, 0.0))

    def test_start_inside_wall_fails(self):
        grid, origin = _ring_grid()
        with self.assertRaises(WallExtractionError):
            derive_centerline(grid, origin, _RESOLUTION, start_xy=(1.0, 0.0))


class TestBuild(unittest.TestCase):
    def test_build_round_trip(self):
        from sim.sketch import loop_to_path

        outer_origin, outer_path = loop_to_path(_rect_loop(1.0))
        inner_origin, inner_path = loop_to_path(_rect_loop(0.5))
        grid, origin = rasterize_walls([_rect_loop(1.0), _rect_loop(0.5)],
                                       _THICKNESS, _RESOLUTION, _MARGIN)
        centerline = derive_centerline(grid, origin, _RESOLUTION, start_xy=(0.75, 0.0))

        meta = {
            "mode": "wall", "resolution": _RESOLUTION,
            "wall_thickness": _THICKNESS, "margin": _MARGIN,
            "walls": [
                {"origin": list(outer_origin), "path": outer_path},
                {"origin": list(inner_origin), "path": inner_path},
            ],
            "centerline": centerline.tolist(),
            "start": centerline[0].tolist(),
        }
        t = build(meta)
        self.assertIsNone(t["width"])
        self.assertEqual(t["grid"].shape, grid.shape)
        np.testing.assert_allclose(t["centerline"], centerline)


if __name__ == "__main__":
    unittest.main()
