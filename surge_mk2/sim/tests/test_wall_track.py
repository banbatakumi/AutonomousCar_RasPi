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


def _diamond_loop(r: float, rot: float) -> Loop:
    """原点まわりに `rot` [rad] 回した一辺の斜めな四角形（斜めの壁の光線抜けを調べる用）。"""
    import math
    loop = Loop()
    for k in range(4):
        a = rot + k * math.pi / 2
        loop.add_line((r * math.cos(a), r * math.sin(a)))
    loop.close()
    return loop


class TestExactThickness(unittest.TestCase):
    res = 0.02

    def _wall_cells_across(self, grid, origin, y_line):
        """x=0 の縦線に沿った壁セルの連続数（壁の太さ[セル]）の最大。"""
        col = int(round((0.0 - origin[0]) / self.res))
        column = grid[:, col]
        best = run = 0
        for v in column:
            run = run + 1 if v else 0
            best = max(best, run)
        return best

    def test_default_is_unchanged_double_thickness(self):
        grid, origin = rasterize_walls([_rect_loop(1.0)], 0.03, self.res, 0.2)
        self.assertEqual(self._wall_cells_across(grid, origin, 1.0), 3)   # 6cm

    def test_exact_is_thinner_than_default(self):
        g0, o0 = rasterize_walls([_rect_loop(1.0)], 0.03, self.res, 0.2)
        g1, o1 = rasterize_walls([_rect_loop(1.0)], 0.03, self.res, 0.2,
                                 exact_thickness=True)
        self.assertLess(g1.sum(), g0.sum())
        # 1.5 セル相当: 1〜2 セル幅（指定 3cm に ±半セルの範囲）
        self.assertIn(self._wall_cells_across(g1, o1, 1.0), (1, 2))

    def test_too_thin_raises(self):
        with self.assertRaises(WallExtractionError):
            rasterize_walls([_rect_loop(1.0)], 0.01, self.res, 0.2,
                            exact_thickness=True)

    def test_oblique_walls_do_not_leak(self):
        """斜めの壁（15〜75°）を内側から外側へ撃っても光線が抜けない。"""
        from sim.course import Course
        from pathlib import Path
        for rot_deg in (0, 15, 30, 45, 60, 75):
            grid, origin = rasterize_walls([_diamond_loop(1.5, np.radians(rot_deg))],
                                           0.03, self.res, 0.5, exact_thickness=True)
            course = Course(name="t", path=Path("t"), resolution=self.res, origin=origin,
                            start=(0.0, 0.0, 0.0), grid=grid)
            # 内側の点から全方向へ。外へ抜けると画像外まで進んで壁(画像端)に当たる
            # ＝ 距離が壁までより大きくなるので、壁の外側から跳ね返った距離を見る
            angles = np.linspace(0, 2 * np.pi, 3600, endpoint=False)
            for ox, oy in ((0.0, 0.0), (0.4, -0.3), (-0.6, 0.5)):
                d = course.raycast(ox, oy, angles, 10.0)
                # 菱形の最遠の頂点は 1.5m。内側の点から壁を抜けずに止まれば 3.1m 未満
                self.assertTrue(((d > 0) & (d < 3.1)).all(),
                                f"rot={rot_deg} origin=({ox},{oy}) 光線が抜けた")


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
