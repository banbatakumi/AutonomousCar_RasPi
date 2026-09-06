"""`sim/sketch.py` のテスト。頂点/円弧ループのジオメトリと往復変換だけを確認する。"""

import math
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))  # surge_mk2/

import numpy as np  # noqa: E402

from sim.sketch import (  # noqa: E402
    Loop,
    loop_to_path,
    path_to_loop,
    sample_loop,
    snap_to_grid,
    snap_to_vertex,
)
from sim.track import centerline as track_centerline  # noqa: E402


class TestSnap(unittest.TestCase):
    def test_grid_snap(self):
        self.assertEqual(snap_to_grid((0.13, 0.24), 0.1), (0.1, 0.2))

    def test_grid_snap_off(self):
        self.assertEqual(snap_to_grid((0.13, 0.24), 0.0), (0.13, 0.24))

    def test_vertex_snap_within_radius(self):
        verts = [(10.0, 10.0), (100.0, 100.0)]
        self.assertEqual(snap_to_vertex((12.0, 10.0), verts, pixel_radius=5.0), 0)

    def test_vertex_snap_out_of_radius(self):
        verts = [(10.0, 10.0)]
        self.assertIsNone(snap_to_vertex((30.0, 10.0), verts, pixel_radius=5.0))


class TestLoopToPath(unittest.TestCase):
    def test_square_inserts_turns_at_corners(self):
        loop = Loop()
        loop.add_line((0.0, 0.0))
        loop.add_line((2.0, 0.0))
        loop.add_line((2.0, 2.0))
        loop.add_line((0.0, 2.0))
        loop.close()

        origin, path = loop_to_path(loop)
        self.assertAlmostEqual(origin[0], 0.0)
        self.assertAlmostEqual(origin[1], 0.0)
        # 始点の向きは「最後の辺(辺3, -y向き)を抜けた向き」にそろえる
        self.assertAlmostEqual(origin[2], -math.pi / 2, places=6)

        kinds = [seg[0] for seg in path]
        self.assertEqual(kinds.count("straight"), 4)
        self.assertEqual(kinds.count("turn"), 4)      # 4隅すべてに turn が挟まる
        total_turn = sum(seg[1] for seg in path if seg[0] == "turn")
        self.assertAlmostEqual(total_turn, 360.0, places=6)

        pts = track_centerline(path, *origin, step=0.05)
        # 閉じたループなので終点の向きは始点の向きにぴったり一致する（2πの差は無視）
        dyaw = (float(pts[-1, 2]) - float(pts[0, 2]) + math.pi) % (2 * math.pi) - math.pi
        self.assertAlmostEqual(dyaw, 0.0, places=6)

    def test_drag_arc_is_tangent_to_preceding_line(self):
        """ドラッグ＝直線(ドラッグ開始点まで)＋そこから接線円弧、という新しい操作を
        直接ジオメトリで確認する。直線の向きは +x（0 rad）、円弧の通過先が
        +y 側にあるので、円弧は左（+y側）へ膨らむはず。始点の接線方向は必ず
        直前の直線と一致する。
        """
        loop = Loop()
        loop.add_line((0.0, 0.0))
        loop.add_line((2.0, 0.0))          # ドラッグ開始位置までの直線。向き=0
        loop.add_arc((4.0, 1.0))           # ドラッグを離した位置

        origin, path = loop_to_path(loop)
        self.assertEqual([seg[0] for seg in path], ["straight", "arc"])
        self.assertAlmostEqual(origin[2], 0.0, places=6)   # 最初の辺(直線)の向き

        pts = sample_loop(loop, step=0.02)
        # 円弧に入る瞬間(直線の終点=ドラッグ開始位置)の向きは直線と同じ 0 rad のはず
        idx = int(np.argmin(np.hypot(pts[:, 0] - 2.0, pts[:, 1] - 0.0)))
        self.assertAlmostEqual(float(pts[idx, 2]), 0.0, places=2)
        # 円弧は終点(4,1)がある+y側へ膨らむ
        self.assertTrue(np.any(pts[idx:, 1] > 0.05))
        # 終点は指定した位置に一致する
        self.assertAlmostEqual(float(pts[-1, 0]), 4.0, places=2)
        self.assertAlmostEqual(float(pts[-1, 1]), 1.0, places=2)

    def test_arc_degenerate_falls_back_to_straight(self):
        """終点が直線の延長線上にある(=円弧にできない)場合は直線として扱う。"""
        loop = Loop()
        loop.add_line((0.0, 0.0))
        loop.add_line((2.0, 0.0))   # 向き=0
        loop.add_arc((4.0, 0.0))   # 延長線上 → 縮退

        _origin, path = loop_to_path(loop)
        self.assertEqual([seg[0] for seg in path], ["straight", "straight"])

    def test_arc_as_first_edge_without_start_heading_raises(self):
        """最初の辺が円弧のとき、接線の元になる「直前の辺」も `start_heading` も
        無ければ解けない。エディタは常に直線→円弧の順で頂点を増やすので、
        この状況は「壊れたループ」の検出として機能する。
        """
        loop = Loop()
        loop.add_line((0.0, 0.0))
        loop.add_arc((2.0, 0.0))
        with self.assertRaises(ValueError):
            loop_to_path(loop)


class TestPathRoundTrip(unittest.TestCase):
    def test_straight_arc_path_round_trips(self):
        origin = (1.0, 1.0, 0.0)
        path = [["straight", 4.0], ["arc", 1.2, 90], ["straight", 2.0], ["arc", 1.2, 90]]
        loop = path_to_loop(origin, path)
        new_origin, new_path = loop_to_path(loop)

        self.assertAlmostEqual(new_origin[0], origin[0], places=6)
        self.assertAlmostEqual(new_origin[1], origin[1], places=6)
        self.assertAlmostEqual(new_origin[2], origin[2], places=6)
        self.assertEqual([seg[0] for seg in new_path], [seg[0] for seg in path])
        for old, new in zip(path, new_path):
            for a, b in zip(old[1:], new[1:]):
                self.assertAlmostEqual(a, b, places=5)

        expect = track_centerline(path, *origin, step=0.05)
        got = sample_loop(loop, step=0.05)
        self.assertEqual(expect.shape, got.shape)
        np.testing.assert_allclose(got, expect, atol=1e-6)


if __name__ == "__main__":
    unittest.main()
