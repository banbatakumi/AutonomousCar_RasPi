"""`core/globalmatch.py`（全域の自己位置探索・多解像度の分枝限定法）のテスト。

確認すること:
- 非対称な部屋（`helpers.ROOM`）で、手がかり無しに 1 位が真の姿勢（探索の刻み 4cm・1° の精度）
- 角度を分けて複数回で探しても、1 回で探したのと同じ候補になる（`MatchJob`）
- ヒントの円の外の候補は出さない
- 同じ形が離れた 2 箇所にある地図では、両方を候補に出す（呼び出し側が曖昧と判定できる）
- 決まった姿勢の得点（`score_at`）は、真の姿勢で高く、ずれた姿勢で低い
"""

import math
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

import numpy as np

from slam2d.core import globalmatch
from slam2d.core.grid import OccGrid
from slam2d.core.types import (
    Pose2D,
    ScanPoints,
)
from slam2d.tests.helpers import make_room_points
from slam2d.tests.test_localize import _build_room_grid, _stamp_l_shape


@unittest.skipUnless(globalmatch.available(), "C が組めない環境")
class TestGlobalMatcherRoom(unittest.TestCase):
    def setUp(self):
        self.g = _build_room_grid()
        self.gm = globalmatch.GlobalMatcher(self.g)

    def test_top_candidate_is_the_true_pose(self):
        for true in (Pose2D(3.0, 2.0, 0.1), Pose2D(2.5, 2.4, -2.0), Pose2D(3.4, 1.7, 2.6)):
            pts = make_room_points(*true)
            c = self.gm.match(pts, 8)
            self.assertTrue(c)
            self.assertLess(math.hypot(c[0].x - true.x, c[0].y - true.y), 0.08, (true, c[0]))
            dyaw = math.atan2(math.sin(c[0].yaw - true.yaw), math.cos(c[0].yaw - true.yaw))
            self.assertLess(abs(math.degrees(dyaw)), 2.0)

    def test_chunked_search_matches_one_shot(self):
        pts = make_room_points(3.0, 2.0, 0.1)
        one = self.gm.match(pts, 8)
        job = self.gm.begin(pts, 8, chunks=5)
        steps = 0
        while not job.step():
            steps += 1
        self.assertEqual(steps, 4)
        many = job.result()
        self.assertEqual((one[0].x, one[0].y, one[0].yaw), (many[0].x, many[0].y, many[0].yaw))

    def test_hint_keeps_candidates_inside_the_circle(self):
        pts = make_room_points(3.0, 2.0, 0.1)
        c = self.gm.match(pts, 8, hint=(3.0, 2.0), radius=0.3)
        self.assertTrue(c)
        for r in c:
            self.assertLessEqual(math.hypot(r.x - 3.0, r.y - 2.0), 0.3 + self.gm.step)

    def test_score_at_prefers_the_true_pose(self):
        pts = make_room_points(3.0, 2.0, 0.1)
        good = self.gm.score_at(pts, (3.0, 2.0, 0.1))
        bad = self.gm.score_at(pts, (2.4, 2.0, 0.6))
        self.assertGreater(good, 0.8)
        self.assertLess(bad, 0.6 * good)


@unittest.skipUnless(globalmatch.available(), "C が組めない環境")
class TestGlobalMatcherSymmetry(unittest.TestCase):
    """同じ L 字の壁を離れた 2 箇所に置いた地図（`test_localize` と同じ）。両方が候補に並ぶ。"""

    def test_both_copies_are_candidates(self):
        g = OccGrid(resolution=0.05, size_m=6.0)
        cells = _stamp_l_shape(g, col0=20, row0=20)
        _stamp_l_shape(g, col0=90, row0=20)
        g.freeze()
        g.seq = 1
        qx, qy = (float(v) for v in g.to_world(18, 18))
        xs, ys = [], []
        for c, r in cells:
            wx, wy = g.to_world(c, r)
            xs.append(wx - qx)
            ys.append(wy - qy)
        pts = ScanPoints(np.array(xs), np.array(ys), np.ones(len(xs), dtype=bool), 0, True)
        cand = globalmatch.GlobalMatcher(g).match(pts, 6, sep_m=1.0)
        self.assertGreaterEqual(len(cand), 2)
        top2 = sorted(cand[:2], key=lambda r: r.x)
        self.assertGreater(top2[1].x - top2[0].x, 3.0)
        self.assertGreater(top2[1].score, 0.9 * top2[0].score)


if __name__ == "__main__":
    unittest.main()
