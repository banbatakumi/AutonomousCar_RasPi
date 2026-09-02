"""`backend/loop_detection.py`（汎用ループ検出）のテスト。"""

import math
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

import numpy as np  # noqa: E402

from slam2d.backend.loop_detection import LoopDetectorConfig, find_loop_closure  # noqa: E402
from slam2d.core.grid import OccGrid  # noqa: E402
from slam2d.core.types import Pose2D  # noqa: E402
from slam2d.tests.helpers import ROOM, make_room_points  # noqa: E402

_INFO = np.eye(3) * 100.0


def _build(poses, *, noise_sigma=0.0, rng=None):
    """poses(のリスト)から、地図とkeyframesリストを両方作る。"""
    grid = OccGrid(resolution=0.05, size_m=12.0, origin=(-1.0, -1.0))
    keyframes = []
    for x, y, yaw in poses:
        pts = make_room_points(x, y, yaw, segs=ROOM, max_range=8.0,
                               noise_sigma=noise_sigma, rng=rng)
        grid.integrate(pts, Pose2D(x, y, yaw))
        keyframes.append((Pose2D(x, y, yaw), pts, _INFO))
    return grid, keyframes


class TestFindLoopClosure(unittest.TestCase):
    def test_revisit_is_detected(self):
        """離れた場所を回ってから最初の位置に戻ると、ループとして検出される。"""
        path = [(3.0, 2.0, 0.0)] * 5           # 最初の位置で地図を育てる(index 0-4)
        path += [(3.0 + 0.1 * i, 2.0, 0.0) for i in range(1, 25)]  # 遠ざかる(index 5-28)
        path += [(3.0, 2.0, 0.0)]               # 最初の位置に戻る(index 29)
        grid, keyframes = _build(path)

        cand = find_loop_closure(keyframes, grid, len(keyframes) - 1,
                                 config=LoopDetectorConfig(min_index_gap=10))
        self.assertIsNotNone(cand)
        self.assertLess(cand.src, 5)   # 最初期のキーフレームと対応するはず
        self.assertGreater(cand.score, 0.5)
        # 検出された相対姿勢はほぼ恒等姿勢のはず(同じ場所に戻ったので)
        self.assertAlmostEqual(cand.delta.x, 0.0, delta=0.05)
        self.assertAlmostEqual(cand.delta.y, 0.0, delta=0.05)

    def test_recent_keyframe_is_not_a_false_positive(self):
        """直近のキーフレーム同士が近いのは当然なので、ループとして検出しない。"""
        path = [(3.0 + 0.01 * i, 2.0, 0.0) for i in range(10)]
        grid, keyframes = _build(path)
        cand = find_loop_closure(keyframes, grid, len(keyframes) - 1,
                                 config=LoopDetectorConfig(min_index_gap=20))
        self.assertIsNone(cand)

    def test_no_closure_when_map_is_too_small(self):
        grid = OccGrid(resolution=0.05, size_m=12.0, origin=(-1.0, -1.0))
        pts = make_room_points(3.0, 2.0, 0.0, segs=ROOM, max_range=8.0)
        keyframes = [(Pose2D(3.0, 2.0, 0.0), pts, _INFO)] * 25
        cand = find_loop_closure(keyframes, grid, 24,
                                 config=LoopDetectorConfig(min_index_gap=20))
        # 地図が空(誰もintegrateしていない)なので探索できず None
        self.assertIsNone(cand)


class TestFigureEightNoBreakage(unittest.TestCase):
    def test_two_separate_loops_both_detected(self):
        """8の字（2つの交差する円環）で、それぞれのループが独立に検出できる。

        交差点を2回通るシナリオを簡略化し、「同じ場所に2回戻る」パターンが
        1回だけの検出で壊れないことを確認する（`min_index_gap`は毎回の
        find_loop_closure呼び出しに対して独立に働くので、複数回の検出は
        呼び出し側が異なる`new_index`で繰り返し呼べばよい、という設計を
        小さく検証する）。
        """
        # 交差点(3,2,0)を起点に、輪1へ行って戻り、輪2へ行って戻る
        path = [(3.0, 2.0, 0.0)] * 5
        path += [(3.0 + 0.1 * i, 2.0, 0.0) for i in range(1, 15)]      # 輪1
        path += [(3.0, 2.0, 0.0)]                                       # 交差点に戻る(1回目)
        loop1_index = len(path) - 1
        path += [(3.0, 2.0 + 0.1 * i, 0.0) for i in range(1, 15)]      # 輪2
        path += [(3.0, 2.0, 0.0)]                                       # 交差点に戻る(2回目)
        loop2_index = len(path) - 1
        grid, keyframes = _build(path)

        cfg = LoopDetectorConfig(min_index_gap=10)
        cand1 = find_loop_closure(keyframes, grid, loop1_index, config=cfg)
        cand2 = find_loop_closure(keyframes, grid, loop2_index, config=cfg)
        self.assertIsNotNone(cand1)
        self.assertIsNotNone(cand2)
        self.assertLess(cand1.src, 5)
        self.assertLess(cand2.src, 5)


if __name__ == "__main__":
    unittest.main()
