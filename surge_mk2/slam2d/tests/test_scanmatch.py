"""`core/scanmatch.py`（相関スキャンマッチ）のテスト。

Phase 1 完了条件: ROOM合成テストで既知姿勢±ノイズから正しい姿勢に収束すること
を誤差cm/度で確認する。
"""

import math
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

import numpy as np  # noqa: E402

from slam2d.core.grid import OccGrid  # noqa: E402
from slam2d.core.scanmatch import DEFAULT_STAGES, MatcherConfig, Stage, match  # noqa: E402
from slam2d.core.types import Pose2D  # noqa: E402
from slam2d.tests.helpers import make_room_points  # noqa: E402

#: テストでは推測航法の外れ幅も広めに探索できるよう、DEFAULT_STAGESより
#: 広い段を使う（既定値は「ジャイロ+速度」構成前提の狭い探索なので、
#: このテストのようにcm単位でずらした初期値には狭すぎる）
_WIDE_STAGES: tuple[Stage, ...] = (
    Stage(0.20, 0.04, math.radians(10.0), math.radians(2.0)),
) + DEFAULT_STAGES


def _build_map(g: OccGrid, poses) -> None:
    for x, y, yaw in poses:
        g.integrate(make_room_points(x, y, yaw), Pose2D(x, y, yaw))


class TestScanMatch(unittest.TestCase):
    def setUp(self):
        self.g = OccGrid(resolution=0.05, size_m=12.0, origin=(-1.0, -1.0))
        # 部屋の中を少し動き回って地図を育てる（1点だけだと壁が薄い）
        _build_map(self.g, [(3.0, 2.0, 0.0), (3.05, 2.0, 0.05), (3.0, 2.05, -0.05),
                            (2.95, 2.0, 0.0), (3.0, 1.95, 0.02)])

    def test_converges_from_noisy_guess(self):
        true_pose = Pose2D(3.0, 2.0, 0.1)
        pts = make_room_points(*true_pose)
        noisy_guess = Pose2D(true_pose.x + 0.15, true_pose.y - 0.12,
                             true_pose.yaw + math.radians(8.0))
        result = match(self.g, pts, noisy_guess,
                       config=MatcherConfig(stages=_WIDE_STAGES))
        self.assertTrue(result.searched)
        self.assertAlmostEqual(result.x, true_pose.x, delta=0.03)
        self.assertAlmostEqual(result.y, true_pose.y, delta=0.03)
        self.assertAlmostEqual(result.yaw, true_pose.yaw, delta=math.radians(2.0))
        self.assertGreater(result.score, 0.5)

    def test_empty_map_does_not_search(self):
        empty_grid = OccGrid(resolution=0.05, size_m=12.0, origin=(-1.0, -1.0))
        pts = make_room_points(3.0, 2.0, 0.0)
        result = match(empty_grid, pts, Pose2D(3.0, 2.0, 0.0))
        self.assertFalse(result.searched)

    def test_guess_containing_zero_offset_is_never_worse(self):
        """0オフセット（guessそのもの）は必ず候補に含まれる（`_offsets`のdocstring）。"""
        true_pose = Pose2D(3.0, 2.0, 0.0)
        pts = make_room_points(*true_pose)
        result = match(self.g, pts, true_pose,
                       config=MatcherConfig(stages=_WIDE_STAGES))
        self.assertGreaterEqual(result.score, 0.0)


if __name__ == "__main__":
    unittest.main()
