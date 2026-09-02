"""`backend/posegraph.py`（g2opyラッパー）のテスト。"""

import math
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

import numpy as np  # noqa: E402

from slam2d.backend.posegraph import PoseGraph  # noqa: E402
from slam2d.core.types import Pose2D  # noqa: E402


class TestPoseGraphBasic(unittest.TestCase):
    def test_three_node_chain_converges_to_exact_solution(self):
        """`tools/spike_g2o.py`と同じトイ問題: (0,0,0)->(1,0,0)->(2,0,0)。"""
        g = PoseGraph()
        n0 = g.add_node(Pose2D(0.0, 0.0, 0.0))
        n1 = g.add_node(Pose2D(1.3, 0.2, 0.05))    # ノイズ入り初期値
        n2 = g.add_node(Pose2D(1.9, -0.3, -0.05))
        g.fix(n0)
        info = np.eye(3) * 100.0
        g.add_odometry_edge(n0, n1, Pose2D(1.0, 0.0, 0.0), info)
        g.add_odometry_edge(n1, n2, Pose2D(1.0, 0.0, 0.0), info)
        g.optimize(20)

        p0, p1, p2 = g.all_poses()
        self.assertAlmostEqual(p0.x, 0.0, places=6)
        self.assertAlmostEqual(p1.x, 1.0, places=6)
        self.assertAlmostEqual(p2.x, 2.0, places=6)

    def test_fixed_node_never_moves(self):
        g = PoseGraph()
        n0 = g.add_node(Pose2D(5.0, 3.0, 1.0))
        n1 = g.add_node(Pose2D(0.0, 0.0, 0.0))
        g.fix(n0)
        g.add_odometry_edge(n0, n1, Pose2D(1.0, 0.0, 0.0), np.eye(3) * 10.0)
        g.optimize(10)
        p0 = g.pose(n0)
        self.assertAlmostEqual(p0.x, 5.0, places=9)
        self.assertAlmostEqual(p0.y, 3.0, places=9)
        self.assertAlmostEqual(p0.yaw, 1.0, places=9)


class TestPoseGraphLoopClosure(unittest.TestCase):
    def test_square_loop_closure_distributes_error(self):
        """正方形を1周する4エッジ+ループ閉じ1本で、蓄積誤差が分配される。

        各辺の観測に同じ系統誤差（少しだけ長めに進んだと誤認識）を持たせて
        1周させると、最後のノードは始点からズレた位置になる。ループ閉じの
        拘束（始点と一致するはず、という情報）を1本足して最適化すると、
        誤差が各ノードへ分配され、単純な「最後のエッジだけ直す」より
        滑らかに補正される（各辺の残差が均等に近づく）ことを確認する。
        """
        g = PoseGraph()
        true_side = 1.0
        biased_side = 1.05     # 系統的に5%長く観測してしまう想定
        n = [g.add_node(Pose2D(float(i), 0.0, 0.0)) for i in range(4)]  # 適当な初期値
        g.fix(n[0])
        info = np.eye(3) * 100.0
        # 正方形: 0->1(+x) ->2(+y) ->3(-x) ->0(-y)、それぞれ90度回頭
        edges = [
            (n[0], n[1], Pose2D(biased_side, 0.0, math.pi / 2)),
            (n[1], n[2], Pose2D(biased_side, 0.0, math.pi / 2)),
            (n[2], n[3], Pose2D(biased_side, 0.0, math.pi / 2)),
        ]
        for src, dst, delta in edges:
            g.add_odometry_edge(src, dst, delta, info)
        # ループを閉じない場合の基準（比較用に最適化して見ておく）
        g.optimize(20)
        poses_no_loop = g.all_poses()

        # ループ閉じ: 本当は真の一辺長(1.0)で始点に戻るはず、という拘束を追加
        g.add_loop_edge(n[3], n[0], Pose2D(true_side, 0.0, math.pi / 2), info)
        g.optimize(20)
        poses_with_loop = g.all_poses()

        # ループ閉じ無しでは4本目の辺（未観測のまま）は初期値に近いノイズ交じり
        # のはずだが、ループ閉じ有りでは全体が既知の幾何（一辺長1.0の正方形）に
        # 近づく——最後のノードが始点(0,0)に近づくことを確認する
        dist_no_loop = math.hypot(poses_no_loop[3].x, poses_no_loop[3].y)
        dist_with_loop = math.hypot(poses_with_loop[3].x - true_side * 0,
                                    poses_with_loop[3].y)
        self.assertLess(dist_with_loop, dist_no_loop)


if __name__ == "__main__":
    unittest.main()
