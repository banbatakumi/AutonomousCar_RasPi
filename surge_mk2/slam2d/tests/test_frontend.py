"""`core/frontend.py`（1周期の処理）のテスト。

縛る性質:

- 止まっていれば姿勢は動かない／キーフレームは増えない
- 動けばキーフレームが増え、地図が育つ
- **見失っても地図作成を永久に止めない**（旧実装が落ちた「見失い→地図を更新
  しない→永久に見失う」の再発防止）
- 凍結地図では地図を焼かない（追跡専用）
- `apply_correction()`で地図・軌跡・今の姿勢がまとめて補正される
"""

import math
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

import numpy as np  # noqa: E402

from slam2d.core.frontend import Frontend, FrontendConfig  # noqa: E402
from slam2d.core.grid import OccGrid  # noqa: E402
from slam2d.core.motion import ConstantVelocityModel, ExternalTwistModel  # noqa: E402
from slam2d.core.types import Pose2D, Twist2D, compose  # noqa: E402
from slam2d.tests.helpers import ROOM, make_raw_scan  # noqa: E402


def _make_frontend(motion=None, **config_kwargs) -> Frontend:
    grid = OccGrid(resolution=0.025, size_m=14.0, origin=(-2.0, -2.0))
    if motion is None:
        motion = ConstantVelocityModel()
    return Frontend(grid, motion, FrontendConfig(max_range=8.0, **config_kwargs))


class TestStationary(unittest.TestCase):
    def test_pose_does_not_drift(self):
        fe = _make_frontend()
        for _ in range(10):
            u = fe.update(make_raw_scan(3.0, 2.0, 0.0, segs=ROOM, max_range=8.0), 0.1)
        self.assertAlmostEqual(fe.pose.x, 0.0, delta=0.01)
        self.assertAlmostEqual(fe.pose.y, 0.0, delta=0.01)
        self.assertFalse(u.lost)

    def test_only_one_keyframe(self):
        fe = _make_frontend()
        for _ in range(10):
            fe.update(make_raw_scan(3.0, 2.0, 0.0, segs=ROOM, max_range=8.0), 0.1)
        self.assertEqual(len(fe.keyframes), 1)


class TestMoving(unittest.TestCase):
    def test_tracks_a_straight_run(self):
        """既知の速度で進む間、推定した移動量が真の移動量に一致する。"""
        motion = ExternalTwistModel(lambda: Twist2D(0.2, 0.0, 0.0))
        fe = _make_frontend(motion=motion, kf_dist=0.05)
        for i in range(25):
            u = fe.update(make_raw_scan(3.0 + 0.02 * i, 2.0, 0.0, segs=ROOM, max_range=8.0), 0.1)
        self.assertFalse(u.lost)
        self.assertAlmostEqual(fe.pose.x, 0.02 * 24, delta=0.02)
        self.assertAlmostEqual(fe.pose.y, 0.0, delta=0.02)
        self.assertGreater(len(fe.keyframes), 3)
        self.assertEqual(len(fe.trajectory), len(fe.keyframes))
        self.assertGreater(int(fe.grid.wall_mask().sum()), 100)

    def test_tracks_rotation(self):
        motion = ExternalTwistModel(lambda: Twist2D(0.0, 0.0, math.radians(20.0)))
        fe = _make_frontend(motion=motion, kf_yaw=math.radians(2.0))
        for i in range(20):
            fe.update(make_raw_scan(3.0, 2.0, math.radians(2.0 * i), segs=ROOM,
                                    max_range=8.0), 0.1)
        self.assertAlmostEqual(math.degrees(fe.pose.yaw), 2.0 * 19, delta=1.0)


class TestLost(unittest.TestCase):
    def _drive(self, fe, n=15):
        for i in range(n):
            fe.update(make_raw_scan(3.0 + 0.01 * i, 2.0, 0.0, segs=ROOM, max_range=8.0), 0.1)

    def test_unrelated_scan_is_lost(self):
        motion = ExternalTwistModel(lambda: Twist2D(0.1, 0.0, 0.0))
        fe = _make_frontend(motion=motion, kf_dist=0.02)
        self._drive(fe)
        weird = [(0.0, 0.0, 1.0, 3.0), (1.0, 3.0, -2.0, 1.0)]
        u = fe.update(make_raw_scan(3.0, 2.0, 0.0, segs=weird, max_range=8.0), 0.1)
        self.assertTrue(u.lost)

    def test_lost_scan_does_not_grow_the_map(self):
        motion = ExternalTwistModel(lambda: Twist2D(0.1, 0.0, 0.0))
        fe = _make_frontend(motion=motion, kf_dist=0.02)
        self._drive(fe)
        before = fe.grid.hits.copy()
        weird = [(0.0, 0.0, 1.0, 3.0)]
        fe.update(make_raw_scan(3.0, 2.0, 0.0, segs=weird, max_range=8.0), 0.1)
        self.assertTrue(np.array_equal(before, fe.grid.hits))

    def test_mapping_restarts_after_a_long_loss(self):
        """見失い続けたら局所地図を作り直して走り続ける（永久に止まらない）。"""
        motion = ExternalTwistModel(lambda: Twist2D(0.1, 0.0, 0.0))
        fe = _make_frontend(motion=motion, kf_dist=0.02, restart_after=5)
        self._drive(fe)
        weird = [(0.0, 0.0, 1.0, 3.0), (1.0, 3.0, -2.0, 1.0)]
        for _ in range(8):
            fe.update(make_raw_scan(3.0, 2.0, 0.0, segs=weird, max_range=8.0), 0.1)
        self.assertGreaterEqual(fe.restarts, 1)
        # 作り直した後は、新しい環境に対して見失っていない
        u = fe.update(make_raw_scan(3.0, 2.0, 0.0, segs=weird, max_range=8.0), 0.1)
        self.assertFalse(u.lost)

    def test_jump_is_recovered_by_wide_search(self):
        """予測が大きく外れても、総当たりで探し直して復帰する。"""
        motion = ExternalTwistModel(lambda: Twist2D(0.0, 0.0, 0.0))
        fe = _make_frontend(motion=motion, kf_dist=0.02)
        self._drive(fe, 20)
        fe.pose = Pose2D(fe.pose.x + 0.25, fe.pose.y - 0.2, fe.pose.yaw)   # 瞬間移動させる
        for _ in range(4):
            u = fe.update(make_raw_scan(3.19, 2.0, 0.0, segs=ROOM, max_range=8.0), 0.1)
        self.assertFalse(u.lost)
        self.assertAlmostEqual(fe.pose.x, 0.19, delta=0.05)
        self.assertAlmostEqual(fe.pose.y, 0.0, delta=0.05)


class TestFrozenMap(unittest.TestCase):
    def test_frozen_map_is_not_modified_and_tracking_continues(self):
        motion = ExternalTwistModel(lambda: Twist2D(0.2, 0.0, 0.0))
        fe = _make_frontend(motion=motion, kf_dist=0.05)
        for i in range(20):
            fe.update(make_raw_scan(3.0 + 0.02 * i, 2.0, 0.0, segs=ROOM, max_range=8.0), 0.1)
        fe.grid.freeze()
        hits = fe.grid.hits.copy()
        n_kf = len(fe.keyframes)
        for i in range(20, 40):
            u = fe.update(make_raw_scan(3.0 + 0.02 * i, 2.0, 0.0, segs=ROOM, max_range=8.0), 0.1)
        self.assertFalse(u.lost)
        self.assertTrue(np.array_equal(hits, fe.grid.hits))
        self.assertEqual(len(fe.keyframes), n_kf)       # 凍結後はキーフレームも増えない
        self.assertAlmostEqual(fe.pose.x, 0.02 * 39, delta=0.03)


class TestApplyCorrection(unittest.TestCase):
    def test_correction_moves_map_trajectory_and_pose(self):
        motion = ExternalTwistModel(lambda: Twist2D(0.2, 0.0, 0.0))
        fe = _make_frontend(motion=motion, kf_dist=0.05)
        for i in range(20):
            fe.update(make_raw_scan(3.0 + 0.02 * i, 2.0, 0.0, segs=ROOM, max_range=8.0), 0.1)
        old_grid = fe.grid
        old_pose = fe.pose
        last_kf = fe.keyframes[-1].pose
        shift = Pose2D(0.5, 0.25, 0.0)
        poses = [compose(shift, k.pose) for k in fe.keyframes]
        fe.apply_correction(poses)
        self.assertIsNot(fe.grid, old_grid)
        self.assertGreater(fe.grid.seq, old_grid.seq)
        self.assertEqual(len(fe.trajectory), len(poses))
        self.assertAlmostEqual(fe.trajectory[-1].x, poses[-1].x, places=6)
        # 今の姿勢も同じだけ動く（最後のキーフレームとの相対関係は保たれる）
        self.assertAlmostEqual(fe.pose.x - old_pose.x, 0.5, delta=1e-6)
        self.assertAlmostEqual(fe.pose.y - old_pose.y, 0.25, delta=1e-6)
        self.assertAlmostEqual(fe.pose.x - poses[-1].x, old_pose.x - last_kf.x, delta=1e-6)
        self.assertGreater(int(fe.grid.wall_mask().sum()), 100)

    def test_rejects_mismatched_length(self):
        fe = _make_frontend()
        fe.update(make_raw_scan(3.0, 2.0, 0.0, segs=ROOM, max_range=8.0), 0.1)
        with self.assertRaises(ValueError):
            fe.apply_correction([])


if __name__ == "__main__":
    unittest.main()
