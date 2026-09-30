"""`core/frontend.py`（1周期の処理）のテスト。

縛る性質:

- 止まっていれば姿勢は動かない／キーフレームは増えない
- 動けばキーフレームが増え、地図が育つ
- **見失っても地図作成を永久に止めない**（旧実装が落ちた「見失い→地図を更新
  しない→永久に見失う」の再発防止）
- 凍結地図では地図を焼かない（追跡専用）
- `apply_correction()`で地図・軌跡・今の姿勢がまとめて補正される
- 凍結地図で壁の向こうに見えた点を位置合わせから外す（外しすぎたらやめる）
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
from slam2d.tests.helpers import ROOM, make_raw_scan, make_room_points  # noqa: E402


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


def _relocalize_single_stage(fe: Frontend, field_, hx, hy, center: Pose2D):
    """`Frontend._relocalize()`の粗→中→細（Issue #17）にする前の1段総当たり。

    等価性テスト専用のリファレンス実装（削除・移動しないこと）。
    """
    from slam2d.core.register import register, search
    cfg = fe.config
    sr = search(field_, hx, hy, center, trans=cfg.reloc_trans, rot=cfg.reloc_rot,
               trans_step=cfg.reloc_trans_step, rot_step=cfg.reloc_rot_step)
    if sr.score <= 0.0:
        return None
    if sr.second >= cfg.reloc_ambiguity * sr.score:
        return None
    return register(field_, hx, hy, sr.pose, config=cfg.register)


class TestRelocalizeCoarseToFine(unittest.TestCase):
    """Issue #17: `_relocalize()`を粗→中→細の3段にした変更の安全性。

    **精度は落とさず、速くなっている**ことを、凍結した部屋の地図に対して
    複数の見失い姿勢（真の姿勢から±0.35m・±12°ずらした初期値）から探し直し、
    旧・単段総当たり版と比べて確認する。1段版が失敗する（`None`を返す）
    ケースで3段版が成功するのは許容する（探索範囲・曖昧さ判定は同じで、
    段を分けたぶん実質的な刻みが細かくなる場面があるため）。逆に**1段版が
    成功したのに3段版が失敗する、または位置誤差が明確に悪化するのは不可**。
    """

    def _frozen_grid(self) -> OccGrid:
        grid = OccGrid(resolution=0.05, size_m=10.0, min_hits=2, min_seen=2)
        for (x, y, yaw) in [(1.0, 1.0, 0.0), (3.0, 1.0, 0.3), (5.0, 1.0, 0.6),
                            (5.0, 3.0, 1.2), (3.0, 3.0, 2.0), (1.0, 3.0, 2.8),
                            (1.5, 2.0, 1.5), (4.0, 2.0, -0.5)]:
            grid.integrate(make_room_points(x, y, yaw, segs=ROOM), Pose2D(x, y, yaw))
        grid.freeze()
        return grid

    def test_matches_or_improves_on_single_stage_search(self):
        grid = self._frozen_grid()
        rng = np.random.default_rng(0)
        true_poses = [(2.0, 1.5, 0.2), (4.5, 1.5, 1.0), (2.5, 3.3, 2.5),
                     (1.2, 2.5, -1.0), (4.0, 2.8, 0.7)]

        both_recovered_err = []
        old_only = 0
        for (tx, ty, tyaw) in true_poses:
            pts = make_room_points(tx, ty, tyaw, segs=ROOM, noise_sigma=0.01, rng=rng)
            for _ in range(3):
                center = Pose2D(tx + rng.uniform(-0.35, 0.35), ty + rng.uniform(-0.35, 0.35),
                               tyaw + rng.uniform(-math.radians(12), math.radians(12)))
                fe = _make_frontend()
                fe.grid = grid
                field_ = fe._field(center)

                old = _relocalize_single_stage(fe, field_, pts.x, pts.y, center)
                new = fe._relocalize(field_, pts.x, pts.y, center)

                if old is not None and new is None:
                    old_only += 1
                if old is not None and new is not None:
                    old_err = math.hypot(old.pose.x - tx, old.pose.y - ty)
                    new_err = math.hypot(new.pose.x - tx, new.pose.y - ty)
                    both_recovered_err.append((old_err, new_err))

        self.assertEqual(old_only, 0,
                         "3段探索が、1段総当たりでは復帰できていたケースを落とした")
        for old_err, new_err in both_recovered_err:
            # 位置合わせ結果が違う局所解に収束しても数mm差までは許容する
            self.assertLess(new_err, max(old_err * 1.5, old_err + 0.01),
                            f"old={old_err:.4f}m new={new_err:.4f}m")


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


def _frozen_tracker(**config_kwargs) -> Frontend:
    """ROOM を20周期ぶん作って凍結した、+x へ 0.2m/s で進む追跡器。"""
    motion = ExternalTwistModel(lambda: Twist2D(0.2, 0.0, 0.0))
    fe = _make_frontend(motion=motion, kf_dist=0.05, **config_kwargs)
    for i in range(20):
        fe.update(make_raw_scan(3.0 + 0.02 * i, 2.0, 0.0, segs=ROOM, max_range=8.0), 0.1)
    fe.grid.freeze()
    return fe


#: 上の壁の代わりに、その 0.4m 外側の壁（低い壁越しに見えた隣の部屋）が見えている
GHOST_ROOM = [s for s in ROOM if s != (6.0, 4.0, 0.0, 4.0)] + [(6.0, 4.4, 0.0, 4.4)]


class TestBehindWall(unittest.TestCase):
    def test_clean_scan_keeps_almost_all_points(self):
        fe = _frozen_tracker()
        for i in range(20, 30):
            u = fe.update(make_raw_scan(3.0 + 0.02 * i, 2.0, 0.0, segs=ROOM, max_range=8.0), 0.1)
        self.assertFalse(u.lost)
        self.assertLess(fe.behind_ratio, 0.03)

    def test_points_seen_over_the_wall_are_dropped(self):
        fe = _frozen_tracker()
        for i in range(20, 30):
            u = fe.update(make_raw_scan(3.0 + 0.02 * i, 2.0, 0.0, segs=GHOST_ROOM, max_range=8.0), 0.1)
        self.assertFalse(u.lost)
        self.assertGreater(fe.behind_ratio, 0.2)
        self.assertAlmostEqual(fe.pose.x, 0.02 * 29, delta=0.02)
        self.assertAlmostEqual(fe.pose.y, 0.0, delta=0.02)

    def test_too_many_dropped_points_disable_the_filter(self):
        # 真上（上の壁まで 2m）へ、壁の手前 1.9m の点と壁の向こう 2.4m の点
        hx, hy = np.array([0.0, 0.0]), np.array([1.9, 2.4])
        fe = _frozen_tracker(behind_max_ratio=0.6)
        self.assertEqual(fe._not_behind_wall(hx, hy, fe.pose).tolist(), [True, False])
        # 半分も外れるなら予測の方を疑い、全点を使う
        fe = _frozen_tracker(behind_max_ratio=0.4)
        self.assertEqual(fe._not_behind_wall(hx, hy, fe.pose).tolist(), [True, True])
        self.assertAlmostEqual(fe.behind_ratio, 0.5)

    def test_disabled_by_zero_margin(self):
        fe = _frozen_tracker(behind_margin=0.0)
        fe.update(make_raw_scan(3.0 + 0.02 * 20, 2.0, 0.0, segs=GHOST_ROOM, max_range=8.0), 0.1)
        self.assertEqual(fe.behind_ratio, 0.0)


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
