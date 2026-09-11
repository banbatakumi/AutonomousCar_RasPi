"""`core/deskew.py`のテスト。

Phase 2 完了条件: 合成した等速円運動+タイムスタンプ付き点群に対し、脱スキュー後
の点群位置が理論値と一致すること（誤差<1mm相当）を数値で確認する。
"""

import math
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

import numpy as np  # noqa: E402

from slam2d.core.deskew import deskew, truncate  # noqa: E402
from slam2d.core.types import Pose2D, RawScan, ScanPoints, Twist2D, compose, inverse  # noqa: E402

NS = 1_000_000_000


def _raw(angles, ranges, *, t_point_ns, valid=None, saturated=None) -> RawScan:
    angles = np.asarray(angles, dtype=np.float64)
    ranges = np.asarray(ranges, dtype=np.float64)
    n = angles.size
    valid = np.ones(n, dtype=bool) if valid is None else np.asarray(valid, dtype=bool)
    saturated = np.zeros(n, dtype=bool) if saturated is None else np.asarray(saturated, dtype=bool)
    return RawScan(angles, ranges, valid, saturated, np.asarray(t_point_ns, dtype=np.int64))


class TestDeskewNoMotion(unittest.TestCase):
    def test_zero_twist_returns_points_unchanged_and_uncorrected(self):
        raw = _raw([0.0, math.pi / 2], [1.0, 2.0], t_point_ns=[0, NS])
        pts = deskew(raw, Twist2D(0.0, 0.0, 0.0))
        self.assertFalse(pts.corrected)
        np.testing.assert_allclose(pts.x, [1.0, 0.0], atol=1e-9)
        np.testing.assert_allclose(pts.y, [0.0, 2.0], atol=1e-9)

    def test_missing_timestamps_skip_correction(self):
        raw = _raw([0.0], [1.0], t_point_ns=[0])  # t_point_ns=0 は「時刻無し」
        pts = deskew(raw, Twist2D(1.0, 0.0, 1.0))
        self.assertFalse(pts.corrected)


class TestDeskewValidHitSaturated(unittest.TestCase):
    def test_invalid_points_are_dropped(self):
        raw = _raw([0.0, math.pi], [1.0, 2.0], t_point_ns=[NS, NS], valid=[True, False])
        pts = deskew(raw, Twist2D(0.0, 0.0, 0.0))
        self.assertEqual(len(pts), 1)
        self.assertAlmostEqual(float(pts.x[0]), 1.0, places=9)

    def test_saturated_point_has_hit_false(self):
        raw = _raw([0.0], [5.0], t_point_ns=[NS], saturated=[True])
        pts = deskew(raw, Twist2D(0.0, 0.0, 0.0), max_range=8.0)
        self.assertFalse(bool(pts.hit[0]))

    def test_over_range_is_truncated_and_not_a_hit(self):
        raw = _raw([0.0], [20.0], t_point_ns=[NS])
        pts = deskew(raw, Twist2D(0.0, 0.0, 0.0), max_range=8.0)
        self.assertAlmostEqual(float(pts.x[0]), 8.0, places=9)
        self.assertFalse(bool(pts.hit[0]))

    def test_nan_range_is_treated_as_over_range_not_a_hit(self):
        """距離がNaN（IMUグリッチ等の混入）でも`r > max_range`はFalseになり素通り
        しうる——それを塞ぐ`~np.isfinite(r)`のガードを確認する。NaNの壁ヒット点が
        そのまま地図に焼かれてはならない。"""
        raw = _raw([0.0], [float("nan")], t_point_ns=[NS])
        pts = deskew(raw, Twist2D(0.0, 0.0, 0.0), max_range=8.0)
        self.assertTrue(math.isfinite(float(pts.x[0])))
        self.assertAlmostEqual(float(pts.x[0]), 8.0, places=9)
        self.assertFalse(bool(pts.hit[0]))

    def test_inf_range_is_treated_as_over_range_not_a_hit(self):
        raw = _raw([0.0], [float("inf")], t_point_ns=[NS])
        pts = deskew(raw, Twist2D(0.0, 0.0, 0.0), max_range=8.0)
        self.assertAlmostEqual(float(pts.x[0]), 8.0, places=9)
        self.assertFalse(bool(pts.hit[0]))


class TestDeskewStraightLine(unittest.TestCase):
    def test_matches_theoretical_translation(self):
        # 車は base_link 原点から x方向に 1.0 m/s。角度0の点(t=1ns、ほぼt=0相当)
        # に測った正面1mの固定点は、1秒後(基準時刻、角度piのダミー点で作る)の
        # 車から見ると 0m 先（そのまま追い越した位置）になる
        raw = _raw([0.0, math.pi], [1.0, 5.0], t_point_ns=[1, 1 + NS])
        pts = deskew(raw, Twist2D(1.0, 0.0, 0.0))
        self.assertTrue(pts.corrected)
        self.assertAlmostEqual(float(pts.x[0]), 0.0, delta=1e-6)
        self.assertAlmostEqual(float(pts.y[0]), 0.0, delta=1e-6)


class TestDeskewCircularArc(unittest.TestCase):
    def test_matches_theoretical_rigid_transform(self):
        """円運動での脱スキューが、姿勢合成の理論値と一致することを確認。

        t=0(車の姿勢を世界原点とする)に測った、車正面1mの固定点は世界座標
        (1, 0)。twist=(v=1, w=1)で1秒間(t_ref)動いた車の姿勢は
        `integrate_twist`の理論値と一致するはずで、その姿勢の逆変換で
        固定点(1,0)をt_ref時点の車座標系へ変換した値が、deskewの出力と
        一致するべき。**`t_point_ns=0`は「時刻無し」の特別値なので、
        非零の時刻を使う。**
        """
        from slam2d.core.types import integrate_twist

        twist = Twist2D(1.0, 0.0, 1.0)
        t0 = 1  # t_point_ns=0 は「時刻無し」の特別値なので、1ns等の非零値を使う
        t_ref = t0 + NS
        # 角度piの点はt_ref自身を作るためだけのダミー（tau=0で無変換のまま残る）
        raw = _raw([0.0, math.pi], [1.0, 5.0], t_point_ns=[t0, t_ref])
        pts = deskew(raw, twist)
        self.assertTrue(pts.corrected)
        self.assertEqual(pts.t_ref_ns, t_ref)

        # 期待値: 固定点(1,0)を、t=0での姿勢(0,0,0)から1秒動いた姿勢の
        # 逆変換でローカル座標へ戻したもの
        ref_pose = integrate_twist(twist, 1.0)
        expected = compose(inverse(ref_pose), Pose2D(1.0, 0.0, 0.0))
        self.assertAlmostEqual(float(pts.x[0]), expected.x, delta=1e-6)
        self.assertAlmostEqual(float(pts.y[0]), expected.y, delta=1e-6)


class TestTruncate(unittest.TestCase):
    def test_far_points_scaled_down_and_not_hit(self):
        pts = ScanPoints(np.array([10.0, 1.0]), np.array([0.0, 0.0]),
                         np.array([True, True]), 0, True)
        out = truncate(pts, 3.0)
        self.assertAlmostEqual(float(out.x[0]), 3.0, places=9)
        self.assertFalse(bool(out.hit[0]))
        self.assertAlmostEqual(float(out.x[1]), 1.0, places=9)
        self.assertTrue(bool(out.hit[1]))

    def test_noop_when_radius_not_exceeded(self):
        pts = ScanPoints(np.array([1.0]), np.array([0.0]), np.array([True]), 0, True)
        out = truncate(pts, 3.0)
        self.assertAlmostEqual(float(out.x[0]), 1.0, places=9)


if __name__ == "__main__":
    unittest.main()
