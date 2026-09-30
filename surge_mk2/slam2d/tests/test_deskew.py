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

from slam2d.core.deskew import TwistBuffer, deskew, deskew_traj, truncate  # noqa: E402
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


class TestTwistBuffer(unittest.TestCase):
    """時刻つき twist の履歴（点ごとの脱スキューと推測航法の予測が共有する）。"""

    def _buf(self, twist, n=6, dt=0.02, t0=NS):
        b = TwistBuffer()
        for i in range(n):
            b.add(t0 + int(i * dt * NS), twist)
        return b

    def test_delta_matches_the_exact_arc(self):
        b = self._buf(Twist2D(1.0, 0.0, 1.0))
        d = b.delta(NS, NS + int(0.1 * NS))
        self.assertAlmostEqual(d.x, math.sin(0.1), places=6)
        self.assertAlmostEqual(d.y, 1.0 - math.cos(0.1), places=6)
        self.assertAlmostEqual(d.yaw, 0.1, places=6)

    def test_extrapolates_past_the_last_sample(self):
        """テレメトリは点群より少し古いので、端は最後の twist で外挿する。"""
        b = self._buf(Twist2D(1.0, 0.0, 1.0))          # 1.00〜1.10 秒ぶん
        d = b.delta(NS, NS + int(0.12 * NS))
        self.assertAlmostEqual(d.yaw, 0.12, places=6)

    def test_covers_allows_small_extrapolation(self):
        b = self._buf(Twist2D(1.0, 0.0, 0.0))
        self.assertTrue(b.covers(NS - int(0.01 * NS), NS + int(0.13 * NS)))
        self.assertFalse(b.covers(NS - int(0.5 * NS), NS + int(0.1 * NS)))

    def test_ignores_out_of_order_samples(self):
        b = TwistBuffer()
        b.add(NS, Twist2D(1.0, 0.0, 0.0))
        b.add(NS - 1000, Twist2D(9.0, 0.0, 0.0))
        self.assertEqual(len(b), 1)


class TestDeskewTraj(unittest.TestCase):
    """回頭が1周のあいだに変化しても、点ごとの時刻で正しく戻せる。"""

    def _raw(self, t_start, dur_ns, n=360):
        ang = np.linspace(0.0, 2 * math.pi, n, endpoint=False)
        rng = np.full(n, 2.0)
        t = t_start + (np.arange(n) * dur_ns // max(1, n - 1))
        return RawScan(ang, rng, np.ones(n, dtype=bool), np.zeros(n, dtype=bool),
                       t.astype(np.int64))

    def test_recovers_points_under_changing_yaw_rate(self):
        # ヨーレートが 0 → 2 rad/s へ立ち上がる1周（100ms）
        t0 = NS
        dur = int(0.1 * NS)
        buf = TwistBuffer()
        for i in range(11):
            w = 2.0 * i / 10.0
            buf.add(t0 + int(i * 0.01 * NS), Twist2D(0.0, 0.0, w))
        raw = self._raw(t0, dur)
        pts = deskew_traj(raw, buf, max_range=8.0)

        # 真値: 各点は「測った時刻の車体」から見て距離2m・角度ang。
        # 基準時刻(t_ref)の車体から見ると、その間の回頭ぶん戻る
        ts, sx, sy, syaw = buf.poses(t0 + dur, t_lo=t0, t_hi=t0 + dur)
        ang = raw.angles
        tq = raw.t_point_ns.astype(np.float64)
        yaw_at = np.interp(tq, ts.astype(np.float64), syaw)
        want_x = 2.0 * np.cos(ang + yaw_at)
        want_y = 2.0 * np.sin(ang + yaw_at)
        self.assertLess(float(np.max(np.hypot(pts.x - want_x, pts.y - want_y))), 2e-3)

    def test_falls_back_without_samples(self):
        raw = self._raw(NS, int(0.1 * NS))
        pts = deskew_traj(raw, TwistBuffer(), max_range=8.0)
        self.assertEqual(len(pts), len(raw))


def _poses_loop_reference(buf: TwistBuffer, t_ref_ns: int, *,
                          t_lo: int | None = None, t_hi: int | None = None):
    """`TwistBuffer.poses()`のベクトル化前の実装（Issue #19 の等価性確認用）。

    1サンプルごとに`Twist2D`/`integrate_twist`/`Pose2D`を作る Python ループ。
    現行の`poses()`（`cumsum`によるベクトル化版）と数値が一致することを
    `TestPosesVectorizedMatchesLoop`で確認する。
    """
    from slam2d.core.deskew import _interp_pose
    from slam2d.core.types import integrate_twist

    lo = min(t_ref_ns, t_lo if t_lo is not None else t_ref_ns)
    hi = max(t_ref_ns, t_hi if t_hi is not None else t_ref_ns)
    t, vxs, vys, ws = buf._samples(lo, hi)
    n = t.size
    x = np.zeros(n); y = np.zeros(n); yaw = np.zeros(n)
    for i in range(1, n):
        dt = (t[i] - t[i - 1]) / NS
        d = integrate_twist(Twist2D(float(vxs[i - 1]), float(vys[i - 1]), float(ws[i - 1])), dt)
        c, s = math.cos(yaw[i - 1]), math.sin(yaw[i - 1])
        x[i] = x[i - 1] + c * d.x - s * d.y
        y[i] = y[i - 1] + s * d.x + c * d.y
        yaw[i] = yaw[i - 1] + d.yaw
    rx, ry, ryaw = _interp_pose(t, x, y, yaw, np.array([t_ref_ns], dtype=np.int64))
    c, s = math.cos(-ryaw[0]), math.sin(-ryaw[0])
    dx, dy = x - rx[0], y - ry[0]
    return t, c * dx - s * dy, s * dx + c * dy, yaw - ryaw[0]


class TestPosesVectorizedMatchesLoop(unittest.TestCase):
    """Issue #19: `poses()`のベクトル化版とループ版の数値的等価性。

    許容誤差は`float64`の丸め誤差だけを許す `1e-9`（`cumsum`と逐次加算は
    加算順序が同じなので理論上は完全一致するが、`np.where`両辺を評価する
    ぶんNaN/Infを経由する丸めが混ざりうるため、余裕を持たせてある）。
    """

    def _buf(self, samples, t0=NS):
        b = TwistBuffer()
        for i, (dt, vx, vy, w) in enumerate(samples):
            b.add(t0 + int(dt * NS), Twist2D(vx, vy, w))
        return b

    def _assert_matches(self, buf, t_ref, t_lo=None, t_hi=None):
        t_a, x_a, y_a, yaw_a = buf.poses(t_ref, t_lo=t_lo, t_hi=t_hi)
        t_b, x_b, y_b, yaw_b = _poses_loop_reference(buf, t_ref, t_lo=t_lo, t_hi=t_hi)
        np.testing.assert_array_equal(t_a, t_b)
        self.assertLess(float(np.max(np.abs(x_a - x_b))), 1e-9)
        self.assertLess(float(np.max(np.abs(y_a - y_b))), 1e-9)
        self.assertLess(float(np.max(np.abs(yaw_a - yaw_b))), 1e-9)

    def test_varying_yaw_rate_and_lateral_speed(self):
        samples = [(0.00 * i, 1.5 + 0.3 * math.sin(i), 0.2 * math.cos(i * 0.5),
                    -1.0 + 0.4 * i) for i in range(40)]
        samples = [(i * 0.02, vx, vy, w) for i, (_, vx, vy, w) in enumerate(samples)]
        buf = self._buf(samples)
        t_ref = NS + int(0.78 * NS)
        self._assert_matches(buf, t_ref, t_lo=NS, t_hi=t_ref)

    def test_near_zero_yaw_rate_straight_branch(self):
        # |w| が _ARC_EPS をまたぐ値を混ぜて分岐の境界を突く
        samples = [(i * 0.02, 1.0, 0.0, w)
                   for i, w in enumerate([0.0, 1e-5, -1e-5, 1e-3, 0.0, 5.0, -5.0, 0.0])]
        buf = self._buf(samples)
        t_ref = NS + int(0.14 * NS)
        self._assert_matches(buf, t_ref, t_lo=NS, t_hi=t_ref)

    def test_extrapolated_endpoints(self):
        samples = [(i * 0.02, 2.0, -0.5, 1.2) for i in range(6)]
        buf = self._buf(samples)
        # buf の範囲外まで挟む（端を一定twistで外挿する経路を通す）
        self._assert_matches(buf, NS + int(0.2 * NS), t_lo=NS - int(0.05 * NS),
                             t_hi=NS + int(0.2 * NS))

    def test_single_sample_range(self):
        buf = self._buf([(0.0, 1.0, 0.0, 0.5)])
        self._assert_matches(buf, NS, t_lo=NS, t_hi=NS)
