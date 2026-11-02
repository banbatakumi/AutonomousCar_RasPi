"""C の内側ループ（`raspi/nav/_native.c`・`native.py`）が Python 版と同じ値を出すか（2026-11-02）。

経路の仕上げの評価・速度プロファイル・曲率・車体外形の余裕・細線化は、C 版と Python 版が**ビット一致**
すること（`np.array_equal`）を確かめる。演算の順を揃え、`-ffp-contract=off` で組んでいるので、どこかで
ずれれば経路が変わる（仕上げは評価の大小だけで線を動かすので、末尾の1ビットでも別の線になりうる）。

QP の帯行列の解法だけは密行列の `solve` と丸めが違う（同じ連立方程式を別の手順で解く）ので、差が
丸め誤差の大きさに収まることを確かめる。
"""

import math
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

import numpy as np  # noqa: E402

from raspi.auto._slam2d_nav import occgrid_from_trinary  # noqa: E402
from raspi.nav import centerline as cl_mod  # noqa: E402
from raspi.nav import native  # noqa: E402
from raspi.nav import raceline as rl_mod  # noqa: E402
from raspi.nav import roadgraph  # noqa: E402
from raspi.tests.test_laptime import FOOTPRINT, RES, SPEED, divider_course, stadium  # noqa: E402

NATIVE = native.lib is not None


class _Python:
    """この中では raceline・roadgraph を Python 版で回す。"""

    def __enter__(self):
        self._saved = rl_mod._NATIVE, roadgraph._NATIVE_THIN
        rl_mod._NATIVE = None
        roadgraph._NATIVE_THIN = False
        return self

    def __exit__(self, *exc):
        rl_mod._NATIVE, roadgraph._NATIVE_THIN = self._saved
        return False


def both(fn):
    """`fn()` を C 版と Python 版で回した結果の組。"""
    a = fn()
    with _Python():
        b = fn()
    return a, b


def wavy_loop(n: int, seed: int) -> np.ndarray:
    """半径の揺れた周回（間隔は不揃い。曲率の正負が入れ替わる所もある）。"""
    rng = np.random.default_rng(seed)
    th = np.sort(rng.uniform(0.0, 2 * math.pi, n))
    r = 3.0 + 0.4 * np.sin(5 * th) + 0.05 * rng.standard_normal(n)
    return np.column_stack([10.0 + r * np.cos(th), 7.0 + 1.3 * r * np.sin(th)])


@unittest.skipUnless(NATIVE, "C コンパイラが無い（Python 版で回る）")
class TestBitExact(unittest.TestCase):
    def test_curvatures(self):
        for n in (17, 64, 301, 510):
            for closed in (True, False):
                xy = wavy_loop(n, n)
                for f in (lambda: rl_mod.curvature(xy, closed),
                          lambda: rl_mod.chord_curvature(xy, 3, closed),
                          lambda: rl_mod.speed_kappa(xy, closed)):
                    a, b = both(f)
                    self.assertTrue(np.array_equal(a, b), (n, closed))

    def test_speed_kappa_median_even_and_odd(self):
        """中央値（`np.median`）は偶数個なら中央2つの平均。点の数の偶奇で両方を通す。"""
        for n in (100, 101):
            xy = wavy_loop(n, 7)
            a, b = both(lambda: rl_mod.speed_kappa(xy, True, chord=0.9))
            self.assertTrue(np.array_equal(a, b), n)

    def test_speed_profile(self):
        for n in (40, 333):
            xy = wavy_loop(n, 3)
            ks = rl_mod.speed_kappa(xy, True)
            for kw in (dict(SPEED), dict(SPEED, a_grip=5.0), dict(SPEED, friction_circle=False)):
                a, b = both(lambda: rl_mod.speed_profile(xy, ks, closed=True, **kw))
                self.assertTrue(np.array_equal(a, b), kw)
                a, b = both(lambda: rl_mod.speed_profile(xy, ks, closed=False, v_start=0.4,
                                                         v_end=0.2, **kw))
                self.assertTrue(np.array_equal(a, b), kw)
                a, b = both(lambda: rl_mod.speed_profile(xy, ks, closed=False, **kw))
                self.assertTrue(np.array_equal(a, b), kw)

    def test_body_clearance_and_band(self):
        t, traj = divider_course()
        grid = occgrid_from_trinary(t, resolution=RES, origin=(0.0, 0.0), seq=0)
        cl = cl_mod.build(grid, traj, step=0.1, max_width=3.0)
        rng = np.random.default_rng(0)
        for closed in (True, False):
            body = rl_mod._BodyCheck(grid, FOOTPRINT, closed)
            self.assertIsNotNone(body._edt32)
            for _ in range(5):
                xy = cl.xy + 0.2 * rng.standard_normal(cl.xy.shape)
                a, b = both(lambda: body.clearance(xy))
                self.assertEqual(a.dtype, b.dtype)
                self.assertTrue(np.array_equal(a, b))
                off = 0.02 * cl.normal
                a, b = both(lambda: body.clearance(xy, off))
                self.assertTrue(np.array_equal(a, b))
        band = rl_mod._Band(grid, cl.xy, cl.normal, -cl.w_right + 0.2, cl.w_left - 0.2, True)
        for s in (0.0, 0.1, 0.3, 0.6):
            xy = cl.xy + s * cl.normal
            a, b = both(lambda: band.inside(xy))
            self.assertEqual(a, b, s)

    def test_refined_line_is_identical(self):
        """仕上げ込みのライン全体（評価をまとめた C 版・前回の値の使い回し・2本の出発点）が Python 版と
        同じ点列になる。QP は両方とも密行列で解く（帯行列の差は下の `TestBandSolve`）。"""
        t, traj = divider_course()
        grid = occgrid_from_trinary(t, resolution=RES, origin=(0.0, 0.0), seq=0)
        cl = cl_mod.build(grid, traj, step=0.1, max_width=3.0)
        saved = rl_mod._BAND_MIN_N
        rl_mod._BAND_MIN_N = 10 ** 9
        try:
            def run():
                return rl_mod.optimize(grid, cl, half_width=0.09, margin=0.05, lam=0.1, passes=2,
                                       front_overhang=0.30, rear_overhang=0.07,
                                       footprint=FOOTPRINT, time_iters=2, kappa_max=2.0,
                                       refine_evals=1500, refine_starts=2, a_grip=4.4, **SPEED)
            a, b = both(run)
        finally:
            rl_mod._BAND_MIN_N = saved
        self.assertTrue(np.array_equal(a.xy, b.xy))
        self.assertTrue(np.array_equal(a.v, b.v))

    def test_parallel_refine_is_identical(self):
        """出発点を fork した子で回しても、逐次と同じ線（`refine_parallel`）。"""
        if not hasattr(__import__("os"), "fork"):
            self.skipTest("fork が無い")
        t, traj = divider_course()
        grid = occgrid_from_trinary(t, resolution=RES, origin=(0.0, 0.0), seq=0)
        cl = cl_mod.build(grid, traj, step=0.1, max_width=3.0)

        def run(par):
            return rl_mod.optimize(grid, cl, half_width=0.09, margin=0.05, lam=0.1, passes=2,
                                   front_overhang=0.30, rear_overhang=0.07, footprint=FOOTPRINT,
                                   time_iters=2, kappa_max=2.0, refine_evals=800, refine_starts=2,
                                   refine_parallel=par, **SPEED)
        import warnings
        saved = rl_mod._can_fork
        # macOS・スレッドのある pytest のプロセスでも試す（本番は Linux のスレッドの無いワーカーだけ）
        rl_mod._can_fork = lambda: True
        try:
            with warnings.catch_warnings():
                warnings.simplefilter("ignore", DeprecationWarning)
                a = run(True)
        finally:
            rl_mod._can_fork = saved
        b = run(False)
        self.assertTrue(np.array_equal(a.xy, b.xy))

    def test_thinning(self):
        import cv2
        rng = np.random.default_rng(1)
        for _ in range(15):
            m = cv2.dilate((rng.random((90, 70)) > 0.93).astype(np.uint8),
                           np.ones((5, 5), np.uint8)).astype(bool)
            a, b = both(lambda: roadgraph.thin(m))
            self.assertTrue(np.array_equal(a, b))


@unittest.skipUnless(NATIVE, "C コンパイラが無い（Python 版で回る）")
class TestBandSolve(unittest.TestCase):
    """帯行列の QP（`_min_curvature_alpha_band`）と密行列の QP が、丸め誤差の範囲で同じ解を出す。"""

    def _compare(self, c, nrm, lo, hi, closed, **kw):
        a = rl_mod.min_curvature_alpha(c, nrm, lo, hi, closed=closed, max_iter=300, **kw)
        with _Python():
            b = rl_mod.min_curvature_alpha(c, nrm, lo, hi, closed=closed, max_iter=300, **kw)
        np.testing.assert_allclose(a, b, rtol=0, atol=1e-9)

    def test_closed_open_weights_and_fixed_points(self):
        c = stadium(r=1.0)
        n = len(c)
        rng = np.random.default_rng(2)
        for closed in (True, False):
            nrm = cl_mod.normals(c) if closed else cl_mod.normals_open(c)
            lo, hi = np.full(n, -0.3), np.full(n, 0.3)
            lo[10] = hi[10] = 0.1                       # 等式の点
            lo[[0, 1, n - 2, n - 1]] = hi[[0, 1, n - 2, n - 1]] = 0.0   # 巡回の角の両側を固定
            self._compare(c, nrm, lo, hi, closed)
            w = rng.uniform(0.5, 3.0, n)
            self._compare(c, nrm, lo, hi, closed, weights=w)
            warm = rl_mod.min_curvature_alpha(c, nrm, lo, hi, closed=closed, weights=w)
            self._compare(c, nrm, lo, hi, closed, weights=w, warm=warm)

    def test_narrow_bounds_many_active(self):
        """制約の効いた点が多い（自由な点が飛び飛び）。巡回の角の結合の分岐を全部通す。"""
        c = stadium(r=0.8, straight=2.0)
        n = len(c)
        nrm = cl_mod.normals(c)
        rng = np.random.default_rng(3)
        for _ in range(5):
            lo = -rng.uniform(0.0, 0.15, n)
            hi = rng.uniform(0.0, 0.15, n)
            self._compare(c, nrm, lo, hi, True)

    def test_band_solve_matches_dense(self):
        """`band_solve` 単体: 帯幅2＋巡回の角の正定値行列を密行列の `solve` と比べる。"""
        rng = np.random.default_rng(4)
        for nf in (8, 9, 30, 200):
            d1 = rng.uniform(-1, 1, nf - 1)
            d2 = rng.uniform(-1, 1, nf - 2)
            c20, c10, c11 = rng.uniform(-1, 1, 3)
            m = np.zeros((nf, nf))
            m[np.arange(nf - 1), np.arange(1, nf)] = d1
            m[np.arange(nf - 2), np.arange(2, nf)] = d2
            m[nf - 2, 0], m[nf - 1, 0], m[nf - 1, 1] = c20, c10, c11
            m = m + m.T
            d0 = np.abs(m).sum(axis=1) + 1.0             # 対角優位 → 正定値
            m[np.arange(nf), np.arange(nf)] = d0
            rhs = rng.standard_normal(nf)
            x = np.empty(nf)
            rc = native.lib.band_solve(nf, native.addr(d0), native.addr(native.f64(d1)),
                                       native.addr(native.f64(d2)), c20, c10, c11,
                                       native.addr(rhs), native.addr(x))
            self.assertEqual(rc, 0)
            np.testing.assert_allclose(x, np.linalg.solve(m, rhs), rtol=0, atol=1e-10)

    def test_not_positive_definite_is_reported(self):
        nf = 10
        d0 = np.full(nf, -1.0)
        z1, z2 = np.zeros(nf - 1), np.zeros(nf - 2)
        rhs, x = np.ones(nf), np.empty(nf)
        rc = native.lib.band_solve(nf, native.addr(d0), native.addr(z1), native.addr(z2),
                                   0.0, 0.0, 0.0, native.addr(rhs), native.addr(x))
        self.assertEqual(rc, 1)


class TestLoader(unittest.TestCase):
    def test_disabled_by_env(self):
        """`SURGE_NO_NATIVE=1` なら C を読まない（Python 版で回る）。"""
        import subprocess
        out = subprocess.run(
            [sys.executable, "-c", "from raspi.nav import native; print(native.lib is None)"],
            env={**__import__("os").environ, "SURGE_NO_NATIVE": "1"},
            cwd=str(Path(__file__).resolve().parents[2]), capture_output=True, text=True,
            timeout=60)
        self.assertEqual(out.stdout.strip(), "True", out.stderr)


if __name__ == "__main__":
    unittest.main()
