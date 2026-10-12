"""ライン生成の別プロセス（`slam2d_raceline.worker_pool`）と、仕上げ・帯・出発点のテスト。

`conftest.py` がテスト全体のワーカーをスレッドにしているので、ここでは明示的にプロセスを作る。
"""

import math
import multiprocessing
import sys
import unittest
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

import numpy as np  # noqa: E402

from raspi.auto import slam2d_raceline as sr  # noqa: E402
from raspi.core.vehicle import Vehicle  # noqa: E402
from raspi.nav import centerline as cl_mod  # noqa: E402
from raspi.nav import raceline as rl  # noqa: E402

RES = 0.025


def oval_trinary(width: float = 1.2, a: float = 3.0, b: float = 1.6):
    """楕円の周回コース（内外に壁）。戻り値 `(trinary, origin, 中心線の点列)`。"""
    size = 9.0
    n = int(size / RES)
    origin = (-size / 2, -size / 2)
    c = (np.arange(n) + 0.5) * RES + origin[0]
    xx, yy = np.meshgrid(c, c)
    t = np.linspace(0, 2 * math.pi, 400, endpoint=False)
    mid = np.column_stack([a * np.cos(t), b * np.sin(t)])
    d = np.full(xx.shape, np.inf)
    for i in range(0, len(mid), 2):
        d = np.minimum(d, np.hypot(xx - mid[i, 0], yy - mid[i, 1]))
    tri = np.zeros(xx.shape, dtype=np.uint8)             # 0 = 未知
    tri[d < width / 2] = 1                               # 1 = 空き
    tri[(d >= width / 2) & (d < width / 2 + 2 * RES)] = 2   # 2 = 壁
    return tri, origin, mid


class TestBand(unittest.TestCase):
    def test_inside_and_outside(self):
        tri, origin, mid = oval_trinary()
        grid = sr.occgrid_from_trinary(tri, resolution=RES, origin=origin, seq=0)
        c = cl_mod.resample_loop(mid, 0.1)
        nrm = cl_mod.normals(c)
        n = len(c)
        band = rl._Band(grid, c, nrm, np.full(n, -0.2), np.full(n, 0.2), True)
        self.assertTrue(band.inside(c))
        self.assertTrue(band.inside(c + nrm * 0.18))
        self.assertFalse(band.inside(c + nrm * 0.45))          # 帯の外（まだ壁の内側）
        self.assertFalse(band.inside(c - nrm * 0.45))
        far = c.copy()
        far[10] = (100.0, 100.0)                               # 格子の外
        self.assertFalse(band.inside(far))


class TestOptimizeRefine(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.veh = Vehicle.load()
        tri, origin, mid = oval_trinary()
        cls.tri, cls.origin = tri, origin
        cls.grid = sr.occgrid_from_trinary(tri, resolution=RES, origin=origin, seq=0)
        cls.cl = cl_mod.build(cls.grid, mid, step=sr.PATH_STEP, max_width=sr.MAX_TRACK_WIDTH)
        cls.p = {s.key: s.default for s in sr.Slam2dRaceLine.params}
        cls.p.update(v_max=3.0)

    def _line(self, **over):
        kw = sr.line_kwargs(self.p, self.veh)
        kw.update(refine_evals=1500, refine_starts=1)
        kw.update(over)
        return rl.optimize(self.grid, self.cl, **kw)

    def test_refine_is_faster_and_keeps_the_constraints(self):
        plain = self._line(refine_evals=0)
        fine = self._line()
        t0 = rl._travel_time(plain.xy, plain.v, True)
        t1 = rl._travel_time(fine.xy, fine.v, True)
        self.assertLess(t1, t0 - 0.01)
        body = rl._BodyCheck(self.grid, self.veh.footprint, True)
        self.assertGreaterEqual(float(body.clearance(fine.xy).min()), self.p["line_margin"] - 0.011)
        self.assertLessEqual(float(rl.speed_kappa(fine.xy, True).max()),
                             sr.kappa_limit(self.p, self.veh) * 1.001)
        # 周回の向きと道は変わらない（中心線から道幅の半分以上は離れない）
        d = np.hypot(fine.xy[:, None, 0] - self.cl.xy[None, :, 0],
                     fine.xy[:, None, 1] - self.cl.xy[None, :, 1]).min(axis=1)
        self.assertLess(float(d.max()), 0.6)

    def test_more_starts_never_worse(self):
        one = self._line(refine_starts=1)
        two = self._line(refine_starts=2)
        self.assertLessEqual(rl._travel_time(two.xy, two.v, True),
                             rl._travel_time(one.xy, one.v, True) + 1e-9)

    def test_quick_line_skips_refine(self):
        self.assertEqual(sr.line_kwargs(self.p, self.veh, refine=False)["refine_evals"], 0)
        self.assertGreater(sr.line_kwargs(self.p, self.veh)["refine_evals"], 0)

    def test_line_is_built_in_another_process(self):
        """本番と同じ渡し方（`spawn` の別プロセス）で引ける＝引数と結果が pickle できる。"""
        with ProcessPoolExecutor(max_workers=1,
                                 mp_context=multiprocessing.get_context("spawn"),
                                 initializer=sr._worker_init) as pool:
            fut = pool.submit(sr._optimize_line, self.tri, RES, self.origin, self.cl,
                              dict(self.p), self.veh, False)
            line = fut.result(timeout=120)
        self.assertIsInstance(line, rl.RaceLine)
        self.assertEqual(len(line.xy), len(line.v))
        self.assertTrue(line.closed)


class TestLimitsAndKeys(unittest.TestCase):
    def test_line_is_redrawn_when_speed_settings_change(self):
        p = {s.key: s.default for s in sr.Slam2dRaceLine.params}
        self.assertNotEqual(sr.line_key(p), sr.line_key({**p, "v_max": 1.0}))
        self.assertNotEqual(sr.line_key(p), sr.line_key({**p, "kappa_use": 0.8}))
        self.assertEqual(sr.line_key(p), sr.line_key({**p, "look_k": 0.9}))


if __name__ == "__main__":
    unittest.main()


class TestSight(unittest.TestCase):
    """見通し（`raceline.sight_distance`）と、その中で止まれる速度（`sight_speed`）。"""

    def test_open_oval_sees_far_and_a_wall_blocks(self):
        tri, origin, mid = oval_trinary(width=1.2)
        grid = sr.occgrid_from_trinary(tri, resolution=RES, origin=origin, seq=0)
        c = cl_mod.resample_loop(mid, 0.1)
        far = rl.sight_distance(grid, c, True)
        self.assertGreater(float(far.min()), 1.0)                  # 緩い楕円では 1m 以上先が見える
        self.assertLessEqual(float(far.max()), rl.SIGHT_MAX_M + 1e-9)
        # 中心線を横切る仕切りを置くと、その手前では仕切りまでしか見えない
        tri2 = tri.copy()
        i = int(np.argmin(np.hypot(c[:, 0] - 0.0, c[:, 1] - 1.6)))   # 楕円の上端付近
        x, y = c[i]
        col = int((x - origin[0]) / RES)
        r0, r1 = int((y - 0.7 - origin[1]) / RES), int((y + 0.7 - origin[1]) / RES)
        tri2[r0:r1, col:col + 2] = 2
        blocked = rl.sight_distance(sr.occgrid_from_trinary(tri2, resolution=RES, origin=origin,
                                                            seq=0), c, True)
        j = (i - 8) % len(c)                                       # 仕切りの 0.8m 手前
        self.assertLess(float(blocked[j]), 1.0)
        self.assertGreater(float(far[j]), float(blocked[j]) + 0.5)

    def test_sight_speed_stops_within_sight(self):
        s = np.array([0.3, 1.0, 3.0])
        v = rl.sight_speed(s, a_brake=2.3, react_s=0.2, reserve=0.35)
        self.assertEqual(float(v[0]), 0.0)                         # 余白より近ければ進めない
        for si, vi in zip(s[1:], v[1:]):
            self.assertAlmostEqual(vi * 0.2 + vi * vi / (2 * 2.3), si - 0.35, places=6)
        self.assertTrue(np.all(np.diff(v) > 0))

    def test_speed_profile_brakes_towards_a_cap(self):
        """点ごとの上限（`v_cap`）へ向けた減速がプロファイルに入る（その点だけ急に下がらない）。"""
        n = 200
        xy = np.column_stack([np.arange(n) * 0.1, np.zeros(n)])
        cap = np.full(n, 9.0)
        cap[100] = 0.5
        v = rl.speed_profile(xy, np.zeros(n), v_max=3.0, v_min=0.35, a_lat=3.0, a_accel=2.0,
                             a_brake=2.0, closed=False, v_start=3.0, v_end=3.0, v_cap=cap)
        self.assertAlmostEqual(float(v[100]), 0.5)
        self.assertLess(float(v[95]), 1.6)                         # 0.5m 手前: sqrt(0.25+2·2·0.5)=1.5
        self.assertAlmostEqual(float(v[50]), 3.0)
        self.assertTrue(np.all(np.abs(np.diff(v ** 2)) <= 2 * 2.0 * 0.1 + 1e-6))
