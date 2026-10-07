"""ラップタイムを詰めるための経路・速度・追従のテスト（2026-09-29）。

- 速度プロファイル: 弦で測る曲率・摩擦円・曲がれる限界の近くの横G・作り直し（`retime`）
- 経路: 車体外形の検査（薄い壁の先端）・最小時間への寄せ・アクティブセットの等式点
- 追従: 速度の先読み・曲率フィードフォワード
"""

import math
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

import numpy as np  # noqa: E402

from raspi.auto._slam2d_nav import occgrid_from_trinary  # noqa: E402
from raspi.auto.slam2d_raceline import SPEED_KEYS, Slam2dRaceLine  # noqa: E402
from raspi.nav import centerline as cl_mod  # noqa: E402
from raspi.nav import raceline as rl_mod  # noqa: E402
from raspi.nav.purepursuit import PursuitConfig, follow  # noqa: E402

RES = 0.025
FOOTPRINT = ((0.30, 0.09), (0.30, -0.09), (-0.07, -0.09), (-0.07, 0.09))
SPEED = dict(v_max=3.0, v_min=0.35, a_lat=4.0, a_accel=2.8, a_brake=2.8)


def _line(xy: np.ndarray, closed: bool = True, v: float = 1.0) -> rl_mod.RaceLine:
    seg = np.hypot(*((np.roll(xy, -1, axis=0) - xy) if closed else np.diff(xy, axis=0)).T)
    n = len(xy)
    return rl_mod.RaceLine(xy=xy, v=np.full(n, v), kappa=rl_mod.curvature(xy, closed),
                           length=float(seg.sum()),
                           closed=closed)


def stadium(r: float = 1.0, straight: float = 4.0, step: float = 0.1) -> np.ndarray:
    """直線 `straight` と半径 `r` の半円2つの周回（反時計回り、0.1m 刻み）。"""
    pts = []
    for x in np.arange(0.0, straight, step):
        pts.append((x, -r))
    for a in np.arange(-math.pi / 2, math.pi / 2, step / r):
        pts.append((straight + r * math.cos(a), r * math.sin(a)))
    for x in np.arange(straight, 0.0, -step):
        pts.append((x, r))
    for a in np.arange(math.pi / 2, 3 * math.pi / 2, step / r):
        pts.append((r * math.cos(a), r * math.sin(a)))
    return np.array(pts)


def divider_course(lane: float = 1.2) -> tuple[np.ndarray, np.ndarray]:
    """厚さ 5cm の仕切り（長さ 3m）の両側に幅 `lane` の車線がある周回路と、仕切りを回る軌跡。

    **ラインは仕切りの先端を回り込む。** 先端は法線のレイでは見えないので、車体外形を
    検査しないと前端がかすめる（道幅 1.2m で余裕 2.7cm、toyota2 の上の仕切りと同じ形）。
    """
    x0, y0, straight = 1.0, 1.0, 3.0
    w, h = straight + 2 * lane + 2.0, 2 * lane + 2.0
    t = np.zeros((int((h + 1) / RES), int((w + 1) / RES)), dtype=np.uint8)

    def rect(a, b, c, d, v):
        t[int(b / RES):int(d / RES), int(a / RES):int(c / RES)] = v

    bx1, by1 = x0 + straight + 2 * lane, y0 + 2 * lane + 0.05
    rect(x0, y0, bx1, by1, 1)
    for q in ((x0, y0, bx1, y0 + 0.05), (x0, by1 - 0.05, bx1, by1),
              (x0, y0, x0 + 0.05, by1), (bx1 - 0.05, y0, bx1, by1)):
        rect(*q, 2)
    cy = y0 + lane + 0.025
    rect(x0 + lane, cy - 0.025, x0 + lane + straight, cy + 0.025, 2)
    traj = stadium(r=lane / 2, straight=straight) + np.array([x0 + lane, cy])
    return t, traj


class TestSpeedProfile(unittest.TestCase):
    def test_speed_kappa_ignores_a_single_kink(self):
        """★ 0.1m 刻みの1点の折れで、3点の曲率は跳ねるが速度の曲率は跳ねない。"""
        xs = np.arange(0.0, 10.0, 0.1)
        xy = np.column_stack([xs, np.zeros_like(xs)])
        xy[50, 1] = 0.01                                  # 1cm の折れ
        self.assertGreater(np.abs(rl_mod.curvature(xy, closed=False)).max(), 1.5)
        self.assertLess(rl_mod.speed_kappa(xy, closed=False).max(), 0.6)

    def test_friction_circle_slows_the_corner_exit(self):
        xy = stadium(r=1.0)
        k = rl_mod.speed_kappa(xy)
        v_fc = rl_mod.speed_profile(xy, k, **SPEED)
        v_no = rl_mod.speed_profile(xy, k, friction_circle=False, **SPEED)
        self.assertTrue(np.all(v_fc <= v_no + 1e-9))
        # コーナーの頂点の速度（横Gで決まる）は同じ
        self.assertAlmostEqual(float(v_fc.min()), float(v_no.min()), places=6)
        self.assertLess(float(v_fc.mean()), float(v_no.mean()))

    def test_retime_uses_the_new_settings(self):
        rl = _line(stadium(r=1.0))
        a = rl_mod.retime(rl, **{**SPEED, "v_max": 1.0})
        b = rl_mod.retime(rl, **{**SPEED, "v_max": 2.5})
        self.assertAlmostEqual(float(a.v.max()), 1.0)
        self.assertGreater(float(b.v.max()), 2.0)
        self.assertTrue(np.array_equal(a.xy, rl.xy))       # 形は変えない

    def test_planner_retimes_when_the_settings_change(self):
        """★ 保存した地図の速度（地図を作ったときの設定）で走り続けない。"""
        pl = Slam2dRaceLine()
        p = {s.key: s.default for s in pl.params}
        pl.path = _line(stadium(r=1.0), v=0.5)
        pl._speed_key = None
        pl._retime({**p, "v_max": 1.5})
        self.assertAlmostEqual(float(pl.path.v.max()), 1.5)
        pl._retime({**p, "v_max": 2.5})
        self.assertGreater(float(pl.path.v.max()), 2.0)
        self.assertEqual(len(SPEED_KEYS), 5)


class TestPathGeneration(unittest.TestCase):
    def _optimize(self, **kw):
        t, traj = divider_course()
        grid = occgrid_from_trinary(t, resolution=RES, origin=(0.0, 0.0), seq=0)
        cl = cl_mod.build(grid, traj, step=0.1, max_width=3.0)
        rl = rl_mod.optimize(grid, cl, half_width=0.09, margin=0.05, lam=0.1, passes=2,
                             front_overhang=0.30, rear_overhang=0.07, **SPEED, **kw)
        body = rl_mod._BodyCheck(grid, FOOTPRINT, True)
        return rl, float(body.clearance(rl.xy).min())

    def test_body_check_keeps_the_nose_off_the_divider_tip(self):
        """★ 法線のレイで測った道幅だけだと、車体の前端が仕切りの先端をかすめる。"""
        _, plain = self._optimize()
        _, checked = self._optimize(footprint=FOOTPRINT)
        self.assertLess(plain, 0.04)
        self.assertGreaterEqual(checked, 0.05 - 0.01)

    def test_open_path_keeps_its_end_points(self):
        """★ 開いた経路の始点（車の位置）と終点（止まる場所）は、壁際でも動かさない。

        幅1mの通路で、始点が壁から 14cm。車体の余裕が足りないので外形の検査が締めに行き、
        直す前は始点が 5.7cm 動いた（経路が車の居ない所から始まる）。
        """
        from slam2d.core.grid import OccGrid
        g = OccGrid(resolution=0.025, size_m=10.0)
        g.misses[:] = 5
        for y in (0.5, -0.5):
            xs = np.linspace(-1.0, 4.0, 501)
            c, r = g.to_cell(xs, np.full(xs.size, y))
            g.hits[r, c], g.misses[r, c] = 10, 0
        g.frozen = True
        xy = np.c_[np.linspace(0.0, 3.0, 31), np.linspace(0.36, 0.0, 31)]
        cl = cl_mod.build_open(g, xy, step=0.1, max_width=3.0)
        rl = rl_mod.optimize(g, cl, half_width=0.09, margin=0.08, lam=0.1, passes=2,
                             front_overhang=0.30, rear_overhang=0.07, footprint=FOOTPRINT,
                             time_iters=2, v_start=0.5, **SPEED)
        self.assertFalse(rl.closed)
        np.testing.assert_allclose(rl.xy[0], cl.xy[0], atol=1e-6)
        np.testing.assert_allclose(rl.xy[-1], cl.xy[-1], atol=1e-6)

    def test_time_iters_is_never_slower(self):
        base, _ = self._optimize(footprint=FOOTPRINT)
        fast, clear = self._optimize(footprint=FOOTPRINT, time_iters=2)
        t0 = rl_mod._travel_time(base.xy, base.v, True)
        t1 = rl_mod._travel_time(fast.xy, fast.v, True)
        self.assertLessEqual(t1, t0 + 1e-6)
        self.assertGreaterEqual(clear, 0.05 - 0.01)

    def test_equal_bounds_stay_fixed_and_warm_start_agrees(self):
        c = stadium(r=1.0)
        nrm = cl_mod.normals(c)
        lo, hi = np.full(len(c), -0.3), np.full(len(c), 0.3)
        lo[10] = hi[10] = 0.1                               # 両側から締めた点
        # 収束させて比べる（既定の反復上限 30 では打ち切られることがある。打ち切っても
        # 箱には収まり、ラップの見積もりの差は 0.01s 以下だった）
        a = rl_mod.min_curvature_alpha(c, nrm, lo, hi, max_iter=300)
        self.assertAlmostEqual(float(a[10]), 0.1)
        warm = rl_mod.min_curvature_alpha(c, nrm, lo, hi, warm=a, max_iter=300)
        self.assertTrue(np.allclose(a, warm, atol=1e-6))


class TestPursuit(unittest.TestCase):
    def _straight_then_arc(self) -> rl_mod.RaceLine:
        pts = [(x, 0.0) for x in np.arange(0.0, 3.0, 0.1)]
        r = 1.0
        for a in np.arange(0.0, math.pi / 2, 0.1 / r):
            pts.append((3.0 + r * math.sin(a), r - r * math.cos(a)))
        xy = np.array(pts)
        rl = _line(xy, closed=False)
        v = np.full(len(xy), 1.0)
        v[:15] = 3.0
        return rl_mod.RaceLine(**{**rl._asdict(), "v": v})

    def test_speed_is_read_just_ahead_not_at_the_lookahead(self):
        """★ 注視点（2m/s で約1.2m 先）の速度ではなく、`v·preview` 先の速度。"""
        path = self._straight_then_arc()
        base = dict(lookahead_k=0.45, lookahead_min=0.3, delay_s=0.0)
        old = follow(path, (0.5, 0.0, 0.0), 2.0, 0.0, PursuitConfig(**base))
        new = follow(path, (0.5, 0.0, 0.0), 2.0, 0.0,
                     PursuitConfig(**base, speed_preview_s=0.1))
        self.assertAlmostEqual(new.speed, 3.0)      # 0.2m 先はまだ速い区間
        self.assertAlmostEqual(old.speed, 1.0)      # 1.2m 先はもう遅い区間

    def test_feedforward_stops_cutting_into_the_corner_early(self):
        """経路の上に居てまだ直線なら、フィードフォワードは舵を切らない（Pure Pursuit は切る）。"""
        path = self._straight_then_arc()
        base = dict(lookahead_k=0.45, lookahead_min=0.3, delay_s=0.0)
        pose = (2.5, 0.0, 0.0)
        pp = follow(path, pose, 3.0, 0.0, PursuitConfig(**base))
        ff = follow(path, pose, 3.0, 0.0, PursuitConfig(**base, ff_gain=1.0))
        self.assertGreater(pp.steer, 0.05)
        self.assertLess(abs(ff.steer), 0.25 * pp.steer)

    def test_feedforward_keeps_the_arc(self):
        """円の上に居れば、フィードフォワードありでも円の曲率で曲がる。"""
        xy = stadium(r=1.0, straight=0.0)
        path = _line(xy)
        pose = (float(xy[5, 0]), float(xy[5, 1]),
                math.atan2(xy[6, 1] - xy[4, 1], xy[6, 0] - xy[4, 0]))
        cfg = PursuitConfig(lookahead_k=0.2, lookahead_min=0.3, delay_s=0.0, ff_gain=1.0)
        out = follow(path, pose, 1.0, 0.0, cfg)
        self.assertAlmostEqual(out.steer, math.atan(0.23 * 1.0), delta=0.03)


if __name__ == "__main__":
    unittest.main()
