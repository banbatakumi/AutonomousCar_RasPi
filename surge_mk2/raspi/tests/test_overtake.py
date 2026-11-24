"""`raspi/nav/overtake.py`（Frenet 格子 × 時刻つきの当たり判定で追い抜く一時経路）のテスト。

半径5mの円周コース（反時計回り、道幅は `room` で与える）で確かめる:

- 経路の上に何も無ければ "clear"（何もしない）
- 静止物は横へ避け、車体＋余裕が重ならない。避ける間に速度を抑えない
- 前を同じ向きに走る遅い車は、抜く間に相手が進むぶん並ぶ区間を伸ばして抜く
- 前を同じ向きに速く走る車は塞がない（追いつかない）
- 道幅が足りなければ "blocked" で、付いていく相手（`lead`）を返す
- 後ろの車は数えない。もう並んでいる車は、今より近づかなければ塞がない
- 横へずれた途中から引き直しても、前の計画の上に居れば形が連続する
- 計算時間（Mac で数 ms）
"""

import math
import time
import unittest


import numpy as np

from raspi.nav import overtake as ot
from raspi.nav.obstacles import Obstacle
from raspi.nav.raceline import RaceLine, curvature

R = 5.0


def _ring(v=3.0) -> RaceLine:
    n = int(2 * math.pi * R / 0.1)
    a = np.arange(n) * 2 * math.pi / n
    xy = np.column_stack([R * np.cos(a), R * np.sin(a)])
    return RaceLine(xy=xy, v=np.full(n, v), kappa=curvature(xy), length=2 * math.pi * R)


def _cfg(**kw) -> ot.OvertakeConfig:
    base = dict(half_width=0.09, front=0.30, rear=0.07, kappa_max=math.tan(0.47) / 0.23,
                a_brake=3.0, a_accel=3.0)
    base.update(kw)
    return ot.OvertakeConfig(**base)


def _room(path, half=0.7):
    keep = 0.09 + 0.05
    return (np.full(len(path), half - keep), np.full(len(path), half - keep))


def _at(path, s, d=0.0, r=0.10, vx_along=0.0):
    """経路の始点から弧長 s・横 d の円柱（経路に沿った速度 vx_along）。"""
    a = s / R
    rr = R - d                    # 反時計回りなので左（+d）は内側
    x, y = rr * math.cos(a), rr * math.sin(a)
    t = (-math.sin(a), math.cos(a))
    ang = np.linspace(-math.pi / 2, math.pi / 2, 9) + math.atan2(-t[1], -t[0])   # 後ろから来る車に見える半周
    pts = np.column_stack([x + r * np.cos(ang), y + r * np.sin(ang)])
    return Obstacle(x, y, r, len(pts), pts, vx_along * t[0], vx_along * t[1])


def _plan(path, obs, v_now=3.0, pose=None, prev=None, half=0.7, **kw):
    pose = pose or (R, 0.0, math.pi / 2)
    return ot.plan(path, 0, pose, v_now, path.v, 6.0, _room(path, half), obs, _cfg(**kw), prev=prev)


def _min_gap(path, res, obs: Obstacle, cfg):
    """選んだ候補の車体（base_link から前後・半幅）と、静止物の点との最小距離 [m]。"""
    seg = np.hypot(*np.diff(np.vstack([path.xy, path.xy[:1]]), axis=0).T)
    s_cum = np.concatenate([[0.0], np.cumsum(seg[:-1])])
    pl = ot.build(path, 0, res.cand, s_cum, None)
    xy = pl.path.xy
    t = np.gradient(xy, axis=0)
    t /= np.hypot(*t.T)[:, None]
    best = math.inf
    for i in range(0, 120):
        c, s = t[i]
        for fx in np.linspace(-cfg.rear, cfg.front, 8):
            for fy in (-cfg.half_width, cfg.half_width):
                px = xy[i, 0] + fx * c - fy * s
                py = xy[i, 1] + fx * s + fy * c
                best = min(best, float(np.hypot(obs.pts[:, 0] - px, obs.pts[:, 1] - py).min()))
    return best


class TestOvertake(unittest.TestCase):
    def test_clear_when_nothing_on_the_line(self):
        path = _ring()
        self.assertEqual(_plan(path, []).status, "clear")
        self.assertEqual(_plan(path, [_at(path, 4.0, d=0.6)]).status, "clear")

    def test_static_obstacle_is_avoided_with_margin_at_full_speed(self):
        path = _ring()
        o = _at(path, 4.0)
        res = _plan(path, [o])
        self.assertEqual(res.status, "avoid")
        self.assertGreater(abs(res.cand.dT), 0.09 + 0.10 + 0.10 - 0.02)
        self.assertGreater(_min_gap(path, res, o, _cfg()), 0.08)

    def test_late_obstacle_uses_a_short_entry(self):
        # 1.2m 手前で見つけても、曲がれる限界の内側の短い入り方で避けられる（下げた速度で）
        path = _ring(v=2.0)
        res = _plan(path, [_at(path, 1.6)], v_now=2.0)
        self.assertEqual(res.status, "avoid")

    def test_slower_car_is_overtaken_with_longer_side_by_side(self):
        path = _ring()
        static = _plan(path, [_at(path, 3.0, r=0.09)])
        mover = _plan(path, [_at(path, 3.0, r=0.09, vx_along=1.5)])
        self.assertEqual(mover.status, "avoid")
        # 抜く間に相手が進むので、並ぶ区間の終わりは静止物より先
        self.assertGreater(mover.cand.S2, static.cand.S2 + 0.5)

    def test_faster_car_ahead_does_not_block(self):
        path = _ring(v=2.0)
        self.assertEqual(_plan(path, [_at(path, 1.5, vx_along=2.5)], v_now=2.0).status, "clear")

    def test_narrow_road_is_blocked_and_reports_the_lead(self):
        """道幅が足りなければ横へ出ない。前の相手に付いていく（線のまま付いていく候補が通れれば "clear"）相手を
        `lead` で返す。"""
        path = _ring()
        o = _at(path, 3.0, vx_along=1.0)
        res = _plan(path, [o], half=0.25)
        self.assertIn(res.status, ("blocked", "clear"))
        self.assertIsNotNone(res.lead)
        self.assertGreater(res.lead.u, 0.5)

    def test_car_behind_is_ignored(self):
        # 後ろから速い車が来ても、こちらの計画は塞がない（止まって避けない）
        path = _ring()
        self.assertEqual(_plan(path, [_at(path, -0.3, vx_along=4.0)]).status, "clear")

    def test_car_alongside_inside_the_margin_does_not_block(self):
        # 横 5cm（余裕 10cm の内側）に並んでいる車がいても、今より近づかない候補は通れる
        path = _ring()
        o = _at(path, 0.1, d=-(0.09 + 0.05 + 0.09), r=0.09, vx_along=2.0)
        res = _plan(path, [o])
        self.assertIn(res.status, ("clear", "avoid"))

    def test_hairpin_obstacle_is_projected_onto_its_own_branch(self):
        # U 字（直線 → 半径 0.3m の折り返し → 直線）。折り返した先の線の上の物は、足元の横（反対の枝）
        # ではなく、経路に沿って先の位置として出る
        a = np.linspace(-math.pi / 2, math.pi / 2, 10)
        xy = np.vstack([np.column_stack([np.arange(-2.0, 0.0, 0.1), np.full(20, -0.3)]),
                        np.column_stack([0.3 * np.cos(a), 0.3 * np.sin(a)]),
                        np.column_stack([np.arange(0.0, -2.0, -0.1), np.full(20, 0.3)])])
        path = RaceLine(xy=xy, v=np.full(len(xy), 2.0), kappa=curvature(xy, False),
                        length=float(np.hypot(*np.diff(xy, axis=0).T).sum()), closed=False)
        from raspi.nav.centerline import normals
        nrm = normals(xy)
        order = np.arange(len(xy))
        s_rel = np.concatenate([[0.0], np.cumsum(np.hypot(*np.diff(xy, axis=0).T))]) - 1.9
        o = Obstacle(-0.2, 0.3, 0.05, 5, np.array([[-0.2, 0.25], [-0.2, 0.3], [-0.2, 0.35]]))
        tr = ot.frenet_tracks(path, order, s_rel, nrm, [o], _cfg())
        far = [t for t in tr if t.s_lo > 1.0]
        self.assertTrue(far, tr)
        self.assertLess(abs(0.5 * (far[0].d_lo + far[0].d_hi)), 0.1)

    def test_replanning_on_the_previous_plan_is_continuous(self):
        path = _ring()
        o = _at(path, 5.0)
        r1 = _plan(path, [o])
        # 前の計画どおり 1m 進んだ位置から引き直す
        d, d1, _ = r1.cand.at(1.0)
        a = 1.0 / R
        pose = ((R - d) * math.cos(a), (R - d) * math.sin(a), a + math.pi / 2 + math.atan(d1))
        r2 = ot.plan(path, 10, pose, 3.0, path.v, 6.0, _room(path), [o], _cfg(), prev=r1.cand)
        self.assertEqual(r2.status, "avoid")
        for x in (0.3, 1.0, 2.0):
            self.assertAlmostEqual(r2.cand.at(x)[0], r1.cand.at(1.0 + x)[0], delta=0.01)

    def test_fast_enough(self):
        path = _ring()
        obs = [_at(path, 4.0), _at(path, 7.0, d=-0.3, vx_along=1.0)]
        _plan(path, obs)
        t0 = time.perf_counter()
        for _ in range(10):
            _plan(path, obs)
        ms = (time.perf_counter() - t0) * 100.0
        self.assertLess(ms, 15.0, f"{ms:.1f}ms")


class TestOpponentModel(unittest.TestCase):
    """他車の予測: 経路に沿った進め方・速度プロファイル・カーブで横の速度を使わない。"""

    def test_ghost_points_advance_along_the_curve(self):
        """経路に沿って進めた点は、同じ横ずれのまま（直線で進めるとカーブの外へはみ出す）。"""
        path = _ring()
        seg = np.hypot(*np.diff(np.vstack([path.xy, path.xy[:1]]), axis=0).T)
        s_cum = np.concatenate([[0.0], np.cumsum(seg[:-1])])
        o = _at(path, 3.0, d=-0.2, vx_along=1.5)
        g = ot.advance_along(path, s_cum, ot.normals(path.xy), o.pts, o.vx, o.vy, 1.0)
        r0 = np.hypot(*o.pts.T)
        r1 = np.hypot(*g.T)
        np.testing.assert_allclose(r1, r0, atol=0.01)
        lin = np.hypot(*(o.pts + np.array([o.vx, o.vy])).T)
        self.assertGreater(float((lin - r0).min()), 0.15)        # 直線ならはみ出す（確かめの確かめ）
        # 進んだ距離は速さ × 時間（弧長）
        self.assertAlmostEqual(float(np.arctan2(g[:, 1], g[:, 0]).mean() - np.arctan2(o.pts[:, 1], o.pts[:, 0]).mean()) * R,
                               1.5, delta=0.1)

    def test_opponent_follows_the_speed_profile(self):
        """遅い区間で 0.7 倍なら、速い区間でも 0.7 倍で進むと予測する（一定の速さにしない）。"""
        n = 300
        xy = np.column_stack([np.arange(n) * 0.1, np.zeros(n)])
        v = np.where(np.arange(n) < 50, 1.0, 3.0)
        path = RaceLine(xy=xy, v=v, kappa=np.zeros(n), length=n * 0.1)
        s_cum = np.arange(n) * 0.1
        tr = ot.Track(0.0, 0.4, -0.1, 0.1, 0.7, 0.0, True, _at(_ring(), 1.0))
        t_arr, s_arr = ot.opp_motion(tr, 0.0, s_cum, v, path.length, t_max=4.0)
        s_at = float(np.interp(4.0, t_arr, s_arr))
        # 5m を 0.7m/s（約 7.1s）… 4s では 2.8m。速い区間に入る前なので一定と同じ
        self.assertAlmostEqual(s_at, 2.8, delta=0.05)
        t_arr, s_arr = ot.opp_motion(tr._replace(u=0.7), 4.5, s_cum, v, path.length, t_max=3.0)
        # 0.5m 走って速い区間（2.1m/s）へ: 一定の 0.7m/s（2.1m）より先へ進む
        self.assertGreater(float(np.interp(3.0, t_arr, s_arr)), 4.5)

    def test_lateral_velocity_is_ignored_in_curves(self):
        """カーブでは、回帰の遅れで接線から傾いた速度を横の速度にしない。前へは速さの大きさで進む。"""
        cfg = _cfg()
        a = math.radians(25.0)
        o = Obstacle(0.0, 0.0, 0.1, 5, None, 1.5 * math.cos(a), 1.5 * math.sin(a))
        s = np.array([1.0, 1.2])
        d = np.array([0.0, 0.1])
        t, n = np.array([1.0, 0.0]), np.array([0.0, 1.0])
        tr = ot._track(o, s, d, t, n, cfg, kappa=1.0)
        self.assertEqual(tr.w, 0.0)
        self.assertAlmostEqual(tr.u, 1.5)
        tr = ot._track(o, s, d, t, n, cfg, kappa=0.0)          # 直線では今までどおり
        self.assertAlmostEqual(tr.w, 1.5 * math.sin(a))

    def test_learned_line_predicts_the_lateral_shift(self):
        """前に見た相手の線（場所ごとの横ずれ）から、相手が進んだ先での横ずれの変化を出す。"""
        n = 200
        xy = np.column_stack([np.arange(n) * 0.1, np.zeros(n)])
        path = RaceLine(xy=xy, v=np.full(n, 2.0), kappa=np.zeros(n), length=n * 0.1, closed=False)
        order = np.arange(0, 120)
        s_rel = order * 0.1
        nrm = np.tile([0.0, 1.0], (n, 1))
        # 相手の線: x=2〜4m で d が 0 → 0.4m（左へ寄る）
        xs = np.arange(0.5, 11.0, 0.05)
        trail = np.column_stack([xs, np.clip((xs - 2.0) / 2.0, 0.0, 1.0) * 0.4])
        prof = ot.line_profile(path, order, s_rel, nrm, trail)
        sh = ot.line_shift(prof, 1.5, np.array([0.0, 1.0, 2.5, 4.0]))
        np.testing.assert_allclose(sh, [0.0, 0.1, 0.4, 0.4], atol=0.03)
        # 今の場所を覚えていなければ予測しない
        self.assertTrue(np.all(ot.line_shift(prof, 11.5, np.array([0.0, 1.0])) == 0.0))



class TestPredictiveOvertake(unittest.TestCase):
    """抜き切れる区間でだけ仕掛ける（区間の入口まで付いていき、入口で横へ出て全速で抜く）。"""

    def test_trails_until_the_wide_section_then_passes(self):
        """狭い所では横へ出ず線の上で付いていき、道幅のある区間の入口で横へ出る候補を選ぶ。抜き切る（S2 が有限）。"""
        path = _ring()
        n = len(path)
        s = np.arange(n) * path.length / n
        wide = (s >= 5.0) & (s <= 16.0)
        room = (np.where(wide, 0.56, 0.05), np.full(n, 0.05))
        opp = _at(path, 1.6, r=0.09, vx_along=1.5)
        res = ot.plan(path, 0, (R, 0.0, math.pi / 2), 1.5, path.v, 6.0, room, [opp], _cfg())
        self.assertEqual(res.status, "avoid")
        self.assertGreater(res.cand.S1, 0.3)                      # 今は横へ出ない
        self.assertGreaterEqual(res.cand.S1 + res.cand.L1, 5.0 - 0.3)   # 出終わるのは広い区間の入口
        self.assertTrue(math.isfinite(res.cand.S2))
        self.assertGreater(res.cand.dT, 0.0)                      # 広いのは左

    def test_no_pass_is_started_without_room_to_finish(self):
        """抜き切る前に道幅が尽きるなら仕掛けない（並んだまま戻れなくなる試みをしない）。"""
        path = _ring()
        n = len(path)
        s = np.arange(n) * path.length / n
        short = (s >= 3.0) & (s <= 4.5)
        room = (np.where(short, 0.56, 0.05), np.full(n, 0.05))
        opp = _at(path, 1.6, r=0.09, vx_along=2.4)
        res = ot.plan(path, 0, (R, 0.0, math.pi / 2), 2.4, path.v, 6.0, room, [opp], _cfg())
        self.assertNotEqual(res.status, "avoid")

    def test_grid_points_start_ahead_of_the_car(self):
        """最も近い点（`index`）が車の少し後ろでも、見る先の格子 `sel` と弧長 x がずれない（最初の点は車より先）。"""
        path = _ring()
        a = 0.06 / R                                         # 車は点 0 の 6cm 先（点の間隔 0.1m）
        pose = (R * math.cos(a), R * math.sin(a), a + math.pi / 2)
        res = ot.plan(path, 0, pose, 2.0, path.v, 6.0, _room(path), [_at(path, 3.0, vx_along=1.0)], _cfg())
        self.assertIsNotNone(res.sel)
        seg = path.length / len(path)
        self.assertGreater(int(res.sel[0]) * seg, 0.06 - 1e-9)

    def test_learned_speed_is_used_for_prediction(self):
        """覚えた場所ごとの相手の速さ（`line_profile` の 4 つ目）で相手を進める。"""
        n = 200
        xy = np.column_stack([np.arange(n) * 0.1, np.zeros(n)])
        path = RaceLine(xy=xy, v=np.full(n, 2.0), kappa=np.zeros(n), length=n * 0.1, closed=False)
        order = np.arange(0, 150)
        s_rel = order * 0.1
        nrm = np.tile([0.0, 1.0], (n, 1))
        xs = np.arange(0.5, 14.0, 0.05)
        u = np.where(xs < 5.0, 1.0, 2.0)
        trail = np.column_stack([xs, np.zeros_like(xs), u, np.zeros_like(xs)])
        prof = ot.line_profile(path, order, s_rel, nrm, trail)
        self.assertAlmostEqual(float(np.nanmean(prof[3][5:20])), 1.0, places=2)
        self.assertAlmostEqual(float(np.nanmean(prof[3][30:60])), 2.0, places=2)
        v_learn = np.full(n, np.nan)
        b = np.clip(((s_rel - prof[0]) / 0.2).astype(int), 0, len(prof[3]) - 1)
        v_learn[order] = prof[3][b]
        s_cum = np.arange(n) * 0.1
        tr = ot.Track(1.0, 1.4, -0.1, 0.1, 1.0, 0.0, True, Obstacle(1.0, 0.0, 0.1, 5))
        t_arr, s_arr = ot.opp_motion(tr, 1.0, s_cum, path.v, path.length, t_max=6.0, v_learn=v_learn)
        # 1m → 5m を 1m/s（4s）、その先 2m/s。6s で 4 + 4 = 8m（比で伸ばすと今の 1m/s のまま 6m）
        self.assertAlmostEqual(float(np.interp(6.0, t_arr, s_arr)), 8.0, delta=0.3)

    def test_learned_speed_uses_position_differences_when_timed(self):
        """軌跡に時刻があれば、速さは位置の前後の差から覚える（追跡の速さが減速中に遅れて高く出ても偏らない）。"""
        n = 200
        xy = np.column_stack([np.arange(n) * 0.1, np.zeros(n)])
        path = RaceLine(xy=xy, v=np.full(n, 2.0), kappa=np.zeros(n), length=n * 0.1, closed=False)
        order = np.arange(0, 150)
        s_rel = order * 0.1
        nrm = np.tile([0.0, 1.0], (n, 1))
        t = np.arange(0.0, 6.0, 0.1)
        xs = 0.5 + 1.5 * t                                   # 真の速さ 1.5m/s
        trail = np.column_stack([xs, np.zeros_like(xs), np.full_like(xs, 2.4), np.zeros_like(xs), t])  # 追跡の速さは 2.4
        prof = ot.line_profile(path, order, s_rel, nrm, trail)
        self.assertAlmostEqual(float(np.nanmean(prof[3][10:40])), 1.5, places=2)

    def test_speed_recovers_with_acceleration_limit_after_trailing(self):
        """付いていった後に元の速さへ戻るところは加速度の上限で上げる（瞬時に戻らない）。"""
        x = np.arange(0.0, 8.01, 0.2)
        ds = np.diff(x)
        t_free = x / 3.0
        follow = (1.0, np.array([0.0, 10.0]), np.array([0.0, 10.0]), 0.3, 0.3)    # 相手は 1m 先を 1m/s
        t, ok, _ = ot._follow_times(t_free[None, :], x, ds, np.array([True]), follow, 3.0,
                                    upto=np.array([2.0]), a_accel=2.0)
        v = ds / np.diff(t[0])
        k = int(np.searchsorted(x, 2.0))
        self.assertLess(v[k - 1], 1.2)                           # 出始めるまでは相手の速さ
        for j in range(k, len(v) - 1):                          # その先は v² の増え方が 2·a·ds 以下
            self.assertLessEqual(v[j + 1] ** 2 - v[j] ** 2, 2 * 2.0 * ds[j] * 2.0 + 1e-6)



class TestLongitudinalModel(unittest.TestCase):
    """前後に出せる加速度は速度プロファイルと同じモデル（横の使い具合 u = v²|κ| ÷ 横の限界 で減る）。"""

    def test_friction_circle_reduces_braking_in_a_corner(self):
        lon = ot._Long(np.array([4.0]), 3.0, 2.5, None)
        self.assertAlmostEqual(float(lon.dec(np.array([2.0]), np.array([0.0]))[0]), 2.5)
        # u = 2²·0.8/4 = 0.8 → 2.5·√(1−0.64) = 1.5
        self.assertAlmostEqual(float(lon.dec(np.array([2.0]), np.array([0.8]))[0]), 1.5, places=6)

    def test_learned_envelope_is_interpolated(self):
        lon = ot._Long(np.array([4.0]), 3.0, 2.5, ((3.0, 2.0, 1.0), (2.6, 2.0, 0.8)))
        # u = 0.25 → 減速の包絡 2.6 と 2.0 の中間
        self.assertAlmostEqual(float(lon.dec(np.array([1.0]), np.array([1.0]))[0]), 2.3, places=6)
        self.assertAlmostEqual(float(lon.acc(np.array([2.0]), np.array([0.0]))[0]), 3.0)

    def test_follower_brakes_earlier_before_a_corner(self):
        """コーナーの手前で減速する相手に付いていく計画は、横を使う所の減速の小さいぶん早く減速し始める。"""
        x = np.arange(0.0, 6.01, 0.2)
        ds = np.diff(x)
        kap = np.where(x > 2.0, 1.2, 0.0)[None, :]
        t_free = x / 2.5
        follow = (1.0, np.array([0.0, 1.0, 10.0]), np.array([0.0, 2.0, 2.0 + 9.0 * 0.6]), 0.3, 0.3)
        lon_c = ot._Long(np.full(len(x), 3.0), 3.0, 2.5, None)
        t1, _, _ = ot._follow_times(t_free[None, :], x, ds, np.array([True]), follow, 2.5,
                                    a_accel=3.0, lon=lon_c, kap=kap)
        t0, _, _ = ot._follow_times(t_free[None, :], x, ds, np.array([True]), follow, 2.5, a_accel=3.0)
        v1 = ds / np.diff(t1[0])
        v0 = ds / np.diff(t0[0])
        k = int(np.searchsorted(x, 1.6))
        self.assertLess(v1[k], v0[k] + 1e-9)               # 横の使い具合を見ると、コーナーより手前で遅い


if __name__ == "__main__":
    unittest.main()


class TestRecordState(unittest.TestCase):
    """`Slam2dRaceLine._record_state`: 追い抜きの判定とアンダーステアの学習を `AutoState` に載せる（記録用）。"""

    def _planner(self, status, lead):
        from types import SimpleNamespace
        us = SimpleNamespace(k=0.0021, ready=True, a_lock=4.8)
        return SimpleNamespace(_ot_status=status, _ot_lead=lead, _avoid_ms=3.456, _us=us)

    def test_overtake_and_understeer_are_recorded(self):
        from raspi.auto.slam2d_raceline import Slam2dRaceLine
        from raspi.msgs import AutoState

        lead = ot.Track(s_lo=1.2, s_hi=1.5, d_lo=-0.1, d_hi=0.1, u=0.8, w=0.0, moving=True, src=None)
        st = AutoState(grip={"lat": 5.0})
        p = {"overtake": 1.0, "obstacle_avoid": 1.0, "us_learn": 1.0}
        Slam2dRaceLine._record_state(self._planner("blocked", lead), st, p)
        self.assertEqual(st.overtake, "blocked")
        self.assertEqual((st.ot_lead_s, st.ot_lead_d, st.ot_lead_u), (1.2, 0.0, 0.8))
        self.assertEqual(st.avoid_ms, 3.46)
        self.assertEqual(st.grip["lat"], 5.0)               # 横グリップの学習の値は消さない
        self.assertEqual((st.grip["us_k"], st.grip["us_ready"], st.grip["us_lock"]), (0.0021, 1.0, 4.8))

    def test_nothing_when_features_are_off(self):
        from raspi.auto.slam2d_raceline import Slam2dRaceLine
        from raspi.msgs import AutoState

        st = AutoState()
        Slam2dRaceLine._record_state(self._planner("blocked", None), st,
                                     {"overtake": 0.0, "obstacle_avoid": 1.0, "us_learn": 0.0})
        self.assertEqual((st.overtake, st.avoid_ms, st.grip), ("", 0.0, {}))
