"""道路グラフ・経路計画・経路の切替（`nav/roadgraph.py`・`nav/route.py`・
`nav/route_switch.py`）と、開いた経路のレーシングライン・追従のテスト。

合成した地図で**トポロジーが期待どおりか**（分岐の数・エッジの数）を先に確かめ、
その上で「経由点で枝を選べるか」「曲がれない分岐を通らないか」「逆走しないか」
「分岐の口で道幅を測りすぎないか」「切替は乗れる場所まで待つか」を試す。
"""

import json
import math
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

import numpy as np  # noqa: E402

from raspi.auto._slam2d_nav import occgrid_from_trinary  # noqa: E402
from raspi.auto.route_config import RouteConfig  # noqa: E402
from raspi.nav import raceline as rl_mod  # noqa: E402
from raspi.nav.purepursuit import PursuitConfig, follow  # noqa: E402
from raspi.nav.roadgraph import CorridorGrid, build_graph  # noqa: E402
from raspi.nav.route import (RouteError, Waypoint, build_raceline,  # noqa: E402
                             plan_loop, plan_to, snap, tightest_radius,
                             waypoints_from_traj)
from raspi.nav.route_switch import Mission, RouteSwitcher, lap_crossed  # noqa: E402

RES = 0.025
KEEP = 0.11          # 車体半幅 0.09 + 余裕 0.02
HALF_WIDTH = 0.09


def _canvas(w_m: float, h_m: float) -> np.ndarray:
    """周りを未知（0）、中を空き（1）にした 3値地図。外周から 0.25m は未知。"""
    t = np.zeros((int(h_m / RES), int(w_m / RES)), dtype=np.uint8)
    return t


def _rect(t: np.ndarray, x0, y0, x1, y1, v: int) -> None:
    t[int(y0 / RES):int(y1 / RES), int(x0 / RES):int(x1 / RES)] = v


def _box_wall(t: np.ndarray, x0, y0, x1, y1, th=0.05) -> None:
    _rect(t, x0, y0, x1, y0 + th, 2)
    _rect(t, x0, y1 - th, x1, y1, 2)
    _rect(t, x0, y0, x0 + th, y1, 2)
    _rect(t, x1 - th, y0, x1, y1, 2)


def oval() -> np.ndarray:
    """6×4m の外壁と 4×2m の島。幅 0.95m の周回路。"""
    t = _canvas(7.0, 5.0)
    _rect(t, 0.5, 0.5, 6.5, 4.5, 1)
    _box_wall(t, 0.5, 0.5, 6.5, 4.5)
    _rect(t, 1.5, 1.5, 5.5, 3.5, 2)
    return t


def shortcut(gap: float = 0.8) -> np.ndarray:
    """オーバルの島を2つに割り、真ん中に上下をつなぐ近道（幅 `gap`）を開けたもの。"""
    t = oval()
    _rect(t, 1.5, 1.5, 5.5, 3.5, 1)
    _rect(t, 1.5, 1.5, 3.5 - gap / 2, 3.5, 2)
    _rect(t, 3.5 + gap / 2, 1.5, 5.5, 3.5, 2)
    return t


def graph_of(t: np.ndarray, traj=None):
    return build_graph(t, resolution=RES, origin=(0.0, 0.0), keep=KEEP, seed=(3.5, 1.0),
                       traj=traj)


def branches(g) -> int:
    return sum(1 for n in g.nodes if len(n.edges) >= 3)


class TestRoadGraph(unittest.TestCase):
    def test_oval_is_one_loop(self):
        g = graph_of(oval())
        self.assertEqual(branches(g), 0)
        self.assertEqual(len(g.edges), 1)
        e = g.edges[0]
        self.assertEqual(e.u, e.v)                 # 自己ループ（周回）
        # 周回路の中心線の長さ ≒ 2×(5+3) = 16m
        self.assertAlmostEqual(e.length, 16.0, delta=1.2)

    def test_shortcut_has_two_branches_three_edges(self):
        g = graph_of(shortcut())
        self.assertEqual(branches(g), 2)
        self.assertEqual(len(g.edges), 3)

    def test_too_narrow_passage_disappears(self):
        # 近道の幅 0.15m は車体（keep 0.11m ×2）より狭い → 通れない＝グラフに無い
        g = graph_of(shortcut(gap=0.15))
        self.assertEqual(branches(g), 0)
        self.assertEqual(len(g.edges), 1)

    def test_unknown_speckles_do_not_create_branches(self):
        t = shortcut()
        rng = np.random.default_rng(0)
        free = np.argwhere(t == 1)
        pick = free[rng.choice(len(free), size=300, replace=False)]
        t[pick[:, 0], pick[:, 1]] = 0              # 見えなかった単独セル
        g = graph_of(t)
        self.assertEqual(branches(g), 2)
        self.assertEqual(len(g.edges), 3)

    def test_thin_on_bounding_box_matches_full_grid(self):
        from raspi.nav.roadgraph import _thin, thin
        for t in (oval(), shortcut()):
            mask = t == 1
            self.assertTrue(np.array_equal(thin(mask), _thin(mask, 500)))
        self.assertFalse(thin(np.zeros((5, 5), dtype=bool)).any())

    def test_label_covers_free_space(self):
        g = graph_of(shortcut())
        free = shortcut() == 1
        # 空きのほぼ全部がどれかのエッジに割り当たる（外の未知・壁は -1）
        self.assertGreater((g.label[free] >= 0).mean(), 0.97)
        self.assertTrue((g.label[~free] < 0).all())

    def test_directions_from_trajectory(self):
        # 下の直線を +x 向きに走った軌跡 → 下を通るエッジは一方向になる
        xs = np.linspace(1.0, 6.0, 200)
        traj = np.column_stack([xs, np.full_like(xs, 1.0)])
        g = graph_of(shortcut(), traj=traj)
        bottom = int(g.label[int(1.0 / RES), int(2.0 / RES)])
        self.assertNotEqual(g.edges[bottom].dir_allowed, 0)


class TestRoute(unittest.TestCase):
    def setUp(self):
        self.t = shortcut()
        self.g = graph_of(self.t)
        self.grid = occgrid_from_trinary(self.t, resolution=RES, origin=(0.0, 0.0), seq=0)

    def _edges_near(self, x, y):
        return int(self.g.label[int(y / RES), int(x / RES)])

    #: 下の直線の左寄り、+x 向き（分岐点 (3.5, 1.0) からは離す。分岐点の上に
    #: 置くとどのエッジに吸着するかが決まらない）
    START = Waypoint(2.0, 1.0, 0.0)

    def test_duplicate_waypoint_does_not_add_a_lap(self):
        """同じ場所の経由点（二度押し・数cm手前）で、1周余計に回る経路にならない。"""
        base = plan_loop(self.g, [self.START, Waypoint(6.0, 2.5)])
        twice = plan_loop(self.g, [self.START, Waypoint(2.0, 1.0), Waypoint(6.0, 2.5)])
        behind = plan_loop(self.g, [self.START, Waypoint(1.98, 1.0), Waypoint(6.0, 2.5)])
        self.assertAlmostEqual(twice.length, base.length, delta=0.05)
        self.assertAlmostEqual(behind.length, base.length, delta=0.05)

    def test_too_close_goal_is_reached_after_a_lap_across_a_junction(self):
        """★ 止まり切れないほど近い停止点へは1周回ってから着く——**分岐をまたいでも**。

        車は分岐（3.5, 1.0）の手前、停止点はその先の別のエッジ。以前は「今のエッジを
        `min_len` 先まで進んだ点」を経由していて、その点がエッジの端で頭打ちになり、
        1.2m しかない経路がそのまま返っていた。
        """
        pose = (3.0, 1.0, 0.0)
        near = plan_to(self.g, pose, Waypoint(4.2, 1.0))
        self.assertLess(near.length, 1.5)
        # 向きを指定しない停止点: 止まり切れる長さの経路になる（反対側から着いてもよい）
        any_dir = plan_to(self.g, pose, Waypoint(4.2, 1.0), min_len=2.5)
        self.assertGreaterEqual(any_dir.length, 2.5)
        np.testing.assert_allclose(any_dir.xy[-1], [4.2, 1.0], atol=0.05)
        # 向き（+x）を指定した停止点: いったん通り過ぎて、1周してから同じ向きで着く
        far = plan_to(self.g, pose, Waypoint(4.2, 1.0, 0.0), min_len=2.5)
        self.assertGreater(far.length, near.length + 8.0)
        np.testing.assert_allclose(far.xy[-1], [4.2, 1.0], atol=0.05)
        d = np.hypot(far.xy[:, 0] - 4.2, far.xy[:, 1] - 1.0)
        self.assertLess(int(np.argmax(d < 0.1)), len(far.xy) // 4)

    def test_start_snaps_to_the_given_edges(self):
        """始点の吸着を今走っている経路のエッジに限れる（分岐の近くで別の枝を拾わない）。"""
        from raspi.nav.route import snap
        passage = self._edges_near(3.5, 2.5)
        bottom = self._edges_near(2.0, 1.0)
        self.assertEqual(snap(self.g, 3.5, 1.6).edge, passage)
        self.assertEqual(snap(self.g, 3.5, 1.6, edges={bottom}).edge, bottom)
        rp = plan_to(self.g, (3.45, 1.3, 0.0), Waypoint(6.0, 2.5), start_edges={bottom})
        self.assertEqual(rp.steps[0][0], bottom)

    def test_waypoint_selects_branch(self):
        start = self.START
        via_short = plan_loop(self.g, [start, Waypoint(3.5, 2.5)])
        full = plan_loop(self.g, [start, Waypoint(6.0, 2.5)])
        passage = self._edges_near(3.5, 2.5)
        self.assertIn(passage, via_short.edge_ids)
        self.assertNotIn(passage, full.edge_ids)
        self.assertLess(via_short.length, full.length)
        for r in (via_short, full):
            self.assertTrue(r.closed)
            gap = np.hypot(*(r.xy[-1] - r.xy[0]))
            self.assertLess(gap, 0.2)               # 閉じている

    def test_turn_limit_forbids_passage(self):
        # 近道に入るには 90°曲がる。上限 60°なら近道を通る周回は作れない
        start = self.START
        with self.assertRaises(RouteError):
            plan_loop(self.g, [start, Waypoint(3.5, 2.5)], turn_max_deg=60.0)
        plan_loop(self.g, [start, Waypoint(3.5, 2.5)], turn_max_deg=100.0)

    def test_no_u_turn_at_junction(self):
        # 下の直線を +x に進みながら、すぐ後ろ（-x 側）の点へ行く最短は「折り返し」だが、
        # それは許されない → 1周回って戻ってくる
        start = Waypoint(2.0, 1.0, 0.0)
        r = plan_loop(self.g, [start, Waypoint(1.5, 1.0, 0.0)])
        self.assertGreater(r.length, 10.0)

    def test_wrong_way_rejected(self):
        sp = snap(self.g, 2.0, 1.0)
        # 下の直線を +x 向きにしか通れないようにする（エッジの u→v が +x なら +1）
        self.g.edges[sp.edge].dir_allowed = +1 if sp.tangent[0] > 0 else -1
        with self.assertRaises(RouteError):
            plan_loop(self.g, [Waypoint(2.0, 1.0, math.pi)])
        plan_loop(self.g, [Waypoint(2.0, 1.0, 0.0)])

    def test_waypoint_off_road(self):
        with self.assertRaises(RouteError):
            plan_loop(self.g, [Waypoint(2.5, 2.5)])  # 島の中（道から 0.6m 以上）

    def test_corridor_blocks_unused_branch(self):
        # 近道を使わない経路の回廊では、下の直線の近道の口から上へ撃ったレイは
        # 近道の奥まで抜けない（分岐の口で道幅を測りすぎない）
        full = plan_loop(self.g, [self.START, Waypoint(6.0, 2.5)])
        corr = CorridorGrid(self.grid, self.g, full.edge_ids, full.node_ids(self.g))
        up = np.array([math.pi / 2])
        d_all = float(self.grid.raycast(3.5, 1.0, up, 3.0)[0])
        d_cor = float(corr.raycast(3.5, 1.0, up, 3.0)[0])
        self.assertGreater(d_all, 2.0)
        self.assertLess(d_cor, 1.2)

    def test_raceline_stays_on_road(self):
        r = plan_loop(self.g, [self.START, Waypoint(3.5, 2.5)])
        rl, cl = build_raceline(self.grid, self.g, r, half_width=HALF_WIDTH, margin=0.05,
                                lam=0.1, passes=2, v_max=2.0, v_min=0.3, a_lat=2.5,
                                a_accel=1.2, a_brake=1.8)
        clear = self.g.clearance
        col = (rl.xy[:, 0] / RES).astype(int)
        row = (rl.xy[:, 1] / RES).astype(int)
        self.assertGreaterEqual(float(clear[row, col].min()), HALF_WIDTH + 0.05 - RES)
        self.assertTrue(rl.closed)

    def test_plan_to_is_open_and_ends_at_goal(self):
        r = plan_to(self.g, (1.0, 1.0, 0.0), Waypoint(5.0, 1.0))
        self.assertFalse(r.closed)
        self.assertLess(np.hypot(*(r.xy[-1] - [5.0, 1.0])), 1e-6)
        self.assertAlmostEqual(r.length, 4.0, delta=0.3)
        # すぐ後ろのゴールへは1周回って行く
        back = plan_to(self.g, (3.0, 1.0, 0.0), Waypoint(2.5, 1.0), min_len=1.0)
        self.assertGreater(back.length, 10.0)

    def test_plan_to_park_ends_past_slot(self):
        # 道から外れた駐車枠 (4.0, 0.7) → 道の上の最寄り点 (≈4.0, 1.0) の 1m 先で終わる
        r = plan_to(self.g, (1.0, 1.0, 0.0), Waypoint(4.0, 0.7), snap_radius=2.5,
                    end_at_goal=False, extend=1.0)
        self.assertAlmostEqual(float(r.xy[-1, 0]), 5.0, delta=0.15)
        self.assertAlmostEqual(float(r.xy[-1, 1]), 1.0, delta=0.15)

    def test_waypoints_from_traj(self):
        xs = np.linspace(0, 10, 101)
        wps = waypoints_from_traj(np.column_stack([xs, np.zeros_like(xs)]), spacing=2.0)
        self.assertEqual(len(wps), 5)
        self.assertTrue(all(abs(w.yaw) < 1e-6 for w in wps))


class TestOpenPath(unittest.TestCase):
    def _line(self, n=51, closed=False):
        xy = np.column_stack([np.linspace(0, 5, n), np.zeros(n)])
        k = rl_mod.curvature(xy, closed)
        v = rl_mod.speed_profile(xy, k, v_max=2.0, v_min=0.25, a_lat=2.5, a_accel=1.0,
                                 a_brake=1.5, closed=closed, v_start=0.0, v_end=0.25)
        seg = np.hypot(*np.diff(xy, axis=0).T)
        return rl_mod.RaceLine(xy=xy, v=v, kappa=k,
                               length=float(seg.sum()), closed=closed)

    def test_open_speed_profile_brakes_to_end(self):
        p = self._line()
        self.assertAlmostEqual(float(p.v[-1]), 0.25)
        self.assertAlmostEqual(float(p.v[0]), 0.25)          # 静止から加速
        self.assertGreater(float(p.v[len(p) // 2]), 1.0)
        self.assertTrue(np.all(np.diff(p.v[len(p) // 2:]) <= 1e-9))

    def test_open_curvature_ends_are_not_wrapped(self):
        xy = np.column_stack([np.linspace(0, 5, 51), np.zeros(51)])
        self.assertTrue(np.allclose(rl_mod.curvature(xy, closed=False), 0.0))
        # L字: 閉じた扱いにすると終点→始点の区間で端の曲率が立つ（開路版が要る理由）
        lxy = np.vstack([xy, np.column_stack([np.full(20, 5.0), np.linspace(0.1, 2, 20)])])
        self.assertAlmostEqual(float(rl_mod.curvature(lxy, closed=False)[0]), 0.0)
        self.assertGreater(abs(rl_mod.curvature(lxy, closed=True)[0]), 0.1)

    def test_follow_open_path_does_not_wrap(self):
        p = self._line()
        cfg = PursuitConfig(lookahead_k=0.5, lookahead_min=0.4, delay_s=0.0)
        pp = follow(p, (4.9, 0.0, 0.0), 0.5, 0.0, cfg)
        self.assertLess(pp.remaining, 0.2)
        self.assertGreater(pp.target[0], 4.9)                # 終点の先へ延ばした点
        self.assertAlmostEqual(pp.steer, 0.0, places=6)

    def test_open_optimize_keeps_endpoints(self):
        t = oval()
        g = graph_of(t)
        grid = occgrid_from_trinary(t, resolution=RES, origin=(0.0, 0.0), seq=0)
        r = plan_to(g, (1.2, 1.0, 0.0), Waypoint(5.8, 4.0))
        rl, _ = build_raceline(grid, g, r, half_width=HALF_WIDTH, margin=0.05, lam=0.1,
                               passes=2, v_max=2.0, v_min=0.25, a_lat=2.5, a_accel=1.2,
                               a_brake=1.8, v_start=0.0, v_end=0.25)
        self.assertFalse(rl.closed)
        self.assertLess(np.hypot(*(rl.xy[0] - [1.2, 1.0])), 0.05)
        self.assertLess(np.hypot(*(rl.xy[-1] - [5.8, 4.0])), 0.05)


class TestTightTurns(unittest.TestCase):
    """車の曲がれる限界に近いヘアピン（toyota2 の近道）で速度を落とすか。"""

    def _circle_track(self, r_small: float):
        # 直線 → 半径 r_small の半円 → 直線 → 半径 1.0 の半円（閉路、0.1m 刻み）
        pts = []
        for x in np.arange(0.0, 3.0, 0.1):
            pts.append((x, 0.0))
        for a in np.arange(-math.pi / 2, math.pi / 2, 0.1 / r_small):
            pts.append((3.0 + r_small * math.cos(a), r_small + r_small * math.sin(a)))
        for x in np.arange(3.0, 0.0, -0.1):
            pts.append((x, 2 * r_small))
        xy = np.array(pts)
        k = rl_mod.curvature(xy)
        v = np.full(len(xy), 2.0)
        return rl_mod.RaceLine(xy=xy, v=v, kappa=k, length=float(len(xy) * 0.1))

    def test_tightest_radius_measures_over_a_chord(self):
        rl = self._circle_track(0.5)
        r, _ = tightest_radius(rl)
        self.assertAlmostEqual(r, 0.5, delta=0.1)

    def test_slows_only_near_the_limit(self):
        """限界に近いヘアピンだけ横Gを削る（一律 `v_min` にはしない）。"""
        kmax = math.tan(0.524) / 0.23                    # 約 2.5（半径 0.4m）
        kw = dict(v_max=2.0, v_min=0.35, a_lat=2.5, a_accel=1.0, a_brake=1.5)
        tight = rl_mod.retime(self._circle_track(0.42), kappa_max=kmax, **kw)
        free = rl_mod.retime(self._circle_track(0.42), **kw)
        self.assertLess(float(tight.v.min()), 0.8 * float(free.v.min()))
        self.assertGreater(float(tight.v.min()), 0.35 + 0.1)   # 最低速度に張り付かない
        wide = rl_mod.retime(self._circle_track(1.5), kappa_max=kmax, **kw)
        wide_free = rl_mod.retime(self._circle_track(1.5), **kw)
        self.assertTrue(np.allclose(wide.v, wide_free.v))      # 限界から遠ければ触らない


def _straight(y0: float, y1: float, v=1.0, n=100):
    """x=0..10 の直線。前半は y=y0、後半は y=y1 へ移る。"""
    xs = np.linspace(0, 10, n)
    ys = np.where(xs < 5, y0, y0 + (y1 - y0) * np.clip((xs - 5) / 2, 0, 1))
    xy = np.column_stack([xs, ys])
    seg = np.hypot(*np.diff(xy, axis=0).T)
    return rl_mod.RaceLine(xy=xy, v=np.full(n, v), kappa=np.zeros(n), length=float(seg.sum()),
                           closed=False)


class TestSwitch(unittest.TestCase):
    def test_switch_before_divergence_is_immediate(self):
        a, b = _straight(0, 0), _straight(0, 1.0)
        sw = RouteSwitcher()
        sw.set_active("A", a)
        sw.request("B", b, "gui")
        r = sw.step((2.0, 0.0, 0.0), 1.0, tol=0.15)
        self.assertTrue(r.switched)
        self.assertEqual(sw.active_key, "B")

    def test_switch_after_divergence_waits(self):
        a, b = _straight(0, 0), _straight(0, 1.0)
        sw = RouteSwitcher()
        sw.set_active("A", a)
        sw.request("B", b)
        r = sw.step((8.0, 0.0, 0.0), 1.0, tol=0.15)      # もう分かれた後
        self.assertFalse(r.switched)
        self.assertEqual(sw.active_key, "A")
        self.assertEqual(sw.pending_key, "B")
        self.assertIn("待っている", r.reason)

    def test_switch_waits_when_too_fast(self):
        a = _straight(0, 0, v=2.0)
        b = _straight(0, 0, v=0.3)
        sw = RouteSwitcher()
        sw.set_active("A", a)
        sw.request("B", b)
        self.assertFalse(sw.step((0.5, 0.0, 0.0), 2.0, tol=0.15, a_brake=1.0).switched)
        self.assertTrue(sw.step((0.5, 0.0, 0.0), 0.4, tol=0.15, a_brake=1.0).switched)

    def test_request_current_cancels_pending(self):
        a, b = _straight(0, 0), _straight(0, 1.0)
        sw = RouteSwitcher()
        sw.set_active("A", a)
        sw.request("B", b)
        sw.request("A", a)
        self.assertIsNone(sw.pending)

    def test_lap_and_mission(self):
        self.assertTrue(lap_crossed(95, 2, 100))
        self.assertFalse(lap_crossed(50, 52, 100))
        m = Mission.from_dict({"laps": 3, "then": "P1"})
        self.assertFalse(m.due(2, 100.0))
        self.assertTrue(m.due(3, 0.0))
        self.assertFalse(Mission.from_dict({"laps": 3}).active())      # 行き先が無い
        self.assertTrue(Mission.from_dict({"time_s": 10, "then": "P"}).due(0, 10.0))


class TestRouteConfig(unittest.TestCase):
    def test_round_trip(self):
        d = {"groups": {"A": [[1, 2], [3, 4, 0.5]]}, "active": "A",
             "stops": {"P1": {"x": 1, "y": 2, "yaw": 0.0, "mode": "park"}},
             "mission": {"laps": 3, "then": "P1"}, "signal_map": {"left": "A"}}
        c = RouteConfig.from_json(json.dumps(d))
        self.assertEqual(len(c.groups["A"]), 2)
        self.assertIsNone(c.groups["A"][0].yaw)
        c2 = RouteConfig.from_json(c.to_json())
        self.assertTrue(c.same_groups(c2))
        self.assertEqual(c2.stops["P1"].mode, "park")

    def test_rejects_bad(self):
        for bad in ({"groups": {"E": [[0, 0]]}},
                    {"groups": {"A": [[0]]}},
                    {"stops": {"P": {"x": 0, "y": 0, "mode": "park"}}},        # 駐車に向きが無い
                    {"mission": {"laps": 1, "then": "nowhere"}},
                    {"signal_map": {"left": "Z"}},
                    {"groups": {"A": [[float("nan"), 0]]}}):
            with self.assertRaises(ValueError, msg=str(bad)):
                RouteConfig.from_dict(bad)


    def test_wrong_types_are_value_errors(self):
        """型の違う設定（手で編集した routes.json 等）も `ValueError` に揃える。

        呼び出し側は `except ValueError` だけなので、`AttributeError`・`TypeError`・
        `OverflowError` で抜けると planning_node まで届いていた。
        """
        for text in ('{"groups": [1]}', '{"stops": [1]}', '{"signal_map": [1]}',
                     '{"mission": {"laps": [1]}}', '{"mission": {"laps": 1e400}}',
                     '{"mission": {"time_s": "x"}}', '{"signal_map": {"a": ["A"]}}',
                     '{"explore_traj": [[1, 2], [3]]}', '[1, 2]'):
            with self.assertRaises(ValueError, msg=text):
                RouteConfig.from_json(text)
        self.assertEqual(RouteConfig.from_json('{"groups": null, "mission": null}').groups, {})


class TestSlam2dRoutePlanner(unittest.TestCase):
    """planner をまとめて: 保存済み地図＋経路の設定を読み、ワーカーで経路を作り、
    GUI 向けの地図に載せ、切替・経由点の編集に応じる。"""

    def setUp(self):
        import tempfile

        from raspi.auto import mapstore
        self._tmp = tempfile.TemporaryDirectory()
        self._old = mapstore.MAPS_DIR
        mapstore.MAPS_DIR = Path(self._tmp.name)
        t = shortcut()
        loop = plan_loop(graph_of(t), [Waypoint(2.0, 1.0, 0.0), Waypoint(6.0, 2.5)]).xy
        mapstore.save_map("m", resolution=RES, origin_x=0.0, origin_y=0.0, trinary=t,
                          centerline_xy=np.zeros((0, 2)), raceline_xy=loop,
                          raceline_v=np.ones(len(loop)))
        cfg = {"groups": {"A": [[2.0, 1.0, 0.0], [6.0, 2.5]],
                          "B": [[2.0, 1.0, 0.0], [3.5, 2.5]]},
               "active": "A", "signal_map": {"left": "B"}}
        mapstore.save_routes("m", json.dumps(cfg))

    def tearDown(self):
        from raspi.auto import mapstore
        mapstore.MAPS_DIR = self._old
        self._tmp.cleanup()

    def _wait(self, pl):
        while pl._job is not None:
            pl._job.result(timeout=30)
            pl._poll()

    def test_load_builds_routes_and_switches(self):
        from raspi.auto.slam2d_route import Slam2dRoute
        pl = Slam2dRoute()
        try:
            pl.request_load("m")
            self.assertEqual(pl._load_error, "")
            self._wait(pl)
            # 自動経路（auto）はグループがあっても作り、選べる
            self.assertEqual(set(pl._routes), {"auto", "A", "B"}, pl._route_err)
            self.assertLess(pl._routes["B"].length, pl._routes["A"].length)   # B は近道

            m = pl.snapshot()
            self.assertEqual(set(m.routes), {"auto", "A", "B"})
            self.assertGreater(len(m.graph_xy), 0)
            self.assertEqual(len(m.graph_breaks), 3)
            self.assertIn('"A"', m.routes_json)

            # 走り出したことにして、信号で B へ（乗れる地点までは保留）
            pl._switch.set_active("A", pl._routes["A"])
            pl.request_signal("left", "test")
            self.assertEqual(pl._switch.pending_key, "B")
            pl.request_signal("unknown", "test")
            self.assertIn("unknown", pl._switch_note)

            # 経由点を変えると作り直す（版が上がり、今の経路は新しい版へ乗り換え待ち）
            ver = pl._route_ver
            cfg = json.loads(m.routes_json)
            cfg["groups"]["C"] = [[2.0, 1.0, 0.0]]
            pl.request_routes(json.dumps(cfg))
            self._wait(pl)
            self.assertIn("C", pl._routes)
            self.assertGreater(pl._route_ver, ver)
        finally:
            pl.close()

    def test_no_waypoints_means_auto_route_not_written(self):
        # 経由点を置いていない地図: 自動経路（auto）で走るが、設定のグループには何も書かない
        from raspi.auto import mapstore
        from raspi.auto.slam2d_route import Slam2dRoute
        mapstore.save_routes("m", "{}")
        pl = Slam2dRoute()
        try:
            pl.request_load("m")
            self._wait(pl)
            self.assertEqual(set(pl._routes), {"auto"})
            self.assertEqual(pl._cfg.groups, {})
            self.assertEqual(json.loads(pl.snapshot().routes_json)["groups"], {})

            # 経由点を置いても自動経路は残る（走行中の経路はそのまま）。A も選べ、自動へ戻せる
            pl._switch.set_active("auto", pl._routes["auto"])
            pl.request_routes(json.dumps({"groups": {"A": [[2.0, 1.0, 0.0], [3.5, 2.5]]}}))
            self._wait(pl)
            self.assertEqual(set(pl._routes), {"auto", "A"})
            self.assertEqual(pl._switch.active_key, "auto")
            pl.request_route("A", "test")
            self.assertEqual(pl._switch.pending_key, "A")
            pl.request_route("auto", "test")
            self.assertEqual(pl._switch.pending_key, "auto")

            # 全部消すと自動経路に戻る
            pl.request_routes(json.dumps({"groups": {}}))
            self._wait(pl)
            self.assertEqual(set(pl._routes), {"auto"})
        finally:
            pl.close()

    def test_auto_route_is_fastest_loop_and_respects_avoid(self):
        # 経由点なし＋地図作成の軌跡あり: スタートから全周回を比べて最速（近道）を選ぶ。
        # 近道に避ける点を置くと、近道を通らない周回になる
        from raspi.auto import mapstore
        from raspi.auto.slam2d_route import Slam2dRoute
        xs = np.linspace(1.5, 3.0, 20)
        traj = np.column_stack([xs, np.full_like(xs, 1.0)]).tolist()   # 下の直線を +x 向き
        mapstore.save_routes("m", json.dumps({"explore_traj": traj}))
        pl = Slam2dRoute()
        try:
            pl.request_load("m")
            self._wait(pl)
            self.assertEqual(set(pl._routes), {"auto"})
            passage = int(pl._graph.label[int(2.5 / RES), int(3.5 / RES)])
            self.assertIn(passage, pl._route_paths_edges("auto"))
            self.assertIn("最速", pl._auto_note)
            self.assertIn("2通り", pl._auto_note)
            short = pl._routes["auto"].length

            pl.request_routes(json.dumps({"avoid": [[3.5, 2.5]]}))
            self._wait(pl)
            self.assertNotIn(passage, pl._route_paths_edges("auto"))
            self.assertGreater(pl._routes["auto"].length, short)
            self.assertEqual(pl._cfg.groups, {})                  # 経由点は書かない
        finally:
            pl.close()

    def test_stop_route_is_kept_and_published(self):
        # 停止点へ向かっている間に経路が作り直されても、停止経路から乗り換えない。
        # 停止経路は版を進めて GUI へ配る（凍結地図では map_seq が変わらないため）
        from raspi.auto.slam2d_route import Slam2dRoute
        pl = Slam2dRoute()
        try:
            pl.request_load("m")
            self._wait(pl)
            pl._switch.set_active("A", pl._routes["A"])
            pl.request_route("B", "gui")                  # 以前に B を押していた
            ver = pl._route_ver
            pl._stopping = "P"
            pl._set_active("stop", pl._routes["A"])
            self.assertGreater(pl._route_ver, ver)
            self.assertIn("stop", pl.snapshot().routes)

            pl._submit(graph=pl._graph)                   # 走行中の作り直し
            self._wait(pl)
            self.assertEqual(pl._switch.active_key, "stop")
            self.assertEqual(pl._switch.pending_key, "")
        finally:
            pl.close()

    def test_edit_during_the_first_build_is_not_lost(self):
        """★ 地図を読んだ直後（最初の計算中）に経由点を保存しても、経路に反映される。"""
        from raspi.auto.slam2d_route import Slam2dRoute
        pl = Slam2dRoute()
        try:
            pl.request_load("m")
            self.assertIsNotNone(pl._job)
            self.assertIsNone(pl._graph)
            cfg = json.loads(pl._cfg.to_json(with_traj=False))
            cfg["groups"]["C"] = [[2.0, 1.0, 0.0]]
            pl.request_routes(json.dumps(cfg))
            self._wait(pl)
            self.assertIn("C", pl._routes, pl._route_err)
        finally:
            pl.close()

    def test_broken_routes_file_does_not_raise(self):
        from raspi.auto import mapstore
        from raspi.auto.slam2d_route import Slam2dRoute
        mapstore.save_routes("m", '{"groups": [1]}')
        pl = Slam2dRoute()
        try:
            pl.request_load("m")                           # 例外にしない
            self.assertIn("経路の設定を読めない", pl._note)
            self._wait(pl)
            self.assertIn("auto", pl._routes)
        finally:
            pl.close()

    def test_stop_route_is_followed_from_its_start(self):
        """★ 1周回ってから止まる経路は、始めと終わりが同じ場所を通る。始点から追う。"""
        from concurrent.futures import Future

        from raspi.auto.slam2d_route import Slam2dRoute, _remaining
        th = np.linspace(0.0, 2.0 * np.pi * 1.15, 300)        # 1.15 周（終わりが始めに重なる）
        xy = 2.0 * np.column_stack([np.cos(th), np.sin(th)])
        seg = np.hypot(*np.diff(xy, axis=0).T)
        line = rl_mod.RaceLine(xy=xy, v=np.ones(300), kappa=np.full(300, 0.5),
                               length=float(seg.sum()), closed=False)
        # 走り出した直後の車は、終わりの区間の点（ここでは 270 番目）の上にも居る
        pose = (float(xy[270, 0]), float(xy[270, 1]), 0.0)
        self.assertLess(_remaining(line, pose), 2.0)           # 全体から探すと「残りわずか」
        self.assertGreater(_remaining(line, pose, 0), 12.0)    # 始点から追えば1周強
        pl = Slam2dRoute()
        try:
            pl.request_load("m")
            self._wait(pl)
            f: Future = Future()
            f.set_result((line, "注意"))
            pl._stopping, pl._stop_job = "P", f
            pl._poll()
            self.assertEqual(pl._switch.active_key, "stop")
            self.assertEqual(pl._hint, 0)
            self.assertEqual(pl._stop_error, "注意")
        finally:
            pl.close()

    def test_stop_route_is_retried_later(self):
        """停止経路を作れなくても、少し進んだらもう一度試す（毎周期は投げない）。"""
        from concurrent.futures import Future

        from raspi.auto.route_config import Stop
        from raspi.auto.slam2d_route import Slam2dRoute
        from raspi.msgs import VehicleState
        pl = Slam2dRoute()
        try:
            pl.request_load("m")
            self._wait(pl)
            pl._switch.set_active("A", pl._routes["A"])
            pl._hint = 0
            pl._cfg.mission = {"laps": 1, "then": "P"}
            pl._cfg.stops = {"P": Stop(6.0, 2.5)}
            pl.laps = 1
            pl._race_t = 10.0
            f: Future = Future()
            f.set_exception(RuntimeError("boom"))
            pl._stopping, pl._stop_job = "P", f
            pl._poll()
            p = {s.key: s.default for s in pl.params}
            pose = tuple(pl._routes["A"].xy[0]) + (0.0,)
            pl._maybe_finish(pose, VehicleState(speed=1.0), p)
            self.assertIsNone(pl._stop_job)
            pl._race_t = 11.5
            pl._maybe_finish(pose, VehicleState(speed=1.0), p)
            self.assertIsNotNone(pl._stop_job)
            line, _warn = pl._stop_job.result(timeout=30)
            self.assertFalse(line.closed)
        finally:
            pl.close()

    def test_build_without_any_route_still_saves_the_map(self):
        """★ 経路が1本も作れなくても、地図は保存して DONE へ進む（BUILD で固まらない）。"""
        from raspi.auto import mapstore
        from raspi.auto.slam2d_raceline import BUILD, DONE
        from raspi.auto.slam2d_route import Slam2dRoute
        from raspi.msgs import AutoState
        pl = Slam2dRoute()
        try:
            pl.phase = BUILD
            pl._graph = graph_of(oval())
            pl._route_err = {"auto": "スタートから戻ってくる周回ルートが無い"}
            p = {s.key: s.default for s in pl.params}
            st = pl._build(AutoState(), p)
            self.assertEqual(pl.phase, DONE)
            self.assertIn("周回ルートが無い", st.reason)
            self.assertTrue(pl._saved_map_name)
            saved = mapstore.load_map(pl._saved_map_name)
            self.assertIsNotNone(saved)
            self.assertEqual(len(saved.raceline_xy), 0)
            self.assertIn("経路が無い", pl._done(AutoState()).reason)
            # ラインの無い地図も、経路を自前で作るこの planner なら読める
            pl.request_load(pl._saved_map_name)
            self.assertEqual(pl._load_error, "")
            self.assertIsNone(pl.path)
        finally:
            pl.close()

    def test_race_start_is_initialised_even_if_a_route_was_chosen_in_locate(self):
        """★ LOCATE の間に切替ボタン・信号が来ても、走り出しの初期化を飛ばさない。"""
        from raspi.auto.slam2d_raceline import RACE
        from raspi.auto.slam2d_route import Slam2dRoute
        from raspi.msgs import AutoState, Scan, VehicleState
        pl = Slam2dRoute()
        try:
            pl.request_load("m")
            self._wait(pl)
            pl.request_route("B", "gui")                   # まだ走り出していない
            self.assertEqual(pl._switch.active_key, "B")
            pl._race_t, pl._lap_s, pl._prev_idx, pl.laps = 123.0, 7.0, 80, 3   # 前回の残り
            ver = pl._route_ver
            x, y = pl._routes["B"].xy[0]
            pl.slam.set_pose(float(x), float(y), 0.0)
            pl.phase = RACE
            pl._dt = 0.1
            st = AutoState(match_score=1.0)
            p = {s.key: s.default for s in pl.params}
            pl._race(st, Scan(dist=[3.0] * 360, sector_seen=[True] * 12),
                     VehicleState(speed=0.0), p, False)
            self.assertTrue(pl._race_started)
            self.assertEqual(pl._switch.active_key, "B")
            self.assertLess(pl._race_t, 1.0)
            self.assertEqual(pl.laps, 0)
            self.assertLess(abs(pl._lap_s), 1.0)
            self.assertGreater(pl._route_ver, ver)          # GUI へ今の経路を配り直す
        finally:
            pl.close()

    def test_stop_failure_is_not_resubmitted(self):
        from concurrent.futures import Future

        from raspi.auto.route_config import Stop
        from raspi.auto.slam2d_route import Slam2dRoute
        from raspi.msgs import VehicleState
        pl = Slam2dRoute()
        try:
            pl.request_load("m")
            self._wait(pl)
            pl._cfg.mission = {"laps": 1, "then": "P"}
            pl._cfg.stops = {"P": Stop(6.0, 2.5)}
            pl.laps = 1
            f: Future = Future()
            f.set_exception(RuntimeError("boom"))
            pl._stopping, pl._stop_job = "P", f
            pl._poll()
            self.assertEqual(pl._stopping, "")
            self.assertIn("boom", pl._stop_error)
            p = {s.key: s.default for s in pl.params}
            pl._maybe_finish((2.0, 1.0, 0.0), VehicleState(speed=1.0), p)
            self.assertIsNone(pl._stop_job)                # 毎周期投げ直さない
        finally:
            pl.close()

    def test_snapshot_is_reused_until_routes_change(self):
        from raspi.auto.slam2d_route import Slam2dRoute
        pl = Slam2dRoute()
        try:
            pl.request_load("m")
            self._wait(pl)
            m1 = pl.snapshot()
            m2 = pl.snapshot()
            self.assertIs(m2, m1)
            self.assertIs(m2.routes, m1.routes)           # 詰め直していない
            seq1 = m1.route_seq
            pl._set_active("A", pl._routes["A"])
            m3 = pl.snapshot()
            self.assertGreater(m3.route_seq, seq1)        # 版が進む（配り直される）
            self.assertEqual(m3.route_active, "A")
        finally:
            pl.close()

    def test_bad_routes_are_reported_not_raised(self):
        from raspi.auto.slam2d_route import Slam2dRoute
        pl = Slam2dRoute()
        try:
            pl.request_routes('{"groups": {"Z": []}}')
            self.assertIn("Z", pl._note)
        finally:
            pl.close()


class TestLapCount(unittest.TestCase):
    """`Slam2dRoute._count_lap`: 最寄り点の揺れで周回を数え間違えない。"""

    def setUp(self):
        from raspi.auto.slam2d_route import Slam2dRoute
        self.pl = Slam2dRoute()
        th = np.linspace(0, 2 * np.pi, 100, endpoint=False)
        xy = np.column_stack([np.cos(th), np.sin(th)])
        self.path = rl_mod.RaceLine(xy=xy, v=np.ones(100), kappa=np.ones(100),
                                    length=2 * np.pi)

    def tearDown(self):
        self.pl.close()

    def test_jitter_before_start_line_is_not_a_lap(self):
        # 終端の直前から走り出し、最寄り点が1つ戻ってから先頭へ回る
        for i in (97, 96, 97, 98, 99, 0, 1, 2):
            self.pl._count_lap(self.path, i)
        self.assertEqual(self.pl.laps, 0)

    def test_full_lap_is_counted_once(self):
        for i in list(range(0, 100, 3)) + [1, 4]:
            self.pl._count_lap(self.path, i)
        self.assertEqual(self.pl.laps, 1)


class TestObstacleFreeMask(unittest.TestCase):
    def test_cached_mask_gives_same_detection(self):
        from raspi.nav import obstacles as obs_mod
        from raspi.nav.deskew import Points
        g = occgrid_from_trinary(oval(), resolution=RES, origin=(0.0, 0.0), seq=0)
        ang = np.radians(np.arange(0, 360, 2.0))
        r = np.full(ang.size, 3.0)
        r[:8] = 0.5                                        # 前方の空きに物がある
        x, y = r * np.cos(ang), r * np.sin(ang)
        pts = Points(x=x, y=y, hit=np.ones(ang.size, dtype=bool), t_ref_ns=0, corrected=False)
        pose = (2.0, 1.0, 0.0)
        a = obs_mod.detect(g, pts, pose, wall_pad=0.1)
        b = obs_mod.detect(g, pts, pose, wall_pad=0.1, free=obs_mod.free_mask(g, 0.1))
        self.assertEqual(a, b)
        self.assertTrue(a)


class TestJoinAndRejoin(unittest.TestCase):
    """走り出しの減速と、外れたときの乗り直し（`slam2d_raceline` と `slam2d_route` 共通）。"""

    def setUp(self):
        from raspi.auto.slam2d_raceline import Slam2dRaceLine
        self.pl = Slam2dRaceLine()
        self.p = {"join_speed": 0.5, "max_cross": 0.5}
        self.path = _straight(0, 0)

    def _pp(self, cross, index=10):
        from types import SimpleNamespace
        return SimpleNamespace(cross_track=cross, index=index)

    def test_caps_speed_until_on_path(self):
        from raspi.msgs import AutoState
        st = AutoState(target_speed=2.0)
        self.pl._join_cap(st, self._pp(0.25), self.path, (1.0, 0.25, 0.0), self.p)
        self.assertEqual(st.target_speed, 0.5)
        st = AutoState(target_speed=2.0)
        self.pl._join_cap(st, self._pp(0.02), self.path, (1.0, 0.02, 0.0), self.p)
        self.assertEqual(st.target_speed, 2.0)              # 乗ったら抑えない
        self.assertTrue(self.pl._joined)

    def test_rejoins_after_stopping(self):
        from raspi.msgs import AutoState, VehicleState
        self.pl._joined = True
        self.pl._dt = 0.125                                # 8回でちょうど1秒（浮動小数の誤差なし）
        still = VehicleState(speed=0.0)
        for _ in range(7):
            self.assertTrue(self.pl._off_route(AutoState(), self._pp(0.6), still, self.p))
        self.assertTrue(self.pl._joined)                   # まだ1秒たっていない
        self.pl._off_route(AutoState(), self._pp(0.6), still, self.p)
        self.assertFalse(self.pl._joined)                  # 1秒止まったら乗り直す
        self.assertEqual(self.pl._rejoins, 1)
        # 乗り直し中は上限 1.5 倍（0.6m では止めない）
        self.assertFalse(self.pl._off_route(AutoState(), self._pp(0.6), still, self.p))


class TestRouteSelectBus(unittest.TestCase):
    """信号認識の受け口（`route/select`）が `signal` から planning_node まで通るか。"""

    def setUp(self):
        import os
        import tempfile
        self._tmp = tempfile.TemporaryDirectory()
        self._old = os.environ.get("SURGE_BUS_DIR")
        os.environ["SURGE_BUS_DIR"] = self._tmp.name

    def tearDown(self):
        import os
        if self._old is None:
            os.environ.pop("SURGE_BUS_DIR", None)
        else:
            os.environ["SURGE_BUS_DIR"] = self._old
        self._tmp.cleanup()

    def test_owner_and_round_trip(self):
        import time

        from raspi.bus import LATEST, Publisher, Subscriber
        from raspi.bus.zbus import endpoint_for_node, endpoints_for_topic
        from raspi.msgs.types import TOPIC_ROUTE_SELECT, RouteSelect

        self.assertEqual(endpoints_for_topic(TOPIC_ROUTE_SELECT), [endpoint_for_node("signal")])
        pub = Publisher("signal")
        sub = Subscriber({TOPIC_ROUTE_SELECT: LATEST})
        try:
            got = []
            end = time.monotonic() + 2.0
            while not got and time.monotonic() < end:
                pub.send(TOPIC_ROUTE_SELECT, RouteSelect(value="left", event_id=7))
                got = sub.poll(20)
            self.assertTrue(got)
            topic, msg = got[-1]
            self.assertEqual(topic, TOPIC_ROUTE_SELECT)
            self.assertEqual((msg.value, msg.event_id), ("left", 7))
        finally:
            sub.close()
            pub.close()

    def test_planning_node_applies_once_per_event(self):
        from types import SimpleNamespace

        from raspi.msgs.types import RouteSelect
        from raspi.nodes.planning_node import PlanningNode

        calls = []
        node = SimpleNamespace(
            _last_route_select=("", 0), quiet=True,
            planner=SimpleNamespace(request_signal=lambda v, s: calls.append((v, s))))
        for _ in range(3):                  # 同じ要求を繰り返し流されても1回
            PlanningNode._apply_route_select(node, RouteSelect(value="left", event_id=1))
        PlanningNode._apply_route_select(node, RouteSelect(value="left", event_id=2))
        PlanningNode._apply_route_select(node, RouteSelect(value="right", event_id=2))
        self.assertEqual([v for v, _ in calls], ["left", "left", "right"])


class TestObstacleResponse(unittest.TestCase):
    """障害物への反応（`Slam2dRaceLine._obstacle_response`、`slam2d_route` と共通）。"""

    def setUp(self):
        from raspi.auto.slam2d_raceline import Slam2dRaceLine
        from raspi.nav.obstacles import Obstacle
        self.pl = Slam2dRaceLine()
        self.pl._dt = 0.1
        self.p = {s.key: s.default for s in self.pl.params}
        self.hit = Obstacle(1.3, 0.0, 0.1, 5)
        th = np.linspace(0, 2 * np.pi, 200, endpoint=False)
        ring = 3.0 * np.column_stack([np.cos(th), np.sin(th)])
        self.closed = rl_mod.RaceLine(xy=ring, v=np.full(200, 1.5), kappa=np.full(200, 1 / 3),
                                      length=2 * np.pi * 3.0)

    def tearDown(self):
        self.pl.close()

    def _respond(self, path, hit, dist, *, speed, armed=True, target=1.5, **over):
        from raspi.msgs import AutoState, VehicleState
        st = AutoState(target_speed=target, ready=True)
        vs = VehicleState(speed=speed, armed=armed)
        return self.pl._obstacle_response(st, hit, dist, vs, {**self.p, **over}, path, 10), st

    def test_open_path_slows_for_an_obstacle(self):
        """★ 停止点へ向かう開いた経路でも、「避ける」が有効なら止まれる速度へ落とす。

        以前は開いた経路で減速も制動もしなかった（`obstacle_stop` が 0 のとき素通し）。
        """
        handled, st = self._respond(_straight(0, 0), self.hit, 1.3, speed=1.5, obstacle_avoid=1)
        self.assertTrue(handled)
        self.assertLess(st.target_speed, 1.5)
        self.assertFalse(st.brake)
        # 目の前まで来たら制動する。横へは避けず、Hybrid A* にも入らない
        for _ in range(30):
            handled, st = self._respond(_straight(0, 0), self.hit, 0.55, speed=0.0,
                                        obstacle_avoid=1)
        self.assertTrue(handled)
        self.assertTrue(st.brake)
        self.assertEqual(self.pl.phase, "EXPLORE")          # DETOUR へ移っていない

    def test_open_path_is_untouched_when_avoidance_is_off(self):
        handled, st = self._respond(_straight(0, 0), self.hit, 1.3, speed=1.5, obstacle_avoid=0)
        self.assertFalse(handled)
        self.assertEqual(st.target_speed, 1.5)

    def test_standing_still_before_arm_is_not_stuck(self):
        """★ engage 済みで ARM 前の静止を「進めない」と数えない（数えると ARM 前に DETOUR）。"""
        for _ in range(40):
            handled, _ = self._respond(self.closed, None, np.inf, speed=0.0, armed=False,
                                       obstacle_avoid=1)
        self.assertFalse(handled)
        self.assertEqual(self.pl._stuck_t, 0.0)
        self.assertEqual(self.pl._detour_tries, 0)

    def test_standing_still_while_armed_is_stuck(self):
        handled = False
        for _ in range(40):
            if handled:
                break
            handled, st = self._respond(self.closed, None, np.inf, speed=0.0, armed=True,
                                        obstacle_avoid=1)
        self.assertTrue(handled)
        self.assertEqual(self.pl.phase, "DETOUR")
        # numpy のスカラを `AutoState` へ流さない（バスのエンコーダが落ちた実績がある）
        tgt = self.pl._detour._target_in_park
        self.assertTrue(all(type(v) is float for v in tgt), tgt)


if __name__ == "__main__":
    unittest.main()
