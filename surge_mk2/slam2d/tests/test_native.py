"""SLAM の C 版（`core/_native.c`・`core/native.py`）が numpy 版と**ビット一致**するか。

`OccGrid.raycast`（追跡の「壁の向こうの点」の判定）と `_carve`（地図作成の「素通り」の数え）、
`SurfaceMap.associate`（ICP の対応付け）と総当たりの採点（`register.search_steps`・`localize`）、
面要素の法線の材料（`SurfaceMap.__init__`）。
どちらも結果が少しでもずれると、位置合わせに使う点・地図の空きが変わり、姿勢が別の値になる。
単体の一致に加え、地図作成 → 凍結 → 追跡を通しで回した姿勢が同じことを確かめる。
"""

import math
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

import numpy as np  # noqa: E402

from slam2d.core import grid as grid_mod  # noqa: E402
from slam2d.core import native  # noqa: E402
from slam2d.core import surfmap as surfmap_mod  # noqa: E402
from slam2d.core.localize import GlobalLocalizer  # noqa: E402
from slam2d.core.register import search_steps  # noqa: E402
from slam2d.core.surfmap import SurfaceMap  # noqa: E402
from slam2d.core.frontend import Frontend, FrontendConfig  # noqa: E402
from slam2d.core.grid import OccGrid, dilate  # noqa: E402
from slam2d.core.motion import ExternalTwistModel  # noqa: E402
from slam2d.core.types import Pose2D, Twist2D  # noqa: E402
from slam2d.tests.helpers import ROOM, make_raw_scan, make_room_points  # noqa: E402

NATIVE = native.lib is not None


def both(fn):
    """`fn()` を C 版と numpy 版で回した結果の組。"""
    saved = grid_mod.USE_NATIVE, surfmap_mod.USE_NATIVE
    try:
        grid_mod.USE_NATIVE = surfmap_mod.USE_NATIVE = True
        a = fn()
        grid_mod.USE_NATIVE = surfmap_mod.USE_NATIVE = False
        b = fn()
    finally:
        grid_mod.USE_NATIVE, surfmap_mod.USE_NATIVE = saved
    return a, b


def room_surfmap(g: OccGrid, **kw) -> SurfaceMap:
    mean = g.hit_mean()
    return SurfaceMap(g.wall_mask(), resolution=g.resolution, origin=g.origin,
                      mean_x=mean[0], mean_y=mean[1], known=g.seen > 0, **kw)


def room_grid(res: float = 0.05, origin=None) -> OccGrid:
    g = OccGrid(resolution=res, size_m=10.0, min_hits=2, min_seen=2, origin=origin)
    for (x, y, yaw) in [(1.0, 1.0, 0.0), (3.0, 1.0, 0.3), (5.0, 1.0, 0.6), (5.0, 3.0, 1.2),
                        (3.0, 3.0, 2.0), (1.0, 3.0, 2.8)]:
        g.integrate(make_room_points(x, y, yaw, segs=ROOM), Pose2D(x, y, yaw))
    return g


@unittest.skipUnless(NATIVE, "C コンパイラが無い（numpy 版で回る）")
class TestRaycast(unittest.TestCase):
    def test_matches_numpy(self):
        g = room_grid()
        self.assertTrue(g._native_cells(np.float32))    # C の経路に入る（numpy 版どうしの比較にしない）
        rng = np.random.default_rng(0)
        ang = rng.uniform(-math.pi, math.pi, 400)
        ox = rng.uniform(-1.0, 7.0, 400)            # 格子の外から撃つレイも入れる
        oy = rng.uniform(-1.0, 5.0, 400)
        mask = dilate(g.wall_mask(), 1)
        for kw in (dict(max_range=8.0), dict(max_range=3.0, fill=0),
                   dict(max_range=5.0, mask=mask, fill=0, step=g.resolution),
                   dict(max_range=0.01)):
            for o in ((2.5, 2.0), (ox, oy)):
                a, b = both(lambda: g.raycast(o[0], o[1], ang, **kw))
                self.assertEqual(a.dtype, b.dtype)
                self.assertTrue(np.array_equal(a, b), kw)

    def test_numpy_float64_origin_uses_numpy(self):
        """原点が numpy の float64 だと numpy は float64 で計算する。C 版（float32）は使わず同じ値を返す。"""
        g = room_grid(origin=(np.float64(-5.0), np.float64(-5.0)))
        self.assertFalse(g._native_cells(np.float32))
        ang = np.linspace(-math.pi, math.pi, 50)
        a, b = both(lambda: g.raycast(2.5, 2.0, ang, 8.0))
        self.assertTrue(np.array_equal(a, b))


@unittest.skipUnless(NATIVE, "C コンパイラが無い（numpy 版で回る）")
class TestCarve(unittest.TestCase):
    def test_integrate_matches_numpy(self):
        def run():
            g = OccGrid(resolution=0.025, size_m=8.0, min_hits=2, min_seen=2, grow=True,
                        grow_step_m=2.0)
            rng = np.random.default_rng(1)
            for _ in range(25):
                x, y, yaw = rng.uniform(0.5, 5.5), rng.uniform(0.5, 3.5), rng.uniform(-3, 3)
                g.integrate(make_room_points(x, y, yaw, segs=ROOM, noise_sigma=0.01, rng=rng),
                            Pose2D(x, y, yaw), origin=(0.1, -0.02))
            return g.misses.copy(), g.hits.copy(), g.origin
        (ma, ha, oa), (mb, hb, ob) = both(run)
        self.assertEqual(oa, ob)
        self.assertTrue(np.array_equal(ma, mb))
        self.assertTrue(np.array_equal(ha, hb))

    def test_counts_saturate(self):
        def run():
            g = room_grid()
            g.misses[:] = grid_mod._COUNT_MAX - 1
            g.integrate(make_room_points(3.0, 2.0, 0.0, segs=ROOM), Pose2D(3.0, 2.0, 0.0))
            g.integrate(make_room_points(3.0, 2.0, 0.0, segs=ROOM), Pose2D(3.0, 2.0, 0.0))
            return g.misses.copy()
        a, b = both(run)
        self.assertTrue(np.array_equal(a, b))
        self.assertEqual(int(a.max()), grid_mod._COUNT_MAX)


@unittest.skipUnless(NATIVE, "C コンパイラが無い（numpy 版で回る）")
class TestSurfaceMap(unittest.TestCase):
    def test_associate_matches_numpy(self):
        g = room_grid()
        rng = np.random.default_rng(3)
        for kw in (dict(), dict(max_dist=0.2, tangent_gate=0.01)):
            m = room_surfmap(g, **kw)
            self.assertIsNotNone(m._c_map())            # C の経路に入る
            wx = rng.uniform(-1.0, 7.0, (7, 300))       # 格子の外・未知・壁の近く・遠くを混ぜる
            wy = rng.uniform(-1.0, 5.0, (7, 300))
            a, b = both(lambda: m.associate(wx, wy))
            for fa, fb in zip(a, b):
                self.assertEqual(fa.dtype, fb.dtype)
                self.assertEqual(fa.shape, fb.shape)
                self.assertTrue(np.array_equal(fa, fb))
        # known=None（全部既知）の版
        m = SurfaceMap(g.wall_mask(), resolution=g.resolution, origin=g.origin)
        a, b = both(lambda: m.associate(wx[0], wy[0]))
        self.assertTrue(all(np.array_equal(fa, fb) for fa, fb in zip(a, b)))

    def test_search_scores_match_numpy(self):
        """総当たりの探索の得点（候補ごとの点の和）と、その結果の姿勢・2位が同じ。"""
        g = room_grid()
        m = room_surfmap(g)
        pts = make_room_points(3.1, 2.05, 0.08, segs=ROOM, noise_sigma=0.01,
                               rng=np.random.default_rng(4))

        def run():
            gen = search_steps(m, pts.x, pts.y, Pose2D(3.0, 2.0, 0.0), trans=0.3, rot=0.2,
                               trans_step=0.05, rot_step=0.05)
            try:
                while True:
                    next(gen)
            except StopIteration as stop:
                return stop.value
        a, b = both(run)
        self.assertEqual(a, b)

    def test_localizer_matches_numpy(self):
        g = room_grid()
        g.freeze()
        pts = make_room_points(2.4, 1.7, 0.6, segs=ROOM)

        def run():
            loc = GlobalLocalizer(g)
            while not loc.step(pts):
                pass
            return loc._by_angle.copy(), loc.result, loc.candidates()
        (sa, ra, ca), (sb, rb, cb) = both(run)
        self.assertTrue(np.array_equal(sa, sb))
        self.assertEqual((ra, ca), (rb, cb))

    def test_normals_match_numpy(self):
        """面要素の法線の材料（25 近傍の和、`normal_sums`）が numpy 版と同じ → 法線・平面の判定も同じ。"""
        g = room_grid(res=0.025)
        rng = np.random.default_rng(5)
        xs = rng.uniform(0.0, 6.0, 3000)
        ys = np.where(rng.random(3000) < 0.5, 0.0, 4.0) + 0.01 * rng.standard_normal(3000)
        for make in (lambda: room_surfmap(g),
                     lambda: room_surfmap(g, normal_radius=1, min_support=2),
                     lambda: SurfaceMap.from_points(xs, ys, center=(3.0, 2.0), radius=4.0,
                                                    resolution=0.025)):
            a, b = both(make)
            for name in ("_nx", "_ny", "_planar", "_support", "_qx", "_qy", "_nearest"):
                self.assertTrue(np.array_equal(getattr(a, name), getattr(b, name)), name)

    def test_pickle_drops_c_struct(self):
        import pickle
        m = room_surfmap(room_grid())
        m._c_map()
        m2 = pickle.loads(pickle.dumps(m))
        self.assertNotIn("_cmap", m2.__dict__)
        wx, wy = np.array([2.0, 5.9]), np.array([2.0, 3.9])
        self.assertTrue(all(np.array_equal(x, y) for x, y in zip(m.associate(wx, wy),
                                                                 m2.associate(wx, wy))))


@unittest.skipUnless(NATIVE, "C コンパイラが無い（numpy 版で回る）")
class TestFrontendIdentical(unittest.TestCase):
    def test_mapping_then_tracking_gives_identical_poses(self):
        """地図作成（`_carve`）→ 凍結 → 追跡（`_not_behind_wall` の `raycast`）の姿勢が全周期で同じ。"""
        def run():
            motion = ExternalTwistModel(lambda: Twist2D(0.2, 0.0, 0.05))
            fe = Frontend(OccGrid(resolution=0.025, size_m=14.0, origin=(-2.0, -2.0)), motion,
                          FrontendConfig(max_range=8.0, kf_dist=0.05))
            poses = []
            rng = np.random.default_rng(2)
            for i in range(40):
                if i == 20:
                    fe.grid.freeze()
                fe.update(make_raw_scan(3.0 + 0.02 * i, 2.0, 0.005 * i, segs=ROOM, max_range=8.0,
                                        noise_sigma=0.005, rng=rng), 0.1)
                poses.append((fe.pose.x, fe.pose.y, fe.pose.yaw, fe.behind_ratio))
            return poses, fe.grid.misses.copy()
        (pa, ma), (pb, mb) = both(run)
        self.assertEqual(pa, pb)
        self.assertTrue(np.array_equal(ma, mb))


if __name__ == "__main__":
    unittest.main()
