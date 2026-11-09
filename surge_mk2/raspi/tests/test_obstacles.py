"""`nav/obstacles.py`（凍結地図に無いものを見つける）のテスト。

縛る性質:

- **真正面の障害物が、点群の継ぎ目（方位 0°）で割れない。** 他に候補点があっても1つのまま、
  遠くて点が少なくても検出される
- 壁の近くの除外は**距離**で効く（格子に対して斜めの壁でも指定の幅だけ除外する）
- 壁寄りの障害物は、`wall_pad` の外に1点でもあれば 10cm 壁寄りの点まで数える。
  `wall_pad` の外に点が無い塊（壁の点のずれ）は数えない
"""

import math
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

import numpy as np  # noqa: E402

from raspi.nav import obstacles as obs_mod  # noqa: E402
from raspi.nav.deskew import Points  # noqa: E402
from slam2d.core.grid import OccGrid  # noqa: E402


def _open_grid() -> OccGrid:
    """全面が空きと分かっている凍結地図（8m 四方、車は中央）。"""
    g = OccGrid(resolution=0.025, size_m=8.0)
    g.misses[:] = 5
    g.frozen = True
    return g


def _scan(*circles: tuple[float, float, float]) -> Points:
    """車体座標の円柱 `(x, y, r)` を、方位 0〜359° の順（実機の点群と同じ）で見た点群。"""
    xs, ys, hit = [], [], []
    for deg in range(360):
        a = math.radians(deg)
        dx, dy = math.cos(a), math.sin(a)
        best = math.inf
        for cx, cy, r in circles:
            b = dx * cx + dy * cy
            disc = b * b - (cx * cx + cy * cy - r * r)
            if disc >= 0.0 and b > 0.0:
                best = min(best, b - math.sqrt(disc))
        ok = math.isfinite(best)
        t = best if ok else 3.9
        xs.append(t * dx)
        ys.append(t * dy)
        hit.append(ok)
    return Points(np.array(xs), np.array(ys), np.array(hit), 1, False)


class TestSeam(unittest.TestCase):
    POSE = (0.0, 0.0, 0.0)

    def _near(self, out, x, y):
        return [o for o in out if math.hypot(o.x - x, o.y - y) < 0.3]

    def test_obstacle_dead_ahead_alone(self):
        out = obs_mod.detect(_open_grid(), _scan((1.5, 0.0, 0.10)), self.POSE)
        self.assertEqual(len(out), 1)
        self.assertEqual(out[0].n, 7)

    def test_obstacle_dead_ahead_is_not_split_by_another_candidate(self):
        """左にもう1つあると、正面の点（357〜359° と 0〜3°）の間に別の候補が挟まる。"""
        alone = obs_mod.detect(_open_grid(), _scan((1.5, 0.0, 0.10)), self.POSE)[0]
        out = obs_mod.detect(_open_grid(), _scan((1.5, 0.0, 0.10), (0.0, 1.5, 0.10)), self.POSE)
        front = self._near(out, 1.5, 0.0)
        self.assertEqual(len(front), 1)
        self.assertEqual(front[0].n, alone.n)
        self.assertAlmostEqual(front[0].y, 0.0, delta=0.01)
        self.assertAlmostEqual(front[0].r, alone.r, delta=0.005)
        self.assertEqual(len(self._near(out, 0.0, 1.5)), 1)

    def test_far_obstacle_dead_ahead_is_still_detected(self):
        """3m 先は3点しか当たらない。割れると `min_points` を切って検出されなかった。"""
        out = obs_mod.detect(_open_grid(), _scan((3.0, 0.0, 0.10), (0.0, 1.5, 0.10)), self.POSE)
        self.assertEqual(len(self._near(out, 3.0, 0.0)), 1)


class TestWallPad(unittest.TestCase):
    def test_diagonal_wall_is_padded_by_distance(self):
        """45° の壁から 0.15m 未満のセルは候補にしない（ひし形の膨張だと 10.6cm しか除外しない）。"""
        g = _open_grid()
        i = np.arange(60, 260)
        g.hits[i, i] = 20                        # 対角の壁
        g.hits[i, i + 1] = 20
        free = obs_mod.free_mask(g, 0.15)
        rows, cols = np.nonzero(free[80:240, 80:240])
        # 壁（直線 row = col）までの距離
        d = np.abs(rows - cols) / math.sqrt(2.0) * g.resolution
        self.assertGreaterEqual(float(d.min()), 0.15 - g.resolution)

    def test_axis_aligned_wall_is_unchanged(self):
        g = _open_grid()
        g.hits[100, 50:250] = 20
        free = obs_mod.free_mask(g, 0.15)
        self.assertFalse(free[94:107, 60:240].any())      # ±6 セル（0.15m）は除外
        self.assertTrue(free[93, 60:240].all())
        self.assertTrue(free[107, 60:240].all())


class TestNearBand(unittest.TestCase):
    """実機（道幅 約0.6m）: 壁から 25cm の障害物が `wall_pad=0.25` で見つからず当たった。"""

    POSE = (0.0, 0.0, 0.0)
    PAD = 0.25

    def _grid(self) -> OccGrid:
        g = _open_grid()
        col, row = g.to_cell(np.array([0.0]), np.array([-0.26]))
        g.hits[int(row[0]), :] = 20                 # 車の右 26cm に、進行方向と平行な壁
        return g

    def _detect(self, pts, *, near: bool):
        g = self._grid()
        return obs_mod.detect(g, pts, self.POSE, wall_pad=self.PAD,
                              free=obs_mod.free_mask(g, self.PAD),
                              near=obs_mod.free_mask(g, obs_mod.near_pad(self.PAD)) if near else None)

    def test_obstacle_beside_wall_is_counted_with_near_band(self):
        pts = _scan((2.0, 0.0, 0.06))               # 壁から 20〜32cm にまたがる
        self.assertEqual(self._detect(pts, near=False), [])
        out = self._detect(pts, near=True)
        self.assertEqual(len(out), 1)
        self.assertGreaterEqual(out[0].n, 3)

    def test_cluster_without_a_point_outside_wall_pad_is_ignored(self):
        pts = _scan((2.0, -0.07, 0.05))             # 全点が壁から 25cm 未満（壁の点のずれに相当）
        self.assertEqual(self._detect(pts, near=True), [])

    def test_near_pad_never_exceeds_wall_pad(self):
        self.assertAlmostEqual(obs_mod.near_pad(0.25), 0.15)
        self.assertAlmostEqual(obs_mod.near_pad(0.15), 0.10)
        self.assertAlmostEqual(obs_mod.near_pad(0.05), 0.05)


class TestVelocityTracker(unittest.TestCase):
    """動く物の速度を出す。静止物の重心のぶれを速度と取り違えない。"""

    def _run(self, track):
        tr = obs_mod.VelocityTracker()
        out = []
        for k in range(10):
            x, y = track(k * 0.1)
            out = tr.update(k * 0.1, [obs_mod.Obstacle(x, y, 0.1, 5)])
        return out[0]

    def test_constant_velocity_is_recovered(self):
        o = self._run(lambda t: (1.0 + 0.8 * t, 2.0 - 0.3 * t))
        self.assertAlmostEqual(o.vx, 0.8, places=2)
        self.assertAlmostEqual(o.vy, -0.3, places=2)

    def test_static_jitter_is_not_a_velocity(self):
        rng = np.random.default_rng(0)
        o = self._run(lambda t: (1.0 + rng.normal(0, 0.03), 2.0 + rng.normal(0, 0.03)))
        self.assertEqual(o.speed, 0.0)

    def test_swept_path_covers_the_travel(self):
        o = obs_mod.Obstacle(0.0, 0.0, 0.1, 5, None, 1.0, 0.0)
        trail = obs_mod.swept(o, 2.0, 0.05)
        self.assertAlmostEqual(trail[-1].x, 2.0)
        self.assertLessEqual(max(b.x - a.x for a, b in zip(trail, trail[1:])), 0.05 + 1e-9)
        self.assertEqual(len(obs_mod.swept(o._replace(vx=0.0), 2.0, 0.05)), 1)


if __name__ == "__main__":
    unittest.main()


class TestBehindWall(unittest.TestCase):
    """`behind`（`Frontend.behind_wall`）が「壁の向こう」と言った候補は捨てる。LiDAR の光は壁を通らないので、
    低い壁の上を越えて見えた奥か壁での反射（木の仕切り壁の向こうの点がヘアピンの空きに障害物として出た）。"""

    POSE = (0.0, 0.0, 0.0)

    def test_candidates_behind_a_wall_are_dropped(self):
        pts = _scan((1.5, 0.0, 0.10), (0.0, 1.5, 0.10))
        seen = []

        def behind(x, y):
            seen.append(len(x))
            return y > 1.0                                  # 左の物は壁の向こうだったとする

        out = obs_mod.detect(_open_grid(), pts, self.POSE, behind=behind)
        self.assertEqual(len(out), 1)
        self.assertAlmostEqual(out[0].x, 1.5, delta=0.15)
        # 候補（地図の空きに落ちた点）だけを渡す。全点を撃たない
        self.assertEqual(len(seen), 1)
        self.assertLess(seen[0], 40)

    def test_all_behind_returns_nothing(self):
        out = obs_mod.detect(_open_grid(), _scan((1.5, 0.0, 0.10)), self.POSE,
                             behind=lambda x, y: np.ones(len(x), dtype=bool))
        self.assertEqual(out, [])
