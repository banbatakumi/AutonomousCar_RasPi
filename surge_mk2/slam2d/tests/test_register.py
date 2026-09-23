"""`core/register.py`（距離最小化による位置合わせ）のテスト。

縛りたい性質は3つ:

1. **既知のずれを正しく戻せる**（並進・回頭とも）
2. **通路では進行方向を勝手に決めない**（観測できない方向は事前分布が残る）
3. **外れ値（動く物・未知の壁）に引きずられない**
"""

import math
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

import numpy as np  # noqa: E402

from slam2d.core.register import RegisterConfig, register, search  # noqa: E402
from slam2d.core.surfmap import SurfaceMap  # noqa: E402
from slam2d.core.types import Pose2D, wrap_angle  # noqa: E402
from slam2d.tests.helpers import ROOM, ray_segments  # noqa: E402

CFG = RegisterConfig()


def _scan(x, y, yaw, segs=ROOM, n=360, max_range=8.0, noise=0.0, rng=None):
    """姿勢`(x,y,yaw)`から`segs`を見た点群（車体座標）。"""
    ang = np.linspace(0.0, 2 * math.pi, n, endpoint=False)
    r = np.array([ray_segments(x, y, yaw + a, segs, max_range) for a in ang])
    ok = r > 0.0
    if noise and rng is not None:
        r = r + rng.normal(0.0, noise, r.shape)
    return r[ok] * np.cos(ang[ok]), r[ok] * np.sin(ang[ok])


def _map_from(poses, segs=ROOM, resolution=0.025, noise=0.0, rng=None):
    xs, ys = [], []
    for (x, y, yaw) in poses:
        px, py = _scan(x, y, yaw, segs=segs, noise=noise, rng=rng)
        c, s = math.cos(yaw), math.sin(yaw)
        xs.append(x + c * px - s * py)
        ys.append(y + s * px + c * py)
    cx = float(np.mean([p[0] for p in poses]))
    cy = float(np.mean([p[1] for p in poses]))
    return SurfaceMap.from_points(np.concatenate(xs), np.concatenate(ys),
                                  center=(cx, cy), radius=9.0, resolution=resolution)


class TestRegisterRecoversKnownOffset(unittest.TestCase):
    def setUp(self):
        self.truth = (3.0, 2.0, 0.2)
        self.map = _map_from([(3.0, 2.0, 0.2)])
        self.px, self.py = _scan(*self.truth)

    def test_translation(self):
        guess = Pose2D(self.truth[0] + 0.15, self.truth[1] - 0.10, self.truth[2])
        r = register(self.map, self.px, self.py, guess)
        self.assertAlmostEqual(r.pose.x, self.truth[0], delta=0.01)
        self.assertAlmostEqual(r.pose.y, self.truth[1], delta=0.01)
        self.assertGreater(r.inlier, 0.9)
        self.assertLess(r.rms, 0.02)

    def test_rotation(self):
        guess = Pose2D(self.truth[0], self.truth[1], self.truth[2] + math.radians(4.0))
        r = register(self.map, self.px, self.py, guess)
        self.assertAlmostEqual(math.degrees(wrap_angle(r.pose.yaw - self.truth[2])), 0.0,
                               delta=0.3)

    def test_information_is_positive_definite(self):
        r = register(self.map, self.px, self.py, Pose2D(*self.truth))
        eig = np.linalg.eigvalsh(r.info)
        self.assertGreater(float(eig.min()), 0.0)


class TestCorridorKeepsPrior(unittest.TestCase):
    """平行な壁だけの通路では、進行方向は点群から決まらない。"""

    def setUp(self):
        self.corridor = [(-10.0, 0.5, 10.0, 0.5), (-10.0, -0.5, 10.0, -0.5)]
        self.map = _map_from([(0.0, 0.0, 0.0)], segs=self.corridor)
        self.px, self.py = _scan(0.0, 0.0, 0.0, segs=self.corridor)

    def test_along_corridor_stays_at_prior(self):
        prior = Pose2D(0.30, 0.0, 0.0)       # 進行方向に30cmずれた予測
        info = np.diag([1 / 0.05 ** 2, 1 / 0.05 ** 2, 1 / math.radians(2.0) ** 2])
        r = register(self.map, self.px, self.py, prior, prior=prior, prior_info=info)
        # 進行方向（x）は予測のまま、横方向（y）は点群が0へ引き戻す
        self.assertAlmostEqual(r.pose.x, 0.30, delta=0.05)
        self.assertAlmostEqual(r.pose.y, 0.0, delta=0.01)

    def test_lateral_error_is_corrected_even_with_prior(self):
        prior = Pose2D(0.0, 0.20, 0.0)
        info = np.diag([1 / 0.05 ** 2, 1 / 0.05 ** 2, 1 / math.radians(2.0) ** 2])
        r = register(self.map, self.px, self.py, prior, prior=prior, prior_info=info)
        self.assertAlmostEqual(r.pose.y, 0.0, delta=0.02)

    def test_information_is_anisotropic(self):
        r = register(self.map, self.px, self.py, Pose2D(0.0, 0.0, 0.0))
        # 横方向(y)の情報は進行方向(x)より桁違いに大きい
        self.assertGreater(r.info[1, 1], 50.0 * max(r.info[0, 0], 1e-9))


class TestRobustness(unittest.TestCase):
    def test_moving_obstacle_does_not_drag_the_pose(self):
        truth = (3.0, 2.0, 0.0)
        smap = _map_from([truth])
        px, py = _scan(*truth)
        # 点群の2割を「目の前を横切る物体」に差し替える
        n = px.size // 5
        px = np.concatenate([px[:-n], np.full(n, 0.6)])
        py = np.concatenate([py[:-n], np.linspace(-0.3, 0.3, n)])
        r = register(smap, px, py, Pose2D(*truth))
        self.assertAlmostEqual(r.pose.x, truth[0], delta=0.02)
        self.assertAlmostEqual(r.pose.y, truth[1], delta=0.02)

    def test_noise_does_not_bias_the_solution(self):
        rng = np.random.default_rng(7)
        truth = (3.0, 2.0, 0.1)
        smap = _map_from([truth], noise=0.01, rng=rng)
        errs = []
        for _ in range(10):
            px, py = _scan(*truth, noise=0.01, rng=rng)
            r = register(smap, px, py, Pose2D(truth[0] + 0.02, truth[1], truth[2]))
            errs.append((r.pose.x - truth[0], r.pose.y - truth[1]))
        e = np.asarray(errs)
        self.assertLess(abs(float(e[:, 0].mean())), 0.01)
        self.assertLess(abs(float(e[:, 1].mean())), 0.01)


class TestSearch(unittest.TestCase):
    def test_finds_a_coarse_offset(self):
        truth = (3.0, 2.0, 0.0)
        smap = _map_from([truth])
        px, py = _scan(*truth)
        center = Pose2D(truth[0] + 0.5, truth[1] - 0.4, truth[2] + math.radians(10.0))
        sr = search(smap, px, py, center, trans=0.8, rot=math.radians(15.0),
                    trans_step=0.1, rot_step=math.radians(2.0), sigma=0.08)
        self.assertLess(math.hypot(sr.pose.x - truth[0], sr.pose.y - truth[1]), 0.12)
        self.assertLess(abs(math.degrees(wrap_angle(sr.pose.yaw - truth[2]))), 3.0)
        self.assertGreater(sr.score, 0.5)

    def test_second_best_is_reported_for_ambiguity(self):
        """対称な形では、離れた候補にも同じくらいの得点が立つ。"""
        square = [(0.0, 0.0, 4.0, 0.0), (4.0, 0.0, 4.0, 4.0),
                  (4.0, 4.0, 0.0, 4.0), (0.0, 4.0, 0.0, 0.0)]
        smap = _map_from([(2.0, 2.0, 0.0)], segs=square)
        px, py = _scan(2.0, 2.0, 0.0, segs=square)
        sr = search(smap, px, py, Pose2D(2.0, 2.0, 0.0), trans=1.0, rot=math.radians(95.0),
                    trans_step=0.1, rot_step=math.radians(5.0), sigma=0.08)
        # 90°回した姿勢でも同じ形に見えるので、2位が1位に肉薄する
        self.assertGreater(sr.second, 0.7 * sr.score)


if __name__ == "__main__":
    unittest.main()
