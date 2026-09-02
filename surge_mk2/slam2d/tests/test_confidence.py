"""`core/confidence.py`（固有値ベースの動的信頼度）のテスト。

`estimate_information`が実際に「観測できる方向・できない方向」を区別できる
こと、`fuse_gaussians`が情報フィルタとして正しく振る舞うことを確認する。
"""

import math
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

import numpy as np  # noqa: E402

from slam2d.core.confidence import ConfidenceConfig, estimate_information, fuse_gaussians  # noqa: E402
from slam2d.core.grid import OccGrid  # noqa: E402
from slam2d.core.types import Pose2D  # noqa: E402
from slam2d.tests.helpers import make_room_points  # noqa: E402

#: 四方を壁で囲まれた部屋。どの方向にも観測情報がある
_ROOM = [(0.0, 0.0, 6.0, 0.0), (6.0, 0.0, 6.0, 4.0),
         (6.0, 4.0, 0.0, 4.0), (0.0, 4.0, 0.0, 0.0)]
#: 水平な壁1枚だけの通路もどき。壁に沿った方向(x)は観測できない
_WALL = [(-500.0, 0.0, 500.0, 0.0)]


def _build_grid(segs, poses) -> OccGrid:
    g = OccGrid(resolution=0.05, size_m=12.0, origin=(-1.0, -1.0))
    for x, y, yaw in poses:
        g.integrate(make_room_points(x, y, yaw, segs=segs, max_range=8.0),
                   Pose2D(x, y, yaw))
    return g


class TestEstimateInformationRoom(unittest.TestCase):
    def test_both_directions_are_observable(self):
        g = _build_grid(_ROOM, [(3.0, 2.0, 0.0)] * 4)
        pts = make_room_points(3.0, 2.0, 0.0, segs=_ROOM, max_range=8.0)
        info = estimate_information(g, pts, Pose2D(3.0, 2.0, 0.0))
        eigvals = np.linalg.eigvalsh(info[:2, :2])
        # 部屋の中なので、どちらの固有方向にもそれなりの拘束がある
        self.assertGreater(float(eigvals.min()), 1.0)


class TestEstimateInformationCorridor(unittest.TestCase):
    def test_along_wall_direction_has_lower_confidence(self):
        """壁に沿った方向(x)は、壁に垂直な方向(y)より情報量が小さいはず。

        地図はレイキャストの`max_range`より十分広く（複数のx位置から観測して
        壁を広い範囲に焼く）作る——地図が壁の観測範囲に対して狭すぎると、
        点群をx方向にずらしたときに地図の端からはみ出る点が増えて「x方向にも
        観測情報がある」という偽の信号が乗ることを実測で確認したため。
        """
        g = OccGrid(resolution=0.05, size_m=40.0, origin=(-20.0, -5.0))
        for x in (-5.0, -2.0, 0.0, 2.0, 5.0):
            g.integrate(make_room_points(x, 1.0, 0.0, segs=_WALL, max_range=15.0),
                       Pose2D(x, 1.0, 0.0))
        pts = make_room_points(0.0, 1.0, 0.0, segs=_WALL, max_range=15.0)
        info = estimate_information(g, pts, Pose2D(0.0, 1.0, 0.0))
        # x方向(壁沿い)の情報 info[0,0] は y方向(壁に垂直) info[1,1] よりずっと小さい
        self.assertLess(info[0, 0], info[1, 1] * 0.5)


class TestFuseGaussians(unittest.TestCase):
    def test_zero_observation_information_keeps_prediction(self):
        pred = Pose2D(1.0, 2.0, 0.3)
        obs = Pose2D(5.0, 5.0, 1.0)  # 全然違う値でも、情報0なら無視される
        info_pred = np.eye(3) * 10.0
        info_obs = np.zeros((3, 3))
        fused = fuse_gaussians(pred, info_pred, obs, info_obs)
        self.assertAlmostEqual(fused.x, pred.x, places=6)
        self.assertAlmostEqual(fused.y, pred.y, places=6)
        self.assertAlmostEqual(fused.yaw, pred.yaw, places=6)

    def test_zero_prediction_information_uses_observation(self):
        pred = Pose2D(1.0, 2.0, 0.3)
        obs = Pose2D(5.0, 5.0, 1.0)
        info_pred = np.zeros((3, 3))
        info_obs = np.eye(3) * 10.0
        fused = fuse_gaussians(pred, info_pred, obs, info_obs)
        self.assertAlmostEqual(fused.x, obs.x, places=6)
        self.assertAlmostEqual(fused.y, obs.y, places=6)

    def test_equal_information_averages(self):
        pred = Pose2D(0.0, 0.0, 0.0)
        obs = Pose2D(2.0, 0.0, 0.0)
        info = np.eye(3) * 5.0
        fused = fuse_gaussians(pred, info, obs, info)
        self.assertAlmostEqual(fused.x, 1.0, places=6)

    def test_both_zero_falls_back_to_prediction(self):
        pred = Pose2D(1.0, 2.0, 0.3)
        obs = Pose2D(5.0, 5.0, 1.0)
        zero = np.zeros((3, 3))
        fused = fuse_gaussians(pred, zero, obs, zero)
        self.assertAlmostEqual(fused.x, pred.x, places=6)

    def test_anisotropic_fusion_weighs_by_direction(self):
        """x方向だけ観測情報が強いなら、xは観測に寄り、yは予測に留まる。"""
        pred = Pose2D(0.0, 0.0, 0.0)
        obs = Pose2D(1.0, 1.0, 0.0)
        info_pred = np.diag([1.0, 1.0, 1.0])
        info_obs = np.diag([100.0, 0.0, 0.0])  # x方向だけ強い観測
        fused = fuse_gaussians(pred, info_pred, obs, info_obs)
        self.assertAlmostEqual(fused.x, obs.x, delta=0.05)   # xはほぼ観測通り
        self.assertAlmostEqual(fused.y, pred.y, places=6)     # yは予測のまま(情報0)

    def test_yaw_wraps_correctly_across_pi_boundary(self):
        """yawが ±π を跨いでも壊れない（例: pred=+3.0, obs=-3.0 は実際には近い）。"""
        pred = Pose2D(0.0, 0.0, 3.0)
        obs = Pose2D(0.0, 0.0, -3.0)
        info = np.eye(3)
        fused = fuse_gaussians(pred, info, obs, info)
        # 3.0 と -3.0 (実質 2π-3.0=3.28) の中間は概ね pi 付近になるはず
        self.assertAlmostEqual(abs(fused.yaw), math.pi, delta=0.2)


if __name__ == "__main__":
    unittest.main()
