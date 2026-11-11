"""`ml_e2e/labels.py`（経路点のラベルの幾何）のテスト。"""

import math
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))  # surge_mk2/

import numpy as np  # noqa: E402

from ml_e2e import labels as L  # noqa: E402


def _circle(r: float = 2.0, n: int = 200) -> np.ndarray:
    """原点中心・反時計回りの円。"""
    a = np.linspace(0.0, 2.0 * math.pi, n, endpoint=False)
    return np.stack([r * np.cos(a), r * np.sin(a)], axis=1)


class TestPoseGeometry(unittest.TestCase):
    def test_advance_pose_uses_motion_in_the_odom_heading(self):
        """推測航法の原点・向きが地図と違っても、動いた量は車体の向きで掛け直される。"""
        # 推測航法では +y へ 1m 進んだ（推測航法の向きは 90°）。地図では車は 0° を向いている
        pose = L.advance_pose((5.0, 5.0, 0.0), (0.0, 0.0, math.pi / 2), (0.0, 1.0, math.pi / 2 + 0.1))
        self.assertAlmostEqual(pose[0], 6.0)
        self.assertAlmostEqual(pose[1], 5.0)
        self.assertAlmostEqual(pose[2], 0.1)

    def test_to_body_puts_a_point_ahead_on_plus_x_and_left_on_plus_y(self):
        pose = (1.0, 1.0, math.pi / 2)                      # +y を向いている
        body = L.to_body(np.array([[1.0, 3.0], [0.0, 1.0]]), pose)
        np.testing.assert_allclose(body, [[2.0, 0.0], [0.0, 1.0]], atol=1e-9)


class TestPathFromLine(unittest.TestCase):
    def setUp(self):
        xy = _circle()
        self.line = L.resample_line(xy, np.full(len(xy), 2.5))

    def test_closed_line_is_detected_and_resampled_evenly(self):
        xy, v, _head, closed = self.line
        self.assertTrue(closed)
        step = np.hypot(*np.diff(xy, axis=0).T)
        self.assertLess(step.max() - step.min(), 1e-3)
        np.testing.assert_allclose(v, 2.5)

    def test_points_are_spaced_ds_along_the_line_and_curve_left(self):
        # 円の上（角度 0）に居て接線方向（+y）を向いている
        lab = L.path_from_line(self.line, (2.0, 0.0, math.pi / 2), k=8, ds=0.25)
        self.assertIsNotNone(lab)
        self.assertTrue(lab.valid.all())
        self.assertAlmostEqual(lab.cross, 0.0, places=2)
        # k 番目は弧長 (k+1)·ds 先。円の上なので原点からの弦の長さで確かめる
        for k in range(8):
            chord = 2.0 * 2.0 * math.sin((k + 1) * 0.25 / 2.0 / 2.0)
            self.assertAlmostEqual(float(np.hypot(*lab.xy[k])), chord, places=1)
        self.assertTrue((lab.xy[:, 0] > 0).all())           # 前
        self.assertTrue((np.diff(lab.xy[:, 1]) > 0).all())  # 左へ曲がっていく
        np.testing.assert_allclose(lab.v, 2.5)

    def test_offset_car_gets_a_path_back_to_the_line(self):
        """線の外側 0.3m に居ても、ラベルは線の上の点（＝戻る経路）。"""
        lab = L.path_from_line(self.line, (2.3, 0.0, math.pi / 2), k=8, ds=0.25)
        self.assertAlmostEqual(lab.cross, -0.3, places=2)   # 線の右（外側）
        self.assertGreater(lab.xy[0, 1], 0.25)              # 最初の点から左（線の側）にある

    def test_heading_picks_the_matching_direction(self):
        """向きが逆（時計回りに走っている）なら、向きの合う点が無い。"""
        self.assertIsNone(L.path_from_line(self.line, (2.0, 0.0, -math.pi / 2), k=8, ds=0.25))

    def test_too_far_from_line_gives_none(self):
        self.assertIsNone(L.path_from_line(self.line, (4.0, 0.0, math.pi / 2), k=8, ds=0.25, max_cross=1.0))

    def test_open_line_marks_points_past_the_end_invalid(self):
        xy = np.stack([np.linspace(0.0, 1.0, 21), np.zeros(21)], axis=1)
        line = L.resample_line(xy, None)
        self.assertFalse(line[3])
        lab = L.path_from_line(line, (0.5, 0.0, 0.0), k=4, ds=0.25)
        self.assertEqual(lab.valid.tolist(), [True, False, False, False])


class TestPathFromFuture(unittest.TestCase):
    def _straight(self, v: float = 1.0, n: int = 500):
        t = (np.arange(n) * 10_000_000).astype(np.int64)        # 100Hz
        x = v * t / 1e9
        return x, np.zeros(n), np.zeros(n), x.copy(), t, t, np.full(n, v)

    def test_straight_run_gives_points_straight_ahead(self):
        lab = L.path_from_future(*self._straight(), t=1_000_000_000, k=6, ds=0.25)
        np.testing.assert_allclose(lab.xy[:, 0], 0.25 * np.arange(1, 7), atol=1e-3)
        np.testing.assert_allclose(lab.xy[:, 1], 0.0, atol=1e-6)
        np.testing.assert_allclose(lab.v, 1.0, atol=1e-6)
        self.assertTrue(lab.valid.all())

    def test_points_beyond_the_recording_are_invalid(self):
        # 記録は 5s・5m。4.5s の時点から先は 0.5m しか無い
        lab = L.path_from_future(*self._straight(), t=4_500_000_000, k=6, ds=0.25)
        self.assertEqual(lab.valid.tolist(), [True, False, False, False, False, False])

    def test_outside_the_recording_gives_none(self):
        self.assertIsNone(L.path_from_future(*self._straight(), t=9_000_000_000, k=6, ds=0.25))


class TestPursuitSteer(unittest.TestCase):
    def test_left_path_gives_positive_steer_and_straight_gives_zero(self):
        straight = np.stack([np.linspace(0.25, 2.0, 8), np.zeros(8)], axis=1)
        self.assertAlmostEqual(L.pursuit_steer(straight, 1.0, 0.23, 0.47), 0.0)
        left = np.stack([np.linspace(0.25, 2.0, 8), np.linspace(0.02, 0.8, 8)], axis=1)
        self.assertGreater(L.pursuit_steer(left, 1.0, 0.23, 0.47), 0.05)
        self.assertLess(L.pursuit_steer(left * [1, -1], 1.0, 0.23, 0.47), -0.05)


if __name__ == "__main__":
    unittest.main()
