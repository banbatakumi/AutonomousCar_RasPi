"""`core/frontend.py`（1周期処理パイプライン）の基本動作テスト。

oval周回での回転ドリフト比較（Phase 4の主目的）は`test_frontend_oval.py`。
"""

import math
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from slam2d.core.frontend import Frontend, FrontendConfig  # noqa: E402
from slam2d.core.grid import OccGrid  # noqa: E402
from slam2d.core.motion import ConstantVelocityModel, ExternalTwistModel  # noqa: E402
from slam2d.core.types import Twist2D  # noqa: E402
from slam2d.tests.helpers import ROOM, make_raw_scan  # noqa: E402


def _make_frontend(motion=None, **config_kwargs) -> Frontend:
    grid = OccGrid(resolution=0.05, size_m=12.0, origin=(-1.0, -1.0))
    if motion is None:
        motion = ConstantVelocityModel()
    return Frontend(grid, motion, FrontendConfig(**config_kwargs))


class TestFrontendStationary(unittest.TestCase):
    def test_stationary_pose_does_not_drift(self):
        """静止したまま同じ点群を何度も入れても、姿勢はほぼ動かない。"""
        fe = _make_frontend()
        for _ in range(10):
            raw = make_raw_scan(3.0, 2.0, 0.0, segs=ROOM, max_range=8.0)
            u = fe.update(raw, 0.1)
        self.assertAlmostEqual(fe.pose.x, 0.0, delta=0.05)
        self.assertAlmostEqual(fe.pose.y, 0.0, delta=0.05)
        self.assertFalse(u.lost)


class TestFrontendKeyframes(unittest.TestCase):
    def test_keyframes_are_recorded_when_moving(self):
        # `ConstantVelocityModel`は観測（スキャンマッチ）のフィードバックでしか
        # 速度を学習しない。地図がまだ空の最初は観測情報もゼロなので、
        # 「動いている」と一度も分からず永遠に(0,0,0)から動けない
        # （鶏と卵）——実測で確認したFrontendの原理的な制約。このテストは
        # 実際のセンサ入力を想定して`ExternalTwistModel`（既知速度0.2m/s）を使う
        motion = ExternalTwistModel(lambda: Twist2D(0.2, 0.0, 0.0))
        fe = _make_frontend(motion=motion, kf_dist=0.02)
        for i in range(20):
            raw = make_raw_scan(3.0 + 0.02 * i, 2.0, 0.0, segs=ROOM, max_range=8.0)
            fe.update(raw, 0.1)
        self.assertGreater(len(fe.keyframes), 3)
        self.assertGreater(len(fe.trajectory), 3)

    def test_no_keyframes_when_stationary(self):
        fe = _make_frontend()
        for _ in range(10):
            raw = make_raw_scan(3.0, 2.0, 0.0, segs=ROOM, max_range=8.0)
            fe.update(raw, 0.1)
        # 最初の1枚だけ焼かれ、以降は「動いていない」ので焼かれない
        self.assertEqual(len(fe.keyframes), 1)


class TestFrontendLostDetection(unittest.TestCase):
    def test_far_from_map_is_lost(self):
        """地図に無い場所（別の部屋の形）に来ると見失う。"""
        motion = ExternalTwistModel(lambda: Twist2D(0.1, 0.0, 0.0))
        fe = _make_frontend(motion=motion, min_score=0.35, kf_dist=0.02)
        for i in range(15):
            raw = make_raw_scan(3.0 + 0.01 * i, 2.0, 0.0, segs=ROOM, max_range=8.0)
            fe.update(raw, 0.1)
        self.assertGreater(len(fe.keyframes), 1)
        # 全く別の壁配置（対応の付かない点群）を入れる
        weird = [(0.0, 0.0, 1.0, 3.0), (1.0, 3.0, -2.0, 1.0)]
        raw = make_raw_scan(3.0, 2.0, 0.0, segs=weird, max_range=8.0)
        u = fe.update(raw, 0.1)
        self.assertTrue(u.lost)

    def test_lost_scan_does_not_grow_map(self):
        motion = ExternalTwistModel(lambda: Twist2D(0.1, 0.0, 0.0))
        fe = _make_frontend(motion=motion, kf_dist=0.02)
        for i in range(15):
            raw = make_raw_scan(3.0 + 0.01 * i, 2.0, 0.0, segs=ROOM, max_range=8.0)
            fe.update(raw, 0.1)
        before = fe.grid.hits.copy()
        weird = [(0.0, 0.0, 1.0, 3.0)]
        raw = make_raw_scan(3.0, 2.0, 0.0, segs=weird, max_range=8.0)
        fe.update(raw, 0.1)
        import numpy as np
        self.assertTrue(np.array_equal(before, fe.grid.hits))


class TestFrontendAnisotropicToggle(unittest.TestCase):
    def test_both_modes_run_without_error(self):
        for anisotropic in (True, False):
            motion = ExternalTwistModel(lambda: Twist2D(0.2, 0.0, 0.1))
            fe = _make_frontend(motion=motion, anisotropic=anisotropic, kf_dist=0.02)
            for i in range(15):
                raw = make_raw_scan(3.0 + 0.02 * i, 2.0, 0.01 * i, segs=ROOM, max_range=8.0)
                u = fe.update(raw, 0.1)
            self.assertFalse(u.lost)


if __name__ == "__main__":
    unittest.main()
