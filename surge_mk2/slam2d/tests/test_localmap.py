"""`core/localmap.py`（直近のキーフレームだけで作る位置合わせ用の地図）のテスト。"""

import math
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

import numpy as np  # noqa: E402

from slam2d.core.localmap import LocalMap, LocalMapConfig  # noqa: E402
from slam2d.core.types import Pose2D  # noqa: E402


def _ring(n=1200, r=3.0):   # セルが連続するくらい密に（疎だと裏付けが無く対応に使われない）
    a = np.linspace(0.0, 2 * math.pi, n, endpoint=False)
    return r * np.cos(a), r * np.sin(a)


class TestWindow(unittest.TestCase):
    def test_old_keyframes_fall_out_of_the_window(self):
        lm = LocalMap(LocalMapConfig(window_m=1.0, min_keyframes=2))
        px, py = _ring()
        for i in range(30):
            lm.add(Pose2D(0.1 * i, 0.0, 0.0), px, py)
        # 窓は走行距離1m ぶん（0.1m間隔なので10〜12枚）
        self.assertLessEqual(len(lm), 13)
        self.assertGreaterEqual(len(lm), 2)

    def test_min_keyframes_is_respected_while_standing_still(self):
        lm = LocalMap(LocalMapConfig(window_m=0.2, min_keyframes=3))
        px, py = _ring()
        for _ in range(10):
            lm.add(Pose2D(0.0, 0.0, 0.0), px, py)
        self.assertGreaterEqual(len(lm), 3)

    def test_field_is_none_until_the_first_keyframe(self):
        lm = LocalMap()
        self.assertIsNone(lm.field(Pose2D(0.0, 0.0, 0.0)))

    def test_field_contains_the_added_points(self):
        lm = LocalMap()
        px, py = _ring()
        lm.add(Pose2D(1.0, 2.0, 0.5), px, py)
        f = lm.field(Pose2D(1.0, 2.0, 0.5))
        self.assertIsNotNone(f)
        self.assertFalse(f.empty)
        # 車体から3mの円なので、世界座標(1+3, 2)付近に壁がある
        a = f.associate(np.array([4.0]), np.array([2.0]))
        self.assertTrue(bool(a.valid[0]))
        self.assertLess(float(a.dist[0]), 0.05)

    def test_reset_to_rebuilds_from_corrected_poses(self):
        lm = LocalMap()
        px, py = _ring()
        for i in range(5):
            lm.add(Pose2D(0.1 * i, 0.0, 0.0), px, py)
        lm.reset_to([(Pose2D(10.0 + 0.1 * i, 0.0, 0.0), px, py) for i in range(5)])
        f = lm.field(Pose2D(10.4, 0.0, 0.0))
        a = f.associate(np.array([13.0]), np.array([0.0]))
        self.assertTrue(bool(a.valid[0]))
        # 元の位置には何も無い
        b = f.associate(np.array([3.0]), np.array([0.0]))
        self.assertFalse(bool(b.valid[0]))


if __name__ == "__main__":
    unittest.main()
