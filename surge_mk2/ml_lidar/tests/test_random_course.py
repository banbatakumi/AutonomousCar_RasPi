"""`sim/random_course.py` のテスト。生成したコースが壊れていないことだけを確認する
（コース品質そのものは学習結果で答え合わせするので、ここでは配管レベルの検証に留める）。
"""

import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))  # surge_mk2/

import numpy as np  # noqa: E402

from sim.random_course import generate_random_course  # noqa: E402


class TestGenerateRandomCourse(unittest.TestCase):
    def test_loop_is_closed(self):
        rng = np.random.default_rng(0)
        c = generate_random_course(rng)
        gap = np.hypot(c.centerline[-1, 0] - c.centerline[0, 0],
                       c.centerline[-1, 1] - c.centerline[0, 1])
        self.assertLess(gap, c.width * 0.5)

    def test_start_is_not_occupied(self):
        rng = np.random.default_rng(1)
        c = generate_random_course(rng)
        self.assertFalse(c.occupied(c.start[0], c.start[1]))

    def test_raycast_and_collides_do_not_raise(self):
        rng = np.random.default_rng(2)
        c = generate_random_course(rng)
        angles = np.linspace(-np.pi, np.pi, 360, endpoint=False)
        d = c.raycast(c.start[0], c.start[1], angles, 5.0)
        self.assertEqual(d.shape, (360,))
        body = c.body_samples([[0.30, 0.09], [0.30, -0.09], [-0.07, -0.09], [-0.07, 0.09]])
        self.assertFalse(c.collides(c.start[0], c.start[1], c.start[2], body))

    def test_different_seeds_give_different_courses(self):
        c1 = generate_random_course(np.random.default_rng(10), name="a")
        c2 = generate_random_course(np.random.default_rng(11), name="b")
        different = (c1.centerline.shape != c2.centerline.shape
                    or not np.allclose(c1.centerline, c2.centerline))
        self.assertTrue(different)


if __name__ == "__main__":
    unittest.main()
