"""`ml_cam_e2e/charts.py` の座標計算（Tk を使わない純粋関数）のテスト。"""

import math
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))  # ml_cam_e2e/

from charts import (  # noqa: E402
    arrow_endpoint,
    decimation_step,
    gap_threshold,
    nearest_index,
    polylines,
    runs,
    time_to_x,
    x_to_time,
)


class TestArrowEndpoint(unittest.TestCase):
    def test_zero_steer_points_straight_up(self):
        x, y = arrow_endpoint((100, 200), 0.0, 50)
        self.assertAlmostEqual(x, 100)
        self.assertAlmostEqual(y, 150)

    def test_positive_steer_leans_left_on_screen(self):
        """舵は反時計回り（左）が正。画面では x が小さい方へ傾く。"""
        x, _ = arrow_endpoint((100, 200), 0.2, 50)
        self.assertLess(x, 100)
        x, _ = arrow_endpoint((100, 200), -0.2, 50)
        self.assertGreater(x, 100)


class TestTimeAxis(unittest.TestCase):
    def test_round_trip(self):
        for t in (10.0, 12.5, 20.0):
            x = time_to_x(t, 10.0, 20.0, 40.0, 440.0)
            self.assertAlmostEqual(x_to_time(x, 10.0, 20.0, 40.0, 440.0), t)

    def test_click_outside_the_plot_is_clamped(self):
        self.assertEqual(x_to_time(-50, 0.0, 10.0, 40.0, 440.0), 0.0)
        self.assertEqual(x_to_time(9999, 0.0, 10.0, 40.0, 440.0), 10.0)

    def test_zero_span_does_not_divide_by_zero(self):
        self.assertEqual(time_to_x(5.0, 5.0, 5.0, 40.0, 440.0), 40.0)


class TestNearestIndex(unittest.TestCase):
    def test_picks_the_closer_neighbour(self):
        times = [0.0, 1.0, 3.0]
        self.assertEqual(nearest_index(times, 0.4), 0)
        self.assertEqual(nearest_index(times, 0.6), 1)
        self.assertEqual(nearest_index(times, 99.0), 2)
        self.assertEqual(nearest_index(times, -5.0), 0)

    def test_empty_is_zero(self):
        self.assertEqual(nearest_index([], 1.0), 0)


class TestPolylines(unittest.TestCase):
    BOX = (0.0, 0.0, 100.0, 50.0)

    def test_maps_value_range_onto_the_box_with_y_flipped(self):
        (line,) = polylines([0.0, 10.0], [-1.0, 1.0], t0=0.0, t1=10.0, v_lo=-1.0, v_hi=1.0,
                            box=self.BOX)
        self.assertEqual(line, [0.0, 50.0, 100.0, 0.0])

    def test_breaks_the_line_at_a_time_gap(self):
        """記録の途切れを直線で結ばない。"""
        times = [0.0, 0.1, 0.2, 5.0, 5.1]
        lines = polylines(times, [0.0] * 5, t0=0.0, t1=5.1, v_lo=-1.0, v_hi=1.0,
                          box=self.BOX, gap_s=0.5)
        self.assertEqual([len(line) // 2 for line in lines], [3, 2])

    def test_nan_breaks_the_line_and_is_not_drawn(self):
        lines = polylines([0.0, 1.0, 2.0, 3.0, 4.0], [0.0, 0.0, math.nan, 0.0, 0.0],
                          t0=0.0, t1=4.0, v_lo=-1.0, v_hi=1.0, box=self.BOX)
        self.assertEqual([len(line) // 2 for line in lines], [2, 2])

    def test_values_are_clipped_to_the_range(self):
        (line,) = polylines([0.0, 1.0], [9.0, -9.0], t0=0.0, t1=1.0, v_lo=-1.0, v_hi=1.0,
                            box=self.BOX)
        self.assertEqual(line[1], 0.0)
        self.assertEqual(line[3], 50.0)

    def test_step_thins_the_points(self):
        times = [float(i) for i in range(100)]
        (line,) = polylines(times, [0.0] * 100, t0=0.0, t1=99.0, v_lo=-1.0, v_hi=1.0,
                            box=self.BOX, step=10)
        self.assertEqual(len(line) // 2, 10)


class TestHelpers(unittest.TestCase):
    def test_decimation_keeps_about_two_points_per_pixel(self):
        self.assertEqual(decimation_step(100, 500), 1)
        self.assertEqual(decimation_step(10_000, 500), 10)

    def test_gap_threshold_follows_the_frame_interval(self):
        self.assertAlmostEqual(gap_threshold([i * 0.2 for i in range(50)]), 0.8)
        self.assertAlmostEqual(gap_threshold([i / 30 for i in range(50)]), 0.5)   # 下限
        self.assertEqual(gap_threshold([0.0, 1.0]), math.inf)

    def test_runs(self):
        self.assertEqual(runs([False, True, True, False, True]), [(1, 2), (4, 4)])
        self.assertEqual(runs([True, True]), [(0, 1)])
        self.assertEqual(runs([]), [])


if __name__ == "__main__":
    unittest.main()
