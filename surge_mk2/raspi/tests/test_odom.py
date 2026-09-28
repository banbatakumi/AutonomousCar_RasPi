"""推測航法（`rec/odom.py`）のテスト。

    python3 -m unittest raspi.tests.test_odom
"""

from __future__ import annotations

import math
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from raspi.msgs import VehicleState  # noqa: E402
from raspi.rec.odom import DeadReckoner  # noqa: E402

MS = 1_000_000
DT_NS = 10 * MS                            # 100Hz


def run(dr: DeadReckoner, n: int, *, v: float, w: float, t0: int = 0,
        stopped: bool | None = None, dt_ns: int = DT_NS):
    p = None
    for i in range(n):
        st = VehicleState(t_capture=t0 + i * dt_ns, speed=v, yaw_rate=w,
                          stopped=(v == 0.0) if stopped is None else stopped)
        p = dr.update(st)
    return p


class TestDeadReckoner(unittest.TestCase):
    def test_straight_line(self):
        dr = DeadReckoner()
        p = run(dr, 101, v=1.0, w=0.0)              # 100 周期 = 1.0 秒
        self.assertAlmostEqual(p.x, 1.0, places=6)
        self.assertAlmostEqual(p.y, 0.0, places=6)
        self.assertAlmostEqual(p.dist, 1.0, places=6)

    def test_full_circle_returns_to_origin(self):
        """一定の v, ω で1周すると原点に戻り、途中の最大 y は直径 2v/ω になる。"""
        v, w = 1.0, 1.0
        n = round(2 * math.pi / w / 0.01) + 1
        dr = DeadReckoner()
        ys = []
        for i in range(n):
            p = dr.update(VehicleState(t_capture=i * DT_NS, speed=v, yaw_rate=w,
                                       stopped=False))
            ys.append(p.y)
        self.assertLess(math.hypot(p.x, p.y), 0.02)
        self.assertAlmostEqual(max(ys), 2 * v / w, delta=0.01)
        self.assertAlmostEqual(p.yaw_unwrapped, 2 * math.pi, delta=0.02)
        self.assertLessEqual(abs(p.yaw), math.pi)

    def test_bias_is_learned_while_stopped(self):
        """静止中にゼロ点ずれを学び、走り出した後に方位がドリフトしない。"""
        bias = 0.02
        dr = DeadReckoner()
        p = run(dr, 1000, v=0.0, w=bias)            # 10 秒止まる
        self.assertAlmostEqual(p.gyro_bias, bias, delta=5e-4)   # 時定数2秒×5 で残り0.7%
        self.assertEqual(p.yaw_unwrapped, 0.0)      # 静止中は回らない
        self.assertGreater(p.bias_learn_s, 9.0)
        p = run(dr, 1000, v=1.0, w=bias, t0=1000 * DT_NS)   # 10 秒まっすぐ走る
        self.assertLess(abs(p.yaw_unwrapped), 0.005)
        self.assertLess(abs(p.y), 0.05)

    def test_without_learning_the_bias_drifts(self):
        """比較のため: 較正しないで走り出すとゼロ点ずれがそのまま方位に乗る。"""
        dr = DeadReckoner()
        p = run(dr, 1001, v=1.0, w=0.02)
        self.assertAlmostEqual(p.yaw_unwrapped, 0.2, delta=1e-3)

    def test_large_rate_while_stopped_is_not_learned(self):
        """手で回した等の大きな読みはゼロ点ずれに取り込まず、回頭として追う。"""
        dr = DeadReckoner()
        p = run(dr, 100, v=0.0, w=1.0)
        self.assertEqual(p.gyro_bias, 0.0)
        self.assertAlmostEqual(p.yaw_unwrapped, 0.99, delta=0.01)

    def test_gap_is_not_integrated(self):
        dr = DeadReckoner()
        dr.update(VehicleState(t_capture=0, speed=1.0, stopped=False))
        p = dr.update(VehicleState(t_capture=500 * MS, speed=1.0, stopped=False))
        self.assertEqual(p.x, 0.0)
        self.assertEqual(p.gaps, 1)
        p = dr.update(VehicleState(t_capture=510 * MS, speed=1.0, stopped=False))
        self.assertAlmostEqual(p.x, 0.01, places=6)

    def test_time_going_backwards_is_ignored(self):
        dr = DeadReckoner()
        dr.update(VehicleState(t_capture=100 * MS, speed=1.0, stopped=False))
        p = dr.update(VehicleState(t_capture=90 * MS, speed=1.0, stopped=False))
        self.assertEqual(p.x, 0.0)

    def test_reverse_moves_backwards_but_counts_distance(self):
        dr = DeadReckoner()
        p = run(dr, 101, v=-0.5, w=0.0)
        self.assertAlmostEqual(p.x, -0.5, places=6)
        self.assertAlmostEqual(p.dist, 0.5, places=6)

    def test_not_learned_right_after_stopping(self):
        """止まった直後（まだ転がって曲がっている）の読みはゼロ点ずれに取り込まない。"""
        dr = DeadReckoner()
        p = run(dr, 30, v=0.0, w=0.1)               # 0.3 秒 < STILL_SETTLE_S
        self.assertEqual(p.gyro_bias, 0.0)

    def test_flickering_stopped_flag_still_learns(self):
        """`stopped` が時々外れても（静止付近のばたつき）学習は続く。"""
        dr = DeadReckoner()
        p = None
        for i in range(500):
            p = dr.update(VehicleState(t_capture=i * DT_NS, speed=0.0, yaw_rate=0.02,
                                       stopped=(i % 7 != 0)))
        self.assertAlmostEqual(p.gyro_bias, 0.02, delta=2e-3)

    def test_stopped_flag_freezes_position(self):
        """`stopped` の間は `speed` にノイズがあっても位置を動かさない。"""
        dr = DeadReckoner()
        p = run(dr, 100, v=0.01, w=0.0, stopped=True)
        self.assertEqual(p.x, 0.0)


if __name__ == "__main__":
    unittest.main()
