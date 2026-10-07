"""`raspi/nav/avoid.py`（障害物を横へ避ける一時経路）のテスト。

半径5mの円周コース（道幅を変えられる）に障害物を置いて確かめる:

- 避ける経路は窓の外では元の経路と同じ（添字もそのまま）
- 車体外形が障害物と壁から `clear` 以上離れる
- 足す曲率が「曲がれる曲率の余り」に収まる
- 道が狭くて通れなければ `None`、近すぎて横へずれる距離が無くても `None`
- 窓の速度を抑え、手前で減速し切れるように下げる
"""

import math
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

import numpy as np  # noqa: E402

from raspi.nav import avoid as av  # noqa: E402
from raspi.nav.obstacles import Obstacle  # noqa: E402
from raspi.nav.raceline import RaceLine, _BodyCheck, curvature  # noqa: E402

R = 5.0
RES = 0.02
FOOT = ((0.30, 0.09), (0.30, -0.09), (-0.07, -0.09), (-0.07, 0.09))


class _RingGrid:
    """半径 `R ± width/2` に壁がある円周コースの格子（`_BodyCheck` が読む面だけ）。"""

    def __init__(self, width: float) -> None:
        self.resolution = RES
        self.origin = (-R - 2.0, -R - 2.0)
        n = int((2 * R + 4.0) / RES)
        c = (np.arange(n) + 0.5) * RES + self.origin[0]
        xx, yy = np.meshgrid(c, c)
        rr = np.hypot(xx, yy)
        self._wall = (np.abs(rr - (R + width / 2)) < RES) | (np.abs(rr - (R - width / 2)) < RES)

    def wall_mask(self):
        return self._wall


def _ring_path(v=2.0) -> RaceLine:
    n = int(2 * math.pi * R / 0.1)
    a = np.arange(n) * 2 * math.pi / n
    xy = np.column_stack([R * np.cos(a), R * np.sin(a)])      # 反時計回り
    return RaceLine(xy=xy, v=np.full(n, v), kappa=curvature(xy), length=2 * math.pi * R)


def _cfg(**kw) -> av.AvoidConfig:
    base = dict(half_width=0.09, front=0.30, rear=0.07, kappa_max=math.tan(0.524) / 0.23,
                footprint=FOOT, margin=0.10, clear=0.05, len_min=0.8)
    base.update(kw)
    return av.AvoidConfig(**base)


def _obstacle_on_path(path: RaceLine, index: int, r=0.08, lateral=0.0) -> Obstacle:
    x, y = path.xy[index]
    nx, ny = x / R, y / R                                      # 外向き（進行方向の右）
    return Obstacle(x - lateral * nx, y - lateral * ny, r, 10)


class TestPlanOffset(unittest.TestCase):
    def setUp(self):
        self.path = _ring_path()
        self.body = _BodyCheck(_RingGrid(1.4), FOOT, closed=False)

    def test_avoids_obstacle_with_clearance_and_keeps_the_rest(self):
        obs = _obstacle_on_path(self.path, 40)                 # 4m 先
        plan = av.plan_offset(self.path, 0, obs, [obs], self.body, _cfg(), v_now=2.0)
        self.assertIsNotNone(plan)
        need = obs.r + 0.09 + 0.10
        self.assertAlmostEqual(abs(plan.offset), need, delta=1e-6)
        self.assertGreaterEqual(plan.clearance, 0.05)
        out = np.setdiff1d(np.arange(len(self.path)), plan.window)
        np.testing.assert_allclose(plan.path.xy[out], self.path.xy[out])
        self.assertEqual(len(plan.path), len(self.path))
        # 窓は車の前から始まり、障害物を含む
        self.assertGreater(int(plan.window[0]), 0)
        self.assertIn(40, plan.window.tolist())
        # 足した曲率が曲がれる曲率の余りに収まる
        extra = np.abs(plan.path.kappa - self.path.kappa).max()
        self.assertLess(extra, 0.6 * _cfg().kappa_max)

    def test_prefers_the_side_needing_less_shift(self):
        # 障害物は経路の左（円の内側）へ 5cm ずれている。左を通るには +0.32m、
        # 右を通るには −0.22m ずらす必要があり、少ない右を選ぶ
        obs = _obstacle_on_path(self.path, 40, lateral=0.05)
        plan = av.plan_offset(self.path, 0, obs, [obs], self.body, _cfg(), v_now=1.0)
        self.assertIsNotNone(plan)
        self.assertAlmostEqual(plan.offset, -(obs.r + 0.09 + 0.10 - 0.05), delta=0.01)

    def test_narrow_road_has_no_plan(self):
        body = _BodyCheck(_RingGrid(0.45), FOOT, closed=False)
        obs = _obstacle_on_path(self.path, 40)
        self.assertIsNone(av.plan_offset(self.path, 0, obs, [obs], body, _cfg(), v_now=1.0))

    def test_too_close_has_no_plan_until_allowed_to_turn_harder(self):
        # 1.4m 先。進入に使える距離は 1.4−0.08(半径)−0.30(前端) ≈ 1.02m
        obs = _obstacle_on_path(self.path, 14)
        # 曲がれる曲率の6割では約1.09m要るので足りない
        self.assertIsNone(av.plan_offset(self.path, 0, obs, [obs], self.body, _cfg(), v_now=0.0))
        # 止まってから9割まで使えば約0.87mで足りる（`_obstacle_response` の引き直し）
        plan = av.plan_offset(self.path, 0, obs, [obs], self.body,
                              _cfg(len_min=0.5, kappa_frac=0.9), v_now=0.0)
        self.assertIsNotNone(plan)
        # 1.1m 先では9割でも足りない（→ Hybrid A* へ）
        obs = _obstacle_on_path(self.path, 11)
        self.assertIsNone(av.plan_offset(self.path, 0, obs, [obs], self.body,
                                         _cfg(len_min=0.5, kappa_frac=0.9), v_now=0.0))


class TestCapSpeed(unittest.TestCase):
    def test_window_is_capped_and_approach_decelerates(self):
        path = _ring_path(v=2.0)
        window = np.arange(50, 70)
        out = av.cap_speed(path, window, 1.0, a_brake=2.0)
        self.assertTrue(np.all(out.v[window] <= 1.0))
        # 手前は窓の入口で 1.0 になるよう減速: v² ≤ 1 + 2·a·d
        d = (50 - np.arange(30, 50)) * (2 * math.pi * R / len(path))
        self.assertTrue(np.all(out.v[30:50] <= np.sqrt(1.0 + 2 * 2.0 * d) + 1e-9))
        self.assertEqual(out.v[0], 2.0)

    def test_exit_accelerates_within_the_limit(self):
        """`a_accel` を渡すと、窓の出口でも1点で元の速度へ跳ばない。"""
        path = _ring_path(v=2.0)
        window = np.arange(50, 70)
        step = 2 * math.pi * R / len(path)
        self.assertEqual(av.cap_speed(path, window, 1.0, a_brake=2.0).v[70], 2.0)
        out = av.cap_speed(path, window, 1.0, a_brake=2.0, a_accel=1.5)
        d = (np.arange(70, 100) - 69) * step
        self.assertTrue(np.all(out.v[70:100] <= np.sqrt(1.0 + 2 * 1.5 * d) + 1e-9))
        self.assertLess(out.v[70], 1.2)
        self.assertEqual(out.v[200], 2.0)


if __name__ == "__main__":
    unittest.main()
