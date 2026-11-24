"""シムの他車（`sim/moving.py`）が格子に跡を残さないこと。"""

from __future__ import annotations

import unittest

import numpy as np

from sim.moving import Mover, update_all

RES = 0.05
ORIGIN = (-1.0, -1.0)


def _line() -> np.ndarray:
    """原点を通る x 方向の長い閉じた線（往復）。"""
    xs = np.arange(0.0, 6.0, 0.05)
    return np.vstack([np.stack([xs, np.zeros_like(xs)], 1), np.stack([xs[::-1], np.full_like(xs, 0.001)], 1)])


class TestMovers(unittest.TestCase):
    def _movers(self) -> list[Mover]:
        # 遅い車に速い車が後ろから追いつき、重なって抜けていく（他車どうしは避けない）
        ms = [Mover(s0=1.0, speed=0.5, d=0.0, r=0.09, length=0.37),
              Mover(s0=0.0, speed=1.5, d=0.0, r=0.09, length=0.37)]
        for m in ms:
            m.bind(_line(), (0.0, 0.0))
        return ms

    def test_overlapping_movers_leave_no_ghost(self):
        """★ 2 台の範囲が重なっても、消した後の格子は元のまま（幻の障害物が残らない）。"""
        grid = np.zeros((60, 200), dtype=bool)
        ms = self._movers()
        for k in range(40):
            update_all(ms, grid, ORIGIN, RES, 0.05 * k)
        for m in reversed(ms):
            m.erase(grid)
        self.assertEqual(int(grid.sum()), 0)

    def test_one_by_one_update_left_ghosts(self):
        """1 台ずつ消して描く（以前の bench）と跡が残る——上のテストが不具合を捉えている確かめ。"""
        grid = np.zeros((60, 200), dtype=bool)
        ms = self._movers()
        for k in range(40):
            for m in ms:
                m.update(grid, ORIGIN, RES, 0.05 * k)
        for m in reversed(ms):
            m.erase(grid)
        self.assertGreater(int(grid.sum()), 0)

    def test_profile_mover_follows_the_line_speed(self):
        """`profile` の他車は線の速度 × 倍率で進む（遅い区間で遅く、`speed` が上限）。"""
        line = _line()
        v = np.where(line[:, 0] < 3.0, 1.0, 3.0)          # 前半 1m/s・後半 3m/s
        v[len(v) // 2:] = 1.0                              # 戻りは 1m/s
        m = Mover(s0=0.0, speed=0.0, d=0.0, r=0.09, profile=0.5)
        m.bind(line, (0.0, 0.0), v)
        for k in range(11):
            m.pose_at(0.1 * k)
        self.assertAlmostEqual(m._travelled(1.0), 0.5, places=2)     # 0.5m/s × 1s
        m2 = Mover(s0=3.5, speed=1.0, d=0.0, r=0.09, profile=0.5)    # 3m/s の区間で 1.5 → 上限 1.0
        m2.bind(line, (0.0, 0.0), v)
        for k in range(11):
            m2.pose_at(0.1 * k)
        self.assertAlmostEqual(m2._travelled(1.0), 1.0, places=2)
        with self.assertRaises(ValueError):
            Mover(s0=0.0, speed=0.0, d=0.0, r=0.09, profile=0.5).bind(line, (0.0, 0.0))

    def test_rebind_continues_from_the_current_position(self):
        """経路の形が替わったら、今の位置のまま新しい線へ乗せ替える（跳ばない・新しい線の上を進む）。"""
        for m in (Mover(s0=1.0, speed=1.0, d=0.0, r=0.09),
                  Mover(s0=1.0, speed=0.0, d=0.0, r=0.09, profile=0.5)):
            line = _line()
            m.bind(line, (0.0, 0.0), np.full(len(line), 2.0) if m.profile else None)
            m.pose_at(0.0)
            p0 = m.pose_at(1.0)
            # 新しい線: y を 0.02m ずらした同じ形
            m.rebind(line + np.array([0.0, 0.02]), 1.0, np.full(len(line), 2.0) if m.profile else None)
            p1 = m.pose_at(1.0)
            self.assertLess(abs(p1[0] - p0[0]), 0.06)
            self.assertAlmostEqual(p1[1], 0.02, places=3)
            p2 = m.pose_at(1.5)
            self.assertAlmostEqual(p2[0] - p1[0], 0.5, delta=0.06)

    def test_walls_are_kept(self):
        """壁のセルは他車が通っても消えない。"""
        grid = np.zeros((60, 200), dtype=bool)
        grid[20, :] = True                     # y = 0 の行（線の上）に壁
        before = grid.copy()
        ms = self._movers()
        for k in range(40):
            update_all(ms, grid, ORIGIN, RES, 0.05 * k)
        for m in reversed(ms):
            m.erase(grid)
        np.testing.assert_array_equal(grid, before)


if __name__ == "__main__":
    unittest.main()
