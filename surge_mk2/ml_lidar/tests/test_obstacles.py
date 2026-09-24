"""`ml_lidar/obstacles.py` のテスト。通過可能性の判定が幾何と一致し、置いた障害物が
どれも前進で抜けられ、`EnvConfig.obstacle_prob=0`では従来と同じ乱数列になることを確認する。"""

import math
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))  # surge_mk2/

import numpy as np  # noqa: E402

from ml_lidar.course_gen import random_walk_loop_course  # noqa: E402
from ml_lidar.env import EnvConfig, LidarE2EEnv  # noqa: E402
from ml_lidar.obstacles import (  # noqa: E402
    _START_CLEAR_AHEAD_M,
    PassabilityChecker,
    add_obstacles,
)
from sim import track  # noqa: E402
from sim.vehicle import DriveInput, VehicleModel, VehicleSpec  # noqa: E402


def _course_and_mid(seed: int = 3):
    course = random_walk_loop_course(np.random.default_rng(seed))
    checker = PassabilityChecker(course)
    s = checker.total_length * 0.5
    return course, checker, s, checker.pose_at(s)


class TestPassability(unittest.TestCase):
    def test_clear_corridor_is_passable(self) -> None:
        _, checker, s, _ = _course_and_mid()
        self.assertIsNotNone(checker.search(s))

    def test_fully_blocked_is_not_passable(self) -> None:
        course, checker, s, (x, y, _) = _course_and_mid()
        track.stamp_discs(course.grid, course.origin, course.resolution,
                          [(x, y, float(course.width) / 2.0)])
        checker.update()
        self.assertIsNone(checker.search(s))

    def test_gap_narrower_than_body_is_not_passable(self) -> None:
        """片側の隙間が車幅(0.18m)未満なら、どんな経路でも通れない。"""
        course, checker, s, (x, y, yaw) = _course_and_mid()
        w = float(course.width)
        gap = 0.15
        r = (w - gap) / 2.0
        off = w / 2.0 - r
        track.stamp_discs(course.grid, course.origin, course.resolution,
                          [(x - math.sin(yaw) * off, y + math.cos(yaw) * off, r)])
        checker.update()
        self.assertIsNone(checker.search(s))

    def test_small_disc_at_wall_is_passable(self) -> None:
        course, checker, s, (x, y, yaw) = _course_and_mid()
        half = float(course.width) / 2.0
        track.stamp_discs(course.grid, course.origin, course.resolution,
                          [(x - math.sin(yaw) * half, y + math.cos(yaw) * half, 0.08)])
        checker.update()
        self.assertIsNotNone(checker.search(s))


class TestAddObstacles(unittest.TestCase):
    def test_every_placed_obstacle_is_passable_and_start_is_clear(self) -> None:
        spec = VehicleSpec.load()
        for seed in range(15):
            rng = np.random.default_rng(seed)
            course = random_walk_loop_course(rng)
            obs = add_obstacles(course, rng, 4)
            self.assertIsNotNone(course.obstacles, f"seed={seed}: 1個も置けなかった")
            self.assertIsNone(course._padded_grid, "raycastキャッシュを捨てていない")
            body = course.body_samples(spec.footprint)
            self.assertFalse(course.collides(*course.start, body), f"seed={seed}: スタートが埋まった")
            checker = PassabilityChecker(course)
            for ox, oy, _ in obs:
                d2 = (course.centerline[:, 0] - ox) ** 2 + (course.centerline[:, 1] - oy) ** 2
                s = float(checker._arc[int(np.argmin(d2))])
                self.assertGreaterEqual(s, _START_CLEAR_AHEAD_M - 0.5)
                self.assertIsNotNone(checker.search(s), f"seed={seed}: 通れない障害物が残った")

    def test_found_path_is_drivable_by_vehicle_model(self) -> None:
        """検証器の検証: 判定が返した経路を、むだ時間・1次遅れつきの車両モデルが
        低速の純追従で衝突せずに辿れる（円被覆の保守性で足りているか）。"""
        spec = VehicleSpec.load()
        rng = np.random.default_rng(7)
        course = random_walk_loop_course(rng)
        obs = add_obstacles(course, rng, 4)
        checker = PassabilityChecker(course)
        body = course.body_samples(spec.footprint)
        for ox, oy, _ in obs:
            d2 = (course.centerline[:, 0] - ox) ** 2 + (course.centerline[:, 1] - oy) ** 2
            path = np.asarray(checker.search(float(checker._arc[int(np.argmin(d2))])))
            m = VehicleModel(spec, tuple(path[0]))
            for _ in range(4000):
                i = int(np.argmin(np.hypot(path[:, 0] - m.x, path[:, 1] - m.y)))
                if i >= len(path) - 2:
                    break
                tx, ty = path[min(len(path) - 1, i + 3), :2]
                a = math.atan2(ty - m.y, tx - m.x) - m.yaw
                ld = max(0.1, math.hypot(tx - m.x, ty - m.y))
                m.apply(DriveInput(armed=True, target_speed=0.5,
                                   target_steer=math.atan(2 * spec.wheelbase * math.sin(a) / ld)))
                m.step(0.005)
                self.assertFalse(course.collides(m.x, m.y, m.yaw, body))
            self.assertGreaterEqual(i, len(path) - 2, "追従が終点まで届かなかった")


class TestEnvIntegration(unittest.TestCase):
    def test_obstacle_prob_zero_keeps_rng_sequence(self) -> None:
        """既定（0）では乱数を消費しない＝v23以前と同じシードで同じコース・同じ観測。"""
        a = LidarE2EEnv(EnvConfig(), seed=11)
        b = LidarE2EEnv(EnvConfig(obstacle_prob=0.0, max_obstacles=9), seed=11)
        oa, _ = a.reset(seed=11)
        ob, _ = b.reset(seed=11)
        np.testing.assert_array_equal(a.course.grid, b.course.grid)
        np.testing.assert_array_equal(oa, ob)
        self.assertIsNone(a.course.obstacles)

    def test_obstacles_reach_lidar(self) -> None:
        env = LidarE2EEnv(EnvConfig(obstacle_prob=1.0), seed=5)
        env.reset(seed=5)
        self.assertIsNotNone(env.course.obstacles)
        # 障害物の中心から外へ向けて撃つと、自分の円盤の縁（半径r）で当たる。
        # ★当たらないレイは0.0を返すので「<閾値」だけでは素通りを見逃す。下限も見る
        c = env.course
        for ox, oy, r in c.obstacles:
            d = c.raycast(float(ox), float(oy), np.linspace(0, 2 * np.pi, 8, endpoint=False), 5.0)
            self.assertTrue(np.all(d > 0.0), "障害物の中心から撃ったレイが当たらない")
            self.assertLessEqual(float(d.min()), r + 2 * c.resolution, "障害物が格子に刻まれていない")

    def test_fixed_course_is_never_mutated(self) -> None:
        course = random_walk_loop_course(np.random.default_rng(0))
        before = course.grid.copy()
        env = LidarE2EEnv(EnvConfig(obstacle_prob=1.0), course=course, seed=0)
        env.reset(seed=0)
        np.testing.assert_array_equal(course.grid, before)
        self.assertIsNone(course.obstacles)


if __name__ == "__main__":
    unittest.main()
