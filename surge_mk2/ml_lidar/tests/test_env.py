"""`sim/gym_env.py` の `SimE2EEnv` のテスト。実際の学習は行わず、
エピソードが仕様通りに終了すること・報酬の符号がおかしくないことだけを確認する。
"""

import math
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))  # surge_mk2/

import numpy as np  # noqa: E402

from sim.gym_env import OBS_DIM, SimE2EEnv  # noqa: E402
from sim.random_course import generate_random_course  # noqa: E402


def _make_env(**kwargs) -> SimE2EEnv:
    rng = np.random.default_rng(0)
    courses = [generate_random_course(rng, name=f"c{i}") for i in range(2)]
    return SimE2EEnv(courses, seed=0, **kwargs)


class TestSimE2EEnv(unittest.TestCase):
    def test_reset_returns_scan_plus_speed_observation(self):
        env = _make_env()
        obs = env.reset()
        self.assertEqual(obs.shape, (OBS_DIM,))
        self.assertTrue(np.all(np.isfinite(obs)))
        self.assertEqual(obs[-1], 0.0)     # 発進直後は速度0のはず

    def test_collision_terminates_episode(self):
        env = _make_env(max_steps=500)
        env.reset()
        terminated = truncated = False
        steps = 0
        # ステアを切らずに全開で直進すれば、ランダムコースのどこかで必ず壁に当たる
        while not (terminated or truncated) and steps < 500:
            _, reward, terminated, truncated, info = env.step(np.array([0.0, 1.0]))
            steps += 1
        self.assertTrue(terminated)
        self.assertTrue(info["collided"])
        self.assertLess(reward, 0.0)   # 衝突ペナルティが乗っているはず

    def test_max_steps_truncates_without_collision_when_stationary(self):
        env = _make_env(max_steps=20)
        env.reset()
        terminated = truncated = False
        for _ in range(20):
            _, _, terminated, truncated, info = env.step(np.array([0.0, 0.0]))
        self.assertFalse(terminated)
        self.assertTrue(truncated)
        self.assertFalse(info["collided"])

    def test_progress_reward_is_near_zero_when_stationary(self):
        env = _make_env(max_steps=50, cross_track_weight=0.0)
        env.reset()
        total = 0.0
        for _ in range(20):
            _, r, _, _, _ = env.step(np.array([0.0, 0.0]))
            total += r
        self.assertAlmostEqual(total, 0.0, delta=1e-6)

    def test_action_is_clamped_to_limits(self):
        env = _make_env(max_speed=0.5, max_steer=0.3)
        env.reset()
        # 範囲外の行動を渡しても例外にならず、内部でクランプされる
        env.step(np.array([10.0, 100.0]))

    def test_observation_speed_tracks_actual_vehicle_speed(self):
        """観測の末尾1個が自車速度になっている（改善1）。"""
        env = _make_env(max_steps=50, max_speed=1.0)
        env.reset()
        obs = None
        for _ in range(20):
            obs, _, _, _, _ = env.step(np.array([0.0, 1.0]))     # 全開で加速
        self.assertGreater(obs[-1], 0.0)
        self.assertAlmostEqual(float(obs[-1]), env.vehicle.speed, places=5)

    def test_randomize_lidar_varies_noise_between_episodes(self):
        """改善2: `randomize_lidar=True`（既定）なら毎エピソードノイズ量が変わる。"""
        env = _make_env(randomize_lidar=True)
        env.reset()
        sigma1 = env.lidar.params.lidar_noise_sigma_m
        env.reset()
        sigma2 = env.lidar.params.lidar_noise_sigma_m
        env.reset()
        sigma3 = env.lidar.params.lidar_noise_sigma_m
        self.assertFalse(sigma1 == sigma2 == sigma3)

    def test_randomize_lidar_false_keeps_noise_fixed(self):
        """評価環境（`train_rl.py`の`make_eval_env`）はここを`False`にして条件を揃える。"""
        env = _make_env(randomize_lidar=False)
        env.reset()
        sigma1 = env.lidar.params.lidar_noise_sigma_m
        env.reset()
        sigma2 = env.lidar.params.lidar_noise_sigma_m
        self.assertEqual(sigma1, sigma2)
        self.assertEqual(sigma1, env.sim_params.lidar_noise_sigma_m)

    def test_course_fn_generates_a_fresh_course_each_episode(self):
        """改善3: 固定プールではなく毎エピソード新しいコースを作れる。
        `generate_random_course`をそのまま渡せる（環境自身の`rng`を引数で受け取る形）。"""
        seen_shapes = set()
        env = SimE2EEnv(course_fn=generate_random_course, seed=1)
        for _ in range(3):
            env.reset()
            seen_shapes.add(env.course.grid.shape)
        self.assertGreater(len(seen_shapes), 1)    # 毎回同じ形にはならないはず

    def test_course_fn_reset_with_same_seed_gives_the_same_course(self):
        """gymnasiumの決定性契約: `course_fn`は環境自身の`rng`を使うので、
        同じseedでresetし直せば同じコースになる（`check_env`の要件でもある）。"""
        env = SimE2EEnv(course_fn=generate_random_course, seed=7)
        env.rng = np.random.default_rng(7)
        env.reset()
        shape1 = env.course.grid.shape
        env.rng = np.random.default_rng(7)
        env.reset()
        shape2 = env.course.grid.shape
        self.assertEqual(shape1, shape2)

    def test_missing_courses_and_course_fn_raises(self):
        with self.assertRaises(ValueError):
            SimE2EEnv()


if __name__ == "__main__":
    unittest.main()
