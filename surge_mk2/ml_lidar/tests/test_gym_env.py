"""`ml_lidar/env.py` の `GymSurgeEnv` のテスト。gymnasium の標準チェッカーで
API 準拠を検証し、観測/行動の正規化が範囲内に収まることを確認する。
"""

import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))  # surge_mk2/

import numpy as np  # noqa: E402
from gymnasium.utils.env_checker import check_env  # noqa: E402

from ml_lidar.env import GymSurgeEnv  # noqa: E402
from sim.gym_env import OBS_DIM  # noqa: E402
from sim.random_course import generate_random_course  # noqa: E402


def _make_courses(n: int = 2):
    rng = np.random.default_rng(0)
    return [generate_random_course(rng, name=f"c{i}") for i in range(n)]


class TestGymSurgeEnv(unittest.TestCase):
    def test_passes_gymnasium_env_checker(self):
        env = GymSurgeEnv(_make_courses(), max_steps=100, seed=0)
        check_env(env, skip_render_check=True)

    def test_observation_is_normalized_to_unit_range(self):
        env = GymSurgeEnv(_make_courses(), max_steps=100, seed=1)
        obs, _ = env.reset()
        self.assertEqual(obs.shape, (OBS_DIM,))
        self.assertTrue(np.all(obs >= 0.0) and np.all(obs <= 1.0))
        obs, _, _, _, _ = env.step(np.array([0.0, 0.0], dtype=np.float32))
        self.assertTrue(np.all(obs >= 0.0) and np.all(obs <= 1.0))

    def test_course_fn_is_accepted_and_passes_env_checker(self):
        """改善3: `GymSurgeEnv`経由でも`course_fn`（毎エピソード新規生成）が使える。
        `generate_random_course`をそのまま渡せる（環境自身の`rng`を引数で受け取る形）。"""
        env = GymSurgeEnv(course_fn=generate_random_course, max_steps=100, seed=5)
        check_env(env, skip_render_check=True)

    def test_action_minus_one_means_zero_speed(self):
        # action=[-1,-1] -> steer=-max_steer, speed=0（[-1,1]->[0,max_speed]変換の下端）
        env = GymSurgeEnv(_make_courses(), max_steps=100, seed=2)
        env.reset()
        phys = env._to_physical(np.array([-1.0, -1.0]))
        self.assertAlmostEqual(float(phys[1]), 0.0, places=5)

    def test_action_plus_one_means_max_speed(self):
        env = GymSurgeEnv(_make_courses(), max_steps=100, max_speed=1.5, seed=3)
        env.reset()
        phys = env._to_physical(np.array([1.0, 1.0]))
        self.assertAlmostEqual(float(phys[1]), 1.5, places=5)
        self.assertAlmostEqual(float(phys[0]), env._max_steer, places=5)


if __name__ == "__main__":
    unittest.main()
