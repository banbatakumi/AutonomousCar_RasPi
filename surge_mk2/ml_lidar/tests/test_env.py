"""`ml_lidar/env.py` のテスト。

センターライン弧長進捗（ラップアラウンド含む）は正方形ループの合成データで
決定的に検証し、reset/step の配線と衝突終端は`course_gen`の実コースで
スモークテストする。
"""

import math
import sys
import types
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))  # surge_mk2/

import numpy as np  # noqa: E402

from ml_lidar.course_gen import random_walk_loop_course  # noqa: E402
from ml_lidar.env import EnvConfig, LidarE2EEnv  # noqa: E402
from sim.vehicle import VehicleSpec  # noqa: E402


def _square_centerline(side: float = 2.0, step: float = 0.1) -> np.ndarray:
    """一辺`side`の正方形ループのセンターライン `(N,3)`（x,y,yaw）。閉じる最終点は含めない。"""
    corners = [(0.0, 0.0), (side, 0.0), (side, side), (0.0, side)]
    yaws = [0.0, math.pi / 2, math.pi, -math.pi / 2]
    pts = []
    for (cx, cy), yaw, (nx, ny) in zip(corners, yaws, corners[1:] + corners[:1]):
        length = math.hypot(nx - cx, ny - cy)
        n = max(1, int(round(length / step)))
        for i in range(n):
            t = i / n
            pts.append((cx + (nx - cx) * t, cy + (ny - cy) * t, yaw))
    return np.asarray(pts, dtype=np.float64)


class TestCenterlineProgress(unittest.TestCase):
    """`_progress_delta()`のラップアラウンド処理を、一辺2mの正方形ループ（周長8m）で検証する。"""

    def setUp(self) -> None:
        self.env = LidarE2EEnv(EnvConfig())
        course = types.SimpleNamespace(centerline=_square_centerline())
        self.env._prepare_centerline(course)

    def test_total_length_matches_perimeter(self) -> None:
        self.assertAlmostEqual(self.env._total_length, 8.0, delta=0.2)

    def test_forward_progress_is_positive(self) -> None:
        s0 = self.env._project_arc_length(0.0, 0.0)
        self.env._s_prev = s0
        s1 = self.env._project_arc_length(1.0, 0.0)
        self.assertGreater(self.env._progress_delta(s1), 0.0)

    def test_reverse_progress_is_negative_before_clipping(self) -> None:
        """`_progress_delta()`自体はクリップしない（クリップは`step()`の報酬計算側の責務）。"""
        s0 = self.env._project_arc_length(1.0, 0.0)
        self.env._s_prev = s0
        s1 = self.env._project_arc_length(0.0, 0.0)
        self.assertLess(self.env._progress_delta(s1), 0.0)

    def test_wraparound_finish_to_start_stays_positive(self) -> None:
        """ゴール手前(0, 0.05)からスタート直後(0.05, 0)へまたぐ動き。

        補正が無いと `delta ≈ -total_length` (≈-7.9m) になり、「後退した」と
        誤判定されて周回完走もprogressも壊れる。正しくは小さな正の値になるべき。
        """
        # サンプリング点(0.1m間隔)の中間を突くと最近傍が始点(0,0)と誤タイになるため、
        # 明確にどちらか一方へ寄せた座標を使う
        near_finish = self.env._project_arc_length(0.0, 0.08)  # 最終エッジの終端付近
        self.env._s_prev = near_finish
        near_start = self.env._project_arc_length(0.02, 0.0)   # スタート直後
        delta = self.env._progress_delta(near_start)
        self.assertGreater(delta, 0.0)
        self.assertLess(delta, 0.5)


class TestRaceline(unittest.TestCase):
    """v18: `_prepare_raceline()`（理想ライン=MCLの追従罰則）のテスト。"""

    def test_disabled_by_default_no_cost(self) -> None:
        """`raceline_weight<=0`(既定)なら`compute_raceline_offsets()`のL-BFGSを
        呼ばず、`_raceline_xy`はNoneのままになる（env.py `_prepare_raceline`
        docstring参照——既存configの学習速度に負担をかけないための仕様）。"""
        env = LidarE2EEnv(EnvConfig())
        course = types.SimpleNamespace(centerline=_square_centerline(), width=1.0)
        env._prepare_raceline(course, VehicleSpec.load())
        self.assertIsNone(env._raceline_xy)

    def test_enabled_moves_off_centerline_near_sharp_corners(self) -> None:
        """一辺2mの正方形ループ(直角コーナー×4)は曲率が局所的に非常に大きいので、
        曲率二乗和を最小化するMCLは各コーナーでセンターラインから明確に離れる
        はず。"""
        env = LidarE2EEnv(EnvConfig(raceline_weight=0.3))
        course = types.SimpleNamespace(centerline=_square_centerline(), width=1.0)
        env._prepare_raceline(course, VehicleSpec.load())
        self.assertIsNotNone(env._raceline_xy)
        self.assertEqual(env._raceline_xy.shape, course.centerline[:, :2].shape)
        dev = np.hypot(*(env._raceline_xy - course.centerline[:, :2]).T)
        self.assertGreater(dev.max(), 0.02,
                           "曲率最小化のMCLが直角コーナーでセンターラインからほぼ動いていない")

    def test_raceline_penalty_reduces_reward(self) -> None:
        """同じコース・同じ行動列でも、raceline_weight>0の方が各ステップの報酬が
        罰則ぶんだけ以下になる——`step()`内の`_project_centerline`インデックス
        流用に誤りがあれば崩れる不変条件。"""
        course = random_walk_loop_course(np.random.default_rng(3))
        action = np.array([0.3, 0.5], dtype=np.float32)

        def _make(weight: float) -> LidarE2EEnv:
            cfg = EnvConfig(randomize_lidar=False, randomize_dynamics=False,
                            raceline_weight=weight)
            return LidarE2EEnv(cfg, course=course)

        env_off, env_on = _make(0.0), _make(0.3)
        env_off.reset(seed=0)
        env_on.reset(seed=0)

        saw_nonzero_dev = False
        for _ in range(30):
            _, r_off, term_off, trunc_off, _ = env_off.step(action)
            _, r_on, term_on, trunc_on, info_on = env_on.step(action)
            self.assertGreaterEqual(info_on["raceline_dev_m"], 0.0)
            saw_nonzero_dev = saw_nonzero_dev or info_on["raceline_dev_m"] > 1e-6
            self.assertLessEqual(r_on, r_off + 1e-9)
            if term_off or term_on:
                break
        self.assertTrue(saw_nonzero_dev, "理想ラインからの偏差が一度も観測されなかった")


class TestEnvSmoke(unittest.TestCase):
    """reset/stepの配線・観測の形状・衝突終端をコース生成込みで確認する。"""

    def _make_env(self, seed: int, **cfg_kwargs) -> LidarE2EEnv:
        course = random_walk_loop_course(np.random.default_rng(seed))
        cfg = EnvConfig(randomize_lidar=False, randomize_dynamics=False, **cfg_kwargs)
        return LidarE2EEnv(cfg, course=course)

    def test_reset_returns_obs_within_bounds(self) -> None:
        env = self._make_env(seed=1)
        obs, info = env.reset(seed=0)
        self.assertEqual(obs.shape, env.observation_space.shape)
        self.assertTrue(np.all(obs >= env.observation_space.low - 1e-4))
        self.assertTrue(np.all(obs <= env.observation_space.high + 1e-4))
        self.assertEqual(info, {})

    def test_step_returns_expected_shapes(self) -> None:
        env = self._make_env(seed=1)
        env.reset(seed=0)
        obs, reward, terminated, truncated, info = env.step(np.array([0.0, 0.3], dtype=np.float32))
        self.assertEqual(obs.shape, env.observation_space.shape)
        self.assertIsInstance(float(reward), float)
        self.assertFalse(truncated)
        self.assertIn("progress_m", info)
        self.assertIn("collided", info)

    def test_collision_terminates_with_penalty(self) -> None:
        env = self._make_env(seed=2)
        env.reset(seed=0)
        # 全開加速＋目一杯の据え切りを続ければ、有限ステップ内に必ず壁へ当たる
        # （最大舵角での旋回半径はコース半径よりはるかに小さい）
        action = np.array([1.0, 1.0], dtype=np.float32)
        terminated = False
        reward = 0.0
        info: dict = {}
        for _ in range(400):
            obs, reward, terminated, truncated, info = env.step(action)
            if terminated:
                break
        self.assertTrue(terminated, "400ステップ以内に衝突しなかった")
        self.assertTrue(info["collided"])
        self.assertLess(reward, -8.0)


if __name__ == "__main__":
    unittest.main()
