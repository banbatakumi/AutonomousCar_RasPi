"""`ml_lidar/env.py` のテスト。

センターライン弧長進捗（ラップアラウンド含む）は正方形ループの合成データで
決定的に検証し、reset/step の配線と衝突終端は`course_gen`の実コースで
スモークテストする。
"""

import dataclasses
import math
import sys
import types
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))  # surge_mk2/

import numpy as np  # noqa: E402

from ml_lidar.course_gen import random_walk_loop_course  # noqa: E402
from ml_lidar.env import EnvConfig, LidarE2EEnv  # noqa: E402
from sim.lidar import SCAN_HZ  # noqa: E402


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


class TestMinimalReward(unittest.TestCase):
    """v21: 報酬が`progress/dt`＋終端項だけであることを固定する回帰テスト。

    v16〜v20で足した罰則3種（`steer_effort_weight`・`raceline_weight`・
    `steer_angle_weight`）は2026-09-17に全て撤去した（env.py EnvConfig
    docstring「v21」参照）。**罰則をうっかり足し直すと静かに壊れる**類の
    不変条件なので、値そのものを突き合わせて固定しておく。
    """

    def test_nonterminal_reward_is_exactly_progress_rate(self) -> None:
        """衝突・完走していないステップの報酬は`info["progress_m"]/dt`に厳密一致する。"""
        cfg = EnvConfig(randomize_lidar=False, randomize_dynamics=False)
        course = random_walk_loop_course(np.random.default_rng(3))
        env = LidarE2EEnv(cfg, course=course)
        env.reset(seed=0)

        checked = 0
        for a0 in (1.0, -1.0, 0.4, 0.0):          # 据え切り・切り戻しも含めて振る
            action = np.array([a0, 0.5], dtype=np.float32)
            for _ in range(10):
                _, reward, terminated, _, info = env.step(action)
                if terminated:
                    break
                self.assertAlmostEqual(reward, info["progress_m"] / cfg.dt, places=9)
                checked += 1
            if terminated:
                break
        self.assertGreater(checked, 0, "非終端ステップが1つも無かった")

    def test_removed_reward_weights_are_gone(self) -> None:
        """撤去した罰則の重みが`EnvConfig`に復活していないこと。"""
        fields = {f.name for f in dataclasses.fields(EnvConfig)}
        for name in ("steer_effort_weight", "raceline_weight", "raceline_tolerance_m",
                     "steer_angle_weight", "steer_rate_weight"):
            self.assertNotIn(name, fields,
                             f"撤去済みの罰則{name}が復活している（env.py docstring「v21」参照）")


class TestControlPeriod(unittest.TestCase):
    """v21b: 制御周期がLiDARの回転周期＝実車の計画周期と一致していることを固定する。"""

    def test_dt_matches_lidar_scan_period(self) -> None:
        """`dt`が`sim.lidar.SCAN_HZ`の周期と一致すること。

        ずれると①観測のスキャン部が更新されないステップが生まれ（`dt=0.05`では
        実測54%）、②`raspi/nodes/planning_node.py:_replan()`が`scan.seq`単位でしか
        `plan()`を呼ばない実車と決定レートがずれる（env.py docstring「v21b」参照）。
        `SCAN_HZ`側が変わったらこのテストが落ちて気づけるように、値を直書きせず
        `sim.lidar`から引く。
        """
        self.assertAlmostEqual(EnvConfig().dt, 1.0 / SCAN_HZ, places=9)

    def test_scan_is_fresh_on_almost_every_step(self) -> None:
        """実際に回して、観測のスキャン部が前ステップと同一になる割合が十分低いこと。

        `dt`と`SCAN_HZ`が一致していても、LiDARの遅延ジッタぶんは据え置きが残る
        （実測10〜12%）。`dt=0.05`時代の54%との差は大きいので、緩い上限で十分に
        退行を検出できる。
        """
        cfg = EnvConfig(randomize_lidar=True, randomize_dynamics=False)
        action = np.array([0.05, 0.4], dtype=np.float32)   # ゆるい旋回で壁まで走る
        same = total = 0
        for seed in range(11, 19):                          # 1本は早期に衝突しうるので束ねる
            env = LidarE2EEnv(cfg, seed=seed)
            obs, _ = env.reset(seed=seed)
            prev = np.asarray(obs[:-2]).copy()
            for _ in range(60):
                obs, _, terminated, truncated, _ = env.step(action)
                cur = np.asarray(obs[:-2])
                same += int(np.array_equal(cur, prev))
                total += 1
                prev = cur.copy()
                if terminated or truncated:
                    break
        self.assertGreater(total, 60)
        self.assertLess(same / total, 0.30, "スキャンが更新されないステップが多すぎる")


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
