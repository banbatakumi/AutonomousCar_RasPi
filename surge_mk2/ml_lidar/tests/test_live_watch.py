"""`ml_lidar/live_watch.py`（GUIを除く駆動ロジック）のテスト。

`eval/best_model.zip`と`checkpoints/`のうちmtimeが新しい方を自動で追従する
（`train_rl.py`の`find_watch_source()`）ことと、複数パネルが独立して進行・
リセットされることを確認する。観戦専用の追加保存機構は持たない設計。
"""

import os
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))  # surge_mk2/

import numpy as np  # noqa: E402
from gymnasium.wrappers import TimeLimit  # noqa: E402
from stable_baselines3 import PPO  # noqa: E402
from stable_baselines3.common.monitor import Monitor  # noqa: E402
from stable_baselines3.common.vec_env import DummyVecEnv  # noqa: E402

from ml_lidar.env import EnvConfig, LidarE2EEnv  # noqa: E402
from ml_lidar.features import Conv1dExtractor  # noqa: E402
from ml_lidar.train_rl import EVAL_COURSE_PARAMS  # noqa: E402
from ml_lidar.live_watch import LiveWatcher, _build_panels  # noqa: E402


def _tiny_model(cfg: EnvConfig) -> PPO:
    def _thunk():
        env = LidarE2EEnv(cfg, seed=0)
        env = TimeLimit(env, max_episode_steps=40)
        return Monitor(env)

    vec_env = DummyVecEnv([_thunk])
    n_window = int(round(cfg.fov_deg)) + 1
    policy_kwargs = dict(
        features_extractor_class=Conv1dExtractor,
        features_extractor_kwargs=dict(n_scan=n_window, features_dim=16),
        net_arch=dict(pi=[16], vf=[16]),
    )
    model = PPO("MlpPolicy", vec_env, n_steps=32, batch_size=16, n_epochs=1,
               policy_kwargs=policy_kwargs, device="cpu", verbose=0)
    model.learn(total_timesteps=32)
    return model


class TestLiveWatch(unittest.TestCase):
    def test_uses_checkpoints_when_no_best_model(self) -> None:
        import tempfile

        cfg = EnvConfig(fov_deg=60.0, randomize_lidar=False, randomize_dynamics=False)
        with tempfile.TemporaryDirectory() as tmp:
            run_dir = Path(tmp)
            ckpt_dir = run_dir / "checkpoints"
            ckpt_dir.mkdir()
            _tiny_model(cfg).save(str(ckpt_dir / "ppo_100_steps.zip"))

            panels = _build_panels(cfg, max_episode_steps=40)
            watcher = LiveWatcher(run_dir, panels, poll_interval_s=0.0)
            self.assertIsNotNone(watcher.model)
            self.assertEqual(watcher.model_path, ckpt_dir / "ppo_100_steps.zip")

    def test_reload_picks_newer_of_best_model_and_checkpoint(self) -> None:
        import tempfile

        cfg = EnvConfig(fov_deg=60.0, randomize_lidar=False, randomize_dynamics=False)
        with tempfile.TemporaryDirectory() as tmp:
            run_dir = Path(tmp)
            eval_dir = run_dir / "eval"
            ckpt_dir = run_dir / "checkpoints"
            eval_dir.mkdir()
            ckpt_dir.mkdir()

            best_path = eval_dir / "best_model.zip"
            ckpt_path = ckpt_dir / "ppo_100_steps.zip"
            _tiny_model(cfg).save(str(best_path))
            os.utime(best_path, (1000, 1000))

            panels = _build_panels(cfg, max_episode_steps=40)
            watcher = LiveWatcher(run_dir, panels, poll_interval_s=0.0)
            self.assertEqual(watcher.model_path, best_path)
            self.assertEqual(watcher.model_mtime, 1000)

            # best_modelのmtimeが変わらないうちは、古いままでも再読み込みしない
            self.assertFalse(watcher.maybe_reload(now=1.0))

            # 改善が止まりcheckpointsの方が新しくなった場合はそちらへ追従する
            _tiny_model(cfg).save(str(ckpt_path))
            os.utime(ckpt_path, (2000, 2000))
            self.assertTrue(watcher.maybe_reload(now=2.0))
            self.assertEqual(watcher.model_path, ckpt_path)
            self.assertEqual(watcher.model_mtime, 2000)

    def test_panels_step_and_reset_independently(self) -> None:
        cfg = EnvConfig(fov_deg=60.0, randomize_lidar=False, randomize_dynamics=False)
        model = _tiny_model(cfg)
        panels = _build_panels(cfg, max_episode_steps=5)  # 早期に打ち切ってreset経路を踏ませる

        # パネルは`EVAL_COURSE_PARAMS`に連動する（アーキタイプを足したらパネルも増える）。
        # 名前を直書きするとコース追加のたびにここが落ちるので、定義側から導く
        names = {p.name for p in panels}
        self.assertEqual(names, {spec.name for spec in EVAL_COURSE_PARAMS})
        self.assertIn("wide", names, "v22で追加した広いコースがパネルに出ていない")

        for _ in range(20):
            for panel in panels:
                panel.step(model)
        # 40ステップ相当も回せば、5ステップ上限のTimeLimitで各パネル最低1回はreset経由のはず
        for panel in panels:
            self.assertIn(panel.status, {"走行中", "衝突", "周回達成", "打ち切り"})
            self.assertEqual(panel.obs.shape, panel.base_env.observation_space.shape)


if __name__ == "__main__":
    unittest.main()
