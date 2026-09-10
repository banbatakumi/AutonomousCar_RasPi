"""`ml_lidar/watch.py`の`Watcher`（可視化を除く駆動ロジック）のテスト。

`raspi/auto/e2e_lidar.py`を実際に読み込んで動かす——GUIを開かずに済む部分
（コースへのcollides判定・LiDAR中間案経路・`E2ELidar.plan()`との配線）を確認する。
"""

import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))  # surge_mk2/

import numpy as np  # noqa: E402
from gymnasium.wrappers import TimeLimit  # noqa: E402
from stable_baselines3 import PPO  # noqa: E402
from stable_baselines3.common.monitor import Monitor  # noqa: E402
from stable_baselines3.common.vec_env import DummyVecEnv  # noqa: E402

from ml_lidar import export_onnx_rl  # noqa: E402
from ml_lidar.course_gen import random_walk_loop_course  # noqa: E402
from ml_lidar.env import EnvConfig, LidarE2EEnv  # noqa: E402
from ml_lidar.features import Conv1dExtractor  # noqa: E402
from ml_lidar.watch import Watcher  # noqa: E402


def _train_and_export_tiny_model(tmp_dir: Path) -> Path:
    cfg = EnvConfig(fov_deg=60.0, randomize_lidar=False, randomize_dynamics=False)

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

    onnx_path = tmp_dir / "watch_smoke.onnx"
    export_onnx_rl.export_model(model, onnx_path)
    (tmp_dir / "watch_smoke.json").write_text(
        '{"fov_deg": 60.0, "max_range": 5.10, "max_steer": 0.45, "max_speed": 1.5}',
        encoding="utf-8")
    return onnx_path


class TestWatcher(unittest.TestCase):
    def test_step_runs_without_crashing(self) -> None:
        import tempfile

        with tempfile.TemporaryDirectory() as tmp:
            tmp_dir = Path(tmp)
            onnx_path = _train_and_export_tiny_model(tmp_dir)
            course = random_walk_loop_course(np.random.default_rng(0))
            watcher = Watcher(onnx_path, course, dt=0.05)

            saw_ready = False
            for _ in range(100):
                info = watcher.step()
                saw_ready = saw_ready or info["ready"]
                if info["collided"]:
                    break

            # モデルが未ロードのまま停止し続けている「壊れた配線」ではなく、
            # 実際に推論結果（ready=True）へ到達していることを確認する
            self.assertTrue(saw_ready, "E2ELidar.plan()がreadyに一度も到達しなかった")


if __name__ == "__main__":
    unittest.main()
