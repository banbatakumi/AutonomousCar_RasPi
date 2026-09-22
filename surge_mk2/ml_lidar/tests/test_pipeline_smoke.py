"""学習→統計評価→ONNXエクスポートの一気通貫スモークテスト。

`train_rl.py`/`eval_stats.py`/`export_onnx_rl.py`の配線がずれていないかを、
実際に極小のPPOモデルを1つ学習させて確認する（数秒で終わる規模に絞ってある）。
個々の単体テストでは拾えない「モデルの受け渡し」自体の破綻（観測次元の不一致・
ONNX入出力パリティ・`raspi/auto/e2e_lidar.py`側の読み込み）を検出する。
"""

import contextlib
import io
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))  # surge_mk2/

import numpy as np  # noqa: E402
from gymnasium.wrappers import TimeLimit  # noqa: E402
from stable_baselines3 import PPO  # noqa: E402
from stable_baselines3.common.monitor import Monitor  # noqa: E402
from stable_baselines3.common.vec_env import DummyVecEnv  # noqa: E402

from ml_lidar import eval_stats, export_onnx_rl  # noqa: E402
from ml_lidar.env import EnvConfig, LidarE2EEnv  # noqa: E402
from ml_lidar.features import Conv1dExtractor  # noqa: E402


def _make_tiny_model(cfg: EnvConfig, max_episode_steps: int) -> PPO:
    def _thunk():
        env = LidarE2EEnv(cfg, seed=0)
        env = TimeLimit(env, max_episode_steps=max_episode_steps)
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


class TestPipelineSmoke(unittest.TestCase):
    def setUp(self) -> None:
        self.cfg = EnvConfig(fov_deg=60.0, randomize_lidar=False, randomize_dynamics=False)
        self.max_episode_steps = 40
        self.model = _make_tiny_model(self.cfg, self.max_episode_steps)

    def test_eval_stats_runs(self) -> None:
        result = eval_stats.evaluate(self.model, self.cfg, n_episodes=2, episode_s=2.0, base_seed=0)
        self.assertEqual(result["n_episodes"], 2)
        self.assertGreaterEqual(result["collision_rate"], 0.0)
        self.assertLessEqual(result["collision_rate"], 1.0)

    def test_export_and_parity(self) -> None:
        import tempfile

        with tempfile.TemporaryDirectory() as tmp:
            onnx_path = Path(tmp) / "smoke.onnx"
            export_onnx_rl.export_model(self.model, onnx_path)
            self.assertTrue(onnx_path.exists())
            max_diff = export_onnx_rl.check_parity(self.model, onnx_path, n_samples=20)
            self.assertLess(max_diff, 1e-3)

    def test_find_env_config_walks_up_from_model_path(self) -> None:
        import json
        import tempfile

        with tempfile.TemporaryDirectory() as tmp:
            run_dir = Path(tmp) / "runs" / "v1"
            eval_dir = run_dir / "eval"
            eval_dir.mkdir(parents=True)
            (run_dir / "env_config.json").write_text(
                json.dumps({"fov_deg": 123.0, "max_speed": 9.0, "unknown_future_field": "x"}),
                encoding="utf-8")
            cfg = eval_stats.find_env_config(eval_dir / "best_model.zip")
            self.assertIsNotNone(cfg)
            self.assertEqual(cfg.fov_deg, 123.0)
            self.assertEqual(cfg.max_speed, 9.0)

    def test_find_env_config_returns_none_when_absent(self) -> None:
        import tempfile

        with tempfile.TemporaryDirectory() as tmp:
            cfg = eval_stats.find_env_config(Path(tmp) / "nowhere" / "model.zip")
            self.assertIsNone(cfg)


class TestCliHelpRenders(unittest.TestCase):
    """`--help`が例外なく描画できること。

    argparseは`help`文字列に`help % params`を掛けるので、**日本語のヘルプに素の`%`を
    書くと`--help`だけが落ちる**（2026-09-17に`--dt`の「54%のステップで」で実際に
    踏んだ。学習自体は動くので他のスモークテストでは気づけない）。`%%`へのエスケープ
    漏れをここで検出する。
    """

    def _assert_help_renders(self, parse_args) -> None:
        """`--help`はヘルプを描画してから`SystemExit(0)`で抜ける。描画に失敗すると
        `SystemExit`ではなく`ValueError`等が飛ぶので、それをそのまま失敗にする。"""
        buf = io.StringIO()
        with contextlib.redirect_stdout(buf):
            with self.assertRaises(SystemExit) as cm:
                parse_args(["--help"])
        self.assertEqual(cm.exception.code, 0)
        self.assertIn("--help", buf.getvalue())

    def test_train_rl_help(self) -> None:
        from ml_lidar import train_rl
        self._assert_help_renders(train_rl.parse_args)

    def test_eval_stats_help(self) -> None:
        from ml_lidar import eval_stats
        self._assert_help_renders(eval_stats.parse_args)


if __name__ == "__main__":
    unittest.main()
