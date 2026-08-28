"""`ml_lidar/export_onnx_rl.py` のテスト。実際の学習は行わず、ランダム初期化した
PPO方策（`.learn()`を呼ばない）をそのままエクスポートし、配管が壊れていないこと・
PyTorch/ONNXRuntimeの出力が一致することだけを確認する。
"""

import json
import sys
import tempfile
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))  # surge_mk2/

import numpy as np  # noqa: E402
from stable_baselines3 import PPO  # noqa: E402

from ml_lidar.env import GymSurgeEnv  # noqa: E402
from ml_lidar.export_onnx_rl import export, verify_parity  # noqa: E402
from sim.gym_env import OBS_DIM  # noqa: E402
from sim.random_course import generate_random_course  # noqa: E402


class TestExportOnnxRl(unittest.TestCase):
    def test_export_and_parity(self):
        rng = np.random.default_rng(0)
        courses = [generate_random_course(rng, name="c0")]
        env = GymSurgeEnv(courses, max_steps=50, seed=0)
        model = PPO("MlpPolicy", env, policy_kwargs=dict(net_arch=[16, 16]), device="cpu")

        with tempfile.TemporaryDirectory() as d:
            model_path = Path(d) / "model.zip"
            onnx_path = Path(d) / "e2e_lidar.onnx"
            model.save(str(model_path))

            export(model_path, onnx_path, max_speed=1.5, max_steer=0.45)
            self.assertTrue(onnx_path.exists())

            cfg = json.loads(onnx_path.with_suffix(".json").read_text())
            self.assertEqual(cfg["in_dim"], OBS_DIM)
            self.assertEqual(cfg["max_speed"], 1.5)
            self.assertEqual(cfg["max_steer"], 0.45)

            diff = verify_parity(model_path, onnx_path)
            self.assertLess(diff, 1e-4)


if __name__ == "__main__":
    unittest.main()
