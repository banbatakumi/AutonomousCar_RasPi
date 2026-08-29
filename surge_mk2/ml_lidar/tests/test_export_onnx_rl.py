"""`ml_lidar/export_onnx_rl.py` のテスト。実際の学習は行わず、ランダム初期化した
PPO方策（`.learn()`を呼ばない）をそのままエクスポートし、配管が壊れていないこと・
PyTorch/ONNXRuntimeの出力が一致することだけを確認する。
"""

import json
import shutil
import sys
import tempfile
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))  # surge_mk2/

import numpy as np  # noqa: E402
import onnxruntime as ort  # noqa: E402
from stable_baselines3 import PPO  # noqa: E402

from ml_lidar.env import GymSurgeEnv  # noqa: E402
from ml_lidar.export_onnx_rl import export, load_run_config_defaults, verify_parity  # noqa: E402
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
            self.assertEqual(cfg["note"], "")   # 省略時は空文字（2026-08-29追加）

            diff = verify_parity(model_path, onnx_path)
            self.assertLess(diff, 1e-4)

    def test_note_is_embedded_in_the_exported_json(self):
        """`ml_lidar/app.py`の備考欄→`note.txt`→エクスポート、という経路で
        `<名前>.json`に書き込まれる自由記述の備考（2026-08-29追加）。GUIの
        モデル選択（`AutoPanel.tsx`）が表示するので、`note`キーの往復を確認する。"""
        rng = np.random.default_rng(0)
        courses = [generate_random_course(rng, name="c0")]
        env = GymSurgeEnv(courses, max_steps=50, seed=0)
        model = PPO("MlpPolicy", env, policy_kwargs=dict(net_arch=[16, 16]), device="cpu")

        with tempfile.TemporaryDirectory() as d:
            model_path = Path(d) / "model.zip"
            onnx_path = Path(d) / "e2e_lidar.onnx"
            model.save(str(model_path))

            export(model_path, onnx_path, max_speed=1.5, max_steer=0.45,
                  note="グリップ限界+報酬改善版、circuitで学習")
            cfg = json.loads(onnx_path.with_suffix(".json").read_text())
            self.assertEqual(cfg["note"], "グリップ限界+報酬改善版、circuitで学習")

    def test_onnx_file_loads_alone_without_its_export_directory(self):
        """`models/e2e_lidar/`への配置運用は`.onnx`単体をコピーする。`.data`（外部重み
        ファイル）が元のディレクトリに残ったままだと隣にある間は気づかず動いてしまう
        ので、別ディレクトリへコピーしてロードし、`external_data=False`の回帰を検出する
        （このリポジトリで実際に踏んだ罠——`ml/export_onnx.py`にも同型の罠があった）。
        """
        rng = np.random.default_rng(0)
        courses = [generate_random_course(rng, name="c0")]
        env = GymSurgeEnv(courses, max_steps=50, seed=0)
        model = PPO("MlpPolicy", env, policy_kwargs=dict(net_arch=[16, 16]), device="cpu")

        with tempfile.TemporaryDirectory() as d:
            model_path = Path(d) / "model.zip"
            onnx_path = Path(d) / "e2e_lidar.onnx"
            model.save(str(model_path))
            export(model_path, onnx_path, max_speed=1.5, max_steer=0.45)

            with tempfile.TemporaryDirectory() as lone_dir:
                lone_path = Path(lone_dir) / "renamed.onnx"
                shutil.copy(onnx_path, lone_path)          # .json も .data も持っていかない
                sess = ort.InferenceSession(str(lone_path), providers=["CPUExecutionProvider"])
                sess.run(None, {"input": np.zeros((1, OBS_DIM), dtype=np.float32)})


class TestLoadRunConfigDefaults(unittest.TestCase):
    """`train_rl.py`が書く`run_config.json`から`--max-speed`/`--max-steer`の既定値を
    拾う`load_run_config_defaults()`のテスト（2026-08-28、値を覚えていなくても
    エクスポートできるようにするための変更）。"""

    def test_reads_values_from_sibling_run_config_json(self):
        with tempfile.TemporaryDirectory() as d:
            model_path = Path(d) / "best_model.zip"
            (Path(d) / "run_config.json").write_text(
                json.dumps({"max_speed": 1.5, "max_steer": 0.45}))
            cfg = load_run_config_defaults(model_path)
            self.assertEqual(cfg["max_speed"], 1.5)
            self.assertEqual(cfg["max_steer"], 0.45)

    def test_missing_run_config_json_returns_empty_dict(self):
        with tempfile.TemporaryDirectory() as d:
            model_path = Path(d) / "best_model.zip"
            self.assertEqual(load_run_config_defaults(model_path), {})

    def test_corrupt_run_config_json_returns_empty_dict(self):
        with tempfile.TemporaryDirectory() as d:
            model_path = Path(d) / "best_model.zip"
            (Path(d) / "run_config.json").write_text("{ not json")
            self.assertEqual(load_run_config_defaults(model_path), {})


if __name__ == "__main__":
    unittest.main()
