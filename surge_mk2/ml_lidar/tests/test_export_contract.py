"""export_onnx_rl の契約 JSON（dt/steer_tau）・外部データ無し・舵角契約の警告のテスト。"""

import contextlib
import dataclasses
import io
import json
import sys
import tempfile
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))  # surge_mk2/

from ml_lidar import export_onnx_rl  # noqa: E402
from ml_lidar.env import EnvConfig  # noqa: E402
from ml_lidar.eval_stats import steer_contract_warnings  # noqa: E402
from ml_lidar.tests.test_pipeline_smoke import _make_tiny_model  # noqa: E402


class TestExportContract(unittest.TestCase):
    def test_json_has_dt_steer_tau_and_onnx_is_self_contained(self) -> None:
        cfg = EnvConfig(fov_deg=60.0, randomize_lidar=False, randomize_dynamics=False,
                        steer_tau=0.12)
        model = _make_tiny_model(cfg, 40)
        with tempfile.TemporaryDirectory() as tmp:
            run = Path(tmp) / "run"
            (run / "eval").mkdir(parents=True)
            model.save(str(run / "eval" / "best_model"))
            (run / "env_config.json").write_text(json.dumps(dataclasses.asdict(cfg)))
            out = Path(tmp) / "models"
            with contextlib.redirect_stdout(io.StringIO()), contextlib.redirect_stderr(io.StringIO()):
                export_onnx_rl.main(["--model", str(run / "eval" / "best_model.zip"),
                                     "--name", "t", "--models-dir", str(out),
                                     "--max-steer", "0.4"])
            data = json.loads((out / "t.json").read_text())
            self.assertEqual(data["dt"], cfg.dt)
            self.assertEqual(data["steer_tau"], 0.12)
            self.assertEqual(data["max_steer"], 0.4)
            self.assertEqual(sorted(p.name for p in out.iterdir()), ["t.json", "t.onnx"])


class TestSteerContractWarnings(unittest.TestCase):
    def _warn(self, data: dict) -> list[str]:
        with tempfile.TemporaryDirectory() as tmp:
            p = Path(tmp) / "env_config.json"
            p.write_text(json.dumps(data))
            return steer_contract_warnings(p, 0.5, (1.0, 0.0))

    def test_same_values_no_warning(self) -> None:
        self.assertEqual(self._warn({"max_steer": 0.5, "steer_link": [1.0, 0.0]}), [])

    def test_differences_warn(self) -> None:
        self.assertEqual(len(self._warn({"max_steer": 0.4, "steer_link": [0.5, 0.1]})), 2)

    def test_old_run_warns_not_recorded(self) -> None:
        w = self._warn({})
        self.assertTrue(any("max_steer を記録していない" in x for x in w))

    def test_missing_file_is_silent(self) -> None:
        self.assertEqual(steer_contract_warnings(None), [])


if __name__ == "__main__":
    unittest.main()
