"""システム同定の通し確認: 実機と同じ経路で録った mcap → 解析（GUI と同じ手順）→
`vehicle.toml` へ書き戻し → **そのシムが真値と同じ応答を返すか**。

個々の値が許容誤差に入っているか（`test_fit.py`）だけでは、解析の順序・値の受け渡し・
toml への書き込み・シム側での読み込みのどこかが壊れていても気づけない。最後に見たいのは
「同定した値を入れたシムが実機（ここでは真値のシム）と同じ動きをするか」なので、それを
直接測る。
"""

import shutil
import sys
from dataclasses import replace
import tempfile
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[3]))  # surge_mk2/

import numpy as np  # noqa: E402

from sim.vehicle import DEFAULT_SPEC_PATH, VehicleSpec  # noqa: E402
from tools.sysid import toml_update  # noqa: E402
from tools.sysid.analyze import analyze  # noqa: E402
from tools.sysid.bench import REALISTIC, TRUTH, reproduction_error, run_test, write_mcap  # noqa: E402

PLANNERS = {"steer": "sysid_steer", "corner": "sysid_corner",
            "accel": "sysid_accel", "latency": "sysid_latency"}


class TestEndToEnd(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.tmp = tempfile.TemporaryDirectory()
        d = Path(cls.tmp.name)
        cls.toml = d / "vehicle.toml"
        shutil.copy(DEFAULT_SPEC_PATH, cls.toml)
        paths = {}
        cls.truth = TRUTH
        for key, pid in PLANNERS.items():
            log, _, lat = run_test(pid, TRUTH, seed=11, noise=REALISTIC)
            paths[key] = d / f"{pid}.mcap"
            write_mcap(log, paths[key])
            if key == "latency":
                # 制御遅延の真値は、ベンチの処理経路（10Hzのスキャン・50Hzの中継・100Hzの
                # COMMAND）が実際に生んだ遅れ。`TRUTH.control_latency_s`（0）ではない
                cls.truth = replace(TRUTH, control_latency_s=float(np.median(lat)))
        cls.before = VehicleSpec.load(cls.toml)
        cls.res = analyze(paths, cls.before)
        toml_update.apply_dynamics(cls.toml, cls.res.results)
        cls.after = VehicleSpec.load(cls.toml)

    @classmethod
    def tearDownClass(cls):
        cls.tmp.cleanup()

    def test_every_test_was_analyzed(self):
        self.assertEqual(self.res.errors, [])
        for key in toml_update.ALLOWED_KEYS:
            self.assertIn(key, self.res.results, f"{key} が求まっていない")

    def test_toml_round_trip(self):
        for k, v in self.res.results.items():
            got = getattr(self.after, k)
            self.assertAlmostEqual(float(got), float(v), places=9, msg=k)

    def test_identified_sim_reproduces_truth(self):
        """同定した値のシムは、同定前（今の vehicle.toml）より桁違いに真値に近い応答を返す。"""
        for seed in (0, 1):
            e_after = reproduction_error(self.truth, self.after, seed=seed)
            e_before = reproduction_error(self.truth, self.before, seed=seed)
            # 絶対的な上限（学習の判断周期0.1sの中で問題にならない大きさ）
            self.assertLess(e_after["speed_rms"], 0.03, e_after)
            self.assertLess(e_after["steer_rms_deg"], 0.5, e_after)
            self.assertLess(e_after["yaw_rate_rms"], 0.05, e_after)
            self.assertLess(e_after["speed_obs_rms"], 0.03, e_after)
            self.assertLess(e_after["heading_1s_rms_deg"], 1.0, e_after)
            for k in e_after:
                self.assertLess(e_after[k], 0.5 * e_before[k] + 1e-6, f"{k}: 後 {e_after} 前 {e_before}")


if __name__ == "__main__":
    unittest.main()
