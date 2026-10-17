"""`toml_update.apply_dynamics` の入力検査と原子的な書き込み。

    cd surge_mk2 && .venv/bin/python -m pytest tools/sysid/tests/test_toml_update.py -q
"""

from __future__ import annotations

import shutil
import sys
import tempfile
import tomllib
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[3]))

from sim.vehicle import DEFAULT_SPEC_PATH  # noqa: E402
from tools.sysid import gui, toml_update  # noqa: E402


class TestApplyGuards(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.toml = Path(self.tmp.name) / "vehicle.toml"
        shutil.copy(DEFAULT_SPEC_PATH, self.toml)
        self.before = self.toml.read_bytes()

    def tearDown(self):
        self.tmp.cleanup()

    def test_nan_and_inf_are_rejected_without_touching_the_file(self):
        for bad in (float("nan"), float("inf")):
            with self.assertRaises(ValueError):
                toml_update.apply_dynamics(self.toml, {"mu": bad})
        self.assertEqual(self.toml.read_bytes(), self.before)

    def test_link_out_of_firmware_range_is_rejected(self):
        for gain, cubic in ((1.6, 0.0), (0.4, 0.0), (1.0, 1.6), (1.0, -1.6)):
            with self.assertRaises(ValueError):
                toml_update.apply_dynamics(self.toml, {"steer_link_gain": gain, "steer_link_cubic": cubic})
        self.assertEqual(self.toml.read_bytes(), self.before)

    def test_link_in_range_is_written_and_leaves_no_temp_file(self):
        toml_update.apply_dynamics(self.toml, {"steer_link_gain": 1.01, "steer_link_cubic": -0.2})
        self.assertNotEqual(self.toml.read_bytes(), self.before)
        self.assertEqual([p.name for p in self.toml.parent.iterdir()], ["vehicle.toml"])


class TestDerivedControlValues(unittest.TestCase):
    """`[control]` のうち `[dynamics]` から式で決まる値は、同定を適用したときに書き直される。"""

    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.toml = Path(self.tmp.name) / "vehicle.toml"
        shutil.copy(DEFAULT_SPEC_PATH, self.toml)

    def tearDown(self):
        self.tmp.cleanup()

    def test_launch_exit_torque_follows_rolling_resistance_and_gain(self):
        changed = toml_update.apply_dynamics(self.toml, {"speed_plant_gain": 20.0, "rolling_resistance": 0.30})
        self.assertIn(toml_update.LAUNCH_EXIT_KEY, changed)
        control = tomllib.loads(self.toml.read_text(encoding="utf-8"))["control"]
        # 転がり抵抗を打ち消す後輪の合計トルク 0.30÷20 を、駆動輪 2 つで割る
        self.assertAlmostEqual(control[toml_update.LAUNCH_EXIT_KEY], 0.0075, places=6)
        self.assertAlmostEqual(control["speed_ff_rr_nm"], 0.015, places=6)

    def test_current_toml_is_consistent(self):
        """リポジトリの toml の値そのものが今の [dynamics] から決まる値になっている。"""
        d = tomllib.loads(Path(DEFAULT_SPEC_PATH).read_text(encoding="utf-8"))
        dyn, control = d["dynamics"], d["control"]
        self.assertAlmostEqual(control[toml_update.LAUNCH_EXIT_KEY],
                               toml_update.launch_exit_value(dyn["speed_plant_gain"], dyn["rolling_resistance"],
                                                             dyn["drive_ratio"]), places=6)
        ff = toml_update.speed_ff_values(dyn["speed_plant_gain"], dyn["rolling_resistance"])
        for key, value in ff.items():
            self.assertAlmostEqual(control[key], value, places=6)


class TestGuiWarnings(unittest.TestCase):
    def test_zero_brake_decel_is_not_suspicious(self):
        self.assertFalse(gui._is_suspicious("brake_decel_m_s2", 0.0))
        self.assertTrue(gui._is_suspicious("mu", 0.0))


if __name__ == "__main__":
    unittest.main()
