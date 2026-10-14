"""`toml_update.apply_dynamics` の入力検査と原子的な書き込み。

    cd surge_mk2 && .venv/bin/python -m pytest tools/sysid/tests/test_toml_update.py -q
"""

from __future__ import annotations

import shutil
import sys
import tempfile
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


class TestGuiWarnings(unittest.TestCase):
    def test_zero_brake_decel_is_not_suspicious(self):
        self.assertFalse(gui._is_suspicious("brake_decel_m_s2", 0.0))
        self.assertTrue(gui._is_suspicious("mu", 0.0))


if __name__ == "__main__":
    unittest.main()
