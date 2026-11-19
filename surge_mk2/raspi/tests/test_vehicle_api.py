"""`raspi/core/vehicle.py` の読み口（`read_toml`・`section`・`Vehicle.from_*`）。"""

from __future__ import annotations

import tempfile
import tomllib
import unittest
from pathlib import Path

from raspi.core.vehicle import DEFAULT_PATH, Vehicle, read_toml, section


class TestVehicleReaders(unittest.TestCase):
    def test_from_toml_text_matches_load(self):
        text = DEFAULT_PATH.read_text(encoding="utf-8")
        self.assertEqual(Vehicle.from_toml_text(text), Vehicle.load())

    def test_from_toml_text_raises_instead_of_defaults(self):
        with self.assertRaises(tomllib.TOMLDecodeError):
            Vehicle.from_toml_text("wheelbase = 1/6")

    def test_from_provenance(self):
        text = DEFAULT_PATH.read_text(encoding="utf-8")
        self.assertEqual(Vehicle.from_provenance({"provenance": {"vehicle_toml": text}}), Vehicle.load())
        self.assertIsNone(Vehicle.from_provenance({}))
        self.assertIsNone(Vehicle.from_provenance({"provenance": {}}))

    def test_read_toml_raises_on_missing_file(self):
        with tempfile.TemporaryDirectory() as td:
            with self.assertRaises(OSError):
                read_toml(Path(td) / "none.toml")

    def test_section(self):
        d = {"control": {"plant": {"a": 1}}, "dynamics": 1}
        self.assertEqual(section(d, "control", "plant"), {"a": 1})
        self.assertEqual(section(d, "sim"), {})
        with self.assertRaises(ValueError):
            section(d, "dynamics")


if __name__ == "__main__":
    unittest.main()
