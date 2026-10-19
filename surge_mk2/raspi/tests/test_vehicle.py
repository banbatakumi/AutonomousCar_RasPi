"""`raspi/core/vehicle.py` の読み手のテスト。**読めない・おかしい `vehicle.toml` で落ちないこと。**

io_node は import の時点で `Vehicle.load()` を呼ぶ（`CMD_TIMEOUT_NS`）。ここが例外を出すと
io_node が起動できず、systemd が再起動を繰り返す。

    python3 -m unittest discover -s raspi/tests -t .
"""

from __future__ import annotations

import contextlib
import io
import sys
import tempfile
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from raspi.core.vehicle import DEFAULT_PATH, Vehicle  # noqa: E402


def load(text: str) -> tuple[Vehicle, str]:
    """`text` を vehicle.toml として読む。`(Vehicle, stderr に出た警告)`。"""
    with tempfile.TemporaryDirectory() as d:
        p = Path(d) / "vehicle.toml"
        p.write_text(text)
        err = io.StringIO()
        with contextlib.redirect_stderr(err):
            v = Vehicle.load(p)
    return v, err.getvalue()


class TestLoad(unittest.TestCase):
    def test_the_real_file_loads(self):
        v = Vehicle.load(DEFAULT_PATH)
        self.assertGreater(v.wheelbase, 0.0)
        self.assertEqual(len(v.footprint), 4)

    def test_a_missing_file_gives_the_defaults(self):
        self.assertEqual(Vehicle.load("/nonexistent/vehicle.toml"), Vehicle())

    def test_broken_toml_gives_the_defaults(self):
        v, _ = load("wheelbase = = 1")
        self.assertEqual(v, Vehicle())

    def test_a_value_of_the_wrong_type_gives_the_defaults_with_a_warning(self):
        for text in ('wheelbase = "long"',
                     "control = 3",
                     "footprint = [1, 2]",
                     '[limits]\na_lat_m_s2 = "high"'):
            with self.subTest(text=text):
                v, warned = load(text)
                self.assertEqual(v, Vehicle())
                self.assertIn("既定値", warned)

    def test_values_are_read(self):
        v, warned = load("wheelbase = 0.3\n[safety]\ncmd_deadman_ms = 200\n")
        self.assertEqual((v.wheelbase, v.cmd_deadman_ms, warned), (0.3, 200.0, ""))


class TestDeadman(unittest.TestCase):
    """デッドマンの時間は安全の値。桁違い・0・負を黙って採らない。"""

    def test_out_of_range_values_fall_back_to_150ms(self):
        for raw in ("0", "-5", "150000", "nan", "inf"):
            with self.subTest(raw=raw):
                v, warned = load(f"wheelbase = 0.3\n[safety]\ncmd_deadman_ms = {raw}\n")
                self.assertEqual(v.cmd_deadman_ms, 150.0)
                self.assertEqual(v.wheelbase, 0.3)        # ほかの値まで捨てない
                self.assertIn("cmd_deadman_ms", warned)


if __name__ == "__main__":
    unittest.main(verbosity=2)
