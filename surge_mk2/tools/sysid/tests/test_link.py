"""ステアのリンクの換算（★プロトコル v0.18）: 舵の効きの畳み込みと `vehicle.toml` への書き込み。

    cd surge_mk2 && .venv/bin/python -m pytest tools/sysid/tests/test_link.py -q
"""

from __future__ import annotations

import math
import shutil
import sys
import tempfile
import tomllib
import unittest
from dataclasses import replace
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[3]))

import numpy as np  # noqa: E402

from raspi.core.control_params import (STEER_LINK_X_MAX_RAD, load_control_params,  # noqa: E402
                                       steer_link_road)
from raspi.core.vehicle import Vehicle  # noqa: E402
from sim.vehicle import DEFAULT_SPEC_PATH, VehicleSpec  # noqa: E402
from tools.sysid import toml_update  # noqa: E402
from tools.sysid.analyze import LINK_KEYS, analyze, fold_steer_link  # noqa: E402
from tools.sysid.bench import TRUTH, run_test  # noqa: E402


class TestFoldSteerLink(unittest.TestCase):
    def test_identity_link_takes_the_fit_as_is(self):
        g, c = fold_steer_link((1.0, 0.0), 0.9928, -0.3917)
        self.assertAlmostEqual(g, 0.9928, places=9)
        self.assertAlmostEqual(c, -0.3917, places=9)

    def test_no_residual_keeps_the_link(self):
        g, c = fold_steer_link((0.97, -0.35), 1.0, 0.0)
        self.assertAlmostEqual(g, 0.97, places=9)
        self.assertAlmostEqual(c, -0.35, places=9)

    def test_composition_matches_the_true_direction(self):
        """換算ありの記録に残りが出たら、合成した向き（残り∘換算）を可動範囲で 0.05° 以内に表す。"""
        link, gain, cubic = (0.9928, -0.3917), 1.03, -0.08
        g, c = fold_steer_link(link, gain, cubic)
        x = np.linspace(-STEER_LINK_X_MAX_RAD, STEER_LINK_X_MAX_RAD, 101)
        d = steer_link_road(x, *link)
        want = gain * d + cubic * d ** 3
        self.assertLess(float(np.max(np.abs(steer_link_road(x, g, c) - want))), math.radians(0.05))


class TestApply(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.toml = Path(self.tmp.name) / "vehicle.toml"
        shutil.copy(DEFAULT_SPEC_PATH, self.toml)

    def tearDown(self):
        self.tmp.cleanup()

    def test_link_is_written_with_its_dependents(self):
        changed = toml_update.apply_dynamics(self.toml, {"steer_link_gain": 0.95, "steer_link_cubic": -0.3,
                                                         "mu": 0.5})
        self.assertLessEqual({"steer_link_gain", "steer_link_cubic", "max_steer", "mu"}, set(changed))
        with open(self.toml, "rb") as f:
            d = tomllib.load(f)
        self.assertEqual(d["control"]["steer_link_gain"], 0.95)
        self.assertEqual(d["control"]["steer_link_cubic"], -0.3)
        self.assertEqual(d["dynamics"]["steer_gain"], 1.0)
        self.assertEqual(d["dynamics"]["steer_gain_cubic"], 0.0)
        want = steer_link_road(STEER_LINK_X_MAX_RAD, 0.95, -0.3)
        self.assertAlmostEqual(d["max_steer"], want, places=12)
        # io_node が STM32 へ送る値・Pi とシムの車両諸元の両方に出る
        self.assertEqual(load_control_params(self.toml)["steer_link_cubic"], -0.3)
        self.assertAlmostEqual(Vehicle.load(self.toml).max_steer, want, places=12)
        self.assertAlmostEqual(VehicleSpec.load(self.toml).max_steer, want, places=12)

    def test_half_of_the_pair_is_rejected(self):
        before = self.toml.read_text(encoding="utf-8")
        with self.assertRaisesRegex(ValueError, "両方"):
            toml_update.apply_dynamics(self.toml, {"steer_link_gain": 0.95})
        with self.assertRaisesRegex(ValueError, "同時に書けません"):
            toml_update.apply_dynamics(self.toml, {"steer_link_gain": 0.95, "steer_link_cubic": -0.3,
                                                   "steer_gain": 0.9})
        self.assertEqual(self.toml.read_text(encoding="utf-8"), before)


class TestRepoToml(unittest.TestCase):
    def test_max_steer_follows_the_link(self):
        """リポジトリの vehicle.toml: `max_steer` はリンクの換算を可動範囲に通した値（手で片方だけ変えていない）。"""
        with open(DEFAULT_SPEC_PATH, "rb") as f:
            d = tomllib.load(f)
        link = (d["control"]["steer_link_gain"], d["control"]["steer_link_cubic"])
        self.assertAlmostEqual(d["max_steer"], steer_link_road(STEER_LINK_X_MAX_RAD, *link), places=9)
        # 可動範囲で単調（STM32 が路面舵角→モータ角を一意に解ける。`STEERING_LINKAGE_MIN_SLOPE`）
        self.assertGreater(link[0] + 3.0 * link[1] * STEER_LINK_X_MAX_RAD ** 2, 0.3)


class TestAnalyzeFolds(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.log, _, _ = run_test("sysid_steer", TRUTH, seed=3)
        cls.base = replace(TRUTH, steer_gain=1.0, steer_gain_cubic=0.0, steer_offset_rad=0.0)

    def test_fit_is_folded_onto_the_recorded_link(self):
        raw = analyze({"steer": self.log}, self.base, fold_link=False).results
        self.assertAlmostEqual(raw["steer_gain"], TRUTH.steer_gain, delta=0.03)

        # 換算なしで録った記録（v0.17 以前・シム）: 当てはめた効きがそのままリンクの換算になる
        res = analyze({"steer": self.log}, self.base)
        self.assertEqual(res.errors, [])
        self.assertNotIn("steer_gain", res.results)
        self.assertNotIn("steer_gain_cubic", res.results)
        self.assertAlmostEqual(res.results["steer_link_gain"], raw["steer_gain"], places=9)
        self.assertAlmostEqual(res.results["steer_link_cubic"], raw["steer_gain_cubic"], places=9)

        # 換算ありで録った記録: その換算の上に重ねる
        link = (0.98, -0.2)
        res = analyze({"steer": replace(self.log, steer_link=link)}, self.base)
        want = fold_steer_link(link, raw["steer_gain"], raw["steer_gain_cubic"])
        for key, value in zip(LINK_KEYS, want):
            self.assertAlmostEqual(res.results[key], value, places=9)

    def test_logs_with_different_links_are_rejected(self):
        res = analyze({"steer": self.log, "corner": replace(self.log, steer_link=(0.98, -0.2))}, self.base)
        self.assertTrue(any("リンクの換算が違います" in e for e in res.errors), res.errors)
        self.assertNotIn("steer_link_gain", res.results)


if __name__ == "__main__":
    unittest.main()
