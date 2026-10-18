"""⑤ 車速PI試験（`raspi/auto/sysid_speed_pi.py` → `tools/sysid/speed_pi.py`）と、車速PIのゲインの極配置
（`toml_update.speed_pi_values`）の確認。

    cd surge_mk2 && .venv/bin/python -m pytest tools/sysid/tests/test_speed_pi.py -q
"""

from __future__ import annotations

import math
import shutil
import sys
import tempfile
import tomllib
import unittest
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[3]))  # surge_mk2/

from raspi.auto.sysid_speed_pi import prbs  # noqa: E402
from sim.vehicle import DEFAULT_SPEC_PATH  # noqa: E402
from tools.sysid import speed_pi as sp  # noqa: E402
from tools.sysid import toml_update  # noqa: E402
from tools.sysid.bench import TRUTH, Pipeline, run_test  # noqa: E402


def _record(dead: float, tau: float, seed: int = 0) -> sp.Record:
    cap: dict = {}
    run_test("sysid_speed_pi", TRUTH, seed=seed, capture=cap, max_s=120,
             pipe=Pipeline(md_dead_s=dead, md_tau_s=tau))
    return sp.build_record(cap["vs"], cap["cmd"], (TRUTH.speed_kp, TRUTH.speed_ki))


class TestPlantRecovery(unittest.TestCase):
    def test_recovers_gain_resistance_and_delay(self):
        for dead, tau in ((0.004, 0.012), (0.0, 0.0)):
            p = sp.fit_plant(_record(dead, tau), 21.5, 0.27)
            self.assertAlmostEqual(p.gain / TRUTH.speed_plant_gain, 1.0, delta=0.02, msg=(dead, tau))
            self.assertAlmostEqual(p.rr_fwd, TRUTH.rolling_resistance, delta=0.02)
            self.assertAlmostEqual(p.rr_rev, TRUTH.rolling_resistance, delta=0.02)
            # TELEMETRY の分解能（`DEAD_TIME_UNCERTAINTY_S`）の内
            self.assertAlmostEqual(p.dead_s + p.lag_s, dead + tau, delta=sp.DEAD_TIME_UNCERTAINTY_S)

    def test_analyze_reports_gains(self):
        res = sp.analyze(_record(0.004, 0.012, seed=1), DEFAULT_SPEC_PATH)
        self.assertEqual(set(res.values), {"speed_plant_gain", "rolling_resistance"})
        self.assertEqual(res.warnings, [])
        self.assertTrue(any("ωn の上限" in n for n in res.notes))


class TestDesign(unittest.TestCase):
    def test_pole_placement(self):
        g = toml_update.speed_pi_values(20.0, 0.8, 4.0)
        self.assertAlmostEqual(g["speed_kp"], 2 * 0.8 * 4.0 / 20.0)
        self.assertAlmostEqual(g["speed_ki"], 16.0 / 20.0)
        # 閉ループの極: s² + G·kp·s + G·ki
        roots = np.roots([1.0, 20.0 * g["speed_kp"], 20.0 * g["speed_ki"]])
        np.testing.assert_allclose(np.abs(roots), 4.0, rtol=1e-6)
        np.testing.assert_allclose(-roots.real / np.abs(roots), 0.8, rtol=1e-6)

    def test_toml_reproduces_hand_gains(self):
        """`vehicle.toml` の狙いは、v0.26 までの手置きの kp 0.3・ki 0.6 を今の G で割り戻した値。"""
        d = tomllib.loads(DEFAULT_SPEC_PATH.read_text(encoding="utf-8"))["dynamics"]
        g = toml_update.speed_pi_values(d["speed_plant_gain"], d["speed_pi_zeta"], d["speed_pi_wn_rad_s"])
        self.assertAlmostEqual(g["speed_kp"], d["speed_kp"], delta=1e-4)
        self.assertAlmostEqual(g["speed_ki"], d["speed_ki"], delta=1e-4)

    def test_apply_rewrites_gains(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "vehicle.toml"
            shutil.copy(DEFAULT_SPEC_PATH, path)
            changed = toml_update.apply_dynamics(path, {"speed_plant_gain": 25.0})
            d = tomllib.loads(path.read_text(encoding="utf-8"))["dynamics"]
            want = toml_update.speed_pi_values(25.0, d["speed_pi_zeta"], d["speed_pi_wn_rad_s"])
            self.assertIn("speed_kp", changed)
            self.assertAlmostEqual(d["speed_kp"], want["speed_kp"])
            self.assertAlmostEqual(d["speed_ki"], want["speed_ki"])

    def test_margin_without_delay(self):
        """遅れの無い G/s に PI: 位相余裕 = atan(ωc·kp/ki)（ωc は |L|=1）。"""
        gain, kp, ki = 20.0, 0.3, 0.6
        pm, wc = sp.margins(gain, kp, ki, 0.0, 1e-6)
        self.assertAlmostEqual(abs(sp._loop(np.array([wc]), gain, kp, ki, 0.0, 1e-6)[0]), 1.0, delta=1e-3)
        self.assertAlmostEqual(pm, math.degrees(math.atan(wc * kp / ki)), delta=0.1)

    def test_wn_max_meets_margin(self):
        top = sp.wn_max(21.5, 0.9, 0.01, 0.01)
        g = toml_update.speed_pi_values(21.5, 0.9, top)
        pm, _ = sp.margins(21.5, g["speed_kp"], g["speed_ki"], 0.01, 0.01)
        self.assertAlmostEqual(pm, sp.PM_MIN_DEG, delta=0.5)
        # 遅れが大きいほど上限は下がる
        self.assertLess(sp.wn_max(21.5, 0.9, 0.03, 0.01), top)


class TestPrbs(unittest.TestCase):
    def test_m_sequence(self):
        bits = prbs(254)
        self.assertEqual(bits[:127], bits[127:])            # 周期 127
        self.assertEqual(sum(bits[:127]), 64)               # M 系列は 1 が 2^(n-1) 個
        self.assertEqual(prbs(20), prbs(20))


if __name__ == "__main__":
    unittest.main()
