"""校正 5（車速制御）: 真値の分かっているシムで「測る → 決める → 確かめる」を通す。"""

from __future__ import annotations

import unittest
from dataclasses import replace

from raspi.auto.calib_speed import CalibSpeed
from raspi.auto.calib_speed_check import CalibSpeedCheck
from tools.calib import simcar, speed, speed_check

R = 0.030


def effective_truth(car: simcar.Car) -> tuple[float, float]:
    """車速の制御から見た真の G と転がり抵抗（後輪 2 輪の回転の慣性・摩擦を含む）。"""
    m_eff = car.mass_kg + 2 * car.wheel_inertia_kgm2 / R ** 2
    G = 1.0 / (R * m_eff)
    rr = car.rolling_decel_m_s2 + 2 * car.wheel_friction_nm * G
    return G, rr


def controller_values(res) -> dict[str, float]:
    """解析の結果のうち、STM32 へ送る値（`[control]`・`speed_kp/ki`）。"""
    out = {}
    for k, v in res.values.items():
        sec, key = k.split(".", 1)
        if sec == "control" and not key.startswith("plant."):
            out[key] = v
        elif key in ("speed_kp", "speed_ki"):
            out[key] = v
    return out


class TestSpeed(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.car = simcar.Car()
        cls.enc = cls.car.enc_params()
        log, _ = simcar.run(CalibSpeed(), cls.car, cls.enc)
        cls.log = log
        cls.res = speed.analyze(log)

    def test_identifies_plant(self):
        self.assertTrue(self.log.finished)
        self.assertEqual(self.res.verdict, "合格", self.res.report())
        G, rr = effective_truth(self.car)
        self.assertAlmostEqual(self.res.values["dynamics.speed_plant_gain"], G, delta=0.02 * G)
        self.assertAlmostEqual(self.res.values["dynamics.rolling_resistance"], rr, delta=0.03)

    def test_design_follows_formula(self):
        G = self.res.values["dynamics.speed_plant_gain"]
        self.assertAlmostEqual(self.res.values["control.speed_ff_nm_per_m_s2"], 1 / G, places=5)
        d = speed.design_pi(G, 0.004, 0.004)
        self.assertGreaterEqual(d.pm_deg, speed.PM_TARGET_DEG - 0.5)
        self.assertLessEqual(d.wc, speed.WC_MAX + 1e-9)
        # 位相余裕を厳しくすると交差は下がる
        self.assertLess(speed.design_pi(G, 0.02, 0.01).wc, speed.design_pi(G, 0.002, 0.001).wc + 1e-9)

    def test_check_passes_with_designed_gains(self):
        G = self.res.values["dynamics.speed_plant_gain"]
        rr = self.res.values["dynamics.rolling_resistance"]
        model = replace(self.car, mass_kg=1 / (R * G) - 2 * self.car.wheel_inertia_kgm2 / R ** 2,
                        wheel_friction_nm=0.0, rolling_decel_m_s2=rr)
        params = {**self.enc, **controller_values(self.res)}
        log, _ = simcar.run(CalibSpeedCheck(), self.car, params)
        res = speed_check.analyze(log, model)
        self.assertNotEqual(res.verdict, "不合格", res.report())

    def test_check_fails_with_wrong_feedforward(self):
        """FF を 3 割少なく、積分も弱くすると、確かめの試験が落とす（検証器が誤りを見分けられること）。"""
        G = self.res.values["dynamics.speed_plant_gain"]
        bad = {**self.enc, **controller_values(self.res)}
        bad["speed_ff_nm_per_m_s2"] = 0.7 / G
        bad["speed_ki"] = 0.2
        bad["speed_kp"] = 0.2
        log, _ = simcar.run(CalibSpeedCheck(), self.car, bad)
        res = speed_check.analyze(log, self.car)
        self.assertEqual(res.verdict, "不合格", res.report())


if __name__ == "__main__":
    unittest.main()
