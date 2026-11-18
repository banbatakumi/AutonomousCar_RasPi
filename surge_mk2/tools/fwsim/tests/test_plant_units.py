"""fwsim の車の単位: MD のトルク定数の誤差 k を入れて、指令あたりの加速度が校正 5 の G と一致すること。"""

from __future__ import annotations

import math
import tomllib
import unittest

from raspi.core.control_params import DEFAULT_TOML
from tools.calib.torque_scale import torque_scale
from tools.fwsim.plant import Plant

R = 0.030


class TestPlantUnits(unittest.TestCase):
    def test_torque_scale_formula(self):
        # 実際の質量 2.0kg・J_true 4e-5 の車を、トルクが指令の 0.8 倍の MD で測ると G・J_meas はこうなる
        k, m, j_true = 0.8, 2.0, 4e-5
        j_meas = j_true / k
        G = k / (R * (m + 2 * j_true / R ** 2))
        self.assertAlmostEqual(torque_scale(G, j_meas, m), k, places=9)
        self.assertTrue(math.isnan(torque_scale(G, 1.0, m)))     # 分母が負＝値の組がおかしい

    def test_loaded_plant_matches_identified_gain(self):
        """`Plant.load()` の車の「トルク指令 → 加速度」の利得が、toml の校正 5 の G と一致する。"""
        with open(DEFAULT_TOML, "rb") as f:
            d = tomllib.load(f)
        p = Plant.load()
        G_sim = 1.0 / (R * (p.mass_kg + 2 * p.wheel_inertia_kgm2 / R ** 2))
        self.assertAlmostEqual(G_sim, d["dynamics"]["speed_plant_gain"], delta=1e-6 * G_sim)

    def test_command_units_keep_tyre_limit_and_scale_body(self):
        """1/k 倍にしても、タイヤが出せる力 ÷ 質量（加速度の上限）と、静止荷重 ÷ 質量は変わらない。"""
        p = Plant(mass_kg=2.0, fz_static_n=5.75, mu=1.0)
        q = p.in_command_units(0.8)
        self.assertAlmostEqual(q.mu * q.fz_static_n / q.mass_kg, p.mu * p.fz_static_n / p.mass_kg)
        self.assertAlmostEqual(q.fz_static_n / q.mass_kg, p.fz_static_n / p.mass_kg)
        self.assertIs(p.in_command_units(float("nan")), p)


if __name__ == "__main__":
    unittest.main()
