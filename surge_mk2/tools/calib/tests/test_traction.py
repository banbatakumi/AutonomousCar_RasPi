"""校正 6〜8（後輪の空転・タイヤ・TC/ABS の確かめ）: 真値の分かっているシムで通す。"""

from __future__ import annotations

import unittest

from raspi.auto.calib_traction_check import CalibTractionCheck
from raspi.auto.calib_tyre import CalibTyre
from raspi.auto.calib_wheel import CalibWheel
from tools.calib import simcar, traction, traction_sim, tyre, wheel

R = 0.030


def consts_for(car: simcar.Car, wf) -> tyre.Consts:
    """校正 5・6 を真値の車で済ませた状態の値（校正 5 は真値の実効の値、校正 6 は解析の結果）。"""
    G = 1 / (R * (car.mass_kg + 2 * car.wheel_inertia_kgm2 / R ** 2))
    rr = car.rolling_decel_m_s2 + 2 * car.wheel_friction_nm * G
    return tyre.Consts(G=G, rr=rr, J=wf["control.plant.wheel_inertia_kgm2"], Tf=wf["control.plant.wheel_friction_nm"],
                       d=wf["control.plant.md_delay_s"], tau=wf["control.plant.md_tau_s"], mass=car.mass_kg)


def control_part(values: dict[str, float]) -> dict[str, float]:
    return {k.split(".", 1)[1]: v for k, v in values.items() if k.startswith("control.") and ".plant." not in k}


class TestTraction(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.car = simcar.Car()
        cls.enc = cls.car.enc_params()
        log, _ = simcar.run(CalibWheel(), cls.car, cls.enc, lifted=True)
        cls.wheel = wheel.analyze(log)
        cls.consts = consts_for(cls.car, cls.wheel.values)
        log, _ = simcar.run(CalibTyre(), cls.car, cls.enc)
        cls.tyre = tyre.analyze(log, cls.consts)

    def test_wheel_recovers_inertia_and_friction(self):
        self.assertNotEqual(self.wheel.verdict, "不合格", self.wheel.report())
        self.assertAlmostEqual(self.wheel.values["control.plant.wheel_inertia_kgm2"], self.car.wheel_inertia_kgm2,
                               delta=0.05 * self.car.wheel_inertia_kgm2)
        self.assertAlmostEqual(self.wheel.values["control.plant.wheel_friction_nm"], self.car.wheel_friction_nm, delta=3e-4)
        # 遅れの合計 = MD のトルク（むだ時間＋1次遅れ）＋MD の速度の報告＋ファームの後輪のローパス（4.75ms）
        lag = self.wheel.values["control.plant.md_delay_s"] + self.wheel.values["control.plant.md_tau_s"]
        truth = self.car.md_delay_s + self.car.md_tau_s + self.car.md_speed_tau_s + 0.00475
        self.assertAlmostEqual(lag, truth, delta=0.003)

    def test_tc_target_is_on_rising_side(self):
        """選んだ TC の目標は「山の 95% の力」なので、発進の距離が最良になる目標（山）より手前にあり、
        最良の 92% 以上は出る（残りは横のグリップのために残している。シムのタイヤは真の山がスリップ率 0.47 と
        試した範囲の外で、当てはめた曲線は山を手前に見積もるので 95% より少し下になる）。"""
        params = {**self.enc, **control_part(self.tyre.values)}
        chosen = traction_sim.launch(self.car, params).distance
        runs = {k: traction_sim.launch(self.car, {**params, "tc_slip_target": k}).distance
                for k in (0.06, 0.08, 0.10, 0.12, 0.15, 0.2, 0.3)}
        best_k = max(runs, key=runs.get)
        self.assertLess(params["tc_slip_target"], best_k)
        self.assertGreater(chosen, 0.92 * runs[best_k], (chosen, runs, self.tyre.report()))

    def test_designed_gains_are_robust(self):
        """設計したゲインは、4 倍のゲインより（山のあるタイヤでも）トルクが暴れず、発進も遅くない。"""
        from dataclasses import replace
        params = {**self.enc, **control_part(self.tyre.values)}
        hot = {**params, "tc_kp_nm_per_m_s": params["tc_kp_nm_per_m_s"] * 4, "tc_ki_nm_per_m": params["tc_ki_nm_per_m"] * 4}
        for car in (self.car, replace(self.car, tyre_c=1.6)):
            ok, bad = traction_sim.launch(car, params), traction_sim.launch(car, hot)
            self.assertLess(ok.chatter, bad.chatter)
            self.assertGreater(ok.distance, 0.99 * bad.distance)

    def test_check_passes_with_designed_values(self):
        params = {**self.enc, **control_part(self.tyre.values)}
        log, _ = simcar.run(CalibTractionCheck(), self.car, params)
        res = traction.analyze(log, self.car)
        self.assertEqual(res.verdict, "合格", res.report())
        self.assertGreater(res.values["limits.a_accel_m_s2"], 1.5)
        self.assertGreater(res.values["limits.a_brake_m_s2"], 1.5)

    def test_check_fails_with_oscillating_tc(self):
        params = {**self.enc, **control_part(self.tyre.values), "tc_kp_nm_per_m_s": 1.5, "tc_ki_nm_per_m": 60.0}
        log, _ = simcar.run(CalibTractionCheck(), self.car, params)
        res = traction.analyze(log, self.car)
        self.assertEqual(res.verdict, "不合格", res.report())


if __name__ == "__main__":
    unittest.main()
