"""校正 6・7・9（後輪の空転とゲイン・タイヤの肩・TC/ABS の確かめ）: 真値の分かっているシムで通す。"""

from __future__ import annotations

import unittest

from raspi.auto.calib_traction_check import CalibTractionCheck
from raspi.auto.calib_tyre import CalibTyre
from raspi.auto.calib_wheel import CalibWheel
from tools.calib import simcar, traction, traction_sim, tyre, wheel
from tools.calib.slip_gains import SLIP_TARGET, gains_for_inertia

R = 0.030


def consts_for(car: simcar.Car, wf) -> tyre.Consts:
    """校正 5・6 を真値の車で済ませた状態の値（校正 5 は真値の実効の値、校正 6 は解析の結果）。"""
    G = 1 / (R * (car.mass_kg + 2 * car.wheel_inertia_kgm2 / R ** 2))
    rr = car.rolling_decel_m_s2 + 2 * car.wheel_friction_nm * G
    return tyre.Consts(G=G, rr=rr, J=wf["control.plant.wheel_inertia_kgm2"], Tf=wf["control.plant.wheel_friction_nm"],
                       d=wf["control.plant.md_delay_s"], tau=wf["control.plant.md_tau_s"], mass=car.mass_kg)


#: 固定の目標スリップ率（`slip_gains.SLIP_TARGET`。校正では決めない）
TARGETS = {"tc_slip_target": SLIP_TARGET, "abs_slip_target": SLIP_TARGET}


def gain_part(inertia: float) -> dict[str, float]:
    """J から式で決まる TC・ABS のゲイン（校正 6 が書く値）。"""
    return gains_for_inertia(inertia)


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

    def test_wheel_writes_gains_from_inertia(self):
        """校正 6 は J から TC・ABS のゲインを書く（kp = ωc·J/r）。旧校正 8 の実機の値（J = 3.581e-5）を再現する。"""
        J = self.wheel.values["control.plant.wheel_inertia_kgm2"]
        for k, v in gain_part(J).items():
            self.assertAlmostEqual(self.wheel.values[f"control.{k}"], v, places=6)
        old = gain_part(3.581e-5)
        self.assertAlmostEqual(old["tc_kp_nm_per_m_s"], 0.14324, places=5)
        self.assertAlmostEqual(old["tc_ki_nm_per_m"], 2.1486, places=4)
        self.assertAlmostEqual(old["abs_kp_nm_per_m_s"], 0.07162, places=5)
        self.assertAlmostEqual(old["abs_ki_nm_per_m"], 1.0743, places=4)

    def test_fixed_tc_target_is_near_best(self):
        """固定の TC の目標は、発進の距離が最良になる目標の 92% 以上を出す（力は肩より先でほぼ平ら）。"""
        params = {**self.enc, **TARGETS, **gain_part(self.car.wheel_inertia_kgm2)}
        chosen = traction_sim.launch(self.car, params).distance
        runs = {k: traction_sim.launch(self.car, {**params, "tc_slip_target": k}).distance
                for k in (0.06, 0.08, 0.10, 0.12, 0.15, 0.2, 0.3)}
        best_k = max(runs, key=runs.get)
        self.assertGreater(chosen, 0.92 * runs[best_k], (chosen, runs, self.tyre.report()))

    def test_designed_gains_are_robust(self):
        """設計したゲインは、4 倍のゲインより（山のあるタイヤでも）トルクが暴れず、発進も遅くない。"""
        from dataclasses import replace
        params = {**self.enc, **TARGETS, **gain_part(self.car.wheel_inertia_kgm2)}
        hot = {**params, "tc_kp_nm_per_m_s": params["tc_kp_nm_per_m_s"] * 4, "tc_ki_nm_per_m": params["tc_ki_nm_per_m"] * 4}
        for car in (self.car, replace(self.car, tyre_c=1.6)):
            ok, bad = traction_sim.launch(car, params), traction_sim.launch(car, hot)
            self.assertLess(ok.chatter, bad.chatter)
            self.assertGreater(ok.distance, 0.99 * bad.distance)

    def test_check_with_designed_values_fails_only_on_low_mu_abs(self):
        params = {**self.enc, **TARGETS, **gain_part(self.car.wheel_inertia_kgm2)}
        log, _ = simcar.run(CalibTractionCheck(), self.car, params)
        res = traction.analyze(log, self.car)
        # このシムの車は μ 0.45 と低い。ABS の ωc を実機の値（60rad/s）に固定したので、低い μ では制動のスリップ率が目標 0.15 を
        # 超える（ωc 90 なら保てる。2026-11-18 に実機の値を残すと決めた）。不合格は ABS の目標の項目だけであること
        fails = [f.text for f in res.findings if f.level == "fail"]
        for t in fails:
            self.assertRegex(t, r"（ABS）: スリップ率の平均", res.report())
        self.assertFalse(any("発振" in f.text for f in res.findings if f.level == "fail"), res.report())
        self.assertGreater(res.values["limits.a_accel_m_s2"], 1.5)
        self.assertGreater(res.values["limits.a_brake_m_s2"], 1.5)

    def test_check_fails_with_oscillating_tc(self):
        params = {**self.enc, **TARGETS, **gain_part(self.car.wheel_inertia_kgm2),
                  "tc_kp_nm_per_m_s": 1.5, "tc_ki_nm_per_m": 60.0}
        log, _ = simcar.run(CalibTractionCheck(), self.car, params)
        res = traction.analyze(log, self.car)
        self.assertEqual(res.verdict, "不合格", res.report())


class TestTyreRamp(unittest.TestCase):
    """校正 7: TC・ABS を切ったトルクのランプ（`calib_tyre`）で、真値の分かっているタイヤの肩が測れること。"""

    @classmethod
    def setUpClass(cls):
        cls.car = simcar.Car()
        cls.enc = cls.car.enc_params()
        log, _ = simcar.run(CalibWheel(), cls.car, cls.enc, lifted=True)
        cls.consts = consts_for(cls.car, wheel.analyze(log).values)
        cls.log, cls.truth = simcar.run(CalibTyre(), cls.car, cls.enc, max_s=200)
        cls.res = tyre.analyze(cls.log, cls.consts)

    def _ramps(self, label):
        import numpy as np
        names = self.log.phase_of_samples()
        return np.array([i for i, n in enumerate(names) if n.startswith(label + "/")])

    def test_ramps_run_with_controllers_off_inside_the_corridor(self):
        import numpy as np
        self.assertTrue(self.log.finished, self.log.reasons[-3:])
        for label in ("dramp", "bramp"):
            idx = self._ramps(label)
            self.assertGreater(len(idx), 100, label)
            active = np.asarray(self.log.vs["tc_active"])[idx].astype(bool) | np.asarray(self.log.vs["abs_active"])[idx].astype(bool)
            self.assertLess(float(np.mean(active)), 0.02, f"{label}: TC・ABS が働いていた")
        d = self.truth.distance
        self.assertLess(float(d.max()), 2.1)
        self.assertGreater(float(d.min()), -0.2)

    def test_torque_ramps_up_slowly(self):
        """駆動のトルク指令は、ランプの間に（ほぼ）単調に増える。"""
        import numpy as np
        idx = self._ramps("dramp")
        tq = np.asarray(self.log.vs["torque_cmd"])[idx].mean(axis=1)
        self.assertGreater(float(tq.max()), 0.07)
        # 片道ごとに増え方を見る（区間の切れ目の下がりは除く）
        d = np.diff(tq)
        self.assertGreater(float(np.mean(d[np.abs(d) < 0.01] >= -1e-3)), 0.9)

    def test_knee_is_the_measured_slip_where_force_reaches_95_percent(self):
        """肩は測定したスリップ率のまま（遅れの補正をしない）で、力が最大の 95% に届く点。上限 0.3。"""
        for key in ("tc_knee", "abs_knee"):
            self.assertTrue(0.02 <= self.res.measured[key] <= tyre._TARGET_CAP, (key, self.res.measured[key]))

    def test_targets_and_gains_are_not_written_here(self):
        """目標スリップ率は固定、ゲインは校正 6 が書く。校正 7 はどちらも書かない。"""
        keys = [k for k in self.res.values if k.endswith(("_kp_nm_per_m_s", "_ki_nm_per_m", "_slip_target"))]
        self.assertEqual(keys, [], keys)

    def test_knee_is_found_for_a_symmetric_tyre(self):
        self.assertNotEqual(self.res.verdict, "不合格", self.res.report())
        # 真のタイヤ（B=15・C=1.1）の山の 95% は真のスリップ率 0.145。STM32 の読みは加速中に a·L/v だけ小さい（約 0.01〜0.03）
        # 測定したスリップ率の 95% 点（遅れの補正なし）なので、真の 0.145 より少し高く出る。山（0.46）の手前で、上限 0.3 より下
        for key in ("tc_knee", "abs_knee"):
            self.assertTrue(0.10 <= self.res.measured[key] <= 0.30, (key, self.res.measured[key]))
        ratio = [f.text for f in self.res.findings if "左右比" in f.text]
        self.assertTrue(ratio and "左/右 = 1.0" in ratio[0], ratio)

    def test_weak_left_wheel_is_reported(self):
        log, _ = simcar.run(CalibTyre(), self.car, self.enc, max_s=200, mu_scale=(0.6, 1.0))
        res = tyre.analyze(log, self.consts)
        self.assertNotEqual(res.verdict, "不合格", res.report())
        warns = " ".join(f.text for f in res.findings if f.level == "warn")
        self.assertIn("左後輪", warns, res.report())
        for key in ("tc_knee", "abs_knee"):
            # シムのタイヤは山が 0.46 と遠く平らで、95% に届く点は試験の細部（toml の車速 PI など）で動く
            self.assertTrue(0.04 < res.measured[key] <= tyre._TARGET_CAP, (key, res.measured[key], res.report()))

    def test_mu_is_physical_when_the_md_gives_less_torque(self):
        """MD のトルクが指令の 0.8 倍の車でも、体重計の質量で力を出すので μ の見積もりは同じ車（k = 1）と同じに出る
        （2026-11-18 までは指令の単位の質量で出していて 1/k 倍に出ていた）。k の車はシムでは質量・荷重を 1/k 倍して作る
        （`tools/fwsim/plant.py` の `in_command_units`）。"""
        import re
        from dataclasses import replace
        k = 0.8
        car_k = replace(self.car, mass_kg=self.car.mass_kg / k, fz_static_n=self.car.fz_static_n / k)
        log, _ = simcar.run(CalibWheel(), car_k, self.enc, lifted=True)
        c_k = consts_for(car_k, wheel.analyze(log).values)
        c_k = replace(c_k, mass=self.car.mass_kg)          # 体重計の質量（実際の値）
        self.assertAlmostEqual(c_k.k, k, delta=0.03)
        log, _ = simcar.run(CalibTyre(), car_k, self.enc, max_s=200)
        res_k = tyre.analyze(log, c_k)

        def mu(res):
            m = [re.search(r"μ(?: ≈ |（≥ )([0-9.]+)", f.text) for f in res.findings]
            return float(next(x for x in m if x).group(1))
        self.assertAlmostEqual(mu(res_k), mu(self.res), delta=0.05 * mu(self.res), msg=res_k.report())

    def test_fails_when_controllers_were_not_switched_off(self):
        from unittest import mock
        from raspi.auto import calib_tyre
        with mock.patch.dict(calib_tyre.TYRE_OVERRIDES, {}, clear=True):
            # toml の目標が高いと（最大トルクまで滑らせる目標）TC・ABS は働かず、切っていなくても見分けられない。低い目標で確かめる
            log, _ = simcar.run(CalibTyre(), self.car, {**self.enc, "tc_slip_target": 0.05, "abs_slip_target": 0.05}, max_s=200)
        res = tyre.analyze(log, self.consts)
        self.assertEqual(res.verdict, "不合格", res.report())


if __name__ == "__main__":
    unittest.main()
