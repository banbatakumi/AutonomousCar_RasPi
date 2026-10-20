"""校正 6〜8（後輪の空転・タイヤ・TC/ABS の確かめ）: 真値の分かっているシムで通す。"""

from __future__ import annotations

import unittest

from raspi.auto.calib_tc_gain import KEYS, base_gains
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


def gain_part(inertia: float, wc: float = 120.0, ratio: float = 4.0) -> dict[str, float]:
    """校正 8 が決めるゲインの代わり（交差周波数 wc の基準のゲイン）。校正 7 は目標スリップ率だけを決める。"""
    kp, ki = base_gains(wc, ratio, inertia)
    return {k: v for pair in KEYS.values() for k, v in zip(pair, (kp, ki))}


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
        params = {**self.enc, **control_part(self.tyre.values), **gain_part(self.car.wheel_inertia_kgm2)}
        chosen = traction_sim.launch(self.car, params).distance
        runs = {k: traction_sim.launch(self.car, {**params, "tc_slip_target": k}).distance
                for k in (0.06, 0.08, 0.10, 0.12, 0.15, 0.2, 0.3)}
        best_k = max(runs, key=runs.get)
        self.assertLess(params["tc_slip_target"], best_k)
        self.assertGreater(chosen, 0.92 * runs[best_k], (chosen, runs, self.tyre.report()))

    def test_designed_gains_are_robust(self):
        """設計したゲインは、4 倍のゲインより（山のあるタイヤでも）トルクが暴れず、発進も遅くない。"""
        from dataclasses import replace
        params = {**self.enc, **control_part(self.tyre.values), **gain_part(self.car.wheel_inertia_kgm2)}
        hot = {**params, "tc_kp_nm_per_m_s": params["tc_kp_nm_per_m_s"] * 4, "tc_ki_nm_per_m": params["tc_ki_nm_per_m"] * 4}
        for car in (self.car, replace(self.car, tyre_c=1.6)):
            ok, bad = traction_sim.launch(car, params), traction_sim.launch(car, hot)
            self.assertLess(ok.chatter, bad.chatter)
            self.assertGreater(ok.distance, 0.99 * bad.distance)

    def test_check_passes_with_designed_values(self):
        params = {**self.enc, **control_part(self.tyre.values), **gain_part(self.car.wheel_inertia_kgm2)}
        log, _ = simcar.run(CalibTractionCheck(), self.car, params)
        res = traction.analyze(log, self.car)
        self.assertEqual(res.verdict, "合格", res.report())
        self.assertGreater(res.values["limits.a_accel_m_s2"], 1.5)
        self.assertGreater(res.values["limits.a_brake_m_s2"], 1.5)

    def test_check_fails_with_oscillating_tc(self):
        params = {**self.enc, **control_part(self.tyre.values), **gain_part(self.car.wheel_inertia_kgm2),
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

    def test_target_is_the_measured_slip_where_force_reaches_95_percent(self):
        """目標は測定したスリップ率のまま（遅れの補正をしない）で、力が最大の 95% に届く点。上限 0.3。"""
        for key in ("control.tc_slip_target", "control.abs_slip_target"):
            self.assertTrue(0.02 <= self.res.values[key] <= tyre._TARGET_CAP, (key, self.res.values[key]))

    def test_gains_are_not_designed_here(self):
        """ゲインは校正 8 が実機の閉ループで決める。校正 7 は目標スリップ率だけを出す。"""
        keys = [k for k in self.res.values if k.endswith(("_kp_nm_per_m_s", "_ki_nm_per_m"))]
        self.assertEqual(keys, [], keys)

    def test_knee_is_found_for_a_symmetric_tyre(self):
        self.assertNotEqual(self.res.verdict, "不合格", self.res.report())
        # 真のタイヤ（B=15・C=1.1）の山の 95% は真のスリップ率 0.145。STM32 の読みは加速中に a·L/v だけ小さい（約 0.01〜0.03）
        # 測定したスリップ率の 95% 点（遅れの補正なし）なので、真の 0.145 より少し高く出る。山（0.46）の手前で、上限 0.3 より下
        for key in ("control.tc_slip_target", "control.abs_slip_target"):
            self.assertTrue(0.10 <= self.res.values[key] <= 0.30, (key, self.res.values[key]))
        ratio = [f.text for f in self.res.findings if "左右比" in f.text]
        self.assertTrue(ratio and "左/右 = 1.0" in ratio[0], ratio)

    def test_weak_left_wheel_is_reported(self):
        log, _ = simcar.run(CalibTyre(), self.car, self.enc, max_s=200, mu_scale=(0.6, 1.0))
        res = tyre.analyze(log, self.consts)
        self.assertNotEqual(res.verdict, "不合格", res.report())
        warns = " ".join(f.text for f in res.findings if f.level == "warn")
        self.assertIn("左後輪", warns, res.report())
        for key in ("control.tc_slip_target", "control.abs_slip_target"):
            # シムのタイヤは山が 0.46 と遠く平らで、95% に届く点は試験の細部（toml の車速 PI など）で動く
            self.assertTrue(0.04 < res.values[key] <= tyre._TARGET_CAP, (key, res.values[key], res.report()))

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
