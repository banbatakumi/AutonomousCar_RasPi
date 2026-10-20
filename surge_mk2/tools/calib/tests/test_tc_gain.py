"""校正 8（TC・ABS のゲイン）と校正 7 の経験的な山の読み: 真値の分かっているシムで通す。"""

from __future__ import annotations

import math
import unittest
from unittest import mock

import numpy as np

from raspi.auto.calib_tc_gain import CalibTcGain, KEYS, OSC_FREQ_MIN_HZ, TEST_TARGETS, base_gains, oscillation
from tools.calib import simcar, tc_gain, tyre

R = 0.030


class TestBaseGains(unittest.TestCase):
    def test_kp_is_crossover_over_integrator_gain_and_ki_follows_ratio(self):
        J = 3.0e-5
        kp, ki = base_gains(80.0, 4.0, J)
        self.assertAlmostEqual(kp, 80.0 * J / R)
        self.assertAlmostEqual(ki, kp * 80.0 / 4.0)
        # 積分器 (r/J)/s に kp を掛けた一巡のゲインが交差で 1 になる（零点の分は ratio で小さく効く）
        self.assertAlmostEqual(kp * (R / J) / 80.0, 1.0)
        # 現行の kp 0.097 は ωc ≈ 80rad/s に当たる（J = 3.58e-5）
        self.assertAlmostEqual(base_gains(80.0, 4.0, 3.581e-5)[0], 0.0955, delta=0.002)

    def test_higher_ratio_means_weaker_integral(self):
        self.assertLess(base_gains(60.0, 8.0, 3e-5)[1], base_gains(60.0, 2.5, 3e-5)[1])


class TestOscillation(unittest.TestCase):
    def test_tonal_jitter_is_oscillation_and_slow_drift_is_not(self):
        n = np.arange(96)
        tone = 0.12 * np.sin(2 * math.pi * 25.0 * n * 0.01)
        std, share, freq = oscillation(tone)
        self.assertGreater(std, 0.04)
        self.assertGreater(share, 0.5)
        self.assertGreater(freq, OSC_FREQ_MIN_HZ)
        slow = 0.5 * (1 - np.cos(2 * math.pi * 1.5 * n * 0.01))      # ゆっくりした空転の広がり
        _, _, f_slow = oscillation(slow)
        self.assertLess(f_slow, OSC_FREQ_MIN_HZ)

    def test_quiet_series_is_not_oscillation(self):
        rng = np.random.default_rng(0)
        std, _, _ = oscillation(0.13 + 0.004 * rng.standard_normal(96))
        self.assertLess(std, 0.04)


class TestGainSweep(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.car = simcar.Car()
        cls.enc = cls.car.enc_params()
        cls.targets = {"tc": 0.13, "abs": 0.13}
        cls.J = cls.car.wheel_inertia_kgm2
        cls.consts = tyre.Consts.from_toml()
        cls.consts.J = cls.J
        cls.log, cls.truth = cls._run(cls.car, cls.targets)
        cls.res = tc_gain.analyze(cls.log, cls.consts)

    @classmethod
    def _run(cls, car, targets):
        params = {**cls.enc, "tc_slip_target": targets["tc"], "abs_slip_target": targets["abs"]}
        with mock.patch("raspi.auto.calib_tc_gain._read_inertia", return_value=cls.J):
            return simcar.run(CalibTcGain(), car, params, max_s=300)

    def test_runs_to_the_end_inside_the_corridor(self):
        self.assertTrue(self.log.finished, self.log.reasons[-3:])
        self.assertLess(float(self.truth.distance.max()), 2.1)

    def test_nominal_car_is_stable_up_to_the_top_of_the_ladder(self):
        self.assertNotEqual(self.res.verdict, "不合格", self.res.report())
        # 梯子の最大は 2 倍の余裕が確かめられないので採らない
        for kind, (kp_key, _) in KEYS.items():
            self.assertLess(self.res.values[f"control.{kp_key}"] * R / self.J, 240.0 - 1.0, kind)

    def test_chosen_gains_are_the_base_gains_for_a_ladder_crossover(self):
        for kind, (kp_key, ki_key) in KEYS.items():
            kp, ki = self.res.values[f"control.{kp_key}"], self.res.values[f"control.{ki_key}"]
            wc = kp * R / self.J
            ratio = kp * wc / ki
            self.assertIn(round(wc), (15, 30, 60, 120, 240), (kind, wc))
            self.assertTrue(any(abs(ratio - r) < 0.05 for r in (2.5, 4.0, 8.0)), (kind, ratio))

    def test_chosen_gains_pass_the_check_run(self):
        from raspi.auto.calib_traction_check import CalibTractionCheck
        from tools.calib import traction
        v = self.res.values
        params = {**self.enc, **TEST_TARGETS,
                  **{k.split(".", 1)[1]: x for k, x in v.items() if k.startswith("control.")}}
        log, _ = simcar.run(CalibTractionCheck(), self.car, params)
        res = traction.analyze(log, self.car)
        # 選んだゲインで TC は目標に保てて振動しない。ABS はシムのタイヤの平らな山で目標への追従が甘いことがある（発振はしない）
        self.assertNotIn("が発振している", " ".join(f.text for f in res.findings if f.level == "fail"), res.report())
        self.assertTrue(any("全開の発進（TC）: スリップ率が目標に保たれている" in f.text for f in res.findings), res.report())


class TestOscillatingLoop(unittest.TestCase):
    """MD の遅れが大きい車では、TC が高い交差周波数（240rad/s）で 9Hz 前後に発振する。選ぶのはその手前。"""

    @classmethod
    def setUpClass(cls):
        from dataclasses import replace
        cls.car = replace(simcar.Car(), md_delay_s=0.012, md_tau_s=0.012)
        cls.J = cls.car.wheel_inertia_kgm2
        cls.consts = tyre.Consts.from_toml()
        cls.consts.J = cls.J
        targets = {"tc": 0.13, "abs": 0.13}
        params = {**cls.car.enc_params(), "tc_slip_target": 0.13, "abs_slip_target": 0.13}
        with mock.patch("raspi.auto.calib_tc_gain._read_inertia", return_value=cls.J):
            cls.log, _ = simcar.run(CalibTcGain(), cls.car, params, max_s=300)
        cls.res = tc_gain.analyze(cls.log, cls.consts)

    def test_oscillating_tc_gain_is_found_and_not_chosen(self):
        self.assertIn("で発振", self.res.report())
        wc = self.res.values["control.tc_kp_nm_per_m_s"] * R / self.J
        self.assertLessEqual(wc, 120.0 + 1.0)
        self.assertGreaterEqual(wc, 29.9)

    def test_no_gain_is_written_when_nothing_tracks_the_target(self):
        """ABS はこの車では低い交差周波数で目標に保てず、60rad/s では発振する（保てる範囲が無い）→ 不合格で値を書かない。"""
        self.assertEqual(self.res.verdict, "不合格", self.res.report())
        self.assertNotIn("control.abs_kp_nm_per_m_s", self.res.values)

    def test_oscillating_gain_is_not_kept_for_later_legs(self):
        """発振を見つけたあとの片道は、発振したゲインへ戻らない（実機で発振したゲインで走り続けない）。"""
        import numpy as np
        wcs = [self.log.cmd_overrides[i].get("tc_kp_nm_per_m_s") for i in range(len(self.log.cmd_overrides))]
        used = sorted({round(w * R / self.J) for w in wcs if w})
        # 梯子の 240 は 1 回だけ（その後の片道は戻した値）
        n240 = sum(1 for a, b in zip(wcs, wcs[1:]) if a and b and round(a * R / self.J) != 240 and round(b * R / self.J) == 240)
        self.assertLessEqual(n240, 1, used)
        del np


class TestEmpiricalPeak(unittest.TestCase):
    @staticmethod
    def _points(curve, lam_max, n=60, a=2.0, v=1.0, noise=0.0, seed=0):
        rng = np.random.default_rng(seed)
        lam = np.linspace(0.005, lam_max, n)
        return [tyre.CurvePoint("drive", float("nan"), float(l), float(curve(l) + noise * rng.standard_normal()), v, float(l), float(l), a)
                for l in lam]

    def test_finds_the_peak_without_assuming_a_shape(self):
        # 山が 0.12 にあり、その先で力が 15% 落ちる曲線（標準の式に合わなくてよい）
        def f(l):
            return 8.0 * min(l / 0.12, 1.0) - (0.15 * 8.0 * (l - 0.12) / 0.2 if l > 0.12 else 0.0)
        pts = self._points(f, 0.32, noise=0.05)
        emp = tyre.empirical_peak(pts, lag=0.0)
        self.assertIsNotNone(emp)
        self.assertTrue(emp.has_peak)
        self.assertAlmostEqual(emp.target, 0.12, delta=0.04)

    def test_monotone_curve_has_no_peak_and_target_is_inside_range(self):
        pts = self._points(lambda l: 9.0 * (1 - math.exp(-l / 0.05)), 0.2, noise=0.05)
        emp = tyre.empirical_peak(pts, lag=0.0)
        self.assertFalse(emp.has_peak)
        self.assertLess(emp.target, 0.2)
        self.assertGreater(emp.target, 0.05)

    def test_too_few_points_gives_none(self):
        self.assertIsNone(tyre.empirical_peak(self._points(lambda l: l, 0.05, n=5), lag=0.0))

    def test_fixed_lag_is_used_by_fit_curve(self):
        pts = self._points(lambda l: 8.0 * math.sin(1.1 * math.atan(15.0 * l)), 0.3, noise=0.02)
        self.assertAlmostEqual(tyre.fit_curve(pts, lag=0.016).L, 0.016)


if __name__ == "__main__":
    unittest.main()
