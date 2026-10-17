"""`raspi/tools/kf_consistency.py`（車速 KF の一貫性の検定）の検証。

検定器が正しく判定できなければ、実機の記録に当てても何も言えない（検証器を先に疑う）ので、
**真値の分かる合成データ**で「整合・保守的・過小見積もり・相関あり」を言い分けられることを確かめる。
ファームの C を使うテスト（シムの閉ループ）は `tools/ctrl_tune/tests/test_speed_kf.py`。
"""

from __future__ import annotations

import unittest

import numpy as np

from raspi.tools.kf_consistency import analyze, format_report, rows_to_arrays

DT = 0.01


def _record(nis_dist: float, nis_accel: float, rho_dist: float = 0.0, rho_accel: float = 0.0,
            seconds: float = 60.0, speed: float = 1.0, seed: int = 0) -> dict:
    """窓ごとの NIS を、平均が指定値・窓の中の本数（20 個・2 個）ぶんのばらつきで作る。"""
    rng = np.random.default_rng(seed)
    n = int(seconds / DT)
    t = np.arange(n) * DT

    def nis(mean: float, k: int) -> np.ndarray:           # 平均 mean の χ² を k 個ぶん平均した窓の値
        return rng.chisquare(k, n) / k * mean

    def ac(mean: float, k: int) -> np.ndarray:
        return mean + rng.normal(0, 1.0 / np.sqrt(k), n)   # 窓の平均の自己相関（k 個の積の平均）

    return {
        "t": t, "speed": np.full(n, speed), "accel_kf": np.zeros(n), "accel_bias_kf": np.zeros(n),
        "kf_gated": np.zeros(n), "kf_nis_dist": nis(nis_dist, 20), "kf_nis_accel": nis(nis_accel, 2),
        "kf_ac_dist": ac(rho_dist * nis_dist, 20) * 1.0, "kf_ac_accel": ac(rho_accel * nis_accel, 2),
    }


class TestKfConsistency(unittest.TestCase):
    def test_consistent_filter_is_judged_consistent(self):
        r = analyze(_record(1.0, 1.0), 0.0008, 0.6)["segments"]["全走行"]
        for ch in ("dist", "accel"):
            with self.subTest(ch=ch):
                self.assertEqual(r[ch]["nis_verdict"], "整合")
                self.assertFalse(r[ch]["correlated"])
                self.assertAlmostEqual(r[ch]["nis"], 1.0, delta=0.1)

    def test_conservative_filter_is_reported_without_a_sigma_value(self):
        """NIS<1 は保守的。**値は言わない**（S は HPHᵀ + R の和なので σ だけでは NIS が 1 にならない）。"""
        res = analyze(_record(0.25, 1.0), 0.0008, 0.6)
        r = res["segments"]["全走行"]["dist"]
        self.assertEqual(r["nis_verdict"], "保守的（誤差を大きく見積もりすぎ）")
        self.assertNotIn("sigma_suggest", r)
        self.assertIn("NIS<1", format_report(res))

    def test_overconfident_filter_is_detected(self):
        res = analyze(_record(1.0, 4.0), 0.0008, 0.6)
        r = res["segments"]["全走行"]["accel"]
        self.assertEqual(r["nis_verdict"], "過小見積もり（現実の誤差が主張より大きい）")
        self.assertIn("NIS>1", format_report(res))

    def test_correlated_residual_is_flagged_not_tuned(self):
        """周期誤差で残差が相関しているとき、NIS が小さくても σ を下げろとは言わない（下げると周期誤差を通す）。"""
        res = analyze(_record(0.05, 1.0, rho_dist=0.9), 0.0008, 0.6)
        r = res["segments"]["全走行"]["dist"]
        self.assertTrue(r["correlated"])
        self.assertNotIn("NIS<1", format_report(res))        # 相関があるときは NIS で調整させない
        self.assertIn("★残差が時間的に相関", format_report(res))

    def test_small_autocorrelation_is_not_flagged(self):
        r = analyze(_record(1.0, 1.0, rho_dist=0.05), 0.0008, 0.6)["segments"]["全走行"]["dist"]
        self.assertFalse(r["correlated"])

    def test_segments_by_speed(self):
        seg = analyze(_record(1.0, 1.0, speed=2.0))["segments"]
        self.assertGreater(seg["高速 1.5m/s〜"]["seconds"], 50)
        self.assertLess(seg["低速 0.02〜0.5m/s"]["seconds"], 1)
        self.assertIsNone(seg["静止"]["dist"])

    def test_gated_count_wraps_at_16bit(self):
        d = _record(1.0, 1.0, seconds=10)
        d["kf_gated"] = np.full(len(d["t"]), 65530.0)
        d["kf_gated"][len(d["t"]) // 2:] = 4.0            # 65530 → 4 = 巻き戻って +10
        self.assertEqual(analyze(d)["gated_total"], 10)

    def test_rows_to_arrays_restores_nis_and_autocorrelation(self):
        """STM32 は √NIS と相関係数で送る。NIS=0.0008（静止の前輪）が 0.001 刻みの RMS で潰れずに戻る。"""
        rows = [{"kf_z_rms_dist": 0.028, "kf_z_rms_accel": 1.2, "kf_rho_dist": 0.5, "kf_rho_accel": -0.1,
                 "speed": 0.0, "accel_kf": 0.0, "accel_bias_kf": 0.0, "kf_gated": 0, "steer_actual": 0.0,
                 "t_stm_us": 1_000_000 + 10_000 * i} for i in range(5)]
        d = rows_to_arrays(rows)
        self.assertAlmostEqual(d["kf_nis_dist"][0], 0.000784, places=6)
        self.assertAlmostEqual(d["kf_ac_dist"][0], 0.000392, places=6)
        self.assertAlmostEqual(d["kf_nis_accel"][0], 1.44, places=6)
        self.assertAlmostEqual(d["kf_ac_accel"][0], -0.144, places=6)
        self.assertAlmostEqual(d["t"][4], 0.04, places=9)

    def test_rows_to_arrays_rejects_old_records(self):
        with self.assertRaises(SystemExit):
            rows_to_arrays([{"speed": 0.0}])

    def test_format_report_runs(self):
        text = format_report(analyze(_record(1.0, 1.0), 0.0008, 0.6))
        self.assertIn("走行距離", text)
        self.assertIn("加速度", text)


if __name__ == "__main__":
    unittest.main()
