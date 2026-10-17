"""`raspi/tools/speed_est_compare.py`（車速の推定2通りの比較）の検証。

実機の記録は真値が無いので、**真値が分かる合成データ**で「基準の作り方・遅れの検出・倍率の取り除き方」が
合っていることを確かめる（検証器を先に疑う）。ファームの C やシムには依存しない（数値だけで作る）。
"""

from __future__ import annotations

import unittest

import numpy as np

from raspi.tools.speed_est_compare import analyze, format_report, reference_speed

DT = 0.01


def _profile() -> np.ndarray:
    """静止 → 加速 3m/s² → 定速 → 減速 -3m/s² → 静止（実機の記録と同じく停止から停止まで）。"""
    segs = [(1.5, 0.0), (1.0, 3.0), (2.0, 0.0), (1.0, -3.0), (1.5, 0.0), (1.0, 2.0), (1.5, 0.0), (1.0, -2.0), (1.5, 0.0)]
    a = np.concatenate([np.full(int(d / DT), x) for d, x in segs])
    # 実機の加減速は階段状ではない（モータの応答・MD の遅れでジャークが有限）。60ms の移動平均で丸める
    k = int(0.06 / DT)
    a = np.convolve(a, np.ones(k) / k, "same")
    v = np.maximum(np.cumsum(a) * DT, 0.0)
    return v


def _record(scale: float = 1.0, lpf_tau: float = 0.010, seed: int = 1) -> tuple[dict, np.ndarray]:
    rng = np.random.default_rng(seed)
    v = _profile()
    n = len(v)
    t = np.arange(n) * DT
    # 位置のスナップショット（台形則。長方形則だと位置が半サンプル進み、基準に 5ms の進みが出る）
    odom = np.r_[0.0, np.cumsum(0.5 * (v[1:] + v[:-1]) * DT)] * scale
    odom = np.round(odom, 4)                       # 0.1mm 刻み（実機の odom_dist）
    # LPF: 一次遅れ + 雑音 2mm/s。KF: 遅れなし + 雑音 1mm/s
    k = np.exp(-DT / lpf_tau)
    lpf = np.empty(n)
    a = 0.0
    for i in range(n):
        a = k * a + (1 - k) * v[i]
        lpf[i] = a
    lpf = (lpf + rng.normal(0, 0.002, n)) * scale
    kf = (v + rng.normal(0, 0.001, n)) * scale
    d = {
        "t": t, "t_stm_us": t * 1e6,
        "odom_dist": np.c_[odom, odom], "steer_actual": np.zeros(n),
        "speed_lpf": lpf, "speed_kf": kf, "speed": lpf, "accel_bias_kf": np.full(n, 0.7),
        "kf_gated": np.zeros(n), "tc_active": np.zeros(n), "tc_slip": np.zeros((n, 2)),
        "speed_source": np.zeros(n),
    }
    return d, v * scale


class TestSpeedEstCompare(unittest.TestCase):
    def test_reference_follows_truth_without_lag(self):
        d, truth = _record()
        ref = reference_speed(d["odom_dist"], d["steer_actual"], d["t"])
        e = (ref - truth)[50:-50]
        self.assertLess(np.sqrt(np.mean(e ** 2)), 0.004, "基準が真値から離れすぎ（雑音の床 数 mm/s）")
        # 遅れが無い: 真値に最も合うずらしは 0
        best = min(range(-5, 6), key=lambda L: np.mean((np.roll(ref, -L) - truth)[50:-50] ** 2))
        self.assertEqual(best, 0)

    def test_detects_lpf_lag_and_kf_accuracy(self):
        d, truth = _record()
        r = analyze(d)
        self.assertAlmostEqual(r["LPF"]["lag_ms"], 10.0, delta=5.0)     # τ=10ms の一次遅れ
        self.assertAlmostEqual(r["KF"]["lag_ms"], 0.0, delta=5.0)
        self.assertGreater(r["LPF"]["high_rmse_mmps"], 2.0 * r["KF"]["high_rmse_mmps"])   # 強い加減速で遅れの分だけ悪い
        self.assertLess(r["KF"]["still_std_mmps"], r["LPF"]["still_std_mmps"])
        # 真値から直接測った誤差と、ツールの値が近い（基準の雑音床ぶんは許す）
        e_lpf = (d["speed_lpf"] - truth)[50:-50]
        self.assertAlmostEqual(r["LPF"]["all_rmse_mmps"], 1e3 * np.sqrt(np.mean(e_lpf[np.abs(truth[50:-50]) > 0.02] ** 2)), delta=5.0)

    def test_common_scale_error_is_removed(self):
        """STM32 の時計が速い（倍率 1.0035）→ 推定器にも基準にも同じ倍率で出る。取り除いて比べる。"""
        d, _ = _record(scale=1.0035)
        r = analyze(d)
        # odom は倍率付き、推定器も倍率付き → 倍率は 1 のまま・遅れも変わらない
        self.assertAlmostEqual(r["scale_lpf_over_ref"], 1.0, delta=0.002)
        self.assertAlmostEqual(r["LPF"]["lag_ms"], 10.0, delta=5.0)
        # 推定器だけが倍率を持つ（時計の誤差）→ 倍率として出て、誤差には出ない
        d2, _ = _record()
        d2["speed_lpf"] = d2["speed_lpf"] * 1.0035
        d2["speed_kf"] = d2["speed_kf"] * 1.0035
        r2 = analyze(d2)
        self.assertAlmostEqual(r2["scale_lpf_over_ref"], 1.0035, delta=0.0015)
        self.assertLess(r2["KF"]["all_rmse_mmps"], 6.0)

    def test_kf_gated_wraps_at_16bit(self):
        """kf_gated は u16 で巻き戻る。65530 から数え始めて途中で 0 を越えても、増えた分を正しく数える。"""
        d, _ = _record()
        n = len(d["t"])
        true_count = 65530 + np.arange(n) // 100         # 100 サンプルごとに 1 増える
        d["kf_gated"] = (true_count % 65536).astype(float)
        r = analyze(d)
        self.assertEqual(r["kf_gated_total"], int(true_count[-1] - true_count[0]))
        self.assertGreater(r["kf_gated_total"], 6)         # 巻き戻りをまたいでいる

    def test_kf_only_record(self):
        """v0.25 以降の記録は speed が KF だけ（speed_lpf が無い）。KF の指標だけを出す。"""
        d, _ = _record()
        d.pop("speed_lpf")
        r = analyze(d)
        self.assertNotIn("LPF", r)
        self.assertLess(r["KF"]["still_std_mmps"], 2.0)
        self.assertAlmostEqual(r["scale_kf_over_ref"], 1.0, delta=0.002)
        text = format_report(r)
        self.assertIn("KF", text)
        self.assertNotIn("LPF", text.split("\n")[2])

    def test_by_source_and_report(self):
        d, _ = _record()
        n = len(d["t"])
        d["speed_source"] = (np.arange(n) >= n // 2).astype(float)
        r = analyze(d)
        self.assertEqual(sorted(r["by_source"]), [0, 1])
        text = format_report(r)
        self.assertIn("LPF", text)
        self.assertIn("KF", text)
        self.assertIn("speed_source", text)


if __name__ == "__main__":
    unittest.main()
