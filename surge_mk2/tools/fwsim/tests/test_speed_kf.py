"""車速のカルマンフィルタ（STM32 `src/control/speed_kf.c`）の回帰テスト。ファームをホストでコンパイルして閉ループで回す。

車速は走行距離と加速度計のカルマンフィルタ**だけ**（前輪の微分+LPF は廃止）。守るもの:
- 実機型の前輪ノイズ（角度の量子化ノイズの差分）・周期誤差（1回転に2周期、速度の約2%）でも、車速が真値に近い
- 加速度計のバイアスを覚える
- 前輪の断線・周期誤差の大きい場合でも発散・NaN にならない
"""

from __future__ import annotations

import sys
import unittest
from dataclasses import replace
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[3]))

from tools.fwsim import scenarios as S  # noqa: E402
from tools.fwsim.fw import FW_DIR, OUT, Firmware  # noqa: E402
from tools.fwsim.plant import NOMINAL  # noqa: E402

#: 実機の静止実測（生の前輪角速度の標準偏差 2.7〜3.4 rad/s、隣り合う周期の相関 -0.45）に合わせた前輪
REAL_FRONT = replace(NOMINAL, front_noise_rad_s=3.0, front_position_noise=1)
#: 実機の走行記録で見えた前輪の周期誤差（1回転に2周期、速度の約 1.5〜2%）
RIPPLE = replace(REAL_FRONT, front_ripple=0.02, front_ripple_2x=1)


def _scenario(key: str) -> S.Scenario:
    return next(s for s in S.ALL if s.key == key)


def _rmse(out: np.ndarray) -> float:
    w = slice(int(0.3 / 5e-4), None)                           # 立ち上がりの 0.3s は捨てる
    return float(np.sqrt(np.mean((out[w, OUT["fw_speed"]] - out[w, OUT["speed"]]) ** 2)))


@unittest.skipUnless((FW_DIR / "src" / "control" / "speed_kf.c").exists(), "ファームに speed_kf が無い")
class TestSpeedKf(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.fw = Firmware()

    def test_speed_is_close_to_truth(self):
        for key in ("launch", "brake", "corner_exit", "cruise_turn"):
            with self.subTest(scenario=key):
                e = _rmse(S.run(self.fw, REAL_FRONT, _scenario(key)).out)
                self.assertLess(e, 0.004, f"RMSE {1e3 * e:.1f}mm/s")

    def test_speed_is_robust_to_encoder_ripple(self):
        """前輪の周期誤差（位置にして 0.2mm 前後）を正しい測定として通さない。走行距離の測定ノイズを静止実測
        （0.03mm）のままにすると周期誤差がそのまま速度の揺れになった（実機の走行記録 2026-10-16。調整値は speed_kf.h）。"""
        for key in ("launch", "brake", "corner_exit", "cruise_turn"):
            with self.subTest(scenario=key):
                e = _rmse(S.run(self.fw, RIPPLE, _scenario(key)).out)
                self.assertLess(e, 0.008, f"RMSE {1e3 * e:.1f}mm/s")

    def test_encoder_calibration_removes_ripple(self):
        """絶対角の読みに周期誤差（角度の関数）があるとき、正しい校正（`enc_*`）を入れると車速の誤差が下がり、
        恒等の校正（既定）は従来どおり（ファームの `lib/encoder_cal` を前輪の読みの経路で通す）。"""
        plant = replace(REAL_FRONT, front_ripple=0.06, front_ripple_2x=1, front_absolute_angle=1)
        sc = _scenario("cruise_turn")
        # toml の enc_* は実機の校正が入っている（恒等でない）ので、校正なし・ありとも全部を明示する
        ident = {f"enc_{side}_{k}": (1.0 if k == "hi" else 0.0)
                 for side in ("left", "right") for k in ("lo", "hi", "h1s", "h1c", "h2s", "h2c")}
        raw = _rmse(S.run(self.fw, plant, sc, ident).out)
        cal = _rmse(S.run(self.fw, plant, sc, {**ident, **plant.encoder_cal_params()}).out)
        clean = _rmse(S.run(self.fw, replace(plant, front_ripple=0.0), sc, ident).out)
        self.assertLess(cal, 0.5 * raw, f"校正なし {1e3 * raw:.2f} → 校正 {1e3 * cal:.2f}mm/s")
        self.assertLess(cal, clean * 1.3 + 2e-4, f"周期誤差なし {1e3 * clean:.2f}mm/s")

    def test_kf_estimates_accel_bias(self):
        """加速度計のバイアス（シムは 0.7m/s²）を静止中に覚える。"""
        out = S.run(self.fw, REAL_FRONT, _scenario("cruise")).out
        self.assertAlmostEqual(float(out[-1, OUT["fw_accel_bias"]]), 0.7, delta=0.15)

    def test_gate_stays_quiet_with_realistic_ripple(self):
        """周期誤差（2%）くらいでは測定を弱めない（実機の記録では調整前に毎秒 60〜110 回作動していた）。"""
        out = S.run(self.fw, RIPPLE, _scenario("cruise_turn")).out
        self.assertLess(float(out[-1, OUT["fw_kf_gated"]]), 50)

    def test_no_nan_when_front_encoder_drops_out(self):
        """前輪の断線（読みが 0）や周期誤差でも発散・NaN にならない。"""
        for plant in (REAL_FRONT, replace(REAL_FRONT, front_ripple=0.05), replace(REAL_FRONT, front_scale_error=-0.1)):
            for key in ("launch", "brake"):
                sc = _scenario(key)
                segs = tuple(replace(g, front_scale=0.0) if i == len(sc.segs) - 1 else g for i, g in enumerate(sc.segs))
                out = S.run(self.fw, plant, replace(sc, segs=segs)).out
                with self.subTest(scenario=key, front_ripple=plant.front_ripple):
                    # 診断の4列は測定の無い行が NaN（仕様）。それ以外は有限
                    diag = [OUT[k] for k in ("fw_nis_dist", "fw_nis_accel", "fw_ac_dist", "fw_ac_accel")]
                    self.assertTrue(np.isfinite(np.delete(out, diag, axis=1)).all())
                    self.assertTrue(np.isfinite(out[:, diag][~np.isnan(out[:, diag])]).all())
                    self.assertLess(np.abs(out[:, OUT["fw_speed"]]).max(), 20.0)


def _telemetry_windows(out: np.ndarray, rows: int = 20) -> dict:
    """シムの 2kHz の出力を、TELEMETRY と同じ 10ms の窓（20 行）に畳む（窓の中の NaN=測定なしは除く）。
    実機の `kf_consistency.load_mcap` が返すのと同じ形にして、検定器にそのまま渡す。"""
    n = (len(out) // rows) * rows
    o = out[:n].reshape(-1, rows, out.shape[1])

    def mean(k: str) -> np.ndarray:
        with np.errstate(all="ignore"):
            m = np.nanmean(o[:, :, OUT[k]], axis=1)
        return np.nan_to_num(m, nan=0.0)                  # 実機は測定が無い窓を 0 で送る

    return {"t": np.arange(len(o)) * 0.01, "speed": np.abs(o[:, -1, OUT["speed"]]),
            "accel_kf": np.zeros(len(o)), "accel_bias_kf": mean("fw_accel_bias"),
            "kf_gated": o[:, -1, OUT["fw_kf_gated"]],
            "kf_nis_dist": mean("fw_nis_dist"), "kf_nis_accel": mean("fw_nis_accel"),
            "kf_ac_dist": mean("fw_ac_dist"), "kf_ac_accel": mean("fw_ac_accel")}


@unittest.skipUnless((FW_DIR / "src" / "control" / "speed_kf.c").exists(), "ファームに speed_kf が無い")
class TestKfDiagnostics(unittest.TestCase):
    """TELEMETRY v0.26 の一貫性の診断（NIS・自己相関）が、雑音の性質を言い分けられること。

    これが言い分けられなければ、実機の記録に当てても調整値の良し悪しは分からない（検証器を先に疑う）。"""

    @classmethod
    def setUpClass(cls):
        from raspi.tools.kf_consistency import analyze
        cls.analyze = staticmethod(analyze)
        cls.fw = Firmware()

    def _dist(self, plant, key="cruise_turn"):
        out = S.run(self.fw, plant, _scenario(key)).out
        out = out[int(0.5 / 5e-4):]
        r = self.analyze(_telemetry_windows(out))["segments"]["全走行"]
        return r["dist"], r["accel"]

    def test_white_encoder_noise_is_white_and_conservative(self):
        """周期誤差の無い前輪: 残差は白色（自己相関≈0）で、σs=0.8mm に対し実際の雑音は小さいので NIS は 1 よりずっと小さい。"""
        dist, _ = self._dist(REAL_FRONT)
        self.assertFalse(dist["correlated"], f"相関係数 {dist['rho']:+.3f}")
        self.assertLess(dist["rho"], 0.5)       # 測定ノイズが過大だと、動きの追従遅れが残差に残って弱い相関は出る
        self.assertLess(dist["nis"], 0.1)

    def test_encoder_ripple_shows_up_as_correlated_residual(self):
        """前輪の周期誤差は、残差の自己相関として見える（白色雑音ではない）。これが測定ノイズを NIS で決められない理由。"""
        dist, _ = self._dist(replace(REAL_FRONT, front_ripple=0.04, front_ripple_2x=1))
        self.assertTrue(dist["correlated"], f"相関係数 {dist['rho']:+.3f}")
        self.assertGreater(dist["rho"], 0.8)

    def test_ripple_raises_nis_monotonically(self):
        """周期誤差が大きいほど NIS が増える（検定器の感度）。"""
        nis = [self._dist(replace(REAL_FRONT, front_ripple=r, front_ripple_2x=1))[0]["nis"] for r in (0.0, 0.02, 0.04)]
        self.assertLess(nis[0], nis[1])
        self.assertLess(nis[1], nis[2])

    def test_accel_nis_tracks_noise_ratio(self):
        """加速度計の雑音が仮定 σa=0.6 より小さければ NIS<1（シムは 0.15）、保守的と判定される。"""
        _, accel = self._dist(REAL_FRONT)
        self.assertLess(accel["nis"], 0.3)
        self.assertEqual(accel["nis_verdict"], "保守的（誤差を大きく見積もりすぎ）")


if __name__ == "__main__":
    unittest.main()
