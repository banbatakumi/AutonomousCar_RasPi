"""校正 1（前輪エンコーダ）: 真値の分かっているシムで試験→解析を通し、真値を復元するか。"""

from __future__ import annotations

import math
import unittest
from dataclasses import replace

import numpy as np

from raspi.auto.calib_encoder import CalibEncoder
from tools.calib import encoder, simcar


def corrected_error_deg(true_cal, fit_cal) -> float:
    """真の特性のエンコーダを `fit_cal` で直した角度と真の角度の差の、定数を除いた最大 [°]。

    lo は周期誤差の位相と区別できない（`encoder.py` の docstring）ので、校正の良し悪しは
    「直した角度が真の角度と定数しか違わないか」で測る。"""
    lo, hi, *h = true_cal
    theta = np.linspace(0, 2 * math.pi, 2000, endpoint=False)
    theta_m = theta + encoder.ripple(np.array(h), theta)
    u = lo + (hi - lo) * np.mod(theta_m, 2 * math.pi) / (2 * math.pi)
    got = encoder.cal_angle(u, fit_cal[0], fit_cal[1], fit_cal[2:])
    d = np.angle(np.exp(1j * (got - theta)))
    d -= np.angle(np.mean(np.exp(1j * d)))
    return float(np.degrees(np.max(np.abs(np.angle(np.exp(1j * d))))))


class TestEncoderCalibration(unittest.TestCase):
    #: STM32 の校正なし（シムは `vehicle.toml` の値を初期値に使うので、実機の校正が入っていても関係なく録る）
    NO_CAL = {f"enc_{s}_{k}": v for s in ("left", "right")
              for k, v in (("lo", 0.0), ("hi", 1.0), ("h1s", 0.0), ("h1c", 0.0), ("h2s", 0.0), ("h2c", 0.0))}

    def _run(self, car):
        log, _ = simcar.run(CalibEncoder(), car, self.NO_CAL)
        self.assertTrue(log.finished, log.reasons[-3:])
        return encoder.analyze(log)

    def _cal(self, res, side):
        return tuple(res.values[f"control.enc_{side}_{k}"] for k in ("lo", "hi", "h1s", "h1c", "h2s", "h2c"))

    def test_recovers_truth(self):
        car = simcar.Car()
        res = self._run(car)
        self.assertEqual(res.verdict, "合格", res.report())
        for side, true in (("left", car.enc_left), ("right", car.enc_right)):
            err = corrected_error_deg(true, self._cal(res, side))
            # 真値の周期誤差は ±1.2°・±3.2°（2026-10-18 の実機と同じ）。直した後に 0.05° 以内
            self.assertLess(err, 0.05, f"{side}: 直した角度の誤差 {err:.3f}°")
            self.assertAlmostEqual(res.values[f"control.enc_{side}_hi"] - res.values[f"control.enc_{side}_lo"],
                                   true[1] - true[0], delta=3e-4)

    def test_values_stay_in_firmware_range(self):
        """惰行が下端まで届かず、読みの最小値（= lo の置き場所）が真の lo より大きくても、求めた hi は
        STM32 の範囲（0.8〜1.0）に収まる。外れると ARM できない（実機で hi = 1.000655 が「範囲外で丸められた」になった）。"""
        base = simcar.Car()
        car = replace(base, enc_right=(0.0035, 0.9999) + tuple(base.enc_right[2:]))
        res = self._run(car)
        for side in ("left", "right"):
            lo, hi = self._cal(res, side)[:2]
            self.assertLessEqual(lo, encoder.LO_MAX)
            self.assertTrue(encoder.HI_MIN <= hi <= encoder.HI_MAX, f"{side}: hi={hi}")
            self.assertTrue(all(abs(h) <= encoder.H_MAX for h in self._cal(res, side)[2:]))
        err = corrected_error_deg(car.enc_right, self._cal(res, "right"))
        self.assertLess(err, 0.05, f"right: 直した角度の誤差 {err:.3f}°")

    def test_no_ripple_gives_identity(self):
        car = replace(simcar.Car(), enc_left=(0.0, 1.0, 0, 0, 0, 0), enc_right=(0.0, 1.0, 0, 0, 0, 0))
        res = self._run(car)
        for side in ("left", "right"):
            self.assertLess(encoder.amplitude_deg(self._cal(res, side)[2:]), 0.05)

    def test_firmware_applies_calibration(self):
        """解析した値を STM32（ホストでコンパイルしたファーム）に入れて録り直すと、走行距離から周期誤差が消える。"""
        car = simcar.Car()
        first = self._run(car)
        params = {k.split(".", 1)[1]: v for k, v in first.values.items()}
        log, _ = simcar.run(CalibEncoder(), car, params)
        second = encoder.analyze(log)
        post = [f.text for f in second.findings if "走行距離に残る周期誤差" in f.text]
        self.assertEqual(len(post), 2, second.report())
        self.assertTrue(all("効いている" in t for t in post), second.report())


if __name__ == "__main__":
    unittest.main()
