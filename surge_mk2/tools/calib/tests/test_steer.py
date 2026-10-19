"""校正 2〜4（ステアの静特性・動特性、遅延）: 真値の分かっている Python のシムで通す。"""

from __future__ import annotations

import math
import unittest

import numpy as np

from raspi.auto.calib_latency import CalibLatency
from raspi.auto.calib_steer import CalibSteer
from raspi.auto.calib_steer_dyn import CalibSteerDyn
from raspi.core.control_params import STEER_LINK_X_MAX_RAD, load_control_params, steer_link_road
from tools.calib import latency, simsteer, steer

TRUTH = dict(steer_gain=0.97, steer_gain_cubic=-0.25, steer_offset_rad=math.radians(0.8), steer_servo_gain=0.975,
             dead_time_s=0.02, tau_steer_s=0.03, yaw_natural_freq_rad_s=25.0, yaw_damping=0.8)


def true_delta(x: np.ndarray) -> np.ndarray:
    """真値の車で、モータ角 x のときに実際に効く舵角（記録時の換算 → サーボの報告値 → 効き）。"""
    cp = load_control_params(strict=False)
    d = steer_link_road(x, cp["steer_link_gain"], cp["steer_link_cubic"])
    return TRUTH["steer_gain"] * d + TRUTH["steer_gain_cubic"] * d ** 3 + TRUTH["steer_offset_rad"]


class TestSteer(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.spec = simsteer.truth_spec(**TRUTH)

    def test_static_recovers_link(self):
        log = simsteer.run(CalibSteer(), self.spec)
        self.assertTrue(log.finished)
        res = steer.analyze_static(log)
        self.assertEqual(res.verdict, "合格", res.report())
        g, c = res.values["control.steer_link_gain"], res.values["control.steer_link_cubic"]
        d0 = res.values["dynamics.steer_offset_rad"]
        x = np.linspace(-STEER_LINK_X_MAX_RAD, STEER_LINK_X_MAX_RAD, 41)
        err = np.degrees(np.max(np.abs(g * x + c * x ** 3 + d0 - true_delta(x))))
        # 真値は「換算の 3 次式の上にさらに 3 次の効き」で、3 次式 1 本では表しきれない。その下限との差で見る
        # （残りの約 0.1° は、段が 7 つで段の間を 3 次式で補うぶん。最大舵角の曲率にして約 1.5%）
        X = np.column_stack([x, x ** 3, np.ones_like(x)])
        best, *_ = np.linalg.lstsq(X, true_delta(x), rcond=None)
        floor = np.degrees(np.max(np.abs(X @ best - true_delta(x))))
        self.assertLess(err, floor + 0.12, f"換算の誤差 {err:.3f}°（3 次式の下限 {floor:.3f}°）")
        self.assertAlmostEqual(res.values["dynamics.steer_servo_gain"], TRUTH["steer_servo_gain"], delta=0.005)

    def test_dynamic_recovers_delays(self):
        log = simsteer.run(CalibSteerDyn(), self.spec)
        self.assertTrue(log.finished)
        res = steer.analyze_dynamic(log)
        self.assertEqual(res.verdict, "合格", res.report())
        servo = res.values["dynamics.dead_time_s"] + res.values["dynamics.tau_steer_s"]
        self.assertAlmostEqual(servo, TRUTH["dead_time_s"] + TRUTH["tau_steer_s"], delta=0.006)
        yaw = 2 * res.values["dynamics.yaw_damping"] / res.values["dynamics.yaw_natural_freq_rad_s"]
        self.assertAlmostEqual(yaw, 2 * TRUTH["yaw_damping"] / TRUTH["yaw_natural_freq_rad_s"], delta=0.012)

    def test_latency(self):
        for lat in (0.012, 0.035):
            log = simsteer.run(CalibLatency(), simsteer.truth_spec(), latency_s=lat)
            res = latency.analyze(log)
            self.assertAlmostEqual(res.values["dynamics.control_latency_s"], lat, delta=0.002)


if __name__ == "__main__":
    unittest.main()
