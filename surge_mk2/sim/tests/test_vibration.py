"""`sim/stm32.py` の走行中の揺れ（`SimParams.vib_*`）。

実機のテレメトリは走行中、約 11Hz で揺れる（`sim/params.py` の `REAL_VIBRATION`）。これを入れないシムで
決めたしきい値は実機で通用しなかった（横グリップの学習の誤検出、`PROGRESS.md` 2026-10-12）。
"""

import sys
import unittest
from dataclasses import replace
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

import numpy as np  # noqa: E402

from sim.course import Course  # noqa: E402
from sim.params import REAL_VIBRATION, SimParams  # noqa: E402
from sim.stm32 import VirtualStm32  # noqa: E402
from sim.vehicle import VehicleSpec  # noqa: E402

COURSE = Path(__file__).resolve().parents[1] / "courses" / "circuit_chicane_a.png"
NS = 1_000_000_000


def make(params: SimParams) -> VirtualStm32:
    return VirtualStm32(VehicleSpec.load(), Course.load(COURSE), params)


def corr(x: np.ndarray, lag: int) -> float:
    return float(np.corrcoef(x[:-lag], x[lag:])[0, 1])


class TestVibration(unittest.TestCase):
    def test_spectrum_matches_the_real_car(self):
        """分散 1、隣との相関はヨーレート・実舵角が 0.7 前後、車速が 0.4 前後（実機の記録の値）。"""
        sim = make(replace(SimParams(), **REAL_VIBRATION))
        x = np.asarray([sim._vibration(i * NS // 100) for i in range(1, 20001)])
        for k in range(3):
            self.assertAlmostEqual(float(np.std(x[:, k])), 1.0, delta=0.08)
        self.assertAlmostEqual(corr(x[:, 0], 1), 0.74, delta=0.05)
        self.assertAlmostEqual(corr(x[:, 2], 1), 0.74, delta=0.05)
        self.assertLess(corr(x[:, 0], 5), -0.1)                     # 約 11Hz で揺れる
        self.assertAlmostEqual(corr(x[:, 1], 1), 0.39, delta=0.06)
        self.assertLess(abs(corr(x[:, 1], 2)), 0.15)

    def test_scales_with_speed_and_is_off_by_default(self):
        t = NS
        off = make(SimParams())
        off.vehicle.speed = off.vehicle.speed_measured = 2.0
        state = off._vib_rng.getstate()
        tlm = off._telemetry(t)
        self.assertEqual(off._vib_rng.getstate(), state)            # 既定では乱数も引かない
        self.assertEqual(tlm.speed, off._telemetry(t + NS // 100).speed)

        on = make(replace(SimParams(), **REAL_VIBRATION, gyro_noise_dps=0.0, gyro_bias_dps=0.0))
        out = {}
        for v in (1.0, 3.0):
            on.vehicle.speed = on.vehicle.speed_measured = v
            rows = [on._telemetry(t + i * NS // 100) for i in range(4000)]
            out[v] = (np.std([r.yaw_rate for r in rows]), np.std([r.speed for r in rows]),
                      np.std([r.steer_actual for r in rows]))
        self.assertAlmostEqual(out[3.0][0] / out[1.0][0], 3.0, delta=0.4)   # ヨーレート ∝ 車速
        self.assertAlmostEqual(out[3.0][1] / out[1.0][1], 3.0, delta=0.4)   # 車速の揺れ ∝ 車速
        self.assertAlmostEqual(out[3.0][2] / out[1.0][2], 1.0, delta=0.15)  # 実舵角は車速に依らない


if __name__ == "__main__":
    unittest.main()
