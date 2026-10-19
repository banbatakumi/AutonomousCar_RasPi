"""車速カルマンフィルタの調整値の最尤推定（`tools/calib/kf.py`）の確認。

    cd surge_mk2 && .venv/bin/python -m pytest tools/calib/tests/test_kf.py -q

1. Python のフィルタ（`kf_tune.replay`）は STM32 の `speed_kf.c`（ホストでコンパイル）と同じ車速を出す
2. フィルタのモデルそのもの（加速度のランダムウォーク・バイアスのランダムウォーク・白色の測定ノイズ）で作った
   記録から、真の調整値を復元する
3. 車速とともに増える加速度の雑音を、増え方まで復元する
"""

from __future__ import annotations

import ctypes as C
import math
import shutil
import sys
import unittest
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[3]))  # surge_mk2/

from tools.fwsim.fw import FW_DIR  # noqa: E402
from tools.calib import kf as kt  # noqa: E402


def synth(seed: int, T: float = 15.0, q: float = 3.0, qb: float = 0.02, ss: float = 0.0003,
          sa: float = 0.35, sa1: float = 0.0, k: float = 0.12, revert_s: float | None = None) -> kt.Series:
    """真値の記録（1kHz で動かして 100Hz で測る。`revert_s` を与えると加速度・車速が0へ戻る現実的な動き）。"""
    rng = np.random.default_rng(seed)
    dt = 0.001
    a = v = s = 0.0
    b = 0.05
    rows = []
    for i in range(int(T / dt)):
        t = i * dt
        a += math.sqrt(q * dt) * rng.standard_normal()
        if revert_s:
            a -= (a / revert_s + v / (3.0 * revert_s)) * dt
        v += a * dt
        s += v * dt
        b += math.sqrt(qb * dt) * rng.standard_normal()
        if i % 10 == 0:
            om = 1.5 * math.sin(2.1 * t) * math.sin(0.3 * t)
            od = round((s + ss * rng.standard_normal()) / 1e-4) * 1e-4
            acc = a + b - k * om * om + (sa + sa1 * abs(v)) * rng.standard_normal()
            rows.append((t, {"odom_dist": [od, od], "accel": [acc, 0.0, 0.0], "yaw_rate": om, "speed": v}))
    return kt.build_series(rows)


class TestReplayMatchesFirmware(unittest.TestCase):
    def test_same_speed_as_speed_kf_c(self):
        if shutil.which("cc") is None or not (FW_DIR / "src/control/speed_kf.c").exists():
            self.skipTest("cc かファームのリポジトリが無い")
        from tools.fwsim.kf_raw_replay import _Kf, build

        lib = build()
        p = kt.KfParams(sigma_s_m=0.0004, sigma_a_m_s2=0.3, sigma_a_per_m_s=0.2, jerk_psd=30.0,
                        bias_walk_psd=0.03, lever_arm_m=0.1)
        ser = synth(1, T=6.0, revert_s=1.0)
        kf = _Kf()
        lib.SpeedKf_Init(C.byref(kf))
        kf.cfg.sigma_s_m, kf.cfg.sigma_a_m_s2, kf.cfg.sigma_a_per_m_s = p.sigma_s_m, p.sigma_a_m_s2, p.sigma_a_per_m_s
        kf.cfg.jerk_psd, kf.cfg.bias_walk_psd = p.jerk_psd, p.bias_walk_psd
        kf.p[0] = p.sigma_s_m ** 2
        v_c = np.zeros(len(ser.t))
        for i in range(1, len(ser.t)):
            lib.SpeedKf_Predict(C.byref(kf), float(ser.t[i] - ser.t[i - 1]))
            lib.SpeedKf_UpdateDistance(C.byref(kf), float(ser.ds[i]))
            lib.SpeedKf_UpdateAccel(C.byref(kf), float(ser.accel[i] + p.lever_arm_m * ser.omega2[i]))
            v_c[i] = kf.x[1]                     # SpeedKf_GetSpeed はヘッダのインライン
        # 量子化の分散は STM32 の測定には無い（記録だけの話）
        v_py = kt.replay(ser, p, extra_s_var=0.0, keep=True).v
        # 差は float と double の丸めだけ
        np.testing.assert_allclose(v_py[1:], v_c[1:], atol=2e-4)


class TestRecovery(unittest.TestCase):
    def test_exact_model(self):
        truth = dict(q=3.0, qb=0.02, ss=0.0003, sa=0.35, k=0.12)
        ser = [synth(s, **truth) for s in range(2)]
        r = kt.estimate(ser, start=kt.KfParams(), fit_lever=True)
        p = r.params
        self.assertAlmostEqual(p.sigma_s_m / truth["ss"], 1.0, delta=0.1)
        self.assertAlmostEqual(p.sigma_a_m_s2 / truth["sa"], 1.0, delta=0.1)
        self.assertLess(p.sigma_a_per_m_s, 0.05)
        self.assertAlmostEqual(math.log(p.jerk_psd / truth["q"]), 0.0, delta=math.log(1.6))
        self.assertAlmostEqual(math.log(p.bias_walk_psd / truth["qb"]), 0.0, delta=math.log(2.5))
        self.assertAlmostEqual(p.lever_arm_m, truth["k"], delta=0.04)
        self.assertEqual(r.warnings, [])

    def test_speed_dependent_accel_noise(self):
        ser = [synth(10 + s, T=20.0, q=40.0, sa=0.2, sa1=0.3, revert_s=0.5) for s in range(2)]
        self.assertGreater(float(np.max(np.abs(np.concatenate([x.speed for x in ser])))), 1.0)
        r = kt.estimate(ser, start=kt.KfParams(), fit_lever=False)
        self.assertAlmostEqual(r.params.sigma_a_m_s2, 0.2, delta=0.06)
        self.assertAlmostEqual(r.params.sigma_a_per_m_s, 0.3, delta=0.08)
        self.assertNotIn("kf_lever_arm_m", r.values)


if __name__ == "__main__":
    unittest.main()
