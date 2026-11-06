"""アンダーステア勾配の学習（`nav/understeer.py`）と、それを入れた舵の換算（`SteerMap.understeer`）のテスト。"""

import json
import math
import random
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

import numpy as np  # noqa: E402

from raspi.nav.grip import GripMap  # noqa: E402
from raspi.nav.mpc import MPCConfig, MPCFollower  # noqa: E402
from raspi.nav.purepursuit import SteerMap  # noqa: E402
from raspi.nav.understeer import K_MAX, UndersteerConfig, UndersteerLearner  # noqa: E402
from raspi.tests.test_laptime import _line, stadium  # noqa: E402

L = 0.23


class TestSteerMap(unittest.TestCase):
    def test_zero_k_is_unchanged(self):
        a = SteerMap(gain=1.05, cubic=-0.3, servo_gain=0.98, offset=0.01)
        b = a._replace(understeer=0.0)
        for d in (-0.3, 0.0, 0.17):
            self.assertEqual(a.effective(d), b.effective(d, 3.0))
            self.assertEqual(a.command(d, 0.46), b.command(d, 0.46, 3.0))

    def test_matches_the_sim_curvature_and_inverts(self):
        """κ = tan(δ)/(L + K·v²)（`sim/vehicle.py` の `understeer_gradient` と同じ式）。command は effective の逆。"""
        k, v = 0.003, 3.0
        sm = SteerMap(gain=1.02, cubic=-0.2, servo_gain=1.0, offset=0.0, understeer=k, wheelbase=L)
        geo = sm._replace(understeer=0.0).effective(0.2)
        self.assertAlmostEqual(math.tan(sm.effective(0.2, v)) / L, math.tan(geo) / (L + k * v * v), places=9)
        for delta in (-0.2, 0.05, 0.15):
            cmd = sm.command(delta, 0.46, v)
            self.assertAlmostEqual(sm.effective(cmd, v), delta, places=5)
            # 速いほど、同じ曲率に要る舵は大きい
            self.assertGreater(abs(cmd), abs(sm.command(delta, 0.46, 0.0)))


def _drive(k_true, seconds=120.0, gyro_noise=0.08, seed=0, lag=0.03):
    """K の車で、車速と舵を揺らしながら走った記録を学習器に入れる（100Hz）。ジャイロに白色の雑音。"""
    rng = random.Random(seed)
    us = UndersteerLearner(SteerMap(wheelbase=L), lag_s=lag)
    hist = []
    t = 0.0
    while t < seconds:
        v = 2.5 + 1.0 * math.sin(2 * math.pi * t / 7.0)
        steer = 0.22 * math.sin(2 * math.pi * t / 3.1)
        hist.append((t, steer))
        # ヨーは舵の lag 前の値で決まる
        d_eff = next((s for ts, s in reversed(hist) if ts <= t - lag), 0.0)
        r = v * math.tan(d_eff) / (L + k_true * v * v) + rng.gauss(0.0, gyro_noise)
        us.observe(t, v, r, steer)
        t += 0.01
    return us


class TestLearner(unittest.TestCase):
    def test_recovers_k_despite_gyro_noise(self):
        """ジャイロの雑音は x（横加速度）と y の両方に入る。操作変数で偏らない（最小二乗なら小さく出る）。"""
        for k in (0.0, 0.002, 0.005):
            us = _drive(k)
            self.assertTrue(us.ready)
            self.assertAlmostEqual(us.k, k, delta=0.0004, msg=f"真値 {k}")

    def test_keeps_prior_until_enough_cornering(self):
        us = _drive(0.004, seconds=6.0)
        self.assertFalse(us.ready)
        us = UndersteerLearner(SteerMap(wheelbase=L), cfg=UndersteerConfig())
        us.reset(prior=0.0015)
        self.assertEqual(us.k, 0.0015)

    def test_clamps(self):
        us = _drive(0.03, seconds=120.0)
        self.assertEqual(us.k, K_MAX)


class TestGripMapJson(unittest.TestCase):
    def test_round_trip_and_old_files(self):
        m = GripMap(level=4.5, understeer=0.0021, understeer_s=33.0)
        back = GripMap.from_json(m.to_json())
        self.assertAlmostEqual(back.understeer, 0.0021)
        self.assertAlmostEqual(back.understeer_s, 33.0)
        raw = json.loads(m.to_json())
        del raw["understeer"], raw["understeer_s"]
        self.assertEqual(GripMap.from_json(json.dumps(raw)).understeer, 0.0)   # 古いファイル
        raw["understeer"] = 5.0
        self.assertEqual(GripMap.from_json(json.dumps(raw)).understeer, K_MAX)
        raw["understeer"] = float("nan")
        self.assertEqual(GripMap.from_json(json.dumps(raw)).understeer, 0.0)

    def test_k_learned_with_another_sampling_is_not_loaded(self):
        """標本の選び方が違う版（`K_FORMAT`）の K は読まない。2026-11-06 に偏った選び方で 0.52e-3 を保存した。"""
        raw = json.loads(GripMap(level=4.5, understeer=0.0021, understeer_s=33.0).to_json())
        del raw["understeer_fmt"]                         # 版の札が無い = 1（横グリップのゲートで選んでいた）
        old = GripMap.from_json(json.dumps(raw))
        self.assertEqual((old.understeer, old.understeer_s), (0.0, 0.0))
        self.assertAlmostEqual(old.level, 4.5)            # 横グリップの学習値はそのまま


class TestMpcWithUndersteer(unittest.TestCase):
    def _corner_bias(self, k_true, k_used, v=3.0):
        """K の車をスタジアム（半径 1.5m、a_y = 6m/s²）で MPC に追わせ、2 周目のコーナーの外向きの横偏差の平均 [m]。"""
        path = _line(stadium(r=1.5, straight=4.0), v=v)
        cfg = MPCConfig(wheelbase=L, delay_s=0.0, steer_map=SteerMap(wheelbase=L, understeer=k_used))
        mpc = MPCFollower()
        x, y, yaw = 0.0, -1.5, 0.0
        steer, dt, out = 0.0, 0.02, []
        lap = path.length / v
        for i in range(int(2 * lap / dt)):
            pp = mpc.follow(path, (x, y, yaw), v, steer, cfg, hint=-1, now=i * dt)
            steer = pp.steer
            x += v * math.cos(yaw) * dt
            y += v * math.sin(yaw) * dt
            yaw += v * math.tan(steer) / (L + k_true * v * v) * dt
            if i * dt > lap and abs(path.kappa[pp.index]) > 0.5:
                out.append(-pp.cross_track)          # 反時計回り: 外（右）が負
        return float(np.mean(out))

    def test_learned_k_removes_the_outward_bias(self):
        """K を知らないと膨らむぶんが、K を入れると K = 0 の車と同じ所まで戻る（MPC 自体の偏りは残る）。"""
        k = 0.003
        base = self._corner_bias(0.0, 0.0)
        biased = self._corner_bias(k, 0.0)
        fixed = self._corner_bias(k, k)
        self.assertGreater(biased - base, 0.01)      # K を知らないと外へ 1cm 以上余計に
        self.assertAlmostEqual(fixed, base, delta=0.002)


if __name__ == "__main__":
    unittest.main()
