"""`raspi/nav/grip.py`（横グリップのリアルタイム最適化）の単体テスト。

車は「一定半径のコーナー 4 つを順に回る」だけの作り物: 舵は経路の曲率どおり、車速は学習した
上限から決まる速度、実際の曲率は真の限界で頭打ち。シムの車（`sim/vehicle.py`）と同じ形の限界。
"""

import json
import math
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

import numpy as np  # noqa: E402

from raspi.nav import grip  # noqa: E402
from raspi.nav import raceline as rl  # noqa: E402
from raspi.nav import speedctl  # noqa: E402

KAPPA = 1.0
DT = 0.01


class Track:
    """コーナー 4 つ（中心が 5m ずつ離れている）と、その間の直線。"""

    def __init__(self, learner: grip.GripLearner, limits=(5.0, 5.0, 5.0, 5.0),
                 noise: float = 0.0, seed: int = 0) -> None:
        """:param noise: 実機の走行中の揺れに対する倍率（1 = `sim.params.REAL_VIBRATION` 相当。
            ヨーレート σ 0.066rad/s × 車速、舵角から出る曲率 σ 0.06/m、どちらも約 11Hz）"""
        self.g = learner
        self.limits = list(limits)
        self.noise = noise
        self.rng = np.random.default_rng(seed)
        self._vib = [[0.0, 0.0], [0.0, 0.0]]
        self.t = 0.0
        self.max_over = 0.0
        self.capped = 0
        # 走り出しの直線 1 秒（学習は走っている間ずっと雑音の幅を測る。測る前は、実機の雑音を
        # 見込んだ粗いしきい値から始まる——`GripConfig.noise_prior`）
        self.straight(-1, seconds=1.0)

    def vib(self, i: int) -> float:
        """約 11Hz で揺れる雑音（分散 1）。白色雑音を 2 次の共振に通す（`sim/stm32.py` と同じ形）。"""
        if not self.noise:
            return 0.0
        r, th = 0.75, 2.0 * math.pi * 11.0 * DT
        a1, a2 = 2.0 * r * math.cos(th), -r * r
        gain = math.sqrt((1.0 + a2) * ((1.0 - a2) ** 2 - a1 * a1) / (1.0 - a2))
        x = self._vib[i]
        y = a1 * x[0] + a2 * x[1] + gain * self.rng.normal()
        x[1], x[0] = x[0], y
        return self.noise * y

    def corner(self, k: int, *, gate: str = "", a_x: float = 0.0, seconds: float = 1.2) -> None:
        g = self.g
        for i in range(int(seconds / DT)):
            x, y = 5.0 * k + 0.012 * i, 0.0
            v = math.sqrt(g.use * g.level_at(x, y) / KAPPA)
            if g.cap is not None:
                v = min(v, g.cap)
                self.capped += 1
            lim = math.sqrt(max(self.limits[k] ** 2 - a_x ** 2, 0.01))
            kap = KAPPA * min(1.0, (i + 1) * DT / 0.3)      # 舵は 0.3 秒かけて切り込む
            k_act = min(kap, lim / (v * v))
            self.max_over = max(self.max_over, v * v * kap / lim - 1.0)
            self.t += DT
            g.observe(self.t, x, y, v, v * k_act + 0.066 * v * self.vib(0), kap + 0.06 * self.vib(1),
                      a_x=a_x, gate=gate)

    def straight(self, k: int, seconds: float = 0.5) -> None:
        for i in range(int(seconds / DT)):
            self.t += DT
            self.g.observe(self.t, 5.0 * k + 2.0 + 0.03 * i, 1.0, 3.0, 0.066 * 3.0 * self.vib(0),
                           0.06 * self.vib(1))

    def laps(self, n: int, **kw) -> None:
        for _ in range(n):
            for k in range(4):
                self.corner(k, **kw)
                self.straight(k)
            self.g.lap(self.t)


def learner(init=3.5, **kw) -> grip.GripLearner:
    g = grip.GripLearner(yaw_wn=0.0)
    g.configure(init_level=init, max_level=kw.pop("max_level", 8.0),
                down_only=kw.pop("down_only", False), lat_use=kw.pop("lat_use", 1.0))
    return g


class TestLearner(unittest.TestCase):
    def test_raises_to_the_limit_and_stops(self):
        g = learner(3.5)
        tr = Track(g)
        tr.laps(12)
        self.assertTrue(g.converged)
        self.assertGreater(g.map.level, 0.93 * 5.0)
        self.assertLessEqual(g.map.level, 5.0)
        n = len(g.events)
        # 収束した後は上げない（限界を超え続けない）
        tr.max_over = 0.0
        tr.laps(5)
        self.assertEqual(len(g.events), n)
        self.assertLessEqual(tr.max_over, 0.0)

    def test_stops_even_when_the_precursor_is_never_detected(self):
        """★ 前兆として検出できないコーナーしか無くても、限界を超えたまま走り続けない:
        上限を上げても出る横加速度が増えなければ、出た最大を限界として止める。"""
        for plateau in (True, False):
            g = learner(3.5)
            g.cfg.dev_thr = 10.0                # 曲率の不足では検出しない
            if not plateau:
                g.cfg.plateau_cells = 10 ** 6   # 同じ場所の頭打ちでも検出しない
            tr = Track(g)
            tr.laps(12)
            self.assertTrue(g.converged, plateau)
            self.assertLessEqual(g.map.level, 5.0 * 1.001)
            self.assertGreater(g.map.level, 0.93 * 5.0)

    def test_one_corner_is_one_trial(self):
        """周の境目を待たずに上がる（基準版 `per_lap` は 1 周に 1 回）。"""
        fast, slow = learner(3.5), learner(3.5)
        slow.cfg.per_lap = True
        Track(fast).laps(2)
        Track(slow).laps(2)
        self.assertGreater(fast.map.level, slow.map.level * 1.15)
        self.assertGreater(slow.map.level, 3.5)

    def test_precursor_lowers_speed_at_once(self):
        g = learner(6.0)                        # 真の限界 5.0 より 20% 高い
        tr = Track(g)
        tr.corner(0, seconds=0.6)
        self.assertIsNotNone(g.cap)
        self.assertEqual(len(g.events), 1)
        self.assertTrue(g.events[0]["surprise"])
        self.assertLess(g.level_at(0.0, 0.0), 5.0)
        self.assertLess(g.map.level, 5.0)       # 想定外の低さは全体もすぐ下げる
        self.assertLess(g.cap, math.sqrt(5.0 / KAPPA))

    def test_low_corner_stays_local(self):
        g = learner(2.5)
        tr = Track(g, limits=(3.0, 5.0, 5.0, 5.0))
        tr.laps(14)
        self.assertLessEqual(g.level_at(0.2, 0.0), 3.0)
        self.assertGreater(g.level_at(5.2, 0.0), 0.93 * 5.0)
        self.assertGreater(g.level_at(10.2, 0.0), 0.93 * 5.0)

    def test_drop_after_convergence_relearns(self):
        g = learner(3.5)
        tr = Track(g)
        tr.laps(12)
        tr.limits = [4.0] * 4                   # 路面が滑りやすくなった
        tr.laps(8)
        self.assertLessEqual(g.map.level, 4.0)
        self.assertGreater(g.map.level, 0.9 * 4.0)

    def test_down_only_never_raises(self):
        g = learner(3.5, down_only=True)
        Track(g).laps(4)
        self.assertEqual(g.map.level, 3.5)
        g2 = learner(6.0, down_only=True)
        Track(g2).laps(4)
        self.assertLess(g2.map.level, 5.0)

    def test_max_level_is_respected(self):
        g = learner(3.5, max_level=4.2)
        Track(g).laps(8)
        self.assertLessEqual(g.map.level, 4.2)
        self.assertEqual(len(g.events), 0)

    def test_gate_stops_learning(self):
        g = learner(6.0)
        tr = Track(g)
        ver = g.ver
        tr.laps(2, gate="障害物")
        self.assertEqual((g.ver, len(g.events), g.trials), (ver, 0, 0))
        self.assertIsNone(g.cap)

    def test_lat_use_keeps_its_meaning(self):
        """9 割で走る設定でも、学習した限界は真の限界の近くに落ち着く（9 割に対して学習しない）。
        探している間は限界そのものまで出し、収束してから 9 割で走る。"""
        g = learner(3.5, lat_use=0.9)
        tr = Track(g)
        self.assertEqual(g.use, 1.0)
        tr.laps(16)
        self.assertTrue(g.converged)
        self.assertEqual(g.use, 0.9)
        self.assertGreater(g.map.level, 0.93 * 5.0)
        self.assertLessEqual(g.map.level, 5.0 * 1.001)
        tr.max_over = -1.0
        tr.laps(2)
        self.assertLess(tr.max_over, -0.05)     # 収束後は限界の 9 割強で走っている

    def test_down_only_always_uses_the_fraction(self):
        g = learner(4.0, lat_use=0.9, down_only=True)
        self.assertEqual(g.use, 0.9)
        tr = Track(g)
        tr.laps(3)
        self.assertEqual(len(g.events), 0)      # 4.0×0.9 は限界 5.0 より下
        self.assertEqual(g.map.level, 4.0)

    def test_lat_use_meaning_survives_a_binding_max_level(self):
        """★ 上限 `grip_max` がちょうど「限界÷割合」の辺りにあっても、限界の 9 割強へ戻る
        （sim.bench で、見積もりが上限に張り付いたまま限界いっぱいで走り続けた）。"""
        g = learner(3.5, lat_use=0.9, max_level=5.0 / 0.9)
        tr = Track(g)
        tr.laps(16)
        self.assertTrue(g.converged)
        self.assertLessEqual(g.map.level, 5.0 * 1.001)
        tr.max_over = -1.0
        tr.laps(2)
        self.assertLess(tr.max_over, -0.05)

    def test_sudden_drop_is_caught_while_turning_in_and_rescales_accel_and_decel(self):
        """★ 路面が急に 4 割下がった: 舵を切り込んでいる途中でも大きな不足は前兆にし、今出ている
        横加速度を採り、加速・減速も同じ割合で先に下げる。"""
        g = learner(init=5.0)
        g.long.configure(acc_init=3.0, dec_init=3.0, acc_max=3.0, dec_max=3.0, down_only=False)
        t, v = 0.0, 2.2
        for i in range(80):                             # 切り込み続ける（定常にならない）
            t += DT
            kap = 0.05 + 0.012 * i
            k_act = min(kap, 3.0 / (v * v))             # 真の限界 3.0（求めているのは 5.0 まで）
            g.observe(t, 0.02 * i, 0.0, v, v * k_act, kap)
        self.assertEqual(len(g.events), 1)
        e = g.events[0]
        self.assertTrue(e["surprise"])
        self.assertLess(e["b"], 3.3)                    # 変わる前の値を引きずらない
        self.assertLess(g.map.level, 3.1)
        self.assertLess(g.map.decel, 3.0 * 0.65)
        self.assertLess(g.map.accel, 3.0 * 0.65)
        self.assertTrue(g.long.probing)

    def test_combined_precursor_is_recorded_as_circle_radius(self):
        g = learner(6.0)
        tr = Track(g)
        tr.corner(0, a_x=2.0, seconds=0.6)
        e = g.events[0]
        self.assertEqual(e["kind"], "加速＋横")
        # 横に出ていた加速度（真の限界 5.0 の円の中で 4.58 まで）と前後 2.0 の合成で記録する
        self.assertLess(e["a"], 4.7)
        self.assertAlmostEqual(e["b"], math.hypot(e["a"], 2.0), places=6)

    def test_repeated_timestamp_is_ignored(self):
        """★ 同じ時刻の状態が続けて届いても、遅れの状態を捨てない（捨てると不足に見えて誤検出した）。"""
        g = learner(3.5)
        v = math.sqrt(3.5)
        t = 0.0
        for i in range(300):
            t += DT
            for _ in range(2 if i % 7 == 0 else 1):
                g.observe(t, 0.01 * i, 0.0, v, v * KAPPA, KAPPA)
        self.assertEqual(len(g.events), 0)

    def test_levels_follow_wall_clearance(self):
        g = learner(3.5)
        tr = Track(g)
        tr.corner(0)
        tr.straight(0)                          # 1 回通って上がった状態（conf < level）
        self.assertGreater(g.map.level, g.conf)
        xy = np.array([[50.0, 50.0], [51.0, 50.0]])
        ask, lv = g.levels(xy, wall=np.array([0.5, 0.02]))
        np.testing.assert_allclose(ask, g.use * lv)
        self.assertAlmostEqual(lv[0], g.map.level)
        self.assertLess(lv[1], lv[0])
        self.assertGreaterEqual(lv[1], g.conf)


DATA = Path(__file__).resolve().parent / "data"


class TestRealNoise(unittest.TestCase):
    """★ 実機の雑音で誤検出しない（2026-10-12: 滑っていない実機で前兆 46 回、横の上限が 1.0m/s² まで
    落ちて 1 周 5.7s → 13.1s になった。しきい値をシムの雑音だけで決めていた）。

    `data/grip_real_*.npz` は実機の記録から 100Hz の車速・ヨーレート・実舵角と 10Hz の姿勢だけを
    抜いたもの（`raspi/tools/grip_replay.py --export`）。どちらも車は滑っていない。
    """

    @classmethod
    def setUpClass(cls):
        from raspi.core.vehicle import Vehicle
        from raspi.tools import grip_replay
        cls.replay = grip_replay
        # 記録を取ったときの同定値に固定する（`config/vehicle.toml` が変わってもテストは変わらない）
        v = Vehicle.load()
        from dataclasses import replace
        cls.vehicle = replace(v, mu=0.4172, yaw_natural_freq_rad_s=58.5, yaw_damping=0.934,
                              steer_gain=1.0, steer_gain_cubic=0.0, steer_servo_gain=0.974,
                              steer_offset_rad=0.0028)

    def test_the_run_that_collapsed_has_no_precursor(self):
        """学習を入れて走った記録（前半 100 秒）。前兆 0・学習値は出発点より下がらない。"""
        rec = self.replay.load(DATA / "grip_real_20261012.npz")
        res = self.replay.replay(rec, self.vehicle)
        self.assertEqual([e["kind"] for e in res.learner.events], [])
        self.assertGreaterEqual(min(lv for _, lv in res.levels), res.levels[0][1] - 1e-9)
        self.assertGreater(res.seen_s, 20.0)
        # 上限が低かったとしても誤検出しない（しきい値の絶対値は上限に依らない）
        for pin in (1.8, 2.0, 2.5, 3.0, 3.9):
            res = self.replay.replay(rec, self.vehicle, pin=pin)
            self.assertEqual([e["kind"] for e in res.learner.events], [], pin)

    def test_clean_laps_at_high_lateral_acceleration_have_no_precursor(self):
        """横 4〜5.8m/s² で滑らずに周回した記録。上限をどこに置いても前兆 0。"""
        rec = self.replay.load(DATA / "grip_real_20261011.npz")
        for pin in (3.0, 4.0, 4.9, 5.5):
            res = self.replay.replay(rec, self.vehicle, pin=pin)
            self.assertEqual([e["kind"] for e in res.learner.events], [], pin)
            self.assertGreater(res.learner.trials, 10, pin)
        # 雑音の幅を測れている（シムの既定の雑音では 0.01 未満）
        self.assertGreater(res.learner.map.noise, 0.02)

    def test_synthetic_noise_does_not_lower_the_level(self):
        """限界の 2 割下を、実機相当の雑音つきで周回する。前兆を出さず、下がらない。"""
        for seed in range(3):
            g = learner(4.0, max_level=4.0)
            tr = Track(g, noise=1.0, seed=seed)
            tr.laps(12)
            self.assertEqual(len(g.events), 0, seed)
            self.assertEqual(g.map.level, 4.0, seed)

    def test_threshold_does_not_shrink_with_the_level(self):
        """★ 上限が下がっても、しきい値の絶対値は雑音より下がらない（下がると、下げるほど誤検出が
        増えて戻れなくなる）。上限 1.5m/s² に固定して同じ雑音で走っても前兆 0。"""
        g = learner(1.5, max_level=1.5)
        tr = Track(g, noise=1.0)
        tr.laps(12)
        self.assertEqual(len(g.events), 0)

    def test_limit_is_still_found_under_noise(self):
        """雑音があっても、限界を大きく超えたまま走り続けない（コーナー 1 つぶんの平均で拾う）。"""
        for seed in range(3):
            g = learner(3.5)
            tr = Track(g, noise=1.0, seed=seed)
            tr.laps(25)
            self.assertLess(g.map.level, 5.0 * 1.10, seed)
            self.assertGreater(g.map.level, 5.0 * 0.80, seed)
            tr.max_over = 0.0
            tr.laps(5)
            self.assertLess(tr.max_over, 0.10, seed)

    def test_floor_stops_a_runaway(self):
        """測り違いが重なっても、出発点の `floor` 倍より下へは下げない。"""
        g = learner(4.0)
        for k in range(6):
            g._precursor(1.0 + k, 3.0 * k, 0.0, 1.5, 1.0, 0.0, 0.9, 4.0, False, 0.3)
            g.cap = None
        self.assertEqual(g.map.level, g.cfg.floor * 4.0)
        self.assertGreaterEqual(g.level_at(0.0, 0.0), g.cfg.floor * 4.0)

    def test_small_shortfall_does_not_rescale_accel_and_decel(self):
        """雑音が大きいとき、小さな不足 1 回の「想定外」では加速・減速まで下げない（大きな不足・
        横偏差のときだけ）。雑音が小さければ下げる（路面が変わった直後の制動に間に合わせる）。"""
        quiet = learner(5.0)
        quiet.long.configure(acc_init=3.0, dec_init=3.0, acc_max=3.0, dec_max=3.0, down_only=False)
        quiet.map.noise, quiet.map.noise_n = 0.003, 1000
        quiet._precursor(1.0, 0.0, 0.0, 2.0, 3.0, 0.0, 0.93, 5.0, False, 0.07)
        self.assertLess(quiet.map.decel, 3.0)
        g = learner(5.0)
        g.map.noise, g.map.noise_n = 0.04, 1000
        g.long.configure(acc_init=3.0, dec_init=3.0, acc_max=3.0, dec_max=3.0, down_only=False)
        g._precursor(1.0, 0.0, 0.0, 2.0, 3.0, 0.0, 0.93, 5.0, False, 0.07)
        self.assertTrue(g.events[0]["surprise"])
        self.assertEqual((g.map.accel, g.map.decel), (3.0, 3.0))
        g.cap = None
        g._precursor(2.0, 5.0, 0.0, 2.0, 3.0, 0.0, 0.7, 5.0, False, 0.3)      # 大きな不足
        self.assertLess(g.map.decel, 3.0)


class TestGripMap(unittest.TestCase):
    def test_noise_is_saved(self):
        g = learner(4.0, max_level=4.0)
        Track(g, noise=1.0).laps(3)
        m = grip.GripMap.from_json(g.map.to_json())
        self.assertAlmostEqual(m.noise, g.map.noise, places=4)
        self.assertGreater(m.noise_n, 0)
        # 古い形式（雑音の欄なし）も読める
        old = json.loads(g.map.to_json())
        del old["noise"], old["noise_n"]
        self.assertEqual(grip.GripMap.from_json(json.dumps(old)).noise, 0.0)

    def test_roundtrip(self):
        g = learner(3.5)
        Track(g).laps(6)
        m = grip.GripMap.from_json(g.map.to_json())
        self.assertAlmostEqual(m.level, g.map.level, places=2)
        self.assertEqual(len(m.cells), len([c for c in g.map.cells.values() if c.ok or c.bad or c.n]))

    def test_values_are_clamped(self):
        text = json.dumps({"format": grip.FORMAT, "cell_m": grip.CELL_M, "level": 99.0, "ok": 50.0,
                           "bad": float("nan"), "cells": [[0, 0, 80.0, 0.0, 9.0, 3],
                                                           [1, 0, -5.0, 0.2, 0.0, 0]]})
        m = grip.GripMap.from_json(text.replace("NaN", "null"))
        self.assertEqual(m.level, grip.A_CEIL)
        self.assertEqual(m.bad, 0.0)
        self.assertEqual(m.cells[(0, 0)].ok, grip.A_CEIL)
        self.assertEqual(m.cells[(0, 0)].base, 0.5)
        self.assertEqual(m.cells[(1, 0)].ok, 0.0)
        self.assertEqual(m.cells[(1, 0)].bad, grip.A_FLOOR)

    def test_wrong_format_is_rejected(self):
        for text in ('{"format": 0, "cell_m": 0.25, "cells": []}',
                     '{"format": 1, "cell_m": 0.5, "cells": []}',
                     '{"format": 1, "cell_m": 0.25, "cells": [[1, 2]]}', "[]", "{"):
            with self.assertRaises(ValueError):
                grip.GripMap.from_json(text)

    def test_load_restarts_a_little_lower_and_recovers(self):
        """保存した値はそのまま信じない: 少し下げた所から再開し、通ったら戻る。"""
        g = learner(3.5)
        Track(g).laps(12)
        saved = g.map.level
        g2 = learner(3.5)
        g2.load(grip.GripMap.from_json(g.map.to_json()))
        self.assertAlmostEqual(g2.map.level, saved * g2.cfg.back_off, places=2)
        tr = Track(g2)
        tr.laps(3)
        self.assertGreater(g2.map.level, 0.97 * saved)
        self.assertEqual(len(g2.events), 0)     # 路面が同じなら、前兆を出さずに戻る
        # 保存した後に滑りやすくなっていた場合
        g3 = learner(3.5)
        g3.load(grip.GripMap.from_json(g.map.to_json()))
        tr = Track(g3, limits=(4.0,) * 4)
        tr.laps(8)
        self.assertLessEqual(g3.map.level, 4.0)


class TestLaunchFit(unittest.TestCase):
    def test_slope_from_tc_window(self):
        f = grip.LaunchFit(offset=0.27)
        out = [f.feed(0.01 * i, 0.3 + 2.5 * 0.01 * i, True, 0.0) for i in range(60)]
        got = [a for a in out if a is not None]
        self.assertAlmostEqual(got[0], 2.5 + 0.27, delta=0.05)     # 傾き＋転がり抵抗
        self.assertAlmostEqual(f.value, 2.77, delta=0.05)

    def test_intermittent_tc_is_not_fitted_across_the_gaps(self):
        """★ TC は断続的に介入する。途切れ（加速が落ちている間）をまたいで直線を引かない。"""
        f = grip.LaunchFit()
        t, v, got = 0.0, 0.3, []
        for burst in range(4):
            for _ in range(12):                 # 0.12 秒 TC が介入（2.5m/s²）
                t += 0.01
                v += 2.5 * 0.01
                got.append(f.feed(t, v, True, 0.0))
            for _ in range(10):                 # 0.1 秒 途切れて車速が落ちる
                t += 0.01
                v -= 0.01
                got.append(f.feed(t, v, False, 0.0))
        got = [a for a in got if a is not None]
        self.assertTrue(got)
        self.assertAlmostEqual(got[0], 2.5, delta=0.1)

    def test_short_or_steered_window_is_ignored(self):
        f = grip.LaunchFit()
        for i in range(20):                     # 0.2 秒しか続かない
            f.feed(0.01 * i, 0.3 + 2.5 * 0.01 * i, True, 0.0)
        self.assertIsNone(f.feed(0.3, 1.0, False, 0.0))
        for i in range(60):                     # 舵を切っている
            f.feed(2.0 + 0.01 * i, 0.3 + 2.5 * 0.01 * i, True, 0.3)
        self.assertIsNone(f.feed(2.7, 2.0, False, 0.0))
        self.assertIsNone(f.value)

    def test_initial_level_from_launch(self):
        g = learner(3.5)
        g.launch_ratio = 1.5
        g.set_launch(3.0)
        self.assertAlmostEqual(g.map.level, 3.0 * 1.5 * g.cfg.launch_safety)
        g.set_launch(1.0)                       # 1 回だけ
        self.assertAlmostEqual(g.map.level, 3.0 * 1.5 * g.cfg.launch_safety)
        g2 = learner(3.5)                       # 比がまだ無い
        g2.set_launch(3.0)
        self.assertEqual(g2.map.level, 3.5)


class TestSpeedProfileArrays(unittest.TestCase):
    def setUp(self):
        t = np.linspace(0.0, 2.0 * np.pi, 240, endpoint=False)
        self.xy = np.c_[4.0 * np.cos(t), 2.0 * np.sin(t)]
        self.k = rl.speed_kappa(self.xy)
        self.kw = dict(v_max=4.0, v_min=0.35, a_accel=2.8, a_brake=2.5)

    def test_uniform_array_equals_scalar(self):
        n = len(self.xy)
        a = rl.speed_profile(self.xy, self.k, a_lat=3.7, a_grip=4.1, **self.kw)
        b = rl.speed_profile(self.xy, self.k, a_lat=np.full(n, 3.7), a_grip=np.full(n, 4.1), **self.kw)
        np.testing.assert_allclose(a, b, rtol=0, atol=1e-12)
        a = rl.speed_profile(self.xy[:100], self.k[:100], a_lat=3.7, closed=False, v_start=1.0, **self.kw)
        b = rl.speed_profile(self.xy[:100], self.k[:100], a_lat=np.full(100, 3.7), closed=False,
                             v_start=1.0, **self.kw)
        np.testing.assert_allclose(a, b, rtol=0, atol=1e-12)

    def test_low_patch_slows_only_around_it(self):
        n = len(self.xy)
        lat = np.full(n, 4.0)
        lat[:20] = 2.5
        a = rl.speed_profile(self.xy, self.k, a_lat=4.0, **self.kw)
        b = rl.speed_profile(self.xy, self.k, a_lat=lat, **self.kw)
        self.assertTrue(np.all(b <= a + 1e-9))
        self.assertLess(b[5], a[5] - 0.2)
        np.testing.assert_allclose(a[100:140], b[100:140], atol=1e-9)

    def test_catch_up_accepts_array(self):
        n = len(self.xy)
        v = rl.speed_profile(self.xy, self.k, a_lat=4.0, **self.kw)
        path = rl.RaceLine(xy=self.xy, v=v, kappa=self.k, length=float(n) * 0.08)
        a = speedctl.catch_up(path, 10, 0.5, a_accel=2.8, a_grip=4.0)
        b = speedctl.catch_up(path, 10, 0.5, a_accel=2.8, a_grip=np.full(n, 4.0))
        np.testing.assert_allclose(a.v, b.v, atol=1e-12)


class Straight:
    """加速・減速の区間を 1 つずつ流す作り物。車速PIを簡単に写す: 計画の加速度に、計画との速度差を
    詰めるぶんを足して求め、路面の上限 `cap` で頭打ちにする。"""

    def __init__(self, lg: grip.LongLearner, noise: float = 0.0, seed: int = 0) -> None:
        self.lg = lg
        self.t = 0.0
        self.noise = noise
        self.rng = np.random.default_rng(seed)

    def zone(self, a_plan: float, cap: float, v0: float = 2.5, seconds: float = 0.6,
             start_off: float = 0.0, lat: float = 0.0, gate: str = "", taper: float = 0.0) -> None:
        """:param start_off: 区間の初めの、車速 − 計画の速度 [m/s]
        :param taper: 区間の終わりのこの時間、車だけ加速をやめる [s]（指令の前倒しで先に細る）"""
        lg = self.lg
        v = vp = v0
        v += start_off
        for _ in range(60):                     # 区間の前: 何も求めていない
            self.t += DT
            lg.observe(self.t, v, 0.0, lat, gate, vp)
        n = int(seconds / DT)
        for i in range(n):
            self.t += DT
            vp += a_plan * DT
            want = a_plan + 8.0 * (vp - v)
            if taper and i >= n - int(taper / DT):
                want = 0.0
            v += max(-cap, min(cap, want)) * DT
            lg.observe(self.t, v + self.rng.normal(0.0, self.noise) if self.noise else v,
                       a_plan, lat, gate, vp)
        for _ in range(3):
            self.t += DT
            lg.observe(self.t, v, 0.0, lat, gate, vp)


def long_learner(acc=2.5, dec=2.0, top=3.0, **kw) -> grip.LongLearner:
    lg = grip.LongLearner()
    lg.configure(acc_init=acc, dec_init=dec, acc_max=top, dec_max=top,
                 down_only=kw.get("down_only", False), dec_use=kw.get("dec_use", 0.95))
    return lg


class TestLongLearner(unittest.TestCase):
    """出ない加速度を求めて、出た値を測る（`grip.LongLearner`）。"""

    def test_accel_rises_while_delivered_and_stops_at_the_cap(self):
        lg = long_learner()
        st = Straight(lg)
        for _ in range(6):
            st.zone(lg.map.accel, cap=9.0)
        self.assertAlmostEqual(lg.map.accel, 3.0)
        self.assertTrue(lg.acc_done)

    def test_accel_shortfall_is_measured_in_one_zone(self):
        """★ 頭打ちになった区間 1 つで、出た値（のすぐ下）になる。"""
        lg = long_learner()
        Straight(lg).zone(2.5, cap=1.8)
        self.assertAlmostEqual(lg.map.accel, 1.8 * lg.cfg.acc_keep, delta=0.03)
        self.assertEqual(len(lg.events), 1)

    def test_accel_finds_a_limit_above_the_start(self):
        """求めたぶん出れば上げ、頭打ちになった所で測った値に落ち着く（上下しない）。"""
        lg = long_learner(acc=2.0)
        st = Straight(lg)
        seen = []
        for _ in range(12):
            st.zone(lg.map.accel, cap=2.6)
            seen.append(lg.map.accel)
        self.assertAlmostEqual(seen[-1], 2.6, delta=0.06 * 2.6)
        self.assertLess(max(seen), 2.6 * 1.14)        # 行き過ぎは 1 回の上げ幅まで
        self.assertLess(max(seen[5:]) - min(seen[5:]), 0.08 * 2.6)     # 見直しは 3% ずつ

    def test_accel_measured_too_low_once_creeps_back(self):
        """1 回の測り違いで低く決めても、出続けていれば少しずつ見直す。"""
        lg = long_learner()
        st = Straight(lg)
        st.zone(2.5, cap=1.8)                   # 測り違い（本当は 2.6 まで出る）
        for _ in range(80):
            st.zone(lg.map.accel, cap=2.6)
        self.assertGreater(lg.map.accel, 2.4)
        self.assertLess(lg.map.accel, 2.6 * 1.08)

    def test_accel_shortfall_lowers_decel_before_any_braking_is_seen(self):
        lg = long_learner()
        Straight(lg).zone(2.5, cap=1.8)
        self.assertLess(lg.map.decel, 2.0 * 0.8)        # 同じ路面なら減速も出ない。先に下げておく

    def test_decel_is_measured_by_asking_for_more_than_the_road_gives(self):
        """★ 制動区間の初めに強く求め、出た値をそのまま採る。2 回そろったら「決まった」。"""
        lg = long_learner()
        st = Straight(lg)
        self.assertTrue(lg.probing)
        self.assertAlmostEqual(lg.probe, 2.0 * lg.cfg.boost)
        st.zone(-lg.probe, cap=2.2)             # 2.3 を求めて 2.2 しか出ない
        self.assertAlmostEqual(lg.map.decel, 2.2, delta=0.08)   # 1 回目は少し控えめに採る
        self.assertFalse(lg.dec_done)
        st.zone(-lg.probe, cap=2.2)
        self.assertAlmostEqual(lg.map.decel, 2.2, delta=0.03)
        self.assertTrue(lg.dec_done)
        self.assertFalse(lg.probing)            # 決まったら強く求めない

    def test_decel_rises_to_the_asked_value_when_delivered(self):
        lg = long_learner()
        st = Straight(lg)
        for _ in range(4):
            st.zone(-lg.probe, cap=9.0)
        self.assertAlmostEqual(lg.map.decel, 3.0)       # 設定の上限まで
        self.assertTrue(lg.dec_done)

    def test_shortfall_in_normal_braking_lowers_at_once(self):
        """★ 決まった後に路面が滑りやすくなった: ふだんの制動で出ていなければ、その区間で下げる。"""
        lg = long_learner()
        st = Straight(lg)
        for _ in range(4):
            st.zone(-lg.probe, cap=2.3)
        self.assertTrue(lg.dec_done)
        st.zone(-0.95 * lg.map.decel, cap=1.9)
        self.assertAlmostEqual(lg.map.decel, 1.9, delta=0.04)
        self.assertFalse(lg.dec_done)           # また測る
        self.assertTrue(lg.probing)

    def test_settled_values_are_not_probed_again(self):
        """★ 決まった値は、時間がたっても強く求めて測り直さない（測るたびにコーナーへ速く入る危険がある）。"""
        lg = long_learner()
        st = Straight(lg)
        for _ in range(3):
            st.zone(-lg.probe, cap=2.3)
        st.zone(2.5, cap=1.8)
        a = lg.map.accel
        st.t += 600.0
        for _ in range(15):
            st.zone(-0.95 * lg.map.decel, cap=2.3)
            st.zone(lg.map.accel, cap=1.8)
        self.assertFalse(lg.probing)
        self.assertLess(abs(lg.map.accel - a), 0.07 * a)        # 限界の見直しは 3% ずつ（跳ばない）
        lg.recheck()                            # 横の学習が「路面が変わった」と見た
        self.assertTrue(lg.probing)

    def test_not_tracking_is_not_taken_for_the_limit(self):
        """★ 路面の限界ではなく、追っていないだけの窓で「出ていない」と取り違えない。"""
        lg = long_learner()
        st = Straight(lg)
        st.zone(2.5, cap=9.0, start_off=0.4)            # 計画より先に居て、追いつかれるだけ
        self.assertEqual(lg.map.accel, 2.5)
        st.zone(-1.9, cap=9.0, start_off=-0.4)          # 計画より遅く制動区間へ入った
        self.assertGreaterEqual(lg.map.decel, 2.0)
        st.zone(2.5, cap=1.0, start_off=-0.6)           # 計画よりずっと遅れて入った（追いつく途中）
        self.assertEqual(lg.map.accel, 2.5)
        st.zone(2.5, cap=9.0, taper=0.08)               # 区間の終わりで車の加速が先に細る
        self.assertGreaterEqual(lg.map.accel, 2.5)
        st.zone(2.5, cap=1.8, seconds=0.12)             # 短すぎる
        st.zone(2.5, cap=1.8, gate="障害物")             # planner が止めた
        st.zone(2.5, cap=1.8, lat=0.8)                  # 横を使っている
        self.assertGreaterEqual(lg.map.accel, 2.5)
        self.assertEqual(lg.events, [])

    def test_a_jump_in_the_plan_discards_the_window(self):
        lg = long_learner()
        t, v, vp = 0.0, 2.5, 2.5
        for i in range(120):
            t += DT
            vp += 2.5 * DT + (0.3 if i == 80 else 0.0)  # プロファイルが作り直された
            v += 1.8 * DT
            lg.observe(t, v, 2.5, 0.0, "", vp)
        lg.observe(t + DT, v, 0.0, 0.0, "", vp)
        self.assertEqual(lg.skips.get("計画が跳んだ"), 1)

    def test_measurement_survives_speed_noise(self):
        """車速に雑音（σ 0.02m/s）が乗っていても、測った値は 6% 以内。"""
        for seed in range(5):
            lg = long_learner()
            st = Straight(lg, noise=0.02, seed=seed)
            for _ in range(14):                 # 雑音があると、窓をいくつか溜めてから決める
                st.zone(-lg.probe, cap=2.3)
                self.assertLess(lg.map.decel, 2.3 * 1.03, seed)     # 途中でも真値を上回らない
            self.assertAlmostEqual(lg.map.decel, 2.3, delta=0.06 * 2.3)

    def test_road_change_seen_in_accel_lowers_decel_ahead_of_the_next_braking(self):
        """★ 落ち着いていた加速がはっきり落ちた＝路面が変わった。減速も先に下げて測り直す。"""
        lg = long_learner()
        st = Straight(lg)
        for _ in range(4):
            st.zone(-lg.probe, cap=2.3)
        for _ in range(4):
            st.zone(lg.map.accel, cap=9.0)
        self.assertTrue(lg.dec_done and lg.acc_done)
        d0, a0 = lg.map.decel, lg.map.accel
        st.zone(a0, cap=0.6 * a0)
        self.assertAlmostEqual(lg.map.decel, d0 * 0.6 * lg.cfg.acc_keep, delta=0.08)
        self.assertTrue(lg.probing)

    def test_road_change_in_the_middle_of_a_window_is_still_measured(self):
        """窓の途中で頭打ちに入っても、後半だけで測る（傾きが違うからと捨てない）。"""
        lg = long_learner()
        lg.dec_done = True
        t, v, vp = 1.0, 3.5, 3.5
        for i in range(60):
            t += DT
            lg.observe(t, v, 0.0, 0.0, "", vp)
        for i in range(45):
            t += DT
            vp -= 1.9 * DT
            v -= (1.9 if i < 20 else 1.3) * DT          # 0.2 秒後に路面が変わった
            lg.observe(t, v, -1.9, 0.0, "", vp)
        lg.observe(t + DT, v, 0.0, 0.0, "", vp)
        self.assertAlmostEqual(lg.map.decel, 1.3, delta=0.08)

    def test_down_only_never_asks_for_more(self):
        lg = long_learner(down_only=True)
        st = Straight(lg)
        self.assertFalse(lg.probing)
        st.zone(2.5, cap=9.0)
        st.zone(-1.9, cap=9.0)
        self.assertEqual((lg.map.accel, lg.map.decel), (2.5, 2.0))
        st.zone(-1.9, cap=1.5)
        self.assertLess(lg.map.decel, 1.6)

    def test_never_drops_below_half_of_the_start(self):
        lg = long_learner()
        st = Straight(lg)
        for _ in range(10):
            st.zone(-lg.map.decel, cap=0.45 * lg.map.decel)
        self.assertGreaterEqual(lg.map.decel, 1.0)

    def test_lateral_learner_only_stops_the_measurement_when_the_profile_changes(self):
        """★ コーナーを抜けるたびに版を進めない（進めると毎回プロファイルを作り直し、測定も止まる）。"""
        g = learner()
        tr = Track(g)
        tr.laps(30)
        self.assertTrue(g.converged)
        ver, quiet = g.ver, g.long._quiet_t
        tr.laps(3)
        self.assertEqual(g.ver, ver)
        self.assertEqual(g.long._quiet_t, quiet)

    def test_saved_values_restart_lower(self):
        g = grip.GripLearner()
        m = grip.GripMap.from_json(json.dumps({"format": 1, "cell_m": 0.25, "level": 5.0,
                                               "accel": 2.9, "decel": 2.8, "cells": []}))
        g.load(m)
        self.assertAlmostEqual(g.map.accel, 2.9 * g.long.cfg.back_off)
        self.assertAlmostEqual(g.map.decel, 2.8 * g.long.cfg.back_off)
        self.assertIn('"decel"', g.map.to_json())


class TestProbeBrake(unittest.TestCase):
    def setUp(self):
        t = np.linspace(0.0, 2.0 * np.pi, 300, endpoint=False)
        self.xy = xy = np.c_[5.0 * np.cos(t), 1.5 * np.sin(t)]
        base = rl.RaceLine(xy=xy, v=np.zeros(300), kappa=rl.curvature(xy), length=22.0)
        self.kw = dict(v_max=4.0, v_min=0.35, a_lat=4.0, a_accel=2.8, a_brake=2.0, a_grip=4.0)
        self.line = rl.retime(base, **self.kw)

    def test_never_faster_than_the_original(self):
        """★ 試す値が出なくても、元のプロファイルより速くコーナーへ入らない。"""
        out = rl.probe_brake(self.line, 2.6, 4.0)
        self.assertTrue(np.all(out.v <= self.line.v + 1e-9))
        self.assertGreater(float((self.line.v - out.v).max()), 0.1)    # 制動の区間では早く落ちる
        self.assertAlmostEqual(float(out.v.min()), float(self.line.v.min()))
        self.assertAlmostEqual(float(out.v.max()), float(self.line.v.max()))   # 始める位置は同じ

    def test_asks_hard_only_for_a_short_while_then_eases_back(self):
        from raspi.nav import speedctl
        out = rl.probe_brake(self.line, 2.6, 4.0, max_s=0.3, ease_m=1.0)
        a = -speedctl.path_accel(out)
        i0 = int(np.argmax(self.line.v))
        hard = [k for k in range(300) if a[k] > 2.4]
        self.assertTrue(hard)
        step = 22.0 / 300
        for zone in (hard[:len(hard) // 2], hard[len(hard) // 2:]):
            self.assertLess(len(zone) * step / 3.5, 0.45)              # 強く求めるのは 0.3 秒ほど
        self.assertNotEqual(i0, -1)
        # 元との差は、戻す距離の間は残っている（すぐには戻さない）
        k = hard[len(hard) // 2 - 1] + 4
        self.assertGreater(self.line.v[k % 300] - out.v[k % 300], 0.03)
        lost = rl._travel_time(self.xy, out.v, True) - rl._travel_time(self.xy, self.line.v, True)
        self.assertLess(lost, 0.25)

    def test_zone_too_short_to_measure_is_left_alone(self):
        out = rl.probe_brake(self.line, 2.6, 4.0, min_s=5.0)
        np.testing.assert_array_equal(out.v, self.line.v)

    def test_same_decel_changes_little(self):
        out = rl.probe_brake(self.line, 2.0, 4.0)
        self.assertTrue(np.all(out.v <= self.line.v + 1e-9))
        self.assertLess(float(np.mean(self.line.v - out.v)), 0.05)


class TestParamsAndModes(unittest.TestCase):
    def test_use_fractions_are_gone_and_hidden_params_stay_constant(self):
        from raspi.auto.registry import catalog, merged_params
        from raspi.auto.slam2d_route import Slam2dRoute
        keys = {s.key for s in Slam2dRoute.params}
        self.assertFalse(keys & {"lat_use", "accel_use", "brake_use"})
        ids = {c["id"]: c for c in catalog()}
        self.assertNotIn("slam2d_raceline", ids)        # GUI のモード選択には出さない
        shown = {q["key"] for q in ids["slam2d_route"]["params"]}
        self.assertIn("v_max", shown)
        self.assertIn("line_margin", shown)
        self.assertNotIn("look_k", shown)
        self.assertLessEqual(len(shown), 22)
        # 隠した項目は、保存済みの設定・GUI から来た値を読まない（bench だけ上書きできる）
        self.assertEqual(merged_params("slam2d_route", {"look_k": 1.5})["look_k"], 0.45)
        self.assertEqual(merged_params("slam2d_route", {"look_k": 1.5}, hidden=True)["look_k"], 1.5)
        self.assertEqual(merged_params("slam2d_route", {"v_max": 3.0})["v_max"], 3.0)
        self.assertIsNotNone(merged_params("slam2d_raceline", {}))     # bench 用には残っている


class TestPlannerWiring(unittest.TestCase):
    """`slam2d_raceline` が学習した値を速度プロファイルへ通す所。"""

    def setUp(self):
        import tempfile

        from raspi.auto import mapstore
        from raspi.auto.slam2d_raceline import Slam2dRaceLine
        from raspi.msgs.types import VehicleState
        self._td = tempfile.TemporaryDirectory()
        self._maps, mapstore.MAPS_DIR = mapstore.MAPS_DIR, Path(self._td.name)
        self.mapstore = mapstore
        self.pl = Slam2dRaceLine()
        self.p = Slam2dRaceLine.merged({"v_max": 4.0})
        t = np.linspace(0.0, 2.0 * np.pi, 300, endpoint=False)
        xy = np.c_[4.0 * np.cos(t), 2.0 * np.sin(t)]
        self.pl.path = rl.RaceLine(xy=xy, v=np.zeros(300), kappa=rl.curvature(xy), length=19.4)
        self.vs = lambda t=1.0: VehicleState(speed=2.0, t_capture=int(t * 1e9))

    def tearDown(self):
        self.pl.close()
        self.mapstore.MAPS_DIR = self._maps
        self._td.cleanup()

    def _retimed(self, p, hint=0):
        self.pl._hint = hint
        self.pl._grip_begin(self.vs(), p)
        self.pl._retime(p)
        return self.pl.path.v.copy()

    def test_off_is_the_identified_value(self):
        """★ 学習を切ると今までと同じ: 同定値のスカラーをそのまま渡し、作り直しの鍵も同じ。"""
        from raspi.auto.slam2d_raceline import LINE_KEYS, speed_key, speed_kwargs
        pl, p = self.pl, self.p
        pl._grip_begin(self.vs(), p)
        self.assertEqual(pl._spd(p, pl.path), speed_kwargs(p, pl.vehicle))
        self.assertEqual(pl._skey(p), speed_key(p))
        self.assertFalse(any(k.startswith("grip") for k in LINE_KEYS))   # 線は引き直さない
        v0 = self._retimed(p)
        ref = rl.retime(pl.path, **speed_kwargs(p, pl.vehicle)).v
        np.testing.assert_array_equal(v0, ref)

    def test_learned_level_reaches_the_profile(self):
        pl = self.pl
        p = {**self.p, "grip_learn": 1.0, "grip_init": 1.0}
        pl._grip_begin(self.vs(), p)
        pl._grip.long.dec_done = True           # 減速を強く求めて測る形（`probe_brake`）は外して比べる
        v0 = self._retimed(p)
        ref = rl.retime(pl.path, **{**pl._spd(self.p, pl.path)}).v      # 初期値 1.0 倍 = 同定値
        np.testing.assert_allclose(v0, ref, atol=1e-9)
        # 経路の 1 か所（左端のコーナー、点 150 付近）だけ前兆の値を書く
        x, y = pl.path.xy[150]
        c = pl._grip.map.cells.setdefault(grip._cell(x, y), grip._Cell())
        c.bad = 2.0
        pl._grip._bump(down=True)
        v1 = self._retimed(p)
        self.assertLess(v1[150], v0[150] - 0.2)
        np.testing.assert_allclose(v1[:20], v0[:20], atol=1e-9)        # 反対側のコーナーは変わらない

    def test_raise_is_applied_ahead_of_the_car_only(self):
        pl = self.pl
        p = {**self.p, "grip_learn": 1.0, "grip_init": 0.8}
        v0 = self._retimed(p, hint=140)
        pl._grip.map.level *= 1.1
        pl._grip.conf = pl._grip.map.level
        pl._grip._bump()                        # 上げただけ
        v1 = self._retimed(p, hint=140)
        step = pl.path.length / len(pl.path)
        near = int(1.2 / step)
        self.assertTrue(np.all(v1[140:140 + near] <= v0[140:140 + near] + 1e-9))   # 足元は跳ばない
        self.assertGreater(v1[0], v0[0] + 0.05)                                    # 遠くは上がる
        v2 = self._retimed(p, hint=140 + int(3.0 / step))                          # 窓を抜けた
        self.assertGreater(v2[150], v0[150] + 0.05)
        # 下げる変更は足元にもすぐ効く
        pl._grip.map.level *= 0.8
        pl._grip._bump(down=True)
        v3 = self._retimed(p, hint=140)
        self.assertLess(v3[150], v2[150] - 0.1)

    def test_rebuilding_in_a_braking_zone_does_not_move_the_braking_earlier(self):
        """★ 「足元は前の速度のまま」の上限は、減速を測る形（`probe_brake`）を重ねる前の速度から作る。
        重ねた後の速度を上限にすると、作り直すたびに制動が手前へずれ、目標速度に段差ができた。"""
        pl = self.pl
        p = {**self.p, "grip_learn": 1.0}
        v0 = self._retimed(p)
        self.assertTrue(pl._grip.long.probing)
        hint = int(np.argmax(v0)) + 6                   # 制動区間に入った所
        for _ in range(4):
            pl._grip._bump()                            # 上げただけの変更（値は同じ）
            v1 = self._retimed(p, hint=hint % len(v0))
        np.testing.assert_allclose(v1, v0, atol=1e-6)

    def test_learned_accel_and_decel_reach_the_profile_and_limits(self):
        pl = self.pl
        p = {**self.p, "grip_learn": 1.0}
        self._retimed(p)
        m = pl._grip.map
        from raspi.auto.slam2d_raceline import LEARN_BRAKE_USE
        self.assertAlmostEqual(pl._lim(p).a_brake, LEARN_BRAKE_USE * m.decel)
        self.assertAlmostEqual(pl._spd(p, pl.path)["a_brake"], LEARN_BRAKE_USE * m.decel)
        self.assertEqual(pl._spd(p, pl.path)["a_accel"], m.accel)
        off = {**self.p, "grip_learn": 0.0}
        pl._grip_begin(self.vs(), off)
        from raspi.auto.slam2d_raceline import FIXED_BRAKE_USE
        self.assertAlmostEqual(pl._lim(off).a_brake, FIXED_BRAKE_USE * pl.vehicle.a_decel_max)

    def test_line_is_redrawn_only_after_learning_settles(self):
        pl = self.pl
        p = {**self.p, "grip_learn": 1.0}
        self._retimed(p)
        g = pl._grip
        g.converged, g.long.dec_done = True, True
        pl._last_p = p
        self.assertFalse(pl._grip_line_stale())         # 既定（`grip_reline` 0）では引き直さない
        g.converged, g.long.dec_done = False, False
        p = {**p, "grip_reline": 1.0}
        pl._last_p = p
        self.assertFalse(pl._grip_line_stale())         # まだ値が動いている
        g.converged = True
        g.long.dec_done = True
        self.assertTrue(pl._grip_line_stale())          # 同定値で引いた線のまま
        pl._grip_line = pl._grip_learned()
        self.assertFalse(pl._grip_line_stale())
        g.map.level *= 1.05
        self.assertTrue(pl._grip_line_stale())          # 3% 以上変わった

    def test_values_are_saved_next_to_the_map_and_reloaded_lower(self):
        pl, ms = self.pl, self.mapstore
        ms.save_map("m", resolution=0.05, origin_x=0.0, origin_y=0.0,
                    trinary=np.ones((8, 8), dtype=np.uint8), centerline_xy=np.zeros((0, 2)),
                    raceline_xy=np.zeros((0, 2)), raceline_v=np.zeros(0))
        pl._grip_load("m")
        g = pl._grip
        g.configure(init_level=4.0, max_level=8.0, down_only=False, lat_use=1.0)
        g.trials = 3
        g.launch_ratio = 1.5
        pl._grip_save()
        self.assertAlmostEqual(json.loads(ms.load_grip("m"))["level"], 4.0)
        self.assertAlmostEqual(ms.load_grip_vehicle(), 1.5)
        pl._grip_load("m")
        self.assertAlmostEqual(pl._grip.map.level, 4.0 * pl._grip.cfg.back_off)
        self.assertAlmostEqual(pl._grip.launch_ratio, 1.5)
        (ms.MAPS_DIR / "m.grip.json").write_text("{broken")
        pl._grip_load("m")                      # 壊れていても走れる（初期値から）
        self.assertEqual(pl._grip.map.level, 0.0)


if __name__ == "__main__":
    unittest.main()
