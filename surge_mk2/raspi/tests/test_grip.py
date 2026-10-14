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

    @staticmethod
    def _pass(g: grip.GripLearner, level: float, a: float, lat_set: bool = True) -> None:
        """同じ 3 セルを、上限 `level` で横加速度 `a` まで出して抜けた試行 1 つ。"""
        g.map.level = level
        g._end_trial(0.0, {"cells": {(k, 0): (a, level, False, lat_set) for k in range(3)},
                           "fail": False, "peak": a, "top": a, "lv": level, "at": (0.0, 0.0),
                           "ex": 0.0, "ex_t": 0.0, "ex_v2": 0.0, "ex_ap": 0.0, "cx": {}})

    def test_plateau_skips_where_speed_is_set_by_accel(self):
        """車速が加減速で決まっている所は、上限を上げても横加速度が増えない。頭打ちに数えない
        （実機 2026-10-13: ヘアピンの出口から加速しながら回るコーナーで誤検出、横が 4.41 で止まった）。"""
        for lat_set, n in ((True, 1), (False, 0)):
            g = learner(4.0)
            self._pass(g, 4.0, 4.0, lat_set)
            self._pass(g, 4.4, 4.0, lat_set)
            self.assertEqual([e["kind"] for e in g.events], ["頭打ち"] * n, lat_set)

    def test_plateau_waits_for_a_wide_step_when_noisy(self):
        """雑音があるときは、上限が `plateau_ask_noisy` 以上上がるまで比べず、前の値を持ち続ける。"""
        g = learner(4.0)
        self.assertGreater(g.noise_sigma, g.cfg.noise_quiet)
        self._pass(g, 4.0, 4.0)
        self._pass(g, 4.12, 4.0)                # +3%: まだ比べない
        self.assertEqual(g.events, [])
        self._pass(g, 4.24, 4.0)                # 始めの値から +6%: まだ
        self.assertEqual(g.events, [])
        self._pass(g, 4.36, 4.0)                # +9%: 比べる
        self.assertEqual([e["kind"] for e in g.events], ["頭打ち"])

    def test_new_line_forgets_previous_passes(self):
        """線を乗り換えたら、前の線で通ったときの値とは比べない。"""
        g = learner(4.0)
        self._pass(g, 4.0, 4.0)
        g.new_line()
        self._pass(g, 4.4, 4.0)
        self.assertEqual(g.events, [])

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
        g.long.configure(acc_init=3.0, dec_init=3.0, dec_max=3.0, down_only=False)
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
        self.assertEqual(g.map.accel, 3.0)              # 加速は学習しない
        self.assertFalse(g.long.dec_done)               # 探し直す

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


class TestTimeLearner(unittest.TestCase):
    """横の上限を周回のタイムで決める版（`grip.TimeLearner`）。"""

    LIMIT = 6.0     #: これより上げてもラップが縮まない（タイヤの限界）

    def _g(self, init=3.5, **kw) -> grip.TimeLearner:
        g = grip.TimeLearner(yaw_wn=0.0)
        g.configure(init_level=init, max_level=kw.pop("max_level", 8.0), down_only=False, lat_use=1.0)
        return g

    @staticmethod
    def _plan(level: float, dec: float = 2.0) -> float:
        """計画のラップ [s]: 横の上限と減速が上がるほど縮む。"""
        return 20.0 / math.sqrt(level) + 2.0 / dec

    def _run(self, g, laps=80, noise=0.0, dec=lambda k: 2.0, seed=0):
        rng = np.random.default_rng(seed)
        top = 0.0
        for k in range(laps):
            if g.converged:
                break
            lv = g.map.level
            top = max(top, lv)
            real = self._plan(min(lv, self.LIMIT), dec(k)) + noise * rng.standard_normal()
            g._lap_done(float(k), real, self._plan(lv, dec(k)),
                        self._plan(g.ref_level, dec(k)) if g.ref_level else 0.0)
        return top

    def test_stops_where_the_lap_stops_improving(self):
        g = self._g()
        top = self._run(g)
        self.assertTrue(g.converged)
        self.assertGreater(g.map.level, 0.93 * self.LIMIT)
        self.assertLessEqual(g.map.level, 1.03 * self.LIMIT)
        self.assertLess(top, 1.09 * self.LIMIT)         # 行き過ぎは 1 段まで

    def test_real_lap_noise_does_not_stop_it_early(self):
        """実機のラップのばらつき（sd 0.04s）では、限界のずっと手前で止まらない。"""
        for seed in range(20):
            g = self._g()
            self._run(g, noise=0.04, seed=seed)
            self.assertGreater(g.map.level, 0.85 * self.LIMIT, seed)
            self.assertLess(g.map.level, 1.10 * self.LIMIT, seed)

    def test_brake_learning_is_not_mistaken_for_lateral_gain(self):
        """減速の学習でラップが縮んでも、計画も同じだけ縮むので、横を上げた効果とは数えない。"""
        g = self._g(init=self.LIMIT)                    # 初めから限界に居る
        self._run(g, dec=lambda k: min(3.0, 2.0 + 0.1 * k))
        self.assertTrue(g.converged)
        self.assertLess(g.map.level, 1.03 * self.LIMIT)

    def test_stops_at_the_configured_maximum(self):
        g = self._g(max_level=5.0)
        self._run(g)
        self.assertTrue(g.converged)
        self.assertAlmostEqual(g.map.level, 5.0, places=6)

    def _drive(self, g, t, seconds, *, v=2.0, radius=2.0, steer_sign=1.0, gate="", plan=6.0):
        """半径 `radius` の円を左回りに走る。`steer_sign` −1 で舵だけ逆（逆ハン）。"""
        w = v / radius
        for _ in range(int(seconds / DT)):
            t += DT
            if int(round(t / DT)) % 10 == 0:
                g.observe_plan(plan)
            g.observe(t, radius * math.cos(w * t), radius * math.sin(w * t), v, w,
                      steer_sign / radius, gate=gate)
        return t

    def test_lap_time_is_measured_at_the_gate(self):
        g = self._g()
        lap = 2.0 * math.pi * 2.0 / 2.0                 # 1 周 6.283s
        self._drive(g, 0.0, 2.6 * lap)
        self.assertAlmostEqual(g.last_lap[0], lap, delta=0.003)
        self.assertAlmostEqual(g.last_lap[1], 6.0, places=6)

    def test_gated_lap_is_not_counted(self):
        g = self._g()
        lap = 2.0 * math.pi
        t = self._drive(g, 0.0, 0.5 * lap)
        t = self._drive(g, t, 0.1 * lap, gate="障害物")
        self._drive(g, t, 0.7 * lap)                    # 1 周目の終わりをまたぐ
        self.assertEqual(g.last_lap, (0.0, 0.0))
        t = self._drive(g, 0.0 + 1.3 * lap, 0.3 * lap, gate="舵が上限")   # これは数えてよい
        self._drive(g, t, 0.6 * lap)
        self.assertAlmostEqual(g.last_lap[0], lap, delta=0.01)

    def test_rear_slide_rejects_the_step_being_tried(self):
        g = self._g(init=5.0)
        g._ref = (5.0, 6.0, 6.0)
        g._raise(0.0)                                   # 5.4 を試している
        self.assertAlmostEqual(g.map.level, 5.4, places=6)
        t = self._drive(g, 0.0, 1.0, v=3.0)
        self.assertEqual(g.events, [])
        self._drive(g, t, 0.3, v=3.0, steer_sign=-1.0)  # 舵と逆向きに回っている
        self.assertEqual([e["kind"] for e in g.events][:1], ["リアの滑り"])
        self.assertIsNotNone(g.cap)
        self.assertAlmostEqual(g.map.level, 5.0 * 1.04, places=6)     # 戻して半分の幅で

    def test_rear_slide_at_the_accepted_level_lowers_it(self):
        g = self._g(init=5.0)
        g._ref = (5.0, 6.0, 6.0)
        g.converged = True
        t = self._drive(g, 0.0, 1.0, v=3.0)
        self._drive(g, t, 0.3, v=3.0, steer_sign=-1.0)
        self.assertAlmostEqual(g.map.level, 5.0 * 0.95, places=6)
        self.assertFalse(g.converged)                   # 下げた所を測り直す

    def test_level_where_the_guard_fired_is_not_tried_again(self):
        g = self._g(init=5.0)
        g._ref = (5.0, 6.0, 6.0)
        g._raise(0.0)
        tried = g.map.level
        g._guard(1.0, 0.0, 0.0, "横偏差", 5.0)
        top = tried
        for k in range(40):                             # どの段も「全部出た」ことにして進める
            if g.converged:
                break
            lv = g.map.level
            top = max(top, lv) if k else lv
            g._lap_done(10.0 + k, self._plan(lv), self._plan(lv),
                        self._plan(g.ref_level) if g.ref_level else 0.0)
        self.assertTrue(g.converged)
        self.assertLess(top, tried)
        self.assertGreater(g.map.level, 5.0)

    def test_guard_ceiling_survives_save_and_reload(self):
        """安全装置が働いた上限は保存され、読み直した次の走行でもその手前で止まる。"""
        g = self._g(init=5.0)
        g._ref = (5.0, self._plan(5.0), self._plan(5.0))
        g._raise(0.0)
        tried = g.map.level
        g._guard(1.0, 0.0, 0.0, "リアの滑り", tried)
        self.assertAlmostEqual(grip.GripMap.from_json(g.map.to_json()).ceil, round(tried, 3))
        g2 = self._g(init=5.0)
        g2.load(grip.GripMap.from_json(g.map.to_json()))
        g2.configure(init_level=5.0, max_level=8.0, down_only=False, lat_use=1.0)
        top = self._run(g2)
        self.assertLessEqual(top, g2.cfg.time_ceil * tried + 1e-3)
        # `ceil` の無い古いファイルは「無い」として読む
        old = json.loads(g.map.to_json())
        del old["ceil"]
        self.assertEqual(grip.TimeLearner().map.ceil, 0.0)
        self.assertEqual(grip.GripMap.from_json(json.dumps(old)).ceil, 0.0)

    def test_not_worth_saving_until_a_lap_is_measured(self):
        """読み込んだ値（0.92 倍）を、1 周も測らないうちに保存しない（短い走行で下がり続けない）。"""
        g = self._g(init=5.0)
        g.load(grip.GripMap(level=6.0))
        g.trials = 3                                    # コーナーは抜けたが周は測っていない
        self.assertFalse(g.worth_saving())
        g._lap_done(0.0, self._plan(g.map.level), self._plan(g.map.level))
        self.assertTrue(g.worth_saving())
        g2 = self._g(init=5.0)
        g2._guard(1.0, 0.0, 0.0, "リアの滑り", 5.0)       # 安全装置が働いたら保存する
        self.assertTrue(g2.worth_saving())

    def test_launch_only_lowers_the_level(self):
        g = self._g(init=3.5)
        g.launch_ratio = 1.5
        g.set_launch(4.0)                               # 4.0×1.5×0.92 = 5.5 へは上げない
        self.assertEqual(g.map.level, 3.5)
        g = self._g(init=3.5)
        g.launch_ratio = 1.5
        g.set_launch(2.0)                               # 2.0×1.5×0.92 = 2.76 へは下げる
        self.assertAlmostEqual(g.map.level, 2.0 * 1.5 * g.cfg.launch_safety)

    def test_one_soft_sign_does_not_fix_the_ceiling(self):
        """「横偏差の増え」1 回では、その上限を諦めない（2 回目で諦める）。"""
        g = self._g(init=4.0)
        g._ref = (4.0, self._plan(4.0), self._plan(4.0))
        g._raise(0.0)
        g._guard(1.0, 0.0, 0.0, "横偏差の増え", 4.0, soft=True)
        self.assertEqual(g._ceil, math.inf)
        self._run(g)
        self.assertGreater(g.map.level, 0.93 * self.LIMIT)
        lv = g.map.level
        g.converged = False
        g._guard(100.0, 0.0, 0.0, "横偏差の増え", lv, soft=True)
        self.assertLessEqual(g._ceil, lv)

    def test_final_stop_is_confirmed_against_a_fresh_reference(self):
        """終える前に採用済みの段を測り直す。たまたま速く出た 1 回の基準で、限界の手前で終わらない。"""
        g = self._g(init=4.0)
        g._ref = (4.0, self._plan(4.0) - 0.3, self._plan(4.0))      # 基準が 0.3 秒速く出ていた
        g._raise(0.0)
        self._run(g)
        self.assertGreater(g.map.level, 0.93 * self.LIMIT)

    def test_saved_cell_values_are_ignored(self):
        """場所ごとの前兆の値（元の方式が保存したもの）は使わない。"""
        g = self._g(init=5.0)
        g.map.cells[(0, 0)] = grip._Cell(bad=2.0)
        self.assertEqual(g.level_at(0.1, 0.1), 5.0)


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

    def test_limit_is_one_value_for_the_course(self):
        """★ 横の限界はコースで 1 つ。どこで出た前兆も全体の値を下げる（きついコーナーの前兆を場所だけの
        値にする形は、実機で全体が 8.0 まで上がって危うい挙動になった、2026-10-13）。"""
        g = learner(5.0)
        g._precursor(1.0, 0.0, 0.0, 1.6, 4.6, 0.0, 0.93, 5.0, False, 0.07)
        self.assertLess(g.map.level, 5.0)

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
        quiet.long.configure(acc_init=3.0, dec_init=3.0, dec_max=3.0, down_only=False)
        quiet.map.noise, quiet.map.noise_n = 0.003, 1000
        quiet._precursor(1.0, 0.0, 0.0, 2.0, 3.0, 0.0, 0.93, 5.0, False, 0.07)
        self.assertLess(quiet.map.decel, 3.0)
        g = learner(5.0)
        g.map.noise, g.map.noise_n = 0.04, 1000
        g.long.configure(acc_init=3.0, dec_init=3.0, dec_max=3.0, down_only=False)
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

    def test_odd_numbers_are_a_value_error(self):
        """巨大な数・型の違いも `ValueError`（呼び出し側はそれだけを受ける）。"""
        good = json.loads(grip.GripMap(level=4.0).to_json())
        for cells in ([[1e999, 0, 1.0, 0.0]], [["a", 0, 1.0, 0.0]], [[None, 0, 1.0, 0.0]], "x"):
            with self.assertRaises(ValueError):
                grip.GripMap.from_json(json.dumps({**good, "cells": cells}))
        with self.assertRaises(ValueError):
            grip.GripMap.from_json("[1, 2]")

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


def long_learner(dec=2.0, top=3.0, acc=2.5, **kw) -> grip.LongLearner:
    lg = grip.LongLearner()
    lg.configure(acc_init=acc, dec_init=dec, dec_max=top, down_only=kw.get("down_only", False))
    return lg


class LongTrack:
    """1 周ぶんの制動区間を流す作り物。路面の上限 `cap` までしか減速できない車は、求められた減速が
    それを超えるぶんだけ計画より速いまま区間の終わりへ来る。`late` は指令の遅れ [s]（制動の出だしが
    遅れるぶん。出だしは計画より速いが、区間の終わりまでに追いつく）。"""

    def __init__(self, lg: grip.LongLearner) -> None:
        self.lg = lg
        self.t = 0.0

    def lap(self, cap: float = 9.0, seconds: float = 1.0, gate: str = "", zones: int = 2,
            late: float = 0.0) -> None:
        lg = self.lg
        for _ in range(zones):
            a = lg.map.decel
            v = vp = 3.5
            for i in range(int(seconds / DT)):
                self.t += DT
                vp -= a * DT
                want = 0.0 if i * DT < late else a + 20.0 * (v - vp)     # 車速PIが遅れを詰める
                v -= min(want, cap) * DT
                lg.observe(self.t, v, -a, gate, vp)
            for _ in range(60):                 # 区間の間: 何も求めていない
                self.t += DT
                lg.observe(self.t, vp, 0.0, gate, vp)
        lg.lap(self.t)


class TestLongLearner(unittest.TestCase):
    """減速: 1 周に 1 段上げ、コーナーへ計画より速く入るようになったら 1 段戻して止める。"""

    def test_rises_while_the_car_follows_and_stops_one_step_below(self):
        lg = long_learner(dec=1.6)
        tr = LongTrack(lg)
        seen = []
        for _ in range(12):
            tr.lap(cap=2.2)
            seen.append(lg.map.decel)
        self.assertTrue(lg.dec_done)
        self.assertLess(max(seen), 2.2 * (1.0 + lg.cfg.step) * 1.06)    # 行き過ぎは 1 段ほどまで
        self.assertLessEqual(seen[-1], 2.2 * 1.06)
        self.assertGreater(seen[-1], 2.2 / (1.0 + lg.cfg.step) * 0.95)
        self.assertEqual(seen[-1], seen[-4])                    # 止めた後は動かない

    def test_stops_at_the_configured_maximum(self):
        lg = long_learner()
        tr = LongTrack(lg)
        for _ in range(8):
            tr.lap()
        self.assertEqual(lg.map.decel, 3.0)
        self.assertTrue(lg.dec_done)
        self.assertEqual(lg.events, [])

    def test_late_start_of_braking_is_not_a_shortfall(self):
        """制動の出だしは指令の遅れのぶん計画より速い。区間の終わりで追いついていれば、付いて行けている
        （区間全体の差で見ていた版は、シムで真値 3.00 に対して 2.40 で止まった）。"""
        lg = long_learner()
        tr = LongTrack(lg)
        for _ in range(8):
            tr.lap(late=0.08)
        self.assertEqual(lg.map.decel, 3.0)
        self.assertEqual(lg.events, [])

    def test_accel_is_not_learned(self):
        lg = long_learner(acc=2.66)
        tr = LongTrack(lg)
        for _ in range(8):
            tr.lap(cap=1.0)
        self.assertEqual(lg.map.accel, 2.66)
        lg.map.accel = 1.77                     # 以前の版が保存した値が残っていても、使わない
        lg.configure(acc_init=2.66, dec_init=2.0, dec_max=3.0, down_only=False)
        self.assertEqual(lg.map.accel, 2.66)

    def test_lowers_again_when_the_road_gets_worse(self):
        """止めた後も毎周見る。付いて行けなくなったら 1 段ずつ下げる。"""
        lg = long_learner()
        tr = LongTrack(lg)
        for _ in range(8):
            tr.lap()
        for _ in range(6):
            tr.lap(cap=2.4)
        self.assertLessEqual(lg.map.decel, 2.4 * 1.06)
        self.assertGreater(lg.map.decel, 2.4 / 1.08 * 0.95)
        self.assertEqual({e["kind"] for e in lg.events}, {"減速"})

    def test_does_not_rise_again_until_told_the_road_changed(self):
        lg = long_learner(dec=1.6)
        tr = LongTrack(lg)
        for _ in range(8):
            tr.lap(cap=2.0)
        d = lg.map.decel
        for _ in range(5):
            tr.lap()                            # 路面は良くなったが、上げ直さない
        self.assertEqual(lg.map.decel, d)
        lg.recheck()                            # 横の学習が「路面が変わった」と見た
        tr.lap()
        self.assertGreater(lg.map.decel, d)

    def test_plan_speed_is_advanced_between_planner_ticks(self):
        """計画は 10Hz でしか届かない。その間を計画の減速度で進めて比べる。"""
        lg = long_learner(dec=2.0)
        t, v, vp = 0.0, 3.5, 3.5
        for i in range(120):
            t += DT
            v -= 2.0 * DT
            if i % 10 == 0:
                vp = v                          # 届いた瞬間は計画どおり。次に届くまで古いまま
            lg.observe(t, v, -2.0, "", vp)
        lg.observe(t + DT, v, 0.0, "", v)
        lg.lap(t)
        self.assertAlmostEqual(lg.last, 0.0, delta=0.03)        # 進めないと −0.2〜0 に見える
        self.assertGreater(lg.map.decel, 2.0)

    def test_short_or_gated_zones_are_not_judged(self):
        lg = long_learner()
        tr = LongTrack(lg)
        tr.lap(cap=1.0, seconds=0.2)            # `min_s` に満たない
        tr.lap(cap=1.0, gate="障害物")
        self.assertEqual(lg.map.decel, 2.0)
        self.assertEqual(lg.windows, 0)

    def test_down_only_never_raises(self):
        lg = long_learner(down_only=True)
        tr = LongTrack(lg)
        for _ in range(4):
            tr.lap()
        self.assertEqual(lg.map.decel, 2.0)
        tr.lap(cap=1.5)
        self.assertLess(lg.map.decel, 2.0)

    def test_floor_and_saved_values(self):
        lg = long_learner()
        tr = LongTrack(lg)
        for _ in range(30):
            tr.lap(cap=0.2)
        self.assertAlmostEqual(lg.map.decel, 0.5 * 2.0)         # 出発点の半分より下へは下げない
        m = grip.GripMap.from_json(lg.map.to_json())
        lg2 = grip.LongLearner()
        lg2.reset(m)
        lg2.load()
        self.assertAlmostEqual(m.decel, 1.0 * lg2.cfg.back_off, places=3)

    def test_rescale_lowers_and_searches_again(self):
        lg = long_learner()
        tr = LongTrack(lg)
        for _ in range(8):
            tr.lap()
        lg.rescale(0.7)
        self.assertAlmostEqual(lg.map.decel, 3.0 * 0.7)
        self.assertFalse(lg.dec_done)

    def test_time_learner_judges_once_per_lap(self):
        """横をタイムで決める学習器は、自前のゲートで 1 周を数えて減速も判断する。"""
        g = grip.TimeLearner(yaw_wn=0.0)
        g.configure(init_level=3.5, max_level=8.0, down_only=False, lat_use=1.0)
        g.long.configure(acc_init=2.0, dec_init=2.0, dec_max=3.0, down_only=False)
        t, w = 0.0, 1.0
        for i in range(int(2.6 * 2.0 * math.pi / DT)):          # 半径 2m を 2m/s で 2.6 周
            t += DT
            brake = (w * t) % (2.0 * math.pi) > math.pi         # 半周は減速を求められている
            g.observe(t, 2.0 * math.cos(w * t), 2.0 * math.sin(w * t), 2.0, w, 0.5,
                      a_x=-2.0 if brake else 0.0, v_plan=2.0 + 1e-9 * i)     # 計画は毎周期届く
        self.assertEqual(g.long.windows, 2)
        self.assertAlmostEqual(g.long.map.decel, 2.0 * 1.08 ** 2, places=6)


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
        p = {**self.p, "grip_learn": 1.0, "grip_init": 1.0, "grip_by_time": 0.0}
        pl._grip_begin(self.vs(), p)
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

    def test_mode_switch_rebuilds_the_learner(self):
        pl = self.pl
        self.assertIsInstance(pl._grip, grip.TimeLearner)       # 既定はタイムで決める
        pl._grip_begin(self.vs(), {**self.p, "grip_learn": 1.0, "grip_by_time": 0.0})
        self.assertNotIsInstance(pl._grip, grip.TimeLearner)
        pl._grip_begin(self.vs(), {**self.p, "grip_learn": 1.0})
        self.assertIsInstance(pl._grip, grip.TimeLearner)
        self.assertGreater(pl._grip.map.level, 0.0)

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

    def test_learned_accel_and_decel_reach_the_profile_and_limits(self):
        pl = self.pl
        p = {**self.p, "grip_learn": 1.0}
        self._retimed(p)
        m = pl._grip.map
        self.assertAlmostEqual(pl._lim(p).a_brake, m.decel)
        self.assertAlmostEqual(pl._spd(p, pl.path)["a_brake"], m.decel)
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
        self.assertEqual(ms.load_grip("m"), "")         # 1 周も測っていない間は保存しない
        g._measured = True
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
