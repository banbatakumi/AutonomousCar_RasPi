"""`raspi/nav/grip.py`（走行中の学習: 横の上限をタイムで決める・前後の包絡）の単体テスト。"""

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

DT = 0.01
DATA = Path(__file__).resolve().parent / "data"


class TestGripLearner(unittest.TestCase):
    """横の上限を周回のタイムで決める（`grip.GripLearner`）。"""

    LIMIT = 6.0     #: これより上げてもラップが縮まない（タイヤの限界）

    def _g(self, init=3.5, **kw) -> grip.GripLearner:
        g = grip.GripLearner(yaw_wn=0.0)
        g.configure(init_level=init, max_level=kw.pop("max_level", 8.0), down_only=False)
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

    def _g_env(self, init: float = 5.0) -> grip.GripLearner:
        g = self._g(init=init)
        g.long.configure(acc_init=3.0, dec_init=2.5, dec_max=4.0, down_only=False)
        return g

    def _circle_ay(self, g, t, seconds, *, excess, v=3.0, radius=1.5, a_x=0.0):
        """円を走り、IMU の横加速度 = ヨーレート×車速 − `excess` を渡す。"""
        w = v / radius
        for _ in range(int(seconds / DT)):
            t += DT
            g.observe(t, radius * math.cos(w * t), radius * math.sin(w * t), v, w, 1.0 / radius,
                      a_y=w * v - excess, a_x=a_x)
        return t

    def test_yaw_excess_over_lateral_acceleration_is_a_rear_slide(self):
        """ヨーレート×車速 が IMU の横加速度を `slide_excess` 以上上回り続けたら、逆ハンが無くてもリアの滑り。
        実機 `083921` の制動しながらの巻き込み。"""
        g = self._g(init=5.0)
        g._ref = (5.0, 6.0, 6.0)
        g.converged = True
        self._circle_ay(g, 0.0, 1.0, excess=0.8)        # しきい値の手前
        self.assertEqual(g.events, [])
        self._circle_ay(g, 10.0, 0.5, excess=2.0)
        self.assertEqual([e["kind"] for e in g.events][:1], ["リアの滑り"])
        self.assertIsNotNone(g.cap)
        self.assertAlmostEqual(g.map.level, 5.0 * 0.95)  # 包絡を上げていないので横を下げる

    def test_yaw_excess_while_braking_undoes_the_last_decel_raise(self):
        g = self._g_env()
        g._ref = (5.0, 6.0, 6.0)
        g._lat_up_t = 0.5
        g.converged = True
        before = self._raise_env(g, 1.0)
        self._circle_ay(g, 2.0, 0.5, excess=2.0, a_x=-2.5)
        self.assertEqual(g.map.dec, before)
        self.assertAlmostEqual(g.map.level, 5.0)

    def test_slide_while_accelerating_blames_the_accel_envelope_not_the_last_raise(self):
        """出口のパワーオーバーは、最後に上げたのが減速でも加速の包絡を戻す。実機 `111523` では減速を 2.96 → 2.56 まで
        下げ続けた（2026-10-29 夜）。"""
        g = self._g_env()
        g._ref = (5.0, 6.0, 6.0)
        g._lat_up_t = 0.5
        g.converged = True
        acc_before = self._raise_env(g, 0.8, side="acc")
        dec_raised = list(self._raise_env(g, 1.0) and g.map.dec)
        self._circle_ay(g, 2.0, 0.5, excess=2.0, a_x=1.3)
        self.assertEqual(g.map.acc, acc_before)
        self.assertEqual(g.map.dec, dec_raised)
        self.assertAlmostEqual(g.map.level, 5.0)
        # 加速を上げた記録が尽きたら、減速ではなく横を下げる
        self._circle_ay(g, 10.0, 0.5, excess=2.0, a_x=1.3)
        self.assertEqual(g.map.dec, dec_raised)
        self.assertAlmostEqual(g.map.level, 5.0 * 0.95)

    def test_slide_without_longitudinal_demand_blames_the_lateral_limit(self):
        g = self._g_env()
        g._ref = (5.0, 6.0, 6.0)
        g._lat_up_t = 0.5
        g.converged = True
        self._raise_env(g, 1.0)
        dec = list(g.map.dec)
        self._circle_ay(g, 2.0, 0.5, excess=2.0, a_x=0.1)
        self.assertEqual(g.map.dec, dec)
        self.assertAlmostEqual(g.map.level, 5.0 * 0.95)

    def _raise_env(self, g, t: float, side: str = "dec") -> list[float]:
        """包絡（既定は減速）の u=0.5 の点を 1 段上げた（付いて行けた）ことにする。上げる前の包絡を返す。"""
        before = list(getattr(g.map, side))
        g.long._set(getattr(g.long, side), 1, before[1] * 1.05, t, side)
        self.assertGreater(getattr(g.map, side)[1], before[1])
        return before

    def test_guard_after_an_envelope_raise_undoes_that_raise(self):
        """最後に上げたものを疑う: 横を採用した後に包絡を上げていたら、横ではなくその包絡を戻し、それより上は試さない。
        シム toyota2 で、減速の包絡が真値より上がって毎周同じ所で外れたのを、u=1 の点ばかり下げて直せなかった。"""
        g = self._g_env()
        g._ref = (5.0, 6.0, 6.0)
        g._lat_up_t = 1.0
        g.converged = True
        before = self._raise_env(g, 5.0)
        g._guard(10.0, 0.0, 0.0, "横偏差", 5.0)
        self.assertEqual(g.map.dec, before)
        self.assertAlmostEqual(g.map.level, 5.0)        # 横の上限はそのまま
        self.assertEqual(g.map.ceil, 0.0)
        self.assertLessEqual(g.long.dec.ceil[1] * g.long.cfg.ceil_gap, before[1] + 1e-9)    # それより上は試さない
        self.assertFalse(g.converged)                   # 包絡が変わったので測り直す
        # 戻す記録が尽きたら、これまでどおり横を下げる
        g.converged = True
        g._guard(20.0, 0.0, 0.0, "横偏差", 5.0)
        self.assertAlmostEqual(g.map.level, 5.0 * 0.95)
        self.assertGreater(g.map.ceil, 0.0)

    def test_guard_after_a_lateral_raise_blames_the_lateral_limit(self):
        """包絡を上げた後に横を上げていたら、横を疑う（試している最中ならその段をやめる。包絡は動かさない）。"""
        g = self._g_env()
        self._raise_env(g, 1.0)
        g._ref = (5.0, 6.0, 6.0)
        g._raise(5.0)                                   # 5.4 を試している
        env = list(g.map.dec)
        g._guard(10.0, 0.0, 0.0, "横偏差の増え", 5.4, soft=True)
        self.assertEqual(g.map.dec, env)
        self.assertLess(g.map.level, 5.4)               # 試していた段をやめた

    def test_rear_slide_without_longitudinal_demand_keeps_the_old_handling(self):
        g = self._g_env()
        g._ref = (5.0, 6.0, 6.0)
        g.converged = True
        before = (list(g.map.acc), list(g.map.dec))
        g._guard(1.0, 0.0, 0.0, "リアの滑り", 5.0)
        self.assertEqual((g.map.acc, g.map.dec), before)
        self.assertAlmostEqual(g.map.level, 5.0 * 0.95)

    def _lap_with_cross(self, g, t0, cross_at, *, v=2.0, radius=2.0, plan=6.0):
        """半径 `radius` の円を 1 周強走る。角度 `cross_at` = (始め, 終わり) [rad] の間だけ横偏差 8cm、ほかは 1cm。"""
        w = v / radius
        t = t0
        for _ in range(int(1.02 * 2.0 * math.pi / w / DT)):
            t += DT
            ang = (w * t) % (2.0 * math.pi)
            g.observe_cross(0.08 if cross_at[0] <= ang < cross_at[1] else 0.01)
            if int(round(t / DT)) % 10 == 0:
                g.observe_plan(plan)
            g.observe(t, radius * math.cos(w * t), radius * math.sin(w * t), v, w, 1.0 / radius, u_plan=0.9)
        return t

    def test_cross_gain_over_the_usual_rejects_the_step(self):
        """コーナーの中の横偏差が、その場所の普段（前の周）より `cross_gain` 以上増えた周は、試している段をやめる。
        1 回では天井にしない。"""
        g = self._g(init=2.0)
        t = 0.0
        for _ in range(4):                              # 普段の値を溜める（どこも 1cm）
            t = self._lap_with_cross(g, t, (9.0, 9.0))
        self.assertEqual(g.events, [])
        g._ref = (2.0, 6.0, 6.0)
        g._raise(t)
        tried = g.map.level
        self.assertGreater(tried, 2.0)
        for _ in range(2):
            t = self._lap_with_cross(g, t, (1.0, 1.5))   # 1 か所だけ 8cm
            if g.events:
                break
        self.assertEqual([e["kind"] for e in g.events], ["横偏差の増え"])
        self.assertLess(g.map.level, tried)
        self.assertEqual(g.map.ceil, 0.0)
        self.assertAlmostEqual(g.last_cross_gain, 0.07, delta=0.005)

    def test_cross_gain_is_not_judged_after_convergence(self):
        g = self._g(init=2.0)
        t = 0.0
        for _ in range(4):
            t = self._lap_with_cross(g, t, (9.0, 9.0))
        g.converged = True
        lv = g.map.level
        t = self._lap_with_cross(g, t, (1.0, 1.5))
        t = self._lap_with_cross(g, t, (1.0, 1.5))
        self.assertEqual(g.events, [])
        self.assertAlmostEqual(g.map.level, lv)

    def test_off_the_line_slows_at_once(self):
        """線から `cross_thr` 外れたら、その周期に目標速度を落とし、採用済みの段なら下げる。"""
        g = self._g(init=2.0)
        g._ref = (2.0, 6.0, 6.0)
        g.converged = True
        t = self._drive(g, 0.0, 0.5)
        g.observe_cross(0.15)
        self._drive(g, t, 0.05)
        self.assertEqual([e["kind"] for e in g.events][:1], ["横偏差"])
        self.assertIsNotNone(g.cap)
        self.assertAlmostEqual(g.map.level, 2.0 * 0.95)

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
        g2.configure(init_level=5.0, max_level=8.0, down_only=False)
        top = self._run(g2)
        self.assertLessEqual(top, g2.cfg.time_ceil * tried + 1e-3)
        # `ceil` の無い古いファイルは「無い」として読む
        old = json.loads(g.map.to_json())
        del old["ceil"]
        self.assertEqual(grip.GripLearner().map.ceil, 0.0)
        self.assertEqual(grip.GripMap.from_json(json.dumps(old)).ceil, 0.0)

    def test_not_worth_saving_until_a_lap_is_measured(self):
        """読み込んだ値（0.92 倍）を、1 周も測らないうちに保存しない（短い走行で下がり続けない）。"""
        g = self._g(init=5.0)
        g.load(grip.GripMap(level=6.0))
        self.assertFalse(g.worth_saving())
        g._lap_done(0.0, self._plan(g.map.level), self._plan(g.map.level))
        self.assertTrue(g.worth_saving())
        g2 = self._g(init=5.0)
        g2._guard(1.0, 0.0, 0.0, "リアの滑り", 5.0)       # 安全装置が働いたら保存する
        self.assertTrue(g2.worth_saving())

    def test_one_soft_sign_does_not_fix_the_ceiling(self):
        """「横偏差の増え」1 回では、その上限を諦めない（2 回目で諦める）。"""
        g = self._g(init=4.0)
        g._ref = (4.0, self._plan(4.0), self._plan(4.0))
        g._raise(0.0)
        g._guard(1.0, 0.0, 0.0, "横偏差の増え", 4.0, soft=True)
        self.assertEqual(g.map.ceil, 0.0)
        self._run(g)
        self.assertGreater(g.map.level, 0.93 * self.LIMIT)
        lv = g.map.level
        g.converged = False
        g._guard(100.0, 0.0, 0.0, "横偏差の増え", lv, soft=True)
        self.assertLessEqual(g.map.ceil, lv)

    def test_final_stop_is_confirmed_against_a_fresh_reference(self):
        """終える前に採用済みの段を測り直す。たまたま速く出た 1 回の基準で、限界の手前で終わらない。"""
        g = self._g(init=4.0)
        g._ref = (4.0, self._plan(4.0) - 0.3, self._plan(4.0))      # 基準が 0.3 秒速く出ていた
        g._raise(0.0)
        self._run(g)
        self.assertGreater(g.map.level, 0.93 * self.LIMIT)

class TestRealRecords(unittest.TestCase):
    """滑っていない実機の記録（`data/grip_real_*.npz`。100Hz の車速・ヨーレート・実舵角と 10Hz の姿勢・横偏差を
    `raspi/tools/grip_replay.py --export` で抜いたもの）を流して、安全装置が働かないこと
    （曲率の不足を見る検出器が、滑っていない車で 46 回働いて 1 周 5.7s → 13.1s になった例がある）。"""

    @classmethod
    def setUpClass(cls):
        from dataclasses import replace

        from raspi.core.vehicle import Vehicle
        from raspi.tools import grip_replay
        cls.replay = grip_replay
        # 記録を取ったときの同定値に固定する（`config/vehicle.toml` が変わってもテストは変わらない）
        cls.vehicle = replace(Vehicle.load(), lim_lat_m_s2=0.4172 * 9.80665, yaw_natural_freq_rad_s=58.5,
                              yaw_damping=0.934, steer_gain=1.0, steer_gain_cubic=0.0, steer_servo_gain=0.974,
                              steer_offset_rad=0.0028)

    def test_brake_slides_are_caught_and_the_earlier_laps_are_not(self):
        """実機 `surge_20261029_083921`（減速の包絡を 3.4 まで上げてリアが巻き込んだ走行）。前半 150 秒（減速 2.54〜2.96、
        横の探索と収束を含む）は 0 回、後半（減速 2.96〜3.6、逆ハンは 20 回中 3 回しか捕まえられなかった）は横滑りの増えで働く。"""
        v = self.vehicle
        first = self.replay.load(DATA / "grip_real_20261029_first150.npz")
        self.assertIsNotNone(first.ay)
        res = self.replay.replay(first, v, pin=4.92)
        self.assertEqual(res.learner.events, [])
        res = self.replay.replay(self.replay.load(DATA / "grip_real_20261029_brake_slide.npz"), v, pin=4.92)
        self.assertGreaterEqual(len(res.learner.events), 10)
        self.assertEqual({e["kind"] for e in res.learner.events}, {"リアの滑り"})

    def test_slides_are_blamed_on_the_side_the_plan_was_using(self):
        """入口の制動の巻き込み（`083921` 後半）は減速、出口のパワーオーバー（`111523` 終盤。減速を 2.96 → 2.56 まで
        下げてしまった所）は加速の包絡を疑う。"""
        lim = grip.GripConfig().slide_long
        brake = self.replay.replay(self.replay.load(DATA / "grip_real_20261029_brake_slide.npz"), self.vehicle, pin=4.92)
        al = [e["along"] for e in brake.learner.events]
        self.assertGreaterEqual(sum(1 for x in al if x <= -lim), 0.8 * len(al))
        rec = self.replay.load(DATA / "grip_real_20261029_exit_slide.npz")
        self.assertIsNotNone(rec.a_plan)
        exit_ = self.replay.replay(rec, self.vehicle, pin=5.117)
        al = [e["along"] for e in exit_.learner.events]
        self.assertGreaterEqual(len(al), 2)
        self.assertTrue(all(x >= lim for x in al), al)

    def test_records_without_imu_still_load(self):
        """IMU の横加速度が無い古い記録（npz）も読める（横滑りの増えは見ない）。"""
        rec = self.replay.load(DATA / "grip_real_20261012.npz")
        self.assertIsNone(rec.ay)
        self.assertIsNone(self.replay.cut(rec, 0.0, 10.0).ay)

    def test_clean_records_have_no_guard(self):
        for name in ("grip_real_20261012.npz", "grip_real_20261011.npz"):
            rec = self.replay.load(DATA / name)
            res = self.replay.replay(rec, self.vehicle)
            self.assertEqual([e["kind"] for e in res.learner.events], [], name)
            self.assertGreater(res.seen_s, 20.0, name)
            for pin in (2.0, 3.0, 4.0, 5.5):
                res = self.replay.replay(rec, self.vehicle, pin=pin, plan_lap=6.0)
                self.assertEqual([e["kind"] for e in res.learner.events], [], (name, pin))


class TestGripMap(unittest.TestCase):
    def test_roundtrip(self):
        m = grip.GripMap(level=4.2, acc=[2.9, 2.5, 1.2], dec=[2.5, 2.2, 1.0], ceil=5.1,
                         best_lat=4.2, best_acc=[2.9, 2.5, 1.2], best_dec=[2.5, 2.2, 1.0])
        r = grip.GripMap.from_json(m.to_json())
        self.assertEqual((r.level, r.acc, r.dec, r.ceil, r.best_lat), (4.2, m.acc, m.dec, 5.1, 4.2))

    def test_old_format_with_cells_is_read_and_the_cells_dropped(self):
        old = {"format": 1, "cell_m": 0.25, "level": 4.5, "ok": 4.0, "bad": 5.0, "noise": 0.04, "noise_n": 9,
               "acc_env": [2.9, 2.5, 1.2], "dec_env": [2.5, 2.2, 1.0], "ceil": 5.2,
               "cells": [[0, 0, 4.0, 5.0, 0.01, 3]]}
        m = grip.GripMap.from_json(json.dumps(old))
        self.assertEqual((m.level, m.ceil, m.acc), (4.5, 5.2, [2.9, 2.5, 1.2]))
        self.assertIn('"format": 2', m.to_json())
        self.assertNotIn("cells", m.to_json())

    def test_odd_values_are_dropped(self):
        """数でない値は「無い」として読む（初期値から学習し直す）。形が違えば `ValueError`。"""
        m = grip.GripMap.from_json('{"format": 2, "acc_env": {"a": 1}, "level": [1], "ceil": 1e999}')
        self.assertEqual((m.level, m.acc, m.ceil), (0.0, [], 0.0))
        with self.assertRaises(ValueError):
            grip.GripMap.from_json("[1, 2]")

    def test_values_are_clamped(self):
        text = json.dumps({"format": grip.FORMAT, "level": 99.0, "ceil": -3.0, "acc_env": [50.0, 2.0, 3.0]})
        m = grip.GripMap.from_json(text)
        self.assertEqual(m.level, grip.A_CEIL)
        self.assertEqual(m.ceil, 0.0)
        self.assertEqual(m.acc, [grip.A_CEIL, 2.0, 2.0])     # u が大きいほど小さい

    def test_wrong_format_is_rejected(self):
        for text in ('{"format": 0}', '{"format": 3}', "[]", "{"):
            with self.assertRaises(ValueError):
                grip.GripMap.from_json(text)


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


def long_learner(dec=2.0, top=4.0, acc=2.5, **kw) -> grip.LongLearner:
    lg = grip.LongLearner()
    lg.configure(acc_init=acc, dec_init=dec, acc_max=top, dec_max=top, down_only=kw.get("down_only", False))
    return lg


def env_at(vals, u):
    return rl.env_at(vals, u)


class LongTrack:
    """1 周ぶんの制動区間と加速区間を流す作り物。計画は横の使い具合 `u` での包絡の `frac` 倍を求め、車はそれを
    `late` 秒遅れで追い（車速PIが遅れを詰める）、出せる加減速は `cap`（減速）・`acap`（加速）まで。"""

    def __init__(self, lg: grip.LongLearner) -> None:
        self.lg = lg
        self.t = 0.0

    def _zone(self, sign: float, a: float, cap: float, seconds: float, gate: str, late: float, u: float) -> None:
        lg = self.lg
        v = vp = 5.0 if sign < 0 else 1.0
        hist: list[float] = []
        for i in range(int(seconds / DT)):
            self.t += DT
            vp += sign * a * DT
            hist.append(vp)
            k = int(round(late / DT))
            ref = hist[-1 - k] if len(hist) > k else hist[0] - sign * a * DT
            want = sign * a + 20.0 * (ref - v)          # 車速PIが遅れを詰める
            v += (max(-cap, want) if sign < 0 else min(cap, want)) * DT
            lg.observe(self.t, v, sign * a, gate, vp, u)
        for _ in range(60):                     # 区間の間: 何も求めていない
            self.t += DT
            lg.observe(self.t, vp, 0.0, gate, vp, u)

    def lap(self, cap: float = 9.0, acap: float = 9.0, seconds: float = 1.0, gate: str = "", zones: int = 2,
            late: float = 0.0, accel: bool = True, frac: float = 1.0, u: float = 0.0) -> None:
        lg = self.lg
        for _ in range(zones):
            self._zone(-1.0, frac * env_at(lg.map.dec, u), cap, seconds, gate, late, u)
            if accel:
                self._zone(1.0, frac * env_at(lg.map.acc, u), acap, seconds, gate, late, u)
        lg.lap(self.t)


class TestLongLearner(unittest.TestCase):
    """前後の上限の包絡: 計画が包絡で決まっている所で、実際に出た ÷ 求めた で点ごとに合わせる。"""

    def test_default_envelope_is_the_calibrated_value_times_the_shape(self):
        lg = long_learner(dec=2.0, acc=2.5)
        self.assertEqual(lg.map.dec, [2.0 * k for k in grip.ENV_SHAPE])
        self.assertEqual(lg.map.acc, [2.5 * k for k in grip.ENV_SHAPE])
        self.assertEqual((lg.map.decel, lg.map.accel), (2.0, 2.5))

    def test_lowers_to_what_the_car_delivers(self):
        """3.0 から始めて、2.4 しか出ない車では 2.4 前後に下がって止まる。u の大きい点も下がる（直線より大きくしない）。"""
        lg = long_learner(dec=3.0)
        tr = LongTrack(lg)
        seen = []
        for _ in range(16):
            tr.lap(cap=2.4)
            seen.append(lg.map.decel)
        self.assertAlmostEqual(seen[5], 2.4, delta=0.15)
        self.assertLess(max(seen[5:]), 2.4 * 1.1)
        self.assertGreater(min(seen[5:]), 2.4 * 0.9)
        self.assertLessEqual(lg.map.dec[1], lg.map.dec[0])
        self.assertIn("減速", {e["kind"] for e in lg.events})

    def test_rises_while_the_car_follows_and_settles_near_its_limit(self):
        """校正値は出発点だけ。出せる車なら上の天井（ランプ）の手前まで探す。"""
        lg = long_learner(dec=2.0)
        tr = LongTrack(lg)
        seen = []
        for _ in range(30):
            tr.lap(cap=3.0)
            seen.append(lg.map.decel)
        self.assertLess(max(seen), 3.0 * 1.17)          # 行き過ぎは 1 段まで
        self.assertGreater(min(seen[-8:]), 3.0 * 0.85)
        self.assertLess(max(seen[-8:]), 3.0 * 1.1)

    def test_each_lateral_band_learns_its_own_value(self):
        """横の使い具合ごとに別の値: 横の限界いっぱい（u = 1）で 1.6 出せる車は、その点だけ 1.6 前後へ上がる。"""
        lg = long_learner(dec=2.0)
        tr = LongTrack(lg)
        for _ in range(30):
            tr.lap(cap=1.6, u=1.0, accel=False)
        self.assertAlmostEqual(lg.map.dec[2], 1.6, delta=0.17)
        self.assertEqual(lg.map.dec[0], 2.0)            # 直線の値は測っていないので動かない
        self.assertGreaterEqual(lg.map.dec[1], lg.map.dec[2])

    def test_raising_a_high_lateral_band_lifts_the_straight_value(self):
        """横を使いながら 3.0 出せたなら、直線でも 3.0 は出せる（u の小さい点を持ち上げる）。"""
        lg = long_learner(dec=2.0)
        tr = LongTrack(lg)
        for _ in range(40):
            tr.lap(cap=3.0, u=0.5, accel=False)
        self.assertGreater(lg.map.dec[1], 2.6)
        self.assertGreaterEqual(lg.map.dec[0], lg.map.dec[1])

    def test_constant_lag_is_not_a_shortfall(self):
        """車速が計画に一定時間遅れて付いて来るだけなら下げない（出だしの過渡は捨てる。前回の加速の学習が
        下がり続けたのは、遅れのぶんを「出ていない」と見ていたため）。"""
        lg = long_learner(dec=2.0, acc=2.0)
        lg.cfg.lag = 1e9            # 入口の安全装置は外す（遅れが区間の終わりまで残れば、実際に速く入るので働いてよい）
        tr = LongTrack(lg)
        for _ in range(20):
            tr.lap(late=0.04)
        self.assertEqual(lg.events, [])
        # 下げない。ただし終わりでも遅れが残る（入口へ計画より速く入る）ので、上げもしない
        self.assertAlmostEqual(lg.map.decel, 2.0)
        self.assertAlmostEqual(lg.map.accel, 2.0)
        lg2 = long_learner(dec=2.0, acc=2.0)
        tr2 = LongTrack(lg2)
        for _ in range(20):
            tr2.lap(late=0.01)                          # 終わりまでに追いつく遅れなら上げる
        self.assertAlmostEqual(lg2.map.decel, 4.0)

    def test_gentle_demands_are_not_measured(self):
        """包絡の 6 割しか求めない所（包絡で速度が決まっていない所）は測らない（上限で付いて行ける証拠にならない）。"""
        lg = long_learner(dec=2.0, acc=2.0)
        lg.cfg.lag = 1e9
        tr = LongTrack(lg)
        for _ in range(10):
            tr.lap(frac=0.6, cap=0.5, acap=0.5)
        self.assertEqual((lg.map.decel, lg.map.accel), (2.0, 2.0))
        self.assertEqual(lg.dec.decisions, 0)

    def test_diag_tells_why_nothing_was_judged(self):
        """判断が出ないとき、強い加減速を求められた時間のうち、どれだけを何の理由で見なかったかが記録に出る。"""
        lg = long_learner(dec=2.0)
        tr = LongTrack(lg)
        tr.lap(gate="経路に乗るまで")
        d = lg.diag()
        self.assertGreater(d["long_want"], 3.0)
        self.assertAlmostEqual(d["long_gated"], d["long_want"], delta=0.02)
        self.assertEqual((d["acc0_s"], d["dec0_s"], d["acc_n"]), (0.0, 0.0, 0.0))
        tr.lap(seconds=0.2)                             # 出だしを捨てると短すぎる
        self.assertGreater(lg.diag()["dec_short"], 0.3)
        tr.lap()
        self.assertGreaterEqual(lg.diag()["dec_n"], 1.0)
        self.assertGreater(lg.diag()["dec0_r"], 0.9)
        self.assertEqual((d["dec0"], d["dec1"]), (2.0, round(2.0 * grip.ENV_SHAPE[1], 3)))

    def test_settles_without_hunting_between_raise_and_safety(self):
        """上げる判断と入口の安全装置の境目が 1 段より狭くても、上げては戻すを繰り返さない（付いて行けなかった
        高さには近づかない。シム toyota2 で 2.44〜2.74 を往復した）。"""
        lg = long_learner(dec=2.0)
        tr = LongTrack(lg)
        seen = []
        for _ in range(40):
            tr.lap(cap=2.5, accel=False)
            seen.append(round(lg.map.decel, 4))
        tail = seen[20:]
        self.assertLessEqual(sum(1 for a, b in zip(tail, tail[1:]) if a != b), 1)
        self.assertGreater(min(tail), 2.5 * 0.88)
        self.assertLess(max(tail), 2.5 * 1.06)

    def test_accel_is_learned_independently(self):
        lg = long_learner(dec=2.0, acc=2.9, top=3.0)
        tr = LongTrack(lg)
        for _ in range(12):
            tr.lap(acap=2.0)
        self.assertAlmostEqual(lg.map.accel, 2.0, delta=0.15)
        self.assertAlmostEqual(lg.map.decel, 3.0)       # 減速は出せるので天井まで
        self.assertEqual({e["kind"] for e in lg.events}, {"加速"})

    def test_stops_at_the_ceiling(self):
        lg = long_learner(dec=2.0, top=3.0)
        tr = LongTrack(lg)
        for _ in range(16):
            tr.lap()
        self.assertEqual(lg.map.decel, 3.0)
        self.assertEqual(lg.events, [])

    def test_entry_lag_safety_lowers_one_step(self):
        """減速の安全装置: 制動区間の終わりで計画より速ければ、溜めた窓を待たずに入口の u の点を 1 段下げる。"""
        lg = long_learner(dec=3.0)
        lg.cfg.node_min_s = 1e9                         # 比での判断はさせない
        tr = LongTrack(lg)
        tr.lap(cap=2.0, accel=False)
        self.assertAlmostEqual(lg.map.decel, 3.0 / (1.0 + lg.cfg.step))
        self.assertEqual(lg.events[0]["kind"], "減速（入口で遅れ）")

    def test_plan_speed_is_advanced_between_planner_ticks(self):
        """計画は 10Hz でしか届かない。その間を計画の減速度で進めて比べる（進めないと、ずれが鋸歯に見える）。"""
        lg = long_learner(dec=2.0)
        t = 0.0
        for _ in range(3):
            v = vp = 3.5
            for i in range(120):
                t += DT
                v -= 2.0 * DT
                if i % 10 == 0:
                    vp = v                      # 届いた瞬間は計画どおり。次に届くまで古いまま
                lg.observe(t, v, -2.0, "", vp)
            for _ in range(60):
                t += DT
                lg.observe(t, v, 0.0, "", v)
        lg.lap(t)
        self.assertAlmostEqual(lg.dec.ratio[0], 1.0, delta=0.03)
        self.assertGreater(lg.map.decel, 2.0)

    def test_short_or_gated_zones_are_not_judged(self):
        lg = long_learner()
        tr = LongTrack(lg)
        for _ in range(3):
            tr.lap(cap=1.0, acap=1.0, seconds=0.15)     # 出だしを捨てると `fit_min_s` に満たない
            tr.lap(cap=1.0, acap=1.0, gate="障害物")
        self.assertEqual((lg.map.decel, lg.map.accel), (2.0, 2.5))
        self.assertEqual(lg.windows, 0)

    def test_down_only_never_raises(self):
        lg = long_learner(down_only=True)
        tr = LongTrack(lg)
        for _ in range(6):
            tr.lap()
        self.assertEqual(lg.map.decel, 2.0)
        for _ in range(4):
            tr.lap(cap=1.5)
        self.assertLess(lg.map.decel, 2.0)

    def test_floor_and_saved_values(self):
        lg = long_learner()
        tr = LongTrack(lg)
        for _ in range(40):
            tr.lap(cap=0.2, acap=0.2)
        self.assertAlmostEqual(lg.map.decel, lg.cfg.floor * 2.0)    # 出発点の `floor` 倍より下へは下げない
        self.assertAlmostEqual(lg.map.accel, lg.cfg.floor * 2.5)
        m = grip.GripMap.from_json(lg.map.to_json())
        self.assertEqual(len(m.dec), 3)
        lg2 = grip.LongLearner()
        lg2.reset(m)
        lg2.load()
        self.assertAlmostEqual(m.decel, lg.cfg.floor * 2.0 * lg2.cfg.back_off, places=3)

    def test_old_file_with_scalar_values_becomes_an_envelope(self):
        old = json.loads(grip.GripMap(level=4.0).to_json())
        del old["acc_env"], old["dec_env"]
        old.update({"accel": 2.9, "decel": 2.5, "best_lat": 5.0, "best_dec": 2.4, "best_acc": 2.8})
        m = grip.GripMap.from_json(json.dumps(old))
        self.assertEqual(m.dec, [2.5 * k for k in grip.ENV_SHAPE])
        self.assertEqual(m.best_acc, [2.8 * k for k in grip.ENV_SHAPE])


    def test_undo_last_restores_the_whole_envelope_before_the_raise(self):
        """上げたときに u の小さい点も持ち上げていたら、それも戻す。戻すのは新しい順。"""
        lg = long_learner(dec=2.0)
        first = list(lg.map.dec)
        lg._set(lg.dec, 2, 2.0, 1.0, "減速")             # u=1 を 2.0 へ → u=0.5 も 2.0 まで持ち上がる
        second = list(lg.map.dec)
        lg._set(lg.dec, 0, 2.2, 2.0, "減速")
        self.assertEqual(lg.last_raise_t, 2.0)
        self.assertTrue(lg.undo_last(3.0, "横偏差"))
        self.assertEqual(lg.map.dec, second)
        self.assertTrue(lg.undo_last(4.0, "横偏差"))
        self.assertEqual(lg.map.dec, first)
        self.assertFalse(lg.undo_last(5.0, "横偏差"))
        self.assertEqual(lg.last_raise_t, -math.inf)

    def test_time_learner_judges_once_per_lap(self):
        """横をタイムで決める学習器は、自前のゲートで 1 周を数えて加速・減速も判断する。"""
        g = grip.GripLearner(yaw_wn=0.0)
        g.configure(init_level=3.5, max_level=8.0, down_only=False)
        g.long.configure(acc_init=2.0, dec_init=2.0, acc_max=3.0, dec_max=3.0, down_only=False)
        t, w = 0.0, 1.0
        for i in range(int(3.6 * 2.0 * math.pi / DT)):          # 半径 2m を 2m/s で 3.6 周
            t += DT
            brake = (w * t) % (2.0 * math.pi) > math.pi         # 半周は減速を求められている
            g.observe(t, 2.0 * math.cos(w * t), 2.0 * math.sin(w * t), 2.0, w, 0.5,
                      a_x=-2.0 if brake else 0.0, v_plan=2.0 + 1e-9 * i)     # 計画は毎周期届く
        self.assertGreaterEqual(g.long.windows, 1)
        self.assertGreater(g.long.map.decel, 2.0)       # 計画どおり（ずれが増えない）なので上げる


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
        # 同時に出る数で数える（`show_if`: 学習 ON なら学習の設定、OFF なら手で決める上限だけが出る）
        params = ids["slam2d_route"]["params"]
        for learn in (0.0, 1.0):
            vis = [q for q in params
                   if all(learn in v for k, v in q["show_if"].items() if k == "grip_learn")]
            self.assertLessEqual(len(vis), 22, learn)
        manual = {q["key"] for q in params if q["show_if"] == {"grip_learn": [0.0]}}
        self.assertEqual(manual, {"lat_max", "accel_max", "brake_max"})
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
        v0 = self._retimed(p)
        lv = pl._grip.map.level
        pl._grip.map.level = 0.8 * lv
        pl._grip._bump(down=True)
        v1 = self._retimed(p)
        self.assertLess(v1[150], v0[150] - 0.2)         # コーナーは遅くなる
        np.testing.assert_allclose(v1[150], v0[150] * math.sqrt(0.8), rtol=0.02)


    def test_raise_is_applied_ahead_of_the_car_only(self):
        pl = self.pl
        p = {**self.p, "grip_learn": 1.0, "grip_init": 0.8}
        v0 = self._retimed(p, hint=140)
        pl._grip.map.level *= 1.1
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
        g.converged = True
        g.long.settle()
        pl._last_p = p
        self.assertFalse(pl._grip_line_stale())         # 既定（`grip_reline` 0）では引き直さない
        g.converged = False
        g.long.dec.decisions = 0
        p = {**p, "grip_reline": 1.0}
        pl._last_p = p
        self.assertFalse(pl._grip_line_stale())         # まだ値が動いている
        g.converged = True
        g.long.settle()
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
        g.configure(init_level=4.0, max_level=8.0, down_only=False)
        pl._grip_save()
        self.assertEqual(ms.load_grip("m"), "")         # 1 周も測っていない間は保存しない
        g._measured = True
        pl._grip_save()
        self.assertAlmostEqual(json.loads(ms.load_grip("m"))["level"], 4.0)
        pl._grip_load("m")
        self.assertAlmostEqual(pl._grip.map.level, 4.0 * pl._grip.cfg.back_off)
        (ms.MAPS_DIR / "m.grip.json").write_text("{broken")
        pl._grip_load("m")                      # 壊れていても走れる（初期値から）
        self.assertEqual(pl._grip.map.level, 0.0)

    def _new_map(self, name):
        self.mapstore.save_map(name, resolution=0.05, origin_x=0.0, origin_y=0.0,
                               trinary=np.ones((8, 8), dtype=np.uint8), centerline_xy=np.zeros((0, 2)),
                               raceline_xy=np.zeros((0, 2)), raceline_v=np.zeros(0))

    def test_new_map_does_not_inherit_anything_from_another_map(self):
        """学習値は地図の横にしか置かない。別の地図で学習した値も、車・床の記録も、新しい地図へは持ち込まない。"""
        pl, ms = self.pl, self.mapstore
        self._new_map("a")
        pl._grip_load("a")
        g = pl._grip
        g.configure(init_level=3.5, max_level=8.0, down_only=False)
        g.long.configure(acc_init=3.0, dec_init=1.9, dec_max=4.0, down_only=False)
        g.map.level, g.map.ceil = 5.5, 6.2
        g.map.dec = [3.0, 2.8, 2.0]
        g._measured = True
        pl._grip_save()
        names = {p.name for p in ms.MAPS_DIR.iterdir()}
        self.assertIn("a.grip.json", names)
        self.assertFalse([n for n in names if n.startswith("_")], names)    # 地図に属さない記録は書かない

        self._new_map("b")
        pl._grip_load("b")
        g = pl._grip
        self.assertEqual(g.map.level, 0.0)
        self.assertEqual(g.map.ceil, 0.0)
        g.configure(init_level=3.5, max_level=8.0, down_only=False)
        self.assertAlmostEqual(g.map.level, 3.5)        # 同定値 × grip_init から

        # 学習値のある地図に戻れば、その地図の値を使う
        pl._grip_load("a")
        self.assertAlmostEqual(pl._grip.map.level, 5.5 * pl._grip.cfg.back_off)

    def test_map_best_values_start_the_next_run_without_probing(self):
        """この地図で学習を終えた値（best）は、次の走行で back_off を掛けずにそのまま使い、探索もしない。"""
        pl, ms = self.pl, self.mapstore
        self._new_map("a")
        pl._grip_load("a")
        g = pl._grip
        g.configure(init_level=3.5, max_level=8.0, down_only=False)
        g.long.configure(acc_init=3.0, dec_init=1.9, dec_max=3.0, down_only=False)
        g.map.level = 5.96
        g.map.dec = [3.0, 2.8, 2.0]
        g.converged, g._measured = True, True
        pl._grip_save()
        saved = json.loads(ms.load_grip("a"))
        self.assertAlmostEqual(saved["best_lat"], 5.96)
        self.assertEqual(saved["best_dec_env"], [3.0, 2.8, 2.0])

        pl._grip_load("a")
        g = pl._grip
        self.assertTrue(g.from_best)
        self.assertAlmostEqual(g.map.level, 5.96)        # 0.92 倍しない
        self.assertEqual(g.map.dec, [3.0, 2.8, 2.0])
        self.assertTrue(g.converged and g.long.dec_done)
        g.configure(init_level=3.5, max_level=8.0, down_only=False)
        g.long.configure(acc_init=3.0, dec_init=1.9, dec_max=3.0, down_only=False)
        self.assertAlmostEqual(g.map.level, 5.96)

    def test_map_best_follows_a_lowered_value(self):
        """終えた後に安全装置で下げたら、終え直さないまま走行をやめても best は下がっている。"""
        pl, ms = self.pl, self.mapstore
        self._new_map("a")
        pl._grip_load("a")
        g = pl._grip
        g.configure(init_level=3.5, max_level=8.0, down_only=False)
        g.long.configure(acc_init=3.0, dec_init=1.9, dec_max=3.0, down_only=False)
        g.map.level = 5.96
        g.map.dec = [3.0, 2.8, 2.0]
        g.converged, g._measured = True, True
        pl._grip_save()
        g.converged = False
        g.map.level = 5.6
        pl._grip_save()
        self.assertAlmostEqual(json.loads(ms.load_grip("a"))["best_lat"], 5.6)

if __name__ == "__main__":
    unittest.main()

