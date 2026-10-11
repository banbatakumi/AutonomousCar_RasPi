"""速度指令の前倒し（`raspi/nav/speedctl.py`）・同定値から出す走りの限界（`Vehicle`）・
ラップ時間で仕上げるライン（`raceline.refine_time`）・余裕で決める回避の速度（`avoid.speed_cap`）のテスト。

ファームの写し（`sim.vehicle.SpeedController`）に対する追従は `sim/tests/test_speed_ff.py`。
"""

import math
import sys
import unittest
from dataclasses import replace
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

import numpy as np  # noqa: E402

from raspi.auto.slam2d_raceline import (Slam2dRaceLine, kappa_limit, limits,  # noqa: E402
                                        speed_kwargs, steer_delay)
from raspi.core.vehicle import GRAVITY, Vehicle  # noqa: E402
from raspi.nav import avoid as av  # noqa: E402
from raspi.nav import raceline as rl  # noqa: E402
from raspi.nav import speedctl  # noqa: E402


def ring(radius: float = 3.0, step: float = 0.1, v: float = 2.0) -> rl.RaceLine:
    n = int(round(2 * math.pi * radius / step))
    a = np.arange(n) * 2 * math.pi / n
    xy = np.column_stack([radius * np.cos(a), radius * np.sin(a)])
    return rl.RaceLine(xy=xy, v=np.full(n, v), kappa=np.full(n, 1 / radius),
                       length=2 * math.pi * radius)


class TestVehicleEnvelope(unittest.TestCase):
    def setUp(self):
        self.v = replace(Vehicle(), mu=0.4, drive_accel_m_s2=2.8, speed_decel_m_s2=2.2,
                         rolling_resistance=0.3, speed_kp=0.25, speed_ki=0.5,
                         speed_plant_gain=25.0, dead_time_s=0.008, tau_steer_s=0.001,
                         yaw_natural_freq_rad_s=58.0, yaw_damping=0.93)

    def test_limits_come_from_identified_values(self):
        v = self.v
        self.assertAlmostEqual(v.a_lat_max, 0.4 * GRAVITY)
        self.assertAlmostEqual(v.a_accel_max(), 2.8 - 0.3)        # 転がり抵抗を引いた正味
        self.assertAlmostEqual(v.a_decel_max, 2.2 + 0.3)          # 減速は転がり抵抗が味方
        self.assertAlmostEqual(v.speed_loop_tau_s, 1 / (25.0 * 0.25))
        self.assertAlmostEqual(v.speed_loop_reset_s, 0.5)
        self.assertAlmostEqual(v.steer_delay_s, 0.008 + 0.001 + 2 * 0.93 / 58.0)

    def test_ramp_caps_the_limits(self):
        """STM32 は目標速度を `speed_ramp_max_m_s2` でならすので、それより上は出ない。"""
        v = replace(self.v, drive_accel_m_s2=9.0, speed_decel_m_s2=9.0)
        self.assertAlmostEqual(v.a_accel_max(), v.speed_ramp_max_m_s2)
        self.assertAlmostEqual(v.a_decel_max, v.speed_ramp_max_m_s2)

    def test_drive_fades_with_speed(self):
        v = replace(self.v, drive_fade_speed_m_s=2.0, drive_top_speed_m_s=4.0)
        self.assertAlmostEqual(v.a_accel_max(1.0), 2.5)
        self.assertAlmostEqual(v.a_accel_max(3.0), 2.8 * 0.5 - 0.3)

    def test_unidentified_values_fall_back(self):
        v = Vehicle()
        self.assertGreater(v.a_lat_max, 1.0)
        self.assertGreater(v.a_accel_max(), 0.5)
        self.assertGreater(v.a_decel_max, 0.5)
        self.assertEqual(v.speed_loop_tau_s, 0.0)                 # 前倒ししない

    def test_limits_scale_by_use(self):
        p = {"lat_use": 0.5, "accel_use": 0.8, "brake_use": 0.9}
        lim = limits(p, self.v)
        self.assertAlmostEqual(lim.a_lat, 0.5 * self.v.a_lat_max)
        self.assertAlmostEqual(lim.a_accel, 0.8 * 2.5)
        self.assertAlmostEqual(lim.a_brake, 0.9 * 2.5)
        # 空走時間 = 点群の周期 + 指令が届くまで + 減速の立ち上がり（速度PIの時定数の半分）
        self.assertAlmostEqual(lim.react_s, 0.1 + 0.02 + 0.5 * self.v.speed_loop_tau_s)

    def test_planner_params_do_not_carry_derived_values(self):
        """設定の dict に上限を混ぜない（同定値から毎回出す）。GUI・保存する設定に漏れない。"""
        p = Slam2dRaceLine.merged({})
        for k in ("a_lat", "a_accel", "a_brake", "react_s"):
            self.assertNotIn(k, p)
        veh = Vehicle.load()
        kw = speed_kwargs(p, veh)
        self.assertAlmostEqual(kw["a_lat"], p["lat_use"] * veh.a_lat_max)
        self.assertAlmostEqual(kw["a_grip"], veh.a_lat_max)
        self.assertLess(kappa_limit(p, veh), veh.kappa_max)       # 既定は限界より内側
        # 舵の遅延補償は同定値＋通信ぶん。姿勢の古さ（旧 0.15s）は含めない
        self.assertLess(steer_delay(p, veh), 0.12)
        self.assertGreater(steer_delay(p, veh), veh.steer_delay_s)


class TestSpeedFF(unittest.TestCase):
    CFG = speedctl.SpeedFFConfig(tau_v=0.16, reset_s=0.5, lead_s=0.1, lead_acc_s=0.05, k_fb=1.0)

    def test_constant_speed_passes_through(self):
        path = ring(v=2.0)
        ff = speedctl.SpeedFF()
        for i in range(5):
            self.assertAlmostEqual(ff.command(path, 10.0 + i, 2.0, 0.02, self.CFG), 2.0)

    def test_disabled_returns_profile(self):
        path = ring()
        path = rl.RaceLine(**{**path._asdict(), "v": np.linspace(1.0, 3.0, len(path))})
        ff = speedctl.SpeedFF()
        got = ff.command(path, 20.5, 1.0, 0.02, speedctl.SpeedFFConfig())
        self.assertAlmostEqual(got, 0.5 * (path.v[20] + path.v[21]))

    def _ramp(self, a: float) -> rl.RaceLine:
        """点 60 から 100 まで加速度 `a` で速度が変わる経路（手前は一定: 減速なら 3.0、加速なら 1.0m/s）。"""
        path = ring(radius=5.0)
        n = len(path)
        v = np.full(n, 3.0 if a < 0 else 1.0)
        for i in range(60, 100):
            v[i] = math.sqrt(v[i - 1] ** 2 + 2 * a * 0.1)
        v[100:] = v[99]
        return rl.RaceLine(**{**path._asdict(), "v": v})

    def test_braking_starts_before_the_profile_does(self):
        """減速は `lead_s` ぶん手前から始め、比例項ぶん（`tau_v`×減速度）低い目標を送る。"""
        path = self._ramp(-1.0)
        ff = speedctl.SpeedFF()
        before = ff.command(path, 54.0, 3.0, 0.02, self.CFG)      # 0.5m 手前（lead 0.3m の外）
        self.assertAlmostEqual(before, 3.0, places=6)
        ff.reset()
        early = ff.command(path, 58.0, 3.0, 0.02, self.CFG)       # 0.1m 手前。プロファイルはまだ 3.0
        self.assertLess(early, 3.0 - 0.05)
        ff.reset()
        mid = ff.command(path, 80.0, float(path.v[80]), 0.02, self.CFG)
        self.assertLess(mid, float(path.v[80]) - 0.12)            # ≒ −tau_v×1.0 = −0.16

    def test_acceleration_is_not_led_as_far(self):
        """加速は早めない: 立ち上がりの手前 `lead_s` の点ではまだ元の速度のまま。"""
        path = self._ramp(+1.0)
        ff = speedctl.SpeedFF()
        # 加速の始まり（点 59→60 の区間）の 0.14m 手前。`lead_s`（0.1m 先）の点はもう加速度の
        # 立ち上がりに掛かっている（そのまま使えば +0.1m/s）が、採るのは目標が低い `lead_acc_s` の側
        got = ff.command(path, 57.6, 1.0, 0.02, self.CFG)
        self.assertLess(got, 1.03)
        ff.reset()
        self.assertGreater(ff.command(path, 80.0, float(path.v[80]), 0.02, self.CFG),
                           float(path.v[80]) + 0.12)

    def test_integral_takeover_fades_the_lead(self):
        """等加速度が続くと、比例項の上乗せは時定数 `reset_s` で抜ける（PI の積分が引き継ぐ）。"""
        path = self._ramp(+1.0)
        ff = speedctl.SpeedFF()
        first = ff.command(path, 70.0, float(path.v[70]), 0.02, self.CFG) - float(path.v[70])
        extra = first
        for _ in range(50):                                        # 同じ所で 1 秒
            extra = ff.command(path, 70.0, float(path.v[70]), 0.02, self.CFG) - float(path.v[70])
        self.assertGreater(first, 0.1)
        self.assertLess(extra, 0.5 * first)

    def test_overspeed_pulls_the_target_down(self):
        path = ring(v=1.0)
        ff = speedctl.SpeedFF()
        self.assertAlmostEqual(ff.command(path, 5.0, 1.5, 0.02, self.CFG), 0.5)
        self.assertEqual(ff.command(path, 5.0, 3.0, 0.02, self.CFG), 0.0)   # 負にはしない

    def test_catch_up_is_limited_by_the_grip_left(self):
        """車速がプロファイルよりずっと低いとき、目標は「摩擦円の残りで出せる加速」までに抑える。"""
        path = ring(v=3.0)
        ff = speedctl.SpeedFF()
        free = ff.command(path, 5.0, 1.0, 0.02, self.CFG)                 # 抑えなし: 全開を要求
        self.assertGreater(free, 4.0)
        ff.reset()
        some = ff.command(path, 5.0, 1.0, 0.02, self.CFG, a_room=2.0)
        self.assertAlmostEqual(some, 1.0 + 0.16 * 2.0)
        ff.reset()
        none = ff.command(path, 5.0, 1.0, 0.02, self.CFG, a_room=0.0)     # 横で使い切っている
        self.assertAlmostEqual(none, 1.0)
        # プロファイルどおりなら効かない
        ff.reset()
        self.assertAlmostEqual(ff.command(path, 5.0, 3.0, 0.02, self.CFG, a_room=2.0), 3.0)

    def test_catch_up_accelerates_inside_the_friction_circle(self):
        """低い車速から追いつく速度は、直線では `a_accel` で伸び、曲がっている所では伸びが細る。"""
        straight = ring(radius=1e6, v=3.0)
        straight = rl.RaceLine(**{**straight._asdict(), "kappa": np.zeros(len(straight)),
                                  "length": len(straight) * 0.1})
        out = speedctl.catch_up(straight, 10, 1.0, a_accel=2.0, a_grip=4.0)
        self.assertAlmostEqual(float(out.v[10]), 1.0)
        self.assertAlmostEqual(float(out.v[20]), math.sqrt(1.0 + 2 * 2.0 * 1.0), places=3)
        self.assertAlmostEqual(float(out.v[9]), 3.0)               # 後ろは元のまま
        self.assertAlmostEqual(float(out.v[40]), 3.0)              # 追いついた先も元のまま
        curve = ring(radius=1.0, v=3.0)                            # 曲率 1.0: 2m/s で横 4.0＝限界
        out = speedctl.catch_up(curve, 10, 1.0, a_accel=2.0, a_grip=4.0)
        self.assertLess(float(out.v[30]), 2.0 + 0.02)              # 限界の速度（2m/s）で頭打ち
        self.assertLess(float(out.v[20]), math.sqrt(1.0 + 2 * 2.0 * 1.0) - 0.05)
        self.assertEqual(len(out), len(curve))

    def test_path_change_does_not_kick_the_highpass(self):
        ff = speedctl.SpeedFF()
        a, b = self._ramp(-1.0), ring(v=2.0)
        ff.command(a, 80.0, float(a.v[80]), 0.02, self.CFG)
        self.assertAlmostEqual(ff.command(b, 80.0, 2.0, 0.02, self.CFG), 2.0)

    def test_arc_index_interpolates_between_points(self):
        path = ring()
        mid = 0.5 * (path.xy[10] + path.xy[11])
        self.assertAlmostEqual(speedctl.arc_index(path, 10, *mid), 10.5, places=2)
        self.assertAlmostEqual(speedctl.arc_index(path, 11, *mid), 10.5, places=2)
        self.assertAlmostEqual(speedctl.arc_index(path, 10, *path.xy[10]), 10.0, places=6)

    def test_path_accel_matches_the_profile_slope(self):
        path = self._ramp(-1.0)
        acc = speedctl.path_accel(path)
        self.assertAlmostEqual(float(acc[80]), -1.0, places=2)
        self.assertAlmostEqual(float(acc[10]), 0.0, places=6)


class TestRefCommand(unittest.TestCase):
    """STM32 の加速度フィードフォワード（★v0.21）が有効なときの指令（`speedctl.ref_command`）。"""

    def _ramp(self, a: float) -> rl.RaceLine:
        path = ring(radius=5.0)
        v = np.full(len(path), 3.0 if a < 0 else 1.0)
        for i in range(60, 100):
            v[i] = math.sqrt(v[i - 1] ** 2 + 2 * a * 0.1)
        v[100:] = v[99]
        return rl.RaceLine(**{**path._asdict(), "v": v})

    def test_sends_the_profile_and_its_slope(self):
        """目標はプロファイルそのもの（比例項の逆算もずれの戻しも足さない）、ランプはその傾き。"""
        path = self._ramp(-1.0)
        v, ramp = speedctl.ref_command(path, 80.0, float(path.v[80]), lead_s=0.0)
        self.assertAlmostEqual(v, float(path.v[80]))
        self.assertAlmostEqual(ramp, speedctl.REF_ACCEL_GAIN * 1.0, places=2)
        # 指令が効くころ（lead 秒後）に居る場所の速度を送る
        v2, _ = speedctl.ref_command(path, 80.0, float(path.v[80]), lead_s=0.05)
        self.assertLess(v2, v)

    def test_cruise_uses_the_minimum_ramp(self):
        path = ring(v=2.0)
        v, ramp = speedctl.ref_command(path, 5.0, 2.0, lead_s=0.04)
        self.assertAlmostEqual(v, 2.0)
        self.assertAlmostEqual(ramp, speedctl.REF_ACCEL_MIN)

    def test_ramp_opens_up_when_off_the_profile(self):
        """車速がプロファイルより速すぎるときは、差を `REF_CATCH_S` で詰められるランプまで許す。"""
        path = ring(v=1.0)
        v, ramp = speedctl.ref_command(path, 5.0, 2.0, lead_s=0.0)
        self.assertAlmostEqual(v, 1.0)
        self.assertAlmostEqual(ramp, 1.0 / speedctl.REF_CATCH_S)

    def test_planner_uses_it_only_when_the_firmware_feeds_forward(self):
        veh = Vehicle.load()
        self.assertIsInstance(veh.speed_ff_firmware, bool)
        self.assertFalse(replace(veh, speed_ff_firmware=False).speed_ff_firmware)


class TestRefineTime(unittest.TestCase):
    def test_reduces_cost_and_never_accepts_a_violation(self):
        """こぶを足して目的を下げる。制約を破る解（`cost` の先頭が大きい）は1度も採らない。"""
        path = ring(radius=3.0)
        seen = []

        def cost(q):
            r = np.hypot(q[:, 0], q[:, 1])
            bad = int(r.min() < 2.5)                               # 内側の壁
            length = float(np.hypot(*(np.roll(q, -1, axis=0) - q).T).sum())
            seen.append(bad)
            return bad, length

        out = rl.refine_time(path.xy, cost, step=0.1, max_evals=3000)
        r = np.hypot(out[:, 0], out[:, 1])
        self.assertGreaterEqual(r.min(), 2.5 - 1e-9)
        self.assertLess(r.mean(), 2.6)                             # 短くなる側（内側の壁）へ寄った
        self.assertEqual(cost(out)[0], 0)

    def test_respects_the_evaluation_budget(self):
        calls = []

        def cost(q):
            calls.append(1)
            return 0, float(np.hypot(*(np.roll(q, -1, axis=0) - q).T).sum())

        rl.refine_time(ring().xy, cost, step=0.1, max_evals=50)
        self.assertLessEqual(len(calls), 50 + 8)                   # 幅ごとの打ち直しの評価ぶん

    def test_short_paths_are_left_alone(self):
        xy = ring().xy[:10]
        self.assertIs(rl.refine_time(xy, lambda q: (0, 0.0), step=0.1), xy)

    def test_tight_sections_reports_what_cannot_be_turned(self):
        ok = rl.tight_sections(ring(radius=1.0).xy, True, 2.0)
        self.assertEqual(ok[0], 0.0)
        self.assertAlmostEqual(ok[1], 0.5, delta=0.02)
        bad = rl.tight_sections(ring(radius=0.4).xy, True, 2.0)
        self.assertGreater(bad[0], 2.0)                            # ほぼ全周
        self.assertAlmostEqual(bad[1], 1.25, delta=0.05)

    def test_friction_circle_uses_the_grip_limit(self):
        """`a_lat` を限界より下げても、頂点で前後に使える力は 0 にならない（`a_grip`）。"""
        path = ring(radius=3.0)
        kw = dict(v_max=3.0, v_min=0.2, a_lat=3.0, a_accel=2.0, a_brake=2.0)
        # 曲率がゆっくり変わる経路（コーナーの中で加減速する）
        k = 0.8 + 0.6 * np.sin(np.arange(len(path)) * 2 * math.pi / len(path))
        own = rl.speed_profile(path.xy, k, **kw)
        grip = rl.speed_profile(path.xy, k, a_grip=4.0, **kw)
        self.assertTrue(np.all(grip >= own - 1e-9))
        self.assertGreater(float((grip - own).max()), 0.01)


class TestAvoidSpeedCap(unittest.TestCase):
    def test_cap_grows_with_clearance(self):
        self.assertAlmostEqual(av.speed_cap(0.05, 0.05, 1.0, 3.0), 1.0)
        self.assertAlmostEqual(av.speed_cap(0.02, 0.05, 1.0, 3.0), 1.0)
        self.assertAlmostEqual(av.speed_cap(0.05 + av.CAP_FREE_M, 0.05, 1.0, 3.0), 3.0)
        self.assertAlmostEqual(av.speed_cap(0.05 + av.CAP_FREE_M / 2, 0.05, 1.0, 3.0), 2.0)
        self.assertAlmostEqual(av.speed_cap(0.05, 0.05, 2.0, 1.5), 1.5)   # 最高速度が低ければそれ


if __name__ == "__main__":
    unittest.main()
