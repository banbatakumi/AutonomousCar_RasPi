"""`raspi/auto/sysid_*.py` の、閉ループのベンチ（`tools/sysid/bench.py`）では見えにくい約束。"""

import math
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from raspi.auto._sysid_common import (ABORT_BRAKE_TORQUE_NM, SETTLE_S, DeadReckon, OdomRun,  # noqa: E402
                                     SensorGuard)
from raspi.auto._sysid_common import TestGate as Gate  # noqa: E402
from raspi.auto.registry import PLANNERS  # noqa: E402
from raspi.auto.sysid_corner import SysIdCorner  # noqa: E402
from raspi.auto.sysid_steer import SysIdSteer  # noqa: E402
from raspi.core.vehicle import Vehicle  # noqa: E402
from raspi.msgs.types import VehicleState  # noqa: E402


class TestCreepDirection(unittest.TestCase):
    def test_direction_never_flips_at_a_steer_step(self):
        """低速部の前後の切り替えは舵のステップ（holdの整数倍）と重ならない
        （重なると、そのステップは実質据え切りになる）。"""
        hold = 1.0
        flips = []
        prev = SysIdSteer.creep_sign(0.0, hold)
        for k in range(1, 9000):
            t = k * 0.001
            sg = SysIdSteer.creep_sign(t, hold)
            if sg != prev:
                flips.append(t)
            prev = sg
        self.assertTrue(flips)
        for t in flips:
            frac = (t / hold) % 1.0
            self.assertAlmostEqual(frac, 0.5, delta=0.01)


class TestSteerStair(unittest.TestCase):
    """ステア試験の階段部（2026-09-27、掃引から置き換え）。"""

    P = SysIdSteer.SETTINGS
    AMAX = math.radians(30.0)

    def _trace(self):
        dt = 0.01
        n = int(SysIdSteer.stair_duration(self.P, self.AMAX) / dt)
        return [SysIdSteer.stair_point(self.P, self.AMAX, k * dt) for k in range(n)]

    def test_steer_changes_only_while_reversing(self):
        """舵は後退中にしか変わらない（止まった車で舵を切らない・前進区間は一定の舵角）。
        最初の段だけは、転がり出すのと同時に切る。"""
        tr = self._trace()
        for (d0, _), (d1, v1) in zip(tr, tr[1:]):
            if abs(d1 - d0) > 1e-12:
                self.assertLess(v1, 0.0)

    def test_every_level_is_approached_from_both_sides(self):
        """どの段（両端を除く）にも上りと下りの両方から近づく（上りと下りの曲率の差＝ガタ）。"""
        levels = SysIdSteer.stair_levels(self.AMAX)
        seen: dict[float, set[int]] = {}
        for prev, cur in zip(levels, levels[1:]):
            seen.setdefault(round(cur, 6), set()).add(1 if cur > prev else -1)
        inner = [x for x in levels if abs(abs(x) - self.AMAX) > 1e-9]
        for x in inner:
            self.assertEqual(seen.get(round(x, 6)), {1, -1}, math.degrees(x))
        # 中立付近（不感帯が効く所）に段がある
        self.assertTrue(any(0 < abs(x) <= math.radians(2.0) for x in levels))


class TestCornerRamp(unittest.TestCase):
    """旋回試験は速度を一定の割合で上げ、限界（横加速度が頭打ち・後輪が流れた）に達したら、舵を
    保ったまま目標速度0へゆっくり止める（2026-09-27）。"""

    def _run(self, a_lat_cap: float | None, slip_above: float | None = None,
             oversteer_above: float | None = None):
        from raspi.msgs.types import Scan
        pl = SysIdCorner()
        prm = {s.key: s.default for s in SysIdCorner.params}
        pl.set_engaged(True)
        scan = Scan(dist=[4.0] * 360, sector_seen=[True] * 12)
        kappa = 1.0 / 0.42
        t_ns, v, speeds = 0, 0.0, []
        for _ in range(1000):
            st = pl.plan(scan, VehicleState(armed=True, speed=v), prm, 0.1)
            if st.reason.startswith("完了"):
                return st, speeds
            v = st.target_speed
            speeds.append(v)
            for _k in range(10):             # 100Hz の全サンプル
                t_ns += 10_000_000
                a = v * v * kappa if a_lat_cap is None else min(v * v * kappa, a_lat_cap)
                if oversteer_above is not None and v > oversteer_above:
                    a *= 1.3                 # 後輪が流れて内側へ巻き込む（曲率が増える）
                rear = v * 1.2 if slip_above is not None and v > slip_above else v
                pl.on_vehicle_state(VehicleState(t_capture=t_ns, speed=v, yaw_rate=a / max(v, 1e-3),
                                                 wheel_speed=[v, v, rear, rear], armed=True))
        raise AssertionError("終わらなかった")

    def test_speed_ramps_and_stops_at_saturation(self):
        st, speeds = self._run(a_lat_cap=4.0)
        self.assertEqual(st.reason, "完了（グリップ限界: 横加速度が頭打ち）")
        moving = [x for x in speeds if x > 0]
        steps = {round(b - a, 6) for a, b in zip(moving, moving[1:])}
        self.assertEqual(steps, {round(SysIdCorner.SETTINGS["ramp_m_s2"] * 0.1, 6)})
        v_limit = math.sqrt(4.0 * 0.42)
        self.assertGreater(max(speeds), v_limit)
        self.assertLess(max(speeds), v_limit + 0.2)        # 限界の直後で止まる

    def test_wheelspin_alone_is_not_the_limit(self):
        """後輪が空転しても、車体が滑っていなければ（横加速度が伸び、曲率も変わらない）続ける。
        実機で内輪の空転だけで打ち切り、mu が低く出た（2026-10-08）。"""
        st, speeds = self._run(a_lat_cap=None, slip_above=1.2)
        self.assertTrue(st.reason.startswith("完了（上限速度まで"), st.reason)

    def test_rear_sliding_out_is_the_limit(self):
        """後輪が流れて曲率が跳ねたら（横加速度は伸び続けていても）限界。"""
        st, speeds = self._run(a_lat_cap=None, oversteer_above=1.2)
        self.assertEqual(st.reason, "完了（グリップ限界: 後輪が流れた）")
        self.assertLess(max(speeds), 1.2 + 0.15)

    def test_stops_gently_holding_the_steer(self):
        """限界の後は制動せず、舵を保ったまま目標速度0へゆっくり（最大制動は旋回中に後輪をロックさせた）。"""
        st, _ = self._run(a_lat_cap=4.0)
        self.assertTrue(st.ready)
        self.assertFalse(st.brake)
        self.assertEqual(st.target_speed, 0.0)
        # 舵いっぱい（設定の 30° は、リンクの換算後の最大舵角 `max_steer` で頭打ちになる）
        want = min(math.radians(SysIdCorner.SETTINGS["steer_deg"]), Vehicle.load().max_steer)
        self.assertAlmostEqual(abs(st.target_steer), want, places=6)
        self.assertEqual(st.accel_limit, SysIdCorner.SETTINGS["stop_decel_m_s2"])

    def test_without_saturation_runs_to_v_max(self):
        st, speeds = self._run(a_lat_cap=None)
        self.assertTrue(st.reason.startswith("完了（上限速度まで"), st.reason)


class TestSettle(unittest.TestCase):
    def test_every_sysid_planner_starts_still(self):
        """全ての同定試験は開始直後に止まる（解析がジャイロのバイアスを読む区間）。"""
        vs = VehicleState(armed=True)
        for pid, cls in PLANNERS.items():
            if cls.category != "sysid":
                continue
            pl = cls()
            prm = {s.key: s.default for s in cls.params}
            pl.set_engaged(True)
            for _ in range(int(SETTLE_S / 0.1)):
                st = pl.plan(None, vs, prm, 0.1)
                self.assertEqual(st.target_speed, 0.0, pid)
                self.assertFalse(st.brake, pid)
                self.assertTrue(st.reason.startswith("静止"), f"{pid}: {st.reason}")

    def test_restart_settles_again(self):
        g = Gate()
        g.set_engaged(True)
        while g.settling(0.1):
            pass
        g.set_engaged(False)
        g.set_engaged(True)
        self.assertTrue(g.settling(0.1))


class TestDeadReckon(unittest.TestCase):
    def test_bias_is_learned_only_while_told_stationary(self):
        dr = DeadReckon()
        t = 0
        dr.stationary = True
        for _ in range(50):
            t += 20_000_000
            dr.update(VehicleState(t_capture=t, speed=0.03, yaw_rate=0.02))   # 速度にノイズがあっても
        self.assertAlmostEqual(dr.bias, 0.02, delta=1e-9)
        dr.stationary = False
        odom = 0.0
        for _ in range(500):                  # 10s 直進（ヨーはバイアスだけ）
            t += 20_000_000
            odom += 0.02
            # speed はローパスで遅れた値が来る想定。距離はオドメトリの増分から積む
            dr.update(VehicleState(t_capture=t, speed=0.5, odom_dist=[odom, odom], yaw_rate=0.02))
        self.assertAlmostEqual(dr.yaw, 0.0, delta=1e-6)
        self.assertAlmostEqual(dr.x, 10.0, delta=0.05)

    def test_lane_keep_steers_back_toward_the_line(self):
        dr = DeadReckon()
        dr.y, dr.yaw = 0.2, 0.0              # 左にずれている
        self.assertLess(dr.lane_keep_steer(True, math.radians(2), 0.23), 0.0)   # 前進は右へ
        dr.y, dr.yaw = 0.0, 0.1              # 左を向いている
        self.assertLess(dr.lane_keep_steer(True, math.radians(2), 0.23), 0.0)
        self.assertGreater(dr.lane_keep_steer(False, math.radians(2), 0.23), 0.0)  # 後退は逆


def _vs(t_ns: int, front: float, rear: float, odom: float = 0.0) -> VehicleState:
    return VehicleState(t_capture=t_ns, speed=front, wheel_speed=[front, front, rear, rear],
                        odom_dist=[odom, odom], armed=True)


class TestSensorGuard(unittest.TestCase):
    """前輪エンコーダの不調で試験が暴走しない（2026-09-26、第三者検証で35m走った件）。"""

    def _feed(self, g: SensorGuard, front: float, rear: float, secs: float) -> None:
        t = getattr(self, "_t", 0)
        for _ in range(int(secs / 0.02)):
            t += 20_000_000
            g.update(_vs(t, front, rear))
        self._t = t

    def test_frozen_front_encoder_trips(self):
        g = SensorGuard()
        self._feed(g, 0.0, 1.0, 0.25)
        self.assertIsNone(g.fault)             # 0.3s 続くまでは待つ（一瞬の食い違いで止めない）
        self._feed(g, 0.0, 1.0, 0.1)
        self.assertIsNotNone(g.fault)

    def test_reverse_is_checked_too(self):
        g = SensorGuard()
        self._feed(g, 0.0, -0.6, 0.5)
        self.assertIsNotNone(g.fault)

    def test_traction_slip_and_brake_lock_do_not_trip(self):
        g = SensorGuard()
        self._feed(g, 2.0, 2.6, 1.0)           # 全開加速の空転（30%）
        self._feed(g, 1.5, 0.0, 1.0)           # 制動で後輪がロック（前輪の方が速い）
        self._feed(g, 0.0, 0.1, 1.0)           # 静止付近の小さな食い違い
        self.assertIsNone(g.fault)

    def test_silent_motor_driver_trips_with_its_own_cause(self):
        """MD が黙ると車輪の値が古いまま固まる。前後輪の食い違いより先に、原因を正しく言って止める
        （2026-10-08、右後輪の MD が電圧異常で止まり「前輪エンコーダの不調？」と表示した）。"""
        ok, lost = 0x11, 0x03                  # 運転中＋通信OK / 運転中＋電圧異常（通信OK なし）
        g = SensorGuard()
        g.update(VehicleState(t_capture=1, speed=1.3, wheel_speed=[1.3, 1.3, 1.3, 1.6],
                              md_status=[ok, ok, 0x31], armed=True))
        self.assertIsNone(g.fault)
        g.update(VehicleState(t_capture=2, speed=1.3, wheel_speed=[1.3, 1.3, 1.3, 1.6],
                              md_status=[ok, lost, 0x31], armed=True))
        self.assertIn("右後輪のモータドライバが無応答", g.fault)
        self.assertIn("電圧異常", g.fault)
        self.assertNotIn("エンコーダ", g.fault)

    def test_unreported_md_status_is_not_a_fault(self):
        g = SensorGuard()
        self._feed(g, 1.0, 1.0, 0.5)           # md_status は既定の 0（状態が来ていない）
        self.assertIsNone(g.fault)

    def test_cause_distinguishes_wheelspin_from_frozen_encoder(self):
        """止めるのは同じ。前輪も動いていて後輪だけ速いなら空転の可能性を表示する（空転を入れた
        真値で「前輪エンコーダの不調？」とだけ出していた、2026-09-27）。"""
        g = SensorGuard()
        self._feed(g, 0.0, 1.0, 0.5)
        self.assertIn("前輪がほぼ止まっている", g.fault)
        g = SensorGuard()
        self._feed(g, 0.9, 2.4, 0.5)
        self.assertIn("空転", g.fault)

    def test_every_moving_planner_aborts_with_brake(self):
        """ガードが落ちたら、走る試験はすべて制動して中止し、「完了」とは言わない。"""
        for pid in ("sysid_accel", "sysid_steer", "sysid_corner"):
            pl = PLANNERS[pid]()
            prm = {sp.key: sp.default for sp in PLANNERS[pid].params}
            pl.set_engaged(True)
            pl.plan(None, _vs(0, 0.0, 0.0), prm, 0.1)   # 試験開始（ここでガードは初期化される）
            t = 0
            for _ in range(20):
                t += 20_000_000
                pl.on_vehicle_state(_vs(t, 0.0, 1.0))
            st = pl.plan(None, _vs(t, 0.0, 1.0), prm, 0.1)
            self.assertTrue(st.brake, pid)
            # 後輪がロックしない強さで制動する（最大制動は旋回中に後輪をロックさせた、2026-09-27）
            self.assertEqual(st.brake_torque, ABORT_BRAKE_TORQUE_NM, pid)
            self.assertTrue(st.reason.startswith("中止"), f"{pid}: {st.reason}")


class TestOdomRun(unittest.TestCase):
    def test_origin_is_kept_across_cycles(self):
        """開始位置は最初の1回だけ（サイクルごとに置き直すと後退の行き過ぎが積み重なる）。"""
        r = OdomRun()
        r.start(_vs(0, 0, 0, odom=0.0))
        r.start(_vs(0, 0, 0, odom=-0.1))       # 2サイクル目の頭（少し後ろへ行き過ぎた）
        self.assertAlmostEqual(r.traveled(_vs(0, 0, 0, odom=0.5)), 0.5)

    def test_return_ends_early_by_the_lead_and_timeout_is_tight(self):
        r = OdomRun()
        r.start(_vs(0, 0, 0, odom=0.0))
        r.arm_return(_vs(0, 0, 0, odom=1.2), 0.6)
        self.assertAlmostEqual(r.return_timeout_s, 1.2 / 0.6 * 1.5 + 1.0)
        self.assertTrue(r.returned(_vs(0, 0, 0, odom=0.15), 0.0))   # 0.05 + 0.2s×0.6m/s
        self.assertFalse(r.returned(_vs(0, 0, 0, odom=0.25), 0.0))


if __name__ == "__main__":
    unittest.main()
