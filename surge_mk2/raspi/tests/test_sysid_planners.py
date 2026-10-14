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


class TestSteerShuttle(unittest.TestCase):
    """ステア試験の階段部・大振幅部（2026-10-13、時間で区切る往復からオドメトリの距離で区切る往復へ）。"""

    AMAX = Vehicle.load().max_steer
    WHEELBASE = Vehicle.load().wheelbase

    def _run(self, reverse_lag_s: float = 0.15):
        """走行部を飛ばして階段部・大振幅部だけを、遅れのある車（速度は1次遅れ、舵は7rad/sの
        ランプ）で回す。返り値は判断ごとの `(reason, 舵の指令, 速度の指令, 車速, x, y, yaw, 舵角)`。"""
        pl = SysIdSteer()
        pl.settings["roll_speed"] = 0.0
        pl.set_engaged(True)
        x = y = yaw = v = steer = odom = 0.0
        t_ns, out = 0, []
        cmd_v = cmd_d = 0.0
        for _ in range(3000):
            vs = VehicleState(t_capture=t_ns, speed=v, steer_actual=steer, steer_cmd_echo=steer,
                              odom_dist=[odom, odom], wheel_speed=[v, v, v, v], armed=True)
            st = pl.plan(None, vs, {}, 0.1)
            if st.reason == "完了":
                return out
            cmd_v, cmd_d = st.target_speed, st.target_steer
            out.append((st.reason, cmd_d, cmd_v, v, x, y, yaw, steer))
            for _k in range(100):            # 1kHz で車を進める
                h = 0.001
                v += (cmd_v - v) * h / reverse_lag_s
                steer += max(-7.0 * h, min(7.0 * h, cmd_d - steer))
                ds = v * h
                yaw += ds * math.tan(steer) / self.WHEELBASE
                x += ds * math.cos(yaw)
                y += ds * math.sin(yaw)
                odom += ds / math.cos(steer)          # 前輪の走行距離
                t_ns += 1_000_000
                if _k % 20 == 19:
                    pl.on_vehicle_state(VehicleState(
                        t_capture=t_ns, speed=v, steer_actual=steer, steer_cmd_echo=steer,
                        odom_dist=[odom, odom], wheel_speed=[v, v, v, v], armed=True))
        raise AssertionError("終わらなかった")

    def test_steer_changes_only_while_rolling(self):
        """舵の指令が変わるのは車が転がっている間だけ（止まった車で舵を切らない）。
        最初の段だけは、転がり出すのと同時に切る。"""
        tr = self._run()
        tr = tr[next(i for i, r in enumerate(tr) if r[0].startswith("階段")):]
        for a, b in zip(tr, tr[1:]):
            if abs(b[1] - a[1]) > 1e-9:
                self.assertGreater(abs(b[3]), 0.1, b[0])

    def test_stair_holds_the_steer_through_the_forward_leg(self):
        """階段の前進区間は丸ごと一定の舵角（舵が動いている最中の遅れを曲率に混ぜない）。"""
        tr = [r for r in self._run() if r[0].startswith("階段")]
        for a, b in zip(tr, tr[1:]):
            if a[2] > 0 and b[2] > 0:
                self.assertAlmostEqual(a[1], b[1], places=9)

    def test_returns_to_the_start_whatever_the_reversal_takes(self):
        """前進した道を同じ舵角で戻るので、反転の遅さによらず元の位置・向きの近くで終わる
        （時間で区切っていた頃は、実機で階段部だけで約0.45mずれた）。"""
        for lag in (0.05, 0.15, 0.3):
            tr = self._run(lag)
            xs, ys = [r[4] for r in tr], [r[5] for r in tr]
            self.assertLess(math.hypot(tr[-1][4], tr[-1][5]), 0.15, lag)
            self.assertLess(abs(math.degrees(tr[-1][6])), 15.0, lag)
            # 掃く範囲（車体原点）も狭い: 前後は中立付近の長い往復＋行き過ぎ、左右は弧のふくらみ
            self.assertLess(max(xs) - min(xs), 0.75, lag)
            self.assertLess(max(ys) - min(ys), 0.25, lag)

    def test_forward_leg_length_does_not_depend_on_the_reversal(self):
        """前進区間の長さ（＝定常の観測の量）は距離で決まる。反転が遅い車でも短くならない。"""
        for lag in (0.05, 0.3):
            first = [r for r in self._run(lag) if r[0].startswith("階段 1/")]   # 舵いっぱいの段
            far = max(math.hypot(r[4], r[5]) for r in first)
            self.assertGreater(far, 0.8 * SysIdSteer.SETTINGS["stair_leg_m"] * math.cos(self.AMAX), lag)

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
        self.assertTrue(any(0 < abs(x) <= math.radians(3.0) for x in levels))

    def test_big_steps_happen_both_forward_and_backward(self):
        """大振幅のステップは前進中にも後退中にもある（前進中のものが低速のヨーの過渡の材料）。"""
        tr = [r for r in self._run() if r[0].startswith("低速")]
        big = [b for a, b in zip(tr, tr[1:]) if abs(b[1] - a[1]) > math.radians(40.0)]
        self.assertGreaterEqual(sum(b[3] > 0.1 for b in big), 2)
        self.assertGreaterEqual(sum(b[3] < -0.1 for b in big), 2)


class TestCornerRamp(unittest.TestCase):
    """旋回試験は速度を一定の割合で上げ、限界（横加速度が頭打ち・後輪が流れた）に達したら、舵を
    保ったまま目標速度0へゆっくり止める（2026-09-27）。"""

    def _run(self, a_lat_cap: float | None, slip_above: float | None = None,
             oversteer_above: float | None = None):
        from raspi.msgs.types import Scan
        pl = self._last = SysIdCorner()
        prm = {s.key: s.default for s in SysIdCorner.params}
        pl.set_engaged(True)
        scan = Scan(dist=[4.0] * 360, sector_seen=[True] * 12)
        kappa = 1.0 / 0.42
        t_ns, v, speeds = 0, 0.0, []
        for _ in range(1000):
            st = pl.plan(scan, VehicleState(armed=True, speed=v, stopped=v < 0.05), prm, 0.1)
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

    def test_clearance_stop_ends_the_run_and_does_not_resume(self):
        """周囲クリアランスで止まったら、空いても走り出さない（停止前の速度へ全開で戻らない）。"""
        from raspi.msgs.types import Scan
        pl = SysIdCorner()
        prm = {s.key: s.default for s in SysIdCorner.params}
        pl.set_engaged(True)
        far = Scan(dist=[4.0] * 360, sector_seen=[True] * 12)
        near = Scan(dist=[0.2] * 360, sector_seen=[True] * 12)
        state = VehicleState(armed=True, stopped=False)
        for _ in range(100):
            st = pl.plan(far, state, prm, 0.1)
        self.assertGreater(st.target_speed, 0.5)
        st = pl.plan(near, state, prm, 0.1)
        self.assertTrue(st.brake)
        self.assertEqual(st.brake_torque, ABORT_BRAKE_TORQUE_NM)
        self.assertNotEqual(st.target_steer, 0.0)              # 舵は保つ
        for _ in range(5):
            st = pl.plan(far, state, prm, 0.1)                 # 空いた
            self.assertTrue(st.brake)
            self.assertEqual(st.target_speed, 0.0)
            self.assertIn("中止", st.reason)
        # 試験開始を押し直せば最初から
        pl.set_engaged(False)
        pl.set_engaged(True)
        st = pl.plan(far, state, prm, 0.1)
        self.assertFalse(st.brake)

    def test_steer_returns_to_neutral_after_standing_still(self):
        """止まって `StandstillSteer.HOLD_S` 経ったら舵を中立へ戻す（据え切りの過熱）。動いている間は保つ。"""
        from raspi.auto.base import StandstillSteer
        from raspi.msgs.types import Scan
        pl = SysIdCorner()
        prm = {s.key: s.default for s in SysIdCorner.params}
        pl.set_engaged(True)
        far = Scan(dist=[4.0] * 360, sector_seen=[True] * 12)
        near = Scan(dist=[0.2] * 360, sector_seen=[True] * 12)
        moving = VehicleState(armed=True, stopped=False, speed=0.5)
        for _ in range(30):
            pl.plan(far, moving, prm, 0.1)
        st = pl.plan(near, moving, prm, 0.1)             # 中止（まだ動いている）
        self.assertNotEqual(st.target_steer, 0.0)
        still = VehicleState(armed=True, stopped=True)
        n = int(StandstillSteer.HOLD_S / 0.1) - 2
        for _ in range(n):
            st = pl.plan(far, still, prm, 0.1)
            self.assertNotEqual(st.target_steer, 0.0)    # 停車の直後は保つ
        for _ in range(5):
            st = pl.plan(far, still, prm, 0.1)
        self.assertEqual(st.target_steer, 0.0)
        self.assertTrue(st.brake)                        # 制動は続ける
        st = pl.plan(far, moving, prm, 0.1)              # 動いたら数え直し（保持へ戻る）
        self.assertNotEqual(st.target_steer, 0.0)

    def test_samples_stop_accumulating_after_the_run(self):
        st, _ = self._run(a_lat_cap=4.0)
        pl = self._last
        n = len(pl._t_s)
        for k in range(50):
            pl.on_vehicle_state(VehicleState(t_capture=10**12 + k, speed=0.0, armed=True))
        self.assertEqual(len(pl._t_s), n)

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


class TestSensorGuardArmed(unittest.TestCase):
    def test_md_status_before_arm_is_ignored(self):
        """試験開始から ARM までの MD 起動待ちの `md_status` で fault を立てない。"""
        g = SensorGuard()
        ok, lost = 0x31, 0x01
        for k in range(50):
            g.update(VehicleState(t_capture=10**9 + k, md_status=[ok, lost, ok], armed=False))
        self.assertIsNone(g.fault)
        g.update(VehicleState(t_capture=10**10, md_status=[ok, ok, ok], armed=True))
        self.assertIsNone(g.fault)

    def test_md_only_guard_ignores_wheel_mismatch(self):
        """`wheel_check=False`（タイヤ試験）は後輪の空転で止めないが、MD の無応答では止める。"""
        g = SensorGuard(wheel_check=False)
        for k in range(100):
            g.update(VehicleState(t_capture=10**9 + k * 10_000_000, speed=0.1,
                                  wheel_speed=[0.1, 0.1, 2.0, 2.0], armed=True))
        self.assertIsNone(g.fault)
        g.update(VehicleState(t_capture=10**11, md_status=[0x31, 0x01, 0x31], armed=True))
        self.assertIsNotNone(g.fault)


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
