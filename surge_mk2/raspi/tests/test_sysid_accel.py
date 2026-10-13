"""`raspi/auto/sysid_accel.py`（システム同定: 前後運動試験）のテスト。

実車の力学は使わず、`VehicleState` の `speed`・`odom_dist` を手で動かして
プランナーの状態機械（段ごとに 加速→定速→減速→停止→戻り）だけを見る。
閉ループでの挙動（使う直線の長さ・同定値の復元）は `tools/sysid/bench.py` が見る。
"""

import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from raspi.auto._sysid_common import ABORT_BRAKE_TORQUE_NM, SETTLE_S  # noqa: E402
from raspi.auto.sysid_accel import ACCEL_FRACTIONS, N_STAGES, SysIdAccel  # noqa: E402
from raspi.msgs.types import VehicleState  # noqa: E402

DT = 0.1      # planning_node は新しいスキャン（10Hz）ごとに plan() を呼ぶ
P = {"v_max": 1.0, "run_length_m": 3.0, "hold_s": 0.3, "return_speed": 0.5}
#: 最初の全開・最大制動＋割り振った段ごとにブレーキ・速度指令0の2回＋最後に全開・最大制動を2回
N_CYCLES = 1 + 2 * (N_STAGES - 1) + 2
#: ブレーキで止めるサイクル（1始まり）と、割り振った段のブレーキのサイクル
BRAKE_CYCLES = [1, 2, 4, 6, 8, 9]
PLACED_BRAKE_CYCLES = [2, 4, 6]


def _vs(odom: float, speed: float = 0.0, armed: bool = True, **kw) -> VehicleState:
    return VehicleState(odom_dist=[odom, odom], speed=speed, armed=armed,
                        wheel_speed=[0.0, 0.0, speed, speed], **kw)


class _FakeCar:
    """指令に一定加速度（加速2.0・減速3.0m/s²）で追従するだけの車。状態機械を最後まで回すため。
    減速はプランナーが停止距離の見積もりに使う値（3.0×0.8）より強い＝実機（約3〜4m/s²）相当。"""

    def __init__(self) -> None:
        self.v = 0.0
        self.odom = 0.0

    def step(self, target: float, brake: bool, dt: float) -> None:
        goal = 0.0 if brake else target
        speeding_up = abs(goal) > abs(self.v)
        a = (2.0 if speeding_up else 3.0) * dt
        self.v += max(-a, min(a, goal - self.v))
        self.odom += self.v * dt

    def vs(self, **kw) -> VehicleState:
        return _vs(self.odom, self.v, **kw)


def _skip_settle(p: SysIdAccel, params: dict = P) -> None:
    """試験開始直後の静止区間（`SETTLE_S`）を進める。"""
    for _ in range(int(SETTLE_S / DT) + 1):
        p.plan(scan=None, vs=_vs(0.0), p=params, dt=DT)


def _run(p: SysIdAccel, params: dict, n: int = 3000, spin_from=None, mode: str = "spin",
         abs_in_cycle: int | None = None) -> list:
    """最後まで回す。`spin_from` が (段, 秒) なら、その段の加速が始まってからその秒数以降、`mode`:
    "spin" 後輪を前輪より大きく速く回す（抑えきれない空転）、"slip" 後輪を15%速く（滑った）、
    "tc_flag" TC の旗だけ立てる（後輪と前輪は同じ）。`abs_in_cycle` はそのサイクル（0始まり）の
    減速中に ABS の旗を立てる。"""
    car = _FakeCar()
    out = []
    t_accel = 0.0
    for _ in range(n):
        st = p.plan(scan=None, vs=car.vs(), p=params, dt=DT)
        out.append((st, car.v, car.odom))
        if st.reason.startswith("完了"):
            break
        car.step(st.target_speed, st.brake, DT)
        t_accel = t_accel + DT if st.reason.startswith("加速") else 0.0
        spin = (spin_from is not None and st.reason.startswith("加速")
                and p.stage_of(p._cycle_i) == spin_from[0] and t_accel >= spin_from[1])
        for k in range(3):     # plan() の間の全サンプル（100Hz 相当）
            vs = car.vs(tc_active=spin, t_capture=int((len(out) * 3 + k) * 3.3e7),
                        abs_active=st.reason.startswith("減速") and p._cycle_i == abs_in_cycle)
            if spin and mode == "spin":
                vs.wheel_speed = [0.0, 0.0, car.v + 1.0, car.v + 1.0]
            elif spin and mode == "slip":
                vs.wheel_speed = [0.0, 0.0, car.v * 1.15, car.v * 1.15]
            p.on_vehicle_state(vs)
    return out


class TestSysIdAccel(unittest.TestCase):
    def test_not_engaged_or_armed_reports_waiting(self):
        p = SysIdAccel()
        st = p.plan(scan=None, vs=None, p=P, dt=DT)
        self.assertEqual(st.reason, "試験開始を押してください")

        p.set_engaged(True)
        st = p.plan(scan=None, vs=_vs(0.0, armed=False), p=P, dt=DT)
        self.assertEqual(st.reason, "ARM待ち（Enterを押してください）")

    def test_full_throttle_first_then_stages_below_the_measured_max(self):
        """最初の段は全開（目標を `v_max` より上、`accel_limit` は指定しない）。残りの段は、そこで
        読んだ最大の加速度の `ACCEL_FRACTIONS` 倍で、目標をその速さで上げ `accel_limit` も同じ値
        （ファームのランプと揃う）。最後にもう2回全開。"""
        p = SysIdAccel()
        p.set_engaged(True)
        trace = _run(p, P)
        accel = [st for st, _, _ in trace if st.reason.startswith("加速")]
        seen = list(dict.fromkeys(st.accel_limit for st in accel))
        # 車（`_FakeCar`）の加速は 2.0m/s²。全開の段の車速の傾きからそれを読む
        self.assertAlmostEqual(p.accel_max, 2.0, delta=0.2)
        want = [round(f * p.accel_max, 2) for f in ACCEL_FRACTIONS]
        self.assertEqual(seen, [0.0] + want)
        self.assertEqual(accel[-1].accel_limit, 0.0)                 # 最後のサイクルは全開
        self.assertTrue(accel[-1].reason.endswith(f" {N_CYCLES}/{N_CYCLES}"))
        self.assertLess(max(want), p.accel_max)
        for st in accel:
            if st.accel_limit > 0:
                self.assertLessEqual(st.target_speed, P["v_max"] + 1e-9)
            else:
                self.assertGreater(st.target_speed, P["v_max"])
        # 段の加速度のランプ: 1周ごとに `a`×dt ずつ増える（最初の周から正＝目標0を挟まない）
        first = [st.target_speed for st in accel if st.reason.endswith(f" 2/{N_CYCLES}")]
        self.assertAlmostEqual(first[0], want[0] * DT)
        self.assertAlmostEqual(first[1] - first[0], want[0] * DT)

    def test_decel_alternates_brake_and_speed_command_zero(self):
        """割り振った段の1回目はブレーキ、2回目は速度指令0で減速する（2種類の減速度を測るため）。
        全開の回はどれもブレーキ（速度指令0では2mの直線で最高速まで出せない）。"""
        p = SysIdAccel()
        p.set_engaged(True)
        decel = [(st.brake, st.target_speed, st.reason) for st, _, _ in _run(p, P)
                 if st.reason.startswith("減速")]
        for k in range(1, N_CYCLES + 1):
            cyc = [d for d in decel if d[2].endswith(f" {k}/{N_CYCLES}")]
            self.assertTrue(cyc, k)
            if k in BRAKE_CYCLES:
                self.assertTrue(all(b for b, _, _ in cyc), k)
            else:
                self.assertTrue(all((not b) and t == 0.0 for b, t, _ in cyc), k)

    def test_stays_within_run_length(self):
        """`v_max`・定速に届かない短い直線でも、前進距離が`run_length_m`に収まる。"""
        params = dict(P, v_max=2.0, run_length_m=1.2)
        p = SysIdAccel()
        p.set_engaged(True)
        trace = _run(p, params)
        # 各サイクルは開始位置（odom≈0、戻りの許容誤差0.05m以内）から始まる
        max_fwd = max(odom for _, _, odom in trace)
        self.assertLessEqual(max_fwd, params["run_length_m"] + 0.05)
        self.assertEqual(trace[-1][0].reason, "完了")

    def _brake_by_cycle(self, trace) -> dict[int, float]:
        out: dict[int, float] = {}
        for st, _, _ in trace:
            if st.reason.startswith("減速") and st.brake:
                out[int(st.reason.split(" ")[-1].split("/")[0])] = st.brake_torque
        return out

    def test_max_brake_first_then_levels_below_it(self):
        """最初（と最後）のブレーキは最大。残りの段は、減速度がその `BRAKE_FRACTIONS` 倍になる強さを
        その場で決める（弱い順。どれも最大より弱い）。"""
        p = SysIdAccel()
        p.set_engaged(True)
        nm = self._brake_by_cycle(_run(p, P))
        self.assertEqual(sorted(nm), BRAKE_CYCLES)
        for k in (1, 8, 9):
            self.assertEqual(nm[k], 0.15)
        levels = [nm[k] for k in PLACED_BRAKE_CYCLES]
        self.assertEqual(levels, sorted(levels))
        self.assertGreater(levels[0], 0.0)
        self.assertLess(levels[-1], 0.13)
        self.assertEqual(levels, [p._brake_nm[k] for k in (1, 2, 3)])

    def test_brake_levels_follow_the_measured_max_decel(self):
        """最大制動で読んだ減速度が小さい路面では、割り振る段も弱くなる（固定の段だと頭打ちの上に
        落ちる。2026-10-10 の実機の記録は4段中3段が頭打ちだった）。"""
        hi, lo = SysIdAccel(), SysIdAccel()
        hi.decel_max, lo.decel_max = 4.0, 1.8
        for k in (1, 2, 3):
            self.assertLess(lo.stage_brake_nm(k), hi.stage_brake_nm(k))
        # 頭打ち 1.8m/s²・傾き約37(m/s²)/(N·m) の路面で、最初の段は傾きを知らなくても（保守側の
        # 見積もりで）頭打ちの下。上の段は、下の段で読んだ傾きで置くので頭打ちの下
        lo = SysIdAccel()
        lo.decel_max = 1.8
        self.assertLess(37.0 * lo.stage_brake_nm(1) + 0.3, 1.8)
        lo._brake_slopes.append(37.0)
        self.assertLess(37.0 * lo.stage_brake_nm(3) + 0.3, 1.8)

    def test_abs_in_a_placed_level_backs_off_the_next_one(self):
        """割り振った段で ABS が効いたら（頭打ち側だった）、次の段はその強さの8割より上にしない。"""
        p = SysIdAccel()
        p.set_engaged(True)
        nm = self._brake_by_cycle(_run(p, P, abs_in_cycle=1))      # 段1のブレーキのサイクル（0始まり）
        self.assertLessEqual(nm[4], 0.8 * nm[2] + 1e-9)
        self.assertLessEqual(nm[6], 0.8 * nm[2] + 1e-9)

    def test_tc_flag_alone_does_not_stop_the_escalation(self):
        """TC の旗だけ（後輪は滑っていない）では上の段へ進む（実機で旗が30〜50msだけ立ったのを
        「介入」と数えて全開の段を飛ばした、2026-09-27）。"""
        p = SysIdAccel()
        p.set_engaged(True)
        trace = _run(p, P, spin_from=(1, 0.3), mode="tc_flag")
        self.assertEqual(trace[-1][0].reason, "完了")
        self.assertTrue(any(st.reason.endswith(f" {N_CYCLES}/{N_CYCLES}") for st, _, _ in trace))

    def test_rear_slip_does_not_stop_the_escalation(self):
        """後輪が滑っても（滑り率0.15＝TC が保つ程度、抑えきれない空転ほどではない）全段を走る。
        TC の目標スリップ率（0.2）のもとでは TC が働くだけで滑り率0.10を超えるので、滑りで打ち切ると
        全開の段と強いブレーキの段を必ず飛ばす（2026-10-10 の実機の記録）。"""
        p = SysIdAccel()
        p.set_engaged(True)
        trace = _run(p, P, spin_from=(1, 0.3), mode="slip")
        self.assertEqual(trace[-1][0].reason, "完了")
        self.assertIsNone(p.spin_stage)
        self.assertTrue(any(st.reason.endswith(f" {N_CYCLES}/{N_CYCLES}") for st, _, _ in trace))

    def test_wheelspin_in_a_placed_stage_ends_the_test(self):
        """割り振った段の加速中に抑えきれない空転（後輪が前輪より大きく速い）が続いたら、その段で
        制動し、戻ってから終える。"""
        p = SysIdAccel()
        p.set_engaged(True)
        trace = _run(p, P, spin_from=(1, 0.3))
        self.assertTrue(trace[-1][0].reason.startswith("完了（空転を検出"), trace[-1][0].reason)
        self.assertIn(f"{p.stage_accel(1):.2f}m/s²", trace[-1][0].reason)
        stages_run = {st.accel_limit for st, _, _ in trace if st.reason.startswith("加速")}
        self.assertNotIn(p.stage_accel(2), stages_run)
        # 空転を見た後は制動（速度指令0ではなく）。強さは後輪がロックしない強さ
        i = next(k for k, (st, _, _) in enumerate(trace)
                 if st.reason.startswith("減速") and st.reason.endswith(f" 2/{N_CYCLES}"))
        self.assertTrue(trace[i][0].brake)
        self.assertEqual(trace[i][0].brake_torque, ABORT_BRAKE_TORQUE_NM)

    def test_wheelspin_at_full_throttle_goes_on_to_the_lower_stages(self):
        """最初の全開で抑えきれない空転が出たら、すぐ制動し、最後の全開（2回）は走らず、
        空転するまでに出ていた加速度を最大とみなして下の段を走る（全開が最初なので、ここで
        終えると何も測れない）。"""
        p = SysIdAccel()
        p.set_engaged(True)
        trace = _run(p, P, spin_from=(0, 0.3))
        self.assertTrue(trace[-1][0].reason.startswith("完了（全開の段で空転"), trace[-1][0].reason)
        accel = [st for st, _, _ in trace if st.reason.startswith("加速")]
        for k in (N_CYCLES - 1, N_CYCLES):
            self.assertFalse([st for st in accel if st.reason.endswith(f" {k}/{N_CYCLES}")])
        seen = list(dict.fromkeys(st.accel_limit for st in accel))
        self.assertEqual(seen, [0.0] + [p.stage_accel(k) for k in (1, 2, 3)])
        i = next(k for k, (st, _, _) in enumerate(trace)
                 if st.reason.startswith("減速") and st.reason.endswith(f" 1/{N_CYCLES}"))
        self.assertEqual(trace[i][0].brake_torque, ABORT_BRAKE_TORQUE_NM)

    def test_return_phase_is_excluded_from_fit_filters(self):
        """戻りフェーズは`target_speed<0`・`brake=False`で、解析の加速・減速の区間選別
        （`target_speed>0`／`brake`／`target_speed==0`）のどれにも該当しない。"""
        p = SysIdAccel()
        p.set_engaged(True)
        ret = [st for st, _, _ in _run(p, P) if st.reason.startswith("戻り")]
        self.assertTrue(ret)
        for st in ret:
            self.assertLess(st.target_speed, 0.0)
            self.assertFalse(st.brake)

    def test_short_runs_return_slowly(self):
        """前進が短い回は遅く戻って時間を取る（速度 PI が落ち着く前に次の加速を始めない。解析は
        加速の頭で制御器が定常だと仮定する）。長い回は `return_speed` のまま。"""
        params = dict(P, v_max=2.0, return_speed=0.9)
        p = SysIdAccel()
        p.set_engaged(True)
        trace = _run(p, params)
        seen = set()
        for (prev, _, _), (st, _, odom) in zip(trace, trace[1:]):
            if st.reason.startswith("戻り") and not prev.reason.startswith("戻り"):
                want = max(0.2, min(params["return_speed"], odom / SysIdAccel._RETURN_MIN_S))
                self.assertAlmostEqual(-st.target_speed, want, places=6)
                seen.add(round(want, 3))
        self.assertIn(params["return_speed"], seen)                      # 全開の回は長い
        self.assertTrue(any(x < params["return_speed"] for x in seen))   # 弱い加速の回は短い

    def test_return_phase_has_safety_timeout_when_odom_never_returns(self):
        """オドメトリが更新されない異常時でも、戻りフェーズに留まり続けない。"""
        p = SysIdAccel()
        p.set_engaged(True)
        _skip_settle(p)
        odom = 0.0
        st = None
        for _ in range(200):          # 加速（odomだけ進める）
            odom += 0.1
            st = p.plan(scan=None, vs=_vs(odom, speed=0.5), p=P, dt=DT)
            if not st.reason.startswith("加速"):
                break
        for _ in range(200):          # 停止まで
            st = p.plan(scan=None, vs=_vs(odom, speed=0.0), p=P, dt=DT)
            if st.reason.startswith("戻り"):
                break
        self.assertTrue(st.reason.startswith("戻り"))
        reached = False
        for _ in range(2000):
            st = p.plan(scan=None, vs=_vs(odom, speed=0.0), p=P, dt=DT)
            if st.reason.startswith("加速") and st.reason.endswith(f" 2/{N_CYCLES}"):
                reached = True
                break
        self.assertTrue(reached, "戻りが完了しなくても安全装置で次のサイクルへ進むはず")

    def test_completes_after_all_cycles(self):
        p = SysIdAccel()
        p.set_engaged(True)
        trace = _run(p, P)
        st = trace[-1][0]
        self.assertEqual(st.reason, "完了")
        self.assertTrue(any(s_.reason.endswith(f" {N_CYCLES}/{N_CYCLES}") for s_, _, _ in trace))
        self.assertEqual(st.target_speed, 0.0)


if __name__ == "__main__":
    unittest.main()
