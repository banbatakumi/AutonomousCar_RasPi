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
from raspi.auto.sysid_accel import STAGES, SysIdAccel  # noqa: E402
from raspi.msgs.types import VehicleState  # noqa: E402

DT = 0.1      # planning_node は新しいスキャン（10Hz）ごとに plan() を呼ぶ
P = {"v_max": 1.0, "run_length_m": 3.0, "hold_s": 0.3, "return_speed": 0.5}
N_CYCLES = 2 * len(STAGES)


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


def _run(p: SysIdAccel, params: dict, n: int = 3000, spin_from=None, mode: str = "spin") -> list:
    """最後まで回す。`spin_from` が (段, 秒) なら、その段の加速が始まってからその秒数以降、`mode`:
    "spin" 後輪を前輪より大きく速く回す（抑えきれない空転）、"slip" 後輪を15%速く（滑った）、
    "tc_flag" TC の旗だけ立てる（後輪と前輪は同じ）。"""
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
            vs = car.vs(tc_active=spin, t_capture=int((len(out) * 3 + k) * 3.3e7))
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

    def test_stages_ramp_the_target_with_accel_limit_then_full_throttle(self):
        """段の加速度では目標をその速さで上げ、`accel_limit` も同じ値（ファームのランプと揃う）。
        段は加速度の小さい順で、最後は全開（目標を `v_max` より上、`accel_limit` は指定しない）。"""
        p = SysIdAccel()
        p.set_engaged(True)
        trace = _run(p, P)
        accel = [(st, p_) for (st, _, _), p_ in zip(trace, range(len(trace))) if st.reason.startswith("加速")]
        seen = list(dict.fromkeys(st.accel_limit for st, _ in accel))
        self.assertEqual(seen, [a for a, _, _ in STAGES])
        for st, _ in accel:
            if st.accel_limit > 0:
                v_hold = {a: v for a, v, _ in STAGES}[st.accel_limit]
                self.assertLessEqual(st.target_speed, v_hold + 1e-9)
            else:
                self.assertGreater(st.target_speed, P["v_max"])
        # 段の加速度のランプ: 1周ごとに `a`×dt ずつ増える（最初の周から正＝目標0を挟まない）
        first = [st.target_speed for st, _ in accel if st.reason.endswith(f" 1/{N_CYCLES}")]
        a0 = STAGES[0][0]
        self.assertAlmostEqual(first[0], a0 * DT)
        self.assertAlmostEqual(first[1] - first[0], a0 * DT)

    def test_decel_alternates_brake_and_speed_command_zero(self):
        """各段の1回目はブレーキ、2回目は速度指令0で減速する（2種類の減速度を測るため）。"""
        p = SysIdAccel()
        p.set_engaged(True)
        decel = [(st.brake, st.target_speed, st.reason) for st, _, _ in _run(p, P)
                 if st.reason.startswith("減速")]
        for k in range(1, N_CYCLES + 1):
            cyc = [d for d in decel if d[2].endswith(f" {k}/{N_CYCLES}")]
            self.assertTrue(cyc, k)
            if k % 2 == 1:
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

    def test_brake_cycles_use_the_stage_brake_torque(self):
        """各段のブレーキのサイクルは、その段の制動トルクを `AutoState.brake_torque` で指定する。"""
        p = SysIdAccel()
        p.set_engaged(True)
        brakes = [(st.reason, st.brake_torque) for st, _, _ in _run(p, P)
                  if st.reason.startswith("減速") and st.brake]
        for k, (_, _, nm) in enumerate(STAGES):
            cyc = [t for r, t in brakes if r.endswith(f" {2 * k + 1}/{N_CYCLES}")]
            self.assertTrue(cyc, k)
            self.assertTrue(all(abs(t - nm) < 1e-9 for t in cyc), (k, cyc))

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

    def test_wheelspin_ends_the_test_without_going_to_higher_stages(self):
        """加速中に抑えきれない空転（後輪が前輪より大きく速い）が続いたら、その段で制動し、
        戻ってから終える。"""
        p = SysIdAccel()
        p.set_engaged(True)
        trace = _run(p, P, spin_from=(1, 0.3))
        self.assertTrue(trace[-1][0].reason.startswith("完了（空転を検出"), trace[-1][0].reason)
        self.assertIn(f"{STAGES[1][0]:.1f}m/s²", trace[-1][0].reason)
        stages_run = {st.accel_limit for st, _, _ in trace if st.reason.startswith("加速")}
        self.assertNotIn(STAGES[2][0], stages_run)
        self.assertNotIn(0.0, stages_run)
        # 空転を見た後は制動（速度指令0ではなく）。強さは後輪がロックしない強さ
        i = next(k for k, (st, _, _) in enumerate(trace)
                 if st.reason.startswith("減速") and st.reason.endswith(f" 3/{N_CYCLES}"))
        self.assertTrue(trace[i][0].brake)
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
