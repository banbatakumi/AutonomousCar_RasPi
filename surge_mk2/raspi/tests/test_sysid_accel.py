"""`raspi/auto/sysid_accel.py`（システム同定: 加減速試験）のテスト。

2026-09-01追加の「制動後、次のサイクルの前にサイクル開始位置まで後退する
戻りフェーズ」を中心に検証する。実車の力学は使わず、`VehicleState.odom_dist`
を手動で操作してプランナーの状態機械（フェーズ遷移）だけを見る。
"""

import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from raspi.auto.sysid_accel import SysIdAccel  # noqa: E402
from raspi.msgs.types import VehicleState  # noqa: E402

DT = 0.02
P = {"target_speed": 1.0, "accel_hold_s": 0.5, "brake_hold_s": 0.3,
    "return_speed": 0.5, "cycles": 2}


def _vs(odom: float, armed: bool = True) -> VehicleState:
    return VehicleState(odom_dist=[odom, odom], armed=armed)


class TestSysIdAccel(unittest.TestCase):
    def test_not_engaged_or_armed_reports_waiting(self):
        p = SysIdAccel()
        st = p.plan(scan=None, vs=None, p=P, dt=DT)
        self.assertEqual(st.reason, "試験開始を押してください")

        p.set_engaged(True)
        st = p.plan(scan=None, vs=_vs(0.0, armed=False), p=P, dt=DT)
        self.assertEqual(st.reason, "ARM待ち（Enterを押してください）")

    def test_cycle_progresses_accel_brake_return_then_next_cycle(self):
        """1サイクル=加速→制動→戻りの順に進み、戻りが完了したら次サイクルの
        加速に戻ることを確認する。"""
        p = SysIdAccel()
        p.set_engaged(True)
        odom = 0.0

        st = p.plan(scan=None, vs=_vs(odom), p=P, dt=DT)
        self.assertEqual(st.reason, "加速 1/2")
        self.assertEqual(st.target_speed, P["target_speed"])
        self.assertFalse(st.brake)

        # 加速フェーズぶん時間を進める（この間odomは前進する想定）
        t = DT
        while t < P["accel_hold_s"]:
            odom += 0.02
            st = p.plan(scan=None, vs=_vs(odom), p=P, dt=DT)
            t += DT
        odom += 0.02
        st = p.plan(scan=None, vs=_vs(odom), p=P, dt=DT)
        self.assertEqual(st.reason, "制動 1/2")
        self.assertTrue(st.brake)
        self.assertEqual(st.target_speed, 0.0)

        # 制動フェーズぶん時間を進める（ここでodomは動かない=完全停止とみなす）
        t = 0.0
        while t < P["brake_hold_s"]:
            st = p.plan(scan=None, vs=_vs(odom), p=P, dt=DT)
            t += DT

        self.assertEqual(st.reason, "戻り 1/2")
        self.assertLess(st.target_speed, 0.0)
        self.assertFalse(st.brake, "戻りフェーズはbrakeフラグを立てない"
                                   "（fit_accel()の集計対象から自動的に外れるように）")

        # 戻りフェーズ: odomをサイクル開始位置(0.0)まで戻す
        while odom > 0.0:
            odom = max(0.0, odom - 0.02)
            st = p.plan(scan=None, vs=_vs(odom), p=P, dt=DT)

        self.assertEqual(st.reason, "加速 2/2")
        self.assertEqual(st.target_speed, P["target_speed"])

    def test_return_phase_target_speed_never_satisfies_fit_accel_filters(self):
        """`tools/sysid/fit.py`の`fit_accel()`は`not brake and target_speed>0.05`
        （加速側）・`brake`（制動側）でサンプルを拾う。戻りフェーズの出力が
        どちらのフィルタにも該当しないこと（＝mcapの自動測定に混入しないこと）
        を直接確認する。"""
        p = SysIdAccel()
        p.set_engaged(True)
        odom = 0.0
        # 十分に前進させてから制動フェーズへ移らせる
        for _ in range(200):
            odom += 0.02
            st = p.plan(scan=None, vs=_vs(odom), p=P, dt=DT)
            if st.reason.startswith("制動"):
                break
        for _ in range(200):
            st = p.plan(scan=None, vs=_vs(odom), p=P, dt=DT)
            if st.reason.startswith("戻り"):
                break
        self.assertTrue(st.reason.startswith("戻り"))

        is_accel_sample = (not st.brake) and st.target_speed > 0.05
        is_brake_sample = st.brake
        self.assertFalse(is_accel_sample)
        self.assertFalse(is_brake_sample)

    def test_return_phase_has_safety_timeout_when_odom_never_returns(self):
        """オドメトリが更新されない（センサ不調等）異常時でも、戻りフェーズに
        無限に留まらず、いずれ次のサイクルへ進む安全装置があること。"""
        p = SysIdAccel()
        p.set_engaged(True)
        odom = 0.0
        for _ in range(200):
            odom += 0.02
            st = p.plan(scan=None, vs=_vs(odom), p=P, dt=DT)
            if st.reason.startswith("制動"):
                break
        for _ in range(200):
            st = p.plan(scan=None, vs=_vs(odom), p=P, dt=DT)
            if st.reason.startswith("戻り"):
                break
        self.assertTrue(st.reason.startswith("戻り"))

        # odomを一切動かさずに時間だけ進める(オドメトリが更新されない異常を模す)
        reached_next_cycle = False
        for _ in range(2000):
            st = p.plan(scan=None, vs=_vs(odom), p=P, dt=DT)
            if st.reason.startswith("加速 2"):
                reached_next_cycle = True
                break
        self.assertTrue(reached_next_cycle,
                        "戻りが完了しなくても安全装置で次のサイクルへ進むはず")

    def test_completes_after_all_cycles(self):
        p = SysIdAccel()
        p.set_engaged(True)
        odom = 0.0
        completed = False
        for _ in range(5000):
            st = p.plan(scan=None, vs=_vs(odom), p=P, dt=DT)
            if st.reason.startswith("加速") or st.reason.startswith("戻り"):
                odom += 0.02 if st.target_speed > 0 else -min(odom, 0.02)
            if st.reason == "完了":
                completed = True
                break
        self.assertTrue(completed)
        self.assertEqual(st.target_speed, 0.0)
        self.assertFalse(st.brake)


if __name__ == "__main__":
    unittest.main()
