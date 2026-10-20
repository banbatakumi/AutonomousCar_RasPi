"""校正の試験の土台（`raspi/auto/_calib_common.py`）と台上の試験の、止まらない・固まるときの振る舞い。"""

from __future__ import annotations

import unittest

from raspi.auto.calib_encoder import CalibEncoder
from raspi.auto.calib_wheel import CalibWheel
from raspi.msgs.types import VehicleState

DT = 0.1


def vstate(t_s: float, speed: float = 0.0, rear: float | None = None) -> VehicleState:
    rear = speed if rear is None else rear
    return VehicleState(t_capture=int(t_s * 1e9), t_pub=int(t_s * 1e9), speed=speed,
                        wheel_speed=[speed, speed, rear, rear], odom_dist=[0.0, 0.0], md_status=[0, 0, 0], armed=True)


def run(planner, seconds: float, speed: float = 0.0, rear: float | None = None, t0: float = 0.0):
    st = None
    for i in range(int(seconds / DT)):
        vs = vstate(t0 + i * DT, speed, rear)
        planner.on_vehicle_state(vs)
        st = planner.plan(None, vs, {}, DT)
    return st, t0 + seconds


class TestStopGuard(unittest.TestCase):
    def _planner_in_stop(self):
        pl = CalibEncoder()
        pl.set_engaged(True)
        _, t = run(pl, 1.3)                       # 静止（ジャイロのバイアス）を終える
        pl._stage, pl._stage_t = "stop", 0.0
        return pl, t

    def test_aborts_when_it_does_not_stop(self):
        pl, t = self._planner_in_stop()
        st, _ = run(pl, 3.5, speed=0.5, t0=t)
        self.assertEqual(st.phase, "abort", st.reason)
        self.assertTrue(st.brake)

    def test_slow_creep_counts_as_stopped(self):
        pl, t = self._planner_in_stop()
        st, _ = run(pl, 3.5, speed=0.05, t0=t)
        self.assertTrue(st.phase.startswith("hold") or st.phase.startswith("run"), st.phase)


class TestWheelBrakeTimeout(unittest.TestCase):
    def test_brake_gives_up_when_rim_never_stops(self):
        pl = CalibWheel()
        pl.set_engaged(True)
        _, t = run(pl, 1.3)
        pl._stage, pl._stage_t = "brake", 0.0
        phases = set()
        for i in range(40):                       # 周速が 0.1m/s で止まったまま
            vs = vstate(t + i * DT, 0.0, rear=0.1)
            pl.on_vehicle_state(vs)
            phases.add(pl.plan(None, vs, {}, DT).phase.split("/")[0])
        self.assertIn("rest", phases)


if __name__ == "__main__":
    unittest.main()
