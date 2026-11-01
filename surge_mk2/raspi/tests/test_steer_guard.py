"""舵が指令に追従しないときに止める（`raspi/core/steer_guard.py`・`io_node._check_steer`）。

会場の試走 `surge_20261031_062217`: 舵が動かないまま 1.4 秒まっすぐ走って壁に当たった。STM32 も Pi も何も言わなかった。
"""

from __future__ import annotations

import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from raspi.core.steer_guard import HOLD_S, SteerGuard  # noqa: E402
from raspi.nodes.io_node import REARM_STEER_STUCK, IoNode  # noqa: E402
from raspi.proto import packets  # noqa: E402
from raspi.tests.test_io_node_log import FakeLink  # noqa: E402

MS = 1_000_000


def _feed(g: SteerGuard, rows, dt_ms: int = 10) -> list[int]:
    """(armed, 指令, 実舵角, 電流) を 100Hz で渡し、止めた時刻 [ms] の列を返す。"""
    out = []
    for k, (armed, cmd, act, cur) in enumerate(rows):
        if g.update(k * dt_ms * MS, armed, cmd, act, cur):
            out.append(k * dt_ms)
    return out


class SteerGuardTest(unittest.TestCase):
    def test_stuck_steer_trips_after_hold(self) -> None:
        # 062217: 指令 −0.15 に実舵角 0.00、電流 4.8A が続く
        trips = _feed(SteerGuard(), [(True, -0.15, 0.0, 4.8)] * 100)
        self.assertEqual(len(trips), 1)                     # 1 回だけ
        self.assertAlmostEqual(trips[0], HOLD_S * 1000, delta=10)

    def test_catching_up_without_saturated_current_is_fine(self) -> None:
        # 急な切り返しで追いつく途中（電流が上限まで行かない）
        self.assertEqual(_feed(SteerGuard(), [(True, 0.4, 0.1, 2.5)] * 100), [])

    def test_short_saturation_is_fine(self) -> None:
        # 据え切り・切り返しで電流が上限に当たるのは記録で最長 0.06 秒
        rows = ([(True, 0.3, 0.1, 4.8)] * 6 + [(True, 0.3, 0.28, 1.0)] * 4) * 20
        self.assertEqual(_feed(SteerGuard(), rows), [])

    def test_disarmed_is_ignored_and_reset_rearms(self) -> None:
        g = SteerGuard()
        self.assertEqual(_feed(g, [(False, -0.15, 0.0, 4.8)] * 100), [])
        self.assertEqual(len(_feed(g, [(True, -0.15, 0.0, 4.8)] * 100)), 1)
        self.assertTrue(g.reason)
        g.reset()
        self.assertEqual(g.reason, "")
        self.assertEqual(len(_feed(g, [(True, -0.15, 0.0, 4.8)] * 100)), 1)


class IoNodeSteerStuckTest(unittest.TestCase):
    def _telemetry(self, armed: bool, cmd: float, act: float, cur: float) -> packets.Telemetry:
        m = packets.Telemetry.META
        return packets.Telemetry(
            flags=packets.FLG_ARMED if armed else 0,
            steer_cmd_echo=round(cmd / m["steer_cmd_echo"][0]),
            steer_actual=round(act / m["steer_actual"][0]),
            motor_current=[0, 0, round(cur / m["motor_current"][0])])

    def test_latches_disarm_until_arm_is_released(self) -> None:
        node = IoNode(FakeLink())
        for k in range(40):
            node._check_steer(self._telemetry(True, -0.15, 0.0, 4.8), k * 10 * MS)
        self.assertEqual(node._rearm_reason, REARM_STEER_STUCK)
        self.assertIn("舵", node.stm_log[0])
        # ARM を外せば次の判定は始めから（ラッチを解くのは `_update_rearm_latch`）
        node._check_steer(self._telemetry(False, 0.0, 0.0, 0.0), 500 * MS)
        self.assertEqual(node.steer_guard.reason, "")

    def test_brakes_before_disarming_when_moving(self) -> None:
        # いきなり DISARM すると駆動電源が切れて惰行する。走っていたなら最大制動から
        node = IoNode(FakeLink())
        node._last_armed_pkt = packets.Command(mode=packets.Mode.AUTO, flags=packets.CMD_FLG_ARM,
                                               target_speed=1000, target_steer=-1500)
        for k in range(40):
            node._check_steer(self._telemetry(True, -0.15, 0.0, 4.8), k * 10 * MS)
        pkt = node._command_packet()
        self.assertTrue(pkt.flags & packets.CMD_FLG_BRAKE)
        self.assertTrue(pkt.flags & packets.CMD_FLG_ARM)
        self.assertEqual(pkt.target_speed, 0)
        node._last_armed_pkt = None                     # 止まった（`_failsafe_packet` が消す）
        self.assertEqual(node._command_packet().mode, packets.Mode.DISARM)

    def test_normal_steering_does_not_latch(self) -> None:
        node = IoNode(FakeLink())
        for k in range(200):
            node._check_steer(self._telemetry(True, 0.2, 0.19, 1.5), k * 10 * MS)
        self.assertEqual(node._rearm_reason, "")


if __name__ == "__main__":
    unittest.main()
