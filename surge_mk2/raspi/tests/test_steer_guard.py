"""舵が指令に追従しないときに止めて待つ（`raspi/core/steer_guard.py`・`io_node._check_steer`）。

会場の試走 `surge_20261031_062217`: 舵が動かないまま 1.4 秒まっすぐ走って壁に当たった。STM32 も Pi も何も言わなかった。
`surge_20261102_094700`: 走り出しで同じことが起き、当時は DISARM して走行が終わった（後で舵は正常に戻った）。
"""

from __future__ import annotations

import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from raspi.core.steer_guard import (GIVE_UP, HOLD_S, MAX_TRIPS, PAUSE, PAUSE_MAX_S, RESUME,  # noqa: E402
                                    RESUME_S, SteerGuard)
from raspi.nodes.io_node import REARM_STEER_STUCK, IoNode  # noqa: E402
from raspi.msgs import DriveCmd  # noqa: E402
from raspi.proto import packets  # noqa: E402
from raspi.tests.test_io_node_log import FakeLink  # noqa: E402

MS = 1_000_000

STUCK = (True, -0.15, 0.0, 4.8, 1.5)        # 062217: 指令 −0.15 に実舵角 0.00、電流 4.8A、1.5m/s


def _feed(g: SteerGuard, rows, dt_ms: int = 10, t0_ms: int = 0) -> list[tuple[int, str]]:
    """(armed, 指令, 実舵角, 電流, 車速) を 100Hz で渡し、(時刻 [ms], 出来事) の列を返す。"""
    out = []
    for k, (armed, cmd, act, cur, v) in enumerate(rows):
        t = t0_ms + k * dt_ms
        ev = g.update(t * MS, armed, cmd, act, cur, v)
        if ev:
            out.append((t, ev))
    return out


class SteerGuardTest(unittest.TestCase):
    def test_stuck_steer_pauses_after_hold(self) -> None:
        evs = _feed(SteerGuard(), [STUCK] * 100)
        self.assertEqual([e for _, e in evs], [PAUSE])      # 1 回だけ（待っている間は数えない）
        self.assertAlmostEqual(evs[0][0], HOLD_S * 1000, delta=10)

    def test_catching_up_without_saturated_current_is_fine(self) -> None:
        # 急な切り返しで追いつく途中（電流が上限まで行かない）
        self.assertEqual(_feed(SteerGuard(), [(True, 0.4, 0.1, 2.5, 1.5)] * 100), [])

    def test_short_saturation_is_fine(self) -> None:
        # 走行中の切り返しで電流が上限に当たるのは記録で最長 0.04 秒
        rows = ([(True, 0.3, 0.1, 4.8, 1.5)] * 6 + [(True, 0.3, 0.28, 1.0, 1.5)] * 4) * 20
        self.assertEqual(_feed(SteerGuard(), rows), [])

    def test_standstill_steering_is_ignored(self) -> None:
        # 095106 214.6s: 停車中の据え切りで +0.45 に 0.26、4.8A が 0.25s 続いた
        self.assertEqual(_feed(SteerGuard(), [(True, 0.45, 0.26, 4.8, 0.01)] * 100), [])

    def test_resumes_when_steer_follows(self) -> None:
        g = SteerGuard()
        evs = _feed(g, [STUCK] * 50)
        self.assertTrue(g.paused)
        # 止まった（車速 0）後も追従しない間は待つ。追従したら RESUME_S で再開
        evs += _feed(g, [(True, -0.15, 0.0, 4.8, 0.0)] * 100, t0_ms=500)
        self.assertEqual([e for _, e in evs], [PAUSE])
        evs += _feed(g, [(True, -0.15, -0.14, 1.0, 0.0)] * 100, t0_ms=1500)
        self.assertEqual([e for _, e in evs], [PAUSE, RESUME])
        self.assertAlmostEqual(evs[1][0], 1500 + RESUME_S * 1000, delta=10)
        self.assertFalse(g.paused)

    def test_gives_up_when_not_recovering(self) -> None:
        g = SteerGuard()
        evs = _feed(g, [STUCK] * round((HOLD_S + PAUSE_MAX_S) * 100 + 50))
        self.assertEqual([e for _, e in evs], [PAUSE, GIVE_UP])
        self.assertTrue(g.reason)
        self.assertEqual(_feed(g, [STUCK] * 100, t0_ms=10_000), [])     # 諦めた後は reset まで黙る

    def test_gives_up_on_repeated_trips(self) -> None:
        g = SteerGuard()
        evs: list[tuple[int, str]] = []
        t = 0
        for _ in range(MAX_TRIPS):
            evs += _feed(g, [STUCK] * 30, t0_ms=t)
            evs += _feed(g, [(True, -0.15, -0.15, 1.0, 0.0)] * 40, t0_ms=t + 300)
            t += 1000
        self.assertEqual([e for _, e in evs], [PAUSE, RESUME] * (MAX_TRIPS - 1) + [GIVE_UP])

    def test_disarmed_is_ignored_and_reset_rearms(self) -> None:
        g = SteerGuard()
        self.assertEqual(_feed(g, [(False, -0.15, 0.0, 4.8, 1.5)] * 100), [])
        self.assertEqual(len(_feed(g, [STUCK] * 100)), 1)
        self.assertTrue(g.paused)
        g.reset()
        self.assertFalse(g.paused)
        self.assertEqual(g.trips, 0)
        self.assertEqual(len(_feed(g, [STUCK] * 100, t0_ms=2000)), 1)


class IoNodeSteerStuckTest(unittest.TestCase):
    def _telemetry(self, armed: bool, cmd: float, act: float, cur: float,
                   speed: float = 1.5) -> packets.Telemetry:
        m = packets.Telemetry.META
        return packets.Telemetry(
            flags=packets.FLG_ARMED if armed else 0,
            speed=round(speed / m["speed"][0]),
            steer_cmd_echo=round(cmd / m["steer_cmd_echo"][0]),
            steer_actual=round(act / m["steer_actual"][0]),
            motor_current=[0, 0, round(cur / m["motor_current"][0])])

    def _armed_pkt(self) -> packets.Command:
        return packets.Command(mode=packets.Mode.AUTO, flags=packets.CMD_FLG_ARM,
                               target_speed=1000, target_steer=-1500)

    def test_pause_brakes_with_arm_and_frozen_steer_then_resumes(self) -> None:
        node = IoNode(FakeLink())
        node._arm_block_reason = lambda: ""            # ARM を通せる状態（版・LIMITS・[control] は別のテスト）
        node.cmd = DriveCmd(mode=packets.Mode.AUTO, arm=True, target_speed=2.0, target_steer=0.3)
        node.cmd_stale = False
        node._last_armed_pkt = self._armed_pkt()
        for k in range(40):
            node._check_steer(self._telemetry(True, -0.15, 0.0, 4.8), k * 10 * MS)
        self.assertTrue(node.steer_guard.paused)
        self.assertTrue(node.bridge.steer_hold)
        self.assertEqual(node._rearm_reason, "")        # DISARM のラッチはしない
        self.assertIn("舵", node.stm_log[0])
        for _ in range(3):                               # 止まっても DISARM に落とさない
            pkt = node._command_packet()
            self.assertTrue(pkt.flags & packets.CMD_FLG_BRAKE)
            self.assertTrue(pkt.flags & packets.CMD_FLG_ARM)
            self.assertEqual(pkt.target_speed, 0)
            self.assertEqual(pkt.target_steer, -1500)   # 見つけた時の舵の指令で据え置く
        # 舵が据え置いた指令に追従した → 通常の指令へ戻る
        for k in range(40):
            node._check_steer(self._telemetry(True, -0.15, -0.15, 1.0, 0.0), (500 + k * 10) * MS)
        self.assertFalse(node.steer_guard.paused)
        self.assertFalse(node.bridge.steer_hold)
        pkt = node._command_packet()
        self.assertFalse(pkt.flags & packets.CMD_FLG_BRAKE)
        self.assertGreater(pkt.target_speed, 0)

    def test_releasing_arm_during_pause_disarms(self) -> None:
        node = IoNode(FakeLink())
        node._arm_block_reason = lambda: ""
        node.cmd = DriveCmd(mode=packets.Mode.AUTO, arm=False)
        node.cmd_stale = False
        node._last_armed_pkt = self._armed_pkt()
        for k in range(40):
            node._check_steer(self._telemetry(True, -0.15, 0.0, 4.8), k * 10 * MS)
        self.assertTrue(node.steer_guard.paused)
        self.assertFalse(node._command_packet().flags & packets.CMD_FLG_ARM)

    def test_gives_up_latches_disarm_until_arm_is_released(self) -> None:
        node = IoNode(FakeLink())
        n = round((HOLD_S + PAUSE_MAX_S) * 100 + 50)
        for k in range(n):
            node._check_steer(self._telemetry(True, -0.15, 0.0, 4.8), k * 10 * MS)
        self.assertEqual(node._rearm_reason, REARM_STEER_STUCK)
        self.assertFalse(node.bridge.steer_hold)
        self.assertIn("舵", node.stm_log[0])
        # ARM を外せば次の判定は始めから（ラッチを解くのは `_update_rearm_latch`）
        node._check_steer(self._telemetry(False, 0.0, 0.0, 0.0), n * 10 * MS)
        self.assertEqual(node.steer_guard.reason, "")

    def test_brakes_before_disarming_after_giving_up(self) -> None:
        # いきなり DISARM すると駆動電源が切れて惰行する。走っていたなら最大制動から
        node = IoNode(FakeLink())
        node._last_armed_pkt = self._armed_pkt()
        node.steer_guard.max_trips = 1                  # 1 回目で諦める
        for k in range(40):
            node._check_steer(self._telemetry(True, -0.15, 0.0, 4.8), k * 10 * MS)
        self.assertEqual(node._rearm_reason, REARM_STEER_STUCK)
        pkt = node._command_packet()
        self.assertTrue(pkt.flags & packets.CMD_FLG_BRAKE)
        self.assertTrue(pkt.flags & packets.CMD_FLG_ARM)
        self.assertEqual(pkt.target_speed, 0)
        node._last_armed_pkt = None                     # 止まった（`_failsafe_packet` が消す）
        self.assertEqual(node._command_packet().mode, packets.Mode.DISARM)

    def test_normal_steering_does_not_pause(self) -> None:
        node = IoNode(FakeLink())
        for k in range(200):
            node._check_steer(self._telemetry(True, 0.2, 0.19, 1.5), k * 10 * MS)
        self.assertFalse(node.steer_guard.paused)
        self.assertEqual(node._rearm_reason, "")


if __name__ == "__main__":
    unittest.main()
