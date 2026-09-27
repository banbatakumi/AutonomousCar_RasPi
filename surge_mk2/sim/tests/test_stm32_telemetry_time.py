"""`sim/stm32.py`（仮想 STM32）の TELEMETRY の時刻と中身の一致（2026-09-27）。

ファームは 100Hz でスナップショットを取り、その時刻を `t_us` に載せる。シムは以前 `advance_to`
で呼ばれた時刻まで積分してから、過去のスナップショットの時刻で送っていたので、中身が `t_us`
より最大 `read_timeout`（5ms）新しかった（インタラクティブシムの遅延試験が小さく出る）。
"""

import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from sim.course import Course  # noqa: E402
from sim.params import SimParams  # noqa: E402
from sim.stm32 import NS, STEP_S, TELEMETRY_HZ, VirtualStm32  # noqa: E402
from sim.vehicle import VehicleSpec  # noqa: E402

COURSE = Path(__file__).resolve().parents[1] / "courses" / "circuit_chicane_a.png"


class TestTelemetrySnapshotTime(unittest.TestCase):
    def test_content_is_taken_at_the_snapshot_time(self):
        sim = VirtualStm32(VehicleSpec.load(), Course.load(COURSE), SimParams())
        t0 = 1_000_000_000
        sim.start(t0)
        state_t = [t0]
        orig = sim._substep

        def substep(dt):
            orig(dt)
            state_t[0] += int(round(dt * NS))
        sim._substep = substep
        gaps = []
        orig_tlm = sim._telemetry

        def telemetry(t_ns):
            gaps.append(state_t[0] - t_ns)          # 中身の時刻 − スナップショットの時刻
            return orig_tlm(t_ns)
        sim._telemetry = telemetry
        t = t0
        for _ in range(200):                          # 周期と揃わない間隔で呼ぶ（`SimLink.poll`）
            t += 7_300_000
            sim.advance_to(t)
        self.assertGreater(len(gaps), 100)
        self.assertGreaterEqual(min(gaps), 0)
        self.assertLess(max(gaps), STEP_S * NS)

    def test_next_due_includes_the_next_snapshot(self):
        """線上に何も無くても、次のスナップショットの時刻に起こす（寝過ごさない）。"""
        sim = VirtualStm32(VehicleSpec.load(), Course.load(COURSE), SimParams())
        sim.start(1_000_000_000)
        sim.advance_to(1_000_000_000 + NS // TELEMETRY_HZ // 2)
        sim.due_bytes(10**12)                          # 線上のものを全部読んだ
        due = sim.next_due_ns()
        self.assertIsNotNone(due)
        self.assertLessEqual(due, 1_000_000_000 + NS // TELEMETRY_HZ + 1_000_000)
