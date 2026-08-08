"""replay_node の単体テスト（実機・シリアル不要）。

合成した `.sfl` を再生し、**実機と同じ結果になること**を確かめる。

    python3 -m unittest discover -s raspi/tests -t .
"""

from __future__ import annotations

import contextlib
import io
import sys
import tempfile
import time
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from raspi.nodes.replay_node import ReplayNode, verify  # noqa: E402
from raspi.proto import packets  # noqa: E402
from raspi.rec import FrameLogWriter  # noqa: E402

MS = 1_000_000


class LogBuilder:
    """テスト用の `.sfl` を組み立てる。時刻はすべて t0 からの ms 指定。"""

    def __init__(self, path: Path, **meta):
        self.w = FrameLogWriter(path, meta=meta or None, flush_interval_s=0.0)
        self.t0 = self.w.t0_mono_ns
        self.seq = 0
        self.n_rx = 0

    def _next(self) -> int:
        self.seq = (self.seq + 1) & 0xFF
        return self.seq

    def rx(self, ms: float, pkt) -> "LogBuilder":
        self.n_rx += 1
        self.w.write_rx(self.t0 + int(ms * MS), pkt.TYPE, self._next(), pkt.encode())
        return self

    def tx(self, ms: float, pkt) -> "LogBuilder":
        self.w.write_tx(self.t0 + int(ms * MS), pkt.TYPE, self._next(), pkt.encode())
        return self

    def event(self, ms: float, name: str, data: dict) -> "LogBuilder":
        self.w.write_event(self.t0 + int(ms * MS), name, data)
        return self

    def linkstats(self, ms: float, **rx) -> "LogBuilder":
        base = {"frame_ok": self.n_rx, "crc_error": 0, "len_error": 0,
                "packet_loss": 0, "reordered": 0, "resync_bytes": 0}
        base.update(rx)
        return self.event(ms, "linkstats", {"health": "OK", "rx": base, "sync": None})

    def close(self):
        self.w.close()


class ReplayCase(unittest.TestCase):
    def setUp(self):
        self._td = tempfile.TemporaryDirectory()
        self.path = Path(self._td.name) / "t.sfl"

    def tearDown(self):
        self._td.cleanup()

    def build(self, **meta) -> LogBuilder:
        return LogBuilder(self.path, **meta)

    def telemetry_stream(self, n=10, period_ms=20, start_ms=0.0) -> LogBuilder:
        b = self.build(node="test")
        for i in range(n):
            b.rx(start_ms + i * period_ms,
                 packets.Telemetry(t_us=int((start_ms + i * period_ms) * 1000),
                                   speed=i * 10))
        return b


class TestBasicReplay(ReplayCase):
    def test_state_matches_the_recording(self):
        self.telemetry_stream(n=10).close()
        node = ReplayNode(self.path, speed=0)
        node.run()
        self.assertEqual(node.rx_records, 10)
        self.assertEqual(node.state.counts[packets.Telemetry.TYPE], 10)
        self.assertEqual(node.state.telemetry.speed, 90)
        self.assertEqual(node.state.health, "OK")

    def test_meta_is_exposed(self):
        self.build(node="io_node", port="/dev/serial0").close()
        node = ReplayNode(self.path, speed=0)
        node.run()
        self.assertEqual(node.meta["port"], "/dev/serial0")

    def test_deterministic(self):
        """何度流しても同じ結果になること（バグ再現が確定的であるための前提）。"""
        b = self.telemetry_stream(n=20)
        b.tx(5, packets.Ping(ping_id=1))
        b.rx(6, packets.Pong(ping_id=1, t_ping_rx_us=5500, t_pong_tx_us=5800))
        b.close()

        runs = []
        for _ in range(3):
            n = ReplayNode(self.path, speed=0)
            n.run()
            runs.append((n.rx_records, dict(n.state.counts), n.sync.offset_ns,
                         n.sync.n_samples, n.state.health,
                         [(f, t) for _, f, t in n.health_changes]))
        self.assertEqual(runs[0], runs[1])
        self.assertEqual(runs[1], runs[2])

    def test_callbacks(self):
        tel, tx, ev = [], [], []
        b = self.telemetry_stream(n=3)
        b.tx(1, packets.Command(mode=packets.Mode.DISARM))
        b.event(2, "handshake", {"match": True})
        b.close()

        node = ReplayNode(self.path, speed=0,
                          on_telemetry=lambda m, t: tel.append(m.speed),
                          on_tx=lambda t, ty, s, m: tx.append(ty),
                          on_event=lambda t, d: ev.append(d["name"]))
        node.run()
        self.assertEqual(tel, [0, 10, 20])
        self.assertEqual(tx, [packets.Command.TYPE])
        self.assertIn("handshake", ev)

    def test_tx_is_not_transmitted_only_reported(self):
        """再生は何も送らない。記録された指令を見せるだけ。"""
        b = self.telemetry_stream(n=2)
        b.tx(1, packets.Command(mode=packets.Mode.DISARM))
        b.tx(2, packets.Ping(ping_id=1))
        b.close()
        node = ReplayNode(self.path, speed=0)
        node.run()
        self.assertEqual(node.tx_counts,
                         {packets.Command.TYPE: 1, packets.Ping.TYPE: 1})
        self.assertFalse(hasattr(node, "link"))


class TestTimeSyncReconstruction(ReplayCase):
    def test_ping_t1_recovered_from_tx_records(self):
        """T1 は送信レコードにしか無い。そこから時刻同期が復元できること。"""
        b = self.build()
        for i in range(6):
            t = i * 100
            b.rx(t, packets.Telemetry(t_us=t * 1000))
            b.tx(t + 1, packets.Ping(ping_id=i))
            b.rx(t + 3, packets.Pong(ping_id=i,
                                     t_ping_rx_us=(t + 1) * 1000 + 500,
                                     t_pong_tx_us=(t + 1) * 1000 + 800))
        b.close()

        node = ReplayNode(self.path, speed=0)
        node.run()
        self.assertEqual(node.sync.n_samples, 6)
        self.assertIsNotNone(node.sync.offset_ns)

    def test_no_tx_means_no_sync(self):
        """PONG だけでは同期できない（T1 が無い）。黙って壊れないこと。"""
        b = self.build()
        b.rx(0, packets.Telemetry(t_us=0))
        b.rx(1, packets.Pong(ping_id=1, t_ping_rx_us=500, t_pong_tx_us=800))
        b.close()
        node = ReplayNode(self.path, speed=0)
        node.run()
        self.assertEqual(node.sync.n_samples, 0)


class TestHealthDuringGaps(ReplayCase):
    def test_fault_is_reproduced_in_a_gap_with_no_records(self):
        """レコードが1つも無い区間でも FAULT が見えること。

        リンクが完全に切れた区間は記録が空になる。次のフレームまで待つ実装だと
        DEGRADED/FAULT を丸ごと見落とす。
        """
        b = self.build()
        for i in range(5):
            b.rx(i * 20, packets.Telemetry(t_us=i * 20000))
        # 80ms .. 580ms は無音（500ms の断）
        for i in range(5):
            b.rx(580 + i * 20, packets.Telemetry(t_us=(580 + i * 20) * 1000))
        b.close()

        node = ReplayNode(self.path, speed=0)
        node.run()
        seq = [(f, t) for _, f, t in node.health_changes]
        self.assertEqual(seq, [("INIT", "OK"), ("OK", "DEGRADED"),
                               ("DEGRADED", "FAULT"), ("FAULT", "OK")])

    def test_transition_times_are_accurate(self):
        b = self.build()
        b.rx(0, packets.Telemetry(t_us=0))
        b.rx(500, packets.Telemetry(t_us=500000))
        b.close()
        node = ReplayNode(self.path, speed=0)
        node.run()
        at = {t: rel for rel, _f, t in node.health_changes}
        # 100ms 超で DEGRADED、200ms 超で FAULT。カーソルの刻みは 10ms
        self.assertAlmostEqual(at["DEGRADED"], 0.10, delta=0.011)
        self.assertAlmostEqual(at["FAULT"], 0.20, delta=0.011)


class TestWindowing(ReplayCase):
    def test_end_truncates(self):
        self.telemetry_stream(n=50, period_ms=20).close()   # 0..980ms
        node = ReplayNode(self.path, speed=0, end_s=0.2)
        node.run()
        self.assertEqual(node.rx_records, 11)               # 0,20,..,200ms

    def test_start_fast_forwards_but_keeps_state_warm(self):
        """--start でも状態は温めて渡す。冷えた状態から始めても役に立たない。"""
        self.telemetry_stream(n=50, period_ms=20).close()
        node = ReplayNode(self.path, speed=0, start_s=0.5)
        node.run()
        self.assertEqual(node.rx_records, 50)               # 早送り分も取り込む
        self.assertEqual(node.state.health, "OK")

    def test_window_combination(self):
        self.telemetry_stream(n=50, period_ms=20).close()
        node = ReplayNode(self.path, speed=0, start_s=0.2, end_s=0.4)
        node.run()
        self.assertEqual(node.rx_records, 21)               # 0..400ms


class TestPacing(ReplayCase):
    def test_speed_zero_is_immediate(self):
        self.telemetry_stream(n=100, period_ms=20).close()  # 約2秒ぶん
        t = time.monotonic()
        ReplayNode(self.path, speed=0).run()
        self.assertLess(time.monotonic() - t, 0.5)

    def test_real_time_pacing_is_roughly_right(self):
        self.telemetry_stream(n=21, period_ms=20).close()   # 400ms ぶん
        t = time.monotonic()
        ReplayNode(self.path, speed=2.0).run()              # 2倍速 → 約200ms
        elapsed = time.monotonic() - t
        self.assertGreater(elapsed, 0.12)
        self.assertLess(elapsed, 0.45)

    def test_stop_interrupts(self):
        self.telemetry_stream(n=200, period_ms=20).close()  # 4秒ぶん
        node = ReplayNode(self.path, speed=1.0)

        def cb(n):
            if n.rel_s(n._cursor_ns) > 0.1:
                n.stop()

        t = time.monotonic()
        node.run(status_cb=cb)
        self.assertLess(time.monotonic() - t, 1.5)


class TestRecordedStats(ReplayCase):
    def test_crc_errors_restored_from_linkstats(self):
        """捨てたフレームは RX に無い。イベントから復元できること。"""
        b = self.telemetry_stream(n=5)
        b.linkstats(100, crc_error=3, resync_bytes=64, packet_loss=2)
        b.close()
        node = ReplayNode(self.path, speed=0)
        node.run()
        self.assertEqual(node.recorded_stats.crc_error, 3)
        self.assertEqual(node.recorded_stats.resync_bytes, 64)
        self.assertEqual(node.recorded_stats.packet_loss, 2)

    def test_last_linkstats_wins(self):
        b = self.telemetry_stream(n=5)
        b.linkstats(50, crc_error=1)
        b.linkstats(100, crc_error=9)
        b.close()
        node = ReplayNode(self.path, speed=0)
        node.run()
        self.assertEqual(node.recorded_stats.crc_error, 9)


class TestVerify(ReplayCase):
    def _verify(self, node) -> bool:
        """verify() は人向けに print するので、テストでは黙らせる。"""
        with contextlib.redirect_stdout(io.StringIO()):
            return verify(node)

    def test_consistent_log_passes(self):
        b = self.telemetry_stream(n=8)
        b.linkstats(200)                 # frame_ok = 実際の受信数
        b.close()
        node = ReplayNode(self.path, speed=0)
        node.run()
        self.assertTrue(self._verify(node))

    def test_missing_frames_are_detected(self):
        """io_node が解析したのに記録し損ねたフレームがあれば落ちること。"""
        b = self.telemetry_stream(n=8)
        b.linkstats(200, frame_ok=12)    # 記録より4本多く解析していた
        b.close()
        node = ReplayNode(self.path, speed=0)
        node.run()
        self.assertFalse(self._verify(node))

    def test_health_mismatch_is_detected(self):
        b = self.telemetry_stream(n=8)
        b.linkstats(200)
        b.event(210, "health", {"from": "OK", "to": "FAULT"})   # 記録にしかない遷移
        b.close()
        node = ReplayNode(self.path, speed=0)
        node.run()
        self.assertFalse(self._verify(node))


class TestRobustness(ReplayCase):
    def test_truncated_log_replays(self):
        b = self.telemetry_stream(n=20)
        cut = b.w.stats.bytes_written - 3
        b.close()
        self.path.write_bytes(self.path.read_bytes()[:cut])

        node = ReplayNode(self.path, speed=0)
        node.run()
        self.assertTrue(node.truncated)
        self.assertEqual(node.rx_records, 19)

    def test_empty_log(self):
        self.build().close()
        node = ReplayNode(self.path, speed=0)
        node.run()
        self.assertEqual(node.rx_records, 0)
        self.assertEqual(node.state.health, "INIT")

    def test_unknown_packet_type_does_not_stop_replay(self):
        b = self.telemetry_stream(n=3)
        b.w.write_rx(b.t0 + 50 * MS, 0x7F, 0, b"\xde\xad")
        b.rx(60, packets.Telemetry(t_us=60000))
        b.close()
        node = ReplayNode(self.path, speed=0)
        node.run()
        self.assertEqual(node.rx_records, 5)
        self.assertEqual(node.state.counts[0x7F], 1)


if __name__ == "__main__":
    unittest.main()
