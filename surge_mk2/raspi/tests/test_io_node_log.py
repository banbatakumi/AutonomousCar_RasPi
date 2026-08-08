"""io_node のログ記録の結合テスト（実機・シリアル不要）。

SerialLink を差し替えた偽リンクで io_node を回し、
「送受信したフレームが漏れなく `.sfl` に落ちる」ことを確認する。
フレーミングは本物（FrameEncoder/FrameParser）を通す。

    python3 -m unittest discover -s raspi/tests -t .
"""

from __future__ import annotations

import sys
import tempfile
import time
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from raspi.nodes.io_node import IoNode  # noqa: E402
from raspi.proto import FrameEncoder, FrameParser, packets  # noqa: E402
from raspi.proto.generated.packets import HEADER_SIZE, S2P_TYPES  # noqa: E402
from raspi.rec import FrameLogReader, FrameLogWriter, Kind  # noqa: E402


class FakeLink:
    """SerialLink と同じ形の偽リンク。STM32 役のバイト列を差し込める。"""

    def __init__(self) -> None:
        self._parser = FrameParser(expect_types=S2P_TYPES)
        self._enc = FrameEncoder()
        self._rx_enc = FrameEncoder()      # STM32 側の SEQ
        self._inbox = bytearray()
        self.sent: list[bytes] = []
        self.on_tx = None
        self.port, self.baud = "fake", 250_000

    # ── テストから STM32 役として送り込む ──
    def feed(self, packet) -> None:
        self._inbox.extend(self._rx_enc.encode(packet))

    # ── SerialLink の API ──
    def poll(self):
        from raspi.io.serial_link import RxFrame
        if not self._inbox:
            return []
        data = bytes(self._inbox)
        self._inbox.clear()
        rx_ns = time.monotonic_ns()
        return [RxFrame(f.type, f.seq, f.payload, rx_ns)
                for f in self._parser.feed(data)]

    @property
    def stats(self):
        return self._parser.stats

    def send(self, packet) -> int:
        return self._write(self._enc.encode(packet))

    def _write(self, frame: bytes) -> int:
        t_ns = time.monotonic_ns()
        self.sent.append(frame)
        if self.on_tx is not None:
            self.on_tx(t_ns, frame[2], frame[3], frame[HEADER_SIZE:HEADER_SIZE + frame[4]])
        return t_ns

    def reset_input(self) -> None:
        self._inbox.clear()
        self._parser.reset()

    def close(self) -> None:
        pass


class TestIoNodeLogging(unittest.TestCase):
    def setUp(self):
        self._td = tempfile.TemporaryDirectory()
        self.path = Path(self._td.name) / "run.sfl"

    def tearDown(self):
        self._td.cleanup()

    def _run(self, duration_s=0.25, feed=None):
        link = FakeLink()
        log = FrameLogWriter(self.path, meta={"node": "io_node", "port": "fake"})
        node = IoNode(link, log=log)
        if feed:
            for pkt in feed:
                link.feed(pkt)
        node.run(duration_s=duration_s)
        log.close()
        return link, node

    def test_tx_frames_are_logged(self):
        link, _ = self._run()
        with FrameLogReader(self.path) as r:
            tx = list(r.frames(Kind.TX))
        # 偽リンクが送った本数と、ログに落ちた本数が一致すること
        self.assertEqual(len(tx), len(link.sent))
        self.assertGreater(len(tx), 0)
        types = {rec.type for rec in tx}
        self.assertIn(packets.Command.TYPE, types)
        self.assertIn(packets.Ping.TYPE, types)

    def test_tx_payload_matches_sent_frame(self):
        link, _ = self._run()
        with FrameLogReader(self.path) as r:
            tx = list(r.frames(Kind.TX))
        for rec, frame in zip(tx, link.sent):
            self.assertEqual(rec.type, frame[2])
            self.assertEqual(rec.seq, frame[3])
            self.assertEqual(rec.payload, frame[HEADER_SIZE:HEADER_SIZE + frame[4]])

    def test_command_is_always_disarm(self):
        """安全側の不変条件。ログから arm が立っていないことを検証できる。"""
        self._run()
        with FrameLogReader(self.path) as r:
            cmds = [rec.decode() for rec in r.frames(Kind.TX)
                    if rec.type == packets.Command.TYPE]
        self.assertGreater(len(cmds), 0)
        for c in cmds:
            self.assertEqual(c.mode, packets.Mode.DISARM)
            self.assertEqual(c.flags & packets.CMD_FLG_ARM, 0)
            self.assertEqual((c.target_speed, c.target_steer), (0, 0))

    def test_rx_frames_are_logged_and_decodable(self):
        t = packets.Telemetry(t_us=555, speed=1234, flags=0x04)
        self._run(feed=[t, packets.Stats(t_us=556)])
        with FrameLogReader(self.path) as r:
            rx = list(r.frames(Kind.RX))
        self.assertEqual([rec.type for rec in rx],
                         [packets.Telemetry.TYPE, packets.Stats.TYPE])
        got = rx[0].decode()
        self.assertEqual((got.t_us, got.speed, got.flags), (555, 1234, 0x04))

    def test_health_transition_is_logged(self):
        self._run(feed=[packets.Telemetry(t_us=1)])
        with FrameLogReader(self.path) as r:
            events = [rec.json() for rec in r if rec.kind == Kind.EVENT]
        health = [e for e in events if e["name"] == "health"]
        self.assertTrue(health, f"health イベントが無い: {events}")
        self.assertEqual((health[0]["from"], health[0]["to"]), ("INIT", "OK"))

    def test_log_write_failure_does_not_stop_the_loop(self):
        """記録が壊れても走行は続くこと。ログはあくまで従属物。"""
        link = FakeLink()
        log = FrameLogWriter(self.path)

        def boom(*a, **k):
            raise OSError("disk full")

        node = IoNode(link, log=log)
        log.write_tx = boom
        node.run(duration_s=0.1)
        log.write_tx = FrameLogWriter.write_tx.__get__(log)
        log.close()

        self.assertGreater(node._log_errors, 0)
        self.assertGreater(len(link.sent), 0)   # 送信は止まっていない

    def test_no_log_means_no_hook(self):
        link = FakeLink()
        node = IoNode(link)
        node.run(duration_s=0.05)
        self.assertIsNone(link.on_tx)
        self.assertGreater(len(link.sent), 0)


if __name__ == "__main__":
    unittest.main()
