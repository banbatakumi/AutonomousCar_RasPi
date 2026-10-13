"""`SerialLink` の受信（疑似端末で本物の pyserial を通す）。"""

import os
import pty
import sys
import time
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from raspi.io.serial_link import SerialLink  # noqa: E402
from raspi.proto import packets  # noqa: E402
from raspi.proto.framing import build_frame  # noqa: E402


class TestNonBlockingPoll(unittest.TestCase):
    """`poll(block=False)` は来ている分だけを1回の read で取り、待たない（io_node が fd の
    受信を待ってから呼ぶ）。"""

    def setUp(self):
        import tty
        self.master, slave = pty.openpty()
        tty.setraw(slave)
        # 疑似端末は 1Mbps を設定できない（macOS）。ボーレートは読み方に関係しない
        self.link = SerialLink(os.ttyname(slave), baud=115200)
        os.close(slave)

    def tearDown(self):
        self.link.close()
        os.close(self.master)

    def _poll_until(self, n, timeout=1.0):
        got = []
        end = time.monotonic() + timeout
        while len(got) < n and time.monotonic() < end:
            got += self.link.poll(block=False)
            time.sleep(0.001)
        return got

    def test_returns_at_once_when_nothing_has_arrived(self):
        t0 = time.monotonic()
        self.assertEqual(self.link.poll(block=False), [])
        self.assertLess(time.monotonic() - t0, 0.004)     # `read_timeout`（5ms）を待たない

    def test_frames_split_across_reads_are_reassembled(self):
        frame = build_frame(packets.Pong.TYPE, 7, packets.Pong(
            ping_id=3, t_ping_rx_us=100, t_pong_tx_us=200).encode())
        os.write(self.master, frame[:6])
        self.assertEqual(self._poll_until(1, timeout=0.05), [])
        os.write(self.master, frame[6:] + frame[:4])
        got = self._poll_until(1)
        self.assertEqual(len(got), 1)
        self.assertEqual((got[0].type, got[0].seq), (packets.Pong.TYPE, 7))
        self.assertEqual(got[0].decode().ping_id, 3)
        self.assertGreater(got[0].rx_ns, 0)
        self.assertEqual(self.link.stats.crc_error, 0)


if __name__ == "__main__":
    unittest.main()
