"""生フレームログ `.sfl` の単体テスト（ハードウェア不要）。

    python3 -m unittest discover -s raspi/tests -t .
"""

from __future__ import annotations

import json
import struct
import sys
import tempfile
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from raspi.proto import packets  # noqa: E402
from raspi.rec.framelog import (  # noqa: E402
    HEADER_SIZE,
    MAGIC,
    REC_HEADER_SIZE,
    FrameLogReader,
    FrameLogWriter,
    Kind,
    default_log_path,
)


class TempDirCase(unittest.TestCase):
    def setUp(self):
        self._td = tempfile.TemporaryDirectory()
        self.tmp = Path(self._td.name)

    def tearDown(self):
        self._td.cleanup()


class TestRoundTrip(TempDirCase):
    def test_frames_round_trip(self):
        p = self.tmp / "a.sfl"
        with FrameLogWriter(p) as w:
            w.write_rx(1_000, packets.Telemetry.TYPE, 7, b"\x01\x02")
            w.write_tx(2_000, packets.Ping.TYPE, 8, b"\x03")
            w.write_event(3_000, "hello", {"x": 1})

        with FrameLogReader(p) as r:
            recs = list(r)
            self.assertFalse(r.truncated)

        # META は Reader が吸い上げるのでレコード列には出てこない
        self.assertEqual([rec.kind for rec in recs[:3]],
                         [Kind.RX, Kind.TX, Kind.EVENT])
        self.assertEqual(recs[0][1:], (1_000, packets.Telemetry.TYPE, 7, b"\x01\x02"))
        self.assertEqual(recs[1][1:], (2_000, packets.Ping.TYPE, 8, b"\x03"))
        self.assertEqual(recs[2].json(), {"name": "hello", "x": 1})

    def test_close_event_is_last(self):
        p = self.tmp / "a.sfl"
        with FrameLogWriter(p) as w:
            w.write_rx(1, packets.Telemetry.TYPE, 0, b"")
        with FrameLogReader(p) as r:
            last = list(r)[-1]
        body = last.json()
        self.assertEqual(body["name"], "close")
        self.assertEqual(body["rx"], 1)

    def test_meta_round_trip(self):
        p = self.tmp / "a.sfl"
        with FrameLogWriter(p, meta={"port": "/dev/serial0", "baud": 250_000}) as w:
            w.write_rx(1, packets.Telemetry.TYPE, 0, b"")
        with FrameLogReader(p) as r:
            self.assertEqual(r.meta["port"], "/dev/serial0")
            self.assertEqual(r.meta["baud"], 250_000)
            # 記録開始時刻は Writer が必ず入れる
            self.assertIn("t0_unix_ns", r.meta)
            self.assertEqual(r.header.t0_mono_ns, r.meta["t0_mono_ns"])

    def test_decode_gives_packet(self):
        p = self.tmp / "a.sfl"
        t = packets.Telemetry(t_us=123456, speed=1500, flags=0x04)
        with FrameLogWriter(p) as w:
            w.write_rx(10, t.TYPE, 3, t.encode())
        with FrameLogReader(p) as r:
            rec = next(iter(r))
        got = rec.decode()
        self.assertIsInstance(got, packets.Telemetry)
        self.assertEqual((got.t_us, got.speed, got.flags), (123456, 1500, 0x04))

    def test_unknown_type_decodes_to_none(self):
        p = self.tmp / "a.sfl"
        with FrameLogWriter(p) as w:
            w.write_rx(10, 0x7F, 0, b"\xff")
        with FrameLogReader(p) as r:
            self.assertIsNone(next(iter(r)).decode())

    def test_rel_s(self):
        p = self.tmp / "a.sfl"
        with FrameLogWriter(p) as w:
            t0 = w.t0_mono_ns
            w.write_rx(t0 + 1_500_000_000, packets.Telemetry.TYPE, 0, b"")
        with FrameLogReader(p) as r:
            rec = next(iter(r))
            self.assertAlmostEqual(r.rel_s(rec.t_ns), 1.5, places=6)

    def test_frames_filter_skips_events(self):
        p = self.tmp / "a.sfl"
        with FrameLogWriter(p) as w:
            w.write_rx(1, packets.Telemetry.TYPE, 0, b"")
            w.write_event(2, "noise")
            w.write_tx(3, packets.Ping.TYPE, 1, b"")
        with FrameLogReader(p) as r:
            self.assertEqual([f.kind for f in r.frames()], [Kind.RX, Kind.TX])
        with FrameLogReader(p) as r:
            self.assertEqual([f.seq for f in r.frames(Kind.TX)], [1])

    def test_order_and_count_preserved(self):
        p = self.tmp / "a.sfl"
        n = 5000
        with FrameLogWriter(p, flush_interval_s=0.0) as w:
            for i in range(n):
                w.write_rx(1000 + i, packets.Telemetry.TYPE, i & 0xFF, bytes([i & 0xFF]))
        with FrameLogReader(p) as r:
            frames = list(r.frames(Kind.RX))
        self.assertEqual(len(frames), n)
        self.assertEqual([f.t_ns for f in frames], list(range(1000, 1000 + n)))


class TestRobustness(TempDirCase):
    def _write_sample(self, p: Path, n: int = 20) -> None:
        with FrameLogWriter(p) as w:
            for i in range(n):
                w.write_rx(1000 + i, packets.Telemetry.TYPE, i, bytes([i]) * 4)

    def test_truncated_tail_is_tolerated(self):
        """記録中に電源が落ちたログでも、切れる前までは読めること。"""
        p = self.tmp / "a.sfl"
        self._write_sample(p, 20)
        data = p.read_bytes()
        # 途中のレコードの真ん中で切る
        cut = HEADER_SIZE + 200
        p.write_bytes(data[:cut])

        with FrameLogReader(p) as r:
            recs = list(r)
        self.assertTrue(r.truncated)
        self.assertGreater(len(recs), 0)
        self.assertTrue(all(rec.kind == Kind.RX for rec in recs))

    def test_truncated_mid_payload(self):
        p = self.tmp / "a.sfl"
        with FrameLogWriter(p) as w:
            w.write_rx(1, packets.Telemetry.TYPE, 0, b"\x01\x02\x03\x04")
            w.write_rx(2, packets.Telemetry.TYPE, 1, b"\x05\x06\x07\x08")
            cut = w.stats.bytes_written - 2      # 2件目の payload の途中
        p.write_bytes(p.read_bytes()[:cut])
        with FrameLogReader(p) as r:
            recs = list(r)
        self.assertTrue(r.truncated)
        self.assertEqual(len(recs), 1)

    def test_empty_after_header(self):
        p = self.tmp / "a.sfl"
        self._write_sample(p, 3)
        p.write_bytes(p.read_bytes()[:HEADER_SIZE])
        with FrameLogReader(p) as r:
            self.assertEqual(list(r), [])
            self.assertFalse(r.truncated)   # ちょうどヘッダまでなら欠損ではない
            self.assertEqual(r.meta, {})

    def test_bad_magic_rejected(self):
        p = self.tmp / "bad.sfl"
        p.write_bytes(b"NOTASFL!" + bytes(HEADER_SIZE - 8))
        with self.assertRaises(ValueError):
            FrameLogReader(p)

    def test_short_file_rejected(self):
        p = self.tmp / "short.sfl"
        p.write_bytes(MAGIC)
        with self.assertRaises(ValueError):
            FrameLogReader(p)

    def test_future_version_rejected(self):
        p = self.tmp / "fut.sfl"
        p.write_bytes(struct.pack("<8sHHQQ4x", MAGIC, 99, HEADER_SIZE, 0, 0))
        with self.assertRaisesRegex(ValueError, "未対応"):
            FrameLogReader(p)

    def test_longer_header_is_skipped(self):
        """ヘッダが将来伸びても、宣言長ぶん読み飛ばして本文に入れること。"""
        p = self.tmp / "big.sfl"
        hdr_len = HEADER_SIZE + 8
        body = struct.pack("<BBBBQH", Kind.RX, packets.Telemetry.TYPE, 5, 0, 777, 1) + b"\xab"
        p.write_bytes(struct.pack("<8sHHQQ4x", MAGIC, 1, hdr_len, 0, 0)
                      + bytes(8) + body)
        with FrameLogReader(p) as r:
            rec = next(iter(r))
        self.assertEqual((rec.t_ns, rec.seq, rec.payload), (777, 5, b"\xab"))

    def test_log_without_meta_still_readable(self):
        """先頭が META でないログでも1件目を落とさないこと。"""
        p = self.tmp / "nometa.sfl"
        body = struct.pack("<BBBBQH", Kind.RX, packets.Telemetry.TYPE, 9, 0, 42, 0)
        p.write_bytes(struct.pack("<8sHHQQ4x", MAGIC, 1, HEADER_SIZE, 0, 0) + body)
        with FrameLogReader(p) as r:
            self.assertEqual(r.meta, {})
            recs = list(r)
        self.assertEqual(len(recs), 1)
        self.assertEqual(recs[0].seq, 9)

    def test_write_after_close_raises(self):
        p = self.tmp / "a.sfl"
        w = FrameLogWriter(p)
        w.close()
        with self.assertRaises(ValueError):
            w.write_rx(1, packets.Telemetry.TYPE, 0, b"")

    def test_close_is_idempotent(self):
        p = self.tmp / "a.sfl"
        w = FrameLogWriter(p)
        w.close()
        w.close()
        with FrameLogReader(p) as r:
            names = [rec.json().get("name") for rec in r if rec.kind == Kind.EVENT]
        self.assertEqual(names, ["close"])

    def test_oversized_payload_rejected(self):
        p = self.tmp / "a.sfl"
        with FrameLogWriter(p) as w:
            with self.assertRaises(ValueError):
                w.write_rx(1, packets.Telemetry.TYPE, 0, bytes(0x10000))


class TestWriterDetails(TempDirCase):
    def test_bytes_written_matches_file_size(self):
        p = self.tmp / "a.sfl"
        w = FrameLogWriter(p)
        for i in range(50):
            w.write_rx(i, packets.Telemetry.TYPE, i & 0xFF, bytes(66))
        w.close()
        self.assertEqual(w.stats.bytes_written, p.stat().st_size)

    def test_record_size_is_header_plus_payload(self):
        p = self.tmp / "a.sfl"
        w = FrameLogWriter(p)
        before = w.stats.bytes_written
        w.write_rx(1, packets.Telemetry.TYPE, 0, bytes(66))
        self.assertEqual(w.stats.bytes_written - before, REC_HEADER_SIZE + 66)
        w.close()

    def test_flush_makes_data_readable_while_open(self):
        """記録中のログを別プロセスから覗けること（走行中の確認に使う）。"""
        p = self.tmp / "a.sfl"
        w = FrameLogWriter(p, flush_interval_s=0.0)
        w.write_rx(1, packets.Telemetry.TYPE, 0, b"\x01")
        with FrameLogReader(p) as r:
            recs = list(r)
        self.assertEqual(len(recs), 1)
        w.close()

    def test_event_serializes_unknown_objects(self):
        """JSON にできない値が来てもログ側で落ちないこと。"""
        p = self.tmp / "a.sfl"
        with FrameLogWriter(p) as w:
            w.write_event(1, "odd", {"path": Path("/dev/serial0")})
        with FrameLogReader(p) as r:
            body = next(iter(r)).json()
        self.assertEqual(body["path"], "/dev/serial0")

    def test_default_log_path(self):
        d = self.tmp / "logs"
        p = default_log_path(d)
        self.assertTrue(d.is_dir())
        self.assertEqual(p.suffix, ".sfl")
        self.assertTrue(p.name.startswith("surge_"))

    def test_parent_dir_created(self):
        p = self.tmp / "deep" / "nested" / "a.sfl"
        FrameLogWriter(p).close()
        self.assertTrue(p.exists())


class TestJsonEncoding(TempDirCase):
    def test_japanese_not_escaped(self):
        p = self.tmp / "a.sfl"
        with FrameLogWriter(p, meta={"note": "ベンチ試験"}) as w:
            pass
        raw = p.read_bytes()
        self.assertIn("ベンチ試験".encode("utf-8"), raw)
        with FrameLogReader(p) as r:
            self.assertEqual(r.meta["note"], "ベンチ試験")

    def test_meta_is_valid_json_record(self):
        p = self.tmp / "a.sfl"
        FrameLogWriter(p, meta={"k": "v"}).close()
        raw = p.read_bytes()
        kind, _, _, _, _, n = struct.unpack("<BBBBQH", raw[HEADER_SIZE:HEADER_SIZE + REC_HEADER_SIZE])
        self.assertEqual(kind, Kind.META)
        body = json.loads(raw[HEADER_SIZE + REC_HEADER_SIZE:
                              HEADER_SIZE + REC_HEADER_SIZE + n])
        self.assertEqual(body["k"], "v")


if __name__ == "__main__":
    unittest.main()
