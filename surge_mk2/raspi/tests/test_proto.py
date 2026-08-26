"""プロトコル層のテスト。

    python3 -m unittest discover -s raspi/tests -t .

追加依存を避けるため標準の unittest を使っている。
"""

from __future__ import annotations

import struct
import subprocess
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from raspi.proto import (  # noqa: E402
    FrameEncoder,
    FrameParser,
    build_frame,
    crc16_ccitt,
    packets,
)

REPO = Path(__file__).resolve().parents[2]


class TestCrc(unittest.TestCase):
    def test_check_value(self):
        """仕様の検査値。ここが違うと通信が一切成立しない。"""
        self.assertEqual(crc16_ccitt(b"123456789"), 0x29B1)

    def test_empty(self):
        self.assertEqual(crc16_ccitt(b""), 0xFFFF)

    def test_single_bit_flip_detected(self):
        a = bytes(range(60))
        b = bytearray(a)
        b[30] ^= 0x01
        self.assertNotEqual(crc16_ccitt(a), crc16_ccitt(bytes(b)))


class TestPacketDefinitions(unittest.TestCase):
    """docs/uart_protocol.md v0.6 の値を直接書いて生成物を検証する。

    protocol.toml と同じ値をここに書き写すのは重複だが、**それが目的**。
    生成器のバグや定義の書き換えミスを、仕様書の値そのもので検出する。
    """

    EXPECTED = {
        "LIDAR_SECTOR": (0x01, 69, "s2p"),
        "TELEMETRY": (0x02, 66, "s2p"),
        "CONFIG_ACK": (0x03, 7, "s2p"),
        "LOG": (0x04, None, "s2p"),
        "LIDAR_SECTOR_I": (0x05, 99, "s2p"),
        "PONG": (0x06, 12, "s2p"),
        "VERSION": (0x07, 10, "s2p"),
        "STATS": (0x08, 48, "s2p"),
        "LIDAR_SECTOR_C": (0x09, 39, "s2p"),
        "LIMITS": (0x0A, 16, "s2p"),
        "COMMAND": (0x10, 14, "p2s"),
        "CONFIG_SET": (0x11, 6, "p2s"),
        "PING": (0x12, 4, "p2s"),
        "CONFIG_GET": (0x13, 2, "p2s"),
        "VERSION_REQ": (0x14, 0, "p2s"),
        "LIMITS_REQ": (0x15, 0, "p2s"),
    }

    def test_all_packets_present(self):
        self.assertEqual(set(packets.BY_NAME), set(self.EXPECTED))

    def test_type_len_dir(self):
        for name, (ptype, plen, direction) in self.EXPECTED.items():
            with self.subTest(packet=name):
                cls = packets.BY_NAME[name]
                self.assertEqual(cls.TYPE, ptype)
                self.assertEqual(cls.LEN, plen)
                self.assertEqual(cls.DIR, direction)

    def test_struct_size_matches_len(self):
        for name, cls in packets.BY_NAME.items():
            if cls.LEN is None or cls.LEN == 0:
                continue
            with self.subTest(packet=name):
                self.assertEqual(struct.calcsize(cls.FMT), cls.LEN)

    def test_direction_rule(self):
        """0x01-0x0F = STM32→Pi、0x10-0x1F = Pi→STM32。"""
        for cls in packets.ALL:
            with self.subTest(packet=cls.NAME):
                self.assertEqual(cls.DIR, "s2p" if cls.TYPE < 0x10 else "p2s")

    def test_protocol_version(self):
        self.assertEqual(packets.PROTOCOL_VERSION, 0x000C)

    def test_generated_files_up_to_date(self):
        """protocol.toml を編集して再生成し忘れていないか。"""
        r = subprocess.run(
            [sys.executable, "raspi/proto/generate.py", "--check"],
            cwd=REPO, capture_output=True, text=True)
        self.assertEqual(r.returncode, 0, r.stderr)


class TestTelemetryWireLayout(unittest.TestCase):
    """TELEMETRY のバイト位置を固定する回帰テスト。

    v0.4 の策定中、`odom_dist` と `accel` の順序が Pi 側と STM32 側で
    食い違ったまま2往復した。生成器を経由せず**バイト位置を直接**検証して、
    同じ事故が二度起きないようにする。
    """

    def test_field_offsets(self):
        t = packets.Telemetry(
            t_us=0x01020304,
            flags=0x0A0B0C0D,
            speed=1111,
            yaw_rate=2222,
            steer_actual=3333,
            steer_cmd_echo=4444,
            wheel_speed=[41, 42, 43, 44],
            odom_dist=[0x11223344, -0x0055667],
            accel_x=51, accel_y=52, accel_z=53,
            pitch=61, roll=62,
            motor_current=[71, 72, 73],
            torque_cmd=[81, 82],
            temp=[11, 22, 33, 44],
            batt_voltage_drive=201, batt_voltage_signal=202,
            batt_current_drive=203, batt_current_signal=204,
            us_front=205, us_rear=206,
            md_status=[0x11, 0x22, 0x33],
            cmd_seq_echo=0xAB,
        )
        buf = t.encode()
        self.assertEqual(len(buf), 66)

        def u32(off): return struct.unpack_from("<I", buf, off)[0]
        def i32(off): return struct.unpack_from("<i", buf, off)[0]
        def i16(off): return struct.unpack_from("<h", buf, off)[0]

        self.assertEqual(u32(0), 0x01020304, "t_us @0")
        self.assertEqual(u32(4), 0x0A0B0C0D, "flags @4")
        self.assertEqual(i16(8), 1111, "speed @8")
        self.assertEqual(i16(10), 2222, "yaw_rate @10")
        self.assertEqual(i16(12), 3333, "steer_actual @12")
        self.assertEqual(i16(14), 4444, "steer_cmd_echo @14")
        self.assertEqual(i16(16), 41, "wheel_speed[0] @16")
        self.assertEqual(i16(22), 44, "wheel_speed[3] @22")
        # ★ ここが v0.4 で確定した順序。accel より odom_dist が先。
        self.assertEqual(i32(24), 0x11223344, "odom_dist[0] @24")
        self.assertEqual(i32(28), -0x0055667, "odom_dist[1] @28")
        self.assertEqual(i16(32), 51, "accel_x @32")
        self.assertEqual(i16(36), 53, "accel_z @36")
        self.assertEqual(i16(38), 61, "pitch @38")
        self.assertEqual(i16(40), 62, "roll @40")
        self.assertEqual(i16(42), 71, "motor_current[0] @42")
        self.assertEqual(i16(48), 81, "torque_cmd[0] @48")
        self.assertEqual(buf[52], 11, "temp[0] @52")
        self.assertEqual(buf[56], 201, "batt_voltage_drive @56")
        self.assertEqual(buf[60], 205, "us_front @60")
        self.assertEqual(buf[62], 0x11, "md_status[0] @62")
        self.assertEqual(buf[65], 0xAB, "cmd_seq_echo @65")

    def test_odom_dist_is_signed(self):
        """後退で負になる。符号なしで読むと 429km 飛ぶ。"""
        t = packets.Telemetry(odom_dist=[-1, -2147483648])
        got = packets.Telemetry.decode(t.encode())
        self.assertEqual(got.odom_dist, [-1, -2147483648])

    def test_motor_current_is_signed(self):
        """制動時に負になる（MD の iq は双方向）。"""
        got = packets.Telemetry.decode(
            packets.Telemetry(motor_current=[-1000, 0, 1000]).encode())
        self.assertEqual(got.motor_current, [-1000, 0, 1000])


class TestRoundTrip(unittest.TestCase):
    def _sample(self, cls):
        """フィールドごとに区別できる値を詰めたインスタンスを作る。"""
        kwargs, n = {}, 1
        for name, f in cls.__dataclass_fields__.items():
            if name.isupper():
                continue
            default = getattr(cls(), name)
            if isinstance(default, list):
                kwargs[name] = [(n + i) for i in range(len(default))]
                n += len(default)
            elif isinstance(default, bytes):
                kwargs[name] = b"hello"
            elif isinstance(default, float):
                kwargs[name] = 1.5
            else:
                kwargs[name] = n
            n += 1
        return cls(**kwargs)

    def test_roundtrip_all_packets(self):
        for name, cls in packets.BY_NAME.items():
            with self.subTest(packet=name):
                original = self._sample(cls)
                payload = original.encode()
                if cls.LEN is not None:
                    self.assertEqual(len(payload), cls.LEN)
                self.assertEqual(cls.decode(payload), original)

    def test_log_variable_length(self):
        msg = "ステア過熱 temp[2]=95".encode()
        payload = packets.Log(severity=packets.LogSeverity.WARN, message=msg).encode()
        got = packets.Log.decode(payload)
        self.assertEqual(got.severity, 2)
        self.assertEqual(got.message, msg)
        self.assertEqual(len(payload), 1 + len(msg))

    def test_version_req_is_empty(self):
        self.assertEqual(packets.VersionReq().encode(), b"")
        self.assertEqual(packets.VersionReq.decode(b""), packets.VersionReq())

    def test_config_ack_float(self):
        got = packets.ConfigAck.decode(
            packets.ConfigAck(param_id=packets.Param.MAX_SPEED,
                              applied=2.5, result=0).encode())
        self.assertAlmostEqual(got.applied, 2.5)


class TestBuildFrame(unittest.TestCase):
    def test_layout(self):
        frame = build_frame(0x12, 57, struct.pack("<I", 0xDEADBEEF))
        self.assertEqual(frame[:2], b"\xAA\x55")
        self.assertEqual(frame[2], 0x12)
        self.assertEqual(frame[3], 57)
        self.assertEqual(frame[4], 4)
        self.assertEqual(len(frame), 4 + 7)
        crc = frame[-2] | frame[-1] << 8
        self.assertEqual(crc, crc16_ccitt(frame[2:-2]), "CRC 範囲は TYPE から PAYLOAD 末尾")

    def test_empty_payload(self):
        frame = build_frame(0x14, 0)
        self.assertEqual(len(frame), 7)
        self.assertEqual(frame[4], 0)

    def test_payload_too_long(self):
        with self.assertRaises(ValueError):
            build_frame(0x02, 0, b"\x00" * 256)

    def test_encoder_increments_and_wraps(self):
        enc = FrameEncoder(start_seq=254)
        seqs = [enc.encode_raw(0x12, b"")[3] for _ in range(4)]
        self.assertEqual(seqs, [254, 255, 0, 1])


class TestFrameParser(unittest.TestCase):
    def setUp(self):
        self.enc = FrameEncoder()
        self.parser = FrameParser()

    def _telemetry(self, **kw):
        return self.enc.encode(packets.Telemetry(**kw))

    def test_single_frame(self):
        frames = self.parser.feed(self._telemetry(speed=1234))
        self.assertEqual(len(frames), 1)
        self.assertEqual(frames[0].type, 0x02)
        self.assertEqual(frames[0].decode().speed, 1234)
        self.assertEqual(self.parser.stats.frame_ok, 1)

    def test_multiple_frames_one_feed(self):
        data = self._telemetry(speed=1) + self._telemetry(speed=2) + self._telemetry(speed=3)
        frames = self.parser.feed(data)
        self.assertEqual([f.decode().speed for f in frames], [1, 2, 3])

    def test_byte_at_a_time(self):
        """分割されて届いても復元できる（UART では日常的に起きる）。"""
        data = self._telemetry(speed=999)
        got = [f for b in data for f in self.parser.feed(bytes([b]))]
        self.assertEqual(len(got), 1)
        self.assertEqual(got[0].decode().speed, 999)

    def test_garbage_prefix_resync(self):
        data = b"\x00\x01\x02garbage" + self._telemetry(speed=7)
        frames = self.parser.feed(data)
        self.assertEqual(len(frames), 1)
        self.assertGreater(self.parser.stats.resync_bytes, 0)

    def test_repeated_sync0(self):
        """0xAA 0xAA 0x55 を取りこぼさない。"""
        data = b"\xAA\xAA" + self._telemetry(speed=5)
        frames = self.parser.feed(data)
        self.assertEqual(len(frames), 1)

    def test_sync_pattern_inside_payload(self):
        """ペイロード中に 0xAA 0x55 が現れても誤同期しない。"""
        payload = self._telemetry(speed=struct.unpack("<h", b"\xAA\x55")[0])
        frames = self.parser.feed(payload)
        self.assertEqual(len(frames), 1)
        self.assertEqual(self.parser.stats.frame_ok, 1)

    def test_crc_error_then_recovers(self):
        bad = bytearray(self._telemetry(speed=1))
        bad[20] ^= 0xFF
        frames = self.parser.feed(bytes(bad) + self._telemetry(speed=2))
        self.assertEqual(self.parser.stats.crc_error, 1)
        self.assertEqual(len(frames), 1)
        self.assertEqual(frames[0].decode().speed, 2)

    def test_len_mismatch(self):
        frame = bytearray(build_frame(0x02, 0, b"\x00" * 10))  # TELEMETRY は 66
        frames = self.parser.feed(bytes(frame) + self._telemetry(speed=3))
        self.assertEqual(self.parser.stats.len_error, 1)
        self.assertEqual(len(frames), 1)

    def test_unknown_type(self):
        frames = self.parser.feed(build_frame(0x7E, 0, b"\x01\x02"))
        self.assertEqual(frames, [])
        self.assertEqual(self.parser.stats.unknown_type, 1)

    def test_zero_length_frame(self):
        """VERSION_REQ (LEN=0)。ここでハングする実装が多い。"""
        frames = self.parser.feed(build_frame(0x14, 0))
        self.assertEqual(len(frames), 1)
        self.assertEqual(frames[0].payload, b"")
        self.assertIsInstance(frames[0].decode(), packets.VersionReq)

    def test_packet_loss_counted(self):
        self.parser.feed(build_frame(0x12, 10, b"\x00" * 4))
        self.parser.feed(build_frame(0x12, 14, b"\x00" * 4))
        self.assertEqual(self.parser.stats.packet_loss, 3)

    def test_packet_loss_across_seq_wrap(self):
        self.parser.feed(build_frame(0x12, 254, b"\x00" * 4))
        self.parser.feed(build_frame(0x12, 1, b"\x00" * 4))
        self.assertEqual(self.parser.stats.packet_loss, 2)

    def test_seq_reorder_not_counted_as_loss(self):
        """STM32 の優先度キューで SEQ が逆行しても大量ロスに化けない。

        単純な (seq-last-1)&0xFF だと 1つの逆行を 254 ロスと誤計上していた。
        """
        for s in (10, 12, 11, 13):        # 11 が 12 に追い越される並び
            self.parser.feed(build_frame(0x12, s, b"\x00" * 4))
        st = self.parser.stats
        self.assertEqual(st.reordered, 1)          # 逆行 1回
        self.assertEqual(st.packet_loss, 0)        # 全フレーム届いたので実ロスは 0
        self.assertEqual(st.duplicate, 0)

    def test_duplicate_seq(self):
        self.parser.feed(build_frame(0x12, 7, b"\x00" * 4))
        self.parser.feed(build_frame(0x12, 7, b"\x00" * 4))
        self.assertEqual(self.parser.stats.duplicate, 1)
        self.assertEqual(self.parser.stats.packet_loss, 0)

    def test_single_backward_step_bounded(self):
        """逆行1個が 254 ロスに化けないことを直接確認（退行テスト）。"""
        self.parser.feed(build_frame(0x12, 100, b"\x00" * 4))
        self.parser.feed(build_frame(0x12, 99, b"\x00" * 4))
        self.assertEqual(self.parser.stats.packet_loss, 0)
        self.assertEqual(self.parser.stats.reordered, 1)

    def test_wrong_direction_rejected(self):
        """Pi 側の受信機は STM32→Pi のパケットだけを受け付ける。"""
        parser = FrameParser(expect_types=packets.S2P_TYPES)
        frames = parser.feed(build_frame(packets.Command.TYPE, 0,
                                         packets.Command().encode()))
        self.assertEqual(frames, [])
        self.assertEqual(parser.stats.wrong_direction, 1)

    def test_reset_clears_state(self):
        self.parser.feed(self._telemetry())
        self.parser.reset()
        self.parser.feed(build_frame(0x12, 200, b"\x00" * 4))
        self.assertEqual(self.parser.stats.packet_loss, 0)

    def test_realistic_mixed_stream(self):
        """LiDAR・テレメトリ・PONG が混ざった実際の並びを流す。"""
        enc = FrameEncoder()
        expected = []
        stream = bytearray()
        for i in range(12):
            stream += enc.encode(packets.LidarSector(sector_idx=i, dist=list(range(30))))
            expected.append(0x01)
            if i % 6 == 0:
                stream += enc.encode(packets.Telemetry(t_us=i))
                expected.append(0x02)
        stream += enc.encode(packets.Pong(ping_id=42))
        expected.append(0x06)

        # 実際の read と同じように 64 バイトずつ細切れで投入する
        frames = [f for i in range(0, len(stream), 64)
                  for f in self.parser.feed(bytes(stream[i:i + 64]))]
        self.assertEqual([f.type for f in frames], expected)
        self.assertEqual(self.parser.stats.crc_error, 0)
        self.assertEqual(self.parser.stats.packet_loss, 0)
        self.assertEqual(self.parser.stats.frame_ok, len(expected))


class TestScaleMetadata(unittest.TestCase):
    def test_telemetry_scales(self):
        meta = packets.Telemetry.META
        self.assertEqual(meta["speed"], (0.001, "m/s"))
        self.assertEqual(meta["steer_actual"], (0.0001, "rad"))
        self.assertEqual(meta["odom_dist"], (0.0001, "m"))
        self.assertEqual(meta["batt_current_signal"], (0.02, "A"))

    def test_si_conversion(self):
        """raw * scale で SI 単位になる。"""
        t = packets.Telemetry(speed=1500, steer_actual=-2618, odom_dist=[12345, 0])
        self.assertAlmostEqual(t.speed * packets.Telemetry.META["speed"][0], 1.5)
        self.assertAlmostEqual(t.steer_actual * 1e-4, -0.2618, places=4)
        self.assertAlmostEqual(t.odom_dist[0] * 1e-4, 1.2345)


if __name__ == "__main__":
    unittest.main(verbosity=2)
