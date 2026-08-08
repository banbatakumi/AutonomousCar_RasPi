"""共有メモリリングの単体テスト（カメラ不要・Mac でも動く）。

seqlock は「正常時に読める」ことより **「壊れたときに壊れたと分かる」**
ことが要件なので、意図的に上書きを起こして検出できるかを見る。

    python3 -m unittest discover -s raspi/tests -t .
"""

from __future__ import annotations

import struct
import sys
import unittest
from multiprocessing import shared_memory
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from raspi.bus.shm_ring import (  # noqa: E402
    HEADER_SIZE,
    MAGIC,
    SLOT_META_SIZE,
    FrameRing,
)

W, H = 8, 4
FMT = "RGB888"
FRAME = W * H * 3


def frame(byte: int) -> bytes:
    return bytes([byte]) * FRAME


class RingCase(unittest.TestCase):
    """テストごとに固有名を使う。/dev/shm に残骸を残さない。"""

    def setUp(self):
        self.name = f"surge_test_{id(self):x}"
        self.rings: list[FrameRing] = []

    def tearDown(self):
        for r in reversed(self.rings):
            try:
                r.unlink()
            except Exception:
                pass

    def create(self, **kw) -> FrameRing:
        r = FrameRing.create(self.name, W, H, FMT, **kw)
        self.rings.append(r)
        return r

    def attach(self) -> FrameRing:
        r = FrameRing.attach(self.name)
        self.rings.append(r)
        return r


class TestLayout(RingCase):
    def test_geometry_round_trips(self):
        r = self.create(n_slots=8)
        self.assertEqual((r.width, r.height, r.fmt), (W, H, FMT))
        self.assertEqual(r.n_slots, 8)
        self.assertEqual(r.stride, W * 3)
        self.assertGreaterEqual(r.slot_bytes, FRAME)

    def test_attach_sees_same_geometry(self):
        self.create(n_slots=4)
        a = self.attach()
        self.assertEqual((a.width, a.height, a.fmt, a.n_slots), (W, H, FMT, 4))

    def test_rejects_unknown_format(self):
        with self.assertRaises(ValueError):
            FrameRing.create(self.name, W, H, "JPEG")

    def test_rejects_single_slot(self):
        """1枚だと読み書きが必ず衝突するので許さない。"""
        with self.assertRaisesRegex(ValueError, "n_slots"):
            FrameRing.create(self.name, W, H, FMT, n_slots=1)

    def test_stale_segment_is_replaced(self):
        """前回異常終了して残骸があっても起動できること。"""
        first = self.create(n_slots=4)
        first.write(frame(1), t_capture_ns=1)
        second = FrameRing.create(self.name, W, H, FMT, n_slots=8)
        self.rings.append(second)
        self.assertEqual(second.n_slots, 8)
        self.assertEqual(second.write_seq, 0)      # 作り直されている

    def test_bad_magic_rejected(self):
        shm = shared_memory.SharedMemory(name=self.name, create=True, size=4096)
        try:
            shm.buf[:8] = b"NOTARING"
            with self.assertRaises(ValueError):
                FrameRing.attach(self.name)
        finally:
            shm.close()
            shm.unlink()


class TestWriteRead(RingCase):
    def test_latest_returns_what_was_written(self):
        r = self.create(n_slots=4)
        r.write(frame(0xAB), t_capture_ns=1234, frame_id=7)
        ref = r.latest()
        self.assertIsNotNone(ref)
        self.assertEqual(ref.desc.t_capture_ns, 1234)
        self.assertEqual(ref.desc.frame_id, 7)
        self.assertEqual(ref.copy(), frame(0xAB))
        self.assertTrue(ref.still_valid())

    def test_latest_is_none_before_any_write(self):
        self.assertIsNone(self.create().latest())

    def test_reader_in_another_handle_sees_writes(self):
        w = self.create(n_slots=4)
        rd = self.attach()
        w.write(frame(0x11), t_capture_ns=5)
        self.assertEqual(rd.latest().copy(), frame(0x11))
        w.write(frame(0x22), t_capture_ns=6)
        self.assertEqual(rd.latest().copy(), frame(0x22))

    def test_slots_rotate(self):
        r = self.create(n_slots=4)
        slots = [r.write(frame(i), t_capture_ns=i).slot for i in range(9)]
        self.assertEqual(slots, [0, 1, 2, 3, 0, 1, 2, 3, 0])

    def test_seq_counts_frames(self):
        r = self.create(n_slots=4)
        for i in range(5):
            r.write(frame(i), t_capture_ns=i)
        self.assertEqual(r.write_seq, 5)
        self.assertEqual(r.latest().desc.seq, 5)

    def test_oversized_frame_rejected(self):
        r = self.create(n_slots=2)
        with self.assertRaisesRegex(ValueError, "大きすぎ"):
            r.write(bytes(r.slot_bytes + 1), t_capture_ns=0)

    def test_reader_cannot_write(self):
        self.create(n_slots=2)
        rd = self.attach()
        with self.assertRaises(RuntimeError):
            rd.write(frame(1), t_capture_ns=0)

    def test_short_frame_records_actual_size(self):
        r = self.create(n_slots=2)
        r.write(b"\x01\x02\x03", t_capture_ns=0)
        ref = r.latest()
        self.assertEqual(ref.desc.nbytes, 3)
        self.assertEqual(ref.copy(), b"\x01\x02\x03")


class TestSeqlock(RingCase):
    """**壊れたと分かること**が要件。正常系より重く見る。"""

    def test_valid_while_untouched(self):
        r = self.create(n_slots=4)
        r.write(frame(1), t_capture_ns=1)
        ref = r.latest()
        r.write(frame(2), t_capture_ns=2)      # 別スロットに書く
        self.assertTrue(ref.still_valid())      # 参照中のスロットは無事

    def test_detects_overwrite_of_the_same_slot(self):
        """一周して同じスロットを踏まれたら検出できること。"""
        r = self.create(n_slots=2)
        r.write(frame(1), t_capture_ns=1)
        ref = r.latest()
        self.assertTrue(ref.still_valid())
        r.write(frame(2), t_capture_ns=2)      # slot1
        r.write(frame(3), t_capture_ns=3)      # slot0 ← ref と同じ
        self.assertFalse(ref.still_valid())

    def test_mid_write_is_not_returned(self):
        """書き込み中（seq が奇数）のスロットは latest() が返さないこと。"""
        r = self.create(n_slots=4)
        r.write(frame(1), t_capture_ns=1)
        # 書き込み中の状態を手で作る
        off = r._slot_meta_off(0)
        seq, t, fid, n = struct.unpack_from("<4Q", r._shm.buf, off)
        struct.pack_into("<4Q", r._shm.buf, off, seq + 1, t, fid, n)
        self.assertIsNone(r.latest())

    def test_latest_copy_retries_and_succeeds(self):
        r = self.create(n_slots=4)
        r.write(frame(9), t_capture_ns=42)
        got = r.latest_copy()
        self.assertIsNotNone(got)
        desc, data = got
        self.assertEqual(desc.t_capture_ns, 42)
        self.assertEqual(data, frame(9))

    def test_latest_copy_gives_up_when_always_torn(self):
        """書き込み中のままなら諦めて None を返すこと（無限ループしない）。"""
        r = self.create(n_slots=4)
        r.write(frame(1), t_capture_ns=1)
        off = r._slot_meta_off(0)
        seq, t, fid, n = struct.unpack_from("<4Q", r._shm.buf, off)
        struct.pack_into("<4Q", r._shm.buf, off, seq + 1, t, fid, n)
        self.assertIsNone(r.latest_copy(retries=3))


class TestReadByDesc(RingCase):
    """バス経由で説明だけ受け取った読み手の経路。"""

    def test_read_by_desc(self):
        w = self.create(n_slots=4)
        desc = w.write(frame(0x55), t_capture_ns=99, frame_id=3)
        rd = self.attach()
        ref = rd.read(desc)
        self.assertIsNotNone(ref)
        self.assertEqual(ref.copy(), frame(0x55))
        self.assertEqual(ref.desc.t_capture_ns, 99)

    def test_stale_desc_is_rejected(self):
        """一周して古くなった説明は None（古い画で制御しないため）。"""
        w = self.create(n_slots=4)
        desc = w.write(frame(1), t_capture_ns=1, frame_id=1)
        for i in range(2, 8):
            w.write(frame(i), t_capture_ns=i, frame_id=i)
        self.assertIsNone(w.read(desc))

    def test_desc_from_the_future_is_rejected(self):
        w = self.create(n_slots=4)
        desc = w.write(frame(1), t_capture_ns=1, frame_id=1)
        self.assertIsNone(w.read(desc._replace(seq=desc.seq + 10)))

    def test_desc_with_bad_slot_is_rejected(self):
        w = self.create(n_slots=4)
        desc = w.write(frame(1), t_capture_ns=1)
        self.assertIsNone(w.read(desc._replace(slot=99)))


class TestNumpyView(RingCase):
    def test_as_array_shape_and_zero_copy(self):
        import numpy as np

        r = self.create(n_slots=4)
        payload = bytes(range(256)) * (FRAME // 256 + 1)
        r.write(payload[:FRAME], t_capture_ns=1)
        arr = r.latest().as_array()
        self.assertEqual(arr.shape, (H, W, 3))
        self.assertEqual(arr.dtype, np.uint8)
        # 共有メモリを直接見ていること（自前のコピーを持っていない）
        self.assertFalse(arr.flags["OWNDATA"])

    def test_grey_is_two_dimensional(self):
        name = self.name
        r = FrameRing.create(name, W, H, "GREY", n_slots=2)
        self.rings.append(r)
        r.write(bytes(W * H), t_capture_ns=1)
        self.assertEqual(r.latest().as_array().shape, (H, W))


class TestCloseContract(RingCase):
    """**参照を残したまま閉じられない**という制約を固定する。

    黙って握り潰すと、後で `SharedMemory.__del__` が
    「Exception ignored」を吐いて原因が分からなくなる。
    """

    def test_close_blocked_while_array_alive(self):
        r = self.create(n_slots=2)
        r.write(frame(1), t_capture_ns=1)
        arr = r.latest().as_array()          # 共有メモリを参照したまま
        r.close()
        self.assertTrue(r.close_blocked)
        self.assertFalse(r._closed)          # 閉じ直せる状態のまま
        del arr
        r.close()
        self.assertTrue(r._closed)

    def test_close_succeeds_when_refs_released(self):
        r = self.create(n_slots=2)
        r.write(frame(1), t_capture_ns=1)
        data = r.latest().copy()             # コピーは参照を残さない
        self.assertEqual(len(data), FRAME)
        r.close()
        self.assertFalse(r.close_blocked)
        self.assertTrue(r._closed)


class TestSizing(RingCase):
    def test_memory_footprint_is_predictable(self):
        """実運用の見積りが立つこと（640x480x3 × 8枚 ≒ 7.4MB）。"""
        r = FrameRing.create(self.name, 640, 480, "RGB888", n_slots=8)
        self.rings.append(r)
        self.assertEqual(r.slot_bytes, 640 * 480 * 3)
        total = r._shm.size
        self.assertLess(total, 8 * 1024 * 1024)
        self.assertGreaterEqual(total, 8 * 640 * 480 * 3)

    def test_header_and_meta_do_not_overlap_data(self):
        r = self.create(n_slots=8)
        self.assertGreaterEqual(r._data_off, HEADER_SIZE + 8 * SLOT_META_SIZE)
        self.assertEqual(r._shm.buf[:8], MAGIC)


if __name__ == "__main__":
    unittest.main()
