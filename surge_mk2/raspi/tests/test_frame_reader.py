"""`raspi/core/frame_reader.py` — 共有メモリの作り直し（camera_node の再起動）から復帰できるか。"""

import sys
import time
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

import numpy as np  # noqa: E402

from raspi.bus import FrameRing  # noqa: E402
from raspi.core import frame_reader  # noqa: E402
from raspi.core.frame_reader import FrameReader  # noqa: E402
from raspi.msgs import ImageRef  # noqa: E402

NAME = "surge_test_frame_reader"


def _ref(desc) -> ImageRef:
    return ImageRef(t_capture=desc.t_capture_ns, cam="front", shm_name=desc.name,
                    slot=desc.slot, ring_seq=desc.seq, frame_id=desc.frame_id,
                    width=desc.width, height=desc.height, fmt=desc.fmt,
                    stride=desc.stride, nbytes=desc.nbytes)


class TestReattachAfterRestart(unittest.TestCase):
    def test_recreated_ring_is_read_on_the_next_frame(self):
        """旧リングが短命（`write_seq` が `_STALE_GAP` より小さい）でも、作り直された
        新しいリングの2枚目までには読める（以前は旧＋1000 枚まで古い方を掴み続けた）。"""
        img = np.zeros((6, 8, 3), np.uint8)
        old = FrameRing.create(NAME, 8, 6, "BGR888")
        reader = FrameReader()
        try:
            for _ in range(20):
                desc = old.write(img, t_capture_ns=time.monotonic_ns())
            self.assertIsNotNone(reader.read(_ref(desc)))
            old.unlink()                                   # camera_node が終了
            time.sleep(frame_reader._MAX_FRAME_AGE_NS / 1e9 + 0.05)
            new = FrameRing.create(NAME, 8, 6, "BGR888")
            try:
                got_at = None
                for i in range(1, 50):
                    desc = new.write(img + 7, t_capture_ns=time.monotonic_ns())
                    got = reader.read(_ref(desc))
                    if got is not None:
                        got_at = i
                        self.assertEqual(int(got[0][0, 0, 0]), 7, "古いリングの画を読んだ")
                        break
                self.assertIsNotNone(got_at)
                self.assertLessEqual(got_at, 2)
            finally:
                new.unlink()
        finally:
            reader.close()

    def test_stale_gap_matches_jpeg(self):
        from raspi.core.jpeg import STALE_GAP

        self.assertEqual(frame_reader._STALE_GAP, STALE_GAP)


if __name__ == "__main__":
    unittest.main()
