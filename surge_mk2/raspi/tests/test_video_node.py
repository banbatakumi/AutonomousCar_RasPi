"""video_node — 固定 fps への並べ直しと、書き込みの終了処理。"""

from __future__ import annotations

import os
import tempfile
import threading
import time
import unittest
from pathlib import Path

import numpy as np

from raspi.bus import FrameRing, Publisher
from raspi.msgs.types import ImageRef
from raspi.nodes.video_node import VideoNode, _CamWriter

NS = 1_000_000_000


class TestCamWriter(unittest.TestCase):
    def _frames(self, times_s, fps=30.0):
        d = tempfile.mkdtemp()
        w = _CamWriter(Path(d) / "t.mp4", fps)
        img = np.zeros((48, 64, 3), dtype=np.uint8)
        for t in times_s:
            w.add(img, int(t * NS))
        w.close()
        return w

    def test_steady_30fps_writes_every_frame(self):
        w = self._frames([i / 30 for i in range(30)])
        self.assertEqual(w.written, 30)
        self.assertGreater(w.path.stat().st_size, 0)

    def test_gap_is_filled_to_keep_real_time(self):
        """0.5s 途切れても、動画の長さは実時間のまま（欠けたコマを直前の画で埋める）。"""
        w = self._frames([0.0, 1 / 30, 1 / 30 + 0.5, 1 / 30 + 0.5 + 1 / 30])
        self.assertEqual(w.written, round((0.5 + 2 / 30) * 30) + 1)

    def test_early_duplicate_is_dropped(self):
        w = self._frames([0.0, 0.001, 1 / 30])
        self.assertEqual(w.written, 2)

    def test_no_frame_no_file(self):
        d = tempfile.mkdtemp()
        w = _CamWriter(Path(d) / "t.mp4", 30.0)
        w.close()
        self.assertFalse(w.path.exists())


class TestVideoNodeBus(unittest.TestCase):
    """バスの `ImageRef` → 共有メモリのフレーム → mp4。`ipc://` はテンポラリに閉じ込める。"""

    def setUp(self):
        self._bus = tempfile.TemporaryDirectory()
        self._out = tempfile.TemporaryDirectory()
        self._old = os.environ.get("SURGE_BUS_DIR")
        os.environ["SURGE_BUS_DIR"] = self._bus.name

    def tearDown(self):
        if self._old is None:
            os.environ.pop("SURGE_BUS_DIR", None)
        else:
            os.environ["SURGE_BUS_DIR"] = self._old
        self._bus.cleanup()
        self._out.cleanup()

    def test_30fps_front_only_makes_one_mp4(self):
        ring = FrameRing.create("surge_vidtest", 64, 48, "RGB888", n_slots=4)
        node = VideoNode(Path(self._out.name) / "v", fps=30.0)
        pub = Publisher("camera")
        time.sleep(0.3)

        def feed():
            for i in range(45):                           # 30fps を1.5秒ぶん
                img = np.full((48, 64, 3), i * 5, dtype=np.uint8)
                d = ring.write(img.tobytes(), t_capture_ns=time.monotonic_ns(), frame_id=i)
                pub.send("image/front", ImageRef(
                    t_capture=d.t_capture_ns, cam="front", shm_name=d.name,
                    slot=d.slot, ring_seq=d.seq, frame_id=d.frame_id,
                    width=d.width, height=d.height, fmt=d.fmt,
                    stride=d.stride, nbytes=d.nbytes))
                time.sleep(1 / 30)

        feeder = threading.Thread(target=feed, daemon=True)
        feeder.start()
        try:
            node.run(duration_s=1.8)
        finally:
            feeder.join(timeout=3)
            node.close()
            pub.close()
            ring.close()
            ring.unlink()
        out = sorted(p.name for p in Path(self._out.name).iterdir())
        self.assertEqual(out, ["v_front.mp4"])            # 映らなかった後カメラのファイルは作らない
        n = node.writers["front"].written
        self.assertTrue(40 <= n <= 50, n)                 # 約1.5秒 × 30fps
        self.assertIsNone(node.error)


if __name__ == "__main__":
    unittest.main()
