"""画素の並びの扱い（カメラ不要・Mac でも動く）。

**libcamera の形式名はメモリ上のバイト順ではない。**
32bit ワードにパックしたときの並びを指すので、リトルエンディアンのメモリ上では逆になる。

実測（2026-08-08・実機）: `format="RGB888"` でカメラの前に赤いシートを置くと、
配列の **ch2 が最大**になった（ch0=151 ch1=115 **ch2=198**）。
つまりメモリ上は `B, G, R`。PNG に素直に書くと赤が青紫に写る。

この対応を忘れると同じ罠を踏むのでテストで固定する。

    python3 -m unittest discover -s raspi/tests -t .
"""

from __future__ import annotations

import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from raspi.tools.shm_view import to_rgb, write_png  # noqa: E402


def _memory_format(name: str) -> str:
    """camera_node の変換表。picamera2 を import せずに取り出す。

    `camera_node` は picamera2 に依存するので Mac では import できない。
    表そのものは純粋なデータなので、ここに写しを持たずソースから読む。
    """
    import ast

    src = (Path(__file__).resolve().parents[1] / "nodes" / "camera_node.py").read_text()
    tree = ast.parse(src)
    for node in tree.body:
        if isinstance(node, ast.Assign) and getattr(node.targets[0], "id", "") == "_MEMORY_ORDER":
            table = ast.literal_eval(node.value)
            return table.get(name, name)
    raise AssertionError("_MEMORY_ORDER が camera_node.py に見つからない")


class TestMemoryFormatTable(unittest.TestCase):
    def test_rgb888_is_bgr_in_memory(self):
        """実測の結論。ここが逆だと下流全部で赤と青が入れ替わる。"""
        self.assertEqual(_memory_format("RGB888"), "BGR888")

    def test_bgr888_is_rgb_in_memory(self):
        self.assertEqual(_memory_format("BGR888"), "RGB888")

    def test_mapping_is_an_involution(self):
        """2回変換すると元に戻ること（表の書き間違い検出）。"""
        for name in ("RGB888", "BGR888", "XRGB8888", "XBGR8888"):
            self.assertEqual(_memory_format(_memory_format(name)), name)

    def test_unknown_format_passes_through(self):
        self.assertEqual(_memory_format("GREY"), "GREY")


class TestToRgb(unittest.TestCase):
    def setUp(self):
        import numpy as np

        self.np = np
        # 1画素だけ。B=10, G=20, R=30 が入っている想定
        self.bgr = np.array([[[10, 20, 30]]], dtype=np.uint8)

    def test_bgr_is_swapped(self):
        out = to_rgb(self.bgr, "BGR888")
        self.assertEqual(list(out[0, 0]), [30, 20, 10])

    def test_rgb_is_untouched(self):
        """正しい並びの画を壊さないこと。"""
        out = to_rgb(self.bgr, "RGB888")
        self.assertEqual(list(out[0, 0]), [10, 20, 30])

    def test_grey_is_untouched(self):
        g = self.np.zeros((2, 2), dtype=self.np.uint8)
        self.assertIs(to_rgb(g, "GREY"), g)

    def test_red_sheet_scenario(self):
        """実機で起きたことの再現: 赤い被写体が BGR で入ってくる。"""
        import numpy as np

        red_in_bgr = np.zeros((4, 4, 3), dtype=np.uint8)
        red_in_bgr[..., 2] = 200          # メモリ上の3バイト目が赤
        out = to_rgb(red_in_bgr, "BGR888")
        self.assertEqual(int(out[..., 0].mean()), 200)   # PNG では ch0 が赤
        self.assertEqual(int(out[..., 2].mean()), 0)


class TestPngWriter(unittest.TestCase):
    def test_png_signature_and_size(self):
        import tempfile

        import numpy as np

        a = np.zeros((5, 7, 3), dtype=np.uint8)
        p = Path(tempfile.mkdtemp()) / "t.png"
        write_png(p, a)
        d = p.read_bytes()
        self.assertEqual(d[:8], b"\x89PNG\r\n\x1a\n")
        import struct

        w, h, depth, color = struct.unpack(">IIBB", d[16:26])
        self.assertEqual((w, h, depth, color), (7, 5, 8, 2))

    def test_grey_written_as_grayscale(self):
        import struct
        import tempfile

        import numpy as np

        p = Path(tempfile.mkdtemp()) / "g.png"
        write_png(p, np.zeros((3, 4), dtype=np.uint8))
        color = struct.unpack(">IIBB", p.read_bytes()[16:26])[3]
        self.assertEqual(color, 0)


if __name__ == "__main__":
    unittest.main()
