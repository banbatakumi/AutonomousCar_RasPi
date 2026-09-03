"""`raspi/nodes/line_perception_node.py` のテスト。

**実カメラは要らない。** 合成フレーム（白い帯を描いただけの画像）で、
色しきい値→帯の重心→IPM逆投影→`LineScan`化という配管が壊れずに流れることと、
白線の位置が動けば目標点の左右も動くことを確認する
（`test_cam_perception_node.py` と同じ「配管のテスト」の流儀）。
"""

import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

import numpy as np  # noqa: E402

from raspi.bus import FrameRing  # noqa: E402
from raspi.core.vehicle import Vehicle  # noqa: E402
from raspi.msgs import AutoCtrl, ImageRef  # noqa: E402
from raspi.msgs.types import TOPIC_AUTO_CTRL, TOPIC_IMAGE_FRONT, TOPIC_LINE_CAM  # noqa: E402
from raspi.nodes.line_perception_node import (  # noqa: E402
    LinePerceptionNode,
    white_mask,
)


def _frame_with_white_column(h: int, w: int, col: int, *, band_width: int = 20) -> np.ndarray:
    """暗い床の上に、縦方向に伸びる幅 `band_width` の白い帯を1本描いた合成フレーム。"""
    frame = np.full((h, w, 3), 30, dtype=np.uint8)          # 暗い床
    lo = max(0, col - band_width // 2)
    hi = min(w, col + band_width // 2)
    frame[:, lo:hi] = 240                                   # 白線
    return frame


def _frame_with_wide_blob_and_thin_line(h: int, w: int, *, blob_col: int, blob_width: int,
                                        line_col: int, line_width: int = 8) -> np.ndarray:
    """暗い床の上に、毛布・段ボール等を模した太い明るい面と、細い白線を1本ずつ描く。"""
    frame = np.full((h, w, 3), 30, dtype=np.uint8)
    blo, bhi = max(0, blob_col - blob_width // 2), min(w, blob_col + blob_width // 2)
    frame[:, blo:bhi] = 235
    llo, lhi = max(0, line_col - line_width // 2), min(w, line_col + line_width // 2)
    frame[:, llo:lhi] = 240
    return frame


class TestWhiteMask(unittest.TestCase):
    def test_bright_gray_is_white_colored_is_not(self):
        frame = np.zeros((4, 3, 3), dtype=np.uint8)
        frame[0, 0] = (240, 240, 240)                       # 明るく無彩色 → 白
        frame[0, 1] = (30, 30, 30)                           # 暗い → 白ではない
        frame[0, 2] = (240, 40, 40)                          # 明るいが彩度が高い → 白ではない
        mask = white_mask(frame)
        self.assertTrue(bool(mask[0, 0]))
        self.assertFalse(bool(mask[0, 1]))
        self.assertFalse(bool(mask[0, 2]))


class TestWideBlobDoesNotOverrideThinLine(unittest.TestCase):
    """実車確認（2026-09-03）で踏んだ不具合の再現テスト。

    毛布・カーテンのような太く明るい面が帯内にあると、単純な帯内重心は
    そちらへ引っ張られ、実際の白線（数cm幅）を見失っていた。`_band_centroid`
    の幅フィルタ（`_MAX_LINE_WIDTH_FRAC`）でこれを防ぐ。
    """

    def test_target_follows_thin_line_not_the_wide_blob(self):
        """線は画面右寄り（col=240）、塊は画面左寄り（col=60）に置く。
        画面右＝車両座標で右（y負）なので、太い塊（左＝y正）に引っ張られて
        いなければ `y < 0` になるはず。"""
        node = LinePerceptionNode(vehicle=Vehicle.load())
        frame = _frame_with_wide_blob_and_thin_line(240, 320, blob_col=60, blob_width=80,
                                                     line_col=240, line_width=8)
        st = node.process_frame(frame)
        self.assertTrue(st.near_seen or st.far_seen, "太い面に阻まれて何も検出できていない")
        y = st.far_y if st.far_seen else st.near_y
        self.assertLess(y, 0, "太い面（左寄り）ではなく細い線（右寄り）を追うべき")


class TestLinePerceptionNodeProcessFrame(unittest.TestCase):
    def test_returns_line_scan_shaped_message(self):
        node = LinePerceptionNode(vehicle=Vehicle.load())
        frame = _frame_with_white_column(240, 320, col=160)
        st = node.process_frame(frame, seq=7)
        self.assertEqual(st.seq, 7)
        self.assertTrue(st.seen)

    def test_line_at_center_gives_near_zero_lateral_offset(self):
        node = LinePerceptionNode(vehicle=Vehicle.load())
        frame = _frame_with_white_column(240, 320, col=160)     # 画面中央
        st = node.process_frame(frame)
        self.assertTrue(st.near_seen or st.far_seen)
        y = st.far_y if st.far_seen else st.near_y
        self.assertAlmostEqual(y, 0.0, delta=0.05)

    def test_line_shifted_left_gives_positive_lateral_offset(self):
        """画面の左寄りの白線は、車両座標で左（y正）に見えるはず。"""
        node = LinePerceptionNode(vehicle=Vehicle.load())
        left = node.process_frame(_frame_with_white_column(240, 320, col=100))
        right = node.process_frame(_frame_with_white_column(240, 320, col=220))

        self.assertTrue(left.near_seen or left.far_seen)
        self.assertTrue(right.near_seen or right.far_seen)
        ly = left.far_y if left.far_seen else left.near_y
        ry = right.far_y if right.far_seen else right.near_y
        self.assertGreater(ly, ry, "画面左の白線が車両座標でも左側に出ていない")

    def test_no_white_pixels_means_not_seen(self):
        node = LinePerceptionNode(vehicle=Vehicle.load())
        frame = np.full((240, 320, 3), 30, dtype=np.uint8)      # 全面「暗い床」
        st = node.process_frame(frame)
        self.assertFalse(st.seen)
        self.assertFalse(st.near_seen)
        self.assertFalse(st.far_seen)
        self.assertEqual(st.coverage, 0.0)

    def test_failed_frame_marks_not_seen(self):
        node = LinePerceptionNode(vehicle=Vehicle.load())
        st = node.failed_frame(seq=3)
        self.assertFalse(st.seen)
        self.assertEqual(st.seq, 3)


class _FakeSub:
    """`Subscriber` の身代わり。`latest` を読むだけ・`poll` は何も待たない。"""

    def __init__(self, latest=None):
        self.latest = latest or {}

    def poll(self, timeout_ms):
        return []


class _FakePub:
    def __init__(self):
        self.sent = []

    def send(self, topic, msg):
        self.sent.append((topic, msg))


class TestModeGating(unittest.TestCase):
    """`auto/ctrl` の `mode` で認識そのものの ACTIVE/IDLE を切り替える（CPU節約）。

    `line_perception_node` は常時起動（`surge-line-perception`）が前提になった
    ため、`line_trace` が選ばれていない間はフレームを読みすらしないことを
    ここで保証する（`cam_perception_node.py` の IDLE と同じ考え方）。
    """

    def _ref_with_frame(self, ring_name: str):
        ring = FrameRing.create(ring_name, 320, 240, "RGB888", n_slots=2)
        data = _frame_with_white_column(240, 320, col=160)
        desc = ring.write(data, t_capture_ns=1, frame_id=1)
        ref = ImageRef(shm_name=ring.name, slot=desc.slot, ring_seq=desc.seq,
                       frame_id=desc.frame_id, width=desc.width, height=desc.height,
                       fmt=desc.fmt, stride=desc.stride, nbytes=desc.nbytes, cam="front")
        return ring, ref

    def test_stays_idle_when_a_different_mode_is_selected(self):
        """白線がはっきり写るフレームがあっても、選ばれているモードが
        `line_trace` でなければ壁扱いのまま（認識を回さない）。"""
        node = LinePerceptionNode(vehicle=Vehicle.load())
        ring, ref = self._ref_with_frame("surge_test_line_gating_idle")
        try:
            sub = _FakeSub({TOPIC_IMAGE_FRONT: ref, TOPIC_AUTO_CTRL: AutoCtrl(mode="ftg_cam")})
            pub = _FakePub()
            node.run(sub=sub, pub=pub, duration_s=0.02)

            lines = [st for topic, st in pub.sent if topic == TOPIC_LINE_CAM]
            self.assertTrue(lines, "line/cam が一度も publish されていない")
            self.assertFalse(lines[-1].seen,
                             "line_trace以外が選ばれているのに認識結果が出ている")
        finally:
            node.close()
            ring.unlink()

    def test_no_auto_ctrl_at_all_stays_idle(self):
        """`auto/ctrl` がまだ一度も届いていない起動直後も IDLE 側に倒す。"""
        node = LinePerceptionNode(vehicle=Vehicle.load())
        ring, ref = self._ref_with_frame("surge_test_line_gating_no_ctrl")
        try:
            sub = _FakeSub({TOPIC_IMAGE_FRONT: ref})
            pub = _FakePub()
            node.run(sub=sub, pub=pub, duration_s=0.02)

            lines = [st for topic, st in pub.sent if topic == TOPIC_LINE_CAM]
            self.assertTrue(lines)
            self.assertFalse(lines[-1].seen)
        finally:
            node.close()
            ring.unlink()

    def test_line_trace_mode_activates_recognition(self):
        """`line_trace` が選ばれているときは実際にフレームを読んで認識すること。"""
        node = LinePerceptionNode(vehicle=Vehicle.load())
        ring, ref = self._ref_with_frame("surge_test_line_gating_active")
        try:
            sub = _FakeSub({TOPIC_IMAGE_FRONT: ref, TOPIC_AUTO_CTRL: AutoCtrl(mode="line_trace")})
            pub = _FakePub()
            node.run(sub=sub, pub=pub, duration_s=0.02)

            lines = [st for topic, st in pub.sent if topic == TOPIC_LINE_CAM]
            self.assertTrue(lines)
            self.assertTrue(any(st.seen for st in lines),
                            "line_trace が選ばれたのに認識結果が出ていない")
        finally:
            node.close()
            ring.unlink()


class TestReadFrame(unittest.TestCase):
    """`FrameReader`（`raspi/core/frame_reader.py`）越しの読み取り。"""

    def test_reads_back_a_written_frame(self):
        node = LinePerceptionNode(vehicle=Vehicle.load())
        ring = FrameRing.create("surge_test_line_perception", 16, 12, "RGB888", n_slots=4)
        try:
            data = np.zeros((12, 16, 3), dtype=np.uint8)
            data[..., 0] = 42
            desc = ring.write(data, t_capture_ns=123456789, frame_id=1)
            ref = ImageRef(shm_name=ring.name, slot=desc.slot, ring_seq=desc.seq,
                           frame_id=desc.frame_id, width=desc.width, height=desc.height,
                           fmt=desc.fmt, stride=desc.stride, nbytes=desc.nbytes, cam="front")

            got = node.read_frame(ref)
            self.assertIsNotNone(got)
            arr, t_capture = got
            self.assertTrue(np.array_equal(arr, data))
            self.assertEqual(t_capture, 123456789)
        finally:
            node.close()
            ring.unlink()

    def test_missing_shm_returns_none_instead_of_raising(self):
        node = LinePerceptionNode(vehicle=Vehicle.load())
        ref = ImageRef(shm_name="surge_does_not_exist", ring_seq=1)
        self.assertIsNone(node.read_frame(ref))


if __name__ == "__main__":
    unittest.main()
