"""`raspi/nodes/arrow_signal_node.py` のテスト。

**実物の電光掲示板は要らない。** 合成フレーム（HSVで狙った色のブロックを
描いただけの画像）で、色しきい値→重心→方向判定→confirm→`route/select`化
という配管が壊れずに流れることを確認する
（`test_line_perception_node.py` と同じ「配管のテスト」の流儀）。
"""

import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

import cv2  # noqa: E402
import numpy as np  # noqa: E402

from raspi.bus import FrameRing  # noqa: E402
from raspi.msgs import ImageRef, SignalConfig, VehicleState  # noqa: E402
from raspi.msgs.types import TOPIC_HB_PREFIX, TOPIC_IMAGE_FRONT, TOPIC_ROUTE_SELECT, \
    TOPIC_SIGNAL_CONFIG, TOPIC_SIGNAL_STATUS, TOPIC_VEHICLE_STATE  # noqa: E402
from raspi.nodes.arrow_signal_node import (  # noqa: E402
    ArrowSignalDetector,
    ArrowSignalNode,
    signal_mask,
)

#: `signal_mask()` の既定「薄い青」帯に入るBGR値（HSV h=110,s=100,v=200 を逆変換）
_LIGHT_BLUE_BGR = tuple(int(c) for c in
                        cv2.cvtColor(np.uint8([[[110, 100, 200]]]), cv2.COLOR_HSV2BGR)[0, 0])
#: 濃い色（彩度が高すぎる → 非該当のはず）
_DARK_BLUE_BGR = tuple(int(c) for c in
                       cv2.cvtColor(np.uint8([[[110, 255, 200]]]), cv2.COLOR_HSV2BGR)[0, 0])


def _frame_with_arrow(h: int, w: int, *, lit_col: int, lit_width: int,
                      color_bgr: tuple[int, int, int] = _LIGHT_BLUE_BGR) -> np.ndarray:
    """暗い背景に、ROI帯の中だけ指定色の矩形を描いた合成フレーム。"""
    frame = np.full((h, w, 3), 20, dtype=np.uint8)
    v0 = int(ArrowSignalDetector().roi_band[0] * h)
    v1 = int(ArrowSignalDetector().roi_band[1] * h)
    lo = max(0, lit_col - lit_width // 2)
    hi = min(w, lit_col + lit_width // 2)
    frame[v0:v1, lo:hi] = color_bgr
    return frame


class TestSignalMask(unittest.TestCase):
    def test_light_color_lit_dark_color_and_background_not(self):
        frame = np.zeros((1, 3, 3), dtype=np.uint8)
        frame[0, 0] = _LIGHT_BLUE_BGR
        frame[0, 1] = _DARK_BLUE_BGR
        frame[0, 2] = (20, 20, 20)
        mask = signal_mask(frame)
        self.assertTrue(bool(mask[0, 0]), "薄い色が点灯扱いになっていない")
        self.assertFalse(bool(mask[0, 1]), "濃い色が点灯扱いになっている（別の意味を持ちうる表示）")
        self.assertFalse(bool(mask[0, 2]), "無点灯の背景が点灯扱いになっている")


class TestArrowSignalDetector(unittest.TestCase):
    def test_no_lit_pixels_means_not_applicable(self):
        det = ArrowSignalDetector()
        frame = np.full((240, 320, 3), 20, dtype=np.uint8)
        self.assertIsNone(det.process_frame(frame).value)

    def test_left_shifted_block_is_left(self):
        det = ArrowSignalDetector()
        frame = _frame_with_arrow(240, 320, lit_col=60, lit_width=40)
        self.assertEqual(det.process_frame(frame).value, "left")

    def test_right_shifted_block_is_right(self):
        det = ArrowSignalDetector()
        frame = _frame_with_arrow(240, 320, lit_col=260, lit_width=40)
        self.assertEqual(det.process_frame(frame).value, "right")

    def test_centered_block_is_straight(self):
        det = ArrowSignalDetector()
        frame = _frame_with_arrow(240, 320, lit_col=160, lit_width=200)
        self.assertEqual(det.process_frame(frame).value, "straight")


class TestArrowSignalNodeProcessCycle(unittest.TestCase):
    def test_single_frame_does_not_confirm(self):
        """単発フレームでは`route/select`を出さない（チャタリング対策）。"""
        node = ArrowSignalNode()
        frame = _frame_with_arrow(240, 320, lit_col=60, lit_width=40)
        self.assertIsNone(node.process_cycle(frame))

    def test_n_consecutive_frames_confirm_and_publish(self):
        node = ArrowSignalNode(confirm_frames=3)
        frame = _frame_with_arrow(240, 320, lit_col=60, lit_width=40)
        for _ in range(2):
            self.assertIsNone(node.process_cycle(frame))
        msg = node.process_cycle(frame)
        self.assertIsNotNone(msg)
        self.assertEqual(msg.value, "left")
        self.assertEqual(msg.source, "arrow_signal")

    def test_confirmed_value_keeps_streaming_with_same_event_id(self):
        """確定後は毎周期同じ`(value, event_id)`を流し続ける（繰り返し契約）。"""
        node = ArrowSignalNode(confirm_frames=2)
        frame = _frame_with_arrow(240, 320, lit_col=60, lit_width=40)
        node.process_cycle(frame)
        first = node.process_cycle(frame)
        second = node.process_cycle(frame)
        self.assertEqual(first.event_id, second.event_id)
        self.assertEqual(first.value, second.value)

    def test_flicker_between_left_and_right_does_not_confirm(self):
        """左右が単発ずつ交互に来るだけでは、どちらも確定しない。"""
        node = ArrowSignalNode(confirm_frames=3)
        left = _frame_with_arrow(240, 320, lit_col=60, lit_width=40)
        right = _frame_with_arrow(240, 320, lit_col=260, lit_width=40)
        for frame in (left, right, left, right):
            self.assertIsNone(node.process_cycle(frame))

    def test_confirmed_left_then_confirmed_right_bumps_event_id(self):
        node = ArrowSignalNode(confirm_frames=2)
        left = _frame_with_arrow(240, 320, lit_col=60, lit_width=40)
        right = _frame_with_arrow(240, 320, lit_col=260, lit_width=40)
        node.process_cycle(left)
        left_msg = node.process_cycle(left)
        node.process_cycle(right)
        right_msg = node.process_cycle(right)
        self.assertEqual(left_msg.value, "left")
        self.assertEqual(right_msg.value, "right")
        self.assertNotEqual(left_msg.event_id, right_msg.event_id)

    def test_none_frame_keeps_last_confirmed_value(self):
        """フレームが読めない周期（DISARM等）でも、確定済みの値は流し続ける。"""
        node = ArrowSignalNode(confirm_frames=2)
        frame = _frame_with_arrow(240, 320, lit_col=60, lit_width=40)
        node.process_cycle(frame)
        confirmed = node.process_cycle(frame)
        self.assertIsNotNone(confirmed)
        still = node.process_cycle(None)
        self.assertEqual(still.value, confirmed.value)
        self.assertEqual(still.event_id, confirmed.event_id)

    def test_reset_clears_confirmed_state(self):
        node = ArrowSignalNode(confirm_frames=2)
        frame = _frame_with_arrow(240, 320, lit_col=60, lit_width=40)
        node.process_cycle(frame)
        node.process_cycle(frame)
        node.reset()
        self.assertIsNone(node.process_cycle(None))


class _FakeSub:
    def __init__(self, latest=None):
        self.latest = latest or {}

    def poll(self, timeout_ms):
        return []


class _FakePub:
    def __init__(self):
        self.sent = []

    def send(self, topic, msg):
        self.sent.append((topic, msg))


class TestArmedGating(unittest.TestCase):
    """DISARM中は共有メモリすら読まない（`cam_track_node.py`と同じ節電）。"""

    def _ref_with_frame(self, ring_name: str, *, lit_col: int):
        frame = _frame_with_arrow(240, 320, lit_col=lit_col, lit_width=40)
        ring = FrameRing.create(ring_name, 320, 240, "BGR888", n_slots=2)
        desc = ring.write(frame, t_capture_ns=1, frame_id=1)
        ref = ImageRef(shm_name=ring.name, slot=desc.slot, ring_seq=desc.seq,
                       frame_id=desc.frame_id, width=desc.width, height=desc.height,
                       fmt=desc.fmt, stride=desc.stride, nbytes=desc.nbytes, cam="front")
        return ring, ref

    def test_disarmed_never_publishes_route_select(self):
        node = ArrowSignalNode(confirm_frames=1)
        ring, ref = self._ref_with_frame("surge_test_arrow_signal_disarm", lit_col=60)
        try:
            sub = _FakeSub({TOPIC_IMAGE_FRONT: ref, TOPIC_VEHICLE_STATE: VehicleState(armed=False)})
            pub = _FakePub()
            node.run(sub=sub, pub=pub, duration_s=0.05)
            routes = [m for topic, m in pub.sent if topic == TOPIC_ROUTE_SELECT]
            self.assertFalse(routes, "DISARM中なのにroute/selectが出ている")
        finally:
            node.close()
            ring.unlink()

    def test_armed_publishes_route_select_once_confirmed(self):
        node = ArrowSignalNode(confirm_frames=1)
        ring, ref = self._ref_with_frame("surge_test_arrow_signal_armed", lit_col=60)
        try:
            sub = _FakeSub({TOPIC_IMAGE_FRONT: ref, TOPIC_VEHICLE_STATE: VehicleState(armed=True)})
            pub = _FakePub()
            node.run(sub=sub, pub=pub, duration_s=0.05)
            routes = [m for topic, m in pub.sent if topic == TOPIC_ROUTE_SELECT]
            self.assertTrue(routes, "ARM中なのにroute/selectが一度も出ていない")
            self.assertEqual(routes[-1].value, "left")
        finally:
            node.close()
            ring.unlink()

    def test_heartbeat_is_published_under_signal_node_name(self):
        node = ArrowSignalNode(confirm_frames=1)
        ring, ref = self._ref_with_frame("surge_test_arrow_signal_hb", lit_col=60)
        try:
            sub = _FakeSub({TOPIC_IMAGE_FRONT: ref, TOPIC_VEHICLE_STATE: VehicleState(armed=True)})
            pub = _FakePub()
            node.run(sub=sub, pub=pub, duration_s=0.05)
            hbs = [m for topic, m in pub.sent if topic == TOPIC_HB_PREFIX + "signal"]
            self.assertTrue(hbs, "hb/signal が一度も publish されていない")
        finally:
            node.close()
            ring.unlink()


class TestSignalConfigGating(unittest.TestCase):
    """`SignalConfig`（GUIからのライブ設定）の反映とON/OFFトグル。"""

    def _ref_with_frame(self, ring_name: str, *, lit_col: int):
        frame = _frame_with_arrow(240, 320, lit_col=lit_col, lit_width=40)
        ring = FrameRing.create(ring_name, 320, 240, "BGR888", n_slots=2)
        desc = ring.write(frame, t_capture_ns=1, frame_id=1)
        ref = ImageRef(shm_name=ring.name, slot=desc.slot, ring_seq=desc.seq,
                       frame_id=desc.frame_id, width=desc.width, height=desc.height,
                       fmt=desc.fmt, stride=desc.stride, nbytes=desc.nbytes, cam="front")
        return ring, ref

    def test_enabled_false_never_reads_frame(self):
        """`enabled=False`の間はDISARM中と同じく共有メモリを読まない（CPU節電）。"""
        node = ArrowSignalNode(confirm_frames=1)
        calls = []
        node.read_frame = lambda ref: (calls.append(ref) or None)
        ring, ref = self._ref_with_frame("surge_test_arrow_sig_disabled", lit_col=60)
        try:
            sub = _FakeSub({TOPIC_IMAGE_FRONT: ref, TOPIC_VEHICLE_STATE: VehicleState(armed=True),
                            TOPIC_SIGNAL_CONFIG: SignalConfig(enabled=False)})
            pub = _FakePub()
            node.run(sub=sub, pub=pub, duration_s=0.05)
            self.assertFalse(calls, "enabled=False なのにフレームを読んでいる")
        finally:
            node.close()
            ring.unlink()

    def test_signal_config_thresholds_are_applied_to_detector(self):
        node = ArrowSignalNode(confirm_frames=1)
        ring, ref = self._ref_with_frame("surge_test_arrow_signal_cfg", lit_col=60)
        try:
            cfg = SignalConfig(enabled=True, roi_top=0.1, roi_bottom=0.3,
                               sat_min=50, sat_max=190, val_min=160, min_lit_frac=0.05)
            sub = _FakeSub({TOPIC_IMAGE_FRONT: ref, TOPIC_VEHICLE_STATE: VehicleState(armed=True),
                            TOPIC_SIGNAL_CONFIG: cfg})
            pub = _FakePub()
            node.run(sub=sub, pub=pub, duration_s=0.02)
            self.assertEqual(node.detector.roi_band, (0.1, 0.3))
            self.assertEqual(node.detector.sat_range, (50, 190))
            self.assertEqual(node.detector.val_min, 160)
            self.assertEqual(node.detector.min_lit_frac, 0.05)
        finally:
            node.close()
            ring.unlink()

    def test_missing_signal_config_defaults_to_enabled(self):
        """`SignalConfig`が一度も届いていない起動直後もON側に倒す（既存の挙動を維持）。"""
        node = ArrowSignalNode(confirm_frames=1)
        ring, ref = self._ref_with_frame("surge_test_arrow_signal_no_cfg", lit_col=60)
        try:
            sub = _FakeSub({TOPIC_IMAGE_FRONT: ref, TOPIC_VEHICLE_STATE: VehicleState(armed=True)})
            pub = _FakePub()
            node.run(sub=sub, pub=pub, duration_s=0.05)
            routes = [m for topic, m in pub.sent if topic == TOPIC_ROUTE_SELECT]
            self.assertTrue(routes, "SignalConfig未着でも認識は動くはず")
        finally:
            node.close()
            ring.unlink()


class TestArrowSignalStatusPublish(unittest.TestCase):
    """`signal/status`（GUIのライブチューニング・映像オーバーレイ向け）の内容。"""

    def _ref_with_frame(self, ring_name: str, *, lit_col: int):
        frame = _frame_with_arrow(240, 320, lit_col=lit_col, lit_width=40)
        ring = FrameRing.create(ring_name, 320, 240, "BGR888", n_slots=2)
        desc = ring.write(frame, t_capture_ns=1, frame_id=1)
        ref = ImageRef(shm_name=ring.name, slot=desc.slot, ring_seq=desc.seq,
                       frame_id=desc.frame_id, width=desc.width, height=desc.height,
                       fmt=desc.fmt, stride=desc.stride, nbytes=desc.nbytes, cam="front")
        return ring, ref

    def test_status_reflects_raw_and_confirmed_value(self):
        node = ArrowSignalNode(confirm_frames=1)
        ring, ref = self._ref_with_frame("surge_test_arrow_signal_status", lit_col=60)
        try:
            sub = _FakeSub({TOPIC_IMAGE_FRONT: ref, TOPIC_VEHICLE_STATE: VehicleState(armed=True)})
            pub = _FakePub()
            node.run(sub=sub, pub=pub, duration_s=0.02)
            last = [m for topic, m in pub.sent if topic == TOPIC_SIGNAL_STATUS][-1]
            self.assertEqual(last.value, "left")
            self.assertGreater(last.lit_frac, 0.0)
            self.assertEqual(last.confirmed_value, "left")
            self.assertTrue(last.enabled)
            self.assertEqual((last.roi_top, last.roi_bottom), node.detector.roi_band)
        finally:
            node.close()
            ring.unlink()

    def test_status_shows_not_yet_confirmed_before_confirm_frames(self):
        """`process_cycle()`を直接叩けば、confirm前は`_confirmed_value`が空のまま
        （`run()`のビジーループはタイミングに依存するため、ここは直接呼ぶ）。"""
        node = ArrowSignalNode(confirm_frames=3)
        frame = _frame_with_arrow(240, 320, lit_col=60, lit_width=40)
        node.process_cycle(frame)
        self.assertIsNone(node._confirmed_value)
        self.assertEqual(node.last_result.value, "left")
        self.assertGreater(node.last_result.lit_frac, 0.0)

    def test_status_shows_disabled_when_signal_config_off(self):
        node = ArrowSignalNode(confirm_frames=1)
        ring, ref = self._ref_with_frame("surge_test_arrow_sig_stat_off", lit_col=60)
        try:
            sub = _FakeSub({TOPIC_IMAGE_FRONT: ref, TOPIC_VEHICLE_STATE: VehicleState(armed=True),
                            TOPIC_SIGNAL_CONFIG: SignalConfig(enabled=False)})
            pub = _FakePub()
            node.run(sub=sub, pub=pub, duration_s=0.02)
            statuses = [m for topic, m in pub.sent if topic == TOPIC_SIGNAL_STATUS]
            self.assertTrue(statuses)
            self.assertFalse(statuses[-1].enabled)
            self.assertEqual(statuses[-1].value, "")
        finally:
            node.close()
            ring.unlink()


class TestRunSurvivesProcessCycleException(unittest.TestCase):
    def test_exception_does_not_propagate(self):
        node = ArrowSignalNode(confirm_frames=1)

        def _raise(*a, **kw):
            raise RuntimeError("認識側のバグ")
        node.process_cycle = _raise

        ring = FrameRing.create("surge_test_arrow_signal_exc", 320, 240, "BGR888", n_slots=2)
        try:
            frame = _frame_with_arrow(240, 320, lit_col=60, lit_width=40)
            desc = ring.write(frame, t_capture_ns=1, frame_id=1)
            ref = ImageRef(shm_name=ring.name, slot=desc.slot, ring_seq=desc.seq,
                           frame_id=desc.frame_id, width=desc.width, height=desc.height,
                           fmt=desc.fmt, stride=desc.stride, nbytes=desc.nbytes, cam="front")
            sub = _FakeSub({TOPIC_IMAGE_FRONT: ref, TOPIC_VEHICLE_STATE: VehicleState(armed=True)})
            pub = _FakePub()
            node.run(sub=sub, pub=pub, duration_s=0.02)  # 例外を外に伝播させないこと
        finally:
            node.close()
            ring.unlink()


if __name__ == "__main__":
    unittest.main()
