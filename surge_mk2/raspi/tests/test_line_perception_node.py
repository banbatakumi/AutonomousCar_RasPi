"""`raspi/nodes/line_perception_node.py` のテスト。

**実カメラは要らない。** 合成フレーム（白い帯を描いただけの画像）で、
色しきい値→帯の重心→IPM逆投影→`LineScan`化という配管が壊れずに流れることと、
白線の位置が動けば目標点の左右も動くことを確認する
（`test_cam_perception_node.py` と同じ「配管のテスト」の流儀）。
"""

import sys
import time
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

import numpy as np  # noqa: E402

from raspi.bus import FrameRing  # noqa: E402
from raspi.core.vehicle import Vehicle  # noqa: E402
from raspi.msgs import AutoCtrl, ImageRef, VehicleState  # noqa: E402
from raspi.msgs.types import (  # noqa: E402
    TOPIC_AUTO_CTRL,
    TOPIC_IMAGE_FRONT,
    TOPIC_LINE_CAM,
    TOPIC_VEHICLE_STATE,
)
from raspi.nav.ipm import vehicle_camera_intrinsics  # noqa: E402
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


def _white_mask_reference(frame: np.ndarray, *, min_brightness: int = 170,
                          max_chroma: int = 40) -> np.ndarray:
    """`white_mask()`の旧実装（int16へ広げてaxis=-1のmin/maxを取る素朴な形）。

    issue #14の高速化（uint8のままチャネル別にmin/maxを取る）が数値的に
    完全に一致することを確認するためだけの参照実装。ここだけに残す
    （本体には置かない——旧実装を本体コードとして生かさない）。
    """
    f = frame[..., :3].astype(np.int16)
    lo = f.min(axis=-1)
    hi = f.max(axis=-1)
    return (lo >= min_brightness) & ((hi - lo) <= max_chroma)


class TestWhiteMaskMatchesReferenceImplementation(unittest.TestCase):
    """issue #14: uint8のまま計算する高速版が、旧int16実装と完全一致すること。

    **この一致確認は必須**（高速化のつもりで近似値になっていないかを保証する）。
    乱数フレーム・境界値（明るさ/彩度の閾値ちょうど）の両方で確認する。
    """

    def test_matches_reference_on_random_frames(self):
        rng = np.random.default_rng(12345)
        for _ in range(20):
            frame = rng.integers(0, 256, size=(37, 53, 3), dtype=np.uint8)
            got = white_mask(frame)
            want = _white_mask_reference(frame)
            self.assertTrue(np.array_equal(got, want),
                            "uint8版とint16参照実装の結果が一致しない")

    def test_matches_reference_on_custom_thresholds(self):
        rng = np.random.default_rng(999)
        frame = rng.integers(0, 256, size=(20, 20, 3), dtype=np.uint8)
        got = white_mask(frame, min_brightness=100, max_chroma=10)
        want = _white_mask_reference(frame, min_brightness=100, max_chroma=10)
        self.assertTrue(np.array_equal(got, want))

    def test_matches_reference_on_boundary_values(self):
        """明るさ・彩度がちょうど閾値の境界にある画素（off-by-one検出用）。"""
        frame = np.array([[[170, 170, 170],     # ちょうどmin_brightness、彩度0
                           [169, 169, 169],     # brightness未満
                           [200, 200, 240],     # chroma=40ちょうど
                           [200, 200, 241],     # chroma=41（超過）
                           [255, 255, 255],     # 全白
                           [0, 0, 0]]],          # 全黒
                        dtype=np.uint8)
        got = white_mask(frame)
        want = _white_mask_reference(frame)
        self.assertTrue(np.array_equal(got, want))
        self.assertTrue(np.array_equal(got, [[True, False, True, False, True, False]]))


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


class TestBandsFromGroundDistance(unittest.TestCase):
    """帯は地面距離から換算する（`bottom_crop`・レンズが変わっても地面の行に当たる）。"""

    def test_bands_are_on_the_ground_and_ordered(self):
        node = LinePerceptionNode(vehicle=Vehicle.load())
        for w, h in ((640, 274), (320, 240)):
            intr = vehicle_camera_intrinsics(node.vehicle, "front", w, h)
            near, far = node._bands(w, h)
            horizon = intr.principal_y / h
            self.assertGreater(far[0], horizon, "遠方帯が地平線より上（空）に当たっている")
            self.assertLess(far[1], near[1])                # 遠方帯は近傍帯より奥（上）
            self.assertLessEqual(near[1], 1.0)              # 画像の外に出ない
            self.assertLess(near[0], near[1])

    def test_explicit_band_overrides_range(self):
        node = LinePerceptionNode(vehicle=Vehicle.load(), near_band=(0.1, 0.2))
        near, _ = node._bands(320, 240)
        self.assertEqual(near, (0.1, 0.2))


class TestLinePerceptionNodeProcessFrame(unittest.TestCase):
    def test_returns_line_scan_shaped_message(self):
        node = LinePerceptionNode(vehicle=Vehicle.load())
        frame = _frame_with_white_column(240, 320, col=160)
        st = node.process_frame(frame, seq=7)
        self.assertEqual(st.seq, 7)
        self.assertTrue(st.seen)

    def test_line_at_center_gives_near_zero_lateral_offset(self):
        node = LinePerceptionNode(vehicle=Vehicle.load())
        # 「正面」は画像中心ではなく主点（校正で中心から数 px ずれる）
        cx = vehicle_camera_intrinsics(node.vehicle, "front", 320, 240).cx
        frame = _frame_with_white_column(240, 320, col=round(cx))
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
        desc = ring.write(data, t_capture_ns=time.monotonic_ns(), frame_id=1)
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

    def test_disarmed_stays_idle_even_when_line_trace_selected(self):
        """`line_trace`が選ばれていてもDISARM中は認識を回さない（省電力バグ修正）。"""
        node = LinePerceptionNode(vehicle=Vehicle.load())
        ring, ref = self._ref_with_frame("surge_test_line_gating_disarm")
        try:
            sub = _FakeSub({TOPIC_IMAGE_FRONT: ref, TOPIC_AUTO_CTRL: AutoCtrl(mode="line_trace"),
                            TOPIC_VEHICLE_STATE: VehicleState(armed=False)})
            pub = _FakePub()
            node.run(sub=sub, pub=pub, duration_s=0.02)

            lines = [st for topic, st in pub.sent if topic == TOPIC_LINE_CAM]
            self.assertTrue(lines)
            self.assertFalse(lines[-1].seen, "DISARM中なのに認識結果が出ている")
        finally:
            node.close()
            ring.unlink()

    def test_armed_with_line_trace_selected_still_activates_recognition(self):
        """ARM中はモード選択だけで認識が回る（回帰防止）。"""
        node = LinePerceptionNode(vehicle=Vehicle.load())
        ring, ref = self._ref_with_frame("surge_test_line_gating_armed")
        try:
            sub = _FakeSub({TOPIC_IMAGE_FRONT: ref, TOPIC_AUTO_CTRL: AutoCtrl(mode="line_trace"),
                            TOPIC_VEHICLE_STATE: VehicleState(armed=True)})
            pub = _FakePub()
            node.run(sub=sub, pub=pub, duration_s=0.02)

            lines = [st for topic, st in pub.sent if topic == TOPIC_LINE_CAM]
            self.assertTrue(any(st.seen for st in lines),
                            "ARM中にline_traceが選ばれたのに認識結果が出ていない")
        finally:
            node.close()
            ring.unlink()


class TestSameFrameIsNotReprocessed(unittest.TestCase):
    """issue #13: `ImageRef.ring_seq`が変わっていない間は`process_frame()`を
    再実行しない（カメラ30fps・ループ約10ms周期で、同じフレームを約3回
    処理していた無駄の解消）。
    """

    def _ref_with_frame(self, ring_name: str):
        ring = FrameRing.create(ring_name, 320, 240, "RGB888", n_slots=2)
        data = _frame_with_white_column(240, 320, col=160)
        desc = ring.write(data, t_capture_ns=time.monotonic_ns(), frame_id=1)
        ref = ImageRef(shm_name=ring.name, slot=desc.slot, ring_seq=desc.seq,
                       frame_id=desc.frame_id, width=desc.width, height=desc.height,
                       fmt=desc.fmt, stride=desc.stride, nbytes=desc.nbytes, cam="front")
        return ring, ref

    def test_unchanged_ring_seq_calls_process_frame_once(self):
        node = LinePerceptionNode(vehicle=Vehicle.load())
        ring, ref = self._ref_with_frame("surge_test_line_dedup")
        calls = []
        orig = node.process_frame

        def _counting(*a, **kw):
            calls.append(1)
            return orig(*a, **kw)
        node.process_frame = _counting
        try:
            sub = _FakeSub({TOPIC_IMAGE_FRONT: ref, TOPIC_AUTO_CTRL: AutoCtrl(mode="line_trace")})
            pub = _FakePub()
            node.run(sub=sub, pub=pub, duration_s=0.08)   # poll=20msで複数周回る

            lines = [st for topic, st in pub.sent if topic == TOPIC_LINE_CAM]
            self.assertGreater(len(lines), 1, "複数周回っていない（テストの前提が崩れている）")
            self.assertEqual(len(calls), 1,
                             "同じring_seqのフレームなのにprocess_frame()が複数回呼ばれている")
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
            t_cap_written = time.monotonic_ns()
            desc = ring.write(data, t_capture_ns=t_cap_written, frame_id=1)
            ref = ImageRef(shm_name=ring.name, slot=desc.slot, ring_seq=desc.seq,
                           frame_id=desc.frame_id, width=desc.width, height=desc.height,
                           fmt=desc.fmt, stride=desc.stride, nbytes=desc.nbytes, cam="front")

            got = node.read_frame(ref)
            self.assertIsNotNone(got)
            arr, t_capture = got
            self.assertTrue(np.array_equal(arr, data))
            self.assertEqual(t_capture, t_cap_written)
        finally:
            node.close()
            ring.unlink()

    def test_missing_shm_returns_none_instead_of_raising(self):
        node = LinePerceptionNode(vehicle=Vehicle.load())
        ref = ImageRef(shm_name="surge_does_not_exist", ring_seq=1)
        self.assertIsNone(node.read_frame(ref))


class TestRunSurvivesProcessFrameException(unittest.TestCase):
    """`process_frame()` が例外を投げてもノードが継続すること。

    `planning_node._replan()` の `planner.plan()` 例外保護（B3）と同じパターンを
    `run()` 側の認識呼び出しにも横展開したもの——認識側のバグでノード全体を
    巻き込んで落とさず、契約の「見失った扱い」に自然に落とす。
    """

    def _ref_with_frame(self, ring_name: str):
        ring = FrameRing.create(ring_name, 320, 240, "RGB888", n_slots=2)
        data = _frame_with_white_column(240, 320, col=160)
        desc = ring.write(data, t_capture_ns=time.monotonic_ns(), frame_id=1)
        ref = ImageRef(shm_name=ring.name, slot=desc.slot, ring_seq=desc.seq,
                       frame_id=desc.frame_id, width=desc.width, height=desc.height,
                       fmt=desc.fmt, stride=desc.stride, nbytes=desc.nbytes, cam="front")
        return ring, ref

    def test_exception_does_not_propagate_and_falls_back_to_failed_frame(self):
        node = LinePerceptionNode(vehicle=Vehicle.load())

        def _raise(*a, **kw):
            raise RuntimeError("認識側のバグ")
        node.process_frame = _raise

        ring, ref = self._ref_with_frame("surge_test_line_perception_exc")
        try:
            sub = _FakeSub({TOPIC_IMAGE_FRONT: ref, TOPIC_AUTO_CTRL: AutoCtrl(mode="line_trace")})
            pub = _FakePub()
            node.run(sub=sub, pub=pub, duration_s=0.02)  # 例外を外に伝播させないこと

            lines = [st for topic, st in pub.sent if topic == TOPIC_LINE_CAM]
            self.assertTrue(lines, "line/cam が一度も publish されていない")
            self.assertFalse(lines[-1].seen,
                             "process_frame()が例外を投げたのに見失った扱いになっていない")
        finally:
            node.close()
            ring.unlink()


if __name__ == "__main__":
    unittest.main()
