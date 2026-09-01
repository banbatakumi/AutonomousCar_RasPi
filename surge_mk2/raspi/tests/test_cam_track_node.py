"""`raspi/nodes/cam_track_node.py` の配線・幾何計算テスト。

**実NanoTrackモデルは要らない。** `TargetTrackerFactory`と同じインタフェース
（`available`/`new_instance()`）を持つ偽トラッカーに差し替え、
bbox→bearing変換・LiDAR距離融合・IDLE/TRACKING状態遷移という配管が
壊れずに流れることだけを確認する（`test_cam_perception_node.py`の
ダミーONNXモデルと同じ考え方——推論の精度そのものは実データが要る領域）。
"""

import math
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

import numpy as np  # noqa: E402

from raspi.core.vehicle import Vehicle  # noqa: E402
from raspi.msgs.types import Scan, TargetRoiCtrl  # noqa: E402
from raspi.nodes.cam_track_node import CamTrackNode  # noqa: E402


def _blank_frame(w: int, h: int) -> np.ndarray:
    return np.zeros((h, w, 3), dtype=np.uint8)


def _vehicle_straight() -> Vehicle:
    """カメラが車体正面を向いている（yaw=0）既定値のVehicle。"""
    return Vehicle()


class _FakeTracker:
    """`box_sequence`を`update()`のたびに順番に返す偽トラッカー。

    `None`が来たら追跡失敗（`update()`が`False`を返す＝見失い）を模す。
    最後の要素に達したら以降はそれを返し続ける。
    """

    def __init__(self, box_sequence: list) -> None:
        self._boxes = list(box_sequence)
        self._i = 0
        self.inited_with = None

    def init(self, frame: np.ndarray, box_px: tuple) -> None:
        self.inited_with = box_px

    def update(self, frame: np.ndarray):
        box = self._boxes[min(self._i, len(self._boxes) - 1)] if self._boxes else None
        self._i += 1
        if box is None:
            return False, (0.0, 0.0, 0.0, 0.0)
        return True, box

    def score(self) -> float:
        return 0.9


class _FakeFactory:
    """`TargetTrackerFactory`と同じインタフェース（`available`/`new_instance()`）。"""

    def __init__(self, box_sequence: list, *, available: bool = True) -> None:
        self._box_sequence = box_sequence
        self.available = available
        self.error = None if available else "テスト用: モデル未配置"

    def new_instance(self):
        if not self.available:
            return None
        return _FakeTracker(self._box_sequence)


class TestBearingFromPixel(unittest.TestCase):
    """画素→方位角変換を、既知の内部パラメータ（`yaw=0`の既定Vehicle）で検証する。"""

    def test_center_pixel_is_zero_bearing(self):
        node = CamTrackNode(vehicle=_vehicle_straight(), factory=_FakeFactory([]))
        frame = _blank_frame(640, 480)
        roi = TargetRoiCtrl(x0=0.45, y0=0.4, x1=0.55, y1=0.6, select_seq=1)
        t = node.process_cycle(frame, roi=roi, scan=None, now_ns=0)
        self.assertTrue(t.tracking)
        self.assertAlmostEqual(t.bearing, 0.0, places=6)

    def test_left_of_center_is_positive_bearing(self):
        """画面左（u < cx）＝反時計回り正（左）の方位になる。"""
        node = CamTrackNode(vehicle=_vehicle_straight(), factory=_FakeFactory([]))
        frame = _blank_frame(640, 480)
        roi = TargetRoiCtrl(x0=0.0, y0=0.4, x1=0.2, y1=0.6, select_seq=1)
        t = node.process_cycle(frame, roi=roi, scan=None, now_ns=0)
        self.assertGreater(t.bearing, 0.0)

    def test_right_of_center_is_negative_bearing(self):
        node = CamTrackNode(vehicle=_vehicle_straight(), factory=_FakeFactory([]))
        frame = _blank_frame(640, 480)
        roi = TargetRoiCtrl(x0=0.8, y0=0.4, x1=1.0, y1=0.6, select_seq=1)
        t = node.process_cycle(frame, roi=roi, scan=None, now_ns=0)
        self.assertLess(t.bearing, 0.0)


class TestStateTransitions(unittest.TestCase):
    def test_idle_until_selected(self):
        node = CamTrackNode(vehicle=_vehicle_straight(), factory=_FakeFactory([]))
        t = node.process_cycle(_blank_frame(640, 480), roi=None, scan=None, now_ns=0)
        self.assertFalse(t.tracking)

    def test_select_then_clear_returns_to_idle(self):
        node = CamTrackNode(vehicle=_vehicle_straight(),
                            factory=_FakeFactory([(100, 100, 50, 50)]))
        frame = _blank_frame(640, 480)
        roi = TargetRoiCtrl(x0=0.3, y0=0.3, x1=0.5, y1=0.5, select_seq=1)
        t = node.process_cycle(frame, roi=roi, scan=None, now_ns=0)
        self.assertTrue(t.tracking)

        cleared = TargetRoiCtrl(x0=0.3, y0=0.3, x1=0.5, y1=0.5, select_seq=1, clear_seq=1)
        t = node.process_cycle(frame, roi=cleared, scan=None, now_ns=1_000_000)
        self.assertFalse(t.tracking)

    def test_too_small_roi_is_rejected_and_seq_not_consumed(self):
        node = CamTrackNode(vehicle=_vehicle_straight(),
                            factory=_FakeFactory([(100, 100, 50, 50)]))
        frame = _blank_frame(640, 480)
        # 640x480に対し、0.1%四方は数px四方 → 最小ROIサイズ(4px)を割る
        roi = TargetRoiCtrl(x0=0.50, y0=0.50, x1=0.501, y1=0.501, select_seq=1)
        t = node.process_cycle(frame, roi=roi, scan=None, now_ns=0)
        self.assertFalse(t.tracking)
        self.assertEqual(node._last_select_seq, 0)          # seqを消費していない

    def test_model_unavailable_is_ignored_and_seq_not_consumed(self):
        node = CamTrackNode(vehicle=_vehicle_straight(),
                            factory=_FakeFactory([], available=False))
        frame = _blank_frame(640, 480)
        roi = TargetRoiCtrl(x0=0.3, y0=0.3, x1=0.5, y1=0.5, select_seq=1)
        t = node.process_cycle(frame, roi=roi, scan=None, now_ns=0)
        self.assertFalse(t.tracking)
        self.assertEqual(node._last_select_seq, 0)

    def test_single_update_failure_is_absorbed_as_flicker(self):
        """★ちらつき吸収（Phase 2）。1回だけの`update()`失敗では`lost`にしない
        （`_LOST_DEBOUNCE_FRAMES`回連続するまで様子を見る）。"""
        node = CamTrackNode(vehicle=_vehicle_straight(),
                            factory=_FakeFactory([None, (100, 100, 50, 50)]))
        frame = _blank_frame(640, 480)
        roi = TargetRoiCtrl(x0=0.3, y0=0.3, x1=0.5, y1=0.5, select_seq=1)
        node.process_cycle(frame, roi=roi, scan=None, now_ns=0)              # 選択

        t1 = node.process_cycle(frame, roi=roi, scan=None, now_ns=20_000_000)  # 1回失敗
        self.assertFalse(t1.lost)

        t2 = node.process_cycle(frame, roi=roi, scan=None, now_ns=40_000_000)  # 次は成功
        self.assertFalse(t2.lost)

    def test_update_failure_marks_lost_after_debounce_and_holds_bearing(self):
        node = CamTrackNode(vehicle=_vehicle_straight(), factory=_FakeFactory([None]))
        frame = _blank_frame(640, 480)
        roi = TargetRoiCtrl(x0=0.3, y0=0.3, x1=0.5, y1=0.5, select_seq=1)
        t1 = node.process_cycle(frame, roi=roi, scan=None, now_ns=0)
        self.assertFalse(t1.lost)
        bearing_before = t1.bearing

        # 1回目の失敗はまだ`lost`にしない（ちらつき吸収）
        t2 = node.process_cycle(frame, roi=roi, scan=None, now_ns=20_000_000)
        self.assertFalse(t2.lost)
        self.assertAlmostEqual(t2.bearing, bearing_before, places=9)

        # `_LOST_DEBOUNCE_FRAMES`回連続で失敗して初めて`lost=True`。
        # そのフレームは`lost_ms=0`（今まさに見失い始めた）
        t3 = node.process_cycle(frame, roi=roi, scan=None, now_ns=100_000_000)
        self.assertTrue(t3.tracking)
        self.assertTrue(t3.lost)
        self.assertAlmostEqual(t3.bearing, bearing_before, places=9)
        self.assertEqual(t3.lost_ms, 0.0)

        t4 = node.process_cycle(frame, roi=roi, scan=None, now_ns=250_000_000)
        self.assertTrue(t4.lost)
        self.assertGreater(t4.lost_ms, 0.0)

    def test_no_frame_available_is_treated_as_lost_after_debounce(self):
        """共有メモリがまだ読めない周期（`frame=None`）も見失い扱いにする
        （ちらつき吸収のデバウンスは掛かる）。"""
        node = CamTrackNode(vehicle=_vehicle_straight(),
                            factory=_FakeFactory([(100, 100, 50, 50)]))
        frame = _blank_frame(640, 480)
        roi = TargetRoiCtrl(x0=0.3, y0=0.3, x1=0.5, y1=0.5, select_seq=1)
        node.process_cycle(frame, roi=roi, scan=None, now_ns=0)

        node.process_cycle(None, roi=roi, scan=None, now_ns=20_000_000)
        t = node.process_cycle(None, roi=roi, scan=None, now_ns=50_000_000)
        self.assertTrue(t.tracking)
        self.assertTrue(t.lost)

    def test_gives_up_and_returns_to_idle_after_long_loss(self):
        """★見失いを諦める。NanoTrackは狭い探索範囲しか見ない「追跡」であって
        「検出」ではないため、長時間見失った後は`IDLE`へ戻し選び直しを要求する
        （別の物体を誤って掴み直すリスクを避ける）。"""
        node = CamTrackNode(vehicle=_vehicle_straight(), factory=_FakeFactory([None]))
        frame = _blank_frame(640, 480)
        roi = TargetRoiCtrl(x0=0.3, y0=0.3, x1=0.5, y1=0.5, select_seq=1)
        node.process_cycle(frame, roi=roi, scan=None, now_ns=0)             # 選択
        node.process_cycle(frame, roi=roi, scan=None, now_ns=20_000_000)    # 1回目失敗
        node.process_cycle(frame, roi=roi, scan=None, now_ns=40_000_000)    # 2回目失敗→lost開始

        # まだ諦める時間には達していない
        t = node.process_cycle(frame, roi=roi, scan=None, now_ns=1_000_000_000)
        self.assertTrue(t.tracking)
        self.assertTrue(t.lost)

        # 諦める時間を超えたら`tracking=False`に戻る
        t = node.process_cycle(frame, roi=roi, scan=None, now_ns=6_000_000_000)
        self.assertFalse(t.tracking)

        # 諦めた後、同じ`select_seq`を送り続けても再選択にはならない
        # （`clear_seq`もしくは新しい`select_seq`が要る）
        t = node.process_cycle(frame, roi=roi, scan=None, now_ns=6_100_000_000)
        self.assertFalse(t.tracking)

    def test_reacquires_if_target_returns_before_give_up(self):
        """諦める前に対象が戻ってくれば、追跡は普通に再開する
        （`_LOST_DEBOUNCE_FRAMES`と同じ`_lost_since_ns`のクリアで足りる）。"""
        node = CamTrackNode(vehicle=_vehicle_straight(),
                            factory=_FakeFactory([None, None, None, (100, 100, 50, 50)]))
        frame = _blank_frame(640, 480)
        roi = TargetRoiCtrl(x0=0.3, y0=0.3, x1=0.5, y1=0.5, select_seq=1)
        node.process_cycle(frame, roi=roi, scan=None, now_ns=0)
        node.process_cycle(frame, roi=roi, scan=None, now_ns=20_000_000)
        node.process_cycle(frame, roi=roi, scan=None, now_ns=40_000_000)

        t = node.process_cycle(frame, roi=roi, scan=None, now_ns=1_000_000_000)
        self.assertTrue(t.lost)

        t = node.process_cycle(frame, roi=roi, scan=None, now_ns=1_020_000_000)
        self.assertTrue(t.tracking)
        self.assertFalse(t.lost)


class TestLidarFusion(unittest.TestCase):
    @staticmethod
    def _scan_with_point_at(deg: int, dist: float, t_pub_ns: int) -> Scan:
        s = Scan(dist=[0.0] * 360, sector_seen=[True] * 12)
        s.dist[deg % 360] = dist
        s.t_pub = t_pub_ns
        return s

    def _node_and_bearing_deg(self):
        # フェイクトラッカーのbboxもROI選択も中心(cx=320,cy=240)に揃えてあるので、
        # 選択直後・追跡中のどちらでもbearing=0（deg=0）になる
        node = CamTrackNode(vehicle=_vehicle_straight(),
                            factory=_FakeFactory([(300, 210, 40, 60)]))
        frame = _blank_frame(640, 480)
        roi = TargetRoiCtrl(x0=0.45, y0=0.4, x1=0.55, y1=0.6, select_seq=1)
        t = node.process_cycle(frame, roi=roi, scan=None, now_ns=0)
        deg = int(round(math.degrees(t.bearing))) % 360
        return node, frame, roi, deg

    def test_distance_from_matching_bearing_sector(self):
        node, frame, roi, deg = self._node_and_bearing_deg()
        scan = self._scan_with_point_at(deg, 1.5, t_pub_ns=1_000_000)
        t2 = node.process_cycle(frame, roi=roi, scan=scan, now_ns=1_000_000)
        self.assertTrue(t2.distance_valid)
        self.assertAlmostEqual(t2.distance, 1.5, places=6)

    def test_stale_scan_is_not_trusted(self):
        node, frame, roi, deg = self._node_and_bearing_deg()
        scan = self._scan_with_point_at(deg, 1.5, t_pub_ns=0)
        t2 = node.process_cycle(frame, roi=roi, scan=scan, now_ns=1_000_000_000)  # 1s後
        self.assertFalse(t2.distance_valid)

    def test_no_measured_point_in_window_is_invalid(self):
        node, frame, roi, deg = self._node_and_bearing_deg()
        scan = Scan(dist=[0.0] * 360, sector_seen=[True] * 12, t_pub=1_000_000)  # 全欠測
        t2 = node.process_cycle(frame, roi=roi, scan=scan, now_ns=1_000_000)
        self.assertFalse(t2.distance_valid)

    def test_far_jump_is_gated_until_confirmed(self):
        """近づく方向は即採用、遠のく方向への大きな飛びは連続一致するまで保留する
        （「距離を近づける方向にしか誤らない」安全側のゲート）。"""
        node, frame, roi, deg = self._node_and_bearing_deg()
        now = 0

        def _step(dist: float) -> float:
            nonlocal now
            now += 1_000_000
            scan = self._scan_with_point_at(deg, dist, t_pub_ns=now)
            return node.process_cycle(frame, roi=roi, scan=scan, now_ns=now).distance

        self.assertAlmostEqual(_step(1.0), 1.0, places=6)      # 最初の値は即採用
        self.assertAlmostEqual(_step(1.0), 1.0, places=6)      # 変化なしも即採用

        # 1.0m→3.0m へ大きく飛ぶ。**直後には採用されず、直前の近い値のまま**
        self.assertAlmostEqual(_step(3.0), 1.0, places=6)

        # 十分な回数同じ値が続けば、ようやく採用される
        got = 1.0
        for _ in range(5):
            got = _step(3.0)
        self.assertAlmostEqual(got, 3.0, places=6)


if __name__ == "__main__":
    unittest.main()
