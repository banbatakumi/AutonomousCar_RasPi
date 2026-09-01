"""`raspi/auto/follow_object.py`（`FollowObject.plan()`）の単体テスト。

**ROI未選択/見失い/距離不明/LiDAR欠測との重なり**という、`Planner`の約束3
「走ってよいか分からないときは`ready=False`」を各分岐で守れているかを確認する。
"""

import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from raspi.auto.follow_object import FollowObject  # noqa: E402
from raspi.msgs.types import TargetTrack  # noqa: E402


class TestNotSelected(unittest.TestCase):
    def test_not_tracking_is_not_ready(self):
        fo = FollowObject()
        p = fo.merged({})
        st = fo.plan(TargetTrack(tracking=False), None, p, 0.02)
        self.assertFalse(st.ready)
        self.assertIn("選択", st.reason)


class TestDistanceControl(unittest.TestCase):
    def test_far_target_accelerates_toward_max_speed(self):
        fo = FollowObject()
        p = fo.merged({})
        track = TargetTrack(tracking=True, lost=False, bearing=0.0,
                            distance=5.0, distance_valid=True)
        st = None
        for _ in range(50):                       # 舵・速度の1次遅れが収束するまで回す
            st = fo.plan(track, None, p, 0.02)
        self.assertTrue(st.ready)
        self.assertFalse(st.brake)
        self.assertAlmostEqual(st.target_speed, p["max_speed"], places=2)

    def test_stop_distance_brakes_without_reverse(self):
        fo = FollowObject()
        p = fo.merged({})
        track = TargetTrack(tracking=True, lost=False, bearing=0.0,
                            distance=p["stop_distance"] * 0.5, distance_valid=True)
        st = fo.plan(track, None, p, 0.02)
        self.assertTrue(st.ready)
        self.assertTrue(st.brake)
        self.assertEqual(st.target_speed, 0.0)

    def test_speed_never_goes_negative_even_when_too_close_but_not_stopped(self):
        """`stop_distance`より遠いが`follow_distance`より近い場合、
        比例制御の出力は負になり得るが**後退はしない**（0でクランプ）。"""
        fo = FollowObject()
        p = fo.merged({})
        near = p["stop_distance"] + 0.01
        self.assertLess(near, p["follow_distance"])
        track = TargetTrack(tracking=True, lost=False, bearing=0.0,
                            distance=near, distance_valid=True)
        st = fo.plan(track, None, p, 0.02)
        self.assertGreaterEqual(st.target_speed, 0.0)
        self.assertFalse(st.brake)

    def test_bearing_is_reflected_in_steer_sign(self):
        """`bearing`（車両座標・反時計回り正）と同じ向きに舵が切れる。"""
        fo = FollowObject()
        p = fo.merged({})
        track = TargetTrack(tracking=True, lost=False, bearing=0.3,
                            distance=2.0, distance_valid=True)
        st = None
        for _ in range(50):
            st = fo.plan(track, None, p, 0.02)
        self.assertGreater(st.target_steer, 0.0)


class TestLostHandling(unittest.TestCase):
    def test_brief_lost_holds_steer_and_decays_speed(self):
        fo = FollowObject()
        p = fo.merged({})
        moving = TargetTrack(tracking=True, lost=False, bearing=0.2,
                            distance=2.0, distance_valid=True)
        st = None
        for _ in range(50):
            st = fo.plan(moving, None, p, 0.02)
        steer_while_moving = st.target_steer
        speed_while_moving = st.target_speed
        self.assertGreater(speed_while_moving, 0.0)

        lost = TargetTrack(tracking=True, lost=True, lost_ms=50.0, bearing=0.2)
        st = fo.plan(lost, None, p, 0.02)
        self.assertTrue(st.ready)
        self.assertFalse(st.brake)
        self.assertEqual(st.target_steer, steer_while_moving)   # 舵は保持
        self.assertLess(st.target_speed, speed_while_moving)    # 速度は減衰し始める
        self.assertGreater(st.target_speed, 0.0)

    def test_lost_timeout_stops(self):
        fo = FollowObject()
        p = fo.merged({})
        timeout_ms = p["lost_timeout_s"] * 1000.0
        lost = TargetTrack(tracking=True, lost=True, lost_ms=timeout_ms + 1.0, bearing=0.0)
        st = fo.plan(lost, None, p, 0.02)
        self.assertTrue(st.ready)
        self.assertTrue(st.brake)
        self.assertEqual(st.target_speed, 0.0)

    def test_reacquiring_after_lost_resumes_control(self):
        fo = FollowObject()
        p = fo.merged({})
        lost = TargetTrack(tracking=True, lost=True, lost_ms=100.0, bearing=0.0)
        fo.plan(lost, None, p, 0.02)

        reacquired = TargetTrack(tracking=True, lost=False, bearing=0.0,
                                 distance=3.0, distance_valid=True)
        st = fo.plan(reacquired, None, p, 0.02)
        self.assertTrue(st.ready)
        self.assertFalse(st.brake)


class TestDistanceInvalidHandling(unittest.TestCase):
    def test_brief_distance_invalid_keeps_last_valid_distance(self):
        """LiDAR欠測との重なり: 1周期だけ`distance_valid=False`でも、
        直前の有効値を使って走り続ける（`distance_invalid_timeout_s`未満）。"""
        fo = FollowObject()
        p = fo.merged({})
        valid = TargetTrack(tracking=True, lost=False, bearing=0.0,
                            distance=2.0, distance_valid=True)
        for _ in range(30):
            fo.plan(valid, None, p, 0.02)

        invalid = TargetTrack(tracking=True, lost=False, bearing=0.0,
                              distance=0.0, distance_valid=False)
        st = fo.plan(invalid, None, p, 0.02)
        self.assertTrue(st.ready)
        self.assertFalse(st.brake)
        self.assertAlmostEqual(st.target_distance, 2.0, places=6)

    def test_distance_invalid_timeout_stops(self):
        fo = FollowObject()
        p = fo.merged({})
        valid = TargetTrack(tracking=True, lost=False, bearing=0.0,
                            distance=2.0, distance_valid=True)
        fo.plan(valid, None, p, 0.02)

        invalid = TargetTrack(tracking=True, lost=False, bearing=0.0,
                              distance=0.0, distance_valid=False)
        timeout_s = p["distance_invalid_timeout_s"]
        st = None
        elapsed = 0.0
        dt = 0.05
        while elapsed < timeout_s + 0.2:
            st = fo.plan(invalid, None, p, dt)
            elapsed += dt
        self.assertTrue(st.ready)
        self.assertTrue(st.brake)
        self.assertEqual(st.target_speed, 0.0)

    def test_distance_never_valid_yet_is_not_ready(self):
        """一度も距離が取れていない（方位だけ分かっている）間は、停止条件を
        誤って報告せず、単に「まだ計測できていない」として`ready=False`にする。"""
        fo = FollowObject()
        p = fo.merged({})
        invalid = TargetTrack(tracking=True, lost=False, bearing=0.0,
                              distance=0.0, distance_valid=False)
        st = fo.plan(invalid, None, p, 0.02)
        self.assertFalse(st.ready)
        self.assertFalse(st.brake)


class TestReset(unittest.TestCase):
    def test_reset_clears_internal_state(self):
        fo = FollowObject()
        p = fo.merged({})
        moving = TargetTrack(tracking=True, lost=False, bearing=0.2,
                            distance=2.0, distance_valid=True)
        for _ in range(50):
            fo.plan(moving, None, p, 0.02)
        self.assertNotEqual(fo._steer, 0.0)

        fo.reset()
        self.assertEqual(fo._steer, 0.0)
        self.assertEqual(fo._speed, 0.0)
        self.assertIsNone(fo._last_valid_distance)

    def test_deselecting_resets_state_so_next_target_starts_clean(self):
        fo = FollowObject()
        p = fo.merged({})
        moving = TargetTrack(tracking=True, lost=False, bearing=0.2,
                            distance=2.0, distance_valid=True)
        for _ in range(50):
            fo.plan(moving, None, p, 0.02)

        fo.plan(TargetTrack(tracking=False), None, p, 0.02)
        self.assertEqual(fo._steer, 0.0)
        self.assertEqual(fo._speed, 0.0)


if __name__ == "__main__":
    unittest.main()
