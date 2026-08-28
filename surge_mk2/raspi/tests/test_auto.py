"""自動運転（`raspi/auto/`）と、自律指令が車に届く経路の安全条件のテスト。

**「走る」より「止まる」を厚く試す。** 合成した点群を流し込むだけなので、
ハードウェアもバスも要らない。
"""

import json
import math
import sys
import tempfile
import time
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

import msgspec  # noqa: E402
import numpy as np  # noqa: E402
import onnx  # noqa: E402
from onnx import TensorProto, helper  # noqa: E402

from raspi.auto import (PLANNERS, DisparityExtender, DisparityPursuit,  # noqa: E402
                        E2ELidar, FollowTheGap, FollowTheGapCam, LineTrace,
                        catalog, make_planner)
from raspi.auto.base import sector_of_deg  # noqa: E402
from raspi.auto.disparity_extender import _extend  # noqa: E402
from raspi.auto.gap_pursuit import _best_band  # noqa: E402
from raspi.msgs import AutoState, DriveCmd, LineScan, Scan, VehicleState  # noqa: E402
from raspi.msgs.types import TOPIC_LINE_CAM, TOPIC_SCAN, TOPIC_SCAN_CAM  # noqa: E402


def make_scan(dist_by_deg=None, *, default=3.0, seen=True) -> Scan:
    """全周 `default` [m] の壁に囲まれた点群を作り、`dist_by_deg` で上書きする。"""
    dist = [default] * 360
    for deg, d in (dist_by_deg or {}).items():
        dist[deg % 360] = d
    return Scan(dist=dist, sector_seen=[seen] * 12, rot_speed_dps=3600.0)


def corridor(width_deg: int, *, wall=0.5, open_dist=5.0) -> Scan:
    """前方に `width_deg` ぶんだけ開いた通路。それ以外は `wall` [m] の壁。"""
    dist = [wall] * 360
    for deg in range(-width_deg // 2, width_deg // 2 + 1):
        dist[deg % 360] = open_dist
    return Scan(dist=dist, sector_seen=[True] * 12, rot_speed_dps=3600.0)


def offset_gap(center_deg: int, half_width: int, *, wall=1.0, open_dist=5.0) -> Scan:
    """`center_deg` を中心に `half_width` ぶん開いた隙間。それ以外は `wall` [m] の壁。"""
    dist = [wall] * 360
    for deg in range(center_deg - half_width, center_deg + half_width + 1):
        dist[deg % 360] = open_dist
    return Scan(dist=dist, sector_seen=[True] * 12, rot_speed_dps=3600.0)


def camera_scan(fov_deg: int, *, wall=0.5, open_dist=5.0, open_within=None) -> Scan:
    """**カメラの視野（±`fov_deg`/2）だけが見えている**点群。

    LiDAR 用の `corridor()`/`offset_gap()` は `sector_seen=[True]*12`（全周が
    見えている）前提だが、カメラは実視野の外を一切受信できない。その外側の
    セクタを `sector_seen=False` にして、`FollowTheGapCam` が本当に
    「カメラが見えている範囲だけ」で判断できることを試す。
    """
    half = fov_deg // 2
    open_half = (fov_deg if open_within is None else open_within) // 2
    dist = [wall] * 360
    for deg in range(-open_half, open_half + 1):
        dist[deg % 360] = open_dist
    visible = {sector_of_deg(d % 360) for d in range(-half, half + 1)}
    sector_seen = [s in visible for s in range(12)]
    return Scan(dist=dist, sector_seen=sector_seen, rot_speed_dps=3600.0)


class TestSectorMapping(unittest.TestCase):
    """`sector_seen` の添字は**境界が1つずれている**（`Scan` の docstring）。"""

    def test_boundary_belongs_to_previous_sector(self):
        # セクタ s が持つのは 30*s+1 〜 30*s+30。30*s ちょうどは隣のセクタ由来
        self.assertEqual(sector_of_deg(1), 0)
        self.assertEqual(sector_of_deg(30), 0)
        self.assertEqual(sector_of_deg(31), 1)
        self.assertEqual(sector_of_deg(0), 11)      # 0° は最後のセクタ側
        self.assertEqual(sector_of_deg(359), 11)

    def test_covers_all_sectors(self):
        self.assertEqual(sorted({sector_of_deg(d) for d in range(360)}), list(range(12)))


class TestFollowTheGap(unittest.TestCase):
    def setUp(self):
        self.p = FollowTheGap()
        self.params = FollowTheGap.merged({})

    def plan(self, scan, vs=None, **over):
        p = {**self.params, **over}
        return self.p.plan(scan, vs, p, 0.1)

    # ── 進む ──

    def test_straight_corridor_goes_straight(self):
        st = self.plan(corridor(60))
        self.assertTrue(st.ready, st.reason)
        self.assertAlmostEqual(st.target_steer, 0.0, delta=0.05)
        self.assertGreater(st.target_speed, 0.0)

    def test_picks_the_open_side(self):
        """左 40° だけが開いていれば**左（正の舵角）**へ向く。"""
        dist = [0.6] * 360
        for deg in range(30, 71):
            dist[deg] = 4.0
        scan = Scan(dist=dist, sector_seen=[True] * 12)
        st = self.plan(scan)
        self.assertTrue(st.ready, st.reason)
        # 反時計回り＝左が正。この隙間は奥行き 4m と深く、Pure Pursuit の式では
        # 深いギャップほど Ld が大きくなり舵は緩やかになる（意図した振る舞い）ので、
        # 閾値は「有意に正」であることだけを見る
        self.assertGreater(st.target_steer, 0.03)
        self.assertGreater(st.gap_start_deg, 0)

    def test_picks_the_larger_of_two_gaps(self):
        dist = [0.4] * 360
        for deg in range(20, 31):                    # 右…ではなく左の細い隙間
            dist[deg] = 4.0
        for deg in range(300, 341):                  # 右の広い隙間（-60〜-20°）
            dist[deg] = 4.0
        st = self.plan(Scan(dist=dist, sector_seen=[True] * 12))
        self.assertTrue(st.ready, st.reason)
        self.assertLess(st.target_steer, 0.0)        # 右へ切る
        self.assertLess(st.gap_end_deg, 0)

    def test_aims_at_the_middle_not_the_deepest(self):
        """一番遠い点ではなく**ギャップの真ん中**を狙う（壁際を縫わない）。"""
        dist = [0.4] * 360
        for deg in range(0, 61):
            dist[deg] = 3.0
        dist[58] = 8.0                               # 端に一番遠い点を置く
        st = self.plan(Scan(dist=dist, sector_seen=[True] * 12))
        self.assertLess(abs(st.heading), 0.9)        # 58°(=1.01rad) を狙っていない

    def test_lookahead_scales_with_speed_not_gap_depth(self):
        """Ld は速度比例。**同じ狙い角でも、速いほど舵は緩くなる。**

        速度不明（`vs is None`）のときは Ld が最小になる ＝ 最も鋭く曲がる側に
        倒れる。奥まで見えるギャップ（奥行き数m）でも、Ld はその奥行きではなく
        速度で決まるので、静止時と低速時で舵の強さがほぼ変わらないことも確認する。
        """
        dist = [0.6] * 360
        for deg in range(30, 71):                     # 幅40°・奥行き4mの深いギャップ
            dist[deg] = 4.0
        scan = Scan(dist=dist, sector_seen=[True] * 12)

        stopped = self.plan(scan, vs=None)
        slow = self.plan(scan, vs=VehicleState(speed=0.1))
        fast = self.plan(scan, vs=VehicleState(speed=2.0))

        self.assertGreater(stopped.target_steer, 0.1)  # 深いギャップでも鋭く曲がる
        self.assertGreater(slow.target_steer, fast.target_steer)

    # ── 止まる ──

    def test_stops_when_wall_is_close_ahead(self):
        """横は抜けていても、**正面が詰まっていればまず止める。**"""
        dist = [4.0] * 360
        for deg in range(-15, 16):                   # 正面だけ 20cm
            dist[deg % 360] = 0.2
        st = self.plan(Scan(dist=dist, sector_seen=[True] * 12))
        self.assertTrue(st.brake)
        self.assertEqual(st.target_speed, 0.0)
        self.assertIn("停止", st.reason)
        self.assertTrue(st.ready)                    # 意図した停止であって計画不能ではない

    def test_slows_down_as_the_wall_approaches(self):
        far = self.plan(corridor(120, open_dist=5.0)).target_speed
        near = self.plan(corridor(120, open_dist=1.2)).target_speed
        self.assertGreater(far, near)
        self.assertGreater(near, 0.0)

    def test_no_gap_is_not_ready(self):
        """全周 50cm の箱の中 ＝ 正面は空いているが**進める隙間が無い**。

        停止距離（既定 35cm）より手前ではないので「正面で停止」には落ちず、
        ギャップ探索が空振りする経路を通る。**惰行させない。**
        """
        st = self.plan(make_scan(default=0.5))
        self.assertFalse(st.ready)
        self.assertIn("隙間", st.reason)
        self.assertEqual(st.target_speed, 0.0)

    def test_missing_sectors_are_walls_not_free_space(self):
        """欠測は「空いている」ではない。**視野の大半が欠ければ計画を放棄する。**"""
        scan = corridor(60)
        scan.sector_seen = [False] * 12
        st = self.plan(scan)
        self.assertFalse(st.ready)
        self.assertIn("欠測", st.reason)

    def test_a_single_missing_sector_blocks_that_direction(self):
        """1セクタだけ欠けたら、その方向は**選ばれない**（走行自体は続く）。"""
        dist = [0.4] * 360
        for deg in range(31, 61):                    # セクタ1（左）を開ける
            dist[deg] = 4.0
        for deg in range(301, 331):                  # セクタ10（右）も開ける
            dist[deg] = 4.0
        scan = Scan(dist=dist, sector_seen=[True] * 12)
        scan.sector_seen[1] = False                  # 左だけ受信できていない
        st = self.plan(scan)
        self.assertTrue(st.ready, st.reason)
        self.assertLess(st.target_steer, 0.0)        # 見えている右を選ぶ

    def test_saturated_points_are_free_space(self):
        """圧縮フォーマットの 255（5.10m 以上）を壁として打たない。"""
        scan = corridor(60, open_dist=5.10)
        scan.saturated = [False] * 360
        for deg in list(range(0, 31)) + list(range(330, 360)):
            scan.saturated[deg] = True
        st = self.plan(scan)
        self.assertTrue(st.ready, st.reason)
        self.assertGreater(st.target_speed, 0.0)

    def test_safety_bubble_masks_around_the_nearest_point(self):
        """最近傍のすぐ横は、距離が足りていても選ばせない。"""
        dist = [3.0] * 360
        dist[20] = 0.15                              # 左前方の至近に障害物
        st = self.plan(Scan(dist=dist, sector_seen=[True] * 12))
        self.assertTrue(st.ready, st.reason)
        self.assertLessEqual(st.bubble_start_deg, 20)
        self.assertGreaterEqual(st.bubble_end_deg, 20)
        # ギャップは 20° を含まない
        self.assertFalse(st.gap_start_deg <= 20 <= st.gap_end_deg)

    # ── 平滑化と初期化 ──

    def test_steer_is_smoothed_over_time(self):
        scan = corridor(60)
        dist = [0.4] * 360
        for deg in range(40, 81):
            dist[deg] = 4.0
        turn = Scan(dist=dist, sector_seen=[True] * 12)
        self.plan(scan)                              # 直進で 0 に落ち着かせる
        first = self.plan(turn).target_steer
        second = self.plan(turn).target_steer
        self.assertLess(first, second)               # 1周期では狙い値まで届かない

    def test_reset_clears_the_steering_state(self):
        dist = [0.4] * 360
        for deg in range(40, 81):
            dist[deg] = 4.0
        turn = Scan(dist=dist, sector_seen=[True] * 12)
        for _ in range(20):
            self.plan(turn)
        converged = self.plan(turn).target_steer
        self.assertGreater(converged, 0.1)
        self.p.reset()
        # reset 直後は 0 から始まるので、1周期ぶんしか動いていない
        self.assertLess(self.plan(turn).target_steer, converged * 0.8)

    # ── パラメータ ──

    def test_params_are_clamped_to_the_declared_range(self):
        """GUI を信用しない。**範囲外の値で走り出す方が、無視されるより危ない。**"""
        p = FollowTheGap.merged({"max_speed": 99.0, "max_steer": -5.0})
        spec = {s.key: s for s in FollowTheGap.params}
        self.assertEqual(p["max_speed"], spec["max_speed"].max)
        self.assertEqual(p["max_steer"], spec["max_steer"].min)

    def test_unknown_and_broken_params_fall_back_to_defaults(self):
        p = FollowTheGap.merged({"nonsense": 1.0, "max_speed": "速い"})
        self.assertNotIn("nonsense", p)
        self.assertEqual(p["max_speed"], FollowTheGap.defaults()["max_speed"])

    def test_speed_never_exceeds_max_speed(self):
        for open_dist in (0.5, 1.0, 2.0, 5.0):
            st = self.plan(corridor(120, open_dist=open_dist), max_speed=0.3)
            self.assertLessEqual(st.target_speed, 0.3 + 1e-9)


class TestFollowTheGapCam(unittest.TestCase):
    """カメラ由来の擬似スキャンでも `FollowTheGap` のロジックがそのまま動くこと。

    `follow_the_gap_cam.py` は `plan()` を一切上書きしていない——それ自体が
    設計の核心（`raspi/nav/ipm.py`・`cam_perception_node.py` が作る）なので、
    ここでまず**同じ関数であること**を直接確認してから、実際にカメラの
    実視野しか見えていない点群で動くことを試す。
    """

    def test_plan_is_not_overridden(self):
        """`FollowTheGapCam` はギャップ探索を1行も書いていない証明。"""
        self.assertIs(FollowTheGapCam.plan, FollowTheGap.plan)

    def test_declares_a_distinct_input_topic_and_staleness(self):
        self.assertEqual(FollowTheGapCam.input_topic, TOPIC_SCAN_CAM)
        self.assertNotEqual(FollowTheGapCam.input_topic, FollowTheGap.input_topic)
        self.assertEqual(FollowTheGap.input_topic, TOPIC_SCAN)
        self.assertGreater(FollowTheGapCam.stale_ms, FollowTheGap.stale_ms)

    def test_only_fov_deg_param_differs_from_follow_the_gap(self):
        """`fov_deg` の範囲/既定値だけをカメラの実視野に合わせて狭めている。"""
        base = {s.key: s for s in FollowTheGap.params}
        cam = {s.key: s for s in FollowTheGapCam.params}
        self.assertEqual(set(base), set(cam), "パラメータの構成が変わっている")
        for key in base:
            if key == "fov_deg":
                self.assertNotEqual(cam[key].default, base[key].default)
                self.assertLessEqual(cam[key].max, base[key].max)
            else:
                self.assertEqual(cam[key], base[key], f"{key} が無断で変わっている")

    def test_drives_using_only_the_camera_fov(self):
        """視野の外（`sector_seen=False`）が壁として扱われても、視野内の
        隙間だけで普通に走り出す（LiDAR 版と同じ安全側の読み方を継承）。"""
        p = FollowTheGapCam()
        params = FollowTheGapCam.merged({})
        scan = camera_scan(int(params["fov_deg"]))
        st = p.plan(scan, None, params, 0.1)
        self.assertTrue(st.ready, st.reason)
        self.assertAlmostEqual(st.target_steer, 0.0, delta=0.05)
        self.assertGreater(st.target_speed, 0.0)

    def test_stops_when_camera_fov_is_too_narrow_to_see_enough(self):
        """視野の大半が `sector_seen=False` なら、LiDAR 版と同じ
        `MIN_SEEN_RATIO` の下限に引っかかって止まる（＝カメラの推論が
        止まった／未起動のフレームを「壁」として安全側に倒す設計の確認）。"""
        p = FollowTheGapCam()
        params = FollowTheGapCam.merged({"fov_deg": 60})
        scan = Scan(dist=[5.0] * 360, sector_seen=[False] * 12)
        st = p.plan(scan, None, params, 0.1)
        self.assertFalse(st.ready)
        self.assertTrue(st.reason)


class TestLineTrace(unittest.TestCase):
    """白線の目標点（`LineScan`）を Pure Pursuit で追う。"""

    def setUp(self):
        self.p = LineTrace()
        self.params = LineTrace.merged({})

    def plan(self, line, vs=None, dt=0.1, **over):
        p = {**self.params, **over}
        return self.p.plan(line, vs, p, dt)

    def test_declares_a_distinct_input_topic(self):
        self.assertEqual(LineTrace.input_topic, TOPIC_LINE_CAM)
        self.assertNotEqual(LineTrace.input_topic, TOPIC_SCAN)

    def test_line_straight_ahead_goes_straight(self):
        line = LineScan(seen=True, far_seen=True, far_x=1.0, far_y=0.0, coverage=0.05)
        st = self.plan(line)
        self.assertTrue(st.ready, st.reason)
        self.assertAlmostEqual(st.target_steer, 0.0, delta=0.05)
        self.assertGreater(st.target_speed, 0.0)

    def test_line_to_the_left_steers_left(self):
        """左（y正）にある目標点へは正の舵角（反時計回り正）で向く。"""
        line = LineScan(seen=True, far_seen=True, far_x=1.0, far_y=0.3, coverage=0.05)
        st = self.plan(line)
        self.assertTrue(st.ready, st.reason)
        self.assertGreater(st.target_steer, 0.0)

    def test_line_to_the_right_steers_right(self):
        line = LineScan(seen=True, far_seen=True, far_x=1.0, far_y=-0.3, coverage=0.05)
        st = self.plan(line)
        self.assertTrue(st.ready, st.reason)
        self.assertLess(st.target_steer, 0.0)

    def test_falls_back_to_near_point_when_far_is_not_seen(self):
        line = LineScan(seen=True, near_seen=True, near_x=0.3, near_y=0.1, coverage=0.05)
        st = self.plan(line)
        self.assertTrue(st.ready, st.reason)
        self.assertGreater(st.target_steer, 0.0)

    def test_stops_when_line_is_not_seen(self):
        line = LineScan(seen=False)
        st = self.plan(line)
        self.assertFalse(st.ready)
        self.assertIn("見失", st.reason)
        self.assertEqual(st.target_speed, 0.0)

    def test_stops_when_coverage_is_below_minimum(self):
        """検出はできているが割合が低すぎる（ノイズの疑い）。"""
        line = LineScan(seen=True, far_seen=True, far_x=1.0, far_y=0.0, coverage=0.001)
        st = self.plan(line, min_coverage=0.01)
        self.assertFalse(st.ready)

    def test_speed_never_exceeds_max_speed(self):
        for y in (-0.3, 0.0, 0.3):
            st = self.plan(LineScan(seen=True, far_seen=True, far_x=1.0, far_y=y,
                                    coverage=0.05), max_speed=0.3)
            self.assertLessEqual(st.target_speed, 0.3 + 1e-9)

    def test_reset_clears_the_steering_state(self):
        line = LineScan(seen=True, far_seen=True, far_x=1.0, far_y=0.4, coverage=0.05)
        for _ in range(20):
            converged = self.plan(line).target_steer
        self.assertGreater(abs(converged), 0.05)
        self.p.reset()
        self.assertLess(abs(self.plan(line).target_steer), abs(converged) * 0.8)

    def test_not_ready_always_carries_a_reason(self):
        for line in (LineScan(seen=False), LineScan(seen=True, coverage=0.0)):
            st = self.plan(line)
            if not st.ready or st.brake:
                self.assertTrue(st.reason)


class TestDisparityExtender(unittest.TestCase):
    """**FTG と同じ安全条件を満たしたうえで、狙点が「一番遠く」になること。**"""

    def setUp(self):
        self.p = DisparityExtender()
        self.params = DisparityExtender.merged({})

    def plan(self, scan, **over):
        p = {**self.params, **over}
        return self.p.plan(scan, None, p, 0.1)

    # ── 段差を埋める本体 ──

    def test_extends_the_far_side_of_a_disparity(self):
        """近い側 0.5m・遠い側 5.0m の縁は、**遠い側**が 0.5m で塗られる。"""
        r = [0.5] * 10 + [5.0] * 60
        out = _extend(r, 0.30, 0.16)
        # atan2(0.16, 0.5) = 17.7° → 18 点ぶん塗られる
        self.assertEqual(out[10:28], [0.5] * 18)
        self.assertEqual(out[28], 5.0)                 # その先は残る
        self.assertEqual(out[:10], [0.5] * 10)         # 近い側は動かさない

    def test_extends_the_other_way_too(self):
        r = [5.0] * 60 + [0.5] * 10
        out = _extend(r, 0.30, 0.16)
        self.assertEqual(out[42:60], [0.5] * 18)
        self.assertEqual(out[41], 5.0)

    def test_nearer_disparity_paints_wider(self):
        """**近いほど広く塗る。** 同じ半幅を見込む角度が広がるため。"""
        near = _extend([0.3] * 5 + [5.0] * 60, 0.30, 0.16)
        far = _extend([2.0] * 5 + [5.0] * 60, 0.30, 0.16)
        self.assertGreater(sum(v < 5.0 for v in near), sum(v < 5.0 for v in far))

    def test_painting_is_order_independent(self):
        """左右から同じ所を塗り合っても答えが変わらない（`min` で入れている）。"""
        r = [5.0] * 20 + [0.4] * 3 + [5.0] * 20
        out = _extend(r, 0.30, 0.16)
        self.assertEqual(out, _extend(list(reversed(r)), 0.30, 0.16)[::-1])

    def test_a_narrow_slot_is_closed_and_the_wide_opening_wins(self):
        """車体が通れない細い抜けは**塞がって選ばれない**。ここが FTG との差。"""
        dist = [0.8] * 360
        for deg in (-5, -4, -3):                     # 右の細い抜け（3°）
            dist[deg % 360] = 5.0
        for deg in range(60, 91):                    # 左の広い開口（31°）
            dist[deg] = 5.0
        st = self.plan(Scan(dist=dist, sector_seen=[True] * 12))
        self.assertTrue(st.ready, st.reason)
        self.assertGreater(st.heading, 0.0, "細い抜けの方を狙っている")

    # ── 狙点 ──

    def test_aims_at_the_deepest_not_the_middle(self):
        """FTG は真ん中を狙うが、DE は**一番遠い方向**を狙う。"""
        # 右 40° が 5m、左 40° が 2m。どちらも十分広い
        dist = [1.0] * 360
        for deg in range(-60, -20):
            dist[deg % 360] = 5.0
        for deg in range(20, 60):
            dist[deg % 360] = 2.0
        scan = Scan(dist=dist, sector_seen=[True] * 12, rot_speed_dps=3600.0)
        st = self.plan(scan)
        self.assertLess(st.heading, 0.0, "遠い右側を狙っていない")

    def test_open_field_goes_straight(self):
        """視野いっぱいが同じ距離なら、**正面**を選ぶ（配列の端に寄らない）。"""
        st = self.plan(make_scan(default=6.0))
        self.assertAlmostEqual(st.target_steer, 0.0, delta=0.05)

    # ── 安全条件は FTG と同じ ──

    def test_stops_when_wall_is_close_ahead(self):
        st = self.plan(make_scan(default=0.2))
        self.assertTrue(st.brake)
        self.assertEqual(st.target_speed, 0.0)
        self.assertIn("停止", st.reason)

    def test_missing_sectors_are_walls_not_free_space(self):
        st = self.plan(make_scan(seen=False))
        self.assertFalse(st.ready)
        self.assertIn("欠測", st.reason)

    def test_a_single_missing_sector_blocks_that_direction(self):
        """1セクタだけ欠けたら、**その方向は壁**として扱われる。"""
        dist = [5.0] * 360
        seen = [True] * 12
        seen[sector_of_deg(45)] = False            # 左 30〜60° が届いていない
        scan = Scan(dist=dist, sector_seen=seen, rot_speed_dps=3600.0)
        st = self.plan(scan)
        self.assertTrue(st.ready, st.reason)
        self.assertLessEqual(st.heading, math.radians(30.0),
                             "欠測している方向を狙っている")

    def test_saturated_points_are_free_space(self):
        dist = [0.8] * 360
        sat = [False] * 360
        for deg in range(-30, 31):
            dist[deg % 360] = 5.10
            sat[deg % 360] = True
        scan = Scan(dist=dist, sector_seen=[True] * 12, saturated=sat,
                    rot_speed_dps=3600.0)
        st = self.plan(scan)
        self.assertTrue(st.ready, st.reason)
        self.assertGreater(st.target_speed, 0.0)

    def test_speed_never_exceeds_max_speed(self):
        for open_dist in (0.5, 1.0, 2.0, 5.0):
            st = self.plan(corridor(120, open_dist=open_dist), max_speed=0.3)
            self.assertLessEqual(st.target_speed, 0.3 + 1e-9)

    def test_reset_clears_the_steering_state(self):
        dist = [0.8] * 360
        # 左が広く開いた場面で舵を寄せる。**安全半幅ぶん塗られても残る幅**にする
        for deg in range(20, 91):
            dist[deg] = 4.0
        turn = Scan(dist=dist, sector_seen=[True] * 12)
        for _ in range(40):
            converged = self.plan(turn).target_steer
        self.assertGreater(abs(converged), 0.1)
        self.p.reset()
        self.assertLess(abs(self.plan(turn).target_steer), abs(converged) * 0.8)

    def test_not_ready_always_carries_a_reason(self):
        """`base.py` の約束2。**止めた理由の無い停止を作らない。**"""
        for scan in (make_scan(seen=False), make_scan(default=0.2),
                     make_scan(default=0.05)):
            st = self.plan(scan)
            if not st.ready or st.brake:
                self.assertTrue(st.reason)


class TestDisparityPursuit(unittest.TestCase):
    """DE の安全マージン・狙点選びと FTG の Pure Pursuit 舵角を両取りし、
    ①狙点のヒステリシス・②曲率ベースの速度上限・③TTC を追加した Planner。

    基本契約（進む／止まる／欠測は壁）は DE と同じ処理経路を通るので、
    ここでは主に③つの新規要素と、DE/FTGと共有する狙点の性質だけを厚く見る。
    """

    def setUp(self):
        self.p = DisparityPursuit()
        self.params = DisparityPursuit.merged({})

    def plan(self, scan, vs=None, dt=0.1, **over):
        p = {**self.params, **over}
        return self.p.plan(scan, vs, p, dt)

    # ── 基本契約 ──

    def test_open_field_goes_straight(self):
        st = self.plan(make_scan(default=6.0))
        self.assertAlmostEqual(st.target_steer, 0.0, delta=0.05)
        self.assertGreater(st.target_speed, 0.0)

    def test_stops_when_wall_is_close_ahead(self):
        st = self.plan(make_scan(default=0.2))
        self.assertTrue(st.brake)
        self.assertEqual(st.target_speed, 0.0)
        self.assertIn("停止", st.reason)

    def test_aims_at_the_farthest_not_the_middle(self):
        """DE と同じく、真ん中ではなく**一番遠い方向**を狙う。"""
        dist = [1.0] * 360
        for deg in range(-60, -20):
            dist[deg % 360] = 5.0
        for deg in range(20, 60):
            dist[deg % 360] = 2.0
        scan = Scan(dist=dist, sector_seen=[True] * 12, rot_speed_dps=3600.0)
        st = self.plan(scan)
        self.assertLess(st.heading, 0.0, "遠い右側を狙っていない")

    def test_lookahead_scales_with_speed_not_gap_depth(self):
        """FTG と同じく、Ld はギャップの奥行きではなく速度で決まる。"""
        dist = [1.0] * 360
        for deg in range(30, 71):
            dist[deg] = 4.0
        scan = Scan(dist=dist, sector_seen=[True] * 12, rot_speed_dps=3600.0)

        stopped = self.plan(scan, vs=None)
        slow = self.plan(scan, vs=VehicleState(speed=0.1))
        fast = self.plan(scan, vs=VehicleState(speed=2.0))

        self.assertGreater(stopped.target_steer, 0.05)
        self.assertGreater(slow.target_steer, fast.target_steer)

    def test_speed_never_exceeds_max_speed(self):
        for open_dist in (0.5, 1.0, 2.0, 5.0):
            st = self.plan(corridor(120, open_dist=open_dist), max_speed=0.3)
            self.assertLessEqual(st.target_speed, 0.3 + 1e-9)

    def test_not_ready_always_carries_a_reason(self):
        for scan in (make_scan(seen=False), make_scan(default=0.2),
                     make_scan(default=0.05)):
            st = self.plan(scan)
            if not st.ready or st.brake:
                self.assertTrue(st.reason)

    # ── ①狙点のヒステリシス ──

    def test_best_band_ties_follow_the_previous_heading(self):
        """同じ広さ・同じ遠さの帯が2つ並ぶ同着は、**前回ヘディングに近い方**を採る。

        `DisparityExtender._best_band()` は常に「正面に近い方」を採るが、
        ここでは前回どちらを向いていたかで結果が変わることを確認する
        （前回ヘディングが 0 付近なら DE と同じ挙動に自然収束する）。
        """
        r = [5.0] * 20 + [1.0] + [5.0] * 20
        degs = list(range(-20, 21))
        threshold = 4.99

        a, b = _best_band(r, threshold, degs, prev_heading_deg=-15.0)
        self.assertLess(degs[(a + b) // 2], 0, "前回右寄りなら右の帯を維持する")

        a, b = _best_band(r, threshold, degs, prev_heading_deg=15.0)
        self.assertGreater(degs[(a + b) // 2], 0, "前回左寄りなら左の帯を維持する")

    # ── ②曲率ベースの速度上限 ──

    def test_curvature_limits_speed_more_in_sharp_turns(self):
        """同じ見通しでも、**急な旋回ほど**曲率上限で速度を落とす。"""
        gentle = offset_gap(10, 15)
        sharp = offset_gap(45, 15)
        # 見通しベースの速度上限(v_range)が効かないよう max_speed を上げ、
        # 曲率上限(v_curve)だけが効くよう a_lat_max を下げて切り分ける。
        # safety_half_width も下げておく（既定 0.30 だと隙間の縁が両側から
        # 塗りつぶされて幅31°の隙間ごと消えてしまい、切り分けにならない）
        over = {"a_lat_max": 0.3, "max_speed": 2.0, "safety_half_width": 0.05}
        params = {**self.params, **over}
        p1, p2 = DisparityPursuit(), DisparityPursuit()
        for _ in range(30):
            st_gentle = p1.plan(gentle, VehicleState(speed=1.0), params, 0.1)
            st_sharp = p2.plan(sharp, VehicleState(speed=1.0), params, 0.1)

        self.assertTrue(st_gentle.ready, st_gentle.reason)
        self.assertTrue(st_sharp.ready, st_sharp.reason)
        self.assertGreater(abs(st_sharp.target_steer), abs(st_gentle.target_steer))
        self.assertLess(st_sharp.target_speed, st_gentle.target_speed)

    # ── ③TTC（衝突余裕時間） ──

    def test_ttc_brakes_before_stop_dist_if_closing_fast(self):
        """`stop_dist` の手前でも、**急速に接近していれば**先に止める。"""
        far = make_scan(default=2.0)
        near = make_scan(default=1.0)
        self.plan(far, dt=0.1)                  # 1周目: 前フレーム値を作るだけ
        st = self.plan(near, dt=0.1)             # 2.0m→1.0m/0.1s ＝ 接近10m/s
        self.assertTrue(st.brake)
        self.assertGreater(st.free_ahead, self.params["stop_dist"],
                            "stop_dist にはまだ届いていない距離であること")
        self.assertIn("接近", st.reason)

    def test_ttc_does_not_trigger_when_closing_slowly(self):
        far = make_scan(default=2.0)
        near = make_scan(default=1.9)
        self.plan(far, dt=0.1)
        st = self.plan(near, dt=0.1)             # 2.0m→1.9m/0.1s ＝ 接近1m/s
        self.assertFalse(st.brake)


class TestRegistry(unittest.TestCase):
    def test_catalog_is_json_serialisable_and_complete(self):
        raw = msgspec.json.encode(catalog())
        got = msgspec.json.decode(raw)
        self.assertEqual([e["id"] for e in got], list(PLANNERS))
        for entry in got:
            self.assertTrue(entry["name"] and entry["description"])
            for s in entry["params"]:
                self.assertLessEqual(s["min"], s["default"])
                self.assertLessEqual(s["default"], s["max"])
                self.assertGreater(s["step"], 0)
                self.assertTrue(s["label"], s["key"])
                self.assertTrue(s["note"], s["key"])

    def test_unknown_mode_returns_none_instead_of_raising(self):
        """古い設定や消した planner の id が来ても落ちない。"""
        self.assertIsNone(make_planner("no-such-mode"))
        self.assertIsNone(make_planner(""))
        self.assertIsInstance(make_planner("ftg"), FollowTheGap)

    def test_camera_planner_is_registered(self):
        """`ftg_cam` が 1 ファイル＋1 行の追加だけで GUI の選択肢に出る。"""
        self.assertIn("ftg_cam", PLANNERS)
        self.assertIsInstance(make_planner("ftg_cam"), FollowTheGapCam)

    def test_line_trace_planner_is_registered(self):
        self.assertIn("line_trace", PLANNERS)
        self.assertIsInstance(make_planner("line_trace"), LineTrace)

    def test_e2e_lidar_planner_is_registered(self):
        self.assertIn("e2e_lidar", PLANNERS)
        self.assertIsInstance(make_planner("e2e_lidar"), E2ELidar)


class FakeSub:
    """`Subscriber` の身代わり。`latest` だけを持つ。"""

    def __init__(self, latest=None):
        self.latest = latest or {}


class TestAutoRelay(unittest.IsolatedAsyncioTestCase):
    """`telemetry_node._merge_auto` — **自律指令が車に届く唯一の口**。

    ここが「意図せず開いている」「止まる指令が届かない」のが一番危ないので、
    通信のテストではなく安全条件のテストとして書く。

    `_merge_auto` は途絶を検出したときに `asyncio.create_task` で GUI へ知らせる
    （本番では `_cmd_pump` の中＝ループ上で呼ばれる）ので、テストもループ上で回す。
    """

    def setUp(self):
        from raspi.nodes.telemetry_node import TelemetryServer

        async def _noop():
            pass

        # バスに繋がずにメソッドだけ使う（`__init__` は ZMQ を bind するので通さない）
        self.srv = TelemetryServer.__new__(TelemetryServer)
        self.srv._auto_mode = "ftg"
        self.srv._auto_engaged = True
        self.srv._auto_was_fresh = True
        self.srv.auto_stalls = 0
        self.srv.control_clients = set()
        self.srv._broadcast_control_status = _noop
        self.gui = DriveCmd(mode=2, arm=True, light_mode=2, horn=True,
                            accel_limit=6.0, steer_rate_limit=7.0,
                            brake_torque=0.125, auto_stop=True,
                            torque_mode=True, target_torque=0.05,
                            target_speed=0.0, target_steer=0.0, source="gui:x")

    def merge(self, auto: DriveCmd | None, *, age_ns: int = 0) -> DriveCmd:
        now = time.monotonic_ns()
        latest = {}
        if auto is not None:
            auto.t_pub = now - age_ns
            latest["auto/cmd"] = auto
        self.srv.sub = FakeSub(latest)
        return self.srv._merge_auto(self.gui, now)

    async def test_speed_and_steer_come_from_the_planner(self):
        out = self.merge(DriveCmd(target_speed=0.4, target_steer=0.3))
        self.assertAlmostEqual(out.target_speed, 0.4)
        self.assertAlmostEqual(out.target_steer, 0.3)
        self.assertEqual(out.mode, 2)
        self.assertTrue(out.source.startswith("auto:ftg"))

    async def test_aux_bits_stay_with_the_human(self):
        """灯火・ホーン・`auto_stop`・レートリミットは GUI の値のまま。"""
        out = self.merge(DriveCmd(target_speed=0.4))
        self.assertEqual(out.light_mode, 2)
        self.assertTrue(out.horn)
        self.assertTrue(out.auto_stop)
        self.assertAlmostEqual(out.brake_torque, 0.125)
        self.assertAlmostEqual(out.accel_limit, 6.0)
        self.assertTrue(out.arm)                     # arm は人間しか立てられない

    async def test_torque_mode_is_never_carried_into_auto(self):
        """ラジコンのトルクモードが ON のまま engage されても持ち込まない。"""
        out = self.merge(DriveCmd(target_speed=0.4))
        self.assertFalse(out.torque_mode)
        self.assertEqual(out.target_torque, 0.0)

    async def test_stale_auto_cmd_becomes_a_brake(self):
        """planning_node が死んだら、engage したままでも**制動**に落ちる。"""
        out = self.merge(DriveCmd(target_speed=0.9), age_ns=500_000_000)
        self.assertTrue(out.brake)
        self.assertEqual(out.target_speed, 0.0)
        self.assertIn("stale", out.source)
        self.assertEqual(self.srv.auto_stalls, 1)

    async def test_missing_auto_cmd_becomes_a_brake(self):
        out = self.merge(None)
        self.assertTrue(out.brake)
        self.assertEqual(out.target_speed, 0.0)

    async def test_planner_brake_is_passed_through(self):
        out = self.merge(DriveCmd(brake=True, target_speed=0.0, target_steer=0.2))
        self.assertTrue(out.brake)
        self.assertEqual(out.target_speed, 0.0)
        self.assertAlmostEqual(out.target_steer, 0.2)  # 曲がりながら止まれる


class TestAutoCtrlGate(unittest.TestCase):
    """`_on_auto` — engage できる条件と、必ず解除される条件。"""

    def setUp(self):
        from raspi.nodes.telemetry_node import TelemetryServer
        self.srv = TelemetryServer.__new__(TelemetryServer)
        self.srv._auto_mode = ""
        self.srv._auto_engaged = False
        self.srv._auto_params = {}
        self.srv._auto_was_fresh = True
        self.srv.auto_stalls = 0
        self.srv._save_auto_conf = lambda: None      # ディスクに触らせない

    def test_cannot_engage_without_a_mode(self):
        self.srv._on_auto({"engaged": True})
        self.assertFalse(self.srv._auto_engaged)

    def test_changing_mode_always_disengages(self):
        self.srv._on_auto({"mode": "ftg"})
        self.srv._on_auto({"engaged": True})
        self.assertTrue(self.srv._auto_engaged)
        self.srv._on_auto({"mode": ""})
        self.assertFalse(self.srv._auto_engaged)
        self.assertEqual(self.srv._auto_mode, "")

    def test_unknown_mode_is_ignored(self):
        self.srv._on_auto({"mode": "ftg"})
        self.srv._on_auto({"mode": "no-such-mode"})
        self.assertEqual(self.srv._auto_mode, "ftg")

    def test_params_are_clamped_server_side(self):
        self.srv._on_auto({"mode": "ftg", "params": {"max_speed": 99.0}})
        spec = {s.key: s for s in FollowTheGap.params}
        self.assertEqual(self.srv._auto_params["max_speed"], spec["max_speed"].max)

    def test_releasing_control_disengages(self):
        self.srv._on_auto({"mode": "ftg", "engaged": True})
        self.assertTrue(self.srv._auto_engaged)
        # `_release_control` は publish するので、そこだけ差し替える
        self.srv._publish_auto_ctrl = lambda: None
        from raspi.nodes.telemetry_node import TelemetryServer
        TelemetryServer._release_control(self.srv, "接続が切れた")
        self.assertFalse(self.srv._auto_engaged)


class TestAutoStateContract(unittest.TestCase):
    def test_not_ready_state_carries_a_reason(self):
        """`ready=False` で `reason` が空なら、GUI に「原因不明で止まった」と出る。"""
        p = FollowTheGap()
        params = FollowTheGap.merged({})
        for scan in (make_scan(default=0.3), corridor(60, wall=0.2, open_dist=0.2)):
            st = p.plan(scan, None, params, 0.1)
            if not st.ready or st.brake:
                self.assertTrue(st.reason, "止めた理由が空")

    def test_default_state_is_a_stop(self):
        """既定の `AutoState` は「走らない」。**取りこぼしが加速にならない。**"""
        st = AutoState()
        self.assertFalse(st.ready)


#: `sim/gym_env.py` の `OBS_DIM`（点群361 + 速度1）と揃える。`raspi/tests/` は
#: `sim/` を import しない方針なので、値をここに書き写す（変えたら両方直すこと）
_OBS_DIM = 362


def _make_dummy_e2e_model(path: Path, in_dim: int = _OBS_DIM) -> None:
    """`(1,in_dim) → (1,2)` の全結合1層だけのONNXモデル。**torch は使わない**
    （`test_cam_perception_node.py` の `_make_dummy_model()` と同じ手法。
    `raspi/tests/` を重い依存から切り離す方針を守る）。

    重みは決め打ちで、テストの入力から出力を逆算しやすくしてある:
    steer は先頭の点にだけ反応、speed は末尾の入力（正規化した自車速度）にだけ反応。
    """
    w = np.zeros((2, in_dim), dtype=np.float32)
    w[0, 0] = 1.0
    w[1, -1] = 1.0
    b = np.zeros(2, dtype=np.float32)
    inp = helper.make_tensor_value_info("input", TensorProto.FLOAT, [1, in_dim])
    out = helper.make_tensor_value_info("output", TensorProto.FLOAT, [1, 2])
    w_init = helper.make_tensor("W", TensorProto.FLOAT, w.shape, w.flatten())
    b_init = helper.make_tensor("B", TensorProto.FLOAT, b.shape, b.flatten())
    node = helper.make_node("Gemm", ["input", "W", "B"], ["output"], transB=1)
    graph = helper.make_graph([node], "dummy_e2e", [inp], [out], [w_init, b_init])
    model = helper.make_model(graph, opset_imports=[helper.make_opsetid("", 13)])
    onnx.checker.check_model(model)
    onnx.save(model, str(path))


class TestE2ELidar(unittest.TestCase):
    def test_missing_model_is_ready_false(self):
        """モデルが `models/` に無くても落ちずに安全側へ倒れる（未学習時の既定状態）。"""
        p = E2ELidar(model_path="/nonexistent/e2e_lidar.onnx")
        st = p.plan(make_scan(), None, E2ELidar.merged({}), 0.1)
        self.assertFalse(st.ready)
        self.assertIn("モデル", st.reason)

    def test_loaded_model_produces_finite_command(self):
        with tempfile.TemporaryDirectory() as d:
            model_path = Path(d) / "e2e_lidar.onnx"
            _make_dummy_e2e_model(model_path)
            model_path.with_suffix(".json").write_text(json.dumps(
                {"fov_deg": 360, "max_range": 5.10, "max_steer": 0.45, "max_speed": 1.5}))

            p = E2ELidar(model_path=model_path)
            st = p.plan(corridor(180), None, E2ELidar.merged({}), 0.1)
            self.assertTrue(st.ready, st.reason)
            self.assertTrue(math.isfinite(st.target_steer))
            self.assertTrue(math.isfinite(st.target_speed))

    def test_vehicle_speed_feeds_into_the_model_input(self):
        """改善1: 観測の末尾に自車速度が乗る。`vs=None`なら0として扱う。"""
        with tempfile.TemporaryDirectory() as d:
            model_path = Path(d) / "e2e_lidar.onnx"
            _make_dummy_e2e_model(model_path)     # w[1,-1]=1.0: speed_norm = 入力の末尾
            model_path.with_suffix(".json").write_text(json.dumps(
                {"fov_deg": 360, "max_range": 5.10, "max_steer": 0.45, "max_speed": 2.0}))

            p = E2ELidar(model_path=model_path)
            # p["max_speed"]（GUI側の安全クランプ、既定1.0）に速度差が飲まれないよう
            # モデルの max_speed(2.0) まで引き上げておく
            params = E2ELidar.merged({"max_speed": 2.0})
            scan = corridor(180, open_dist=5.0)

            st_none = p.plan(scan, None, params, 0.1)
            st_zero = p.plan(scan, VehicleState(speed=0.0), params, 0.1)
            st_half = p.plan(scan, VehicleState(speed=1.0), params, 0.1)   # 2.0の半分

            # vs=None は speed=0 と同じ扱い（未着信時の安全な既定）
            self.assertAlmostEqual(st_none.target_speed, st_zero.target_speed, places=4)
            # 入力速度が上がれば、モデルへの入力（=このダミーモデルの出力）も上がる
            self.assertGreater(st_half.target_speed, st_zero.target_speed)

    def test_front_obstacle_forces_stop_regardless_of_model_output(self):
        """★独立安全策: モデルが前進を指示しても正面が詰まっていれば止める。"""
        with tempfile.TemporaryDirectory() as d:
            model_path = Path(d) / "e2e_lidar.onnx"
            _make_dummy_e2e_model(model_path)
            model_path.with_suffix(".json").write_text(json.dumps(
                {"fov_deg": 360, "max_range": 5.10, "max_steer": 0.45, "max_speed": 1.5}))

            p = E2ELidar(model_path=model_path)
            scan = make_scan({d: 0.1 for d in range(-15, 16)}, default=5.0)
            st = p.plan(scan, None, E2ELidar.merged({}), 0.1)
            self.assertTrue(st.brake)
            self.assertEqual(st.target_speed, 0.0)

    def test_output_is_clamped_to_param_limits(self):
        """モデルの生出力がどうであれ、`p["max_steer"]`が最終的な上限になる。"""
        with tempfile.TemporaryDirectory() as d:
            model_path = Path(d) / "e2e_lidar.onnx"
            # W[0,:] を大きくして steer_norm が確実に1.0でクリップされるようにする
            w = np.zeros((2, _OBS_DIM), dtype=np.float32)
            w[0, :] = 10.0
            b = np.zeros(2, dtype=np.float32)
            inp = helper.make_tensor_value_info("input", TensorProto.FLOAT, [1, _OBS_DIM])
            out = helper.make_tensor_value_info("output", TensorProto.FLOAT, [1, 2])
            w_init = helper.make_tensor("W", TensorProto.FLOAT, w.shape, w.flatten())
            b_init = helper.make_tensor("B", TensorProto.FLOAT, b.shape, b.flatten())
            node = helper.make_node("Gemm", ["input", "W", "B"], ["output"], transB=1)
            graph = helper.make_graph([node], "dummy_e2e", [inp], [out], [w_init, b_init])
            model = helper.make_model(graph, opset_imports=[helper.make_opsetid("", 13)])
            onnx.save(model, str(model_path))
            model_path.with_suffix(".json").write_text(json.dumps(
                {"fov_deg": 360, "max_range": 5.10, "max_steer": 0.20, "max_speed": 1.5}))

            p = E2ELidar(model_path=model_path)
            st = p.plan(corridor(180, open_dist=5.0), None,
                       E2ELidar.merged({"max_steer": 0.20}), 0.1)
            self.assertLessEqual(abs(st.target_steer), 0.20 + 1e-9)

    def test_reload_if_changed_picks_up_a_named_model(self):
        """GUIの`e2e/model`選択(名前)から`models_dir/<name>.onnx`を解決してロードする。"""
        with tempfile.TemporaryDirectory() as d:
            models_dir = Path(d)
            _make_dummy_e2e_model(models_dir / "alpha.onnx")
            (models_dir / "alpha.json").write_text(json.dumps(
                {"fov_deg": 360, "max_range": 5.10, "max_steer": 0.45, "max_speed": 1.5}))

            p = E2ELidar(models_dir=models_dir)
            st = p.plan(make_scan(), None, E2ELidar.merged({}), 0.1)
            self.assertFalse(st.ready)          # まだ何も選んでいない

            changed = p.reload_if_changed("alpha")
            self.assertTrue(changed)
            st = p.plan(corridor(180), None, E2ELidar.merged({}), 0.1)
            self.assertTrue(st.ready, st.reason)

    def test_reload_if_changed_keeps_current_model_on_failure(self):
        """存在しない名前を選んでも、今動いているモデルのまま走り続ける（安全側）。"""
        with tempfile.TemporaryDirectory() as d:
            models_dir = Path(d)
            _make_dummy_e2e_model(models_dir / "alpha.onnx")
            (models_dir / "alpha.json").write_text(json.dumps(
                {"fov_deg": 360, "max_range": 5.10, "max_steer": 0.45, "max_speed": 1.5}))

            p = E2ELidar(models_dir=models_dir)
            self.assertTrue(p.reload_if_changed("alpha"))

            changed = p.reload_if_changed("no-such-model")
            self.assertFalse(changed)
            st = p.plan(corridor(180), None, E2ELidar.merged({}), 0.1)
            self.assertTrue(st.ready, st.reason)   # alpha のまま生きている

    def test_reload_if_changed_is_a_noop_for_empty_or_same_name(self):
        with tempfile.TemporaryDirectory() as d:
            models_dir = Path(d)
            _make_dummy_e2e_model(models_dir / "alpha.onnx")
            (models_dir / "alpha.json").write_text(json.dumps(
                {"fov_deg": 360, "max_range": 5.10, "max_steer": 0.45, "max_speed": 1.5}))

            p = E2ELidar(models_dir=models_dir)
            self.assertFalse(p.reload_if_changed(""))
            self.assertTrue(p.reload_if_changed("alpha"))
            self.assertFalse(p.reload_if_changed("alpha"))   # 同じ名前は変化なし


if __name__ == "__main__":
    unittest.main()
