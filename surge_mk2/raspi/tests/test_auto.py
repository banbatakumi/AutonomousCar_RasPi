"""自動運転（`raspi/auto/`）と、自律指令が車に届く経路の安全条件のテスト。

**「走る」より「止まる」を厚く試す。** 合成した点群を流し込むだけなので、
ハードウェアもバスも要らない。
"""

import sys
import time
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

import msgspec  # noqa: E402

from raspi.auto import PLANNERS, FollowTheGap, catalog, make_planner  # noqa: E402
from raspi.auto.base import sector_of_deg  # noqa: E402
from raspi.msgs import AutoState, DriveCmd, Scan  # noqa: E402


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

    def plan(self, scan, **over):
        p = {**self.params, **over}
        return self.p.plan(scan, None, p, 0.1)

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
        self.assertGreater(st.target_steer, 0.1)    # 反時計回り＝左が正
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
        mid_deg = st.target_steer / self.params["steer_gain"]
        self.assertLess(abs(mid_deg), 0.9)           # 58°(=1.01rad) を狙っていない

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
        self.assertEqual(st.target_speed, 0.0)
        self.assertFalse(st.engaged)


if __name__ == "__main__":
    unittest.main()
