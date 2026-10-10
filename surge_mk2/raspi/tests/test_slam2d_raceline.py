"""`raspi/auto/slam2d_raceline.py`（slam2d版レーシングライン）の統合テスト。

`test_raceline.py`と同じ流儀。`slam2d`のFrontendが実際にraspi側の型
（`Scan`/`VehicleState`）を受け取って動くこと、既存の`raspi/nav/`（経路生成・
追従・障害物検出）がダックタイピングでそのまま動くことを確認する。
"""

import math
import sys
import tempfile
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

import numpy as np  # noqa: E402

from raspi.auto import mapstore  # noqa: E402
from raspi.auto.registry import PLANNERS, make_planner  # noqa: E402
from raspi.auto.slam2d_raceline import (BUILD, DONE, EXPLORE, LOCATE, RACE,  # noqa: E402
                                        Slam2dRaceLine, forward_traj)
from raspi.msgs import VehicleState  # noqa: E402
from raspi.nav.grid import unpack_trinary  # noqa: E402
from raspi.tests.test_nav import ROOM, make_room_scan  # noqa: E402
from slam2d.core.grid import OccGrid  # noqa: E402
from slam2d.core.types import Pose2D  # noqa: E402
from slam2d.tests.helpers import make_room_points  # noqa: E402


def vs(speed=0.0, yaw_rate=0.0) -> VehicleState:
    return VehicleState(speed=speed, yaw_rate=yaw_rate, steer_actual=0.0)


#: 部屋の中を1周する合成軌道（中心 (4.0, 2.0)・半径 1.2m。中の板を避け、壁から 0.8m 以上）
_LAP_R = 1.2
_LAP_SPEED = 0.3
_LAP_START = (4.0, 0.8, 0.0)


def drive_one_lap_and_build(p: Slam2dRaceLine, params: dict, *, steps: int = 265,
                            dt: float = 0.1):
    """EXPLORE で1周強走って地図を確定し、BUILD が終わるまで回す。最後の `AutoState` を返す。

    車は**進む向きを向いて**走る（`forward_traj` は車の向きに前へ進んだ点だけを残す。
    以前の軌道は向きが 90° ずれていて真横に進んでおり、全点が落ちて BUILD が必ず失敗していた）。
    レーシングラインはワーカーで引くので、計算が終わるのを待ってから次の `plan()` を呼ぶ。
    """
    w = _LAP_SPEED / _LAP_R
    x0, y0, _ = _LAP_START
    for i in range(steps):
        a = w * i * dt
        pose = (x0 + _LAP_R * math.sin(a), y0 + _LAP_R * (1.0 - math.cos(a)), a)
        p.plan(make_room_scan(*pose, segs=ROOM), vs(speed=_LAP_SPEED, yaw_rate=w), params, dt)
    a = w * (steps - 1) * dt
    stop = (x0 + _LAP_R * math.sin(a), y0 + _LAP_R * (1.0 - math.cos(a)), a)
    p.request_freeze()
    st = p.plan(make_room_scan(*stop, segs=ROOM), vs(), params, dt)
    for _ in range(10):
        if st.phase != BUILD:
            break
        if p._build_job is not None:
            p._build_job.result(timeout=60)
        st = p.plan(make_room_scan(*stop, segs=ROOM), vs(), params, dt)
    return st


class TestSlam2dRaceLineStateMachine(unittest.TestCase):
    def setUp(self):
        # BUILD完了で`_auto_save_map()`が実際の`saved_maps/`へ書き込まないよう、
        # このクラスの全テストで一時ディレクトリへ差し替える
        self._td = tempfile.TemporaryDirectory()
        self._orig_maps_dir = mapstore.MAPS_DIR
        mapstore.MAPS_DIR = Path(self._td.name) / "saved_maps"

        self.p = Slam2dRaceLine()
        self.params = Slam2dRaceLine.merged({})

    def tearDown(self):
        mapstore.MAPS_DIR = self._orig_maps_dir
        self._td.cleanup()

    def drive(self, n=10, dt=0.1, *, move=0.0, segs=ROOM):
        out = None
        for i in range(n):
            pose = (3.0 + move * i, 2.0, 0.0)
            out = self.p.plan(make_room_scan(*pose, segs=segs), vs(speed=move / dt),
                              self.params, dt)
        return out

    def test_registered(self):
        self.assertIn(Slam2dRaceLine.id, PLANNERS)
        self.assertIsInstance(make_planner("slam2d_raceline"), Slam2dRaceLine)

    def test_starts_in_explore(self):
        self.assertEqual(self.p.phase, EXPLORE)

    def test_without_vehicle_state_it_does_not_run(self):
        st = self.p.plan(make_room_scan(3.0, 2.0, 0.0), None, self.params, 0.1)
        self.assertFalse(st.ready)
        self.assertIn("車両状態", st.reason)

    def test_explore_delegates_to_follow_the_gap(self):
        st = self.drive(6, move=0.02)
        self.assertEqual(st.phase, EXPLORE)
        self.assertIn("地図作成", st.reason)
        self.assertNotEqual((st.gap_start_deg, st.gap_end_deg), (0.0, 0.0))

    def test_manual_explore_does_not_drive_and_still_maps(self):
        """自動運転に入っていない EXPLORE は人のラジコンでの地図作成（指令を出さない）。"""
        self.p.set_engaged(False)
        st = self.drive(30, move=0.02)
        self.assertEqual(st.phase, EXPLORE)
        self.assertFalse(st.ready)
        self.assertEqual((st.gap_start_deg, st.gap_end_deg), (0.0, 0.0))
        self.assertIn("手動で地図作成", st.reason)
        self.assertGreater(len(self.p.slam.trajectory), 3)

    def test_manual_freeze_from_manual_explore_moves_to_build(self):
        self.p.set_engaged(False)
        self.drive(30, move=0.02)
        self.p.request_freeze()
        st = self.drive(1)
        self.assertEqual(st.phase, BUILD)

    def test_steering_while_stopped_warns(self):
        self.p.set_engaged(False)
        st = None
        for i in range(40):
            v = VehicleState(speed=0.0, yaw_rate=0.0, steer_actual=0.3 * (i % 2))
            st = self.p.plan(make_room_scan(3.0, 2.0, 0.0), v, self.params, 0.1)
        self.assertIn("据え切り", st.reason)

    def test_disengage_during_explore_keeps_the_map(self):
        self.drive(30, move=0.02)
        n = len(self.p.slam.trajectory)
        self.assertGreater(n, 3)
        self.p.on_disengage()
        self.assertEqual(self.p.phase, EXPLORE)
        self.assertEqual(len(self.p.slam.trajectory), n)

    def test_disengage_outside_explore_resets(self):
        self.drive(30, move=0.02)
        self.p.phase = DONE
        self.p.on_disengage()
        self.assertEqual(self.p.phase, EXPLORE)
        self.assertEqual(len(self.p.slam.trajectory), 0)

    def test_map_grows_while_moving(self):
        """動きながら食わせると地図（占有格子）が育つ。"""
        self.drive(30, move=0.02)
        self.assertGreater(int(self.p.slam.grid.wall_mask().sum()), 0)

    def test_lap_progress_accumulates(self):
        """円弧状に回頭させると`lap_progress`（累積回頭÷360°）が増える。"""
        dt = 0.1
        yaw_rate = math.radians(36.0)   # 1周10秒相当
        out = None
        yaw = 0.0
        for i in range(20):
            yaw += yaw_rate * dt
            out = self.p.plan(make_room_scan(3.0, 2.0, yaw), vs(speed=0.05, yaw_rate=yaw_rate),
                              self.params, dt)
        self.assertGreater(out.lap_progress, 0.0)

    def test_reset_clears_phase_and_map(self):
        self.drive(10, move=0.02)
        self.p.reset()
        self.assertEqual(self.p.phase, EXPLORE)
        self.assertEqual(self.p.laps, 0)
        self.assertIsNone(self.p.path)

    def test_clear_bumps_grid_seq_monotonically(self):
        """「地図を削除」で新しい(空の)地図のseqが前より必ず大きいこと。

        seqが0に戻ると、GUI側の「古い版で新しい版を上書きしない」ガード
        （`gui/src/ws/map.ts`）に新しい地図が弾かれ、削除前の地図が
        画面に残り続ける不具合になる（実車で確認済み）。
        """
        self.drive(20, move=0.02)
        seq_before = self.p.slam.grid.seq
        self.p.request_clear()
        self.assertGreater(self.p.slam.grid.seq, seq_before)

    def test_snapshot_returns_a_map(self):
        self.drive(5, move=0.02)
        snap = self.p.snapshot()
        self.assertIsNotNone(snap)
        self.assertGreater(len(snap.cells), 0)

    def test_manual_freeze_moves_to_build(self):
        """「地図を確定」は自動判定の逃げ道。押したら次の周期でBUILD。"""
        self.drive(14, move=0.02)
        self.p.request_freeze()
        st = self.drive(1)
        self.assertEqual(st.phase, BUILD)
        self.assertTrue(self.p.slam.grid.frozen)

    def test_freeze_flushes_loop_closures_and_deactivates_backend(self):
        """EXPLORE中に複数周回るとループ拘束が検出され、`request_freeze()`で
        `flush()`により反映される。凍結後は`Frontend`直結の追跡専用に切り替わる
        （`_slam2d_nav.py`のモジュールdocstring「ループ閉じを使う」参照）。

        円軌道は`speed`(前進方向)と`yaw_rate`が実際の移動方向と整合する正しい
        unicycle運動学（`x = cx - r(1-cos(wt))`, `y = cy + r*sin(wt)`,
        `yaw = wt + pi/2`）を使う——他のテストの`_drive_to_race()`等が使う式
        （`x`に`sin`、`y`に`1-cos`を当てる形）は進行方向と機体姿勢が90°
        ズレており、短い距離では影響が出ないが、複数周走らせるとFrontendが
        早期に見失い状態になり地図が育たなくなる（ループ閉じを検証できない）
        ことを実測で確認したため、ここでは使わない。
        """
        dt = 0.1
        speed = 0.3
        radius = 1.3   # ROOM(6x4m、中心(3,2))の壁に接触しない範囲
        cx, cy = 3.0, 2.0
        yaw_rate = speed / radius
        n_steps = 500   # 円周(2*pi*1.3m)を1.8周ぶん、min_index_gap(既定20)を十分超える
        for i in range(n_steps):
            t = i * dt
            yaw = yaw_rate * t + math.pi / 2
            x = cx - radius * (1 - math.cos(yaw_rate * t))
            y = cy + radius * math.sin(yaw_rate * t)
            self.p.plan(make_room_scan(x, y, yaw, segs=ROOM),
                       vs(speed=speed, yaw_rate=yaw_rate), self.params, dt)

        self.assertTrue(self.p.slam._backend_active)
        self.p.request_freeze()
        self.p.plan(make_room_scan(cx, cy, math.pi / 2, segs=ROOM),
                   vs(speed=0.0, yaw_rate=0.0), self.params, dt)

        self.assertFalse(self.p.slam._backend_active)
        self.assertGreater(self.p.slam.loop_closures, 0)

    def test_build_brakes_and_reports_progress(self):
        self.drive(14, move=0.02)
        self.p.request_freeze()
        st = self.drive(1)
        self.assertFalse(st.ready)
        self.assertEqual(st.phase, BUILD)
        self.assertIn("経路", st.reason)

    def test_build_failure_does_not_raise(self):
        """経路を作れなくても例外を投げない（軌跡が短すぎる状態で確定させる）。"""
        self.drive(3)
        self.p.request_freeze()
        for _ in range(4):
            st = self.drive(1)
        self.assertFalse(st.ready)
        self.assertEqual(st.phase, BUILD)
        self.assertIn("作れなかった", st.reason)

    def test_full_state_machine_builds_saves_and_waits_in_done(self):
        """★ EXPLORE→BUILD→DONEまで通しで動き、地図が自動保存されること。

        BUILD完了後は**自動ではRACEへ進まず**`DONE`で待つ（バンビの指示、
        2026-09-03——地図内の好きな場所へ車を動かし直す間を作るため）。
        その間に地図が自動保存され、「レーシングライン走行」
        （`request_load()`）を押すと`LOCATE`へ進むことを確認する。

        `LOCATE`→`RACE`の自己位置復元の精度自体は`TestSlam2dRaceLineSavedMapLoad`
        （既知姿勢から構築した地図）で検証する——この合成円弧軌道はSLAM追跡の
        精度検証を意図したものではないため（十分な距離を動かして地図を
        育てるためだけの軌道）。
        """
        self.assertEqual(self.p.phase, EXPLORE)
        st = drive_one_lap_and_build(self.p, self.params)
        self.assertGreater(self.p.slam.lap_progress(), 0.9)
        self.assertEqual(st.phase, DONE, st.reason)
        self.assertIsNotNone(self.p.path)

        # 地図が自動保存されていること
        self.assertTrue(self.p._saved_map_name)
        self.assertIsNotNone(mapstore.load_map(self.p._saved_map_name))

        # 「レーシングライン走行」を押す＝`request_load()`でLOCATEへ進む
        self.p.request_load(self.p._saved_map_name)
        self.assertEqual(self.p.phase, LOCATE)


class TestObstacleBrakeAndMdFault(unittest.TestCase):
    """障害物で始めた制動を足元で解かない／駆動 MD が無応答のときの抑え（実機 2026-10-08）。"""

    def setUp(self):
        from raspi.msgs.types import AutoState
        from raspi.nav import obstacles as obs_mod
        from raspi.nav.raceline import RaceLine, curvature
        from raspi.proto.generated import packets

        self.AutoState, self.Obstacle, self.packets = AutoState, obs_mod.Obstacle, packets
        self.p = Slam2dRaceLine()
        self.params = Slam2dRaceLine.merged({"obstacle_avoid": 1})
        n = 200
        a = np.arange(n) * 2 * math.pi / n
        xy = np.column_stack([5.0 * np.cos(a), 5.0 * np.sin(a)])       # 半径5m・点間 約16cm
        self.path = RaceLine(xy=xy, v=np.full(n, 2.0), kappa=curvature(xy),
                             length=2 * math.pi * 5.0)
        self.p._dt = 0.1

    def _vs(self, speed, md=(17, 17, 49)):
        return VehicleState(speed=speed, armed=True, md_status=list(md))

    def test_obstacle_under_the_nose_still_blocks(self):
        """base_link の 20cm 先・横 5cm（`blocking` が飛ばす足元の帯）でも塞がれ扱いになる。"""
        self.p._obs = [self.Obstacle(5.0 - 0.05, 0.20, 0.05, 5)]
        hit, dist = self.p._blocking(self.path, 0, self._vs(1.0), self.params)
        self.assertIsNotNone(hit)
        st = self.AutoState(target_speed=2.0)
        self.assertTrue(self.p._obstacle_response(st, hit, dist, self._vs(1.0), self.params,
                                                  self.path, 0))
        self.assertTrue(st.brake)
        self.assertEqual(st.target_speed, 0.0)

    def test_visible_points_decide_instead_of_the_fitted_circle(self):
        """重心と半径の円では当たるが、見えている点は車体から 10cm 以上離れている。"""
        xy = self.path.xy[1:4]
        pts = np.array([[5.0 - 0.20, 0.20], [5.0 - 0.21, 0.24], [5.0 - 0.22, 0.28]])
        self.assertGreater(self.p._clear_of(xy, self.Obstacle(4.90, 0.22, 0.12, 3, pts)), 0.05)
        self.assertLess(self.p._clear_of(xy, self.Obstacle(4.90, 0.22, 0.12, 3)), 0.0)

    def test_obstacle_beside_the_body_does_not_block(self):
        self.p._obs = [self.Obstacle(5.0 - 0.30, 0.20, 0.05, 5)]       # 横 30cm（当たらない）
        hit, _ = self.p._blocking(self.path, 0, self._vs(1.0), self.params)
        self.assertIsNone(hit)

    def test_brake_is_held_when_detection_drops_out(self):
        o = self.Obstacle(5.0, 0.6, 0.05, 5)
        st = self.AutoState(target_speed=2.0)
        self.p._obstacle_response(st, o, 0.6, self._vs(1.5), self.params, self.path, 0)
        self.assertTrue(st.brake)
        st = self.AutoState(target_speed=2.0)                          # 次の周期: 検出が消えた
        self.assertTrue(self.p._obstacle_response(st, None, math.inf, self._vs(1.2), self.params,
                                                  self.path, 0))
        self.assertTrue(st.brake)
        st = self.AutoState(target_speed=2.0)                          # 止まったら解く
        self.assertFalse(self.p._obstacle_response(st, None, math.inf, self._vs(0.0), self.params,
                                                   self.path, 0))
        self.assertFalse(st.brake)

    def test_one_md_down_caps_speed_and_both_down_stops(self):
        run = self.packets.MDS_RUNNING
        st = self.AutoState(target_speed=2.5)
        note = self.p._md_fault(st, self._vs(2.0, (17, run, 49)), self.params)
        self.assertIn("右後輪", note)
        self.assertEqual(st.target_speed, self.params["md_fault_speed"])
        st = self.AutoState(target_speed=2.5)
        self.assertIsNone(self.p._md_fault(st, self._vs(2.0, (run, run, 49)), self.params))
        self.assertEqual(st.target_speed, 0.0)
        st = self.AutoState(target_speed=2.5)                          # 戻れば元どおり
        self.assertEqual(self.p._md_fault(st, self._vs(2.0), self.params), "")
        self.assertEqual(st.target_speed, 2.5)

    def test_one_md_down_too_long_stops(self):
        run = self.packets.MDS_RUNNING
        self.p._md_t = self.params["md_fault_s"]
        st = self.AutoState(target_speed=2.5)
        self.assertIsNone(self.p._md_fault(st, self._vs(2.0, (run, 17, 49)), self.params))
        self.assertTrue(st.brake)

    def _launch(self, st, speed, **kw):
        return self.p._launch_ok(st, self._vs(speed), kw.get("hit"), kw.get("coasting", False),
                                 kw.get("md", ""), kw.get("params", {**self.params, "launch": 1}))

    def test_launch_is_off_by_default(self):
        """既定は無効（2026-10-10: 今は手動操作でだけ使う）。"""
        self.p._joined = True
        st = self.AutoState(target_speed=2.0, target_steer=0.02)
        self.assertFalse(self._launch(st, 0.0, params=self.params))

    def test_launch_is_requested_from_a_stop_until_the_path_speed(self):
        """ローンチコントロール（v0.20）: 止まってから経路の速度に届くまでの間だけ要求する。"""
        st = self.AutoState(target_speed=2.0, target_steer=0.02)
        self.p._joined = True
        self.assertFalse(self._launch(st, 1.0))                         # 周回中（止まっていない）
        self.assertTrue(self._launch(st, 0.0))                          # 止まった → 発進
        self.assertTrue(self._launch(st, 0.5))                          # 加速中も要求し続ける
        self.assertTrue(self._launch(st, 1.5))
        self.assertFalse(self._launch(st, 1.95))                        # 経路の速度に届いた
        self.assertFalse(self._launch(st, 1.2))                         # 次のコーナーの立ち上がりでは出さない

    def test_launch_waits_for_the_join_and_then_starts_while_moving(self):
        """走り出しは経路から外れていて `join_speed` で乗りに行く。乗った後に要求する。"""
        self.p._joined = False
        self.assertFalse(self._launch(self.AutoState(target_speed=0.5), 0.0))
        self.assertFalse(self._launch(self.AutoState(target_speed=0.5), 0.5))   # 抑えた速度に届いても終わらない
        self.p._joined = True
        self.assertTrue(self._launch(self.AutoState(target_speed=2.0), 0.5))

    def test_launch_is_requested_only_when_clear_and_straight(self):
        st = self.AutoState(target_speed=2.0, target_steer=0.02)
        self.p._joined = True
        self.assertTrue(self._launch(st, 0.0))
        self.assertFalse(self._launch(st, 0.0, hit=self.Obstacle(5.0, 0.6, 0.05, 5)))
        self.assertFalse(self._launch(st, 0.0, coasting=True))
        self.assertFalse(self._launch(st, 0.0, md="・★右後輪の MD が無応答"))
        self.assertFalse(self._launch(st, 0.0, params={**self.params, "launch": 0}))
        self.assertFalse(self._launch(self.AutoState(target_speed=2.0, target_steer=0.2), 0.0))
        self.assertTrue(self._launch(st, 0.6))                          # 条件が戻れば続きから

    def test_unreported_md_status_is_not_a_fault(self):
        st = self.AutoState(target_speed=2.5)
        self.assertEqual(self.p._md_fault(st, self._vs(2.0, (0, 0, 0)), self.params), "")


class TestKappaMax(unittest.TestCase):
    """最小旋回半径は幾何だけでなく、同定した舵の効きを通して決める。"""

    def test_default_is_geometric(self):
        from raspi.core.vehicle import Vehicle
        v = Vehicle()
        self.assertAlmostEqual(v.kappa_max, math.tan(v.max_steer) / v.wheelbase, places=6)

    def test_identified_steer_widens_min_radius(self):
        from dataclasses import replace

        from raspi.core.vehicle import Vehicle
        v = replace(Vehicle(), steer_gain=0.993, steer_gain_cubic=-0.392)
        self.assertAlmostEqual(1.0 / v.kappa_max, 0.46, delta=0.01)
        # 実舵角が指令に届かないぶんと、オフセットの不利な側も半径を広げる
        self.assertGreater(1.0 / replace(v, steer_servo_gain=0.87).kappa_max, 0.52)
        self.assertLess(replace(v, steer_offset_rad=0.01).kappa_max, v.kappa_max)
        self.assertEqual(replace(v, steer_offset_rad=-0.01).kappa_max,
                         replace(v, steer_offset_rad=0.01).kappa_max)


class TestForwardTraj(unittest.TestCase):
    def test_reverse_and_turn_in_place_are_dropped(self):
        # +x へ 1m 進み、0.5m 後退し、その場で向きを変え、+x へ 1.5m まで進み直す
        fwd = [(0.1 * i, 0.0, 0.0) for i in range(11)]
        back = [(1.0 - 0.1 * i, 0.0, 0.0) for i in range(1, 6)]
        spin = [(0.5, 0.0, 0.1 * k) for k in range(1, 4)] + [(0.5, 0.0, 0.0)]
        again = [(0.5 + 0.1 * i, 0.0, 0.0) for i in range(1, 11)]
        out = forward_traj(np.array(fwd + back + spin + again))
        self.assertTrue(np.all(np.diff(out[:, 0]) > 0))
        self.assertAlmostEqual(out[-1, 0], 1.5)
        self.assertEqual(len(out), 16)       # 0.0〜1.5 を 0.1 刻み（重複なし）

    def test_short_input_is_returned_as_is(self):
        one = np.array([[0.0, 0.0, 0.0]])
        self.assertIs(forward_traj(one), one)


class TestSlam2dNavDistance(unittest.TestCase):
    """`Slam2dNav.distance` の増分計算が、軌跡を全部足し直した値と一致するか。"""

    def test_incremental_matches_full_sum(self):
        from raspi.auto._slam2d_nav import Slam2dNav
        nav = Slam2dNav(resolution=0.05, size_m=4.0)
        traj = nav._fe.trajectory
        self.assertEqual(nav.distance, 0.0)
        rng = np.random.default_rng(0)
        for _ in range(3):
            for _ in range(10):
                traj.append(Pose2D(*rng.normal(size=3)))
            arr = np.asarray(traj)[:, :2]
            full = float(np.hypot(*np.diff(arr, axis=0).T).sum())
            self.assertAlmostEqual(nav.distance, full)
        # ループ閉じで軌跡のリストごと差し替わったら数え直す
        nav._fe.trajectory = [Pose2D(0, 0, 0), Pose2D(3, 4, 0)]
        self.assertAlmostEqual(nav.distance, 5.0)


class TestSlam2dNavDeskewScanCache(unittest.TestCase):
    """Issue #19: `deskew_scan()`が`update()`の脱スキューを使い回すこと。

    RACEでは`plan()`が`update()`のすぐ後に同じ`scan`を`_detect_obstacles`
    （`deskew_scan`経由）へも渡す。二重に`deskew_traj`を走らせないための
    キャッシュが、**同じscanでは同一の点群を返し**、**違うscanでは
    ちゃんと別の結果を返す**（古い結果を使い回さない）ことを確認する。
    """

    def test_same_scan_reuses_update_result_identically(self):
        from raspi.auto._slam2d_nav import Slam2dNav

        nav = Slam2dNav(resolution=0.05, size_m=8.0, lidar_x=0.05, lidar_y=0.0)
        scan = make_room_scan(1.0, 1.0, 0.0)
        nav.on_vehicle_state(VehicleState(t_capture=scan.t_capture - 90_000_000,
                                          speed=0.5, yaw_rate=0.1))
        nav.on_vehicle_state(VehicleState(t_capture=scan.t_capture,
                                          speed=0.5, yaw_rate=0.1))
        nav.update(scan, 0.1, yaw_rate=0.1, speed=0.5)

        cached = nav._fe.last_points
        self.assertIsNotNone(cached)
        reused = nav.deskew_scan(scan)
        # 同一オブジェクト（コピーでなく使い回し）であること
        self.assertIs(reused, cached)

    def test_different_scan_is_not_reused(self):
        from raspi.auto._slam2d_nav import Slam2dNav

        nav = Slam2dNav(resolution=0.05, size_m=8.0)
        scan1 = make_room_scan(1.0, 1.0, 0.0, t0=1_000_000_000)
        scan2 = make_room_scan(1.5, 1.2, 0.3, t0=1_300_000_000)
        nav.on_vehicle_state(VehicleState(t_capture=scan1.t_capture, speed=0.3, yaw_rate=0.0))
        nav.update(scan1, 0.1, yaw_rate=0.0, speed=0.3)
        after_scan1 = nav._fe.last_points

        pts2 = nav.deskew_scan(scan2)
        self.assertIsNot(pts2, after_scan1)
        self.assertNotEqual(pts2.t_ref_ns, after_scan1.t_ref_ns)


class TestSlam2dRaceLineSavedMapLoad(unittest.TestCase):
    """保存済み地図の読み込み（`request_load`）→ `LOCATE`（自己位置復元）→
    `RACE`までの統合テスト。`raspi/auto/mapstore.py`（保存側）と
    `slam2d.core.localize.GlobalLocalizer`（復元側）の両方が実際に噛み合うこと
    を確認する（それぞれの単体テストは`test_mapstore.py`/`slam2d/tests/test_localize.py`）。
    """

    def setUp(self):
        self._td = tempfile.TemporaryDirectory()
        self._orig_dir = mapstore.MAPS_DIR
        mapstore.MAPS_DIR = Path(self._td.name) / "saved_maps"

    def tearDown(self):
        mapstore.MAPS_DIR = self._orig_dir
        self._td.cleanup()

    def _drive_to_race(self) -> Slam2dRaceLine:
        """`test_full_state_machine_builds_saves_and_waits_in_done`と同じ軌道で
        `DONE`（BUILD完了・自動保存済み・レーシングライン走行待ち）まで到達させる。
        """
        p = Slam2dRaceLine()
        st = drive_one_lap_and_build(p, Slam2dRaceLine.merged({}))
        self.assertEqual(st.phase, DONE, st.reason)
        return p

    def _save_current_map(self, p: Slam2dRaceLine, name: str) -> None:
        am = p.snapshot()
        self.assertIsNotNone(am)
        trinary = unpack_trinary(am.cells, am.width, am.height)
        mapstore.save_map(
            name, resolution=am.resolution, origin_x=am.origin_x, origin_y=am.origin_y,
            trinary=trinary, centerline_xy=np.asarray(am.centerline, dtype=np.float64),
            raceline_xy=np.asarray(am.raceline, dtype=np.float64),
            raceline_v=np.asarray(am.raceline_v, dtype=np.float64))

    def test_request_load_enters_locate_phase_with_frozen_grid_and_path(self):
        p = self._drive_to_race()
        self._save_current_map(p, "course_a")

        fresh = Slam2dRaceLine()
        fresh.request_load("course_a")
        self.assertEqual(fresh.phase, LOCATE)
        self.assertTrue(fresh.slam.grid.frozen)
        self.assertIsNotNone(fresh.path)
        self.assertEqual(fresh.laps, 0)
        # 保存済み地図の読み込み後は`Frontend`直結（追跡専用）に切り替わり、
        # `SlamSystem`のバッチ最適化は経由しない（`_slam2d_nav.py`参照）
        self.assertFalse(fresh.slam._backend_active)

    def test_request_load_missing_map_leaves_phase_unchanged(self):
        fresh = Slam2dRaceLine()
        fresh.request_load("no_such_map")
        self.assertEqual(fresh.phase, EXPLORE)
        self.assertIn("読み込めない", fresh._load_error)

    def test_failed_load_does_not_start_driving(self):
        """★ 読めない地図を指定しても、engage 済みの EXPLORE（FTG の地図作成走行）を始めない。

        GUI の「レーシングライン走行」は読み込みと engage を同じメッセージで送る。
        """
        params = Slam2dRaceLine.merged({})
        scan = make_room_scan(3.0, 2.0, 0.0, segs=ROOM)
        p = Slam2dRaceLine()
        p.set_engaged(True)
        self.assertTrue(p.plan(scan, vs(), params, 0.1).ready)      # 素の EXPLORE は FTG で走る
        p.request_load("no_such_map")
        st = p.plan(scan, vs(), params, 0.1)
        self.assertFalse(st.ready)
        self.assertIn("読み込めない", st.reason)
        # 自動運転を解けば、人の手での地図作成に戻れる
        p.on_disengage()
        p.set_engaged(False)
        self.assertNotIn("読み込めない", p.plan(scan, vs(), params, 0.1).reason)

    def test_map_without_a_line_is_refused(self):
        self._save_manual_map("no_line", raceline_xy=np.zeros((0, 2)))
        p = Slam2dRaceLine()
        p.request_load("no_line")
        self.assertEqual(p.phase, EXPLORE)
        self.assertIn("レーシングラインが無い", p._load_error)
        st = p.plan(make_room_scan(3.0, 2.0, 0.0, segs=ROOM), vs(), Slam2dRaceLine.merged({}), 0.1)
        self.assertFalse(st.ready)

    def test_path_changes_are_published_on_a_frozen_map(self):
        """★ 地図が凍結した後に経路だけ変わっても `(map_seq, route_seq)` が変わる。

        `planning_node`・`telemetry_node` はこの組が変わったときだけ `auto/map` を流す。
        以前は BUILD でできたラインも、設定を変えて作り直した速度も配られなかった。
        """
        params = Slam2dRaceLine.merged({})
        p = Slam2dRaceLine()
        keys, lines = [], []
        orig = p.plan

        def plan(*a, **k):
            st = orig(*a, **k)
            m = p.snapshot()
            if (m.map_seq, m.route_seq) not in keys:
                keys.append((m.map_seq, m.route_seq))
                lines.append((st.phase, len(m.centerline) // 2, len(m.raceline) // 2))
            return st

        p.plan = plan
        st = drive_one_lap_and_build(p, params)
        self.assertEqual(st.phase, DONE, st.reason)
        # 最後に配られた版に、中心線とラインが載っている
        self.assertEqual(lines[-1][0], DONE)
        self.assertGreater(lines[-1][1], 20)
        self.assertGreater(lines[-1][2], 20)
        # 走行中に速度の設定を変えると、作り直した速度が配り直される
        name = p._saved_map_name
        p.request_load(name)
        p._line_job.result(timeout=60)
        st = None
        for _ in range(40):
            st = p.plan(make_room_scan(*_LAP_START, segs=ROOM), vs(), params, 0.1)
            if st.phase == RACE:
                break
        self.assertEqual(st.phase, RACE, st.reason)
        n = len(keys)
        p.plan(make_room_scan(*_LAP_START, segs=ROOM), vs(), {**params, "v_max": 0.8}, 0.1)
        self.assertGreater(len(keys), n)
        self.assertLessEqual(max(p.snapshot().raceline_v), 0.8 + 1e-9)

    def test_line_margin_change_redraws_the_line(self):
        """★ RACE 中に `line_margin` を変えたら、速度だけでなくラインの形も引き直す。"""
        params = Slam2dRaceLine.merged({})
        p = Slam2dRaceLine()
        st = drive_one_lap_and_build(p, params)
        self.assertEqual(st.phase, DONE, st.reason)
        p.request_load(p._saved_map_name)
        p._line_job.result(timeout=60)
        scan = make_room_scan(*_LAP_START, segs=ROOM)
        for _ in range(40):
            if p.plan(scan, vs(), params, 0.1).phase == RACE:
                break
        self.assertEqual(p.phase, RACE)
        before = p.path
        p.plan(scan, vs(), params, 0.1)
        self.assertIsNone(p._line_job)                     # 変えていなければ引き直さない
        wide = {**params, "line_margin": 0.35}
        p.plan(scan, vs(), wide, 0.1)
        self.assertIsNotNone(p._line_job)
        self.assertEqual(p._line_margin, 0.35)
        p._line_job.result(timeout=60)
        p.plan(scan, vs(), wide, 0.1)
        self.assertIsNone(p._line_job)                     # 同じ値では投げ直さない
        self.assertIsNot(p.path, before)                   # 引き直したラインに差し替わった

    def test_locate_does_not_run_the_tracker(self):
        """LOCATE の間は追跡（`slam.update`）を回さない。ヒントは RACE に入ったら使い切る。"""
        self._save_manual_map("course_b")
        p = Slam2dRaceLine()
        p.request_load("course_b")
        p.request_locate_hint(3.0, 2.0)
        updates = []
        orig = p.slam.update
        p.slam.update = lambda *a, **k: (updates.append(p.phase), orig(*a, **k))[1]
        params = Slam2dRaceLine.merged({})
        for _ in range(30):
            st = p.plan(make_room_scan(3.0, 2.0, 0.1, segs=ROOM), vs(), params, 0.1)
            if st.phase == RACE:
                break
        self.assertEqual(st.phase, RACE)
        self.assertEqual(updates, [])
        self.assertIsNone(p._loc_hint)
        st = p.plan(make_room_scan(3.0, 2.0, 0.1, segs=ROOM), vs(), params, 0.1)
        self.assertEqual(updates, [RACE])
        self.assertGreater(st.match_score, 0.5)

    def test_failed_auto_save_is_reported_in_done(self):
        params = Slam2dRaceLine.merged({})
        p = Slam2dRaceLine()
        blocker = Path(self._td.name) / "blocked"
        blocker.write_text("")                              # ファイルなので下にディレクトリを作れない
        mapstore.MAPS_DIR = blocker / "saved_maps"
        st = drive_one_lap_and_build(p, params)
        self.assertEqual(st.phase, DONE, st.reason)
        st = p.plan(make_room_scan(*_LAP_START, segs=ROOM), vs(), params, 0.1)
        self.assertNotIn("保存した", st.reason)
        self.assertIn("保存に失敗", st.reason)

    def _save_manual_map(self, name: str, raceline_xy=None) -> None:
        """`_drive_to_race()`（実際のEXPLORE走行によるSLAM追跡）は円弧軌道での
        追跡が本来の精度検証を意図していない合成シナリオのため、姿勢が地面
        真値から大きくずれうる（`_build_map`と違って`Frontend`が推測航法
        だけで動くため）。**ここでは自己位置復元の精度そのものを確かめたい**
        ので、`slam2d/tests/test_scanmatch.py`と同じ「既知姿勢から直接
        `integrate()`する」手順で、地面真値と完全に一致した地図を作って保存する。
        """
        g = OccGrid(resolution=0.05, size_m=12.0, origin=(-1.0, -1.0))
        for x, y, yaw in [(3.0, 2.0, 0.0), (3.05, 2.0, 0.05), (3.0, 2.05, -0.05),
                         (2.95, 2.0, 0.0), (3.0, 1.95, 0.02)]:
            g.integrate(make_room_points(x, y, yaw), Pose2D(x, y, yaw))
        g.freeze()
        if raceline_xy is None:
            raceline_xy = np.array([[2.0, 1.5], [4.0, 1.5], [4.0, 2.5], [2.0, 2.5]])
        mapstore.save_map(
            name, resolution=g.resolution, origin_x=g.origin[0], origin_y=g.origin[1],
            trinary=g.trinary(), centerline_xy=np.zeros((0, 2)),
            raceline_xy=np.asarray(raceline_xy, dtype=np.float64),
            raceline_v=np.ones(len(raceline_xy)))

    def test_locate_recovers_pose_and_reaches_race(self):
        self._save_manual_map("course_b")
        true_pose = (3.0, 2.0, 0.1)

        fresh = Slam2dRaceLine()
        fresh.request_load("course_b")
        fresh.request_locate_hint(true_pose[0], true_pose[1])   # 対称の疑いによる
        # やり直しを避け、収束を安定させる（GUIの「地図パネルをタップ」と同じ操作）
        params = Slam2dRaceLine.merged({})
        dt = 0.1

        st = None
        for _ in range(30):
            st = fresh.plan(make_room_scan(*true_pose, segs=ROOM), vs(), params, dt)
            if st.phase == RACE:
                break
        self.assertIsNotNone(st)
        self.assertEqual(st.phase, RACE)
        self.assertAlmostEqual(fresh.slam.pose.x, true_pose[0], delta=0.1)
        self.assertAlmostEqual(fresh.slam.pose.y, true_pose[1], delta=0.1)
        self.assertGreater(st.match_score, 0.5)

    def _race_on_line(self, params) -> Slam2dRaceLine:
        """車の居る (3.0, 2.0) を +x 向きに通る経路の地図で、RACE まで進めて経路に乗せる。"""
        xy = np.array([[1.0 + 0.1 * i, 2.0] for i in range(41)]
                      + [[5.0 - 0.1 * i, 3.0] for i in range(41)])
        self._save_manual_map("course_line", raceline_xy=xy)
        p = Slam2dRaceLine()
        p.request_load("course_line")
        p.request_locate_hint(3.0, 2.0)
        for _ in range(30):
            st = p.plan(make_room_scan(3.0, 2.0, 0.0, segs=ROOM), vs(), params, 0.1)
            if st.phase == RACE:
                break
        self.assertEqual(st.phase, RACE)
        p._joined = True
        return p

    def test_lost_coasts_on_dead_reckoning_then_stops(self):
        params = Slam2dRaceLine.merged({"coast_s": 0.5, "coast_speed": 0.4})
        p = self._race_on_line(params)
        weird = [(0.0, 0.0, 1.0, 3.0), (1.0, 3.0, -2.0, 1.0)]
        states = [p.plan(make_room_scan(3.0, 2.0, 0.0, segs=weird), vs(), params, 0.1)
                  for _ in range(8)]
        # 見失って 0.5s までは推測航法で経路を追い続ける（速度は上限まで）
        self.assertTrue(states[0].ready)
        self.assertLessEqual(states[0].target_speed, 0.4 + 1e-9)
        self.assertIn("推測航法", states[0].reason)
        # 超えたら止まる
        self.assertFalse(states[-1].ready)
        self.assertEqual(states[-1].target_speed, 0.0)
        self.assertIn("信用できない", states[-1].reason)
        # 自己位置が戻れば数え直して走る
        st = p.plan(make_room_scan(3.0, 2.0, 0.0, segs=ROOM), vs(), params, 0.1)
        self.assertTrue(st.ready)
        self.assertEqual(p._coast_t, 0.0)

    def test_coast_disabled_stops_immediately(self):
        params = Slam2dRaceLine.merged({"coast_s": 0.0})
        p = self._race_on_line(params)
        weird = [(0.0, 0.0, 1.0, 3.0), (1.0, 3.0, -2.0, 1.0)]
        st = p.plan(make_room_scan(3.0, 2.0, 0.0, segs=weird), vs(), params, 0.1)
        self.assertFalse(st.ready)

    def test_locate_hint_is_recorded_and_resets_in_progress_search(self):
        p = self._drive_to_race()
        self._save_current_map(p, "course_c")

        fresh = Slam2dRaceLine()
        fresh.request_load("course_c")
        params = Slam2dRaceLine.merged({})
        fresh.plan(make_room_scan(3.0, 2.0, math.pi / 2, segs=ROOM), vs(), params, 0.1)
        self.assertIsNotNone(fresh._localizer)

        fresh.request_locate_hint(3.0, 2.0)
        self.assertEqual(fresh._loc_hint, (3.0, 2.0))
        self.assertIsNone(fresh._localizer)   # ヒントで探索をやり直す


if __name__ == "__main__":
    unittest.main()
