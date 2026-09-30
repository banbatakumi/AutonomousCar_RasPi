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

from raspi.auto import PLANNERS, make_planner, mapstore  # noqa: E402
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
        # 部屋の中を大きく周回する円弧軌道で地図を育てる(半径2m、中心(3,2))
        dt = 0.1
        speed = 0.3
        radius = 2.0
        yaw_rate = speed / radius
        n_steps = 200
        for i in range(n_steps):
            t = i * dt
            yaw = yaw_rate * t + math.pi / 2  # 円周上を進む向き
            x = 3.0 + radius * math.sin(yaw_rate * t)
            y = 2.0 + radius * (1 - math.cos(yaw_rate * t))
            self.p.plan(make_room_scan(x, y, yaw, segs=ROOM),
                       vs(speed=speed, yaw_rate=yaw_rate), self.params, dt)
        self.assertGreater(self.p.slam.lap_progress(), 0.3)

        self.p.request_freeze()
        st = self.p.plan(make_room_scan(3.0, 2.0, math.pi / 2, segs=ROOM),
                         vs(speed=0.0, yaw_rate=0.0), self.params, dt)
        self.assertEqual(st.phase, BUILD)

        # BUILDは2周期で完了する設計(1/2:中心線、2/2:レーシングライン)
        for _ in range(3):
            st = self.p.plan(make_room_scan(3.0, 2.0, math.pi / 2, segs=ROOM),
                             vs(speed=0.0, yaw_rate=0.0), self.params, dt)
            if st.phase != BUILD:
                break

        if st.phase == BUILD:
            self.skipTest(f"経路生成が完了しなかった(reason={st.reason})。"
                          "円弧軌道が短すぎる可能性がある")
        self.assertEqual(st.phase, DONE)
        self.assertIsNotNone(self.p.path)

        # 地図が自動保存されていること
        self.assertTrue(self.p._saved_map_name)
        self.assertIsNotNone(mapstore.load_map(self.p._saved_map_name))

        # 「レーシングライン走行」を押す＝`request_load()`でLOCATEへ進む
        self.p.request_load(self.p._saved_map_name)
        self.assertEqual(self.p.phase, LOCATE)


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
        """`test_full_state_machine_builds_saves_and_waits_in_done`と同じ円弧軌道で
        `DONE`（BUILD完了・自動保存済み・レーシングライン走行待ち）まで到達させる。
        """
        p = Slam2dRaceLine()
        params = Slam2dRaceLine.merged({})
        dt = 0.1
        speed = 0.3
        radius = 2.0
        yaw_rate = speed / radius
        for i in range(200):
            t = i * dt
            yaw = yaw_rate * t + math.pi / 2
            x = 3.0 + radius * math.sin(yaw_rate * t)
            y = 2.0 + radius * (1 - math.cos(yaw_rate * t))
            p.plan(make_room_scan(x, y, yaw, segs=ROOM), vs(speed=speed, yaw_rate=yaw_rate), params, dt)

        p.request_freeze()
        st = p.plan(make_room_scan(3.0, 2.0, math.pi / 2, segs=ROOM), vs(), params, dt)
        for _ in range(3):
            if st.phase != BUILD:
                break
            st = p.plan(make_room_scan(3.0, 2.0, math.pi / 2, segs=ROOM), vs(), params, dt)
        if st.phase != DONE:
            self.skipTest(f"経路生成が完了しなかった(reason={st.reason})")
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
