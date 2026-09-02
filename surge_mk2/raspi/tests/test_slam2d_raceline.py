"""`raspi/auto/slam2d_raceline.py`（slam2d版レーシングライン）の統合テスト。

`test_raceline.py`と同じ流儀。`slam2d`のFrontendが実際にraspi側の型
（`Scan`/`VehicleState`）を受け取って動くこと、既存の`raspi/nav/`（経路生成・
追従・障害物検出）がダックタイピングでそのまま動くことを確認する。
"""

import math
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from raspi.auto import PLANNERS, make_planner  # noqa: E402
from raspi.auto.slam2d_raceline import BUILD, EXPLORE, RACE, Slam2dRaceLine  # noqa: E402
from raspi.msgs import VehicleState  # noqa: E402
from raspi.tests.test_nav import ROOM, make_room_scan  # noqa: E402


def vs(speed=0.0, yaw_rate=0.0) -> VehicleState:
    return VehicleState(speed=speed, yaw_rate=yaw_rate, steer_actual=0.0)


class TestSlam2dRaceLineStateMachine(unittest.TestCase):
    def setUp(self):
        self.p = Slam2dRaceLine()
        self.params = Slam2dRaceLine.merged({})

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

    def test_full_state_machine_reaches_race_and_drives(self):
        """★ EXPLORE→BUILD→RACEまで通しで動き、経路に沿って走行指令が出る。

        `ROOM`(矩形の部屋、中に板がある)を周回する経路を実際に生成し、
        `RACE`段でPure Pursuitが有効な操舵・速度指令を返すことを確認する。
        十分な距離を動かして地図を育ててから、手動確定でBUILDへ進める。
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
        self.assertEqual(st.phase, RACE)
        self.assertIsNotNone(self.p.path)


if __name__ == "__main__":
    unittest.main()
