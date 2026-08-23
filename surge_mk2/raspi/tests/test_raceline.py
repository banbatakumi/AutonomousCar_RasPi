"""レーシングライン planner（`raspi/auto/raceline.py`）の状態機械のテスト。

`test_auto.py` と同じ流儀で**「走る」より「止まる」を厚く試す**。段の遷移は
不可逆（EXPLORE → BUILD → RACE）なので、**遷移してはいけない場面で遷移しない**
ことの方が、遷移することより大事。

点群は `test_nav.py` の `make_room_scan` を使い回す（部屋の中を走る想定）。
"""

import math
import sys
import time
import unittest
import zlib
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

import numpy as np  # noqa: E402

from raspi.auto import PLANNERS, make_planner  # noqa: E402
from raspi.auto.raceline import BUILD, EXPLORE, RACE, RaceLine  # noqa: E402
from raspi.msgs import AutoMap, VehicleState  # noqa: E402
from raspi.nav.grid import OccGrid, pack_trinary  # noqa: E402
from raspi.tests.test_nav import make_room_scan  # noqa: E402


def vs(speed=0.0, yaw_rate=0.0) -> VehicleState:
    return VehicleState(speed=speed, yaw_rate=yaw_rate, steer_actual=0.0)


def unpack(cells: bytes, n: int) -> np.ndarray:
    """GUI（`gui/src/ws/map.ts`）と**同じ手順**で戻す。ビット順が合っているか。"""
    packed = np.frombuffer(zlib.decompress(cells), dtype=np.uint8)
    out = np.empty(n, dtype=np.uint8)
    for i in range(n):
        out[i] = (packed[i >> 2] >> ((i & 3) * 2)) & 3
    return out


class TestPacking(unittest.TestCase):
    """★ 2bit の詰め方が GUI の展開手順と一致していること。"""

    def test_roundtrip(self):
        g = OccGrid(resolution=0.05, size_m=2.0)
        g.hits[:8, :8] = 5                      # 占有
        g.misses[10:16, 10:16] = 5              # 空き
        t = g.trinary()
        back = unpack(pack_trinary(t), t.size).reshape(t.shape)
        self.assertTrue(np.array_equal(t, back))

    def test_all_three_values_survive(self):
        t = np.array([[0, 1, 2, 1, 0, 2]], dtype=np.uint8)
        self.assertTrue(np.array_equal(unpack(pack_trinary(t), t.size), t.ravel()))

    def test_length_not_multiple_of_four(self):
        """**端数があっても壊れない。** 400×400 は割り切れるが、将来変わりうる。"""
        t = np.array([[2, 1, 0]], dtype=np.uint8)
        self.assertTrue(np.array_equal(unpack(pack_trinary(t), 3), t.ravel()))


class TestStateMachine(unittest.TestCase):
    def setUp(self):
        self.p = RaceLine()
        self.params = RaceLine.merged({})

    def drive(self, n=10, dt=0.1, *, move=0.0):
        """`move` [m/周期] だけ前進しながら食わせる。

        **止まったままでは地図が育たない**（`nav/slam.py` のキーフレーム条件）ので、
        地図が要るテストは `move` を与えること。
        """
        out = None
        for i in range(n):
            pose = (3.0 + move * i, 2.0, 0.0)
            out = self.p.plan(make_room_scan(*pose), vs(speed=move / dt),
                              self.params, dt)
        return out

    def test_registered(self):
        self.assertIn(RaceLine.id, PLANNERS)
        self.assertIsInstance(make_planner("raceline"), RaceLine)

    def test_starts_in_explore(self):
        self.assertEqual(self.p.phase, EXPLORE)

    def test_without_vehicle_state_it_does_not_run(self):
        """`vs` が無い ＝ ジャイロが来ていない。**推測で埋めない。**"""
        st = self.p.plan(make_room_scan(3.0, 2.0, 0.0), None, self.params, 0.1)
        self.assertFalse(st.ready)
        self.assertIn("車両状態", st.reason)

    def test_explore_delegates_to_follow_the_gap(self):
        """EXPLORE 中は FTG が走らせる。**ギャップの情報がそのまま出る。**"""
        st = self.drive(6, move=0.02)
        self.assertEqual(st.phase, EXPLORE)
        self.assertIn("地図作成", st.reason)
        # FTG が選んだギャップが載っている（`AutoState` の既定 0,0 のままではない）
        self.assertNotEqual((st.gap_start_deg, st.gap_end_deg), (0.0, 0.0))

    def test_does_not_leave_explore_without_a_lap(self):
        """★止まったまま何周期回しても BUILD に行かない。

        累積回頭も走行距離も増えていないのだから、周回は終わっていない。
        """
        self.drive(40, move=0.02)
        self.assertEqual(self.p.phase, EXPLORE)
        self.assertEqual(self.p.laps, 0)

    def test_close_loop_runs_on_every_lap_not_only_the_last(self):
        """★ 2周目まで待たず、周回を検出するたびに `close_loop()` を呼ぶ。

        以前は `laps >= explore_laps` のときしか周回終了と判定せず、
        `close_loop()` も同じ条件でしか呼ばれなかった。`explore_laps=2` だと
        1周目のループ閉じが永遠に走らず、2周ぶんのドリフトを貯めてから
        1回だけ閉じることになり、oval コースの地図が「渦巻き」になっていた
        （`docs/progress_archive.md`「oval が渦巻きになる」節）。
        """
        self.drive(6, move=0.02)
        calls = []
        orig = self.p.slam.close_loop
        self.p.slam.close_loop = lambda *a, **k: calls.append(1) or orig(*a, **k)

        # 1周目だけ検出（探索そのものはまだ終わらない）→ BUILD へは行かない
        self.p._check_lap = lambda p: (True, False)
        st = self.drive(1)
        self.assertEqual(len(calls), 1)
        self.assertEqual(st.phase, EXPLORE)
        self.assertEqual(self.p.phase, EXPLORE)

        # 最終周（探索終了）→ BUILD へ。この周のループ閉じも呼ばれる
        self.p._check_lap = lambda p: (True, True)
        st = self.drive(1)
        self.assertEqual(len(calls), 2)
        self.assertEqual(st.phase, BUILD)

    def test_manual_freeze_moves_to_build(self):
        """「地図を確定」は**自動判定の逃げ道**。押したら次の周期で BUILD。"""
        self.drive(14, move=0.02)
        self.p.request_freeze()
        st = self.drive(1)
        self.assertEqual(st.phase, BUILD)
        self.assertTrue(self.p.slam.grid.frozen)

    def test_build_brakes_and_reports_progress(self):
        """BUILD 中は毎周期 `ready=False`（＝制動）で、進み具合が reason に出る。"""
        self.drive(14, move=0.02)
        self.p.request_freeze()
        st = self.drive(1)
        self.assertFalse(st.ready)
        self.assertEqual(st.phase, BUILD)
        self.assertIn("経路", st.reason)

    def test_build_failure_does_not_raise(self):
        """★経路を作れなくても例外を投げない。**理由が GUI に出る**方が大事。

        落ちると「engage したのに何も起きない」だけが残り、原因が分からない。
        """
        self.drive(3)                            # 軌跡が短すぎる状態で確定させる
        self.p.request_freeze()
        for _ in range(4):
            st = self.drive(1)
        self.assertFalse(st.ready)
        self.assertEqual(st.phase, BUILD)
        self.assertIn("作れなかった", st.reason)

    def test_race_without_a_path_does_not_run(self):
        self.p.phase = RACE
        st = self.drive(1)
        self.assertFalse(st.ready)
        self.assertIn("経路", st.reason)

    def test_reset_clears_everything(self):
        self.drive(14, move=0.02)
        self.p.request_freeze()
        self.drive(2)
        self.p.reset()
        self.assertEqual(self.p.phase, EXPLORE)
        self.assertEqual(self.p.laps, 0)
        self.assertIsNone(self.p.path)
        self.assertIsNone(self.p.centerline)
        self.assertFalse(self.p.slam.grid.frozen)
        self.assertEqual(int(self.p.slam.grid.hits.sum()), 0)
        self.assertEqual(self.p.slam.trajectory, [])

    def test_gyro_is_used(self):
        """★ `vs.yaw_rate` が SLAM に渡っていること（配線が切れていないか）。"""
        for _ in range(5):
            self.p.plan(make_room_scan(3.0 + 0.02 * _, 2.0, 0.0),
                        vs(yaw_rate=0.2, speed=0.2), self.params, 0.1)
        self.assertGreater(self.p.slam.gyro_updates, 0)

    def test_lidar_only_hides_the_gyro_and_speed_from_slam(self):
        """★ `lidar_only=1` なら、ジャイロ/`speed` が来ていても SLAM には渡さない。

        IMU/オドメトリへの依存を切り分ける実験用トグル。ジャイロが実際に
        使われたかは `Slam.gyro_updates` に出る（`test_gyro_is_used` の逆）。
        """
        params = RaceLine.merged({"lidar_only": 1})
        for i in range(5):
            self.p.plan(make_room_scan(3.0 + 0.02 * i, 2.0, 0.0),
                        vs(yaw_rate=0.2, speed=0.2), params, 0.1)
        self.assertEqual(self.p.slam.gyro_updates, 0)

    def test_lidar_only_switches_on_scan_to_scan(self):
        """`lidar_only` はジャイロ/`speed` を隠すだけでなく、そのぶんの動き推定を
        `scan_to_scan`（LiDAR 同士の直接照合）に肩代わりさせる。"""
        self.assertFalse(self.p.slam.cfg.scan_to_scan)
        self.p.plan(make_room_scan(3.0, 2.0, 0.0), vs(speed=0.2),
                    RaceLine.merged({"lidar_only": 1}), 0.1)
        self.assertTrue(self.p.slam.cfg.scan_to_scan)

        self.p.plan(make_room_scan(3.02, 2.0, 0.0), vs(speed=0.2),
                    RaceLine.merged({"lidar_only": 0}), 0.1)
        self.assertFalse(self.p.slam.cfg.scan_to_scan)

    def test_plan_stays_within_the_cycle_budget(self):
        """1周期が長すぎると `auto/cmd` が途絶し、中継側が制動に落とす。

        （`test_auto.py` の `test_stale_auto_cmd_becomes_a_brake` が守っている経路。
        安全側ではあるが `reason` が出ないので原因が読めなくなる。）
        """
        self.drive(5, move=0.02)
        t0 = time.perf_counter()
        self.drive(5, move=0.02)
        ms = (time.perf_counter() - t0) / 5 * 1000
        self.assertLess(ms, 60.0, f"1周期 {ms:.0f}ms は遅すぎる")


class TestSnapshot(unittest.TestCase):
    """`Planner.snapshot()` は**表示専用**。指令に影響してはいけない。"""

    def setUp(self):
        self.p = RaceLine()
        self.params = RaceLine.merged({})

    def test_none_before_any_scan(self):
        m = self.p.snapshot()
        # 地図が空でも形は返す（GUI 側で「まだ何も無い」と描けるように）
        self.assertIsInstance(m, AutoMap)
        self.assertEqual(m.width, self.p.slam.grid.width)

    def test_cells_decode_to_the_grid(self):
        for i in range(14):
            self.p.plan(make_room_scan(3.0 + 0.02 * i, 2.0, 0.0),
                        vs(speed=0.2), self.params, 0.1)
        m = self.p.snapshot()
        back = unpack(m.cells, m.width * m.height).reshape(m.height, m.width)
        self.assertTrue(np.array_equal(back, self.p.slam.grid.trinary()))
        self.assertGreater(int((back == 2).sum()), 50)

    def test_seq_advances_only_when_the_map_changes(self):
        for i in range(12):
            self.p.plan(make_room_scan(3.0 + 0.02 * i, 2.0, 0.0),
                        vs(speed=0.2), self.params, 0.1)
        a = self.p.snapshot()
        b = self.p.snapshot()
        self.assertEqual(a.map_seq, b.map_seq)   # 呼んだだけでは増えない

    def test_frozen_map_is_rebuilt_immediately(self):
        """★凍結した瞬間だけは間引きを無視して作り直す。

        確定した地図が GUI に出ないと、走り出してから「どんな地図で走っている
        のか」を確かめる手段が無くなる。
        """
        for i in range(14):
            self.p.plan(make_room_scan(3.0 + 0.02 * i, 2.0, 0.0),
                        vs(speed=0.2), self.params, 0.1)
        before = self.p.snapshot().map_seq
        self.p.slam.freeze()
        self.assertNotEqual(self.p.snapshot().map_seq, before)

    def test_origin_matches_the_grid(self):
        """原点がずれると GUI の地図と経路が重ならない。"""
        m = self.p.snapshot()
        g = self.p.slam.grid
        self.assertAlmostEqual(m.origin_x, g.origin[0])
        self.assertAlmostEqual(m.origin_y, g.origin[1])
        self.assertAlmostEqual(m.resolution, g.resolution)


if __name__ == "__main__":
    unittest.main()
