"""`raspi/nav/hybrid_astar.py`（非ホロノミック格子探索）のテスト。

**「経路が見つかる」だけでなく「見つかった経路が実際に通れる」ことを縛る。**
返る経路は`(gear, curvature, length)`の列なので、`sample_path_array()`で
掃引して`LocalMap`のクリアランスを測れば、計画と同じ基準で検算できる。

縛る性質:

1. Reeds-Shepp候補では解けない配置（奥まった車庫）で**経路を見つける**
2. 切り返しが必要な配置で**前後進が混ざった経路**を返す
3. 返した経路の最小クリアランスが、**そのとき使った硬い拘束`margin`以上**
   ——ここが食い違うと、呼び出し側の安全判定と噛み合わず制動と再計画が
   交代して永久に進まない（実機でこの症状を実測した）
4. 目標が壁に埋まっていれば**即座に失敗を返す**（探索上限まで粘らない）
5. **時間予算を守る**（`planning_node`の周期ループを止めない）
"""

import math
import sys
import time
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from raspi.core.vehicle import Vehicle  # noqa: E402
from raspi.nav import deskew  # noqa: E402
from raspi.nav.hybrid_astar import HybridConfig, plan  # noqa: E402
from raspi.nav.local_map import LocalMap  # noqa: E402
from raspi.nav.reeds_shepp import (  # noqa: E402
    candidate_paths, sample_path_array,
)

from .test_nav import make_room_scan  # noqa: E402

VEH = Vehicle.load()
FOOTPRINT = list(VEH.footprint)
#: 計画に使う旋回半径（実舵角の9割。追従の補正余地を残す）
R_PLAN = VEH.wheelbase / math.tan(0.9 * VEH.max_steer)

ROOM = [
    (-2.5, -2.5, 2.5, -2.5),
    (2.5, -2.5, 2.5, 2.5),
    (2.5, 2.5, -2.5, 2.5),
    (-2.5, 2.5, -2.5, -2.5),
]
#: 車庫（通路0.6m幅＋間口0.5mの奥まった枠）。**RSの固定半径の弧では
#: 通路の壁を掠めてしまい解けない**配置
GARAGE = [
    (-2.5, 0.55, 2.5, 0.55),          # 通路の向かい側の壁
    (-2.5, -0.05, -0.25, -0.05),      # 縁石（間口の左）
    (0.25, -0.05, 2.5, -0.05),        # 縁石（間口の右）
    (-0.25, -0.05, -0.25, -0.75),     # 車庫の左壁
    (0.25, -0.05, 0.25, -0.75),       # 車庫の右壁
    (-0.25, -0.75, 0.25, -0.75),      # 車庫の奥
]
#: 幅1mの行き止まり通路（切り返し必須）
DEADEND = [
    (-2.5, -0.5, 2.0, -0.5),
    (-2.5, 0.5, 2.0, 0.5),
    (2.0, -0.6, 2.0, 0.6),
]


def _map(segs, pose=(0.0, 0.0, 0.0), n=3):
    lmap = LocalMap(footprint=FOOTPRINT, resolution=0.01, size_m=6.0)
    for _ in range(n):
        pts = deskew(make_room_scan(*pose, segs=segs, max_range=6.0), max_range=6.0)
        lmap.integrate(pts, pose)
    lmap.refresh()
    return lmap


def _cfg(**kw):
    kw.setdefault("time_budget_s", 2.0)
    return HybridConfig(turning_radius=R_PLAN, **kw)


def _rs_alone_works(lmap, start, goal, margin):
    """Reeds-Shepp候補だけで衝突しない経路が見つかるか（旧方式の再現）。"""
    for cand in candidate_paths(start, goal, R_PLAN):
        if not cand.segments:
            continue
        if lmap.path_clearance(sample_path_array(start, cand, step=0.03)) >= margin:
            return True
    return False


class TestFindsPaths(unittest.TestCase):
    def test_open_space_straight(self):
        lmap = _map(ROOM)
        res = plan((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), lmap, _cfg())
        self.assertTrue(res.ok, res.how)
        self.assertTrue(all(s.gear > 0 for s in res.path.segments),
                        "開けた直線で後退が混ざるのは無駄")

    def test_solves_a_garage_that_rs_candidates_cannot(self):
        """★Reeds-Shepp候補では解けない配置で経路を見つけること。

        これが Hybrid A* を入れた理由そのもの（シム実測で車庫入れの
        回避経路発見率が 43% → 100% になった）。
        """
        start = (-0.9, 0.25, 0.0)
        goal = (0.0, -0.60, math.pi / 2)
        lmap = _map(GARAGE, pose=start)
        cfg = _cfg()
        self.assertFalse(_rs_alone_works(lmap, start, goal, cfg.margin),
                         "この配置はRS候補で解けてしまう（テストの前提が崩れている）")
        res = plan(start, goal, lmap, cfg)
        self.assertTrue(res.ok, res.how)

    def test_dead_end_requires_a_gear_change(self):
        """行き止まりで向きを反転する配置では前後進が混ざること。"""
        lmap = _map(DEADEND, pose=(1.2, 0.0, 0.0))
        res = plan((1.2, 0.0, 0.0), (0.0, 0.0, math.pi), lmap, _cfg())
        self.assertTrue(res.ok, res.how)
        gears = {s.gear for s in res.path.segments}
        self.assertEqual(gears, {1, -1}, "切り返しが入っていない")


class TestPathIsActuallyDrivable(unittest.TestCase):
    """★**返した経路の余裕が、使った`margin`以上であること。**

    ここが守られないと、呼び出し側の安全判定が同じ地図で「食い込む」と
    言い出し、制動と再計画が交代して車両が一歩も進まなくなる。
    """

    def _check(self, segs, start, goal):
        lmap = _map(segs, pose=start)
        res = plan(start, goal, lmap, _cfg())
        self.assertTrue(res.ok, res.how)
        got = lmap.path_clearance(sample_path_array(start, res.path, step=0.01))
        self.assertGreaterEqual(got, res.margin - 1e-6,
                                f"余裕{got * 100:.2f}cm < margin{res.margin * 100:.2f}cm")
        self.assertAlmostEqual(got, res.clearance, delta=0.01)

    def test_open_space(self):
        self._check(ROOM, (0.0, 0.0, 0.0), (1.0, 0.8, math.radians(60)))

    def test_garage(self):
        self._check(GARAGE, (-0.9, 0.25, 0.0), (0.0, -0.60, math.pi / 2))

    def test_dead_end(self):
        self._check(DEADEND, (1.2, 0.0, 0.0), (0.0, 0.0, math.pi))


class TestGoalHandling(unittest.TestCase):
    def test_goal_buried_in_a_wall_fails_immediately(self):
        lmap = _map(ROOM)
        # 前端が壁(x=2.5)を5cm越える位置＝車体が壁セルを跨いでいる
        res = plan((0.0, 0.0, 0.0), (2.25, 0.0, 0.0), lmap, _cfg())
        self.assertFalse(res.ok)
        self.assertEqual(res.expansions, 0, "壁の中の目標で探索してはいけない")
        self.assertIn("重なって", res.how)

    def test_tight_goal_relaxes_the_margin(self):
        """**壁に詰める駐車では、必要なぶんだけマージンを緩めること。**

        目標の余裕が`margin`より小さい配置でマージンを緩めなければ、
        目標へ到達する経路が構造的に存在せず、探索上限まで無駄に粘る。
        """
        lmap = _map(ROOM)
        # 壁(x=2.5)から前端が3cmの位置＝目標の余裕は margin(5cm) より小さい
        goal = (2.5 - 0.30 - 0.03, 0.0, 0.0)
        cfg = _cfg()
        goal_clear = float(lmap.body_clearance(*goal))
        self.assertLess(goal_clear, cfg.margin, "テストの前提が崩れている")
        res = plan((0.0, 0.0, 0.0), goal, lmap, cfg)
        self.assertTrue(res.ok, res.how)
        self.assertLess(res.margin, cfg.margin)


class TestBudget(unittest.TestCase):
    def test_time_budget_is_respected(self):
        """解けない配置でも予算内で返ること（周期ループを止めない）。"""
        lmap = _map(DEADEND, pose=(1.2, 0.0, 0.0))
        # 通路の外＝未知の彼方にある目標。硬い拘束を厳しくして解けなくする
        cfg = _cfg(time_budget_s=0.08, margin=0.20)
        t0 = time.perf_counter()
        res = plan((1.2, 0.0, 0.0), (1.2, 2.0, 0.0), lmap, cfg)
        dt = time.perf_counter() - t0
        self.assertFalse(res.ok)
        self.assertLess(dt, 0.5, f"予算80msに対して{dt * 1000:.0f}msかかった")

    def test_unready_map_is_reported(self):
        lmap = LocalMap(footprint=FOOTPRINT)
        res = plan((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), lmap, _cfg())
        self.assertFalse(res.ok)
        self.assertIn("地図", res.how)


if __name__ == "__main__":
    unittest.main()
