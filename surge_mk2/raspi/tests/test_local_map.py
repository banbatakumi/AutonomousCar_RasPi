"""`raspi/nav/local_map.py`（局所占有格子＋ESDF＋円被覆）のテスト。

縛る性質は3つ:

1. **円被覆が矩形を覆い、かつはみ出しが小さいこと。** 覆えていなければ
   衝突を見逃す（危険側）。はみ出しが大きすぎると、通れる駐車枠を
   「入らない」と判定する（実際に1列被覆で前後6.4cmはみ出し、斜め駐車が
   1/8になった）
2. **未知を空きと混ぜないこと。** 混ぜると見えていない場所へ楽観的に
   経路を引く
3. **`ready`が`min_hits`を待つこと。** 待たずに計画すると、まだ壁が
   確定していない地図（＝全部空き）に対して経路を引いてしまう
"""

import math
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from raspi.nav import deskew  # noqa: E402
from raspi.nav.local_map import LocalMap, footprint_circles  # noqa: E402

from .test_nav import make_room_scan  # noqa: E402

#: `config/vehicle.toml` の実測値と同じ形
FOOTPRINT = [(0.30, 0.09), (0.30, -0.09), (-0.07, -0.09), (-0.07, 0.09)]
#: 4m×4mの部屋（原点中心）。壁の位置が分かっているので距離を数値で縛れる
ROOM4 = [
    (-2.0, -2.0, 2.0, -2.0),
    (2.0, -2.0, 2.0, 2.0),
    (2.0, 2.0, -2.0, 2.0),
    (-2.0, 2.0, -2.0, -2.0),
]


def _feed(lmap, pose=(0.0, 0.0, 0.0), n=3, segs=ROOM4):
    for _ in range(n):
        pts = deskew(make_room_scan(*pose, segs=segs, max_range=6.0), max_range=6.0)
        lmap.integrate(pts, pose)
    lmap.refresh()


class TestFootprintCircles(unittest.TestCase):
    def test_circles_cover_the_rectangle(self):
        """矩形内の全点がどれかの円に入ること（見逃しが無いこと）。"""
        centers, r = footprint_circles(FOOTPRINT, 5, 3)
        xs = [x / 100.0 for x in range(-7, 31)]
        ys = [y / 100.0 for y in range(-9, 10)]
        for x in xs:
            for y in ys:
                d = min(math.hypot(x - cx, y - cy) for cx, cy in centers)
                self.assertLessEqual(d, r + 1e-9, f"({x},{y})がどの円にも入らない")

    def test_protrusion_is_small(self):
        """はみ出しが前後・左右とも2cm以下であること。"""
        centers, r = footprint_circles(FOOTPRINT, 5, 3)
        dx = 0.37 / 5
        dy = 0.18 / 3
        self.assertLess(r - dx / 2, 0.02)
        self.assertLess(r - dy / 2, 0.02)

    def test_single_row_protrudes_a_lot(self):
        """★1列（ny=1）では前後に6cm以上はみ出す——採用してはいけない理由。"""
        _, r = footprint_circles(FOOTPRINT, 6, 1)
        self.assertGreater(r - (0.37 / 6) / 2, 0.06)


class TestReadiness(unittest.TestCase):
    def test_not_ready_before_min_hits(self):
        lmap = LocalMap(footprint=FOOTPRINT, min_hits=2)
        _feed(lmap, n=1)
        self.assertFalse(lmap.ready, "1周だけで ready になってはいけない")
        _feed(lmap, n=1)
        self.assertTrue(lmap.ready)

    def test_clearance_is_zero_before_refresh(self):
        lmap = LocalMap(footprint=FOOTPRINT)
        self.assertEqual(float(lmap.clearance(0.0, 0.0)), 0.0)


class TestClearance(unittest.TestCase):
    def setUp(self):
        self.lmap = LocalMap(footprint=FOOTPRINT, resolution=0.01, size_m=6.0)
        _feed(self.lmap)

    def test_distance_to_a_known_wall(self):
        """原点は4m部屋の中心なので、壁まで2m。格子解像度ぶんの誤差で一致する。"""
        self.assertAlmostEqual(float(self.lmap.clearance(0.0, 0.0)), 2.0, delta=0.03)

    def test_closer_to_the_wall_is_smaller(self):
        near = float(self.lmap.clearance(1.5, 0.0))
        self.assertAlmostEqual(near, 0.5, delta=0.03)

    def test_unknown_is_not_free(self):
        """観測していない領域（部屋の外）は未知。**壁とは別枠で塞がれている。**"""
        outside = (2.5, 0.0)
        # 壁だけを見る距離場では「壁の向こう側」も距離を持つ
        self.assertGreater(float(self.lmap.clearance(*outside)), 0.0)
        # 未知を含めると塞がれている
        self.assertEqual(float(self.lmap.clearance(*outside, include_unknown=True)), 0.0)

    def test_grid_border_is_blocked(self):
        """★格子の外へ距離場が伸びないこと。伸びると「端に寄れば余裕がある」
        と誤答し、地図の外へ経路を引く。"""
        edge = self.lmap.size_m / 2.0 - 0.005
        self.assertLess(float(self.lmap.clearance(edge, 0.0)), 0.02)

    def test_body_clearance_scalar_and_array_agree(self):
        import numpy as np
        one = float(self.lmap.body_clearance(0.5, 0.2, 0.3))
        arr = self.lmap.body_clearance(np.array([0.5]), np.array([0.2]),
                                       np.array([0.3]))
        self.assertAlmostEqual(one, float(arr[0]), places=9)

    def test_body_clearance_accounts_for_heading(self):
        """車体は前に長いので、壁に向いているときの方が余裕が小さい。"""
        facing = float(self.lmap.body_clearance(1.6, 0.0, 0.0))        # 壁を向く
        sideways = float(self.lmap.body_clearance(1.6, 0.0, math.pi / 2))
        self.assertLess(facing, sideways)

    def test_body_clearance_is_negative_when_straddling_a_wall(self):
        """車体が壁セルを跨いでいれば負になること。

        ★壁の**向こう側**（x=2.1、車体全体が壁の外）では負にならない——
        壁は1セルの厚みしかなく、その先は「未知」なので壁までの距離場は
        正の値を返す。未知を塞ぐのは`include_unknown=True`の役目。
        """
        # 前端(base_link+0.30)が壁(x=2.0)を越える位置
        self.assertLess(float(self.lmap.body_clearance(1.95, 0.0, 0.0)), 0.0)
        # 壁の向こう側は「未知」として塞がれている
        self.assertAlmostEqual(
            float(self.lmap.body_clearance(2.4, 0.0, 0.0, include_unknown=True)),
            -self.lmap.circle_radius, places=6)


if __name__ == "__main__":
    unittest.main()
