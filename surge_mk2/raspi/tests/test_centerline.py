"""中心線生成（`raspi/nav/centerline.py`）のテスト。

★ 主眼は「壁の小さな穴を抜けて奥の壁を拾った異常値が、中心線を暴れさせない」
こと（`centerline.py` の docstring「壁の小さな穴が中心線を暴れさせる」参照）。
"""

import math
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

import numpy as np  # noqa: E402

from raspi.nav import centerline as cl  # noqa: E402
from raspi.nav.grid import OccGrid  # noqa: E402


def straight_corridor(width: float = 0.6, length: float = 4.0, res: float = 0.05,
                       gap: tuple[float, float] | None = None) -> OccGrid:
    """x 方向にまっすぐ伸びる幅 `width` の廊下。壁は直接 `hits` へ焼く。

    :param gap: `(x0, x1)` を渡すと上側の壁のその区間だけ穴を空け、
        代わりにずっと奥（無関係な壁）を置く。**`OccGrid.raycast()` の既定
        `fill=1`（1セル=`res`分の穴埋め）では塞ぎきれない広さにする**こと。
    """
    g = OccGrid(resolution=res, size_m=8.0, origin=(-1.0, -2.0))
    y_bottom, y_top = -width / 2.0, width / 2.0
    xs = np.arange(0.0, length, res / 2.0)
    for x in xs:
        col, row = g.to_cell(x, y_bottom)
        g.hits[row, col] = g.min_hits
        if gap and gap[0] <= x <= gap[1]:
            continue
        col, row = g.to_cell(x, y_top)
        g.hits[row, col] = g.min_hits
    if gap:
        far_y = y_top + 1.5           # 穴の奥、本来のレーンとは無関係な壁
        for x in xs:
            if gap[0] <= x <= gap[1]:
                col, row = g.to_cell(x, far_y)
                g.hits[row, col] = g.min_hits
    return g


class TestDeclutter(unittest.TestCase):
    def test_clamps_a_lone_spike_to_the_neighbourhood(self):
        width = np.full(21, 0.3)
        width[10] = 2.5                       # 穴を抜けて奥の壁を拾った1点
        out = cl._declutter(width)
        self.assertLess(out[10], 0.5)

    def test_leaves_other_points_unchanged(self):
        width = np.full(21, 0.3)
        width[10] = 2.5
        out = cl._declutter(width)
        self.assertTrue(np.array_equal(np.delete(out, 10), np.delete(width, 10)))

    def test_does_not_touch_a_genuinely_wide_stretch(self):
        """本物の広い区間（コース出口など）は連続して広い＝中央値も高いので残す。"""
        width = np.full(21, 0.3)
        width[8:13] = 2.0                     # 5点連続で広い＝本物の開けた場所
        out = cl._declutter(width)
        self.assertTrue(np.allclose(out[8:13], 2.0))

    def test_never_widens_a_genuinely_narrow_point(self):
        """狭い側へは倒さない。本当に狭い場所を誤って広げてはいけない。"""
        width = np.full(21, 0.3)
        width[10] = 0.05                      # 局所的に狭い＝本物かもしれない
        out = cl._declutter(width)
        self.assertAlmostEqual(float(out[10]), 0.05)

    def test_short_array_is_a_no_op(self):
        width = np.array([0.3, 2.5])
        out = cl._declutter(width)
        self.assertTrue(np.array_equal(out, width))


class TestMeasureRobustToHoles(unittest.TestCase):
    """`measure()` は `_declutter()` を通すので、穴を抜けた1点も暴れない。"""

    def test_ignores_a_small_hole_in_the_wall(self):
        width = 0.6
        g = straight_corridor(width=width, gap=(1.0, 1.1))
        xy = np.column_stack([np.arange(0.2, 3.8, 0.1),
                              np.zeros(36)])
        nrm = cl.normals(xy)
        left, right = cl.measure(g, xy, nrm, max_width=3.0)
        i = int(np.argmin(np.abs(xy[:, 0] - 1.05)))
        # 穴を抜けた生の値なら 1.5m 級（far_y ぶん）になるはず。
        # 近傍の中央値へ均されて、本来の壁までの距離（0.3m）に近いこと
        self.assertLess(max(float(left[i]), float(right[i])), width,
                        "穴を抜けた異常値がそのまま残っている")

    def test_a_real_wide_opening_is_reported_as_wide(self):
        """本物の開けた場所（穴ではなく、実際に壁が無い区間）は削らない。"""
        width = 0.6
        g = straight_corridor(width=width, gap=(1.0, 2.0))   # 1m ぶん本当に壁が無い
        xy = np.column_stack([np.arange(0.2, 3.8, 0.1), np.zeros(36)])
        nrm = cl.normals(xy)
        # `normals()` は進行方向(+x)を+90°回した向き（+y＝上側の壁）を返すので、
        # 上側の穴は `left` 側に出る
        left, _right = cl.measure(g, xy, nrm, max_width=3.0)
        i = int(np.argmin(np.abs(xy[:, 0] - 1.5)))           # 開口部のど真ん中
        self.assertGreater(float(left[i]), width)


if __name__ == "__main__":
    unittest.main()
