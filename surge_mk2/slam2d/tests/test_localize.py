"""`core/localize.py`（グローバルローカリゼーション）のテスト。

確認すること:
- 非対称な部屋（`helpers.ROOM`）なら、手がかり無しでも既知姿勢へ収束すること
- クリックヒントで探索範囲を絞れること
- 左右対称（同じ形が複数箇所にある）な地図では`ambiguous`が立つこと
"""

import math
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

import numpy as np  # noqa: E402

from slam2d.core.grid import OccGrid  # noqa: E402
from slam2d.core.localize import GlobalLocalizer, LocalizeConfig  # noqa: E402
from slam2d.core.types import Pose2D, ScanPoints  # noqa: E402
from slam2d.tests.helpers import ROOM, make_room_points  # noqa: E402


def _build_room_grid() -> OccGrid:
    g = OccGrid(resolution=0.05, size_m=12.0, origin=(-1.0, -1.0))
    # 部屋の中を少し動き回って地図を育てる（`test_scanmatch.py`と同じ手順）
    for x, y, yaw in [(3.0, 2.0, 0.0), (3.05, 2.0, 0.05), (3.0, 2.05, -0.05),
                      (2.95, 2.0, 0.0), (3.0, 1.95, 0.02)]:
        g.integrate(make_room_points(x, y, yaw), Pose2D(x, y, yaw))
    g.freeze()
    return g


def _run_to_done(localizer: GlobalLocalizer, pts: ScanPoints) -> None:
    guard = 0
    while not localizer.done:
        localizer.step(pts)
        guard += 1
        if guard > 10_000:
            raise AssertionError("GlobalLocalizer が収束しない（無限ループの疑い）")


class TestGlobalLocalizerUnambiguousRoom(unittest.TestCase):
    """`ROOM`は中の板で左右非対称にしてある（`helpers.py`docstring参照）ので、
    手がかり無しの全探索でも一意に決まるはず。"""

    def setUp(self):
        self.g = _build_room_grid()

    #: **既定の`spacing=0.3m`は実運用の16m級コース向けの粗さ**——6x4mの
    #: この合成テスト部屋にそのまま使うと、量子化誤差で離れた候補が偶然
    #: 僅差になり`ambiguous`が誤検出される（実測済み）。`test_scanmatch.py`が
    #: `DEFAULT_STAGES`をテスト用に広げているのと同じ理由で、部屋の大きさに
    #: 合わせて細かく刻む
    _FINE = LocalizeConfig(spacing=0.1, angle_step=math.radians(10.0))

    def test_recovers_known_pose_without_hint(self):
        true_pose = Pose2D(3.0, 2.0, 0.1)
        pts = make_room_points(*true_pose)

        loc = GlobalLocalizer(self.g, hint=None, config=self._FINE)
        _run_to_done(loc, pts)
        r = loc.result

        self.assertFalse(r.ambiguous)
        self.assertAlmostEqual(r.x, true_pose.x, delta=0.15)
        self.assertAlmostEqual(r.y, true_pose.y, delta=0.15)
        self.assertAlmostEqual(math.degrees(r.yaw), math.degrees(true_pose.yaw), delta=20.0)
        self.assertGreater(r.score, 0.4)

    def test_hint_narrows_candidates_and_still_converges(self):
        true_pose = Pose2D(3.0, 2.0, 0.1)
        pts = make_room_points(*true_pose)

        loc_wide = GlobalLocalizer(self.g, hint=None, config=self._FINE)
        loc_hint = GlobalLocalizer(self.g, hint=(true_pose.x, true_pose.y), config=self._FINE)
        self.assertLess(loc_hint._xs.size, loc_wide._xs.size)

        _run_to_done(loc_hint, pts)
        r = loc_hint.result
        self.assertAlmostEqual(r.x, true_pose.x, delta=0.15)
        self.assertAlmostEqual(r.y, true_pose.y, delta=0.15)

    def test_hint_far_from_any_free_cell_yields_no_candidates(self):
        loc = GlobalLocalizer(self.g, hint=(-50.0, -50.0),
                              config=LocalizeConfig(hint_radius=0.3))
        self.assertTrue(loc.done)
        r = loc.result
        self.assertTrue(r.ambiguous)
        self.assertEqual(r.score, 0.0)

    def test_progress_reaches_one_when_done(self):
        pts = make_room_points(3.0, 2.0, 0.1)
        loc = GlobalLocalizer(self.g, hint=None, config=LocalizeConfig())
        _run_to_done(loc, pts)
        self.assertAlmostEqual(loc.progress, 1.0)


def _stamp_l_shape(g: OccGrid, *, col0: int, row0: int) -> list[tuple[int, int]]:
    """`(col0, row0)`を起点にL字の壁を1つ焼き、周囲を「空き」にする。
    壁セルの(col, row)一覧を返す。"""
    cells = [(col0 + i, row0) for i in range(5)] + [(col0, row0 + i) for i in range(1, 5)]
    for c, r in cells:
        g.hits[r, c] = g.min_hits
    for r in range(row0 - 3, row0 + 8):
        for c in range(col0 - 3, col0 + 8):
            if 0 <= r < g.height and 0 <= c < g.width and g.hits[r, c] == 0:
                g.misses[r, c] = g.min_seen
    return cells


class TestGlobalLocalizerAmbiguousSymmetry(unittest.TestCase):
    """同じ形（L字の壁）を離れた2箇所に置いた地図。**並進対称の最小構成。**"""

    def setUp(self):
        self.res = 0.05
        self.g = OccGrid(resolution=self.res, size_m=6.0)
        self.col_a, self.row_a = 20, 20
        self.col_b, self.row_b = 90, 20   # 70セル=3.5m離れた複製
        cells_a = _stamp_l_shape(self.g, col0=self.col_a, row0=self.row_a)
        _stamp_l_shape(self.g, col0=self.col_b, row0=self.row_b)
        self.g.freeze()
        self.g.seq = 1

        # L字を「起点の少し外側」から見た局所点群（yaw=0固定）。もう一方の複製でも
        # 同じ相対配置なので同点になるはず
        self.qx, self.qy = (float(v) for v in self.g.to_world(self.col_a - 2, self.row_a - 2))
        self.qx_b, self.qy_b = (float(v) for v in self.g.to_world(self.col_b - 2, self.row_b - 2))
        xs, ys = [], []
        for c, r in cells_a:
            wx, wy = self.g.to_world(c, r)
            xs.append(wx - self.qx)
            ys.append(wy - self.qy)
        self.pts = ScanPoints(np.array(xs), np.array(ys),
                              np.ones(len(xs), dtype=bool), 0, True)

    def test_ambiguous_when_both_copies_visible(self):
        loc = GlobalLocalizer(self.g, hint=None,
                              config=LocalizeConfig(spacing=0.1, angle_step=math.radians(15.0)))
        _run_to_done(loc, self.pts)
        r = loc.result
        self.assertTrue(r.ambiguous)

    def test_hint_near_one_copy_resolves_ambiguity(self):
        """ヒントで片方の複製に絞れば`ambiguous`は消え、**もう一方より明らかに
        ヒント側に近い**候補が選ばれる（9点のL字だけの合成テストなので、
        sub-cm精度の一致までは要求しない——粗探索は`_locate()`側の仕上げ
        マッチングに引き継ぐ設計）。"""
        loc = GlobalLocalizer(self.g, hint=(self.qx, self.qy),
                              config=LocalizeConfig(spacing=0.05, angle_step=math.radians(15.0),
                                                    hint_radius=1.0))
        _run_to_done(loc, self.pts)
        r = loc.result
        self.assertFalse(r.ambiguous)
        d_a = math.hypot(r.x - self.qx, r.y - self.qy)
        d_b = math.hypot(r.x - self.qx_b, r.y - self.qy_b)
        self.assertLess(d_a, d_b)
        self.assertLess(d_a, 1.0)


if __name__ == "__main__":
    unittest.main()
