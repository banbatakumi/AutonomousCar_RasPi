"""`core/scan2scan.py`（Point-to-Line ICP）のテスト。

Phase 3 完了条件: 平行な壁の合成コースで「観測できない方向は初期値のまま
残る」ことを数値確認する。
"""

import math
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from slam2d.core.scan2scan import IcpConfig, match_scans  # noqa: E402
from slam2d.core.types import Pose2D  # noqa: E402
from slam2d.tests.helpers import make_room_points  # noqa: E402

#: 水平な壁1枚だけ（無限に長い直線壁のつもり）。壁に沿った方向(x)の位置は
#: 点群の形だけからは決まらないはず。
#:
#: ★ 上下2本の壁（通路）にはしない——`_normals()`は点群が方位順に**連続した
#: 1本の弧**であることを前提に前後点で法線を作るため、視野内に上下2本の
#: 「別々の壁」の点が混在すると、有効点配列上でその境界に来る点（実際には
#: 方位が大きく離れているのに配列上は隣接する）の法線がでたらめになり、
#: この境界のノイズがtx方向にも間接的に漏れてcorridor problemの前提が
#: 崩れることを実測で確認した（`_normals`のdocstring「点群は方位順に並んで
#: いるので隣り合う添字は空間的にも隣」という前提が、視野が分断される
#: シーンでは成り立たない）。壁1枚なら有効点は連続した1つの弧になる
WALL = [(-500.0, 0.0, 500.0, 0.0)]

#: 部屋（4方向すべて壁あり）は全方向が観測可能
_ROOM_ISH = [(0.0, 0.0, 6.0, 0.0), (6.0, 0.0, 6.0, 4.0),
             (6.0, 4.0, 0.0, 4.0), (0.0, 4.0, 0.0, 0.0)]


class TestCorridorProblem(unittest.TestCase):
    def test_forward_position_stays_at_guess_not_true_value(self):
        """通路方向(x)は観測できないので、ICP結果はguessに留まり真値には寄らない。

        対応点は**方位**で探すため（`MAX_PAIR_M`のdocstring参照）、この手法は
        そもそも「1周期ぶんの小さな移動」を補正する設計であり、真値からの
        オフセットが大きいと対応点探索自体が破綻する。ここでは実運用に即した
        小さな移動量（推測航法が数cmの精度を持つ想定）でテストする。
        """
        prev = make_room_points(0.0, 1.0, 0.0, segs=WALL, max_range=8.0)
        true_dx = 0.10
        cur = make_room_points(true_dx, 1.0, 0.0, segs=WALL, max_range=8.0)
        wrong_guess = Pose2D(0.08, 0.0, 0.0)   # 推測航法の2cm誤差を模す

        d = match_scans(prev, cur, wrong_guess)
        # 観測できない方向(x)はguessのまま残る。真値0.10には寄らない
        self.assertAlmostEqual(d.dx, wrong_guess.x, delta=0.01)
        self.assertNotAlmostEqual(d.dx, true_dx, delta=0.01)

    def test_lateral_position_is_recovered_even_with_wrong_guess(self):
        """壁に垂直な方向(y)は観測できるので、誤ったguessからでもICPが正す。

        `match_scans`の戻り値`(dx,dy,dyaw)`は「carがprev基準で見てどれだけ
        動いたか」（そのままcur点群をprev座標系へ変換する量）。carが
        (0,1,0)→(0,1+true_dy,0)へ動いたなら、期待値は(0,+true_dy,0)。
        """
        prev = make_room_points(0.0, 1.0, 0.0, segs=WALL, max_range=8.0)
        true_dy = 0.1
        cur = make_room_points(0.0, 1.0 + true_dy, 0.0, segs=WALL, max_range=8.0)
        wrong_guess = Pose2D(0.0, 0.0, 0.0)  # yについては真値からずれた初期値

        d = match_scans(prev, cur, wrong_guess)
        self.assertAlmostEqual(d.dy, true_dy, delta=0.02)


class TestFullyObservableRoom(unittest.TestCase):
    def test_recovers_translation_and_rotation(self):
        """4方向すべてに壁がある部屋では、並進・回転とも正しく復元できる。"""
        prev = make_room_points(3.0, 2.0, 0.0, segs=_ROOM_ISH, max_range=8.0)
        true_delta = Pose2D(0.08, -0.03, math.radians(2.0))
        cur = make_room_points(3.0 + true_delta.x, 2.0 + true_delta.y,
                               true_delta.yaw, segs=_ROOM_ISH, max_range=8.0)
        guess = Pose2D(0.0, 0.0, 0.0)

        d = match_scans(prev, cur, guess, config=IcpConfig(prior_xy=0.05, prior_yaw=math.radians(3.0)))
        self.assertTrue(d.ok)
        self.assertAlmostEqual(d.dx, true_delta.x, delta=0.02)
        self.assertAlmostEqual(d.dy, true_delta.y, delta=0.02)
        self.assertAlmostEqual(d.dyaw, true_delta.yaw, delta=math.radians(1.0))


class TestInsufficientPoints(unittest.TestCase):
    def test_too_few_points_returns_not_ok(self):
        from slam2d.core.types import ScanPoints
        import numpy as np

        tiny = ScanPoints(np.array([1.0, 2.0]), np.array([0.0, 0.0]),
                          np.array([True, True]), 0, True)
        d = match_scans(tiny, tiny, Pose2D(0.1, 0.0, 0.0))
        self.assertFalse(d.ok)
        self.assertEqual(d.dx, 0.1)  # guessがそのまま返る


if __name__ == "__main__":
    unittest.main()
