"""`raspi/nav/scan2scan.py`（点対線ICP）と`raspi/nav/scan_anchor.py`のテスト。

`match_scans()`は`raspi/nav/slam.py`から使われていたが**単体テストが無かった**。
`park_to_point`の駐車目標アンカリングがこれに依存するので、先に性質を縛る:

1. 既知の並進・回頭を復元する（初期値が間違っていてもデータが勝つ）
2. **観測できない方向は初期値に留まる**（平行壁の通路で進行方向が暴れない）
   ——ティホノフ正則化の存在意義そのもの
3. 遮蔽で`inlier`が下がる（＝信用判定が機能する）

`test_nav.py`の`make_room_scan()`を再利用する（同じ約束の`Scan`を作る）。
"""

import math
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from raspi.nav import deskew  # noqa: E402
from raspi.nav.scan2scan import match_scans  # noqa: E402
from raspi.nav.scan_anchor import ScanAnchor  # noqa: E402
from raspi.nav.se2 import compose  # noqa: E402

from .test_nav import ROOM, make_room_scan  # noqa: E402

#: 平行な壁だけの通路。**進行方向（x）が観測できない**配置
CORRIDOR = [
    (-10.0, 0.0, 10.0, 0.0),
    (-10.0, 1.0, 10.0, 1.0),
]


def _pts(x, y, yaw, *, segs=ROOM, missing=()):
    """姿勢`(x,y,yaw)`から見た脱スキュー済み点群（base_link座標）。"""
    return deskew(make_room_scan(x, y, yaw, segs=segs, missing=missing),
                  max_range=8.0)


class TestMatchScans(unittest.TestCase):
    def test_recovers_translation_despite_wrong_guess(self):
        """初期値を0にしても、データ（約300点）が事前分布（σ=5mm）に勝つ。"""
        ref = _pts(3.0, 2.0, 0.0)
        cur = _pts(3.15, 2.0, 0.0)
        d = match_scans(ref, cur, (0.0, 0.0, 0.0))
        self.assertTrue(d.ok)
        self.assertAlmostEqual(d.dx, 0.15, delta=0.02)
        self.assertAlmostEqual(d.dy, 0.0, delta=0.02)
        self.assertAlmostEqual(d.dyaw, 0.0, delta=math.radians(1.5))

    def test_recovers_rotation(self):
        ref = _pts(3.0, 2.0, 0.0)
        cur = _pts(3.0, 2.0, math.radians(10))
        d = match_scans(ref, cur, (0.0, 0.0, 0.0))
        self.assertTrue(d.ok)
        self.assertAlmostEqual(d.dyaw, math.radians(10), delta=math.radians(1.5))

    def test_recovers_both(self):
        ref = _pts(3.0, 2.0, 0.0)
        cur = _pts(3.12, 1.94, math.radians(-8))
        d = match_scans(ref, cur, (0.0, 0.0, 0.0))
        self.assertTrue(d.ok)
        self.assertAlmostEqual(d.dx, 0.12, delta=0.03)
        self.assertAlmostEqual(d.dy, -0.06, delta=0.03)
        self.assertAlmostEqual(d.dyaw, math.radians(-8), delta=math.radians(2))

    def test_corridor_keeps_guess_along_unobservable_axis(self):
        """平行壁だけの通路では進行方向が幾何的に決まらない。

        **勝手に0へ潰れてもいけないし、暴れてもいけない**——初期値
        （推測航法）が残るのが正しい（`scan2scan.py`のティホノフ正則化）。
        横方向（壁が拘束する方向）は逆にデータで直る。
        """
        ref = _pts(0.0, 0.5, 0.0, segs=CORRIDOR)
        cur = _pts(0.20, 0.44, 0.0, segs=CORRIDOR)
        # 進行方向の初期値は真値、横方向はわざと外す
        d = match_scans(ref, cur, (0.20, 0.0, 0.0))
        self.assertTrue(d.ok)
        self.assertAlmostEqual(d.dx, 0.20, delta=0.03)      # 初期値が残る
        self.assertAlmostEqual(d.dy, -0.06, delta=0.02)     # データで直る

    def test_corridor_does_not_invent_forward_motion(self):
        """初期値を0にしたら進行方向は0のまま（嘘の前進を作らない）。"""
        ref = _pts(0.0, 0.5, 0.0, segs=CORRIDOR)
        cur = _pts(0.20, 0.50, 0.0, segs=CORRIDOR)
        d = match_scans(ref, cur, (0.0, 0.0, 0.0))
        self.assertLess(abs(d.dx), 0.05)

    def test_new_object_lowers_inlier(self):
        """`inlier`は「現在の点のうち相手が見つかった割合」。

        **欠測（点が消える）では下がらない**——残った点は相手を見つけるため。
        下がるのは「基準に無かったものが現れた」場合で、そちらが信用判定として
        意味を持つケース（人が入った・目標を取り違えた等）。
        """
        ref = _pts(3.0, 2.0, 0.0)
        clean = match_scans(ref, _pts(3.05, 2.0, 0.0), (0.05, 0.0, 0.0))
        board = ROOM + [(3.8, 1.6, 3.8, 2.4)]        # 前方0.8mに無かった板
        appeared = match_scans(ref, _pts(3.05, 2.0, 0.0, segs=board),
                               (0.05, 0.0, 0.0))
        self.assertLess(appeared.inlier, clean.inlier)

    def test_missing_sectors_do_not_lower_inlier(self):
        """欠測で`inlier`が下がらないことを明示的に縛る（上の裏返し）。

        ★`ScanAnchor`が`inlier`だけで信用判定すると、**点が激減しても
        気付けない**。点数の下限は`match_scans`側の`MIN_POINTS`が持っている。
        """
        ref = _pts(3.0, 2.0, 0.0)
        occl = match_scans(ref, _pts(3.05, 2.0, 0.0, missing=(0, 1, 2, 3, 4, 5)),
                           (0.05, 0.0, 0.0))
        self.assertGreater(occl.inlier, 0.9)

    def test_too_few_points_is_not_ok(self):
        ref = _pts(3.0, 2.0, 0.0)
        thin = _pts(3.0, 2.0, 0.0, missing=tuple(range(11)))
        d = match_scans(ref, thin, (0.0, 0.0, 0.0))
        self.assertFalse(d.ok)


class TestScanAnchor(unittest.TestCase):
    def setUp(self):
        self.a = ScanAnchor()

    def test_no_reference_returns_guess(self):
        u = self.a.update(_pts(3.0, 2.0, 0.0), (0.1, 0.2, 0.3))
        self.assertFalse(u.ok)
        self.assertEqual(u.pose, (0.1, 0.2, 0.3))

    def test_same_scan_is_origin(self):
        ref = _pts(3.0, 2.0, 0.0)
        self.a.set_reference(ref)
        u = self.a.update(ref, (0.0, 0.0, 0.0))
        self.assertTrue(u.ok)
        self.assertLess(math.hypot(*u.pose[:2]), 0.01)

    def test_corrects_dead_reckoning_error(self):
        """推測航法が過小に進んだと言っても、真値へ引き直すこと。

        ★補正できる幅は`max_pair`（対応点として認める距離、既定8cm）で
        決まる。**それを超える初期誤差は直せない**——対応点が1つも作れない
        ので登録が成立しない。毎周期の誤差はmm級なので運用上は足りるが、
        「大きく飛んだ推定を1回で引き戻す」ことはできない（`max_pair`を
        広げると誤対応で精度が落ちる。同フィールドのdocstring参照）。
        """
        self.a.set_reference(_pts(3.0, 2.0, 0.0))
        u = self.a.update(_pts(3.08, 2.0, 0.0), (0.05, 0.0, 0.0))
        self.assertTrue(u.ok)
        self.assertAlmostEqual(u.pose[0], 0.08, delta=0.015)
        self.assertGreater(u.correction_m, 0.02)

    def test_error_beyond_max_pair_is_not_corrected(self):
        """`max_pair`を超える初期誤差では登録が成立しないこと（上の裏返し）。"""
        self.a.set_reference(_pts(3.0, 2.0, 0.0))
        u = self.a.update(_pts(3.30, 2.0, 0.0), (0.0, 0.0, 0.0))
        self.assertFalse(u.ok)

    def test_tracks_a_long_run_without_accumulating(self):
        """**姿勢は常に原点フレームで返り、長い走行でも真値に追従すること。**

        推測航法が毎周期25%過大に進んでも、登録がその都度引き戻す。
        キーフレームは内部で進むが、返る姿勢のフレームは変わらない。
        """
        self.a.set_reference(_pts(3.0, 2.0, 0.0))
        dr = (0.0, 0.0, 0.0)
        for i in range(1, 13):
            true_x = 3.0 + 0.04 * i
            dr = (dr[0] + 0.04 + 0.01, dr[1], dr[2])
            u = self.a.update(_pts(true_x, 2.0, 0.0), dr)
            self.assertTrue(u.ok, f"i={i} で登録が採用されなかった")
            dr = u.pose
        self.assertAlmostEqual(dr[0], 0.48, delta=0.03)
        self.assertAlmostEqual(dr[1], 0.0, delta=0.02)

    def test_garbage_is_rejected_and_keyframe_is_relinked(self):
        """全く違う形のスキャンは採用せず、続いたらキーフレームを繋ぎ直す。"""
        self.a.set_reference(_pts(3.0, 2.0, 0.0))
        good = self.a.update(_pts(3.05, 2.0, 0.0), (0.05, 0.0, 0.0))
        self.assertTrue(good.ok)
        garbage = _pts(0.3, 0.5, 0.0, segs=CORRIDOR)
        u = None
        before = self.a.keyframes
        for _ in range(self.a.reanchor_after):
            u = self.a.update(garbage, (0.10, 0.0, 0.0))
            self.assertFalse(u.ok)
        # 姿勢は推測航法のまま返る（嘘の補正をしない）
        self.assertEqual(u.pose, (0.10, 0.0, 0.0))
        self.assertGreater(self.a.keyframes, before)

    def test_large_correction_is_rejected(self):
        """初期値から極端に離れた解は捨てる（ICP破綻の保険）。"""
        a = ScanAnchor(max_correction=0.02)
        a.set_reference(_pts(3.0, 2.0, 0.0))
        u = a.update(_pts(3.20, 2.0, 0.0), (0.0, 0.0, 0.0))
        self.assertFalse(u.ok)
        self.assertEqual(u.pose, (0.0, 0.0, 0.0))

    def test_keyframe_advances_on_long_motion(self):
        """原点から離れるとキーフレームが進むこと（1リンクを短く保つ）。"""
        self.a.set_reference(_pts(3.0, 2.0, 0.0))
        dr = (0.0, 0.0, 0.0)
        for i in range(1, 12):
            dr = (0.03 * i, 0.0, 0.0)
            u = self.a.update(_pts(3.0 + 0.03 * i, 2.0, 0.0), dr)
            dr = u.pose
        self.assertGreater(self.a.keyframes, 0)


if __name__ == "__main__":
    unittest.main()
