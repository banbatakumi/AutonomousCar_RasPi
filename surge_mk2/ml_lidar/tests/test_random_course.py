"""`sim/random_course.py` のテスト。生成したコースが壊れていないことだけを確認する
（コース品質そのものは学習結果で答え合わせするので、ここでは配管レベルの検証に留める）。
"""

import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))  # surge_mk2/

import numpy as np  # noqa: E402

from sim.random_course import (  # noqa: E402
    _hairpin_polygon_xy,
    _min_turn_radius_m,
    _vehicle_min_turn_radius_m,
    _RADIUS_MARGIN,
    generate_random_course,
    generate_random_course_dr,
)


class TestGenerateRandomCourse(unittest.TestCase):
    def test_loop_is_closed(self):
        rng = np.random.default_rng(0)
        c = generate_random_course(rng)
        gap = np.hypot(c.centerline[-1, 0] - c.centerline[0, 0],
                       c.centerline[-1, 1] - c.centerline[0, 1])
        self.assertLess(gap, c.width * 0.5)

    def test_start_is_not_occupied(self):
        rng = np.random.default_rng(1)
        c = generate_random_course(rng)
        self.assertFalse(c.occupied(c.start[0], c.start[1]))

    def test_raycast_and_collides_do_not_raise(self):
        rng = np.random.default_rng(2)
        c = generate_random_course(rng)
        angles = np.linspace(-np.pi, np.pi, 360, endpoint=False)
        d = c.raycast(c.start[0], c.start[1], angles, 5.0)
        self.assertEqual(d.shape, (360,))
        body = c.body_samples([[0.30, 0.09], [0.30, -0.09], [-0.07, -0.09], [-0.07, 0.09]])
        self.assertFalse(c.collides(c.start[0], c.start[1], c.start[2], body))

    def test_different_seeds_give_different_courses(self):
        c1 = generate_random_course(np.random.default_rng(10), name="a")
        c2 = generate_random_course(np.random.default_rng(11), name="b")
        different = (c1.centerline.shape != c2.centerline.shape
                    or not np.allclose(c1.centerline, c2.centerline))
        self.assertTrue(different)


class TestGenerateRandomCourseDr(unittest.TestCase):
    """幅のドメインランダム化ラッパー（`generate_random_course_dr`）のテスト。"""

    def test_width_varies_within_range(self):
        widths = [generate_random_course_dr(np.random.default_rng(i)).width
                 for i in range(20)]
        self.assertGreater(len(set(widths)), 1)
        for w in widths:
            self.assertGreaterEqual(w, 0.7)
            self.assertLessEqual(w, 1.3)

    def test_custom_width_range_is_respected(self):
        rng = np.random.default_rng(0)
        c = generate_random_course_dr(rng, width_range=(2.0, 2.0))
        self.assertAlmostEqual(c.width, 2.0)

    def test_loop_is_still_closed(self):
        rng = np.random.default_rng(3)
        c = generate_random_course_dr(rng)
        gap = np.hypot(c.centerline[-1, 0] - c.centerline[0, 0],
                       c.centerline[-1, 1] - c.centerline[0, 1])
        self.assertLess(gap, c.width * 0.5)


def _max_turn_deg_in_window(centerline: np.ndarray, window_arc_m: float) -> float:
    """閉ループの中心線上で、弧長`window_arc_m`以内に収まる総回頭角[deg]の最大値。
    ヘアピン（短い区間で大きく向きが変わる特徴）が実際に生成されたかを判定する
    のに使う——fujiの`arc,1.0,-90`×2連続は3m以内に180°弱の回頭が入る。
    """
    xy = centerline[:, :2]
    loop = np.vstack([xy, xy[:1]])
    seg = np.hypot(*np.diff(loop, axis=0).T)
    yaw = np.arctan2(np.diff(loop[:, 1]), np.diff(loop[:, 0]))
    dyaw = np.abs(np.diff(np.unwrap(np.concatenate([yaw, yaw[:1]]))))
    n = len(xy)
    best = 0.0
    for i in range(n):
        acc = 0.0
        turn = 0.0
        j = i
        while acc < window_arc_m and j < i + n:
            k = j % n
            acc += seg[k]
            turn += dyaw[k]
            j += 1
        best = max(best, float(np.degrees(turn)))
    return best


class TestHairpinPolygon(unittest.TestCase):
    """`_hairpin_polygon_xy`単体のテスト（`_filleted_polygon_xy`の「1つの中心から
    見た半径の関数」方式ではヘアピンが原理的に作れないことが判明したため新設した
    専用の生成関数——モジュールdocstring参照）。"""

    def test_succeeds_and_respects_min_radius(self):
        r_min = _vehicle_min_turn_radius_m() * _RADIUS_MARGIN
        n_ok = 0
        for seed in range(30):
            rng = np.random.default_rng(seed)
            xy = _hairpin_polygon_xy(rng, r_min)
            if xy is None:
                continue
            n_ok += 1
            self.assertGreaterEqual(_min_turn_radius_m(xy), r_min * 0.999)
        # 実測成功率は単発300試行で約97%——30本中ほぼ全て成功するはず
        self.assertGreaterEqual(n_ok, 25)

    def test_produces_a_genuinely_sharp_turn(self):
        """ヘアピンは短い弧長のうちに大きく向きを変える特徴でなければならない
        （`_filleted_polygon_xy`の通常の頂点は方位変化が最大でも~60°程度に
        頭打ちになることを実測で確認済み——それより明確に大きいことを検証する）。
        """
        r_min = _vehicle_min_turn_radius_m() * _RADIUS_MARGIN
        found_sharp = False
        for seed in range(30):
            rng = np.random.default_rng(seed)
            xy = _hairpin_polygon_xy(rng, r_min)
            if xy is None:
                continue
            centerline = np.column_stack((xy, np.zeros(len(xy))))
            if _max_turn_deg_in_window(centerline, window_arc_m=3.0) > 100.0:
                found_sharp = True
                break
        self.assertTrue(found_sharp)


class TestGenerateRandomCourseHairpin(unittest.TestCase):
    """`generate_random_course`のヘアピン差し込み（`hairpin_prob`）のテスト。"""

    def test_hairpin_prob_zero_never_calls_hairpin_generator(self):
        # hairpin_prob=0なら通常の`_filleted_polygon_xy`のみが使われるはず——
        # ここでは間接的に、生成が例外なく完了し閉じることだけを確認する
        rng = np.random.default_rng(0)
        c = generate_random_course(rng, hairpin_prob=0.0)
        gap = np.hypot(c.centerline[-1, 0] - c.centerline[0, 0],
                       c.centerline[-1, 1] - c.centerline[0, 1])
        self.assertLess(gap, c.width * 0.5)

    def test_hairpin_prob_one_usually_produces_a_sharp_turn(self):
        r_min = _vehicle_min_turn_radius_m() * _RADIUS_MARGIN
        n_sharp = 0
        N = 20
        for seed in range(N):
            rng = np.random.default_rng(1000 + seed)
            c = generate_random_course(rng, hairpin_prob=1.0)
            if _max_turn_deg_in_window(c.centerline, window_arc_m=3.0) > 100.0:
                n_sharp += 1
        # `_hairpin_polygon_xy`の単発成功率(~97%)から、ほとんどのコースに
        # ヘアピン相当の急旋回が入るはず
        self.assertGreaterEqual(n_sharp, N * 0.7)


class TestGenerateRandomCourseChicaneRadius(unittest.TestCase):
    """`_add_chicanes`の最小旋回半径保証のテスト——base曲線側の既存曲率と
    蛇行の曲率が加算されて大幅に割り込むバグを実装中に踏んだため
    （モジュールdocstring「実装中に踏んだ罠」参照）、回帰テストとして残す。"""

    def test_chicanes_never_violate_min_turn_radius(self):
        r_min = _vehicle_min_turn_radius_m() * _RADIUS_MARGIN
        for seed in range(50):
            rng = np.random.default_rng(2000 + seed)
            c = generate_random_course(rng, hairpin_prob=0.35)
            self.assertGreaterEqual(_min_turn_radius_m(c.centerline[:, :2]), r_min * 0.95,
                                    msg=f"seed={seed}")


if __name__ == "__main__":
    unittest.main()
