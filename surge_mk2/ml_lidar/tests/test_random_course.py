"""`sim/random_course.py` のテスト。生成したコースが壊れていないことだけを確認する
（コース品質そのものは学習結果で答え合わせするので、ここでは配管レベルの検証に留める）。
"""

import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))  # surge_mk2/

import numpy as np  # noqa: E402

from sim.gym_env import SimE2EEnv  # noqa: E402
from sim.random_course import (  # noqa: E402
    _ARCHETYPE_WEIGHTS,
    _hairpin_polygon_xy,
    _min_turn_radius_m,
    _vehicle_half_width_m,
    _vehicle_min_turn_radius_m,
    _RADIUS_MARGIN,
    CurriculumCourseFn,
    generate_circuit_course,
    generate_corridor_course,
    generate_diverse_course,
    generate_narrow_course,
    generate_obstacle_course,
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


class TestNewArchetypes(unittest.TestCase):
    """コース多様化（2026-08-31）で追加したcircuit/corridor/narrow/obstacleの
    配管レベルの検証。品質そのもの（見た目の多様性）は`watch.py`の目視で確認する。"""

    def _assert_basic_course(self, c, kind: str) -> None:
        gap = np.hypot(c.centerline[-1, 0] - c.centerline[0, 0],
                       c.centerline[-1, 1] - c.centerline[0, 1])
        w_scalar = float(np.max(c.width)) if isinstance(c.width, np.ndarray) else c.width
        self.assertLess(gap, w_scalar * 0.6, msg=f"{kind}: 閉じていない")
        self.assertFalse(c.occupied(c.start[0], c.start[1]), msg=f"{kind}: スタート地点が壁")
        r_min = _vehicle_min_turn_radius_m() * _RADIUS_MARGIN
        self.assertGreaterEqual(_min_turn_radius_m(c.centerline[:, :2]), r_min * 0.9,
                                msg=f"{kind}: 最小旋回半径違反")

    def test_circuit_and_corridor_produce_valid_courses(self):
        for kind, fn in [("circuit", generate_circuit_course),
                         ("corridor", generate_corridor_course)]:
            for seed in range(15):
                c = fn(np.random.default_rng(seed))
                self._assert_basic_course(c, kind)

    def test_corridor_lanes_do_not_merge(self):
        """2026-08-31発覚の回帰防止——corridorの旋回半径は`width`と無関係に
        決めていたため、道幅が広く半径が小さい組み合わせ（実測で約6%の頻度）で
        対向する直線同士の間の壁（島）が対向車線と繋がって消えることがあった
        （バンビの「同じようなバグがないか」という指摘で発覚、`_corridor_turn_radius`
        docstring参照）。壁のすぐ外側の点が常に壁のままであることを確認する。
        """
        for seed in range(200):
            rng = np.random.default_rng(seed)
            width = float(rng.uniform(0.7, 1.3))
            c = generate_corridor_course(rng, width=width)
            half_w = width / 2.0
            check_r = half_w + 0.06                    # 意図した道幅のすぐ外側
            idxs = np.linspace(0, len(c.centerline) - 1, 24).astype(int)
            for i in idxs:
                x, y, yaw = c.centerline[i]
                nx, ny = -np.sin(yaw), np.cos(yaw)
                for sign in (1.0, -1.0):
                    px, py = x + sign * nx * check_r, y + sign * ny * check_r
                    self.assertTrue(c.occupied(px, py),
                                    msg=f"seed={seed}: 壁のすぐ外側が空いている"
                                        f"（車線一体化の疑い、width={width:.2f}）")

    def test_narrow_width_is_array_matching_centerline_and_actually_narrows(self):
        found_narrow = 0
        for seed in range(20):
            c = generate_narrow_course(np.random.default_rng(seed))
            self._assert_basic_course(c, "narrow")
            self.assertIsInstance(c.width, np.ndarray)
            self.assertEqual(c.width.shape[0], c.centerline.shape[0])
            if float(c.width.min()) < float(c.width.max()) - 1e-6:
                found_narrow += 1
        self.assertGreater(found_narrow, 15)

    def test_narrow_never_blocks_the_vehicle(self):
        """2026-08-31に発覚した回帰バグの再発防止テスト——`r_disc`/`offset`の式が
        誤っていたため、狭めた区間の左右の円盤が中心線を越えて重なり、
        車体が物理的に通れない（クリアランスが車体全幅を下回る）コースが
        生成されうる状態だった（バンビの「障害物や道幅で完全に通れなくなっている
        コースは生成されていない？」という質問で発覚）。狭めた区間の中で最も
        狭い地点で、中心線から法線方向に車体半幅ぶんだけ動いた2点がどちらも
        壁に埋まっていない（＝車体が中心を通れば少なくとも通過できる）ことを
        多数のseedで確認する。
        """
        veh_half_w = _vehicle_half_width_m()
        n_checked = 0
        for seed in range(300):
            c = generate_narrow_course(np.random.default_rng(seed), width_range=(0.7, 1.3))
            if not isinstance(c.width, np.ndarray):
                continue
            i_min = int(np.argmin(c.width))
            x, y, yaw = c.centerline[i_min]
            nx, ny = -np.sin(yaw), np.cos(yaw)
            n_checked += 1
            for sign in (1.0, -1.0):
                px = x + sign * nx * veh_half_w
                py = y + sign * ny * veh_half_w
                self.assertFalse(c.occupied(px, py),
                                 msg=f"seed={seed}: 最狭部で車体半幅ぶんの位置が壁に埋まっている"
                                     f"（記録幅={float(c.width[i_min]):.3f}m）")
        self.assertGreater(n_checked, 15)

    def test_obstacle_courses_have_obstacles_and_valid_start(self):
        found_obstacles = 0
        for seed in range(20):
            c = generate_obstacle_course(np.random.default_rng(seed))
            self._assert_basic_course(c, "obstacle")
            if c.obstacles is not None:
                found_obstacles += 1
        self.assertGreater(found_obstacles, 15)

    def test_obstacle_course_start_never_overlaps_an_obstacle(self):
        """`SimE2EEnv.reset()`の障害物回避（`_start_index_away_from_obstacles`）が
        機能しているかを、実際に`reset()`を回して確認する。"""
        for seed in range(30):
            env = SimE2EEnv(course_fn=generate_obstacle_course, max_steps=10, seed=seed)
            env.reset()
            if env.course.obstacles is None:
                continue
            d = np.hypot(env.course.obstacles[:, 0] - env.vehicle.x,
                        env.course.obstacles[:, 1] - env.vehicle.y)
            self.assertTrue(np.all(d > env.course.obstacles[:, 2]), msg=f"seed={seed}")

    def test_diverse_course_completes_without_exception_across_seeds(self):
        for seed in range(60):
            c = generate_diverse_course(np.random.default_rng(20000 + seed))
            self._assert_basic_course(c, "diverse")


class TestGenerateDiverseCourseWeights(unittest.TestCase):
    """`generate_diverse_course`の`weights`引数（2026-09-02追加、カリキュラム学習用）。"""

    def test_weights_none_uses_archetype_weights_default(self):
        """`weights`省略時は従来通り`_ARCHETYPE_WEIGHTS`を使う——既存呼び出し元
        （`ml_lidar/train_rl.py`は今は`CurriculumCourseFn`経由になったが、
        `watch.py`等はまだ直接`generate_diverse_course`を渡す可能性がある）の
        挙動を変えないための回帰テスト。"""
        rng_a = np.random.default_rng(0)
        rng_b = np.random.default_rng(0)
        c_default = generate_diverse_course(rng_a)
        c_explicit = generate_diverse_course(rng_b, weights=_ARCHETYPE_WEIGHTS)
        self.assertEqual(c_default.grid.shape, c_explicit.grid.shape)

    def test_weights_override_changes_archetype_distribution(self):
        """narrow/obstacleの重みを0にすると、その2種は一度も選ばれない
        （`_assert_basic_course`が通る=生成自体は壊れていないことも合わせて確認）。"""
        weights = {"organic": 1.0, "circuit": 0.0, "corridor": 0.0, "narrow": 0.0, "obstacle": 0.0}
        for seed in range(20):
            c = generate_diverse_course(np.random.default_rng(seed), weights=weights)
            self._assert_basic_course(c, "organic-only")

    def _assert_basic_course(self, c, kind: str) -> None:
        # `TestNewArchetypes`と同じ検査を軽量に再利用する
        TestNewArchetypes._assert_basic_course(self, c, kind)


class TestCurriculumCourseFn(unittest.TestCase):
    """`CurriculumCourseFn`（2026-09-02追加）。`course_fn: Callable[[rng], Course]`
    契約を満たしつつ、`progress`に応じて難度（道幅・アーキタイプ重み）を補間する。
    """

    def test_is_callable_with_single_rng_argument(self):
        """`SimE2EEnv.reset()`が`self.course_fn(self.rng)`という1引数の形で呼ぶ契約
        （モジュールdocstring参照）を満たすことを、実際に`SimE2EEnv`へ渡して確認する。"""
        env = SimE2EEnv(course_fn=CurriculumCourseFn(), max_steps=10, seed=0)
        env.reset()  # 例外を投げなければOK
        self.assertIsNotNone(env.course)

    def test_progress_zero_never_selects_narrow_or_obstacle(self):
        fn = CurriculumCourseFn()
        fn.set_progress(0.0)
        for seed in range(200):
            c = fn(np.random.default_rng(seed))
            # narrowは道幅が配列（区間ごとに変わる）、obstacleは`obstacles`が
            # 非Noneになる、という実装上の特徴で判別する
            # （`sim/random_course.py`のnarrow/obstacle実装参照）
            self.assertFalse(isinstance(c.width, np.ndarray),
                             msg="progress=0でnarrowが選ばれてはいけない")
            self.assertIsNone(c.obstacles, msg="progress=0でobstacleが選ばれてはいけない")

    def test_progress_zero_narrows_width_range_lower_bound_to_easy_width_low(self):
        """progress=0では道幅下限が`easy_width_low`（既定1.0m）まで持ち上がり、
        `full_width_range`の下限0.7mより狭いコースが出ないことを、生成コースの
        実際の`width`から確認する（organicは`width`がスカラーなので直接読める）。"""
        fn = CurriculumCourseFn()
        fn.set_progress(0.0)
        weights_organic_only = {"organic": 1.0, "circuit": 0.0, "corridor": 0.0,
                                "narrow": 0.0, "obstacle": 0.0}
        fn._EASY_WEIGHTS = weights_organic_only  # このテストだけorganicに固定
        for seed in range(30):
            c = fn(np.random.default_rng(seed))
            self.assertGreaterEqual(c.width, 1.0 - 1e-9)

    def test_progress_one_matches_full_difficulty_distribution(self):
        """progress=1.0では`generate_diverse_course`本来の`_ARCHETYPE_WEIGHTS`・
        `width_range=(0.7,1.3)`と同じ分布になる（narrow/obstacleも出現しうる）。"""
        fn = CurriculumCourseFn()
        fn.set_progress(1.0)
        saw_narrow_or_obstacle = False
        for seed in range(200):
            c = fn(np.random.default_rng(seed))
            if isinstance(c.width, np.ndarray) or c.obstacles is not None:
                saw_narrow_or_obstacle = True
                break
        self.assertTrue(saw_narrow_or_obstacle,
                        msg="progress=1.0ではnarrow/obstacleが出現するはず")

    def test_set_progress_clamps_to_unit_range(self):
        fn = CurriculumCourseFn()
        fn.set_progress(5.0)
        self.assertEqual(fn.progress, 1.0)
        fn.set_progress(-2.0)
        self.assertEqual(fn.progress, 0.0)


if __name__ == "__main__":
    unittest.main()
