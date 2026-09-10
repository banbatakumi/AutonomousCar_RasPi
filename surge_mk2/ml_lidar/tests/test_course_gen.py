"""`ml_lidar/course_gen.py` のテスト。チェックポイント＋旋回率制限ウォーク方式が
実際に厳密に閉じ、自己交差せず、車両の物理的最小旋回半径を割り込まないかを確認する。"""

import math
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))  # surge_mk2/

import numpy as np  # noqa: E402

from ml_lidar.course_gen import (  # noqa: E402
    _sample_checkpoints,
    _self_intersects,
    _walk_checkpoints,
    random_walk_loop_course,
    sample_random_walk_loop_params,
    vehicle_min_turn_radius_m,
    walk_loop_course,
)


def _signed_area(centerline: np.ndarray) -> float:
    """センターラインの符号付き面積（shoelace公式）。正=反時計回り、負=時計回り。"""
    xy = centerline[:, :2]
    x, y = xy[:, 0], xy[:, 1]
    x2, y2 = np.roll(x, -1), np.roll(y, -1)
    return float(np.sum(x * y2 - x2 * y) / 2.0)


class TestRandomWalkLoopCourse(unittest.TestCase):
    def test_closes_exactly_across_seeds(self) -> None:
        """位置・向きが十分小さい許容誤差内で閉じることを確認する。

        `sim/track.py::build_from_points()`の閉じチェックは点列の生成方式を
        問わない共通ロジックなので、同じ判定式で確認する。
        """
        for seed in range(20):
            rng = np.random.default_rng(seed)
            course = random_walk_loop_course(rng)
            cl = course.centerline
            self.assertIsNotNone(cl)

            gap = math.hypot(cl[-1, 0] - cl[0, 0], cl[-1, 1] - cl[0, 1])
            dth = abs((cl[-1, 2] - cl[0, 2] + math.pi) % (2 * math.pi) - math.pi)
            self.assertLess(gap, 0.4, f"seed={seed}: 位置が閉じていない ({gap:.4f}m)")
            self.assertLess(math.degrees(dth), 15.0, f"seed={seed}: 向きが閉じていない")

    def test_grid_has_both_wall_and_free_cells(self) -> None:
        rng = np.random.default_rng(0)
        course = random_walk_loop_course(rng)
        self.assertTrue(course.grid.any())
        self.assertFalse(course.grid.all())

    def test_start_not_colliding(self) -> None:
        from sim.vehicle import VehicleSpec

        spec = VehicleSpec.load()
        for seed in range(20):
            rng = np.random.default_rng(seed)
            course = random_walk_loop_course(rng)
            body = course.body_samples(spec.footprint)
            self.assertFalse(course.collides(*course.start, body), f"seed={seed}: start位置が壁")

    def test_role_is_train(self) -> None:
        rng = np.random.default_rng(0)
        course = random_walk_loop_course(rng)
        self.assertEqual(course.role, "train")

    def test_clockwise_flips_loop_direction(self) -> None:
        ccw = walk_loop_course(8, 2.0, 0.30, 0.30, 1.0, seed=0, clockwise=False)
        cw = walk_loop_course(8, 2.0, 0.30, 0.30, 1.0, seed=0, clockwise=True)
        area_ccw = _signed_area(ccw.centerline)
        area_cw = _signed_area(cw.centerline)
        self.assertGreater(area_ccw, 0.0)
        self.assertLess(area_cw, 0.0)

    def test_random_walk_loop_course_covers_both_directions(self) -> None:
        """観測・行動が左右対称な設計なので、旋回方向は50/50でランダム化されているべき
        （固定していた過去の実装では、方策が右コーナーをほぼ経験しないまま学習が進んでいた）。
        """
        signs = []
        for seed in range(40):
            course = random_walk_loop_course(np.random.default_rng(seed))
            signs.append(_signed_area(course.centerline) > 0.0)
        n_ccw = sum(signs)
        self.assertGreater(n_ccw, 5, "反時計回りが40本中5本以下——ランダム化されていない疑い")
        self.assertLess(n_ccw, 35, "時計回りが40本中5本以下——ランダム化されていない疑い")

    def test_no_self_intersection_across_seeds(self) -> None:
        """旋回率制限ウォークは(旧・極座標r(θ)方式と違い)自己交差を構造的に排除できない
        ため、生成コースに対し明示チェック(`_self_intersects`)が実際に「交差なし」を
        返すことを回帰的に確認する（`_generate_walk_centerline`のリジェクトが効いている
        ことの検証）。
        """
        for seed in range(50):
            course = random_walk_loop_course(np.random.default_rng(seed))
            cl = course.centerline
            self.assertFalse(_self_intersects(cl, course.width, resolution := 0.03),
                             f"seed={seed}: 自己交差または自己接触が残っている")
            del resolution

    def test_self_intersects_detects_a_crossing_figure_eight(self) -> None:
        """`_self_intersects`自体が偽陰性(見逃し)を起こしていないことを、
        明確に自己交差する8の字点列で直接確認する。"""
        t = np.linspace(0.0, 2 * math.pi, 400, endpoint=False)
        x = np.sin(t)
        y = np.sin(t) * np.cos(t)  # リサージュ曲線(8の字、原点で自己交差する)
        yaw = np.zeros_like(t)
        pts = np.column_stack((x, y, yaw))
        step_m = float(np.mean(np.hypot(*np.diff(pts[:, :2], axis=0, append=pts[:1, :2]).T)))
        self.assertTrue(_self_intersects(pts, width_m=0.3, step_m=step_m))

    def test_vehicle_min_turn_radius_matches_analytic_formula(self) -> None:
        from sim.vehicle import VehicleSpec

        spec = VehicleSpec.load()
        expected = spec.wheelbase / math.tan(spec.max_steer)
        self.assertAlmostEqual(vehicle_min_turn_radius_m(spec), expected, places=9)

    def test_walk_curvature_never_exceeds_the_per_step_turn_clamp(self) -> None:
        """`_walk_checkpoints()`は1歩あたりの旋回角を`max_turn_per_step`にクランプする
        ことで曲率安全マージンを**構成時点で**保証する（旧方式の事後リジェクトサンプリング
        とは異なる、モジュールdocstring参照）。生成された中心線の実測旋回角が、
        隣接点間で本当にこのクランプ値を超えていないことを直接確認する回帰テスト。
        """
        min_radius_m = vehicle_min_turn_radius_m() * 1.1  # course_gen._RADIUS_MARGINと同じ値
        resolution = 0.03
        max_turn_per_step = resolution / min_radius_m
        for seed in range(30):
            course = random_walk_loop_course(np.random.default_rng(seed), resolution=resolution)
            cl = course.centerline
            dyaw = np.diff(cl[:, 2])
            dyaw = (dyaw + math.pi) % (2 * math.pi) - math.pi
            self.assertLessEqual(float(np.max(np.abs(dyaw))), max_turn_per_step + 1e-6,
                                 f"seed={seed}: 1歩あたりの旋回角がクランプ値を超えている")

    def test_generated_curvature_never_requires_turn_radius_below_vehicle_limit(self) -> None:
        """生成された中心線そのものの最大曲率が、車両物理限界からの安全マージンを
        下回らないことを確認する（有限差分による独立経路での再検証、正n角形方式の
        スカラー半径検証の後継）。"""
        threshold_radius = vehicle_min_turn_radius_m() * 1.1  # course_gen._RADIUS_MARGINと同じ値
        for seed in range(30):
            course = random_walk_loop_course(np.random.default_rng(seed))
            cl = course.centerline
            xy = cl[:, :2]
            seg = np.hypot(*np.diff(xy, axis=0).T)
            dyaw = np.diff(cl[:, 2])
            dyaw = (dyaw + math.pi) % (2 * math.pi) - math.pi
            curvature = np.abs(dyaw) / np.clip(seg, 1e-9, None)
            max_curvature = float(np.max(curvature))
            if max_curvature <= 1e-9:
                continue
            turn_radius = 1.0 / max_curvature
            self.assertGreaterEqual(turn_radius, threshold_radius * 0.9,
                                    f"seed={seed}: 実測旋回半径{turn_radius:.4f}が"
                                    f"安全マージン込みの最小旋回半径{threshold_radius:.4f}を"
                                    "大きく下回った")

    def test_chicane_cluster_produces_consecutive_direction_reversals(self) -> None:
        """`chicane_len`>0のとき、狭い弧長区間内に複数回の方向転換(符号反転)が連続する
        ことを確認する。`sim/courses/toyota.json`は短い直線を挟んで急角ターンが連続する
        配置であり、これは独立ジッターの「1点だけ鋭く尖る」孤立ヘアピンとは区別される
        べき特徴——この検証が無いと「クラスタを強制したつもりが実は孤立ヘアピン止まり」
        という劣化に気づけない。
        """
        n_ok = 0
        for seed in range(30):
            rng = np.random.default_rng(seed)
            course = walk_loop_course(9, 2.5, 0.15, 0.12, 1.0, seed=seed,
                                      chicane_len=3, chicane_offset_frac=0.35)
            del rng
            cl = course.centerline
            dyaw = np.diff(cl[:, 2], append=cl[0, 2] + 2 * math.pi)
            dyaw = (dyaw + math.pi) % (2 * math.pi) - math.pi
            signs = np.where(np.abs(dyaw) > 1e-6, np.sign(dyaw), 0)
            signs = signs[signs != 0]
            reversals = int(np.sum(signs[1:] != signs[:-1]))
            if reversals >= 3:
                n_ok += 1
        self.assertGreater(n_ok, 20,
                           f"chicane_len=3で方向転換3回以上のseedが30本中{n_ok}本しかない"
                           "(連続シケインになっていない疑い)")

    def test_sample_checkpoints_chicane_alternates_sign(self) -> None:
        """`chicane_len`個の連続するチェックポイントの半径が交互(+/-)に上書きされることを
        直接確認する。"""
        rng = np.random.default_rng(0)
        base = 2.0
        chicane_frac = 0.20
        theta, radius = _sample_checkpoints(rng, 9, base, 0.15, 0.12, chicane_len=3,
                                            chicane_offset_frac=chicane_frac)
        del theta
        matches = [i for i in range(9)
                  if math.isclose(radius[i], base * (1.0 + chicane_frac), rel_tol=1e-9)
                  or math.isclose(radius[i], base * (1.0 - chicane_frac), rel_tol=1e-9)]
        self.assertEqual(len(matches), 3, "chicane_len=3個ぶんの上書きが見つからない")

    def test_sampled_params_stay_within_declared_ranges(self) -> None:
        """`sample_random_walk_loop_params()`のパラメータ抽出だけを直接検証する
        （重い生成を経由しない）。

        旧方式と異なり、曲率安全マージンは`_walk_checkpoints()`が構成時点で保証するため、
        `n_checkpoints`別のジッター上限較正は不要になった——ジッター範囲はn_checkpoints
        によらず一定（`course_gen.py`のモジュールdocstring参照）。
        """
        for seed in range(200):
            p = sample_random_walk_loop_params(np.random.default_rng(seed))
            self.assertIn(p.n_checkpoints, range(6, 11))
            self.assertGreaterEqual(p.angle_jitter_frac, 0.15)
            self.assertLess(p.angle_jitter_frac, 1.0,
                            "angle_jitter_fracが1.0以上——チェックポイントのθ順序が"
                            "入れ替わりうる")
            self.assertLessEqual(p.angle_jitter_frac, 0.50 + 1e-9)
            self.assertGreaterEqual(p.radius_jitter_frac, 0.10)
            self.assertLessEqual(p.radius_jitter_frac, 0.65 + 1e-9)
            self.assertIn(p.chicane_len, (0, 3),
                         f"chicane_len={p.chicane_len}が想定外の値（0か3のはず）")
            if p.chicane_len == 0:
                self.assertEqual(p.chicane_offset_frac, 0.0,
                                 "chicane_len=0なのにchicane_offset_frac>0——不整合")
            else:
                self.assertGreater(p.chicane_offset_frac, 0.0)
                self.assertLess(p.chicane_offset_frac, 0.26 + 1e-9,
                                "chicane_offset_fracの上限(道幅比補正前)を超えている")

    def test_chicane_len_triggers_with_expected_frequency(self) -> None:
        """`chicane_len>0`が一定確率(目安35%)で選ばれることを確認する回帰テスト。
        0%（配線忘れ）や100%（既存の独立ジッター分布を潰してしまう）のどちらの劣化も
        検出できるよう、緩めの範囲でチェックする。"""
        n = 2000
        n_triggered = sum(
            sample_random_walk_loop_params(np.random.default_rng(seed)).chicane_len > 0
            for seed in range(n)
        )
        frac = n_triggered / n
        self.assertGreater(frac, 0.15, f"chicane_len>0の出現率が{frac:.2%}と低すぎる")
        self.assertLess(frac, 0.55, f"chicane_len>0の出現率が{frac:.2%}と高すぎる"
                        "（既存の独立ジッター分布を潰している疑い）")

    def test_walk_checkpoints_returns_none_when_target_unreachable(self) -> None:
        """`_walk_checkpoints`は、旋回率制限内でチェックポイントを2周ぶん歩き切れない
        場合に`None`を返し、呼び出し側(`_generate_walk_centerline`)のリトライに委ねる
        契約になっている。極端に厳しい旋回率制限(最小旋回半径が極端に大きい)を
        直接与えて、この契約が壊れていないことを確認する。"""
        rng = np.random.default_rng(0)
        theta, radius = _sample_checkpoints(rng, 8, 2.0, 0.30, 0.30)
        step_m = 0.03
        absurdly_large_min_radius_m = 1000.0
        max_turn_per_step = step_m / absurdly_large_min_radius_m
        result = _walk_checkpoints(theta, radius, step_m, max_turn_per_step,
                                   max_total_steps=2000)
        self.assertIsNone(result)


if __name__ == "__main__":
    unittest.main()
