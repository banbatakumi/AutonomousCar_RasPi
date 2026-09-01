"""`sim/gym_env.py` の `SimE2EEnv` のテスト。実際の学習は行わず、
エピソードが仕様通りに終了すること・報酬の符号がおかしくないことだけを確認する。
"""

import math
import sys
import unittest
import unittest.mock
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))  # surge_mk2/

import numpy as np  # noqa: E402

from sim.gym_env import OBS_DIM, SimE2EEnv  # noqa: E402
from sim.random_course import generate_random_course  # noqa: E402
from sim.raceline import compute_raceline_offsets as _real_compute_raceline_offsets  # noqa: E402
from sim.vehicle import VehicleSpec  # noqa: E402


def _make_env(**kwargs) -> SimE2EEnv:
    rng = np.random.default_rng(0)
    courses = [generate_random_course(rng, name=f"c{i}") for i in range(2)]
    return SimE2EEnv(courses, seed=0, **kwargs)


class TestSimE2EEnv(unittest.TestCase):
    def test_reset_returns_scan_plus_speed_observation(self):
        env = _make_env()
        obs = env.reset()
        self.assertEqual(obs.shape, (OBS_DIM,))
        self.assertTrue(np.all(np.isfinite(obs)))
        self.assertEqual(obs[-2], 0.0)     # 発進直後は速度0のはず
        self.assertEqual(obs[-1], 0.0)     # 発進直後はステアも0のはず

    def test_collision_terminates_episode(self):
        env = _make_env(max_steps=500)
        env.reset()
        terminated = truncated = False
        steps = 0
        # ステアを切らずに全開で直進すれば、ランダムコースのどこかで必ず壁に当たる
        while not (terminated or truncated) and steps < 500:
            _, reward, terminated, truncated, info = env.step(np.array([0.0, 1.0]))
            steps += 1
        self.assertTrue(terminated)
        self.assertTrue(info["collided"])
        self.assertLess(reward, 0.0)   # 衝突ペナルティが乗っているはず

    def test_max_steps_truncates_without_collision_when_stationary(self):
        env = _make_env(max_steps=20)
        env.reset()
        terminated = truncated = False
        for _ in range(20):
            _, _, terminated, truncated, info = env.step(np.array([0.0, 0.0]))
        self.assertFalse(terminated)
        self.assertTrue(truncated)
        self.assertFalse(info["collided"])

    def test_progress_reward_is_near_zero_when_stationary(self):
        # raceline_weight/speed_match_weightも0にする——理想ラインは中心線からの
        # オフセットを持つので(sim/raceline.py)、スポーン地点がコーナー付近だと
        # 静止していても`raceline_tolerance_m`超過の罰則が乗ってしまい、この
        # テストが検証したい「progress_weightだけ」の話ではなくなる
        env = _make_env(max_steps=50, cross_track_weight=0.0,
                        raceline_weight=0.0, speed_match_weight=0.0)
        env.reset()
        total = 0.0
        for _ in range(20):
            _, r, _, _, _ = env.step(np.array([0.0, 0.0]))
            total += r
        self.assertAlmostEqual(total, 0.0, delta=1e-6)

    def test_action_is_clamped_to_limits(self):
        # 最大舵角は`spec.max_steer`（vehicle.toml相当）から決まる。独立引数は無い
        # （2026-08-28、バンビの指示）ので、別の値を試したいときは`spec`を差し替える
        env = _make_env(max_speed=0.5, spec=VehicleSpec(max_steer=0.3))
        env.reset()
        # 範囲外の行動を渡しても例外にならず、内部でクランプされる
        env.step(np.array([10.0, 100.0]))

    def test_observation_speed_tracks_actual_vehicle_speed(self):
        """観測の末尾1個が自車速度になっている（改善1）。

        以前は20ステップ直進させてから確認していたが、コースの形状によっては
        途中で壁に当たり（衝突後は速度が0に落ちる）、テストの意図と無関係な
        理由で失敗しうる状態だった（2026-08-29、`sim/random_course.py`の
        生成方式を書き換えた際に発覚）。速度は1ステップ目から既に正になる
        （`tau_speed_s`の一次遅れでも0.1sで動き出すため）ので、衝突の心配が
        ほぼ無い1ステップだけで十分に検証できる。"""
        env = _make_env(max_steps=50, max_speed=1.0)
        env.reset()
        obs, _, _, _, _ = env.step(np.array([0.0, 1.0]))     # 全開で加速
        self.assertGreater(obs[-2], 0.0)
        self.assertAlmostEqual(float(obs[-2]), env.vehicle.speed, places=5)

    def test_randomize_lidar_varies_noise_between_episodes(self):
        """改善2: `randomize_lidar=True`（既定）なら毎エピソードノイズ量が変わる。"""
        env = _make_env(randomize_lidar=True)
        env.reset()
        sigma1 = env.lidar.params.lidar_noise_sigma_m
        env.reset()
        sigma2 = env.lidar.params.lidar_noise_sigma_m
        env.reset()
        sigma3 = env.lidar.params.lidar_noise_sigma_m
        self.assertFalse(sigma1 == sigma2 == sigma3)

    def test_randomize_lidar_false_keeps_noise_fixed(self):
        """評価環境（`train_rl.py`の`make_eval_env`）はここを`False`にして条件を揃える。"""
        env = _make_env(randomize_lidar=False)
        env.reset()
        sigma1 = env.lidar.params.lidar_noise_sigma_m
        env.reset()
        sigma2 = env.lidar.params.lidar_noise_sigma_m
        self.assertEqual(sigma1, sigma2)
        self.assertEqual(sigma1, env.sim_params.lidar_noise_sigma_m)

    def test_course_fn_generates_a_fresh_course_each_episode(self):
        """改善3: 固定プールではなく毎エピソード新しいコースを作れる。
        `generate_random_course`をそのまま渡せる（環境自身の`rng`を引数で受け取る形）。"""
        seen_shapes = set()
        env = SimE2EEnv(course_fn=generate_random_course, seed=1)
        for _ in range(3):
            env.reset()
            seen_shapes.add(env.course.grid.shape)
        self.assertGreater(len(seen_shapes), 1)    # 毎回同じ形にはならないはず

    def test_course_fn_reset_with_same_seed_gives_the_same_course(self):
        """gymnasiumの決定性契約: `course_fn`は環境自身の`rng`を使うので、
        同じseedでresetし直せば同じコースになる（`check_env`の要件でもある）。"""
        env = SimE2EEnv(course_fn=generate_random_course, seed=7)
        env.rng = np.random.default_rng(7)
        env.reset()
        shape1 = env.course.grid.shape
        env.rng = np.random.default_rng(7)
        env.reset()
        shape2 = env.course.grid.shape
        self.assertEqual(shape1, shape2)

    def test_missing_courses_and_course_fn_raises(self):
        with self.assertRaises(ValueError):
            SimE2EEnv()

    def test_cross_track_margin_frees_deviation_within_margin(self):
        """道幅の`cross_track_margin_frac`以内のずれはペナルティ無しになる（2026-08-28、
        レーシングライン学習を阻害しないための変更）。同じ行動列を与えたとき、
        `cross_track_margin_frac=1.0`（実質ペナルティ無し）の方が`=0.0`（旧来通り
        常に中心線距離を罰する）より合計報酬が高くなるはず。"""

        def run(margin_frac: float) -> float:
            env = _make_env(max_steps=60, cross_track_margin_frac=margin_frac,
                            randomize_lidar=False)
            env.reset()
            total = 0.0
            for _ in range(60):
                _, r, terminated, truncated, _ = env.step(np.array([0.15, 0.8]))
                total += r
                if terminated or truncated:
                    break
            return total

        self.assertGreater(run(1.0), run(0.0))

    def test_speed_weight_rewards_higher_speed(self):
        """速度ボーナス（2026-08-30追加、v5評価で速度・ライン取りが消極的だった
        ことへの対応）: 同じ「直進・全開」行動でも`speed_weight`が大きいほど
        合計報酬が高くなるはず。"""

        def run(speed_weight: float) -> float:
            env = _make_env(max_steps=5, speed_weight=speed_weight, randomize_lidar=False)
            env.reset()
            total = 0.0
            for _ in range(5):
                _, r, terminated, truncated, _ = env.step(np.array([0.0, 1.0]))
                total += r
                if terminated or truncated:
                    break
            return total

        self.assertGreater(run(0.5), run(0.0))

    def test_speed_weight_has_no_effect_when_stationary(self):
        """静止（speed=0）なら`speed_weight`をいくつにしても速度ボーナスは0のまま。"""
        # raceline_weightも0にする理由は上のtest_progress_reward_is_near_zero_when_stationary参照
        env = _make_env(max_steps=20, cross_track_weight=0.0, speed_weight=0.5,
                        raceline_weight=0.0, speed_match_weight=0.0)
        env.reset()
        total = 0.0
        for _ in range(20):
            _, r, _, _, _ = env.step(np.array([0.0, 0.0]))
            total += r
        self.assertAlmostEqual(total, 0.0, delta=1e-6)

    def test_randomize_dynamics_varies_mu_between_episodes(self):
        """`randomize_dynamics=True`（既定、2026-08-31追加）なら毎エピソード
        `[dynamics]`未実測パラメータ（mu等）が変わる。"""
        env = _make_env(randomize_dynamics=True)
        env.reset()
        mu1 = env.vehicle.spec.mu
        env.reset()
        mu2 = env.vehicle.spec.mu
        env.reset()
        mu3 = env.vehicle.spec.mu
        self.assertFalse(mu1 == mu2 == mu3)

    def test_randomize_dynamics_false_keeps_spec_fixed(self):
        """評価環境（`train_rl.py`の`make_eval_env`）はここを`False`にして条件を揃える。"""
        env = _make_env(randomize_dynamics=False)
        env.reset()
        mu1 = env.vehicle.spec.mu
        env.reset()
        mu2 = env.vehicle.spec.mu
        self.assertEqual(mu1, mu2)
        self.assertEqual(mu1, env.spec.mu)

    def test_slip_weight_penalizes_grip_limit_violation(self):
        """slip_weight（2026-08-31追加、`VehicleModel.slip_frac`に掛ける罰則）:
        高速で走り続けるとグリップ限界を超え続けるので、slip_weightが大きいほど
        合計報酬が低くなるはず。壁への衝突で早期終了すると2つの`run()`が同じ
        ステップ数しか比較できず差が出ない（＝collisionの影響が支配的になる）
        ことがあるため、幅の広いコース（width=20m）を使って衝突を避ける。"""

        def run(slip_weight: float) -> float:
            rng = np.random.default_rng(0)
            courses = [generate_random_course(rng, name="wide", width=20.0)]
            env = SimE2EEnv(courses, seed=0, max_steps=10, max_speed=6.0,
                            randomize_dynamics=False, randomize_lidar=False,
                            slip_weight=slip_weight)
            env.reset()
            action = np.array([0.1, 6.0])
            total = 0.0
            for _ in range(10):
                _, r, terminated, truncated, _ = env.step(action)
                total += r
                if terminated or truncated:
                    break
            return total

        self.assertGreater(run(0.0), run(1.0))

    def test_step_info_includes_slip(self):
        env = _make_env(max_steps=5)
        env.reset()
        _, _, _, _, info = env.step(np.array([0.0, 1.0]))
        self.assertIn("slip", info)
        self.assertGreaterEqual(info["slip"], 0.0)

    def test_step_info_includes_raceline_fields(self):
        env = _make_env(max_steps=5)
        env.reset()
        _, _, _, _, info = env.step(np.array([0.0, 1.0]))
        self.assertIn("raceline_cross", info)
        self.assertIn("target_speed", info)
        self.assertGreaterEqual(info["raceline_cross"], 0.0)
        self.assertGreaterEqual(info["target_speed"], 0.0)

    def test_raceline_weight_penalizes_deviation_beyond_tolerance(self):
        """raceline_weight（2026-09-01追加、`sim/raceline.py`の理想ラインからの
        横偏差のうち`raceline_tolerance_m`を超えた分への罰則）: 同じ行動列でも
        大きく舵を切って理想ラインから外れ続ければ、raceline_weightが大きいほど
        合計報酬が低くなるはず。`test_cross_track_margin_frees_deviation_within_margin`
        と同じ「舵0.15・全開」の行動で道幅を大きく使わせる。"""

        def run(raceline_weight: float) -> float:
            env = _make_env(max_steps=60, raceline_weight=raceline_weight,
                            raceline_tolerance_m=0.08, randomize_lidar=False)
            env.reset()
            total = 0.0
            for _ in range(60):
                _, r, terminated, truncated, _ = env.step(np.array([0.15, 0.8]))
                total += r
                if terminated or truncated:
                    break
            return total

        self.assertGreater(run(0.0), run(1.0))

    def test_speed_match_weight_rewards_matching_target_speed(self):
        """speed_match_weight（2026-09-01追加、理想ラインの目標速度とのズレの
        小ささに応じたボーナス）: 曲率がほぼ0の広いコース（目標速度≈max_speed）で
        全開走行すれば目標速度によく一致するはずなので、speed_match_weightが
        大きいほど合計報酬が高くなるはず（`test_slip_weight_penalizes_grip_limit_violation`
        と同じ「幅20mの広いコースで衝突を避ける」構成）。"""

        def run(speed_match_weight: float) -> float:
            rng = np.random.default_rng(0)
            courses = [generate_random_course(rng, name="wide", width=20.0)]
            env = SimE2EEnv(courses, seed=0, max_steps=10, max_speed=1.5,
                            randomize_dynamics=False, randomize_lidar=False,
                            speed_match_weight=speed_match_weight)
            env.reset()
            action = np.array([0.0, 1.5])
            total = 0.0
            for _ in range(10):
                _, r, terminated, truncated, _ = env.step(action)
                total += r
                if terminated or truncated:
                    break
            return total

        self.assertGreater(run(1.0), run(0.0))


class TestRacelinePrefetchAndCache(unittest.TestCase):
    """①reset()のraceline計算の高速化（2026-09-02追加）。バックグラウンド先読み
    （手続き生成コース用）とメモ化キャッシュ（固定courses用）、双方の正しさを
    確認する——高速化そのもの（実測fps）はこのテストでは検証しない（`docs/`の
    実測メモ参照）、ここでは「結果が変わらないこと」だけを見る。
    """

    def test_course_fn_prefetch_gives_same_trajectory_as_two_independent_envs(self):
        """先読みが有効（`course_fn`使用）でも、同じseedの2つの環境インスタンスが
        複数エピソードにわたって同じコース列を生成する（決定性回帰）。"""

        def course_shapes(n_episodes: int) -> list[tuple]:
            env = SimE2EEnv(course_fn=generate_random_course, max_steps=5, seed=42)
            shapes = []
            for _ in range(n_episodes):
                env.reset()
                shapes.append(env.course.grid.shape)
                for _ in range(5):     # 先読みスレッドが動く時間を与える
                    env.step(np.array([0.0, 0.0]))
            return shapes

        self.assertEqual(course_shapes(4), course_shapes(4))

    def test_reset_seed_reassignment_discards_stale_prefetch(self):
        """`ml_lidar/env.py`の`GymSurgeEnv.reset(seed=...)`と同じパターン——
        `env.rng`を丸ごと差し替えてから`reset()`を呼ぶ——を2回繰り返しても、
        同じ結果になること（2026-09-02、先読み実装直後に踏んだ回帰の再発防止。
        `sim/gym_env.py`の`reset()`docstring参照）。"""
        env = SimE2EEnv(course_fn=generate_random_course, max_steps=5, seed=0)

        env.rng = np.random.default_rng(7)
        env.reset()
        env.step(np.array([0.0, 0.0]))   # 先読みスレッドを走らせる
        shape1 = env.course.grid.shape

        env.rng = np.random.default_rng(7)   # 差し替え。先読み済みだった結果は捨てるべき
        env.reset()
        shape2 = env.course.grid.shape

        self.assertEqual(shape1, shape2)

    def test_fixed_courses_raceline_is_memoized_across_episodes(self):
        """固定`courses`（`make_eval_env`のcircuit/fuji相当）は、同じcourse×同じ
        dynamics（`randomize_dynamics=False`）なら`compute_raceline_offsets`を
        エピソードごとに呼び直さない。"""
        rng = np.random.default_rng(0)
        courses = [generate_random_course(rng, name="c0")]
        env = SimE2EEnv(courses, max_steps=5, seed=0, randomize_dynamics=False,
                        randomize_lidar=False)

        with unittest.mock.patch("sim.gym_env.compute_raceline_offsets",
                                 wraps=_real_compute_raceline_offsets) as m:
            env.reset()
            env.reset()
            env.reset()
            self.assertEqual(m.call_count, 1)

    def test_fixed_courses_raceline_cache_invalidates_on_mu_change(self):
        """`randomize_dynamics=True`で`mu`がエピソードごとに変わる場合は、
        キャッシュキーに`mu`が含まれるので毎回律儀に計算し直す（誤ってキャッシュ
        ヒットし古い`mu`の理想ラインを使い回すことがないように）。"""
        rng = np.random.default_rng(0)
        courses = [generate_random_course(rng, name="c0")]
        env = SimE2EEnv(courses, max_steps=5, seed=0, randomize_dynamics=True,
                        randomize_lidar=False)

        with unittest.mock.patch("sim.gym_env.compute_raceline_offsets",
                                 wraps=_real_compute_raceline_offsets) as m:
            for _ in range(5):
                env.reset()
            self.assertGreaterEqual(m.call_count, 2)

    def test_set_curriculum_progress_propagates_to_course_fn_with_set_progress(self):
        """`course_fn`が`set_progress()`を持つ（`CurriculumCourseFn`）場合だけ
        伝播する。持たない素の関数（`generate_random_course`）を渡した場合は
        単に無視される（`hasattr`ガード、例外にならないことを確認）。"""

        class _FakeCourseFn:
            def __init__(self):
                self.progress = None

            def set_progress(self, p):
                self.progress = p

            def __call__(self, rng):
                return generate_random_course(rng)

        fake = _FakeCourseFn()
        env = SimE2EEnv(course_fn=fake, max_steps=5, seed=0)
        env.set_curriculum_progress(0.5)
        self.assertEqual(fake.progress, 0.5)

        env2 = SimE2EEnv(course_fn=generate_random_course, max_steps=5, seed=0)
        env2.set_curriculum_progress(0.5)   # 例外にならなければOK


if __name__ == "__main__":
    unittest.main()
