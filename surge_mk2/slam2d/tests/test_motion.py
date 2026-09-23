"""`core/motion.py`（運動予測の抽象化）と`integrate_twist`のテスト。

Phase 2 完了条件: 合成した等速円運動+タイムスタンプ点群で、脱スキュー後の
点群位置が理論値と一致すること（誤差<1mm相当）。
"""

import math
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from slam2d.core.motion import (  # noqa: E402
    ConstantVelocityModel, ExternalTwistModel, GyroBiasEstimator, OdometryNoise,
    SpeedScaleEstimator,
)
from slam2d.core.types import Twist2D, integrate_twist  # noqa: E402


class TestIntegrateTwist(unittest.TestCase):
    def test_straight_line(self):
        d = integrate_twist(Twist2D(2.0, 0.0, 0.0), 0.5)
        self.assertAlmostEqual(d.x, 1.0, places=9)
        self.assertAlmostEqual(d.y, 0.0, places=9)
        self.assertAlmostEqual(d.yaw, 0.0, places=9)

    def test_pure_rotation_in_place(self):
        d = integrate_twist(Twist2D(0.0, 0.0, math.pi), 0.5)
        self.assertAlmostEqual(d.x, 0.0, places=9)
        self.assertAlmostEqual(d.y, 0.0, places=9)
        self.assertAlmostEqual(d.yaw, math.pi / 2, places=9)

    def test_circular_arc_matches_known_radius(self):
        # v=1, w=1 -> 半径1mの円弧。1/4周(T=pi/2)で(1,1,90°)動く
        d = integrate_twist(Twist2D(1.0, 0.0, 1.0), math.pi / 2)
        self.assertAlmostEqual(d.x, 1.0, places=6)
        self.assertAlmostEqual(d.y, 1.0, places=6)
        self.assertAlmostEqual(d.yaw, math.pi / 2, places=6)

    def test_matches_straight_limit_as_w_shrinks(self):
        # wがゼロに近づくほど直線運動に一致する（発散しないことの確認）
        d_small_w = integrate_twist(Twist2D(2.0, 0.0, 1e-6), 0.5)
        d_zero_w = integrate_twist(Twist2D(2.0, 0.0, 0.0), 0.5)
        self.assertAlmostEqual(d_small_w.x, d_zero_w.x, places=3)
        self.assertAlmostEqual(d_small_w.y, d_zero_w.y, places=3)


class TestConstantVelocityModel(unittest.TestCase):
    def test_predicts_zero_from_rest(self):
        m = ConstantVelocityModel()
        self.assertEqual(m.current_twist(), Twist2D(0.0, 0.0, 0.0))
        cov = m.prediction_covariance(0.1)
        self.assertEqual(cov.shape, (3, 3))

    def test_converges_to_observed_velocity(self):
        from slam2d.core.types import Pose2D
        m = ConstantVelocityModel(vel_alpha=0.5)
        # 一定して (0.1m, 0, 0) ずつ進んだと観測し続けると、速度推定が1.0 m/sへ収束する
        for _ in range(30):
            m.update(Pose2D(0.1, 0.0, 0.0), 0.1)
        self.assertAlmostEqual(m.twist.vx, 1.0, delta=0.01)
        self.assertAlmostEqual(m.current_twist().vx, 1.0, delta=0.01)


class TestExternalTwistModel(unittest.TestCase):
    def test_uses_external_source(self):
        m = ExternalTwistModel(lambda: Twist2D(2.0, 0.0, 0.0))
        self.assertEqual(m.current_twist(), Twist2D(2.0, 0.0, 0.0))

    def test_applies_gyro_bias_correction(self):
        bias_est = GyroBiasEstimator()
        bias_est.bias = 0.05
        m = ExternalTwistModel(lambda: Twist2D(1.0, 0.0, 1.0), bias_estimator=bias_est)
        twist = m.current_twist()
        self.assertAlmostEqual(twist.yaw_rate, 1.0 - 0.05, places=9)

    def test_update_feeds_bias_estimator_only_against_a_fixed_map(self):
        """走行中の学習は**凍結地図が基準のとき（`absolute=True`）だけ**。

        地図を作りながらの観測は自分の推定で焼いた局所地図が基準なので、
        ゼロ点ずれは原理的に観測できない（`core/motion.py`のZUPTの節）。
        """
        from slam2d.core.types import Pose2D
        bias_est = GyroBiasEstimator(alpha=1.0)  # 1回で追いつく設定にして検証しやすくする
        m = ExternalTwistModel(lambda: Twist2D(0.5, 0.0, 1.1), bias_estimator=bias_est)
        m.current_twist()
        # 地図作成中（absolute=False）は学習しない
        m.update(Pose2D(0.05, 0.0, 1.0 * 0.1), 0.1, raw_delta=Pose2D(0.05, 0.0, 1.1 * 0.1))
        self.assertAlmostEqual(bias_est.bias, 0.0, places=9)
        # 凍結地図に対する観測なら学習する
        m.update(Pose2D(0.05, 0.0, 1.0 * 0.1), 0.1, raw_delta=Pose2D(0.05, 0.0, 1.1 * 0.1),
                 absolute=True)
        self.assertAlmostEqual(bias_est.bias, 0.1, places=6)

    def test_zupt_calibrates_bias_while_standing_still(self):
        """止まっている間はヨーレートの読みがそのままゼロ点ずれ。"""
        from slam2d.core.types import Pose2D
        bias_est = GyroBiasEstimator(alpha_still=0.5)
        m = ExternalTwistModel(lambda: Twist2D(0.0, 0.0, 0.02), bias_estimator=bias_est)
        for _ in range(20):
            m.current_twist()
            m.update(Pose2D(0.0, 0.0, 0.0), 0.1, raw_delta=Pose2D(0.0, 0.0, 0.002))
        self.assertAlmostEqual(bias_est.bias, 0.02, delta=0.001)
        self.assertGreater(bias_est.still_updates, 10)

    def test_correct_applies_bias_and_scale(self):
        bias_est = GyroBiasEstimator()
        bias_est.bias = 0.05
        scale = SpeedScaleEstimator()
        scale.scale = 1.1
        m = ExternalTwistModel(lambda: Twist2D(1.0, 0.0, 1.0), bias_estimator=bias_est,
                               scale_estimator=scale)
        t = m.correct(Twist2D(2.0, 0.0, 0.5))
        self.assertAlmostEqual(t.vx, 2.2, places=9)
        self.assertAlmostEqual(t.yaw_rate, 0.45, places=9)


class TestOdometryNoise(unittest.TestCase):
    def test_covariance_grows_with_distance_and_time(self):
        n = OdometryNoise()
        slow = n.covariance(Twist2D(0.1, 0.0, 0.0), 0.1)
        fast = n.covariance(Twist2D(3.0, 0.0, 0.0), 0.1)
        self.assertGreater(fast[0, 0], slow[0, 0])          # 進行方向は距離に比例
        turning = n.covariance(Twist2D(0.1, 0.0, 2.0), 0.1)
        self.assertGreater(turning[2, 2], slow[2, 2])       # 回頭は回った角度に比例
        longer = n.covariance(Twist2D(0.0, 0.0, 0.0), 1.0)
        still = n.covariance(Twist2D(0.0, 0.0, 0.0), 0.1)
        self.assertGreater(longer[2, 2], still[2, 2])       # 止まっていても時間で育つ


class TestSpeedScaleEstimator(unittest.TestCase):
    def test_learns_the_ratio_when_forward_is_observable(self):
        est = SpeedScaleEstimator(alpha=0.2)
        for _ in range(50):
            est.update(raw_forward=0.10, measured_forward=0.098, info_xx=1e8)
        self.assertAlmostEqual(est.scale, 0.98, delta=0.005)

    def test_ignores_unobservable_directions(self):
        """進行方向が点群で決まっていないときは学習しない（通路の中など）。"""
        est = SpeedScaleEstimator()
        for _ in range(50):
            est.update(raw_forward=0.10, measured_forward=0.05, info_xx=1.0)
        self.assertEqual(est.updates, 0)
        self.assertAlmostEqual(est.scale, 1.0, places=9)


class TestGyroBiasEstimator(unittest.TestCase):
    def test_converges_to_true_bias(self):
        est = GyroBiasEstimator(alpha=0.2)
        true_bias = 0.02
        for _ in range(200):
            raw = 1.0 + true_bias  # ジャイロは常に true_bias だけ多く読む
            measured = 1.0          # 点群側は真値を正しく測る
            est.update(raw, measured)
        self.assertAlmostEqual(est.bias, true_bias, delta=0.002)

    def test_stays_within_limit(self):
        est = GyroBiasEstimator(alpha=0.5, limit=0.1)
        for _ in range(100):
            est.update(raw_yaw_rate=100.0, measured_yaw_rate=0.0)
        self.assertLessEqual(abs(est.bias), 0.1 + 1e-9)


if __name__ == "__main__":
    unittest.main()
