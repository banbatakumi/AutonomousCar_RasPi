"""`sim/vehicle.py` の横方向グリップ限界（`mu`）のテスト。

要求される向心加速度（`speed^2 * tan(steer)/wheelbase`）が `mu * g` を超えたときに
達成できる曲率が頭打ちになる（アンダーステア）ことだけを確認する。厳密なタイヤ物理の
再現ではなく、E2E LiDAR の学習がコーナー前の減速を学ぶ動機を持たせるための簡易な
上限なので、テストも「上限を超えない」「低速では効かない」の2点に留める。
"""

import math
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))  # surge_mk2/

from sim.vehicle import DriveInput, GRAVITY_MPS2, VehicleModel, VehicleSpec  # noqa: E402


def _settle(vehicle: VehicleModel, *, steer: float, speed: float,
           seconds: float = 5.0, dt: float = 0.01) -> VehicleModel:
    """操舵・速度の1次遅れが十分収束するまで同じ指令を送り続ける。"""
    for _ in range(int(seconds / dt)):
        vehicle.apply(DriveInput(armed=True, target_steer=steer, target_speed=speed))
        vehicle.step(dt)
    return vehicle


class TestVehicleGripLimit(unittest.TestCase):
    def test_low_speed_matches_pure_kinematic_yaw_rate(self):
        """向心加速度がグリップ限界を大きく下回る低速では、従来通りの
        自転車運動学モデルの計算結果と一致する（頭打ちが働かない）。"""
        spec = VehicleSpec(mu=0.8)
        v = _settle(VehicleModel(spec, (0.0, 0.0, 0.0)), steer=0.3, speed=0.3)
        kinematic = v.speed / spec.wheelbase * math.tan(v.steer_actual)
        self.assertAlmostEqual(v.yaw_rate, kinematic, places=3)

    def test_high_speed_understeers_below_kinematic_yaw_rate(self):
        """高速+最大舵角では、要求される曲率をそのまま実現できず
        （アンダーステア）、達成ヨーレートが運動学だけの計算値を下回る。"""
        spec = VehicleSpec(mu=0.8)
        v = _settle(VehicleModel(spec, (0.0, 0.0, 0.0)), steer=spec.max_steer, speed=5.0)
        kinematic = v.speed / spec.wheelbase * math.tan(v.steer_actual)
        self.assertLess(abs(v.yaw_rate), abs(kinematic))
        self.assertGreater(v.yaw_rate, 0.0)    # 頭打ちでも向き(符号)は保たれる

    def test_lateral_acceleration_never_exceeds_grip_limit(self):
        spec = VehicleSpec(mu=0.8)
        a_lat_max = spec.mu * GRAVITY_MPS2
        for speed in (0.5, 1.0, 2.0, 5.0, 10.0):
            v = _settle(VehicleModel(spec, (0.0, 0.0, 0.0)), steer=spec.max_steer, speed=speed)
            self.assertLessEqual(abs(v.accel_lateral), a_lat_max + 1e-6)

    def test_saturated_case_reaches_exactly_the_grip_limit(self):
        """限界を大きく超える要求なら、実現される向心加速度は`mu*g`にほぼ張り付く。"""
        spec = VehicleSpec(mu=0.8)
        a_lat_max = spec.mu * GRAVITY_MPS2
        v = _settle(VehicleModel(spec, (0.0, 0.0, 0.0)), steer=spec.max_steer, speed=5.0)
        self.assertAlmostEqual(abs(v.accel_lateral), a_lat_max, delta=0.05)

    def test_mu_loaded_from_toml_default(self):
        spec = VehicleSpec.load()
        self.assertGreater(spec.mu, 0.0)

    def test_slip_frac_zero_when_within_grip_limit(self):
        spec = VehicleSpec(mu=0.8)
        v = _settle(VehicleModel(spec, (0.0, 0.0, 0.0)), steer=0.3, speed=0.3)
        self.assertEqual(v.slip_frac, 0.0)

    def test_slip_frac_positive_when_grip_limit_saturated(self):
        """`test_saturated_case_reaches_exactly_the_grip_limit`と同条件——
        頭打ちが働いているときは`slip_frac`が正になる。"""
        spec = VehicleSpec(mu=0.8)
        v = _settle(VehicleModel(spec, (0.0, 0.0, 0.0)), steer=spec.max_steer, speed=5.0)
        self.assertGreater(v.slip_frac, 0.0)

    def test_braking_mid_corner_reduces_available_lateral_accel(self):
        """摩擦円によるRWD連成——駆動・制動はどちらも後輪だけが担うため
        （`_next_speed()`参照）、定常円旋回中に急制動すると同じ後輪タイヤの
        横方向の余力が減り、達成できる横加速度が`mu*g`単独のときより下がる。"""
        spec = VehicleSpec(mu=0.8)
        a_lat_max = spec.mu * GRAVITY_MPS2
        v = _settle(VehicleModel(spec, (0.0, 0.0, 0.0)), steer=spec.max_steer, speed=5.0)
        baseline = abs(v.accel_lateral)
        self.assertAlmostEqual(baseline, a_lat_max, delta=0.05)

        dt = 0.02
        v.apply(DriveInput(armed=True, brake=True, target_steer=spec.max_steer))
        v.step(dt)

        expected_decel = v.MAX_BRAKE_TORQUE_NM * spec.drive_ratio / (spec.wheel_radius * spec.mass)
        expected_a_lat_max = math.sqrt(max(0.0, a_lat_max ** 2 - expected_decel ** 2))
        self.assertLess(abs(v.accel_lateral), baseline - 0.5)
        self.assertAlmostEqual(abs(v.accel_lateral), expected_a_lat_max, delta=0.05)

    def test_lateral_acceleration_never_exceeds_grip_limit_at_measured_mu(self):
        """`mu=0.8`（合成値）ではなく`config/vehicle.toml`の実測値
        （`VehicleSpec.load()`）でも上限を超えないことを確認する——
        `mu`は実測されたが摩擦円が組み合わせる加減速側の定数
        （`MAX_BRAKE_TORQUE_NM`・`DRIVE_MAX_ACCEL_M_S2`）は未実測のままなので、
        実際どこまで削られるかは合成値`mu=0.8`のテストだけでは分からない。"""
        spec = VehicleSpec.load()
        a_lat_max = spec.mu * GRAVITY_MPS2
        for speed in (0.5, 1.0, 2.0, 5.0):
            v = _settle(VehicleModel(spec, (0.0, 0.0, 0.0)), steer=spec.max_steer, speed=speed)
            self.assertLessEqual(abs(v.accel_lateral), a_lat_max + 1e-6)

    def test_braking_mid_corner_at_measured_mu_can_zero_out_lateral_grip(self):
        """`test_braking_mid_corner_reduces_available_lateral_accel`と同じ現象を
        実測`mu`（`config/vehicle.toml`。約0.454）で確認する。実測`mu*g`（≈4.46m/s²）
        は`MAX_BRAKE_TORQUE_NM`から逆算される後輪の制動減速度（≈5.0m/s²）を下回るため、
        `mu=0.8`の合成値のテストでは緩やかに減るだけだった横方向グリップが、
        実測値では定常円旋回中の急制動でほぼゼロまで削られる——摩擦円のRWD連成が
        実際どの程度効くかは、`mu`だけでなく未実測の制動側の定数にも左右される
        ことを示す回帰テスト（`docs/`のシステム同定に加減速試験を追加する動機）。"""
        spec = VehicleSpec.load()
        a_lat_max = spec.mu * GRAVITY_MPS2
        decel = VehicleModel.MAX_BRAKE_TORQUE_NM * spec.drive_ratio / (spec.wheel_radius * spec.mass)

        v = _settle(VehicleModel(spec, (0.0, 0.0, 0.0)), steer=spec.max_steer, speed=5.0)
        v.apply(DriveInput(armed=True, brake=True, target_steer=spec.max_steer))
        v.step(0.02)

        expected_a_lat_max = math.sqrt(max(0.0, a_lat_max ** 2 - decel ** 2))
        self.assertAlmostEqual(abs(v.accel_lateral), expected_a_lat_max, delta=0.05)
        if decel >= a_lat_max:
            self.assertLess(abs(v.accel_lateral), 0.1)

    def test_measured_brake_decel_overrides_torque_derived_value(self):
        """`brake_decel_m_s2`（`sysid_accel`の実測値）が設定されていれば、
        `MAX_BRAKE_TORQUE_NM`からの逆算より優先される。"""
        spec = VehicleSpec(mu=0.8, brake_decel_m_s2=2.0)
        v = VehicleModel(spec, (0.0, 0.0, 0.0))
        v.apply(DriveInput(armed=True, target_speed=5.0))
        for _ in range(500):
            v.step(0.01)
        speed_before = v.speed
        v.apply(DriveInput(armed=True, brake=True))
        v.step(0.1)
        self.assertAlmostEqual(speed_before - v.speed, 2.0 * 0.1, delta=0.01)

    def test_measured_drive_accel_caps_speed_tracking(self):
        """`drive_accel_m_s2`（`sysid_accel`の実測値）は`cmd.accel_limit`が
        指定されていない呼び出し元（`sim/gym_env.py`のRL訓練など）でも一律に効く。"""
        spec = VehicleSpec(mu=0.8, tau_speed_s=0.0, drive_accel_m_s2=1.0)
        v = VehicleModel(spec, (0.0, 0.0, 0.0))
        v.apply(DriveInput(armed=True, target_speed=10.0))
        v.step(0.1)
        self.assertAlmostEqual(v.speed, 0.1, delta=1e-6)

    def test_steady_cornering_a_lat_max_matches_mu_g(self):
        """縦加速度がゼロに収束した定常円旋回では、摩擦円連成を入れても
        従来通り`mu*g`単独の限界に一致する（`sysid_corner.py`の測定条件・
        既存テストの前提と矛盾しないことの確認）。"""
        spec = VehicleSpec(mu=0.8)
        v = _settle(VehicleModel(spec, (0.0, 0.0, 0.0)), steer=spec.max_steer, speed=5.0)
        self.assertAlmostEqual(v.accel_x, 0.0, delta=1e-3)
        self.assertAlmostEqual(v._a_lat_max, spec.mu * GRAVITY_MPS2, delta=1e-6)


if __name__ == "__main__":
    unittest.main()
