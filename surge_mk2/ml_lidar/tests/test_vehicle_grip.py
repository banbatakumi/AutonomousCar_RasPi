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


if __name__ == "__main__":
    unittest.main()
