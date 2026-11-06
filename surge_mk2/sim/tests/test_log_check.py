"""`sim/log_check.py` の物差しそのものを確かめる（検証器を先に疑う）。

シムの車が作った記録を「実機の記録」として入れたら、シムとの差はほぼ 0・ヨー比は実機もシムも同じになるはず。
グリップの上限を変えた車の記録なら、上限を超えた帯でだけヨー比が食い違うはず。
"""

import math
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))  # surge_mk2/

from sim.log_check import check  # noqa: E402
from sim.vehicle import DriveInput, VehicleModel, VehicleSpec  # noqa: E402
from tools.calib.log import build  # noqa: E402

DT = 0.001
TELEM_DT = 0.01


def _record(spec: VehicleSpec, seconds: float = 24.0):
    """スラロームしながら車速を 1→3.5m/s で上げ下げした走行を、シムの車で作る（100Hz の TELEMETRY・50Hz の指令）。"""
    m = VehicleModel(spec, (0.0, 0.0, 0.0))
    vs_rows, cmd_rows = [], []
    t = 0.0
    cmd = DriveInput()
    for i in range(int(seconds / DT)):
        if i % 20 == 0:          # 50Hz
            speed = 2.25 + 1.25 * math.sin(2 * math.pi * t / 8.0)
            steer = 0.35 * math.sin(2 * math.pi * t / 1.7)
            cmd = DriveInput(armed=True, target_speed=speed, target_steer=steer, accel_limit=4.0,
                             steer_rate_limit=7.0)
            cmd_rows.append((t, {"arm": True, "target_speed": speed, "target_steer": steer, "accel_limit": 4.0,
                                 "steer_rate_limit": 7.0, "brake": False, "brake_torque": 0.0,
                                 "torque_mode": False, "target_torque": 0.0}))
        m.apply(cmd)
        m.step(DT)
        t += DT
        if i % int(TELEM_DT / DT) == 0:
            vs_rows.append((t, {"speed": m.speed, "yaw_rate": m.yaw_rate, "steer_actual": m.steer_actual,
                                "armed": 1.0}))
    return build(vs_rows, cmd_rows, [])


class TestLogCheck(unittest.TestCase):
    def test_sim_against_itself_has_no_error(self):
        spec = VehicleSpec.load()
        res = check(_record(spec), spec, {}, latency=0.0)
        self.assertGreater(res["moving_samples"], 1000)
        self.assertLess(res["steer"]["rms"], 0.005)
        self.assertLess(res["speed"]["rms"], 0.03)
        for band, r in res["yaw_ratio_by_ay_kin"].items():
            if r["real"] is not None:
                self.assertAlmostEqual(r["real"]["med"], r["sim"]["med"], delta=0.03, msg=band)

    def test_detects_higher_grip_only_above_the_limit(self):
        """実機（記録）の方がグリップが高いと、シムの上限を超えた帯でだけシムのヨー比が下がる。"""
        spec = VehicleSpec.load()
        grippy = VehicleSpec.load()
        grippy.mu = spec.mu * 1.5
        res = check(_record(grippy), spec, {}, latency=0.0)
        lim = spec.mu * 9.81
        low = res["yaw_ratio_by_ay_kin"]["1-2"]
        self.assertAlmostEqual(low["real"]["med"], low["sim"]["med"], delta=0.03)
        high = [r for k, r in res["yaw_ratio_by_ay_kin"].items()
                if r["real"] is not None and float(k.split("-")[0]) >= lim + 1.0]
        self.assertTrue(high, "上限を超える帯の標本が無い（記録の作り方を見直す）")
        for r in high:
            self.assertLess(r["sim"]["med"], r["real"]["med"] - 0.1)


if __name__ == "__main__":
    unittest.main()
