"""シムの駆動系（`sim/stm32.py`・`sim/vehicle.py` の Python の写し）と、実ファーム（`tools/fwsim`。
ファームをホストでコンパイルして車のモデルと閉ループ）の**ずれを見張る**。

**正は fwsim**（ファームそのもの）。シムの写しは TC/TV・ステアのリンクの換算を持たない（`sim/stm32.py` の
冒頭）ので完全には一致しない。ここでは、走りの判断に効く「車速の指令への追従」だけを、許容差つきで比べる。
同定（`vehicle.toml`）や制御を直してこのテストが落ちたら、シムの写しが実車から離れた合図——シム側を直すか、
許容差を見直した理由を書く。ファームのリポジトリ（`tools/fwsim/fw.FW_DIR`）と C コンパイラが無ければ飛ばす。
"""

from __future__ import annotations

import shutil
import unittest

import numpy as np

from sim.vehicle import DriveInput, VehicleModel, VehicleSpec
from tools.fwsim import scenarios as S
from tools.fwsim.fw import FW_DIR, MODE_SPEED, OUT, firmware
from tools.fwsim.plant import Plant

HAVE_FW = (FW_DIR / "host" / "host_sim.c").exists() and shutil.which("cc") is not None
TARGET_M_S = 1.5
ACCEL_LIMIT = 2.0
DURATION_S = 3.0


def _t_reach(t: np.ndarray, v: np.ndarray, frac: float) -> float:
    idx = np.nonzero(v >= frac * TARGET_M_S)[0]
    return float(t[idx[0]]) if idx.size else float("inf")


def _sim_trace() -> tuple[np.ndarray, np.ndarray]:
    vm = VehicleModel(VehicleSpec.load(), (0.0, 0.0, 0.0))
    vm.apply(DriveInput(armed=True, target_speed=TARGET_M_S, accel_limit=ACCEL_LIMIT))
    dt = 0.001
    n = int(DURATION_S / dt)
    t = np.arange(1, n + 1) * dt
    v = np.empty(n)
    for i in range(n):
        vm.step(dt)
        v[i] = vm.speed
    return t, v


@unittest.skipUnless(HAVE_FW, "ファームのリポジトリか cc が無い")
class TestSimFollowsFirmwareSpeedStep(unittest.TestCase):
    def test_speed_step_matches_within_tolerance(self):
        sc = S.Scenario("drift", "車速の段差", (S.Seg(DURATION_S, MODE_SPEED, TARGET_M_S, accel_limit=ACCEL_LIMIT),))
        res = S.run(firmware(), Plant.load(), sc)
        fw_t, fw_v = res.col("t"), res.col("speed")
        sim_t, sim_v = _sim_trace()
        # 1. 最後の 0.5s の平均車速（定常の追従）: 5% 以内
        fw_end = float(fw_v[fw_t >= DURATION_S - 0.5].mean())
        sim_end = float(sim_v[sim_t >= DURATION_S - 0.5].mean())
        self.assertAlmostEqual(sim_end, fw_end, delta=0.05 * TARGET_M_S,
                               msg=f"定常の車速 シム {sim_end:.3f} / ファーム {fw_end:.3f} m/s")
        # 2. 目標の 90% に届く時刻（加速の速さ）: 0.25s 以内
        fw_t90, sim_t90 = _t_reach(fw_t, fw_v, 0.9), _t_reach(sim_t, sim_v, 0.9)
        self.assertLess(abs(sim_t90 - fw_t90), 0.25,
                        msg=f"90% に届く時刻 シム {sim_t90:.3f} / ファーム {fw_t90:.3f} s")
        self.assertIn("speed", OUT)


if __name__ == "__main__":
    unittest.main()
