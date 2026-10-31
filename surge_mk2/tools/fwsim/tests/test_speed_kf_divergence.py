"""車速 KF（STM32 `src/control/speed_kf.c`）が、測定から大きく外れても自分で戻れること。

2026-10-31、実機で車速が 32.767（TELEMETRY の i16 の上限）に張り付き、STM32 のリセットまで戻らなかった。
走行距離の残差は累積値なので、一度 6σ を超えるとゲートが効きっぱなしになり、加速度 a とバイアス b が逆向きに
膨らんで車速が上がり続け、最後は 1 回転の折り返しの補正（drive.c）が外れた推定に測定を合わせて 188m/s 付近に
張り付いていた。引き金は推定のずれ（2m/s 以上）・前輪の空転（加速度計は 0）・加速度計（±2g）が飽和する衝突。
ゲートが 20ms 続いたら測り直す・折り返しの補正の基準をその周期の予測の増分にする・物理的にありえない車速での初期化で直した。
drive.c の折り返しの補正は Python で写して KF の C に流す（test_speed_kf_stall と同じ）。
"""

from __future__ import annotations

import ctypes as C
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[3]))

from tools.fwsim.fw import FW_DIR  # noqa: E402
from tools.fwsim.kf_raw_replay import _Kf, build  # noqa: E402

R = 0.03     # 前輪の半径 [m]（drive.h の DRIVE_FRONT_WHEEL_RADIUS_M）
DT = 5e-4
G2 = 19.6    # 加速度計のフルスケール（±2g、mpu6050.c）


class _Car:
    """前輪の絶対角エンコーダ（±半周に畳む）と加速度計を持つ車。drive.c の UpdateSpeedKf と同じ順で KF に流す。"""

    def __init__(self, lib: C.CDLL) -> None:
        self.lib = lib
        self.kf = _Kf()
        lib.SpeedKf_Init(C.byref(self.kf))
        self.rng = np.random.default_rng(0)
        self.v = 0.0
        self.s = 0.0           # 前輪が転がった距離（空転を含む）
        self.prev = 0.0
        self.i = 0

    def run(self, secs: float, accel: float = 0.0, accel_meas: float | None = None) -> float:
        """`secs` 秒、前輪の周速を `accel` で変える。加速度計は `accel_meas`（既定は真値を ±2g で切った値）。"""
        period, half = 2 * np.pi * R, np.pi * R
        for _ in range(int(round(secs / DT))):
            self.v += accel * DT
            self.s += self.v * DT
            meas = self.s + self.rng.normal(0, 1.2e-5)
            gap = (meas - self.prev + half) % period - half          # Encoder: ±半周に畳む
            self.prev = meas
            self.lib.SpeedKf_Predict(C.byref(self.kf), DT)
            gap += period * np.round((self.kf.predicted_step_m - gap) / period)
            self.lib.SpeedKf_UpdateDistance(C.byref(self.kf), gap)
            if self.i % 10 == 0:
                a = np.clip(accel, -G2, G2) if accel_meas is None else accel_meas
                self.lib.SpeedKf_UpdateAccel(C.byref(self.kf), a + self.rng.normal(0, 0.3))
            self.i += 1
        return self.kf.x[1]


@unittest.skipUnless((FW_DIR / "src" / "control" / "speed_kf.c").exists(), "ファームに speed_kf が無い")
class TestSpeedKfDivergence(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.lib = build()

    def _car(self) -> _Car:
        car = _Car(self.lib)
        car.run(0.5)
        return car

    def test_struct_mirror_matches_c_layout(self):
        """検証器の確認: Python の `_Kf` の並びが C の `SpeedKf` と同じ（ずれると下の試験が別の値を読む）。"""
        names = ("x", "s_meas", "gated_count", "diag", "last_dt_s", "gated_run_s", "gated_run_m", "resync_count",
                 "predicted_step_m")
        src = Path(tempfile.mkdtemp()) / "off.c"
        src.write_text('#include <stdio.h>\n#include <stddef.h>\n#include "speed_kf.h"\nint main(void){'
                       + "".join(f'printf("%zu\\n", offsetof(SpeedKf, {n}));' for n in names)
                       + 'printf("%zu\\n", sizeof(SpeedKf));return 0;}', encoding="utf-8")
        exe = src.with_suffix("")
        subprocess.run(["cc", f"-I{FW_DIR / 'src/control'}", str(src), "-o", str(exe)], check=True)
        out = [int(v) for v in subprocess.run([str(exe)], check=True, capture_output=True, text=True).stdout.split()]
        self.assertEqual(out[:-1], [getattr(_Kf, n).offset for n in names])
        self.assertLessEqual(out[-1], C.sizeof(_Kf))

    def test_recovers_from_injected_speed_error(self):
        for dv in (2.0, 8.0, 50.0, 377.0):
            with self.subTest(dv=dv):
                car = self._car()
                car.kf.x[1] += dv
                self.assertLess(abs(car.run(1.0)), 0.05)

    def test_recovers_after_front_wheel_spins_in_air(self):
        """前輪が浮いて空転（加速度計は 0）。空転中は前輪の周速を追い、接地して止まれば 0 に戻る。"""
        for top in (3.0, 10.0):
            with self.subTest(top=top):
                car = self._car()
                car.run(0.05, accel=top / 0.05, accel_meas=0.0)
                self.assertAlmostEqual(car.run(0.3, accel_meas=0.0), top, delta=0.1)
                car.v = 0.0
                self.assertLess(abs(car.run(1.0)), 0.05)

    def test_recovers_after_crash_beyond_accel_range(self):
        """加速度計が ±2g で飽和する衝突停止。"""
        for v0, stop in ((2.0, 0.010), (3.0, 0.020), (4.0, 0.050)):
            with self.subTest(v0=v0, stop_ms=stop * 1e3):
                car = self._car()
                car.run(v0 / 2.0, accel=2.0)
                car.run(stop, accel=-v0 / stop)
                car.v = 0.0
                self.assertLess(abs(car.run(1.0)), 0.05)

    def test_normal_driving_never_resyncs(self):
        """通常の加減速（±6m/s²）と定速では、測り直しは起きない（ゲートが続かない）。"""
        car = self._car()
        for a, t in ((6.0, 0.5), (0.0, 1.0), (-6.0, 0.5), (3.0, 0.5), (-3.0, 0.5)):
            car.run(t, accel=a)
        self.assertEqual(car.kf.resync_count, 0)

    def test_large_steady_accel_offset_is_absorbed_by_bias(self):
        """止まった車を手で傾けて持つ（加速度計がずっと 10m/s² を言う）。バイアスが吸収し、偽の車速を出さない。
        バイアスに上限（例えば 3m/s²）を掛けると、残りを加速度が持って車速が 0.18m/s ずれた（だから上限は置かない）。"""
        car = self._car()
        car.run(1.0, accel_meas=10.0)
        self.assertLess(abs(car.kf.x[1]), 0.05)
        self.assertEqual(car.kf.resync_count, 0)


if __name__ == "__main__":
    unittest.main()
