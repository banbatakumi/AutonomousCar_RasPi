"""車速 KF（STM32 `src/control/speed_kf.c`）が、制御ループの長い停止から復帰できること。

前輪エンコーダは絶対角で、角度差を ±半周に畳んで増分を出す。ループが止まって半周を超えて回ると、増分は 1 回転ぶん
折り返す（前輪の周速 3m/s で 31ms 以上）。予測した増分に最も近い側へ 1 回転の整数倍を足して戻す（`drive.c`
`UpdateSpeedKf`）。戻さないと、40〜250ms の停止の後に車速が最大 5m/s 狂い、収束に 0.3〜0.8 秒かかった
（2026-10-17、C の KF を単体で動かして確認）。この手順は drive.c と同じ式を Python で再現して KF の C に流す。
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

from tools.ctrl_tune.fw import FW_DIR  # noqa: E402
from tools.ctrl_tune.kf_raw_replay import _Kf  # noqa: E402  speed_kf.h の SpeedKf の写し（並びを1か所に）

R = 0.03                       # 前輪の半径 [m]（drive.h の DRIVE_FRONT_WHEEL_RADIUS_M）
DT = 5e-4


def _build() -> C.CDLL:
    out = Path(tempfile.mkdtemp()) / "libspeedkf.so"
    subprocess.run(["cc", "-O2", "-shared", "-fPIC", f"-I{FW_DIR / 'src/control'}", str(FW_DIR / "src/control/speed_kf.c"),
                    "-o", str(out), "-lm"], check=True)
    lib = C.CDLL(str(out))
    p = C.POINTER(_Kf)
    for name, args in (("SpeedKf_Init", [p]), ("SpeedKf_Predict", [p, C.c_float]),
                       ("SpeedKf_UpdateDistance", [p, C.c_float]), ("SpeedKf_UpdateAccel", [p, C.c_float])):
        getattr(lib, name).argtypes = args
    return lib


def _run(lib: C.CDLL, speed: float, stall_s: float, unwrap: bool, t_stall: float = 1.0, t_end: float = 3.0) -> float:
    """定速 `speed` で走る車の途中で `stall_s` 秒だけループが止まる。停止の後の車速の最大誤差 [m/s]。"""
    rng = np.random.default_rng(0)
    kf = _Kf()
    lib.SpeedKf_Init(C.byref(kf))
    s = 0.0
    prev_meas = 0.0
    last_t = 0.0
    err = 0.0
    n = int(t_end / DT)
    for i in range(n):
        t = i * DT
        s += speed * DT
        if t_stall <= t < t_stall + stall_s:
            continue
        meas = s + rng.normal(0, 1.2e-5)                        # 角度の量子化ノイズ ≒ 0.012mm
        gap = meas - prev_meas if i else 0.0
        dt = t - last_t if i else DT
        prev_meas, last_t = meas, t
        half = np.pi * R
        gap = (gap + half) % (2 * half) - half                  # Encoder: ±半周に畳む
        lib.SpeedKf_Predict(C.byref(kf), dt)
        if unwrap:                                              # drive.c: 予測した増分に近い側の折り返しを選ぶ
            period = 2 * np.pi * R
            gap += period * np.round((kf.x[0] - kf.s_meas - gap) / period)
        lib.SpeedKf_UpdateDistance(C.byref(kf), gap)
        if i % 10 == 0:
            lib.SpeedKf_UpdateAccel(C.byref(kf), 0.0 + rng.normal(0, 0.3))
        if t > t_stall + stall_s:
            err = max(err, abs(kf.x[1] - speed))
    return err


@unittest.skipUnless((FW_DIR / "src" / "control" / "speed_kf.c").exists(), "ファームに speed_kf が無い")
class TestSpeedKfStall(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.lib = _build()

    def test_recovers_after_long_stalls(self):
        """半周を超えて回る長さ（3m/s で 31ms 超）の停止の後も、車速が通常の誤差（約 12mm/s）に収まる。"""
        for stall in (0.040, 0.080, 0.250, 0.5):
            with self.subTest(stall_ms=int(stall * 1e3)):
                self.assertLess(_run(self.lib, 3.0, stall, unwrap=True), 0.03)

    def test_without_unwrapping_it_would_fail(self):
        """検証器の確認: 折り返しを戻さないと、この試験は確かに落ちる（試験が現象を測れている）。"""
        self.assertGreater(_run(self.lib, 3.0, 0.080, unwrap=False), 0.5)

    def test_short_stalls_are_unaffected_by_unwrapping(self):
        for stall in (0.010, 0.030):
            with self.subTest(stall_ms=int(stall * 1e3)):
                self.assertAlmostEqual(_run(self.lib, 3.0, stall, unwrap=True), _run(self.lib, 3.0, stall, unwrap=False),
                                       places=3)

    def test_stall_longer_than_max_dt_resets_to_zero_speed_but_recovers(self):
        """1 秒以上の停止（起動直後の長い Setup など）は初期化し直す。それでも収束はする。"""
        # 初期化直後は車速 0 から数えるので誤差は最大で速度ぶん。止まっていた間の回転（折り返した値）は捨てるので、それを超えない
        self.assertLess(_run(self.lib, 2.0, 1.2, unwrap=True, t_end=4.0), 2.05)


if __name__ == "__main__":
    unittest.main()
