"""自動停止の予想進路（STM32 `src/control/auto_stop_path.c`）が、舵角どおりに進む車体の掃く範囲を見落とさないこと。

- 総当たり: 車体の矩形を円弧に沿って細かく動かし、どこかで矩形に入る点は必ず「当たる」に数えること
- 直進のときは、それまでの矩形（LiDAR から先端 + 停止距離、左右 ±15cm）と一致すること
- 曲がった先の障害物を拾い、曲がって外れる真っ直ぐ前の点では止まらないこと
- シム（`sim/stm32.py` の `auto_stop_path_contains`）がファームと同じ判定をすること
"""

from __future__ import annotations

import ctypes as C
import math
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[3]))

import sim.stm32 as st  # noqa: E402
from tools.fwsim.fw import FW_DIR  # noqa: E402

FRONT, REAR = st.AUTO_STOP_BODY_FRONT_X_M, st.AUTO_STOP_BODY_REAR_X_M
W = st.AUTO_STOP_HALF_WIDTH_M
LIDAR_X = st.AUTO_STOP_LIDAR_X_M
L = st.AUTO_STOP_WHEELBASE_M


class _Path(C.Structure):
    """`auto_stop_path.h` の `AutoStopPath`（並びも同じ）。"""
    _fields_ = [("forward", C.c_bool), ("straight", C.c_bool), ("turn_sign", C.c_float), ("radius_m", C.c_float),
                ("limit_x_m", C.c_float), ("half_width_m", C.c_float), ("lead_m", C.c_float),
                ("inner_m", C.c_float), ("outer_m", C.c_float), ("r_in_sq", C.c_float), ("r_out_sq", C.c_float),
                ("r_corner_sq", C.c_float), ("r_lead_out_sq", C.c_float), ("cos_sweep", C.c_float), ("sin_sweep", C.c_float),
                ("sweep_all", C.c_bool)]


def _build() -> C.CDLL:
    out = Path(tempfile.mkdtemp()) / "libautostoppath.so"
    src = FW_DIR / "src/control"
    subprocess.run(["cc", "-O2", "-shared", "-fPIC", f"-I{src}", str(src / "auto_stop_path.c"), "-o", str(out), "-lm"],
                   check=True)
    lib = C.CDLL(str(out))
    lib.AutoStopPath_Init.argtypes = [C.POINTER(_Path), C.c_bool, C.c_float, C.c_float, C.c_float, C.c_float,
                                      C.c_float]
    lib.AutoStopPath_Contains.argtypes = [C.POINTER(_Path), C.c_float, C.c_float]
    lib.AutoStopPath_Contains.restype = C.c_bool
    lib.AutoStopPath_Curvature.argtypes = [C.c_float, C.c_float]
    lib.AutoStopPath_Curvature.restype = C.c_float
    return lib


def _swept(px: np.ndarray, py: np.ndarray, forward: bool, k: float, length: float, steps: int = 600) -> np.ndarray:
    """車体の矩形を、先端の中心が length 進むまで円弧（直進なら直線）に沿って動かし、どこかで入る点。"""
    lead = FRONT if forward else -REAR
    sgn = 1.0 if forward else -1.0
    # 先端の中心の速さは後車軸の |(lead, R)| / R 倍
    s_max = length / math.hypot(1.0, lead * k)
    hit = np.zeros(px.shape, dtype=bool)
    for s in np.linspace(0.0, s_max, steps) * sgn:
        th = k * s
        if abs(k) < 1e-9:
            ox, oy = s, 0.0
        else:
            ox, oy = math.sin(th) / k, (1.0 - math.cos(th)) / k
        c, sn = math.cos(th), math.sin(th)
        bx = c * (px - ox) + sn * (py - oy)
        by = -sn * (px - ox) + c * (py - oy)
        hit |= (bx >= REAR) & (bx <= FRONT) & (np.abs(by) <= W)
    return hit


class AutoStopPathTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.lib = _build()

    def contains(self, forward: bool, k: float, length: float, px: np.ndarray, py: np.ndarray) -> np.ndarray:
        p = _Path()
        self.lib.AutoStopPath_Init(C.byref(p), forward, k, FRONT, REAR, W, length)
        return np.array([self.lib.AutoStopPath_Contains(C.byref(p), float(x), float(y)) for x, y in zip(px, py)])

    @staticmethod
    def scanned(forward: bool, n: int, rng: np.random.Generator) -> tuple[np.ndarray, np.ndarray]:
        """LiDAR が進行方向に見る半面（前進なら LiDAR より前、後退なら後ろ）の点。"""
        ahead = rng.uniform(0.0, 2.0, n)
        px = LIDAR_X + ahead if forward else LIDAR_X - ahead
        return px, rng.uniform(-2.0, 2.0, n)

    def test_never_misses_what_the_body_sweeps(self):
        rng = np.random.default_rng(0)
        for forward in (True, False):
            for steer in (0.0, 0.05, -0.15, 0.3, -0.45, 0.52):
                for length in (0.0, 0.15, 0.6, 1.5):
                    k = math.tan(steer) / L
                    with self.subTest(forward=forward, steer=steer, length=length):
                        px, py = self.scanned(forward, 4000, rng)
                        truth = _swept(px, py, forward, k, length)
                        got = self.contains(forward, k, length, px, py)
                        missed = truth & ~got
                        self.assertFalse(missed.any(), f"見落とし {missed.sum()} 点: 例 "
                                         f"({px[missed][0]:.3f}, {py[missed][0]:.3f})" if missed.any() else "")

    def test_never_misses_random_cases(self):
        rng = np.random.default_rng(11)
        for _ in range(120):
            forward = bool(rng.integers(2))
            steer = float(rng.uniform(-0.55, 0.55))
            length = float(rng.choice([0.0, rng.uniform(0.0, 2.5)]))
            k = math.tan(steer) / L
            px, py = self.scanned(forward, 800, rng)
            missed = _swept(px, py, forward, k, length, steps=800) & ~self.contains(forward, k, length, px, py)
            with self.subTest(forward=forward, steer=steer, length=length):
                self.assertFalse(missed.any())

    def test_reverse_catches_the_front_swinging_out(self):
        """右へ切って下がると前端が左へ振り出す。後車軸の左真横（LiDAR より後ろ）の点に当たる。"""
        k, length = math.tan(-0.48) / L, 0.18
        px, py = np.array([0.0068]), np.array([0.2000])
        self.assertTrue(_swept(px, py, False, k, length, steps=2000)[0])
        self.assertTrue(self.contains(False, k, length, px, py)[0])

    def test_extra_hits_stay_close_to_the_swept_area(self):
        """当たるに数えるのに総当たりでは当たらない点は、車体の真横（角度の下限を見ない。従来の矩形と同じ）か、
        掃く範囲の縁から 2cm 以内だけ。"""
        rng = np.random.default_rng(1)
        for forward in (True, False):
            for steer in (0.15, -0.3, 0.5):
                k, length = math.tan(steer) / L, 0.6
                with self.subTest(forward=forward, steer=steer):
                    px, py = self.scanned(forward, 6000, rng)
                    extra = self.contains(forward, k, length, px, py) & ~_swept(px, py, forward, k, length)
                    beside = (px <= FRONT) if forward else (px >= REAR)
                    ex, ey = px[extra & ~beside], py[extra & ~beside]
                    # 少し（2cm）動かせば掃く範囲に入るか: 近傍の格子を総当たりで見る
                    near = np.zeros(ex.shape, dtype=bool)
                    for dx in np.linspace(-0.02, 0.02, 5):
                        for dy in np.linspace(-0.02, 0.02, 5):
                            near |= _swept(ex + dx, ey + dy, forward, k, length, steps=300)
                    self.assertTrue(near.all(), f"掃く範囲から 2cm 以上離れた点を {(~near).sum()} 点拾った")

    def test_straight_matches_the_old_rectangle(self):
        rng = np.random.default_rng(2)
        for forward in (True, False):
            for length in (0.15, 0.9):
                px, py = self.scanned(forward, 3000, rng)
                ahead = (px - LIDAR_X) if forward else (LIDAR_X - px)
                offset = (FRONT - LIDAR_X) if forward else (LIDAR_X - REAR)       # 旧 LIDAR_*_OFFSET（23/14cm）
                old = (ahead <= length + offset) & (np.abs(py) <= W)
                with self.subTest(forward=forward, length=length):
                    np.testing.assert_array_equal(self.contains(forward, 0.0, length, px, py), old)

    def test_turning_sees_the_curve_not_the_straight_line(self):
        steer, length = 0.35, 0.8                       # 左へ大きく切って 0.8m で止まる
        k = math.tan(steer) / L
        r = 1.0 / k
        # 後車軸の円弧の上、0.6m 先（左へ曲がった先）: 矩形では幅の外
        th = 0.6 / r
        on_curve = (r * math.sin(th), r * (1.0 - math.cos(th)))
        self.assertGreater(abs(on_curve[1]), W)
        # 真っ直ぐ前、先端の 0.6m 先: 曲がって外れるので当たらない
        straight_ahead = (FRONT + 0.6, 0.0)
        got = self.contains(True, k, length, np.array([on_curve[0], straight_ahead[0]]),
                            np.array([on_curve[1], straight_ahead[1]]))
        self.assertTrue(got[0], "曲がった先の障害物を見落とした")
        self.assertFalse(got[1], "曲がって通らない真っ直ぐ前の点で止まる")
        # 右へ切れば左右が入れ替わる
        got = self.contains(True, -k, length, np.array([on_curve[0]]), np.array([-on_curve[1]]))
        self.assertTrue(got[0])

    def test_reverse_uses_the_same_turning_centre(self):
        """後退でも舵が同じなら回転の中心は同じ側（左に切って下がると、後ろは左へ回り込む）。"""
        k = math.tan(0.35) / L
        r = 1.0 / k
        th = 0.4 / r
        behind_left = (-r * math.sin(th), r * (1.0 - math.cos(th)))
        got = self.contains(False, k, 0.6, np.array([behind_left[0], behind_left[0]]),
                            np.array([behind_left[1], -behind_left[1]]))
        self.assertTrue(got[0])
        self.assertFalse(got[1])

    def test_curvature_is_the_bicycle_model(self):
        self.assertAlmostEqual(self.lib.AutoStopPath_Curvature(0.3, L), math.tan(0.3) / L, places=5)
        self.assertAlmostEqual(self.lib.AutoStopPath_Curvature(-0.3, L), -math.tan(0.3) / L, places=5)

    def test_sim_matches_firmware(self):
        rng = np.random.default_rng(3)
        for forward in (True, False):
            for steer in (0.0, 0.004, 0.2, -0.4, 0.52):
                for length in (0.15, 1.2):
                    k = math.tan(steer) / L
                    px, py = self.scanned(forward, 3000, rng)
                    fw = self.contains(forward, k, length, px, py)
                    sim = st.auto_stop_path_contains(px, py, forward, k, length)
                    # float と double の丸めで境界ちょうどの点だけ食い違いうる
                    with self.subTest(forward=forward, steer=steer, length=length):
                        self.assertLessEqual(int((fw != sim).sum()), 2)


if __name__ == "__main__":
    unittest.main()
