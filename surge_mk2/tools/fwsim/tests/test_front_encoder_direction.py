"""前輪エンコーダの読みの鏡像（STM32 `src/control/encoder_direction.c`）を後輪の速度で直せること。

2026-10-31 の会場の試走（surge_20261031_044620）で、近道の出口の同じ場所を通るたびに前輪の左右の生の角度が
「2π − 本来の角度」になり（AS5600 の DIR が切り替わったときと同じ出方）、約1秒後に戻った。車速 KF が
-1.5〜-2.0m/s になり、TC がトルクを切る・MPC が舵を逆に振る・SLAM が崩れる、でコースから外れていた。
LiDAR を地図に合わせた車体の速度では車は前進しており（前輪の中心は車輪の向きに +1.2〜1.5m/s、横滑りほぼ 0）、
後輪の周速はそれと一致していた（後輪は滑っていない）。

- 合成: 2kHz で角度を読む車に鏡像を入れ、向きの判定と車速 KF（drive.c の UpdateSpeedKf と同じ順）に流す
- 誤判定しないこと: 後退・後輪のロック・後輪の空転・静止
- 実記録: 上の記録の抜粋（`data/front_encoder_mirror_20261031.npz`、100Hz の生の角度と後輪の周速）を流し、
  LiDAR の車体速度と比べる
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
from tools.fwsim.kf_raw_replay import _Kf  # noqa: E402

R = 0.03                       # 前輪の半径 [m]（drive.h の DRIVE_FRONT_WHEEL_RADIUS_M）
DIRS = (-1.0, 1.0)             # drive.h の DRIVE_FRONT_LEFT_DIR / RIGHT_DIR
DT = 5e-4
TWO_PI = 2 * np.pi
DATA = Path(__file__).parent / "data" / "front_encoder_mirror_20261031.npz"


class _Dir(C.Structure):
    """`encoder_direction.h` の `EncoderDirection`（並びも同じ）。"""
    _fields_ = [("sign", C.c_float), ("front_m", C.c_float), ("rear_m", C.c_float), ("window_s", C.c_float),
                ("slow_front_m", C.c_float), ("slow_rear_m", C.c_float), ("slow_window_s", C.c_float),
                ("flips", C.c_uint32), ("jumps", C.c_uint32)]


def _build() -> C.CDLL:
    out = Path(tempfile.mkdtemp()) / "libencdir.so"
    src = FW_DIR / "src/control"
    subprocess.run(["cc", "-O2", "-shared", "-fPIC", f"-I{src}", str(src / "speed_kf.c"), str(src / "encoder_direction.c"),
                    "-o", str(out), "-lm"], check=True)
    lib = C.CDLL(str(out))
    pk, pd = C.POINTER(_Kf), C.POINTER(_Dir)
    for name, args in (("SpeedKf_Init", [pk]), ("SpeedKf_Predict", [pk, C.c_float]),
                       ("SpeedKf_UpdateDistance", [pk, C.c_float]), ("SpeedKf_UpdateAccel", [pk, C.c_float]),
                       ("EncoderDirection_Init", [pd])):
        getattr(lib, name).argtypes = args
    lib.EncoderDirection_Apply.argtypes = [pd, C.c_float, C.c_float, C.c_float, C.c_float]
    lib.EncoderDirection_Apply.restype = C.c_float
    return lib


class _Pipeline:
    """drive.c の UpdateFrontDirection → UpdateSpeedKf と同じ順で、生の角度から車速を出す。"""

    def __init__(self, lib: C.CDLL, fix: bool = True) -> None:
        self.lib, self.fix = lib, fix
        self.kf = _Kf()
        lib.SpeedKf_Init(C.byref(self.kf))
        self.dirs = [_Dir(), _Dir()]
        for d in self.dirs:
            lib.EncoderDirection_Init(C.byref(d))
        self.prev: np.ndarray | None = None

    def step(self, raw: np.ndarray, rear_m_s: float, accel: float, dt: float, steer: float = 0.0) -> float:
        if self.prev is None:
            self.prev = raw.copy()
            return 0.0
        gaps = (raw - self.prev + np.pi) % TWO_PI - np.pi            # Encoder: ±半周に畳む
        self.prev = raw.copy()
        fwd = []
        v_prev = float(self.kf.x[1])                                  # 前周期の KF（drive.c の vehicle_speed_m_s）
        for k in range(2):
            g = gaps[k] * DIRS[k]
            if self.fix:
                g = self.lib.EncoderDirection_Apply(C.byref(self.dirs[k]), float(g * R), rear_m_s, v_prev, dt) / R
            fwd.append(g)
        cos_steer = np.cos(steer)
        self.lib.SpeedKf_Predict(C.byref(self.kf), dt)
        expected = self.kf.predicted_step_m / (R * cos_steer)
        fwd = [g + TWO_PI * np.round((expected - g) / TWO_PI) for g in fwd]
        self.lib.SpeedKf_UpdateDistance(C.byref(self.kf), 0.5 * (fwd[0] + fwd[1]) * R * cos_steer)
        self.lib.SpeedKf_UpdateAccel(C.byref(self.kf), accel)
        return float(self.kf.x[1])

    @property
    def flips(self) -> list[int]:
        return [d.flips for d in self.dirs]


def _drive(lib: C.CDLL, secs: float, v_car, v_rear, mirror: tuple = ((), ()),
           fix: bool = True, seed: int = 0) -> tuple[np.ndarray, np.ndarray, _Pipeline]:
    """速度 `v_car` で走る車の生の角度（取付向きの符号のまま）を 2kHz で読む。`mirror[k]` の (開始, 終了) の間は
    その輪の読みを 2π − θ にする。後輪の速度は `v_rear`（空転・ロック・逆回転を模す）。速度は定数か時刻の関数。"""
    car = v_car if callable(v_car) else (lambda _t, c=v_car: c)
    rear = v_rear if callable(v_rear) else (lambda _t, c=v_rear: c)
    rng = np.random.default_rng(seed)
    p = _Pipeline(lib, fix)
    n = int(round(secs / DT))
    t = np.arange(n) * DT
    out = np.zeros(n)
    s = 0.0
    for i in range(n):
        v = car(t[i])
        s += v * DT
        raw = np.empty(2)
        for k in range(2):
            theta = DIRS[k] * s / R + rng.normal(0, 0.002)
            if any(a <= t[i] < b for a, b in mirror[k]):
                theta = -theta
            raw[k] = theta % TWO_PI
        out[i] = p.step(raw, rear(t[i]) + rng.normal(0, 0.02), rng.normal(0, 0.3), DT)
    return t, out, p


@unittest.skipUnless((FW_DIR / "src/control/encoder_direction.c").exists(), "ファームに encoder_direction が無い")
class FrontEncoderDirectionTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.lib = _build()

    def test_mirror_is_corrected(self) -> None:
        """記録と同じ形（左右が 60ms ずれて約1秒鏡像）。直さないと車速が負になり、直せば前進のまま。"""
        mirror = (((1.06, 2.00),), ((1.00, 2.02),))
        t, v_raw, p_raw = _drive(self.lib, 3.0, 1.4, 1.4, mirror, fix=False)
        self.assertLess(v_raw[(t > 1.2) & (t < 1.9)].mean(), -1.0)      # 前提: 直さなければ記録と同じく負になる
        t, v, p = _drive(self.lib, 3.0, 1.4, 1.4, mirror)
        self.assertEqual(p.flips, [2, 2])
        # 切り替わりの瞬間も含めて外れない（飛びを後輪の距離で置き換え、窓を待たずに向きを直すため）
        self.assertLess(np.abs(v[t > 0.5] - 1.4).max(), 0.05)

    def test_mirror_during_rear_wheelspin(self) -> None:
        """後輪が空転していても（後輪 3m/s・車 1m/s）鏡像を拾う。"""
        t, v, p = _drive(self.lib, 2.5, 1.0, 3.0, (((1.0, 2.0),), ((1.0, 2.0),)))
        self.assertEqual(p.flips, [2, 2])
        self.assertGreater(v[(t > 1.1) & (t < 1.9)].min(), 0.8)

    def test_no_false_flip(self) -> None:
        """鏡像が無ければ向きを変えない: 後退・後輪のロック（ABS）・後輪の空転・静止。"""
        for name, v_car, v_rear in (("後退", -1.0, -1.0), ("後輪ロック", 1.5, 0.0), ("空転", 0.5, 4.0),
                                    ("静止", 0.0, 0.0), ("低速の後輪の雑音", 0.0, 0.25)):
            with self.subTest(name):
                _, _, p = _drive(self.lib, 1.5, v_car, v_rear)
                self.assertEqual(p.flips, [0, 0])

    def test_rear_spinning_backward(self) -> None:
        """低μの旋回中の回生で後輪が後ろへ回っても（シムの低μ×0.6 で後輪 -0.4m/s・車 +1.4m/s）前輪を反転しない。
        後輪だけを基準にしていた版はここで前輪を反転し、車速が 2.7m/s 狂った。"""
        rear = lambda t: -0.4 if 1.0 <= t < 1.05 else 1.4               # noqa: E731
        t, v, p = _drive(self.lib, 2.0, 1.4, rear)
        self.assertEqual(p.flips, [0, 0])
        self.assertLess(np.abs(v[t > 0.5] - 1.4).max(), 0.05)

    def test_mirror_from_standstill(self) -> None:
        """止まっている間に鏡像になってから走り出す: 車速の推定が先に負へ狂うので毎回の直しは働かないが、
        立て直し（後輪と大きさが近く向きが逆）で 0.3s 以内に戻る。"""
        car = lambda t: 0.0 if t < 0.5 else min(1.2, 3.0 * (t - 0.5))   # noqa: E731
        t, v, p = _drive(self.lib, 2.0, car, car, (((0.2, 9.0),), ((0.2, 9.0),)))
        self.assertEqual(p.flips, [1, 1])
        ok = t > 1.2
        self.assertLess(np.abs(v[ok] - 1.2).max(), 0.05)

    def test_real_log(self) -> None:
        """実記録の抜粋: 直した車速は LiDAR の車体速度と合い、負にならない。"""
        d = np.load(DATA)
        t, tc = d["t"].astype(float), d["t_capture"]
        rear = d["rear_speed"].mean(axis=1)
        seg = np.r_[True, np.diff(t) > 0.5]                              # 抜粋は2区間。区間の頭で作り直す
        v_fix, v_raw = np.zeros(len(t)), np.zeros(len(t))
        flips = [0, 0]
        p_fix = p_raw = None
        for i in range(len(t)):
            if seg[i]:
                if p_fix is not None:
                    flips = [a + b for a, b in zip(flips, p_fix.flips)]
                p_fix, p_raw = _Pipeline(self.lib), _Pipeline(self.lib, fix=False)
                dt = 0.01
            else:
                dt = float(tc[i] - tc[i - 1])
            prev_rear = rear[i - 1] if not seg[i] else rear[i]
            for p, out in ((p_fix, v_fix), (p_raw, v_raw)):
                out[i] = p.step(d["front_angle"][i].astype(float), float(prev_rear), float(d["accel_x"][i]), dt,
                                float(d["steer"][i]))
        flips = [a + b for a, b in zip(flips, p_fix.flips)]
        lt, lv = d["lidar_t"].astype(float), d["lidar_vx"].astype(float)
        moving = lv > 0.3
        near = [np.abs(t - x) < 0.1 for x in lt[moving]]
        e_fix = np.array([v_fix[m].mean() for m in near]) - lv[moving]
        e_raw = np.array([v_raw[m].mean() for m in near]) - lv[moving]
        e_fw = np.array([d["speed_fw"][m].mean() for m in near]) - lv[moving]
        self.assertLess(e_fw.min(), -2.0)                                 # 前提: 実機の車速は大きく負へ外れていた
        self.assertLess(e_raw.min(), -2.0)                                # 直さない再生も同じく外れる
        self.assertLess(np.abs(e_fix).max(), 0.35)
        self.assertLess(np.median(np.abs(e_fix)), 0.1)
        self.assertEqual(flips[0], 2)                                     # 左はヘアピンで鏡像になり戻る1回だけ
        self.assertGreater(flips[1], 6)                                   # 右は別のコーナーでも短く何度も鏡像になる
        self.assertGreater(v_fix[(t > 23.5) & (t < 26.0)].min(), 0.0)


if __name__ == "__main__":
    unittest.main()
