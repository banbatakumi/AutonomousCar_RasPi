"""ステレオの障害物を自動停止に足す判定（STM32 `src/control/stereo_obstacle.c`）。

- 直近 2 フレームの両方で予想進路に点があるときだけ当たり（単発の誤検知では止まらない）
- 古いフレーム・途絶後の 1 枚目・後退・車体の内側の点では当たらない
- 撮影からの古さ（age）ぶん、予想進路を v×age だけ延ばす
- 点が 0 個のフレームは「生きている」として受ける（IsFresh）
"""

from __future__ import annotations

import ctypes as C
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[3]))

from tools.fwsim.fw import FW_DIR  # noqa: E402

MAXP = 40


class _Frame(C.Structure):
    _fields_ = [("valid", C.c_bool), ("count", C.c_uint8), ("rx_us", C.c_uint32), ("age_rx_us", C.c_uint32),
                ("x_m", C.c_float * MAXP), ("y_m", C.c_float * MAXP)]


class _Obs(C.Structure):
    _fields_ = [("frames", _Frame * 3)]


class _Params(C.Structure):
    _fields_ = [("body_front_x_m", C.c_float), ("body_rear_x_m", C.c_float), ("body_half_width_m", C.c_float),
                ("path_half_width_m", C.c_float), ("curvature_1_m", C.c_float), ("d_stop_m", C.c_float),
                ("speed_abs_m_s", C.c_float), ("confirm_period_s", C.c_float), ("comm_delay_s", C.c_float),
                ("max_age_s", C.c_float), ("min_points", C.c_uint8), ("confirm_frames", C.c_uint8),
                ("self_ext_front_m", C.c_float), ("self_ext_side_m", C.c_float)]


def _build() -> C.CDLL:
    out = Path(tempfile.mkdtemp()) / "libstereoobs.so"
    src = FW_DIR / "src/control"
    subprocess.run(["cc", "-O2", "-shared", "-fPIC", f"-I{src}", str(src / "stereo_obstacle.c"),
                    str(src / "auto_stop_path.c"), "-o", str(out), "-lm"], check=True)
    lib = C.CDLL(str(out))
    lib.StereoObstacles_Init.argtypes = [C.POINTER(_Obs)]
    lib.StereoObstacles_Push.argtypes = [C.POINTER(_Obs), C.c_uint32, C.c_uint16, C.c_uint8, C.POINTER(C.c_int16)]
    lib.StereoObstacles_IsFresh.argtypes = [C.POINTER(_Obs), C.c_uint32, C.c_float]
    lib.StereoObstacles_IsFresh.restype = C.c_bool
    lib.StereoObstacles_Evaluate.argtypes = [C.POINTER(_Obs), C.c_uint32, C.c_bool, C.POINTER(_Params)]
    lib.StereoObstacles_Evaluate.restype = C.c_bool
    return lib


LIB = _build()


def params(**kw) -> _Params:
    d = dict(body_front_x_m=0.30, body_rear_x_m=-0.05, body_half_width_m=0.09, path_half_width_m=0.15,
             curvature_1_m=0.0, d_stop_m=1.0, speed_abs_m_s=2.0, confirm_period_s=0.1, comm_delay_s=0.01,
             max_age_s=0.3, min_points=3, confirm_frames=2, self_ext_front_m=0.0, self_ext_side_m=0.0)
    d.update(kw)
    return _Params(**d)


def push(obs, t_us: int, pts_m, age_ms: int = 50) -> None:
    arr = (C.c_int16 * (2 * len(pts_m)))(*[int(round(v * 1000)) for p in pts_m for v in p])
    LIB.StereoObstacles_Push(C.byref(obs), t_us, age_ms, len(pts_m), arr)


def new():
    o = _Obs()
    LIB.StereoObstacles_Init(C.byref(o))
    return o


def ev(obs, t_us: int, forward: bool = True, p: _Params | None = None) -> bool:
    return bool(LIB.StereoObstacles_Evaluate(C.byref(obs), t_us, forward, C.byref(p or params())))


WALL = [(1.0, y) for y in (-0.05, 0.0, 0.05)]  # 先端 0.3m から 0.7m 先


class StereoObstacleTest(unittest.TestCase):
    def test_two_frames_needed(self):
        o = new()
        push(o, 100_000, WALL)
        self.assertFalse(ev(o, 105_000), "1 枚だけでは止めない")
        push(o, 200_000, WALL)
        self.assertTrue(ev(o, 205_000))

    def test_single_frame_noise_does_not_stop(self):
        o = new()
        push(o, 100_000, WALL)
        push(o, 200_000, [])  # 次は何も見えない
        self.assertFalse(ev(o, 205_000))
        push(o, 300_000, WALL)
        self.assertFalse(ev(o, 305_000), "間に空きのあるフレームは連続と見ない")

    def test_stale_frame_ignored(self):
        o = new()
        push(o, 100_000, WALL)
        push(o, 200_000, WALL)
        self.assertTrue(ev(o, 250_000))
        self.assertFalse(ev(o, 600_000), "0.3s 超の古さは使わない（LiDAR・超音波だけに戻る）")

    def test_gap_after_dropout(self):
        o = new()
        push(o, 100_000, WALL)
        push(o, 900_000, WALL)  # 途絶の後の 1 枚目
        self.assertFalse(ev(o, 905_000))

    def test_reverse_not_checked(self):
        o = new()
        push(o, 100_000, WALL)
        push(o, 200_000, WALL)
        self.assertFalse(ev(o, 205_000, forward=False))

    def test_min_points(self):
        o = new()
        two = WALL[:2]
        push(o, 100_000, two)
        push(o, 200_000, two)
        self.assertFalse(ev(o, 205_000))

    def test_inside_body_ignored(self):
        o = new()
        body = [(0.1, 0.0), (0.2, 0.02), (0.25, -0.02)]
        push(o, 100_000, body)
        push(o, 200_000, body)
        self.assertFalse(ev(o, 205_000))

    def test_side_obstacle_ignored(self):
        o = new()
        side = [(0.8, 0.4), (0.8, 0.45), (0.8, 0.5)]
        push(o, 100_000, side)
        push(o, 200_000, side)
        self.assertFalse(ev(o, 205_000))

    def test_age_extends_path(self):
        # 先端から 1.05m 先（d_stop=1.0 の外）。v=2、age 0 なら届かないが、age 200ms 分 (0.4m) 延びて届く
        far = [(1.35, y) for y in (-0.05, 0.0, 0.05)]
        young, old = new(), new()
        for t in (100_000, 200_000):
            push(young, t, far, age_ms=0)
            push(old, t, far, age_ms=200)
        p = params(d_stop_m=1.0, speed_abs_m_s=2.0, confirm_period_s=0.0, comm_delay_s=0.0)
        self.assertFalse(ev(young, 200_000, p=p))
        self.assertTrue(ev(old, 200_000, p=p))

    def test_curved_path(self):
        # 左へ曲がる（κ>0）ときは、真っ直ぐ前の壁は外れ、左前の点は当たる
        straight_wall = [(0.9, y) for y in (-0.05, 0.0, 0.05)]
        o = new()
        push(o, 100_000, straight_wall)
        push(o, 200_000, straight_wall)
        self.assertTrue(ev(o, 205_000, p=params(curvature_1_m=0.0)))
        left = [(0.7, 0.35), (0.7, 0.4), (0.75, 0.38)]
        o2 = new()
        push(o2, 100_000, left)
        push(o2, 200_000, left)
        self.assertFalse(ev(o2, 205_000, p=params(curvature_1_m=0.0)))
        self.assertTrue(ev(o2, 205_000, p=params(curvature_1_m=1.0, d_stop_m=1.5)))

    def test_three_frames_confirm(self):
        p = params(confirm_frames=3)
        o = new()
        push(o, 100_000, WALL)
        push(o, 200_000, WALL)
        self.assertFalse(ev(o, 205_000, p=p), "2 枚では 3 枚確認に足りない")
        push(o, 300_000, WALL)
        self.assertTrue(ev(o, 305_000, p=p))
        push(o, 400_000, [])  # 途中で途切れたら最初から
        self.assertFalse(ev(o, 405_000, p=p))

    def test_confirm_adds_delay_to_path(self):
        # 確認フレームが増えると、その分だけ予想進路が延びる（点が少し遠くても届く）
        far = [(1.55, y) for y in (-0.05, 0.0, 0.05)]
        a, b = new(), new()
        for t in (100_000, 200_000, 300_000):
            push(a, t, far, age_ms=0)
            push(b, t, far, age_ms=0)
        base = dict(d_stop_m=1.0, speed_abs_m_s=2.0, comm_delay_s=0.0)
        self.assertFalse(ev(a, 300_000, p=params(confirm_frames=2, **base)))
        self.assertTrue(ev(b, 300_000, p=params(confirm_frames=3, **base)))

    def test_self_exclusion_zone(self):
        # 車体の外（先端 0.30m の 1cm 前・横 0.10m）の前輪付近の点: 広げた除外で数えない
        wheel = [(0.31, -0.10), (0.32, -0.11), (0.31, -0.12)]
        o = new()
        push(o, 100_000, wheel)
        push(o, 200_000, wheel)
        self.assertTrue(ev(o, 205_000, p=params()))
        self.assertFalse(ev(o, 205_000, p=params(self_ext_front_m=0.10, self_ext_side_m=0.11)))
        # 除外の外側の本物の障害物は数える
        real = [(0.60, y) for y in (-0.05, 0.0, 0.05)]
        o2 = new()
        push(o2, 100_000, real)
        push(o2, 200_000, real)
        self.assertTrue(ev(o2, 205_000, p=params(self_ext_front_m=0.10, self_ext_side_m=0.11)))

    def test_fresh(self):
        o = new()
        self.assertFalse(bool(LIB.StereoObstacles_IsFresh(C.byref(o), 0, 0.3)))
        push(o, 100_000, [])
        self.assertTrue(bool(LIB.StereoObstacles_IsFresh(C.byref(o), 150_000, 0.3)))
        self.assertFalse(bool(LIB.StereoObstacles_IsFresh(C.byref(o), 500_000, 0.3)))

    def test_micros_wrap(self):
        o = new()
        t0 = 0xFFFF_FF00
        push(o, t0, WALL)
        push(o, (t0 + 100_000) & 0xFFFF_FFFF, WALL)
        self.assertTrue(ev(o, (t0 + 105_000) & 0xFFFF_FFFF))


if __name__ == "__main__":
    unittest.main()
