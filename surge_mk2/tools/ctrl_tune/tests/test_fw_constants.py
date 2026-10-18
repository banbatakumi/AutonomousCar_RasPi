"""ファームに手で写してある物理定数・同定値・ファームの定数の写しが、正と食い違っていないか。

ファームは `config/vehicle.toml` を読めないので、車体寸法・同定値の一部を `#define` で持っている。
逆に Pi 側（toml・シムの `SpeedController`）はファームの定数を写している。どちらも片方だけ直すと
黙って食い違う（シムと実機が別の車になる・自動停止の距離が実車と合わない）ので、ここで突き合わせる。
ファームのリポジトリ（`tools/ctrl_tune/fw.py` の `FW_DIR`）が無ければ飛ばす。

    cd surge_mk2 && .venv/bin/python -m pytest tools/ctrl_tune/tests/test_fw_constants.py -q
"""

from __future__ import annotations

import math
import re
import sys
import tomllib
import unittest
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[3]))

from raspi.auto.base import CMD_HZ  # noqa: E402
from raspi.core.vehicle import DEFAULT_PATH  # noqa: E402
from sim.vehicle import SpeedController  # noqa: E402
from tools.ctrl_tune import scenarios as S  # noqa: E402
from tools.ctrl_tune.fw import FW_DIR, OUT, Firmware  # noqa: E402
from tools.ctrl_tune.plant import NOMINAL  # noqa: E402

HEADERS = ("src/control/drive.h", "src/vehicle/vehicle.h", "src/sensing/lidar.h")

_DEFINE = re.compile(r"^\s*#define\s+([A-Z][A-Z0-9_]*)\s+(.+)$")
_NUM_SUFFIX = re.compile(r"(?<=[0-9.])[fFuU]+\b")
_IDENT = re.compile(r"\b[A-Z][A-Z0-9_]*\b")


def fw_defines(fw_dir: Path) -> dict[str, str]:
    """`HEADERS` の `#define NAME 式`（行継続・行末コメントを畳んだもの）。"""
    out: dict[str, str] = {}
    for rel in HEADERS:
        text = (fw_dir / rel).read_text(encoding="utf-8").replace("\\\n", " ")
        for line in text.splitlines():
            m = _DEFINE.match(line.split("//")[0])
            if m:
                out[m.group(1)] = m.group(2).strip()
    return out


def fw_value(defines: dict[str, str], name: str, _depth: int = 0) -> float:
    """マクロを数値へ展開する（四則演算と括弧だけの式）。"""
    if _depth > 10:
        raise ValueError(f"{name}: 展開が深すぎる")
    expr = _NUM_SUFFIX.sub("", defines[name])
    expr = _IDENT.sub(lambda m: repr(fw_value(defines, m.group(0), _depth + 1)), expr)
    if not re.fullmatch(r"[0-9eE.+\-*/() ]+", expr):
        raise ValueError(f"{name}: 数式でない（{defines[name]}）")
    return float(eval(expr, {"__builtins__": {}}))  # noqa: S307  上の正規表現で数式だけに絞ってある


@unittest.skipUnless((FW_DIR / HEADERS[0]).exists(), f"ファームのリポジトリが無い（{FW_DIR}）")
class TestFirmwareConstants(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.d = fw_defines(FW_DIR)
        with open(DEFAULT_PATH, "rb") as fp:
            cls.toml = tomllib.load(fp)

    def fw(self, name: str) -> float:
        return fw_value(self.d, name)

    def assertSame(self, fw_name: str, value: float, tol: float = 1e-6):
        self.assertLessEqual(abs(self.fw(fw_name) - float(value)), tol,
                             f"ファームの {fw_name} = {self.fw(fw_name):.6g} と正の値 {float(value):.6g} が食い違う")

    def test_geometry_matches_vehicle_toml(self):
        t = self.toml
        self.assertSame("DRIVE_WHEELBASE_M", t["wheelbase"])
        self.assertSame("DRIVE_REAR_TRACK_M", t["track"])
        self.assertSame("DRIVE_FRONT_WHEEL_RADIUS_M", t["wheel_radius"])
        self.assertSame("DRIVE_REAR_WHEEL_RADIUS_M", t["wheel_radius"])
        xs = [p[0] for p in t["footprint"]]
        self.assertSame("VEHICLE_BODY_FRONT_X_M", max(xs))
        self.assertSame("VEHICLE_BODY_REAR_X_M", min(xs))
        self.assertSame("VEHICLE_BODY_HALF_WIDTH_M", max(abs(p[1]) for p in t["footprint"]))
        s = t["sensors"]
        self.assertSame("VEHICLE_LIDAR_X_M", s["lidar"]["x"])
        self.assertEqual(float(s["lidar"]["y"]), 0.0, "自動停止の矩形は LiDAR の y オフセットを 0 と仮定している")
        self.assertSame("VEHICLE_ULTRASONIC_FRONT_X_M", s["us_front"]["x"])
        self.assertSame("VEHICLE_ULTRASONIC_REAR_X_M", s["us_rear"]["x"])

    def test_auto_stop_offsets_follow_geometry(self):
        self.assertAlmostEqual(self.fw("VEHICLE_AUTO_STOP_LIDAR_FRONT_OFFSET_CM"), 23.0, places=4)
        self.assertAlmostEqual(self.fw("VEHICLE_AUTO_STOP_LIDAR_REAR_OFFSET_CM"), 14.0, places=4)
        self.assertAlmostEqual(self.fw("VEHICLE_AUTO_STOP_ULTRASONIC_FRONT_OFFSET_CM"), 4.0, places=4)
        self.assertAlmostEqual(self.fw("VEHICLE_AUTO_STOP_ULTRASONIC_REAR_OFFSET_CM"), 3.0, places=4)
        self.assertAlmostEqual(self.fw("VEHICLE_AUTO_STOP_LIDAR_HALF_WIDTH_CM"), 15.0, places=4)

    def test_identified_values_match_vehicle_toml(self):
        """同定し直したら toml とファームの両方を直す（丸めの範囲で一致していること）。"""
        self.assertSame("VEHICLE_BRAKE_DECEL_M_S2", self.toml["dynamics"]["brake_decel_m_s2"], tol=0.01)
        self.assertSame("VEHICLE_MD_TORQUE_TAU_S", self.toml["control"]["plant"]["md_tau_s"], tol=1e-4)
        # 自動停止は同定した制動の減速度で止まり切る前提（以前は楽観的な 3.0 の手置きだった）
        self.assertSame("VEHICLE_AUTO_STOP_DECEL_M_S2", self.fw("VEHICLE_BRAKE_DECEL_M_S2"))

    def test_firmware_constants_copied_to_the_pi(self):
        dyn = self.toml["dynamics"]
        self.assertSame("DRIVE_LPF_TAU_FRONT_S", dyn["speed_filter_s"])
        self.assertSame("DRIVE_SPEED_FF_TAU_S", dyn["speed_ff_filter_s"])
        self.assertSame("DRIVE_HOST_COMMAND_PERIOD_S", 1.0 / CMD_HZ)
        self.assertSame("DRIVE_MAX_TORQUE_NM", dyn["speed_torque_max_nm"] / dyn["drive_ratio"])

    def test_sim_speed_controller_mirrors_drive_h(self):
        for attr, name in (("STANDSTILL_M_S", "DRIVE_STANDSTILL_SPEED_M_S"),
                           ("ANTIWINDUP_TT_S", "DRIVE_ANTIWINDUP_TT_S"),
                           ("LAUNCH_REVERSE_M_S", "DRIVE_LAUNCH_REVERSE_SPEED_M_S"),
                           ("LAUNCH_MIN_STEP_M_S", "DRIVE_LAUNCH_MIN_STEP_M_S"),
                           ("LAUNCH_END_MARGIN_M_S", "DRIVE_LAUNCH_END_MARGIN_M_S"),
                           ("LAUNCH_TIMEOUT_S", "DRIVE_LAUNCH_TIMEOUT_S")):
            with self.subTest(attr=attr):
                self.assertSame(name, getattr(SpeedController, attr))

    def test_derived_timeouts_are_physical(self):
        """DISARM 後の制動の上限は、最高速から同定した減速度で止まり切れる時間より長い。"""
        stop_s = self.fw("DRIVE_MAX_SPEED_M_S") / self.fw("VEHICLE_BRAKE_DECEL_M_S2")
        self.assertGreater(self.fw("VEHICLE_DISARM_BRAKE_TIMEOUT_S"), stop_s)
        self.assertTrue(math.isclose(self.fw("VEHICLE_AUTO_STOP_DELAY_S"),
                                     1.0 / self.fw("LIDAR_SCAN_FREQ_HZ") + self.fw("VEHICLE_MD_TORQUE_TAU_S")))


def kf_lag_s(out: np.ndarray, dt: float = 5e-4) -> float:
    """加減速している間の「真の車速 − ファームの車速」÷ 加速度 [s]（1次遅れなら時定数に等しい）。"""
    v, k = out[:, OUT["speed"]], out[:, OUT["fw_speed"]]
    a = np.gradient(v, dt)
    m = np.abs(a) > 0.5
    return float(np.mean((v - k)[m] * np.sign(a[m])) / np.mean(np.abs(a[m])))


@unittest.skipUnless((FW_DIR / "src" / "control" / "speed_kf.c").exists(), "ファームに speed_kf が無い")
class TestSpeedKfLag(unittest.TestCase):
    """`[dynamics] speed_kf_lag_s`（シムの車速PIの帰還・`speed` の遅れ）を、コンパイルしたファームで測る。"""

    @classmethod
    def setUpClass(cls):
        cls.fw = Firmware()
        with open(DEFAULT_PATH, "rb") as fp:
            cls.lag = float(tomllib.load(fp)["dynamics"]["speed_kf_lag_s"])

    def test_measured_lag_matches_vehicle_toml(self):
        for key in ("launch_pi", "brake", "decel_pi"):
            sc = next(s for s in S.ALL if s.key == key)
            measured = kf_lag_s(S.run(self.fw, NOMINAL, sc).out)
            with self.subTest(scenario=key):
                self.assertLess(abs(measured - self.lag), 1e-3,
                                f"ファームの車速の遅れ {measured * 1e3:.2f}ms と speed_kf_lag_s {self.lag * 1e3:.2f}ms")


if __name__ == "__main__":
    unittest.main()
