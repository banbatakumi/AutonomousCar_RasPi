"""速度指令の前倒し（`raspi/nav/speedctl.py`）を、ファームの速度制御の写し
（`sim.vehicle.SpeedController`）に通して確かめる。

直線と一定速のコーナーが並ぶ 1 次元の周回を、指令 50Hz・遅れ 25ms で走らせる。
**経路の速度をそのまま送る**（旧方式、先読み 0.1s）とコーナーへ 0.4m/s 以上速く入るのに対し、
前倒しすると出過ぎと追従の誤差が小さくなること、同定値がずれた車でも壊れないことを見る。
"""

import math
import sys
import unittest
from dataclasses import replace
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

import numpy as np  # noqa: E402

from raspi.core.vehicle import Vehicle  # noqa: E402
from raspi.nav import speedctl  # noqa: E402
from raspi.nav.raceline import RaceLine  # noqa: E402
from sim.vehicle import DriveInput, SpeedController, VehicleSpec  # noqa: E402

DS = 0.1
SEGS = ((4.0, 3.0), (1.2, 1.2), (2.5, 3.0), (1.0, 0.9), (6.0, 3.0), (1.5, 1.6), (1.5, 3.0),
        (1.0, 1.1))


def make_track(veh: Vehicle, use: float = 0.95) -> tuple[np.ndarray, RaceLine]:
    lim = np.concatenate([np.full(int(round(length / DS)), v) for length, v in SEGS])
    n = len(lim)
    v = lim.copy()
    aa, ab = use * veh.a_accel_max(), use * veh.a_decel_max
    for _ in range(2):
        for i in range(n - 1, -1, -1):
            v[i] = min(v[i], math.sqrt(v[(i + 1) % n] ** 2 + 2 * ab * DS))
        for i in range(n):
            v[i] = min(v[i], math.sqrt(v[(i - 1) % n] ** 2 + 2 * aa * DS))
    # 形は直線でよい（速度指令は弧長しか見ない）。閉路として扱うので一周の長さだけ合わせる
    xy = np.column_stack([np.arange(n) * DS, np.zeros(n)])
    return lim, RaceLine(xy=xy, v=v, kappa=np.zeros(n), length=n * DS)


def run(spec: VehicleSpec, veh: Vehicle, *, feedforward: bool, laps: int = 3):
    """戻り値 `(コーナーの上限からの出過ぎの最大 [m/s], 速度の誤差の RMS [m/s])`（2周目以降）。"""
    lim, path = make_track(veh)
    n = len(path)
    length = n * DS
    ctl = SpeedController(spec)
    v = float(path.v[0])
    ctl.reset(v=v, ref=v)
    ff = speedctl.SpeedFF()
    cfg = speedctl.config_from_vehicle(veh)
    s = t = 0.0
    dt, period, latency = 0.002, 0.02, 0.025
    next_cmd, target, queue = 0.0, v, []
    over, err2, cnt = 0.0, 0.0, 0

    def at(arr, pos):
        x = (pos / DS) % n
        i = int(x)
        return arr[i] * (1 - (x - i)) + arr[(i + 1) % n] * (x - i)

    while s < laps * length:
        if t >= next_cmd:
            next_cmd += period
            vm = ctl.v_meas
            if feedforward:
                tgt = ff.command(path, s / DS, vm, period, cfg)
            else:
                tgt = at(path.v, s + vm * 0.1)
            queue.append((t + latency, tgt))
        while queue and queue[0][0] <= t:
            target = queue.pop(0)[1]
        v = ctl.step(DriveInput(armed=True, target_speed=target, accel_limit=6.0), v, dt)
        s += v * dt
        t += dt
        if s >= length:
            over = max(over, v - at(lim, s))
            err2 += (v - at(path.v, s)) ** 2
            cnt += 1
    return over, math.sqrt(err2 / cnt)


def run_firmware_ff(spec: VehicleSpec, veh: Vehicle, *, gain_error: float = 1.0, laps: int = 3):
    """STM32 の加速度フィードフォワード（★v0.21）を入れた車を `speedctl.ref_command` で走らせる。

    `gain_error` は STM32 に入れる値の外し方（1.2 = 車体の利得を 20% 小さく見積もった）。
    """
    lim, path = make_track(veh)
    n = len(path)
    length = n * DS
    ctl = SpeedController(spec)
    ctl.speed_ff_nm_per_m_s2 = gain_error / spec.speed_plant_gain
    ctl.speed_ff_rr_nm = gain_error * spec.rolling_resistance / spec.speed_plant_gain
    v = float(path.v[0])
    ctl.reset(v=v, ref=v)
    acc = speedctl.path_accel(path)
    s = t = 0.0
    dt, period, latency = 0.002, 0.02, 0.025
    lead = 0.5 * period + latency + speedctl.FW_FF_TAU_S
    next_cmd, target, ramp, queue = 0.0, v, 3.0, []
    over, err2, cnt = 0.0, 0.0, 0

    def at(arr, pos):
        x = (pos / DS) % n
        i = int(x)
        return arr[i] * (1 - (x - i)) + arr[(i + 1) % n] * (x - i)

    while s < laps * length:
        if t >= next_cmd:
            next_cmd += period
            queue.append((t + latency,) + speedctl.ref_command(path, s / DS, ctl.v_meas,
                                                               lead_s=lead, acc=acc))
        while queue and queue[0][0] <= t:
            _, target, ramp = queue.pop(0)
        v = ctl.step(DriveInput(armed=True, target_speed=target, accel_limit=min(6.0, ramp)), v, dt)
        s += v * dt
        t += dt
        if s >= length:
            over = max(over, v - at(lim, s))
            err2 += (v - at(path.v, s)) ** 2
            cnt += 1
    return over, math.sqrt(err2 / cnt)


class TestFirmwareFeedForward(unittest.TestCase):
    """STM32 の車速PIに加速度フィードフォワードを足した場合（`SpeedController.speed_ff_*`）。"""

    @classmethod
    def setUpClass(cls):
        cls.spec = VehicleSpec.load()
        cls.veh = Vehicle.load()

    def test_tracks_better_than_the_pi_side_lead(self):
        over_pi, rms_pi = run(self.spec, self.veh, feedforward=True)
        over_fw, rms_fw = run_firmware_ff(self.spec, self.veh)
        self.assertLess(over_fw, 0.12)
        self.assertLess(rms_fw, 0.04)
        self.assertLess(rms_fw, rms_pi)

    def test_robust_to_a_wrong_gain(self):
        for err in (0.8, 1.2):
            over, rms = run_firmware_ff(self.spec, self.veh, gain_error=err)
            self.assertLess(over, 0.15, err)
            self.assertLess(rms, 0.06, err)

    def test_zero_gain_is_the_old_controller(self):
        """値が 0（既定）なら、足す前と 1 ステップも違わない。"""
        a, b = SpeedController(self.spec), SpeedController(self.spec)
        b.speed_ff_nm_per_m_s2 = b.speed_ff_rr_nm = 0.0
        va = vb = 0.0
        for k in range(2000):
            cmd = DriveInput(armed=True, target_speed=2.0 if k < 1000 else 0.5, accel_limit=2.0)
            va, vb = a.step(cmd, va, 0.001), b.step(cmd, vb, 0.001)
        self.assertEqual(va, vb)

    def test_feedforward_removes_the_ramp_lag(self):
        """一定の傾きで目標を上げる間、車速は目標にほぼ遅れない（無しだと 0.1m/s 前後遅れたまま）。"""
        lag = {}
        for name, on in (("off", False), ("on", True)):
            ctl = SpeedController(self.spec)
            if on:
                ctl.speed_ff_nm_per_m_s2 = 1.0 / self.spec.speed_plant_gain
                ctl.speed_ff_rr_nm = self.spec.rolling_resistance / self.spec.speed_plant_gain
            v = 0.5
            ctl.reset(v=v, ref=v)
            for _ in range(700):                       # 0.7 秒、目標 3.0 へ 2.0m/s² で
                v = ctl.step(DriveInput(armed=True, target_speed=3.0, accel_limit=2.0), v, 0.001)
            lag[name] = ctl.ref - v
        self.assertGreater(lag["off"], 0.08)           # 積分が効き始めても 0.1m/s 前後残る
        self.assertLess(abs(lag["on"]), 0.03)
        self.assertLess(abs(lag["on"]), 0.3 * lag["off"])


class TestSpeedFeedForward(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.spec = VehicleSpec.load()
        cls.veh = Vehicle.load()

    def test_feedforward_enters_corners_on_speed(self):
        over_old, rms_old = run(self.spec, self.veh, feedforward=False)
        over_new, rms_new = run(self.spec, self.veh, feedforward=True)
        self.assertGreater(over_old, 0.35)          # 旧方式は速すぎる速度で入る
        self.assertLess(over_new, 0.2)
        self.assertLess(over_new, 0.5 * over_old)
        self.assertLess(rms_new, 0.08)
        self.assertLess(rms_new, 0.6 * rms_old)

    def test_robust_to_a_different_pi_gain(self):
        """ファームの kp/ki が toml と違っていても（0.25/0.5 → 0.3/0.6）壊れない。"""
        spec = replace(self.spec, speed_kp=self.spec.speed_kp * 1.2, speed_ki=self.spec.speed_ki * 1.2)
        over, rms = run(spec, self.veh, feedforward=True)
        self.assertLess(over, 0.25)
        self.assertLess(rms, 0.1)

    def test_robust_to_a_weaker_car(self):
        """加減速が同定値の 85% しか出ない車でも、旧方式より出過ぎない。"""
        spec = replace(self.spec, drive_accel_m_s2=self.spec.drive_accel_m_s2 * 0.85,
                       speed_decel_m_s2=self.spec.speed_decel_m_s2 * 0.85)
        over_old, _ = run(spec, self.veh, feedforward=False)
        over_new, _ = run(spec, self.veh, feedforward=True)
        self.assertLess(over_new, 0.3)
        self.assertLess(over_new, over_old)


if __name__ == "__main__":
    unittest.main()
