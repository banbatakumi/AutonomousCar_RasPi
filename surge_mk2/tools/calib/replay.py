"""記録した指令を、ホストでコンパイルしたファーム＋車のモデルにそのまま入れ直す（開ループの再生）。

確かめの試験（車速・TC/ABS）の「予測」に使う: 実機と同じ指令・同じ調整値で、同定したモデルならどう走るはずか。
planner は通さない（記録の `/cmd` が入力）。指令が STM32 に効くまでの遅れ `latency` は記録からは分からないので、
呼び出し側が探す（`best_latency`）。
"""

from __future__ import annotations

import tomllib
from dataclasses import replace
from pathlib import Path

import numpy as np

from raspi.core.control_params import DEFAULT_TOML
from tools.fwsim.fw import IN, MODE_BRAKE, MODE_SPEED, MODE_TORQUE, OUT, HostConfig

from .log import Log
from .simcar import DT, SUBSTEPS, Car, firmware

__all__ = ["replay", "car_from_toml", "best_latency"]

_ENABLE_KEYS = ("tc_enable", "abs_enable", "wheel_lift_guard_enable", "tv_enable")


def car_from_toml(path: str | Path = DEFAULT_TOML) -> Car:
    """`config/vehicle.toml` の同定値から、予測に使う車を作る。

    車速の同定（`[dynamics] speed_plant_gain`・`rolling_resistance`）は、車輪の回転の慣性・摩擦を含んだ実効の値。
    モデルの車輪の慣性 J（`[control.plant]`）ぶんを質量から引き、摩擦は転がり抵抗に含めて 0 にする。
    MD の遅れ・タイヤは `[control.plant]`（校正 6・7）。"""
    with open(path, "rb") as f:
        d = tomllib.load(f)
    dyn = d.get("dynamics", {})
    plant = d.get("control", {}).get("plant", {})
    car = Car()
    r = 0.030
    J = float(plant.get("wheel_inertia_kgm2", car.wheel_inertia_kgm2))
    G = float(dyn.get("speed_plant_gain", 0.0)) or 1.0 / (r * float(d.get("mass", 2.0)))
    over = dict(mass_kg=max(0.5, 1.0 / (r * G) - 2 * J / r ** 2), wheel_inertia_kgm2=J, wheel_friction_nm=0.0,
                rolling_decel_m_s2=float(dyn.get("rolling_resistance", car.rolling_decel_m_s2)))
    for k in ("md_delay_s", "md_tau_s", "mu", "tyre_b", "tyre_c", "load_transfer"):
        if k in plant:
            over[k] = float(plant[k])
    return replace(car, **over)


def _inputs(log: Log, latency: float, n: int, t0: float) -> tuple[np.ndarray, list[tuple[int, dict]]]:
    """2kHz の入力の行列と、上書き（fw_overrides）が変わる周期の番号。"""
    u = np.zeros((n, len(IN)))
    u[:, IN["mu_left"]] = u[:, IN["mu_right"]] = 1.0
    u[:, IN["front_scale"]] = 1.0
    u[:, IN["test_moment"]] = np.nan
    u[:, IN["mode"]] = MODE_BRAKE
    u[:, IN["value"]] = 0.13
    tf = t0 + np.arange(n) * DT
    k = np.searchsorted(log.cmd_t + latency, tf, side="right") - 1
    valid = k >= 0
    kk = np.clip(k, 0, None)
    brake = log.cmd["brake"][kk] > 0.5
    torque = log.cmd["torque_mode"][kk] > 0.5
    mode = np.where(brake, MODE_BRAKE, np.where(torque, MODE_TORQUE, MODE_SPEED))
    bt = log.cmd["brake_torque"][kk]
    value = np.where(brake, np.where(bt > 0, bt, 0.13), np.where(torque, log.cmd["target_torque"][kk],
                                                                 log.cmd["target_speed"][kk]))
    u[valid, IN["mode"]] = mode[valid]
    u[valid, IN["value"]] = value[valid]
    u[valid, IN["accel_limit"]] = log.cmd["accel_limit"][kk][valid]
    u[valid, IN["steer"]] = log.cmd["target_steer"][kk][valid]
    changes = []
    prev = None
    for i in np.flatnonzero(np.diff(np.concatenate([[-1], kk])) != 0):
        ov = log.cmd_overrides[kk[i]] if valid[i] else {}
        if ov != prev:
            changes.append((int(i), ov))
            prev = ov
    return u, changes


def replay(log: Log, car: Car, params: dict[str, float] | None = None, latency: float = 0.01) -> dict[str, np.ndarray]:
    """記録の指令でファーム＋`car` を回し、TELEMETRY の時刻での予測（`speed`・`distance`・`torque`・`slip` など）を返す。

    `params` は `[control]` の調整値（既定は記録したときに STM32 に入っていた値 `log.control_params`）。"""
    fw = firmware()
    p = dict(log.control_params if params is None else params)
    p = {k: v for k, v in p.items() if k in fw.params}
    t0 = float(log.t[0]) - 0.01
    n = int((log.t[-1] - t0) / DT) + 2
    u, changes = _inputs(log, latency, n, t0)
    cfg = HostConfig(dt_s=DT, substeps=SUBSTEPS, tc_enabled=int(p.get("tc_enable", 1)), abs_enabled=int(p.get("abs_enable", 1)),
                     wheel_lift_guard_enabled=int(p.get("wheel_lift_guard_enable", 1)), tv_enabled=0, imu_ready=1,
                     initial_speed_m_s=0.0)
    sess = fw.session(car.to_c(), cfg, p)
    outs = []
    pos = 0
    ci = 0
    while pos < n:
        while ci < len(changes) and changes[ci][0] <= pos:
            ov = changes[ci][1]
            sess.set_enables(tc=ov.get("tc_enable", p.get("tc_enable", 1)) != 0,
                             abs_=ov.get("abs_enable", p.get("abs_enable", 1)) != 0,
                             lift=ov.get("wheel_lift_guard_enable", p.get("wheel_lift_guard_enable", 1)) != 0, tv=False)
            ci += 1
        nxt = changes[ci][0] if ci < len(changes) else n
        nxt = max(nxt, pos + 1)
        outs.append(sess.step(u[pos:nxt]))
        pos = nxt
    sess.close()
    out = np.vstack(outs)[:n]
    tf = t0 + np.arange(1, n + 1) * DT
    pick = np.clip(np.searchsorted(tf, log.t), 0, n - 1)
    o = out[pick]
    return {"speed": o[:, OUT["speed"]], "fw_speed": o[:, OUT["fw_speed"]], "distance": o[:, OUT["distance"]],
            "torque": o[:, OUT["cmd_left"]] + o[:, OUT["cmd_right"]],
            "slip_left": o[:, OUT["fw_slip_left"]], "slip_right": o[:, OUT["fw_slip_right"]],
            "accel": o[:, OUT["accel"]], "flags": o[:, OUT["flags"]]}


def best_latency(log: Log, car: Car, measured_speed: np.ndarray, mask: np.ndarray,
                 grid=(0.005, 0.01, 0.015, 0.02, 0.025, 0.03)) -> tuple[float, dict[str, np.ndarray]]:
    """指令が効くまでの遅れを、予測と実測の車速の差が最小になるように格子で探す。"""
    best = None
    for lat in grid:
        pr = replay(log, car, latency=lat)
        e = float(np.sqrt(np.mean((pr["speed"][mask] - measured_speed[mask]) ** 2)))
        if best is None or e < best[0]:
            best = (e, lat, pr)
    return best[1], best[2]
