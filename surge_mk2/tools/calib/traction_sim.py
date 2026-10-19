"""TC・ABS の効きをシムで測る（ホストでコンパイルしたファーム＋車のモデル）。設計の確かめと比較に使う。

- `launch()`: 止まった状態から全開（トルク直接指令 0.13N·m、TC あり）で `duration` 秒。進んだ距離が長いほど良い
- `brake()`: `v0` から最大の制動（0.13N·m、ABS あり）で止まるまで。止まるまでの距離が短いほど良い

どちらもスリップ率のばらつき（STM32 の測り方）と、トルクの暴れ（10ms ごとのトルクの変化の RMS）も返す。
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from tools.fwsim.fw import IN, MODE_BRAKE, MODE_SPEED, MODE_TORQUE, OUT, HostConfig

from .simcar import DT, SUBSTEPS, Car, firmware

__all__ = ["Outcome", "launch", "brake"]


@dataclass
class Outcome:
    distance: float      #: 発進: 進んだ距離 [m]／制動: 止まるまでの距離 [m]
    slip_mean: float
    slip_std: float
    chatter: float       #: トルクの暴れ [N·m]
    v_end: float


def _run(car: Car, params: dict[str, float], u: np.ndarray, initial_speed: float) -> np.ndarray:
    fw = firmware()
    cfg = HostConfig(dt_s=DT, substeps=SUBSTEPS, tc_enabled=1, abs_enabled=1, wheel_lift_guard_enabled=1, tv_enabled=0,
                     imu_ready=1, initial_speed_m_s=initial_speed)
    p = {k: v for k, v in params.items() if k in fw.params}
    return fw.run(car.to_c(), cfg, u, p)


def _inputs(n: int) -> np.ndarray:
    u = np.zeros((n, len(IN)))
    u[:, IN["mu_left"]] = u[:, IN["mu_right"]] = 1.0
    u[:, IN["front_scale"]] = 1.0
    u[:, IN["test_moment"]] = np.nan
    return u


def _stats(out: np.ndarray, mask: np.ndarray, side_select_low: bool) -> tuple[float, float, float]:
    sl = out[:, OUT["fw_slip_left"]]
    sr = out[:, OUT["fw_slip_right"]]
    lam = np.minimum(sl, sr) if side_select_low else 0.5 * (sl + sr)
    tq = out[:, OUT["cmd_left"]] + out[:, OUT["cmd_right"]]
    tq10 = tq[::20]
    m10 = mask[::20]
    d = np.diff(tq10)[m10[1:] & m10[:-1]]
    chatter = float(np.sqrt(np.mean(d ** 2))) if len(d) else 0.0
    sel = lam[mask]
    return (float(np.mean(sel)) if len(sel) else 0.0, float(np.std(sel)) if len(sel) else 0.0, chatter)


def launch(car: Car, params: dict[str, float], duration: float = 1.0) -> Outcome:
    n = int(duration / DT)
    u = _inputs(n)
    u[:, IN["mode"]] = MODE_TORQUE
    u[:, IN["value"]] = 0.13
    out = _run(car, params, u, 0.0)
    v = out[:, OUT["speed"]]
    mask = v > 0.6
    m, s, ch = _stats(out, mask, False)
    return Outcome(float(out[-1, OUT["distance"]]), m, s, ch, float(v[-1]))


def brake(car: Car, params: dict[str, float], v0: float = 2.0) -> Outcome:
    n = int(2.5 / DT)
    u = _inputs(n)
    u[:200, IN["mode"]] = MODE_SPEED
    u[:200, IN["value"]] = v0
    u[200:, IN["mode"]] = MODE_BRAKE
    u[200:, IN["value"]] = 0.13
    out = _run(car, params, u, v0)
    v = out[:, OUT["speed"]]
    i0 = 200
    stop = np.flatnonzero(v[i0:] < 0.05)
    i1 = i0 + (int(stop[0]) if len(stop) else len(v) - i0 - 1)
    mask = np.zeros(len(v), dtype=bool)
    mask[i0:i1] = v[i0:i1] > 0.5
    m, s, ch = _stats(out, mask, True)
    dist = float(out[i1, OUT["distance"]] - out[i0, OUT["distance"]])
    return Outcome(dist, -m, s, ch, float(v[i1]))
