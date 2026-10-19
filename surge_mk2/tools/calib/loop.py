"""PI のゲインを位相余裕から決める（車速の PI・TC・ABS で共通）。

植物 P(s) に PI C(s) = kp·(1 + ωz/s) を掛けた一巡 L = C·P について、

1. PI の零点を交差の 1/`ratio` に置く（ωz = ωc/ratio。積分は交差より十分下で効かせ、交差の位相をあまり削らない）
2. 位相余裕 180° + ∠L(jωc) が `pm_deg` になる ωc を探す（上限 `wc_max`）
3. |L(jωc)| = 1 から kp、ki = kp·ωz

植物（`Plant`）は振幅と位相を要素ごとの式の和で返す（数値の偏角は ±180° で折り返すので使わない）:
- 車速: P = G·e^(−s·Td)/(s·(1 + s·τ))（トルク → 車速。積分器）
- TC・ABS: P = (r/J)·e^(−s·d)/((s + a)·(1 + s·τ))（トルク → スリップ速度。a = r²·k/(J·v)、k はタイヤの曲線の傾き）
"""

from __future__ import annotations

import math
from dataclasses import dataclass

__all__ = ["Design", "Plant", "design", "evaluate"]


@dataclass(frozen=True)
class Plant:
    """gain·e^(−s·delay)/((s + pole)·(1 + s·tau))。pole = 0 なら積分器。"""

    gain: float
    pole: float
    delay: float
    tau: float

    def mag(self, w: float) -> float:
        return self.gain / (math.hypot(w, self.pole) * math.hypot(1.0, w * self.tau))

    def phase_deg(self, w: float) -> float:
        return -math.degrees(math.atan2(w, self.pole) + w * self.delay + math.atan(w * self.tau))


@dataclass
class Design:
    kp: float
    ki: float
    wc: float
    pm_deg: float
    limited_by: str


def _pi_phase_deg(w: float, wz: float) -> float:
    """PI kp·(1 + ωz/(jω)) の位相（−90°〜0°）。"""
    return -math.degrees(math.atan2(wz, w))


def design(P: Plant, pm_deg: float, ratio: float = 4.0, wc_max: float = float("inf")) -> Design:
    def pm(w):
        return 180.0 + P.phase_deg(w) + _pi_phase_deg(w, w / ratio)

    lo, hi = 1e-3, 1e5
    if pm(lo) < pm_deg:
        raise ValueError("どの交差でも位相余裕が足りない")
    for _ in range(200):
        mid = math.sqrt(lo * hi)
        if pm(mid) > pm_deg:
            lo = mid
        else:
            hi = mid
    wc, limited = lo, "位相余裕"
    if wc > wc_max:
        wc, limited = wc_max, "交差の上限"
    wz = wc / ratio
    kp = 1.0 / (P.mag(wc) * math.hypot(1.0, wz / wc))
    return Design(kp=kp, ki=kp * wz, wc=wc, pm_deg=pm(wc), limited_by=limited)


def evaluate(P: Plant, kp: float, ki: float) -> tuple[float, float]:
    """与えたゲインの (位相余裕 [°], 交差 [rad/s])。交差が無ければ (nan, nan)。"""
    import numpy as np

    ws = np.logspace(-2, 5, 4000)
    mags = [kp * math.hypot(1.0, (ki / kp) / w) * P.mag(w) if kp > 0 else (ki / w) * P.mag(w) for w in ws]
    for w, m0, m1 in zip(ws[1:], mags[:-1], mags[1:]):
        if m0 >= 1.0 > m1:
            wz = ki / kp if kp > 0 else float("inf")
            ph = P.phase_deg(w) + (-math.degrees(math.atan2(wz, w)) if kp > 0 else -90.0)
            return 180.0 + ph, float(w)
    return float("nan"), float("nan")
