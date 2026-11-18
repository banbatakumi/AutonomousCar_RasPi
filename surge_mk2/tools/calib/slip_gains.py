"""TC・ABS の目標スリップ率とゲインの決め方（2026-11-18〜）。設計は `docs/calibration.md` §4.8。

## 目標スリップ率は測らず固定する

前後の力はタイヤの肩より先でほぼ平ら（fwsim で目標 0.12〜0.40 の発進の効率は 0.945〜0.950）で、目標を上げるほど
横グリップが減り、立ち上がりでリアが出やすくなる。目標は「タイヤの山」ではなく**前後と横への配分の方針**なので、
床・タイヤごとに測らず `SLIP_TARGET` に固定する。実機の駆動の肩（右輪 0.13〜0.25）に近く、制動の左右差でも崩れない値。
校正 7 は肩を測って、この値と大きく食い違わないかを確かめるだけ。

## ゲインは車体の値（後輪の慣性 J）から式で決める

スリップ速度はトルクに対してほぼ積分器 (r/J)/s なので、交差周波数 ωc を決めれば

    kp = ωc·J/r,   ki = kp·ωc/ratio       （PI の零点を交差の 1/ratio に置く）

ωc と ratio は、旧校正 8（交差周波数を実機で振って発振の手前を採る試験。2026-11-18 に廃止）が実機で採った値に固定する
（そのときの J = 3.581e-5 で今の toml のゲインと一致する）。J は校正 6（後輪を浮かせる。床に依らない）が書くので、
校正 6 を適用すればゲインも決まる。
"""

from __future__ import annotations

__all__ = ["SLIP_TARGET", "WHEEL_RADIUS_M", "WC_RAD_S", "ZERO_RATIO", "KEYS", "base_gains", "gains_for_inertia"]

#: TC・ABS の目標スリップ率（STM32 の測るスリップ率のまま）
SLIP_TARGET = 0.15
WHEEL_RADIUS_M = 0.030
#: 交差周波数 [rad/s]（旧校正 8 の実機の結果）
WC_RAD_S = {"tc": 120.0, "abs": 60.0}
#: PI の零点 = ωc / ratio（同）
ZERO_RATIO = {"tc": 8.0, "abs": 4.0}
#: 設定の名前（TC・ABS）
KEYS = {"tc": ("tc_kp_nm_per_m_s", "tc_ki_nm_per_m"), "abs": ("abs_kp_nm_per_m_s", "abs_ki_nm_per_m")}


def base_gains(wc: float, ratio: float, inertia_kgm2: float) -> tuple[float, float]:
    """交差周波数 `wc` [rad/s]・PI の零点の比 `ratio`（零点 = wc/ratio）から (kp, ki)。スリップ速度が
    トルクに対して積分器 (r/J)/s とみなせるとして、|L(jωc)| = 1 から。"""
    kp = wc * inertia_kgm2 / WHEEL_RADIUS_M
    return kp, kp * wc / ratio


def gains_for_inertia(inertia_kgm2: float) -> dict[str, float]:
    """後輪の慣性 J から `[control]` の TC・ABS のゲイン（キー → 値）。"""
    out: dict[str, float] = {}
    for kind, (kp_key, ki_key) in KEYS.items():
        kp, ki = base_gains(WC_RAD_S[kind], ZERO_RATIO[kind], inertia_kgm2)
        out[kp_key] = float(f"{kp:.5g}")
        out[ki_key] = float(f"{ki:.5g}")
    return out
