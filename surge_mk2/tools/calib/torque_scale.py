"""MD のトルク定数の誤差 k（実際のトルク ÷ トルク指令）を、校正 5・6 と体重計の質量から式で出す。

校正 5 の利得 G と校正 6 の後輪の慣性 J は、どちらも**トルク指令の単位**で測っている（J_meas = J_true/k）。車体の運動は

    1/G = r·m/k + 2·J_meas/r        → k = r·m / (1/G − 2·J_meas/r)

（m は体重計の質量、r は後輪の半径）。k は**書き込まない**（G・J・m から決まる式の値。どれかを校正し直せば変わる）。

使う所: 制御はすべて指令の単位で閉じている（車速の FF は 1/G、TC・ABS のゲインは J_meas）ので k を要らない。物理の単位と
混ぜる所だけが使う:

- 校正 7（`tyre.py`）: タイヤの前後力を実際の力（体重計の質量 × 加速度、車輪の運動方程式 × k）で出す → μ が実際の値になる
- fwsim の車（`tools/fwsim/plant.py`）: 車体側の質量と荷重を 1/k 倍する（トルクを k 倍した車と同じ運動。ファームの
  `host_sim.c` を変えずに済む）

k には MD のトルクの誤差のほか、前輪など回転部の慣性・G の同定誤差も入る（それらを含めた「指令あたりの実効の倍率」）。
"""

from __future__ import annotations

import math

from . import toml_io

__all__ = ["WHEEL_RADIUS_M", "K_RANGE", "torque_scale", "torque_scale_from_toml"]

WHEEL_RADIUS_M = 0.030
#: ありそうな範囲。外れたら G・J・質量のどれかを疑う
K_RANGE = (0.6, 1.15)


def torque_scale(G: float, wheel_inertia_kgm2: float, mass_kg: float, r: float = WHEEL_RADIUS_M) -> float:
    """k = 実際のトルク ÷ トルク指令。決まらない（分母が 0 以下・値が無い）ときは nan。"""
    den = 1.0 / G - 2.0 * wheel_inertia_kgm2 / r if G > 0 else float("nan")
    if not math.isfinite(den) or den <= 0 or mass_kg <= 0:
        return float("nan")
    return r * mass_kg / den


def torque_scale_from_toml(path=toml_io.DEFAULT_TOML, *, G: float | None = None,
                           wheel_inertia_kgm2: float | None = None) -> float:
    """`vehicle.toml` の G（`dynamics.speed_plant_gain`）・J（`control.plant.wheel_inertia_kgm2`）・質量から k。
    G・J を渡せばその値を使う（解析したばかりの値で見積もる）。"""
    f = toml_io.read_flat(path)
    G = f.get("dynamics.speed_plant_gain", float("nan")) if G is None else G
    J = f.get("control.plant.wheel_inertia_kgm2", float("nan")) if wheel_inertia_kgm2 is None else wheel_inertia_kgm2
    return torque_scale(G, J, f.get("mass", float("nan")))
