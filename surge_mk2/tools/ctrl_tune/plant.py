"""後輪＋タイヤ＋車体のモデルのパラメータと、それを揺らした「あり得る車」の組。

モデルの式は ファーム側 `host/host_sim.c`（C。速さのため車両モデルもそちらに置く）。ここは値だけ。

## 値の出どころ

`NOMINAL` は実機の同定（`tools/ctrl_tune/fit.py`）が出るまでの机上値で、`vehicle.toml` の
実測（質量・転がり抵抗・加速と制動の頭打ち・舵の非線形・ヨーの遅れ）から逆算してある:

- 加速の頭打ち 3.42m/s²・制動の頭打ち（ロックなし）約3.3m/s² → 後輪1つの前後力の上限は
  駆動 約3.8N・制動 約3.1N。差は加減速の荷重移動（駆動で後輪に乗り、制動で抜ける）なので、
  `mu`=0.6・静止荷重 5.75N（後軸に車重の59%）・重心高/ホイールベース 0.17 で両方に合う
- 車輪の慣性 4e-5kg·m²: 実機の ABS の記録で、限界 0.09N·m に対し 0.15N·m を掛けると約20ms で
  深くロックした（超過 0.06N·m × 20ms ÷ 30rad/s）
- タイヤの形 `tyre_c`: 実機の記録は「滑らせても駆動力がほぼ落ちない」形（1.0〜1.1）を示唆するが
  確かでないので、調整はピークのある形（1.6）までを `variants()` で揺らして最悪側で選ぶ

**同定した値は `config/vehicle.toml` の `[control.plant]` に入り、`Plant.load()` がそれで上書きする。**
"""

from __future__ import annotations

import tomllib
from dataclasses import dataclass, fields, replace
from pathlib import Path

from .fw import HostPlant

__all__ = ["Plant", "NOMINAL", "variants", "IDENTIFIED_KEYS"]

REPO_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_TOML = REPO_ROOT / "config" / "vehicle.toml"

#: 実機の記録から同定する項目（`[control.plant]` に書くキー）
IDENTIFIED_KEYS = ("wheel_inertia_kgm2", "wheel_friction_nm", "md_delay_s", "md_tau_s",
                   "mu", "fz_static_n", "load_transfer", "tyre_b", "tyre_c",
                   "yaw_inertia_kgm2", "yaw_damping")


@dataclass(frozen=True)
class Plant:
    mass_kg: float = 2.0
    wheel_inertia_kgm2: float = 4.0e-5
    wheel_friction_nm: float = 0.002
    wheel_damping_nms: float = 0.0
    rolling_decel_m_s2: float = 0.366
    mu: float = 0.6
    fz_static_n: float = 5.75
    load_transfer: float = 0.17
    lateral_transfer: float = 0.13
    rear_lateral_share: float = 0.59
    tyre_b: float = 20.0
    tyre_c: float = 1.1
    slip_speed_floor_m_s: float = 0.3
    md_delay_s: float = 0.0015
    md_tau_s: float = 0.003
    md_brake_fade_rad_s: float = 20.0
    md_speed_tau_s: float = 0.002
    yaw_inertia_kgm2: float = 0.02
    yaw_damping: float = 1.5
    steer_gain: float = 0.993
    steer_gain_cubic: float = -0.39
    lateral_accel_max_m_s2: float = 4.48
    front_noise_rad_s: float = 2.2
    rear_noise_rad_s: float = 0.5
    gyro_noise_rad_s: float = 0.01
    gyro_tau_s: float = 0.0166
    gyro_bias_rad_s: float = 0.0
    #: 前輪の速度の倍率誤差（タイヤ径の誤差）と、エンコーダの1回転周期の誤差（速度に対する割合）。
    #: 机上値は 0。`variants()` が「前輪が遅く読む＝常に少し空転して見える」側へ揺らす
    front_scale_error: float = 0.0
    front_ripple: float = 0.0
    seed: int = 0

    def to_c(self) -> HostPlant:
        return HostPlant(**{f.name: getattr(self, f.name) for f in fields(self)})

    @classmethod
    def load(cls, toml_path: str | Path = DEFAULT_TOML) -> "Plant":
        """`vehicle.toml` の実測（質量・`[dynamics]`・`[control.plant]`）で机上値を上書きする。"""
        with open(toml_path, "rb") as f:
            d = tomllib.load(f)
        dyn = d.get("dynamics", {})
        over: dict[str, float] = {}
        if "mass" in d:
            over["mass_kg"] = float(d["mass"])
        for src, dst in (("rolling_resistance", "rolling_decel_m_s2"), ("steer_gain", "steer_gain"),
                         ("steer_gain_cubic", "steer_gain_cubic"), ("yaw_rate_filter_s", "gyro_tau_s")):
            if dyn.get(src):
                over[dst] = float(dyn[src])
        if dyn.get("mu"):
            over["lateral_accel_max_m_s2"] = float(dyn["mu"]) * 9.80665
        names = {f.name for f in fields(cls)}
        for k, v in d.get("control", {}).get("plant", {}).items():
            if k in names and isinstance(v, (int, float)):
                over[k] = float(v)
        return replace(cls(), **over)


NOMINAL = Plant()


def variants(base: Plant, seeds: int = 1) -> list[tuple[str, Plant]]:
    """調整・比較で使う「あり得る車」。同定の誤差と路面の違いに対して最悪側を見るため。

    1つずつ動かす（全部の組み合わせは数が増えるわりに、最悪の組は端の1つで決まることが多い）。
    """
    out: list[tuple[str, Plant]] = []
    for s in range(seeds):
        b = replace(base, seed=s)
        tag = f"#{s}" if seeds > 1 else ""
        out += [
            (f"基準{tag}", b),
            (f"低μ×0.6{tag}", replace(b, mu=b.mu * 0.6)),
            (f"高μ×1.4{tag}", replace(b, mu=b.mu * 1.4)),
            (f"ピークのあるタイヤ{tag}", replace(b, tyre_c=1.6)),
            (f"平らなタイヤ{tag}", replace(b, tyre_c=1.0)),
            (f"慣性×0.5{tag}", replace(b, wheel_inertia_kgm2=b.wheel_inertia_kgm2 * 0.5)),
            (f"慣性×2{tag}", replace(b, wheel_inertia_kgm2=b.wheel_inertia_kgm2 * 2.0)),
            (f"MDが遅い{tag}", replace(b, md_delay_s=b.md_delay_s + 0.004, md_tau_s=b.md_tau_s * 3.0)),
            (f"ノイズ×3{tag}", replace(b, front_noise_rad_s=b.front_noise_rad_s * 3.0,
                                      rear_noise_rad_s=b.rear_noise_rad_s * 3.0)),
            # 前輪が4%遅く読み（タイヤ径）、さらに1回転周期で±3%揺れる: 滑っていないのに最大7%の
            # スリップに見える。スリップ率の目標をこれより十分上に置かないと定速で誤介入する
            (f"前輪の読みの誤差{tag}", replace(b, front_scale_error=-0.04, front_ripple=0.03)),
        ]
    return out
