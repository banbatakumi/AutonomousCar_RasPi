"""`config/vehicle.toml` の `[control]`（調整パラメータ）・`[control.plant]`（同定した車両モデル）の読み書き。

書き込みは `tools/sysid/toml_update.py` と同じ方針: **該当する数値だけ**を `tomlkit` で書き換え、
コメント・書式はそのまま残す。
"""

from __future__ import annotations

from pathlib import Path

import tomlkit

from raspi.core.control_params import load_control_params

from .fw import Firmware
from .plant import DEFAULT_TOML, IDENTIFIED_KEYS

__all__ = ["load_params", "apply_control", "apply_plant", "DERIVED"]

#: `[control]` には書かず `[dynamics]` から作る項目（`raspi/core/control_params.py`）と、同定用の項目
DERIVED = frozenset({"tv_steer_gain", "tv_steer_gain_cubic", "tv_max_lateral_accel_m_s2",
                     "tv_test_moment_nm"})


def load_params(fw: Firmware, toml_path: str | Path = DEFAULT_TOML) -> dict[str, float]:
    """今の調整パラメータ（名前 → 値）。`[control]` に無い項目はファームの既定値。

    Pi の io_node が STM32 へ送るのと同じ値になる（`[dynamics]` から作る3項目を含む）。
    """
    values = {k: spec.default for k, spec in fw.params.items()}
    values.update({k: v for k, v in load_control_params(toml_path).items() if k in values})
    values["tv_test_moment_nm"] = 0.0
    return values


def _write(toml_path: str | Path, table_path: tuple[str, ...], values: dict[str, float]) -> list[str]:
    path = Path(toml_path)
    doc = tomlkit.parse(path.read_text(encoding="utf-8"))
    node = doc
    for name in table_path:
        if name not in node:
            raise ValueError(f"{path} に [{'.'.join(table_path)}] がありません")
        node = node[name]
    changed = []
    for key, value in values.items():
        old = node.get(key)
        if old is not None and abs(float(old) - float(value)) <= 1e-9 * max(1.0, abs(float(value))):
            continue
        node[key] = float(value)
        changed.append(key)
    if changed:
        path.write_text(tomlkit.dumps(doc), encoding="utf-8")
    return changed


def apply_control(toml_path: str | Path, values: dict[str, float], fw: Firmware) -> list[str]:
    """調整パラメータを `[control]` に書き、実際に変えたキーを返す。

    :raises ValueError: ファームの表に無いキー・`[dynamics]` から作る項目・範囲外の値
    """
    for key, value in values.items():
        spec = fw.params.get(key)
        if spec is None or key in DERIVED:
            raise ValueError(f"[control] に書けないキーです: {key}")
        if not (spec.lo <= value <= spec.hi):
            raise ValueError(f"{key} = {value:g} がファームの範囲 {spec.lo:g}〜{spec.hi:g} の外です")
    return _write(toml_path, ("control",), values)


def apply_plant(toml_path: str | Path, values: dict[str, float]) -> list[str]:
    """同定した車両モデルを `[control.plant]` に書く。"""
    unknown = set(values) - set(IDENTIFIED_KEYS)
    if unknown:
        raise ValueError(f"[control.plant] に書けないキーです: {sorted(unknown)}")
    return _write(toml_path, ("control", "plant"), values)
