"""`config/vehicle.toml` の読み書き（校正の結果を書き込む）。

キーは `Result.values` と同じ `"<表>.<キー>"`（`"control.plant.mu"` のように入れ子も `.` でつなぐ。最上位は
`"max_steer"`）。**書き換えるのは数値だけ**で、コメント・並びは `tomlkit` でそのまま残す。無いキーは表の末尾に足す。
"""

from __future__ import annotations

import os
import tempfile
from pathlib import Path

import tomlkit

from raspi.core.control_params import DEFAULT_TOML

__all__ = ["read_flat", "apply", "DEFAULT_TOML"]


def read_flat(path: str | Path = DEFAULT_TOML) -> dict[str, float]:
    """数値の項目を `"<表>.<キー>"` → 値 で返す。"""
    doc = tomlkit.parse(Path(path).read_text(encoding="utf-8"))
    out: dict[str, float] = {}

    def walk(tbl, prefix):
        for k, v in tbl.items():
            key = f"{prefix}{k}"
            if hasattr(v, "items"):
                walk(v, key + ".")
            elif isinstance(v, (int, float)) and not isinstance(v, bool):
                out[key] = float(v)

    walk(doc, "")
    return out


def _add_after_last_key(tbl, key: str, value) -> None:
    """表の最後のキーの直後に足す。tomlkit の `tbl[key] = …` は表の末尾（次の節の見出しの注釈の後ろ）に足すので、
    次の節の注釈の下に入って読み違える（2026-10-19、`md_delay_s` が「センサ取付位置」の注釈の下に入った）。"""
    body = getattr(tbl, "value", tbl).body
    last = max((i for i, (k, _) in enumerate(body) if k is not None), default=-1)
    if last < 0 or last == len(body) - 1:
        tbl[key] = value
        return
    getattr(tbl, "value", tbl)._insert_at(last + 1, key, value)


def apply(values: dict[str, float], path: str | Path = DEFAULT_TOML) -> list[str]:
    """`values` を書き込み、変わった項目の説明（"キー: 前 → 後"）を返す。"""
    p = Path(path)
    doc = tomlkit.parse(p.read_text(encoding="utf-8"))
    changed = []
    for full, val in values.items():
        parts = full.split(".")
        tbl = doc
        for name in parts[:-1]:
            if name not in tbl:
                tbl[name] = tomlkit.table()
            tbl = tbl[name]
        key = parts[-1]
        old = tbl.get(key)
        new = float(val)
        if isinstance(old, int) and not isinstance(old, bool) and float(int(new)) == new:
            new = int(new)
        if old is None or float(old) != float(new):
            changed.append(f"{full}: {old if old is not None else '（無し）'} → {new}")
            if old is None:
                _add_after_last_key(tbl, key, new)
            else:
                tbl[key] = new
    text = tomlkit.dumps(doc)
    fd, tmp = tempfile.mkstemp(dir=p.parent, prefix=".vehicle.", suffix=".toml")
    with os.fdopen(fd, "w", encoding="utf-8") as f:
        f.write(text)
    os.replace(tmp, p)
    return changed
