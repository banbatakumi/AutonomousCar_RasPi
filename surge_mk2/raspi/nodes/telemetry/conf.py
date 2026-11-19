"""`config/*.json`（GUI の設定・選んだモデル。機械ごとの状態）の読み書き。

- 読む: **無ければ黙って `None`**（初回起動は正常）。読めない・壊れている・dict でなければ**警告して** `None`
- 書く: 原子的に置き換える（`core/atomic.py`）。失敗しても走行は続けるが、**警告は必ず出す**——
  黙って捨てると、再起動して設定が消えていても原因を追えない
"""

from __future__ import annotations

from pathlib import Path

import msgspec

from raspi.core.atomic import atomic_write_bytes
from raspi.core.cleanup import warn_on_error
from raspi.core.log import get_logger

_log = get_logger("telemetry")

__all__ = ["read_conf", "write_conf"]

_encode = msgspec.json.encode
_decode = msgspec.json.decode


def read_conf(path: Path) -> dict | None:
    """`path` の JSON（dict）を返す。無い・壊れている・dict でないなら `None`。"""
    try:
        raw = path.read_bytes()
    except FileNotFoundError:
        return None
    except OSError as e:
        _log.warning("%s を読めない（既定のまま）: %r", path.name, e)
        return None
    try:
        obj = _decode(raw)
    except msgspec.DecodeError as e:
        _log.warning("%s が壊れている（既定のまま）: %s", path.name, e)
        return None
    if not isinstance(obj, dict):
        _log.warning("%s が dict でない（既定のまま）", path.name)
        return None
    return obj


def write_conf(path: Path, obj) -> None:
    """`obj` を JSON で `path` へ原子的に書く。失敗は警告して捨てる（保存できなくても走行は続ける）。"""
    with warn_on_error(f"{path.name} の保存"):
        atomic_write_bytes(path, _encode(obj))
