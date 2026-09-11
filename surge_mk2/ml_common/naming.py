"""`v1`・`v2`…形式のrun名／モデル名の一覧・採番（Tkinterを一切知らない純粋関数）。"""

from __future__ import annotations

import re
from pathlib import Path

__all__ = ["list_versioned_names", "next_versioned_name", "latest_run_name"]

_V_NAME_RE = re.compile(r"^v(\d+)$")


def list_versioned_names(root_dir: Path) -> list[str]:
    if not root_dir.exists():
        return []
    return sorted(p.name for p in root_dir.iterdir() if p.is_dir())


def latest_run_name(root_dir: Path) -> str | None:
    """最終更新（mtime最大）のrunディレクトリ名を返す。無ければ`None`。"""
    names = list_versioned_names(root_dir)
    if not names:
        return None
    return max(names, key=lambda n: (root_dir / n).stat().st_mtime)


def next_versioned_name(existing: list[str]) -> str:
    """`v1`・`v2`…のうち一番大きい番号の次を提案する。`v`始まりでない名前は無視する。
    該当が無ければ`v1`。"""
    nums = [int(m.group(1)) for name in existing if (m := _V_NAME_RE.match(name))]
    return f"v{max(nums) + 1}" if nums else "v1"
