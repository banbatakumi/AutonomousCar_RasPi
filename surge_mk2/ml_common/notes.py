"""run/モデルディレクトリの `note.txt`（自由記述の備考欄）読み書き。"""

from __future__ import annotations

from pathlib import Path

__all__ = ["read_note", "write_note"]


def read_note(dir_: Path) -> str:
    """`<dir_>/note.txt` の中身。無ければ空文字。"""
    try:
        return (dir_ / "note.txt").read_text(encoding="utf-8")
    except OSError:
        return ""


def write_note(dir_: Path, text: str) -> None:
    dir_.mkdir(parents=True, exist_ok=True)
    (dir_ / "note.txt").write_text(text, encoding="utf-8")
