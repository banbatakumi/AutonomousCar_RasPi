"""共通のTkinter確認ダイアログ。"""

from __future__ import annotations

from pathlib import Path
from tkinter import messagebox
from typing import Callable

__all__ = ["confirm_overwrite"]


def confirm_overwrite(path: Path, rel: Callable[[Path], str]) -> bool:
    """`path` が存在しなければ `True`。存在すれば上書き確認ダイアログを出し、その結果を返す。"""
    if not path.exists():
        return True
    return messagebox.askyesno("上書き確認", f"{rel(path)} は既にあります。上書きしますか？")
