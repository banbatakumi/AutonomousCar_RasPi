"""サブプロセスの標準出力を流し込むだけの薄いログ欄。"""

from __future__ import annotations

import tkinter as tk
from tkinter.scrolledtext import ScrolledText

__all__ = ["LogConsole"]


class LogConsole:
    """`ScrolledText` 1枚のラッパー。"""

    def __init__(self, parent: tk.Widget, *, height: int = 16) -> None:
        self._widget = ScrolledText(parent, height=height, state="disabled")

    @property
    def widget(self) -> ScrolledText:
        """pack/gridはApp側が行う。"""
        return self._widget

    def append(self, text: str) -> None:
        self._widget.config(state="normal")
        self._widget.insert("end", text)
        self._widget.see("end")
        self._widget.config(state="disabled")
