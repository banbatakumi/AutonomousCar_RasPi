"""CPU/メモリ使用率をオンデマンドの別ウィンドウで表示する軽量モニタ。

matplotlibは使わずTkinter Canvasへ手描きの折れ線（スパークライン）・棒グラフを描画する——
`ml_common/log_console.py`のログ別窓と同じ「隠して起動→ボタンで表示」の設計。
"""

from __future__ import annotations

import tkinter as tk
from collections import deque
from tkinter import ttk

import psutil

__all__ = ["SysMonitorWindow"]

_CPU_COLOR = "#00e5ff"
_MEM_COLOR = "#ff4da6"
_CORE_ROW_H = 18


class SysMonitorWindow:
    def __init__(self, root: tk.Misc, *, history: int = 60, interval_ms: int = 1000) -> None:
        self.interval_ms = interval_ms
        self.cpu_history: deque[float] = deque(maxlen=history)
        self.mem_history: deque[float] = deque(maxlen=history)
        self.per_core: list[float] = []

        n_cores = psutil.cpu_count(logical=True) or 1
        self.window = tk.Toplevel(root)
        self.window.title("CPU/メモリ")
        self.window.geometry(f"380x{220 + n_cores * _CORE_ROW_H}")

        self.label_var = tk.StringVar(value="")
        ttk.Label(self.window, textvariable=self.label_var).pack(anchor="w", padx=6, pady=(6, 0))
        self.canvas = tk.Canvas(self.window, height=140, bg="black", highlightthickness=0)
        self.canvas.pack(fill="x", padx=6, pady=6)
        self.canvas.bind("<Configure>", lambda _e: self._draw())

        ttk.Label(self.window, text="コア別:").pack(anchor="w", padx=6)
        self.cores_canvas = tk.Canvas(self.window, bg="black", highlightthickness=0)
        self.cores_canvas.pack(fill="both", expand=True, padx=6, pady=(2, 6))
        self.cores_canvas.bind("<Configure>", lambda _e: self._draw_cores())

        self.window.protocol("WM_DELETE_WINDOW", self.window.withdraw)
        self.window.withdraw()

        # percpu=True/Falseはpsutil内部で別々に直前値を持つので、両方ウォームアップする
        psutil.cpu_percent(interval=None)
        psutil.cpu_percent(interval=None, percpu=True)
        self._tick()

    def show(self) -> None:
        self.window.deiconify()
        self.window.lift()

    def _tick(self) -> None:
        self.cpu_history.append(psutil.cpu_percent(interval=None))
        self.per_core = psutil.cpu_percent(interval=None, percpu=True)
        self.mem_history.append(psutil.virtual_memory().percent)
        self.label_var.set(f"CPU {self.cpu_history[-1]:.0f}%　MEM {self.mem_history[-1]:.0f}%")
        self._draw()
        self._draw_cores()
        self.window.after(self.interval_ms, self._tick)

    def _draw(self) -> None:
        self.canvas.delete("line")
        w = self.canvas.winfo_width()
        h = self.canvas.winfo_height()
        if w <= 1 or h <= 1:
            return
        for history, color in ((self.cpu_history, _CPU_COLOR), (self.mem_history, _MEM_COLOR)):
            if len(history) < 2:
                continue
            n = len(history)
            pts = []
            for i, v in enumerate(history):
                x = w * i / (n - 1)
                y = h - (h * min(max(v, 0.0), 100.0) / 100.0)
                pts += [x, y]
            self.canvas.create_line(*pts, fill=color, width=2, tags="line")

    def _draw_cores(self) -> None:
        self.cores_canvas.delete("bar")
        w = self.cores_canvas.winfo_width()
        h = self.cores_canvas.winfo_height()
        n = len(self.per_core)
        if w <= 1 or h <= 1 or n == 0:
            return
        row_h = h / n
        for i, pct in enumerate(self.per_core):
            y0 = i * row_h
            y1 = y0 + row_h - 2
            pct = min(max(pct, 0.0), 100.0)
            bar_w = w * pct / 100.0
            self.cores_canvas.create_rectangle(0, y0, w, y1, outline="#333333", tags="bar")
            self.cores_canvas.create_rectangle(0, y0, bar_w, y1, fill=_CPU_COLOR,
                                               outline="", tags="bar")
            self.cores_canvas.create_text(4, (y0 + y1) / 2, anchor="w", fill="white",
                                          text=f"core{i}: {pct:.0f}%", tags="bar")
