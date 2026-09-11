"""ログ表示用の別ウィンドウ（`ml_lidar`・`ml_cam`・`ml_cam_e2e`の学習GUI共通）。

常時メインウィンドウを占有する埋め込みログ欄ではなく、隠した状態で起動して
ボタンで開閉するToplevelにする。閉じても`withdraw()`で隠すだけなので、
非表示中もジョブの出力はバックグラウンドで蓄積され続ける。
"""

from __future__ import annotations

import tkinter as tk

from ml_common.log_console import LogConsole

__all__ = ["LogWindow"]


class LogWindow:
    def __init__(self, root: tk.Misc, *, height: int = 20, geometry: str = "760x420") -> None:
        self.window = tk.Toplevel(root)
        self.window.title("ログ")
        self.window.geometry(geometry)
        self.console = LogConsole(self.window, height=height)
        self.console.widget.pack(fill="both", expand=True, padx=4, pady=4)
        self.window.protocol("WM_DELETE_WINDOW", self.window.withdraw)
        self.window.withdraw()

    def append(self, text: str) -> None:
        self.console.append(text)

    def show(self) -> None:
        self.window.deiconify()
        self.window.lift()
