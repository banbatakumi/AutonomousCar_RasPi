"""ウィンドウクローズ時に実行中の子プロセスを道連れに終了する共通ハンドラ。

「学習中なら確認、その他は無条件終了」という挙動を3アプリ共通にする
（`WM_DELETE_WINDOW`のハンドラが無いと、閉じても子プロセスが孤児化して残る）。
"""

from __future__ import annotations

import tkinter as tk
from tkinter import messagebox
from typing import Callable

from ml_common.job_runner import JobRunner

__all__ = ["confirm_and_close"]


def confirm_and_close(root: tk.Tk, jobs: JobRunner, *,
                      confirm_job_keys: frozenset[str] = frozenset(),
                      confirm_message: Callable[[str], str] | None = None) -> None:
    """`WM_DELETE_WINDOW` ハンドラ本体。

    `confirm_job_keys` に含まれるジョブが実行中なら `askyesno` で確認し、
    「いいえ」ならウィンドウを閉じない。それ以外の実行中ジョブは無条件で
    `terminate()` する。全部OKなら全ジョブを終了させてから `root.destroy()`。
    """
    running = jobs.running_job_keys()
    for key in running:
        if key not in confirm_job_keys:
            continue
        label = jobs.label_of(key) or key
        message = (confirm_message(label) if confirm_message is not None else
                  f"{label} が実行中です。ウィンドウを閉じるとこのプロセスも終了します。\n\n"
                  "本当に終了しますか？")
        if not messagebox.askyesno("終了しますか？", message):
            return
    jobs.terminate_all()
    root.destroy()
