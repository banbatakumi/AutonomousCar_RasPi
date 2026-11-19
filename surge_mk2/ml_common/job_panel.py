"""学習 GUI のジョブ実行・ログ・停止・備考・モデル名の変更の共通部分（`ml_cam`・`ml_cam_e2e`・`ml_e2e`・`ml_lidar`
の `app.py`）。

4つの `App` が `_run`・`_on_job_done`・`_stop`・`_drain_log`・`_save_note`・`_on_model_name_changed` をほぼ同じ文で
持ち、完了後の処理をラベル文字列（`label in ("ペア抽出", "学習")`）で分岐していた。ラベルを言い換えると完了後の
処理が黙って抜けるので、**完了後の処理は `_run(..., on_done=...)` のコールバックで渡す**。どのジョブが終わっても
する処理（状態の表示の更新など）は `_after_job` を上書きする。

`__init__.py` の方針（巨大な基底クラスにしない）に合わせ、ここに置くのは「ジョブを1つ走らせて結果をログに出す」
ことと、モデル名に紐づく備考欄だけ。タブの中身・ウィジェットの配置は各 `App` が持つ。

使い方:

    class App(JobPanelBase):
        def __init__(self, root):
            self._init_job_panel(root, REPO_ROOT)              # self.jobs・self.python ができる
            ...ウィジェットを作る（ログ欄は `_build_log_and_status`、備考欄は `self.note_text`）...
            self._bind_model_name(self.model_name_var)         # 名前の変更 → `_on_model_name_changed`
            self._start_job_panel()                             # ログの定期取り出し・閉じるときの確認

        def _current_model_dir(self): ...                       # 名前 → フォルダ（空なら None）
"""

from __future__ import annotations

import sys
import tkinter as tk
from pathlib import Path
from tkinter import messagebox, ttk
from typing import Callable

from ml_common.close_handler import confirm_and_close
from ml_common.job_runner import JobRunner
from ml_common.log_console import LogConsole
from ml_common.notes import read_note, write_note

__all__ = ["JobPanelBase", "CONFIRM_STOP_JOB_KEYS"]

#: 学習中でも壊れない・止めても惜しくない類のジョブは確認無しで終了時に落とす。
#: 学習だけは数十分〜数時間かかりうるので、ウィンドウを閉じる前に一言確認する
CONFIRM_STOP_JOB_KEYS = frozenset({"train"})

#: ログを取り出す周期 [ms]
_DRAIN_MS = 100


class JobPanelBase:
    """ジョブ実行・ログ・停止・備考・モデル名の変更。

    `allow_concurrent=False`（既定。`ml_cam` 系・`ml_e2e`）は「同時に1つだけ」で、下の帯の `status_label`・
    `stop_btn` を使う。`True`（`ml_lidar`）は学習中も観戦・エクスポートを並行でき、停止は各 `App` がジョブ一覧から
    選んで行う（`_on_jobs_changed` で一覧を作り直す）。
    """

    root: tk.Misc
    jobs: JobRunner
    python: str
    #: 備考欄（各 `App` がタブの中に作る）
    note_text: tk.Text
    #: ログの書き先（`LogConsole` か `LogWindow`。どちらも `append(text)` を持つ）
    log_sink = None
    status_label: ttk.Label | None = None
    stop_btn: ttk.Button | None = None
    #: モデル名が空のときのエラーメッセージ
    model_name_hint = "①タブでモデル名を入力してください"

    # ── 初期化 ──

    def _init_job_panel(self, root: tk.Misc, cwd: Path, *, allow_concurrent: bool = False,
                        prefix_labels: bool = False,
                        confirm_job_keys: frozenset[str] = CONFIRM_STOP_JOB_KEYS) -> None:
        self.root = root
        self.python = sys.executable
        self.jobs = JobRunner(cwd, allow_concurrent=allow_concurrent, prefix_labels=prefix_labels)
        self._confirm_job_keys = confirm_job_keys

    def _start_job_panel(self) -> None:
        """ウィジェットを作り終えてから呼ぶ。ログの定期取り出しと、閉じるときの確認を始める。"""
        self.root.after(_DRAIN_MS, self._drain_log)
        self.root.protocol("WM_DELETE_WINDOW", lambda: confirm_and_close(
            self.root, self.jobs, confirm_job_keys=self._confirm_job_keys,
            confirm_message=lambda label: f"{label}が実行中です。終了すると学習が中断されます。"
            "\n\n本当に終了しますか？"))

    def _build_log_and_status(self, *, log_height: int, at_bottom: bool) -> LogConsole:
        """ログ欄と、その下の帯（状態の表示・停止ボタン）を作る。

        `at_bottom=True` はタブより先に呼んで下から積む（ログ欄は固定の高さ、残りをタブに渡す）。
        `False` はタブの後に呼び、ログ欄が残りの高さを取る。
        """
        def bar() -> None:
            bottom = ttk.Frame(self.root)
            bottom.pack(side="bottom" if at_bottom else "top", fill="x", padx=8, pady=(0, 8))
            self.status_label = ttk.Label(bottom, text="待機中")
            self.status_label.pack(side="left")
            self.stop_btn = ttk.Button(bottom, text="停止", command=self._stop, state="disabled")
            self.stop_btn.pack(side="right")

        if at_bottom:
            bar()
        log_frame = ttk.LabelFrame(self.root, text="ログ")
        if at_bottom:
            log_frame.pack(side="bottom", fill="x", padx=8, pady=(0, 8))
        else:
            log_frame.pack(fill="both", expand=True, padx=8, pady=(0, 8))
        console = LogConsole(log_frame, height=log_height)
        console.widget.pack(fill="both", expand=True, padx=4, pady=4)
        self.log_sink = console
        if not at_bottom:
            bar()
        return console

    # ── ジョブ ──

    def _append_log(self, text: str) -> None:
        self.log_sink.append(text)

    def _is_busy(self) -> bool:
        """同時に1つだけのとき、新しいジョブを断るか（ジョブ以外の処理を足すなら上書きする）。"""
        return self.jobs.is_busy()

    def _set_idle(self, idle: bool, label: str = "") -> None:
        if self.status_label is not None:
            self.status_label.config(text="待機中" if idle else f"{label} 実行中…")
        if self.stop_btn is not None:
            self.stop_btn.config(state="disabled" if idle else "normal")

    def _run(self, job_key: str, cmd: list[str], label: str, *,
             on_line: Callable[[str], None] | None = None,
             on_done: Callable[[bool], None] | None = None) -> None:
        """`cmd` をジョブ `job_key` として走らせる。`on_done(成功したか)` は完了後、`_after_job` の後に呼ぶ。"""
        if self.jobs.allow_concurrent:
            self._append_log(f"\n$ {' '.join(cmd)}\n")
            if not self.jobs.start(job_key, cmd, label, on_line=on_line,
                                   on_done=lambda code: self._on_job_done(code, on_done)):
                messagebox.showwarning("実行中", f"{label}は既に実行中です")
            self._on_jobs_changed()
            return
        if self._is_busy():
            messagebox.showwarning("実行中", "他の処理が終わってから実行してください")
            return
        self._set_idle(False, label)
        self._append_log(f"\n$ {' '.join(cmd)}\n")
        self.jobs.start(job_key, cmd, label, on_line=on_line,
                        on_done=lambda code: self._on_job_done(code, on_done))

    def _on_job_done(self, code: int, on_done: Callable[[bool], None] | None = None) -> None:
        ok = code == 0
        if not self.jobs.allow_concurrent:
            self._set_idle(True)
        self._after_job(ok)
        if on_done is not None:
            on_done(ok)
        self._on_jobs_changed()

    def _after_job(self, ok: bool) -> None:
        """どのジョブが終わってもする処理（既定は何もしない）。"""

    def _on_jobs_changed(self) -> None:
        """ジョブが始まった・終わった（並行実行のジョブ一覧を作り直す。既定は何もしない）。"""

    def _stop(self) -> None:
        """実行中のジョブ（同時に1つだけなのでその1つ）へ停止を指示する。"""
        active = self.jobs.active_jobs()
        if active and self.jobs.stop(active[0][0]):
            self._append_log("\n（停止を指示しました）\n")

    def _drain_log(self) -> None:
        self.jobs.drain(self._append_log)
        self._poll()
        self.root.after(_DRAIN_MS, self._drain_log)

    def _poll(self) -> None:
        """ログの取り出しのたびに呼ぶ（ジョブ以外の処理の進み具合を拾う。既定は何もしない）。"""

    # ── モデル名・備考 ──

    def _current_model_dir(self) -> Path | None:
        """今のモデル名のフォルダ。名前が空なら `None`。"""
        raise NotImplementedError

    def _require_model_dir(self) -> Path | None:
        d = self._current_model_dir()
        if d is None:
            messagebox.showerror("入力エラー", self.model_name_hint)
        return d

    def _bind_model_name(self, var: tk.StringVar) -> None:
        """名前が変わるたびに `_on_model_name_changed` を呼ぶ。ウィジェット（`note_text` 等）が揃ってから呼ぶ。
        今の値でも1回呼ぶ。"""
        var.trace_add("write", lambda *_a: self._on_model_name_changed())
        self._on_model_name_changed()

    def _on_model_name_changed(self) -> None:
        """既定は備考欄を今のモデルの `note.txt` に差し替えるだけ。パスの表示などは上書きして足す。"""
        self._show_note(self._current_model_dir())

    def _show_note(self, model_dir: Path | None) -> None:
        self.note_text.delete("1.0", "end")
        self.note_text.insert("1.0", read_note(model_dir) if model_dir is not None else "")

    def _save_note(self) -> None:
        d = self._require_model_dir()
        if d is None:
            return
        write_note(d, self.note_text.get("1.0", "end-1c"))
        self._append_log(f"\n備考を保存しました（{d.name}）\n")
