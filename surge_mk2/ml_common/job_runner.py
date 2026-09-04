"""サブプロセス実行の共通基盤。

`ml_cam`/`ml_cam_e2e` の「同時に1つしか動かさない」運用（`allow_concurrent=False`）と、
`ml_lidar` の「学習+TensorBoard+観戦+エクスポートを並行実行したい」運用
（`allow_concurrent=True`）を、同じ辞書ベースAPIで両立させる。

ワーカースレッドは標準出力を1行ずつ `queue.Queue` に積み、`drain()` を
`root.after(100, ...)` から定期的に呼んでUIスレッドへ反映する（旧
`ml_lidar/app.py` の `_active` 辞書管理の一般化）。
"""

from __future__ import annotations

import queue
import subprocess
import threading
from pathlib import Path
from typing import Callable

__all__ = ["JobRunner"]


class JobRunner:
    """サブプロセスジョブをスレッドで実行し、ログ・完了通知をUIスレッドへ届ける。

    :param cwd: 起動する子プロセスの作業ディレクトリ（通常はリポジトリルート）。
    :param allow_concurrent: `False` なら「他に1つでも実行中のジョブがあれば
        `start()` を拒否する」（`ml_cam`/`ml_cam_e2e` の `self.busy` と同じ意味論）。
        `True` なら `job_key` ごとに独立実行できる（`ml_lidar` の複数ジョブ運用）。
    :param prefix_labels: `True` なら実行中のログ各行の先頭に `"[label] "` を
        前置する（`ml_lidar` 方式）。`False` なら前置しない（`ml_cam` 方式）。
        ジョブ完了時のメッセージには `allow_concurrent` に関わらず常に付く。
    """

    def __init__(self, cwd: Path, *, allow_concurrent: bool = True,
                 prefix_labels: bool = False) -> None:
        self.cwd = cwd
        self.allow_concurrent = allow_concurrent
        self.prefix_labels = prefix_labels
        self._procs: dict[str, subprocess.Popen | None] = {}
        self._labels: dict[str, str] = {}
        self._on_line_cbs: dict[str, Callable[[str], None]] = {}
        self._on_done_cbs: dict[str, Callable[[int], None]] = {}
        self.log_queue: "queue.Queue[tuple]" = queue.Queue()

    def is_busy(self) -> bool:
        """1つでも実行中（起動処理中を含む）のジョブがあれば `True`。"""
        return bool(self._procs)

    def active_jobs(self) -> list[tuple[str, str]]:
        """`[(job_key, label), ...]`。`ml_lidar` の `jobs_listbox` 表示用。"""
        return [(k, self._labels[k]) for k in self._procs]

    def label_of(self, job_key: str) -> str | None:
        return self._labels.get(job_key)

    def running_job_keys(self) -> list[str]:
        """起動処理中を除く、実際にプロセスが動いているジョブキー一覧。"""
        return [k for k, p in self._procs.items() if p is not None and p.poll() is None]

    def start(self, job_key: str, cmd: list[str], label: str, *,
              on_line: Callable[[str], None] | None = None,
              on_done: Callable[[int], None] | None = None) -> bool:
        """ジョブを起動する。同じ `job_key` が既に実行中、または
        `allow_concurrent=False` で他ジョブが実行中なら起動せず `False` を返す
        （呼び出し側が `messagebox` 等で伝える）。起動できれば `True`。"""
        if job_key in self._procs:
            return False
        if not self.allow_concurrent and self._procs:
            return False

        self._procs[job_key] = None                    # 起動処理中プレースホルダ
        self._labels[job_key] = label
        if on_line is not None:
            self._on_line_cbs[job_key] = on_line
        if on_done is not None:
            self._on_done_cbs[job_key] = on_done

        def worker() -> None:
            code = -1
            try:
                proc = subprocess.Popen(cmd, cwd=str(self.cwd), stdout=subprocess.PIPE,
                                        stderr=subprocess.STDOUT, text=True, bufsize=1)
                self._procs[job_key] = proc
                for line in proc.stdout:                # type: ignore[union-attr]
                    text = f"[{label}] {line}" if self.prefix_labels else line
                    self.log_queue.put(("log", job_key, text))
                code = proc.wait()
            except Exception as e:                       # noqa: BLE001 — GUIなので握って表示する
                text = (f"[{label}] 実行に失敗しました: {e}\n" if self.prefix_labels
                       else f"実行に失敗しました: {e}\n")
                self.log_queue.put(("log", job_key, text))
            finally:
                self.log_queue.put(("done", job_key, code))

        threading.Thread(target=worker, daemon=True).start()
        return True

    def stop(self, job_key: str) -> bool:
        """`proc.terminate()` を送る。起動処理中（Popen未生成）なら何もせず `False`。"""
        proc = self._procs.get(job_key)
        if proc is None:
            return False
        proc.terminate()
        return True

    def terminate_all(self) -> list[str]:
        """実行中の全ジョブへ `terminate()` を送る。停止させた `job_key` の一覧を返す
        （`close_handler.confirm_and_close` から呼ばれる）。"""
        stopped = []
        for k, p in list(self._procs.items()):
            if p is not None and p.poll() is None:
                p.terminate()
                stopped.append(k)
        return stopped

    def drain(self, on_log: Callable[[str], None]) -> None:
        """`root.after(100, ...)` から毎回呼ぶ。queueが空になるまで処理し、
        `on_log(text)` でログ欄へ、`start()`で登録した `on_line`/`on_done` を
        各ジョブへ配る。"done" イベントでは完了メッセージを自動的に `on_log` 経由で流す。"""
        try:
            while True:
                kind, job_key, payload = self.log_queue.get_nowait()
                if kind == "log":
                    on_log(payload)
                    cb = self._on_line_cbs.get(job_key)
                    if cb is not None:
                        cb(payload)
                elif kind == "done":
                    code = payload
                    label = self._labels.pop(job_key, job_key)
                    self._procs.pop(job_key, None)
                    self._on_line_cbs.pop(job_key, None)
                    ok = code == 0
                    on_log(f"\n[{label}] {'完了' if ok else f'終了コード {code}'}\n")
                    done_cb = self._on_done_cbs.pop(job_key, None)
                    if done_cb is not None:
                        done_cb(code)
        except queue.Empty:
            pass
