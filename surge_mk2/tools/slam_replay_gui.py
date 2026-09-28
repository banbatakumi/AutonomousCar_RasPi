"""記録に SLAM の自己位置を足す GUI（Tkinter、Mac 側）。

    .venv/bin/python -m tools.slam_replay_gui
    （または surge_mk2 直下の launcher.command から起動）

`.mcap` / `.sfl` を選んで「実行」を押すと、`raspi/tools/slam_replay.py` を通して
`<入力>_slam.mcap` を作り、終わったら Foxglove で開く。CLI と中身は同じ。

ネイティブ GUI にしてあるのは、ファイル選択から Foxglove で開くまでを
ターミナル操作なしで完結させるため（バンビの補助ツール全般の方針。`tools/sysid/gui.py` と同じ）。

## 処理は別スレッドで回す

slam2d は1本あたり数秒〜数十秒かかる。Tk の主スレッドで回すと、その間ウィンドウが
固まって「応答なし」になる。作業スレッドは結果を `queue` に積むだけにして、
画面の更新は主スレッドが `after()` で拾う（**Tk は主スレッド以外から触ると壊れる**）。
"""

from __future__ import annotations

import queue
import subprocess
import threading
import time
import tkinter as tk
import traceback
from pathlib import Path
from tkinter import filedialog, messagebox, ttk

from raspi.tools.slam_replay import default_out_path, format_summary, replay

DEFAULT_DIR = Path.home() / "Downloads"


def _open_in_foxglove(path: Path) -> None:
    """Foxglove で開く。**入っていなければ既定のアプリ**に任せる。"""
    r = subprocess.run(["open", "-a", "Foxglove", str(path)], capture_output=True)
    if r.returncode != 0:
        subprocess.run(["open", str(path)])


class SlamReplayApp(tk.Tk):
    def __init__(self) -> None:
        super().__init__()
        self.title("SLAM 再処理 — 記録に自己位置を足す")
        self.geometry("820x600")

        self.files: list[Path] = []
        self.outputs: list[Path] = []
        self.loop_closure = tk.BooleanVar(value=True)
        self.open_after = tk.BooleanVar(value=True)
        self.out_dir = tk.StringVar(value="")         # 空なら入力と同じフォルダ
        self._q: queue.Queue = queue.Queue()
        self._worker: threading.Thread | None = None

        self._build()
        self.after(100, self._drain)

    # ── 画面 ──

    def _build(self) -> None:
        pad = {"padx": 8, "pady": 4}

        top = ttk.LabelFrame(self, text="入力（.mcap / .sfl）")
        top.pack(fill="both", expand=False, **pad)
        self.listbox = tk.Listbox(top, height=6, selectmode="extended")
        self.listbox.pack(side="left", fill="both", expand=True, padx=4, pady=4)
        btns = ttk.Frame(top)
        btns.pack(side="left", fill="y", padx=4)
        ttk.Button(btns, text="ファイルを追加…", command=self._add_files).pack(fill="x", pady=2)
        ttk.Button(btns, text="選択を外す", command=self._remove_selected).pack(fill="x", pady=2)
        ttk.Button(btns, text="全部外す", command=self._clear).pack(fill="x", pady=2)

        opt = ttk.LabelFrame(self, text="設定")
        opt.pack(fill="x", **pad)
        ttk.Checkbutton(opt, text="ループ閉じを使う（実機の slam2d_raceline と同じ。最後に1回だけ最適化）",
                        variable=self.loop_closure).pack(anchor="w", padx=4)
        ttk.Checkbutton(opt, text="終わったら Foxglove で開く（最後の1本）",
                        variable=self.open_after).pack(anchor="w", padx=4)
        row = ttk.Frame(opt)
        row.pack(fill="x", padx=4, pady=2)
        ttk.Label(row, text="出力先:").pack(side="left")
        self.out_label = ttk.Label(row, text="入力と同じフォルダ（<入力>_slam.mcap）")
        self.out_label.pack(side="left", padx=4)
        ttk.Button(row, text="変更…", command=self._choose_out_dir).pack(side="left")
        ttk.Button(row, text="元に戻す", command=self._reset_out_dir).pack(side="left", padx=2)

        run = ttk.Frame(self)
        run.pack(fill="x", **pad)
        self.run_btn = ttk.Button(run, text="実行", command=self._run)
        self.run_btn.pack(side="left")
        self.progress = ttk.Progressbar(run, length=320, mode="determinate", maximum=1.0)
        self.progress.pack(side="left", padx=8)
        self.status = ttk.Label(run, text="ファイルを追加してください")
        self.status.pack(side="left")

        res = ttk.LabelFrame(self, text="結果")
        res.pack(fill="both", expand=True, **pad)
        self.text = tk.Text(res, height=14, wrap="none", font=("Menlo", 11))
        self.text.pack(fill="both", expand=True, padx=4, pady=4)
        self.text.configure(state="disabled")

        after = ttk.Frame(self)
        after.pack(fill="x", **pad)
        self.fox_btn = ttk.Button(after, text="Foxglove で開く", state="disabled",
                                  command=self._open_last)
        self.fox_btn.pack(side="left")
        self.finder_btn = ttk.Button(after, text="Finder で表示", state="disabled",
                                     command=self._reveal_last)
        self.finder_btn.pack(side="left", padx=4)
        ttk.Label(after, foreground="gray",
                  text="Foxglove: 3D パネルで Display frame を odom にし、/viz/* の目を点ける"
                  ).pack(side="left", padx=8)

    def _log(self, s: str) -> None:
        self.text.configure(state="normal")
        self.text.insert("end", s + "\n")
        self.text.see("end")
        self.text.configure(state="disabled")

    # ── 入力の管理 ──

    def _add_files(self) -> None:
        paths = filedialog.askopenfilenames(
            title="記録を選ぶ", initialdir=str(DEFAULT_DIR) if DEFAULT_DIR.exists() else None,
            filetypes=[("記録", "*.mcap *.sfl"), ("MCAP", "*.mcap"), ("SFL", "*.sfl")])
        for p in map(Path, paths):
            if p.stem.endswith("_slam"):
                # **出力をもう一度通すと `_slam_slam` になるだけ**（中身は作り直しで同じ）
                self._log(f"スキップ: {p.name}（このツールの出力）")
                continue
            if p not in self.files:
                self.files.append(p)
                self.listbox.insert("end", str(p))
        self.status.configure(text=f"{len(self.files)}本")

    def _remove_selected(self) -> None:
        for i in reversed(self.listbox.curselection()):
            self.listbox.delete(i)
            del self.files[i]
        self.status.configure(text=f"{len(self.files)}本")

    def _clear(self) -> None:
        self.listbox.delete(0, "end")
        self.files.clear()
        self.status.configure(text="ファイルを追加してください")

    def _choose_out_dir(self) -> None:
        d = filedialog.askdirectory(title="出力先フォルダ")
        if d:
            self.out_dir.set(d)
            self.out_label.configure(text=d)

    def _reset_out_dir(self) -> None:
        self.out_dir.set("")
        self.out_label.configure(text="入力と同じフォルダ（<入力>_slam.mcap）")

    def _out_path(self, src: Path) -> Path:
        out = default_out_path(src)
        return Path(self.out_dir.get()) / out.name if self.out_dir.get() else out

    # ── 実行 ──

    def _run(self) -> None:
        if self._worker is not None and self._worker.is_alive():
            return
        if not self.files:
            messagebox.showinfo("入力がありません", "「ファイルを追加…」で記録を選んでください")
            return
        jobs = [(src, self._out_path(src)) for src in self.files]
        exists = [o for _, o in jobs if o.exists()]
        if exists and not messagebox.askyesno(
                "上書きの確認",
                "次のファイルは既にあります。上書きしますか？\n\n"
                + "\n".join(f"・{o.name}" for o in exists[:8])
                + (f"\n…他{len(exists) - 8}本" if len(exists) > 8 else "")):
            return
        self.run_btn.configure(state="disabled")
        self.outputs.clear()
        self._worker = threading.Thread(target=self._work, args=(jobs, self.loop_closure.get()),
                                        daemon=True)
        self._worker.start()

    def _work(self, jobs: list[tuple[Path, Path]], loop_closure: bool) -> None:
        """作業スレッド。**Tk には触らず `_q` に積むだけ**（モジュール docstring）。"""
        for i, (src, out) in enumerate(jobs):
            self._q.put(("start", i, len(jobs), src))
            t0 = time.perf_counter()
            try:
                log, slam = replay(src, out, loop_closure=loop_closure,
                                   progress=lambda f: self._q.put(("progress", f)))
                self._q.put(("done", out,
                             format_summary(src, out, log, slam.summary(),
                                            time.perf_counter() - t0)))
            except Exception as e:  # noqa: BLE001 — 1本の失敗で残りを止めない
                self._q.put(("error", src, f"{type(e).__name__}: {e}",
                             traceback.format_exc()))
        self._q.put(("finished",))

    def _drain(self) -> None:
        try:
            while True:
                self._handle(self._q.get_nowait())
        except queue.Empty:
            pass
        self.after(100, self._drain)

    def _handle(self, ev: tuple) -> None:
        kind = ev[0]
        if kind == "start":
            _, i, n, src = ev
            self.status.configure(text=f"{i + 1}/{n}  {src.name}")
            if src.suffix == ".sfl":
                # `.sfl` は全体の長さが先に分からない（`replay` の docstring）
                self.progress.configure(mode="indeterminate")
                self.progress.start(15)
            else:
                self.progress.stop()
                self.progress.configure(mode="determinate", value=0.0)
        elif kind == "progress":
            self.progress.configure(value=min(1.0, max(0.0, ev[1])))
        elif kind == "done":
            _, out, summary = ev
            self.outputs.append(out)
            self._log(summary + "\n")
        elif kind == "error":
            _, src, msg, tb = ev
            self._log(f"!! {src.name}: {msg}\n")
            print(tb)                                  # 詳細はターミナル側に残す
        elif kind == "finished":
            self.progress.stop()
            self.progress.configure(mode="determinate", value=1.0 if self.outputs else 0.0)
            self.run_btn.configure(state="normal")
            ok = bool(self.outputs)
            self.fox_btn.configure(state="normal" if ok else "disabled")
            self.finder_btn.configure(state="normal" if ok else "disabled")
            self.status.configure(text=f"完了（{len(self.outputs)}本）")
            if ok and self.open_after.get():
                _open_in_foxglove(self.outputs[-1])

    def _open_last(self) -> None:
        if self.outputs:
            _open_in_foxglove(self.outputs[-1])

    def _reveal_last(self) -> None:
        if self.outputs:
            subprocess.run(["open", "-R", str(self.outputs[-1])])


def main() -> None:
    SlamReplayApp().mainloop()


if __name__ == "__main__":
    main()
