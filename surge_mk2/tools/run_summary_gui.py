"""走行サマリの GUI（Tkinter、Mac 側）。

    .venv/bin/python -m tools.run_summary_gui
    （または surge_mk2 直下の launcher.command から起動）

実機の `.mcap` を選んで「作成」を押すと、`raspi/tools/run_summary.py` で周回タイム・安全・電源・学習・
通信を 1 枚にまとめて画面に出す。CLI と中身は同じ。

- **保存は任意**。「結果を保存する」を外せば画面に出すだけ。保存先は自由に選べる
  （既定は記録と同じフォルダ。`<記録>_summary.json` と `.md`）
- **前回との比較**は、保存先に残っている `*_summary.json` のうち直前の走行を自動で選ぶ。
  手で選ぶことも、比較しないこともできる
- 結果は Markdown なので「コピー」で PROGRESS.md にそのまま貼れる

処理は別スレッドで回し、画面は `queue` を `after()` で拾う（Tk は主スレッド以外から触ると壊れる。
`tools/slam_replay_gui.py` と同じ）。
"""

from __future__ import annotations

import queue
import subprocess
import threading
import tkinter as tk
import traceback
from pathlib import Path
from tkinter import filedialog, messagebox, ttk

from raspi.tools import run_summary as rs

DEFAULT_DIR = Path.home() / "Downloads"


def latest_record(directory: Path = DEFAULT_DIR) -> Path | None:
    """`surge_*.mcap`（SLAM 再処理の出力 `_slam` を除く）のうち、いちばん新しいもの。"""
    try:
        files = [p for p in directory.glob("surge_*.mcap") if not p.stem.endswith("_slam")]
    except OSError:
        return None
    return max(files, key=lambda p: p.stat().st_mtime, default=None)


class RunSummaryApp(tk.Tk):
    def __init__(self) -> None:
        super().__init__()
        self.title("走行サマリ — 実機の記録を 1 枚にまとめる")
        self.geometry("980x760")

        self.src = tk.StringVar(value="")
        self.save = tk.BooleanVar(value=True)
        self.out_dir = tk.StringVar(value="")             # 空なら記録と同じフォルダ
        self.prev_mode = tk.StringVar(value="auto")       # auto / file / none
        self.prev_file = tk.StringVar(value="")
        self._q: queue.Queue = queue.Queue()
        self._worker: threading.Thread | None = None
        self._md = ""
        self._saved: list[Path] = []

        self._build()
        latest = latest_record()
        if latest is not None:
            self.src.set(str(latest))
            self.status.configure(text="最新の記録を選んであります。「作成」を押してください")
        self.after(100, self._drain)

    # ── 画面 ──

    def _build(self) -> None:
        pad = {"padx": 8, "pady": 4}

        top = ttk.LabelFrame(self, text="記録（.mcap）")
        top.pack(fill="x", **pad)
        ttk.Entry(top, textvariable=self.src).pack(side="left", fill="x", expand=True, padx=4, pady=4)
        ttk.Button(top, text="選ぶ…", command=self._choose_src).pack(side="left", padx=2)
        ttk.Button(top, text="最新", command=self._use_latest).pack(side="left", padx=(2, 4))

        opt = ttk.LabelFrame(self, text="設定")
        opt.pack(fill="x", **pad)
        row = ttk.Frame(opt)
        row.pack(fill="x", padx=4, pady=2)
        ttk.Checkbutton(row, text="結果を保存する", variable=self.save,
                        command=self._refresh_labels).pack(side="left")
        ttk.Label(row, text="  保存先:").pack(side="left")
        self.out_label = ttk.Label(row, text="")
        self.out_label.pack(side="left", padx=4)
        ttk.Button(row, text="変更…", command=self._choose_out_dir).pack(side="left")
        ttk.Button(row, text="記録と同じ", command=self._reset_out_dir).pack(side="left", padx=2)

        row2 = ttk.Frame(opt)
        row2.pack(fill="x", padx=4, pady=2)
        ttk.Label(row2, text="前回との比較:").pack(side="left")
        for text, val in (("保存先の直前の走行（自動）", "auto"), ("ファイルを選ぶ", "file"),
                          ("しない", "none")):
            ttk.Radiobutton(row2, text=text, value=val, variable=self.prev_mode,
                            command=self._refresh_labels).pack(side="left", padx=4)
        self.prev_label = ttk.Label(row2, text="", foreground="gray")
        self.prev_label.pack(side="left", padx=4)
        ttk.Button(row2, text="比較元…", command=self._choose_prev).pack(side="left")

        run_row = ttk.Frame(self)
        run_row.pack(fill="x", **pad)
        self.run_btn = ttk.Button(run_row, text="作成", command=self._run)
        self.run_btn.pack(side="left")
        self.progress = ttk.Progressbar(run_row, length=320, mode="determinate", maximum=1.0)
        self.progress.pack(side="left", padx=8)
        self.status = ttk.Label(run_row, text="記録を選んでください")
        self.status.pack(side="left")

        res = ttk.LabelFrame(self, text="サマリ（Markdown）")
        res.pack(fill="both", expand=True, **pad)
        self.text = tk.Text(res, wrap="none", font=("Menlo", 11))
        ys = ttk.Scrollbar(res, orient="vertical", command=self.text.yview)
        self.text.configure(yscrollcommand=ys.set, state="disabled")
        ys.pack(side="right", fill="y")
        self.text.pack(fill="both", expand=True, padx=4, pady=4)

        bot = ttk.Frame(self)
        bot.pack(fill="x", **pad)
        self.copy_btn = ttk.Button(bot, text="コピー", state="disabled", command=self._copy)
        self.copy_btn.pack(side="left")
        self.reveal_btn = ttk.Button(bot, text="Finder で表示", state="disabled", command=self._reveal)
        self.reveal_btn.pack(side="left", padx=4)

        self._refresh_labels()

    def _refresh_labels(self) -> None:
        if not self.save.get():
            self.out_label.configure(text="（保存しない）")
        else:
            self.out_label.configure(text=self.out_dir.get() or "記録と同じフォルダ")
        mode = self.prev_mode.get()
        self.prev_label.configure(text={
            "auto": "", "none": "",
            "file": self.prev_file.get() or "（未選択）"}[mode])

    # ── 入力 ──

    def _choose_src(self) -> None:
        p = filedialog.askopenfilename(
            title="記録を選ぶ", initialdir=str(DEFAULT_DIR) if DEFAULT_DIR.exists() else None,
            filetypes=[("MCAP", "*.mcap")])
        if p:
            self.src.set(p)

    def _use_latest(self) -> None:
        latest = latest_record()
        if latest is None:
            messagebox.showinfo("記録がありません", f"{DEFAULT_DIR} に surge_*.mcap がありません")
        else:
            self.src.set(str(latest))

    def _choose_out_dir(self) -> None:
        d = filedialog.askdirectory(title="保存先フォルダ")
        if d:
            self.out_dir.set(d)
            self.save.set(True)
            self._refresh_labels()

    def _reset_out_dir(self) -> None:
        self.out_dir.set("")
        self._refresh_labels()

    def _choose_prev(self) -> None:
        p = filedialog.askopenfilename(
            title="比較元のサマリ（*_summary.json）",
            initialdir=self.out_dir.get() or str(DEFAULT_DIR),
            filetypes=[("サマリ", "*_summary.json"), ("JSON", "*.json")])
        if p:
            self.prev_file.set(p)
            self.prev_mode.set("file")
            self._refresh_labels()

    # ── 実行 ──

    def _run(self) -> None:
        if self._worker is not None and self._worker.is_alive():
            return
        src = Path(self.src.get()).expanduser()
        if not src.is_file():
            messagebox.showinfo("記録がありません", "「選ぶ…」で .mcap を選んでください")
            return
        out_dir: Path | None = None
        if self.save.get():
            out_dir = Path(self.out_dir.get()).expanduser() if self.out_dir.get() else src.parent
        mode = self.prev_mode.get()
        prev: Path | None = None
        if mode == "file":
            if not self.prev_file.get():
                messagebox.showinfo("比較元がありません", "「比較元…」でサマリを選ぶか、比較しない設定にしてください")
                return
            prev = Path(self.prev_file.get())
        # 保存しない・比較なしなら、保存先を探させない（`run` は out_dir から前回を探す）
        search_dir = out_dir if mode == "auto" else None
        if mode == "auto" and out_dir is None:
            search_dir = src.parent
        if out_dir is not None:
            existing = [out_dir / f"{src.stem}_summary.json", out_dir / f"{src.stem}_summary.md"]
            if any(p.exists() for p in existing) and not messagebox.askyesno(
                    "上書きの確認", "このサマリは既にあります。上書きしますか？\n\n"
                    + "\n".join(f"・{p.name}" for p in existing if p.exists())):
                return
        self.run_btn.configure(state="disabled")
        self.copy_btn.configure(state="disabled")
        self.reveal_btn.configure(state="disabled")
        self._worker = threading.Thread(
            target=self._work, args=(src, out_dir, search_dir, prev, mode == "none"), daemon=True)
        self._worker.start()

    def _work(self, src: Path, out_dir: Path | None, search_dir: Path | None, prev: Path | None,
              no_compare: bool) -> None:
        """作業スレッド。**Tk には触らず `_q` に積むだけ**（モジュール docstring）。"""
        self._q.put(("start", src))
        try:
            s = rs.summarize(src, progress=lambda f: self._q.put(("progress", f)))
            prev_s = None
            if not no_compare:
                if prev is not None:
                    prev_s = rs.load_summary(prev)
                elif search_dir is not None and search_dir.is_dir():
                    found = rs.find_previous(search_dir, s)
                    prev_s = rs.load_summary(found) if found else None
            md = rs.render_markdown(s, rs.compare(s, prev_s) if prev_s else None)
            saved = list(rs.save(s, md, out_dir)) if out_dir is not None else []
            self._q.put(("done", md, saved))
        except Exception as e:  # noqa: BLE001 — 失敗は画面に出して続ける
            self._q.put(("error", f"{type(e).__name__}: {e}", traceback.format_exc()))

    def _drain(self) -> None:
        try:
            while True:
                self._handle(self._q.get_nowait())
        except queue.Empty:
            pass
        self.after(100, self._drain)

    def _set_text(self, s: str) -> None:
        self.text.configure(state="normal")
        self.text.delete("1.0", "end")
        self.text.insert("1.0", s)
        self.text.configure(state="disabled")

    def _handle(self, ev: tuple) -> None:
        kind = ev[0]
        if kind == "start":
            self.status.configure(text=f"解析中… {ev[1].name}")
            self.progress.configure(value=0.0)
        elif kind == "progress":
            self.progress.configure(value=min(1.0, max(0.0, ev[1])))
        elif kind == "done":
            _, md, saved = ev
            self._md, self._saved = md, saved
            self._set_text(md)
            self.progress.configure(value=1.0)
            self.run_btn.configure(state="normal")
            self.copy_btn.configure(state="normal")
            self.reveal_btn.configure(state="normal" if saved else "disabled")
            self.status.configure(text=f"完了・{len(saved)}ファイルを保存" if saved else "完了（保存なし）")
        elif kind == "error":
            _, msg, tb = ev
            self._set_text(f"!! {msg}\n")
            print(tb)
            self.run_btn.configure(state="normal")
            self.status.configure(text="失敗")

    def _copy(self) -> None:
        self.clipboard_clear()
        self.clipboard_append(self._md)
        self.status.configure(text="コピーしました")

    def _reveal(self) -> None:
        if self._saved:
            subprocess.run(["open", "-R", str(self._saved[0])])


def main() -> None:
    RunSummaryApp().mainloop()


if __name__ == "__main__":
    main()
