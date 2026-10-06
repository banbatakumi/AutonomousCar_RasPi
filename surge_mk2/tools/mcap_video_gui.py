"""記録のカメラ画像を動画にする GUI（Tkinter、Mac 側）。

    .venv/bin/python -m tools.mcap_video_gui
    （または surge_mk2 直下の launcher.command から起動）

`.mcap` を選んで「書き出す」を押すと、前後カメラそれぞれの `<入力>_front.mp4` /
`<入力>_rear.mp4` を作る。中身は `tools/mcap_video.py`（CLI と同じ）。

処理を別スレッドで回す理由と作りは `tools/slam_replay_gui.py` と同じ
（**Tk は主スレッド以外から触ると壊れる**ので、作業スレッドは `queue` に積むだけ）。
"""

from __future__ import annotations

import queue
import subprocess
import threading
import tkinter as tk
import traceback
from pathlib import Path
from tkinter import filedialog, messagebox, ttk

from tools.mcap_video import DEFAULT_FPS, default_out_path, export_videos

DEFAULT_DIR = Path.home() / "Downloads"

#: 画面に出すカメラ（役割名, 表示名）
CAMS = (("front", "前"), ("rear", "後"))
FPS_CHOICES = ("10", "15", "30", "60")


class McapVideoApp(tk.Tk):
    def __init__(self) -> None:
        super().__init__()
        self.title("動画の書き出し — 記録のカメラ画像を mp4 に")
        self.geometry("820x560")

        self.files: list[Path] = []
        self.outputs: list[Path] = []
        self.cam_vars = {cam: tk.BooleanVar(value=True) for cam, _ in CAMS}
        self.fps = tk.StringVar(value=str(int(DEFAULT_FPS)))
        self.out_dir = tk.StringVar(value="")         # 空なら入力と同じフォルダ
        self._q: queue.Queue = queue.Queue()
        self._worker: threading.Thread | None = None

        self._build()
        self.after(100, self._drain)

    # ── 画面 ──

    def _build(self) -> None:
        pad = {"padx": 8, "pady": 4}

        top = ttk.LabelFrame(self, text="入力（.mcap）")
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
        row = ttk.Frame(opt)
        row.pack(fill="x", padx=4, pady=2)
        ttk.Label(row, text="カメラ:").pack(side="left")
        for cam, label in CAMS:
            ttk.Checkbutton(row, text=label, variable=self.cam_vars[cam]).pack(side="left", padx=4)
        ttk.Label(row, text="fps:").pack(side="left", padx=(16, 0))
        ttk.Combobox(row, textvariable=self.fps, values=FPS_CHOICES, width=5,
                     state="readonly").pack(side="left", padx=4)
        ttk.Label(row, foreground="gray",
                  text="記録は間引かれているので、同じ画像を繰り返して実時間に合わせる"
                  ).pack(side="left", padx=8)
        row = ttk.Frame(opt)
        row.pack(fill="x", padx=4, pady=2)
        ttk.Label(row, text="出力先:").pack(side="left")
        self.out_label = ttk.Label(row, text="入力と同じフォルダ（<入力>_front.mp4 / _rear.mp4）")
        self.out_label.pack(side="left", padx=4)
        ttk.Button(row, text="変更…", command=self._choose_out_dir).pack(side="left")
        ttk.Button(row, text="元に戻す", command=self._reset_out_dir).pack(side="left", padx=2)

        run = ttk.Frame(self)
        run.pack(fill="x", **pad)
        self.run_btn = ttk.Button(run, text="書き出す", command=self._run)
        self.run_btn.pack(side="left")
        self.progress = ttk.Progressbar(run, length=320, mode="determinate", maximum=1.0)
        self.progress.pack(side="left", padx=8)
        self.status = ttk.Label(run, text="ファイルを追加してください")
        self.status.pack(side="left")

        res = ttk.LabelFrame(self, text="結果")
        res.pack(fill="both", expand=True, **pad)
        self.text = tk.Text(res, height=12, wrap="none", font=("Menlo", 11))
        self.text.pack(fill="both", expand=True, padx=4, pady=4)
        self.text.configure(state="disabled")

        after = ttk.Frame(self)
        after.pack(fill="x", **pad)
        self.play_btn = ttk.Button(after, text="動画を開く", state="disabled",
                                   command=self._open_last)
        self.play_btn.pack(side="left")
        self.finder_btn = ttk.Button(after, text="Finder で表示", state="disabled",
                                     command=self._reveal_last)
        self.finder_btn.pack(side="left", padx=4)

    def _log(self, s: str) -> None:
        self.text.configure(state="normal")
        self.text.insert("end", s + "\n")
        self.text.see("end")
        self.text.configure(state="disabled")

    # ── 入力の管理 ──

    def _add_files(self) -> None:
        paths = filedialog.askopenfilenames(
            title="記録を選ぶ", initialdir=str(DEFAULT_DIR) if DEFAULT_DIR.exists() else None,
            filetypes=[("MCAP", "*.mcap")])
        for p in map(Path, paths):
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
        self.out_label.configure(text="入力と同じフォルダ（<入力>_front.mp4 / _rear.mp4）")

    # ── 実行 ──

    def _run(self) -> None:
        if self._worker is not None and self._worker.is_alive():
            return
        if not self.files:
            messagebox.showinfo("入力がありません", "「ファイルを追加…」で記録を選んでください")
            return
        cams = [cam for cam, _ in CAMS if self.cam_vars[cam].get()]
        if not cams:
            messagebox.showinfo("カメラ未選択", "書き出すカメラを選んでください")
            return
        out_dir = Path(self.out_dir.get()) if self.out_dir.get() else None
        exists = [o for src in self.files for cam in cams
                  if (o := default_out_path(src, cam, out_dir)).exists()]
        if exists and not messagebox.askyesno(
                "上書きの確認",
                "次のファイルは既にあります。上書きしますか？\n\n"
                + "\n".join(f"・{o.name}" for o in exists[:8])
                + (f"\n…他{len(exists) - 8}本" if len(exists) > 8 else "")):
            return
        self.run_btn.configure(state="disabled")
        self.outputs.clear()
        self._worker = threading.Thread(
            target=self._work, args=(list(self.files), out_dir, cams, float(self.fps.get())),
            daemon=True)
        self._worker.start()

    def _work(self, files: list[Path], out_dir: Path | None, cams: list[str],
              fps: float) -> None:
        """作業スレッド。**Tk には触らず `_q` に積むだけ**（モジュール docstring）。"""
        for i, src in enumerate(files):
            self._q.put(("start", i, len(files), src))
            try:
                results = export_videos(src, out_dir, cams=cams, fps=fps,
                                        progress=lambda f: self._q.put(("progress", f)))
                self._q.put(("done", src, results))
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
            self.progress.configure(value=0.0)
        elif kind == "progress":
            self.progress.configure(value=min(1.0, max(0.0, ev[1])))
        elif kind == "done":
            _, src, results = ev
            if not results:
                self._log(f"!! {src.name}: 画像がありません（--image-hz 0 で記録した？）\n")
            for r in results:
                self.outputs.append(r.path)
                self._log(f"{r.path}\n    {r.images}枚 / {r.duration_s:.1f}s / {r.codec}")
            if results and results[0].truncated:
                self._log(f"!! {src.name}: 記録が途中で壊れています。読めた所までを書き出しました"
                          f"（{results[0].truncated}）")
            if results:
                self._log("")
        elif kind == "error":
            _, src, msg, tb = ev
            self._log(f"!! {src.name}: {msg}\n")
            print(tb)                                  # 詳細はターミナル側に残す
        elif kind == "finished":
            self.progress.configure(value=1.0 if self.outputs else 0.0)
            self.run_btn.configure(state="normal")
            ok = bool(self.outputs)
            self.play_btn.configure(state="normal" if ok else "disabled")
            self.finder_btn.configure(state="normal" if ok else "disabled")
            self.status.configure(text=f"完了（{len(self.outputs)}本）")

    def _open_last(self) -> None:
        if self.outputs:
            subprocess.run(["open", str(self.outputs[-1])])

    def _reveal_last(self) -> None:
        if self.outputs:
            subprocess.run(["open", "-R", *map(str, self.outputs)])


def main() -> None:
    McapVideoApp().mainloop()


if __name__ == "__main__":
    main()
