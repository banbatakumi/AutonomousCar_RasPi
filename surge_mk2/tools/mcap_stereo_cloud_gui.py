"""記録の左右画像から視差の点群を作る GUI（Tkinter、Mac 側）。

    .venv/bin/python -m tools.mcap_stereo_cloud_gui
    （または surge_mk2 直下の launcher.command から起動）

`.mcap` を選んで「作る」を押すと、`/viz/stereo/points` を足した `<入力>_stereo.mcap` を作る。
Foxglove の 3D パネルで開いて見る。中身は `tools/mcap_stereo_cloud.py`（CLI と同じ）。

処理を別スレッドで回す理由と作りは `tools/mcap_video_gui.py` と同じ
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

from tools.mcap_stereo_cloud import add_stereo_cloud, default_out_path

DEFAULT_DIR = Path.home() / "Downloads"
EVERY_CHOICES = ("1", "2", "3", "5", "10")
STRIDE_CHOICES = ("1", "2", "3", "4")
RANGE_CHOICES = ("3", "5", "10", "20")
OUT_DEFAULT_TEXT = "入力と同じフォルダ（<入力>_stereo.mcap）"


class StereoCloudApp(tk.Tk):
    def __init__(self) -> None:
        super().__init__()
        self.title("視差の点群 — 記録の左右画像から Foxglove 用の 3D 点群を作る")
        self.geometry("820x560")

        self.files: list[Path] = []
        self.outputs: list[Path] = []
        self.every = tk.StringVar(value="1")
        self.stride = tk.StringVar(value="2")
        self.max_range = tk.StringVar(value="10")
        self.vehicle = tk.StringVar(value="")         # 空なら記録の控え（provenance）
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
        for label, var, choices in (("組の間引き（N組に1組）:", self.every, EVERY_CHOICES),
                                    ("点の間引き（N画素おき）:", self.stride, STRIDE_CHOICES),
                                    ("最大距離 [m]:", self.max_range, RANGE_CHOICES)):
            ttk.Label(row, text=label).pack(side="left", padx=(8, 0))
            ttk.Combobox(row, textvariable=var, values=choices, width=4,
                         state="readonly").pack(side="left", padx=4)
        ttk.Label(opt, foreground="gray", padding=(4, 0),
                  text="増やすほど軽い。点群は記録の画像（5Hz）に合わせて作る。Pi の照合とは完全には一致しない"
                  ).pack(fill="x")
        row = ttk.Frame(opt)
        row.pack(fill="x", padx=4, pady=2)
        ttk.Label(row, text="校正値:").pack(side="left")
        self.vehicle_label = ttk.Label(row, text="記録に入っていた値（無ければ config/vehicle.toml）")
        self.vehicle_label.pack(side="left", padx=4)
        ttk.Button(row, text="変更…", command=self._choose_vehicle).pack(side="left")
        ttk.Button(row, text="元に戻す", command=self._reset_vehicle).pack(side="left", padx=2)
        row = ttk.Frame(opt)
        row.pack(fill="x", padx=4, pady=2)
        ttk.Label(row, text="出力先:").pack(side="left")
        self.out_label = ttk.Label(row, text=OUT_DEFAULT_TEXT)
        self.out_label.pack(side="left", padx=4)
        ttk.Button(row, text="変更…", command=self._choose_out_dir).pack(side="left")
        ttk.Button(row, text="元に戻す", command=self._reset_out_dir).pack(side="left", padx=2)

        run = ttk.Frame(self)
        run.pack(fill="x", **pad)
        self.run_btn = ttk.Button(run, text="作る", command=self._run)
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
        self.open_btn = ttk.Button(after, text="Foxglove で開く", state="disabled", command=self._open_last)
        self.open_btn.pack(side="left")
        self.finder_btn = ttk.Button(after, text="Finder で表示", state="disabled", command=self._reveal_last)
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
        self.out_label.configure(text=OUT_DEFAULT_TEXT)

    def _choose_vehicle(self) -> None:
        p = filedialog.askopenfilename(title="vehicle.toml", filetypes=[("TOML", "*.toml")])
        if p:
            self.vehicle.set(p)
            self.vehicle_label.configure(text=p)

    def _reset_vehicle(self) -> None:
        self.vehicle.set("")
        self.vehicle_label.configure(text="記録に入っていた値（無ければ config/vehicle.toml）")

    # ── 実行 ──

    def _run(self) -> None:
        if self._worker is not None and self._worker.is_alive():
            return
        if not self.files:
            messagebox.showinfo("入力がありません", "「ファイルを追加…」で記録を選んでください")
            return
        out_dir = Path(self.out_dir.get()) if self.out_dir.get() else None
        exists = [o for src in self.files if (o := default_out_path(src, out_dir)).exists()]
        if exists and not messagebox.askyesno(
                "上書きの確認",
                "次のファイルは既にあります。上書きしますか？\n\n"
                + "\n".join(f"・{o.name}" for o in exists[:8])
                + (f"\n…他{len(exists) - 8}本" if len(exists) > 8 else "")):
            return
        if any(o.resolve() == s.resolve() for s in self.files for o in [default_out_path(s, out_dir)]):
            messagebox.showerror("出力先が入力と同じ", "出力先を変えてください")
            return
        self.run_btn.configure(state="disabled")
        self.outputs.clear()
        opts = {"every": int(self.every.get()), "stride": int(self.stride.get()),
                "max_range": float(self.max_range.get()),
                "vehicle_toml": Path(self.vehicle.get()) if self.vehicle.get() else None}
        self._worker = threading.Thread(target=self._work, args=(list(self.files), out_dir, opts), daemon=True)
        self._worker.start()

    def _work(self, files: list[Path], out_dir: Path | None, opts: dict) -> None:
        """作業スレッド。**Tk には触らず `_q` に積むだけ**（モジュール docstring）。"""
        for i, src in enumerate(files):
            self._q.put(("start", i, len(files), src))
            out = default_out_path(src, out_dir)
            try:
                n, origin = add_stereo_cloud(src, out, progress=lambda f: self._q.put(("progress", f)), **opts)
                self._q.put(("done", src, out, n, origin))
            except Exception as e:  # noqa: BLE001 — 1本の失敗で残りを止めない
                self._q.put(("error", src, f"{type(e).__name__}: {e}", traceback.format_exc()))
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
            self.status.configure(text=f"{i + 1}/{n}  {src.name}（画像を読んでいます…）")
            self.progress.configure(value=0.0)
        elif kind == "progress":
            self.progress.configure(value=min(1.0, max(0.0, ev[1])))
        elif kind == "done":
            _, src, out, n, origin = ev
            if n == 0:
                self._log(f"!! {src.name}: 左右の組がありません（同期していない・--image-hz 0 で記録した？）\n")
                out.unlink(missing_ok=True)
            else:
                self.outputs.append(out)
                self._log(f"{out}\n    点群 {n} 枚 / 校正値: {origin}\n")
        elif kind == "error":
            _, src, msg, tb = ev
            self._log(f"!! {src.name}: {msg}\n")
            print(tb)                                  # 詳細はターミナル側に残す
        elif kind == "finished":
            self.progress.configure(value=1.0 if self.outputs else 0.0)
            self.run_btn.configure(state="normal")
            state = "normal" if self.outputs else "disabled"
            self.open_btn.configure(state=state)
            self.finder_btn.configure(state=state)
            self.status.configure(text=f"完了（{len(self.outputs)}本）")

    def _open_last(self) -> None:
        if self.outputs:
            subprocess.run(["open", str(self.outputs[-1])], check=False)      # 既定のアプリ（Foxglove）で開く

    def _reveal_last(self) -> None:
        if self.outputs:
            subprocess.run(["open", "-R", *map(str, self.outputs)], check=False)


def main() -> None:
    StereoCloudApp().mainloop()


if __name__ == "__main__":
    main()
