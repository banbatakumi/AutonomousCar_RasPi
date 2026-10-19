"""車両の校正の GUI（Tkinter、Mac 側）。設計は `docs/calibration.md`。

    .venv/bin/python -m tools.calib.gui       （launcher の「車両の校正」）

GUI の「システム同定」タブで録った mcap を開くと、記録の planner から校正の種類を判別して解析し、所見・合否・
書き込む値（今の `config/vehicle.toml` の値と並べる）・グラフを出す。値を選んで「適用」で toml に書く。
書いた後は `./tools/deploy.sh --restart-io` で Pi へ（`[control]` は接続のたびに STM32 へ送られる）。
"""

from __future__ import annotations

import queue
import threading
import tkinter as tk
from tkinter import filedialog, messagebox, ttk

from . import CALIBRATIONS, analyze_file, toml_io

_MARK = {"ok": "✓", "info": "・", "warn": "★", "fail": "✗"}
_COLOR = {"合格": "#1a7f37", "要確認": "#9a6700", "不合格": "#cf222e", "": "#57606a"}


class CalibApp(tk.Tk):
    def __init__(self) -> None:
        super().__init__()
        self.title("車両の校正")
        self.geometry("1280x820")
        self._results: dict[int, tuple[str, object]] = {}     # 校正の添字 → (記録のパス, Result)
        self._checked: dict[str, bool] = {}
        self._q: queue.Queue = queue.Queue()
        self._build()
        self.after(100, self._poll)

    # ── 画面 ────────────────────────────────────────────────────────────

    def _build(self) -> None:
        left = ttk.Frame(self, padding=8)
        left.pack(side=tk.LEFT, fill=tk.Y)
        ttk.Button(left, text="記録を開く…（複数可）", command=self._open).pack(fill=tk.X)
        ttk.Label(left, text="順番の目安: 1 → 2・3・4 → 5 測る → 適用 → 5 確かめる\n"
                             "→ 6（台上）→ 7 → 適用 → 8", justify=tk.LEFT, foreground="#57606a").pack(anchor="w", pady=6)
        self.tree = ttk.Treeview(left, columns=("state", "file"), show="tree headings", height=12)
        self.tree.heading("#0", text="校正")
        self.tree.heading("state", text="結果")
        self.tree.heading("file", text="記録")
        self.tree.column("#0", width=230)
        self.tree.column("state", width=70)
        self.tree.column("file", width=200)
        for i, c in enumerate(CALIBRATIONS):
            self.tree.insert("", tk.END, iid=str(i), text=f"{c.order}  {c.title}", values=("未解析", ""))
        self.tree.pack(fill=tk.Y, expand=True)
        self.tree.bind("<<TreeviewSelect>>", lambda e: self._show())
        self.status = ttk.Label(left, text="", foreground="#57606a", wraplength=480)
        self.status.pack(fill=tk.X, pady=4)

        right = ttk.Frame(self, padding=8)
        right.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        self.verdict = tk.Label(right, text="校正を選ぶか、記録を開く", font=("", 15, "bold"), anchor="w")
        self.verdict.pack(fill=tk.X)
        self.text = tk.Text(right, height=14, wrap=tk.WORD, font=("Menlo", 11))
        self.text.pack(fill=tk.X)
        for lv, col in (("ok", "#1a7f37"), ("warn", "#9a6700"), ("fail", "#cf222e"), ("info", "#24292f")):
            self.text.tag_configure(lv, foreground=col)

        vals = ttk.Frame(right)
        vals.pack(fill=tk.BOTH, expand=True, pady=6)
        self.values = ttk.Treeview(vals, columns=("use", "cur", "new", "diff", "note"), show="headings", height=9)
        for col, txt, w in (("use", "適用", 44), ("cur", "今の値", 110), ("new", "新しい値", 110),
                            ("diff", "差", 80), ("note", "説明", 420)):
            self.values.heading(col, text=txt)
            self.values.column(col, width=w, anchor="w")
        self.values.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        self.values.bind("<Button-1>", self._toggle)
        bar = ttk.Frame(right)
        bar.pack(fill=tk.X)
        ttk.Button(bar, text="選んだ値を vehicle.toml に適用", command=self._apply).pack(side=tk.RIGHT)
        self.plot_sel = ttk.Combobox(bar, state="readonly", width=60)
        self.plot_sel.pack(side=tk.LEFT)
        self.plot_sel.bind("<<ComboboxSelected>>", lambda e: self._draw())

        try:
            from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
            from matplotlib.figure import Figure
            import matplotlib
            matplotlib.rcParams["font.family"] = ["Hiragino Sans", "Hiragino Maru Gothic Pro", "sans-serif"]
            self.fig = Figure(figsize=(8, 3.0), dpi=90)
            self.ax = self.fig.add_subplot(111)
            self.canvas = FigureCanvasTkAgg(self.fig, master=right)
            self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)
        except Exception:                              # noqa: BLE001  グラフ無しでも使える
            self.fig = None

    # ── 解析 ────────────────────────────────────────────────────────────

    def _open(self) -> None:
        paths = filedialog.askopenfilenames(title="校正の試験の記録", filetypes=[("mcap", "*.mcap"), ("すべて", "*")])
        if not paths:
            return
        self.status.configure(text=f"{len(paths)} 本を解析中…")
        threading.Thread(target=self._work, args=(list(paths),), daemon=True).start()

    def _work(self, paths: list[str]) -> None:
        for p in paths:
            try:
                cal, res = analyze_file(p)
                self._q.put(("ok", p, cal, res))
            except Exception as e:                     # noqa: BLE001
                self._q.put(("err", p, None, e))
        self._q.put(("done", None, None, None))

    def _poll(self) -> None:
        try:
            while True:
                kind, p, cal, res = self._q.get_nowait()
                if kind == "ok":
                    i = CALIBRATIONS.index(cal)
                    self._results[i] = (p, res)
                    self.tree.item(str(i), values=(res.verdict, p.rsplit("/", 1)[-1]))
                    self.tree.selection_set(str(i))
                    self._show()
                elif kind == "err":
                    messagebox.showerror("解析できない", f"{p}\n\n{res}")
                else:
                    self.status.configure(text="解析が終わった")
        except queue.Empty:
            pass
        self.after(100, self._poll)

    # ── 表示 ────────────────────────────────────────────────────────────

    def _current(self):
        sel = self.tree.selection()
        if not sel:
            return None, None
        i = int(sel[0])
        return i, self._results.get(i)

    def _show(self) -> None:
        i, got = self._current()
        self.text.delete("1.0", tk.END)
        for r in self.values.get_children():
            self.values.delete(r)
        if i is None:
            return
        cal = CALIBRATIONS[i]
        if got is None:
            self.verdict.configure(text=f"{cal.order} {cal.title}: 未解析", fg=_COLOR[""])
            self.text.insert(tk.END, f"記録（planner: {', '.join(cal.planners)}）を開くと解析する。"
                                     + (f"\n先に適用しておく校正: {cal.needs}" if cal.needs else ""))
            return
        path, res = got
        self.verdict.configure(text=f"{cal.order} {cal.title}: {res.verdict}", fg=_COLOR[res.verdict])
        self.text.insert(tk.END, f"{path}\n", "info")
        if cal.needs:
            self.text.insert(tk.END, f"（この解析は {cal.needs} の値を使う）\n", "info")
        for f in res.findings:
            self.text.insert(tk.END, f"{_MARK[f.level]} {f.text}\n", f.level)
        cur = toml_io.read_flat()
        for k, v in res.values.items():
            old = cur.get(k)
            key = f"{i}:{k}"
            self._checked.setdefault(key, res.usable)
            diff = "" if old is None else (f"{(v - old) / abs(old) * 100:+.1f}%" if abs(old) > 1e-12 else f"{v - old:+.4g}")
            self.values.insert("", tk.END, iid=key, values=("☑" if self._checked[key] else "☐",
                                                             "（無し）" if old is None else f"{old:.6g}", f"{v:.6g}", diff,
                                                             f"{k}   {res.notes.get(k, '')}"))
        self.plot_sel.configure(values=list(res.series))
        if res.series:
            self.plot_sel.current(0)
        self._draw()

    def _draw(self) -> None:
        if self.fig is None:
            return
        self.ax.clear()
        i, got = self._current()
        name = self.plot_sel.get()
        if got and name in got[1].series:
            for label, x, y in got[1].series[name]:
                style = "o" if label.startswith("実測") and len(x) < 200 else "-"
                self.ax.plot(x, y, style, label=label, markersize=4)
            self.ax.set_title(name, fontsize=10)
            self.ax.grid(True, alpha=0.3)
            self.ax.legend(fontsize=8)
        self.fig.tight_layout()
        self.canvas.draw()

    def _toggle(self, ev) -> None:
        row = self.values.identify_row(ev.y)
        if not row or self.values.identify_column(ev.x) != "#1":
            return
        self._checked[row] = not self._checked.get(row, False)
        vals = list(self.values.item(row, "values"))
        vals[0] = "☑" if self._checked[row] else "☐"
        self.values.item(row, values=vals)

    def _apply(self) -> None:
        i, got = self._current()
        if got is None:
            return
        path, res = got
        if not res.usable and not messagebox.askyesno("不合格の記録", "この記録は不合格。それでも選んだ値を書くか？"):
            return
        pick = {k: v for k, v in res.values.items() if self._checked.get(f"{i}:{k}")}
        if not pick:
            messagebox.showinfo("適用", "選んだ値が無い")
            return
        changed = toml_io.apply(pick)
        messagebox.showinfo("適用した", ("\n".join(changed) or "（変わった値は無い）")
                            + "\n\nPi へは ./tools/deploy.sh --restart-io（GUI で「制御パラメータ: 適用済み ✓」を確かめる）")
        self._show()


def main() -> None:
    CalibApp().mainloop()


if __name__ == "__main__":
    main()
