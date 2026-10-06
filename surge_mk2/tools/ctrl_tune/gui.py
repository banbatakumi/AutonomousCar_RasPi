"""足回りの制御の調整 GUI（Tkinter、Mac側）。

    .venv/bin/python -m tools.ctrl_tune.gui

1. **車両モデルの同定**: GUI の「システム同定」タブで録った「制御の同定」2試験の mcap を開き、
   `fit.py` で解析して `config/vehicle.toml` の `[control.plant]` に書く
2. **パラメータの最適化**（TC・ABS。TV は対象外——`fit.py` 冒頭）: その車両モデルと、ホストでコンパイルした STM32 の制御を閉ループにして
   `optimize.py` で調整し、`[control]` に書く（Pi へ反映すると io_node が STM32 へ送る）

`tools/sysid/gui.py` と同じ作り（ファイル選択・結果の見比べ・適用の可否判断をターミナルなしで）。
"""

from __future__ import annotations

import queue
import threading
import tkinter as tk
import tomllib
from tkinter import filedialog, messagebox, ttk

from . import optimize, tuning
from .fit import TESTS, analyze
from .fw import FW_DIR, Firmware
from .plant import DEFAULT_TOML, Plant

def _current(toml_path: str, table: tuple[str, ...]) -> dict[str, float]:
    try:
        with open(toml_path, "rb") as f:
            d = tomllib.load(f)
    except (OSError, tomllib.TOMLDecodeError):
        return {}
    for name in table:
        d = d.get(name, {})
    return {k: float(v) for k, v in d.items() if isinstance(v, (int, float))}


class CtrlTuneApp(tk.Tk):
    def __init__(self) -> None:
        super().__init__()
        self.title("制御の調整 — vehicle.toml [control] 更新")
        self.geometry("900x820")
        self.toml_path = tk.StringVar(value=str(DEFAULT_TOML))
        self.file_vars: dict[str, tk.StringVar] = {}
        self.plant_results: dict[str, float] = {}
        self.plant_checks: dict[str, tk.BooleanVar] = {}
        self.tune_results: dict[str, float] = {}
        self.tune_checks: dict[str, tk.BooleanVar] = {}
        self._queue: queue.Queue = queue.Queue()
        self._busy = False
        self._build()
        self.after(200, self._poll)

    # ── 画面構築 ──

    def _build(self) -> None:
        top = ttk.Frame(self, padding=8)
        top.pack(fill="x")
        ttk.Label(top, text="vehicle.toml:").pack(side="left")
        ttk.Entry(top, textvariable=self.toml_path, width=60).pack(side="left", padx=4, fill="x", expand=True)
        ttk.Button(top, text="参照", command=self._pick_toml).pack(side="left")
        ttk.Label(self, text=f"ファーム: {FW_DIR}", foreground="gray", padding=(8, 0)).pack(anchor="w")

        tabs = ttk.Notebook(self)
        tabs.pack(fill="both", expand=True, padx=8, pady=8)
        self._build_fit(tabs)
        self._build_tune(tabs)

    def _build_fit(self, tabs: ttk.Notebook) -> None:
        tab = ttk.Frame(tabs, padding=8)
        tabs.add(tab, text="① 車両モデルの同定")
        for key, label, planner, _params in TESTS:
            row = ttk.Frame(tab, padding=(0, 3))
            row.pack(fill="x")
            ttk.Label(row, text=f"{label}（{planner}）", width=38).pack(side="left")
            var = tk.StringVar(value="（未選択）")
            self.file_vars[key] = var
            ttk.Label(row, textvariable=var, foreground="gray").pack(side="left", padx=4, fill="x", expand=True)
            ttk.Button(row, text="mcapを選ぶ", command=lambda k=key: self._pick_mcap(k)).pack(side="right")
        ttk.Button(tab, text="解析", command=self._analyze).pack(pady=6)
        ttk.Separator(tab).pack(fill="x")
        self.fit_frame = ttk.Frame(tab, padding=(0, 8))
        self.fit_frame.pack(fill="both", expand=True)
        ttk.Button(tab, text="[control.plant] に適用", command=self._apply_plant).pack(anchor="e")

    def _build_tune(self, tabs: ttk.Notebook) -> None:
        tab = ttk.Frame(tabs, padding=8)
        tabs.add(tab, text="② パラメータの最適化")
        row = ttk.Frame(tab)
        row.pack(fill="x")
        self.problem_vars: dict[str, tk.BooleanVar] = {}
        for key, make in optimize.PROBLEMS.items():
            var = tk.BooleanVar(value=True)
            self.problem_vars[key] = var
            ttk.Checkbutton(row, text=make().label, variable=var).pack(side="left", padx=(0, 12))
        row = ttk.Frame(tab, padding=(0, 6))
        row.pack(fill="x")
        # スリップ率の目標は最適化しない（前後の力と横グリップのどちらを取るかの方針。`optimize.py` 冒頭）。
        # 下の表を見て人が決め、最適化はその目標を保つゲインだけを決める
        current = _current(self.toml_path.get(), ("control",))
        self.targets: dict[str, tk.StringVar] = {}
        for key, label in (("tc_slip_target", "スリップ率の目標  TC:"), ("abs_slip_target", "ABS:")):
            ttk.Label(row, text=label).pack(side="left")
            var = tk.StringVar(value=f"{current.get(key, 0.1):g}")
            self.targets[key] = var
            ttk.Entry(row, textvariable=var, width=6).pack(side="left", padx=(4, 12))
        lo, hi = optimize.SLIP_TARGET_RANGE
        ttk.Label(row, text=f"（{lo:g}〜{hi:g}。小さいほど横グリップが残り、前後の力は減る。下の表）",
                  foreground="gray").pack(side="left")
        self.tradeoff_label = ttk.Label(tab, text="", foreground="gray", justify="left")
        self.tradeoff_label.pack(anchor="w", pady=(0, 6))
        self._show_tradeoff()
        row = ttk.Frame(tab)
        row.pack(fill="x")
        self.run_button = ttk.Button(row, text="最適化（数分）", command=self._optimise)
        self.run_button.pack(side="left")
        self.status = tk.StringVar(value="車両モデル: [control.plant]（無い項目は机上値）")
        ttk.Label(row, textvariable=self.status, foreground="gray").pack(side="left", padx=8)
        ttk.Separator(tab).pack(fill="x", pady=6)
        self.tune_frame = ttk.Frame(tab)
        self.tune_frame.pack(fill="both", expand=True)
        ttk.Button(tab, text="[control] に適用", command=self._apply_control).pack(anchor="e")

    def _show_tradeoff(self) -> None:
        try:
            rows = optimize.tradeoff(Plant.load(self.toml_path.get()))
        except Exception:  # noqa: BLE001 - toml が読めないときは表を出さないだけ
            return
        self.tradeoff_label.configure(text=(
            "同定したタイヤでの目安（スリップ率の目標 → 前後力 / 残る横グリップ）:  "
            + "   ".join(f"{t:g} → {f * 100:.0f}% / {lat * 100:.0f}%" for t, f, lat in rows)))

    def _pick_toml(self) -> None:
        p = filedialog.askopenfilename(title="vehicle.toml", filetypes=[("TOML", "*.toml")])
        if p:
            self.toml_path.set(p)

    def _pick_mcap(self, key: str) -> None:
        p = filedialog.askopenfilename(title="mcapファイル", filetypes=[("mcap", "*.mcap")])
        if p:
            self.file_vars[key].set(p)

    # ── ① 同定 ──

    def _analyze(self) -> None:
        try:
            base = Plant.load(self.toml_path.get())
        except (OSError, ValueError) as e:
            messagebox.showerror("vehicle.toml", str(e))
            return
        sources = {key: self.file_vars[key].get() for key, *_ in TESTS
                   if self.file_vars[key].get() not in ("", "（未選択）")}
        res = analyze(sources, base)
        if res.errors:
            messagebox.showerror("解析エラー", "\n".join(res.errors))
        self.plant_results = res.results
        for w in self.fit_frame.winfo_children():
            w.destroy()
        self.plant_checks.clear()
        if not res.results:
            ttk.Label(self.fit_frame, text="解析結果がありません（mcapを選んで「解析」を押してください）").grid()
            return
        current = _current(self.toml_path.get(), ("control", "plant"))
        nominal = Plant()
        for col, text in enumerate(("適用", "パラメータ", "現在値 → 測定値")):
            ttk.Label(self.fit_frame, text=text).grid(row=0, column=col, sticky="w")
        order = [k for _, _, _, params in TESTS for k in params if k in res.results]
        for r, key in enumerate(order, start=1):
            warned = res.warned.get(key)
            var = tk.BooleanVar(value=warned is None)
            self.plant_checks[key] = var
            ttk.Checkbutton(self.fit_frame, variable=var).grid(row=r, column=0)
            ttk.Label(self.fit_frame, text=key, width=22).grid(row=r, column=1, sticky="w")
            old = current.get(key)
            old_s = f"{old:.4g}" if old is not None else f"机上値 {getattr(nominal, key):.4g}"
            text = f"{old_s} → {res.results[key]:.4g}"
            if warned is not None:
                text += f"  ★{warned}に警告（既定で適用しない。所見を読んで判断）"
            ttk.Label(self.fit_frame, text=text, foreground="red" if warned else "black").grid(
                row=r, column=2, sticky="w")
        lines = [f"{label}: {n}" for label, ns in res.notes.items() for n in ns]
        box = tk.Text(self.fit_frame, height=min(14, len(lines) + 1), wrap="word")
        box.insert("1.0", "\n".join(lines))
        box.configure(state="disabled")
        box.grid(row=len(order) + 1, column=0, columnspan=3, sticky="we", pady=(8, 0))

    def _apply_plant(self) -> None:
        chosen = {k: v for k, v in self.plant_results.items()
                  if k in self.plant_checks and self.plant_checks[k].get()}
        if not chosen:
            messagebox.showinfo("適用", "先に解析し、適用する項目をチェックしてください")
            return
        try:
            changed = tuning.apply_plant(self.toml_path.get(), chosen)
        except Exception as e:  # noqa: BLE001
            messagebox.showerror("適用エラー", str(e))
            return
        messagebox.showinfo("適用しました", (f"更新: {', '.join(changed)}" if changed else "変更はありませんでした")
                            + "\n\n続けて「② パラメータの最適化」で、この車両モデルに合わせて調整してください。")

    # ── ② 最適化 ──

    def _optimise(self) -> None:
        if self._busy:
            return
        keys = [k for k, v in self.problem_vars.items() if v.get()]
        if not keys:
            messagebox.showinfo("最適化", "調整する制御を選んでください")
            return
        try:
            weights = optimize.Weights()
            plant = Plant.load(self.toml_path.get())
            fw = Firmware()
            start = tuning.load_params(fw, self.toml_path.get())
            lo, hi = optimize.SLIP_TARGET_RANGE
            for key, var in self.targets.items():
                start[key] = float(var.get())
                if not (lo <= start[key] <= hi):
                    raise ValueError(f"スリップ率の目標は {lo:g}〜{hi:g} にしてください（{key} = {start[key]:g}）")
            self._chosen_targets = {key: start[key] for key in self.targets}
            self._show_tradeoff()
        except Exception as e:  # noqa: BLE001
            messagebox.showerror("最適化", str(e))
            return
        self._busy = True
        self.run_button.configure(state="disabled")

        def work() -> None:
            try:
                out = []
                for key in keys:
                    problem = optimize.PROBLEMS[key]()
                    self._queue.put(("status", f"{problem.label}: 準備中…"))
                    out.append(optimize.optimise(
                        fw, plant, problem, start, weights,
                        progress=lambda g, c, label=problem.label: self._queue.put(
                            ("status", f"{label}: {g}世代目・コスト {c:.4f}"))))
                self._queue.put(("done", out))
            except Exception as e:  # noqa: BLE001
                self._queue.put(("error", str(e)))

        threading.Thread(target=work, daemon=True).start()

    def _poll(self) -> None:
        try:
            while True:
                kind, payload = self._queue.get_nowait()
                if kind == "status":
                    self.status.set(payload)
                elif kind == "error":
                    self._busy = False
                    self.run_button.configure(state="normal")
                    self.status.set("失敗")
                    messagebox.showerror("最適化", payload)
                else:
                    self._busy = False
                    self.run_button.configure(state="normal")
                    self.status.set("完了")
                    self._render_tuning(payload)
        except queue.Empty:
            pass
        self.after(200, self._poll)

    def _render_tuning(self, tunings: list[optimize.Tuning]) -> None:
        for w in self.tune_frame.winfo_children():
            w.destroy()
        self.tune_checks.clear()
        self.tune_results = {}
        for col, text in enumerate(("適用", "パラメータ", "現在値 → 調整後")):
            ttk.Label(self.tune_frame, text=text).grid(row=0, column=col, sticky="w")
        r = 1
        lines: list[str] = []
        # 人が選んだスリップ率の目標（その制御を最適化したときだけ。ゲインはこの目標に合わせてある）
        current = _current(self.toml_path.get(), ("control",))
        ran = {t.problem.key for t in tunings}
        for key, value in getattr(self, "_chosen_targets", {}).items():
            if key.split("_")[0] not in ran:
                continue
            self.tune_results[key] = value
            var = tk.BooleanVar(value=True)
            self.tune_checks[key] = var
            ttk.Checkbutton(self.tune_frame, variable=var).grid(row=r, column=0)
            ttk.Label(self.tune_frame, text=key, width=30).grid(row=r, column=1, sticky="w")
            old = current.get(key)
            ttk.Label(self.tune_frame, text=f"{old:.4g} → {value:.4g}  （選んだ値）" if old is not None
                      else f"{value:.4g}  （選んだ値）").grid(row=r, column=2, sticky="w")
            r += 1
        for t in tunings:
            better = t.cost < t.baseline_cost
            for key, value in t.values.items():
                at_bound = any(abs(value - b) <= 1e-6 * max(1.0, abs(b)) for b in t.problem.bounds[key])
                self.tune_results[key] = value
                var = tk.BooleanVar(value=better and key not in tuning.DERIVED)
                self.tune_checks[key] = var
                ttk.Checkbutton(self.tune_frame, variable=var).grid(row=r, column=0)
                ttk.Label(self.tune_frame, text=key, width=30).grid(row=r, column=1, sticky="w")
                text = f"{t.baseline[key]:.4g} → {value:.4g}"
                if at_bound and better:
                    text += "  ★探索の範囲の端（もっと外が良い＝コストが実情に合っていない可能性）"
                ttk.Label(self.tune_frame, text=text,
                          foreground="red" if at_bound and better else "black").grid(row=r, column=2, sticky="w")
                r += 1
            lines.append(f"■ {t.problem.label}: コスト {t.baseline_cost:.4f} → {t.cost:.4f}"
                         f"（{t.evaluations}回評価）" + ("" if better else "  今の値より良くならなかった"))
            for sc in t.problem.scenarios:
                a, b = t.table[sc.key]
                parts = []
                if "efficiency" in a:
                    parts.append(f"効率 {a['efficiency']:.2f}→{b['efficiency']:.2f}")
                parts.append(f"滑り {a['slipping']:.2f}→{b['slipping']:.2f}")
                parts.append(f"平均スリップ率 {a['slip_mean']:.2f}→{b['slip_mean']:.2f}")
                lines.append(f"  {sc.label}: " + "・".join(parts))
            worst = max(t.by_variant.items(), key=lambda kv: kv[1][1])
            lines.append(f"  いちばん悪い車: {worst[0]}（コスト {worst[1][0]:.3f}→{worst[1][1]:.3f}）")
        box = tk.Text(self.tune_frame, height=22, wrap="word")
        box.insert("1.0", "\n".join(lines))
        box.configure(state="disabled")
        box.grid(row=r, column=0, columnspan=3, sticky="we", pady=(8, 0))

    def _apply_control(self) -> None:
        chosen = {k: v for k, v in self.tune_results.items()
                  if k in self.tune_checks and self.tune_checks[k].get()}
        if not chosen:
            messagebox.showinfo("適用", "先に最適化し、適用する項目をチェックしてください")
            return
        try:
            changed = tuning.apply_control(self.toml_path.get(), chosen, Firmware())
        except Exception as e:  # noqa: BLE001
            messagebox.showerror("適用エラー", str(e))
            return
        messagebox.showinfo(
            "適用しました",
            (f"更新: {', '.join(changed)}" if changed else "変更はありませんでした")
            + "\n\nPi へ反映すると（tools/deploy.sh --restart-io）、io_node が STM32 へ送ります。"
              "入ったかどうかは GUI の診断（control_params_status = ok）で確かめられます。\n"
              "★実機で前後運動・旋回の試験を録り直し、効きを確かめてください。")


def main() -> None:
    CtrlTuneApp().mainloop()


if __name__ == "__main__":
    main()
