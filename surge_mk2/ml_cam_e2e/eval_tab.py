"""ml_cam_e2e/eval_tab.py — 「評価」タブ。エクスポートしたモデルの予測を、手本と並べて見る。

`eval_model.py` をジョブとして走らせ、書かれた `eval.csv` を読んで表示する
（このタブ自体は推論しない）。見るのは3つ:

- **検証データでの誤差**（学習に使っていないコマ。これがモデルの実力）
- **時系列**（手本=青、予測=橙）。予測が手本より一貫して遅れているなら
  ①の「手本を遅らせる」を試す。カーブで予測が浅いなら実車の「舵の倍率」を上げる
- **誤差の大きい場面**。モデルが悪いのではなく手本が悪い（コースアウト等）
  ことが多い——そのまま「選別タブで開く」で除外できる
"""

from __future__ import annotations

import math
import tkinter as tk
from pathlib import Path
from tkinter import messagebox, ttk
from typing import Callable

import samples as S
from charts import (
    COLOR_DROPPED,
    COLOR_PRED,
    COLOR_TRUE,
    COLOR_VAL,
    FrameView,
    SeriesCanvas,
    nearest_index,
    runs,
)
from eval_model import SPLIT_TRAIN, SPLIT_UNUSED, SPLIT_VAL, read_eval, summarize, worst_segments

__all__ = ["EvalTab"]

NS = 1_000_000_000
_SPLIT_LABEL = {SPLIT_VAL: "検証", SPLIT_TRAIN: "学習", SPLIT_UNUSED: "不使用"}


class EvalTab:
    def __init__(self, parent: tk.Widget, *, get_frames_dir: Callable[[], Path | None],
                 get_run_dir: Callable[[], Path | None], models_dir: Path,
                 run_eval: Callable[[Path, Callable[[bool], None]], None],
                 open_in_review: Callable[[str, str, int], None]) -> None:
        self.frame = ttk.Frame(parent, padding=8)
        self._get_frames_dir = get_frames_dir
        self._get_run_dir = get_run_dir
        self._models_dir = models_dir
        self._run_eval = run_eval
        self._open_in_review = open_in_review

        self._rows: list[dict] = []
        self._keys: list[tuple[str, str]] = []
        self._cur: list[dict] = []
        self._times: list[float] = []
        self._idx = 0
        self._worst: list[dict] = []
        self._build()

    def _build(self) -> None:
        top = ttk.Frame(self.frame)
        top.pack(fill="x")
        ttk.Label(top, text="モデル:").pack(side="left")
        self.model_var = tk.StringVar()
        self.model_combo = ttk.Combobox(top, textvariable=self.model_var, state="readonly",
                                        width=18, postcommand=self.refresh_models)
        self.model_combo.pack(side="left", padx=(4, 8))
        ttk.Button(top, text="評価実行", command=self._start).pack(side="left")
        ttk.Button(top, text="前回の結果を読む", command=self.load_results).pack(
            side="left", padx=(6, 0))
        self.summary_var = tk.StringVar(value="⑤でエクスポートしたモデルを選んで「評価実行」")
        ttk.Label(top, textvariable=self.summary_var).pack(side="left", padx=(12, 0))

        mid = ttk.Frame(self.frame)
        mid.pack(fill="x", pady=(6, 0))
        left = ttk.Frame(mid)
        left.pack(side="left", anchor="n")
        self.view = FrameView(left)
        self.view.widget.pack()
        ctrl = ttk.Frame(left)
        ctrl.pack(fill="x", pady=(4, 0))
        ttk.Button(ctrl, text="◀", width=3, command=lambda: self.step(-1)).pack(side="left")
        ttk.Button(ctrl, text="▶", width=3, command=lambda: self.step(1)).pack(
            side="left", padx=2)
        self.pos_var = tk.StringVar(value="")
        ttk.Label(ctrl, textvariable=self.pos_var).pack(side="left", padx=(8, 0))
        ttk.Button(ctrl, text="選別タブで開く", command=self._to_review).pack(side="right")

        right = ttk.LabelFrame(mid, text="誤差の大きい場面（舵）")
        right.pack(side="left", fill="both", expand=True, padx=(10, 0))
        cols = (("source", "記録", 190), ("t", "時刻", 60), ("err", "誤差", 60),
                ("split", "区分", 50), ("n", "枚", 40))
        self.tree = ttk.Treeview(right, columns=[c[0] for c in cols], show="headings",
                                 height=10)
        for key, title, width in cols:
            self.tree.heading(key, text=title)
            self.tree.column(key, width=width, anchor="w" if key == "source" else "e")
        self.tree.pack(fill="both", expand=True, padx=6, pady=6)
        self.tree.bind("<<TreeviewSelect>>", lambda _e: self._jump_to_worst())

        row = ttk.Frame(self.frame)
        row.pack(fill="x", pady=(6, 0))
        ttk.Label(row, text="記録:").pack(side="left")
        self.record_var = tk.StringVar()
        self.record_combo = ttk.Combobox(row, textvariable=self.record_var, state="readonly",
                                         width=44)
        self.record_combo.pack(side="left", padx=(4, 8))
        self.record_combo.bind("<<ComboboxSelected>>", lambda _e: self._select_record())
        ttk.Label(row, foreground="gray",
                  text="青=手本　橙=予測　黄=検証データの区間　灰=学習に使っていない区間").pack(
            side="left")
        self.steer_plot = SeriesCanvas(self.frame, height=110, on_seek=self.seek_time)
        self.steer_plot.widget.pack(pady=(2, 0))
        self.speed_plot = SeriesCanvas(self.frame, height=80, on_seek=self.seek_time)
        self.speed_plot.widget.pack(pady=(4, 0))

    # ── モデルと実行 ──

    def refresh_models(self) -> None:
        names = (sorted(p.stem for p in self._models_dir.glob("*.onnx"))
                 if self._models_dir.is_dir() else [])
        self.model_combo["values"] = names

    def select_model(self, name: str) -> None:
        self.refresh_models()
        if name in self.model_combo["values"]:
            self.model_var.set(name)

    def _start(self) -> None:
        name = self.model_var.get().strip()
        if not name:
            messagebox.showwarning("未選択", "モデルを選んでください（⑤でエクスポートが必要です）")
            return
        self._run_eval(self._models_dir / f"{name}.onnx",
                       lambda ok: self.load_results() if ok else None)

    def _eval_path(self) -> Path | None:
        run_dir = self._get_run_dir()
        return run_dir / "eval.csv" if run_dir else None

    def load_results(self) -> None:
        path = self._eval_path()
        if path is None or not path.exists():
            self._rows = []
            self.summary_var.set("評価結果がまだありません（「評価実行」を押してください）")
        else:
            self._rows = read_eval(path)
            s = summarize(self._rows)
            v, t = s[SPLIT_VAL], s[SPLIT_TRAIN]
            self.summary_var.set(
                f"検証 {v['n']}枚: 舵 {v['steer_mae_deg']:.1f}°・速度 {v['speed_mae']:.2f}m/s　"
                f"｜ 学習 {t['n']}枚: 舵 {t['steer_mae_deg']:.1f}°・速度 {t['speed_mae']:.2f}m/s")
        self._keys = list(dict.fromkeys((r["source_mcap"], r["cam"]) for r in self._rows))
        labels = [self._label(k) for k in self._keys]
        self.record_combo["values"] = labels
        self.record_var.set(labels[0] if labels else "")

        self._worst = worst_segments(self._rows, k=30)
        self.tree.delete(*self.tree.get_children())
        t0_by_key = {}
        for r in self._rows:
            t0_by_key.setdefault((r["source_mcap"], r["cam"]), r["t_capture_ns"])
        for i, w in enumerate(self._worst):
            r = self._rows[w["index"]]
            t0 = t0_by_key[(r["source_mcap"], r["cam"])]
            self.tree.insert("", "end", iid=str(i), values=(
                w["source_mcap"], f"{(w['t_capture_ns'] - t0) / NS:.1f}s",
                f"{w['err_deg']:.1f}°", _SPLIT_LABEL[w["split"]], w["n_frames"]))
        self._select_record()

    # ── 表示 ──

    @staticmethod
    def _label(key: tuple[str, str]) -> str:
        return key[0] if key[1] == "front" else f"{key[0]} [{key[1]}]"

    def _select_record(self) -> None:
        labels = [self._label(k) for k in self._keys]
        label = self.record_var.get()
        key = self._keys[labels.index(label)] if label in labels else None
        self._cur = [r for r in self._rows if (r["source_mcap"], r["cam"]) == key]
        t0 = self._cur[0]["t_capture_ns"] if self._cur else 0
        self._times = [(r["t_capture_ns"] - t0) / NS for r in self._cur]
        self._idx = 0

        spans = []
        for split, color in ((SPLIT_UNUSED, COLOR_DROPPED), (SPLIT_VAL, COLOR_VAL)):
            flags = [r["split"] == split for r in self._cur]
            spans += [(self._times[a], self._times[b], color) for a, b in runs(flags)]
        steer_hi = max([10.0] + [abs(math.degrees(r[k])) for r in self._cur
                                 for k in ("steer_true", "steer_pred")])
        speed_hi = max([0.5] + [r[k] for r in self._cur for k in ("speed_true", "speed_pred")])
        steer_hi = math.ceil(steer_hi / 5) * 5
        self.steer_plot.set_data(self._times, [
            ("手本の舵", COLOR_TRUE, [math.degrees(r["steer_true"]) for r in self._cur]),
            ("予測", COLOR_PRED, [math.degrees(r["steer_pred"]) for r in self._cur]),
        ], v_lo=-steer_hi, v_hi=steer_hi, spans=spans, unit="deg")
        self.speed_plot.set_data(self._times, [
            ("手本の速度", COLOR_TRUE, [r["speed_true"] for r in self._cur]),
            ("予測", COLOR_PRED, [r["speed_pred"] for r in self._cur]),
        ], v_lo=0.0, v_hi=math.ceil(speed_hi * 2) / 2, spans=spans, unit="m/s")
        self._show()

    def _show(self) -> None:
        frames_dir = self._get_frames_dir()
        if not self._cur or frames_dir is None:
            self.view.clear("評価結果がありません")
            self.pos_var.set("")
            self.steer_plot.set_cursor(None)
            self.speed_plot.set_cursor(None)
            return
        r = self._cur[self._idx]
        err = math.degrees(r["steer_pred"] - r["steer_true"])
        try:
            rgb = S.load_rgb(frames_dir / r["file"])
        except Exception as e:                              # noqa: BLE001
            self.view.clear(f"画像を読めない: {e}")
        else:
            self.view.show(rgb, arrows=[(r["steer_true"], COLOR_TRUE),
                                        (r["steer_pred"], COLOR_PRED)], lines=[
                (f"手本 舵 {math.degrees(r['steer_true']):+.1f}°  "
                 f"速度 {r['speed_true']:.2f}", "#9cc4ff"),
                (f"予測 舵 {math.degrees(r['steer_pred']):+.1f}°  "
                 f"速度 {r['speed_pred']:.2f}", "#ffc27a"),
                (f"舵の差 {err:+.1f}°  [{_SPLIT_LABEL[r['split']]}]", "white"),
            ])
        t = self._times[self._idx]
        self.pos_var.set(f"{self._idx + 1}/{len(self._cur)}  {t:.1f}s")
        self.steer_plot.set_cursor(t)
        self.speed_plot.set_cursor(t)

    def seek_time(self, t: float) -> None:
        if self._cur:
            self._idx = nearest_index(self._times, t)
            self._show()

    def step(self, delta: int) -> None:
        if self._cur:
            self._idx = max(0, min(len(self._cur) - 1, self._idx + delta))
            self._show()

    def _jump_to_worst(self) -> None:
        sel = self.tree.selection()
        if not sel:
            return
        row = self._rows[self._worst[int(sel[0])]["index"]]
        self.record_var.set(self._label((row["source_mcap"], row["cam"])))
        self._select_record()
        self._idx = nearest_index([r["t_capture_ns"] for r in self._cur], row["t_capture_ns"])
        self._show()

    def _to_review(self) -> None:
        if self._cur:
            r = self._cur[self._idx]
            self._open_in_review(r["source_mcap"], r["cam"], r["t_capture_ns"])

    def handle_key(self, event) -> bool:
        big = 10 if event.state & 0x1 else 1          # Shift
        if event.keysym == "Left":
            self.step(-big)
        elif event.keysym == "Right":
            self.step(big)
        else:
            return False
        return True
