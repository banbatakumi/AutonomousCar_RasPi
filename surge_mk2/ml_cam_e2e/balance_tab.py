"""ml_cam_e2e/balance_tab.py — 「偏り」タブ。学習に使うデータが舵・速度のどこに偏っているかを見る。

模倣学習の典型的な失敗は「直進ばかりのデータで学習して、カーブで切らない
モデルになる」こと。学習の前にここで分布を見て、

- 直進が大半なら「直進過多の補正」を上げる（学習タブと同じ値を共有する）
- 左右の枚数が大きく違うなら、逆回りの走行を録り足す
- 速度が1点に固まっているなら、速度の学習は意味を持たない（常にその値を出すだけ）

を判断する。数えるのは**実際に学習に使われるコマだけ**（除外・静止・後退を除く）。
"""

from __future__ import annotations

import math
import tkinter as tk
from pathlib import Path
from tkinter import ttk
from typing import Callable

import samples as S
from charts import HistogramCanvas

__all__ = ["N_STEER_BINS", "N_SPEED_BINS", "BalanceTab"]

#: `samples.balance_weights` の既定と同じ区切りで見せる（見ている棒＝重みの単位）
N_STEER_BINS = 15
N_SPEED_BINS = 12

_COLUMNS = (("source", "記録", 260), ("total", "抽出", 60), ("excluded", "除外", 60),
            ("dropped", "静止/後退", 80), ("used", "使用", 60), ("duration", "時間", 70),
            ("straight", "直進", 60), ("lr", "左 / 右", 90))


class BalanceTab:
    def __init__(self, parent: tk.Widget, *, get_frames_dir: Callable[[], Path | None],
                 max_steer: float, balance_var: tk.DoubleVar) -> None:
        self.frame = ttk.Frame(parent, padding=8)
        self._get_frames_dir = get_frames_dir
        self._max_steer = max_steer
        self.balance_var = balance_var
        self._steer_counts: list[int] = []
        self._build()
        balance_var.trace_add("write", lambda *_a: self._draw_steer())

    def _build(self) -> None:
        top = ttk.Frame(self.frame)
        top.pack(fill="x")
        ttk.Button(top, text="集計を更新", command=self.reload).pack(side="left")
        self.summary_var = tk.StringVar(value="")
        ttk.Label(top, textvariable=self.summary_var).pack(side="left", padx=(12, 0))

        charts = ttk.Frame(self.frame)
        charts.pack(fill="x", pady=(8, 0))
        self.steer_hist = HistogramCanvas(charts)
        self.steer_hist.widget.pack(side="left")
        self.speed_hist = HistogramCanvas(charts)
        self.speed_hist.widget.pack(side="left", padx=(12, 0))

        row = ttk.Frame(self.frame)
        row.pack(fill="x", pady=(8, 0))
        ttk.Label(row, text="直進過多の補正:").pack(side="left")
        ttk.Scale(row, from_=0.0, to=1.0, variable=self.balance_var, length=220).pack(
            side="left", padx=6)
        self.balance_label = ttk.Label(row, text="")
        self.balance_label.pack(side="left")
        ttk.Label(self.frame, foreground="gray", wraplength=940, justify="left",
                  text="灰色の棒＝実際の枚数、青い線＝補正後に学習で選ばれる枚数の見込み。"
                       "0 は補正なし、1 は舵の全区間を同じ回数だけ見せる（少ないカーブを"
                       "繰り返すので覚え込みやすい）。学習タブと同じ値です").pack(anchor="w")

        table = ttk.Frame(self.frame)
        table.pack(fill="both", expand=True, pady=(8, 0))
        self.tree = ttk.Treeview(table, columns=[c[0] for c in _COLUMNS], show="headings",
                                 height=7)
        for key, title, width in _COLUMNS:
            self.tree.heading(key, text=title)
            self.tree.column(key, width=width, anchor="w" if key == "source" else "e")
        scroll = ttk.Scrollbar(table, orient="vertical", command=self.tree.yview)
        self.tree.configure(yscrollcommand=scroll.set)
        self.tree.pack(side="left", fill="both", expand=True)
        scroll.pack(side="left", fill="y")

    def reload(self) -> None:
        frames_dir = self._get_frames_dir()
        samples = S.load_manifest(frames_dir) if frames_dir else []
        exclusions = S.load_exclusions(frames_dir) if samples else []
        used = [s for s in samples if S.usable(s, exclusions)]

        self.tree.delete(*self.tree.get_children())
        for r in S.per_record_stats(samples, exclusions, self._max_steer):
            self.tree.insert("", "end", values=(
                r["source_mcap"], r["total"], r["excluded"], r["dropped"], r["used"],
                f"{r['duration_s']:.0f}s", f"{r['straight_ratio'] * 100:.0f}%",
                f"{r['left']} / {r['right']}"))

        norms = [S.steer_norm(s, self._max_steer) for s in used]
        self._steer_counts = S.histogram(norms, -1.0, 1.0, N_STEER_BINS)
        self._draw_steer()

        speed_ref = S.auto_speed_ref(used) if used else 1.0
        speeds = [S.speed_label(s) for s in used]
        self.speed_hist.set(S.histogram(speeds, 0.0, speed_ref, N_SPEED_BINS),
                            lo_label="0", hi_label=f"{speed_ref:.2f} m/s",
                            title="速度指令（学習に使うコマ）")

        if not samples:
            self.summary_var.set("ペアがありません（①で抽出してください）")
            return
        left = sum(1 for v in norms if v > S.STRAIGHT_NORM)
        right = sum(1 for v in norms if v < -S.STRAIGHT_NORM)
        straight = len(used) - left - right
        ratio = straight / len(used) * 100 if used else 0.0
        self.summary_var.set(
            f"抽出 {len(samples)}枚 → 学習に使う {len(used)}枚　"
            f"直進 {ratio:.0f}%・左 {left}・右 {right}　"
            f"速度の基準（最高速度）{speed_ref:.2f} m/s")

    def _draw_steer(self) -> None:
        try:
            alpha = max(0.0, min(1.0, float(self.balance_var.get())))
        except (tk.TclError, ValueError):
            return
        self.balance_label.config(text=f"{alpha:.2f}")
        deg = math.degrees(self._max_steer)
        overlay = S.balanced_histogram(self._steer_counts, alpha) if alpha > 0 else None
        self.steer_hist.set(self._steer_counts, lo_label=f"右 {deg:.0f}°", mid_label="直進",
                            hi_label=f"左 {deg:.0f}°", title="舵指令（学習に使うコマ）",
                            overlay=overlay)
