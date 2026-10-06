"""ml_cam_e2e/review_tab.py — 「確認・選別」タブ。抽出したペアを映像で見て、手本にしない区間を除外する。

模倣学習は手本をそのまま真似る。コースアウトして戻した所、壁に当たった所、
やり直しのために止まって下がった所が混ざっていると、それも「正しい運転」
として覚える。ここで映像と操作を見ながら範囲で外す。

除外は `frames/exclusions.json` に時刻の範囲で保存する（`samples.py` 参照）。
画像は消さないので、取り消せばいつでも戻る。

    ←/→     1枚送り（Shift で10枚）
    space   再生/停止
    [ / ]   除外したい範囲の開始/終了
    x       その範囲を除外
"""

from __future__ import annotations

import math
import time
import tkinter as tk
from pathlib import Path
from tkinter import messagebox, ttk
from typing import Callable

import samples as S
from charts import (
    COLOR_DROPPED,
    COLOR_EXCLUDED,
    COLOR_TRUE,
    FrameView,
    SeriesCanvas,
    nearest_index,
    runs,
)

__all__ = ["record_key", "record_label", "status_of", "ReviewTab"]

NS = 1_000_000_000
_SPEED_COLOR = "#1a8f3c"


def record_key(sample: S.Sample) -> tuple[str, str]:
    return sample.source_mcap, sample.cam


def record_label(key: tuple[str, str]) -> str:
    source, cam = key
    return source if cam == "front" else f"{source} [{cam}]"


def status_of(sample: S.Sample, exclusions: list[S.Exclusion]) -> str:
    """このコマが学習に使われるか。使われないなら理由。"""
    if S.is_excluded(sample, exclusions):
        return "除外"
    return S.drop_reason(sample) or "使用"


class ReviewTab:
    def __init__(self, parent: tk.Widget, *, get_frames_dir: Callable[[], Path | None],
                 max_steer: float, log: Callable[[str], None]) -> None:
        self.frame = ttk.Frame(parent, padding=8)
        self._get_frames_dir = get_frames_dir
        self._max_steer = max_steer
        self._log = log

        self._frames_dir: Path | None = None
        self._all: list[S.Sample] = []
        self._exclusions: list[S.Exclusion] = []
        self._keys: list[tuple[str, str]] = []
        self._cur: list[S.Sample] = []          # 選択中の記録のコマ（時刻順）
        self._times: list[float] = []           # 記録の先頭からの秒
        self._idx = 0
        self._mark_start: int | None = None
        self._mark_end: int | None = None
        self._playing = False
        self._play_origin = (0.0, 0.0)          # (再生を始めた実時刻, そのときの記録内時刻)

        self._build()

    # ── 画面 ──

    def _build(self) -> None:
        top = ttk.Frame(self.frame)
        top.pack(fill="x")
        ttk.Label(top, text="記録:").pack(side="left")
        self.record_var = tk.StringVar()
        self.record_combo = ttk.Combobox(top, textvariable=self.record_var, state="readonly",
                                         width=44)
        self.record_combo.pack(side="left", padx=(4, 8))
        self.record_combo.bind("<<ComboboxSelected>>", lambda _e: self._select_record())
        ttk.Button(top, text="読み込み直す", command=self.reload).pack(side="left")
        self.summary_var = tk.StringVar(value="①でペアを抽出すると、ここで映像を確認できます")
        ttk.Label(top, textvariable=self.summary_var, foreground="gray").pack(
            side="left", padx=(12, 0))

        mid = ttk.Frame(self.frame)
        mid.pack(fill="x", pady=(6, 0))
        left = ttk.Frame(mid)
        left.pack(side="left", anchor="n")
        self.view = FrameView(left)
        self.view.widget.pack()

        ctrl = ttk.Frame(left)
        ctrl.pack(fill="x", pady=(4, 0))
        ttk.Button(ctrl, text="◀", width=3, command=lambda: self.step(-1)).pack(side="left")
        self.play_btn = ttk.Button(ctrl, text="▶ 再生", width=8, command=self.toggle_play)
        self.play_btn.pack(side="left", padx=2)
        ttk.Button(ctrl, text="▶", width=3, command=lambda: self.step(1)).pack(side="left")
        self.rate_var = tk.StringVar(value="1x")
        ttk.Combobox(ctrl, textvariable=self.rate_var, values=["1x", "2x", "4x", "8x"],
                     state="readonly", width=4).pack(side="left", padx=(8, 0))
        self.pos_var = tk.StringVar(value="")
        ttk.Label(ctrl, textvariable=self.pos_var).pack(side="left", padx=(10, 0))

        right = ttk.LabelFrame(mid, text="手本にしない区間")
        right.pack(side="left", fill="both", expand=True, padx=(10, 0))
        row = ttk.Frame(right)
        row.pack(fill="x", padx=6, pady=(6, 0))
        ttk.Button(row, text="[ 開始", command=self.mark_start).pack(side="left")
        ttk.Button(row, text="] 終了", command=self.mark_end).pack(side="left", padx=4)
        ttk.Button(row, text="選択を消す", command=self.clear_mark).pack(side="left")
        self.mark_var = tk.StringVar(value="範囲: 未選択")
        ttk.Label(right, textvariable=self.mark_var).pack(anchor="w", padx=6, pady=(4, 0))
        row = ttk.Frame(right)
        row.pack(fill="x", padx=6, pady=(4, 0))
        ttk.Label(row, text="メモ:").pack(side="left")
        self.note_var = tk.StringVar()
        ttk.Entry(row, textvariable=self.note_var).pack(side="left", fill="x", expand=True,
                                                         padx=(4, 0))
        ttk.Button(right, text="この範囲を除外 (x)", command=self.exclude_mark).pack(
            anchor="w", padx=6, pady=(6, 0))

        self.ex_list = tk.Listbox(right, height=6, exportselection=False)
        self.ex_list.pack(fill="both", expand=True, padx=6, pady=(8, 0))
        self.ex_list.bind("<<ListboxSelect>>", lambda _e: self._jump_to_selected_exclusion())
        ttk.Button(right, text="選んだ除外を取り消す", command=self.remove_exclusion).pack(
            anchor="w", padx=6, pady=6)

        ttk.Label(self.frame, foreground="gray",
                  text="←/→ 1枚送り（Shiftで10枚）・space 再生/停止・[ ] 範囲・x 除外　　"
                       "赤=除外した区間　灰=静止/後退で自動的に外れる区間").pack(
            anchor="w", pady=(6, 0))
        self.steer_plot = SeriesCanvas(self.frame, height=110, on_seek=self.seek_time)
        self.steer_plot.widget.pack(pady=(2, 0))
        self.speed_plot = SeriesCanvas(self.frame, height=80, on_seek=self.seek_time)
        self.speed_plot.widget.pack(pady=(4, 0))

    # ── 読み込み ──

    def reload(self) -> None:
        """manifest と除外を読み直す。タブを開いたとき・モデル名が変わったときに呼ぶ。"""
        self.stop()
        keep = self.record_var.get()
        # 同じ記録が残るなら、見ていたコマへ戻す（他のタブを見て戻ってきたときに
        # 先頭へ飛ばされると、選別の続きができない）
        keep_t = self._cur[self._idx].t_ns if self._cur else None
        self._frames_dir = self._get_frames_dir()
        self._all = S.load_manifest(self._frames_dir) if self._frames_dir else []
        self._exclusions = S.load_exclusions(self._frames_dir) if self._all else []
        self._keys = list(dict.fromkeys(record_key(s) for s in self._all))
        labels = [record_label(k) for k in self._keys]
        self.record_combo["values"] = labels
        self.record_var.set(keep if keep in labels else (labels[0] if labels else ""))
        self._select_record()
        if keep_t is not None and keep in labels:
            self.seek_index(nearest_index([s.t_ns for s in self._cur], keep_t))

    def _select_record(self) -> None:
        self.stop()
        labels = [record_label(k) for k in self._keys]
        label = self.record_var.get()
        key = self._keys[labels.index(label)] if label in labels else None
        self._cur = [s for s in self._all if record_key(s) == key]
        t0 = self._cur[0].t_ns if self._cur else 0
        self._times = [(s.t_ns - t0) / NS for s in self._cur]
        self._idx = 0
        self._mark_start = self._mark_end = None
        self._redraw_plots()
        self._refresh_exclusion_list()
        self._show()

    def open_at(self, source_mcap: str, cam: str, t_ns: int) -> None:
        """評価タブの「選別タブで開く」から呼ばれる。その記録のその時刻へ移る。"""
        self.reload()
        key = (source_mcap, cam)
        if key not in self._keys:
            return
        self.record_var.set(record_label(key))
        self._select_record()
        self.seek_index(nearest_index([s.t_ns for s in self._cur], t_ns))

    # ── 描画 ──

    def _spans(self) -> list[tuple[float, float, str]]:
        statuses = [status_of(s, self._exclusions) for s in self._cur]
        spans = []
        for flags, color in (([st in ("静止", "後退") for st in statuses], COLOR_DROPPED),
                             ([st == "除外" for st in statuses], COLOR_EXCLUDED)):
            spans += [(self._times[a], self._times[b], color) for a, b in runs(flags)]
        return spans

    def _redraw_plots(self) -> None:
        max_deg = math.degrees(self._max_steer)
        speed_hi = max([1.0] + [max(s.target_speed, 0.0 if math.isnan(s.speed_actual)
                                    else s.speed_actual) for s in self._cur])
        spans = self._spans() if self._cur else []
        self.steer_plot.set_data(
            self._times, [("舵指令", COLOR_TRUE, [math.degrees(s.target_steer)
                                                for s in self._cur])],
            v_lo=-round(max_deg), v_hi=round(max_deg), spans=spans, unit="deg")
        self.speed_plot.set_data(
            self._times, [("速度指令", _SPEED_COLOR, [s.target_speed for s in self._cur]),
                          ("実速度", "#999", [s.speed_actual for s in self._cur])],
            v_lo=0.0, v_hi=math.ceil(speed_hi * 2) / 2, spans=spans, unit="m/s")
        self._update_summary()

    def _update_summary(self) -> None:
        if not self._all:
            self.summary_var.set("ペアが0枚です。①で抽出してください（抽出しても0枚なら、"
                                 "①のログに落ちた理由が出ています）")
            return
        statuses = [status_of(s, self._exclusions) for s in self._cur]
        n_ex = statuses.count("除外")
        n_drop = sum(1 for st in statuses if st in ("静止", "後退"))
        self.summary_var.set(f"この記録 {len(self._cur)}枚: 使用 {statuses.count('使用')}・"
                             f"除外 {n_ex}・静止/後退 {n_drop}")

    def _show(self) -> None:
        if not self._cur:
            self.view.clear("ペアがありません")
            self.pos_var.set("")
            self.steer_plot.set_cursor(None)
            self.speed_plot.set_cursor(None)
            return
        s = self._cur[self._idx]
        status = status_of(s, self._exclusions)
        try:
            rgb = S.load_rgb(s.path)
        except Exception as e:                              # noqa: BLE001
            self.view.clear(f"画像を読めない: {e}")
        else:
            actual = "" if math.isnan(s.speed_actual) else f"（実 {s.speed_actual:.2f}）"
            self.view.show(rgb, arrows=[(s.target_steer, COLOR_TRUE)], lines=[
                (f"舵 {math.degrees(s.target_steer):+.1f}°", "white"),
                (f"速度 {s.target_speed:.2f} m/s{actual}" + ("  ブレーキ" if s.brake else ""),
                 "white"),
                (status, "#7CFC00" if status == "使用" else "#ff6b6b"),
            ])
        t = self._times[self._idx]
        self.pos_var.set(f"{self._idx + 1}/{len(self._cur)}  {t:.1f}s")
        self.steer_plot.set_cursor(t)
        self.speed_plot.set_cursor(t)

    def _update_mark(self) -> None:
        a, b = self._mark_start, self._mark_end
        ta = None if a is None else self._times[a]
        tb = None if b is None else self._times[b]
        for plot in (self.steer_plot, self.speed_plot):
            plot.set_mark(ta, tb)
        if a is None and b is None:
            self.mark_var.set("範囲: 未選択")
        elif a is None or b is None:
            t = ta if ta is not None else tb
            self.mark_var.set(f"範囲: {'開始' if a is not None else '終了'} {t:.1f}s"
                              f"（もう片方を指定）")
        else:
            lo, hi = sorted((a, b))
            self.mark_var.set(f"範囲: {self._times[lo]:.1f}s 〜 {self._times[hi]:.1f}s"
                              f"（{hi - lo + 1}枚）")

    # ── 移動・再生 ──

    def seek_index(self, idx: int) -> None:
        if not self._cur:
            return
        self._idx = max(0, min(len(self._cur) - 1, idx))
        self._show()

    def seek_time(self, t: float) -> None:
        self.stop()
        self.seek_index(nearest_index(self._times, t))

    def step(self, delta: int) -> None:
        self.stop()
        self.seek_index(self._idx + delta)

    def toggle_play(self) -> None:
        if self._playing:
            self.stop()
        elif self._cur:
            if self._idx >= len(self._cur) - 1:
                self._idx = 0
            self._playing = True
            self._play_origin = (time.monotonic(), self._times[self._idx])
            self.play_btn.config(text="■ 停止")
            self._tick()

    def stop(self) -> None:
        self._playing = False
        self.play_btn.config(text="▶ 再生")

    def _tick(self) -> None:
        if not self._playing:
            return
        # 実時間に合わせて進める。描画が間に合わなければコマを飛ばす
        # （遅れを溜めると、倍速にしても速くならない）
        rate = float(self.rate_var.get().rstrip("x"))
        t_wall, t_rec = self._play_origin
        t_now = t_rec + (time.monotonic() - t_wall) * rate
        idx = max(self._idx, nearest_index(self._times, t_now))
        if idx != self._idx:
            self.seek_index(idx)
        if self._idx >= len(self._cur) - 1:
            self.stop()
            return
        self.frame.after(15, self._tick)

    # ── 除外 ──

    def mark_start(self) -> None:
        if self._cur:
            self._mark_start = self._idx
            self._update_mark()

    def mark_end(self) -> None:
        if self._cur:
            self._mark_end = self._idx
            self._update_mark()

    def clear_mark(self) -> None:
        self._mark_start = self._mark_end = None
        self._update_mark()

    def exclude_mark(self) -> None:
        if self._mark_start is None or self._mark_end is None:
            messagebox.showinfo("範囲が未選択", "「[ 開始」と「] 終了」で範囲を決めてください")
            return
        lo, hi = sorted((self._mark_start, self._mark_end))
        source, cam = record_key(self._cur[lo])
        self._exclusions.append(S.Exclusion(source, cam, self._cur[lo].t_ns,
                                            self._cur[hi].t_ns, self.note_var.get().strip()))
        self._save()
        self._log(f"\n除外を追加: {source} {self._times[lo]:.1f}s〜{self._times[hi]:.1f}s"
                  f"（{hi - lo + 1}枚）\n")
        self.note_var.set("")
        self.clear_mark()
        self._after_exclusions_changed()

    def _current_exclusions(self) -> list[S.Exclusion]:
        if not self._cur:
            return []
        source, cam = record_key(self._cur[0])
        return sorted((e for e in self._exclusions if (e.source_mcap, e.cam) == (source, cam)),
                      key=lambda e: e.t_start_ns)

    def remove_exclusion(self) -> None:
        sel = self.ex_list.curselection()
        if not sel:
            return
        target = self._current_exclusions()[sel[0]]
        self._exclusions.remove(target)
        self._save()
        self._log(f"\n除外を取り消し: {target.source_mcap}\n")
        self._after_exclusions_changed()

    def _jump_to_selected_exclusion(self) -> None:
        sel = self.ex_list.curselection()
        if sel:
            self.stop()
            ex = self._current_exclusions()[sel[0]]
            self.seek_index(nearest_index([s.t_ns for s in self._cur], ex.t_start_ns))

    def _save(self) -> None:
        if self._frames_dir is not None:
            S.save_exclusions(self._frames_dir, self._exclusions)

    def _after_exclusions_changed(self) -> None:
        spans = self._spans()
        self.steer_plot.set_spans(spans)
        self.speed_plot.set_spans(spans)
        self._refresh_exclusion_list()
        self._update_summary()
        self._show()

    def _refresh_exclusion_list(self) -> None:
        self.ex_list.delete(0, "end")
        if not self._cur:
            return
        t0 = self._cur[0].t_ns
        for e in self._current_exclusions():
            n = sum(1 for s in self._cur if e.t_start_ns <= s.t_ns <= e.t_end_ns)
            note = f"  {e.note}" if e.note else ""
            self.ex_list.insert("end", f"{(e.t_start_ns - t0) / NS:6.1f}s 〜 "
                                       f"{(e.t_end_ns - t0) / NS:6.1f}s  {n}枚{note}")

    # ── キー操作（`app.py` が、このタブが表に出ているときだけ渡す） ──

    def handle_key(self, event) -> bool:
        big = 10 if event.state & 0x1 else 1          # Shift
        actions = {
            "Left": lambda: self.step(-big), "Right": lambda: self.step(big),
            "space": self.toggle_play, "bracketleft": self.mark_start,
            "bracketright": self.mark_end, "x": self.exclude_mark,
        }
        action = actions.get(event.keysym)
        if action is None:
            return False
        action()
        return True
