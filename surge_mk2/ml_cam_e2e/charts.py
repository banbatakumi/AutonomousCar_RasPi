"""ml_cam_e2e/charts.py — 操作パネルの図（映像・時系列・ヒストグラム）。

`ml_common/train_graph.py` と同じく Tkinter の Canvas に手書きする
（matplotlib を埋め込むと起動が重くなり、再生の毎コマ更新にも向かない）。
座標の計算は Tk を知らない純粋関数に分けてあり、`tests/test_charts.py` で確かめる。
"""

from __future__ import annotations

import bisect
import math
import sys
import tkinter as tk
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))  # repo root

from raspi.core.cam_e2e_preproc import resize_area  # noqa: E402

__all__ = ["arrow_endpoint", "decimation_step", "gap_threshold", "polylines",
           "time_to_x", "x_to_time", "nearest_index", "runs", "photo_from_rgb",
           "FrameView", "SeriesCanvas", "HistogramCanvas"]

COLOR_TRUE = "#2a6fdb"        #: 人の操作（手本）
COLOR_PRED = "#e07b00"        #: モデルの予測
COLOR_EXCLUDED = "#f6c9c9"    #: 人が除外した区間
COLOR_DROPPED = "#e2e2e2"     #: 静止・後退で自動的に外れる区間
COLOR_VAL = "#fff2b3"         #: 検証データの区間


# ── 純粋関数 ──

def arrow_endpoint(origin: tuple[float, float], steer_rad: float, length: float,
                   *, gain: float = 3.0) -> tuple[float, float]:
    """矢印の先端座標。**`steer_rad` をそのまま角度に使うと振れ幅が小さすぎて
    見えないので `gain` 倍して誇張する**（目視確認用の表示で、実車の
    ステアリングジオメトリを表しているわけではない）。反時計回り正の `steer_rad`
    は画面上では左（x が小さい方）へ傾く。
    """
    ox, oy = origin
    angle = steer_rad * gain
    return ox - length * math.sin(angle), oy - length * math.cos(angle)


def decimation_step(n: int, plot_w: int) -> int:
    """`n` 点を幅 `plot_w` px に描くときの間引き幅（1px に2点まで）。"""
    return max(1, n // max(1, 2 * plot_w))


def gap_threshold(times: list[float]) -> float:
    """[s] これより間が空いたら線を切る。記録の途切れや除外で抜けた所を
    直線で結ぶと、そこを走っていたように見えてしまう。"""
    if len(times) < 3:
        return math.inf
    dts = sorted(b - a for a, b in zip(times, times[1:]))
    return max(0.5, 4.0 * dts[len(dts) // 2])


def time_to_x(t: float, t0: float, t1: float, x0: float, x1: float) -> float:
    span = t1 - t0
    return x0 if span <= 0 else x0 + (x1 - x0) * (t - t0) / span


def x_to_time(x: float, t0: float, t1: float, x0: float, x1: float) -> float:
    if x1 <= x0:
        return t0
    return t0 + (t1 - t0) * max(0.0, min(1.0, (x - x0) / (x1 - x0)))


def polylines(times: list[float], values: list[float], *, t0: float, t1: float,
              v_lo: float, v_hi: float, box: tuple[float, float, float, float],
              step: int = 1, gap_s: float = math.inf) -> list[list[float]]:
    """折れ線の座標列（`Canvas.create_line` にそのまま渡せる平らなリスト）の一覧。

    `gap_s` より間が空いた所と NaN で線を切る。`box` は `(x0, y0, x1, y1)`、
    値は `v_lo`〜`v_hi` に切って描く。
    """
    x0, y0, x1, y1 = box
    v_span = v_hi - v_lo
    out: list[list[float]] = []
    cur: list[float] = []
    prev_t: float | None = None
    for i in range(0, len(times), step):
        t, v = times[i], values[i]
        if math.isnan(v) or (prev_t is not None and t - prev_t > gap_s):
            if len(cur) >= 4:
                out.append(cur)
            cur = []
        if math.isnan(v):
            prev_t = None
            continue
        frac = 0.5 if v_span <= 0 else (max(v_lo, min(v_hi, v)) - v_lo) / v_span
        cur += [time_to_x(t, t0, t1, x0, x1), y1 - (y1 - y0) * frac]
        prev_t = t
    if len(cur) >= 4:
        out.append(cur)
    return out


def nearest_index(times: list[float], t: float) -> int:
    """`t` に最も近い要素の添字（`times` は昇順）。空なら 0。"""
    if not times:
        return 0
    i = bisect.bisect_left(times, t)
    cand = [j for j in (i - 1, i) if 0 <= j < len(times)]
    return min(cand, key=lambda j: abs(times[j] - t))


def runs(flags: list[bool]) -> list[tuple[int, int]]:
    """True が続く区間を `(開始添字, 終了添字)`（両端を含む）で返す。"""
    out: list[tuple[int, int]] = []
    start: int | None = None
    for i, f in enumerate(flags):
        if f and start is None:
            start = i
        elif not f and start is not None:
            out.append((start, i - 1))
            start = None
    if start is not None:
        out.append((start, len(flags) - 1))
    return out


def photo_from_rgb(rgb) -> tk.PhotoImage:
    """RGB の (H, W, 3) uint8 → `tk.PhotoImage`。

    Tk が直接読める PPM にする（PNG に圧縮してから渡すより一桁速く、
    再生の毎コマ更新に間に合う。Pillow の ImageTk にも頼らない）。
    """
    h, w = rgb.shape[:2]
    return tk.PhotoImage(data=b"P6 %d %d 255\n" % (w, h) + rgb.tobytes(), format="PPM")


# ── 映像 ──

class FrameView:
    """1コマの映像に、舵の矢印と数値を重ねて見せる。"""

    def __init__(self, parent: tk.Widget, *, width: int = 480, height: int = 270) -> None:
        self.width = width
        self.height = height
        self._canvas = tk.Canvas(parent, width=width, height=height, bg="#222",
                                 highlightthickness=0)
        self._photo: tk.PhotoImage | None = None      # 参照を持たないと GC で消える

    @property
    def widget(self) -> tk.Canvas:
        return self._canvas

    def clear(self, text: str = "") -> None:
        self._canvas.delete("all")
        self._photo = None
        if text:
            self._canvas.create_text(self.width / 2, self.height / 2, text=text, fill="#bbb")

    def show(self, rgb, *, arrows: list[tuple[float, str]] = (),
             lines: list[tuple[str, str]] = ()) -> None:
        """`arrows` は `(舵角[rad], 色)`、`lines` は左上に並べる `(文字列, 色)`。"""
        c = self._canvas
        c.delete("all")
        self._photo = photo_from_rgb(resize_area(rgb, self.width, self.height))
        c.create_image(0, 0, anchor="nw", image=self._photo)
        origin = (self.width / 2, self.height - 8)
        for steer, color in arrows:
            tip = arrow_endpoint(origin, steer, self.height * 0.45)
            c.create_line(*origin, *tip, fill="black", width=6, arrow="last")
            c.create_line(*origin, *tip, fill=color, width=3, arrow="last")
        for i, (text, color) in enumerate(lines):
            y = 12 + 18 * i
            c.create_text(9, y + 1, anchor="w", text=text, fill="black")
            c.create_text(8, y, anchor="w", text=text, fill=color)


# ── 時系列 ──

class SeriesCanvas:
    """時刻に対する折れ線。背景に区間（除外・検証データなど）を塗り、
    クリックした時刻を `on_seek` に渡す。"""

    PAD_L, PAD_R, PAD_T, PAD_B = 46, 8, 16, 16

    def __init__(self, parent: tk.Widget, *, width: int = 940, height: int = 120,
                 on_seek=None) -> None:
        self._canvas = tk.Canvas(parent, width=width, height=height, bg="white",
                                 highlightthickness=1, highlightbackground="#ccc")
        self._on_seek = on_seek
        self._times: list[float] = []
        self._series: list[tuple[str, str, list[float]]] = []
        self._spans: list[tuple[float, float, str]] = []
        self._v_lo, self._v_hi = -1.0, 1.0
        self._unit = ""
        self._cursor_t: float | None = None
        self._mark: tuple[float | None, float | None] = (None, None)
        self._canvas.bind("<Button-1>", self._click)
        self._canvas.bind("<B1-Motion>", self._click)

    @property
    def widget(self) -> tk.Canvas:
        return self._canvas

    def _box(self) -> tuple[float, float, float, float]:
        w, h = int(self._canvas["width"]), int(self._canvas["height"])
        return self.PAD_L, self.PAD_T, w - self.PAD_R, h - self.PAD_B

    def _t_range(self) -> tuple[float, float]:
        if not self._times:
            return 0.0, 1.0
        t0, t1 = self._times[0], self._times[-1]
        return (t0, t1) if t1 > t0 else (t0, t0 + 1.0)

    def set_data(self, times: list[float], series: list[tuple[str, str, list[float]]], *,
                 v_lo: float, v_hi: float, spans: list[tuple[float, float, str]] = (),
                 unit: str = "") -> None:
        """`series` は `(凡例, 色, 値の並び)`。`spans` は `(開始[s], 終了[s], 色)`。"""
        self._times = times
        self._series = series
        self._spans = list(spans)
        self._v_lo, self._v_hi = v_lo, v_hi
        self._unit = unit
        self._redraw()

    def set_spans(self, spans: list[tuple[float, float, str]]) -> None:
        self._spans = list(spans)
        self._redraw()

    def set_cursor(self, t: float | None) -> None:
        self._cursor_t = t
        self._draw_overlay()

    def set_mark(self, t_start: float | None, t_end: float | None) -> None:
        self._mark = (t_start, t_end)
        self._draw_overlay()

    def _click(self, event) -> None:
        if self._on_seek is None or not self._times:
            return
        x0, _y0, x1, _y1 = self._box()
        self._on_seek(x_to_time(event.x, *self._t_range(), x0, x1))

    def _redraw(self) -> None:
        c = self._canvas
        c.delete("all")
        x0, y0, x1, y1 = self._box()
        t0, t1 = self._t_range()
        for s0, s1, color in self._spans:
            xa = time_to_x(max(s0, t0), t0, t1, x0, x1)
            xb = time_to_x(min(s1, t1), t0, t1, x0, x1)
            c.create_rectangle(xa, y0, max(xb, xa + 1), y1, fill=color, outline="")
        c.create_rectangle(x0, y0, x1, y1, outline="#bbb")
        if self._v_lo < 0 < self._v_hi:
            yz = y1 - (y1 - y0) * (0 - self._v_lo) / (self._v_hi - self._v_lo)
            c.create_line(x0, yz, x1, yz, fill="#bbb", dash=(2, 3))
        c.create_text(x0 - 4, y0, anchor="ne", text=f"{self._v_hi:g}", fill="#666")
        c.create_text(x0 - 4, y1, anchor="se", text=f"{self._v_lo:g}", fill="#666")
        if self._unit:
            c.create_text(x0 - 4, (y0 + y1) / 2, anchor="e", text=self._unit, fill="#666")
        if self._times:
            c.create_text(x0, y1 + 2, anchor="nw", text="0s", fill="#666")
            c.create_text(x1, y1 + 2, anchor="ne", text=f"{t1 - t0:.0f}s", fill="#666")
            step = decimation_step(len(self._times), int(x1 - x0))
            gap = gap_threshold(self._times)
            for _label, color, values in self._series:
                for line in polylines(self._times, values, t0=t0, t1=t1, v_lo=self._v_lo,
                                      v_hi=self._v_hi, box=(x0, y0, x1, y1), step=step,
                                      gap_s=gap):
                    c.create_line(*line, fill=color, width=1.5)
        lx = x0 + 6
        for label, color, _values in self._series:
            item = c.create_text(lx, 2, anchor="nw", text=label, fill=color)
            lx = c.bbox(item)[2] + 12
        self._draw_overlay()

    def _draw_overlay(self) -> None:
        """カーソルと選択中の範囲だけを描き直す（再生中に毎コマ呼ばれるので軽く）。"""
        c = self._canvas
        c.delete("overlay")
        if not self._times:
            return
        x0, y0, x1, y1 = self._box()
        t0, t1 = self._t_range()
        ms, me = self._mark
        if ms is not None and me is not None:
            xa, xb = sorted((time_to_x(ms, t0, t1, x0, x1), time_to_x(me, t0, t1, x0, x1)))
            c.create_rectangle(xa, y0, max(xb, xa + 1), y1, outline="#c00", width=2,
                               tags="overlay")
        else:
            for t in (ms, me):
                if t is not None:
                    x = time_to_x(t, t0, t1, x0, x1)
                    c.create_line(x, y0, x, y1, fill="#c00", width=2, dash=(4, 2),
                                  tags="overlay")
        if self._cursor_t is not None:
            x = time_to_x(self._cursor_t, t0, t1, x0, x1)
            c.create_line(x, y0 - 4, x, y1 + 4, fill="black", width=1, tags="overlay")


# ── ヒストグラム ──

class HistogramCanvas:
    """区間ごとの枚数（灰色の棒）と、重み付け後の期待枚数（青い段線）。"""

    PAD_L, PAD_R, PAD_T, PAD_B = 10, 10, 20, 18

    def __init__(self, parent: tk.Widget, *, width: int = 460, height: int = 170) -> None:
        self._canvas = tk.Canvas(parent, width=width, height=height, bg="white",
                                 highlightthickness=1, highlightbackground="#ccc")

    @property
    def widget(self) -> tk.Canvas:
        return self._canvas

    def set(self, counts: list[int], *, lo_label: str, hi_label: str, title: str,
            overlay: list[float] | None = None, mid_label: str = "") -> None:
        c = self._canvas
        c.delete("all")
        w, h = int(c["width"]), int(c["height"])
        x0, y0, x1, y1 = self.PAD_L, self.PAD_T, w - self.PAD_R, h - self.PAD_B
        c.create_text(x0, 3, anchor="nw", text=title, fill="#333")
        c.create_text(x0, y1 + 2, anchor="nw", text=lo_label, fill="#666")
        c.create_text(x1, y1 + 2, anchor="ne", text=hi_label, fill="#666")
        if mid_label:
            c.create_text((x0 + x1) / 2, y1 + 2, anchor="n", text=mid_label, fill="#666")
        c.create_line(x0, y1, x1, y1, fill="#999")
        n = len(counts)
        peak = max([*counts, *(overlay or []), 1])
        if n == 0:
            return
        bar_w = (x1 - x0) / n
        for i, v in enumerate(counts):
            top = y1 - (y1 - y0) * v / peak
            c.create_rectangle(x0 + bar_w * i + 1, top, x0 + bar_w * (i + 1) - 1, y1,
                               fill="#b9b9b9", outline="")
        if overlay:
            pts: list[float] = []
            for i, v in enumerate(overlay):
                y = y1 - (y1 - y0) * v / peak
                pts += [x0 + bar_w * i, y, x0 + bar_w * (i + 1), y]
            c.create_line(*pts, fill=COLOR_TRUE, width=2)
        c.create_text(x1, 3, anchor="ne", text=f"最大 {max(counts)}枚", fill="#666")
