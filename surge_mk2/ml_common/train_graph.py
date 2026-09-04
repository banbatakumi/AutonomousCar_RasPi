"""学習曲線（loss・任意の第2指標）をTkinter Canvasに手書きで描く。

`ml_cam`/`ml_cam_e2e` のみ使用する。`ml_lidar` はTensorBoardで代替しており、
インスタンス化しなければ何のコストも掛からない。
"""

from __future__ import annotations

import tkinter as tk

__all__ = ["TrainCurveCanvas"]


class TrainCurveCanvas:
    """loss(赤)と任意の第2指標(青)を描く学習曲線Canvas。"""

    def __init__(self, parent: tk.Widget, *, metric_label: str,
                 width: int = 520, height: int = 150) -> None:
        self.metric_label = metric_label
        self._canvas = tk.Canvas(parent, width=width, height=height, bg="white",
                                 highlightthickness=0)
        self._epochs: list[int] = []
        self._losses: list[float] = []
        self._metrics: list[float | None] = []

    @property
    def widget(self) -> tk.Canvas:
        """pack/gridはApp側が行う。"""
        return self._canvas

    def reset(self) -> None:
        """エポック履歴をクリアして再描画する（学習開始時に呼ぶ）。"""
        self._epochs = []
        self._losses = []
        self._metrics = []
        self._redraw()

    def add_point(self, epoch: int, loss: float, metric: float | None) -> None:
        """1エポック分の値を追加して再描画する。"""
        self._epochs.append(epoch)
        self._losses.append(loss)
        self._metrics.append(metric)
        self._redraw()

    def _redraw(self) -> None:
        c = self._canvas
        c.delete("all")
        n = len(self._epochs)
        if n == 0:
            return
        width = int(c["width"])
        height = int(c["height"])
        pad = 24
        plot_w = max(width - 2 * pad, 1)
        plot_h = max(height - 2 * pad, 1)

        def points(values: list[float]) -> list[tuple[float, float]]:
            vmin, vmax = min(values), max(values)
            span = vmax - vmin
            pts = []
            for i, v in enumerate(values):
                x = pad + plot_w * i / max(n - 1, 1)
                y = pad + plot_h * (1 - (v - vmin) / span) if span > 0 else pad + plot_h / 2
                pts.append((x, y))
            return pts

        loss_pts = points(self._losses)
        for (x1, y1), (x2, y2) in zip(loss_pts, loss_pts[1:]):
            c.create_line(x1, y1, x2, y2, fill="#d33", width=2)
        c.create_text(pad, 10, anchor="w", text=f"loss: {self._losses[-1]:.4f}", fill="#d33")

        metric_values = [v for v in self._metrics if v is not None]
        if metric_values:
            metric_pts_all = [(i, v) for i, v in enumerate(self._metrics) if v is not None]
            vmin, vmax = min(metric_values), max(metric_values)
            span = vmax - vmin
            for (i1, v1), (i2, v2) in zip(metric_pts_all, metric_pts_all[1:]):
                x1 = pad + plot_w * i1 / max(n - 1, 1)
                x2 = pad + plot_w * i2 / max(n - 1, 1)
                y1 = pad + plot_h * (1 - (v1 - vmin) / span) if span > 0 else pad + plot_h / 2
                y2 = pad + plot_h * (1 - (v2 - vmin) / span) if span > 0 else pad + plot_h / 2
                c.create_line(x1, y1, x2, y2, fill="#37c", width=2)
            c.create_text(width - pad, 10, anchor="e",
                          text=f"{self.metric_label}: {metric_values[-1]:.3f}", fill="#37c")
