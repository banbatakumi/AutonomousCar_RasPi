"""`train.py`の1エポック分の出力行パース。

例: "epoch   3/30  loss=0.1234  val_iou=0.567  (12s)"
    "epoch   3/30  loss=0.1234  val_mae=0.045  (2.7deg, 12s)"

対象メトリクス名（`val_iou`・`val_mae`等）だけがアプリごとに違うので、
`make_epoch_line_parser(metric_name)` でパーサーを組み立てる。
"""

from __future__ import annotations

import re
from typing import Callable

__all__ = ["make_epoch_line_parser"]


def make_epoch_line_parser(
    metric_name: str,
) -> Callable[[str], tuple[int, float, float | None] | None]:
    """"epoch N/M  loss=X.XXXX  <metric_name>=X.XXX" 形式の行から
    `(epoch, loss, metric)` を取り出すパーサーを組み立てる。
    metric値が "nan"（検証データが無いとき）なら `None`。該当しない行なら `None` を返す。"""
    pattern = re.compile(
        rf"epoch\s+(\d+)/\d+\s+loss=([\d.]+)\s+{re.escape(metric_name)}=([\d.]+|nan)")

    def parse(line: str) -> tuple[int, float, float | None] | None:
        m = pattern.search(line)
        if not m:
            return None
        epoch = int(m.group(1))
        loss = float(m.group(2))
        metric_s = m.group(3)
        metric = None if metric_s == "nan" else float(metric_s)
        return epoch, loss, metric

    return parse
