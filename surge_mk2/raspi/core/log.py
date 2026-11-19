"""全ノード共通のログ（`logging`）。

以前は各ノードが `print(..., flush=True)` で書き、行頭の `# `（情報）・`!! `（警告）で重さを表していた。
それだとレベルで絞れず、時刻も付かない（camera_node は stdout を包んで時刻を足していた）。

    from raspi.core.log import get_logger, setup_logging
    log = get_logger("camera")          # モジュールの先頭で
    ...
    def main():
        setup_logging("camera")         # main の最初に1回

- 出力は stderr に `2026-11-18 12:34:56 W camera: 本文`（journald 下でもそのまま読める）
- レベルは環境変数 `SURGE_LOG_LEVEL`（`DEBUG`・`INFO`・`WARNING`。既定 `INFO`）
- **書き出しの失敗で落ちない。** `logging` は書けなかった行を捨てて続ける（io_node が stdout の
  `OSError` で落ちて E-Stop がラッチした件。`io_node._QuietStream`）
- `setup_logging` を呼ばないテストでは、警告以上だけが stderr に出る（`logging` の既定）

終了時の統計のような「人が読む報告」は `print` のままでよい（ログではなく出力）。
"""

from __future__ import annotations

import logging
import os
import sys

__all__ = ["get_logger", "setup_logging", "ENV_LEVEL"]

ENV_LEVEL = "SURGE_LOG_LEVEL"
_ROOT = "surge"
_FORMAT = "%(asctime)s %(levelname).1s %(name_short)s: %(message)s"
_DATEFMT = "%Y-%m-%d %H:%M:%S"


class _ShortName(logging.Filter):
    """`surge.camera` → `camera`（行を短くする）。"""

    def filter(self, record: logging.LogRecord) -> bool:
        record.name_short = record.name.removeprefix(_ROOT + ".")
        return True


def get_logger(node: str) -> logging.Logger:
    """ノード（または部品）のロガー。名前は `surge.<node>`。"""
    return logging.getLogger(f"{_ROOT}.{node}")


def setup_logging(node: str = "", *, level: str | int | None = None) -> logging.Logger:
    """プロセスに1回。`surge.*` のログを stderr へ出す。何度呼んでも重ねて出さない。"""
    root = logging.getLogger(_ROOT)
    lv = level if level is not None else os.environ.get(ENV_LEVEL, "INFO")
    root.setLevel(lv.upper() if isinstance(lv, str) else lv)
    if not any(getattr(h, "_surge", False) for h in root.handlers):
        h = logging.StreamHandler(sys.stderr)
        h._surge = True                       # type: ignore[attr-defined]
        h.setFormatter(logging.Formatter(_FORMAT, _DATEFMT))
        h.addFilter(_ShortName())
        root.addHandler(h)
        root.propagate = False
    return get_logger(node) if node else root
