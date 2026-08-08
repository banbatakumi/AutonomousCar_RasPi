"""記録と再生。

生フレームログ `.sfl` が土台。MCAP へはここから変換する（実時間パスに
外部依存を持ち込まないため、MCAP 書き出しはオフラインのツール側に置く）。
"""

from .framelog import (
    FileHeader,
    FrameLogReader,
    FrameLogWriter,
    Kind,
    LogRecord,
    default_log_path,
)

__all__ = [
    "FileHeader",
    "FrameLogReader",
    "FrameLogWriter",
    "Kind",
    "LogRecord",
    "default_log_path",
]
