"""記録と再生。

- `framelog` … 生フレームログ `.sfl`。**stdlib のみ**で書けるので io_node の
  実時間ループから直接叩ける。UART を流れたバイトそのものが残る
- `mcap_log` … MCAP 書き出し。`mcap` に依存するので、**io_node には入れず**
  別プロセス（`logger_node`）とオフライン変換（`tools/sfl2mcap.py`）だけが使う

`mcap` が入っていない環境でも `from raspi.rec import FrameLogWriter` は通る。
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

try:                                        # mcap があるときだけ
    from .mcap_log import HAS_MCAP, McapLog, default_mcap_path  # noqa: F401
except ImportError:                         # pragma: no cover - 環境依存
    HAS_MCAP = False
else:
    __all__ += ["McapLog", "default_mcap_path"]

__all__ += ["HAS_MCAP"]
