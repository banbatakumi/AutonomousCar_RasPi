"""ノードの `main()` の定型（シグナルで止める・終わったら必ず閉じる）。

SIGINT だけでなく **SIGTERM も拾う**（`systemctl stop`・`deploy.sh --restart` は SIGTERM を送る。拾わないと
即死して `finally` が走らず、記録の close・終了時の DISARM・統計が失われる）。
"""

from __future__ import annotations

import signal
from typing import Callable, Iterable

from .cleanup import quiet_close

__all__ = ["run_node"]


def run_node(node, run: Callable[[], object], *, close: Iterable[object] = ()) -> None:
    """SIGINT/SIGTERM で `node.stop()` を呼ぶようにしてから `run()` を回し、終わったら
    `node.close()`（あれば）と `close` の各 `.close()` を順に呼ぶ（後始末の例外は記録して続ける）。"""
    def _shutdown(*_):
        node.stop()

    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, _shutdown)
    try:
        run()
    finally:
        for obj in (node, *close):
            if obj is not None and hasattr(obj, "close"):
                with quiet_close(f"{type(obj).__name__} の後始末"):
                    obj.close()
