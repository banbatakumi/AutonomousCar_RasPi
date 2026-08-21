"""後始末の例外を「意図的に」捨てるための道具（2026-08-21 のレビュー 🟢10）。

## なぜ要るか

非テストコードに `except Exception: pass` が 17 箇所あった。読んでみると
**そのほとんどは `close()` 系の正当な後始末**で、握り潰して正しい。

問題は、**正当なものと危険なものが見分けられない**こと。

    except Exception:
        pass                # ← どちらの意味か読み手に分からない

`raspi/core/jpeg.py` が良いお手本になっている——`last_error` に理由を文字列で
残し、「`errors=448` だけ見えても『なぜ』が分からず時間を溶かす」とコメントが
ある。**この方針を GPIO とバスにも広げる**のがこのモジュール。

## 使い方

    from raspi.core.cleanup import quiet_close

    def close(self) -> None:
        with quiet_close("gpiozero DigitalOutputDevice"):
            self._dev.close()

`with` の名前そのものが「後始末なので捨ててよい」という主張になる。
捨てた例外は `recent_failures()` に残るので、**「なぜ」を後から引ける**。

## 使ってよい場所

**後始末だけ。** 具体的には `close()` / `stop()` / `__exit__` / `finally` の中で、
「失敗しても、この時点でできることはもう無い」場合に限る。

平常運転の経路には使わないこと。そこで例外が出るのは設計の想定外なので、
**握り潰さずに上へ投げるか、専用のカウンタを持たせて数える**のが正しい
（`RingJpeg.errors` / `TelemetryServer.bad_cmds` がその形）。
"""

from __future__ import annotations

import time
from collections import deque
from contextlib import contextmanager
from typing import Iterator

__all__ = ["quiet_close", "note_cleanup_failure", "recent_failures", "failure_count"]

#: 捨てた後始末の記録。**上限つきの環状バッファ。**
#: 長時間走行で無限に伸びるとそれ自体がメモリリークになる
_RECENT: deque[tuple[float, str, str]] = deque(maxlen=32)
_COUNT = 0


@contextmanager
def quiet_close(what: str) -> Iterator[None]:
    """後始末の例外を捨てる。**捨てた事実と理由は残す。**

    Args:
        what: 何を閉じようとしたか。`recent_failures()` にそのまま出るので、
            「どのファイルのどの資源か」が分かる粒度で書くこと
            （例: `"gpiozero DigitalOutputDevice"`, `"SerialLink の pyserial"`）。
    """
    global _COUNT
    try:
        yield
    except Exception as e:
        _COUNT += 1
        _RECENT.append((time.monotonic(), what, f"{type(e).__name__}: {e}"))


def note_cleanup_failure(what: str, exc: BaseException) -> None:
    """`quiet_close` を使えない形（`except` を型で分けている等）から記録だけする。

    `with` で囲めない場所——たとえば `BufferError` だけ別扱いにしたい
    `shm_ring.close()` ——のための入口。**捨てる判断はあちら側に残る。**
    """
    global _COUNT
    _COUNT += 1
    _RECENT.append((time.monotonic(), what, f"{type(exc).__name__}: {exc}"))


def recent_failures() -> list[tuple[float, str, str]]:
    """捨てた後始末の直近の記録 `(monotonic秒, 何を, 理由)`。

    終了時の統計や診断から呼ぶ。**空なのが正常**で、増えているなら
    「閉じられていない資源がある」ことを示す。
    """
    return list(_RECENT)


def failure_count() -> int:
    """捨てた後始末の総数。`recent_failures()` は直近ぶんしか持たない。"""
    return _COUNT
