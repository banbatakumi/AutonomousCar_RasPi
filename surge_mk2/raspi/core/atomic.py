"""ファイルを原子的に置き換える（一時ファイルに書いて `os.replace()`）。

`write_bytes()` の直書きだと、書き込み中の電源断で中身が半端なファイルが残りうる。設定・総走行距離・地図は
バッテリーを抜いて電源を切られることが多いので、全部これで書く。

- 一時ファイルは**同じディレクトリ**に `mkstemp` で作る（別のファイルシステムだと `os.replace` が原子的でない。
  名前は `.<元の名前>-XXXX` なので、`*.npz` などの一覧には出ない。同時に2つ書いても衝突しない）
- 書けなかったら（満杯など）一時ファイルを消す。残すと失敗のたびに増えていく
- `fsync=True` なら差し替える前にディスクまで落とす。fsync しないと、差し替えだけがディスクに残って
  中身の無いファイルになりうる。走行中に何度も書く小さなファイルは `False`（周期を止めない）
"""

from __future__ import annotations

import os
import tempfile
from contextlib import contextmanager
from pathlib import Path
from typing import IO, Iterator

from .cleanup import quiet_close

__all__ = ["atomic_writer", "atomic_write_bytes", "atomic_write_text"]


@contextmanager
def atomic_writer(path: str | Path, mode: str = "wb", *, fsync: bool = False,
                  encoding: str | None = None) -> Iterator[IO]:
    """`with atomic_writer(p) as f: f.write(...)` で、ブロックを抜けたときに `p` を差し替える。

    ブロックの中で例外が出たら `p` は元のまま（一時ファイルは消す）。
    """
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    fd, tmp = tempfile.mkstemp(dir=str(path.parent), prefix=f".{path.name}-")
    try:
        with os.fdopen(fd, mode, encoding=encoding) as f:
            yield f
            if fsync:
                f.flush()
                os.fsync(f.fileno())
        os.replace(tmp, path)
    except BaseException:
        with quiet_close("原子的な書き込みの一時ファイルの削除"):
            os.unlink(tmp)
        raise


def atomic_write_bytes(path: str | Path, data: bytes, *, fsync: bool = False) -> None:
    with atomic_writer(path, "wb", fsync=fsync) as f:
        f.write(data)


def atomic_write_text(path: str | Path, text: str, *, fsync: bool = False) -> None:
    with atomic_writer(path, "w", fsync=fsync, encoding="utf-8") as f:
        f.write(text)
