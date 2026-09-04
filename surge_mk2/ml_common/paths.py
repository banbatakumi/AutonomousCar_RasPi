"""リポジトリルートからの相対パス整形。"""

from __future__ import annotations

from pathlib import Path
from typing import Callable

__all__ = ["make_rel"]


def make_rel(repo_root: Path) -> Callable[[Path], str]:
    """`repo_root` からの相対パス文字列を返す関数を作る。画面表示を短くするためだけの整形。
    `repo_root` の外なら絶対パスのまま返す。"""

    def rel(p: Path) -> str:
        try:
            return str(p.relative_to(repo_root))
        except ValueError:
            return str(p)

    return rel
