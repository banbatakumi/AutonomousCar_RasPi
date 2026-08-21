#!/usr/bin/env python3
"""文書に書かれたプロトコル版が `protocol.toml` と一致しているか確かめる。

    python3 config/check_docs.py            食い違いを表示（不一致なら終了コード 1）
    python3 config/check_docs.py --check    同じ（CI と揃えるための別名）

## なぜ要るか（2026-08-21 のレビュー 🟢13）

`docs/README.md` だけがプロトコル **v0.7** を掲げたまま 2 版遅れていた。
`protocol.toml` が `0x0009`、`uart_protocol.md` と `PROGRESS.md` は v0.9。
**同じ事実が複数箇所に手書きで散っている**ので、片方だけ腐っても誰も気づかない。

生成コードには `--check` があるのに、**文書の版番号だけ人間の記憶に頼っていた。**
`generate.py --check` の対象を「生成コード」から「文書中の版番号」まで広げる。

## 何をどこまで見るか

**版番号だけ。** 本文の中身が仕様と合っているかは機械には判定できない。
ここで潰したいのは「表紙の数字が古い」という、**読む人ほど間違える**形の腐り方。

`stm32_interface.md` の見出しが v0.7 のまま本文に v0.9 の仕様が混ざっていた
（同じレビューで発見）のが、まさにこの検査が拾うべきものだった。
"""

from __future__ import annotations

import argparse
import re
import sys
import tomllib
from pathlib import Path

CONFIG_DIR = Path(__file__).resolve().parent
REPO_ROOT = CONFIG_DIR.parent
TOML_PATH = REPO_ROOT / "raspi" / "proto" / "protocol.toml"
DOCS = REPO_ROOT / "docs"

#: 版番号を掲げている文書と、そこから版を拾う正規表現。
#:
#: **「どこかに v0.9 と書いてある」では駄目。** 改版履歴には過去の版が全部
#: 並んでいるので、拾うのは**その文書が自分の版として名乗っている箇所**に限る。
CLAIMS: list[tuple[str, str]] = [
    # 冒頭の「**バージョン**: v0.9」
    ("uart_protocol.md", r"\*\*バージョン\*\*:\s*v(\d+\.\d+)"),
    ("stm32_interface.md", r"\*\*バージョン\*\*:\s*v(\d+\.\d+)"),
    # 一覧表の「**UART プロトコル仕様**（**v0.9**）」と本文の「プロトコルは v0.9」
    ("README.md", r"\*\*UART プロトコル仕様\*\*（\*\*v(\d+\.\d+)\*\*）"),
    ("README.md", r"\*\*プロトコルは v(\d+\.\d+)\*\*"),
]


def expected_version() -> str:
    """`protocol.toml` の `protocol_version`（`0x0009` → `"0.9"`）。"""
    with TOML_PATH.open("rb") as fp:
        spec = tomllib.load(fp)
    raw = int(spec["meta"]["protocol_version"])
    return f"{raw >> 8}.{raw & 0xFF}"


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--check", action="store_true",
                    help="CI と名前を揃えるための別名（挙動は同じ）")
    ap.parse_args()

    want = expected_version()
    bad: list[str] = []
    checked = 0

    for name, pattern in CLAIMS:
        path = DOCS / name
        if not path.is_file():
            bad.append(f"{name}: 見つかりません")
            continue
        text = path.read_text(encoding="utf-8")
        m = re.search(pattern, text)
        if m is None:
            # 見出しの書き方を変えたらここも直す。**黙って素通りさせない**
            bad.append(f"{name}: 版を名乗る箇所が見つかりません（正規表現 {pattern}）")
            continue
        checked += 1
        got = m.group(1)
        line = text[:m.start()].count("\n") + 1
        if got != want:
            bad.append(f"{name}:{line}: v{got} ← protocol.toml は v{want}")

    if bad:
        print(f"文書のプロトコル版が protocol.toml（v{want}）と食い違っています:",
              file=sys.stderr)
        for b in bad:
            print(f"  {b}", file=sys.stderr)
        print("\n**protocol.toml が正。** 文書側を直してください "
              "（docs/README.md 自身がそう書いています）", file=sys.stderr)
        return 1

    print(f"文書のプロトコル版は protocol.toml と一致しています（v{want}、{checked}箇所）")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
