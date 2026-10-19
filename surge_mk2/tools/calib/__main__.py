"""`python -m tools.calib 記録.mcap … [--apply]` — 解析して所見と書き込む値を出す。`--apply` で `config/vehicle.toml` へ書く
（不合格の記録の値は書かない）。"""

from __future__ import annotations

import argparse
import sys

from . import analyze_file, toml_io


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(prog="python -m tools.calib", description=__doc__)
    ap.add_argument("paths", nargs="+", help="校正の試験の記録（mcap）")
    ap.add_argument("--apply", action="store_true", help="合格・要確認の記録の値を config/vehicle.toml に書く")
    a = ap.parse_args(argv)
    rc = 0
    for p in a.paths:
        try:
            cal, res = analyze_file(p)
        except Exception as e:                      # noqa: BLE001  1本の失敗で残りを止めない
            print(f"■ {p}: 解析できない（{e}）")
            rc = 1
            continue
        print(f"[{p}]  校正 {cal.order}: {cal.title}" + (f"（先に {cal.needs}）" if cal.needs else ""))
        print(res.report())
        print()
        if a.apply:
            if not res.usable:
                print("  → 不合格なので書かない\n")
                rc = 1
                continue
            for line in toml_io.apply(res.values):
                print(f"  書いた: {line}")
            print()
    return rc


if __name__ == "__main__":
    sys.exit(main())
