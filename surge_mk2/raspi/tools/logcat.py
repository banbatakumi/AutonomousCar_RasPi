"""`.sfl` 生フレームログを読む。要約・ダンプ・CSV 書き出し。

    python3 -m raspi.tools.logcat logs/surge_20260807_153012.sfl
    python3 -m raspi.tools.logcat run.sfl --dump 40
    python3 -m raspi.tools.logcat run.sfl --dump 40 --type TELEMETRY --rx
    python3 -m raspi.tools.logcat run.sfl --csv telem.csv --type TELEMETRY

CSV は protocol.toml の `META`（スケール係数と単位）を当てて**物理量**で出す。
生の整数のままだと表計算に貼ったときに何の値か分からなくなるため。
配列フィールドは `wheel_speed[0]` のように展開する。
"""

from __future__ import annotations

import argparse
import csv
import sys
from collections import Counter, defaultdict
from dataclasses import fields as dc_fields
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from raspi.proto import packets  # noqa: E402
from raspi.rec import FrameLogReader, Kind  # noqa: E402


def _type_name(t: int) -> str:
    cls = packets.BY_TYPE.get(t)
    return cls.NAME if cls else f"0x{t:02X}?"


def _resolve_type(name: str) -> int:
    """TYPE 名（TELEMETRY）または番号（0x02 / 2）を TYPE 値にする。"""
    cls = packets.BY_NAME.get(name.upper())
    if cls is not None:
        return cls.TYPE
    try:
        return int(name, 0)
    except ValueError:
        raise SystemExit(f"不明な TYPE: {name}\n候補: {', '.join(sorted(packets.BY_NAME))}")


# ── 要約 ────────────────────────────────────────────────────────────────

def summarize(path: Path) -> int:
    with FrameLogReader(path) as r:
        counts: dict[int, Counter] = {Kind.RX: Counter(), Kind.TX: Counter()}
        n_events = 0
        first_ns = last_ns = None
        last_seen: dict[tuple[int, int], int] = {}
        max_gap: dict[tuple[int, int], int] = defaultdict(int)
        events: list[tuple[float, dict]] = []
        last_linkstats: dict | None = None
        closed = False

        for rec in r:
            if rec.is_frame:
                # 長さはフレームの時刻だけで測る。イベントは実時刻で入ることがあり、
                # また記録順が時刻順とは限らないので min/max を取る。
                first_ns = rec.t_ns if first_ns is None else min(first_ns, rec.t_ns)
                last_ns = rec.t_ns if last_ns is None else max(last_ns, rec.t_ns)
                counts[rec.kind][rec.type] += 1
                key = (rec.kind, rec.type)
                prev = last_seen.get(key)
                if prev is not None and rec.t_ns > prev:
                    max_gap[key] = max(max_gap[key], rec.t_ns - prev)
                    last_seen[key] = rec.t_ns
                elif prev is None:
                    last_seen[key] = rec.t_ns
            elif rec.kind == Kind.EVENT:
                n_events += 1
                body = rec.json()
                if body.get("name") == "linkstats":
                    last_linkstats = body
                else:
                    if body.get("name") == "close":
                        closed = True
                    events.append((r.rel_s(rec.t_ns), body))

        dur = (last_ns - first_ns) / 1e9 if first_ns is not None and last_ns else 0.0

        print(f"== {path}  ({path.stat().st_size / 1e6:.2f} MB) ==")
        print(f"形式 v{r.header.format_version}  長さ {dur:.2f} s"
              + ("  ** 末尾が切れている（記録中に落ちた可能性）**" if r.truncated else "")
              + ("" if closed or r.truncated else "  ** close イベント無し **"))

        if r.meta:
            print("\n-- META --")
            for k, v in r.meta.items():
                print(f"  {k}: {v}")

        for kind in (Kind.RX, Kind.TX):
            c = counts[kind]
            if not c:
                continue
            print(f"\n-- {Kind.NAMES[kind]} フレーム （計 {sum(c.values())}）--")
            print(f"  {'TYPE':<16} {'count':>8} {'Hz':>8} {'期待Hz':>8} {'最大間隔':>10}")
            for t, n in sorted(c.items(), key=lambda kv: -kv[1]):
                cls = packets.BY_TYPE.get(t)
                exp = getattr(cls, "RATE_HZ", None) if cls else None
                hz = n / dur if dur > 0 else 0.0
                gap_ms = max_gap[(kind, t)] / 1e6
                print(f"  {_type_name(t):<16} {n:>8} {hz:>8.1f} "
                      f"{(f'{exp:.0f}' if exp else '—'):>8} {gap_ms:>9.1f}ms")

        if last_linkstats:
            rx = last_linkstats.get("rx", {})
            sync = last_linkstats.get("sync", {})
            print("\n-- 最後のリンク統計 --")
            print("  " + " ".join(f"{k}={v}" for k, v in rx.items() if v or k == "frame_ok"))
            print(f"  health={last_linkstats.get('health')} "
                  f"sync n={sync.get('n')} offset={sync.get('offset_ns')}ns "
                  f"drift={sync.get('drift_ppm')}ppm")

        if events:
            print(f"\n-- イベント （linkstats を除き {len(events)} 件）--")
            for t_s, body in events[:40]:
                name = body.pop("name", "?")
                extra = " ".join(f"{k}={v}" for k, v in body.items())
                print(f"  {t_s:8.3f}s  {name:<12} {extra}")
            if len(events) > 40:
                print(f"  … 他 {len(events) - 40} 件")
        print(f"\nEVENT レコード合計: {n_events}")
    return 0


# ── ダンプ ──────────────────────────────────────────────────────────────

def dump(path: Path, limit: int, want_type: int | None,
         kinds: set[int]) -> int:
    with FrameLogReader(path) as r:
        n = 0
        for rec in r:
            if rec.kind not in kinds:
                continue
            if want_type is not None and (not rec.is_frame or rec.type != want_type):
                continue
            t = r.rel_s(rec.t_ns)
            if rec.is_frame:
                msg = rec.decode()
                print(f"{t:10.4f}s {Kind.NAMES[rec.kind]:<5} seq={rec.seq:>3} "
                      f"{_type_name(rec.type):<16} {msg if msg else rec.payload.hex()}")
            else:
                print(f"{t:10.4f}s {Kind.NAMES[rec.kind]:<5} {rec.json()}")
            n += 1
            if limit and n >= limit:
                break
    return 0


# ── CSV ─────────────────────────────────────────────────────────────────

def to_csv(path: Path, out: Path, want_type: int, kind: int) -> int:
    cls = packets.BY_TYPE.get(want_type)
    if cls is None:
        raise SystemExit(f"未知の TYPE 0x{want_type:02X}")
    meta = getattr(cls, "META", {})
    names = [f.name for f in dc_fields(cls)]

    with FrameLogReader(path) as r, open(out, "w", newline="", encoding="utf-8") as fp:
        w = csv.writer(fp)
        header: list[str] | None = None
        n = 0
        for rec in r.frames(kind):
            if rec.type != want_type:
                continue
            msg = rec.decode()
            if msg is None:
                continue

            row: list = [round(r.rel_s(rec.t_ns), 6), rec.seq]
            cols: list[str] = []
            for name in names:
                scale, unit = meta.get(name, (None, None))
                v = getattr(msg, name)
                items = v if isinstance(v, list) else [v]
                for i, item in enumerate(items):
                    col = f"{name}[{i}]" if isinstance(v, list) else name
                    # 単位は換算したときだけ付ける。t_us のように名前へ単位が
                    # 入っているフィールドはスケール無しなので t_us_us にならない
                    if scale is None:
                        cols.append(col)
                        row.append(item)
                    else:
                        cols.append(f"{col}_{unit}" if unit else col)
                        # スケールは 1e-4 桁までなので 9 桁で丸めれば
                        # 0.0133000000000001 のような二進丸め誤差だけが落ちる
                        row.append(round(item * scale, 9))
            if header is None:
                header = ["t_rel_s", "seq"] + cols
                w.writerow(header)
            w.writerow(row)
            n += 1

    if n == 0:
        print(f"{cls.NAME} のフレームが1つもありません", file=sys.stderr)
        return 1
    print(f"{out}: {cls.NAME} {n} 行（スケール適用済み・物理量）")
    return 0


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("path", type=Path)
    ap.add_argument("--dump", type=int, nargs="?", const=0, default=None,
                    metavar="N", help="レコードを表示（N 件で打ち切り、0=全部）")
    ap.add_argument("--csv", type=Path, default=None, metavar="OUT",
                    help="指定 TYPE を CSV に書き出す（--type 必須）")
    ap.add_argument("--type", default=None,
                    help="TYPE 名（TELEMETRY）か番号（0x02）で絞る")
    ap.add_argument("--rx", action="store_true", help="受信のみ")
    ap.add_argument("--tx", action="store_true", help="送信のみ")
    args = ap.parse_args()

    if not args.path.exists():
        print(f"ファイルがありません: {args.path}", file=sys.stderr)
        return 2

    want_type = _resolve_type(args.type) if args.type else None
    kind = Kind.TX if args.tx and not args.rx else Kind.RX

    if args.csv is not None:
        if want_type is None:
            print("--csv には --type が要ります（例: --type TELEMETRY）", file=sys.stderr)
            return 2
        return to_csv(args.path, args.csv, want_type, kind)

    if args.dump is not None:
        kinds = set()
        if args.rx:
            kinds.add(Kind.RX)
        if args.tx:
            kinds.add(Kind.TX)
        if not kinds:
            kinds = {Kind.RX, Kind.TX, Kind.META, Kind.EVENT}
        return dump(args.path, args.dump, want_type, kinds)

    return summarize(args.path)


if __name__ == "__main__":
    raise SystemExit(main())
