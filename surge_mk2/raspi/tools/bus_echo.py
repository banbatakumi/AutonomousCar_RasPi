"""bus_echo — 内部バスを覗く。**配線が通っているかを最初に確かめる道具。**

    .venv/bin/python -m raspi.tools.bus_echo                    # 頻度だけ 1Hz で表示
    .venv/bin/python -m raspi.tools.bus_echo -t vehicle_state --dump
    .venv/bin/python -m raspi.tools.bus_echo -t scan -t diag/link --dump --fields health,rx

「GUI に何も出ない」ときに、**バスに流れていないのか WS で落ちているのか**を
切り分けられないと時間を溶かす。まずここで publish 側を確かめること。
"""

from __future__ import annotations

import argparse
import signal
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

import msgspec  # noqa: E402

from raspi.bus import LATEST, RELIABLE, Subscriber  # noqa: E402
from raspi.msgs.types import (  # noqa: E402
    TOPIC_DIAG_LINK,
    TOPIC_SCAN,
    TOPIC_VEHICLE_STATE,
)

DEFAULT_TOPICS = [TOPIC_VEHICLE_STATE, TOPIC_SCAN, TOPIC_DIAG_LINK]


def _brief(topic: str, msg, fields: list[str] | None) -> str:
    d = msgspec.structs.asdict(msg)
    if fields:
        d = {k: d.get(k) for k in fields}
    else:
        # 360点の配列をそのまま出すと読めないので畳む
        d = {k: (f"[{len(v)}件 {v[:3]}…]" if isinstance(v, list) and len(v) > 8 else v)
             for k, v in d.items()}
    return f"{topic:14} " + " ".join(f"{k}={v}" for k, v in d.items())


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("-t", "--topic", action="append", default=None,
                    help="購読するトピック（複数可）。既定は vehicle_state/scan/diag/link")
    ap.add_argument("--dump", action="store_true", help="1件ずつ中身を出す")
    ap.add_argument("--fields", default=None, help="dump するフィールドをカンマ区切りで絞る")
    ap.add_argument("--reliable", action="store_true",
                    help="CONFLATE せず全部受ける（取りこぼしを見たいとき）")
    ap.add_argument("--duration", type=float, default=None)
    args = ap.parse_args()

    topics = args.topic or DEFAULT_TOPICS
    fields = args.fields.split(",") if args.fields else None
    pol = RELIABLE if args.reliable else LATEST
    sub = Subscriber({t: pol for t in topics})
    print(f"# 購読: {', '.join(topics)}  ({'RELIABLE' if args.reliable else 'LATEST'})")
    print("# publish 側（io_node / replay_node --bus / bus_demo）を別ターミナルで動かすこと\n")

    running = [True]
    signal.signal(signal.SIGINT, lambda *_: running.__setitem__(0, False))

    counts: dict[str, int] = {}
    last_ns: dict[str, int] = {}
    t0 = time.monotonic()
    next_report = t0 + 1.0
    prev_counts: dict[str, int] = {}

    while running[0]:
        if args.duration and time.monotonic() - t0 >= args.duration:
            break
        for topic, msg in sub.poll(50):
            counts[topic] = counts.get(topic, 0) + 1
            last_ns[topic] = msg.t_pub
            if args.dump:
                print(_brief(topic, msg, fields))

        now = time.monotonic()
        if not args.dump and now >= next_report:
            parts = []
            for t in topics:
                n = counts.get(t, 0)
                hz = n - prev_counts.get(t, 0)
                parts.append(f"{t}={n}({hz}Hz)")
            prev_counts = dict(counts)
            sys.stdout.write("\r" + "  ".join(parts) + "    ")
            sys.stdout.flush()
            next_report = now + 1.0

    print(f"\n\n=== 受信 === " + " ".join(f"{k}={v}" for k, v in counts.items())
          + (" — 1件も来ていない。publish 側が動いているか、"
             "SURGE_BUS_DIR が食い違っていないかを確認" if not counts else ""))
    sub.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
