"""route_select — 信号認識の代わりに `route/select` を送る（経路グループの切替の試験用）。

    .venv/bin/python -m raspi.tools.route_select left      # signal_map で読み替え
    .venv/bin/python -m raspi.tools.route_select B         # グループ名をそのまま

矢印信号（大会⑥）の認識ノードはまだ無い。**その受け口（`route/select` →
planning_node → `slam2d_route.request_signal()`）を先に通しておく**ための道具。
認識ノードを作るときも、このファイルと同じく `signal` というノード名で
`RouteSelect` を publish すればよい（`raspi/bus/zbus.py` の `TOPIC_OWNER`）。

**認識ノードが動いている間は使えない**（同じ `signal` の endpoint に bind するため）。

## 1秒ほど繰り返し送る

ZeroMQ の PUB は、購読側の接続が間に合う前に送った分を捨てる（slow joiner）。
1回だけ送ると届かないことがあるので、同じ要求（同じ `event_id`）を1秒間
流す。planning_node は `(value, event_id)` が変わったときだけ効かせるので、
何回届いても切替は1回になる（`RouteSelect` の docstring）。
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from raspi.bus import Publisher  # noqa: E402
from raspi.msgs.types import TOPIC_ROUTE_SELECT, RouteSelect  # noqa: E402


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("value", help="信号の値（left/right 等）またはグループ名（A〜D）")
    ap.add_argument("--source", default="cli", help="表示用の送り手の名前")
    ap.add_argument("--repeat-s", type=float, default=1.0, help="繰り返し送る時間 [s]")
    args = ap.parse_args()

    pub = Publisher("signal")
    msg = RouteSelect(value=args.value, source=args.source, event_id=time.time_ns())
    t_end = time.monotonic() + max(0.0, args.repeat_s)
    n = 0
    while True:
        pub.send(TOPIC_ROUTE_SELECT, msg)
        n += 1
        if time.monotonic() >= t_end:
            break
        time.sleep(0.1)
    pub.close()
    print(f"# route/select: {args.value}（{n} 回送った）")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
