"""`.sfl` → `.mcap` 変換。**記録済みの生フレームログを Foxglove で開ける形にする。**

    python3 -m raspi.tools.sfl2mcap logs/surge_20260808_120000.sfl
    python3 -m raspi.tools.sfl2mcap logs/run.sfl -o /tmp/run.mcap --start 10 --end 40
    python3 -m raspi.tools.sfl2mcap logs/*.sfl            # まとめて変換

## 変換ではなく「再生してから書く」

`.sfl` は UART の生バイトなので、`vehicle_state` に直すには SI 換算・前輪
オドメトリの射影・LiDAR 12セクタの組み立てが要る。**これを書き直すと
実機とズレる**ので、`replay_node` と `BusBridge` をそのまま使う
（`Publisher` の代わりに MCAP に書くだけ）。実機・再生・変換の3つで
同じコードが動くことになる。

時刻は `.sfl` ヘッダの `t0_mono_ns` / `t0_unix_ns` で epoch に直すので、
**Foxglove には記録した日時が出る**（変換した日時ではない）。

## 書き出すトピック

| トピック | 中身 |
|---|---|
| `/vehicle_state` `/scan` `/diag/link` | バスに流れるものと同じ msgspec 型 |
| `/uart/tx/*` `/uart/rx/*` | UART フレームの生値（TELEMETRY と LIDAR_SECTOR を除く） |
| `/events` | `.sfl` の EVENT レコード（linkstats・health 遷移など） |
| `/viz/scan` | Foxglove の点群（`--no-viz` で止まる） |

`TELEMETRY` と `LIDAR_SECTOR` を `/uart/rx/*` に出さないのは、
**`/vehicle_state` と `/scan` が同じ中身の解釈済みの姿**だから。両方書くと
ファイルがほぼ倍になる。生値がそのまま要るときは `.sfl` を読むこと。
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path
from typing import Any

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from raspi.core.bus_bridge import BusBridge  # noqa: E402
from raspi.nodes.replay_node import ReplayNode  # noqa: E402
from raspi.proto import packets  # noqa: E402
from raspi.proto.generated.packets import PROTOCOL_VERSION  # noqa: E402
from raspi.rec import FrameLogReader  # noqa: E402
from raspi.rec.mcap_log import McapLog  # noqa: E402

__all__ = ["export"]

#: `/uart/rx/*` に出さない TYPE。解釈済みの `/vehicle_state` `/scan` と重複するため
_RX_SKIP = {packets.Telemetry.TYPE, packets.LidarSector.TYPE,
            packets.LidarSectorI.TYPE, packets.LidarSectorC.TYPE}

#: `.sfl` の EVENT は形が決まっていない（linkstats / health / close …）ので、
#: スキーマは「何でも入る object」にしておく。**中身を型で縛ると記録が落ちる**
_EVENT_SCHEMA: dict[str, Any] = {
    "type": "object",
    "title": "SflEvent",
    "properties": {"name": {"type": "string"}},
    "additionalProperties": True,
}

#: `diag/link` を書く間隔。TELEMETRY 50Hz の 1/5 = 10Hz（replay_node --bus と同じ）
_DIAG_EVERY = 5


class _McapSink:
    """`BusBridge` から見た `Publisher` のふり。送る代わりに MCAP へ書く。

    `Publisher.send()` と同じく `seq` と `t_pub` を押す。**押さないと
    実機の記録と再生結果でメッセージの中身が変わる**（seq が 0 のまま残る）。
    """

    def __init__(self, log: McapLog, clock) -> None:
        self.log = log
        self.clock = clock
        self.viz = True
        self._seq: dict[str, int] = {}
        self.sent = 0

    def send(self, topic: str, msg):
        seq = self._seq.get(topic, 0) + 1
        self._seq[topic] = seq
        msg.seq = seq
        msg.t_pub = self.clock()
        if msg.t_capture == 0:
            msg.t_capture = msg.t_pub
        self.log.write("/" + topic, msg)
        if self.viz and topic == "scan":
            self.log.write_viz_scan(msg)
        self.sent += 1
        return msg


def export(path: str | Path, out: str | Path, *, viz: bool = True,
           compression: str = "zstd", start_s: float = 0.0,
           end_s: float | None = None, quiet: bool = False) -> McapLog:
    """`.sfl` 1本を `.mcap` に変換して、閉じた `McapLog` を返す（統計を見るため）。"""
    path, out = Path(path), Path(out)

    # 時刻の基準はログのヘッダから取る。**現在時刻で換算してはいけない**
    with FrameLogReader(path) as r:
        t0_mono, t0_unix, meta = r.header.t0_mono_ns, r.header.t0_unix_ns, r.meta

    log = McapLog(out, t0_mono_ns=t0_mono, t0_unix_ns=t0_unix,
                  compression=compression,
                  metadata={"source": path.name, "converter": "sfl2mcap",
                            **{k: v for k, v in meta.items() if k != "t0_mono_ns"}})

    holder: dict[str, ReplayNode] = {}
    sink = _McapSink(log, clock=lambda: holder["node"]._cursor_ns)
    sink.viz = viz
    bridge = BusBridge(sink, clock=lambda: holder["node"]._cursor_ns)
    n_telem = 0

    def on_telemetry(t, t_pi_ns):
        nonlocal n_telem
        bridge.on_telemetry(t, t_pi_ns)
        n_telem += 1
        if n_telem % _DIAG_EVERY == 0:
            node = holder["node"]
            bridge.publish_diag(node.state, node.sync, node.recorded_stats,
                                arm_inhibited=True, cmd_source="replay",
                                cmd_stale=True, expected_version=PROTOCOL_VERSION)

    def on_frame(t_ns, ptype, seq, msg):
        bridge.on_frame(t_ns, ptype, seq, msg)
        if msg is not None and ptype not in _RX_SKIP:
            log.write(f"/uart/rx/{_name(ptype)}", msg, t_mono_ns=t_ns)

    def on_tx(t_ns, ptype, seq, msg):
        if msg is not None:
            log.write(f"/uart/tx/{_name(ptype)}", msg, t_mono_ns=t_ns)

    def on_event(t_ns, body):
        log.write_dict("/events", body, "SflEvent", _EVENT_SCHEMA, t_ns)

    node = ReplayNode(path, speed=0.0, start_s=start_s, end_s=end_s,
                      on_telemetry=on_telemetry, on_frame=on_frame,
                      on_tx=on_tx, on_event=on_event)
    holder["node"] = node
    try:
        node.run()
    finally:
        log.close()

    if not quiet:
        dur = node.rel_s(node._cursor_ns)
        print(f"{path.name} → {out}  {log.size_text}  {dur:.1f}s分")
        for topic, n in sorted(log.counts.items()):
            print(f"  {topic:24} {n:8}")
        if getattr(node, "truncated", False):
            print("  !! 元の .sfl の末尾が切れている（記録中に落ちた可能性）")
    return log


def _name(ptype: int) -> str:
    cls = packets.BY_TYPE.get(ptype)
    return cls.NAME.lower() if cls is not None else f"type_{ptype:#04x}"


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("paths", type=Path, nargs="+", help="変換する .sfl")
    ap.add_argument("-o", "--out", type=Path, default=None,
                    help="出力先（既定は入力の拡張子を .mcap に変えたもの）")
    ap.add_argument("--start", type=float, default=0.0, help="開始位置 [s]")
    ap.add_argument("--end", type=float, default=None, help="終了位置 [s]")
    ap.add_argument("--compression", default="zstd", choices=["zstd", "lz4", "none"])
    ap.add_argument("--no-viz", action="store_true",
                    help="Foxglove 用の /viz/scan を書かない")
    args = ap.parse_args()

    if args.out is not None and len(args.paths) > 1:
        print("-o は入力1本のときだけ使えます", file=sys.stderr)
        return 2

    rc = 0
    for p in args.paths:
        if not p.exists():
            print(f"ファイルがありません: {p}", file=sys.stderr)
            rc = 2
            continue
        out = args.out or p.with_suffix(".mcap")
        try:
            export(p, out, viz=not args.no_viz, compression=args.compression,
                   start_s=args.start, end_s=args.end)
        except RuntimeError as e:             # mcap が無い
            print(f"!! {e}", file=sys.stderr)
            return 2
        except ValueError as e:               # .sfl ではない / 壊れている
            print(f"!! {p}: {e}", file=sys.stderr)
            rc = 1
    return rc


if __name__ == "__main__":
    raise SystemExit(main())
