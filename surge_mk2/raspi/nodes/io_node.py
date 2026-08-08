"""io_node — STM32 との UART 送受信ノード（Phase 0・バス無しの単体版）。

    .venv/bin/python -m raspi.nodes.io_node                # ライブ状態表示
    .venv/bin/python -m raspi.nodes.io_node --duration 10  # 10秒で終了
    .venv/bin/python -m raspi.nodes.io_node --quiet        # 表示なし（統計のみ最後に）
    .venv/bin/python -m raspi.nodes.io_node --log          # logs/ に .sfl を記録
    .venv/bin/python -m raspi.nodes.io_node --log run1.sfl # ファイル名を指定

やること:
- 起動時に VERSION_REQ → VERSION を照合（protocol_version 不一致は警告）
- PING を 5Hz（最初の3秒は 20Hz）で送り、PONG から時刻同期を推定
- COMMAND を 100Hz で送る。**安全のため常に DISARM・停止指令**（STM32 の
  COMMAND タイムアウトを防ぐハートビートを兼ねる。arm は絶対にしない）
- TELEMETRY / STATS / PONG / VERSION / LIDAR を受信・集計
- リンク健全性（TELEMETRY 途絶 100ms=警告 / 200ms=FAULT）を判定
- ライブ状態を1行で表示
- 送受信フレームを生のまま `.sfl` に記録（`--log`）

下流配信（ZeroMQ）はまだ入れない。受信は self.latest に保持し、コールバックで渡す。
"""

from __future__ import annotations

import argparse
import signal
import sys
import time

from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from raspi.core.link_tracker import (  # noqa: E402
    LinkState,
    LinkTracker,
    format_status,
    write_status,
)
from raspi.io.gpio import (  # noqa: E402
    PIN_BUZZER,
    PIN_HEARTBEAT,
    PIN_LED_GREEN,
    PIN_LED_RED,
    Heartbeat,
    Indication,
    StatusIndicator,
    open_output,
)
from raspi.io.serial_link import SerialLink  # noqa: E402
from raspi.proto import packets  # noqa: E402
from raspi.proto.generated.packets import PROTOCOL_VERSION  # noqa: E402
from raspi.rec import FrameLogWriter, default_log_path  # noqa: E402

__all__ = ["IoNode", "LinkState"]

NS = 1_000_000_000
COMMAND_HZ = 100
PING_HZ = 5
PING_WARMUP_HZ = 20
WARMUP_S = 3.0
LINKSTATS_HZ = 1


class IoNode:
    """:param log: 生フレームログの Writer。None なら記録しない。

    記録は「回っている限り必ず残る」ことが価値なので、ログ書き込みで例外が出ても
    ループは止めない（記録が壊れても走行は続けられるべき）。
    """

    def __init__(self, link: SerialLink, on_telemetry=None,
                 log: FrameLogWriter | None = None,
                 heartbeat: Heartbeat | None = None,
                 indicator: StatusIndicator | None = None) -> None:
        self.link = link
        self.tracker = LinkTracker(on_telemetry=on_telemetry, on_latch=self._on_latch)
        # 受信状態と時刻同期の実体は tracker が持つ。ここは同じ物への別名。
        self.state = self.tracker.state
        self.sync = self.tracker.sync
        self.heartbeat = heartbeat
        self.indicator = indicator
        self._log = log
        self._log_errors = 0
        self._ping_seq = 0
        self._running = False
        self._t_start = 0
        if log is not None:
            link.on_tx = self._log_tx

    # ── LED 表示 ──

    def _indication(self) -> Indication:
        """`LinkState` から LED に出す状態を作る。

        `armed` と低電圧は毎フレーム変わるのでラッチ扱いにせず、ここで読む。
        """
        t = self.state.telemetry
        flags = t.flags if t else 0
        undervolt = (packets.FLG_FAULT_DRIVE_UNDERVOLTAGE
                     | packets.FLG_FAULT_SIGNAL_UNDERVOLTAGE)
        return Indication(
            health=self.state.health,
            armed=bool(flags & packets.FLG_ARMED),
            estop=self.state.estop_active,
            power_locked=self.state.drive_power_locked,
            warning=bool(flags & undervolt),
        )

    # ── ラッチ系フラグ ──

    def _on_latch(self, name: str, value: bool, t_ns: int) -> None:
        """E-Stop / 駆動電源ラッチの変化。**必ず記録して人間に見せる。**

        どちらも人間が物理操作しないと戻らないので、気づかないまま
        「なぜ動かないのか」を探す時間が一番もったいない。
        """
        self._log_event("latch", {"flag": name, "value": value})
        if name == "estop_active" and value:
            print("\n!! E-STOP 発動 — 車両のボタン2を押すまで解除されません。"
                  "この間 COMMAND は一切効きません", file=sys.stderr)
        elif name == "estop_active":
            print("\n#  E-STOP 解除を確認", file=sys.stderr)
        elif name == "drive_power_locked" and value:
            print("\n!! 駆動電源ラッチ — 電源を入れ直すまで復帰しません", file=sys.stderr)

    # ── ログ ──

    def _log_tx(self, t_ns: int, pkt_type: int, seq: int, payload: bytes) -> None:
        try:
            self._log.write_tx(t_ns, pkt_type, seq, payload)
        except Exception:
            self._log_errors += 1

    def _log_event(self, name: str, data: dict | None = None) -> None:
        if self._log is None:
            return
        try:
            self._log.write_event(time.monotonic_ns(), name, data)
        except Exception:
            self._log_errors += 1

    def _log_linkstats(self, now_ns: int) -> None:
        """1Hz のリンク健全性スナップショット。

        CRC エラーや再同期バイト数は「捨てたフレーム」なので RX レコードには
        残らない。数字として別に残さないと、後から再生しても異常が見えない。
        ハートビートの実測も同じ理由でここに入れる（GPIO はログに現れない）。
        """
        hb = self.heartbeat
        self._log_event("linkstats", {
            "health": self.state.health,
            "rx": self.link.stats.as_dict(),
            "hb": ({"alive": hb.alive, **hb.stats.as_dict()} if hb else None),
            "sync": {
                "n": self.sync.n_samples,
                "offset_ns": self.sync.offset_ns,
                "best_delay_ns": self.sync.best_delay_ns,
                "drift_ppm": self.sync.drift_ppm,
            },
        })

    # ── 起動時の VERSION 照合 ──

    def handshake(self, timeout_s: float = 1.0) -> packets.Version | None:
        self.link.reset_input()
        self.link.send(packets.VersionReq())
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            for rx in self.link.poll():
                if self._log is not None:
                    self._log_rx(rx)
                msg = rx.decode()
                if isinstance(msg, packets.Version):
                    self.state.version = msg
                    self._log_event("handshake", {
                        "protocol_version": msg.protocol_version,
                        "expected": PROTOCOL_VERSION,
                        "match": msg.protocol_version == PROTOCOL_VERSION,
                        "fw_id": msg.fw_id,
                    })
                    return msg
        self._log_event("handshake", {"match": False, "reason": "timeout"})
        return None

    def _log_rx(self, rx) -> None:
        try:
            self._log.write_rx(rx.rx_ns, rx.type, rx.seq, rx.payload)
        except Exception:
            self._log_errors += 1

    # ── 送信 ──

    def _send_ping(self) -> None:
        self._ping_seq = (self._ping_seq + 1) & 0xFFFFFFFF
        t1 = self.link.send(packets.Ping(ping_id=self._ping_seq))
        self.tracker.note_ping_sent(self._ping_seq, t1)

    def _send_command_disarm(self) -> None:
        """安全な停止ハートビート。mode=DISARM, arm=0, 速度・舵角ゼロ。"""
        self.link.send(packets.Command(
            mode=packets.Mode.DISARM, flags=0,
            target_speed=0, target_steer=0,
            accel_limit=0, steer_rate_limit=0))

    # ── メインループ ──

    def run(self, duration_s: float | None = None, status_cb=None) -> None:
        self._running = True
        self._t_start = time.monotonic()
        next_cmd = time.monotonic_ns()
        next_ping = next_cmd
        next_status = next_cmd
        next_linkstats = next_cmd + NS // LINKSTATS_HZ
        cmd_period = NS // COMMAND_HZ

        while self._running:
            now = time.monotonic_ns()
            elapsed = (now - int(self._t_start * NS)) / NS

            # 「メインループは生きている」の申告。これが途切れるとハートビートが
            # 自分で波形を止め、STM32 が E-Stop に入る（フリーズ検出）
            if self.heartbeat is not None:
                self.heartbeat.kick()

            for rx in self.link.poll():
                if self._log is not None:
                    self._log_rx(rx)
                self.tracker.feed(rx.rx_ns, rx.type, rx.seq, rx.payload)

            if now >= next_cmd:
                self._send_command_disarm()
                next_cmd += cmd_period
                if now - next_cmd > cmd_period * 5:   # 大きく遅れたら追いつきをやめる
                    next_cmd = now + cmd_period

            if now >= next_ping:
                self._send_ping()
                hz = PING_WARMUP_HZ if elapsed < WARMUP_S else PING_HZ
                next_ping = now + NS // hz

            prev_health = self.state.health
            changed = self.tracker.update_health(now)
            if self.indicator is not None:
                self.indicator.update(self._indication())
            if self._log is not None:
                if changed is not None:
                    self._log_event("health", {"from": prev_health, "to": changed})
                if now >= next_linkstats:
                    self._log_linkstats(now)
                    next_linkstats = now + NS // LINKSTATS_HZ

            if status_cb and now >= next_status:
                status_cb(self)
                next_status = now + NS // 2      # 2Hz 更新

            if duration_s is not None and time.monotonic() - self._t_start >= duration_s:
                break

    def stop(self) -> None:
        self._running = False


# ── ライブ表示 ──

def _status_line(node: IoNode) -> None:
    write_status(format_status(node.state, node.sync, node.link.stats))


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", default="/dev/serial0")
    ap.add_argument("--baud", type=int, default=250_000)
    ap.add_argument("--duration", type=float, default=None, help="秒で自動終了")
    ap.add_argument("--quiet", action="store_true", help="ライブ表示なし")
    ap.add_argument("--log", nargs="?", const="logs", default=None, metavar="PATH",
                    help="生フレームログを記録。値なしで logs/ に自動命名、"
                         "*.sfl でファイル指定、それ以外はディレクトリ扱い")
    ap.add_argument("--no-heartbeat", action="store_true",
                    help="GPIO6 の E-Stop ハートビートを出さない（診断用）")
    ap.add_argument("--require-gpio", action="store_true",
                    help="GPIO を開けなければ起動しない")
    ap.add_argument("--no-leds", action="store_true", help="LED/ブザーを使わない")
    args = ap.parse_args()

    try:
        link = SerialLink(args.port, args.baud)
    except Exception as e:
        print(f"ポートを開けない: {e}", file=sys.stderr)
        return 2

    log = None
    if args.log is not None:
        path = Path(args.log) if args.log.endswith(".sfl") else default_log_path(args.log)
        log = FrameLogWriter(path, meta={
            "node": "io_node",
            "port": args.port,
            "baud": args.baud,
            "protocol_version": PROTOCOL_VERSION,
        })
        print(f"# 記録: {path}")

    # ── GPIO ──
    heartbeat = indicator = None
    if not args.no_heartbeat:
        pin = open_output(PIN_HEARTBEAT)
        if pin is None:
            msg = f"GPIO{PIN_HEARTBEAT} を開けない（E-Stop ハートビートなし）"
            if args.require_gpio:
                print(f"!! {msg}", file=sys.stderr)
                link.close()
                return 2
            print(f"!! {msg}。--require-gpio で起動を止められる")
        else:
            heartbeat = Heartbeat(pin)
            print(f"# E-Stop ハートビート GPIO{PIN_HEARTBEAT} "
                  f"{heartbeat.hz}Hz（kick 途絶 {heartbeat.kick_timeout_s}s で停止）")
            print("#  注意: 一度出し始めたら、止めた時点で STM32 が E-Stop をラッチする。"
                  "\n#        解除には車両のボタン2を押す必要がある（--no-heartbeat で回避）")
    else:
        # STM32 はハートビートを一度も見ていない間は E-Stop を発動しない。
        # ベンチ診断で E-Stop をラッチさせたくないときはこちら
        print("# ハートビート無効（ベンチ診断用）。STM32 は未接続扱いのまま")

    if not args.no_leds:
        pins = [open_output(p) for p in (PIN_LED_GREEN, PIN_LED_RED, PIN_BUZZER)]
        if any(p is not None for p in pins):
            indicator = StatusIndicator(*pins)

    node = IoNode(link, log=log, heartbeat=heartbeat, indicator=indicator)

    def _shutdown(*_):
        node.stop()

    signal.signal(signal.SIGINT, _shutdown)

    print(f"# io_node port={args.port} baud={args.baud} 期待 v{PROTOCOL_VERSION:#06x}")
    ver = node.handshake()
    if ver is None:
        print("!! VERSION 応答なし。STM32 が繋がっていない可能性。受信は続行する。")
    else:
        ok = "✓" if ver.protocol_version == PROTOCOL_VERSION else "✗ 不一致!"
        print(f"# STM32 protocol_version=0x{ver.protocol_version:04X} "
              f"fw=0x{ver.fw_id:08X} {ok}")

    print("# COMMAND は常に DISARM（安全）。Ctrl-C で停止。\n")
    if heartbeat is not None:
        node._log_event("heartbeat_start", {"pin": PIN_HEARTBEAT, "hz": heartbeat.hz})
        heartbeat.start()
    try:
        node.run(duration_s=args.duration,
                 status_cb=None if args.quiet else _status_line)
    finally:
        # 終了時にも DISARM を一発
        try:
            node._send_command_disarm()
        except Exception:
            pass
        if heartbeat is not None:
            # 波形を止める＝E-Stop を掛けて終わる。正常終了でもそうする。
            # Pi が監視をやめた以上、車両を止めておくのが正しい
            heartbeat.stop()
            node._log_event("heartbeat_stop", heartbeat.stats.as_dict())
            print("\n!! ハートビート停止 → STM32 は 50ms 後に E-STOP をラッチする。"
                  "\n   次に走らせる前に車両のボタン2を押して解除すること")
        if indicator is not None:
            indicator.close()
        if log is not None:
            node._log_linkstats(time.monotonic_ns())
            log.close()
        link.close()

    st = link.stats
    print("\n\n=== 終了時の統計 ===")
    print(f"frames_ok={st.frame_ok} crc_err={st.crc_error} len_err={st.len_error} "
          f"loss={st.packet_loss} reordered={st.reordered} dup={st.duplicate} "
          f"unknown={st.unknown_type} resync={st.resync_bytes}B")
    print(f"time sync: n={node.sync.n_samples} "
          f"offset={node.sync.offset_ns} ns best_delay={node.sync.best_delay_ns} ns "
          f"drift={node.sync.drift_ppm} ppm")
    if node.state.lidar_sectors:
        print(f"lidar sectors seen: {len(node.state.lidar_sectors)}/12")
    if heartbeat is not None:
        st = heartbeat.stats
        hz = st.edges / 2 / args.duration if args.duration else None
        print(f"heartbeat: edges={st.edges}"
              + (f" ({hz:.1f}Hz)" if hz else "")
              + f" 最大遅れ={st.max_late_ns / 1e6:.2f}ms "
                f"停止={st.stalls}回 skip={st.skipped} {st.late_hist}")
    if log is not None:
        mb = log.stats.bytes_written / 1e6
        print(f"log: {log.path} rx={log.stats.rx} tx={log.stats.tx} "
              f"events={log.stats.events} {mb:.1f}MB"
              + (f" !! 書き込みエラー {node._log_errors} 回" if node._log_errors else ""))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
