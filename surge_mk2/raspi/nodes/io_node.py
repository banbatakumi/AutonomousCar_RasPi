"""io_node — STM32 との UART 送受信ノード（Phase 0・バス無しの単体版）。

    .venv/bin/python -m raspi.nodes.io_node                # ライブ状態表示
    .venv/bin/python -m raspi.nodes.io_node --duration 10  # 10秒で終了
    .venv/bin/python -m raspi.nodes.io_node --quiet        # 表示なし（統計のみ最後に）

やること:
- 起動時に VERSION_REQ → VERSION を照合（protocol_version 不一致は警告）
- PING を 5Hz（最初の3秒は 20Hz）で送り、PONG から時刻同期を推定
- COMMAND を 100Hz で送る。**安全のため常に DISARM・停止指令**（STM32 の
  COMMAND タイムアウトを防ぐハートビートを兼ねる。arm は絶対にしない）
- TELEMETRY / STATS / PONG / VERSION / LIDAR を受信・集計
- リンク健全性（TELEMETRY 途絶 100ms=警告 / 200ms=FAULT）を判定
- ライブ状態を1行で表示

下流配信（ZeroMQ）はまだ入れない。受信は self.latest に保持し、コールバックで渡す。
"""

from __future__ import annotations

import argparse
import signal
import sys
import time
from dataclasses import dataclass, field
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from raspi.core.timesync import TimeSync  # noqa: E402
from raspi.io.serial_link import SerialLink  # noqa: E402
from raspi.proto import packets  # noqa: E402
from raspi.proto.generated.packets import PROTOCOL_VERSION  # noqa: E402

NS = 1_000_000_000
COMMAND_HZ = 100
PING_HZ = 5
PING_WARMUP_HZ = 20
WARMUP_S = 3.0
TELEM_DEGRADED_NS = 100 * 1_000_000
TELEM_FAULT_NS = 200 * 1_000_000


@dataclass(slots=True)
class LinkState:
    """受信状況のスナップショット。GUI/下流に出す最小集合。"""

    telemetry: packets.Telemetry | None = None
    stats: packets.Stats | None = None
    version: packets.Version | None = None
    last_telem_ns: int = 0
    lidar_sectors: set[int] = field(default_factory=set)
    counts: dict[int, int] = field(default_factory=dict)
    health: str = "INIT"          # INIT / OK / DEGRADED / FAULT


class IoNode:
    def __init__(self, link: SerialLink, on_telemetry=None) -> None:
        self.link = link
        self.sync = TimeSync()
        self.state = LinkState()
        self._on_telemetry = on_telemetry
        self._pending_pings: dict[int, int] = {}   # ping_id -> T1 ns
        self._ping_seq = 0
        self._cmd_seq = 0
        self._running = False
        self._t_start = 0

    # ── 起動時の VERSION 照合 ──

    def handshake(self, timeout_s: float = 1.0) -> packets.Version | None:
        self.link.reset_input()
        self.link.send(packets.VersionReq())
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            for rx in self.link.poll():
                msg = rx.decode()
                if isinstance(msg, packets.Version):
                    self.state.version = msg
                    return msg
        return None

    # ── 送信 ──

    def _send_ping(self) -> None:
        self._ping_seq = (self._ping_seq + 1) & 0xFFFFFFFF
        t1 = self.link.send(packets.Ping(ping_id=self._ping_seq))
        self._pending_pings[self._ping_seq] = t1
        # 取りこぼした PONG の T1 が溜まり続けないよう上限を設ける
        if len(self._pending_pings) > 32:
            oldest = min(self._pending_pings)
            self._pending_pings.pop(oldest, None)

    def _send_command_disarm(self) -> None:
        """安全な停止ハートビート。mode=DISARM, arm=0, 速度・舵角ゼロ。"""
        self._cmd_seq = (self._cmd_seq + 1) & 0xFF
        self.link.send(packets.Command(
            mode=packets.Mode.DISARM, flags=0,
            target_speed=0, target_steer=0,
            accel_limit=0, steer_rate_limit=0))

    # ── 受信ディスパッチ ──

    def _dispatch(self, rx) -> None:
        c = self.state.counts
        c[rx.type] = c.get(rx.type, 0) + 1
        msg = rx.decode()

        if isinstance(msg, packets.Telemetry):
            self.sync.observe_stm_us(msg.t_us)
            self.state.telemetry = msg
            self.state.last_telem_ns = rx.rx_ns
            if self._on_telemetry:
                self._on_telemetry(msg, self.sync.to_pi_ns(msg.t_us))
        elif isinstance(msg, packets.Pong):
            t1 = self._pending_pings.pop(msg.ping_id, None)
            if t1 is not None:
                self.sync.add_pong(t1, msg.t_ping_rx_us, msg.t_pong_tx_us, rx.rx_ns)
        elif isinstance(msg, packets.Stats):
            self.sync.observe_stm_us(msg.t_us)
            self.state.stats = msg
        elif isinstance(msg, packets.Version):
            self.state.version = msg
        elif isinstance(msg, (packets.LidarSector, packets.LidarSectorI,
                              packets.LidarSectorC)):
            self.state.lidar_sectors.add(msg.sector_idx)

    def _update_health(self, now_ns: int) -> None:
        if self.state.last_telem_ns == 0:
            self.state.health = "INIT"
            return
        gap = now_ns - self.state.last_telem_ns
        if gap > TELEM_FAULT_NS:
            self.state.health = "FAULT"
        elif gap > TELEM_DEGRADED_NS:
            self.state.health = "DEGRADED"
        else:
            self.state.health = "OK"

    # ── メインループ ──

    def run(self, duration_s: float | None = None, status_cb=None) -> None:
        self._running = True
        self._t_start = time.monotonic()
        next_cmd = time.monotonic_ns()
        next_ping = next_cmd
        next_status = next_cmd
        cmd_period = NS // COMMAND_HZ

        while self._running:
            now = time.monotonic_ns()
            elapsed = (now - int(self._t_start * NS)) / NS

            for rx in self.link.poll():
                self._dispatch(rx)

            if now >= next_cmd:
                self._send_command_disarm()
                next_cmd += cmd_period
                if now - next_cmd > cmd_period * 5:   # 大きく遅れたら追いつきをやめる
                    next_cmd = now + cmd_period

            if now >= next_ping:
                self._send_ping()
                hz = PING_WARMUP_HZ if elapsed < WARMUP_S else PING_HZ
                next_ping = now + NS // hz

            self._update_health(now)

            if status_cb and now >= next_status:
                status_cb(self)
                next_status = now + NS // 2      # 2Hz 更新

            if duration_s is not None and time.monotonic() - self._t_start >= duration_s:
                break

    def stop(self) -> None:
        self._running = False


# ── ライブ表示 ──

def _status_line(node: IoNode) -> None:
    s = node.state
    t = s.telemetry
    sync = node.sync
    off = sync.offset_ns
    delay = sync.best_delay_ns
    drift = sync.drift_ppm

    off_s = f"{off / 1000:+.1f}μs" if off is not None else "—"
    delay_s = f"{delay / 1000:.0f}μs" if delay is not None else "—"
    drift_s = f"{drift:+.1f}ppm" if drift is not None else "—"

    if t:
        veh = (f"v={t.speed * 1e-3:+.2f} steer={t.steer_actual * 1e-4:+.3f} "
               f"az={t.accel_z * 1e-3:.1f} flags=0x{t.flags:04X}")
    else:
        veh = "TELEMETRY 未受信"

    rst = node.link.stats
    counts = " ".join(f"{packets.BY_TYPE[t].NAME[:4]}={n}"
                      for t, n in sorted(s.counts.items()) if t in packets.BY_TYPE)
    sys.stdout.write(
        f"\r[{s.health:8}] sync off={off_s} delay={delay_s} drift={drift_s} "
        f"n={sync.n_samples:>3} | {veh} | {counts} "
        f"crc={rst.crc_error} loss={rst.packet_loss} reord={rst.reordered}    ")
    sys.stdout.flush()


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", default="/dev/serial0")
    ap.add_argument("--baud", type=int, default=250_000)
    ap.add_argument("--duration", type=float, default=None, help="秒で自動終了")
    ap.add_argument("--quiet", action="store_true", help="ライブ表示なし")
    args = ap.parse_args()

    try:
        link = SerialLink(args.port, args.baud)
    except Exception as e:
        print(f"ポートを開けない: {e}", file=sys.stderr)
        return 2

    node = IoNode(link)

    signal.signal(signal.SIGINT, lambda *_: node.stop())

    print(f"# io_node port={args.port} baud={args.baud} 期待 v{PROTOCOL_VERSION:#06x}")
    ver = node.handshake()
    if ver is None:
        print("!! VERSION 応答なし。STM32 が繋がっていない可能性。受信は続行する。")
    else:
        ok = "✓" if ver.protocol_version == PROTOCOL_VERSION else "✗ 不一致!"
        print(f"# STM32 protocol_version=0x{ver.protocol_version:04X} "
              f"fw=0x{ver.fw_id:08X} {ok}")

    print("# COMMAND は常に DISARM（安全）。Ctrl-C で停止。\n")
    try:
        node.run(duration_s=args.duration,
                 status_cb=None if args.quiet else _status_line)
    finally:
        # 終了時にも DISARM を一発
        try:
            node._send_command_disarm()
        except Exception:
            pass
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
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
