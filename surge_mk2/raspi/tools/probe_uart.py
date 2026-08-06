#!/usr/bin/env python3
"""STM32 との UART 疎通を確認する診断ツール。

io_node を書く前に「そもそも通信できているか」「ファームが v0.4 か」を切り分ける。

    .venv/bin/python raspi/tools/probe_uart.py                 # VERSION_REQ を投げて 3秒聞く
    .venv/bin/python raspi/tools/probe_uart.py --passive       # 何も送らず受信だけ見る
    .venv/bin/python raspi/tools/probe_uart.py --seconds 10 --ping

出力:
- 生バイトの先頭サンプル（フレーミングが合わなくても中身が見える）
- TYPE 別のフレーム数
- VERSION が返れば protocol_version を照合（0x0004 = v0.4）
- 最初の TELEMETRY を SI 単位でデコード表示
- CRC / ロス / 長さエラーの統計
"""

from __future__ import annotations

import argparse
import sys
import time
from collections import Counter
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from raspi.proto import FrameEncoder, FrameParser, packets  # noqa: E402
from raspi.proto.generated.packets import PROTOCOL_VERSION, S2P_TYPES  # noqa: E402

try:
    import serial
except ImportError:
    sys.exit("pyserial が無い。 .venv/bin/pip install pyserial")


def hexdump(data: bytes, limit: int = 64) -> str:
    chunk = data[:limit]
    out = " ".join(f"{b:02X}" for b in chunk)
    if len(data) > limit:
        out += f" … (+{len(data) - limit}B)"
    return out


def fmt_telemetry(t: packets.Telemetry) -> str:
    m = packets.Telemetry.META
    def si(name):
        raw = getattr(t, name)
        scale = m.get(name, (1, ""))[0] or 1
        return raw * scale
    lines = [
        f"    t_us         = {t.t_us}",
        f"    flags        = 0x{t.flags:08X}  mode={t.flags & 0x3} "
        f"armed={bool(t.flags & packets.FLG_ARMED)} "
        f"estop={bool(t.flags & packets.FLG_ESTOP_ACTIVE)} "
        f"steer_center_valid={bool(t.flags & packets.FLG_STEER_CENTER_VALID)}",
        f"    speed        = {si('speed'):+.3f} m/s",
        f"    steer_actual = {si('steer_actual'):+.4f} rad",
        f"    yaw_rate     = {si('yaw_rate'):+.3f} rad/s",
        f"    wheel_speed  = {[round(x * 0.001, 3) for x in t.wheel_speed]} m/s [FL,FR,RL,RR]",
        f"    odom_dist    = {[round(x * 1e-4, 4) for x in t.odom_dist]} m [FL,FR]",
        f"    accel        = ({si('accel_x'):+.2f}, {si('accel_y'):+.2f}, {si('accel_z'):+.2f}) m/s²",
        f"    temp         = {list(t.temp)} °C [RL,RR,ST,MCU]",
        f"    batt_drive   = {t.batt_voltage_drive * 0.05:.2f} V / {t.batt_current_drive * 0.05:.2f} A",
        f"    batt_signal  = {t.batt_voltage_signal * 0.05:.2f} V / {t.batt_current_signal * 0.02:.2f} A",
        f"    us_front/rear= {t.us_front * 0.02:.2f} / {t.us_rear * 0.02:.2f} m (0=無効)",
        f"    md_status    = {[f'0x{s:02X}' for s in t.md_status]} [RL,RR,ST]",
        f"    cmd_seq_echo = {t.cmd_seq_echo}",
    ]
    return "\n".join(lines)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", default="/dev/serial0")
    ap.add_argument("--baud", type=int, default=250000)
    ap.add_argument("--seconds", type=float, default=3.0, help="受信を聞く秒数")
    ap.add_argument("--passive", action="store_true", help="何も送信せず受信だけ")
    ap.add_argument("--ping", action="store_true", help="PING も送る")
    args = ap.parse_args()

    print(f"# port={args.port} baud={args.baud} v{PROTOCOL_VERSION:#06x} 期待")
    try:
        ser = serial.Serial(args.port, args.baud, timeout=0.1)
    except serial.SerialException as e:
        print(f"ポートを開けない: {e}", file=sys.stderr)
        print("  → 配線・enable_uart・dialout 権限を確認", file=sys.stderr)
        return 2

    enc = FrameEncoder()
    parser = FrameParser(expect_types=S2P_TYPES)

    if not args.passive:
        ser.reset_input_buffer()
        ser.write(enc.encode(packets.VersionReq()))
        print("→ VERSION_REQ (0x14) 送信")
        if args.ping:
            ser.write(enc.encode(packets.Ping(ping_id=1)))
            print("→ PING (0x12) 送信")

    counts: Counter[int] = Counter()
    raw_total = 0
    first_raw = b""
    version_seen: packets.Version | None = None
    first_telemetry: packets.Telemetry | None = None
    lidar_sectors: set[int] = set()

    deadline = time.monotonic() + args.seconds
    while time.monotonic() < deadline:
        data = ser.read(max(1, ser.in_waiting))
        if not data:
            continue
        raw_total += len(data)
        if len(first_raw) < 64:
            first_raw += data
        for frame in parser.feed(data):
            counts[frame.type] += 1
            msg = frame.decode()
            if isinstance(msg, packets.Version) and version_seen is None:
                version_seen = msg
            elif isinstance(msg, packets.Telemetry) and first_telemetry is None:
                first_telemetry = msg
            elif isinstance(msg, packets.LidarSector):
                lidar_sectors.add(msg.sector_idx)

    ser.close()

    # ── レポート ──
    print(f"\n=== 受信 {raw_total} バイト / {args.seconds:.0f}秒 ===")
    if raw_total == 0:
        print("!! 1バイトも来ていない。")
        print("   - STM32 の電源は入っているか / TX-RX が正しく交差しているか")
        print("   - Pi TX(GPIO14) → STM32 RX、Pi RX(GPIO15) ← STM32 TX、GND 共通")
        return 1

    print(f"生バイト先頭: {hexdump(first_raw)}")

    if not counts:
        print("\n!! バイトは来ているがフレームを1つも復元できない。")
        st = parser.stats
        print(f"   crc_error={st.crc_error} len_error={st.len_error} "
              f"resync_bytes={st.resync_bytes}")
        print("   → ボーレート違い / CRC 実装差 / 別プロトコルの可能性。上の生バイトを確認。")
        return 1

    print("\nフレーム TYPE 別:")
    for t in sorted(counts):
        cls = packets.BY_TYPE.get(t)
        name = cls.NAME if cls else f"未知(0x{t:02X})"
        print(f"    0x{t:02X} {name:<16} × {counts[t]}")

    st = parser.stats
    print(f"\n統計: ok={st.frame_ok} crc_err={st.crc_error} len_err={st.len_error} "
          f"loss={st.packet_loss} unknown={st.unknown_type} resync={st.resync_bytes}")

    # バージョン照合
    print("\n--- バージョン照合 ---")
    if version_seen:
        pv = version_seen.protocol_version
        mark = "✓ 一致" if pv == PROTOCOL_VERSION else "✗ 不一致!"
        print(f"    STM32 protocol_version = 0x{pv:04X}  (Pi 期待 0x{PROTOCOL_VERSION:04X}) {mark}")
        print(f"    fw_id = 0x{version_seen.fw_id:08X}  build_epoch = {version_seen.build_epoch}")
        if pv != PROTOCOL_VERSION:
            print("    !! メジャー不一致なら通信を開始してはいけない（仕様 §8）")
    else:
        print("    VERSION が返っていない。--passive だと送信していないため。")
        print("    ファームが VERSION_REQ(0x14) に応答しない旧仕様の可能性もある。")

    if lidar_sectors:
        print(f"\nLiDAR: セクタ {sorted(lidar_sectors)} を受信"
              f"（{len(lidar_sectors)}/12）")

    if first_telemetry:
        print("\n--- 最初の TELEMETRY をデコード ---")
        print(fmt_telemetry(first_telemetry))
    else:
        print("\nTELEMETRY は受信していない。")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
