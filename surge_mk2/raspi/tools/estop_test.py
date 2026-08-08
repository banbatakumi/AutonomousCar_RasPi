"""E-Stop（第1安全層）が端から端まで効くことを実機で確認する。

    .venv/bin/python -m raspi.tools.estop_test

**この検査は実際に E-Stop をラッチさせる。** 解除には車両のボタン2を押す必要があるので、
**車両のそばにいるときだけ実行すること。** 検査の最後に解除まで見届ける。

ファーム・配線・Pi 側のどれかを触ったら回し直す。「波形は出ている」だけでは
安全機構が効いている証拠にならない。**STM32 が実際に反応することを確かめる。**

## 4段階で見る

1. **接続** — ハートビートを出し、`estop_active` が立たないこと（正常運転）
2. **途絶** — ハートビートを止め、`estop_active` が立つこと ＋ その所要時間
3. **自動復帰しないこと** — ハートビートを戻しても `estop_active` が立ったままであること。
   **ここが抜けていると「原因未解消のまま走り出す」事故になる**
4. **解除** — 人間がボタン2を押して初めて解除されること

期待値（`docs/uart_protocol.md` §9）: 途絶から 50ms でラッチ。TELEMETRY は 50Hz
（20ms 周期）なので、Pi から観測できるのは概ね 50〜90ms。
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from raspi.core.link_tracker import LinkTracker  # noqa: E402
from raspi.io.gpio import PIN_HEARTBEAT, Heartbeat, open_output  # noqa: E402
from raspi.io.serial_link import SerialLink  # noqa: E402
from raspi.proto import packets  # noqa: E402
from raspi.rec import FrameLogWriter, default_log_path  # noqa: E402

NS = 1_000_000_000
CMD_PERIOD_NS = NS // 100


class Rig:
    """検査中ずっと COMMAND(DISARM) を 100Hz で送りながら受信する。

    COMMAND を止めると第2安全層（`uart_timeout`）が絡んで結果が濁るので、
    **見たい第1安全層だけを切り分けるために送り続ける。**
    """

    def __init__(self, link: SerialLink, log: FrameLogWriter | None = None) -> None:
        self.link = link
        self.log = log
        self.latches: list[tuple[float, str, bool]] = []
        self.tracker = LinkTracker(on_latch=self._on_latch)
        self.state = self.tracker.state
        self._next_cmd = time.monotonic_ns()
        self.t0 = time.monotonic()

    def _on_latch(self, name: str, value: bool, t_ns: int) -> None:
        self.latches.append((time.monotonic(), name, value))
        if self.log:
            self.log.write_event(t_ns, "latch", {"flag": name, "value": value})

    def pump(self, seconds: float) -> None:
        """指定秒だけ送受信を回す。"""
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            self._pump_once()

    def wait_for(self, flag: str, value: bool, timeout_s: float) -> float | None:
        """フラグが目的の値になるまで回す。かかった秒数を返す（タイムアウトは None）。"""
        start = time.monotonic()
        while time.monotonic() - start < timeout_s:
            self._pump_once()
            if getattr(self.state, flag) == value:
                return time.monotonic() - start
        return None

    def _pump_once(self) -> None:
        now = time.monotonic_ns()
        for rx in self.link.poll():
            if self.log:
                self.log.write_rx(rx.rx_ns, rx.type, rx.seq, rx.payload)
            self.tracker.feed(rx.rx_ns, rx.type, rx.seq, rx.payload)
        if now >= self._next_cmd:
            self.link.send(packets.Command(mode=packets.Mode.DISARM, flags=0,
                                           target_speed=0, target_steer=0,
                                           accel_limit=0, steer_rate_limit=0))
            self._next_cmd = now + CMD_PERIOD_NS
        self.tracker.update_health(now)


def _flags(rig: Rig) -> str:
    t = rig.state.telemetry
    return f"flags=0x{t.flags:04X}" if t else "TELEMETRY 未受信"


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", default="/dev/serial0")
    ap.add_argument("--baud", type=int, default=250_000)
    ap.add_argument("--hold", type=float, default=3.0,
                    help="接続を確立するためにハートビートを出す秒数（既定 3.0）")
    ap.add_argument("--trip-timeout", type=float, default=3.0,
                    help="E-Stop が立つのを待つ上限 [s]")
    ap.add_argument("--release-timeout", type=float, default=90.0,
                    help="ボタン2が押されるのを待つ上限 [s]")
    ap.add_argument("--no-log", action="store_true", help="記録しない")
    ap.add_argument("--release-only", action="store_true",
                    help="検査せず、ハートビートを出してボタン2による解除だけ待つ"
                         "（ラッチしたまま終わったときの後片付け用）")
    args = ap.parse_args()

    try:
        link = SerialLink(args.port, args.baud)
    except Exception as e:
        print(f"ポートを開けない: {e}", file=sys.stderr)
        return 2

    pin = open_output(PIN_HEARTBEAT)
    if pin is None:
        print(f"GPIO{PIN_HEARTBEAT} を開けない。検査できない。", file=sys.stderr)
        link.close()
        return 2

    log = None
    if not args.no_log:
        log = FrameLogWriter(default_log_path("logs", prefix="estop"),
                            meta={"node": "estop_test", "port": args.port})
        link.on_tx = log.write_tx

    rig = Rig(link, log)
    hb = Heartbeat(pin, kick_timeout_s=None)   # 手動で止めたいので kick は要求しない
    hb2: Heartbeat | None = None

    #: 安全上の検査項目。**車両を E-Stop のまま終えたかどうかとは別に集計する。**
    #: 「安全機構は正しい」と「後片付けが済んでいない」は違う話なので混ぜない
    checks: dict[str, bool] = {}

    def head(n: int, title: str) -> None:
        print(f"\n{'─' * 62}\n[{n}/4] {title}\n{'─' * 62}")

    try:
        if args.release_only:
            # 後片付け専用。解除にはハートビートが出ている必要がある
            print("解除待ち（検査はしません）。ハートビートを出します。")
            hb.start()
            rig.pump(0.5)
            if not rig.state.estop_active:
                print(f"E-Stop は掛かっていません（{_flags(rig)}）。何もしません。")
                return 0
            print(f"**車両のボタン2を押してください。**"
                  f"（最大 {args.release_timeout:.0f}s 待ちます）", flush=True)
            dt = rig.wait_for("estop_active", False, args.release_timeout)
            if dt is None:
                print("!! 解除されませんでした。車両は E-Stop のままです。", file=sys.stderr)
                return 1
            print(f"✓ {dt:.1f}s で解除を確認（{_flags(rig)}）")
            return 0

        print("E-Stop 端から端まで検査")
        print("！ この検査は実際に E-Stop をラッチさせます。最後に解除まで行います。")

        # ── 0. まず現状を見る ──
        rig.pump(0.5)
        if rig.state.telemetry is None:
            print("\n!! TELEMETRY が来ない。STM32 の給電・配線を確認すること。",
                  file=sys.stderr)
            return 2
        if rig.state.estop_active:
            # 前回の検査などでラッチしたまま。ここで解除してもらうことが、
            # そのまま「段階4（人間の操作でのみ解除される）」の確認になる
            print(f"\n開始前から E-Stop がラッチしています（{_flags(rig)}）。")
            print("ハートビートを出します。**車両のボタン2を押して解除してください。**"
                  f"（最大 {args.release_timeout:.0f}s 待ちます）", flush=True)
            if log:
                log.write_event(time.monotonic_ns(), "phase",
                                {"n": 0, "await_button": True})
            hb.start()
            dt = rig.wait_for("estop_active", False, args.release_timeout)
            if dt is None:
                print(f"\n!! 解除されませんでした。**車両は E-Stop のまま**です。",
                      file=sys.stderr)
                return 2
            print(f"  ✓ {dt:.1f}s で解除を確認（{_flags(rig)}）"
                  " — 解除がボタン操作を要することも確認できた")
            checks["ボタン2で解除される"] = True
        else:
            print(f"開始時の状態: {_flags(rig)} estop=False  OK")

        # ── 1. 接続 ──
        head(1, f"ハートビートを {args.hold}s 出す（STM32 に接続と認めさせる）")
        if log:
            log.write_event(time.monotonic_ns(), "phase", {"n": 1, "hb": True})
        if not hb._running:
            hb.start()
        rig.pump(args.hold)
        print(f"  edges={hb.stats.edges} 最大遅れ={hb.stats.max_late_ns / 1e6:.2f}ms")
        print(f"  {_flags(rig)} estop={rig.state.estop_active}")
        good = not rig.state.estop_active
        checks["ハートビート中は正常運転"] = good
        print("  ✓ 正常運転（E-Stop なし）" if good
              else "  ✗ ハートビートを出しているのに E-Stop が立っている")

        # ── 2. 途絶 ──
        head(2, "ハートビートを止める → E-Stop がラッチするはず（期待 50〜90ms）")
        if log:
            log.write_event(time.monotonic_ns(), "phase", {"n": 2, "hb": False})
        hb.stop()
        t_stop = time.monotonic()
        dt = rig.wait_for("estop_active", True, args.trip_timeout)
        if dt is None:
            print(f"  ✗ {args.trip_timeout}s 待っても estop_active が立たない（{_flags(rig)}）")
            print("     → GPIO6 ↔ STM32 PB12 の配線か、ファームの監視を確認すること")
            print(f"\n  Pi 側は正常に出力していた: edges={hb.stats.edges} "
                  f"最大遅れ={hb.stats.max_late_ns / 1e6:.2f}ms")
            # ここで落ちた以上、段階3・4 は検査にならない。
            # E-Stop が一度も掛かっていないのに「解除を確認」と出すのは有害。
            print("\n段階3（自動復帰しないこと）と段階4（ボタンで解除）は、"
                  "\nE-Stop が発動していないため**実施しない**。")
            print(f"\n{'=' * 62}\n結果: **不合格** — 第1安全層が効いていない\n{'=' * 62}")
            return 1
        else:
            ms = dt * 1000
            print(f"  ✓ {ms:.0f}ms で E-Stop がラッチ（{_flags(rig)}）")
            checks["途絶でラッチする"] = True
            if ms > 200:
                print("  ! 期待より遅い（50〜90ms のはず）")

        # ── 3. 自動復帰しないこと ──
        head(3, "ハートビートを戻す → **それでも解除されない**はず")
        if log:
            log.write_event(time.monotonic_ns(), "phase", {"n": 3, "hb": True})
        pin2 = open_output(PIN_HEARTBEAT)
        if pin2 is None:
            print("  !! GPIO を開き直せない。段階3・4 は実施できない。", file=sys.stderr)
            return 2
        hb2 = Heartbeat(pin2, kick_timeout_s=None)
        hb2.start()
        rig.pump(1.5)
        latched = rig.state.estop_active
        checks["自動復帰しない"] = latched
        print(f"  ✓ ラッチ継続（{_flags(rig)}）。人間の操作なしには戻らない" if latched
              else f"  ✗ **自動復帰した**（{_flags(rig)}）。原因未解消のまま走り出す危険がある")

        # ── 4. 解除（＝後片付け） ──
        already = checks.get("ボタン2で解除される", False)
        head(4, "車両のボタン2を押して解除してください"
                + ("（開始時にも押してもらったので**2回目**です）" if already else ""))
        if log:
            log.write_event(time.monotonic_ns(), "phase", {"n": 4, "await_button": True})
        print(f"  最大 {args.release_timeout:.0f}s 待ちます…", flush=True)
        dt = rig.wait_for("estop_active", False, args.release_timeout)
        left_latched = dt is None
        if left_latched:
            print(f"  — {args.release_timeout:.0f}s 以内に解除されませんでした。")
        else:
            print(f"  ✓ {dt:.1f}s 後に解除を確認（{_flags(rig)}）")
            checks["ボタン2で解除される"] = True

        # ── 結果 ──
        # 安全項目の合否と、車両を E-Stop のまま残したかは別物として出す。
        # ここを混ぜると「安全機構が壊れている」と「押し忘れ」が区別できない
        print(f"\n{'=' * 62}")
        for name, good in checks.items():
            print(f"  {'✓' if good else '✗'} {name}")
        ok = bool(checks) and all(checks.values()) and len(checks) == 4
        if not checks.get("ボタン2で解除される"):
            print("  ✗ ボタン2で解除される（未確認）")
            ok = False
        print("\n結果: " + ("**合格** — 第1安全層は端から端まで効いている"
                           if ok else "**不合格** — 上の ✗ を確認すること"))
        if left_latched:
            print("\n!! ただし**車両は E-Stop がラッチしたまま**です。"
                  "\n   ボタン2を押して解除してください（`--release-only` でも待てます）")
        print("=" * 62)
        return 0 if ok and not left_latched else 1

    finally:
        for h in (hb, hb2):
            if h is not None:
                try:
                    h.stop()
                except Exception:
                    pass
        # 検査で E-Stop を掛けた以上、最後に停止指令を送っておく
        try:
            link.send(packets.Command(mode=packets.Mode.DISARM, flags=0,
                                      target_speed=0, target_steer=0,
                                      accel_limit=0, steer_rate_limit=0))
        except Exception:
            pass
        if log:
            log.close()
            print(f"記録: {log.path}")
        link.close()


if __name__ == "__main__":
    raise SystemExit(main())
