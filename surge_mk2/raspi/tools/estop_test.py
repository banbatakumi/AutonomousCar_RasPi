"""E-Stop（第1安全層）が端から端まで効くことを実機で確認する。

    .venv/bin/python -m raspi.tools.estop_test                  # DISARM のみ（arm 経路は未検査）
    .venv/bin/python -m raspi.tools.estop_test --wheels-clear   # arm 経路まで検査（推奨）
    .venv/bin/python -m raspi.tools.estop_test --wheels-clear --disarm-before-trip

**この検査は実際に E-Stop をラッチさせる。** 解除には車両のボタン2を押す必要があるので、
**車両のそばにいるときだけ実行すること。** 検査の最後に解除まで見届ける。

ファーム・配線・Pi 側のどれかを触ったら回し直す。「波形は出ている」だけでは
安全機構が効いている証拠にならない。**STM32 が実際に反応することを確かめる。**

## 6段階で見る（`--wheels-clear` 無しなら 1・3・4・6 の4段階）

1. **接続** — ハートビートを出し、`estop_active` が立たないこと（正常運転）
2. **arm** — 駆動電源を入れる。**E-Stop が守るべき状態を実際に作る**
3. **途絶** — ハートビートを止め、`estop_active` が立つこと ＋ その所要時間。
   arm していれば、**駆動電源が維持されたまま制動に入ること**もここで見る。
   `armed` は立ったままが正常（電源を切ると MD が制動できず惰行するため）
4. **自動復帰しないこと** — ハートビートを戻しても `estop_active` が立ったままであること。
   **ここが抜けていると「原因未解消のまま走り出す」事故になる**
5. **ラッチ中は arm できないこと** — E-Stop 中に `arm=1` を送り続けても
   `armed` が立たないこと。
   **ここが破れていると、E-Stop 中に GUI の ARM ボタンで駆動が復活する**
6. **解除** — 人間がボタン2を押して初めて解除されること

期待値（`docs/uart_protocol.md` §9）: 途絶から 50ms でラッチ。TELEMETRY は 50Hz
（20ms 周期）なので、Pi から観測できるのは概ね 50〜90ms。

## ★ 2回に分けて回すこと（2026-08-10 に判明）

**ラッチ中は arm 状態が凍結する。** arm したまま発動させると、`arm=0` を送っても
`armed` は落ちない（COMMAND が一切効かないため。仕様どおり）。
つまり **arm したままの1回では「0→1 が拒否されるか」を観測できない。**
危険なのはまさにその向き（E-Stop 中に GUI の ARM ボタンを押す）なので、
2回に分けて両方を埋める:

```
① .venv/bin/python -m raspi.tools.estop_test --wheels-clear
     → ARM 中でも発動する / 駆動電源が維持される
② .venv/bin/python -m raspi.tools.estop_test --wheels-clear --disarm-before-trip
     → ラッチ中に arm=1 が拒否される
```

## なぜ arm 経路を足したか

`io_node --allow-arm`（2026-08-08 追加）で駆動電源を入れられるようになった。
**DISARM のままの検査では、E-Stop が本当に守るべき状態を一度も作っていない。**
段階2・5 はそのための追加。`--wheels-clear` を付けないと実施せず、
最後に「arm 経路は未検査」と明示して終わる。
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

#: 目標速度0を送っているのに駆動トルク指令がこれを超えたら異常（0.0001 N.m 単位）。
#: `arm_test.py` と同じ閾値にしてある
TORQUE_LIMIT = 10
#: 異常がこれだけ継続したら降りる。comm_ok 復帰直後の古い値で誤検出しないため
VIOLATION_HOLD_S = 0.15

#: 段階2 で駆動電源が入ったとみなす電圧 [V]
DRIVE_ON_V = 5.0


class Aborted(Exception):
    """検査を続けると危険な状態になったので降りる。"""


class Rig:
    """検査中ずっと `COMMAND` を 100Hz で送りながら受信する。

    COMMAND を止めると第2安全層（`uart_timeout`）が絡んで結果が濁るので、
    **見たい第1安全層だけを切り分けるために送り続ける。**

    送る中身は `set_cmd()` で切り替える。既定は DISARM。
    """

    def __init__(self, link: SerialLink, log: FrameLogWriter | None = None) -> None:
        self.link = link
        self.log = log
        self.latches: list[tuple[float, str, bool]] = []
        self.tracker = LinkTracker(on_latch=self._on_latch)
        self.state = self.tracker.state
        self._next_cmd = time.monotonic_ns()
        self.t0 = time.monotonic()
        # 送信内容。**既定は DISARM**（明示的に上げない限り駆動は入らない）
        self._arm = False
        self._mode = packets.Mode.DISARM
        self._steer = 0
        # 動きの監視
        self._watch = False
        self._bad_since: float | None = None
        self.max_torque = 0

    # ── 送る中身 ──

    def set_cmd(self, *, arm: bool = False, mode: int = packets.Mode.DISARM,
                steer: int = 0, watch: bool = False) -> None:
        """以後 `pump`/`wait_for` が送る `COMMAND` を差し替える。

        :param watch: 目標速度0なのに駆動トルクが出ていないかを監視するか。
            **途絶させたあとは False にする**（E-Stop の制動トルクで必ず引っかかるため）
        """
        self._arm = arm
        self._mode = mode
        self._steer = steer
        self._watch = watch
        self._bad_since = None

    def set_watch(self, watch: bool) -> None:
        """送る中身は変えずに、駆動トルクの監視だけ切り替える。"""
        self._watch = watch
        self._bad_since = None

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

    def wait_armed(self, value: bool, timeout_s: float) -> float | None:
        """`TELEMETRY.flags` の `armed` が目的の値になるまで回す。"""
        start = time.monotonic()
        while time.monotonic() - start < timeout_s:
            self._pump_once()
            if self.armed == value:
                return time.monotonic() - start
        return None

    def _pump_once(self) -> None:
        now = time.monotonic_ns()
        for rx in self.link.poll():
            if self.log:
                self.log.write_rx(rx.rx_ns, rx.type, rx.seq, rx.payload)
            self.tracker.feed(rx.rx_ns, rx.type, rx.seq, rx.payload)
        if now >= self._next_cmd:
            self.link.send(packets.Command(
                mode=self._mode,
                flags=packets.CMD_FLG_ARM if self._arm else 0,
                target_speed=0,              # **常にゼロ。この検査で速度は一切出さない**
                target_steer=self._steer,
                accel_limit=0, steer_rate_limit=0))
            self._next_cmd = now + CMD_PERIOD_NS
        self.tracker.update_health(now)
        self._check()

    # ── 状態の読み ──

    @property
    def armed(self) -> bool:
        t = self.state.telemetry
        return bool(t and (t.flags & packets.FLG_ARMED))

    @property
    def drive_v(self) -> float:
        t = self.state.telemetry
        return t.batt_voltage_drive * 0.05 if t else 0.0

    def _check(self) -> None:
        """目標速度0を送っているのに駆動トルクが出ていないか。

        **`watch` が立っている間だけ見る。** 途絶後は E-Stop が最大制動トルクを
        掛けるので（v0.5）、そこで見ると必ず引っかかる。
        """
        t = self.state.telemetry
        if t is None:
            return
        if t.flags & packets.FLG_DRIVE_POWER_LOCKED:
            raise Aborted("駆動電源がラッチ遮断された（過電流）")
        if t.flags & packets.FLG_FAULT_DRIVE_OVERCURRENT:
            raise Aborted("駆動系の過電流")
        if not self._watch:
            return
        worst = max(abs(t.torque_cmd[0]), abs(t.torque_cmd[1]))
        self.max_torque = max(self.max_torque, worst)
        now = time.monotonic()
        if worst <= TORQUE_LIMIT:
            self._bad_since = None
            return
        # **単発の過渡値では止めない。** comm_ok 復帰直後は MD の古い値が数フレーム残る
        if self._bad_since is None:
            self._bad_since = now
            return
        if now - self._bad_since >= VIOLATION_HOLD_S:
            raise Aborted(f"目標速度0なのに駆動トルク指令が出た"
                          f"（{worst * 1e-4:+.4f} N.m が {VIOLATION_HOLD_S * 1000:.0f}ms 継続）")


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
    ap.add_argument("--wheels-clear", action="store_true",
                    help="車輪が接地しておらず動いても安全であることの確認。"
                         "**付けると arm 経路（段階2・5）まで検査する**")
    ap.add_argument("--manual", action="store_true",
                    help="段階2 で mode=MANUAL まで上げる（既定は arm=1 + DISARM）。"
                         "--wheels-clear が必要")
    ap.add_argument("--disarm-before-trip", action="store_true",
                    help="段階2 で arm を確認したあと**降ろしてから**途絶させる。"
                         "ラッチ中に arm=1 を送って拒否されることを検査できる"
                         "（ラッチ中は arm 状態が凍結するので、この向きは"
                         "arm したまま発動させると試せない）。--wheels-clear が必要")
    ap.add_argument("--release-only", action="store_true",
                    help="検査せず、ハートビートを出してボタン2による解除だけ待つ"
                         "（ラッチしたまま終わったときの後片付け用）")
    args = ap.parse_args()

    if args.manual and not args.wheels_clear:
        print("!! --manual には --wheels-clear が要ります。", file=sys.stderr)
        return 2
    if args.disarm_before_trip and not args.wheels_clear:
        print("!! --disarm-before-trip には --wheels-clear が要ります。", file=sys.stderr)
        return 2

    arm_path = args.wheels_clear
    n_stages = 6 if arm_path else 4

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
                             meta={"node": "estop_test", "port": args.port,
                                   "arm_path": arm_path, "manual": args.manual})
        link.on_tx = log.write_tx

    rig = Rig(link, log)
    hb = Heartbeat(pin, kick_timeout_s=None)   # 手動で止めたいので kick は要求しない
    hb2: Heartbeat | None = None

    #: 安全上の検査項目。**車両を E-Stop のまま終えたかどうかとは別に集計する。**
    #: 「安全機構は正しい」と「後片付けが済んでいない」は違う話なので混ぜない
    checks: dict[str, bool] = {}
    #: 実施しなかった検査。**合格と混ぜず、名指しで残す**
    skipped: list[str] = []

    def head(n: int, title: str) -> None:
        print(f"\n{'─' * 62}\n[{n}/{n_stages}] {title}\n{'─' * 62}")

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
        if arm_path:
            print("！ **arm 経路まで検査します。駆動電源が入ります。**"
                  "車輪が浮いていることを確認してください。")
        else:
            print("※ arm 経路（段階2・5）は検査しません。"
                  "**駆動電源が入った状態での E-Stop は未検査のままになります。**"
                  "\n   検査するには車輪を浮かせて --wheels-clear を付けてください。")

        # ── 0. まず現状を見る ──
        rig.pump(0.5)
        if rig.state.telemetry is None:
            print("\n!! TELEMETRY が来ない。STM32 の給電・配線を確認すること。",
                  file=sys.stderr)
            return 2
        if rig.state.estop_active:
            # 前回の検査などでラッチしたまま。ここで解除してもらうことが、
            # そのまま「段階6（人間の操作でのみ解除される）」の確認になる
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

        # ── 2. arm（--wheels-clear のときだけ） ──
        armed_for_trip = False     # 途絶の瞬間に armed だったか
        arm_works = False          # そもそも arm できたか（段階5 の前提）
        hold_steer = 0
        if arm_path:
            mode = packets.Mode.MANUAL if args.manual else packets.Mode.DISARM
            mode_name = "MANUAL" if args.manual else "DISARM"
            head(2, f"arm=1 / mode={mode_name} を送る（**駆動電源が入る**。目標速度は0）")
            if log:
                log.write_event(time.monotonic_ns(), "phase",
                                {"n": 2, "arm": True, "mode": int(mode)})
            # **舵角は今の位置を保持する。** 中央へ戻す動きを起こさないため
            hold_steer = rig.state.telemetry.steer_actual
            print(f"  舵角 {hold_steer * 1e-4:+.4f} rad を保持する指令にします")
            rig.set_cmd(arm=True, mode=mode, steer=hold_steer, watch=True)
            dt = rig.wait_armed(True, 5.0)
            rig.pump(1.5)          # 立ってから少し観測する（トルクが出ないこと）
            if dt is None:
                # arm できないなら段階3の「armed が落ちる」も段階5も検査にならない。
                # **合格にはせず、実施しなかったと残す**
                print(f"  ✗ 5s 待っても armed が立たない（{_flags(rig)} "
                      f"駆動 {rig.drive_v:.2f}V）")
                print("     → 駆動電源の元スイッチ、drive_power_locked、"
                      "STM32 側の arm 拒否条件を確認すること")
                skipped.append("ARM 中の E-Stop（arm できなかったため未実施）")
                skipped.append("ラッチ中は arm できない（arm できなかったため未実施）")
            else:
                armed_for_trip = True
                arm_works = True
                print(f"  ✓ {dt * 1000:.0f}ms で armed（{_flags(rig)} "
                      f"駆動 {rig.drive_v:.2f}V）")
                print(f"  駆動トルク指令の最大 {rig.max_torque * 1e-4:+.4f} N.m"
                      "（目標速度0なのでゼロのはず）")
                if args.disarm_before_trip:
                    # **降ろしてから途絶させる。** ラッチ中は arm 状態が凍結するので、
                    # 「ラッチ中に arm=1 が拒否されるか」はこの順でしか試せない。
                    # ここで落ちること自体が「ラッチしていなければ arm=0 は効く」の対照になる
                    print("  arm=0 を送って降ろします（途絶は降ろした状態で起こす）…",
                          flush=True)
                    rig.set_cmd(arm=False, mode=packets.Mode.DISARM, steer=hold_steer)
                    d0 = rig.wait_armed(False, 3.0)
                    if d0 is None:
                        print(f"  ✗ arm=0 を送っても armed が落ちない（{_flags(rig)}）。"
                              "\n     ラッチしていないのに降ろせないのは異常")
                        checks["ラッチ外なら arm=0 で降ろせる"] = False
                    else:
                        armed_for_trip = False
                        checks["ラッチ外なら arm=0 で降ろせる"] = True
                        print(f"  ✓ {d0 * 1000:.0f}ms で armed が落ちた"
                              f"（駆動 {rig.drive_v:.2f}V）")
        else:
            skipped.append("ARM 中の E-Stop（--wheels-clear 無しのため未実施）")
            skipped.append("ラッチ中は arm できない（--wheels-clear 無しのため未実施）")

        # ── 3. 途絶 ──
        head(3, "ハートビートを止める → E-Stop がラッチするはず（期待 50〜90ms）")
        if log:
            log.write_event(time.monotonic_ns(), "phase", {"n": 3, "hb": False})
        # **途絶後は駆動トルクの監視を切る。** v0.5 の E-Stop は最大制動トルクを
        # 直接掛けるので、監視したままだと「制動している」ことで中断してしまう。
        # arm 指令は送り続ける — E-Stop が COMMAND に優先することを見たいため
        rig.set_watch(False)
        hb.stop()
        dt = rig.wait_for("estop_active", True, args.trip_timeout)
        if dt is None:
            print(f"  ✗ {args.trip_timeout}s 待っても estop_active が立たない（{_flags(rig)}）")
            print("     → GPIO6 ↔ STM32 PB12 の配線か、ファームの監視を確認すること")
            print(f"\n  Pi 側は正常に出力していた: edges={hb.stats.edges} "
                  f"最大遅れ={hb.stats.max_late_ns / 1e6:.2f}ms")
            # ここで落ちた以上、以降の段階は検査にならない。
            # E-Stop が一度も掛かっていないのに「解除を確認」と出すのは有害。
            print(f"\n以降の段階（自動復帰しない／ラッチ中は arm できない／ボタンで解除）は、"
                  "\nE-Stop が発動していないため**実施しない**。")
            print(f"\n{'=' * 62}\n結果: **不合格** — 第1安全層が効いていない\n{'=' * 62}")
            return 1
        ms = dt * 1000
        print(f"  ✓ {ms:.0f}ms で E-Stop がラッチ（{_flags(rig)}）")
        checks["途絶でラッチする"] = True
        if ms > 200:
            print("  ! 期待より遅い（50〜90ms のはず）")

        # arm した状態で発動させられたこと自体が、この検査で足した価値。
        # **`armed` が落ちることを期待してはいけない。**
        # §9 の仕様は「駆動電源は切らない」— 切ると MD が制動をかけられず惰行して
        # 停止距離が伸びるため。つまり E-Stop 中も armed が立っているのが**正常**
        if armed_for_trip:
            checks["ARM 中でも途絶でラッチする"] = True
            rig.pump(0.5)
            t = rig.state.telemetry
            tq = max(abs(t.torque_cmd[0]), abs(t.torque_cmd[1])) * 1e-4
            print(f"  発動後: armed={rig.armed} 駆動 {rig.drive_v:.2f}V "
                  f"制動トルク {tq:.4f} N.m")
            print("    ※ armed が立ったままなのは**仕様どおり**"
                  "（駆動電源を切ると MD が制動できず惰行するため）")
            kept = rig.drive_v > DRIVE_ON_V
            checks["E-Stop 中も駆動電源が維持される"] = kept
            print("  ✓ 駆動電源は維持されている（制動をかけられる状態）" if kept
                  else "  ✗ **駆動電源が落ちた**。MD が制動できず惰行する恐れ")

        # ── 4. 自動復帰しないこと ──
        head(4, "ハートビートを戻す → **それでも解除されない**はず")
        if log:
            log.write_event(time.monotonic_ns(), "phase", {"n": 4, "hb": True})
        pin2 = open_output(PIN_HEARTBEAT)
        if pin2 is None:
            print("  !! GPIO を開き直せない。以降の段階は実施できない。", file=sys.stderr)
            return 2
        hb2 = Heartbeat(pin2, kick_timeout_s=None)
        hb2.start()
        rig.pump(1.5)
        latched = rig.state.estop_active
        checks["自動復帰しない"] = latched
        print(f"  ✓ ラッチ継続（{_flags(rig)}）。人間の操作なしには戻らない" if latched
              else f"  ✗ **自動復帰した**（{_flags(rig)}）。原因未解消のまま走り出す危険がある")

        # ── 5. ラッチ中は arm できないこと ──
        if arm_path and arm_works and not armed_for_trip:
            # **これが本命。** 降ろした状態で発動させたので、
            # 「E-Stop 中に GUI の ARM ボタンを押したら駆動が復活するか」を直接試せる
            head(5, "E-Stop ラッチ中に arm=1 を送り続ける → **armed は立たないはず**")
            if log:
                log.write_event(time.monotonic_ns(), "phase", {"n": 5, "arm": True})
            print("  仕様: estop_active 中は COMMAND が一切効かない"
                  "（uart_protocol.md §9）")
            print("  arm=1 / mode=MANUAL を 3s 送り続けます…", flush=True)
            rig.set_cmd(arm=True, mode=packets.Mode.MANUAL, steer=hold_steer)
            d2 = rig.wait_armed(True, 3.0)
            refused = d2 is None
            checks["ラッチ中は arm できない"] = refused
            print(f"  ✓ 3s 送り続けても armed は立たなかった"
                  f"（{_flags(rig)} 駆動 {rig.drive_v:.2f}V）" if refused else
                  f"  ✗ **E-Stop 中に arm が通った**（{d2 * 1000:.0f}ms 後 / "
                  f"駆動 {rig.drive_v:.2f}V）。"
                  "\n     GUI の ARM ボタンで駆動が復活してしまう")
            rig.set_cmd(arm=False, mode=packets.Mode.DISARM, steer=0)

        elif arm_path and armed_for_trip:
            # arm したまま発動させた場合。**ラッチ中は arm 状態が凍結する**ので、
            # 「立たないこと」を観測できない。arm=0 すら効かないことを確かめて、
            # 「COMMAND が一切効かない」の傍証として残す（合否には数えない）
            head(5, "E-Stop ラッチ中に COMMAND が効かないこと（arm=0 を送ってみる）")
            if log:
                log.write_event(time.monotonic_ns(), "phase", {"n": 5, "arm": False})
            print("  arm したまま発動させたので、arm 状態は凍結している。"
                  "\n  **0→1 の向き（GUI の ARM ボタン相当）はこの順では試せない。**"
                  "\n  試すには --disarm-before-trip を付けて回し直すこと。", flush=True)
            rig.set_cmd(arm=False, mode=packets.Mode.DISARM, steer=hold_steer)
            d = rig.wait_armed(False, 3.0)
            if d is None:
                print(f"  → arm=0 を送っても armed は落ちなかった"
                      f"（{_flags(rig)} 駆動 {rig.drive_v:.2f}V）。"
                      "\n     COMMAND が一切効いていない＝仕様どおりの傍証")
            else:
                print(f"  ! arm=0 が {d * 1000:.0f}ms で効いた"
                      f"（駆動 {rig.drive_v:.2f}V）。"
                      "\n     ラッチ中でも arm=0 は通るということ。"
                      "0→1 も通らないか --disarm-before-trip で確認すること")
            skipped.append("ラッチ中は arm できない（arm 0→1 はこの順では試せない。"
                           "--disarm-before-trip で回すこと）")
        elif arm_path:
            head(5, "ラッチ中の arm 拒否 — **実施しない**（段階2 で arm できなかったため）")

        # ── 6. 解除（＝後片付け） ──
        already = checks.get("ボタン2で解除される", False)
        head(n_stages, "車両のボタン2を押して解除してください"
             + ("（開始時にも押してもらったので**2回目**です）" if already else ""))
        if log:
            log.write_event(time.monotonic_ns(), "phase",
                            {"n": n_stages, "await_button": True})
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
        if not checks.get("ボタン2で解除される"):
            print("  ✗ ボタン2で解除される（未確認）")
            checks["ボタン2で解除される"] = False
        for name in skipped:
            print(f"  — {name}")

        required = ["ハートビート中は正常運転", "途絶でラッチする", "自動復帰しない",
                    "ボタン2で解除される"]
        if armed_for_trip:
            required += ["ARM 中でも途絶でラッチする", "E-Stop 中も駆動電源が維持される"]
        # 実施できた検査だけを必須に足す。**実施しなかったものは skipped に出る**
        for name in ("ラッチ外なら arm=0 で降ろせる", "ラッチ中は arm できない"):
            if name in checks:
                required.append(name)
        ok = all(checks.get(n, False) for n in required)

        if ok and skipped:
            # **合格と言い切らない。** 未実施を「効いている」と読み替えられないように
            print("\n結果: **条件付き合格** — 実施した検査はすべて通ったが、"
                  "\n      上の — は実施していない。arm 経路は未検査のまま。")
        else:
            print("\n結果: " + ("**合格** — 第1安全層は端から端まで効いている"
                               if ok else "**不合格** — 上の ✗ を確認すること"))
        if left_latched:
            print("\n!! ただし**車両は E-Stop がラッチしたまま**です。"
                  "\n   ボタン2を押して解除してください（`--release-only` でも待てます）")
        print("=" * 62)
        return 0 if ok and not left_latched else 1

    except Aborted as e:
        print(f"\n!! 中断: {e}", file=sys.stderr)
        print("   安全側に倒して検査を打ち切りました。", file=sys.stderr)
        return 1

    finally:
        for h in (hb, hb2):
            if h is not None:
                try:
                    h.stop()
                except Exception:
                    pass
        # 検査で E-Stop を掛けた以上、最後に停止指令を送っておく。
        # **arm を上げたなら確実に降ろす**ため複数回送る
        for _ in range(10):
            try:
                link.send(packets.Command(mode=packets.Mode.DISARM, flags=0,
                                          target_speed=0, target_steer=0,
                                          accel_limit=0, steer_rate_limit=0))
            except Exception:
                break
            time.sleep(0.01)
        if log:
            log.close()
            print(f"記録: {log.path}")
        link.close()


if __name__ == "__main__":
    raise SystemExit(main())
