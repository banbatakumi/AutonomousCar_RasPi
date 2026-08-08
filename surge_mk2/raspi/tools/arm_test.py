"""arm を通す最小実験。**車輪が動きうる。車両を浮かせてから実行すること。**

    .venv/bin/python -m raspi.tools.arm_test --wheels-clear

`--wheels-clear` は「車輪が接地しておらず、動いても安全」の確認。
付けないと実行しない。

## この実験で確かめること

Pi が `arm=0` を送ると STM32 が駆動電源を切る（2026-08-08 に判明）。
では **`arm=1` なら電源が維持されるのか**、そのとき車両は動かずにいられるのか。

## 安全のための作り

- **E-Stop ハートビートを必須にする。** GPIO が開けなければ実行しない。
  実験中にこのプロセスが死ねば 150ms 以内に STM32 が制動をかける
- **段階を分ける。** いきなり MANUAL にせず、まず `arm=1` + `mode=DISARM` を試す
- **動いたら即座に降りる。** 速度・車輪速が閾値を超えたら自動で DISARM して終了
- **舵角は今の位置を保持する指令にする。** 中央に戻す動きを起こさないため
- 全フレームを `.sfl` に記録する

**終了時は必ず DISARM を送り、ハートビートを止める**（＝E-Stop がラッチする）。
解除には車両のボタン2が要る。
"""

from __future__ import annotations

import argparse
import signal
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from raspi.core.link_tracker import LinkTracker  # noqa: E402
from raspi.io.gpio import PIN_HEARTBEAT, Heartbeat, open_output  # noqa: E402
from raspi.io.serial_link import SerialLink  # noqa: E402
from raspi.proto import packets as P  # noqa: E402
from raspi.rec import FrameLogWriter, default_log_path  # noqa: E402

CMD_PERIOD_NS = 10_000_000          # 100Hz

#: 後輪（駆動輪）がこれを超えて回ったら降りる。
#: **前輪は見ない。** 前輪は駆動されないエンコーダ輪で、台に乗せると自由に回るため
#: （`wheel_speed` は [FL, FR, RL, RR]。`speed` も前輪由来なので判定に使わない）
REAR_WHEEL_MPS = 0.10
#: 駆動トルク指令。速度0を出しているのに STM32 がトルクを出していたら異常
TORQUE_LIMIT = 10          # 0.0001 N.m 単位 → 0.001 N.m
#: 異常がこれだけ継続したら降りる。単発の過渡値で止めないため
#: （comm_ok 復帰直後は MD の古い値が 1〜2フレーム残る）
VIOLATION_HOLD_S = 0.15
IDX_RL, IDX_RR = 2, 3


class Aborted(Exception):
    pass


class Rig:
    def __init__(self, link: SerialLink, log: FrameLogWriter, hb: Heartbeat) -> None:
        self.link = link
        self.log = log
        self.hb = hb
        self.tr = LinkTracker()
        self.state = self.tr.state
        self._next_cmd = time.monotonic_ns()
        self._bad_since: float | None = None
        self.abort_reason: str | None = None

    # ── 送信 ──

    def _send(self, *, arm: bool, mode: int, steer: int) -> None:
        self.link.send(P.Command(
            mode=mode,
            flags=P.CMD_FLG_ARM if arm else 0,
            target_speed=0,                      # **常にゼロ。速度は一切出さない**
            target_steer=steer,
            accel_limit=0, steer_rate_limit=0))

    def pump(self, seconds: float, *, arm: bool, mode: int, steer: int,
             label: str, check_motion: bool = True,
             watch_wheels: bool = False) -> None:
        """指定秒だけ指令を送り続ける。異常があれば Aborted。

        :param check_motion: 動きの検査をするか。**静止待ちでは False**。
            待っている条件そのもので中断してしまうため
        """
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            now = time.monotonic_ns()
            self.hb.kick()
            for rx in self.link.poll():
                self.log.write_rx(rx.rx_ns, rx.type, rx.seq, rx.payload)
                self.tr.feed(rx.rx_ns, rx.type, rx.seq, rx.payload)
            if now >= self._next_cmd:
                self._send(arm=arm, mode=mode, steer=steer)
                self._next_cmd = now + CMD_PERIOD_NS
            self.tr.update_health(now)
            self._check(label, check_motion, watch_wheels)

    @staticmethod
    def rear_valid(t) -> bool:
        """後輪の速度を信用してよいか。

        **`comm_ok` が落ちている間の `wheel_speed[RL/RR]` は無効値。**
        MD から来る値なので、通信が死んでいれば古い値かゴミが残る。
        駆動電源が切れている間は必ずこの状態になる。
        """
        return bool(t.md_status[0] & P.MDS_COMM_OK) and \
            bool(t.md_status[1] & P.MDS_COMM_OK)

    def _violation(self, t, motion: bool, watch_wheels: bool) -> str | None:
        """今このフレームで「まずい」と言える状態か。理由文字列を返す。"""
        if not motion:
            return None
        # **速度0を指令しているのに駆動トルクが出ている** = 本命の判定。
        # torque_cmd は STM32 自身が計算した指令値なので MD 通信に依存せず常に有効
        for i, name in ((0, "RL"), (1, "RR")):
            if abs(t.torque_cmd[i]) > TORQUE_LIMIT:
                return f"{name} に駆動トルク指令が出た（{t.torque_cmd[i]*1e-4:+.4f} N.m）"
        # 車輪速度は **MANUAL のときだけ**見る。
        # DISARM 中は駆動されないので、輪が回っていても惰性か外力であって危険ではない。
        # また comm_ok が無い間の値は無効なので除く
        if watch_wheels and self.rear_valid(t):
            for i, name in ((IDX_RL, "RL"), (IDX_RR, "RR")):
                w = t.wheel_speed[i]
                if abs(w) * 1e-3 > REAR_WHEEL_MPS:
                    return f"駆動輪 {name} が回った（{w*1e-3:+.3f} m/s）"
        return None

    def _check(self, label: str, motion: bool = True,
               watch_wheels: bool = False) -> None:
        t = self.state.telemetry
        if t is None:
            return
        f = t.flags
        if f & P.FLG_ESTOP_ACTIVE:
            raise Aborted(f"{label}: E-Stop が発動した")
        if f & P.FLG_DRIVE_POWER_LOCKED:
            raise Aborted(f"{label}: 駆動電源がラッチ遮断された")
        if f & P.FLG_FAULT_DRIVE_OVERCURRENT:
            raise Aborted(f"{label}: 駆動系の過電流")
        if f & P.FLG_FAULT_SIGNAL_OVERCURRENT:
            raise Aborted(f"{label}: シグナル系の過電流")

        why = self._violation(t, motion, watch_wheels)
        now = time.monotonic()
        if why is None:
            self._bad_since = None
            return
        # **単発の過渡値では止めない。** comm_ok 復帰直後は MD の古い値が
        # 1〜2フレーム残る。継続していることを確かめてから降りる
        if self._bad_since is None:
            self._bad_since = now
            return
        if now - self._bad_since >= VIOLATION_HOLD_S:
            raise Aborted(f"{label}: {why}（{VIOLATION_HOLD_S*1000:.0f}ms 継続）")

    def wait_still(self, timeout_s: float = 20.0, thresh: float = 0.05,
                   *, arm: bool, mode: int, steer: int) -> float:
        """車体が止まるまで待つ。止まった時点の `speed` [m/s] を返す。

        **台上では前輪（センシング）と後輪（駆動）が機械的に切り離される。**
        前輪が惰性で回っていると `speed` が非ゼロになり、MANUAL の速度制御が
        「減速しろ」と後輪に制動トルクを出す。幻の速度と戦うことになるので、
        MANUAL に入る前に静止を確認する。

        **判定に使うのは `speed` であって `wheel_speed[]` ではない。**
        `wheel_speed[FL,FR]` は粗い絶対角エンコーダの微分で、静止中でも
        ±12 m/s のスパイクが 7割のサンプルに出る（2026-08-08 実測）。
        `speed` は STM32 側でローパスを通した値で、静止時 ±0.026 m/s に収まる。
        """
        end = time.monotonic() + timeout_s
        worst = 9.9
        while time.monotonic() < end:
            # **動きの検査は切る。** 待っている条件そのもので中断してしまう
            self.pump(0.3, arm=arm, mode=mode, steer=steer, label="静止待ち",
                      check_motion=False)
            worst = abs(self.state.telemetry.speed) * 1e-3
            if worst < thresh:
                return worst
        return worst

    # ── 表示 ──

    def show(self, label: str) -> None:
        t = self.state.telemetry
        if t is None:
            print(f"  {label}: TELEMETRY 未受信")
            return
        f = t.flags
        marks = [n for n, b in (("ARMED", P.FLG_ARMED),
                                ("駆動低電圧", P.FLG_FAULT_DRIVE_UNDERVOLTAGE),
                                ("E-STOP", P.FLG_ESTOP_ACTIVE),
                                ("ラッチ", P.FLG_DRIVE_POWER_LOCKED)) if f & b]
        md = [f"0x{x:02X}" for x in t.md_status]
        rear = (f"{t.wheel_speed[IDX_RL]*1e-3:+.3f}/{t.wheel_speed[IDX_RR]*1e-3:+.3f} m/s"
                if self.rear_valid(t) else "（comm_ok 無し＝無効値）")
        print(f"  {label:<22} 駆動 {t.batt_voltage_drive*0.05:5.2f}V  後輪 {rear}  "
              f"トルク {t.torque_cmd[0]*1e-4:+.4f}/{t.torque_cmd[1]*1e-4:+.4f} N.m")
        print(f"  {'':<22} 舵角 {t.steer_actual*1e-4:+.4f} rad  "
              f"前輪 {t.wheel_speed[0]*1e-3:+.3f}/{t.wheel_speed[1]*1e-3:+.3f} m/s(参考)  "
              f"MD {' '.join(md)}  0x{f:04X} {' '.join(marks)}")


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--wheels-clear", action="store_true",
                    help="車輪が接地しておらず動いても安全であることの確認（必須）")
    ap.add_argument("--manual", action="store_true",
                    help="段階3（mode=MANUAL）まで進める")
    ap.add_argument("--hold", type=float, default=4.0, help="各段階の秒数")
    args = ap.parse_args()

    if not args.wheels_clear:
        print("!! 車輪が動いても安全なことを確認し、--wheels-clear を付けて実行してください。",
              file=sys.stderr)
        return 2

    pin = open_output(PIN_HEARTBEAT)
    if pin is None:
        print("!! E-Stop ハートビートの GPIO を開けない。安全層なしでは実行しない。",
              file=sys.stderr)
        return 2
    try:
        link = SerialLink("/dev/serial0", 250_000)
    except Exception as e:
        print(f"ポートを開けない: {e}", file=sys.stderr)
        pin.close()
        return 2

    log = FrameLogWriter(default_log_path("logs", prefix="armtest"),
                         meta={"node": "arm_test", "note": "arm を通す最小実験"})
    link.on_tx = log.write_tx
    hb = Heartbeat(pin)
    rig = Rig(link, log, hb)
    stop = {"v": False}
    signal.signal(signal.SIGINT, lambda *_: stop.__setitem__("v", True))

    rc = 0
    try:
        print("=" * 66)
        print("arm 最小実験  — 目標速度は常にゼロ。動いたら自動で降ります")
        print("=" * 66)

        hb.start()
        print(f"\n# E-Stop ハートビート開始（GPIO{PIN_HEARTBEAT} {hb.hz}Hz）")
        print("#  終了時に停止するので E-Stop がラッチします（ボタン2で解除）\n")

        # ── 段階0: 現状把握。まだ何も送らない ──
        print("[0/3] 送信せずに現状を見る")
        end = time.monotonic() + 2.0
        while time.monotonic() < end:
            hb.kick()
            for rx in link.poll():
                log.write_rx(rx.rx_ns, rx.type, rx.seq, rx.payload)
                rig.tr.feed(rx.rx_ns, rx.type, rx.seq, rx.payload)
            rig.tr.update_health(time.monotonic_ns())
        rig.show("送信なし")
        t = rig.state.telemetry
        if t is None:
            print("!! TELEMETRY が来ない。中止。", file=sys.stderr)
            return 2
        if t.flags & P.FLG_ESTOP_ACTIVE:
            # 前回の実験でラッチしたまま、はよくある。ここで解除を待てば
            # コマンドを打ち直さずに続けられる（ハートビートは既に出ている）
            print("\n   E-Stop がラッチしています。**車両のボタン2を押してください。**"
                  "（最大120秒待ちます）", flush=True)
            end = time.monotonic() + 120
            while time.monotonic() < end and (t.flags & P.FLG_ESTOP_ACTIVE):
                hb.kick()
                for rx in link.poll():
                    log.write_rx(rx.rx_ns, rx.type, rx.seq, rx.payload)
                    rig.tr.feed(rx.rx_ns, rx.type, rx.seq, rx.payload)
                rig.tr.update_health(time.monotonic_ns())
                t = rig.state.telemetry
            if t.flags & P.FLG_ESTOP_ACTIVE:
                print("!! 解除されませんでした。中止。", file=sys.stderr)
                return 2
            print("   → 解除を確認しました。続行します。")
        # **今の舵角を保持する指令にする**（中央へ動かさないため）
        hold_steer = t.steer_actual
        print(f"       → 舵角 {hold_steer*1e-4:+.4f} rad を保持する指令にします")

        # ── 段階1: DISARM を送って電源が落ちることを再現 ──
        # ここは駆動電源が切れる段階なので、輪が惰性で回っていても危険はない。
        # 判定は「速度0を指令しているのに駆動トルクが出る／駆動輪が回る」なので、
        # 惰性で誤検出しないよう先に静止を待つ
        # 段階1・2 は輪の静止を要求しない。
        # 段階1 は駆動電源が切れるので何も駆動されず、段階2 は DISARM なので
        # 速度制御ループが回らない。**前輪の空転が問題になるのは MANUAL だけ。**
        print(f"\n[1/3] arm=0 / mode=DISARM を {args.hold:.0f}秒"
              "（駆動電源が落ちるはず）")
        rig.pump(args.hold, arm=False, mode=P.Mode.DISARM, steer=hold_steer,
                 label="段階1")
        rig.show("arm=0 DISARM")

        # ── 段階2: arm=1 のまま mode=DISARM ──
        print(f"\n[2/3] **arm=1** / mode=DISARM を {args.hold:.0f}秒"
              "（電源が戻るか？ 動かないか？）")
        rig.pump(args.hold, arm=True, mode=P.Mode.DISARM, steer=hold_steer,
                 label="段階2")
        rig.show("arm=1 DISARM")
        v2 = rig.state.telemetry.batt_voltage_drive * 0.05

        # ── 段階3: MANUAL（任意） ──
        v3 = None
        skipped = None
        if args.manual and not stop["v"]:
            print("\n   MANUAL に入る前に輪の静止を待ちます"
                  "（台上では前輪の空転が幻の速度になるため）", flush=True)
            w = rig.wait_still(timeout_s=20.0, arm=True, mode=P.Mode.DISARM,
                               steer=hold_steer)
            if w >= 0.05:
                # **段階1・2 の結果は有効なので、ここで全体を落とさない**
                skipped = (f"輪が {w:.3f} m/s で回り続けているため MANUAL は省略。"
                           "\n      前輪が本当に回っているか、"
                           "エンコーダが静止時にゼロを返さないかを確認すること")
                print(f"   !! {skipped}")
            else:
                print(f"   → 最大輪速 {w:.3f} m/s。静止を確認しました")
                print(f"\n[3/3] **arm=1 / mode=MANUAL** を {args.hold:.0f}秒"
                      "（目標速度は 0 のまま）")
                rig.pump(args.hold, arm=True, mode=P.Mode.MANUAL, steer=hold_steer,
                         label="段階3", watch_wheels=True)
                rig.show("arm=1 MANUAL")
                v3 = rig.state.telemetry.batt_voltage_drive * 0.05
        else:
            skipped = "mode=MANUAL は省略（--manual で実施）"
            print(f"\n[3/3] {skipped}")

        print("\n" + "=" * 66)
        print(f"結果: arm=1 + DISARM のとき駆動 {v2:.2f} V "
              + ("→ **電源が維持された**" if v2 > 5 else "→ 電源は落ちたまま"))
        if v3 is not None:
            print(f"      arm=1 + MANUAL のとき駆動 {v3:.2f} V "
                  + ("→ **電源が維持された**" if v3 > 5 else "→ 電源は落ちたまま"))
        elif skipped:
            print(f"      MANUAL: {skipped}")
        print("      駆動トルク指令は全段階でゼロ（出ていれば中断していた）")
        print("=" * 66)

    except Aborted as e:
        print(f"\n!! 中断: {e}", file=sys.stderr)
        rc = 1
    finally:
        # **必ず DISARM で降りる**
        for _ in range(10):
            try:
                link.send(P.Command(mode=P.Mode.DISARM, flags=0, target_speed=0,
                                    target_steer=0, accel_limit=0,
                                    steer_rate_limit=0))
            except Exception:
                break
            time.sleep(0.01)
        hb.stop()
        log.close()
        link.close()
        print(f"\nDISARM を送って終了しました。ハートビートも停止（E-Stop ラッチ）。")
        print(f"次に走らせる前に車両のボタン2を押してください。")
        print(f"記録: {log.path}")
    return rc


if __name__ == "__main__":
    raise SystemExit(main())
