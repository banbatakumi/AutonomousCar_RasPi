"""LED とブザーの配線・極性を目視で確認する。

    .venv/bin/python -m raspi.tools.led_test              # LED だけ（ブザーは鳴らさない）
    .venv/bin/python -m raspi.tools.led_test --buzzer     # ブザーも鳴らす
    .venv/bin/python -m raspi.tools.led_test --active-low # 極性が逆の配線の場合

**人間が見ないと判定できない検査。** 何が起きているはずかを毎ステップ表示するので、
実物と見比べること。食い違ったら配線か極性かピン割り当てが違う。

割り当ては `docs/architecture.md` §2:
GPIO19=緑 / GPIO13=赤 / GPIO18=ブザー。
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from raspi.io.gpio import (  # noqa: E402
    PIN_BUZZER,
    PIN_LED_GREEN,
    PIN_LED_RED,
    open_output,
)


class Lamp:
    """LED 1個。`active_low` なら 0 で点灯する配線。"""

    def __init__(self, label: str, gpio: int, active_low: bool) -> None:
        self.label = label
        self.gpio = gpio
        self.active_low = active_low
        self.pin = open_output(gpio, initial=active_low)

    @property
    def ok(self) -> bool:
        return self.pin is not None

    def set(self, on: bool) -> None:
        if self.pin is not None:
            self.pin.write((not on) if self.active_low else on)

    def close(self) -> None:
        if self.pin is not None:
            self.set(False)
            self.pin.close()


def step(n: int, total: int, text: str, seconds: float) -> None:
    print(f"[{n}/{total}] {text}  （{seconds:.0f}秒）", flush=True)
    time.sleep(seconds)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--green", type=int, default=PIN_LED_GREEN)
    ap.add_argument("--red", type=int, default=PIN_LED_RED)
    ap.add_argument("--buzzer-pin", type=int, default=PIN_BUZZER)
    ap.add_argument("--buzzer", action="store_true", help="ブザーも鳴らす")
    ap.add_argument("--active-low", action="store_true",
                    help="0 で点灯する配線（コモンアノード）")
    ap.add_argument("--hold", type=float, default=2.0, help="各ステップの秒数")
    args = ap.parse_args()

    g = Lamp("緑", args.green, args.active_low)
    r = Lamp("赤", args.red, args.active_low)
    b = Lamp("ブザー", args.buzzer_pin, args.active_low) if args.buzzer else None

    for lamp in (g, r, b):
        if lamp is not None and not lamp.ok:
            print(f"!! GPIO{lamp.gpio}（{lamp.label}）を開けない", file=sys.stderr)
            return 2

    print(f"LED 配線確認  緑=GPIO{args.green} 赤=GPIO{args.red}"
          + (f" ブザー=GPIO{args.buzzer_pin}" if b else " （ブザーは鳴らしません）")
          + (f"  極性=アクティブLow" if args.active_low else "  極性=アクティブHigh"))
    print("見えたとおりを教えてください。表示と食い違ったら配線か極性が違います。\n")

    total = 6 if b else 5
    try:
        g.set(False); r.set(False)
        step(1, total, "**両方とも消灯**しているはず", args.hold)

        g.set(True)
        step(2, total, "**緑だけ点灯**しているはず", args.hold)

        g.set(False); r.set(True)
        step(3, total, "**赤だけ点灯**しているはず", args.hold)

        g.set(True)
        step(4, total, "**両方とも点灯**しているはず", args.hold)

        print(f"[5/{total}] **緑と赤が交互に点滅**するはず（2Hz）  （4秒）", flush=True)
        end = time.monotonic() + 4.0
        phase = False
        while time.monotonic() < end:
            phase = not phase
            g.set(phase); r.set(not phase)
            time.sleep(0.25)
        g.set(False); r.set(False)

        if b:
            print(f"[6/{total}] **ブザーが 0.3秒 × 2回**鳴るはず", flush=True)
            for _ in range(2):
                b.set(True); time.sleep(0.3)
                b.set(False); time.sleep(0.3)

        print("\n終了。全部消灯しています。")
        print("見えたとおりと違った場合:")
        print("  - 全部逆（消えるはずが点く）→ --active-low を付けて再実行")
        print("  - 緑と赤が入れ替わっている  → --green / --red でピンを入れ替え")
        print("  - どちらも光らない          → 配線・抵抗・GND を確認")
    finally:
        for lamp in (g, r, b):
            if lamp is not None:
                lamp.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
