"""LED の表示パターンを一通り出して目で確かめる。

    .venv/bin/python -m raspi.tools.led_demo
    .venv/bin/python -m raspi.tools.led_demo --hold 6      # 各状態を長めに

`StatusIndicator` の仕様（`raspi/io/gpio.py`）を実物で見るためのもの。
**離れた所から見て区別できるか**は、実機で見ないと分からない。
区別しづらいパターンがあれば周期を調整すること。

ブザーは既定で鳴らさない（`--buzzer` で有効）。
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
    Indication,
    StatusIndicator,
    open_output,
)

SCENES = [
    ("起動中・通信待ち", "緑が**速く点滅**(4Hz) / 赤 消灯",
     Indication(health="INIT")),
    ("正常・DISARM（動かない）", "緑が**チカッと短く明滅**(1Hz) / 赤 消灯",
     Indication(health="OK")),
    ("正常・ARMED（車輪が動きうる）", "緑**点灯しっぱなし** / 赤 消灯",
     Indication(health="OK", armed=True)),
    ("警告（低電圧など・走行は継続可）", "緑 明滅 / 赤が**ゆっくり点滅**(1Hz)",
     Indication(health="OK", warning=True)),
    ("FAULT（リンク断・自動復帰しうる）", "緑 明滅 / 赤が**速く点滅**(4Hz)",
     Indication(health="FAULT")),
    ("E-Stop ラッチ（車両のボタン2が要る）", "緑 明滅 / 赤**点灯しっぱなし**",
     Indication(health="OK", estop=True)),
    ("駆動電源ラッチ（電源入れ直しが要る）", "緑 明滅 / 赤**点灯しっぱなし**",
     Indication(health="OK", power_locked=True)),
    ("ARMED かつ警告あり（2軸が独立していること）", "緑**点灯** / 赤**ゆっくり点滅**",
     Indication(health="OK", armed=True, warning=True)),
]


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--hold", type=float, default=5.0, help="各状態の表示秒数")
    ap.add_argument("--buzzer", action="store_true", help="ブザーも鳴らす")
    args = ap.parse_args()

    green = open_output(PIN_LED_GREEN)
    red = open_output(PIN_LED_RED)
    buzzer = open_output(PIN_BUZZER) if args.buzzer else None
    if green is None or red is None:
        print("LED の GPIO を開けない", file=sys.stderr)
        return 2

    ind = StatusIndicator(green, red, buzzer)
    print(f"LED 表示デモ  各 {args.hold:.0f}秒"
          + ("" if args.buzzer else "（ブザーは鳴らしません）") + "\n")
    try:
        for i, (name, expect, state) in enumerate(SCENES, 1):
            print(f"[{i}/{len(SCENES)}] {name}\n      → {expect}", flush=True)
            end = time.monotonic() + args.hold
            while time.monotonic() < end:
                ind.update(state)
                time.sleep(0.01)          # 点滅を作るため細かく更新する
        print("\n終了。消灯します。")
    finally:
        ind.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
