"""Pi 5 純正クーリングファン — GUI からの自動/手動デューティ制御。

Pi 5 の純正ファンコネクタは `gpiozero` で触れる普通の GPIO ピンではなく、
カーネルの `pwm-fan` cooling device が `hwmon` インターフェースとして
sysfs に公開している（`/sys/class/hwmon/hwmonN/pwm1` 系）。`raspi/io/gpio.py`
の GPIO6/13/18/19 とは別経路なので、`io_node` を経由せず本モジュールで完結する。

## `pwm1_enable` の意味（実機で確認済み、2026-08-17）

`hwmon2`（`name`=`pwmfan`）に対し、以下を実機（Pi5, kernel既定の `pwm-fan` ドライバ）で確認した:

| `pwm1_enable` | 意味 |
|---|---|
| `1` | 手動 — `pwm1`（0-255）に書いた値がそのまま出力され、**サーマルガバナーに
        巻き戻されず数秒以上安定して保持される**（`fan1_input` の実測RPMも追従） |
| `2` | 自動 — 温度がしきい値（既定カーブ、5°Cヒステリシス）を跨いだ次のポーリングで
        `cooling_device0` のガバナー（`step_wise`）が `pwm1` を書き換え、追従を再開する |

起動直後の既定値は `pwm1_enable=1` だが、`cooling_device0` 側が起動時から追従して
いる（`pwm1_enable` の値自体はこのドライバでは「有効/無効」の意味合いが強く、
実際に governor が効くかどうかは温度としきい値の関係で決まる）。

**自動へ戻すときは `pwm1_enable` に明示的に `2` を書く。** 書き込みをやめるだけでは
最後の手動値に固定され続け、気づかず低速のまま発熱し続ける事故になりうる
（`2` を書いてもガバナーが実際に上書きするのは温度がしきい値を跨いだときなので、
即座に反映されるとは限らない。実機では温度58→60.6°Cへの上昇で数秒〜十数秒後
に反映された）。

`pwm1`/`pwm1_enable` は既定 `root:root 644` で `pi` ユーザーからは書けない
（実機で確認済み）。`raspi/setup/99-surge-fan.rules`（`install_services.sh` が
適用する）で `gpio` グループに `g+w` を付与している。

## ★ 高温になると手動指定はハードウェアに上書きされる（実機で確認済み、2026-08-17）

`pwm1_enable=1`（手動）のままでも、**温度が危険域（実機で約75°C付近を確認、
既定カーブの最終段に相当）を超えた瞬間、`pwm1` の値がハードウェア/ファーム
ウェア側から強制的に上書きされる**（`pwm1_enable` の値は `1` のまま変わらない
のに `pwm1` だけ変わる）。CPUに意図的な負荷をかけて温度を75°C付近まで上げ、
`pwm1` が指定値から最大値付近へ強制的に切り替わることを確認した。

これは熱暴走防止のフェイルセーフであり、**ソフトウェアから無効化しない・
すべきでない。** 低い温度域（50〜74°C程度）では `pwm1_enable=1` の手動指定は
ガバナーに巻き戻されず安定して保持される（同じく実機確認済み）。

呼び出し側（`telemetry_node.py`）はこの上書きを検知しない・止めようとしない。
`read_rpm()` は実測値なので、上書きが起きると `duty`（GUIが指定した値）と
`rpm`（実際の回転数）が食い違って見える。GUI側は `rpm` を表示し、
「指定通りに動いていない」ことが分かるようにしてある（`DriveControls.tsx`）。
"""

from __future__ import annotations

import glob
import os
from dataclasses import dataclass
from pathlib import Path

__all__ = ["FanState", "FakeFan", "SysfsFan", "open_fan"]

#: `pwm-fan` ドライバの hwmon 登録名。実機（Pi5）で確認済み
FAN_HWMON_NAME = "pwmfan"

HWMON_GLOB = "/sys/class/hwmon/hwmon*"


@dataclass(slots=True)
class FanState:
    mode: str            # "auto" | "manual"
    duty: float           # 手動時の目標値 [0.0-1.0]
    available: bool       # この機体で手動デューティ制御が使えるか
    rpm: int | None       # 実測回転数。読めなければ None


class FakeFan:
    """ハードウェアのない環境（Mac・テスト）用。状態を保持するだけで常に成功扱い。"""

    __slots__ = ("available",)

    def __init__(self) -> None:
        self.available = True

    def set_auto(self) -> bool:
        return True

    def set_manual(self, duty: float) -> bool:
        return True

    def read_rpm(self) -> int | None:
        return None


class SysfsFan:
    """実機用。`pwm-fan` の hwmon ノードを直接読み書きする。

    見つからない・書き込めない場合は `available=False` のまま動く
    （呼び出し側はこれを見て GUI の手動モードを無効化する）。
    """

    __slots__ = ("available", "_pwm1", "_pwm1_enable", "_fan1_input")

    def __init__(self) -> None:
        self.available = False
        self._pwm1: Path | None = None
        self._pwm1_enable: Path | None = None
        self._fan1_input: Path | None = None
        self._probe()

    def _probe(self) -> None:
        for hwmon in glob.glob(HWMON_GLOB):
            try:
                name = (Path(hwmon) / "name").read_text().strip()
            except OSError:
                continue
            if name != FAN_HWMON_NAME:
                continue
            pwm1 = Path(hwmon) / "pwm1"
            pwm1_enable = Path(hwmon) / "pwm1_enable"
            if not (os.access(pwm1, os.W_OK) and os.access(pwm1_enable, os.W_OK)):
                continue
            self._pwm1, self._pwm1_enable = pwm1, pwm1_enable
            fan1_input = Path(hwmon) / "fan1_input"
            self._fan1_input = fan1_input if fan1_input.exists() else None
            self.available = True
            return

    def set_auto(self) -> bool:
        if not self.available:
            return False
        try:
            self._pwm1_enable.write_text("2")
            return True
        except OSError:
            return False

    def set_manual(self, duty: float) -> bool:
        if not self.available:
            return False
        duty = max(0.0, min(1.0, duty))
        try:
            self._pwm1_enable.write_text("1")
            self._pwm1.write_text(str(round(duty * 255)))
            return True
        except OSError:
            return False

    def read_rpm(self) -> int | None:
        if self._fan1_input is None:
            return None
        try:
            return int(self._fan1_input.read_text().strip())
        except (OSError, ValueError):
            return None


def open_fan() -> SysfsFan | FakeFan:
    """ファン制御を開く。実機で `pwm-fan` の hwmon が見つからない・書き込めない
    環境でも例外を出さず、`available=False` の `SysfsFan` を返す。"""
    try:
        return SysfsFan()
    except Exception:
        return FakeFan()
