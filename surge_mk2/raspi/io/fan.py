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

呼び出し側（`telemetry_node.py`）はこの上書きを個別には検知しない・止めようと
しない。`read_rpm()` は実測値なので、上書きが起きると `duty`（GUIが指定した値）
と `rpm`（実際の回転数）が食い違って見える。GUI側は `rpm` を表示し、
「指定通りに動いていない」ことが分かるようにしてある（`DriveControls.tsx`）。

## governor による巻き戻しは高温時に限らない（GUI操作で再現、2026-08-19）

上の「★高温になると〜」は約75°C付近での確認だが、GUIから手動デューティを
1回送っただけだとその後に体感できる頻度（1/3程度）で回転数が落ちる不具合が
報告された。`pwm1_enable=1` の安定保持は「数秒〜十数秒」の範囲でしか実機
確認していなかった点、温度がしきい値をまたぐタイミング次第でもっと低い
温度域から `cooling_device0` が介入しうる点を踏まえ、`telemetry_node.py` の
`_fan_pump`（1Hz）で manual 時のデューティを低頻度に再送するようにした
（`_log_ctrl_pump`/`_auto_ctrl_pump` と同じ「意思は1回で終わらせず再送する」
という既存パターンに合わせた）。governor に書き換えられても最大1秒で
指定値へ復帰する。

## ★★ auto に戻しても governor 自体が停止して pwm1 が下がらないことがある
（実機で確認済み、2026-08-19、kernel 6.18.34+rpt-rpi-2712）

manual → auto と切り替えたあと、`pwm1_enable=2` は正しく書けているのに
`cooling_device0/cur_state` が一切更新されず、`pwm1` が manual 時の高い値の
まま張り付き続ける現象を実機で再現した。温度は 47〜51°C で推移し、
`trip_point_2/3_temp`（60°C/67.5°C、ヒステリシス5°C）の解除ラインを明らかに
下回り続けているのに、5分間 `cur_state` が微動だにしなかった
（`thermal_zone0/policy` に同じガバナー名を書き戻す＝再アタッチでも復帰せず）。

**回避策（実機で有効性を確認）**: `cooling_device0/cur_state` に値を書き込む
と、その書き込みイベント自体が thermal zone の再評価をトリガーし、
数秒以内に現在温度に応じた正しい `cur_state`/`pwm1` へ収束する。試しに
「今読んだ値をそのまま書き戻す」だけでも再評価が起きることを確認したので、
`set_auto()` はこれを毎回行う（`_kick_governor()`）。`cur_state` ノードも
`pwm1` 系と同じく既定 `root:root 644` で `pi` からは書けないため、
`99-surge-fan.rules` 側でも `gpio` グループに書き込み権限を付与している。
書き込めない環境（権限未適用・非対応カーネル）では黙ってスキップし、
`pwm1_enable=2` の書き込みだけで従来どおり動く。
"""

from __future__ import annotations

import glob
import os
from dataclasses import dataclass
from pathlib import Path

__all__ = ["FanState", "FakeFan", "SysfsFan", "open_fan"]

#: `pwm-fan` ドライバの hwmon 登録名。実機（Pi5）で確認済み
FAN_HWMON_NAME = "pwmfan"

#: `pwm-fan` が登録する cooling device の type 名。実機（Pi5）で確認済み
FAN_COOLING_TYPE = "pwm-fan"

HWMON_GLOB = "/sys/class/hwmon/hwmon*"
COOLING_DEVICE_GLOB = "/sys/class/thermal/cooling_device*"


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

    __slots__ = ("available", "_pwm1", "_pwm1_enable", "_fan1_input", "_cur_state")

    def __init__(self) -> None:
        self.available = False
        self._pwm1: Path | None = None
        self._pwm1_enable: Path | None = None
        self._fan1_input: Path | None = None
        self._cur_state: Path | None = None
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
            break
        self._cur_state = self._probe_cur_state()

    def _probe_cur_state(self) -> Path | None:
        """governor 停止時の再評価トリガー用。書き込めなければ `None`
        （`_kick_governor()` が黙ってスキップする）。見つからなくても
        `pwm1`/`pwm1_enable` の基本制御には影響しない。"""
        for cdev in glob.glob(COOLING_DEVICE_GLOB):
            try:
                kind = (Path(cdev) / "type").read_text().strip()
            except OSError:
                continue
            if kind != FAN_COOLING_TYPE:
                continue
            cur_state = Path(cdev) / "cur_state"
            if os.access(cur_state, os.W_OK):
                return cur_state
            return None
        return None

    def _kick_governor(self) -> None:
        """`cur_state` に**別の値**を書き込み、thermal governor の再評価を強制する。

        実機で `pwm1_enable=2` に戻しても `cooling_device0` の再評価自体が
        止まったままになる現象を確認した（詳細はモジュール docstring）。
        **同じ値を書き戻すだけでは再評価は起きない**（sysfs の store が
        値の変化なしと判定して早期リターンする。実機で確認済み）。1段階
        だけ動かして書き込むと、その書き込みイベントをきっかけに governor
        が起き、以降は自分で正しい値へ収束していく（実機で確認済み）。
        """
        if self._cur_state is None:
            return
        try:
            cur = int(self._cur_state.read_text().strip())
        except (OSError, ValueError):
            return
        nudge = cur - 1 if cur > 0 else cur + 1
        try:
            self._cur_state.write_text(str(nudge))
        except OSError:
            pass

    def set_auto(self) -> bool:
        if not self.available:
            return False
        try:
            self._pwm1_enable.write_text("2")
        except OSError:
            return False
        self._kick_governor()
        return True

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
