"""GPIO — E-Stop ハートビート（第1安全層）と状態表示。

配線は `docs/architecture.md` §2 / §7.1。

| ピン | 用途 |
|---|---|
| GPIO6 | **E-Stop ハートビート** 100Hz 矩形波 → STM32 PB12 |
| GPIO19 | LED 緑 |
| GPIO13 | LED 赤 |
| GPIO18 | 圧電ブザー |

## ハートビートの考え方

単純なアクティブ Low の E-Stop は**フェイルセーフにならない**。断線するとプルアップで
High（正常）に戻ってしまうため。矩形波を出し続け、STM32 が「エッジが 50ms 途切れたら
即ブレーキ」とすることで、**Pi のフリーズ・電源断・断線・プロセスクラッシュを
1本の線で同時に検出**できる。

### だからこそ、ハードウェア PWM を使ってはいけない

ハードウェア PWM に矩形波を任せると、**Python プロセスが死んでも波形が出続ける**。
STM32 は「Pi は生きている」と誤認し、安全機構が丸ごと無効になる。
波形を出すのは必ずプロセス自身の仕事にする。プロセスが止まれば波形も止まる。
（プロセスが死ねば OS が GPIO を解放し、プルアップで High に張り付いてエッジが消える。）

### スレッドが生きているだけでは不十分 — `kick()`

専用スレッドで叩くだけだと、**メインループが固まってもスレッドが元気に波形を出し続ける**。
それでは「Pi のフリーズ」を検出できない。そこでメインループが毎周 `kick()` を呼び、
`kick_timeout_s` 途切れたら**ハートビート側が意図的に波形を止める**。

    メインループ停止 → kick 途絶 (既定 100ms) → 波形停止 → STM32 が 50ms で E-Stop
    合計およそ 150ms で駆動が切れる。

### オープンドレインについて

仕様は「オープンドレイン推奨」だが、**既定はプッシュプル**にしている。
Pi も STM32 も 3.3V で電位差が無く、プッシュプルで電気的に問題が無いこと、
gpiozero がオープンドレインを直接サポートしておらず、入出力を切り替えて模擬すると
1秒あたり 200回の切り替えでジッタが増えるため。Pi が電源断になれば
ピンはハイインピーダンスになり、STM32 側プルアップが High を保つので
フェイルセーフ性は失われない。
"""

from __future__ import annotations

import threading
import time
from dataclasses import dataclass, field

__all__ = [
    "PIN_HEARTBEAT", "PIN_LED_GREEN", "PIN_LED_RED", "PIN_BUZZER",
    "FakePin", "GpiozeroPin", "open_output",
    "Heartbeat", "HeartbeatStats", "Indication", "StatusIndicator",
]

PIN_HEARTBEAT = 6
PIN_LED_GREEN = 19
PIN_LED_RED = 13
PIN_BUZZER = 18

HEARTBEAT_HZ = 100
#: メインループの `kick()` がこれだけ途切れたら波形を止める
KICK_TIMEOUT_S = 0.1


# ── ピン ────────────────────────────────────────────────────────────────

class FakePin:
    """ハードウェアのない環境（Mac・テスト）用。遷移を記録するだけ。"""

    __slots__ = ("level", "writes", "closed", "name")

    def __init__(self, name: str = "fake") -> None:
        self.name = name
        self.level = False
        self.writes = 0
        self.closed = False

    def write(self, value: bool) -> None:
        self.level = bool(value)
        self.writes += 1

    def close(self) -> None:
        self.closed = True


class GpiozeroPin:
    """gpiozero の DigitalOutputDevice をこのモジュールの形に合わせる薄い層。"""

    __slots__ = ("_dev", "name")

    def __init__(self, gpio: int, initial: bool = False) -> None:
        from gpiozero import DigitalOutputDevice
        self.name = f"GPIO{gpio}"
        self._dev = DigitalOutputDevice(gpio, initial_value=initial)

    def write(self, value: bool) -> None:
        # on()/off() より value 代入のほうが呼び出しが浅く、100Hz でのばらつきが小さい
        self._dev.value = 1 if value else 0

    def close(self) -> None:
        try:
            self._dev.close()
        except Exception:
            pass


def open_output(gpio: int, initial: bool = False):
    """出力ピンを開く。GPIO が使えなければ `None` を返す（例外にしない）。

    Mac や権限の無い環境でも io_node が起動できるようにするため。
    **黙って落とすのではなく、呼び出し側が警告を出して記録すること。**
    """
    try:
        return GpiozeroPin(gpio, initial)
    except Exception:
        return None


# ── ハートビート ────────────────────────────────────────────────────────

@dataclass(slots=True)
class HeartbeatStats:
    """波形の実測。**Python でこれを回してよいかを判断する材料。**"""

    edges: int = 0                 #: 実際に出したエッジ数
    skipped: int = 0               #: kick 途絶で止めた回数
    stalls: int = 0                #: 止め始めた回数（連続した停止は1と数える）
    max_late_ns: int = 0           #: 予定時刻からの最大遅れ
    late_hist: dict[str, int] = field(default_factory=dict)   #: 遅れの分布

    def observe_late(self, late_ns: int) -> None:
        if late_ns > self.max_late_ns:
            self.max_late_ns = late_ns
        ms = late_ns / 1e6
        b = ("<1ms" if ms < 1 else "<2ms" if ms < 2 else "<5ms" if ms < 5
             else "<10ms" if ms < 10 else "<50ms" if ms < 50 else ">=50ms")
        self.late_hist[b] = self.late_hist.get(b, 0) + 1

    def as_dict(self) -> dict:
        return {"edges": self.edges, "skipped": self.skipped, "stalls": self.stalls,
                "max_late_ms": round(self.max_late_ns / 1e6, 2),
                "late_hist": dict(self.late_hist)}


class Heartbeat:
    """GPIO6 に矩形波を出し続ける。専用スレッド。

    :param pin: `write(bool)` / `close()` を持つ出力ピン
    :param hz: 矩形波の周波数（既定 100Hz = 5ms ごとにトグル）
    :param kick_timeout_s: `kick()` がこれだけ途切れたら波形を止める。
        `None` なら kick を要求しない（波形確認用）
    :param clock, sleep: テストから差し替えるため

    スレッドを使うのは、シリアル I/O でメインループが数十 ms 止まっても
    波形を絶やさないため。ただし止まったメインループを生きていると誤認しないよう、
    `kick()` を安否確認に使う。
    """

    def __init__(self, pin, *, hz: int = HEARTBEAT_HZ,
                 kick_timeout_s: float | None = KICK_TIMEOUT_S,
                 clock=time.monotonic, sleep=time.sleep) -> None:
        self._pin = pin
        self.hz = hz
        self.kick_timeout_s = kick_timeout_s
        self._clock = clock
        self._sleep = sleep
        self._half = 1.0 / (2 * hz)          # トグル間隔 [s]
        self.stats = HeartbeatStats()

        self._level = False
        self._running = False
        self._stalled = False
        self._last_kick = 0.0
        self._next_t = 0.0
        self._thread: threading.Thread | None = None

    # ── 安否確認 ──

    def kick(self) -> None:
        """メインループが生きていることを知らせる。**毎周呼ぶこと。**"""
        self._last_kick = self._clock()

    @property
    def alive(self) -> bool:
        """今この瞬間、波形が出ているか。"""
        return self._running and not self._stalled

    # ── 1回分の処理（テストから直接叩ける） ──

    def _step(self) -> None:
        now = self._clock()
        late = now - self._next_t
        if late > 0:
            self.stats.observe_late(int(late * 1e9))
        # 大きく遅れたら取り戻そうとせず基準を今に置き直す
        # （溜まった遅れを一気に吐き出しても STM32 から見て意味がない）
        self._next_t = now + self._half if late > self._half * 4 else self._next_t + self._half

        if self.kick_timeout_s is not None and now - self._last_kick > self.kick_timeout_s:
            # メインループが固まっている。**意図的に波形を止める**
            if not self._stalled:
                self._stalled = True
                self.stats.stalls += 1
            self.stats.skipped += 1
            return

        self._stalled = False
        self._level = not self._level
        self._pin.write(self._level)
        self.stats.edges += 1

    def _loop(self) -> None:
        while self._running:
            d = self._next_t - self._clock()
            if d > 0:
                self._sleep(d)
            if not self._running:
                break
            self._step()

    # ── 開始・停止 ──

    def start(self) -> None:
        if self._running:
            return
        self._running = True
        self._stalled = False
        self._last_kick = self._clock()
        self._next_t = self._clock() + self._half
        self._thread = threading.Thread(target=self._loop, name="heartbeat", daemon=True)
        self._thread.start()

    def stop(self, join_timeout: float = 1.0) -> None:
        """波形を止める。**これは E-Stop を発動させる操作**（正常終了時もそうする）。"""
        self._running = False
        if self._thread is not None:
            self._thread.join(timeout=join_timeout)
            self._thread = None
        try:
            self._pin.write(False)
            self._pin.close()
        except Exception:
            pass

    def __enter__(self) -> "Heartbeat":
        self.start()
        return self

    def __exit__(self, *exc) -> None:
        self.stop()


# ── 状態表示 ────────────────────────────────────────────────────────────

@dataclass(slots=True)
class Indication:
    """LED に出す状態。**何を優先して見せるかの判断をここに集約する。**"""

    health: str = "INIT"          #: INIT / OK / DEGRADED / FAULT
    armed: bool = False           #: 車輪が動きうるか
    estop: bool = False           #: E-Stop ラッチ中（車両のボタン2で解除）
    power_locked: bool = False    #: 駆動電源ラッチ中（電源を入れ直す）
    warning: bool = False         #: 低電圧など。走行は継続できる

    @property
    def latched(self) -> bool:
        """人間が車両を物理的に触らないと戻らない状態か。"""
        return self.estop or self.power_locked


class StatusIndicator:
    """LED 2個で「生存と可動性」「異常の重さ」を示す。

    **配線は実機で確認済み**（2026-08-08）: GPIO19=緑 / GPIO13=赤、アクティブ High。

    ## 緑 — 生存と可動性

    | 緑 | 意味 |
    |---|---|
    | 消灯 | **Pi が動いていない**（電源断・フリーズ・未起動） |
    | 速い点滅 4Hz | 起動中・通信待ち（INIT） |
    | 短い明滅 1Hz | 正常・**DISARM**（動かない） |
    | 点灯 | **ARMED — 車輪が動きうる** |

    **「点いている」ことより「変化し続けている」ことに意味がある。**
    明滅そのものがソフトウェアが生きている証拠で、固まれば LED も固まる。
    車両のそばに手を伸ばすとき最初に知りたいのは「動きうるか」なので、
    ARMED だけを点灯しっぱなしにして最も目立たせる。

    ## 赤 — 異常の重さ（＝人間が何をすべきか）

    | 赤 | 意味 | すべきこと |
    |---|---|---|
    | 消灯 | 異常なし | — |
    | ゆっくり点滅 1Hz | 警告（低電圧・DEGRADED） | 覚えておく。走行は継続可 |
    | 速い点滅 4Hz | FAULT（リンク断） | 見に行く。自動復帰しうる |
    | 点灯 | **ラッチ**（E-Stop / 駆動電源） | **車両を触りに行く** |

    「異常」で一括りにせず、**要求される行動**で段階を分けている。
    ラッチ系だけが「今すぐ車両まで歩く」必要があるので、点灯しっぱなしで最も強く出す。

    ピンが開けなかったものは黙って無視する（表示が出ないだけで走行に影響しない）。
    """

    BEEP_S = 0.2
    ESTOP_BEEP_S = 0.6
    ALIVE_FLASH_S = 0.12          #: DISARM 時に緑を光らせる長さ（1秒周期の中で）

    def __init__(self, green=None, red=None, buzzer=None,
                 *, clock=time.monotonic) -> None:
        self.green, self.red, self.buzzer = green, red, buzzer
        self._clock = clock
        self._prev: Indication | None = None
        self._beep_until = 0.0

    def update(self, ind: Indication) -> None:
        now = self._clock()
        self._maybe_beep(ind, now)
        self._prev = ind

        self._set(self.green, self._green_on(ind, now))
        self._set(self.red, self._red_on(ind, now))
        self._set(self.buzzer, now < self._beep_until)

    # ── 各色の判断 ──

    def _green_on(self, ind: Indication, now: float) -> bool:
        if ind.armed:
            return True                                  # 動きうる → 点灯
        if ind.health == "INIT":
            return int(now * 8) % 2 == 0                 # 4Hz 起動中
        return (now % 1.0) < self.ALIVE_FLASH_S          # 1Hz の短い明滅＝生存

    def _red_on(self, ind: Indication, now: float) -> bool:
        if ind.latched:
            return True                                  # 人間が触るまで戻らない
        if ind.health == "FAULT":
            return int(now * 8) % 2 == 0                 # 4Hz
        if ind.warning or ind.health == "DEGRADED":
            return int(now * 2) % 2 == 0                 # 1Hz
        return False

    def _maybe_beep(self, ind: Indication, now: float) -> None:
        """**状態が悪化した瞬間だけ**鳴らす。鳴りっぱなしにはしない。"""
        p = self._prev
        if p is None:
            return
        if ind.estop and not p.estop:
            self._beep_until = now + self.ESTOP_BEEP_S
        elif ind.power_locked and not p.power_locked:
            self._beep_until = now + self.ESTOP_BEEP_S
        elif ind.health == "FAULT" and p.health != "FAULT":
            self._beep_until = now + self.BEEP_S

    @staticmethod
    def _set(pin, value: bool) -> None:
        if pin is not None:
            pin.write(value)

    def close(self) -> None:
        for pin in (self.green, self.red, self.buzzer):
            if pin is not None:
                try:
                    pin.write(False)
                    pin.close()
                except Exception:
                    pass
