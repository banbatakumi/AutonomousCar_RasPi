"""受信フレームからリンクの状態を組み立てる部分。

**io_node（実機）と replay_node（ログ再生）が共有する。** 実機と再生でここが
違うと、再生した結果が実機と食い違い、シミュレータ代わりとして成立しなくなる。
2つ書けば必ずズレるので、1つに閉じる。

責務はこれだけ:

- 受信フレームをデコードして最新値を保持する（`LinkState`）
- `TELEMETRY` / `STATS` の STM32 時刻を時刻同期に食わせる
- `PING`/`PONG` の4タイムスタンプを突き合わせて時刻同期を進める
- `TELEMETRY` 途絶から health（INIT/OK/DEGRADED/FAULT）を判定する

**「今」は呼び出し側が渡す。** 実機は `time.monotonic_ns()`、再生はログ時刻の
カーソルを渡す。こうすると再生が**記録時と同じ座標系**で動き、health の遷移まで
そのまま再現できる（`docs/architecture.md` §11 の「バグ再現が確定的にできる」）。
"""

from __future__ import annotations

import sys
from dataclasses import dataclass, field

from ..proto import packets
from ..proto.framing import RxStats
from .timesync import TimeSync

__all__ = ["LinkState", "LinkTracker", "format_status",
           "TELEM_DEGRADED_NS", "TELEM_FAULT_NS"]

TELEM_DEGRADED_NS = 100 * 1_000_000
TELEM_FAULT_NS = 200 * 1_000_000

#: 応答が返らなかった PING の T1 を溜め込まないための上限
_MAX_PENDING_PINGS = 32


#: `TELEMETRY.flags` のうち、**ラッチして人間の操作でしか戻らない**もの。
#: リンクの健全性（health）とは別の軸なので、混ぜずに独立して持つ。
#: `uart_protocol.md` §10 の安全層 1 と 3 に対応する。
LATCHING_FLAGS = {
    "estop_active": packets.FLG_ESTOP_ACTIVE,
    "drive_power_locked": packets.FLG_DRIVE_POWER_LOCKED,
}

#: `CONFIG_ACK` の `param_id` のうち、`LinkState` に bool として持たせるもの（★v0.8）。
#: `CONFIG_SET`/`CONFIG_GET` どちらの応答でも同じ形式で返るので区別しない
#: `WHEEL_LIFT_GUARD_ENABLE` も同様（★v0.9）。TC/TV本体とは独立した別機構
CONFIG_ACK_BOOL_PARAMS = {
    packets.Param.TC_ENABLE: "tc_enabled",
    packets.Param.TV_ENABLE: "tv_enabled",
    packets.Param.WHEEL_LIFT_GUARD_ENABLE: "wheel_lift_guard_enabled",
}


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

    #: E-Stop 発動中。**車両のボタン2を人間が押すまで解除されない。**
    #: この間 `COMMAND` は一切効かない
    estop_active: bool = False
    #: 過電流で駆動電源がラッチ遮断。**電源を入れ直すまで復帰しない**
    drive_power_locked: bool = False

    #: TC/TV が STM32 側で実際に有効化されているか（`CONFIG_ACK` から取得。★v0.8）。
    #: まだ `CONFIG_ACK` を受け取っていなければ None
    tc_enabled: bool | None = None
    tv_enabled: bool | None = None

    #: 片輪浮き対策（Wheel Lift Guard）が STM32 側で実際に有効化されているか
    #: （`CONFIG_ACK` から取得。★v0.9）。TC/TV本体とは独立した別機構。
    #: まだ `CONFIG_ACK` を受け取っていなければ None
    wheel_lift_guard_enabled: bool | None = None


class LinkTracker:
    """フレーム列 → `LinkState` + `TimeSync`。

    :param on_telemetry: `(Telemetry, t_pi_ns)` で呼ばれる。`t_pi_ns` は
        STM32 時刻を時刻同期で Pi 時刻に換算したもの
    :param on_frame: 全受信フレームで `(t_ns, type, seq, msg)` で呼ばれる
    :param on_latch: ラッチ系フラグが変化したとき `(名前, bool, t_ns)` で呼ばれる。
        E-Stop は人間の操作でしか戻らないので、**立った瞬間を捉えて残す**必要がある
    """

    def __init__(self, on_telemetry=None, on_frame=None, on_latch=None) -> None:
        self.state = LinkState()
        self.sync = TimeSync()
        self.on_telemetry = on_telemetry
        self.on_frame = on_frame
        self.on_latch = on_latch
        self._pending_pings: dict[int, int] = {}   # ping_id -> T1 ns

    # ── 送信側から教えてもらう ──

    def note_ping_sent(self, ping_id: int, t1_ns: int) -> None:
        """PING を送った時刻 T1 を控える。PONG が来たら突き合わせる。"""
        self._pending_pings[ping_id] = t1_ns
        if len(self._pending_pings) > _MAX_PENDING_PINGS:
            self._pending_pings.pop(min(self._pending_pings), None)

    # ── 受信 ──

    def feed(self, rx_ns: int, pkt_type: int, seq: int, payload: bytes):
        """受信フレーム1個を取り込み、デコード結果を返す（未知の TYPE なら None）。"""
        c = self.state.counts
        c[pkt_type] = c.get(pkt_type, 0) + 1

        cls = packets.BY_TYPE.get(pkt_type)
        if cls is None:
            return None
        msg = cls.decode(payload)

        if isinstance(msg, packets.Telemetry):
            self.sync.observe_stm_us(msg.t_us)
            self.state.telemetry = msg
            self.state.last_telem_ns = rx_ns
            self._update_latches(msg.flags, rx_ns)
            if self.on_telemetry:
                self.on_telemetry(msg, self.sync.to_pi_ns(msg.t_us))
        elif isinstance(msg, packets.Pong):
            t1 = self._pending_pings.pop(msg.ping_id, None)
            if t1 is not None:
                self.sync.add_pong(t1, msg.t_ping_rx_us, msg.t_pong_tx_us, rx_ns)
        elif isinstance(msg, packets.Stats):
            self.sync.observe_stm_us(msg.t_us)
            self.state.stats = msg
        elif isinstance(msg, packets.Version):
            self.state.version = msg
        elif isinstance(msg, (packets.LidarSector, packets.LidarSectorI,
                              packets.LidarSectorC)):
            self.state.lidar_sectors.add(msg.sector_idx)
        elif isinstance(msg, packets.ConfigAck):
            self._update_config_ack(msg)

        if self.on_frame:
            self.on_frame(rx_ns, pkt_type, seq, msg)
        return msg

    def _update_latches(self, flags: int, t_ns: int) -> None:
        """ラッチ系フラグの立ち上がり/立ち下がりを捉える。

        `estop_active` は**車両のボタンを人間が押すまで戻らない**ので、
        立った瞬間を逃すと「なぜ動かないのか」が後から分からなくなる。
        """
        for name, bit in LATCHING_FLAGS.items():
            new = bool(flags & bit)
            if new != getattr(self.state, name):
                setattr(self.state, name, new)
                if self.on_latch:
                    self.on_latch(name, new, t_ns)

    def _update_config_ack(self, msg) -> None:
        """`CONFIG_ACK`（`CONFIG_SET`/`CONFIG_GET` どちらへの応答も含む）を取り込む。

        対象外の `param_id` や、`result != OK`（範囲外・不明ID等）は無視する。
        `result == OK` のときの `applied` が STM32 側で実際に適用された値
        （クランプ後・現在値そのもの）
        """
        name = CONFIG_ACK_BOOL_PARAMS.get(msg.param_id)
        if name is None or msg.result != packets.ConfigResult.OK:
            return
        setattr(self.state, name, msg.applied != 0.0)

    # ── health ──

    def update_health(self, now_ns: int) -> str | None:
        """health を更新し、**変わったときだけ**新しい値を返す。

        :param now_ns: 「今」。実機は単調時刻、再生はログ時刻のカーソル
        """
        prev = self.state.health
        if self.state.last_telem_ns == 0:
            new = "INIT"
        else:
            gap = now_ns - self.state.last_telem_ns
            if gap > TELEM_FAULT_NS:
                new = "FAULT"
            elif gap > TELEM_DEGRADED_NS:
                new = "DEGRADED"
            else:
                new = "OK"
        self.state.health = new
        return new if new != prev else None


# ── 表示 ──────────────────────────────────────────────────────────────

def format_status(state: LinkState, sync: TimeSync, rx: RxStats | None = None) -> str:
    """ライブ表示の1行。実機と再生で同じ形にして、見比べられるようにする。"""
    off, delay, drift = sync.offset_ns, sync.best_delay_ns, sync.drift_ppm
    off_s = f"{off / 1000:+.1f}μs" if off is not None else "—"
    delay_s = f"{delay / 1000:.0f}μs" if delay is not None else "—"
    drift_s = f"{drift:+.1f}ppm" if drift is not None else "—"

    t = state.telemetry
    if t:
        veh = (f"v={t.speed * 1e-3:+.2f} steer={t.steer_actual * 1e-4:+.3f} "
               f"az={t.accel_z * 1e-3:.1f} flags=0x{t.flags:04X}")
    else:
        veh = "TELEMETRY 未受信"

    counts = " ".join(f"{packets.BY_TYPE[k].NAME[:4]}={n}"
                      for k, n in sorted(state.counts.items()) if k in packets.BY_TYPE)
    tail = (f"crc={rx.crc_error} loss={rx.packet_loss} reord={rx.reordered}"
            if rx is not None else "")
    # ラッチ系は health より目立たせる。人間が動かないと解除できないため
    latch = ""
    if state.estop_active:
        latch += " ★E-STOP発動中(車両のボタン2で解除)"
    if state.drive_power_locked:
        latch += " ★駆動電源ラッチ(電源を入れ直す)"
    return (f"[{state.health:8}] sync off={off_s} delay={delay_s} drift={drift_s} "
            f"n={sync.n_samples:>3} | {veh} | {counts} {tail}{latch}")


def write_status(line: str) -> None:
    """同じ行を上書きし続ける（実機・再生で共通）。"""
    sys.stdout.write("\r" + line + "    ")
    sys.stdout.flush()
