"""受信フレーム → 内部バスへの publish。

**io_node（実機）と replay_node（ログ再生）が共有する。** `LinkTracker` と同じ理由で、
2つ書けば必ずズレて「再生した結果が実機と違う」状態になる。

責務:

- `TELEMETRY` → `vehicle_state`（SI 換算・前輪射影は `msgs.convert` が行う）
- `LIDAR_SECTOR` 12個 → `scan`（1周ぶん）
- リンクの健全性 → `diag/link`
- **エンドツーエンド遅延の実測**（`COMMAND` を送った時刻と `cmd_seq_echo` の突き合わせ）

エンドツーエンド遅延は `uart_protocol.md` §6.1 のとおり**時刻同期を必要としない**。
Pi の時計だけで完結する往復測定なので、時刻同期が未収束でも値が出る。
"""

from __future__ import annotations

import time

from ..msgs import LinkDiag, ScanAssembler, StateBuilder
from ..msgs.types import TOPIC_DIAG_LINK, TOPIC_SCAN, TOPIC_VEHICLE_STATE
from ..proto import packets

__all__ = ["BusBridge"]

#: `cmd_seq_echo` の突き合わせで、これを超える往復は「SEQ が一周した別物」と見なす。
#: SEQ は u8 なので 100Hz では 2.56 秒で一周する
_MAX_PLAUSIBLE_RTT_NS = 500_000_000


class BusBridge:
    """`LinkTracker` のコールバックを受けてバスへ流す。

    :param pub: `bus.Publisher`。None なら何もしない（バス無しで動かす診断モード）
    :param clock: 「今」を返す関数。実機は `time.monotonic_ns`、
        再生はログ時刻のカーソル。**ここを差し替えられることが replay の肝**
    """

    def __init__(self, pub=None, clock=time.monotonic_ns) -> None:
        self.pub = pub
        self.clock = clock
        self.state_builder = StateBuilder()
        self.scans = ScanAssembler()

        #: SEQ(u8) → その COMMAND を送った時刻。SEQ が一周するので固定長で持つ
        self._cmd_tx_ns: list[int] = [0] * 256
        self._last_echo: int = -1
        #: 直近のエンドツーエンド遅延 [ms]。GUI に常時出す（50ms 超で警告）
        self.cmd_rtt_ms: float | None = None

        self.vehicle_states = 0
        self.published_scans = 0

    # ── 送信側から教えてもらう ──

    def note_command_tx(self, seq: int, t_ns: int) -> None:
        """`COMMAND` を送った時刻を控える。`cmd_seq_echo` と突き合わせる。"""
        self._cmd_tx_ns[seq & 0xFF] = t_ns

    # ── LinkTracker のコールバック ──

    def on_telemetry(self, t: packets.Telemetry, t_pi_ns: int | None) -> None:
        now = self.clock()
        # 時刻同期が未収束のうちは STM32 時刻を Pi 時刻に直せない。
        # そのときは受信時刻で代用する（**0 を入れると 1970年扱いになる**）
        capture = t_pi_ns if t_pi_ns is not None else now

        echo = t.cmd_seq_echo
        if echo != self._last_echo:
            tx = self._cmd_tx_ns[echo & 0xFF]
            if tx:
                rtt = now - tx
                # 一周した SEQ を拾って「遅延 2.5 秒」と表示するのを防ぐ
                self.cmd_rtt_ms = rtt / 1e6 if 0 <= rtt < _MAX_PLAUSIBLE_RTT_NS else None
            self._last_echo = echo

        st = self.state_builder.build(t, capture)
        self.vehicle_states += 1
        if self.pub is not None:
            self.pub.send(TOPIC_VEHICLE_STATE, st)

    def on_frame(self, t_ns: int, pkt_type: int, seq: int, msg) -> None:
        """全受信フレーム。LiDAR セクタだけを見る。"""
        if not isinstance(msg, (packets.LidarSector, packets.LidarSectorI,
                                packets.LidarSectorC)):
            return
        scan = self.scans.feed(msg, t_ns)
        if scan is None:
            return
        self.published_scans += 1
        if self.pub is not None:
            self.pub.send(TOPIC_SCAN, scan)

    # ── 診断 ──

    def build_diag(self, state, sync, rx_stats=None, *, heartbeat=None,
                   arm_inhibited: bool = True, cmd_source: str = "",
                   cmd_stale: bool = True, expected_version: int | None = None,
                   sim: bool = False) -> LinkDiag:
        """`diag/link` の中身を組み立てる。

        **Pi 側の受信統計（`rx`）と STM32 側の `STATS`（`stm_rx`）を並べて出す。**
        片方向だけエラーが増えるのか両方向なのかで原因の切り分けが変わる
        （`uart_protocol.md` §5.10）。
        """
        s = state.stats
        ver = state.version
        d = LinkDiag(
            t_capture=self.clock(),
            health=state.health,
            estop_active=state.estop_active,
            drive_power_locked=state.drive_power_locked,
            tc_enabled=state.tc_enabled,
            tv_enabled=state.tv_enabled,
            wheel_lift_guard_enabled=state.wheel_lift_guard_enabled,
            arm_inhibited=arm_inhibited,
            cmd_source=cmd_source,
            cmd_stale=cmd_stale,
            rx=rx_stats.as_dict() if rx_stats is not None else {},
            counts={packets.BY_TYPE[k].NAME: n for k, n in state.counts.items()
                    if k in packets.BY_TYPE},
            sync_offset_ns=sync.offset_ns,
            sync_delay_ns=sync.best_delay_ns,
            sync_drift_ppm=sync.drift_ppm,
            sync_n=sync.n_samples,
            cmd_rtt_ms=self.cmd_rtt_ms,
            lidar_scans=self.scans.scans,
            lidar_sectors_lost=self.scans.sectors_lost,
            sim=sim,
        )
        if s is not None:
            d.stm_rx = {
                "rx_frame_ok": s.rx_frame_ok, "rx_crc_error": s.rx_crc_error,
                "rx_len_error": s.rx_len_error, "rx_unknown_type": s.rx_unknown_type,
                "tx_drop": s.tx_drop,
            }
            # **MD ごとに分けて出す。** 合計にすると「1台だけ黙っている」が埋もれる
            d.md_rx_count = list(s.md_rx_count)
            d.md_rx_error = list(s.md_rx_error)
        if ver is not None:
            d.protocol_version = ver.protocol_version
            d.fw_id = ver.fw_id
            if expected_version is not None:
                d.protocol_match = ver.protocol_version == expected_version
        if heartbeat is not None:
            d.hb_alive = heartbeat.alive
            d.hb_max_late_ms = heartbeat.stats.max_late_ns / 1e6
            d.hb_stalls = heartbeat.stats.stalls
        if state.limits is not None:
            d.max_speed_m_s = state.limits.max_speed_m_s
            d.max_accel_m_s2 = state.limits.max_accel_m_s2
            d.max_torque_nm = state.limits.max_torque_nm
            d.max_steer_rad = state.limits.max_steer_rad
        return d

    def publish_diag(self, *args, **kw) -> LinkDiag:
        d = self.build_diag(*args, **kw)
        if self.pub is not None:
            self.pub.send(TOPIC_DIAG_LINK, d)
        return d
