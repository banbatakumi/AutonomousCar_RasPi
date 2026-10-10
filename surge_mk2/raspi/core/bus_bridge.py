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
#: LiDAR セクタの先頭点の時刻（換算後）が受信よりこれ以上古ければ、時刻同期が外れていると見て
#: 受信時刻で代用する。正常なら 1セクタ（約 8.3ms）＋送信待ちで 10〜20ms
_MAX_SECTOR_AGE_NS = 200_000_000


class BusBridge:
    """`LinkTracker` のコールバックを受けてバスへ流す。

    :param pub: `bus.Publisher`。None なら何もしない（バス無しで動かす診断モード）
    :param clock: 「今」を返す関数。実機は `time.monotonic_ns`、
        再生はログ時刻のカーソル。**ここを差し替えられることが replay の肝**
    :param base_m: `StateBuilder.odom_center` の起点。**実機（`io_node`）だけが
        SDの永続値を渡す。** replay/ツール類は既定の 0.0 のままでよい
        （再生対象のログはそのログの走行だけで完結すべきなので、実機の
        総走行距離を混ぜてはいけない）
    """

    def __init__(self, pub=None, clock=time.monotonic_ns, base_m: float = 0.0) -> None:
        self.pub = pub
        self.clock = clock
        #: STM32 の生 `t_us` → Pi の単調時刻 [ns]（`TimeSync.to_pi_ns`）。LiDAR セクタの時刻に使う。
        #: 持ち主（io_node・replay）が `LinkTracker` を作ったあとで差す。None なら受信時刻で代用
        self.to_pi_ns = None
        self.state_builder = StateBuilder(base_m=base_m)
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
        scan = self.scans.feed(msg, self._sector_start_ns(msg, t_ns))
        if scan is None:
            return
        self.published_scans += 1
        if self.pub is not None:
            self.pub.send(TOPIC_SCAN, scan)

    def _sector_start_ns(self, msg, rx_ns: int) -> int:
        """セクタ先頭点を測った時刻（Pi 時刻）。STM32 が付けた `t_start_us` を時刻同期で換算する。

        以前は受信時刻をそのまま入れていた。受信はセクタを測り終えて送り切ったあとなので、
        1セクタぶん（約 8.3ms）＋送信待ちだけ遅く、`VehicleState.t_capture`（こちらは換算済み）と
        時間軸がずれていた（3m/s で約 3cm。点ごとの脱スキューが `sector_t_ns` を「先頭点の時刻」
        として使う）。換算できない（同期が未収束）・受信より後・古すぎる（同期の外れ）ときは
        受信時刻で代用する。
        """
        if self.to_pi_ns is None:
            return rx_ns
        t = self.to_pi_ns(msg.t_start_us)
        if t is None or t > rx_ns or rx_ns - t > _MAX_SECTOR_AGE_NS:
            return rx_ns
        return t

    # ── 診断 ──

    def build_diag(self, state, sync, rx_stats=None, *, heartbeat=None,
                   arm_inhibited: bool = True, arm_inhibit_reason: str = "",
                   stm_log: list[str] | None = None,
                   cmd_source: str = "",
                   cmd_stale: bool = True, expected_version: int | None = None,
                   sim: bool = False, control_sync=None,
                   control_params_error: str = "",
                   loop_max_ms: float | None = None, cmd_timeouts: int = 0,
                   disk_free_pct: float | None = None,
                   log_errors: int = 0) -> LinkDiag:
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
            abs_enabled=state.abs_enabled,
            brake_hold_enabled=state.brake_hold_enabled,
            auto_stop_margin_cm=state.auto_stop_margin_cm,
            control_params_status=("invalid" if control_params_error
                                   else control_sync.status if control_sync is not None else "none"),
            control_params=control_sync.applied if control_sync is not None else {},
            control_params_problems=([control_params_error] if control_params_error
                                     else control_sync.problems if control_sync is not None else []),
            control_params_drift=control_sync.drift_count if control_sync is not None else 0,
            arm_inhibited=arm_inhibited,
            arm_inhibit_reason=arm_inhibit_reason,
            stm_log=list(stm_log or ()),
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
            loop_max_ms=loop_max_ms,
            cmd_timeouts=cmd_timeouts,
            # 記録の再生（`replay_node`）では同期器が別物のことがある
            stm_resets=getattr(sync, "resets", 0),
            odom_jumps=self.state_builder.odom_jumps,
            disk_free_pct=disk_free_pct,
            log_errors=log_errors,
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
            d.fw_build_epoch = ver.build_epoch
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
