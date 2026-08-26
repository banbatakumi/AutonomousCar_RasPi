"""io_node — STM32 との UART 送受信ノード。

    .venv/bin/python -m raspi.nodes.io_node                # ライブ状態表示＋バス配信
    .venv/bin/python -m raspi.nodes.io_node --duration 10  # 10秒で終了
    .venv/bin/python -m raspi.nodes.io_node --quiet        # 表示なし（統計のみ最後に）
    .venv/bin/python -m raspi.nodes.io_node --log          # logs/ に .sfl を記録
    .venv/bin/python -m raspi.nodes.io_node --no-bus       # バスを使わない（診断用）
    .venv/bin/python -m raspi.nodes.io_node --allow-arm    # ★ モータを回せる状態にする
    .venv/bin/python -m raspi.nodes.io_node --sim          # Mac 上のシミュレータに繋ぐ（`sim/`）

やること:
- 起動時に VERSION_REQ → VERSION を照合（protocol_version 不一致は警告）、
  LIMITS_REQ → LIMITS で車両の物理的な上限値を取得（★v0.11。速度・舵角クランプに使う）
- PING を 5Hz（最初の3秒は 20Hz）で送り、PONG から時刻同期を推定
- COMMAND を 100Hz で送る（STM32 の COMMAND タイムアウトを防ぐハートビートを兼ねる）
- TELEMETRY / STATS / PONG / VERSION / LIDAR を受信・集計
- リンク健全性（TELEMETRY 途絶 100ms=警告 / 200ms=FAULT）を判定
- **バスへ配信**: `vehicle_state`(50Hz) / `scan`(10Hz) / `diag/link`(10Hz) / `hb/io`(10Hz)
- **バスから購読**: `cmd`（走行指令）、`log/ctrl`（`.sfl` 記録の開始/停止）
- 送受信フレームを生のまま `.sfl` に記録（`--log` で起動時から、または `log/ctrl`
  でセッション中いつでも GUI から開始/停止できる。§後述）

## COMMAND の安全設計 — 3つの独立した条件が全部そろわないとモータは回らない

1. **`--allow-arm` が無ければ DISARM 固定。** GUI・WS・バスに何が流れていようと
   ここで断つ。判断を1箇所に閉じないと「どこかで解禁されていた」が起きる
2. **`cmd` が 150ms 途絶したら DISARM に落とす。** publish 側（GUI や
   planning_node）が死んだ＝上位が落ちた、なので止めるのが正しい
3. **Pi 側でも速度・舵角を上限でクランプする。** STM32 にも上限はあるが、
   下位だけが守る構造にしない。上限は STM32 から取得した `LIMITS`（★v0.11・
   車両の物理的な上限）を受信済みならそちらを使う。`--max-speed`/`--max-steer`
   は `LIMITS` 未受信の間だけ使うフォールバック（`_send_command`）

さらに GPIO6 ハートビート（第1層）と STM32 の COMMAND タイムアウト（第2層）が
下にある。ここは第3層（`docs/architecture.md` §7）。
"""

from __future__ import annotations

import argparse
import json
import math
import os
import signal
import sys
import tempfile
import time

from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from raspi.core.bus_bridge import BusBridge  # noqa: E402
from raspi.core.link_tracker import (  # noqa: E402
    LinkState,
    LinkTracker,
    format_status,
    write_status,
)
from raspi.io.gpio import (  # noqa: E402
    PIN_BUZZER,
    PIN_HEARTBEAT,
    PIN_LED_GREEN,
    PIN_LED_RED,
    MELODY_BOOT,
    MELODY_GUI_CONNECT,
    Buzzer,
    Heartbeat,
    Indication,
    StatusIndicator,
    open_output,
    open_tone,
)
from raspi.io.serial_link import SerialLink  # noqa: E402
from raspi.msgs import DriveCmd, Heartbeat as HbMsg, command_from_cmd  # noqa: E402
from raspi.msgs.types import (  # noqa: E402
    TOPIC_CMD,
    TOPIC_HB_PREFIX,
    TOPIC_LOG_CTRL,
    TOPIC_UI_EVENT,
)
from raspi.proto import packets  # noqa: E402
from raspi.proto.generated.packets import PROTOCOL_VERSION  # noqa: E402
from raspi.rec import FrameLogWriter, default_log_path  # noqa: E402
from raspi.core.cleanup import quiet_close  # noqa: E402
from raspi.core.vehicle import Vehicle  # noqa: E402

__all__ = ["IoNode", "LinkState"]

NS = 1_000_000_000
COMMAND_HZ = 100
PING_HZ = 5
PING_WARMUP_HZ = 20
WARMUP_S = 3.0
LINKSTATS_HZ = 1
DIAG_HZ = 10
HB_HZ = 10

#: `handshake()` で `LIMITS` を取り損ねた場合（STM32 の起動burstを取り逃した、
#: 応答が1回ノイズで落ちた等）に `LIMITS_REQ` を再送する間隔（★v0.11）。
#: `state.limits` が一度埋まれば以降は送らない（実行時に変化しない値のため）
LIMITS_RETRY_S = 1.0

#: `cmd` がこれだけ途絶したら DISARM に落とす。
#: **値は `config/vehicle.toml` の `[safety]` が正**（`docs/architecture.md` §9.4）。
#: telemetry_node も同じ判定を独立に持つ（片方が死んでも成立させるための冗長）が、
#: **数字そのものは1箇所**にしてある。以前は3箇所に手書きされていた
#: （2026-08-21 のレビュー 🟢11）
CMD_TIMEOUT_NS = int(Vehicle.load().cmd_deadman_ms * 1_000_000)

#: Pi 側のクランプ既定値。**STM32 から `LIMITS`（★v0.11・車両の物理的な上限）を
#: まだ受信していない間だけ使うフォールバック。** 受信済みならそちらを無条件に使う
#: ので、実車の挙動を変えたいときはここではなく STM32 側の固定定数を直すこと
#: （`IoNode._send_command` 参照。2026-08-22、GUI 側の `effectiveRange()` と方針統一）
DEFAULT_MAX_SPEED = 1.0        # m/s
DEFAULT_MAX_STEER = 0.35       # rad（約 20°）

#: 総走行距離（`odom_center` の起点）を永続化する場所。**`.gitignore` 済み**
#: （機体ごとの実測値なのでコミットしない、`config/auto.json` 等と同じ理由）
REPO_ROOT = Path(__file__).resolve().parents[2]
ODOM_CONF = REPO_ROOT / "config" / "odometer.json"
#: 保存の間引き条件（両方満たしたときだけ書く）。**単調増加の値なので
#: 停止中は書かない**（SDカードの消耗を避ける。バンビ指定、2026-08-25）
ODOM_SAVE_INTERVAL_S = 30.0
ODOM_SAVE_MIN_DELTA_M = 1.0


def _load_odometer_base() -> float:
    """`ODOM_CONF` から総走行距離を読む。無い・壊れている・負値なら 0.0（初回起動と同じ扱い）"""
    try:
        v = float(json.loads(ODOM_CONF.read_text()).get("total_m", 0.0))
        return v if math.isfinite(v) and v >= 0 else 0.0
    except Exception:
        return 0.0


def _save_odometer_base(v: float) -> None:
    """total_m を書く。**単調増加であるべき値**なので、書き込み中の電源断で
    壊れないよう tmp ファイル + `os.replace()` でアトミックに置き換える
    （`config/auto.json` 等の直接書きとは異なる、このリポジトリ初のパターン）"""
    try:
        ODOM_CONF.parent.mkdir(parents=True, exist_ok=True)
        fd, tmp = tempfile.mkstemp(dir=str(ODOM_CONF.parent), prefix=".odometer-")
        with os.fdopen(fd, "w") as f:
            json.dump({"total_m": v}, f)
        os.replace(tmp, ODOM_CONF)
    except Exception:
        pass  # 保存できなくても走行は続けられるべき


class IoNode:
    """:param log: 生フレームログの Writer。None なら記録しない（起動時点の状態）。
    :param log_meta: `log/ctrl` で動的に記録を開始したときに使うメタデータ
        （`port`/`baud`/`protocol_version` など）。`--log` を付けなかった場合でも
        GUI から開始できるよう、常に渡しておく。

    記録は「回っている限り必ず残る」ことが価値なので、ログ書き込みで例外が出ても
    ループは止めない（記録が壊れても走行は続けられるべき）。
    """

    def __init__(self, link: SerialLink, on_telemetry=None,
                 log: FrameLogWriter | None = None,
                 log_meta: dict | None = None,
                 heartbeat: Heartbeat | None = None,
                 indicator: StatusIndicator | None = None,
                 buzzer: Buzzer | None = None,
                 pub=None, sub=None, *,
                 allow_arm: bool = False,
                 max_speed: float = DEFAULT_MAX_SPEED,
                 max_steer: float = DEFAULT_MAX_STEER) -> None:
        self.link = link
        self.bridge = BusBridge(pub, base_m=_load_odometer_base())
        #: 直近に保存した総走行距離。周期保存の間引き判定（1m以上動いたか）に使う
        self._last_saved_odom = self.bridge.state_builder.odom_center
        self.tracker = LinkTracker(
            on_telemetry=self._on_telemetry_chain(on_telemetry),
            on_frame=self.bridge.on_frame,
            on_latch=self._on_latch)
        # 受信状態と時刻同期の実体は tracker が持つ。ここは同じ物への別名。
        self.state = self.tracker.state
        self.sync = self.tracker.sync
        self.heartbeat = heartbeat
        self.indicator = indicator
        #: GUI 接続音などのメロディ再生に使う。`indicator` の内部（`StatusIndicator.buzzer`）
        #: と同じ物理ブザーを指す（`--no-leds` なら両方 None）
        self.buzzer = buzzer
        self.pub = pub
        self.sub = sub
        self.allow_arm = allow_arm
        self.max_speed = max_speed
        self.max_steer = max_steer
        #: `ui/event` の既知の `seq`。これより大きければ「新しいイベント」
        self._ui_event_seq = 0

        #: 直近に受け取った走行指令と、その受信時刻。**古くなったら使わない**
        self.cmd: DriveCmd | None = None
        self._cmd_ns = 0
        self.cmd_stale = True
        #: `cmd` 途絶で DISARM に落とした回数。1回でも起きたら GUI 側を疑う
        self.cmd_timeouts = 0

        self._log = log
        self._log_meta = log_meta
        self._log_errors = 0
        self._ping_seq = 0
        self._running = False
        self._t_start = 0
        link.on_tx = self._on_tx

    def _on_telemetry_chain(self, extra):
        """バスへの配信を必ず通しつつ、呼び出し側のコールバックも呼ぶ。"""
        def _cb(t, t_pi_ns):
            self.bridge.on_telemetry(t, t_pi_ns)
            if extra is not None:
                extra(t, t_pi_ns)
        return _cb

    # ── LED 表示 ──

    def _indication(self) -> Indication:
        """`LinkState` から LED に出す状態を作る。

        `armed` と低電圧は毎フレーム変わるのでラッチ扱いにせず、ここで読む。
        """
        t = self.state.telemetry
        flags = t.flags if t else 0
        undervolt = (packets.FLG_FAULT_DRIVE_UNDERVOLTAGE
                     | packets.FLG_FAULT_SIGNAL_UNDERVOLTAGE)
        return Indication(
            health=self.state.health,
            armed=bool(flags & packets.FLG_ARMED),
            estop=self.state.estop_active,
            power_locked=self.state.drive_power_locked,
            warning=bool(flags & undervolt),
        )

    # ── ラッチ系フラグ ──

    def _on_latch(self, name: str, value: bool, t_ns: int) -> None:
        """E-Stop / 駆動電源ラッチの変化。**必ず記録して人間に見せる。**

        どちらも人間が物理操作しないと戻らないので、気づかないまま
        「なぜ動かないのか」を探す時間が一番もったいない。
        """
        self._log_event("latch", {"flag": name, "value": value})
        if name == "estop_active" and value:
            print("\n!! E-STOP 発動 — 車両のボタン2を押すまで解除されません。"
                  "この間 COMMAND は一切効きません", file=sys.stderr)
        elif name == "estop_active":
            print("\n#  E-STOP 解除を確認", file=sys.stderr)
        elif name == "drive_power_locked" and value:
            print("\n!! 駆動電源ラッチ — 電源を入れ直すまで復帰しません", file=sys.stderr)

    # ── ログ ──

    def _on_tx(self, t_ns: int, pkt_type: int, seq: int, payload: bytes) -> None:
        """送信フック。記録と、エンドツーエンド遅延の測定に使う。

        送信 SEQ はエンコーダ内部にしか無く呼び出し側から見えないので、
        `COMMAND` を送った時刻の記録はここでしかできない。
        """
        if pkt_type == packets.Command.TYPE:
            self.bridge.note_command_tx(seq, t_ns)
        if self._log is None:
            return
        try:
            self._log.write_tx(t_ns, pkt_type, seq, payload)
        except Exception:
            self._log_errors += 1

    def _log_event(self, name: str, data: dict | None = None) -> None:
        if self._log is None:
            return
        try:
            self._log.write_event(time.monotonic_ns(), name, data)
        except Exception:
            self._log_errors += 1

    def _set_log_active(self, active: bool) -> None:
        """`log/ctrl`（GUI の記録ボタン）に従って `.sfl` を開始/停止する。

        **`self._log is not None` が現在の記録状態そのもの。** 呼び出し側
        （`_log_rx`/`_on_tx`/`_log_event` 等）は既にこれを None チェックしているので、
        ここで差し替えるだけで他のコードは変更しなくてよい。書き込み例外と同じ理由で、
        開始に失敗してもループは止めない（走行は記録より優先する）。
        """
        if active == (self._log is not None):
            return
        if active:
            try:
                path = default_log_path()
                self._log = FrameLogWriter(path, meta=self._log_meta)
            except Exception as e:
                print(f"\n!! .sfl 記録を開始できない: {e}", file=sys.stderr)
                return
            print(f"\n# .sfl 記録開始: {path}", file=sys.stderr)
            self._log_event("record_start", {"path": str(path)})
        else:
            self._log_event("record_stop")
            self._log.close()
            self._log = None
            print("\n# .sfl 記録停止", file=sys.stderr)

    def _log_linkstats(self, now_ns: int) -> None:
        """1Hz のリンク健全性スナップショット。

        CRC エラーや再同期バイト数は「捨てたフレーム」なので RX レコードには
        残らない。数字として別に残さないと、後から再生しても異常が見えない。
        ハートビートの実測も同じ理由でここに入れる（GPIO はログに現れない）。
        """
        hb = self.heartbeat
        self._log_event("linkstats", {
            "health": self.state.health,
            "rx": self.link.stats.as_dict(),
            "hb": ({"alive": hb.alive, **hb.stats.as_dict()} if hb else None),
            "sync": {
                "n": self.sync.n_samples,
                "offset_ns": self.sync.offset_ns,
                "best_delay_ns": self.sync.best_delay_ns,
                "drift_ppm": self.sync.drift_ppm,
            },
        })

    # ── 起動時の VERSION / LIMITS 照合 ──

    def handshake(self, timeout_s: float = 1.0) -> packets.Version | None:
        """`VERSION_REQ`/`LIMITS_REQ` を送り、両方の応答を待つ（★v0.11: `LIMITS` 追加）。

        `LIMITS` は読み取り専用・実行時に変化しない値なので、ここで一度取得できれば
        以降は `state.limits` をそのまま使い回してよい（`_send_command` 参照）。
        STM32 は起動直後に `VERSION`/`LIMITS` をどちらも自動送信するが、Pi の再起動・
        再接続時はその窓を過ぎている可能性があるため、明示的に `_REQ` を送って揃える。
        `LIMITS` が届かなくても（旧ファームウェア等）通信自体は継続する。
        """
        self.link.reset_input()
        self.link.send(packets.VersionReq())
        self.link.send(packets.LimitsReq())
        deadline = time.monotonic() + timeout_s
        version = None
        while time.monotonic() < deadline:
            for rx in self.link.poll():
                if self._log is not None:
                    self._log_rx(rx)
                msg = rx.decode()
                if isinstance(msg, packets.Version):
                    version = msg
                    self.state.version = msg
                    self._log_event("handshake", {
                        "protocol_version": msg.protocol_version,
                        "expected": PROTOCOL_VERSION,
                        "match": msg.protocol_version == PROTOCOL_VERSION,
                        "fw_id": msg.fw_id,
                    })
                elif isinstance(msg, packets.Limits):
                    self.state.limits = msg
                    self._log_event("limits", {
                        "max_speed_m_s": msg.max_speed_m_s,
                        "max_accel_m_s2": msg.max_accel_m_s2,
                        "max_torque_nm": msg.max_torque_nm,
                        "max_steer_rad": msg.max_steer_rad,
                    })
            if version is not None and self.state.limits is not None:
                return version
        if version is None:
            self._log_event("handshake", {"match": False, "reason": "timeout"})
        return version

    def _log_rx(self, rx) -> None:
        try:
            self._log.write_rx(rx.rx_ns, rx.type, rx.seq, rx.payload)
        except Exception:
            self._log_errors += 1

    # ── 送信 ──

    def _send_ping(self) -> None:
        self._ping_seq = (self._ping_seq + 1) & 0xFFFFFFFF
        t1 = self.link.send(packets.Ping(ping_id=self._ping_seq))
        self.tracker.note_ping_sent(self._ping_seq, t1)

    def _send_command_disarm(self) -> None:
        """安全な停止ハートビート。mode=DISARM, arm=0, 速度・舵角ゼロ。"""
        self.link.send(packets.Command(
            mode=packets.Mode.DISARM, flags=0,
            target_speed=0, target_steer=0,
            accel_limit=0, steer_rate_limit=0, brake_torque=0))

    def _recv_cmd(self, now_ns: int) -> None:
        """バスから走行指令・`.sfl` 記録の意思（`log/ctrl`）・GUI イベントを取り込む。"""
        if self.sub is not None:
            for topic, msg in self.sub.poll(0):
                if topic == TOPIC_CMD:
                    self.cmd = msg
                    self._cmd_ns = now_ns
                elif topic == TOPIC_LOG_CTRL:
                    self._set_log_active(msg.active)
                elif topic == TOPIC_UI_EVENT:
                    self._on_ui_event(msg)
        stale = self.cmd is None or (now_ns - self._cmd_ns) > CMD_TIMEOUT_NS
        if stale and not self.cmd_stale and self.cmd is not None:
            self.cmd_timeouts += 1
            self._log_event("cmd_timeout", {"source": self.cmd.source})
        self.cmd_stale = stale

    def _on_ui_event(self, msg) -> None:
        """`ui/event`（GUI 側の単発イベント）。`seq` が増えていなければ二重処理しない。"""
        if msg.seq <= self._ui_event_seq:
            return
        self._ui_event_seq = msg.seq
        if msg.kind == "gui_connect" and self.buzzer is not None:
            self.buzzer.play(MELODY_GUI_CONNECT)
        elif msg.kind == "tc_enable":
            self.link.send(packets.ConfigSet(
                param_id=packets.Param.TC_ENABLE, value=1.0 if msg.value else 0.0))
        elif msg.kind == "tv_enable":
            self.link.send(packets.ConfigSet(
                param_id=packets.Param.TV_ENABLE, value=1.0 if msg.value else 0.0))
        elif msg.kind == "wheel_lift_guard_enable":
            self.link.send(packets.ConfigSet(
                param_id=packets.Param.WHEEL_LIFT_GUARD_ENABLE, value=1.0 if msg.value else 0.0))
        elif msg.kind == "auto_stop_margin_cm":
            self.link.send(packets.ConfigSet(
                param_id=packets.Param.AUTO_STOP_MARGIN_CM, value=msg.float_value))

    def _send_command(self) -> None:
        """今この瞬間送るべき `COMMAND` を決めて送る。

        **`allow_arm` が False、または `cmd` が古ければ必ず DISARM。**
        分岐をここ1箇所に集めてある（判断が散ると、どこかで解禁されていたが起きる）。

        速度・舵角・加速度・トルクの上限は、STM32 から受け取った `LIMITS`（★v0.11。
        車両の物理的な上限）を**受信済みなら無条件にそちらを使う**（2026-08-22、GUI
        側の `effectiveRange()` と方針を合わせた。それまでは `self.max_speed`/
        `self.max_steer` との小さい方だったが、それだと LIMITS の方が緩い場合に
        `--max-speed`/`--max-steer` の古い値で頭打ちのままになり、GUI 側が実測値
        まで見せているのに実車だけ変わらない食い違いが起きるため）。`self.max_speed`/
        `self.max_steer`（CLI/GUI で設定した値）は **`LIMITS` をまだ受け取っていない
        間だけ**使うフォールバック。RC（MANUAL）・自律走行（AUTO）どちらの `cmd` も
        この1箇所を通るため、モードを問わず同じ上限が効く。
        """
        if self.cmd_stale or not self.allow_arm:
            self._send_command_disarm()
            return
        lim = self.state.limits
        max_speed = lim.max_speed_m_s if lim else self.max_speed
        max_steer = lim.max_steer_rad if lim else self.max_steer
        self.link.send(command_from_cmd(
            self.cmd, allow_arm=True,
            max_speed=max_speed, max_steer=max_steer,
            max_accel=lim.max_accel_m_s2 if lim else None,
            max_torque=lim.max_torque_nm if lim else None))

    # ── メインループ ──

    def run(self, duration_s: float | None = None, status_cb=None) -> None:
        self._running = True
        self._t_start = time.monotonic()
        next_cmd = time.monotonic_ns()
        next_ping = next_cmd
        next_status = next_cmd
        next_linkstats = next_cmd + NS // LINKSTATS_HZ
        next_diag = next_cmd
        next_hb = next_cmd
        next_limits_retry = next_cmd
        next_odom_save = next_cmd + int(ODOM_SAVE_INTERVAL_S * NS)
        cmd_period = NS // COMMAND_HZ

        while self._running:
            now = time.monotonic_ns()
            elapsed = (now - int(self._t_start * NS)) / NS

            # 「メインループは生きている」の申告。これが途切れるとハートビートが
            # 自分で波形を止め、STM32 が E-Stop に入る（フリーズ検出）
            if self.heartbeat is not None:
                self.heartbeat.kick()

            had_limits = self.state.limits is not None
            for rx in self.link.poll():
                if self._log is not None:
                    self._log_rx(rx)
                self.tracker.feed(rx.rx_ns, rx.type, rx.seq, rx.payload)
            if self.state.limits is not None and not had_limits:
                lim = self.state.limits
                print(f"# STM32 LIMITS 受信（handshake 後の再送で取得）: "
                      f"速度{lim.max_speed_m_s:.2f}m/s 加速度{lim.max_accel_m_s2:.2f}m/s² "
                      f"トルク{lim.max_torque_nm:.3f}N·m 舵角±{math.degrees(lim.max_steer_rad):.1f}°",
                      file=sys.stderr)
                self._log_event("limits", {
                    "max_speed_m_s": lim.max_speed_m_s,
                    "max_accel_m_s2": lim.max_accel_m_s2,
                    "max_torque_nm": lim.max_torque_nm,
                    "max_steer_rad": lim.max_steer_rad,
                    "via": "retry",
                })

            # `handshake()` が LIMITS を取り損ねていたら定期的に再送する（★v0.11）。
            # STM32 起動時の3回burstを取り逃した／応答が1回落ちた場合の保険。
            # 一度受け取れば `state.limits` が埋まるので、以降は送らなくなる
            if self.state.limits is None and now >= next_limits_retry:
                self.link.send(packets.LimitsReq())
                next_limits_retry = now + int(LIMITS_RETRY_S * NS)

            self._recv_cmd(now)

            if now >= next_cmd:
                self._send_command()
                next_cmd += cmd_period
                if now - next_cmd > cmd_period * 5:   # 大きく遅れたら追いつきをやめる
                    next_cmd = now + cmd_period

            if now >= next_ping:
                self._send_ping()
                hz = PING_WARMUP_HZ if elapsed < WARMUP_S else PING_HZ
                next_ping = now + NS // hz

            prev_health = self.state.health
            changed = self.tracker.update_health(now)
            if self.indicator is not None:
                self.indicator.update(self._indication())
            if self._log is not None:
                if changed is not None:
                    self._log_event("health", {"from": prev_health, "to": changed})
                if now >= next_linkstats:
                    self._log_linkstats(now)
                    next_linkstats = now + NS // LINKSTATS_HZ

            if status_cb and now >= next_status:
                status_cb(self)
                next_status = now + NS // 2      # 2Hz 更新

            if now >= next_diag:
                self.bridge.publish_diag(
                    self.state, self.sync, self.link.stats,
                    heartbeat=self.heartbeat,
                    arm_inhibited=not self.allow_arm,
                    cmd_source=self.cmd.source if self.cmd else "",
                    cmd_stale=self.cmd_stale,
                    expected_version=PROTOCOL_VERSION,
                    # シミュレータの link だけがこれを持つ（`sim/link.py`）。
                    # GUI に SIM バッジを出す唯一の根拠。**実機の link には何も足さない**
                    sim=getattr(self.link, "is_sim", False))
                next_diag = now + NS // DIAG_HZ

            if self.pub is not None and now >= next_hb:
                self.pub.send(TOPIC_HB_PREFIX + "io",
                              HbMsg(node="io", pid=os.getpid(),
                                    detail=self.state.health))
                next_hb = now + NS // HB_HZ

            # 総走行距離の永続化。**両方満たしたときだけ書く**——止まっている間は
            # 30秒経っても書かない（SDカードの消耗を避ける、バンビ指定）
            if now >= next_odom_save:
                next_odom_save = now + int(ODOM_SAVE_INTERVAL_S * NS)
                current_odom = self.bridge.state_builder.odom_center
                if abs(current_odom - self._last_saved_odom) >= ODOM_SAVE_MIN_DELTA_M:
                    _save_odometer_base(current_odom)
                    self._last_saved_odom = current_odom

            if duration_s is not None and time.monotonic() - self._t_start >= duration_s:
                break

    def stop(self) -> None:
        self._running = False


# ── ライブ表示 ──

def _status_line(node: IoNode) -> None:
    write_status(format_status(node.state, node.sync, node.link.stats))


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", default="/dev/serial0")
    ap.add_argument("--baud", type=int, default=250_000)
    ap.add_argument("--duration", type=float, default=None, help="秒で自動終了")
    ap.add_argument("--quiet", action="store_true", help="ライブ表示なし")
    ap.add_argument("--log", nargs="?", const="logs", default=None, metavar="PATH",
                    help="起動直後から生フレームログを記録。値なしで logs/ に自動命名、"
                         "*.sfl でファイル指定、それ以外はディレクトリ扱い。"
                         "付けなくても log/ctrl（GUI の記録ボタン）でいつでも開始できる")
    ap.add_argument("--no-heartbeat", action="store_true",
                    help="GPIO6 の E-Stop ハートビートを出さない（診断用）")
    ap.add_argument("--require-gpio", action="store_true",
                    help="GPIO を開けなければ起動しない")
    ap.add_argument("--no-leds", action="store_true", help="LED/ブザーを使わない")
    ap.add_argument("--no-bus", action="store_true",
                    help="ZeroMQ に配信しない（pyzmq 無しでも動かせる診断モード）")
    ap.add_argument("--allow-arm", action="store_true",
                    help="★ バスの cmd を実際に STM32 へ通す。"
                         "付けない限り COMMAND は DISARM 固定")
    ap.add_argument("--max-speed", type=float, default=DEFAULT_MAX_SPEED,
                    help=f"Pi 側の速度クランプ [m/s]（既定 {DEFAULT_MAX_SPEED}）。"
                         "STM32 から LIMITS を受信済みならそちらが優先されるので、"
                         "実質的には未接続時のフォールバックにしかならない")
    ap.add_argument("--max-steer", type=float, default=DEFAULT_MAX_STEER,
                    help=f"Pi 側の舵角クランプ [rad]（既定 {DEFAULT_MAX_STEER}）。"
                         "同上、LIMITS 受信後はそちらが優先される")
    ap.add_argument("--sim", action="store_true",
                    help="UART の相手を Mac 上のシミュレータにする（`sim/` 参照）。"
                         "実機では使わない")
    ap.add_argument("--course", default="sim/courses/oval.png",
                    help="--sim のときのコース PNG")
    args = ap.parse_args()

    try:
        if args.sim:
            # **遅延 import。** Pi にはこのパッケージを配らないので、
            # --sim を付けない限り評価されてはいけない
            from sim.link import create_sim_link
            link = create_sim_link(args.course)
        else:
            link = SerialLink(args.port, args.baud)
    except Exception as e:
        print(f"ポートを開けない: {e}", file=sys.stderr)
        return 2

    # ── バス ──
    pub = sub = None
    if not args.no_bus:
        try:
            from raspi.bus import LATEST, RELIABLE, Publisher, Subscriber
            pub = Publisher("io")
            # UiEvent は「今起きた単発イベント」の列であって「現在の状態」ではない
            # （`msgs/types.py` の UiEvent docstring参照）。LATEST(CONFLATE=1) だと、
            # 例えば tc_enable と tv_enable を短時間に連続 publish したとき片方が
            # 消費前に上書きされて消える（2026-08-19、TC/TVトグルの実装で発覚）。
            # 取りこぼしてはいけないので RELIABLE にする
            sub = Subscriber({TOPIC_CMD: LATEST, TOPIC_LOG_CTRL: LATEST,
                               TOPIC_UI_EVENT: RELIABLE})
            print(f"# バス配信 {pub.endpoint} "
                  f"(vehicle_state / scan / diag/link / hb/io)")
        except ImportError as e:
            print(f"!! pyzmq/msgspec が無いのでバス無しで動く: {e}")
        except Exception as e:
            print(f"!! バスを開けない（バス無しで続行）: {e}", file=sys.stderr)

    # **`args.log` 無しでも常に用意しておく。** `log/ctrl`（GUI の記録ボタン）で
    # セッション中に記録を始めるとき、同じメタデータで書けるようにするため
    # `args.port` ではなく `link.port` を書く。--sim のとき記録に "/dev/serial0" と
    # 残ると、後からログを見て実機の記録と区別がつかなくなる
    log_meta = {"node": "io_node", "port": link.port, "baud": link.baud,
                "protocol_version": PROTOCOL_VERSION, "sim": args.sim}
    log = None
    if args.log is not None:
        path = Path(args.log) if args.log.endswith(".sfl") else default_log_path(args.log)
        log = FrameLogWriter(path, meta=log_meta)
        print(f"# 記録: {path}")

    # ── GPIO ──
    heartbeat = indicator = None
    if not args.no_heartbeat:
        pin = open_output(PIN_HEARTBEAT)
        if pin is None:
            msg = f"GPIO{PIN_HEARTBEAT} を開けない（E-Stop ハートビートなし）"
            if args.require_gpio:
                print(f"!! {msg}", file=sys.stderr)
                link.close()
                return 2
            print(f"!! {msg}。--require-gpio で起動を止められる")
        else:
            heartbeat = Heartbeat(pin)
            print(f"# E-Stop ハートビート GPIO{PIN_HEARTBEAT} "
                  f"{heartbeat.hz}Hz（kick 途絶 {heartbeat.kick_timeout_s}s で停止）")
            print("#  注意: 一度出し始めたら、止めた時点で STM32 が E-Stop をラッチする。"
                  "\n#        解除には車両のボタン2を押す必要がある（--no-heartbeat で回避）")
    else:
        # STM32 はハートビートを一度も見ていない間は E-Stop を発動しない。
        # ベンチ診断で E-Stop をラッチさせたくないときはこちら
        print("# ハートビート無効（ベンチ診断用）。STM32 は未接続扱いのまま")

    buzzer = None
    if not args.no_leds:
        green_pin = open_output(PIN_LED_GREEN)
        red_pin = open_output(PIN_LED_RED)
        tone_dev = open_tone(PIN_BUZZER)
        if tone_dev is not None:
            buzzer = Buzzer(tone_dev)
        if any(p is not None for p in (green_pin, red_pin, buzzer)):
            indicator = StatusIndicator(green_pin, red_pin, buzzer)
        if buzzer is not None:
            # 起動音。STM32 側（C5→E5→G5 の上昇）と被らない下降アルペジオ
            buzzer.play(MELODY_BOOT)
            print(f"# 起動音 GPIO{PIN_BUZZER}")

    node = IoNode(link, log=log, log_meta=log_meta, heartbeat=heartbeat,
                  indicator=indicator, buzzer=buzzer, pub=pub, sub=sub, allow_arm=args.allow_arm,
                  max_speed=args.max_speed, max_steer=args.max_steer)

    def _shutdown(*_):
        node.stop()

    signal.signal(signal.SIGINT, _shutdown)

    print(f"# io_node port={link.port} baud={link.baud} 期待 v{PROTOCOL_VERSION:#06x}")
    ver = node.handshake()
    if ver is None:
        print("!! VERSION 応答なし。STM32 が繋がっていない可能性。受信は続行する。")
    else:
        ok = "✓" if ver.protocol_version == PROTOCOL_VERSION else "✗ 不一致!"
        print(f"# STM32 protocol_version=0x{ver.protocol_version:04X} "
              f"fw=0x{ver.fw_id:08X} {ok}")
        # ★v0.8: TC/TV、★v0.9: 片輪浮き対策の有効/無効はGUI操作を待たず、接続確立直後に
        # 一度だけ現在値を取得しておく（`CONFIG_SET` はFlash永続化されないので、GUI操作前でも
        # STM32起動直後の実際の状態をGUIに出せるようにするため）
        node.link.send(packets.ConfigGet(param_id=packets.Param.TC_ENABLE))
        node.link.send(packets.ConfigGet(param_id=packets.Param.TV_ENABLE))
        node.link.send(packets.ConfigGet(param_id=packets.Param.WHEEL_LIFT_GUARD_ENABLE))
        # ★v0.12: 自動停止の安全マージン[cm]。CONFIG_SET はFlash非永続化なので、
        # GUI操作前でもSTM32起動直後の実際の値（既定15cm）をGUIに出せるようにする
        node.link.send(packets.ConfigGet(param_id=packets.Param.AUTO_STOP_MARGIN_CM))

    # ★v0.11: 車両の物理的な上限値。RC/AUTO 問わず _send_command が毎回これで
    # クランプする。受信済みなら無条件にこちらを使う（2026-08-22、--max-speed/
    # --max-steer との「小さい方」から変更。詳細は _send_command）
    lim = node.state.limits
    if lim is not None:
        print(f"# STM32 上限値(LIMITS): 速度{lim.max_speed_m_s:.2f}m/s "
              f"加速度{lim.max_accel_m_s2:.2f}m/s² トルク{lim.max_torque_nm:.3f}N·m "
              f"舵角±{math.degrees(lim.max_steer_rad):.1f}°")
    else:
        print("!! LIMITS 応答なし。Pi 側の既定上限（--max-speed/--max-steer）のみでクランプする"
              "（受信でき次第そちらへ切り替わる）")

    if args.allow_arm:
        eff_speed = lim.max_speed_m_s if lim else args.max_speed
        eff_steer = lim.max_steer_rad if lim else args.max_steer
        source = "LIMITS" if lim else f"--max-speed={args.max_speed}/--max-steer={args.max_steer}（未受信のフォールバック）"
        print("#\n#  ★★ --allow-arm 指定。バスの cmd が STM32 に届く＝モータが回りうる。")
        print(f"#     実効クランプ: 速度 ±{eff_speed:.3f} m/s / 舵角 ±{eff_steer:.3f} rad（出所: {source}）")
        print(f"#     cmd が {CMD_TIMEOUT_NS // 1_000_000}ms 途絶したら DISARM に落ちる")
        print("#     車輪を浮かせるか、周囲に人と物が無いことを確認すること\n")
    else:
        print("# COMMAND は常に DISARM（安全）。--allow-arm で解禁。Ctrl-C で停止。\n")
    if heartbeat is not None:
        node._log_event("heartbeat_start", {"pin": PIN_HEARTBEAT, "hz": heartbeat.hz})
        heartbeat.start()
    try:
        node.run(duration_s=args.duration,
                 status_cb=None if args.quiet else _status_line)
    finally:
        # 終了時にも DISARM を一発
        # UART が既に死んでいれば送れないが、**その場合 STM32 側は
        # `uart_timeout` で自力で DISARM に落ちる**ので、ここで止まる必要は無い
        with quiet_close("終了時の DISARM 送信"):
            node._send_command_disarm()
        # 総走行距離の最終保存。**間引き条件（1m以上）を無視して無条件に書く**——
        # 正常終了の取りこぼし（直近30秒未満の差分）をここで拾う
        _save_odometer_base(node.bridge.state_builder.odom_center)
        if heartbeat is not None:
            # 波形を止める＝E-Stop を掛けて終わる。正常終了でもそうする。
            # Pi が監視をやめた以上、車両を止めておくのが正しい
            heartbeat.stop()
            node._log_event("heartbeat_stop", heartbeat.stats.as_dict())
            print("\n!! ハートビート停止 → STM32 は 50ms 後に E-STOP をラッチする。"
                  "\n   次に走らせる前に車両のボタン2を押して解除すること")
        if indicator is not None:
            indicator.close()
        # **`log` ではなく `node._log` を見る。** `log/ctrl` で記録が途中から
        # 始まった／途中で止められた場合、起動時の `log` 変数はもう最新ではない
        if node._log is not None:
            node._log_linkstats(time.monotonic_ns())
            node._log.close()
        link.close()
        if sub is not None:
            sub.close()
        if pub is not None:
            pub.close()

    st = link.stats
    print("\n\n=== 終了時の統計 ===")
    print(f"frames_ok={st.frame_ok} crc_err={st.crc_error} len_err={st.len_error} "
          f"loss={st.packet_loss} reordered={st.reordered} dup={st.duplicate} "
          f"unknown={st.unknown_type} resync={st.resync_bytes}B")
    print(f"time sync: n={node.sync.n_samples} "
          f"offset={node.sync.offset_ns} ns best_delay={node.sync.best_delay_ns} ns "
          f"drift={node.sync.drift_ppm} ppm")
    if node.state.lidar_sectors:
        print(f"lidar sectors seen: {len(node.state.lidar_sectors)}/12")
    b = node.bridge
    print(f"bus: vehicle_state={b.vehicle_states} scan={b.published_scans} "
          f"lidar_sectors_lost={b.scans.sectors_lost} "
          f"cmd_rtt={'—' if b.cmd_rtt_ms is None else f'{b.cmd_rtt_ms:.1f}ms'} "
          f"cmd_timeout={node.cmd_timeouts}回 "
          f"odom_center={b.state_builder.odom_center:.3f}m "
          f"(jump={b.state_builder.odom_jumps})")
    if heartbeat is not None:
        st = heartbeat.stats
        hz = st.edges / 2 / args.duration if args.duration else None
        print(f"heartbeat: edges={st.edges}"
              + (f" ({hz:.1f}Hz)" if hz else "")
              + f" 最大遅れ={st.max_late_ns / 1e6:.2f}ms "
                f"停止={st.stalls}回 skip={st.skipped} {st.late_hist}")
    if node._log is not None:
        lg = node._log
        mb = lg.stats.bytes_written / 1e6
        print(f"log: {lg.path} rx={lg.stats.rx} tx={lg.stats.tx} "
              f"events={lg.stats.events} {mb:.1f}MB"
              + (f" !! 書き込みエラー {node._log_errors} 回" if node._log_errors else ""))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
