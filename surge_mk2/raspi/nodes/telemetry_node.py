"""telemetry_node — バス ⇄ PC(GUI) の WebSocket サーバ（`docs/architecture.md` §9）。

    .venv/bin/python -m raspi.nodes.telemetry_node               # :8000 で待ち受け
    .venv/bin/python -m raspi.nodes.telemetry_node --port 8080
    .venv/bin/python -m raspi.nodes.telemetry_node --no-camera   # JPEG 配信を切る

ブラウザから http://surge.local:8000/ を開けば GUI（`gui/dist`）が出る。
`gui/dist` が無ければビルド方法を書いたページを返す。

## チャンネルを4本に分ける（§9.2）

**1本にまとめない。** カメラのフレームが詰まった瞬間にラジコン入力まで遅延する。

| パス | 内容 | 形式 | 頻度 |
|---|---|---|---|
| `/ws/telemetry` | 車両状態・点群・リンク診断 | msgpack バイナリ | 20Hz |
| `/ws/camera/front`, `/ws/camera/rear` | JPEG | バイナリ | 最大15Hz |
| `/ws/control` | ラジコン入力・操縦権・E-Stop・記録の操作 | JSON | イベント + 20Hz |
| `/ws/map` | 地図・中心線・レーシングライン | msgpack バイナリ | **変わったときだけ** |
| `/ws/record` | mcap記録の生バイト列 | バイナリ | 録画中のみ |

点群を JSON で送ると CPU を無駄に食うのでテレメトリはバイナリにする。

## 記録は2本とも「開始/停止」だけを `/ws/control` で操作する（`docs/architecture.md` §11）

- **`.sfl`**: `io_node` の実時間ループが直接 Pi のSDに書く（本ノードはバス経由で
  「録ってほしい/やめてほしい」という意思を `log/ctrl` に流すだけ）。Wi-Fi が
  切れても Pi 単体で記録が続くのがこの形式の価値なので、ネットワーク越しには
  中継しない。ファイルは `/ws/control` の `logs_list`/`logs_delete`、
  および `GET /logs/<name>` でGUIから一覧・ダウンロード・削除できる。
- **`.mcap`**: `logger_node -o -` をサブプロセスとして起動し、標準出力（＝MCAP
  バイナリそのもの）を `/ws/record` へそのまま中継する。**Piのディスクには
  一度も書かない**（`tools/record.sh` が SSH パイプでやっているのと同じこと）。
  ブラウザ側がバイト列を保持し、停止時に手元のPCへダウンロードする。

**`.sfl` の再生（`replay_node --bus`）は本ノードには無い。** 実車の `surge-io`
と同じ `io` エンドポイントを取り合うため、Pi上でGUI経由にすると「使うには先に
SSHで surge-io を止める」手順が結局要り、GUIに置く意味が薄い。実車なしで
知覚/SLAM/経路生成を開発したいだけなら、Mac側でローカルに
`replay_node --bus` と本ノードを直接叩けば足りる（そちらは別のバスで動くので
実車と衝突しない）。

## 操縦権は同時に1人だけ

2つブラウザを開いて両方から舵を切ると、最後に届いた方が勝つ形になり
**「なぜか勝手にハンドルが戻る」**という再現困難な症状になる。
明示的に `take_control` した1クライアントだけが `cmd` を出せる。

## デッドマン — 3段構え

1. GUI がフォーカスを失う / キーを離すと送信を止める（ブラウザ側）
2. **本ノードは 150ms 指令が来なければ DISARM をバスに流す**（下記）
3. io_node は `cmd` が 150ms 途絶したら DISARM に落とす（telemetry_node ごと死んだ場合）

本ノードは**誰も操縦していなくても 50Hz で `cmd` を publish し続ける**。
「沈黙」ではなく「明示的な DISARM」を流すことで、
io_node 側の途絶判定は純粋に「telemetry_node が死んだか」だけを見ればよくなる。

## 自律走行は「中継」であって「別経路」ではない（§8）

`cmd` に publish するノードは**本ノードだけ**にしてある。planning_node は
`auto/cmd` に出し、engage されている間だけここが `cmd` へ差し替える。

    GUI  ──cmd(mode=AUTO, arm, 灯火…)──▶ telemetry_node ──cmd──▶ io_node
    planning_node ──auto/cmd(速度・舵)──▶      ↑ ここで速度と舵だけ差し替える

2つのノードが `cmd` に publish すると、購読側からは区別できないまま 50Hz で
交互に上書きし合う。**止める側が負けることがある**ので、経路は1本に保つ。

この形にすると、自律走行中も人間の安全網がそのまま生きる:

- **ARM は人間が GUI で保持する。** planning_node は arm を立てられない
- GUI が 50Hz で送り続けている限りだけ走る（150ms 途絶 → DISARM）
- E-Stop・操縦権の解放・接続断は、いずれも **engage も同時に落とす**
- `auto/cmd` が 200ms 途絶したら、engage したままでも**制動**に読み替える

差し替えるのは `target_speed` / `target_steer` / `brake` だけ。灯火・ホーン・
`auto_stop`・`brake_torque`・レートリミットは**GUI が送ってきた値のまま**通す。
自律走行中に前照灯の操作だけ効かなくなる理由が無いため。

## arm はここでは解禁できない

モータを回してよいかの判断は io_node の `--allow-arm` にしかない。
本ノードは `diag/link.arm_inhibited` を GUI に中継するだけで、自分では覆せない。
判断が2箇所にあると「どこかで解禁されていた」が起きる。
"""

from __future__ import annotations

import argparse
import asyncio
import http
import mimetypes
import signal
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

import msgspec  # noqa: E402
import websockets  # noqa: E402
from websockets.asyncio.server import serve  # noqa: E402
from websockets.datastructures import Headers  # noqa: E402
from websockets.http11 import Response  # noqa: E402

from raspi.auto import PLANNERS, catalog as auto_catalog, merged_params  # noqa: E402
from raspi.bus import LATEST, Publisher, Subscriber  # noqa: E402
from raspi.core.jpeg import RingJpeg  # noqa: E402
from raspi.io.fan import open_fan  # noqa: E402
from raspi.io.wifi import WifiState, open_wifi  # noqa: E402
from raspi.msgs import AutoCtrl, DriveCmd, Heartbeat as HbMsg, LogCtrl, UiEvent  # noqa: E402
from raspi.msgs.types import (  # noqa: E402
    TOPIC_AUTO_CMD,
    TOPIC_AUTO_CTRL,
    TOPIC_AUTO_MAP,
    TOPIC_AUTO_STATE,
    TOPIC_CMD,
    TOPIC_DIAG_LINK,
    TOPIC_HB_PREFIX,
    TOPIC_IMAGE_FRONT,
    TOPIC_IMAGE_REAR,
    TOPIC_LOG_CTRL,
    TOPIC_SCAN,
    TOPIC_UI_EVENT,
    TOPIC_VEHICLE_STATE,
)

__all__ = ["TelemetryServer"]

NS = 1_000_000_000
DEFAULT_PORT = 8000
REPO_ROOT = Path(__file__).resolve().parents[2]
GUI_DIST = REPO_ROOT / "gui" / "dist"
LOGS_DIR = REPO_ROOT / "logs"
#: 自動運転のモードとパラメータを覚えておく場所。**`engaged` は保存しない**
#: （電源投入で自律走行が始まる経路を作らない）
AUTO_CONF = REPO_ROOT / "config" / "auto.json"

TELEMETRY_HZ = 20
CMD_PUB_HZ = 50
CAMERA_HZ = 15
HB_HZ = 10
#: `.sfl` の意思（`log/ctrl`）・録画/再生ステータスの再送周期。
#: `_cmd_pump` と同じ理由（1回だけ送ると取りこぼしで食い違ったままになる）
LOG_CTRL_HZ = 1
#: 自動運転の意思（`auto/ctrl`）の再送周期。`log/ctrl` より速いのは、
#: **planning_node が落ちて上がり直したときに engage が届くまでの空白を短くする**ため
AUTO_CTRL_HZ = 5
#: 手動ファンデューティの再送周期。カーネルの thermal governor（`cooling_device0`）が
#: `pwm1_enable=1` のままでも温度のしきい値越えで `pwm1` を書き換えてくることがあり
#: （`raspi/io/fan.py` 参照）、GUI からの指定が 1 回きりだと巻き戻されたまま気づけない
FAN_PUMP_HZ = 1
#: Wi-Fi(SSID・電波強度)の再取得周期。`nmcli` のサブプロセス起動が数十〜百数十msかかる
#: ため 20Hz の `/ws/telemetry` には乗せず、`fan` と同じ低頻度ポーリングで `/ws/control` に載せる
WIFI_PUMP_HZ = 1
#: `auto/cmd` がこれだけ古ければ中継しない（＝制動に落とす）。
#: planning_node は 50Hz で出しているので 10 発ぶんの猶予
AUTO_CMD_STALE_NS = 200 * 1_000_000
#: バスを覗きに行く間隔。20Hz 配信に対して十分細かく、CPU は無視できる
BUS_POLL_S = 0.005
#: 操縦クライアントからの指令がこれだけ途絶したら DISARM に落とす（§9.4）
CMD_DEADMAN_NS = 150 * 1_000_000
#: mcap のライブ中継で1台あたりに焼く画像頻度の既定（`logger_node` と同じ値）
DEFAULT_MCAP_IMAGE_HZ = 5.0
#: 中継の読み取り単位
RECORD_CHUNK = 65536

_encoder = msgspec.msgpack.Encoder()
_json_encode = msgspec.json.encode
_json_decode = msgspec.json.decode

#: RasPi 本体（SoC）の温度。カーネルがミリ℃で出している
PI_THERMAL_ZONE = Path("/sys/class/thermal/thermal_zone0/temp")


def _read_pi_temp_c() -> float | None:
    """RasPi 本体の CPU 温度。GUI のラジコンビュー温度計に出す（2026-08-17）。

    `vcgencmd` はサブプロセス起動を挟む分だけ重く環境依存も増えるので使わない。
    このパスは Pi 5 / Debian Trixie でも変わらず、20Hz で毎回開いても軽いので
    キャッシュしない。**Pi 実機以外（Mac上のシムなど）ではファイルが無く None**
    になる——STM32 側の `vs.temp` が `null = MD が無言で信用できない` を示すのと
    同じ扱いにして、GUI 側のしきい値判定（`tempLevel()`）を1本にまとめている。
    """
    try:
        return int(PI_THERMAL_ZONE.read_text().strip()) / 1000.0
    except (OSError, ValueError):
        return None


# ── サーバ ──────────────────────────────────────────────────────────

class TelemetryServer:
    def __init__(self, *, port: int = DEFAULT_PORT, host: str = "0.0.0.0",
                 dist: Path = GUI_DIST, camera: bool = True,
                 jpeg_quality: int = 70, camera_hz: float = CAMERA_HZ) -> None:
        self.port = port
        self.host = host
        self.dist = dist
        self.camera_hz = camera_hz

        self.sub = Subscriber({
            TOPIC_VEHICLE_STATE: LATEST,
            TOPIC_SCAN: LATEST,
            TOPIC_DIAG_LINK: LATEST,
            TOPIC_IMAGE_FRONT: LATEST,
            TOPIC_IMAGE_REAR: LATEST,
            TOPIC_AUTO_CMD: LATEST,
            TOPIC_AUTO_STATE: LATEST,
            TOPIC_AUTO_MAP: LATEST,
        })
        self.pub = Publisher("control")

        self.telemetry_clients: set = set()
        self.control_clients: set = set()
        self.camera_clients: dict[str, set] = {"front": set(), "rear": set()}
        #: mcap のライブ中継を見ている `/ws/record` クライアント
        self.record_clients: set = set()
        #: 地図を見ている `/ws/map` クライアント
        self.map_clients: set = set()
        #: 最後に配った地図の版。**変わったときだけ流す**
        self._last_map_seq = -1

        #: 操縦権を持つ接続。**同時に1つだけ**
        self.controller = None
        self.controller_name = ""
        self._last_cmd: DriveCmd | None = None
        self._last_cmd_ns = 0
        #: デッドマンが働いて DISARM に落とした回数
        self.deadman_trips = 0
        self._why_released = ""

        self.logs_dir = LOGS_DIR

        #: `.sfl` を録ってほしいという「意思」。実際に開閉するのは io_node
        self._sfl_active = False

        # ── 自動運転（§8） ──
        #: 選ばれている planner の id。**空文字 = 自動運転しない**
        self._auto_mode = ""
        #: 人間が engage したか。**永続化しない**（起動しただけで走り出さない）
        self._auto_engaged = False
        #: planner のパラメータ。`config/auto.json` に保存され、次回起動で戻る
        self._auto_params: dict[str, float] = {}
        #: 「地図を確定」を押した回数。**保存しない**（電源投入で確定済みに
        #: なると、地図が無いのに走る段へ進んでしまう）
        self._auto_freeze_seq = 0
        self._auto_clear_seq = 0
        #: engage したまま `auto/cmd` が途絶して制動に落とした回数
        self.auto_stalls = 0
        self._auto_was_fresh = True
        self._load_auto_conf()

        # ── ファン（Pi5純正クーリング） ──
        #: **永続化しない。** Pi再起動をまたいで前回の手動固定値のまま
        #: 気づかず発熱し続ける事故を防ぐため、必ず自動から起動する
        self._fan = open_fan()
        self._fan_mode = "auto"
        self._fan_duty = 0.5
        # 前回異常終了で手動固定のまま残っていた場合の保険
        self._fan.set_auto()

        # ── Wi-Fi（SSID・電波強度） ──
        self._wifi = open_wifi()
        #: `_wifi_pump` が低頻度で更新するキャッシュ。読み取り自体が `nmcli` の
        #: サブプロセス起動を伴うため、`_control_status()` から毎回同期で呼ばない
        self._wifi_state = WifiState(ssid=None, rssi_dbm=None, available=False)

        #: mcap のライブ中継（`logger_node -o -` のサブプロセス）
        self._mcap_proc: asyncio.subprocess.Process | None = None
        self._mcap_started_ns = 0
        self._mcap_error: str | None = None


        self._last_scan_seq = -1
        self._running = False
        self.frames_sent = 0
        self.cmds_published = 0

        # エンコーダが無い環境は「カメラ配信なし」に落とす。**`None` に潰しておく**
        # ことで、以降の判定を `self._jpeg is None` の1種類に保つ
        self._jpeg = RingJpeg(jpeg_quality) if camera else None
        if self._jpeg is not None and not self._jpeg.ok:
            self._jpeg = None
        self._jpeg_impl = self._jpeg.impl if self._jpeg is not None else None

    # ── 静的ファイル（GUI 本体） ──

    def _serve_static(self, path: str) -> Response:
        rel = path.split("?")[0].lstrip("/") or "index.html"
        target = (self.dist / rel).resolve()
        # ディレクトリ外への脱出を許さない
        if not str(target).startswith(str(self.dist.resolve())):
            return _response(403, "text/plain; charset=utf-8", b"forbidden")
        if target.is_dir():
            target = target / "index.html"
        if not target.is_file():
            if not self.dist.is_dir():
                return _response(503, "text/html; charset=utf-8",
                                 _no_gui_page(self.dist))
            # SPA なので、知らないパスは index.html に落とす
            target = self.dist / "index.html"
            if not target.is_file():
                return _response(404, "text/plain; charset=utf-8", b"not found")
        ctype = mimetypes.guess_type(target.name)[0] or "application/octet-stream"
        if ctype.startswith("text/") or ctype.endswith(("javascript", "json")):
            ctype += "; charset=utf-8"
        return _response(200, ctype, target.read_bytes())

    def _serve_log_file(self, path: str) -> Response:
        """`GET /logs/<name>` — `.sfl`/`.mcap` のダウンロード。**GETのみ**。

        削除・一覧は `/ws/control` のJSONメッセージ（`logs_delete`/`logs_list`）
        で扱う。ダウンロードだけはブラウザの `<a download>` で完結させたいので
        HTTP に残してある。
        """
        from urllib.parse import unquote
        name = unquote(path[len("/logs/"):].split("?")[0])
        target = self._resolve_log_path(name)
        if target is None or not target.is_file():
            return _response(404, "text/plain; charset=utf-8", b"not found")
        return _response(200, "application/octet-stream", target.read_bytes(),
                         extra={"Content-Disposition": f'attachment; filename="{target.name}"'})

    def _process_request(self, connection, request):
        """WebSocket 以外のリクエストは静的ファイルまたはログのダウンロードとして返す。

        HTTP サーバを別プロセスに分けないのは、**GUI と WS を同一オリジンに
        載せるため**。別ポートにすると「開発中は動くのに Pi では繋がらない」
        という接続先の食い違いを毎回踏む。
        """
        if request.path.startswith("/ws/"):
            return None                       # WebSocket として処理させる
        if request.path.startswith("/logs/"):
            return self._serve_log_file(request.path)
        return self._serve_static(request.path)

    # ── WebSocket ──

    async def _handler(self, ws) -> None:
        path = ws.request.path.split("?")[0]
        if path == "/ws/telemetry":
            await self._telemetry_channel(ws)
        elif path == "/ws/control":
            await self._control_channel(ws)
        elif path.startswith("/ws/camera/"):
            await self._camera_channel(ws, path.rsplit("/", 1)[-1])
        elif path == "/ws/map":
            await self._map_channel(ws)
        elif path == "/ws/record":
            await self._record_channel(ws)
        else:
            await ws.close(1008, "unknown channel")

    async def _telemetry_channel(self, ws) -> None:
        self.telemetry_clients.add(ws)
        try:
            await ws.wait_closed()
        finally:
            self.telemetry_clients.discard(ws)

    async def _camera_channel(self, ws, cam: str) -> None:
        if cam not in self.camera_clients:
            await ws.close(1008, "unknown camera")
            return
        if self._jpeg is None:
            await ws.close(1011, "no jpeg encoder on the pi")
            return
        self.camera_clients[cam].add(ws)
        try:
            await ws.wait_closed()
        finally:
            self.camera_clients[cam].discard(ws)

    async def _map_channel(self, ws) -> None:
        """地図と経路を見るクライアント。**繋いだ瞬間に最新の1枚を送る。**

        地図は「変わったときだけ」流れる（凍結後は二度と変わらない）ので、
        後から画面を開いた人が**何も映らないまま待つ**ことになる。接続時に
        1枚送っておけば、その後は差分のように振る舞う。
        """
        self.map_clients.add(ws)
        try:
            m = self.sub.latest.get(TOPIC_AUTO_MAP)
            if m is not None:
                await ws.send(_encoder.encode(m))
            await ws.wait_closed()
        finally:
            self.map_clients.discard(ws)

    async def _record_channel(self, ws) -> None:
        """mcap のライブ中継を見るクライアント。**Piは中継するだけでSDに書かない。**"""
        self.record_clients.add(ws)
        try:
            await ws.wait_closed()
        finally:
            self.record_clients.discard(ws)
            if not self.record_clients and self._mcap_proc is not None:
                # 見ている人が誰もいなくなった録画を続ける意味は無い
                await self._mcap_stop()
                await self._broadcast_control_status()

    async def _control_channel(self, ws) -> None:
        self.control_clients.add(ws)
        # **GUI が繋がった合図。** io_node が拾ってブザーで接続音を鳴らす
        # （起動音・STM32 の起動音とは別の音形。`raspi/io/gpio.py` 参照）
        self.pub.send(TOPIC_UI_EVENT, UiEvent(kind="gui_connect"))
        await self._send_json(ws, self._control_status())
        try:
            async for raw in ws:
                await self._on_control(ws, raw)
        except websockets.exceptions.ConnectionClosed:
            pass
        finally:
            self.control_clients.discard(ws)
            if self.controller is ws:
                # **操縦者が消えたら即座に手放す。** タイムアウトを待たない
                self._release_control("接続が切れた")
                await self._broadcast_control_status()

    async def _on_control(self, ws, raw) -> None:
        try:
            m = _json_decode(raw)
        except Exception:
            return
        kind = m.get("type")

        if kind == "take_control":
            if self.controller is not None and self.controller is not ws:
                await self._send_json(ws, {"type": "control_denied",
                                           "holder": self.controller_name})
                return
            self.controller = ws
            self.controller_name = str(m.get("name", "gui"))
            self._last_cmd = None
            self._last_cmd_ns = 0
            await self._broadcast_control_status()

        elif kind == "release_control":
            if self.controller is ws:
                self._release_control("解放")
                await self._broadcast_control_status()

        elif kind == "cmd":
            if self.controller is not ws:
                return                        # 操縦権が無い接続の指令は捨てる
            self._last_cmd = DriveCmd(
                mode=int(m.get("mode", 0)),
                arm=bool(m.get("arm", False)),
                brake=bool(m.get("brake", False)),
                horn=bool(m.get("horn", False)),
                light_mode=int(m.get("light_mode", 0)),
                passing=bool(m.get("passing", False)),
                target_speed=float(m.get("speed", 0.0)),
                target_steer=float(m.get("steer", 0.0)),
                accel_limit=float(m.get("accel_limit", 0.0)),
                steer_rate_limit=float(m.get("steer_rate_limit", 0.0)),
                # **既定は 0 = 未指定 = STM32 の最大制動。** 古い GUI が繋がって
                # このキーを送ってこなくても、ブレーキが弱くなる方には転ばない
                brake_torque=float(m.get("brake_torque", 0.0)),
                # v0.6: 古い GUI はこのキーを送らないので、既定 False/0.0（速度指令のまま）
                torque_mode=bool(m.get("torque_mode", False)),
                target_torque=float(m.get("target_torque", 0.0)),
                # v0.7: 超音波の自動停止。**既定は False**（キーを送ってこない古い GUI に
                # 勝手な自動介入を足さない）。有効にするかは GUI の設定パネルで人間が決める
                auto_stop=bool(m.get("auto_stop", False)),
                source=f"gui:{self.controller_name}")
            self._last_cmd_ns = time.monotonic_ns()

        elif kind == "estop":
            # **誰であっても止められる。** 操縦権を持っていなくても通す
            self._last_cmd = None
            self._last_cmd_ns = 0
            self._release_control("E-Stop 要求")
            await self._broadcast_control_status()

        # ── 自動運転（誰でも操作できる。**解除だけは特に条件をつけない**） ──

        elif kind == "auto":
            self._on_auto(m)
            self._publish_auto_ctrl()      # 待たせない。engage は即座に効かせる
            await self._broadcast_control_status()

        # ── ファン（誰でも操作できる。灯火と同じ「状態」トグル） ──

        elif kind == "fan":
            self._on_fan(m)
            await self._broadcast_control_status()

        # ── TC/TV 有効切り替え（誰でも操作できる。★v0.8） ──
        # STM32側の適用結果は `diag/link`(tc_enabled/tv_enabled) 経由で戻ってくるので
        # ここでは broadcast しない（fan と違いサーバ内で完結する状態を持たないため）

        elif kind == "tc_tv":
            if "tc" in m:
                self.pub.send(TOPIC_UI_EVENT, UiEvent(kind="tc_enable", value=bool(m["tc"])))
            if "tv" in m:
                self.pub.send(TOPIC_UI_EVENT, UiEvent(kind="tv_enable", value=bool(m["tv"])))

        # ── 片輪浮き対策（誰でも操作できる。TC/TV本体とは独立した別機構。★v0.9） ──

        elif kind == "wheel_lift_guard":
            if "enabled" in m:
                self.pub.send(TOPIC_UI_EVENT,
                               UiEvent(kind="wheel_lift_guard_enable", value=bool(m["enabled"])))

        elif kind == "ping":
            await self._send_json(ws, {"type": "pong", "id": m.get("id"),
                                       "t_server": time.monotonic_ns()})

        # ── 記録（誰でも操作できる。走行の操縦権とは別物） ──

        elif kind == "sfl_record":
            self._sfl_active = bool(m.get("active", False))
            await self._broadcast_control_status()

        elif kind == "mcap_record_start":
            await self._mcap_start(image_hz=float(m.get("image_hz", DEFAULT_MCAP_IMAGE_HZ)))
            await self._broadcast_control_status()

        elif kind == "mcap_record_stop":
            await self._mcap_stop()
            await self._broadcast_control_status()

        elif kind == "logs_list":
            await self._send_json(ws, self._logs_list())

        elif kind == "logs_delete":
            self._logs_delete(str(m.get("name", "")))
            await self._send_json(ws, self._logs_list())

    def _release_control(self, why: str) -> None:
        self.controller = None
        self.controller_name = ""
        self._last_cmd = None
        self._last_cmd_ns = 0
        self._why_released = why
        # **操縦者が消えたら自律走行も解除する。** engage を残すと、次に誰かが
        # 操縦権を取って ARM した瞬間に、その人の意思と無関係に走り出す
        if self._auto_engaged:
            self._auto_engaged = False
            self._publish_auto_ctrl()

    # ── 自動運転（§8） ──

    def _on_auto(self, m: dict) -> None:
        """`{"type":"auto", "mode"?, "engaged"?, "params"?, "freeze_map"?, "clear_map"?}`。

        どれも省略可。**モードを変えたら engage は必ず落とす**（前のモードの
        つもりで engage したままアルゴリズムだけ入れ替わるのを防ぐ）。
        """
        if "mode" in m:
            mode = str(m.get("mode") or "")
            if mode and mode not in PLANNERS:
                return                     # 知らないモードは黙って捨てる
            if mode != self._auto_mode:
                self._auto_mode = mode
                self._auto_engaged = False
                self._auto_params = merged_params(mode, self._auto_params)
        if "params" in m and isinstance(m["params"], dict):
            raw = {k: v for k, v in m["params"].items() if isinstance(v, (int, float))}
            # **サーバ側でもクランプする。** GUI を信じて素通しにすると、
            # 古いタブや手書きの WS クライアントの値がそのまま planner に入る
            self._auto_params = merged_params(self._auto_mode,
                                              {**self._auto_params, **raw})
        if m.get("clear_map"):
            self._auto_clear_seq += 1
        if m.get("freeze_map"):
            # **回数で渡す。** `auto/ctrl` は現在の意思を繰り返し流すので、
            # 真偽値だと押していないのに毎回確定してしまう（`AutoCtrl.freeze_seq`）
            self._auto_freeze_seq += 1
        if "engaged" in m:
            want = bool(m.get("engaged"))
            # モードが無いのに engage はできない。**解除は常に通す**
            self._auto_engaged = want and bool(self._auto_mode)
            if self._auto_engaged:
                self.auto_stalls = 0
                self._auto_was_fresh = True
        self._save_auto_conf()

    def _auto_ctrl(self) -> AutoCtrl:
        return AutoCtrl(mode=self._auto_mode, engaged=self._auto_engaged,
                        params=dict(self._auto_params),
                        freeze_seq=self._auto_freeze_seq,
                        clear_seq=self._auto_clear_seq)

    def _publish_auto_ctrl(self) -> None:
        self.pub.send(TOPIC_AUTO_CTRL, self._auto_ctrl())

    def _load_auto_conf(self) -> None:
        """`config/auto.json` からモードとパラメータを戻す。**engage は戻さない。**"""
        try:
            raw = _json_decode(AUTO_CONF.read_bytes())
        except Exception:
            return                         # 無い・壊れている → 既定のまま
        mode = str(raw.get("mode") or "")
        self._auto_mode = mode if mode in PLANNERS else ""
        params = raw.get("params")
        self._auto_params = merged_params(
            self._auto_mode, params if isinstance(params, dict) else {})

    def _save_auto_conf(self) -> None:
        try:
            AUTO_CONF.parent.mkdir(parents=True, exist_ok=True)
            AUTO_CONF.write_bytes(_json_encode(
                {"mode": self._auto_mode, "params": self._auto_params}))
        except Exception:
            pass                           # 保存できなくても走行は続けられるべき

    # ── ファン（Pi5純正クーリング） ──

    def _on_fan(self, m: dict) -> None:
        """`{"type":"fan", "mode"?: "auto"|"manual", "duty"?: number}`。両方省略可。

        `_on_auto()` と同じく**サーバ側でも必ずクランプする**。実際に sysfs へ
        反映するのはここ1箇所だけにして、判断を散らさない。
        """
        if "duty" in m:
            raw = m.get("duty")
            if isinstance(raw, (int, float)):
                self._fan_duty = max(0.0, min(1.0, float(raw)))
        if "mode" in m:
            mode = str(m.get("mode") or "")
            if mode in ("auto", "manual"):
                self._fan_mode = mode
        if self._fan_mode == "manual" and self._fan.available:
            self._fan.set_manual(self._fan_duty)
        else:
            self._fan.set_auto()

    def _fan_status(self) -> dict:
        return {
            "mode": self._fan_mode,
            "duty": self._fan_duty,
            "available": self._fan.available,
            "rpm": self._fan.read_rpm(),
        }

    # ── Wi-Fi（SSID・電波強度） ──

    def _wifi_status(self) -> dict:
        st = self._wifi_state
        return {
            "ssid": st.ssid,
            "rssi_dbm": st.rssi_dbm,
            "available": st.available,
        }

    async def _wifi_pump(self) -> None:
        """Wi-Fi状態（SSID・電波強度）を低頻度で読み直し、変化をGUIへ流す。

        `nmcli` のサブプロセス起動はイベントループを塞ぐため `to_thread` に逃がす
        （`_camera_pump` のJPEGエンコードと同じ理由）。
        """
        period = 1.0 / WIFI_PUMP_HZ
        while self._running:
            await asyncio.sleep(period)
            if not self._wifi.available:
                continue
            self._wifi_state = await asyncio.to_thread(self._wifi.read)
            await self._broadcast_control_status()

    def _control_status(self) -> dict:
        link = self.sub.latest.get(TOPIC_DIAG_LINK)
        return {
            "type": "status",
            "has_controller": self.controller is not None,
            "controller": self.controller_name,
            # **io_node が封印しているかどうか。ここでは覆せない**
            "arm_inhibited": link.arm_inhibited if link else True,
            "health": link.health if link else "INIT",
            "estop_active": link.estop_active if link else False,
            "drive_power_locked": link.drive_power_locked if link else False,
            # ★v0.8: STM32が実際に適用しているTC/TVの有効状態。CONFIG_ACKをまだ
            # 受け取っていなければ None（未確定）
            "tc_enabled": link.tc_enabled if link else None,
            "tv_enabled": link.tv_enabled if link else None,
            "wheel_lift_guard_enabled": link.wheel_lift_guard_enabled if link else None,
            "clients": {"telemetry": len(self.telemetry_clients),
                        "control": len(self.control_clients),
                        "camera": {k: len(v) for k, v in self.camera_clients.items()}},
            "camera_encoder": self._jpeg_impl,
            "deadman_trips": self.deadman_trips,
            "sfl": {"active": self._sfl_active},
            "mcap": self._mcap_status(),
            "auto": self._auto_status(),
            "fan": self._fan_status(),
            "wifi": self._wifi_status(),
        }

    def _auto_status(self) -> dict:
        """自動運転の意思と、**選べるモードの宣言そのもの**。

        `catalog` は `raspi/auto/registry.py` の内容をそのまま流している。
        GUI のモード選択もパラメータのスライダもこれ 1 つから組まれるので、
        **planner を足しても GUI は 1 行も変えなくてよい。**
        """
        return {
            "mode": self._auto_mode,
            "engaged": self._auto_engaged,
            "params": self._auto_params,
            "catalog": auto_catalog(),
            "stalls": self.auto_stalls,
        }

    def _mcap_status(self) -> dict:
        active = self._mcap_proc is not None
        elapsed = (time.monotonic_ns() - self._mcap_started_ns) / NS if active else 0.0
        return {"active": active, "elapsed_s": elapsed, "error": self._mcap_error}

    # ── mcap のライブ中継（Piのディスクには書かない） ──

    async def _mcap_start(self, *, image_hz: float) -> None:
        if self._mcap_proc is not None:
            return                            # 既に録画中
        self._mcap_error = None
        cmd = [sys.executable, "-u", "-m", "raspi.nodes.logger_node",
               "--quiet", "-o", "-", "--image-hz", str(image_hz)]
        try:
            proc = await asyncio.create_subprocess_exec(
                *cmd, cwd=str(REPO_ROOT),
                stdout=asyncio.subprocess.PIPE, stderr=asyncio.subprocess.DEVNULL)
        except Exception as e:
            self._mcap_error = f"logger_node を起動できない: {e}"
            return
        self._mcap_proc = proc
        self._mcap_started_ns = time.monotonic_ns()
        asyncio.create_task(self._mcap_pump(proc))

    async def _mcap_pump(self, proc: asyncio.subprocess.Process) -> None:
        """`logger_node -o -` の標準出力を `/ws/record` へ生中継する。

        **視聴者が居なくても読み続ける。** 読まないと子プロセスのパイプが詰まって
        書き込みがブロックしてしまう（Piには保存しないので読んだ分は捨てるだけ）。
        """
        assert proc.stdout is not None
        try:
            while True:
                chunk = await proc.stdout.read(RECORD_CHUNK)
                if not chunk:
                    break
                if not self.record_clients:
                    continue
                dead = []
                for ws in list(self.record_clients):
                    try:
                        await ws.send(chunk)
                    except Exception:
                        dead.append(ws)
                for ws in dead:
                    self.record_clients.discard(ws)
        finally:
            rc = await proc.wait()
            # `self._mcap_proc is proc` が False なら `_mcap_stop` が既に片付け済み
            # （＝人間が止めた）。ここで上書きすると正常終了がエラー扱いになる
            if self._mcap_proc is proc:
                if rc != 0:
                    self._mcap_error = f"logger_node が異常終了しました（rc={rc}）"
                self._mcap_proc = None
                asyncio.create_task(self._broadcast_control_status())
            # **中継が完全に終わった合図として `/ws/record` を切る。** MCAP の索引は
            # 最後に書かれるので、GUI 側はこの切断イベントを「もう来ない」の目印にして
            # ダウンロードを発火する（`record.ts` の `onClose`）
            for ws in list(self.record_clients):
                try:
                    await ws.close()
                except Exception:
                    pass

    async def _mcap_stop(self) -> None:
        proc = self._mcap_proc
        if proc is None:
            return
        self._mcap_proc = None
        try:
            proc.terminate()
        except ProcessLookupError:
            pass
        # 実際の終了待ち・末尾チャンク（MCAPの索引）の中継は `_mcap_pump` が続ける

    # ── `logs/` の一覧・ダウンロード・削除 ──
    #
    # **`.sfl` の再生はここでは扱わない。** `replay_node --bus` は実車の
    # `surge-io` と同じ `io` エンドポイントを取り合うため、Pi上でGUI経由にすると
    # 「使うにはまずSSHで surge-io を止める」という手順が結局要り、GUIボタンの
    # 意味が薄い。**実車なしで知覚/SLAM/経路生成を開発する**という本来の目的には
    # `replay_node --bus` をローカル（Mac）でCLIから直接叩けば十分足りる
    # （そちらは `.sfl` を記録したPiと無関係な別のバスで動くので衝突しない）。

    def _resolve_log_path(self, name: str) -> Path | None:
        """`logs/` の外に出るファイル名（`../`・絶対パス等）を弾く。"""
        if not name or "/" in name or "\\" in name or name in (".", ".."):
            return None
        base = self.logs_dir.resolve()
        target = (base / name).resolve()
        if target.parent != base:
            return None
        return target

    def _logs_list(self) -> dict:
        files = []
        if self.logs_dir.is_dir():
            for p in sorted(self.logs_dir.iterdir()):
                if not p.is_file() or p.suffix not in (".sfl", ".mcap"):
                    continue
                st = p.stat()
                files.append({"name": p.name, "kind": p.suffix.lstrip("."),
                              "size": st.st_size, "mtime": st.st_mtime})
        return {"type": "logs", "files": files}

    def _logs_delete(self, name: str) -> None:
        path = self._resolve_log_path(name)
        if path is None or not path.is_file():
            return
        path.unlink(missing_ok=True)

    async def _broadcast_control_status(self) -> None:
        st = self._control_status()
        await asyncio.gather(*(self._send_json(c, st) for c in list(self.control_clients)),
                             return_exceptions=True)

    @staticmethod
    async def _send_json(ws, obj) -> None:
        try:
            await ws.send(_json_encode(obj).decode())
        except Exception:
            pass

    # ── 定期タスク ──

    async def _bus_pump(self) -> None:
        """バスを吸い上げて `sub.latest` を新しく保つ。

        `zmq.asyncio` を使わず素の poll を短周期で回している。ソケットは
        数本・メッセージは数百バイトなので、200Hz で回しても負荷は測定限界以下。
        非同期版を混ぜるより、**再生でも実機でも同じ Subscriber を使える**方を取った。
        """
        while self._running:
            self.sub.poll(0)
            await asyncio.sleep(BUS_POLL_S)

    async def _map_pump(self) -> None:
        """地図を `/ws/map` へ。**版が変わったときだけ。**

        `_telemetry_pump`（20Hz）に相乗りさせない。地図は 400×400 あって、
        しかも凍結したら二度と変わらない。同じ頻度で流す理由が無い
        （`docs/architecture.md` §6.2 の「画像を流してはいけない」の趣旨）。
        """
        while self._running:
            await asyncio.sleep(0.5)
            m = self.sub.latest.get(TOPIC_AUTO_MAP)
            if m is None or m.map_seq == self._last_map_seq:
                continue
            self._last_map_seq = m.map_seq
            if not self.map_clients:
                continue
            payload = _encoder.encode(m)
            dead = []
            for ws in list(self.map_clients):
                try:
                    await ws.send(payload)
                except Exception:
                    dead.append(ws)
            for ws in dead:
                self.map_clients.discard(ws)

    async def _telemetry_pump(self) -> None:
        period = 1.0 / TELEMETRY_HZ
        while self._running:
            await asyncio.sleep(period)
            if not self.telemetry_clients:
                continue
            payload = self._snapshot()
            dead = []
            for ws in list(self.telemetry_clients):
                try:
                    await ws.send(payload)
                    self.frames_sent += 1
                except Exception:
                    dead.append(ws)
            for ws in dead:
                self.telemetry_clients.discard(ws)

    def _snapshot(self) -> bytes:
        vs = self.sub.latest.get(TOPIC_VEHICLE_STATE)
        link = self.sub.latest.get(TOPIC_DIAG_LINK)
        scan = self.sub.latest.get(TOPIC_SCAN)
        # 点群は 10Hz なので、20Hz の配信では半分が同じもの。**同じ周を2回送らない**
        if scan is not None and scan.seq == self._last_scan_seq:
            scan = None
        elif scan is not None:
            self._last_scan_seq = scan.seq
        return _encoder.encode({
            "t_server": time.monotonic_ns(),
            "vs": vs,
            "link": link,
            "scan": scan,
            # 自律走行の「判断の根拠」。**engage していなくても流れる**ので、
            # 手動走行しながら planner が何を選ぶかを見比べられる
            "auto": self.sub.latest.get(TOPIC_AUTO_STATE),
            "ctl": {"has_controller": self.controller is not None,
                    "controller": self.controller_name},
            # STM32 側の温度（`vs.temp`）とは別枠。RasPi 自体は STM32 のバスに乗らない
            "pi_temp_c": _read_pi_temp_c(),
        })

    async def _cmd_pump(self) -> None:
        """`cmd` を 50Hz で publish し続ける。**沈黙ではなく明示的な DISARM を流す。**

        操縦者がいない・指令が古い場合は DISARM。こうしておくと io_node 側の
        途絶判定は「telemetry_node が死んだか」だけを見ればよくなる。
        """
        period = 1.0 / CMD_PUB_HZ
        was_live = False
        while self._running:
            await asyncio.sleep(period)
            now = time.monotonic_ns()
            live = (self._last_cmd is not None
                    and (now - self._last_cmd_ns) <= CMD_DEADMAN_NS)
            if live:
                cmd = self._last_cmd
                if cmd.mode == 2 and self._auto_engaged and self._auto_mode:
                    cmd = self._merge_auto(cmd, now)
            else:
                if was_live:
                    self.deadman_trips += 1
                    asyncio.create_task(self._broadcast_control_status())
                cmd = DriveCmd(mode=0, source="deadman")
            was_live = live
            self.pub.send(TOPIC_CMD, cmd)
            self.cmds_published += 1

    def _merge_auto(self, gui: DriveCmd, now: int) -> DriveCmd:
        """GUI の `cmd` の**速度・舵・制動だけ**を `auto/cmd` で差し替える。

        差し替えないもの（灯火・ホーン・パッシング・`auto_stop`・`brake_torque`・
        レートリミット・`arm`）は GUI の値のまま通す。**自律走行中に前照灯の
        操作だけ効かなくなる理由が無い**し、`arm` は人間しか立てられない。

        `auto/cmd` が古ければ**制動**に落とす。engage したまま planning_node が
        死んだときに、最後の「走れ」が 150ms 残って壁に向かうのを防ぐ。
        `brake_torque` は GUI の値をそのまま使う（0 なら STM32 の最大制動）。
        """
        auto = self.sub.latest.get(TOPIC_AUTO_CMD)
        fresh = auto is not None and (now - auto.t_pub) <= AUTO_CMD_STALE_NS
        if not fresh:
            if self._auto_was_fresh:
                self.auto_stalls += 1
                asyncio.create_task(self._broadcast_control_status())
            self._auto_was_fresh = False
            return msgspec.structs.replace(
                gui, mode=2, brake=True, target_speed=0.0, target_steer=0.0,
                torque_mode=False, target_torque=0.0,
                source=f"auto:{self._auto_mode}:stale")
        self._auto_was_fresh = True
        # **人間のブレーキは自律中も必ず通る。** GUI は engage 解除も同時に投げるが、
        # その往復（数十 ms）のあいだブレーキが効かない時間を作らない
        return msgspec.structs.replace(
            gui, mode=2, brake=gui.brake or auto.brake,
            target_speed=auto.target_speed, target_steer=auto.target_steer,
            # 自律走行はトルク直接指令を使わない。**GUI 側の設定を持ち込まない**
            # （ラジコンのトルクモードが ON のまま engage されうる）
            torque_mode=False, target_torque=0.0,
            source=f"auto:{self._auto_mode}")

    async def _hb_pump(self) -> None:
        period = 1.0 / HB_HZ
        while self._running:
            await asyncio.sleep(period)
            self.pub.send(TOPIC_HB_PREFIX + "control",
                          HbMsg(node="control",
                                detail=f"ws={len(self.telemetry_clients)}"))

    async def _fan_pump(self) -> None:
        """ファンの意思（manualデューティ／autoへの追従）を低頻度で再適用する。

        `_log_ctrl_pump`/`_auto_ctrl_pump` と同じ理由。`_on_fan` はGUIからの
        メッセージが来た瞬間にしか書き込まないため：

        - manual時: そのあとカーネルの thermal governor が `pwm1` を上書き
          してくると（`raspi/io/fan.py` の「高温になると強制上書き」の記述
          どおり、温度がしきい値を跨ぐたびに低温側でも起こりうる）、GUI
          表示は manual/指定値のままなのに実回転数だけ落ちて気づけない
        - auto時: governor 自体が再評価を止めてしまい、`pwm1` が manual
          時の値に張り付いたまま下がらないことがある（同モジュール参照）

        どちらも定期的に `set_manual`/`set_auto` を再送することで、巻き
        戻され／固まっても最大1秒で復帰させる。
        """
        period = 1.0 / FAN_PUMP_HZ
        while self._running:
            await asyncio.sleep(period)
            if not self._fan.available:
                continue
            if self._fan_mode == "manual":
                self._fan.set_manual(self._fan_duty)
            else:
                self._fan.set_auto()

    async def _log_ctrl_pump(self) -> None:
        """`.sfl` の意思（`log/ctrl`）と、録画中の経過時間を低頻度で再送する。

        `_cmd_pump` と同じ理由（1回だけ送ると io_node の再起動や取りこぼしで
        食い違ったままになる）。録画中だけステータスも再送し、GUI の
        経過時間表示を進める（アイドル時は既存どおりイベント駆動のみ）。
        """
        period = 1.0 / LOG_CTRL_HZ
        while self._running:
            await asyncio.sleep(period)
            self.pub.send(TOPIC_LOG_CTRL, LogCtrl(active=self._sfl_active))
            if self._mcap_proc is not None:
                await self._broadcast_control_status()

    async def _auto_ctrl_pump(self) -> None:
        """自動運転の意思（`auto/ctrl`）を低頻度で再送する。

        `_log_ctrl_pump` と同じ理由（1回だけ送ると planning_node の再起動や
        取りこぼしで食い違ったままになる）。**変更時は `_on_control` が
        即座に 1 発送っている**ので、こちらは取りこぼしの保険。
        """
        period = 1.0 / AUTO_CTRL_HZ
        while self._running:
            await asyncio.sleep(period)
            self._publish_auto_ctrl()

    async def _camera_pump(self) -> None:
        """共有メモリから読んで JPEG にして送る。**購読者がいないときは何もしない。**

        エンコードは CPU を食うので別スレッド（`to_thread`）に出す。
        イベントループを塞ぐと 20Hz のテレメトリまで遅れる。
        """
        if self._jpeg is None:
            return
        period = 1.0 / self.camera_hz
        topics = {"front": TOPIC_IMAGE_FRONT, "rear": TOPIC_IMAGE_REAR}
        last_seq = {"front": -1, "rear": -1}
        while self._running:
            await asyncio.sleep(period)
            for cam, clients in self.camera_clients.items():
                if not clients:
                    continue
                ref = self.sub.latest.get(topics[cam])
                if ref is None or ref.ring_seq == last_seq[cam]:
                    continue
                last_seq[cam] = ref.ring_seq
                jpg = await asyncio.to_thread(self._encode_frame, ref)
                if jpg is None:
                    continue
                for ws in list(clients):
                    try:
                        await ws.send(jpg)
                    except Exception:
                        clients.discard(ws)

    def _encode_frame(self, ref) -> bytes | None:
        """`ImageRef` → JPEG。共有メモリからゼロコピーで読む（`core.jpeg`）。"""
        got = self._jpeg.encode_latest(ref.shm_name, expect_seq=ref.ring_seq)
        return got[0] if got else None

    # ── 起動 ──

    async def serve_forever(self, stop: asyncio.Future) -> None:
        self._running = True
        async with serve(self._handler, self.host, self.port,
                         process_request=self._process_request,
                         max_queue=8, ping_interval=20) as server:  # noqa: F841
            tasks = [asyncio.create_task(t()) for t in (
                self._bus_pump, self._telemetry_pump, self._map_pump, self._cmd_pump,
                self._camera_pump, self._hb_pump, self._log_ctrl_pump,
                self._auto_ctrl_pump, self._fan_pump, self._wifi_pump)]
            try:
                await stop
            finally:
                self._running = False
                for t in tasks:
                    t.cancel()
                await asyncio.gather(*tasks, return_exceptions=True)

    def close(self) -> None:
        # 終了時に一発 DISARM を置いてから消える。**自律走行の engage も落とす**
        # （planning_node だけが生き残っても、走れの意思が残らないように）
        try:
            self._auto_engaged = False
            self._publish_auto_ctrl()
            self.pub.send(TOPIC_CMD, DriveCmd(mode=0, source="shutdown"))
            self._fan.set_auto()   # プロセスが消えても手動固定を残さない
        except Exception:
            pass
        # mcap 中継のサブプロセスも道連れにする（ベストエフォート。
        # systemd 配下なら KillMode=control-group で既に片付いているはず）
        if self._mcap_proc is not None:
            try:
                self._mcap_proc.terminate()
            except ProcessLookupError:
                pass
        self.sub.close()
        self.pub.close()


# ── HTTP ヘルパ ─────────────────────────────────────────────────────

def _response(status: int, ctype: str, body: bytes, *, extra: dict[str, str] | None = None) -> Response:
    h = {
        "Content-Type": ctype,
        "Content-Length": str(len(body)),
        "Cache-Control": "no-store",
    }
    if extra:
        h.update(extra)
    return Response(status, http.HTTPStatus(status).phrase, Headers(h), body)


def _no_gui_page(dist: Path) -> bytes:
    return f"""<!doctype html><meta charset="utf-8">
<title>SURGE Mk.2 — GUI 未ビルド</title>
<style>body{{background:#111;color:#ddd;font:14px/1.7 ui-monospace,monospace;
padding:2rem;max-width:44rem;margin:auto}}code{{color:#7cf}}</style>
<h1>GUI がまだビルドされていません</h1>
<p>WebSocket は動いています。GUI を出すにはビルドしてください。</p>
<pre><code>cd surge_mk2/gui
npm install
npm run build      # -&gt; {dist}</code></pre>
<p>開発中は Vite の dev サーバを使うほうが速いです（自動リロードが効く）。</p>
<pre><code>npm run dev        # http://localhost:5173</code></pre>
<p>チャンネル: <code>/ws/telemetry</code> <code>/ws/control</code>
<code>/ws/camera/front</code> <code>/ws/camera/rear</code></p>
""".encode()


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", type=int, default=DEFAULT_PORT)
    ap.add_argument("--host", default="0.0.0.0")
    ap.add_argument("--dist", type=Path, default=GUI_DIST, help="GUI のビルド成果物")
    ap.add_argument("--no-camera", action="store_true", help="JPEG 配信をしない")
    ap.add_argument("--jpeg-quality", type=int, default=70)
    ap.add_argument("--camera-hz", type=float, default=CAMERA_HZ)
    args = ap.parse_args()

    srv = TelemetryServer(port=args.port, host=args.host, dist=args.dist,
                          camera=not args.no_camera,
                          jpeg_quality=args.jpeg_quality, camera_hz=args.camera_hz)

    print(f"# telemetry_node  http://{args.host}:{args.port}/  "
          f"(mDNS を入れてあれば http://surge.local:{args.port}/)")
    print(f"# GUI: {args.dist}" + ("" if args.dist.is_dir() else "  ← 未ビルド"))
    from raspi.bus import endpoints_for_topic
    print(f"# バス購読 {' '.join(endpoints_for_topic(TOPIC_VEHICLE_STATE))} ほか")
    print(f"# cmd publish {srv.pub.endpoint} @{CMD_PUB_HZ}Hz "
          f"(操縦者不在なら DISARM)")
    print(f"# JPEG エンコーダ: {srv._jpeg_impl or '無し（カメラ配信は無効）'}")
    print(f"# 自動運転 {', '.join(PLANNERS) or '（planner 無し）'}"
          f" / 起動時のモード {srv._auto_mode or 'なし'}"
          f"（engage は保存しない）")
    print("# arm の可否は io_node が持つ。ここでは解禁できない\n")

    async def run() -> None:
        loop = asyncio.get_running_loop()
        stop = loop.create_future()
        for sig in (signal.SIGINT, signal.SIGTERM):
            loop.add_signal_handler(sig, lambda: stop.done() or stop.set_result(None))
        await srv.serve_forever(stop)

    try:
        asyncio.run(run())
    finally:
        srv.close()
        print(f"\n=== 終了時の統計 ===\n"
              f"telemetry frames={srv.frames_sent} cmd published={srv.cmds_published} "
              f"deadman={srv.deadman_trips}回")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
