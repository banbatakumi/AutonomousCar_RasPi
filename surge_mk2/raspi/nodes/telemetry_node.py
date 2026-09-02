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

## 公開面の守り方（2026-08-21 のレビュー指摘）

制御 API を持つサーバなので、**既定は安全側**に倒してある。

| 層 | 何を止めるか |
|---|---|
| `--host` の既定が `127.0.0.1` | 「打ち忘れ」で外に開くのを止める。外に出すなら `--host 0.0.0.0` を明示 |
| Origin 検査（`_origin_ok`） | **クロスサイト WebSocket ハイジャック**。操縦中に別タブで開いた悪意あるページが `/ws/control` を張るのを弾く |
| 共有トークン（`--token`） | 同一ネットワーク上の第三者。実際には**隣の班の PC の誤接続**を止める意味が大きい |
| `engage` に操縦権を要求 | 操縦権を持たない接続が自律走行を**開始**するのを止める |

**止める操作には条件をつけない。** E-Stop・操縦権の解放・engage の解除は
トークンも操縦権も要求しない。止める指令に条件を足すと、
**一番止めたい状況で止まらない**設計になる。

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
import os
import secrets
import signal
import sys
import tempfile
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
from raspi.core.cleanup import failure_count, quiet_close, recent_failures  # noqa: E402
from raspi.core.vehicle import Vehicle  # noqa: E402
from raspi.core.jpeg import RingJpeg  # noqa: E402
from raspi.io.fan import open_fan  # noqa: E402
from raspi.io.wifi import WifiState, open_wifi  # noqa: E402
from raspi.msgs import (  # noqa: E402
    AutoCtrl,
    CamConfig,
    CamModelCtrl,
    DriveCmd,
    E2EModelCtrl,
    Heartbeat as HbMsg,
    LogCtrl,
    TargetRoiCtrl,
    UiEvent,
)
from raspi.msgs.schema import schema_version  # noqa: E402
from raspi.msgs.types import (  # noqa: E402
    TOPIC_AUTO_CMD,
    TOPIC_AUTO_CTRL,
    TOPIC_AUTO_MAP,
    TOPIC_AUTO_STATE,
    TOPIC_CAM_CONFIG,
    TOPIC_CAM_MASK,
    TOPIC_CAM_MODEL,
    TOPIC_CMD,
    TOPIC_DIAG_LINK,
    TOPIC_E2E_MODEL,
    TOPIC_HB_PREFIX,
    TOPIC_IMAGE_FRONT,
    TOPIC_IMAGE_REAR,
    TOPIC_LINE_CAM,
    TOPIC_LOG_CTRL,
    TOPIC_SCAN,
    TOPIC_TRACK_ROI,
    TOPIC_TRACK_TARGET,
    TOPIC_UI_EVENT,
    TOPIC_VEHICLE_STATE,
)

__all__ = ["TelemetryServer"]

NS = 1_000_000_000
DEFAULT_PORT = 8000
#: **既定は loopback。** 外から見せるには `--host 0.0.0.0` を明示的に打つ
#: （`raspi/setup/run_stack.sh` は打っている）。制御 API を持つサーバの既定が
#: 全インタフェース待ち受けだと、「打ち忘れ」ではなく「打たなかった日」が危険側に転ぶ
DEFAULT_HOST = "127.0.0.1"
REPO_ROOT = Path(__file__).resolve().parents[2]
GUI_DIST = REPO_ROOT / "gui" / "dist"
LOGS_DIR = REPO_ROOT / "logs"
#: 自動運転のモードとパラメータを覚えておく場所。**`engaged` は保存しない**
#: （電源投入で自律走行が始まる経路を作らない）
AUTO_CONF = REPO_ROOT / "config" / "auto.json"
#: カメラ capture 側の設定（後方カメラON/OFF・前後のFPS上限・GUI配信fps）を覚えておく場所
CAMERA_CONF = REPO_ROOT / "config" / "camera.json"
#: `cam_perception_node` へ渡す ONNX モデル（`<name>.onnx` + `<name>.json`）の置き場。
#: `ml_cam/export_onnx.py` の出力をここへ手動で配置する運用（`.gitignore` 済み）
MODELS_DIR = REPO_ROOT / "models"
#: 選ばれているモデル名を覚えておく場所。`AUTO_CONF`/`CAMERA_CONF` と同じ流儀
CAM_MODEL_CONF = REPO_ROOT / "config" / "cam_model.json"
#: `e2e_lidar` へ渡す ONNX モデルの置き場。**カメラ用（`MODELS_DIR`直下）とは別**
#: （前処理契約が違うので、同じ一覧に混ぜると選び間違いの温床になる。
#: `raspi/auto/e2e_lidar.py`・`ml_lidar/export_onnx_rl.py` 参照）
E2E_MODELS_DIR = MODELS_DIR / "e2e_lidar"
#: 選ばれている E2E LiDAR モデル名。`CAM_MODEL_CONF` と同じ流儀
E2E_MODEL_CONF = REPO_ROOT / "config" / "e2e_lidar_model.json"
#: 共有トークンの既定の置き場。**`.gitignore` 済み**。`--token` / `SURGE_TOKEN` が
#: 無ければここを読む。無ければトークン無し（＝従来どおり誰でも操縦権を取れる）
SECRET_PATH = REPO_ROOT / "config" / "secret.txt"

TELEMETRY_HZ = 20
CMD_PUB_HZ = 50
#: GUIへ配信する既定のJPEG頻度。当初は`docs/architecture.md` §9の帯域見積もり
#: （640×480 JPEG q70 @15fps ≒ 7Mbps）を理由に15で始めたが、2026-08-24に実機で
#: 前後カメラ同時30fps配信を検証——**帯域自体は問題なかった**。ただし当時の
#: `_camera_pump` は相対 `sleep` でドリフトしており実測22fps止まりだったのを
#: 絶対時刻スケジューリングに直して解消済み（同日）。以後は既定30
CAMERA_HZ = 30
HB_HZ = 10
#: capture側(camera_node)のFPS上限の許容範囲、および前後カメラそれぞれの既定値。
#: 2026-08-24の実機計測で「30→10fpsで全体消費電力が約13〜15%下がる・
#: 10→5fpsはほぼ横ばい」と分かった一方、GUI配信を30fpsにしても問題ないことも
#: 実機で確認できたため、**前カメラは自動運転の応答性を優先して既定30**（ほぼ
#: 常に画角内の変化を追う）、**後カメラはGUI表示とロギング専用**（駐車・後退の
#: 目視確認程度で足りる）なので既定10に分けてある
CAM_FRONT_FPS_DEFAULT = 30.0
CAM_REAR_FPS_DEFAULT = 10.0
CAM_FPS_MIN = 1.0
CAM_FPS_MAX = 30.0
#: capture側の意思（`cam/config`）の再送周期。`_fan_pump`/`_auto_ctrl_pump` と
#: 同じ理由（camera_node の再起動や取りこぼしで食い違ったままにならないように）
CAM_CONFIG_HZ = 1
#: カメラ画像を実際に使う自動運転モードの id（`raspi/auto/registry.py`）。
#: この集合に入っているモードが engage されている間だけ、telemetry_node は
#: 前カメラの capture fps をユーザー設定の上限を無視して `CAM_FPS_MAX` まで上げる。
#: 後方カメラはどの自動運転モードも使わない（GUI表示とロギング専用）ので対象外。
#: `follow_object` は `cam_track_node.py` が毎フレーム対象を追跡し続ける必要が
#: あるので他の2つと同じ扱いにする
CAMERA_AUTO_MODES = frozenset({"line_trace", "ftg_cam", "follow_object"})
#: `track/roi`（★対象追従のROI選択）の再送周期。`_auto_ctrl_pump`と同じ理由
#: （cam_track_node の再起動や取りこぼしで選択が食い違ったままにならないように）
TRACK_ROI_HZ = 5
#: `cam/model`（★モデル選択）の再送周期。`CAM_CONFIG_HZ` と同じ理由
#: （cam_perception_node の再起動や取りこぼしで食い違ったままにならないように）
CAM_MODEL_HZ = 1
#: `e2e/model`（★モデル選択）の再送周期。`CAM_MODEL_HZ` と同じ理由
E2E_MODEL_HZ = 1
#: `cam/mask`（`ftg_cam` の走行可否マスク）を中継するポーリング周期。
#: カメラ本編（`CAMERA_HZ`）ほどの滑らかさは要らないデバッグ表示用途なので、
#: 低めに抑えて CPU/帯域を節約する
CAM_MASK_POLL_HZ = 8
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
#: 安全タイマの正。**`config/vehicle.toml` の `[safety]` が唯一の出どころ**で、
#: io_node は同じファイルから、GUI は `config/generate.py` の生成物から読む
#: （2026-08-21 のレビュー 🟢11）
_SAFETY = Vehicle.load()
#: `auto/cmd` がこれだけ古ければ中継しない（＝制動に落とす）。
#: planning_node は 50Hz で出しているので 10 発ぶんの猶予
AUTO_CMD_STALE_NS = int(_SAFETY.auto_cmd_stale_ms * 1_000_000)
#: バスを覗きに行く間隔。20Hz 配信に対して十分細かく、CPU は無視できる
BUS_POLL_S = 0.005
#: 操縦クライアントからの指令がこれだけ途絶したら DISARM に落とす（§9.4）。
#: io_node の `CMD_TIMEOUT_NS` と**同じ値**（別々に判定するが数字は1つ）
CMD_DEADMAN_NS = int(_SAFETY.cmd_deadman_ms * 1_000_000)
#: mcap のライブ中継で1台あたりに焼く画像頻度の既定（`logger_node` と同じ値）
DEFAULT_MCAP_IMAGE_HZ = 5.0
#: 中継の読み取り単位
RECORD_CHUNK = 65536
#: `GET /logs/<name>` でメモリに載せてよい上限。これを超えたら 413 を返して
#: `scp` 運用に倒す。**全量を一度に `read_bytes()` すると Pi のメモリが尽きる**
#: （`image_hz=5` × カメラ2台で `.mcap` は毎時 1GB 近く育つ）
MAX_INLINE_LOG_BYTES = 64 * 1024 * 1024
#: 同一オリジン以外で許すオリジン。**Vite の dev サーバだけ。**
#: 本番（telemetry_node が GUI も配る）は Host ヘッダと突き合わせるので列挙しない
DEV_ORIGINS = frozenset(
    f"http://{h}:5173" for h in ("localhost", "127.0.0.1", "surge.local"))

#: `/ws/telemetry` の各フレームに載せる型定義の札（`raspi/msgs/schema.py`）。
#: **起動時に1回だけ計算する**（20Hz のホットパスで毎回ハッシュを取らない）
SCHEMA_VERSION = schema_version()

_encoder = msgspec.msgpack.Encoder()
_json_encode = msgspec.json.encode
_json_decode = msgspec.json.decode


def _atomic_write_bytes(path: Path, data: bytes) -> None:
    """設定ファイルを tmp ファイル + `os.replace()` でアトミックに書く。

    `write_bytes()` の直書きだと、書き込み中の電源断で壊れた（中身が
    半端な）ファイルが残りうる。`raspi/nodes/io_node.py` の
    `_save_odometer_base` と同じパターン（このリポジトリでの初出）。
    """
    path.parent.mkdir(parents=True, exist_ok=True)
    fd, tmp = tempfile.mkstemp(dir=str(path.parent), prefix=f".{path.name}-")
    with os.fdopen(fd, "wb") as f:
        f.write(data)
    os.replace(tmp, path)

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
    def __init__(self, *, port: int = DEFAULT_PORT, host: str = DEFAULT_HOST,
                 dist: Path = GUI_DIST, camera: bool = True,
                 jpeg_quality: int = 70, camera_hz: float = CAMERA_HZ,
                 token: str = "") -> None:
        self.port = port
        self.host = host
        self.dist = dist
        self.camera_hz = camera_hz
        #: 共有トークン。**空文字 = 検査しない。** 「操縦権を取る」「自律走行を
        #: 開始する」の2つだけに掛ける。止める側（E-Stop・解放・engage 解除）には
        #: 掛けない——**止める操作に条件をつけると、一番止めたい状況で止まらない**
        self.token = token
        #: トークン不一致で弾いた回数。診断タブに出す
        self.auth_rejects = 0
        #: Origin 不一致で弾いた回数
        self.origin_rejects = 0
        #: 型が壊れた `cmd` を捨てた回数
        self.bad_cmds = 0

        self.sub = Subscriber({
            TOPIC_VEHICLE_STATE: LATEST,
            TOPIC_SCAN: LATEST,
            TOPIC_DIAG_LINK: LATEST,
            TOPIC_IMAGE_FRONT: LATEST,
            TOPIC_IMAGE_REAR: LATEST,
            TOPIC_AUTO_CMD: LATEST,
            TOPIC_AUTO_STATE: LATEST,
            TOPIC_AUTO_MAP: LATEST,
            #: ライントレースの認識点（GUI がカメラ映像へ重畳する。§`_snapshot`）
            TOPIC_LINE_CAM: LATEST,
            #: `ftg_cam` の走行可否マスク（JPEG。GUI がカメラ映像へ重畳する。§`_mask_pump`）
            TOPIC_CAM_MASK: LATEST,
            #: `follow_object` の追跡結果（GUI がカメラ映像へ重畳する。§`_snapshot`）
            TOPIC_TRACK_TARGET: LATEST,
        })
        self.pub = Publisher("control")

        self.telemetry_clients: set = set()
        self.control_clients: set = set()
        #: `"mask"` は `ftg_cam` の走行可否マスク（`_mask_pump` が中継。
        #: front/rear と違い telemetry_node 自身のエンコーダを使わないので
        #: `_camera_channel` の `self._jpeg is None` チェックの対象外にしてある）
        self.camera_clients: dict[str, set] = {"front": set(), "rear": set(), "mask": set()}
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

        # ── カメラ capture 設定（後方ON/OFF・前後FPS上限・GUI配信fps） ──
        #: `config/camera.json` に保存され、次回起動で戻る（`auto.json` と同じ流儀）
        self._cam_rear_enabled = True
        self._cam_front_cap_hz = CAM_FRONT_FPS_DEFAULT
        self._cam_rear_cap_hz = CAM_REAR_FPS_DEFAULT
        self._load_camera_conf()          # camera_hz(GUI配信)もここで上書きされうる

        # ── カメラセグメンテーションモデルの選択（`ftg_cam` 用） ──
        #: 選ばれているモデル名。**空文字 = 未選択**（`config/cam_model.json` に
        #: 保存され次回起動で戻る。`engaged` は他の自動運転設定と同じく保存しない
        #: が、こちらはモデル選択そのものなので `auto.json` 側には持たせない）
        self._cam_model = ""
        self._load_cam_model_conf()

        # ── E2E LiDAR モデルの選択（`e2e_lidar` 用） ──
        #: 選ばれているモデル名。**空文字 = 未選択**（`config/e2e_lidar_model.json`に
        #: 保存され次回起動で戻る）。`_cam_model` と同じ流儀
        self._e2e_model = ""
        self._load_e2e_model_conf()

        # ── 対象追従（`follow_object`）のROI選択 ──
        #: GUIがドラッグ選択した矩形（正規化座標）。**永続化しない**
        #: （`_auto_freeze_seq`/`_auto_clear_seq` と同じ理由——電源投入で
        #: 前回の選択のまま追跡が始まる経路を作らない）
        self._track_roi_box = (0.0, 0.0, 0.0, 0.0)
        self._track_select_seq = 0
        self._track_clear_seq = 0

        #: mcap のライブ中継（`logger_node -o -` のサブプロセス）
        self._mcap_proc: asyncio.subprocess.Process | None = None
        #: `_mcap_start` の `await create_subprocess_exec` 中の再入防止フラグ
        self._mcap_starting = False
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
        # ディレクトリ外への脱出を許さない。**前方一致では駄目**——`dist` に対して
        # 兄弟の `dist-old/secret` が通ってしまう。パス要素単位で判定する
        if not target.is_relative_to(self.dist.resolve()):
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
        size = target.stat().st_size
        if size > MAX_INLINE_LOG_BYTES:
            # **全量をメモリに載せてから返す実装なので、上限を切る。**
            # 長時間走行の `.mcap` は毎時 1GB 近くになり、繰り返し叩かれると
            # それだけで Pi が OOM で落ちる。大きいログは scp で取る運用にする
            return _response(413, "text/plain; charset=utf-8",
                             f"{target.name} は {size >> 20}MB あります"
                             f"（上限 {MAX_INLINE_LOG_BYTES >> 20}MB）。"
                             f"scp で取得してください: "
                             f"scp pi@surge.local:~/surge_mk2/logs/{target.name} .".encode())
        return _response(200, "application/octet-stream", target.read_bytes(),
                         extra={"Content-Disposition": f'attachment; filename="{target.name}"'})

    # ── Origin 検査（クロスサイト WebSocket ハイジャック対策） ──

    @staticmethod
    def _origin_ok(request) -> bool:
        """ブラウザから来た接続のうち、**別サイトが張った WS を弾く**。

        WebSocket は同一オリジンポリシーの対象外で、preflight も飛ばない。
        操縦中に別タブで悪意あるページを開くと、そのページの JS がそのまま
        `ws://surge.local:8000/ws/control` を張れてしまう。ブラウザは `Origin`
        を必ず付けるので、**サーバが見さえすれば**この経路は塞げる。

        `Origin` が無い＝ブラウザ以外（curl・自作クライアント・`tools/`）。
        そちらは通す——クロスサイトの話ではないので、この検査の対象外。
        同一ネットワークの第三者を止めるのは Origin ではなくトークン（`self.token`）。

        許可の判定は **Host ヘッダとの一致**で行う。オリジンを定数で列挙すると、
        `surge.local` でも `192.168.x.x` でも繋ぎたい実運用で必ず取りこぼす。
        同一オリジンなら Origin は必ず `scheme://<Host と同じ>` になる。
        """
        origin = request.headers.get("Origin")
        if origin is None:
            return True
        host = request.headers.get("Host") or ""
        return origin in (f"http://{host}", f"https://{host}") or origin in DEV_ORIGINS

    def _process_request(self, connection, request):
        """WebSocket 以外のリクエストは静的ファイルまたはログのダウンロードとして返す。

        HTTP サーバを別プロセスに分けないのは、**GUI と WS を同一オリジンに
        載せるため**。別ポートにすると「開発中は動くのに Pi では繋がらない」
        という接続先の食い違いを毎回踏む。
        """
        if request.path.startswith("/ws/"):
            if not self._origin_ok(request):
                self.origin_rejects += 1
                return _response(403, "text/plain; charset=utf-8", b"bad origin")
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
        # "mask" は cam_perception_node が既に JPEG 化した `cam/mask` を
        # そのまま中継するだけ（`_mask_pump`）——telemetry_node 自身の
        # 共有メモリ用エンコーダ（`self._jpeg`）は使わないので、この要求から除外する
        if cam != "mask" and self._jpeg is None:
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
            if not self._token_ok(m):
                self.auth_rejects += 1
                await self._send_json(ws, {"type": "control_denied",
                                           "holder": "", "reason": "bad_token"})
                return
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
            try:
                cmd = self._decode_cmd(m)
            except (TypeError, ValueError):
                # **壊れた指令は捨てるだけ。接続は維持する。**
                # msgspec は NaN / 1e400 を弾くが `{"speed": "abc"}` は素通りし、
                # `float("abc")` が伝播すると制御チャネルのタスクごと死ぬ。
                # 切ると 150ms 後に DISARM して復帰に人手が要るので、
                # **GUI のバグ1つで走行中に操縦が切れる**方には倒さない
                self.bad_cmds += 1
                return
            self._last_cmd = cmd
            self._last_cmd_ns = time.monotonic_ns()

        elif kind == "estop":
            # **誰であっても止められる。** 操縦権を持っていなくても通す
            self._last_cmd = None
            self._last_cmd_ns = 0
            self._release_control("E-Stop 要求")
            await self._broadcast_control_status()

        # ── 自動運転（誰でも操作できる。**解除だけは特に条件をつけない**） ──

        elif kind == "auto":
            # **解除・停止は誰でも通す。開始だけ操縦権を要求する。**
            # engage も同じ経路を通していたので、操縦権を持たない第三者が
            # 自律走行を開始できてしまっていた。止める側の条件は増やさない
            if m.get("engaged") and self.controller is not ws:
                await self._send_json(ws, {"type": "control_denied",
                                           "holder": self.controller_name,
                                           "reason": "auto_engage"})
                return
            self._on_auto(m)
            self._publish_auto_ctrl()      # 待たせない。engage は即座に効かせる
            # **モード・engage が変わると前カメラの capture fps の希望も変わりうる**
            # （`CAMERA_AUTO_MODES` を engage した瞬間に上限まで上げたい）
            self._publish_cam_config()
            await self._broadcast_control_status()

        # ── ファン（誰でも操作できる。灯火と同じ「状態」トグル） ──

        elif kind == "fan":
            self._on_fan(m)
            await self._broadcast_control_status()

        # ── カメラ capture 設定（誰でも操作できる。ファンと同じ「状態」トグル） ──

        elif kind == "camera":
            self._on_camera(m)
            await self._broadcast_control_status()

        # ── カメラセグメンテーションモデルの選択（誰でも操作できる。
        #    `ftg_cam` を engage する前に選び直す想定——一覧は要求があった
        #    クライアントにだけ返す（`logs_list` と同じ流儀）） ──

        elif kind == "cam_model_list":
            await self._send_json(ws, self._cam_models_list())

        elif kind == "cam_model_select":
            self._on_cam_model(m)
            await self._broadcast_control_status()

        # ── E2E LiDAR モデルの選択（誰でも操作できる。`cam_model_list`/`select`
        #    と全く同じ形。`e2e_lidar` を engage する前に選び直す想定） ──

        elif kind == "e2e_model_list":
            await self._send_json(ws, self._e2e_models_list())

        elif kind == "e2e_model_select":
            self._on_e2e_model(m)
            await self._broadcast_control_status()

        # ── 対象追従（`follow_object`）のROI選択（誰でも操作できる。
        #    `AutoCtrl.freeze_seq`/`clear_map` と同じ「回数」の約束） ──

        elif kind == "track_roi_select":
            self._on_track_roi_select(m)

        elif kind == "track_roi_clear":
            self._on_track_roi_clear()

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

        # ── 自動停止の安全マージン[cm]（誰でも操作できる。★v0.12） ──
        # STM32側の適用結果は tc_tv/wheel_lift_guard と同様
        # `diag/link`(auto_stop_margin_cm) 経由で戻ってくるのでここでは broadcast しない

        elif kind == "auto_stop_margin":
            if "margin_cm" in m:
                self.pub.send(TOPIC_UI_EVENT,
                               UiEvent(kind="auto_stop_margin_cm", float_value=float(m["margin_cm"])))

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

        # ── シャットダウン（誰でも実行できる。estop と同じく、走行の操縦権とは無関係。
        #    誤操作は GUI 側の window.confirm で防ぐ——操縦権を要求すると
        #    「操縦権が無い接続には holder="" の control_denied が返るだけで GUI に
        #    何も表示されない」という無反応バグになる。2026-08-25 実機で発覚） ──

        elif kind == "shutdown":
            await self._shutdown_pi()

    async def _shutdown_pi(self) -> None:
        """Pi を安全にシャットダウンする。`raspi/setup/install_services.sh` が入れる
        `sudoers.d` で、この固定コマンドだけ NOPASSWD 許可されている前提"""
        try:
            await asyncio.create_subprocess_exec("sudo", "/sbin/shutdown", "-h", "now")
        except Exception:
            pass  # sudoers 未設定等。GUI 側は反応が無ければ気付ける

    def _token_ok(self, m: dict) -> bool:
        """共有トークンの照合。**未設定なら常に通す**（従来どおりの挙動）。

        学内デモで攻撃者を想定するというより、**隣の班の PC が誤接続する事故**を
        防ぐ意味が大きい。だから掛けるのは「操縦権を取る」ときだけで、
        止める操作（E-Stop・解放）には掛けない。
        """
        if not self.token:
            return True
        return secrets.compare_digest(str(m.get("token", "")), self.token)

    def _decode_cmd(self, m: dict) -> DriveCmd:
        """`{"type":"cmd", ...}` → `DriveCmd`。**`float()`/`int()` をここに閉じる。**

        壊れた値は `TypeError` / `ValueError` として呼び元に返す。呼び元は
        捨てるだけで接続を維持する（切ると 150ms 後に DISARM してしまう）。
        """
        return DriveCmd(
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
            # v0.13: サイドブレーキ。**ここへの追加が漏れていたため、GUI 側は
            # 「要求中」と表示されるのに実際には一切 STM32 へ届いていなかった**
            # （2026-08-30 発覚）。DriveCmd に新しいトグル系フィールドを足したら
            # 必ずここも直すこと——型を直しただけでは WS の JSON から拾われない
            side_brake=bool(m.get("side_brake", False)),
            # v0.14: ウィンカー。side_brake と同じ理由で追加
            winker_left=bool(m.get("winker_left", False)),
            winker_right=bool(m.get("winker_right", False)),
            source=f"gui:{self.controller_name}")

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
            _atomic_write_bytes(AUTO_CONF, _json_encode(
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

    # ── カメラ capture 設定（後方ON/OFF・前後FPS上限・GUI配信fps） ──

    @staticmethod
    def _clamp_cam_hz(v: float) -> float:
        return max(CAM_FPS_MIN, min(CAM_FPS_MAX, v))

    def _on_camera(self, m: dict) -> None:
        """`{"type":"camera", "rear_enabled"?, "front_cap_hz"?, "rear_cap_hz"?, "gui_hz"?}`。

        どれも省略可。`_on_auto`/`_on_fan` と同じく**サーバ側でも必ずクランプする**。
        `front_cap_hz`/`rear_cap_hz` は capture 側（camera_node）のFPS上限、
        `gui_hz` はブラウザへ配信するJPEGの頻度で、**別物**（前者は電力、
        後者はWi-Fi帯域が理由）。
        """
        if "rear_enabled" in m:
            self._cam_rear_enabled = bool(m["rear_enabled"])
        if "front_cap_hz" in m and isinstance(m["front_cap_hz"], (int, float)):
            self._cam_front_cap_hz = self._clamp_cam_hz(float(m["front_cap_hz"]))
        if "rear_cap_hz" in m and isinstance(m["rear_cap_hz"], (int, float)):
            self._cam_rear_cap_hz = self._clamp_cam_hz(float(m["rear_cap_hz"]))
        if "gui_hz" in m and isinstance(m["gui_hz"], (int, float)):
            self.camera_hz = self._clamp_cam_hz(float(m["gui_hz"]))
        self._save_camera_conf()
        self._publish_cam_config()      # 待たせない。fan と同じく即座に効かせる

    def _desired_front_fps(self) -> float:
        """前カメラの capture fps。**カメラを使う自動運転が engage 中は上限を無視する。**

        `line_trace`/`ftg_cam` はライン検出・走行可能領域セグメンテーションに
        毎フレーム新しい画を欲しがる。ユーザーの節電設定（既定15fps）より
        走行の安全性を優先する。
        """
        if self._auto_engaged and self._auto_mode in CAMERA_AUTO_MODES:
            return CAM_FPS_MAX
        return self._cam_front_cap_hz

    def _camera_config_status(self) -> dict:
        return {
            "rear_enabled": self._cam_rear_enabled,
            "front_cap_hz": self._cam_front_cap_hz,
            "rear_cap_hz": self._cam_rear_cap_hz,
            "gui_hz": self.camera_hz,
            "front_fps_effective": self._desired_front_fps(),
            "auto_override": self._auto_engaged and self._auto_mode in CAMERA_AUTO_MODES,
        }

    def _publish_cam_config(self) -> None:
        self.pub.send(TOPIC_CAM_CONFIG, CamConfig(
            front_fps=self._desired_front_fps(),
            rear_fps=self._cam_rear_cap_hz,
            rear_enabled=self._cam_rear_enabled))

    def _load_camera_conf(self) -> None:
        """`config/camera.json` から戻す。`_load_auto_conf` と同じ流儀。"""
        try:
            raw = _json_decode(CAMERA_CONF.read_bytes())
        except Exception:
            return
        if isinstance(raw.get("rear_enabled"), bool):
            self._cam_rear_enabled = raw["rear_enabled"]
        if isinstance(raw.get("front_cap_hz"), (int, float)):
            self._cam_front_cap_hz = self._clamp_cam_hz(float(raw["front_cap_hz"]))
        if isinstance(raw.get("rear_cap_hz"), (int, float)):
            self._cam_rear_cap_hz = self._clamp_cam_hz(float(raw["rear_cap_hz"]))
        if isinstance(raw.get("gui_hz"), (int, float)):
            self.camera_hz = self._clamp_cam_hz(float(raw["gui_hz"]))

    def _save_camera_conf(self) -> None:
        try:
            _atomic_write_bytes(CAMERA_CONF, _json_encode({
                "rear_enabled": self._cam_rear_enabled,
                "front_cap_hz": self._cam_front_cap_hz,
                "rear_cap_hz": self._cam_rear_cap_hz,
                "gui_hz": self.camera_hz,
            }))
        except Exception:
            pass

    # ── カメラセグメンテーションモデルの選択（`ftg_cam` 用） ──
    #
    # `cam_perception_node` を再起動もSSHも無しに切り替えるための口。
    # **走行開始（engage）ボタンを押す前に選び直す**という使い方を想定していて、
    # `_on_auto` と同じ「変えたら engage は必ず落とす」を踏襲する（このコード
    # ベースには「engage 中は操作を拒否する」という前例が無く、代わりに
    # 「変わったものを反映するために engage を落とす」という設計で統一されている
    # ——`_on_auto` のモード変更、`_release_control` の操縦者消失と同じ形）。

    def _cam_models_list(self) -> dict:
        """`models/` にある `.onnx` の一覧。`_logs_list` と同じ流儀
        （要求したクライアントにだけ返す。ブロードキャストしない）。

        **キー名は `files` ではなく `cam_model_files`。** `_logs_list()` の
        `files`（`LogFile[]`）と型が違うので、GUI 側の `ServerMsg`（型で
        振り分けられない1つの受け皿）で混ざらないよう分けてある。

        `note`（`ml_cam/export_onnx.py`が`<名前>.json`に書く自由記述の備考。
        `_e2e_models_list()`と同じ流儀。2026-08-29追加）も一緒に返す。
        `.json`が壊れていても一覧自体は壊さない（空文字で返す）。
        """
        files = []
        if MODELS_DIR.is_dir():
            for p in sorted(MODELS_DIR.iterdir()):
                if not p.is_file() or p.suffix != ".onnx":
                    continue
                st = p.stat()
                cfg_path = p.with_suffix(".json")
                note = ""
                if cfg_path.exists():
                    try:
                        note = str(_json_decode(cfg_path.read_bytes()).get("note", ""))
                    except Exception:                                    # noqa: BLE001
                        pass
                files.append({"name": p.stem, "size": st.st_size, "mtime": st.st_mtime,
                             "has_config": cfg_path.exists(), "note": note})
        return {"type": "cam_models", "cam_model_files": files}

    def _on_cam_model(self, m: dict) -> None:
        """`{"type":"cam_model_select", "name": str}`。

        `name` が `models/` に実在する `.onnx` と一致しなければ黙って捨てる
        （`_on_auto` が知らない planner id を捨てるのと同じ——GUI の一覧に
        出ていないものは選べないので、ここに来る時点で基本的に一致するはずだが、
        一覧取得後にファイルが削除された等のズレを机上でも弾いておく）。
        """
        name = str(m.get("name") or "")
        if name and not (MODELS_DIR / f"{name}.onnx").is_file():
            return
        if name == self._cam_model:
            return
        self._cam_model = name
        # **モデルを変えたら ftg_cam の engage は必ず落とす。** 前のモデルの
        # つもりで engage したまま推論だけ入れ替わるのを防ぐ（`_on_auto` の
        # 「モードを変えたら engage は必ず落とす」と同じ理由・同じ形）
        if self._auto_engaged and self._auto_mode == "ftg_cam":
            self._auto_engaged = False
            self._publish_auto_ctrl()
        self._save_cam_model_conf()
        self._publish_cam_model()          # 待たせない。cam_config と同じく即座に効かせる

    def _cam_model_status(self) -> dict:
        return {"name": self._cam_model}

    def _publish_cam_model(self) -> None:
        self.pub.send(TOPIC_CAM_MODEL, CamModelCtrl(name=self._cam_model))

    def _load_cam_model_conf(self) -> None:
        """`config/cam_model.json` から戻す。`_load_camera_conf` と同じ流儀。"""
        try:
            raw = _json_decode(CAM_MODEL_CONF.read_bytes())
        except Exception:
            return                         # 無い・壊れている → 既定（未選択）のまま
        name = raw.get("name")
        if isinstance(name, str) and (not name or (MODELS_DIR / f"{name}.onnx").is_file()):
            self._cam_model = name

    def _save_cam_model_conf(self) -> None:
        try:
            _atomic_write_bytes(CAM_MODEL_CONF, _json_encode({"name": self._cam_model}))
        except Exception:
            pass

    # ── E2E LiDAR モデルの選択（`e2e_lidar` 用） ──
    #
    # `_cam_model*` と全く同じ構造。`planning_node` を再起動もSSHも無しに
    # 切り替えるための口で、`e2e_lidar` を engage する前に選び直す想定。
    # 「変えたら engage を必ず落とす」も `_on_cam_model` と同じ理由で踏襲する。

    def _e2e_models_list(self) -> dict:
        """`models/e2e_lidar/` にある `.onnx` の一覧。`_cam_models_list()` と同じ流儀
        （要求したクライアントにだけ返す）。**キー名は `e2e_model_files`**
        （`cam_model_files` と型は同じだが、GUI側の受け皿で混ざらないよう分ける）。

        `note`（`ml_lidar/export_onnx_rl.py`が`<名前>.json`に書く自由記述の備考。
        2026-08-29追加）も一緒に返す——GUIのモデル選択でどんな変更・どのコースかを
        確認できるようにするため。`.json`が壊れていても一覧自体は壊さない（空文字で返す）。
        """
        files = []
        if E2E_MODELS_DIR.is_dir():
            for p in sorted(E2E_MODELS_DIR.iterdir()):
                if not p.is_file() or p.suffix != ".onnx":
                    continue
                st = p.stat()
                cfg_path = p.with_suffix(".json")
                note = ""
                if cfg_path.exists():
                    try:
                        note = str(_json_decode(cfg_path.read_bytes()).get("note", ""))
                    except Exception:                                    # noqa: BLE001
                        pass
                files.append({"name": p.stem, "size": st.st_size, "mtime": st.st_mtime,
                             "has_config": cfg_path.exists(), "note": note})
        return {"type": "e2e_models", "e2e_model_files": files}

    def _on_e2e_model(self, m: dict) -> None:
        """`{"type":"e2e_model_select", "name": str}`。

        `name` が `models/e2e_lidar/` に実在する `.onnx` と一致しなければ黙って捨てる
        （`_on_cam_model` と同じ理由）。
        """
        name = str(m.get("name") or "")
        if name and not (E2E_MODELS_DIR / f"{name}.onnx").is_file():
            return
        if name == self._e2e_model:
            return
        self._e2e_model = name
        # **モデルを変えたら e2e_lidar の engage は必ず落とす。**
        # `_on_cam_model` の「モデルを変えたら ftg_cam の engage を必ず落とす」と同じ理由
        if self._auto_engaged and self._auto_mode == "e2e_lidar":
            self._auto_engaged = False
            self._publish_auto_ctrl()
        self._save_e2e_model_conf()
        self._publish_e2e_model()          # 待たせない。cam_model と同じく即座に効かせる

    def _e2e_model_status(self) -> dict:
        return {"name": self._e2e_model}

    def _publish_e2e_model(self) -> None:
        self.pub.send(TOPIC_E2E_MODEL, E2EModelCtrl(name=self._e2e_model))

    def _load_e2e_model_conf(self) -> None:
        """`config/e2e_lidar_model.json` から戻す。`_load_cam_model_conf` と同じ流儀。"""
        try:
            raw = _json_decode(E2E_MODEL_CONF.read_bytes())
        except Exception:
            return
        name = raw.get("name")
        if isinstance(name, str) and (not name or (E2E_MODELS_DIR / f"{name}.onnx").is_file()):
            self._e2e_model = name

    def _save_e2e_model_conf(self) -> None:
        try:
            _atomic_write_bytes(E2E_MODEL_CONF, _json_encode({"name": self._e2e_model}))
        except Exception:
            pass

    # ── 対象追従（`follow_object`）のROI選択 ──
    #
    # `AutoCtrl.freeze_seq`/`clear_seq` と同じ「回数」の約束（真偽値だと、
    # このメッセージは現在の意思を繰り返し流す設計なので、選び直していないのに
    # 再送のたびに毎回選択が発生してしまう）。`cam_track_node.py` が
    # `select_seq`/`clear_seq` の増加だけを見て追跡の開始/終了を判断する。

    def _on_track_roi_select(self, m: dict) -> None:
        """`{"type":"track_roi_select", "x0","y0","x1","y1"}`（前方カメラ映像の
        正規化座標、0〜1）。**選び直すたびに `follow_object` の engage を必ず落とす**
        （`_on_cam_model`/`_on_e2e_model` の「モデルを変えたら engage を必ず落とす」
        と同じ理由——前の対象のつもりで engage したまま追跡対象だけ入れ替わるのを防ぐ）。
        """
        try:
            box = (float(m["x0"]), float(m["y0"]), float(m["x1"]), float(m["y1"]))
        except (KeyError, TypeError, ValueError):
            return                         # 壊れた座標は黙って捨てる
        self._track_roi_box = box
        self._track_select_seq += 1
        if self._auto_engaged and self._auto_mode == "follow_object":
            self._auto_engaged = False
            self._publish_auto_ctrl()
        self._publish_track_roi()          # 待たせない。cam_model 等と同じく即座に効かせる

    def _on_track_roi_clear(self) -> None:
        """「選択を解除」。**engage は落とさない**——`clear_map` と同じ形。
        `follow_object` は対象未選択に戻れば `FollowObject.plan()` が自然に
        `ready=False`（停止）へ倒れるので、ここで engage を強制的に落とす必要が無い。
        """
        self._track_clear_seq += 1
        self._publish_track_roi()

    def _track_roi_ctrl(self) -> TargetRoiCtrl:
        x0, y0, x1, y1 = self._track_roi_box
        return TargetRoiCtrl(x0=x0, y0=y0, x1=x1, y1=y1,
                             select_seq=self._track_select_seq,
                             clear_seq=self._track_clear_seq)

    def _publish_track_roi(self) -> None:
        self.pub.send(TOPIC_TRACK_ROI, self._track_roi_ctrl())

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
            # 車両の物理的な上限値（`LIMITS` パケット由来。★v0.11）は、ここ（`status`、
            # イベント発生時にしかbroadcastされない）ではなく `LinkDiag`（`/ws/telemetry`
            # 8Hz、`bus/live.ts`）から出す。`tc_enabled`/`wheel_lift_guard_enabled` と
            # 同じ理由（2026-08-19 実機で発覚）: status から読むと、受信が status の
            # 最後のbroadcastより後になった場合に GUI が更新されずリロードするまで
            # 気づかない
            "clients": {"telemetry": len(self.telemetry_clients),
                        "control": len(self.control_clients),
                        "camera": {k: len(v) for k, v in self.camera_clients.items()}},
            "camera_encoder": self._jpeg_impl,
            # JPEG エンコードの実測（2026-08-21 のレビュー 🟡7）。
            # **同じフレームを logger_node も焼いている。** 素朴な共通化は
            # 「誰も見ていない間も camera_node が焼き続ける」形になって idle が
            # 悪化するので、まず本当にボトルネックかをここで見えるようにした
            "camera_jpeg": self._jpeg.stats() if self._jpeg is not None else None,
            "deadman_trips": self.deadman_trips,
            # **無言で捨てた回数を必ず表に出す。** 捨てるだけの処理は、
            # 出さないと「効いていない」と「そもそも来ていない」が区別できない
            "auth_required": bool(self.token),
            "auth_rejects": self.auth_rejects,
            "origin_rejects": self.origin_rejects,
            "bad_cmds": self.bad_cmds,
            "sfl": {"active": self._sfl_active},
            "mcap": self._mcap_status(),
            "auto": self._auto_status(),
            "fan": self._fan_status(),
            "wifi": self._wifi_status(),
            "camera_config": self._camera_config_status(),
            "cam_model": self._cam_model_status(),
            "e2e_model": self._e2e_model_status(),
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
        if self._mcap_proc is not None or self._mcap_starting:
            return                            # 既に録画中、または起動処理の途中
        # **`await create_subprocess_exec` の間だけこのフラグで再入を防ぐ。**
        # `self._mcap_proc is not None` のチェックと代入の間には中断点（await）
        # があり、素通しだと短時間の二重呼び出しで logger_node を二重起動して
        # しまう（録画データの混線・先発プロセスのリーク）
        self._mcap_starting = True
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
        finally:
            self._mcap_starting = False
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
                await asyncio.gather(
                    *(self._send_or_drop(ws, chunk, self.record_clients)
                      for ws in list(self.record_clients)),
                    return_exceptions=True)
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
                # 既に相手が切っていれば例外になる。**それはむしろ望んだ状態**
                with quiet_close("/ws/record の切断（mcap 中継の終了合図）"):
                    await ws.close()

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
    async def _send_or_drop(ws, payload, clients: set) -> bool:
        """1クライアントへバイト列を1本送る。**失敗したら即座に `clients` から外す。**

        呼び出し側が `asyncio.gather` でクライアントぶん並行に呼ぶための単位。
        直列に `await` すると、1台の送信が詰まった（TCP輻輳・受信停止）ぶんだけ
        同じループ内の他クライアントへの配信も足止めされる（`websockets` の
        `send()` にタイムアウトが無いため）。1タスクずつ独立させれば、
        詰まったクライアントだけが取り残される。
        """
        try:
            await ws.send(payload)
            return True
        except Exception:
            clients.discard(ws)
            return False

    @staticmethod
    async def _send_json(ws, obj) -> None:
        """1クライアントへ JSON を1本送る。**送れなくても黙って捨てる。**

        ここで例外が漏れると `_broadcast_control_status()` の
        `asyncio.gather` が全クライアントぶん巻き添えになる。切れた接続は
        `_control_channel` の `finally` が片付けるので、ここでは何もしない
        （後始末ではないが「捨ててよい」根拠が別にある形。`quiet_close` は
        使わない——20Hz で回るので記録が環状バッファを埋め尽くす）。
        """
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
            await asyncio.gather(
                *(self._send_or_drop(ws, payload, self.map_clients)
                  for ws in list(self.map_clients)),
                return_exceptions=True)

    async def _telemetry_pump(self) -> None:
        period = 1.0 / TELEMETRY_HZ
        while self._running:
            await asyncio.sleep(period)
            if not self.telemetry_clients:
                continue
            payload = self._snapshot()
            results = await asyncio.gather(
                *(self._send_or_drop(ws, payload, self.telemetry_clients)
                  for ws in list(self.telemetry_clients)),
                return_exceptions=True)
            self.frames_sent += sum(1 for r in results if r is True)

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
            # **型定義そのものから決まる札。** GUI はこれが自分の持つ値と違えば
            # そのフレームを捨てて警告を出す。Pi だけ新しくなった GUI が、
            # 消えたフィールドを静かに `undefined` として読んで
            # 空欄や NaN を出したまま動き続けるのを止める（レビュー 🟠3）
            "schema": SCHEMA_VERSION,
            "t_server": time.monotonic_ns(),
            "vs": vs,
            "link": link,
            "scan": scan,
            # 自律走行の「判断の根拠」。**engage していなくても流れる**ので、
            # 手動走行しながら planner が何を選ぶかを見比べられる
            "auto": self.sub.latest.get(TOPIC_AUTO_STATE),
            # `line_trace` が認識している白線の目標点。**engage していなくても流れる**
            # （`auto` と同じ理由——手動走行中でも見え方を確認できた方が良い）
            "line_cam": self.sub.latest.get(TOPIC_LINE_CAM),
            # `follow_object` の追跡結果。**engage していなくても流れる**
            # （`auto`/`line_cam` と同じ理由——対象を選んだだけで見え方を確認できる）
            "track": self.sub.latest.get(TOPIC_TRACK_TARGET),
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

    async def _cam_config_pump(self) -> None:
        """capture側(camera_node)への希望（FPS上限・後方ON/OFF）を低頻度で再送する。

        `_fan_pump`/`_auto_ctrl_pump` と同じ理由：camera_node が再起動しても
        1秒以内に最新の意思へ復帰させる。また `_desired_front_fps()` は
        自動運転の engage 状態にも依存するため、engage 側のイベントを
        取りこぼしても最大1秒で追従する保険にもなる。
        """
        period = 1.0 / CAM_CONFIG_HZ
        while self._running:
            await asyncio.sleep(period)
            self._publish_cam_config()

    async def _cam_model_pump(self) -> None:
        """`cam_perception_node` への希望モデル名を低頻度で再送する。

        `_cam_config_pump` と同じ理由：cam_perception_node が再起動しても
        1秒以内に最新の選択へ復帰させる（GUI で何も操作していなくても）。
        """
        period = 1.0 / CAM_MODEL_HZ
        while self._running:
            await asyncio.sleep(period)
            self._publish_cam_model()

    async def _e2e_model_pump(self) -> None:
        """`planning_node`（`e2e_lidar`）への希望モデル名を低頻度で再送する。
        `_cam_model_pump` と同じ理由。"""
        period = 1.0 / E2E_MODEL_HZ
        while self._running:
            await asyncio.sleep(period)
            self._publish_e2e_model()

    async def _track_roi_pump(self) -> None:
        """対象追従のROI選択（`track/roi`）を低頻度で再送する。
        `_cam_model_pump`/`_e2e_model_pump` と同じ理由
        （cam_track_node の再起動や取りこぼしで選択が食い違ったままにならないように）。
        """
        period = 1.0 / TRACK_ROI_HZ
        while self._running:
            await asyncio.sleep(period)
            self._publish_track_roi()

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
        topics = {"front": TOPIC_IMAGE_FRONT, "rear": TOPIC_IMAGE_REAR}
        last_seq = {"front": -1, "rear": -1}
        #: **相対 `sleep(period)` ではなく絶対時刻で刻む。** 前後カメラ2台ぶんの
        #: エンコード＋送信（数ms）が毎周期そのまま次の待ちに積み増しされると、
        #: 目標30fpsのつもりが実測22fps止まりになる（2026-08-24 実機で発覚）。
        #: 遅れが大きすぎるとき（クライアント遅延等）は基準を今に取り直し、
        #: 積み残しを一気に消化しようとして詰まるのを防ぐ
        next_tick = time.monotonic()
        while self._running:
            # `camera_hz` は設定パネルの `gui_hz`（`setCamera`）で実行中に変わりうる
            # ので、周期はループの中で毎回読む
            next_tick += 1.0 / self.camera_hz
            delay = next_tick - time.monotonic()
            if delay > 0:
                await asyncio.sleep(delay)
            else:
                next_tick = time.monotonic()
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
                await asyncio.gather(
                    *(self._send_or_drop(ws, jpg, clients) for ws in list(clients)),
                    return_exceptions=True)

    def _encode_frame(self, ref) -> bytes | None:
        """`ImageRef` → JPEG。共有メモリからゼロコピーで読む（`core.jpeg`）。"""
        got = self._jpeg.encode_latest(ref.shm_name, expect_seq=ref.ring_seq)
        return got[0] if got else None

    async def _mask_pump(self) -> None:
        """`cam/mask`（`cam_perception_node` が JPEG 化済み）をそのまま中継する。

        `_camera_pump` と同じ「購読者がいなければ何もしない」節約はするが、
        **エンコード作業がここには無い**（`ImageRef` の共有メモリ読み出しも
        `to_thread` も不要）ので専用の軽いポンプにしてある。周期は
        `camera_hz` と揃える理由が無い（マスクの更新は cam_perception_node の
        推論周期任せ）ので、`_cam_model_pump` 等と同じ低頻度ポーリングにする。
        """
        last_seq = -1
        period = 1.0 / CAM_MASK_POLL_HZ
        while self._running:
            await asyncio.sleep(period)
            clients = self.camera_clients["mask"]
            if not clients:
                continue
            m = self.sub.latest.get(TOPIC_CAM_MASK)
            if m is None or m.seq == last_seq:
                continue
            last_seq = m.seq
            await asyncio.gather(
                *(self._send_or_drop(ws, m.jpeg, clients) for ws in list(clients)),
                return_exceptions=True)

    # ── 起動 ──

    async def serve_forever(self, stop: asyncio.Future) -> None:
        self._running = True
        async with serve(self._handler, self.host, self.port,
                         process_request=self._process_request,
                         max_queue=8, ping_interval=20) as server:  # noqa: F841
            tasks = [asyncio.create_task(t()) for t in (
                self._bus_pump, self._telemetry_pump, self._map_pump, self._cmd_pump,
                self._camera_pump, self._hb_pump, self._log_ctrl_pump,
                self._auto_ctrl_pump, self._fan_pump, self._wifi_pump,
                self._cam_config_pump, self._cam_model_pump, self._e2e_model_pump,
                self._mask_pump, self._track_roi_pump)]
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
        # バスが既に閉じていれば送れないが、**その場合 io_node 側は
        # `cmd` の途絶を 150ms で検出して自力で DISARM に落ちる**
        with quiet_close("終了時の DISARM / engage 解除 / ファン自動化"):
            self._auto_engaged = False
            self._publish_auto_ctrl()
            self.pub.send(TOPIC_CMD, DriveCmd(mode=0, source="shutdown"))
            self._fan.set_auto()   # プロセスが消えても手動固定を残さない
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


def _load_token() -> str:
    """`$SURGE_TOKEN` → `config/secret.txt` の順に共有トークンを探す。

    **見つからなければ空文字**＝検査しない。トークンを必須にすると、
    「走行日にトークンを忘れて操縦できない」という**止まる方向ではない事故**が
    起きる（そのとき人は `--host 0.0.0.0` ごと安全機構を外しにかかる）。
    """
    env = os.environ.get("SURGE_TOKEN", "").strip()
    if env:
        return env
    try:
        return SECRET_PATH.read_text(encoding="utf-8").strip()
    except OSError:
        return ""


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", type=int, default=DEFAULT_PORT)
    ap.add_argument("--host", default=DEFAULT_HOST,
                    help="待ち受けアドレス。**外から繋ぐには 0.0.0.0 を明示する**"
                         f"（既定 {DEFAULT_HOST} = このPi からのみ）")
    ap.add_argument("--dist", type=Path, default=GUI_DIST, help="GUI のビルド成果物")
    ap.add_argument("--no-camera", action="store_true", help="JPEG 配信をしない")
    ap.add_argument("--jpeg-quality", type=int, default=70)
    ap.add_argument("--camera-hz", type=float, default=CAMERA_HZ)
    ap.add_argument("--token", default="",
                    help="操縦権を取るのに要る共有トークン。省略時は "
                         f"$SURGE_TOKEN → {SECRET_PATH.name} の順に探す")
    args = ap.parse_args()

    token = args.token or _load_token()

    srv = TelemetryServer(port=args.port, host=args.host, dist=args.dist,
                          camera=not args.no_camera,
                          jpeg_quality=args.jpeg_quality, camera_hz=args.camera_hz,
                          token=token)

    print(f"# telemetry_node  http://{args.host}:{args.port}/  "
          f"(mDNS を入れてあれば http://surge.local:{args.port}/)")
    if args.host in ("127.0.0.1", "localhost"):
        print("# ★ loopback のみで待ち受け中。外の PC から繋ぐには --host 0.0.0.0")
    print("# 操縦権トークン: " + ("設定あり（GUI は ?token=… で一度開く）"
                                  if token else "無し（誰でも操縦権を取れる）"))
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
        if srv._jpeg is not None and srv._jpeg.encoded:
            jp = srv._jpeg
            print(f"JPEG: {jp.encoded}枚 {jp.cpu_per_frame_ms:.1f}ms/枚 "
                  f"(CPU 計 {jp.encode_cpu_s:.1f}s) [{jp.impl}]"
                  f"  ※ logger_node も同じフレームを焼いている（レビュー 🟡7）")
        if srv.bad_cmds or srv.auth_rejects or srv.origin_rejects:
            print(f"捨てた入力: 壊れた cmd={srv.bad_cmds} "
                  f"トークン不一致={srv.auth_rejects} Origin 不一致={srv.origin_rejects}")
        # **握り潰した後始末は必ず表に出す。** 0 なのが正常で、
        # 増えているなら閉じられていない資源がある（`raspi/core/cleanup.py`）
        if failure_count():
            print(f"後始末で捨てた例外 {failure_count()}件（直近）:")
            for _, what, why in recent_failures():
                print(f"  - {what}: {why}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
