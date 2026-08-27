"""内部バスに流れるメッセージ型（`docs/architecture.md` §6.3）。

`msgspec.Struct` で定義する。Rust 実装で速く、そのまま msgpack に落ちる。

## 全メッセージが `t_capture` / `t_pub` / `seq` を持つ

後からセンサ融合を作るときに全メッセージ定義を書き直さずに済ませるため。
**`t_capture` は「センサが取得した時刻」**であって publish した時刻ではない。
この2つを混ぜると、UART の往復や Python の処理遅延がそのまま位置推定の誤差になる。

- `t_capture` … STM32 が測った時刻を時刻同期で Pi の `CLOCK_MONOTONIC` に換算した値 [ns]
- `t_pub` … `Publisher.send()` が押す [ns]
- `seq` … トピックごとの通し番号。`Publisher` が押す

## 単位は SI で固定する

UART 上は `0.001 m/s` のような整数スケールだが、**バスに出た時点で SI の float にする**
（`docs/architecture.md` §5.1）。km/h や度への変換は GUI の表示直前だけ。
どの層でスケールが掛かっているか分からなくなるのが一番危ない。

## 「値が無い」は 0 ではなく None で表す

`us_front` は `0 = 無効`、`temp` は MD の `comm_ok=0` なら**信じてはいけない**。
これらを 0.0 のまま下流に流すと「0m に障害物がある」「0℃ である」と読める。
型で潰しておく。
"""

from __future__ import annotations

import msgspec

__all__ = [
    "MsgBase", "VehicleState", "Scan", "LineScan", "DriveCmd", "LinkDiag", "ImageRef",
    "Heartbeat", "AutoCtrl", "AutoState", "AutoMap", "UiEvent",
    "TOPIC_VEHICLE_STATE", "TOPIC_SCAN", "TOPIC_SCAN_CAM", "TOPIC_LINE_CAM", "TOPIC_CMD",
    "TOPIC_DIAG_LINK",
    "TOPIC_IMAGE_FRONT", "TOPIC_IMAGE_REAR", "TOPIC_HB_PREFIX",
    "TOPIC_AUTO_CTRL", "TOPIC_AUTO_CMD", "TOPIC_AUTO_STATE", "TOPIC_AUTO_MAP",
    "TOPIC_UI_EVENT", "CamModelCtrl", "TOPIC_CAM_MODEL",
    "TOPIC_TYPES", "type_for_topic",
]

# ── トピック名 ──────────────────────────────────────────────────────────
#: `/` で階層にする。購読は前方一致なので `image/` で両カメラを取れる
TOPIC_VEHICLE_STATE = "vehicle_state"
TOPIC_SCAN = "scan"
#: カメラ由来の擬似 `Scan`（`cam_perception_node.py` が publish）。
#: **`scan`（実 LiDAR）とは別トピックにしてある。** 同じトピックに2つの
#: publisher が出すと、どちらが勝つか購読側から区別できなくなる
#: （`planning_node.py` が `cmd`/`auto/cmd` を分けた理由と同じ）。
#: 型は `Scan` をそのまま使う（専用の CamScan 型は作らない）
TOPIC_SCAN_CAM = "scan/cam"
#: カメラで検出した白線の目標点（`line_perception_node.py` が publish）。
#: 型は `Scan`（距離配列）とは形が違うので専用の `LineScan` を使う
TOPIC_LINE_CAM = "line/cam"
TOPIC_CMD = "cmd"
TOPIC_DIAG_LINK = "diag/link"
TOPIC_IMAGE_FRONT = "image/front"
TOPIC_IMAGE_REAR = "image/rear"
TOPIC_HB_PREFIX = "hb/"
TOPIC_LOG_CTRL = "log/ctrl"

#: どの自動運転モードで走るかの意思。telemetry_node が流し planning_node が従う
TOPIC_AUTO_CTRL = "auto/ctrl"
#: planning_node が出す走行指令。**`cmd` とは別トピック**（下の AutoCtrl 参照）
TOPIC_AUTO_CMD = "auto/cmd"
#: planning_node が「何を見てそう決めたか」。GUI のデバッグ表示用
TOPIC_AUTO_STATE = "auto/state"
#: 地図と経路。**変わったときだけ**流れる（`AutoMap` の docstring）
TOPIC_AUTO_MAP = "auto/map"

#: GUI 側の単発イベント（例: `/ws/control` への新規接続）。telemetry_node が
#: 出し、io_node が拾ってブザーのメロディ（起動音とは別の音形）を鳴らす
TOPIC_UI_EVENT = "ui/event"

#: capture側(camera_node)のFPS上限・後方カメラON/OFFの意思。telemetry_node が出し、
#: camera_node が拾って `Picamera2.set_controls()`/`stop()`/`start()` を呼ぶ
TOPIC_CAM_CONFIG = "cam/config"

#: `cam_perception_node` が使うセグメンテーションモデルの希望。telemetry_node が
#: `cam/config` と同じ流儀（現在の意思を繰り返し流す）で出し、cam_perception_node
#: が拾って ONNX 推論セッションを作り直す。**`scan/cam`（推論の出力）とは別物**
TOPIC_CAM_MODEL = "cam/model"


class MsgBase(msgspec.Struct):
    """全メッセージ共通のヘッダ。

    **全フィールドに既定値を持たせてある。** msgspec は dataclass と同じ引数順の
    規則を持つので、基底に既定値なしのフィールドがあると派生側で詰む。
    """

    t_capture: int = 0
    t_pub: int = 0
    seq: int = 0


class VehicleState(MsgBase):
    """`TELEMETRY`(0x02) を SI に直したもの。50Hz。

    生値と Pi 側の派生量を**同じメッセージに入れる**。別トピックに分けると
    「どの派生量がどの生値から出たか」の対応が時刻合わせの問題に化ける。
    """

    # ── STM32 が測った生の値（SI 換算のみ） ──
    #: 車体中心線方向に射影済み [m/s]（射影済みなのはこれだけ）
    speed: float = 0.0
    yaw_rate: float = 0.0                  #: [rad/s] IMU。**yaw の絶対値は出ない**
    steer_actual: float = 0.0              #: 路面舵角 [rad] 反時計回り正
    steer_cmd_echo: float = 0.0            #: 指令のエコーバック [rad]
    #: 車輪周速 [m/s] [FL, FR, RL, RR]。**前2輪は操舵輪なので射影されていない**
    wheel_speed: list[float] = msgspec.field(default_factory=lambda: [0.0] * 4)
    #: 前輪の累積走行距離 [m] [FL, FR]。**累積値・射影なし**
    odom_dist: list[float] = msgspec.field(default_factory=lambda: [0.0] * 2)
    accel: list[float] = msgspec.field(default_factory=lambda: [0.0] * 3)  #: [m/s²] x,y,z
    pitch: float = 0.0                     #: [rad] 重力で補正済み。ドリフトしない
    roll: float = 0.0                      #: [rad] 同上
    motor_current: list[float] = msgspec.field(default_factory=lambda: [0.0] * 3)  #: [A] RL,RR,ST
    #: [N·m] [RL, RR]。**指令値であって実測ではない**（Kt が未実測のため測れない）
    torque_cmd: list[float] = msgspec.field(default_factory=lambda: [0.0] * 2)
    #: STM32 の TC が内部で見ているスリップ率 [RL, RR]（無次元。正=空転 負=ロック傾向）。
    #: 基準速度が低い区間は STM32 側で 0 を送ってくる（★v0.13）。`slip_rear` とは別物——
    #: こちらは STM32 自身の推定値、`slip_rear` は Pi 側が `wheel_speed`/`speed` から計算する値
    tc_slip: list[float] = msgspec.field(default_factory=lambda: [0.0] * 2)
    #: TC が動的に決めているトルク上限 [N·m] [RL, RR]。介入していなければ
    #: `LinkDiag.max_torque_nm` と同じ値になる（★v0.13）
    tc_limit_nm: list[float] = msgspec.field(default_factory=lambda: [0.0] * 2)
    #: [℃] [RL, RR, ST, MCU]。MD の `comm_ok=0` なら該当要素は None
    temp: list[int | None] = msgspec.field(default_factory=lambda: [None] * 4)
    batt_voltage: list[float] = msgspec.field(default_factory=lambda: [0.0] * 2)  #: [V] 駆動,信号
    #: [A] 駆動,信号。**単方向センサなので回生中は 0 に張り付く。信用しないこと**
    batt_current: list[float] = msgspec.field(default_factory=lambda: [0.0] * 2)
    us_front: float | None = None          #: [m] 無効なら None。実質 20Hz 更新
    us_rear: float | None = None           #: [m] 同上
    md_status: list[int] = msgspec.field(default_factory=lambda: [0] * 3)  #: MDS_* ビット
    flags: int = 0                         #: FLG_* ビット（生のまま。下流で解釈する）
    cmd_seq_echo: int = 0                  #: 最後に受理した COMMAND の SEQ
    t_stm_us: int = 0                      #: STM32 の生の時刻。ログ突き合わせ用

    # ── flags を解いたもの（GUI と安全判定が毎回ビット演算するのを避ける） ──
    mode: int = 0                          #: 0=DISARM 1=MANUAL 2=AUTO
    armed: bool = False
    estop_active: bool = False
    uart_timeout: bool = False
    tc_active: bool = False                #: 今まさに TC が介入中
    tv_active: bool = False                #: 今まさに TV が介入中
    imu_ok: bool = False
    lidar_ok: bool = False
    steer_center_valid: bool = False       #: 0 なら走行禁止
    #: 過電流ラッチで `arm` を拒否中。**これは仕様であって FAULT ではない**
    drive_power_locked: bool = False
    #: 超音波の自動停止が**今まさに制動へ介入中**（v0.7）。`tc_active` と同じ「介入中」の意味で、
    #: 有効/無効そのものではない（有効かどうかは Pi が送った `auto_stop` を見れば分かる）。
    #: **急に減速した理由がこれかどうかを後から説明できるようにするため**に持つ
    auto_stop_active: bool = False
    #: サイドブレーキ（`COMMAND.flags2` bit0）が**今まさに位置制御へ切り替わり固定中**
    #: （★v0.13）。`auto_stop_active` と同じ「介入中」の意味で、有効かどうかそのものではない
    side_brake_active: bool = False
    faults: list[str] = msgspec.field(default_factory=list)  #: 立っている fault の名前

    # ── Pi 側で計算した派生量 ──
    #: 車体中心線方向の累積走行距離 [m]。**1周期ぶんの差分に舵角で射影して積算**
    #: したもの。累積値に cos を掛けるのは誤り（`uart_protocol.md` §5.3）
    odom_center: float = 0.0
    #: 前輪スリップ [m/s] [FL, FR]。`wheel_speed * cos(δ) - speed`
    slip_front: list[float] = msgspec.field(default_factory=lambda: [0.0] * 2)
    #: 後輪スリップ [m/s] [RL, RR]。非操舵輪なので射影不要
    slip_rear: list[float] = msgspec.field(default_factory=lambda: [0.0] * 2)
    #: `|speed|` がデッドバンド未満。前輪エンコーダは静止付近で符号がばたつくので、
    #: **生値はそのまま残し、「止まっている」の判断だけを別に持つ**。
    #: GUI はこれが True なら 0.00 と表示する
    stopped: bool = True


class Scan(MsgBase):
    """LiDAR 1周ぶん。10Hz。

    **角度は配列の添字がそのまま度**（`dist[i]` は i 度の距離）。STM32 側で
    1° ビンに整形済みなので、実測角度は送られてこない。

    **添字は車両座標の角度**（x=前 が 0°、反時計回りが正）。LD06 は裏向きに
    取り付けてあり生の角度は左右が鏡像なので、`ScanAssembler` が反転済みにして渡す。

    `t_capture` はこの1周で最初に取得したセクタの時刻。**1周に 100ms かかるので、
    点ごとの時刻は `sector_t_ns` から引くこと。** 時速 10km/h なら1周で 28cm 動き、
    点群が扇形に歪む。
    """

    dist: list[float] = msgspec.field(default_factory=lambda: [0.0] * 360)  #: [m] 0=無効
    #: 各セクタ先頭の時刻 [ns]（Pi 時刻に換算済み）。欠けたセクタは 0。
    #: **反転の結果、セクタ `s` が持つのは `dist[30*s+1]` 〜 `dist[(30*s+30) % 360]` の30点**
    #: で、`30*s` ちょうどの点は**隣のセクタ `s-1` のパケット由来**。境界がずれている
    sector_t_ns: list[int] = msgspec.field(default_factory=lambda: [0] * 12)
    #: 各セクタ30点の取得所要時間 [μs]。**車両角 `deg` の点の時刻はこう引く**：
    #:
    #:     sensor = (360 - deg) % 360                 # センサ角に戻す
    #:     s, i = 11 - sensor // 30, sensor % 30      # セクタ番号とパケット内の添字
    #:     t_ns = sector_t_ns[s] + sector_dur_us[s] * 1000 * i / 29
    #:
    #: `i` はパケット内の添字なので、**車両角では添字が減る向きに時刻が進む**。
    #: `deg` から直に `30*s+j` と分解して式を立てると、`j=0` の点で最大1周ぶん外す
    sector_dur_us: list[int] = msgspec.field(default_factory=lambda: [0] * 12)
    #: この1周で受信できたセクタ。**False の区間は「障害物なし」ではない**。
    #: `False` のとき欠けている車両角は上と同じ `30*s+1` 〜 `(30*s+30) % 360`
    sector_seen: list[bool] = msgspec.field(default_factory=lambda: [False] * 12)
    rot_speed_dps: float = 0.0             #: LD06 の回転速度 [deg/s]。正常は約 3600
    intensity: list[int] | None = None     #: 強度付きフォーマットのときだけ入る
    #: 圧縮フォーマットで 255（**5.10m ちょうどではなく「5.10m 以上」**）だった点。
    #: ここを実測点として地図に打つと実在しない壁が円状に生成される
    saturated: list[bool] | None = None
    lidar_format: int = 0                  #: 0=通常 1=強度付き 2=圧縮


class LineScan(MsgBase):
    """前方カメラで検出した白線1本ぶん。`line_perception_node.py` が publish する。

    白線の画素を画面下寄り（近傍）・中央寄り（遠方）の2つの帯で検出し、
    それぞれの重心を `raspi.nav.ipm.pixel_to_ground()` で地面座標（base_link
    基準）へ逆投影した2点として持つ。**Pure Pursuit の目標点そのものの形**
    （原点から見た方位と距離だけで舵角が決まる）にしてあるので、
    `raspi/auto/line_trace.py` は `follow_the_gap.py` と同じ
    `nav.purepursuit.steer_for_target()` をそのまま使える。

    `Scan`（距離配列）とは形が違う情報のため専用の型にしてある
    （`follow_the_gap_cam.py` が `Scan` を再利用できたのとは対照的）。
    """

    seen: bool = False              #: 近傍・遠方いずれかの帯で白線を検出できたか
    near_seen: bool = False
    far_seen: bool = False
    near_x: float = 0.0             #: [m] base_link 座標。近傍帯の白線重心
    near_y: float = 0.0
    far_x: float = 0.0              #: [m] 遠方帯の白線重心
    far_y: float = 0.0
    #: ROI（近傍・遠方の帯）内で白と判定された画素の割合の最大値 [0..1]。
    #: 低いのに走っているなら疑う（`follow_the_gap.py` の `valid_ratio` と同じ扱い）
    coverage: float = 0.0


class DriveCmd(MsgBase):
    """走行指令。GUI（MANUAL）または planning_node（AUTO）が publish する。

    **io_node はこれを 150ms 受け取らなければ DISARM に落とす。**
    publish 側が止まった＝上位が死んだ、なので車を止めるのが正しい。
    """

    mode: int = 0                          #: 0=DISARM 1=MANUAL 2=AUTO
    arm: bool = False
    #: 制動。**v0.5 では速度 PI を迂回して `brake_torque` を直接掛ける。**
    #: 立てている間 `target_speed` は無視される
    brake: bool = False
    horn: bool = False                     #: **立てている間ずっと鳴る**（v0.4 は単発）
    light_mode: int = 0                    #: 0=OFF 1=DAYTIME 2=NORMAL（`packets.LightMode`）
    #: パッシング。立てている間 `light_mode` に関わらず前照灯だけが全光量になる
    passing: bool = False
    target_speed: float = 0.0              #: [m/s] 負で後退
    target_steer: float = 0.0              #: 路面舵角 [rad] 反時計回り正
    accel_limit: float = 0.0               #: [m/s²] 0 は STM32 の既定に任せる意
    steer_rate_limit: float = 0.0          #: [rad/s]
    #: 後輪**各輪**の制動トルク [N·m]。`brake` が立っているときだけ意味を持つ。
    #: **0 は「制動しない」ではなく「未指定」で、STM32 が最大値で制動する**
    #: （`accel_limit` と同じ約束。埋め忘れでブレーキが効かない方が危険なため）
    brake_torque: float = 0.0
    #: 立っている間 `target_speed` を無視し `target_torque` を直接掛ける（v0.6）。
    #: `brake` と同様に速度 PI を迂回する。`brake` が同時に立っていれば `brake` が優先
    torque_mode: bool = False
    #: [N·m] 駆動トルク直接指令。`torque_mode` のときだけ意味を持つ。負は後退方向。
    #: 範囲 ±`convert.MAX_TARGET_TORQUE_NM`（v0.6）
    target_torque: float = 0.0
    #: 立っている間、**STM32 が単独で**進行方向の超音波を見て 20cm 未満なら最大制動する（v0.7）。
    #: 進行方向は `torque_mode` なら `target_torque`、そうでなければ `target_speed` の符号で決まり、
    #: **逆方向のセンサは見ない**（前に障害物があっても後退はできる）。
    #: 優先順位は `brake` > `auto_stop` > 通常指令。**Pi 側で二重に制御する必要はない**
    auto_stop: bool = False
    #: サイドブレーキ。立てている間、**速度に関わらず即座に**後輪を機械的な位置制御へ
    #: 切り替えて固定する（★v0.13）。`brake` より優先。解除は明示的に下ろすまで有効
    side_brake: bool = False
    source: str = ""                       #: 誰が出したか（"gui" / "planning" / "safety"）


class LinkDiag(MsgBase):
    """リンクの健全性。1〜10Hz。**上りと下りのどちらが悪いかを切り分けるため**の情報。

    Pi 側のカウンタ（`rx`）だけでは STM32 → Pi の品質しか分からないので、
    STM32 が数えた `stm_rx`（累積）と並べて出す。
    """

    health: str = "INIT"                   #: INIT / OK / DEGRADED / FAULT
    #: E-Stop 発動中。**車両のボタン2を人間が押すまで解除されない**
    estop_active: bool = False
    #: 過電流で駆動電源がラッチ遮断。**電源を入れ直すまで復帰しない**
    drive_power_locked: bool = False
    #: io_node が `arm` を封印しているか（`--allow-arm` が無い状態）
    arm_inhibited: bool = True
    #: 直近に受理した `cmd` の発行元。誰も出していなければ ""
    cmd_source: str = ""
    #: `cmd` が途絶して DISARM にフォールバックしているか
    cmd_stale: bool = True

    #: TC/TV が STM32 側で実際に有効化されているか（`CONFIG_ACK` から取得。★v0.8）。
    #: 未確認（起動直後でまだ `CONFIG_ACK` を受け取っていない）なら None
    tc_enabled: bool | None = None
    tv_enabled: bool | None = None

    #: 片輪浮き対策（Wheel Lift Guard）が STM32 側で実際に有効化されているか
    #: （`CONFIG_ACK` から取得。★v0.9）。TC/TV本体とは独立した別機構。未確認なら None
    wheel_lift_guard_enabled: bool | None = None

    #: 自動停止（`COMMAND.flags` bit7=AUTO_STOP）の安全マージン [cm]（`CONFIG_ACK`
    #: から取得。★v0.12。範囲0.0-100.0の連続値。未確認（起動直後でまだ `CONFIG_ACK`
    #: を受け取っていない）なら None（None の間もSTM32側は既定15cmで動いている）
    auto_stop_margin_cm: float | None = None

    rx: dict[str, int] = msgspec.field(default_factory=dict)      #: Pi 側 RxStats
    stm_rx: dict[str, int] | None = None   #: STM32 の STATS（**累積値**）

    #: STM32 ⇄ MD 間の受信数とエラー数 [RL, RR, ST]。**合計にしないこと。**
    #: 合計にすると「1台だけ起動が遅れている」「1台だけ配線が悪い」が
    #: 他の2台に埋もれて見えなくなる（2026-08-08 に実際に誤診した）
    md_rx_count: list[int] | None = None
    md_rx_error: list[int] | None = None
    counts: dict[str, int] = msgspec.field(default_factory=dict)  #: TYPE 名 → 受信数

    sync_offset_ns: int | None = None
    sync_delay_ns: int | None = None       #: min filter 後の片道遅延
    sync_drift_ppm: float | None = None
    sync_n: int = 0

    #: `COMMAND` を送ってから `cmd_seq_echo` で返るまで [ms]。**時刻同期に依存しない**
    #: 純粋な往復測定。50ms を超えたら警告（`uart_protocol.md` §6.1）
    cmd_rtt_ms: float | None = None

    protocol_version: int | None = None
    fw_id: int | None = None
    protocol_match: bool | None = None

    hb_alive: bool | None = None           #: GPIO6 ハートビートを出せているか
    hb_max_late_ms: float | None = None
    hb_stalls: int = 0

    lidar_scans: int = 0
    lidar_sectors_lost: int = 0

    #: **このリンクの相手が実 STM32 ではなくシミュレータか。**
    #: 利便性ではなく安全のために出す。シムと実機の画面が見分けられないと
    #: 「シムのつもりで --allow-arm した実車が動く」が起きる（GUI が SIM バッジを出す）
    sim: bool = False

    #: 車両の物理的な上限値（`LIMITS` パケットから取得。★v0.11）。
    #: 読み取り専用・実行時に変化しない。まだ受け取っていなければ None
    max_speed_m_s: float | None = None
    max_accel_m_s2: float | None = None
    max_torque_nm: float | None = None
    max_steer_rad: float | None = None


class ImageRef(MsgBase):
    """共有メモリ上の画像1枚への参照。**画素はバスに流さない**（§6.2）。

    640×480×3 を 30Hz で msgpack に通すと 28MB/s のコピーが起きる。
    下流は `FrameRing.attach(shm_name)` してゼロコピーで読む。
    """

    cam: str = ""                          #: "front" / "rear"
    shm_name: str = ""                     #: 共有メモリ名（例 "surge_cam0"）
    slot: int = 0
    ring_seq: int = 0                      #: リングの write_seq。seqlock の検証に使う
    frame_id: int = 0                      #: picamera2 の FrameCount
    width: int = 0
    height: int = 0
    fmt: str = ""                          #: **メモリ上の実際のバイト順**（BGR888 等）
    stride: int = 0
    nbytes: int = 0


class Heartbeat(MsgBase):
    """ノードの生存申告。10Hz。`safety_node` が 300ms 途絶で FAULT にする。"""

    node: str = ""
    pid: int = 0
    detail: str = ""


class LogCtrl(MsgBase):
    """`.sfl` 記録の意思表示。**telemetry_node が継続的に送り続け、io_node が従う。**

    1回だけ送ると io_node の再起動やメッセージ取りこぼしで食い違ったままになるので、
    `cmd`（`TOPIC_CMD`）と同じく「現在の意思」を繰り返し流す設計にしてある。
    """

    active: bool = False


class AutoCtrl(MsgBase):
    """どの自動運転モードで走るかの意思。**telemetry_node が送り続け、planning_node が従う。**

    `log/ctrl` と同じく「現在の意思」を繰り返し流す（1回だけ送ると、どちらかの
    再起動や取りこぼしで食い違ったままになる）。

    ## `engaged` は「モータを回してよい」ではない

    自律走行が実際に車を動かすには、これとは独立に**人間が GUI で ARM を保持**して
    いる必要がある。`planning_node` は `engaged=False` でも計画そのものは回し続け、
    `auto/state` に「今なら何をするか」を出す。**走らせる前に見られる**ようにするため。
    """

    #: 使う planner の id（`raspi/auto/registry.py`）。**空文字は「自動運転しない」**
    mode: str = ""
    #: 人間が自律走行を engage したか。**`False` なら `auto/cmd` は車に届かない**
    engaged: bool = False
    #: planner のパラメータ。**未指定のキーは planner 側の既定値**
    params: dict[str, float] = msgspec.field(default_factory=dict)
    #: 「地図を確定」を押した回数。**真偽値ではなく回数**にしてある。
    #: このメッセージは現在の意思を繰り返し流す（CONFLATE）ので、`True` を立てると
    #: 次に押していないのに何度も確定してしまう。**値が増えたときだけ**効かせる
    freeze_seq: int = 0
    #: 「地図を削除」を押した回数。理由は `freeze_seq` と同じ
    clear_seq: int = 0


class AutoState(MsgBase):
    """planning_node の判断そのもの。**指令ではなく「なぜそう決めたか」を運ぶ。**

    `auto/cmd`（実際に出す指令）と分けてあるのは、**指令を見ても原因が分からない**
    ため。「なぜ急に止まったか」「なぜ左を選んだか」は、選んだギャップと前方余裕を
    並べて初めて読める。GUI の LiDAR ビューはここを重畳して描く。

    **角度はすべて車両座標**（x=前 が 0、反時計回りが正）。`*_deg` は
    **符号付き ±180 度**で持つ（`Scan.dist` の添字は 0〜359 だが、前方を跨ぐ
    ギャップが `350〜10` のように分断されて読めなくなるため）。
    """

    mode: str = ""                         #: planner の id。空なら自動運転していない
    planner: str = ""                      #: 表示名
    engaged: bool = False                  #: `AutoCtrl.engaged` のエコーバック
    #: 走らせてよい状態か。**False の理由は必ず `reason` に入る**
    ready: bool = False
    #: ready でない理由、または減速・停止の理由。**常に日本語で入れる**（GUI にそのまま出す）
    reason: str = ""

    # ── planner が出した指令（`auto/cmd` と同じ値。表示用に重複させてある） ──
    target_speed: float = 0.0              #: [m/s]
    target_steer: float = 0.0              #: 路面舵角 [rad] 反時計回り正
    brake: bool = False                    #: 制動を掛けているか

    # ── 判断の根拠（Follow the Gap の場合） ──
    heading: float = 0.0                   #: 狙っている方位 [rad]
    gap_start_deg: float = 0.0             #: 選んだギャップの右端 [deg] 符号付き
    gap_end_deg: float = 0.0               #: 同 左端 [deg]。**`start <= end`**
    #: 安全バブル（最近傍点の周りに置く侵入禁止の扇）。`bubble_start > bubble_end` は
    #: バブルが無いことを表す（最近傍が遠すぎて置く必要が無かった場合）
    bubble_start_deg: float = 0.0
    bubble_end_deg: float = -1.0
    free_ahead: float = 0.0                #: 正面の余裕 [m]。**速度を決めているのはこれ**
    nearest: float = 0.0                   #: 最近傍の距離 [m]
    nearest_deg: float = 0.0               #: その方位 [deg] 符号付き
    #: 視野内で有効だった点の割合 [0..1]。**低いのに走っているなら疑う**
    valid_ratio: float = 0.0

    # ── 地図を使う planner の状態（`raspi/auto/raceline.py`） ──
    #: 走行段。"EXPLORE"（地図を作っている）/"BUILD"（経路を作っている）/"RACE"
    phase: str = ""
    pose_x: float = 0.0                    #: [m] map フレーム。**base_link 基準**
    pose_y: float = 0.0
    pose_yaw: float = 0.0                  #: [rad] 反時計回り正
    #: スキャンマッチの一致度 0〜1。**低いのに走っているなら疑う**
    match_score: float = 0.0
    #: 周回の進み具合 [周]。累積回頭 ÷ 360°。位置ではなく回頭で数えている
    lap_progress: float = 0.0
    laps: int = 0                          #: 走り切った周回数
    #: レーシングラインからの横偏差 [m]。**RACE 段の主要な指標**
    cross_track: float = 0.0
    #: Pure Pursuit が狙っている経路上の点（map フレーム）。**舵角の理由がこれ**
    target_x: float = 0.0
    target_y: float = 0.0
    #: 検出した動的障害物 `[x, y, r, ...]`（map フレーム）。上限あり
    obstacles: list[float] = msgspec.field(default_factory=list)

    # ── 鮮度（planning_node が入れる） ──
    scan_age_ms: float = 0.0               #: 使った点群の古さ [ms]
    plan_hz: float = 0.0                   #: 実測の計画レート [Hz]


class AutoMap(MsgBase):
    """地図と経路。**`auto/state` と分けてあるのは寿命と大きさが違うから。**

    `AutoState` は 10Hz で流れる「今の判断」だが、地図は 400×400 = 160KB あり、
    しかも**確定したら二度と変わらない**。同じ頻度で流す理由が無い。

    planning_node は `Planner.snapshot()` の `map_seq` が変わったときだけ publish し、
    telemetry_node は `/ws/map` に流す。**地図を凍結したあとは 1 回も流れない**
    （新しく繋いだ GUI にだけ最後の1枚を送る）。

    `cells` は 3 値（`0=未知 1=空き 2=占有`）を 2bit に詰めて `zlib` で圧縮したもの。
    生の 160KB が数KB になる。展開はブラウザ標準の `DecompressionStream('deflate')`
    でできるので、GUI 側にライブラリを増やさずに済む。
    """

    map_seq: int = 0                       #: 地図の版。**変わったときだけ送る**
    resolution: float = 0.05               #: [m/セル]
    origin_x: float = 0.0                  #: セル(0,0)の角の世界座標 [m]
    origin_y: float = 0.0
    width: int = 0                         #: 列数（+x 方向）
    height: int = 0                        #: 行数（+y 方向）。**行0 が y 最小**
    cells: bytes = b""                     #: 上記の圧縮済み 2bit 配列
    #: 中心線 `[x0, y0, x1, y1, ...]`（map フレーム）
    centerline: list[float] = msgspec.field(default_factory=list)
    #: レーシングライン `[x0, y0, ...]`。**アウトインアウトはこれを見れば分かる**
    raceline: list[float] = msgspec.field(default_factory=list)
    #: 各点の目標速度 [m/s]。`raceline` の半分の要素数（点ごとに1つ）
    raceline_v: list[float] = msgspec.field(default_factory=list)


class UiEvent(MsgBase):
    """GUI 側の単発イベント。**「現在の意思」ではなく「今起きたこと」**なので、
    `AutoCtrl` のように繰り返し流さず1回だけ publish する。

    受け手（io_node）は継承元 `MsgBase.seq`（`Publisher` が自動で振る通し番号）
    が前回より増えていたら「新しいイベント」と判断する。専用のカウンタを
    別途持たなくて済む。
    """

    kind: str = ""                         #: 例: "gui_connect", "tc_enable", "tv_enable", "wheel_lift_guard_enable", "auto_stop_margin_cm"
    #: `kind` が真偽値を伴うイベント（"tc_enable"/"tv_enable"/"wheel_lift_guard_enable" 等）のときだけ意味を持つ
    value: bool = False
    #: `kind` が連続値を伴うイベント（"auto_stop_margin_cm" のみ。★v0.12）のときだけ意味を持つ。
    #: `value`（bool）とは別枠にした——既存の bool イベントと混ぜると 0/1 に丸まってしまうため
    float_value: float = 0.0


class CamConfig(MsgBase):
    """camera_node への capture 側の希望設定。`AutoCtrl`/ファンと同じく
    **「現在の意思」を繰り返し流す**（telemetry_node の `_cam_config_pump`）。

    一度きりの `UiEvent` にしなかったのは、camera_node が再起動したときに
    1秒以内で最新の意思に復帰させたいため（`_fan_pump`/`_auto_ctrl_pump` と同じ理由）。

    後方カメラは自動運転が使わない（GUI表示とロギング専用）ので `rear_fps` に
    自動運転による上書きは無い。前カメラだけ、カメラを使う自動運転モードで
    engage 中は telemetry_node 側が `front_fps` を上限まで引き上げる。
    """

    front_fps: float = 30.0
    rear_fps: float = 30.0
    #: False なら camera_node が該当カメラの `Picamera2.stop()` を呼び、実際に撮像を止める
    rear_enabled: bool = True


class CamModelCtrl(MsgBase):
    """`cam_perception_node` への「このモデルを使ってほしい」という意思。

    `CamConfig` と同じく**「現在の意思」を繰り返し流す**（telemetry_node の
    `_cam_model_pump`）——cam_perception_node が再起動しても、GUI で何も
    操作しないまま数秒で最新の意思に復帰させたい。

    `name` は `models/<name>.onnx`（+ 同名の `.json`。前処理設定。
    `ml/export_onnx.py` が書く）を指す。**空文字は「未選択」**——
    cam_perception_node はモデルを持たないまま `scan/cam` を「壁」扱いで
    出し続ける（`follow_the_gap_cam.py` 側の `MIN_SEEN_RATIO` で自然に停止する）。
    """

    name: str = ""


#: トピック → 型。`Subscriber` がデコードに使う。
#: `image/` は前方一致で両カメラに効かせたいので接頭辞でも引けるようにしてある
TOPIC_TYPES: dict[str, type[MsgBase]] = {
    TOPIC_VEHICLE_STATE: VehicleState,
    TOPIC_SCAN: Scan,
    TOPIC_SCAN_CAM: Scan,
    TOPIC_LINE_CAM: LineScan,
    TOPIC_CMD: DriveCmd,
    TOPIC_DIAG_LINK: LinkDiag,
    TOPIC_IMAGE_FRONT: ImageRef,
    TOPIC_IMAGE_REAR: ImageRef,
    TOPIC_LOG_CTRL: LogCtrl,
    TOPIC_AUTO_CTRL: AutoCtrl,
    #: **`cmd` と同じ型。** io_node は `auto/cmd` を購読しない（`AutoCtrl` 参照）
    TOPIC_AUTO_CMD: DriveCmd,
    TOPIC_AUTO_STATE: AutoState,
    TOPIC_AUTO_MAP: AutoMap,
    TOPIC_UI_EVENT: UiEvent,
    TOPIC_CAM_CONFIG: CamConfig,
    TOPIC_CAM_MODEL: CamModelCtrl,
}


def type_for_topic(topic: str) -> type[MsgBase]:
    """トピック名から型を引く。前方一致も見る（`image/` → `ImageRef`）。

    :raises KeyError: 未登録のトピック。**黙って dict を返さない。**
        型が分からないまま下流に流すと、フィールド名の typo が実行時まで残る
    """
    t = TOPIC_TYPES.get(topic)
    if t is not None:
        return t
    if topic.startswith(TOPIC_HB_PREFIX):
        return Heartbeat
    if topic.startswith("image/"):
        return ImageRef
    raise KeyError(f"未登録のトピック: {topic!r}")
