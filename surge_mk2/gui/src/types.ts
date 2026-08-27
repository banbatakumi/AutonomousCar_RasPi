/**
 * GUI 側の型。
 *
 * **バスのメッセージ型（`VehicleState`・`Scan`・`LinkDiag`・`AutoState`・
 * `AutoMapMsg`）は `generated/msgs.ts` から再エクスポートしている。**
 * `raspi/msgs/types.py` が唯一の正で、`python3 config/gen_msgs.py` が吐く。
 * 以前はここに手で写していたが、45 フィールドの写経は片方だけ直しても
 * どのツールもエラーを出さず、**画面に静かに `undefined` が出る**形だった
 * （2026-08-21 のレビュー 🟠2）。
 *
 * ここに手書きで残してあるのは、**Python 側に対応物が無い型だけ**——
 * WS の制御 JSON（`ControlStatus`・`CmdOut`）、複数メッセージを束ねた
 * `Snapshot`、描画用に焼き直した `MapData` など。
 *
 * **単位はすべて SI**（m/s, rad, m, A, V, ℃）。km/h や度への変換は
 * 表示の直前（`format.ts`）でだけ行う。ここで度に直すと、以降どの値に
 * 換算が掛かっているのか分からなくなる。
 */

export type {
  AutoMapMsg,
  AutoState,
  LinkDiag,
  Scan,
  VehicleState,
} from './generated/msgs'

// 下の `Snapshot` が参照するので、再エクスポートとは別に取り込む
// （`export ... from` は名前をこのモジュールのスコープには入れない）
import type { AutoState, LinkDiag, Scan, VehicleState } from './generated/msgs'

/** 展開して描画用に焼いたあとの地図。**`live.map` に置く。** */
export type MapData = {
  seq: number
  res: number
  originX: number
  originY: number
  width: number
  height: number
  /** 着色済みの画像。**描画側は貼るだけ**（展開は受信時に1回だけ） */
  bitmap: ImageBitmap | null
  centerline: Float64Array
  raceline: Float64Array
  racelineV: Float64Array
  /** 観測できた範囲 [m]。**画面はここに合わせる**（地図の枠ではなく） */
  known: { x0: number; y0: number; x1: number; y1: number } | null
}

/** 調整できるパラメータ1つ。**スライダはこれ1件から自動で作られる。** */
export type AutoParamSpec = {
  key: string
  label: string
  min: number
  max: number
  step: number
  default: number
  unit: string
  note: string
}

/** 選べる自動運転モード1つ（`raspi/auto/registry.py` の `catalog()`）。 */
export type AutoPlannerInfo = {
  id: string
  name: string
  description: string
  params: AutoParamSpec[]
}

/**
 * 自動運転の意思と、選べるモードの宣言。
 *
 * **`catalog` はサーバから降ってくる。** GUI にモードの一覧を書かないので、
 * `raspi/auto/` に planner を足せばこの画面に勝手に出る。
 */
export type AutoStatus = {
  /** 選ばれている planner の id。**空文字 = 自動運転しない** */
  mode: string
  /** 人間が engage したか。**サーバが真値**（GUI 側で持たない） */
  engaged: boolean
  params: Record<string, number>
  catalog: AutoPlannerInfo[]
  /** engage したまま `auto/cmd` が途絶して制動に落ちた回数 */
  stalls: number
}

/** `/ws/telemetry` が 20Hz で送ってくるスナップショット。 */
export type Snapshot = {
  /** 型定義そのものから決まる札（`generated/msgs.ts` の `MSGS_SCHEMA`）。
   * **食い違ったらこのフレームを信じてはいけない**（`ws/telemetry.ts` が捨てる） */
  schema: number
  t_server: number
  vs: VehicleState | null
  link: LinkDiag | null
  /** 点群は 10Hz なので、同じ周は2回送られてこない（新しいときだけ入る） */
  scan: Scan | null
  /** **engage していなくても流れる**（走らせる前に何をするか見られる） */
  auto: AutoState | null
  ctl: { has_controller: boolean; controller: string }
  /** RasPi 本体（SoC）の温度 ℃。STM32側の `vs.temp` とは別枠。実機以外（シム等）では null */
  pi_temp_c: number | null
}

/** Pi5純正クーリングファンの状態。**サーバが真値**（`ws/control.ts` の `setFan`）。 */
export type FanStatus = {
  mode: 'auto' | 'manual'
  /** 手動時の目標値 [0.0-1.0] */
  duty: number
  /** この機体で手動デューティ制御が使えるか。false なら手動UIを無効化する */
  available: boolean
  /** 実測回転数。取得できなければ null */
  rpm: number | null
}

/** `models/` にある `.onnx` の1件（`cam_model_list` の応答）。 */
export type CamModelFile = {
  name: string
  size: number
  /** UNIX epoch秒 */
  mtime: number
  /** 前処理設定（`ml/export_onnx.py` が書く `<name>.json`）が同梱されているか。
   * 無ければ既定値（0-1正規化・224x224）で読まれる */
  has_config: boolean
}

/** cam_perception_node が使うセグメンテーションモデルの選択。**サーバが真値**
 * （`ws/control.ts` の `camModelSelect`）。空文字は未選択 */
export type CamModelStatus = {
  name: string
}

/** capture側(camera_node)のFPS上限・後方カメラON/OFFの意思と、GUIへの配信頻度。
 * **サーバが真値**（`ws/control.ts` の `setCamera`）。 */
export type CameraConfigStatus = {
  /** 後方カメラの取得(capture)自体を止めるか */
  rear_enabled: boolean
  /** 前カメラのcapture fps上限（目安）。カメラを使う自動運転モードのengage中は無視される */
  front_cap_hz: number
  /** 後カメラのcapture fps上限。自動運転はrear_capture_fpsを使わないので上書きは無い */
  rear_cap_hz: number
  /** ブラウザへ送るJPEGの頻度。`front_cap_hz`/`rear_cap_hz` とは別物（Wi-Fi帯域が理由） */
  gui_hz: number
  /** 実際にcamera_nodeへ指示している前カメラのfps（自動運転中はfront_cap_hzより優先して最大になる） */
  front_fps_effective: number
  /** カメラを使う自動運転モードでengage中で、front_cap_hzを上書きしているか */
  auto_override: boolean
}

/** 接続中Wi-FiのSSID・電波強度。**サーバが真値**（`raspi/io/wifi.py`、1Hzで再取得）。 */
export type WifiStatus = {
  /** 接続中のSSID。未接続なら null */
  ssid: string | null
  /** 電波強度 [dBm]。取得できなければ null */
  rssi_dbm: number | null
  /** この機体でWi-Fi状態の取得自体ができるか（Mac等の開発機では false） */
  available: boolean
}

/** `/ws/control` のサーバ → GUI。 */
export type ControlStatus = {
  type: 'status'
  has_controller: boolean
  controller: string
  /** io_node が arm を封印しているか。**GUI 側では覆せない** */
  arm_inhibited: boolean
  health: string
  estop_active: boolean
  drive_power_locked: boolean
  /** STM32側で実際に適用されているTC/TVの有効状態（★v0.8）。CONFIG_ACK未受信ならnull */
  tc_enabled: boolean | null
  tv_enabled: boolean | null
  /** STM32側で実際に適用されている片輪浮き対策の有効状態（★v0.9）。TC/TV本体とは独立した別機構。CONFIG_ACK未受信ならnull */
  wheel_lift_guard_enabled: boolean | null
  clients: { telemetry: number; control: number; camera: Record<string, number> }
  camera_encoder: string | null
  /** JPEG エンコードの実測。**telemetry_node と logger_node が同じフレームを
   * 別々に焼いている**ので、共通化する価値があるかをここの数字で判断する
   * （2026-08-21 のレビュー 🟡7）。カメラ配信が無効なら null */
  camera_jpeg: {
    impl: string | null
    encoded: number
    /** 1枚あたりのエンコード CPU 時間 [ms] */
    cpu_per_frame_ms: number
    cpu_total_s: number
    /** 書き手に上書きされて捨てた枚数（seqlock の検証で弾いたぶん） */
    torn: number
    errors: number
    /** 共有メモリが作り直されて attach し直した回数 */
    reattached: number
    /** 直近の失敗理由。**数だけでは原因が分からない** */
    last_error: string
  } | null
  deadman_trips: number
  /** サーバが共有トークンを要求しているか（`--token`）。**要求されていて手元に
   * トークンが無ければ操縦権が取れない** ので、GUI はその旨を出す */
  auth_required: boolean
  /** トークン不一致で弾いた回数。**無言で捨てた数は必ず表に出す**
   * （出さないと「効いていない」と「そもそも来ていない」が区別できない） */
  auth_rejects: number
  /** Origin 不一致で弾いた WebSocket 接続の回数 */
  origin_rejects: number
  /** 型が壊れていて捨てた `cmd` の回数。**増え続けるなら GUI 側のバグ** */
  bad_cmds: number
  /** `.sfl` を録ってほしいという意思。実際に開閉するのは io_node 側 */
  sfl: { active: boolean }
  /** mcap のライブ中継。**Piのディスクには一切書かない**（ブラウザが直接ダウンロードする） */
  mcap: { active: boolean; elapsed_s: number; error: string | null }
  /** 自動運転の意思と、選べるモードの宣言。**サーバが真値** */
  auto: AutoStatus
  /** Pi5純正ファンの意思。**サーバが真値** */
  fan: FanStatus
  /** 接続中Wi-FiのSSID・電波強度。**サーバが真値** */
  wifi: WifiStatus
  /** capture側のFPS上限・後方カメラON/OFF・GUI配信頻度。**サーバが真値** */
  camera_config: CameraConfigStatus
  /** cam_perception_node が使うセグメンテーションモデルの選択。**サーバが真値** */
  cam_model: CamModelStatus
}

/** `logs/` にある `.sfl`/`.mcap` の1件（`logs_list` の応答）。 */
export type LogFile = {
  name: string
  kind: 'sfl' | 'mcap'
  size: number
  /** UNIX epoch秒（Pythonの `os.stat().st_mtime` そのまま） */
  mtime: number
}

/** `/ws/control` のサーバ → GUI。`logs_list`/`logs_delete` への応答。 */
export type LogsMsg = {
  type: 'logs'
  files: LogFile[]
}

/** GUI → サーバ の走行指令。SI 単位。 */
export type CmdOut = {
  type: 'cmd'
  /**
   * 0=DISARM 1=MANUAL 2=AUTO。
   *
   * **`2` は「自律走行を許す」の意味。** このとき `speed`/`steer` は
   * telemetry_node が `auto/cmd` の値に差し替えるので、GUI は 0 を送る。
   * それ以外（灯火・ホーン・`auto_stop`・レートリミット・`arm`）はそのまま通る。
   */
  mode: number
  arm: boolean
  /** **立てている間 `speed` は STM32 側で無視され、`brake_torque` が直接掛かる**（v0.5） */
  brake: boolean
  /** **立てている間ずっと鳴る。** v0.4 の「押した瞬間に1発」ではない */
  horn: boolean
  /** 0=OFF 1=DAYTIME 2=NORMAL。**v0.4 の `light=1` は NORMAL だったが 1 は DAYTIME** */
  light_mode: number
  /** パッシング。前照灯だけが全光量になる（**尾灯は連動しない**） */
  passing: boolean
  speed: number
  steer: number
  accel_limit: number
  steer_rate_limit: number
  /** 後輪**各輪**の制動トルク [N·m]。
   * **0 は「制動しない」ではなく「未指定」で、STM32 の最大値で制動する** */
  brake_torque: number
  /** **立っている間 `speed` は無視され、`target_torque` が駆動トルクとして直接掛かる**（v0.6） */
  torque_mode: boolean
  /** [N·m] 駆動トルク直接指令。`torque_mode` のときだけ意味を持つ。負は後退方向（v0.6） */
  target_torque: number
  /** **立っている間、STM32 が単独で**進行方向の超音波を見て 20cm 未満なら最大制動する（v0.7）。
   * 進行方向は `torque_mode` なら `target_torque`、そうでなければ `speed` の符号で決まり、
   * 逆方向のセンサは見ない。優先順位は `side_brake` > `brake` > `auto_stop` > 通常指令 */
  auto_stop: boolean
  /** サイドブレーキ（v0.13）。**立っている間、速度に関わらず即座に**後輪を機械的な
   * 位置制御へ切り替えて固定する。`brake` より優先。**トグル**（ON にしたら明示的に
   * OFF にするまで送り続ける）で扱う想定——駐車ブレーキのように「かけたら離れられる」
   * ものであって、押しっぱなしが必要な `brake`/`horn` とは性質が違う */
  side_brake: boolean
}
