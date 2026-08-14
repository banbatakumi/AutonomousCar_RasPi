/**
 * バスのメッセージ型（`raspi/msgs/types.py` の写し）。
 *
 * **単位はすべて SI**（m/s, rad, m, A, V, ℃）。km/h や度への変換は
 * 表示の直前（`format.ts`）でだけ行う。ここで度に直すと、以降どの値に
 * 換算が掛かっているのか分からなくなる。
 *
 * Python 側を変えたらここも直すこと。**型が2箇所にあるのでズレうる**が、
 * 生成器を挟むほどの規模ではないと判断している（UART の方は生成している）。
 */

export type VehicleState = {
  t_capture: number
  t_pub: number
  seq: number

  speed: number // m/s 車体中心線方向に射影済み
  yaw_rate: number // rad/s
  steer_actual: number // rad 路面舵角・反時計回り正
  steer_cmd_echo: number // rad 指令のエコーバック
  wheel_speed: number[] // m/s [FL,FR,RL,RR] 前2輪は射影なし
  odom_dist: number[] // m [FL,FR] 累積
  accel: number[] // m/s² [x,y,z]
  pitch: number
  roll: number
  motor_current: number[] // A [RL,RR,ST]
  torque_cmd: number[] // N·m [RL,RR] ★指令値であって実測ではない
  temp: (number | null)[] // ℃ [RL,RR,ST,MCU] null = MD が無言で信用できない
  batt_voltage: number[] // V [駆動, 信号]
  batt_current: number[] // A ★単方向センサ。回生中は 0 に張り付く
  us_front: number | null // m null = 無効
  us_rear: number | null
  md_status: number[]
  flags: number
  cmd_seq_echo: number
  t_stm_us: number

  mode: number // 0=DISARM 1=MANUAL 2=AUTO
  armed: boolean
  estop_active: boolean
  uart_timeout: boolean
  tc_active: boolean
  tv_active: boolean
  imu_ok: boolean
  lidar_ok: boolean
  steer_center_valid: boolean
  drive_power_locked: boolean
  /** 超音波の自動停止が**今まさに制動へ介入中**（v0.7）。有効/無効そのものではない
   * （有効かどうかは GUI が送っている `auto_stop` の側で分かる） */
  auto_stop_active: boolean
  faults: string[]

  odom_center: number // m 射影して積算した中心線距離
  slip_front: number[] // m/s 射影後 - speed
  slip_rear: number[]
  stopped: boolean // デッドバンド内。生値は speed に残っている
}

export type Scan = {
  t_capture: number
  t_pub: number
  seq: number
  // 360点 [m] 添字がそのまま度。0 = 無効。添字は**車両座標**の角度
  // （x=前 が 0°、反時計回りが正）。センサ基準からの左右反転は ScanAssembler で
  // 済ませてあるので、描画側で座標変換してはいけない
  dist: number[]
  sector_t_ns: number[] // 12（こちらも車両座標のセクタ番号）
  sector_dur_us: number[]
  sector_seen: boolean[] // false の区間は「障害物なし」ではない
  rot_speed_dps: number
  intensity: number[] | null
  saturated: boolean[] | null // 5.10m 以上（点を打ってはいけない）
  lidar_format: number
}

export type LinkDiag = {
  t_capture: number
  t_pub: number
  seq: number
  health: 'INIT' | 'OK' | 'DEGRADED' | 'FAULT'
  estop_active: boolean
  drive_power_locked: boolean
  arm_inhibited: boolean
  cmd_source: string
  cmd_stale: boolean
  rx: Record<string, number>
  stm_rx: Record<string, number> | null
  /** STM32 ⇄ MD の受信数 / エラー数 [RL, RR, ST]。**合計にしない**（1台の異常が埋もれる） */
  md_rx_count: number[] | null
  md_rx_error: number[] | null
  counts: Record<string, number>
  sync_offset_ns: number | null
  sync_delay_ns: number | null
  sync_drift_ppm: number | null
  sync_n: number
  cmd_rtt_ms: number | null
  protocol_version: number | null
  fw_id: number | null
  protocol_match: boolean | null
  hb_alive: boolean | null
  hb_max_late_ms: number | null
  hb_stalls: number
  lidar_scans: number
  lidar_sectors_lost: number
  /**
   * 相手が実 STM32 ではなくシミュレータか（`sim/link.py`）。
   *
   * **GUI がシミュレータについて知っている唯一のこと。** コース切替やノイズ量の
   * 操作は `sim.gui`（pygame）側にあり、ここには持ち込まない。これは利便性ではなく
   * 安全のための表示で、シムと実機の画面が見分けられないと
   * 「シムのつもりで --allow-arm した実車が動く」が起きる。
   */
  sim: boolean
}

/**
 * planning_node の判断（`raspi/msgs/types.py` の `AutoState`）。
 *
 * **指令ではなく「なぜそう決めたか」。** 指令だけ見ても原因は分からないので、
 * 選んだギャップ・正面余裕・最近傍を並べて出す。LiDAR ビューがこれを重畳する。
 *
 * 角度は車両座標（x=前 が 0、反時計回りが正）。`*_deg` は**符号付き ±180 度**
 * （前方を跨ぐギャップが `350〜10` に分断されて読めなくなるのを避けるため）。
 */
export type AutoState = {
  t_capture: number
  t_pub: number
  seq: number
  mode: string
  planner: string
  engaged: boolean
  /** 走らせてよい状態か。**false の理由は必ず `reason` に入る** */
  ready: boolean
  /** ready でない理由、または減速・停止の理由。日本語がそのまま入る */
  reason: string
  target_speed: number
  target_steer: number
  brake: boolean
  heading: number // rad
  gap_start_deg: number
  gap_end_deg: number
  /** `start > end` は「バブル無し」を表す */
  bubble_start_deg: number
  bubble_end_deg: number
  free_ahead: number // m ★速度を決めているのはこれ
  nearest: number // m
  nearest_deg: number
  valid_ratio: number

  // ── 地図を使う planner（`raspi/auto/raceline.py`）── */
  /** "EXPLORE"（地図を作っている）/"BUILD"（経路を作っている）/"RACE" */
  phase: string
  /** map フレームの自己位置。**base_link（後輪車軸中心）基準** */
  pose_x: number
  pose_y: number
  pose_yaw: number // rad
  /** スキャンマッチの一致度 0〜1。**低いのに走っているなら疑う** */
  match_score: number
  /** 周回の進み具合 [周]。位置ではなく**累積回頭**で数えている */
  lap_progress: number
  laps: number
  /** レーシングラインからの横偏差 [m]。**RACE 段の主要な指標** */
  cross_track: number
  /** Pure Pursuit が狙っている経路上の点（map フレーム） */
  target_x: number
  target_y: number
  /** 動的障害物 `[x, y, r, ...]` を平坦化したもの（map フレーム） */
  obstacles: number[]

  scan_age_ms: number
  plan_hz: number
}

/** `/ws/map` で届く生のメッセージ（`raspi/msgs/types.py` の `AutoMap`）。 */
export type AutoMapMsg = {
  map_seq: number
  resolution: number
  origin_x: number
  origin_y: number
  width: number
  height: number
  /** 3値を 2bit に詰めて zlib 圧縮したもの。展開は `ws/map.ts` */
  cells: Uint8Array
  centerline: number[]
  raceline: number[]
  raceline_v: number[]
}

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
  t_server: number
  vs: VehicleState | null
  link: LinkDiag | null
  /** 点群は 10Hz なので、同じ周は2回送られてこない（新しいときだけ入る） */
  scan: Scan | null
  /** **engage していなくても流れる**（走らせる前に何をするか見られる） */
  auto: AutoState | null
  ctl: { has_controller: boolean; controller: string }
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
  clients: { telemetry: number; control: number; camera: Record<string, number> }
  camera_encoder: string | null
  deadman_trips: number
  /** `.sfl` を録ってほしいという意思。実際に開閉するのは io_node 側 */
  sfl: { active: boolean }
  /** mcap のライブ中継。**Piのディスクには一切書かない**（ブラウザが直接ダウンロードする） */
  mcap: { active: boolean; elapsed_s: number; error: string | null }
  /** 自動運転の意思と、選べるモードの宣言。**サーバが真値** */
  auto: AutoStatus
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
   * 逆方向のセンサは見ない。優先順位は `brake` > `auto_stop` > 通常指令 */
  auto_stop: boolean
}
