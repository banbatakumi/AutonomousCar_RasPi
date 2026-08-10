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
}

/** `/ws/telemetry` が 20Hz で送ってくるスナップショット。 */
export type Snapshot = {
  t_server: number
  vs: VehicleState | null
  link: LinkDiag | null
  /** 点群は 10Hz なので、同じ周は2回送られてこない（新しいときだけ入る） */
  scan: Scan | null
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
}
