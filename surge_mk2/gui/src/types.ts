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
  dist: number[] // 360点 [m] 添字がそのまま度。0 = 無効
  sector_t_ns: number[] // 12
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
}

/** GUI → サーバ の走行指令。SI 単位。 */
export type CmdOut = {
  type: 'cmd'
  mode: number
  arm: boolean
  brake: boolean
  horn: boolean
  light: boolean
  speed: number
  steer: number
  accel_limit: number
  steer_rate_limit: number
}
