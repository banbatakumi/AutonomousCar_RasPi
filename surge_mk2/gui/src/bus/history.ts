/**
 * 診断タブの時系列グラフ用のリングバッファ。
 *
 * ## なぜ常時記録するのか
 *
 * **異常に気づいてから診断タブを開いても手遅れ**では意味がない。開いた瞬間に
 * 「さっきの3分」が見えている必要がある。そのため記録はタブの表示と無関係に、
 * WS がつながっている間ずっと走らせる（`ws/telemetry.ts` から `pushHistory`）。
 *
 * ## 20Hz を 10Hz に間引く
 *
 * 人間が形として読めるのはこの程度で、点を倍にしても線の形は変わらない。
 * 180秒 × 10Hz = 1800 点 × 系列数。Float32 で持てば 100KB 前後に収まる。
 *
 * ## 累積カウンタは「増分」で持つ
 *
 * CRC エラーやパケットロスは単調増加するので、そのまま描くと右上がりの直線に
 * なるだけで「いつ起きたか」が読めない。**前回サンプルとの差**を格納する。
 *
 * ## フラグも貯める（イベント帯・イベントログ）
 *
 * ARM・E-STOP・TC 介入などのフラグは、現在値だけ見ても「いつ起きたか」「何と同時か」が
 * 分からない。サンプルごとにビット列（`EV`）で貯めて時間軸の帯にし、変化した瞬間は
 * 文章にして `eventLog()` に残す。
 */
import type { LinkDiag, VehicleState } from '../types'
import { RAD2DEG, healthLevel, type Level } from '../format'
import { VEHICLE } from '../generated/vehicle'
import { useUi } from '../store/ui'
import { cmdOut, live } from './live'

/** 左右のトルク差（右−左）[N·m] → ヨーモーメント [N·m]（駆動力の差 × トレッドの半分） */
const DIFF_TORQUE_TO_MOMENT = VEHICLE.track / (2 * VEHICLE.wheelRadius)

/** 保持時間 [s]。180秒あれば「走り1本ぶん」を振り返れる */
export const HISTORY_SPAN_S = 180
const HISTORY_HZ = 10
/** 切断のたびに1点（`markHistoryGap`）余分に積むので、少しだけ余裕を持たせる */
const CAP = HISTORY_SPAN_S * HISTORY_HZ + 64
const MIN_INTERVAL_MS = 1000 / HISTORY_HZ
/** サンプルの間隔がこれより空いたら「その間は届いていなかった」とみなす [s] */
const STALL_S = 0.5
/** 1サンプルが表す長さ [s]（次のサンプルが無い・遠いときに帯を伸ばす量） */
const SAMPLE_S = 1.5 / HISTORY_HZ

/**
 * 系列の定義。**ここと `pushHistory` に足せば貯まる**（グラフの定義は `components/diag/chartDefs.ts`）。
 * 単位はグラフにそのまま出すので、SI から離れる場合はここで変換する。
 */
export const SERIES = [
  'speed', // m/s 実測
  'speedCmd', // m/s 自分が送っている指令
  'wheelFL', // m/s 車輪速（射影なし）
  'wheelFR',
  'wheelRL',
  'wheelRR',
  'steer', // ° 実測（rad から変換して格納する。グラフは度で読む）
  'steerCmd', // ° STM32 が受理した指令
  'temp0', // ℃ MD後左
  'temp1', // ℃ MD後右
  'temp2', // ℃ MDステア
  'temp3', // ℃ MCU
  'tempPi', // ℃ RasPi の CPU
  'battDrive', // V 駆動系
  'battSignal', // V 信号系
  'currDrive', // A 駆動系（単方向。回生中は 0 に張り付く）
  'currSignal', // A 信号系
  'motorRL', // A
  'motorRR', // A
  'motorST', // A
  'slipRL', // % TC用スリップ率（無次元を100倍して%表示）
  'slipRR', // %
  'tcLimitRL', // N·m TCが動的に決めるトルク上限
  'tcLimitRR', // N·m
  'trqReqRL', // N·m スリップ制限が絞る前に掛けたかったトルク（制動は負）
  'trqReqRR', // N·m
  'trqCmdRL', // N·m 実際に送ったトルク指令
  'trqCmdRR', // N·m
  'absLimit', // N·m ABS が決める制動トルクの上限
  'yawRate', // rad/s 実測
  'tvRatio', // % TV の配分の比率（左右の荷重差÷荷重和 の見積り。左旋回で正＝右輪が多い）
  'tvMoment', // N·m TV が要求した左右差のヨーモーメント換算
  'tvApplied', // N·m 実際に付いた左右差をヨーモーメントに直したもの（TC の片輪絞りぶんも含む）
  'accelX', // m/s² 前後
  'accelY', // m/s² 左右
  'usFront', // m 超音波（無効は欠測）
  'usRear', // m
  'rttMs', // ms UART 往復
  'wsRttMs', // ms ブラウザ ⇄ Pi の往復
  'rxHz', // Hz テレメトリ実受信レート
  'scanHz', // Hz LiDAR の周が届くレート
  'crcDelta', // 件/サンプル Pi が受けた側の CRC エラーの増分
  'lossDelta', // 件/サンプル パケットロスの増分
  'stmCrcDelta', // 件/サンプル STM32 が受けた側の CRC エラーの増分
  'loopMaxMs', // ms io_node のループ1周の最大
  'hbLateMs', // ms GPIO6 ハートビートの最大遅れ（起動からの最大）
  'wifiDbm', // dBm
  'piCpu', // % RasPi の CPU 使用率（全コア平均）
  'piCpuMax', // % いちばん忙しいコア
] as const

export type SeriesKey = (typeof SERIES)[number]

/** フラグのビット（`readFlags` が返す `bits`）。帯の色は `components/diag/EventTimeline.tsx` */
export const EV = {
  ARM: 1 << 0,
  AUTO: 1 << 1,
  ESTOP: 1 << 2,
  /** 過電流・低電圧（`vs.faults`）と駆動電源のラッチ遮断 */
  FAULT: 1 << 3,
  /** リンクの health が OK でない */
  LINK: 1 << 4,
  TC: 1 << 5,
  ABS: 1 << 6,
  TV: 1 << 7,
  AUTOSTOP: 1 << 8,
  LIFT: 1 << 9,
  /** STM32 が COMMAND 途絶で自動ブレーキ中 */
  UART: 1 << 10,
  /** テレメトリが届いていない区間（切断・途絶）。`readFlags` が合成する */
  GAP: 1 << 11,
} as const

/** `md_status` のビット（`raspi/proto/protocol.toml` の `[enums.md_status]`） */
export const MD = {
  RUNNING: 0x01,
  VOLTAGE_OOR: 0x02,
  OVERHEAT: 0x04,
  OVERCURRENT: 0x08,
  COMM_OK: 0x10,
  LIMIT_SYNCED: 0x20,
} as const

export const MD_LABEL = ['MD後左', 'MD後右', 'MDステア'] as const

/** 時刻 [s]（`performance.now()` 由来。ページを開いてからの経過時間） */
const ts = new Float64Array(CAP)
const data: Record<SeriesKey, Float32Array> = Object.fromEntries(
  SERIES.map((k) => [k, new Float32Array(CAP)]),
) as Record<SeriesKey, Float32Array>
const flags = new Uint32Array(CAP)

let head = 0 // 次に書く位置
let count = 0 // 有効サンプル数（CAP で頭打ち）
let lastPushMs = 0

/** 累積カウンタの前回値。差を取って増分にする */
const prevCount: Record<string, number | null> = {}

function delta(name: string, cur: number | null | undefined): number {
  const prev = prevCount[name] ?? null
  prevCount[name] = cur ?? null
  // 初回は差が取れない。**0 ではなく NaN**（「エラー無し」と混同させない）
  return cur == null || prev == null ? NaN : Math.max(0, cur - prev)
}

/** 温度は `null`（MD が無言）がありうる。**0 で埋めない** — グラフが 0℃ に落ちて誤読する */
function nz(v: number | null | undefined): number {
  return v == null ? NaN : v
}

// ── イベントログ ──────────────────────────────────────────────────

export type DiagEvent = {
  /** 壁時計 [ms]（`Date.now()`。一覧の時刻表示用） */
  wall: number
  /** 履歴と同じ時刻軸 [s] */
  t: number
  level: Level | 'info'
  text: string
}

const EVENT_CAP = 300
const events: DiagEvent[] = []
let eventRevision = 0

function logEvent(level: DiagEvent['level'], text: string): void {
  events.push({ wall: Date.now(), t: performance.now() / 1000, level, text })
  if (events.length > EVENT_CAP) events.shift()
  eventRevision++
}

/** 新しいものが末尾。**同じ配列を書き換え続ける**ので、変化は `eventRev()` で見る */
export function eventLog(): readonly DiagEvent[] {
  return events
}

export function eventRev(): number {
  return eventRevision
}

/** 変化を見るための前回値。`null` は「まだ1回も見ていない」（最初の1点では何も言わない） */
let prevSeen: {
  bits: number
  health: string
  faults: string
  locked: boolean
  cmdSource: string
  cmdStale: boolean
  md: [number, number, number]
  imuOk: boolean
  lidarOk: boolean
  stmResets: number
  hbStalls: number
  drift: number
} | null = null
let gapOpen = false

const MD_FLAG_TEXT: [number, string, Level][] = [
  [MD.OVERHEAT, '過熱', 'bad'],
  [MD.OVERCURRENT, '過電流', 'bad'],
  [MD.VOLTAGE_OOR, '電圧異常', 'warn'],
]

function noteChanges(vs: VehicleState, l: LinkDiag | null, bits: number): void {
  const cur = {
    bits,
    health: l?.health ?? '',
    faults: vs.faults.join(','),
    locked: vs.drive_power_locked,
    cmdSource: l?.cmd_source ?? '',
    cmdStale: l?.cmd_stale ?? false,
    md: [vs.md_status[0], vs.md_status[1], vs.md_status[2]] as [number, number, number],
    imuOk: vs.imu_ok,
    lidarOk: vs.lidar_ok,
    stmResets: l?.stm_resets ?? 0,
    hbStalls: l?.hb_stalls ?? 0,
    drift: l?.control_params_drift ?? 0,
  }
  const p = prevSeen
  prevSeen = cur
  if (gapOpen) {
    gapOpen = false
    logEvent('ok', 'テレメトリ再開')
  }
  if (!p) return

  const edge = (bit: number, on: string, off: string, onLevel: DiagEvent['level'], offLevel: DiagEvent['level'] = 'info') => {
    if ((bits & bit) && !(p.bits & bit)) logEvent(onLevel, on)
    else if (!(bits & bit) && (p.bits & bit)) logEvent(offLevel, off)
  }
  edge(EV.ARM, 'ARM', 'DISARM', 'info')
  edge(EV.AUTO, 'AUTO モード開始', 'AUTO モード終了', 'info')
  edge(EV.ESTOP, 'E-STOP 発動', 'E-STOP 解除', 'bad', 'ok')
  edge(EV.UART, 'STM32: COMMAND 途絶で自動ブレーキ', 'STM32: COMMAND 復帰', 'bad', 'ok')
  edge(EV.AUTOSTOP, '自動停止 作動', '自動停止 解除', 'warn')

  if (cur.health !== p.health && l) {
    logEvent(healthLevel(cur.health), `リンク ${p.health || '—'} → ${cur.health}`)
  }
  if (cur.faults !== p.faults) {
    const was = new Set(p.faults ? p.faults.split(',') : [])
    const now = new Set(vs.faults)
    for (const f of now) if (!was.has(f)) logEvent('bad', `フォルト発生: ${f}`)
    for (const f of was) if (!now.has(f)) logEvent('ok', `フォルト解消: ${f}`)
  }
  if (cur.locked !== p.locked) {
    logEvent(cur.locked ? 'bad' : 'ok', cur.locked ? '駆動電源 ラッチ遮断' : '駆動電源 ラッチ解除')
  }
  if (l && cur.cmdSource !== p.cmdSource) {
    const odd = /deadman|stale/.test(cur.cmdSource)
    logEvent(odd ? 'warn' : 'info', `指令元 ${p.cmdSource || 'なし'} → ${cur.cmdSource || 'なし'}`)
  }
  if (l && cur.cmdStale !== p.cmdStale) {
    logEvent(cur.cmdStale ? 'warn' : 'info', cur.cmdStale ? 'cmd 途絶（io_node が DISARM を送る）' : 'cmd 復帰')
  }
  for (let i = 0; i < 3; i++) {
    const a = p.md[i as 0 | 1 | 2]
    const b = cur.md[i as 0 | 1 | 2]
    if (a === b) continue
    const name = MD_LABEL[i as 0 | 1 | 2]
    if ((a & MD.COMM_OK) !== (b & MD.COMM_OK)) {
      logEvent(b & MD.COMM_OK ? 'ok' : 'bad', `${name} ${b & MD.COMM_OK ? '通信復帰' : '通信断'}`)
    }
    for (const [bit, text, level] of MD_FLAG_TEXT) {
      if ((b & bit) && !(a & bit)) logEvent(level, `${name} ${text}`)
      else if (!(b & bit) && (a & bit)) logEvent('ok', `${name} ${text} 解消`)
    }
  }
  if (cur.imuOk !== p.imuOk) logEvent(cur.imuOk ? 'ok' : 'bad', cur.imuOk ? 'IMU 復帰' : 'IMU 異常')
  if (cur.lidarOk !== p.lidarOk) logEvent(cur.lidarOk ? 'ok' : 'bad', cur.lidarOk ? 'LiDAR 復帰' : 'LiDAR 異常')
  if (cur.stmResets > p.stmResets) logEvent('bad', 'STM32 の再起動を検出')
  if (cur.hbStalls > p.hbStalls) logEvent('bad', 'GPIO6 ハートビートが停滞')
  if (cur.drift > p.drift) logEvent('warn', '制御パラメータの食い違いを検出して再送')
}

// ── 記録 ──────────────────────────────────────────────────────────

/**
 * 1サンプル積む。20Hz で呼ばれるが、10Hz に間引かれる。
 * `link` は `vs` と同じ頻度で来るとは限らないので、無いときは直前の `live.link` を使う。
 */
export function pushHistory(vs: VehicleState | null, link: LinkDiag | null): void {
  if (!vs) return
  const now = performance.now()
  if (now - lastPushMs < MIN_INTERVAL_MS) return
  lastPushMs = now

  const l = link ?? live.link
  const ui = useUi.getState()
  const pi = ui.status?.pi

  const i = head
  ts[i] = now / 1000
  data.speed[i] = vs.stopped ? 0 : vs.speed
  data.speedCmd[i] = cmdOut.active && !cmdOut.torqueMode ? cmdOut.speed : NaN
  data.wheelFL[i] = vs.wheel_speed[0]
  data.wheelFR[i] = vs.wheel_speed[1]
  data.wheelRL[i] = vs.wheel_speed[2]
  data.wheelRR[i] = vs.wheel_speed[3]
  data.steer[i] = vs.steer_actual * RAD2DEG
  data.steerCmd[i] = vs.steer_cmd_echo * RAD2DEG
  data.temp0[i] = nz(vs.temp[0])
  data.temp1[i] = nz(vs.temp[1])
  data.temp2[i] = nz(vs.temp[2])
  data.temp3[i] = nz(vs.temp[3])
  data.tempPi[i] = nz(live.piTempC)
  data.battDrive[i] = vs.batt_voltage[0]
  data.battSignal[i] = vs.batt_voltage[1]
  data.currDrive[i] = vs.batt_current[0]
  data.currSignal[i] = vs.batt_current[1]
  data.motorRL[i] = vs.motor_current[0]
  data.motorRR[i] = vs.motor_current[1]
  data.motorST[i] = vs.motor_current[2]
  // 無次元（±1=±100%空転/ロック）を % 表示にする。基準速度未満は STM32 側が 0 を送る
  data.slipRL[i] = vs.tc_slip[0] * 100
  data.slipRR[i] = vs.tc_slip[1] * 100
  data.tcLimitRL[i] = vs.tc_limit_nm[0]
  data.tcLimitRR[i] = vs.tc_limit_nm[1]
  // v0.16 より前のファーム・古い記録の再生ではこれらのキーが無い。0 の線を引かず欠測にする
  const req = vs.torque_req
  data.trqReqRL[i] = nz(req?.[0])
  data.trqReqRR[i] = nz(req?.[1])
  data.trqCmdRL[i] = vs.torque_cmd[0]
  data.trqCmdRR[i] = vs.torque_cmd[1]
  data.absLimit[i] = nz(vs.abs_limit_nm)
  data.yawRate[i] = vs.yaw_rate
  data.tvRatio[i] = nz(vs.tv_ratio) * 100
  data.tvMoment[i] = nz(vs.tv_moment_nm)
  data.tvApplied[i] = (vs.torque_cmd[1] - vs.torque_cmd[0]) * DIFF_TORQUE_TO_MOMENT
  data.accelX[i] = vs.accel[0]
  data.accelY[i] = vs.accel[1]
  data.usFront[i] = nz(vs.us_front)
  data.usRear[i] = nz(vs.us_rear)
  data.rttMs[i] = nz(l?.cmd_rtt_ms)
  data.wsRttMs[i] = nz(ui.wsRttMs)
  data.rxHz[i] = live.rxHz
  data.scanHz[i] = live.scanHz
  data.crcDelta[i] = delta('crc', l?.rx?.crc_error)
  data.lossDelta[i] = delta('loss', l?.rx?.packet_loss)
  data.stmCrcDelta[i] = delta('stmCrc', l?.stm_rx?.rx_crc_error)
  data.loopMaxMs[i] = nz(l?.loop_max_ms)
  data.hbLateMs[i] = nz(l?.hb_max_late_ms)
  data.wifiDbm[i] = nz(ui.status?.wifi?.rssi_dbm)
  data.piCpu[i] = nz(pi?.cpu_pct)
  data.piCpuMax[i] = nz(pi?.cpu_max_pct)

  const bits =
    (vs.armed ? EV.ARM : 0) |
    (vs.mode === 2 ? EV.AUTO : 0) |
    (vs.estop_active ? EV.ESTOP : 0) |
    (vs.faults.length > 0 || vs.drive_power_locked ? EV.FAULT : 0) |
    (l && l.health !== 'OK' ? EV.LINK : 0) |
    (vs.tc_active ? EV.TC : 0) |
    (vs.abs_active ? EV.ABS : 0) |
    (vs.tv_active ? EV.TV : 0) |
    (vs.auto_stop_active ? EV.AUTOSTOP : 0) |
    (vs.wheel_lift_active ? EV.LIFT : 0) |
    (vs.uart_timeout ? EV.UART : 0)
  flags[i] = bits
  noteChanges(vs, l, bits)

  head = (head + 1) % CAP
  if (count < CAP) count++
}

/**
 * 切断時に呼ぶ。**バッファは消さない。**
 *
 * 「切れた」こと自体が後から見たい事象なので、履歴を捨てるのは最悪の対応になる。
 * 代わりに全系列 `NaN` の1点を積んで**線をそこで断つ**。uPlot は NaN を欠測として
 * 扱うので、切断の前後がつながって見えることはない。
 *
 * 累積カウンタの基準も落とす。再接続後に STM32 側がリセットされていると
 * 差が負に振れるため、次の1点は増分を出さない（`NaN` になる）。
 */
export function markHistoryGap(): void {
  const i = head
  ts[i] = performance.now() / 1000
  for (const k of SERIES) data[k][i] = NaN
  flags[i] = EV.GAP
  head = (head + 1) % CAP
  if (count < CAP) count++
  for (const k of Object.keys(prevCount)) prevCount[k] = null
  // 1度も受けていない（起動直後の接続待ち）なら、切れたとは言わない
  if (prevSeen && !gapOpen) logEvent('bad', 'テレメトリ切断')
  gapOpen = prevSeen != null
}

// ── 読み出し ──────────────────────────────────────────────────────

/** いまの時刻（履歴と同じ軸）[s]。グラフの右端に使う */
export function historyNow(): number {
  return performance.now() / 1000
}

/** 窓 `(end − spanS, end]` に入るサンプルの、リング上の添字（古い順） */
function windowIndices(spanS: number, end: number): number[] {
  const out: number[] = []
  const start = (head - count + CAP) % CAP
  const from = end - spanS
  for (let k = 0; k < count; k++) {
    const i = (start + k) % CAP
    const t = ts[i]!
    if (t > end) break
    if (t > from) out.push(i)
  }
  return out
}

/**
 * uPlot に渡す形（`[x, y0, y1, ...]`）に展開する。`end` を過去にすれば一時停止中の表示になる
 * （記録は止めないので、止めている間のぶんは窓の外に貯まっていく）。
 *
 * ## ⚠ 欠測は `NaN` ではなく `null` にして返す
 *
 * **uPlot の欠測マーカーは `null` であって `NaN` ではない。** uPlot の範囲計算は
 * `v != null` で値を拾うので、`NaN` はその判定を通り抜けて `Math.min/max` に入り、
 * **min/max が丸ごと NaN に汚染される**。すると軸のスケールが決まらず、
 * その系列どころか**グラフ全体が1本も描かれない**（値は正しく入っているのに白紙になる）。
 *
 * 保持側を Float32Array にしている以上 `null` は格納できないので、
 * 「貯めるときは NaN・渡すときは null」に変換するのがここの役目。
 */
export function readHistory(
  keys: readonly SeriesKey[],
  spanS: number = HISTORY_SPAN_S,
  end: number = historyNow(),
): (number | null)[][] {
  const idx = windowIndices(spanS, end)
  const n = idx.length
  const x: number[] = new Array(n)
  const ys: (number | null)[][] = keys.map(() => new Array(n))
  // 系列ごとの Float32Array は**ループの外で1回だけ引く。**
  const src = keys.map((key) => data[key])
  for (let k = 0; k < n; k++) {
    const i = idx[k]!
    x[k] = ts[i]!
    for (let s = 0; s < keys.length; s++) {
      const v = src[s]![i]!
      ys[s]![k] = Number.isNaN(v) ? null : v
    }
  }
  return [x, ...ys]
}

/** フラグが立っていた区間 `[開始, 終了]`（履歴と同じ時刻軸）の並び */
export type Span = [number, number]

export type FlagTrack = { t: number[]; bits: number[]; end: number }

/**
 * 窓の中のフラグ列。**サンプルが空いた区間には `EV.GAP` を合成して挟む**
 * （WS は切れていないのに届いていない「途絶」も、切断と同じく帯で見えるようにする）。
 */
export function readFlags(spanS: number = HISTORY_SPAN_S, end: number = historyNow()): FlagTrack {
  const idx = windowIndices(spanS, end)
  const t: number[] = []
  const bits: number[] = []
  for (let k = 0; k < idx.length; k++) {
    const i = idx[k]!
    const ti = ts[i]!
    const b = flags[i]!
    t.push(ti)
    bits.push(b)
    const next = k + 1 < idx.length ? ts[idx[k + 1]!]! : end
    if (!(b & EV.GAP) && next - ti > STALL_S) {
      t.push(ti + SAMPLE_S)
      bits.push(EV.GAP)
    }
  }
  return { t, bits, end }
}

/** `mask` のどれかが立っていた区間をつなげて返す */
export function flagSpans(track: FlagTrack, mask: number): Span[] {
  const out: Span[] = []
  const { t, bits, end } = track
  for (let k = 0; k < t.length; k++) {
    if (!(bits[k]! & mask)) continue
    const a = t[k]!
    const b = k + 1 < t.length ? t[k + 1]! : bits[k]! & EV.GAP ? end : Math.min(end, a + SAMPLE_S)
    const last = out[out.length - 1]
    if (last && a <= last[1] + 1e-6) last[1] = b
    else out.push([a, b])
  }
  return out
}

/** 窓の中の最小・最大と、最初と最後の有効値（上昇率の計算用）。有効値が無ければ null */
export function seriesStats(
  key: SeriesKey,
  spanS: number,
  end: number = historyNow(),
): { min: number; max: number; first: number; last: number; dtS: number } | null {
  const src = data[key]
  let min = Infinity
  let max = -Infinity
  let first = NaN
  let last = NaN
  let tFirst = 0
  let tLast = 0
  for (const i of windowIndices(spanS, end)) {
    const v = src[i]!
    if (Number.isNaN(v)) continue
    if (Number.isNaN(first)) {
      first = v
      tFirst = ts[i]!
    }
    last = v
    tLast = ts[i]!
    if (v < min) min = v
    if (v > max) max = v
  }
  if (Number.isNaN(first)) return null
  return { min, max, first, last, dtS: tLast - tFirst }
}

export function historyCount(): number {
  return count
}
