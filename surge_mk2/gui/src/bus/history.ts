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
 */
import type { LinkDiag, VehicleState } from '../types'
import { RAD2DEG } from '../format'
import { cmdOut, live } from './live'

/** 保持時間 [s]。180秒あれば「走り1本ぶん」を振り返れる */
export const HISTORY_SPAN_S = 180
const HISTORY_HZ = 10
const CAP = HISTORY_SPAN_S * HISTORY_HZ
const MIN_INTERVAL_MS = 1000 / HISTORY_HZ

/**
 * 系列の定義。**ここに足せばバッファもグラフの選択肢も増える**（1箇所で完結させる）。
 * 単位はグラフの軸ラベルにそのまま出すので、SI から離れる場合はここで変換する。
 */
export const SERIES = [
  'speed', // m/s 実測
  'speedCmd', // m/s 自分が送っている指令
  'steer', // ° 実測（rad から変換して格納する。グラフは度で読む）
  'steerCmd', // ° STM32 が受理した指令
  'temp0', // ℃ MD後左
  'temp1', // ℃ MD後右
  'temp2', // ℃ MDステア
  'temp3', // ℃ MCU
  'battDrive', // V 駆動系
  'battSignal', // V 信号系
  'currDrive', // A 駆動系
  'motorRL', // A
  'motorRR', // A
  'motorST', // A
  'accelX', // m/s² 前後
  'accelY', // m/s² 左右
  'rttMs', // ms UART 往復
  'rxHz', // Hz テレメトリ実受信レート
  'crcDelta', // 件/サンプル CRC エラーの増分
  'lossDelta', // 件/サンプル パケットロスの増分
] as const

export type SeriesKey = (typeof SERIES)[number]

/** 時刻 [s]（uPlot は秒で受け取る。ページを開いてからの経過時間） */
const ts = new Float64Array(CAP)
const data: Record<SeriesKey, Float32Array> = Object.fromEntries(
  SERIES.map((k) => [k, new Float32Array(CAP)]),
) as Record<SeriesKey, Float32Array>

let head = 0 // 次に書く位置
let count = 0 // 有効サンプル数（CAP で頭打ち）
let lastPushMs = 0
let prevCrc: number | null = null
let prevLoss: number | null = null

/** 温度は `null`（MD が無言）がありうる。**0 で埋めない** — グラフが 0℃ に落ちて誤読する */
function nz(v: number | null | undefined): number {
  return v == null ? NaN : v
}

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
  const crc = l?.rx?.crc_error ?? null
  const loss = l?.rx?.packet_loss ?? null

  const i = head
  ts[i] = now / 1000
  data.speed[i] = vs.stopped ? 0 : vs.speed
  data.speedCmd[i] = cmdOut.active && !cmdOut.torqueMode ? cmdOut.speed : NaN
  data.steer[i] = vs.steer_actual * RAD2DEG
  data.steerCmd[i] = vs.steer_cmd_echo * RAD2DEG
  data.temp0[i] = nz(vs.temp[0])
  data.temp1[i] = nz(vs.temp[1])
  data.temp2[i] = nz(vs.temp[2])
  data.temp3[i] = nz(vs.temp[3])
  data.battDrive[i] = vs.batt_voltage[0]
  data.battSignal[i] = vs.batt_voltage[1]
  data.currDrive[i] = vs.batt_current[0]
  data.motorRL[i] = vs.motor_current[0]
  data.motorRR[i] = vs.motor_current[1]
  data.motorST[i] = vs.motor_current[2]
  data.accelX[i] = vs.accel[0]
  data.accelY[i] = vs.accel[1]
  data.rttMs[i] = nz(l?.cmd_rtt_ms)
  data.rxHz[i] = live.rxHz
  // 初回は差が取れない。**0 ではなく NaN**（「エラー無し」と混同させない）
  data.crcDelta[i] = crc == null || prevCrc == null ? NaN : Math.max(0, crc - prevCrc)
  data.lossDelta[i] = loss == null || prevLoss == null ? NaN : Math.max(0, loss - prevLoss)
  prevCrc = crc
  prevLoss = loss

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
  head = (head + 1) % CAP
  if (count < CAP) count++
  prevCrc = null
  prevLoss = null
}

/**
 * uPlot に渡す形（`[x, y0, y1, ...]`）に展開する。
 * リングを古い順に並べ直すだけなので、呼ぶたびに配列を作り直してよい
 * （1800点 × 数系列で、4Hz の再描画には十分速い）。
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
export function readHistory(keys: readonly SeriesKey[]): (number | null)[][] {
  const n = count
  const start = (head - n + CAP) % CAP
  const x: number[] = new Array(n)
  const ys: (number | null)[][] = keys.map(() => new Array(n))
  // 系列ごとの Float32Array は**ループの外で1回だけ引く。**
  // `noUncheckedIndexedAccess` で `keys[s]` が `| undefined` になるのを
  // 内側で毎回捌くより、ここで確定させたほうが速くも読みやすくもなる
  const src = keys.map((key) => data[key])
  for (let k = 0; k < n; k++) {
    const i = (start + k) % CAP
    x[k] = ts[i]!
    for (let s = 0; s < keys.length; s++) {
      const v = src[s]![i]!
      ys[s]![k] = Number.isNaN(v) ? null : v
    }
  }
  return [x, ...ys]
}

export function historyCount(): number {
  return count
}
