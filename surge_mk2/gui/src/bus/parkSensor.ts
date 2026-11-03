/**
 * クリアランスソナー（2026-11-03）。車体まわり 360° を `SECTORS` 方向に分け、方向ごとに
 * **車体の外形から一番近い物までの距離**を出して4段に直す。表示（`components/rc/ParkSensor.tsx`）と
 * 音（`hooks/useParkBeep.ts`）が同じしきい値を使うように、ここにだけ書く。
 *
 * 距離の出どころ:
 * - **LiDAR**（全周）: 点を base_link に置き、車体の外形（`VEHICLE.footprint`）までの距離を測る。
 *   方向は車体の中心から見た角度で決める。外形の中の点（自分の車体の写り込み）は捨てる
 * - **超音波**（前後の正面）: 真正面・真後ろの方向だけ、LiDAR と比べて近い方を使う。LiDAR は
 *   取付高さの一平面しか見ないので、低い物は超音波にしか映らない
 *
 * **自動停止（STM32、20cm 未満で最大制動）とは独立した GUI の演出。** 止めるのは STM32 で、
 * ここは「止まる前に分かる」ためのもの。
 */
import { VEHICLE } from '../generated/vehicle'
import type { Scan, VehicleState } from '../types'

/** 方向の数。0 が真正面、反時計回り（左回り）に増える */
export const SECTORS = 16
/** 段のしきい値 [m]（車体の外形から）。遠い順。距離がこれを下回るたびに1段近くなる（4段） */
export const PARK_STEPS_M = [0.4, 0.3, 0.2, 0.1] as const
/** これより近ければ連続音 [m]（一番近い段と同じ） */
const CONT_M = PARK_STEPS_M[3]
/** 断続音が鳴り始める距離 [m] と、そこでの間隔・連続音の直前の間隔 [ms] */
const BEEP_START_M = PARK_STEPS_M[0]
const BEEP_SLOW_MS = 700
const BEEP_FAST_MS = 120
/** これより速く走っている間は表示も音も出さない [m/s]（コースの壁で出っぱなしにしない。駐車の速さだけ） */
export const PARK_MAX_SPEED_MS = 1.0
/** 見る範囲 [m]。一番遠い段より少し先まで */
const MAX_RANGE_M = 1.2

const XS = VEHICLE.footprint.map(([x]) => x)
const YS = VEHICLE.footprint.map(([, y]) => y)
const X0 = Math.min(...XS)
const X1 = Math.max(...XS)
const Y0 = Math.min(...YS)
const Y1 = Math.max(...YS)
/** 車体の中心（base_link）。方向はここから見た角度 */
export const BODY_CENTER_X = (X0 + X1) / 2
/** 車体の大きさ [m]（表示の縮尺に使う） */
export const BODY_LEN_M = X1 - X0
export const BODY_WIDTH_M = Y1 - Y0

/** 段（0=何も無い、1=遠い〜4=一番近い） */
export function stepOf(d: number | null | undefined): number {
  if (d == null || !isFinite(d)) return 0
  return PARK_STEPS_M.filter((t) => d < t).length
}

/** 方向ごとの、車体の外形から一番近い物までの距離 [m]（無ければ Infinity）。長さ `SECTORS` */
export function sectorClearance(scan: Scan | null, vs: VehicleState | null): number[] {
  const out = new Array<number>(SECTORS).fill(Infinity)
  const width = (2 * Math.PI) / SECTORS
  const sectorOf = (x: number, y: number) => {
    const a = Math.atan2(y, x - BODY_CENTER_X)
    return ((Math.round(a / width) % SECTORS) + SECTORS) % SECTORS
  }
  if (scan) {
    for (let deg = 0; deg < 360; deg++) {
      const d = scan.dist[deg]
      if (!d || d > MAX_RANGE_M + BODY_LEN_M) continue
      const a = (deg * Math.PI) / 180
      const x = VEHICLE.lidar.x + d * Math.cos(a)
      const y = VEHICLE.lidar.y + d * Math.sin(a)
      // 外形（四角）までの距離。中なら 0 → 自分の車体の写り込みとして捨てる
      const dx = Math.max(X0 - x, 0, x - X1)
      const dy = Math.max(Y0 - y, 0, y - Y1)
      const c = Math.hypot(dx, dy)
      if (c <= 0 || c > MAX_RANGE_M) continue
      const s = sectorOf(x, y)
      if (c < out[s]!) out[s] = c
    }
  }
  // 超音波は前後のバンパーからの距離（`vs.us_*`）。真正面・真後ろの方向に足す
  if (vs?.us_front != null) out[0] = Math.min(out[0]!, vs.us_front)
  if (vs?.us_rear != null) out[SECTORS / 2] = Math.min(out[SECTORS / 2]!, vs.us_rear)
  return out
}

/**
 * 鳴らし方。`null` は鳴らさない、`0` は連続音、正の値は断続音の周期 [ms]
 */
export function beepPeriodMs(d: number | null | undefined): number | null {
  if (d == null || !isFinite(d) || d >= BEEP_START_M) return null
  if (d < CONT_M) return 0
  const t = (d - CONT_M) / (BEEP_START_M - CONT_M)
  return BEEP_FAST_MS + t * (BEEP_SLOW_MS - BEEP_FAST_MS)
}

/**
 * 音で知らせる距離。**進む向きの半周**（後退なら後ろ半分、前進なら前半分）の一番近い距離。
 * 止まっている前進・速い前進は null（鳴らさない）
 */
export function relevantDistance(sectors: number[], vs: VehicleState | null, reverse: boolean): number | null {
  if (!vs) return null
  const back = reverse || vs.speed < -0.05
  if (!back && !(vs.speed > 0.05 && vs.speed < PARK_MAX_SPEED_MS)) return null
  // 前半分 = 方向 -3..+3（±67.5°）、後ろ半分 = 方向 5..11
  const half = SECTORS / 4
  let m = Infinity
  for (let i = -half + 1; i <= half - 1; i++) {
    const s = ((back ? SECTORS / 2 + i : i) + SECTORS) % SECTORS
    m = Math.min(m, sectors[s]!)
  }
  return m
}
