/**
 * 進路上の障害物（2026-11-03）。**今の舵角のまま進んだら、どこで何に当たるか**を LiDAR の点群から求める。
 * 前方映像への重畳（`CameraView.tsx` の `drawObstacles`）が使う。
 *
 * ## やり方
 *
 * 1. 点群（LiDAR の車両角・取付位置ぶんずらして base_link に置く）を、**取得してから今までに
 *    車が動いた分だけ**戻す（`vs.t_capture − scan.t_capture`。どちらも Pi の時計 [ns]）。
 *    LiDAR は 10Hz で、届くのはカメラより遅い。速く走るほど点が実際より遠くに見えるので補う
 * 2. 進路ガイドと同じ自転車モデル（dθ/ds = tanδ/L）で弧を刻み、**刻みごとに車体の外形
 *    （`VEHICLE.footprint` に余裕を足した四角）を置いて、中に入る点を探す**。最初に点が入った
 *    弧長が「当たるまでの距離」。車体の幅で掃くので、ガイドの2本の線の間＝当たる、になる
 * 3. 当たるまでの時間（距離 ÷ 車速）で段を決める。**止まっている（遅い）ときは光らせない**
 *    ——近くに壁があるだけで赤くなると、正常なのに光る画面になる（ダーク・コックピット）
 *
 * 後退は弧長を負に刻むだけでよい（式は弧長の符号によらない。`drawGuide` と同じ）。
 *
 * 範囲外: 車体の外形にすでに入っている点（s=0 で当たる点）は捨てる。LiDAR が自分の車体の
 * 一部を拾うことがあり、常に赤くなるのを避けるため。その近さは超音波（自動停止）が受け持つ。
 */
import { VEHICLE } from '../generated/vehicle'
import type { Scan, VehicleState } from '../types'

/** 弧を刻む間隔 [m] と、見る最大の弧長 [m] */
const STEP_M = 0.04
const MAX_S_M = 3.5
/** 車体の外形に足す余裕 [m]（左右・前後）。ぎりぎり擦る点も「当たる」に入れる */
const MARGIN_M = 0.03
/** 点群の補正に使う時間差の上限 [s]。これより古い点群は補正せず、そもそも描かない側で弾く */
const MAX_COMP_S = 0.4
/** これより近い点は LiDAR 自身・車体の写り込みとして捨てる [m] */
const MIN_RANGE_M = 0.08

/** 当たるまでの時間の段。`danger` 赤・`caution` 琥珀・`info` 白（進路上にあるが余裕がある） */
export type ObstacleLevel = 'danger' | 'caution' | 'info'

/** 赤・琥珀にする当たるまでの時間 [s] と距離 [m]（どちらかを下回ったら。近づいているときだけ） */
const TTC_DANGER_S = 1.0
const TTC_CAUTION_S = 2.0
const DIST_DANGER_M = 0.25
const DIST_CAUTION_M = 0.5
/** これより遅ければ「止まっている」として時間ではなく距離だけで段を決める [m/s] */
const MOVING_MS = 0.15

export type PathObstacle = {
  /** 進行方向に当たるまでの弧長 [m]（車体の前端・後端から） */
  distM: number
  /** 当たるまでの時間 [s]。止まっているときは Infinity */
  ttcS: number
  level: ObstacleLevel
  /** 当たる位置での車体の姿勢（base_link、今の車体基準）。重畳の「止まる線」に使う */
  pose: { x: number; y: number; th: number }
  /** 当たった点（base_link）。重畳で点として出す */
  hits: [number, number][]
  /** 進行方向。後退中は true */
  reverse: boolean
}

const [FOOT_X0, FOOT_X1, FOOT_Y0, FOOT_Y1] = (() => {
  const xs = VEHICLE.footprint.map(([x]) => x)
  const ys = VEHICLE.footprint.map(([, y]) => y)
  return [Math.min(...xs) - MARGIN_M, Math.max(...xs) + MARGIN_M, Math.min(...ys) - MARGIN_M, Math.max(...ys) + MARGIN_M]
})()

/** 車体の前端（前進）・後端（後退）の base_link 上の x [m]。止まる線をここに引く */
export const BUMPER_X = { front: FOOT_X1 - MARGIN_M, rear: FOOT_X0 + MARGIN_M }
/** 車体の半幅 [m] */
export const HALF_WIDTH = (FOOT_Y1 - FOOT_Y0) / 2 - MARGIN_M

/** 点群を base_link に置き、取得から今までの車の動きぶん戻す */
export function scanPointsNow(scan: Scan, vs: VehicleState | null): [number, number][] {
  let dt = 0
  if (vs && vs.t_capture && scan.t_capture) {
    // `t_capture` は1周の最初のセクタの時刻。1周の真ん中（+50ms）を代表にする
    dt = (vs.t_capture - scan.t_capture) / 1e9 - 0.05
    dt = Math.max(0, Math.min(MAX_COMP_S, dt))
  }
  const v = vs?.speed ?? 0
  const dth = (vs?.yaw_rate ?? 0) * dt
  // その間の移動（向きは半分回った方向で近似）と回頭
  const tx = v * dt * Math.cos(dth / 2)
  const ty = v * dt * Math.sin(dth / 2)
  const c = Math.cos(-dth)
  const s = Math.sin(-dth)
  const out: [number, number][] = []
  for (let deg = 0; deg < 360; deg++) {
    const d = scan.dist[deg]
    if (!d || d < MIN_RANGE_M || d > MAX_S_M + 1) continue
    const a = (deg * Math.PI) / 180
    const x = VEHICLE.lidar.x + d * Math.cos(a) - tx
    const y = VEHICLE.lidar.y + d * Math.sin(a) - ty
    out.push([x * c - y * s, x * s + y * c])
  }
  return out
}

/** 外形（余裕込み）の中に入っているか。点 (px,py) を姿勢 (x,y,th) の車体座標に直して比べる */
function inside(px: number, py: number, x: number, y: number, cth: number, sth: number): boolean {
  const dx = px - x
  const dy = py - y
  const lx = dx * cth + dy * sth
  const ly = -dx * sth + dy * cth
  return lx >= FOOT_X0 && lx <= FOOT_X1 && ly >= FOOT_Y0 && ly <= FOOT_Y1
}

/**
 * 今の舵角のまま `reverse` の向きへ進んだとき、最初に当たる点。当たらなければ null。
 * `steer` は路面舵角 [rad]、`speed` は車速 [m/s]（符号付き）
 */
export function findPathObstacle(
  pts: [number, number][],
  steer: number,
  speed: number,
  reverse: boolean,
): PathObstacle | null {
  // s=0 で外形に入っている点は捨てる（冒頭「範囲外」）
  const cand = pts.filter(([px, py]) => !inside(px, py, 0, 0, 1, 0))
  if (cand.length === 0) return null

  const ds = reverse ? -STEP_M : STEP_M
  const k = Math.tan(steer) / VEHICLE.wheelbase
  let x = 0
  let y = 0
  let th = 0
  for (let s = STEP_M; s <= MAX_S_M; s += STEP_M) {
    th += ds * k
    if (Math.abs(th) > Math.PI / 2) break // 横〜後ろへ回り込む先は見ない（`drawGuide` と同じ）
    x += ds * Math.cos(th)
    y += ds * Math.sin(th)
    const cth = Math.cos(th)
    const sth = Math.sin(th)
    const hits = cand.filter(([px, py]) => inside(px, py, x, y, cth, sth))
    if (hits.length === 0) continue

    const distM = Math.max(0, s - STEP_M)
    const v = Math.abs(speed)
    // 進む向きと車速の向きが合っているときだけ「近づいている」
    const approaching = v > MOVING_MS && (speed < 0) === reverse
    const ttcS = approaching ? distM / v : Infinity
    // 近づいていないなら距離が近くても白のまま（止まって壁の前にいるだけで光らせない）
    const level: ObstacleLevel = !approaching
      ? 'info'
      : ttcS < TTC_DANGER_S || distM < DIST_DANGER_M
        ? 'danger'
        : ttcS < TTC_CAUTION_S || distM < DIST_CAUTION_M
          ? 'caution'
          : 'info'
    // 止まる線は「当たる手前の刻み」の姿勢に引く（その刻みではまだ当たっていない）
    const pth = th - ds * k
    const px = x - ds * Math.cos(th)
    const py = y - ds * Math.sin(th)
    return { distM, ttcS, level, pose: { x: px, y: py, th: pth }, hits, reverse }
  }
  return null
}
