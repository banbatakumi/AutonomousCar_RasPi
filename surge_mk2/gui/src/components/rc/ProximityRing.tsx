/**
 * 障害物近接リング — `DrivePanel` の車体図を囲む8角形の線。
 *
 * LiDAR の360点（`Scan.dist[deg]`、車両角）それぞれについて、その方向の障害物点を
 * 車体外形（`VEHICLE.footprint`）からの最短距離に変換し、10cm 未満のときだけ
 * その角度に対応するリング上の位置を黄→赤で塗る。**リングの形・位置は固定**
 * （車体を実寸で囲む8角形をあらかじめ360方向ぶん計算済み）で、変わるのは
 * 各方向の色（＝表示/非表示）だけ——「近づいた分だけ線が動く」ような
 * インジケータにはしない（指示どおり）。
 *
 * 等分割の少数セクタではなく LiDAR の360分解能をそのまま使うので、障害物が
 * 車体角のごく一部だけに迫っている状況（壁の角、他車の先端など）も
 * その方向だけ光る形で表現できる。
 *
 * 障害物点は LiDAR 取付位置（`VEHICLE.lidar`、base_link 基準）ぶん平行移動して
 * から車体外形との距離を測る——センサは車体中心には無い（前寄り7cm）ので、
 * ここを省くと前後で近接判定がずれる。
 */
import { live, STALE_MS, useNumbers } from '../../bus/live'
import { VEHICLE } from '../../generated/vehicle'
import { FOOT_MAX_X, FOOT_MAX_Y, FOOT_MIN_X, FOOT_MIN_Y, toSvg } from './carLayout'

/** これ以上離れていたら非表示 [m] */
const PROX_MAX_M = 0.10
/** 車体外形からリングまでの隙間 [m]（見た目上の余白）。`DrivePanel.tsx` の
 * viewBox（`27 0 146 260`、実描画内容に合わせて横を詰めてある）のもとでは
 * 0.055m 以上で上端・左右どちらかがはみ出す（0.05 は miny≈1.4 でぎりぎり収まる）。
 * viewBox を変えたら要再計算 */
const RING_GAP_M = 0.05
/** 8角形の角の面取り量 [m] */
const RING_CHAMFER_M = 0.045

/** `--warn` と同じ意味の色（10cm ちょうど＝まだ余裕がある側） */
const WARN_RGB: readonly [number, number, number] = [232, 171, 33]
/** `--bad` と同じ意味の色（接触寸前） */
const BAD_RGB: readonly [number, number, number] = [242, 84, 75]

// `carLayout.ts` の車体外形バウンディングボックス・座標変換を共有する
// （`DrivePanel` の車輪配置と同じ実寸比率で揃えるため）。
const minX = FOOT_MIN_X
const maxX = FOOT_MAX_X
const minY = FOOT_MIN_Y
const maxY = FOOT_MAX_Y
const CENTER_X = (minX + maxX) / 2
const CENTER_Y = (minY + maxY) / 2

/** 凸多角形の辺リスト。原点（車体外形の中心）からの角度でリング上の点を引く */
type Edge = readonly [readonly [number, number], readonly [number, number]]

function buildRingEdges(): Edge[] {
  const hl = (maxX - minX) / 2 + RING_GAP_M
  const hw = (maxY - minY) / 2 + RING_GAP_M
  const c = Math.min(RING_CHAMFER_M, hl, hw)
  const a = hl - c
  const b = hw - c
  // 中心（CENTER_X, CENTER_Y）基準のローカル座標 (u=前, v=左)。反時計回り8点。
  const pts: [number, number][] = [
    [hl, -b],
    [hl, b],
    [a, hw],
    [-a, hw],
    [-hl, b],
    [-hl, -b],
    [-a, -hw],
    [a, -hw],
  ]
  return pts.map((p, i) => [p, pts[(i + 1) % pts.length]] as Edge)
}

const RING_EDGES = buildRingEdges()

/** 原点からの角度 `angleRad` の光線が、この凸多角形と交わる点（ローカル座標） */
function raycast(edges: Edge[], angleRad: number): [number, number] {
  const dx = Math.cos(angleRad)
  const dy = Math.sin(angleRad)
  for (const [[x1, y1], [x2, y2]] of edges) {
    const ex = x2 - x1
    const ey = y2 - y1
    const denom = ex * dy - ey * dx
    if (Math.abs(denom) < 1e-9) continue
    const t = (ex * y1 - ey * x1) / denom
    if (t <= 0) continue
    const s = (dx * y1 - dy * x1) / denom
    if (s < -1e-6 || s > 1 + 1e-6) continue
    return [x1 + s * ex, y1 + s * ey]
  }
  return [0, 0]
}

/** 車両角 deg（0=前, 反時計回り）ごとの、リング上の点（svg座標）。並進・回転が
 * 無い限り毎フレーム変わらないので1回だけ計算する */
const RING_SVG_PTS: [number, number][] = Array.from({ length: 360 }, (_, deg) => {
  const [u, v] = raycast(RING_EDGES, (deg * Math.PI) / 180)
  return toSvg(CENTER_X + u, CENTER_Y + v)
})

function lerp(a: number, b: number, t: number): number {
  return a + (b - a) * t
}

function proximityColor(distToBody: number): string {
  const t = Math.max(0, Math.min(1, 1 - distToBody / PROX_MAX_M))
  const r = Math.round(lerp(WARN_RGB[0], BAD_RGB[0], t))
  const g = Math.round(lerp(WARN_RGB[1], BAD_RGB[1], t))
  const b = Math.round(lerp(WARN_RGB[2], BAD_RGB[2], t))
  return `rgb(${r},${g},${b})`
}

export function ProximityRing() {
  // 高頻度の scan.dist は state に入れず、8Hz の再描画トリガー（`useNumbers`）
  // に相乗りして `live.scan` を直接読む（`bus/live.ts` の方針どおり）。
  useNumbers()
  const scan = live.scan
  const fresh = scan != null && performance.now() - live.lastScanMs <= STALE_MS
  if (!fresh || !scan) return null

  const segs: { deg: number; color: string }[] = []
  for (let deg = 0; deg < 360; deg++) {
    const d = scan.dist[deg]
    if (!d) continue
    const a = (deg * Math.PI) / 180
    const obsX = VEHICLE.lidar.x + d * Math.cos(a)
    const obsY = VEHICLE.lidar.y + d * Math.sin(a)
    const dxOut = Math.max(minX - obsX, obsX - maxX, 0)
    const dyOut = Math.max(minY - obsY, obsY - maxY, 0)
    const distToBody = Math.hypot(dxOut, dyOut)
    if (distToBody >= PROX_MAX_M) continue
    segs.push({ deg, color: proximityColor(distToBody) })
  }

  if (segs.length === 0) return null

  return (
    <g>
      {segs.map(({ deg, color }) => {
        const [x1, y1] = RING_SVG_PTS[deg]!
        const [x2, y2] = RING_SVG_PTS[(deg + 1) % 360]!
        return (
          <line
            key={deg}
            className="dp-proximity-seg"
            x1={x1}
            y1={y1}
            x2={x2}
            y2={y2}
            style={{ stroke: color }}
          />
        )
      })}
    </g>
  )
}
