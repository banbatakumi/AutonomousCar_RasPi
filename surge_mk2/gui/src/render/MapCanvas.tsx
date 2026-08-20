/**
 * 地図ビュー — **世界座標（map フレーム）**の俯瞰。
 *
 * `LidarView` が車両基準なのに対し、こちらは地図が固定で車が動く。座標は
 * `architecture.md` §5.1 のまま（x = 前/右、y = 左/上）で、画面では x を右・
 * y を上に取る（数学の教科書と同じ向き。点群ビューのように「上が進行方向」に
 * すると、地図が周回のたびに回って形が読めなくなる）。
 *
 * ## これは「地図生成中だけ見る画面」ではない
 *
 * EXPLORE で地図が育つのを見る画面であると同時に、**RACE 中に自車が
 * レーシングラインのどこに居るか・動的障害物がどこに出たか**を見る画面でもある。
 * どちらも世界座標の情報で、車両基準のビューには置きようがない。
 *
 * ## 描く順序に意味がある
 *
 *   占有格子 → 中心線 → レーシングライン → 軌跡 → 障害物 → 自車
 *
 * **後に描いたものほど手前**。判断の材料（格子）より判断の結果（経路）を、
 * それより今の状態（自車・障害物）を上に置く。
 *
 * ## 拡大率は「観測できた範囲」に合わせる
 *
 * 地図の枠は 24m 四方あるが、コースはその一部しか占めない。枠に合わせると
 * 豆粒になるので、`MapData.known`（未知でないセルの外接矩形）に合わせる。
 */
import { useEffect, useRef } from 'react'
import { live } from '../bus/live'
import { VEHICLE as VEHICLE_GEOM } from '../generated/vehicle'

/** 車体の描画用寸法 [m]。`LidarView.tsx` と同じ算出方法（`config/vehicle.toml` の
 * `footprint` から生成）。手で数値を書かない。 */
const VEHICLE = {
  wheelbase: VEHICLE_GEOM.wheelbase,
  front: Math.max(...VEHICLE_GEOM.footprint.map(([x]) => x)),
  back: -Math.min(...VEHICLE_GEOM.footprint.map(([x]) => x)),
  width: 2 * Math.max(...VEHICLE_GEOM.footprint.map(([, y]) => Math.abs(y))),
}

const C = {
  bg: '#0b0e11',
  grid: '#161d24',
  gridText: '#3d4852',
  center: '#4a5560',
  trail: '#2f4a5c',
  body: '#2f3d4a',
  bodyLine: '#8fb6d1',
  target: '#5ef0a8',
  obstacle: '#e0574d',
  text: '#8a99a8',
}

/** 速度 [m/s] → 色。**遅い＝赤、速い＝緑**。アウトインアウトが効いているかは
 *  「コーナーで赤く、立ち上がりで緑に戻る」で読む。 */
function speedColor(v: number, lo: number, hi: number): string {
  const t = hi > lo ? Math.max(0, Math.min(1, (v - lo) / (hi - lo))) : 0.5
  const h = 0 + t * 120 // 0=赤 120=緑
  return `hsl(${h}, 75%, 55%)`
}

/** 走った跡（世界座標）。**コンポーネントの外に置く。** タブを切り替えて
 *  戻ってきたときに軌跡が消えていると、走行中の経過を見返せない。 */
const trail: [number, number][] = []

/** 軌跡を捨てる。engage し直したとき（＝地図を作り直すとき）に押す。 */
export function clearTrail() {
  trail.length = 0
}

export function MapCanvas() {
  const ref = useRef<HTMLCanvasElement>(null)

  useEffect(() => {
    const cv = ref.current
    if (!cv) return
    const ctx = cv.getContext('2d')
    if (!ctx) return
    let raf = 0

    const draw = () => {
      raf = requestAnimationFrame(draw)
      const dpr = window.devicePixelRatio || 1
      const w = cv.clientWidth
      const h = cv.clientHeight
      if (cv.width !== w * dpr || cv.height !== h * dpr) {
        cv.width = w * dpr
        cv.height = h * dpr
      }
      ctx.setTransform(dpr, 0, 0, dpr, 0, 0)
      ctx.fillStyle = C.bg
      ctx.fillRect(0, 0, w, h)

      const map = live.map
      const auto = live.auto

      // ── 表示範囲を決める。地図が無い間は自車の周り 8m ──
      let box = map?.known
      if (!box) {
        const cx = auto?.pose_x ?? 0
        const cy = auto?.pose_y ?? 0
        box = { x0: cx - 4, y0: cy - 4, x1: cx + 4, y1: cy + 4 }
      }
      const pad = 0.4
      const bw = Math.max(0.5, box.x1 - box.x0 + pad * 2)
      const bh = Math.max(0.5, box.y1 - box.y0 + pad * 2)
      const px = Math.min(w / bw, h / bh)
      const ox = (box.x0 + box.x1) / 2
      const oy = (box.y0 + box.y1) / 2
      // 世界 → 画面。**y を反転**（世界は上が +y、画面は下が +y）
      const sx = (x: number) => w / 2 + (x - ox) * px
      const sy = (y: number) => h / 2 - (y - oy) * px

      drawGrid(ctx, w, h, sx, sy, px, box)

      // ── 占有格子 ──
      if (map?.bitmap) {
        ctx.imageSmoothingEnabled = false
        ctx.drawImage(
          map.bitmap,
          sx(map.originX),
          sy(map.originY + map.height * map.res),
          map.width * map.res * px,
          map.height * map.res * px,
        )
        ctx.imageSmoothingEnabled = true
      }

      if (map) {
        drawPolyline(ctx, map.centerline, sx, sy, C.center, 1, [4, 4])
        drawRaceline(ctx, map, sx, sy)
      }

      // ── 軌跡。**地図が無い段でも自分がどう走ったかは見たい** ──
      if (auto && (auto.pose_x || auto.pose_y)) {
        const t = trail
        const last = t[t.length - 1]
        if (!last || Math.hypot(auto.pose_x - last[0], auto.pose_y - last[1]) > 0.05) {
          t.push([auto.pose_x, auto.pose_y])
          if (t.length > 4000) t.shift()
        }
        ctx.beginPath()
        for (let i = 0; i < t.length; i++) {
          const p = t[i]
          if (i === 0) ctx.moveTo(sx(p[0]), sy(p[1]))
          else ctx.lineTo(sx(p[0]), sy(p[1]))
        }
        ctx.strokeStyle = C.trail
        ctx.lineWidth = 1.5
        ctx.stroke()
      }

      if (auto) {
        drawObstacles(ctx, auto.obstacles ?? [], sx, sy, px)
        if (auto.phase === 'RACE' && (auto.target_x || auto.target_y)) {
          ctx.beginPath()
          ctx.arc(sx(auto.target_x), sy(auto.target_y), 4, 0, Math.PI * 2)
          ctx.strokeStyle = C.target
          ctx.lineWidth = 2
          ctx.stroke()
        }
        drawVehicle(ctx, auto.pose_x, auto.pose_y, auto.pose_yaw, sx, sy, px)
      }
      drawScale(ctx, w, h, px)
    }
    raf = requestAnimationFrame(draw)
    return () => cancelAnimationFrame(raf)
  }, [])

  return <canvas ref={ref} className="map-canvas" />
}

function drawGrid(
  ctx: CanvasRenderingContext2D,
  w: number,
  h: number,
  sx: (x: number) => number,
  sy: (y: number) => number,
  px: number,
  box: { x0: number; y0: number; x1: number; y1: number },
) {
  // 1m 刻み。狭いときだけ 0.5m にする
  const step = px > 90 ? 0.5 : px > 25 ? 1 : 2
  ctx.strokeStyle = C.grid
  ctx.lineWidth = 1
  ctx.beginPath()
  for (let x = Math.floor(box.x0 / step) * step; x <= box.x1 + step; x += step) {
    ctx.moveTo(sx(x), 0)
    ctx.lineTo(sx(x), h)
  }
  for (let y = Math.floor(box.y0 / step) * step; y <= box.y1 + step; y += step) {
    ctx.moveTo(0, sy(y))
    ctx.lineTo(w, sy(y))
  }
  ctx.stroke()
  // 原点（SLAM の起点 ＝ engage した場所）
  ctx.strokeStyle = C.gridText
  ctx.beginPath()
  ctx.moveTo(sx(0) - 6, sy(0))
  ctx.lineTo(sx(0) + 6, sy(0))
  ctx.moveTo(sx(0), sy(0) - 6)
  ctx.lineTo(sx(0), sy(0) + 6)
  ctx.stroke()
}

function drawPolyline(
  ctx: CanvasRenderingContext2D,
  pts: Float64Array,
  sx: (x: number) => number,
  sy: (y: number) => number,
  color: string,
  width: number,
  dash: number[] = [],
) {
  if (pts.length < 4) return
  ctx.setLineDash(dash)
  ctx.beginPath()
  for (let i = 0; i + 1 < pts.length; i += 2) {
    const x = sx(pts[i])
    const y = sy(pts[i + 1])
    if (i === 0) ctx.moveTo(x, y)
    else ctx.lineTo(x, y)
  }
  ctx.closePath()
  ctx.strokeStyle = color
  ctx.lineWidth = width
  ctx.stroke()
  ctx.setLineDash([])
}

/** レーシングラインを**速度で色分け**して描く。
 *  1本の線を色だけ変えると継ぎ目が切れるので、区間ごとに引き直す。 */
function drawRaceline(
  ctx: CanvasRenderingContext2D,
  map: { raceline: Float64Array; racelineV: Float64Array },
  sx: (x: number) => number,
  sy: (y: number) => number,
) {
  const p = map.raceline
  const v = map.racelineV
  if (p.length < 4) return
  let lo = Infinity
  let hi = -Infinity
  for (const s of v) {
    if (s < lo) lo = s
    if (s > hi) hi = s
  }
  const n = p.length / 2
  ctx.lineWidth = 3
  ctx.lineCap = 'round'
  for (let i = 0; i < n; i++) {
    const j = (i + 1) % n
    ctx.beginPath()
    ctx.moveTo(sx(p[i * 2]), sy(p[i * 2 + 1]))
    ctx.lineTo(sx(p[j * 2]), sy(p[j * 2 + 1]))
    ctx.strokeStyle = v.length > i ? speedColor(v[i], lo, hi) : '#ffc63f'
    ctx.stroke()
  }
  ctx.lineCap = 'butt'
}

/** 動的障害物。**半径ぶんの円**で描く（点で描くと大きさが読めない）。 */
function drawObstacles(
  ctx: CanvasRenderingContext2D,
  flat: number[],
  sx: (x: number) => number,
  sy: (y: number) => number,
  px: number,
) {
  for (let i = 0; i + 2 < flat.length; i += 3) {
    ctx.beginPath()
    ctx.arc(sx(flat[i]), sy(flat[i + 1]), Math.max(3, flat[i + 2] * px), 0, Math.PI * 2)
    ctx.fillStyle = `${C.obstacle}55`
    ctx.fill()
    ctx.strokeStyle = C.obstacle
    ctx.lineWidth = 1.5
    ctx.stroke()
  }
}

function drawVehicle(
  ctx: CanvasRenderingContext2D,
  x: number,
  y: number,
  yaw: number,
  sx: (v: number) => number,
  sy: (v: number) => number,
  px: number,
) {
  const W = VEHICLE.width * px
  const front = VEHICLE.front * px
  const back = VEHICLE.back * px
  ctx.save()
  ctx.translate(sx(x), sy(y))
  // 世界の反時計回り正 → 画面は y 反転なので符号を返す
  ctx.rotate(-yaw)
  ctx.beginPath()
  ctx.rect(-back, -W / 2, front + back, W)
  ctx.fillStyle = C.body
  ctx.fill()
  ctx.strokeStyle = C.bodyLine
  ctx.lineWidth = 1.5
  ctx.stroke()
  // 前を示す線
  ctx.beginPath()
  ctx.moveTo(0, 0)
  ctx.lineTo(front, 0)
  ctx.stroke()
  ctx.restore()
}

function drawScale(ctx: CanvasRenderingContext2D, w: number, h: number, px: number) {
  const meters = px > 90 ? 0.5 : px > 25 ? 1 : 5
  const len = meters * px
  const x = w - len - 16
  const y = h - 16
  ctx.strokeStyle = C.text
  ctx.lineWidth = 2
  ctx.beginPath()
  ctx.moveTo(x, y)
  ctx.lineTo(x + len, y)
  ctx.moveTo(x, y - 4)
  ctx.lineTo(x, y + 4)
  ctx.moveTo(x + len, y - 4)
  ctx.lineTo(x + len, y + 4)
  ctx.stroke()
  ctx.fillStyle = C.text
  ctx.font = '11px ui-monospace, monospace'
  ctx.fillText(`${meters}m`, x + len / 2 - 8, y - 6)
}
