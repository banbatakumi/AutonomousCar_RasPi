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
 *   占有格子 → 道路グラフ → 中心線 → 他のグループの経路 → レーシングライン
 *   → 経由点・停止点 → 軌跡 → 障害物 → 自車
 *
 * **後に描いたものほど手前**。判断の材料（格子）より判断の結果（経路）を、
 * それより今の状態（自車・障害物）を上に置く。
 *
 * ## 拡大率は「観測できた範囲」に合わせる
 *
 * 地図の枠は 24m 四方あるが、コースはその一部しか占めない。枠に合わせると
 * 豆粒になるので、`MapData.known`（未知でないセルの外接矩形）に合わせる。
 *
 * ## プレビュー中は地図だけ描き、自車・軌跡・障害物は出さない
 *
 * `bus/mapPreview.ts`の`mapPreview.data`が非nullの間はそちらを描く
 * （「レーシングライン走行」を押す前の下見、`AutoMapPanel.tsx`参照）。
 * プレビュー中の地図に対応する実測の自己位置は無いので、`live.auto`は
 * 読まない——`auto`をnullのままにしておけば、下の`if (auto) {...}`群が
 * 自然に何も描かなくなる（2026-09-03）。
 */
import { useEffect, useRef } from 'react'
import { live } from '../bus/live'
import { mapPreview } from '../bus/mapPreview'
import { GROUP_COLOR, parseCfg, routeEdit, type RouteCfg } from '../bus/routeEdit'
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
  //: 自己位置ヒント（`bus/mapPreview.ts`の`mapPreview.hint`）。自車（`bodyLine`の
  //: 青系）・障害物（赤）・経路（速度で色分け）のどれとも被らない色にする
  hint: '#ffc63f',
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

/** 地図に載っている経路の設定。**文字列が変わったときだけ**解析し直す（毎フレーム
 *  `JSON.parse` しない） */
let cfgText = ''
let cfgParsed: RouteCfg | null = null
function cachedCfg(text: string): RouteCfg | null {
  if (!text) return null
  if (text !== cfgText) {
    cfgText = text
    cfgParsed = parseCfg(text)
  }
  return cfgParsed
}

/** 軌跡を捨てる。engage し直したとき（＝地図を作り直すとき）に押す。 */
export function clearTrail() {
  trail.length = 0
}

export function MapCanvas({
  onWorldClick,
}: {
  /** `pxPerM` は今の拡大率（画面ピクセル/m）。経由点の削除判定を画面の距離で揃えるのに使う */
  onWorldClick?: (x: number, y: number, pxPerM: number) => void
}) {
  const ref = useRef<HTMLCanvasElement>(null)
  //: 直近フレームの世界→画面変換。`onClick`は描画ループの外（Reactのイベント
  //: ハンドラ）から呼ばれるので、`sx`/`sy`のクロージャではなく`ref`で最新値を渡す
  const xform = useRef({ ox: 0, oy: 0, px: 1 })

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

      const previewing = mapPreview.data !== null
      const map = previewing ? mapPreview.data : live.map
      const auto = previewing ? null : live.auto

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
      xform.current = { ox, oy, px }

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
        drawGraph(ctx, map.graph, map.graphBreaks, sx, sy)
        drawPolyline(ctx, map.centerline, sx, sy, C.center, 1, [4, 4])
        // 走っていないグループの経路（細く）。今の経路は下の速度色の線が重なる
        for (const [k, pts] of Object.entries(map.routes)) {
          if (k === map.routeActive) continue
          const pending = auto?.route_pending === k
          drawPolyline(ctx, pts, sx, sy, GROUP_COLOR[k] ?? '#888', pending ? 2 : 1,
            pending ? [8, 5] : [2, 4], k !== 'stop')
        }
        drawRaceline(ctx, map, sx, sy, map.routeActive !== 'stop')
        // 経由点・停止点。**編集中は手元の写し**を、それ以外は地図に載っている設定を描く
        const cfg = routeEdit.cfg ?? cachedCfg(map.routesJson)
        if (cfg) drawRouteCfg(ctx, cfg, sx, sy, routeEdit.active ? routeEdit.target : '')
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
          const p = t[i]!            // 添字は length 未満なので必ず在る
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
      if (mapPreview.hint) drawHint(ctx, mapPreview.hint.x, mapPreview.hint.y, sx, sy)
      drawScale(ctx, w, h, px)
    }
    raf = requestAnimationFrame(draw)
    return () => cancelAnimationFrame(raf)
  }, [])

  const handleClick = (e: React.MouseEvent<HTMLCanvasElement>) => {
    if (!onWorldClick || !ref.current) return
    const rect = ref.current.getBoundingClientRect()
    const { ox, oy, px } = xform.current
    // 画面座標 → 世界座標。`sx`/`sy`の逆変換（中心基準、yは反転）
    const wx = ox + (e.clientX - rect.left - rect.width / 2) / px
    const wy = oy - (e.clientY - rect.top - rect.height / 2) / px
    onWorldClick(wx, wy, px)
  }

  return <canvas ref={ref} className="map-canvas" onClick={onWorldClick ? handleClick : undefined} />
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
  closed = true,
) {
  if (pts.length < 4) return
  ctx.setLineDash(dash)
  ctx.beginPath()
  // `i + 1 < length` で回しているので、`i` と `i+1` はどちらも範囲内
  for (let i = 0; i + 1 < pts.length; i += 2) {
    const x = sx(pts[i]!)
    const y = sy(pts[i + 1]!)
    if (i === 0) ctx.moveTo(x, y)
    else ctx.lineTo(x, y)
  }
  if (closed) ctx.closePath()
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
  closed = true,
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
  // 開いた経路（停止点へ向かう経路）は終点→始点をつながない
  for (let i = 0; i < (closed ? n : n - 1); i++) {
    const j = (i + 1) % n
    ctx.beginPath()
    // `n = p.length / 2` なので `i*2+1` も `j*2+1` も範囲内（`j` は環状に折り返す）
    ctx.moveTo(sx(p[i * 2]!), sy(p[i * 2 + 1]!))
    ctx.lineTo(sx(p[j * 2]!), sy(p[j * 2 + 1]!))
    ctx.strokeStyle = v.length > i ? speedColor(v[i]!, lo, hi) : '#ffc63f'
    ctx.stroke()
  }
  ctx.lineCap = 'butt'
}

/** 道路グラフ（`slam2d_route`）。**判断の材料**なので格子の次に、目立たない色で描く。
 *  分岐・合流の節点は小さな点で示す（エッジの端がそれ）。 */
function drawGraph(
  ctx: CanvasRenderingContext2D,
  pts: Float64Array,
  breaks: number[],
  sx: (x: number) => number,
  sy: (y: number) => number,
) {
  const n = pts.length / 2
  if (n < 2) return
  ctx.strokeStyle = '#3a4a58'
  ctx.lineWidth = 1
  ctx.beginPath()
  for (let b = 0; b < breaks.length; b++) {
    const i0 = breaks[b]!
    const i1 = b + 1 < breaks.length ? breaks[b + 1]! : n
    for (let i = i0; i < i1; i++) {
      const x = sx(pts[i * 2]!)
      const y = sy(pts[i * 2 + 1]!)
      if (i === i0) ctx.moveTo(x, y)
      else ctx.lineTo(x, y)
    }
  }
  ctx.stroke()
  ctx.fillStyle = '#6b7c8c'
  for (const i0 of breaks) {
    ctx.beginPath()
    ctx.arc(sx(pts[i0 * 2]!), sy(pts[i0 * 2 + 1]!), 2.5, 0, Math.PI * 2)
    ctx.fill()
  }
}

/** 経由点（グループの色の番号付き丸）と停止点（名前付きの四角＋向きの矢印）。
 *  `editing` は今編集している対象（強調して描く）。 */
function drawRouteCfg(
  ctx: CanvasRenderingContext2D,
  cfg: RouteCfg,
  sx: (x: number) => number,
  sy: (y: number) => number,
  editing: string,
) {
  ctx.font = '10px ui-monospace, monospace'
  ctx.textAlign = 'center'
  ctx.textBaseline = 'middle'
  for (const [g, pts] of Object.entries(cfg.groups)) {
    const col = GROUP_COLOR[g] ?? '#888'
    const hot = editing === g
    pts.forEach((p, i) => {
      const cx = sx(p[0]!)
      const cy = sy(p[1]!)
      ctx.beginPath()
      ctx.arc(cx, cy, hot ? 8 : 6, 0, Math.PI * 2)
      ctx.fillStyle = hot ? col : `${col}66`
      ctx.fill()
      ctx.strokeStyle = col
      ctx.lineWidth = 1.5
      ctx.stroke()
      ctx.fillStyle = hot ? '#0b0e11' : '#e8e8e8'
      ctx.fillText(`${hot ? '' : g}${i + 1}`, cx, cy)
      if (p.length > 2 && p[2] !== null && p[2] !== undefined) {
        const a = p[2]
        ctx.beginPath()
        ctx.moveTo(cx + Math.cos(a) * 9, cy - Math.sin(a) * 9)
        ctx.lineTo(cx + Math.cos(a) * 16, cy - Math.sin(a) * 16)
        ctx.strokeStyle = col
        ctx.stroke()
      }
    })
  }
  for (const [name, st] of Object.entries(cfg.stops)) {
    const cx = sx(st.x)
    const cy = sy(st.y)
    const hot = editing === name
    ctx.strokeStyle = hot ? '#ffffff' : '#c8c8c8'
    ctx.lineWidth = hot ? 2.5 : 1.5
    ctx.strokeRect(cx - 7, cy - 7, 14, 14)
    if (st.yaw !== null) {
      ctx.beginPath()
      ctx.moveTo(cx, cy)
      ctx.lineTo(cx + Math.cos(st.yaw) * 18, cy - Math.sin(st.yaw) * 18)
      ctx.stroke()
    }
    ctx.fillStyle = '#e8e8e8'
    ctx.fillText(`${name}${st.mode === 'park' ? '🅿' : ''}`, cx, cy - 14)
  }
  ctx.textAlign = 'start'
  ctx.textBaseline = 'alphabetic'
}

/** 動的障害物。**半径ぶんの円**で描く（点で描くと大きさが読めない）。 */
function drawObstacles(
  ctx: CanvasRenderingContext2D,
  flat: number[],
  sx: (x: number) => number,
  sy: (y: number) => number,
  px: number,
) {
  // `[x, y, r]` の3つ組。`i + 2 < length` で回すので3要素とも範囲内
  for (let i = 0; i + 2 < flat.length; i += 3) {
    ctx.beginPath()
    ctx.arc(sx(flat[i]!), sy(flat[i + 1]!), Math.max(3, flat[i + 2]! * px), 0, Math.PI * 2)
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

/** 自己位置ヒント（`bus/mapPreview.ts`）。**十字＋輪**にして、点だけより
 *  地図上で見失いにくくする（`drawObstacles`の塗り円と紛れないよう輪だけ）。 */
function drawHint(
  ctx: CanvasRenderingContext2D,
  x: number,
  y: number,
  sx: (v: number) => number,
  sy: (v: number) => number,
) {
  const cx = sx(x)
  const cy = sy(y)
  ctx.strokeStyle = C.hint
  ctx.lineWidth = 2
  ctx.beginPath()
  ctx.arc(cx, cy, 8, 0, Math.PI * 2)
  ctx.moveTo(cx - 12, cy)
  ctx.lineTo(cx - 4, cy)
  ctx.moveTo(cx + 4, cy)
  ctx.lineTo(cx + 12, cy)
  ctx.moveTo(cx, cy - 12)
  ctx.lineTo(cx, cy - 4)
  ctx.moveTo(cx, cy + 4)
  ctx.lineTo(cx, cy + 12)
  ctx.stroke()
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
