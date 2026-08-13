/**
 * LiDAR 俯瞰ビュー。**Canvas 2D に rAF で直接描く。React を経由させない。**
 *
 * 座標は車両基準（`architecture.md` §5.1）: x = 前, y = 左, 反時計回りが正。
 * 画面では x を上、y を左に取る（上が進行方向）。
 *
 * ## 欠測を「障害物なし」に見せない
 *
 * 受信できなかったセクタは `dist=0` で来る。そのまま点を打たないだけだと
 * **「そこには何も無い」と同じ絵**になり、LiDAR が落ちているのに気づけない。
 * 欠測セクタは扇形をハッチで塗り、見て分かるようにしてある。
 *
 * ## 圧縮フォーマットの飽和点は打たない
 *
 * `saturated`（生値 255 = 5.10m 以上）を実測点として打つと、**実在しない壁が
 * 円状に生成される**（`uart_protocol.md` §5.2）。マーカーを変えて別扱いにする。
 *
 * ## 自動運転の判断を重ねる
 *
 * planning_node が選んだギャップ・安全バブル・狙っている方位を点群の上に描く。
 * **数字だけでは「なぜそっちへ曲がったか」が読めない。** 点群と重ねて初めて、
 * 「この壁を避けている」「欠測を壁として避けている」が一目で分かる。
 *
 * engage していなくても描く（planning_node は常に判断を出している）ので、
 * **手動で走らせながら planner の狙いを検分できる。**
 */
import { useEffect, useRef } from 'react'
import { live } from '../bus/live'
import { useUi } from '../store/ui'

/** 車体寸法 [m]。**未実測の暫定値**（`architecture.md` §15 の #2〜#4）。
 * 原点は base_link ＝ 後輪車軸の中心。実測したら `config/vehicle.yaml` を正とし、
 * ここへは WS 経由で配る（今は形が分かればよいので直書き）。 */
const VEHICLE = {
  width: 0.19,
  wheelbase: 0.25,
  rearOverhang: 0.03, // 原点より後ろに出ている量
  frontOverhang: 0.06, // 前輪より前に出ている量
}

const C = {
  bg: '#0b0e11',
  grid: '#1b2229',
  gridText: '#4a5560',
  body: '#2f3d4a',
  bodyLine: '#6c8296',
  point: '#7fd3ff',
  pointFar: '#3c6f8c',
  saturated: '#4b5a66',
  missing: '#3a2a20',
  path: '#ffc63f',
  us: '#8b6bd6',
  gap: '#2bd67b',
  bubble: '#e0574d',
  heading: '#5ef0a8',
}

export function LidarView() {
  const ref = useRef<HTMLCanvasElement>(null)
  const zoom = useUi((s) => s.lidarZoom)

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
      ctx.clearRect(0, 0, w, h)
      ctx.fillStyle = C.bg
      ctx.fillRect(0, 0, w, h)

      const cx = w / 2
      const cy = h * 0.62 // 車体をやや下に置いて前方を広く取る
      // 画面の短辺半分が zoom [m] に対応する
      const px = Math.min(w, h) / 2 / zoom

      // 車両座標 (x=前, y=左) → 画面
      const sx = (_x: number, y: number) => cx - y * px
      const sy = (x: number, _y: number) => cy - x * px

      drawGrid(ctx, w, h, cx, cy, px, zoom)

      const scan = live.scan
      if (scan) {
        drawMissingSectors(ctx, scan.sector_seen, cx, cy, px, zoom)
        // **点群より先に塗る。** 後から塗ると点が半透明の下に沈んで、
        // ギャップの中に何が見えているのかが読めなくなる
        drawPlan(ctx, cx, cy, px, zoom)
        ctx.save()
        for (let deg = 0; deg < 360; deg++) {
          const d = scan.dist[deg]
          if (!d) continue
          const a = (deg * Math.PI) / 180
          const x = d * Math.cos(a)
          const y = d * Math.sin(a)
          const px2 = sx(x, y)
          const py2 = sy(x, y)
          if (px2 < -10 || px2 > w + 10 || py2 < -10 || py2 > h + 10) continue
          if (scan.saturated?.[deg]) {
            // 「5.1m 以上」であって測距点ではない。**壁として打たない**
            ctx.fillStyle = C.saturated
            ctx.fillRect(px2 - 0.5, py2 - 0.5, 1, 1)
          } else {
            ctx.fillStyle = d > zoom * 0.8 ? C.pointFar : C.point
            ctx.fillRect(px2 - 1.5, py2 - 1.5, 3, 3)
          }
        }
        ctx.restore()
      }

      const vs = live.vs
      if (vs) {
        drawUltrasonic(ctx, vs.us_front, vs.us_rear, sx, sy, px)
        drawPredictedPath(ctx, vs.steer_actual, vs.speed, sx, sy)
      }
      drawVehicle(ctx, cx, cy, px, vs?.steer_actual ?? 0)
    }
    raf = requestAnimationFrame(draw)
    return () => cancelAnimationFrame(raf)
  }, [zoom])

  return <canvas ref={ref} className="lidar-canvas" />
}

function drawGrid(
  ctx: CanvasRenderingContext2D,
  w: number,
  h: number,
  cx: number,
  cy: number,
  px: number,
  zoom: number,
) {
  const step = zoom <= 2 ? 0.5 : zoom <= 5 ? 1 : 2
  ctx.strokeStyle = C.grid
  ctx.fillStyle = C.gridText
  ctx.lineWidth = 1
  ctx.font = '10px ui-monospace, monospace'
  for (let r = step; r <= zoom * 1.6; r += step) {
    ctx.beginPath()
    ctx.arc(cx, cy, r * px, 0, Math.PI * 2)
    ctx.stroke()
    ctx.fillText(`${r}m`, cx + 3, cy - r * px - 2)
  }
  // 正面方向
  ctx.beginPath()
  ctx.moveTo(cx, 0)
  ctx.lineTo(cx, h)
  ctx.moveTo(0, cy)
  ctx.lineTo(w, cy)
  ctx.stroke()
}

/** 受信できなかったセクタ（30°ぶん）を塗る。**空白＝安全ではない。** */
function drawMissingSectors(
  ctx: CanvasRenderingContext2D,
  seen: boolean[],
  cx: number,
  cy: number,
  px: number,
  zoom: number,
) {
  const r = zoom * 1.6 * px
  for (let i = 0; i < 12; i++) {
    if (seen[i]) continue
    // 車両座標の角度 θ → 画面角。画面は上が +x、左が +y
    const a0 = (i * 30 * Math.PI) / 180
    const a1 = ((i + 1) * 30 * Math.PI) / 180
    ctx.beginPath()
    ctx.moveTo(cx, cy)
    // 画面角 = -(θ + 90°) … 上向きを 0 にするための回転
    ctx.arc(cx, cy, r, -a1 - Math.PI / 2, -a0 - Math.PI / 2)
    ctx.closePath()
    ctx.fillStyle = C.missing
    ctx.fill()
  }
}

/**
 * planning_node の判断（`live.auto`）を重ねる。
 *
 * 描くのは3つだけ。**選んだギャップ（緑の扇）・安全バブル（赤の扇）・狙う方位**。
 * これ以上足すと点群が読めなくなる。
 *
 * 角度は車両座標（x=前 が 0、反時計回りが正）で、画面角は `-(θ + 90°)`
 * （`drawMissingSectors` と同じ変換。上向きを 0 にする回転）。
 */
function drawPlan(
  ctx: CanvasRenderingContext2D,
  cx: number,
  cy: number,
  px: number,
  zoom: number,
) {
  const a = live.auto
  if (!a || !a.mode) return
  const r = zoom * 1.6 * px
  const wedge = (fromDeg: number, toDeg: number, fill: string) => {
    const a0 = (fromDeg * Math.PI) / 180
    const a1 = (toDeg * Math.PI) / 180
    ctx.beginPath()
    ctx.moveTo(cx, cy)
    ctx.arc(cx, cy, r, -a1 - Math.PI / 2, -a0 - Math.PI / 2)
    ctx.closePath()
    ctx.fillStyle = fill
    ctx.fill()
  }

  // 選んだギャップ。**ready でないときは描かない**（選べていないので）
  if (a.ready && a.gap_end_deg > a.gap_start_deg) {
    wedge(a.gap_start_deg, a.gap_end_deg, `${C.gap}22`)
  }
  // 安全バブル。`start > end` は「バブル無し」（`AutoState` の約束）
  if (a.bubble_end_deg >= a.bubble_start_deg) {
    wedge(a.bubble_start_deg, a.bubble_end_deg, `${C.bubble}2e`)
  }
  // 狙っている方位。**舵角ではなく「どこを向きたいか」**
  if (a.ready) {
    const th = a.heading
    ctx.beginPath()
    ctx.moveTo(cx, cy)
    ctx.lineTo(cx - Math.sin(th) * r, cy - Math.cos(th) * r)
    ctx.strokeStyle = C.heading
    ctx.lineWidth = 2
    ctx.setLineDash([2, 5])
    ctx.stroke()
    ctx.setLineDash([])
  }
}

function drawVehicle(
  ctx: CanvasRenderingContext2D,
  cx: number,
  cy: number,
  px: number,
  steer: number,
) {
  // base_link は後輪車軸の中心（`architecture.md` §5.2）。原点はそこ
  const W = VEHICLE.width * px
  const front = (VEHICLE.wheelbase + VEHICLE.frontOverhang) * px
  const back = VEHICLE.rearOverhang * px
  ctx.save()
  ctx.translate(cx, cy)

  ctx.beginPath()
  ctx.rect(-W / 2, -front, W, front + back)
  ctx.fillStyle = C.body
  ctx.fill()
  ctx.strokeStyle = C.bodyLine
  ctx.lineWidth = 1.5
  ctx.stroke()

  // 後輪車軸（原点）を示す線
  ctx.beginPath()
  ctx.moveTo(-W / 2, 0)
  ctx.lineTo(W / 2, 0)
  ctx.stroke()

  // 前輪の向き（**実測舵角**）。指令との差がステアリングの遅れそのもの
  ctx.strokeStyle = C.path
  ctx.lineWidth = 3
  for (const side of [-1, 1]) {
    ctx.save()
    ctx.translate((side * W) / 2, -VEHICLE.wheelbase * px)
    ctx.rotate(-steer) // 反時計回りが正。画面は y 反転なので符号を返す
    ctx.beginPath()
    ctx.moveTo(0, -7)
    ctx.lineTo(0, 7)
    ctx.stroke()
    ctx.restore()
  }
  ctx.restore()
}

/**
 * 自転車モデルによる予測進路（`architecture.md` §5.2 の式）。
 *
 * 舵角の実測値から描く。**指令値ではない**ので、ステアリングの遅れが
 * そのまま見える。速度が 0 のときは向きだけを短く描く。
 */
function drawPredictedPath(
  ctx: CanvasRenderingContext2D,
  steer: number,
  speed: number,
  sx: (x: number, y: number) => number,
  sy: (x: number, y: number) => number,
) {
  const horizon = Math.max(1.0, Math.min(4.0, Math.abs(speed) * 2.5))
  const n = 40
  ctx.beginPath()
  let x = 0
  let y = 0
  let th = 0
  const ds = horizon / n
  for (let i = 0; i <= n; i++) {
    if (i === 0) ctx.moveTo(sx(x, y), sy(x, y))
    else ctx.lineTo(sx(x, y), sy(x, y))
    th += (ds / VEHICLE.wheelbase) * Math.tan(steer)
    x += ds * Math.cos(th)
    y += ds * Math.sin(th)
  }
  ctx.strokeStyle = C.path
  ctx.lineWidth = 2
  ctx.setLineDash([6, 4])
  ctx.stroke()
  ctx.setLineDash([])
}

/** 超音波の測距。**円弧ではなく「その距離に何かある」線**として描く。 */
function drawUltrasonic(
  ctx: CanvasRenderingContext2D,
  front: number | null,
  rear: number | null,
  sx: (x: number, y: number) => number,
  sy: (x: number, y: number) => number,
  px: number,
) {
  ctx.strokeStyle = C.us
  ctx.lineWidth = 2
  for (const [d, sign] of [
    [front, 1],
    [rear, -1],
  ] as const) {
    if (d == null) continue
    const x = sign * d
    const spread = 0.15 * px
    ctx.beginPath()
    ctx.moveTo(sx(x, 0) - spread, sy(x, 0))
    ctx.lineTo(sx(x, 0) + spread, sy(x, 0))
    ctx.stroke()
  }
}
