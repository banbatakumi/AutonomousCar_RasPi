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
import type { PointerEvent as ReactPointerEvent } from 'react'
import { useEffect, useRef } from 'react'
import { live } from '../bus/live'
import { useUi } from '../store/ui'
import { VEHICLE as VEHICLE_GEOM } from '../generated/vehicle'
import type { ControlChannel } from '../ws/control'
import type { AutoState } from '../generated/msgs'

/** 車体の描画用寸法 [m]。原点は base_link ＝ 後輪車軸の中心。
 * `config/vehicle.toml`（唯一の正）から生成された `footprint` ポリゴンの
 * バウンディングボックスとして算出する。手で数値を書かない。 */
const VEHICLE = {
  wheelbase: VEHICLE_GEOM.wheelbase,
  front: Math.max(...VEHICLE_GEOM.footprint.map(([x]) => x)), // 原点より前に出ている量
  back: -Math.min(...VEHICLE_GEOM.footprint.map(([x]) => x)), // 原点より後ろに出ている量
  width: 2 * Math.max(...VEHICLE_GEOM.footprint.map(([, y]) => Math.abs(y))),
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
  parkTarget: '#ff9f43',
  parkPathFwd: '#4ad6ff',
  parkPathRev: '#ff6b9d',
}

export function LidarView({ ch }: { ch?: ControlChannel | null } = {}) {
  const ref = useRef<HTMLCanvasElement>(null)
  const zoom = useUi((s) => s.lidarZoom)
  const auto = useUi((s) => s.auto)
  //: 駐車目標のクリック+ドラッグは `park_to_point` 選択中だけ有効
  // （`CameraView.tsx` の `trackingMode` と同じ ref 経由の方針）
  const parkingMode = auto?.mode === 'park_to_point'
  const parkingModeRef = useRef(parkingMode)
  parkingModeRef.current = parkingMode
  const chRef = useRef(ch)
  chRef.current = ch
  //: 直近の `draw()` が計算した画面変換（CSS px 基準）。ポインタ座標 →
  //: 車両ローカル座標[m] の逆変換に使う（`CameraView.tsx` の `imgBoxRef` と同じ方針）
  const xformRef = useRef({ cx: 0, cy: 0, px: 1 })
  //: ドラッグ中の選択（車両ローカル座標[m]）。React state を介さず
  //: `draw()` が rAF で直接参照する（`CameraView.tsx` の `dragRef` と同じ方針）
  const dragRef = useRef<{ x0: number; y0: number; x1: number; y1: number } | null>(null)

  const toLocal = (cv: HTMLCanvasElement, clientX: number, clientY: number) => {
    const rect = cv.getBoundingClientRect()
    const { cx, cy, px } = xformRef.current
    const sx = clientX - rect.left
    const sy = clientY - rect.top
    // `draw()` の順変換 `sx = cx - y*px, sy = cy - x*px` の逆
    return { x: (cy - sy) / px, y: (cx - sx) / px }
  }

  const onParkPointerDown = (e: ReactPointerEvent<HTMLCanvasElement>) => {
    if (!parkingModeRef.current) return
    const p = toLocal(e.currentTarget, e.clientX, e.clientY)
    e.currentTarget.setPointerCapture(e.pointerId)
    dragRef.current = { x0: p.x, y0: p.y, x1: p.x, y1: p.y }
  }
  const onParkPointerMove = (e: ReactPointerEvent<HTMLCanvasElement>) => {
    const drag = dragRef.current
    if (!drag) return
    const p = toLocal(e.currentTarget, e.clientX, e.clientY)
    dragRef.current = { ...drag, x1: p.x, y1: p.y }
  }
  const onParkPointerUp = () => {
    const drag = dragRef.current
    dragRef.current = null
    if (!drag) return
    // ドラッグ量が小さい（クリックだけ・手ぶれ）なら「今の向きのまま」（yaw=0）
    const yaw = Math.hypot(drag.x1 - drag.x0, drag.y1 - drag.y0) < 0.12
      ? 0
      : Math.atan2(drag.y1 - drag.y0, drag.x1 - drag.x0)
    chRef.current?.setParkTarget(drag.x0, drag.y0, yaw)
  }

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
      xformRef.current = { cx, cy, px }

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

      if (parkingModeRef.current) {
        //: 計画した経路。**ドラッグ中でも描く**（今の経路と新しい目標を
        //: 見比べたいので）
        if (live.auto?.park_active) drawParkPath(ctx, sx, sy, live.auto)
        if (dragRef.current) {
          drawParkArrow(ctx, sx, sy, dragRef.current.x0, dragRef.current.y0,
            dragRef.current.x1, dragRef.current.y1, `${C.parkTarget}99`)
        } else if (live.auto?.park_active) {
          const a = live.auto
          const tx = a.park_target_x + Math.cos(a.park_target_yaw) * 0.25
          const ty = a.park_target_y + Math.sin(a.park_target_yaw) * 0.25
          drawParkArrow(ctx, sx, sy, a.park_target_x, a.park_target_y, tx, ty, C.parkTarget)
        }
      }
    }
    raf = requestAnimationFrame(draw)
    return () => cancelAnimationFrame(raf)
  }, [zoom])

  return (
    <canvas
      ref={ref}
      className="lidar-canvas"
      onPointerDown={onParkPointerDown}
      onPointerMove={onParkPointerMove}
      onPointerUp={onParkPointerUp}
      onPointerCancel={() => {
        dragRef.current = null
      }}
    />
  )
}

/**
 * 計画した経路を折れ線で描く。**前進区間と後退区間を色で分ける。**
 *
 * `AutoState.park_path_*` は車両基準ローカル座標（点群と同じ系）で来るので、
 * 画面変換をそのまま掛けられる。★これが無いと実機で「なぜこの経路を
 * 選んだか」が読めない——障害物回避が効いているのか偶然なのかが画面から
 * 判断できず、デバッグ手段が無くなる。
 */
function drawParkPath(
  ctx: CanvasRenderingContext2D,
  sx: (x: number, y: number) => number,
  sy: (x: number, y: number) => number,
  a: AutoState,
) {
  const n = Math.min(a.park_path_x.length, a.park_path_y.length)
  if (n < 2) return
  const revFrom = a.park_path_reverse_from
  ctx.save()
  ctx.lineWidth = 2.5
  ctx.lineJoin = 'round'
  for (let i = 1; i < n; i++) {
    const x0 = a.park_path_x[i - 1] ?? 0
    const y0 = a.park_path_y[i - 1] ?? 0
    const x1 = a.park_path_x[i] ?? 0
    const y1 = a.park_path_y[i] ?? 0
    const reverse = revFrom >= 0 && i >= revFrom
    ctx.strokeStyle = reverse ? C.parkPathRev : C.parkPathFwd
    ctx.beginPath()
    ctx.moveTo(sx(x0, y0), sy(x0, y0))
    ctx.lineTo(sx(x1, y1), sy(x1, y1))
    ctx.stroke()
  }
  ctx.restore()
}

/**
 * 駐車目標（位置+向き）を十字＋矢印で描く。`(x0,y0)` が位置、`(x0,y0)→(x1,y1)`
 * が向き。ドラッグ中はプレビュー、確定後は `AutoState.park_target_*`
 * （毎周期デッドレコニングで更新される値）を描く——クリック時点のローカル
 * 座標を固定描画すると車が動いた瞬間に画面上で嘘になるため。
 */
function drawParkArrow(
  ctx: CanvasRenderingContext2D,
  sx: (x: number, y: number) => number,
  sy: (x: number, y: number) => number,
  x0: number,
  y0: number,
  x1: number,
  y1: number,
  color: string,
) {
  const px0 = sx(x0, y0)
  const py0 = sy(x0, y0)
  const px1 = sx(x1, y1)
  const py1 = sy(x1, y1)
  ctx.save()
  ctx.strokeStyle = color
  ctx.fillStyle = color
  ctx.lineWidth = 2
  // 十字（位置）
  ctx.beginPath()
  ctx.moveTo(px0 - 6, py0)
  ctx.lineTo(px0 + 6, py0)
  ctx.moveTo(px0, py0 - 6)
  ctx.lineTo(px0, py0 + 6)
  ctx.stroke()
  // 矢印（向き）
  ctx.beginPath()
  ctx.moveTo(px0, py0)
  ctx.lineTo(px1, py1)
  ctx.stroke()
  const ang = Math.atan2(py1 - py0, px1 - px0)
  const head = 6
  ctx.beginPath()
  ctx.moveTo(px1, py1)
  ctx.lineTo(px1 - head * Math.cos(ang - Math.PI / 6), py1 - head * Math.sin(ang - Math.PI / 6))
  ctx.lineTo(px1 - head * Math.cos(ang + Math.PI / 6), py1 - head * Math.sin(ang + Math.PI / 6))
  ctx.closePath()
  ctx.fill()
  ctx.restore()
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
  const front = VEHICLE.front * px
  const back = VEHICLE.back * px
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
