/**
 * LiDAR ミニマップ — 前方映像右上に丸く重ねる「ゲームのマップ」風表示（2026-08-17）。
 *
 * `LidarView`（自動運転ビュー・診断用）とはあえて別コンポーネントにした。
 * あちらは車体イラスト・予測進路・planner のギャップ／バブル・超音波・
 * 欠測ハッチングまで全部乗せる「解析するための地図」。こちらは運転中に
 * 一瞬だけ視線を送って「今どっちが空いているか」を掴むための道具なので、
 * **点群の色だけで距離を伝える**（遠い＝緑、近い＝赤）。車体アイコンも
 * 軌跡も描かない — 増やすほど一瞬で読めなくなる。
 *
 * 色は `--ok`/`--bad` と同じ意味（遠い＝安全＝緑、近い＝危険＝赤）だが、
 * Canvas 直描きなので CSS 変数は読めず、同じ意味の色を直接埋め込んでいる。
 *
 * 円形クリップは親側（`.rc-lidar-mini`、`styles.css`）の `border-radius: 50%`
 * に任せる。ここでは正方形いっぱいに描くだけでよい。
 *
 * ズームは固定（`RADIUS_M`）。前方映像の隅に置く小さな丸にコントロール UI を
 * 足す余地がないので、常時 3m 圏を映す。詳しく見たいときは診断タブ／
 * 自動運転ビューの `LidarView`（ズーム可）を使う。
 *
 * ## 極座標グリッド（2026-08-20）
 *
 * `LidarView` と同じ「1m 間隔の同心円＋十字」を薄く重ねる。線が無いと
 * 縮小時は距離感がまるで掴めず、点の並びだけでは「今どれくらい近いか」が
 * 読めなかったため。**ただし数字（`1m` 等のラベル）は縮小時には出さない**
 * ——小さな丸に文字を詰めると点群そのものが読めなくなる。拡大時
 * （`ui.lidarExpanded`）だけ `LidarView` と同じ流儀でラベルを出す。
 * この値は rAF ループの中で `useUi.getState()` を毎フレーム直接読む
 * （`live.scan` と同じ理由——React の再レンダーを待たずに反映するため）。
 */
import { useEffect, useRef } from 'react'
import { live, useNumbers } from '../../bus/live'
import { useUi } from '../../store/ui'

const RADIUS_M = 3
const GRID_STEP_M = 1

const BG = '#0d0b0c'
/** 近い＝危険＝赤（`--bad` と同じ意味の色） */
const NEAR = [242, 84, 75] as const
/** 遠い＝安全＝緑（`--ok` と同じ意味の色） */
const FAR = [58, 194, 106] as const
const SATURATED = '#3a3234'

function lerp(a: number, b: number, t: number): number {
  return a + (b - a) * t
}

/** 1m 間隔の同心円＋十字（`LidarView.tsx` の `drawGrid` と同じ流儀）。
 * ラベルは拡大表示中だけ出す（縮小時は文字が点群と重なって読めなくなる） */
function drawGrid(
  ctx: CanvasRenderingContext2D,
  w: number,
  h: number,
  cx: number,
  cy: number,
  px: number,
  expanded: boolean,
) {
  ctx.strokeStyle = 'rgba(255,255,255,0.12)'
  ctx.lineWidth = 1
  for (let r = GRID_STEP_M; r <= RADIUS_M; r += GRID_STEP_M) {
    ctx.beginPath()
    ctx.arc(cx, cy, r * px, 0, Math.PI * 2)
    ctx.stroke()
    if (expanded) {
      ctx.fillStyle = 'rgba(255,255,255,0.5)'
      ctx.font = '9px ui-monospace, monospace'
      ctx.fillText(`${r}m`, cx + 3, cy - r * px - 2)
    }
  }
  // 正面方向・左右の十字
  ctx.beginPath()
  ctx.moveTo(cx, 0)
  ctx.lineTo(cx, h)
  ctx.moveTo(0, cy)
  ctx.lineTo(w, cy)
  ctx.stroke()
}

export function LidarMini() {
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
      if (!w || !h) return
      if (cv.width !== w * dpr || cv.height !== h * dpr) {
        cv.width = w * dpr
        cv.height = h * dpr
      }
      ctx.setTransform(dpr, 0, 0, dpr, 0, 0)
      ctx.fillStyle = BG
      ctx.fillRect(0, 0, w, h)

      const cx = w / 2
      const cy = h / 2
      const px = Math.min(w, h) / 2 / RADIUS_M

      drawGrid(ctx, w, h, cx, cy, px, useUi.getState().lidarExpanded)

      const scan = live.scan
      if (scan) {
        for (let deg = 0; deg < 360; deg++) {
          const d = scan.dist[deg]
          if (!d) continue
          const a = (deg * Math.PI) / 180
          const x = d * Math.cos(a)
          const y = d * Math.sin(a)
          // 車両座標 (x=前, y=左) → 画面。上を前方にする（画面角 = -(θ+90°)）
          const sx = cx - y * px
          const sy = cy - x * px
          if (sx < -2 || sx > w + 2 || sy < -2 || sy > h + 2) continue
          if (scan.saturated?.[deg]) {
            ctx.fillStyle = SATURATED
            ctx.fillRect(sx - 0.5, sy - 0.5, 1, 1)
          } else {
            const t = Math.max(0, Math.min(1, d / RADIUS_M))
            const r = Math.round(lerp(NEAR[0], FAR[0], t))
            const g = Math.round(lerp(NEAR[1], FAR[1], t))
            const b = Math.round(lerp(NEAR[2], FAR[2], t))
            ctx.fillStyle = `rgb(${r},${g},${b})`
            ctx.fillRect(sx - 1.5, sy - 1.5, 3, 3)
          }
        }
      }
    }
    raf = requestAnimationFrame(draw)
    return () => cancelAnimationFrame(raf)
  }, [])

  return (
    <div className="lidar-mini">
      <canvas ref={ref} className="lidar-mini-canvas" />
      <LidarMiniBadge />
    </div>
  )
}

/** 点群が死んでいるときだけ、丸の中央に小さく出す。**それ以外は何も出さない** */
function LidarMiniBadge() {
  const n = useNumbers()
  if (!(n.scanAgeMs > 400)) return null
  return <div className="lidar-mini-ng">{isFinite(n.scanAgeMs) ? 'LiDAR 遅延' : 'LiDAR NG'}</div>
}
