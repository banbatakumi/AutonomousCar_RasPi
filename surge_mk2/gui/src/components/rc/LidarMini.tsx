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
 */
import { useEffect, useRef } from 'react'
import { live, useNumbers } from '../../bus/live'

const RADIUS_M = 3

const BG = '#0d0b0c'
/** 近い＝危険＝赤（`--bad` と同じ意味の色） */
const NEAR = [242, 84, 75] as const
/** 遠い＝安全＝緑（`--ok` と同じ意味の色） */
const FAR = [58, 194, 106] as const
const SATURATED = '#3a3234'

function lerp(a: number, b: number, t: number): number {
  return a + (b - a) * t
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
