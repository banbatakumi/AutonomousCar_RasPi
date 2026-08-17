/**
 * G メータ — 前後G / 横G の散布と軌跡。
 *
 * ## なぜ Canvas + rAF なのか
 *
 * 軌跡（残像）を出すので、`useNumbers()` の 8Hz では点が飛んでカクつく。
 * `live.vs` を rAF で直読する層1の描き方に寄せる（`bus/live.ts` 冒頭、`LidarView.tsx`）。
 * **React の state は一切使わない。**
 *
 * ## 軸の向きは実機で確認すること
 *
 * `accel[0]=x` を前後、`accel[1]=y` を左右として扱っているが、**符号は IMU の
 * 取り付け向きに依存する**。実車で
 *
 *   - 前進加速で点が「上」に振れるか → 違えば `AX_SIGN` を −1
 *   - 右旋回で点が「右」に振れるか   → 違えば `AY_SIGN` を −1
 *
 * を確認して、この2定数だけ直す。描画側には手を入れないで済むようにしてある。
 *
 * ## 重力の扱い
 *
 * IMU の生値には重力が乗る。平坦路では x/y 成分にはほぼ出ないが、坂では
 * `pitch`/`roll` のぶんだけ傾く。**補正はしていない**（見て楽しむための計器であり、
 * 制御には使わないため）。坂で中心がずれるのは仕様。
 *
 * ## 色（2026-08-17、GR 風に変更）
 *
 * 軌跡は `.rc` のスコープ変数 `--accent`（赤）と同じ色を直書きしている
 * （Canvas は CSS 変数を読めないため）。**上限超過の点だけ従来どおり目立つ色を
 * 別に割り当てる**——軌跡自体が赤になったので、そのままだと超過の合図が
 * 埋もれる。ここだけ白にして「振り切れた」を伝える。
 *
 * ## 最大 G の表示は無くした（2026-08-17、指示による）
 *
 * 走行中に見るものではないと判断し、ピーク保持のロジックごと削除した。
 * 「今どれだけ G が出ているか」は点の位置そのもので分かる。
 *
 * ## `dialHeight`（2026-08-17）
 *
 * `RcView.tsx` がメータ行の実測高さから逆算して渡す px。速度計・舵角計と
 * サイズを揃えるために使う（`SpeedGauge.tsx` 冒頭のコメント参照）。描画ループ
 * 自体は元々 `getBoundingClientRect()` で毎フレーム実寸を読んでいるので、
 * ここでは `.gmeter-wrap` の CSS サイズを変えるだけで追従する。
 */
import { useEffect, useRef } from 'react'
import { live } from '../../bus/live'
import { gForce } from '../../format'

/** 実機で合わせる符号。前進加速 → 上、右旋回 → 右 になるように */
const AX_SIGN = 1
const AY_SIGN = 1

/** 外周が何 G か。ミニカーの実力に対して広すぎると点が真ん中で動かない */
const FULL_G = 0.6
/** 軌跡の長さ [ms]。長すぎると団子になる */
const TRAIL_MS = 2000
const TRAIL_MAX = 120

type P = { x: number; y: number; t: number }

export function GMeter({ dialHeight }: { dialHeight: number | null }) {
  const ref = useRef<HTMLCanvasElement>(null)

  useEffect(() => {
    const cv = ref.current
    if (!cv) return
    const ctx = cv.getContext('2d')
    if (!ctx) return

    const trail: P[] = []
    let raf = 0

    const draw = () => {
      raf = requestAnimationFrame(draw)

      const rect = cv.getBoundingClientRect()
      const dpr = Math.min(2, window.devicePixelRatio || 1)
      const w = Math.max(1, Math.round(rect.width * dpr))
      const h = Math.max(1, Math.round(rect.height * dpr))
      if (cv.width !== w || cv.height !== h) {
        cv.width = w
        cv.height = h
      }

      const now = performance.now()
      const vs = live.vs
      const fresh = vs != null && now - live.lastRxMs < 500

      if (fresh && vs) {
        const gx = gForce(AY_SIGN * vs.accel[1]) // 横G（右が正）
        const gy = gForce(AX_SIGN * vs.accel[0]) // 前後G（加速が正）
        trail.push({ x: gx, y: gy, t: now })
      }
      while (trail.length && now - trail[0].t > TRAIL_MS) trail.shift()
      while (trail.length > TRAIL_MAX) trail.shift()

      const size = Math.min(w, h)
      const cx = w / 2
      const cy = h / 2
      const r = size / 2 - 4 * dpr

      ctx.clearRect(0, 0, w, h)
      // レイアウト確定前など、箱がまだ豆粒サイズのフレームがある。
      // `r<=0` で `ctx.arc()` に渡すと例外になるので、その回は素通りする
      if (r <= 0) return

      // ── 目盛り（同心円と十字） ──
      ctx.lineWidth = dpr
      for (const ring of [1 / 3, 2 / 3, 1]) {
        ctx.beginPath()
        ctx.arc(cx, cy, r * ring, 0, Math.PI * 2)
        ctx.strokeStyle = ring === 1 ? '#3a2226' : '#241b1d'
        ctx.stroke()
      }
      ctx.strokeStyle = '#241b1d'
      ctx.beginPath()
      ctx.moveTo(cx - r, cy)
      ctx.lineTo(cx + r, cy)
      ctx.moveTo(cx, cy - r)
      ctx.lineTo(cx, cy + r)
      ctx.stroke()

      // ── 軌跡。**古いほど薄く**して「今どこにいるか」を見失わないようにする ──
      const px = (g: number) => cx + (g / FULL_G) * r
      const py = (g: number) => cy - (g / FULL_G) * r
      for (let i = 1; i < trail.length; i++) {
        const age = (now - trail[i].t) / TRAIL_MS
        ctx.globalAlpha = Math.max(0, 0.55 * (1 - age))
        ctx.strokeStyle = '#d5303f'
        ctx.lineWidth = 1.6 * dpr
        ctx.beginPath()
        ctx.moveTo(px(trail[i - 1].x), py(trail[i - 1].y))
        ctx.lineTo(px(trail[i].x), py(trail[i].y))
        ctx.stroke()
      }
      ctx.globalAlpha = 1

      // ── 現在値 ──
      const cur = trail[trail.length - 1]
      if (cur && fresh) {
        const mag = Math.hypot(cur.x, cur.y)
        ctx.fillStyle = mag > FULL_G ? '#ffffff' : '#ffc63f'
        ctx.beginPath()
        ctx.arc(px(cur.x), py(cur.y), 3.5 * dpr, 0, Math.PI * 2)
        ctx.fill()
      }
    }
    raf = requestAnimationFrame(draw)
    return () => cancelAnimationFrame(raf)
  }, [])

  return (
    <div className="meter meter-g">
      <div className="gmeter-wrap" style={dialHeight ? { width: dialHeight, height: dialHeight } : undefined}>
        <canvas ref={ref} className="gmeter" />
        <span className="gmeter-axis gmeter-axis-t">加速</span>
        <span className="gmeter-axis gmeter-axis-b">制動</span>
        <span className="gmeter-axis gmeter-axis-l">左</span>
        <span className="gmeter-axis gmeter-axis-r">右</span>
      </div>
    </div>
  )
}
