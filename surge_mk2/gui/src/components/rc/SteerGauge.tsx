/**
 * 舵角メータ — 速度計の左に置く（2026-08-17、指示による追加）。
 *
 * `SpeedGauge.tsx` と同じ円弧の作法（真上=0°、時計回りが正）を使うが、
 * こちらは中央（真上＝直進）から左右対称に振れるレンジ。速度計の240°より
 * 狭い140°にしてある——舵角のレンジは速度ほど広くする理由がなく、狭い方が
 * 同じ操作量でも針の動きが大きく見えて読み取りやすい。
 *
 * ## 左右の符号
 *
 * `steer_actual` は反時計回りが正（`architecture.md` §5.1、車体を上から見て
 * 正なら前輪は左を向く）。円弧メータの角度系は時計回りが正（`polar()`）なので、
 * そのまま描くと「左に切ったのに針が右に振れる」になる。**符号を反転させて、
 * 左に切ったら針も画面の左に振れるようにしてある**——`DriveBar` が舵角の
 * `BiBar` に `flip` を渡しているのと同じ理由・同じ向き。
 *
 * 実測（針・塗り）と指令エコー（マーカー）を重ねるのも `SpeedGauge` と同じ
 * 狙い。**実測との差がそのままアクチュエータの遅れ**。
 */
import { useNumbers } from '../../bus/live'
import { deg } from '../../format'
import { useUi } from '../../store/ui'

/** 中心（真上=直進）から ±70°、計140°。0.5 が真上＝直進に対応する */
const MIN_DEG = -70
const MAX_DEG = 70
const SWEEP_DEG = MAX_DEG - MIN_DEG
const R = 46
const CX = 60
const CY = 58
/** viewBox `0 0 120 92` の幅/高さ比。`dialHeight` から幅を逆算するのに使う */
const DIAL_AR = 120 / 92

function polar(deg: number, r: number): [number, number] {
  const rad = ((deg - 90) * Math.PI) / 180
  return [CX + r * Math.cos(rad), CY + r * Math.sin(rad)]
}

/** 0..1 の位置 → 円弧上の点。`from <= to` で呼ぶこと（sweep-flag を固定にしているため） */
function arcPath(from: number, to: number, r: number): string {
  const a0 = MIN_DEG + SWEEP_DEG * from
  const a1 = MIN_DEG + SWEEP_DEG * to
  const [x0, y0] = polar(a0, r)
  const [x1, y1] = polar(a1, r)
  const large = a1 - a0 > 180 ? 1 : 0
  return `M ${x0.toFixed(2)} ${y0.toFixed(2)} A ${r} ${r} 0 ${large} 1 ${x1.toFixed(2)} ${y1.toFixed(2)}`
}

export function SteerGauge({ dialHeight }: { dialHeight: number | null }) {
  const n = useNumbers()
  const { maxSteer } = useUi((s) => s.settings)
  const vs = n.vs

  const full = Math.max(0.05, maxSteer)
  const clamp = (v: number) => Math.max(-1, Math.min(1, v / full))
  // 左（CCW=正）を frac の小さい側（MIN_DEG=左）に対応させる（符号反転）
  const actualFrac = vs ? 0.5 - clamp(vs.steer_actual) / 2 : 0.5
  const cmdFrac = vs ? 0.5 - clamp(vs.steer_cmd_echo) / 2 : null

  const angle = MIN_DEG + SWEEP_DEG * actualFrac
  const fillFrom = Math.min(0.5, actualFrac)
  const fillTo = Math.max(0.5, actualFrac)

  // 目盛りは中央(直進)＋左右端の3つだけ。**狭いレンジに詰めすぎない**
  const ticks = [0, 0.5, 1]

  return (
    <div className={`meter meter-steer ${n.stale ? 'stale' : ''}`}>
      {/* `dialHeight` は `RcView.tsx` が実測から逆算して渡す px（`SpeedGauge.tsx`
          冒頭のコメント参照・同じ理由）。来るまでは CSS 側のフォールバックに任せる */}
      <div
        className="dial-wrap"
        style={dialHeight ? { width: dialHeight * DIAL_AR, height: dialHeight } : undefined}
      >
        <svg viewBox="0 0 120 92" className="arc">
          <path className="arc-bg" d={arcPath(0, 1, R)} />
          {/* 中央（直進）から実測角まで塗る。BiBar の「中央0の両振れ」と同じ考え方 */}
          <path className="arc-fill" d={arcPath(fillFrom, Math.max(fillTo, fillFrom + 0.001), R)} />

          {ticks.map((t) => {
            const [x0, y0] = polar(MIN_DEG + SWEEP_DEG * t, R - 7)
            const [x1, y1] = polar(MIN_DEG + SWEEP_DEG * t, R - 1)
            return (
              <line key={t} className={t === 0.5 ? 'arc-zero' : 'arc-tick'} x1={x0} y1={y0} x2={x1} y2={y1} />
            )
          })}

          {/* 指令舵角。**実測との差がそのままステアリングの遅れ**（`SpeedGauge` と同じ狙い） */}
          {cmdFrac != null && (
            <line
              className="arc-target"
              x1={polar(MIN_DEG + SWEEP_DEG * cmdFrac, R - 9)[0]}
              y1={polar(MIN_DEG + SWEEP_DEG * cmdFrac, R - 9)[1]}
              x2={polar(MIN_DEG + SWEEP_DEG * cmdFrac, R + 3)[0]}
              y2={polar(MIN_DEG + SWEEP_DEG * cmdFrac, R + 3)[1]}
            />
          )}

          <g className="needle" style={{ transform: `rotate(${angle}deg)` }}>
            <line x1={CX} y1={CY} x2={CX} y2={CY - R + 4} />
          </g>
          <circle className="needle-hub" cx={CX} cy={CY} r={3.5} />
        </svg>
      </div>

      <div className="meter-read">
        <b>{vs ? deg(vs.steer_actual, 0) : '—'}</b>
      </div>
    </div>
  )
}
