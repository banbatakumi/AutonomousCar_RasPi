/**
 * 速度計 — ラジコンモードの主計器。
 *
 * ## なぜ円弧なのか
 *
 * 数字だけだと「今どのくらい出ているか」が読めない。**針の角度は目を向けなくても
 * 周辺視で分かる**（`architecture.md` §10.1 の層 B）。実車のメータが円弧なのは
 * 装飾ではなく、この理由による。
 *
 * ## レッドゾーンの意味
 *
 * 巡航レンジ（`cruiseScale`）を超える範囲を赤帯にしてある。ここは
 * **Shift / パッド R1 を押している間しか入れない領域**で、レブリミットではない。
 * 「今どこまで出るか」を握る前に知りたい、という StatusBar の BOOST ピルと同じ狙い。
 *
 * ## 更新は 8Hz + CSS 補間
 *
 * 針は `useNumbers()` の 8Hz で角度を更新し、間を CSS の transition で埋める。
 * rAF で回す必要があるのは軌跡を描く G メータだけ（`GMeter.tsx`）。
 */
import { useNumbers } from '../../bus/live'
import { kmh } from '../../format'
import { useUi } from '../../store/ui'

/** 目盛りの円弧。真下を空けた 240°（実車のメータと同じ配置） */
const START_DEG = 150
const SWEEP_DEG = 240
const R = 46
const CX = 60
const CY = 58

function polar(deg: number, r: number): [number, number] {
  const rad = ((deg - 90) * Math.PI) / 180
  return [CX + r * Math.cos(rad), CY + r * Math.sin(rad)]
}

/** 0..1 の位置 → 円弧上の点。SVG の path 用 */
function arcPath(from: number, to: number, r: number): string {
  const a0 = START_DEG + SWEEP_DEG * from
  const a1 = START_DEG + SWEEP_DEG * to
  const [x0, y0] = polar(a0, r)
  const [x1, y1] = polar(a1, r)
  const large = a1 - a0 > 180 ? 1 : 0
  return `M ${x0.toFixed(2)} ${y0.toFixed(2)} A ${r} ${r} 0 ${large} 1 ${x1.toFixed(2)} ${y1.toFixed(2)}`
}

export function SpeedGauge() {
  const n = useNumbers()
  const { maxSpeed, cruiseScale, torqueMode } = useUi((s) => s.settings)
  const vs = n.vs

  // 停止判定中は 0 に落とす（`DriveBar` と同じ扱い。デッドバンド内の生値を速度に見せない）
  const speed = vs ? (vs.stopped ? 0 : Math.abs(vs.speed)) : 0
  const full = Math.max(0.1, maxSpeed)
  const frac = Math.min(1, speed / full)
  // トルクモード中は速度指令を送っていないので目標マーカーを出さない（`DriveBar` と同じ判断）
  const showTarget = n.out.active && !(torqueMode && n.out.torqueMode)
  const targetFrac = showTarget ? Math.min(1, Math.abs(n.out.speed) / full) : null

  const angle = START_DEG + SWEEP_DEG * frac
  const redFrom = Math.min(1, cruiseScale)
  const reversing = (vs?.speed ?? 0) < -0.02 && !vs?.stopped

  // 目盛りは 5 分割。**数字を振るのは端と中央だけ**（小さい円弧に数字を詰めると読めない）
  const ticks = Array.from({ length: 6 }, (_, i) => i / 5)

  return (
    <div className={`meter meter-speed ${n.stale ? 'stale' : ''}`}>
      {/*
        ⚠ viewBox を変えると G メータと中心がずれる。針の軸は (60, 58) で、
        箱の高さ 92 に対して 63% の位置にある（中央ではない）。隣の G メータは
        正方形の中心が計器の中心なので、**両者を揃える計算が `styles.css` の
        `.gmeter-wrap` の margin-top に入っている。** 片方だけ触らないこと。
      */}
      <svg viewBox="0 0 120 92" className="arc">
        <path className="arc-bg" d={arcPath(0, 1, R)} />
        {redFrom < 1 && <path className="arc-red" d={arcPath(redFrom, 1, R)} />}
        <path className="arc-fill" d={arcPath(0, Math.max(frac, 0.001), R)} />

        {ticks.map((t) => {
          const [x0, y0] = polar(START_DEG + SWEEP_DEG * t, R - 7)
          const [x1, y1] = polar(START_DEG + SWEEP_DEG * t, R - 1)
          return <line key={t} className="arc-tick" x1={x0} y1={y0} x2={x1} y2={y1} />
        })}

        {/* 指令速度。**実測との差がそのまま応答の遅れ**（`DriveBar` の BiBar と同じ狙い） */}
        {targetFrac != null && (
          <line
            className="arc-target"
            x1={polar(START_DEG + SWEEP_DEG * targetFrac, R - 9)[0]}
            y1={polar(START_DEG + SWEEP_DEG * targetFrac, R - 9)[1]}
            x2={polar(START_DEG + SWEEP_DEG * targetFrac, R + 3)[0]}
            y2={polar(START_DEG + SWEEP_DEG * targetFrac, R + 3)[1]}
          />
        )}

        <g className="needle" style={{ transform: `rotate(${angle}deg)` }}>
          <line x1={CX} y1={CY} x2={CX} y2={CY - R + 4} />
        </g>
        <circle className="needle-hub" cx={CX} cy={CY} r={3.5} />
      </svg>

      <div className="meter-read">
        <b>{kmh(speed)}</b>
        <i>km/h</i>
      </div>
      <div className="meter-sub">
        <span>{speed.toFixed(2)} m/s</span>
        {reversing && <span className="lv-warn">後退</span>}
      </div>
      <div className="meter-scale dim">
        0 — {kmh(full)} km/h
      </div>
    </div>
  )
}
