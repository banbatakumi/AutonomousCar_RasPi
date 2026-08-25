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
import { kmh, mps } from '../../format'
import { PI_MAX_SPEED_CAP, useUi } from '../../store/ui'

/**
 * 目盛りの円弧。**真上（0°）を直進側の基準に、-120°〜120°の240°**
 * （2026-08-17、指示により明示的なパラメータへ変更。sweep 量は従来と同じ240°で、
 * 真下120°分が空く＝実車のメータと同じ配置）。`polar()` は時計回りが正なので、
 * 0 が真上、-120 が左下寄り（8時方向）、120 が右下寄り（4時方向）になる。
 */
const MIN_DEG = -120
const MAX_DEG = 120
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

/** 0..1 の位置 → 円弧上の点。SVG の path 用 */
function arcPath(from: number, to: number, r: number): string {
  const a0 = MIN_DEG + SWEEP_DEG * from
  const a1 = MIN_DEG + SWEEP_DEG * to
  const [x0, y0] = polar(a0, r)
  const [x1, y1] = polar(a1, r)
  const large = a1 - a0 > 180 ? 1 : 0
  return `M ${x0.toFixed(2)} ${y0.toFixed(2)} A ${r} ${r} 0 ${large} 1 ${x1.toFixed(2)} ${y1.toFixed(2)}`
}

export function SpeedGauge({ dialHeight }: { dialHeight: number | null }) {
  const n = useNumbers()
  const { cruiseScale, torqueMode, speedUnit } = useUi((s) => s.settings)
  const setSettings = useUi((s) => s.setSettings)
  const vs = n.vs

  // 停止判定中は 0 に落とす（`DriveBar` と同じ扱い。デッドバンド内の生値を速度に見せない）
  const speed = vs ? (vs.stopped ? 0 : Math.abs(vs.speed)) : 0
  // ★2026-08-22: 目盛りは常に STM32 実測の物理上限（`LIMITS.max_speed_m_s`）に固定する。
  // 速度制御・トルク制御どちらでも同じ扱い。以前は RC の速度ダイヤル（`settings.maxSpeed`）
  // を目盛りにも流用していたため、ダイヤルを下げるとメータの目盛りまで一緒に縮み、
  // 「実際に出せる速度」とメータの見え方が一致しなくなっていた（トルク制御は特に、
  // 速度指令自体を送らないのでダイヤルとメータを結びつける理由が無い）。
  // `LIMITS` 未受信の間は `PI_MAX_SPEED_CAP` にフォールバックする
  const full = Math.max(0.1, n.link?.max_speed_m_s ?? PI_MAX_SPEED_CAP)
  const frac = Math.min(1, speed / full)
  // トルクモード中は速度指令を送っていないので目標マーカーを出さない（`DriveBar` と同じ判断）
  const showTarget = n.out.active && !(torqueMode && n.out.torqueMode)
  const targetFrac = showTarget ? Math.min(1, Math.abs(n.out.speed) / full) : null

  const angle = MIN_DEG + SWEEP_DEG * frac
  const redFrom = Math.min(1, cruiseScale)
  const reversing = (vs?.speed ?? 0) < -0.02 && !vs?.stopped

  // 目盛りは 5 分割。**数字を振るのは端と中央だけ**（小さい円弧に数字を詰めると読めない）
  const ticks = Array.from({ length: 6 }, (_, i) => i / 5)

  return (
    <div className={`meter meter-speed ${n.stale ? 'stale' : ''}`}>
      {/*
        2026-08-17: 映像下のメータ行が可変高さになったため（`RcView.tsx` の
        `useElementSize`）、G メータとの光学中心を揃える固定 margin-top の
        ハックは廃止した。3つのメータは上端揃え（top-align）に割り切っている。

        `dialHeight` は `RcView.tsx` がメータ行の実測高さから逆算して渡す px。
        **CSS の `flex`/`aspect-ratio` だけに任せなかった理由**——`<svg>` は
        replaced element なので、intrinsic サイズ（既定 300×150 相当）に負けて
        縮んだままになる挙動を実機で確認したため（`hooks/useContainFit.ts`
        冒頭のコメント）。`dialHeight` が来るまでは CSS 側のフォールバック
        （`styles.css` の `.dial-wrap`）に任せる。
      */}
      <div
        className="dial-wrap"
        style={dialHeight ? { width: dialHeight * DIAL_AR, height: dialHeight } : undefined}
      >
        <svg viewBox="0 0 120 92" className="arc">
          <path className="arc-bg" d={arcPath(0, 1, R)} />
          {redFrom < 1 && <path className="arc-red" d={arcPath(redFrom, 1, R)} />}
          <path className="arc-fill" d={arcPath(0, Math.max(frac, 0.001), R)} />

          {ticks.map((t) => {
            const [x0, y0] = polar(MIN_DEG + SWEEP_DEG * t, R - 7)
            const [x1, y1] = polar(MIN_DEG + SWEEP_DEG * t, R - 1)
            return <line key={t} className="arc-tick" x1={x0} y1={y0} x2={x1} y2={y1} />
          })}

          {/* 指令速度。**実測との差がそのまま応答の遅れ**（`DriveBar` の BiBar と同じ狙い） */}
          {targetFrac != null && (
            <line
              className="arc-target"
              x1={polar(MIN_DEG + SWEEP_DEG * targetFrac, R - 9)[0]}
              y1={polar(MIN_DEG + SWEEP_DEG * targetFrac, R - 9)[1]}
              x2={polar(MIN_DEG + SWEEP_DEG * targetFrac, R + 3)[0]}
              y2={polar(MIN_DEG + SWEEP_DEG * targetFrac, R + 3)[1]}
            />
          )}

          <g className="needle" style={{ transform: `rotate(${angle}deg)` }}>
            <line x1={CX} y1={CY} x2={CX} y2={CY - R + 4} />
          </g>
          <circle className="needle-hub" cx={CX} cy={CY} r={3.5} />
        </svg>
      </div>

      <div
        className="meter-read"
        onClick={() => setSettings({ speedUnit: speedUnit === 'kmh' ? 'ms' : 'kmh' })}
        title="クリックで m/s ⇔ km/h を切替"
      >
        <b>{speedUnit === 'kmh' ? kmh(speed) : mps(speed)}</b>
        <i>{speedUnit === 'kmh' ? 'km/h' : 'm/s'}</i>
      </div>
      {/* 正常時の数値（m/s・レンジ）は出さない。**後退だけは異常系として残す** */}
      {reversing && (
        <div className="meter-sub">
          <span className="lv-warn">後退</span>
        </div>
      )}
    </div>
  )
}
