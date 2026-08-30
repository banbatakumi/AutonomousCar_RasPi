/**
 * 速度計 — ラジコンモードの主計器。
 *
 * ## なぜ円弧なのか
 *
 * 数字だけだと「今どのくらい出ているか」が読めない。**針の角度は目を向けなくても
 * 周辺視で分かる**（`architecture.md` §10.1 の層 B）。実車のメータが円弧なのは
 * 装飾ではなく、この理由による。
 *
 * ## 更新は 8Hz + CSS 補間
 *
 * 針は `useNumbers()` の 8Hz で角度を更新し、間を CSS の transition で埋める。
 * rAF で回す必要があるのは軌跡を描く G メータだけ（`GMeter.tsx`）。
 */
import { useNumbers } from '../../bus/live'
import { kmh, mps } from '../../format'
import { mtGearRatio, PI_MAX_SPEED_CAP, useUi } from '../../store/ui'

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
/** MT モードのレッドライン帯の幅（弧全体に対する割合）。ギア上限のこの手前から色が付く */
const REDLINE_BAND = 0.08

/**
 * 針（v0.18、指示により変更）。**中心から伸びる長い針ではなく、目盛りの
 * 半径よりわずかに内側から目盛りの外周までの短い針にする。** 中心（`needle-hub`）
 * とは切り離し、目盛り（`R-7`〜`R-1`）に寄り添う短いマーカーだけを回転させる——
 * 今どきのデジタルダッシュに近い見た目にする狙い
 */
const NEEDLE_R_OUTER = R - 1 // 目盛りの外周と同じ
const NEEDLE_R_INNER = R - 16 // 目盛りの直径よりわずかに小さい位置から始める

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
  const settings = useUi((s) => s.settings)
  const { driveMode, speedUnit } = settings
  const torqueMode = driveMode === 'torque'
  const setSettings = useUi((s) => s.setSettings)
  // レンジ/ギア表示用（v0.18）。速度制御・トルク制御中は `gear`（D/R）、
  // MT モード中は `mtGear`（R, N, D1〜D5）——`useDriving.ts` の `shiftGear()` と同じ store
  const gear = useUi((s) => s.gear)
  const mtGear = useUi((s) => s.mtGear)
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
  const reversing = (vs?.speed ?? 0) < -0.02 && !vs?.stopped

  // 目盛りは 5 分割。**数字を振るのは端と中央だけ**（小さい円弧に数字を詰めると読めない）
  const ticks = Array.from({ length: 6 }, (_, i) => i / 5)

  // ── ギア/レンジ表示（v0.18） ──────────────────────────────────────
  //
  // ダイヤル中央下、240°スイープの下120°が空いている場所に常時表示する
  // （弧・目盛り・指令マーカーのどれとも幾何学的に重ならない）。
  // 'D1'〜'D5' は '1'〜'5' に縮める。'R'/'N' はそのまま（`.slice(1)` すると
  // 'N' が空文字になってしまうので、1文字ギアは縮めない扱いに含める）
  const mtMode = driveMode === 'mt'
  const gearLabel = mtMode ? (mtGear === 'R' || mtGear === 'N' ? mtGear : mtGear.slice(1)) : gear
  const gearIsReverse = mtMode ? mtGear === 'R' : gear === 'R'

  // ── MT モードのレッドライン（v0.18） ────────────────────────────────
  //
  // 「このギアの上限（`maxSpeed × mtGearRatio`）が近い＝そろそろシフトアップ」を、
  // 現在速度に関わらず常時見える静的な帯として弧の外側（r=52）に描く。
  // R では上限が変わらず（`mtGearRatio('R', …)` は D1 と同じ）、N は上限そのものが
  // 0（`mtGearRatio('N', …)`）、D5 はこれ以上上のギアが無いので、
  // どれも「シフトアップを促す」意味が成立せず出さない
  const redlineFrac =
    mtMode && mtGear !== 'R' && mtGear !== 'N' && mtGear !== 'D5'
      ? Math.min(1, Math.max(0, (settings.maxSpeed * mtGearRatio(mtGear, settings)) / full))
      : null
  const redlineFrom = redlineFrac != null ? Math.max(0, redlineFrac - REDLINE_BAND) : null

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

          {/* MT のレッドライン。本体の弧（r=46）と同じ半径だと `--accent` の赤に
              埋もれるので、外側の別半径（r=52）に描く */}
          {redlineFrac != null && redlineFrom != null && redlineFrac > 0 && (
            <path
              className={`arc-redline ${frac >= redlineFrom ? 'hot' : ''}`}
              d={arcPath(redlineFrom, redlineFrac, 52)}
            />
          )}

          <g className="needle" style={{ transform: `rotate(${angle}deg)` }}>
            <line x1={CX} y1={CY - NEEDLE_R_INNER} x2={CX} y2={CY - NEEDLE_R_OUTER} />
          </g>
          <circle className="needle-hub" cx={CX} cy={CY} r={3.5} />

          {/* ギア/レンジ。運転者の選択状態であって車両テレメトリではないので、
              未接続・stale でも常に表示する（`DriveControls.tsx` のボタンと同じ扱い） */}
          <rect className="gear-chip" x={47} y={65} width={26} height={22} rx={4} />
          <text
            className={`gear-label ${gearIsReverse ? 'rev' : ''}`}
            x={CX}
            y={76}
            textAnchor="middle"
            dominantBaseline="central"
          >
            {gearLabel}
          </text>
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
