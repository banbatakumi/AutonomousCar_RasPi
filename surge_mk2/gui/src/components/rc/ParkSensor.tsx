/**
 * クリアランスソナーの表示 — ラジコンビューの左上、電池・温度パネルの下（指示による。
 * 映像の下側は運転に要る視界のため）。
 *
 * 実車のクリアランスソナーと同じく、上から見た車体のまわりを 16 方向に分け、方向ごとに4段の帯を
 * 描く。**近いほど内側の帯が点く**（遠い段から 緑 → 琥珀 → 琥珀 → 赤。色に加えて帯の位置でも
 * 段が分かる）。距離は `bus/parkSensor.ts`（LiDAR の全周＋前後の超音波、車体の外形から測る）。
 * 数字は出さない（指示による）。
 *
 * **出すのは要るときだけ**: 後退を選んでいるとき、またはゆっくり（駐車の速さ）動いていて
 * どこかの方向が一番遠い段より近いとき。それ以外は何も出さない（正常は光らせない）。
 * 音は `hooks/useParkBeep.ts`（同じしきい値）。
 */
import { live, useNumbers } from '../../bus/live'
import {
  BODY_LEN_M, BODY_WIDTH_M, PARK_MAX_SPEED_MS, SECTORS, sectorClearance, stepOf,
} from '../../bus/parkSensor'
import { reverseSelected, useUi } from '../../store/ui'

/** 段ごとの色（段 1=遠い〜4=一番近い） */
const STEP_COLOR = ['', '#3ac26a', '#e8ab21', '#e8ab21', '#f2544b']
const OFF = '#26262c'
/** 点群がこれより古ければ使わない（超音波だけで出す）[ms] */
const SCAN_FRESH_MS = 400

/** 図の寸法（viewBox 単位）。車体の長さを `CAR_L` にして幅は実寸比 */
const VB_W = 100
const VB_H = 124
const CX = VB_W / 2
const CY = VB_H / 2
const CAR_L = 46
const CAR_W = (CAR_L * BODY_WIDTH_M) / BODY_LEN_M
/** 帯の中心の、車体の外形からの距離（内側＝一番近い段から）と太さ */
const BAND_OFFSET = [6, 13, 20, 27]
const BAND_STROKE = 5
/** 方向と方向の間の隙間 [rad]（帯を区切って見せる） */
const GAP = (2.2 * Math.PI) / 180

export function ParkSensor() {
  const n = useNumbers()
  const reverse = useUi((s) => reverseSelected(s))
  const vs = n.vs
  if (!vs) return null
  const scan = performance.now() - live.lastScanMs < SCAN_FRESH_MS ? live.scan : null
  const sectors = sectorClearance(scan, vs)
  const steps = sectors.map(stepOf)
  const slow = Math.abs(vs.speed) < PARK_MAX_SPEED_MS
  if (!reverse && !(slow && steps.some((s) => s > 0))) return null

  return (
    <div className={`park-sensor ${n.stale ? 'stale' : ''}`}>
      <div className="rcs-head">SONAR{scan ? '' : ' · 超音波のみ'}</div>
      <svg viewBox={`0 0 ${VB_W} ${VB_H}`} className="park-svg" aria-label="クリアランスソナー">
        <defs>
          {Array.from({ length: SECTORS }, (_, i) => (
            <clipPath key={i} id={`ps-sector-${i}`}>
              <polygon points={wedge(i)} />
            </clipPath>
          ))}
        </defs>
        {Array.from({ length: SECTORS }, (_, i) => {
          const step = steps[i]!
          return (
            <g key={i} clipPath={`url(#ps-sector-${i})`}>
              {BAND_OFFSET.map((o, b) => {
                // 帯 b=0 が一番内側（段4）。点けるのはその方向の段に当たる帯だけ
                const lit = step > 0 && b === 4 - step
                return (
                  <rect
                    key={b}
                    x={CX - CAR_W / 2 - o}
                    y={CY - CAR_L / 2 - o}
                    width={CAR_W + 2 * o}
                    height={CAR_L + 2 * o}
                    rx={4 + o}
                    fill="none"
                    stroke={lit ? STEP_COLOR[step] : OFF}
                    strokeWidth={BAND_STROKE}
                    style={lit ? { filter: `drop-shadow(0 0 3px ${STEP_COLOR[step]})` } : undefined}
                  />
                )
              })}
            </g>
          )
        })}
        {/* 車体（上から見た形）。前が上 */}
        <rect x={CX - CAR_W / 2} y={CY - CAR_L / 2} width={CAR_W} height={CAR_L} rx={4} className="park-car" />
        <rect x={CX - CAR_W / 2 + 3} y={CY - CAR_L / 2 + 8} width={CAR_W - 6} height={9} rx={2} className="park-glass" />
      </svg>
    </div>
  )
}

/** 方向 `i` の扇（車体の中心から外へ）。0 が真正面＝上、反時計回り＝画面の左回り */
function wedge(i: number): string {
  const w = (2 * Math.PI) / SECTORS
  const a0 = i * w - w / 2 + GAP / 2
  const a1 = i * w + w / 2 - GAP / 2
  const R = 200
  // 車両の角度 a（左回り）→ 画面: 前が上、左が左
  const pt = (a: number) => `${CX - R * Math.sin(a)},${CY - R * Math.cos(a)}`
  return `${CX},${CY} ${pt(a0)} ${pt(a1)}`
}
