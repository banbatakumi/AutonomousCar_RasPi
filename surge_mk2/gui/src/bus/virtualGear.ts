/**
 * 仮想変速 — シフトライト（`ShiftLights.tsx`）・エンジン音（`useEngineSound.ts`）・
 * 速度計のギア表示（`SpeedGauge.tsx`）が共用する「今何速で、回転数はどのくらいか」。
 *
 * **表示と音の演出だけ。車への指令は一切変えない**（指令は `useDriving.ts` が別に作る）。
 *
 * - **MT モード**: ギアは運転者が選んだ `ui.mtGear`。上限は `mtMaxSpeed × mtGearRatio()`
 *   （`useDriving.ts` が速度指令の上限に使うのと同じ式）
 * - **速度指令・トルク指令**: ギアが無いので、速度から 1〜6 速を自動で選ぶ仮想 AT にする
 *   （2026-11-03、指示による）。各速の上限は **車の最大速度**（STM32 の `LIMITS.max_speed_m_s`、
 *   速度計の目盛りと同じ。未受信の間は `PI_MAX_SPEED_CAP`）× `AT_RATIOS`。
 *   RC の速度ダイヤル（`maxSpeed`）を基準にすると、ダイヤルを絞ったとき低速で最上段まで
 *   上がってしまう（1m/s で 5 速に入った。指示により変更）。段数と比は固定で、設定は持たない
 *   （MT は今どおり設定の `mtGear1〜5`）
 *   - シフトアップ: そのギアの上限の `UPSHIFT_FRAC` に達したとき
 *   - シフトダウン: 1段下のギアの上限の `DOWNSHIFT_FRAC` を下回ったとき。
 *     上げと下げの基準をずらして（ヒステリシス）、境目でギアが行き来しないようにする
 *
 * 状態（仮想 AT の今のギア）はモジュールに1つだけ持つ。**同じ入力で何度呼んでも結果が
 * 変わらない**（ギアの移動を while で一度に済ませる）ので、複数の rAF ループと 8Hz の
 * 描画から同じフレームに呼ばれても、片方だけ余分に進むことはない。
 */
import { live } from './live'
import { mtGearRatio, PI_MAX_SPEED_CAP, useUi } from '../store/ui'

/** 仮想 AT の各速の上限（車の最大速度に対する比）。上の段ほど間隔を詰める（実車の AT と同じ傾向） */
const AT_RATIOS = [0.18, 0.31, 0.45, 0.6, 0.78, 1.0]
const AT_TOP = AT_RATIOS.length
/** シフトアップする回転数（そのギアの上限に対する割合） */
const UPSHIFT_FRAC = 0.92
/** シフトダウンする速度（1段下のギアの上限に対する割合） */
const DOWNSHIFT_FRAC = 0.7
/** これ未満は停止とみなして 1 速に戻す [m/s] */
const STOP_SPEED = 0.02

export type VirtualGear = {
  /** 表示用。'R' / 'N' / '1'〜'6'（MT は '1'〜'5'） */
  label: string
  /** そのギアの上限に対する速度の割合（0〜1）。回転数の代わり */
  rpmFrac: number
  /** これ以上シフトアップできない（最上段・R・N）。シフトライトの「上げろ」の点滅を出さない */
  noUpshift: boolean
  /** 変速のたびに 1 増える。呼び出し側は前回値と比べて「今変速した」を知る */
  shiftCount: number
}

let atGear = 1
let shiftCount = 0
let lastLabel = ''

function atRatio(g: number): number {
  return AT_RATIOS[g - 1] ?? 1
}

export function virtualGear(): VirtualGear {
  const ui = useUi.getState()
  const s = ui.settings
  const vs = live.vs
  const speed = vs && !vs.stopped ? vs.speed : 0
  const v = Math.abs(speed)

  let label: string
  let gearMax: number
  let noUpshift: boolean
  if (s.driveMode === 'mt') {
    const g = ui.mtGear
    label = g === 'R' || g === 'N' ? g : g.slice(1)
    gearMax = s.mtMaxSpeed * mtGearRatio(g, s)
    noUpshift = g === 'R' || g === 'N' || g === 'D5'
  } else {
    const carMax = Math.max(0.1, live.link?.max_speed_m_s ?? PI_MAX_SPEED_CAP)
    if (ui.gear === 'R' || speed < -STOP_SPEED) {
      atGear = 1
      label = 'R'
      gearMax = carMax * atRatio(1)
      noUpshift = true
    } else {
      if (v < STOP_SPEED) atGear = 1
      while (atGear < AT_TOP && v >= UPSHIFT_FRAC * carMax * atRatio(atGear)) atGear++
      while (atGear > 1 && v < DOWNSHIFT_FRAC * carMax * atRatio(atGear - 1)) atGear--
      label = String(atGear)
      gearMax = carMax * atRatio(atGear)
      noUpshift = atGear === AT_TOP
    }
  }

  if (label !== lastLabel) {
    if (lastLabel !== '') shiftCount++
    lastLabel = label
  }
  const rpmFrac = gearMax > 0.001 ? Math.min(1, v / gearMax) : 0
  return { label, rpmFrac, noUpshift, shiftCount }
}
