/**
 * シフトライト — HUD の台形の上辺に並ぶ LED 列（指示による）。
 *
 * F1 やレース用ダッシュ（AEM CD-7 など）の上辺の LED に倣った。**回転数の代わりに
 * 仮想変速の `rpmFrac`**（`bus/virtualGear.ts`。今のギアの上限に対する速度の割合）で
 * 点ける。MT はもちろん、速度指令・トルク指令でも仮想 AT のギアに従う。
 *
 * - 左右の外側から中央へ向かって点く。外から 緑 3 段・黄 2 段・赤 3 段（15 個）
 * - `SHIFT_FRAC` 以上で全灯を青く点滅＝シフトアップの合図。上げられないギア
 *   （最上段・R・N、`noUpshift`）では出さず、赤までで止める
 * - TC/ABS の介入中は両端の 1 個ずつを黄で点滅させる（F1 で LED が警告も兼ねるのに倣う）。
 *   介入ランプ（`AssistLamps`）はそのまま残す
 * - 受信が途絶・DISARM 中は全部消す
 *
 * 更新は rAF で class の付け外しだけ（`AssistLamps` と同じ。React の state は使わない）。
 */
import { useEffect, useRef } from 'react'
import { live } from '../../bus/live'
import { virtualGear } from '../../bus/virtualGear'

const N_LEDS = 15
/** 外側からの段（0〜7）ごとの点灯しきい値（rpmFrac）。最初の段から最後の段まで等間隔 */
const FIRST_ON = 0.3
const LAST_ON = 0.9
const STEPS = Math.ceil(N_LEDS / 2)
/** 全灯の青い点滅（シフトアップの合図）を出す rpmFrac */
const SHIFT_FRAC = 0.95

function stepOf(i: number): number {
  return Math.min(i, N_LEDS - 1 - i)
}

function colorOf(step: number): 'g' | 'y' | 'r' {
  if (step < 3) return 'g'
  if (step < 5) return 'y'
  return 'r'
}

export function ShiftLights() {
  const refs = useRef<(HTMLSpanElement | null)[]>([])

  useEffect(() => {
    let raf = 0
    const shown: string[] = []
    const tick = () => {
      raf = requestAnimationFrame(tick)
      const vs = live.vs
      const fresh = vs != null && performance.now() - live.lastRxMs < 500
      const on = fresh && !!vs?.armed
      const vg = virtualGear()
      const shift = on && !vg.noUpshift && vg.rpmFrac >= SHIFT_FRAC
      const assist = on && !!(vs?.tc_active || vs?.abs_active)
      for (let i = 0; i < N_LEDS; i++) {
        const step = stepOf(i)
        const th = FIRST_ON + ((LAST_ON - FIRST_ON) * step) / (STEPS - 1)
        let cls = `sl-led sl-${colorOf(step)}`
        if (shift) cls += ' shift'
        else if (assist && step === 0) cls += ' assist'
        else if (on && vg.rpmFrac >= th) cls += ' on'
        if (shown[i] !== cls) {
          shown[i] = cls
          const el = refs.current[i]
          if (el) el.className = cls
        }
      }
    }
    raf = requestAnimationFrame(tick)
    return () => cancelAnimationFrame(raf)
  }, [])

  return (
    <div className="shift-lights" aria-hidden="true">
      {Array.from({ length: N_LEDS }, (_, i) => (
        <span
          key={i}
          ref={(el) => {
            refs.current[i] = el
          }}
          className={`sl-led sl-${colorOf(stepOf(i))}`}
        />
      ))}
    </div>
  )
}
