/**
 * エンジン音シミュレーションの駆動ループ。`ui.engineSoundOn` の間だけ
 * `EngineSound`（`audio/engineSound.ts`）を rAF で更新する——`GMeter.tsx` と
 * 同じく `live.vs`/`useUi.getState()` を直読みする層1の書き方（`bus/live.ts`
 * 冒頭のコメント参照）。React state 経由だと 8Hz に間引かれ音がカクつく。
 *
 * 回転数相当（`rpmFrac`）の作り方は**内燃機関と EV で別ロジック**にしてある
 *——エンジン音と EV/モータ音は本質的に別物で、単に周波数や
 * ゲインの数値を変えるだけでは片方の演出（レブリミット、シフトブリップ）が
 * もう片方にも意味もなく残ってしまうため。
 *
 * - **内燃機関**: 仮想変速（`bus/virtualGear.ts`）の `rpmFrac`——**今のギアの上限**を
 *   分母にする。MT は運転者が選んだギア、速度指令・トルク指令は速度から自動で選ぶ
 *   仮想 AT（2026-11-03、指示により MT 以外でも変速させた）。実車と同じで、ギアで
 *   音程が上下する。ギアが変わった瞬間だけ `shiftBlip()`（クラッチを切ったような
 *   一瞬の音の落ち込み）も呼ぶ。
 * - **EV**: ギアを使わない。MT は `settings.mtMaxSpeed`、それ以外は `settings.maxSpeed`
 *   （どちらもギア比を掛けない車体全体の上限）をそのまま分母にする。EV は単一の
 *   減速機なので、ギアごとのレッドラインという概念自体が無い。`shiftBlip()` も呼ばない。
 *
 * レブリミッター演出自体も `EngineSound` 側で内燃機関だけに限定してある
 * （`Profile.hasIgnitionCutLimiter`、`audio/engineSound.ts` 参照）——EV は
 * `frac` がどれだけ上がっても点火カットのような断続は起きない。
 *
 * 音色は `EngineSound.setType()` で
 * **鳴っている最中でも `AudioContext` を作り直さずに**切り替える。ON/OFF
 * （`on`）と音色（`type`）を別の `useEffect` に分けているのはそのため——
 * 音色を変えるたびに rAF ループや `AudioContext` を作り直す必要は無い。
 */
import { useEffect, useRef } from 'react'
import { EngineSound } from '../audio/engineSound'
import { live } from '../bus/live'
import { virtualGear } from '../bus/virtualGear'
import { useUi } from '../store/ui'

export function useEngineSound() {
  const engineRef = useRef<EngineSound | null>(null)
  if (!engineRef.current) engineRef.current = new EngineSound()
  const engine = engineRef.current

  const on = useUi((s) => s.engineSoundOn)
  const type = useUi((s) => s.settings.engineSoundType)

  // 音色の切り替えだけを別に持つ。ON/OFF の effect を作り直さない
  useEffect(() => {
    engine.setType(type)
  }, [engine, type])

  useEffect(() => {
    if (!on) {
      engine.stop()
      return
    }
    // トグル ON のクリックというユーザー操作のすぐ後で呼ぶ（自動再生制限を通すため）
    engine.start(type)

    let raf = 0
    let prevShift: number | null = null
    const tick = () => {
      raf = requestAnimationFrame(tick)
      const ui = useUi.getState()
      const s = ui.settings
      const vs = live.vs
      const armed = ui.armRequested
      const speedAbs = vs ? Math.abs(vs.speed) : 0
      // **エンジン音と EV/モータ音で別ロジック**（ファイル冒頭コメント参照）。
      // 内燃機関だけ仮想変速に従う——EV に「ギアごとのレッドライン」は無い
      const isCombustion = s.engineSoundType === 'combustion'
      const vg = virtualGear()
      let rpmFrac: number
      if (isCombustion) {
        rpmFrac = vg.rpmFrac
      } else {
        const whole = s.driveMode === 'mt' ? s.mtMaxSpeed : s.maxSpeed
        rpmFrac = whole > 0.001 ? Math.min(1, speedAbs / whole) : 0
      }
      engine.update(rpmFrac, armed)

      // シフトブリップ（クラッチ操作の一時演出）も内燃機関だけ
      if (isCombustion && prevShift != null && prevShift !== vg.shiftCount) engine.shiftBlip()
      prevShift = vg.shiftCount
    }
    raf = requestAnimationFrame(tick)
    // タブが背後に回ると rAF が止まり、最後の周波数のまま鳴り続ける。非表示の間は止める
    // （表示に戻れば `update()` が `resume()` する）
    const onVisibility = () => {
      if (document.hidden) engine.suspend()
    }
    document.addEventListener('visibilitychange', onVisibility)
    return () => {
      cancelAnimationFrame(raf)
      document.removeEventListener('visibilitychange', onVisibility)
    }
    // `type` は初回 start() のみに使う（以後の変更は上の effect が `setType()` で担う）ので、
    // 依存配列にはあえて含めない——含めると音色を変えるたびに rAF ループを作り直してしまう
  }, [engine, on])

  // アンマウント時（タブごと閉じる等）は確実に止める
  useEffect(() => () => engine.stop(), [engine])
}
