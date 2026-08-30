/**
 * エンジン音シミュレーションの駆動ループ。`ui.engineSoundOn` の間だけ
 * `EngineSound`（`audio/engineSound.ts`）を rAF で更新する——`GMeter.tsx` と
 * 同じく `live.vs`/`useUi.getState()` を直読みする層1の書き方（`bus/live.ts`
 * 冒頭のコメント参照）。React state 経由だと 8Hz に間引かれ音がカクつく。
 *
 * 回転数相当（`rpmFrac`）の作り方は**内燃機関と EV で別ロジック**にしてある
 * （2026-08-30）——エンジン音と EV/モータ音は本質的に別物で、単に周波数や
 * ゲインの数値を変えるだけでは片方の演出（レブリミット、シフトブリップ）が
 * もう片方にも意味もなく残ってしまうため。
 *
 * - **内燃機関**: `settings.mtMaxSpeed × mtGearRatio(ui.mtGear, s)`——**現在の
 *   ギアの上限**を分母にする。実車の MT は各ギアにそれぞれのレッドラインがある
 *   （1速を踏み切れば低速でもレブに当たるし、シフトダウンすれば同じ実速度でも
 *   相対的な回転数＝音程が上がる）ため、ギアで音程が変わること自体は正しい。
 *   ギアが変わった瞬間だけ `shiftBlip()`（クラッチを切ったような一瞬の音の
 *   落ち込み）も呼ぶ——これも内燃機関（MT）だけの演出。
 * - **EV**: `settings.mtMaxSpeed`（ギア比を掛けない、車体全体の上限）をそのまま
 *   分母にする。EV にはこのシミュレーション上「複数速のギア」という概念が無い
 *   （単一の減速機のみ）ので、ギアごとの相対レッドラインという概念自体が無い。
 *   `shiftBlip()` も呼ばない（EV にクラッチは無い）。
 *
 * レブリミッター演出自体も `EngineSound` 側で内燃機関だけに限定してある
 * （`Profile.hasIgnitionCutLimiter`、`audio/engineSound.ts` 参照）——EV は
 * `frac` がどれだけ上がっても点火カットのような断続は起きない。
 *
 * 音色（`settings.engineSoundType`、2026-08-30 追加）は `EngineSound.setType()` で
 * **鳴っている最中でも `AudioContext` を作り直さずに**切り替える。ON/OFF
 * （`on`）と音色（`type`）を別の `useEffect` に分けているのはそのため——
 * 音色を変えるたびに rAF ループや `AudioContext` を作り直す必要は無い。
 */
import { useEffect, useRef } from 'react'
import { EngineSound } from '../audio/engineSound'
import { live } from '../bus/live'
import { mtGearRatio, useUi } from '../store/ui'

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
    let prevMtGear: string | null = null
    const tick = () => {
      raf = requestAnimationFrame(tick)
      const ui = useUi.getState()
      const s = ui.settings
      const vs = live.vs
      const armed = ui.armRequested
      const speedAbs = vs ? Math.abs(vs.speed) : 0
      const mtMode = s.driveMode === 'mt'
      // **エンジン音と EV/モータ音で別ロジック**（ファイル冒頭コメント参照）。
      // 内燃機関だけギア比を分母に掛ける——EV に「ギアごとのレッドライン」は無い
      const isCombustion = s.engineSoundType === 'combustion'
      const gearMax = mtMode
        ? (isCombustion ? s.mtMaxSpeed * mtGearRatio(ui.mtGear, s) : s.mtMaxSpeed)
        : s.maxSpeed
      const rpmFrac = gearMax > 0.001 ? Math.min(1, speedAbs / gearMax) : 0
      engine.update(rpmFrac, armed)

      // シフトブリップ（クラッチ操作の一時演出）も内燃機関の MT だけ
      if (isCombustion && mtMode && prevMtGear != null && prevMtGear !== ui.mtGear) engine.shiftBlip()
      prevMtGear = isCombustion && mtMode ? ui.mtGear : null
    }
    raf = requestAnimationFrame(tick)
    return () => cancelAnimationFrame(raf)
    // `type` は初回 start() のみに使う（以後の変更は上の effect が `setType()` で担う）ので、
    // 依存配列にはあえて含めない——含めると音色を変えるたびに rAF ループを作り直してしまう
  }, [engine, on])

  // アンマウント時（タブごと閉じる等）は確実に止める
  useEffect(() => () => engine.stop(), [engine])
}
