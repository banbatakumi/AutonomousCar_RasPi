/**
 * キーボード入力（`useDriving.ts` から分けた）。押しているキーの集合を ref で返し、送信ループが毎フレーム読む。
 *
 * ARM のトグル（Enter）・E-STOP（Esc）・灯火（L）・ギア（←/→）は押した瞬間にここで処理する。
 * タブが背後に回ったら即 DISARM、フォーカスを失ったら押下状態を捨てる（理由は下のコメント）。
 */
import { useEffect, useRef, type RefObject } from 'react'
import { cmdOut, live } from '../bus/live'
import { shiftGearUpdate } from '../store/gear'
import { LIGHT_CYCLE, useUi } from '../store/ui'
import type { ControlChannel } from '../ws/control'
import { nextLight } from './drivingLogic'
import { BRAKE_KEY, DRIVING_KEYS, GEAR_KEY_DOWN, GEAR_KEY_UP, LIGHT_KEY } from './keys'

/** `<input>` のうち文字を打たないもの。ここにフォーカスがあっても操縦キーは効かせる */
const NON_TEXT_INPUT_TYPES = new Set([
  'range', 'checkbox', 'radio', 'button', 'submit', 'reset', 'file', 'color', 'image',
])

/** 文字を打つ欄か（テキスト・数値の入力欄、TEXTAREA、contentEditable） */
export function isTextEntry(el: HTMLElement): boolean {
  if (el.isContentEditable || el.tagName === 'TEXTAREA') return true
  return el.tagName === 'INPUT' && !NON_TEXT_INPUT_TYPES.has((el as HTMLInputElement).type)
}

/**
 * ギアを1段シフトする。`delta=-1` でダウン、`+1` でアップ（規則は `store/gear.ts` の `shiftGearUpdate`）。
 * パッドの L1/R1（送信ループ）とキーボードの←/→（下の `down` ハンドラ）の両方から呼ぶ。
 */
export function shiftGear(delta: number) {
  const ui = useUi.getState()
  // テレメトリが届いていない間は車速が分からない。**指令を出していないときだけ**停止とみなす
  // （届いていないことを「止まっている」と読むと、走行中に D↔R を切り替えられる）
  const stopped = live.vs != null
    ? live.vs.stopped
    : !cmdOut.active || (cmdOut.speed === 0 && cmdOut.torque === 0)
  const p = shiftGearUpdate(ui, delta, stopped)
  if (p) ui.set(p)
}

export function useKeyboardInput(ch: ControlChannel | null): RefObject<Set<string>> {
  const keys = useRef(new Set<string>())
  useEffect(() => {
    const disarm = (why: string) => {
      keys.current.clear()
      useUi.getState().disarm(why)
    }

    const down = (e: KeyboardEvent) => {
      if (e.repeat) return
      const ui = useUi.getState()
      if (e.code === 'Escape') {
        ch?.estop()
        disarm('Esc で E-STOP')
        return
      }
      // **Cmd（Meta）を押している間のキーは操縦に使わない。** macOS は Cmd を押している間、
      // 他のキーの keyup を配らない（Cmd+Tab で別アプリへ移ると W が押されたまま残る）。
      // Cmd が下りた時点で押下状態を捨て、Cmd 付きのキーは集合に入れない
      if (e.metaKey || e.code === 'MetaLeft' || e.code === 'MetaRight') {
        keys.current.clear()
        return
      }
      // **文字を打つ欄にフォーカスがある間は操縦キーを奪わない。** 地図名の入力欄
      // （`MapLibrary.tsx`）等で `KeyD`/`Space` が打てなくなるため。`SELECT` は矢印キーでの
      // 選択がギアシフトに奪われるので同じ扱い（ただしブレーキの Space だけは通す）。
      //
      // ⚠ **スライダ・チェックボックス・ボタンは除外しない。** 設定ドロワーのスライダを
      // 触った後もフォーカスはそこに残るので、除外すると走行中に Space（ブレーキ）も逆キーも
      // 効かなくなる。E-STOP（上の`Escape`）はフォーカスに関係なく効く
      const focused = e.target instanceof HTMLElement ? e.target : null
      if (focused && isTextEntry(focused)) return
      if (focused && focused.tagName === 'SELECT') {
        if (e.code === BRAKE_KEY) keys.current.add(e.code)
        return
      }
      // 操縦キーを押した時点で、フォーカスの残っている操作部品（ボタン・スライダ等）から
      // フォーカスを外す。残したままだと Enter/Space がその部品も動かす（ARM したのに
      // 「自律走行を開始」がもう一度押されて解除される、矢印キーでスライダが動く、など）
      const isDrivingKey = e.code === 'Enter' || e.code === LIGHT_KEY || e.code === GEAR_KEY_DOWN
        || e.code === GEAR_KEY_UP || DRIVING_KEYS.includes(e.code)
      if (isDrivingKey && focused && focused !== document.body) {
        e.preventDefault()
        focused.blur()
      }
      if (e.code === 'Enter') {
        // ARM のトグル。**入れるのも切るのも同じキー**
        // 自分で解除したときは理由を出さない（ARM ボタンと同じ。`StatusBar.tsx` の `ArmButton`）
        if (ui.armRequested) disarm('')
        else ui.arm()
        return
      }
      if (e.code === LIGHT_KEY) {
        // **押した瞬間に1つ進めるだけ。** 押しっぱなしで回り続けると
        // どのモードで手を離したのか分からなくなる（`e.repeat` は上で弾いてある）
        ui.set({ lightMode: nextLight(LIGHT_CYCLE, ui.lightMode) })
        return
      }
      if (e.code === GEAR_KEY_DOWN || e.code === GEAR_KEY_UP) {
        e.preventDefault() // 矢印キーでページがスクロールしないように
        shiftGear(e.code === GEAR_KEY_DOWN ? -1 : 1)
        return
      }
      if (DRIVING_KEYS.includes(e.code)) {
        e.preventDefault() // 矢印キーでページがスクロールしないように
        keys.current.add(e.code)
      }
    }
    const up = (e: KeyboardEvent) => keys.current.delete(e.code)

    // **タブが背後に回ったら即 DISARM。** 押しっぱなし扱いのまま裏に回ると、
    // 戻ってきたときに突然走り出す。
    //
    // ⚠ 単なる `blur`（ウィンドウのフォーカス喪失）では DISARM しない。
    // iPad の Split View・OS の通知バナー・ゲームパッド接続時のシステム
    // オーバーレイなどで、操作者が離席していなくても `blur` は容易に発火する。
    //
    // **ただしキーの押下状態は捨てる。** フォーカスを失うと keyup は別のウィンドウへ行き、
    // 押していたキーが「押されたまま」残る（ウィンドウは見えているので `hidden` にもならず、
    // 加速し続ける）。捨てれば惰行で止まり、戻ってきて押し直せば続きから操作できる。
    // パッドは毎フレーム読むだけで押下状態を持たないので影響しない
    const onBlur = () => keys.current.clear()
    const onHide = () => {
      if (document.hidden) disarm('タブが背後に回った')
    }

    window.addEventListener('keydown', down)
    window.addEventListener('keyup', up)
    window.addEventListener('blur', onBlur)
    document.addEventListener('visibilitychange', onHide)
    return () => {
      window.removeEventListener('keydown', down)
      window.removeEventListener('keyup', up)
      window.removeEventListener('blur', onBlur)
      document.removeEventListener('visibilitychange', onHide)
    }
  }, [ch])
  return keys
}
