/**
 * 介入ランプ — 「今、車が勝手に何をしたか」だけを光で伝える。
 *
 * ## 数字を出さない
 *
 * 診断タブにはスリップ量 [m/s] が並んでいるが、**運転中にあの数字を読む人はいない**。
 * 知りたいのは「今トラコンが入ったか」の一点なので、点灯だけにする。
 * 原因を追いたくなったら診断タブへ行けばよい（層 C/D への隔離、`architecture.md` §10.1）。
 *
 * ## 最低点灯時間を設ける
 *
 * `tc_active` / `tv_active` は 1〜2 フレームで落ちることがある。8Hz のスナップショット
 * （`useNumbers`）で光らせると**介入したのに光らない**。`live.vs` を rAF で見て、
 * 立った瞬間から `HOLD_MS` は消さないようにラッチする。
 *
 * React の state は使わない（60Hz で setState すると再レンダリングで破綻する）。
 * class の付け外しを直接やる。
 */
/**
 * 2026-08-17: BOOST・HORN・PASS のランプを削除（指示による）。
 * いずれも「押している間だけ光る」自分の操作の可視化で、押しっぱなしを防ぐ目的
 * ——だったが、常時ここに並んでいる価値がないと判断された。HORN・PASS のボタン自体・
 * `useDriving.ts` の入力処理は変えていない（鳴らす・光らせる機能自体は残る）。
 * BOOST（Shift / パッド R1 を押している間だけ速度レンジが `maxSpeed` まで伸びる機能）は
 * この時点ではまだ機能自体は残っていたが、**v0.16 で機能ごと削除した**（指示による）。
 */
import { useEffect, useRef } from 'react'
import { live } from '../../bus/live'
import { useUi } from '../../store/ui'

/** 一瞬の介入も目視できる最低点灯時間 */
const HOLD_MS = 300

export function AssistLamps() {
  const tcRef = useRef<HTMLSpanElement>(null)
  const tvRef = useRef<HTMLSpanElement>(null)
  const asRef = useRef<HTMLSpanElement>(null)
  const ui = useUi()

  useEffect(() => {
    let raf = 0
    // 立ち上がりの時刻。0 は「一度も点いていない」
    const since = { tc: 0, tv: 0, as: 0 }
    const shown = { tc: false, tv: false, as: false }

    const apply = (el: HTMLSpanElement | null, key: 'tc' | 'tv' | 'as', on: boolean) => {
      const now = performance.now()
      if (on) since[key] = now
      const lit = on || now - since[key] < HOLD_MS
      if (el && lit !== shown[key]) {
        el.classList.toggle('on', lit)
        shown[key] = lit
      }
    }

    const tick = () => {
      raf = requestAnimationFrame(tick)
      const vs = live.vs
      const fresh = vs != null && performance.now() - live.lastRxMs < 500
      apply(tcRef.current, 'tc', !!(fresh && vs?.tc_active))
      apply(tvRef.current, 'tv', !!(fresh && vs?.tv_active))
      apply(asRef.current, 'as', !!(fresh && vs?.auto_stop_active))
    }
    raf = requestAnimationFrame(tick)
    return () => cancelAnimationFrame(raf)
  }, [])

  return (
    <div className="lamps">
      <span ref={tcRef} className="lamp lamp-warn" title="トラクションコントロール介入中（駆動輪の空転を抑えた）">
        TC
      </span>
      <span ref={tvRef} className="lamp lamp-warn" title="トルクベクタリング介入中（左右輪へ配分した）">
        TV
      </span>
      <span
        ref={asRef}
        className="lamp lamp-bad"
        title="超音波の自動停止が作動中。STM32 が指令を無視して最大制動している"
      >
        STOP
      </span>
      {/* ここから下は自分の操作なのでラッチ不要。押している間だけ光ればよい。
          **ボタンではなくランプにしてある** — マウスで押しっぱなしにすると、
          カーソルが外れた瞬間に取り残されてクラクションが鳴り続ける（`AuxPanel.tsx` 参照） */}
      <span className={`lamp lamp-warn ${ui.braking ? 'on' : ''}`} title="Space / パッド L2">
        BRAKE
      </span>
    </div>
  )
}
