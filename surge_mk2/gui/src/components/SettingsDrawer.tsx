/**
 * 設定ドロワー — 走行タブ（ラジコン／自動運転）の右上の歯車から出てくるパネル。
 *
 * 2026-08-20 まではラジコンタブ専用だった。自動運転タブでも `useDriving` は
 * 生きており、engage 中の微調整や engage 前後のキーボード操作で同じ調整値
 * （速度・舵のレート等）を使うため、`AutoView.tsx` にも同じコンポーネントを
 * 置いた。中身（`SettingsPanel`）は1つを共有する——タブごとに別の設定を
 * 持たせると「どちらのタブで変えた値が効いているか」が分からなくなる。
 *
 * ## 走りながら開けること
 *
 * 「走らせて → 加速が鈍い → 直す → また走らせる」を切り替え無しで回すのが目的なので、
 * **開いている間も操縦は生きたまま**にする。`useDriving` は `window` の keydown を
 * 見ているので、ドロワーにフォーカスがあっても W A S D はそのまま効く。
 *
 * ## Esc で閉じない
 *
 * Esc は E-STOP（`useDriving.ts` の `down`）。**ここで奪うと、パニックで叩いた Esc が
 * ただパネルを閉じるだけになる。** 閉じるのは歯車の再クリックと × のみ。
 *
 * ## 背景を覆わない
 *
 * オーバーレイで画面全体を暗くすると、開けている間カメラが見えなくなる。
 * 右端に寄せるだけにして、カメラと計器は出したままにする。
 */
import { useUi } from '../store/ui'
import type { ControlChannel } from '../ws/control'
import { SettingsPanel } from './SettingsPanel'

export function SettingsDrawer({ ch }: { ch: ControlChannel | null }) {
  const open = useUi((s) => s.settingsOpen)
  const set = useUi((s) => s.set)

  return (
    <>
      <button
        className={`gear ${open ? 'on' : ''}`}
        onClick={() => set({ settingsOpen: !open })}
        title="操作設定（速度・舵・ブレーキ）。走りながら変えられる"
        aria-label="設定"
      >
        ⚙
      </button>

      {/* 閉じているときも DOM に残す。開くたびにスライダを作り直すとスクロール位置が飛ぶ */}
      <aside className={`drawer ${open ? 'open' : ''}`} aria-hidden={!open}>
        <div className="drawer-head">
          <h2>操作設定</h2>
          <button onClick={() => set({ settingsOpen: false })} title="閉じる（Esc は E-STOP なので使わない）">
            ✕
          </button>
        </div>
        <div className="drawer-body">
          <SettingsPanel ch={ch} />
        </div>
      </aside>
    </>
  )
}
