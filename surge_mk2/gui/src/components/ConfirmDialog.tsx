/**
 * 確認・お知らせのダイアログ。**`window.confirm`/`window.alert` を使わないためのもの。**
 *
 * ブラウザのダイアログは開いている間 JavaScript を丸ごと止める。送信ループ（`useDriving.ts` の
 * rAF）も WebSocket も Esc のハンドラも動かないので、手動走行はデッドマンで止まるが、
 * **自律走行は Wi-Fi の瞬断を待つ猶予のあいだ走り続け、その間 E-STOP が押せない**。
 * 「走行中です。本当に削除しますか？」は、まさに走っている最中に出す。
 *
 * こちらは普通の DOM なので、出している間も操縦・E-STOP・テレメトリは生きたまま。
 *
 * - `askConfirm(文)` … はい/いいえ。`Promise<boolean>`
 * - `notify(文)` …… 了解だけ
 *
 * **キーでは答えさせない。** Esc は E-STOP、Enter は ARM のトグル（`useDriving.ts`）で、
 * ここで奪うとパニックで叩いた Esc がダイアログを閉じるだけになる。ボタンのクリックだけで答える。
 * 続けて呼ばれたら順に出す（後から来たものが前のものを消さない）。
 */
import { useSyncExternalStore } from 'react'

type Entry = { id: number; message: string; confirm: boolean; resolve: (ok: boolean) => void }

let queue: Entry[] = []
let nextId = 1
const listeners = new Set<() => void>()

function emit() {
  for (const l of listeners) l()
}

function push(message: string, confirm: boolean): Promise<boolean> {
  return new Promise((resolve) => {
    queue = [...queue, { id: nextId++, message, confirm, resolve }]
    emit()
  })
}

function answer(e: Entry, ok: boolean) {
  queue = queue.filter((q) => q.id !== e.id)
  emit()
  e.resolve(ok)
}

/** はい/いいえを尋ねる。「はい」で true */
export function askConfirm(message: string): Promise<boolean> {
  return push(message, true)
}

/** 読んでもらうだけのお知らせ */
export function notify(message: string): void {
  void push(message, false)
}

function subscribe(fn: () => void) {
  listeners.add(fn)
  return () => listeners.delete(fn)
}

/** ダイアログの置き場。**`App` の直下に1つだけ置く**（タブの中に置くと、タブを替えたら消える） */
export function ConfirmHost() {
  const q = useSyncExternalStore(subscribe, () => queue)
  const cur = q[0]
  if (!cur) return null
  return (
    // 背景は暗くしない・クリックも奪わない（E-STOP ボタンもカメラも見えたまま、押せるまま）
    <div
      role="dialog"
      aria-live="assertive"
      style={{
        position: 'fixed',
        top: 64,
        left: '50%',
        transform: 'translateX(-50%)',
        zIndex: 1000,
        maxWidth: 'min(520px, 90vw)',
        padding: '14px 16px',
        background: 'var(--panel2)',
        border: '1px solid var(--warn)',
        borderRadius: 8,
        boxShadow: '0 8px 28px rgba(0, 0, 0, 0.6)',
        color: 'var(--fg)',
      }}
    >
      <p style={{ margin: '0 0 12px', whiteSpace: 'pre-wrap' }}>{cur.message}</p>
      <div style={{ display: 'flex', gap: 8, justifyContent: 'flex-end' }}>
        {cur.confirm && <button onClick={() => answer(cur, false)}>いいえ</button>}
        <button className="on" onClick={() => answer(cur, true)}>
          {cur.confirm ? 'はい' : '了解'}
        </button>
      </div>
    </div>
  )
}
