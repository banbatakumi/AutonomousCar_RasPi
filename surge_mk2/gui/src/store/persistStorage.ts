/**
 * zustand `persist` の保存先（localStorage）。ストアごとに `try { localStorage }` を書き分けていたのを1つにした。
 *
 * - **読み書きの失敗は握りつぶす。** プライベートウィンドウ・容量超過などで保存できなくても、
 *   走行と表示は続けられるべき
 * - **同じ中身は書き直さない。** `persist` は `set` のたびに保存する。`ui` ストアは status の受信や
 *   補機のランプでも `set` されるので、保存する部分が変わっていなければ書かない
 * - **新しいキーが無ければ `legacy()` で旧形式を読む。** `persist` 導入前は値ごとに別のキーへ
 *   生の値を書いていた。読めたら版 0 として渡し、各ストアの `migrate` が今の形に直す
 *   （既存の端末の設定を消さないため）
 */
import type { PersistStorage, StorageValue } from 'zustand/middleware'

export function localPersistStorage<S>(legacy?: () => unknown): PersistStorage<S> {
  let lastWritten: string | null = null
  return {
    getItem: (name) => {
      try {
        const raw = localStorage.getItem(name)
        if (raw != null) {
          lastWritten = raw
          return JSON.parse(raw) as StorageValue<S>
        }
      } catch {
        // 壊れた値・読めない環境は「保存なし」として扱う（下の旧形式か既定値に倒れる）
      }
      const old = legacy?.()
      return old == null ? null : { state: old as S, version: 0 }
    },
    setItem: (name, value) => {
      const raw = JSON.stringify(value)
      if (raw === lastWritten) return
      try {
        localStorage.setItem(name, raw)
        lastWritten = raw
      } catch {
        // 保存できなくても走行・表示は続けられるべきなので無視する
      }
    },
    removeItem: (name) => {
      try {
        localStorage.removeItem(name)
      } catch {
        /* 同上 */
      }
    },
  }
}
