/**
 * Wake Lock: ARM 中は画面ロックを抑止する（`useDriving.ts` から分けた）。
 *
 * `blur` DISARM を撤廃した後も、iPad の画面自動ロックは `document.hidden` を
 * 真にするので `onHide` 経由で（正しく）DISARM される。走行中に画面が消灯して
 * 止まるのは運用上困るので、ARM 中だけ Wake Lock で画面ロックそのものを防ぐ。
 *
 * 失敗（非対応ブラウザ・API 拒否）は握りつぶす。**Wake Lock は快適性のための
 * 機能であり、取得できなくても走行の安全性には影響しない**。
 *
 * ⚠ **Wake Lock は HTTPS（secure context）でしか使えない。** Pi が配る
 * `http://surge-mk2.local:8000` では `navigator.wakeLock` 自体が無く、画面が自動ロック
 * すると DISARM する（localhost の `npm run dev` では使えるので開発中は気づけない）。
 * 使えないときは何もしない（知らせると毎回出て邪魔なため。指示による）。対処は端末側で自動ロックを切る
 */
import { useEffect } from 'react'
import { useUi } from '../store/ui'

export function useWakeLock(): void {
  useEffect(() => {
    if (!('wakeLock' in navigator)) return

    let sentinel: WakeLockSentinel | null = null
    let cancelled = false

    const release = () => {
      const s = sentinel
      sentinel = null
      s?.release().catch(() => {})
    }
    const acquire = () => {
      if (sentinel || document.hidden) return
      navigator.wakeLock.request('screen').then((s) => {
        if (cancelled) {
          s.release().catch(() => {})
          return
        }
        sentinel = s
      }).catch(() => {
        // 非対応・拒否・低バッテリー等はここに来る。ARM 自体は妨げない
      })
    }

    const unsub = useUi.subscribe((state, prev) => {
      if (state.armRequested === prev.armRequested) return
      if (state.armRequested) acquire()
      else release()
    })
    // Wake Lock はタブが hidden になると自動解放される。ARM したまま復帰したら取り直す
    const onVisible = () => {
      if (!document.hidden && useUi.getState().armRequested) acquire()
    }
    document.addEventListener('visibilitychange', onVisible)

    if (useUi.getState().armRequested) acquire()

    return () => {
      cancelled = true
      unsub()
      document.removeEventListener('visibilitychange', onVisible)
      release()
    }
  }, [])
}
