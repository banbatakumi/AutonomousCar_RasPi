/**
 * rAF の描画・送信ループの共通部品（技術的負債の棚卸し 2026-11-18 中9）。
 *
 * 各所で `raf = requestAnimationFrame(tick)` を手書きしていたのをまとめた。
 * **タブが見えていない間（`document.hidden`）は回さない。** ブラウザも裏のタブでは rAF を
 * ほぼ止めるが、止め方は実装次第（iframe・一部のブラウザは間引くだけ）なので、こちらで明示的に止める。
 * 見えたら次のフレームから再開する（1フレームの時間差は各ループが自分で上限を掛ける）。
 *
 * 次のフレームの予約は**呼ぶ前**にする（手書きの頃と同じ）。1フレームで例外が出てもループは止まらない。
 */
import { useEffect, useRef } from 'react'

/** `cb` を毎フレーム呼ぶループを始める。戻り値で止める */
export function rafLoop(cb: (now: number) => void): () => void {
  let raf = 0
  let stopped = false
  const frame = (now: number) => {
    if (stopped || document.hidden) {
      raf = 0
      return
    }
    raf = requestAnimationFrame(frame)
    cb(now)
  }
  const start = () => {
    if (!stopped && !document.hidden && raf === 0) raf = requestAnimationFrame(frame)
  }
  const onVisibility = () => {
    if (document.hidden) {
      cancelAnimationFrame(raf)
      raf = 0
    } else {
      start()
    }
  }
  document.addEventListener('visibilitychange', onVisibility)
  start()
  return () => {
    stopped = true
    cancelAnimationFrame(raf)
    raf = 0
    document.removeEventListener('visibilitychange', onVisibility)
  }
}

/**
 * `enabled` の間、`cb` を毎フレーム呼ぶ。`cb` は毎回の描画で差し替わってよい
 * （ref 経由で最新を呼ぶので、ループは作り直さない）。
 */
export function useRaf(cb: (now: number) => void, enabled = true): void {
  const ref = useRef(cb)
  useEffect(() => {
    ref.current = cb
  })
  useEffect(() => {
    if (!enabled) return
    return rafLoop((now) => ref.current(now))
  }, [enabled])
}
