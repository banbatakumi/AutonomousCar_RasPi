import { useEffect, useState } from 'react'
import emblemUrl from '../assets/emblem-red.svg'

/** エンブレムのフェードイン → 静止 → 全体フェードアウトの時間（CSS の transition と合わせる） */
const EMBLEM_FADE_MS = 700
const HOLD_MS = 1000
const FADE_MS = 900

/**
 * 起動時に一度だけ出るエンブレム。**外枠（黒背景）はマウント直後から不透明**で
 * 本体画面ごと覆い続け、その中でエンブレム画像だけがフェードインする
 * （外枠までフェードインさせると裏の本体が一瞬透けて見える）。
 * 静止後は外枠ごとフェードアウトして本体を現す。操作をブロックしないよう `pointer-events: none`。
 */
export function SplashEmblem({ onDone }: { onDone: () => void }) {
  const [emblemVisible, setEmblemVisible] = useState(false)
  const [fadeOut, setFadeOut] = useState(false)

  useEffect(() => {
    // 初期 opacity:0 のまま transition させるため、マウント直後の1フレームを空ける
    const show = requestAnimationFrame(() => setEmblemVisible(true))
    const hide = window.setTimeout(() => setFadeOut(true), EMBLEM_FADE_MS + HOLD_MS)
    const done = window.setTimeout(onDone, EMBLEM_FADE_MS + HOLD_MS + FADE_MS)
    return () => {
      cancelAnimationFrame(show)
      window.clearTimeout(hide)
      window.clearTimeout(done)
    }
  }, [onDone])

  return (
    <div className={`splash-emblem${fadeOut ? ' fade-out' : ''}`} aria-hidden="true">
      <img className={emblemVisible ? 'show' : ''} src={emblemUrl} alt="" />
    </div>
  )
}
