/**
 * 画面上部のステータスバーに数秒だけ出す通知（`ui.notice`）。`store/ui.ts` から分けた。
 *
 * 消すためのタイマーはモジュールに1つだけ持つ（同時に出す通知は1つ。新しい通知が古い通知を上書きする）。
 */
import { useUi } from './ui'

const NOTICE_MS = 8000
let noticeSeq = 0
let noticeTimer: number | undefined

/** 画面上部のステータスバーへ一過性の通知を出す（`NOTICE_MS` で消える）。同じ文面の連続は上書き */
export function notify(text: string, level: 'warn' | 'bad' = 'warn') {
  const id = ++noticeSeq
  window.clearTimeout(noticeTimer)
  useUi.setState({ notice: { text, level, id } })
  noticeTimer = window.setTimeout(() => {
    if (useUi.getState().notice?.id === id) useUi.setState({ notice: null })
  }, NOTICE_MS)
}
