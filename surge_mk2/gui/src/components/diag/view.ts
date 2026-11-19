/**
 * 診断タブの表示状態（サブページ・時間幅・一時停止）と、グラフ共通の再描画の刻み。
 *
 * ## 再描画は1本の刻みにまとめる
 *
 * グラフごとに `setInterval` を持つと、枚数ぶんのタイマが別々の瞬間に履歴を読み、
 * カーソルを合わせても隣のグラフと右端がずれる。**1回の刻みで窓（右端と幅）と
 * フラグ列を1度だけ決め**、全グラフがそれを使う。
 *
 * ブラウザのタブが裏にある間と、一時停止中で窓が変わっていない間は何もしない。
 */
import { create } from 'zustand'
import { persist } from 'zustand/middleware'
import { localPersistStorage } from '../../store/persistStorage'
import { HISTORY_SPAN_S, historyNow, readFlags, type FlagTrack } from '../../bus/history'

export const PAGES = ['概要', '電源・熱', '駆動・制御', '通信', 'センサ・Pi', '記録'] as const
export type DiagPage = (typeof PAGES)[number]

export const SPANS = [30, 60, HISTORY_SPAN_S] as const

const REDRAW_HZ = 4

/** `persist` 導入前のキー（ページ名・幅の秒数をそれぞれ生の文字列で持っていた）。読めたら移行する */
function readLegacyView(): Partial<Pick<DiagViewState, 'page' | 'spanS'>> | null {
  try {
    const page = localStorage.getItem('surge.diag.page')
    const span = localStorage.getItem('surge.diag.span')
    if (page == null && span == null) return null
    return { page: (page ?? undefined) as DiagPage | undefined, spanS: span == null ? undefined : Number(span) }
  } catch {
    return null
  }
}

type DiagViewState = {
  page: DiagPage
  spanS: number
  /** 一時停止した時刻（履歴の時刻軸）。null は追従中。**記録は止まらない** */
  frozenAt: number | null
  setPage: (p: DiagPage) => void
  setSpan: (s: number) => void
  togglePause: () => void
}

export const useDiagView = create<DiagViewState>()(
  persist(
    (set, get) => ({
      page: '概要',
      spanS: 60,
      frozenAt: null,
      setPage: (page) => set({ page }),
      setSpan: (spanS) => set({ spanS }),
      togglePause: () => set({ frozenAt: get().frozenAt == null ? historyNow() : null }),
    }),
    {
      // 保存するのはページと幅だけ（一時停止は開き直したら解く）。保存できなくても表示には関係ない
      name: 'surge.diag',
      version: 1,
      storage: localPersistStorage<Pick<DiagViewState, 'page' | 'spanS'>>(readLegacyView),
      partialize: (s) => ({ page: s.page, spanS: s.spanS }),
      // 版 0（旧キー）も今と同じ形。中身の検査は `merge` が毎回する
      migrate: (state) => state as Pick<DiagViewState, 'page' | 'spanS'>,
      // 知らないページ名・幅（版が変わって消えた等）は既定に倒す
      merge: (persisted, current) => {
        const p = (persisted ?? {}) as Partial<DiagViewState>
        return {
          ...current,
          page: (PAGES as readonly string[]).includes(p.page ?? '') ? (p.page as DiagPage) : current.page,
          spanS: (SPANS as readonly number[]).includes(Number(p.spanS)) ? Number(p.spanS) : current.spanS,
        }
      },
    },
  ),
)

/** 1回の刻みで全グラフが共有する窓 */
export type Frame = {
  /** 窓の右端（履歴の時刻軸）[s] */
  end: number
  spanS: number
  flags: FlagTrack
}

type Listener = (f: Frame) => void

const listeners = new Set<Listener>()
let timer: number | null = null
let lastKey = ''
let frame: Frame | null = null

function tick(force = false): void {
  if (document.hidden && !force) return
  const { spanS, frozenAt } = useDiagView.getState()
  // 止めている間は窓が動かないので、幅を変えたとき以外は描き直さない
  const key = frozenAt == null ? '' : `${frozenAt}:${spanS}`
  if (!force && key !== '' && key === lastKey) return
  lastKey = key
  const end = frozenAt ?? historyNow()
  frame = { end, spanS, flags: readFlags(spanS, end) }
  for (const l of listeners) l(frame)
}

/** いまの窓。まだ1度も刻んでいなければ作る（グラフが生成時の初期データに使う） */
export function currentFrame(): Frame {
  if (!frame) tick(true)
  return frame!
}

/** 刻みごとに呼ばれる。戻り値で解除。**最後の購読者が抜けたらタイマも止める** */
export function onFrame(fn: Listener): () => void {
  listeners.add(fn)
  if (timer == null) {
    tick(true)
    timer = window.setInterval(tick, 1000 / REDRAW_HZ)
  }
  return () => {
    listeners.delete(fn)
    if (listeners.size === 0 && timer != null) {
      window.clearInterval(timer)
      timer = null
      frame = null
    }
  }
}

// 幅の切替・一時停止の切替は、次の刻みを待たずに反映する
useDiagView.subscribe((s, prev) => {
  if (s.spanS !== prev.spanS || s.frozenAt !== prev.frozenAt) {
    if (timer != null) tick(true)
  }
})
