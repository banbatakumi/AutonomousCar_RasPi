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
import { HISTORY_SPAN_S, historyNow, readFlags, type FlagTrack } from '../../bus/history'

export const PAGES = ['概要', '電源・熱', '駆動・制御', '通信', 'センサ・Pi', '記録'] as const
export type DiagPage = (typeof PAGES)[number]

export const SPANS = [30, 60, HISTORY_SPAN_S] as const

const REDRAW_HZ = 4
const KEY_PAGE = 'surge.diag.page'
const KEY_SPAN = 'surge.diag.span'

function loadPage(): DiagPage {
  try {
    const v = localStorage.getItem(KEY_PAGE)
    return (PAGES as readonly string[]).includes(v ?? '') ? (v as DiagPage) : '概要'
  } catch {
    return '概要'
  }
}

function loadSpan(): number {
  try {
    const v = Number(localStorage.getItem(KEY_SPAN))
    return (SPANS as readonly number[]).includes(v) ? v : 60
  } catch {
    return 60
  }
}

function save(key: string, value: string): void {
  try {
    localStorage.setItem(key, value)
  } catch {
    // 保存できなくても表示には関係ない（プライベートウィンドウなど）
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

export const useDiagView = create<DiagViewState>((set, get) => ({
  page: loadPage(),
  spanS: loadSpan(),
  frozenAt: null,
  setPage: (page) => {
    save(KEY_PAGE, page)
    set({ page })
  },
  setSpan: (spanS) => {
    save(KEY_SPAN, String(spanS))
    set({ spanS })
  },
  togglePause: () => set({ frozenAt: get().frozenAt == null ? historyNow() : null }),
}))

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
