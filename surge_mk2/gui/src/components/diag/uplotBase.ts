/**
 * 診断タブの uPlot（時系列グラフ・イベント帯）で共通の部品。
 *
 * - 色は CSS 変数から引く（`styles.css` 冒頭の規約。**ここに色の値を書かない**）
 * - 横軸は全グラフで同じ窓 `[end − span, end]`。目盛りは「何秒前か」
 * - カーソルは `SYNC_KEY` で全グラフ同期（どれか1枚に当てると全部に縦線が出る）
 */
import uPlot from 'uplot'
import { flagSpans, type FlagTrack } from '../../bus/history'
import type { Frame } from './view'

export const SYNC_KEY = 'surge-diag'
/** 縦軸の幅 [px]。**全グラフで揃える**（揃っていないとカーソルの縦線が隣とずれて見える） */
export const Y_AXIS_PX = 52

export type Tone = 'accent' | 'live' | 'ok' | 'warn' | 'bad' | 'dim' | 'fg' | 'line'

let cache: Record<Tone, string> | null = null

/** CSS 変数の実際の色。uPlot は Canvas に描くので `var(--x)` を渡せない */
export function tones(): Record<Tone, string> {
  if (cache) return cache
  const cs = getComputedStyle(document.documentElement)
  const get = (name: string) => cs.getPropertyValue(name).trim()
  cache = {
    accent: get('--accent'),
    live: get('--live'),
    ok: get('--ok'),
    warn: get('--warn'),
    bad: get('--bad'),
    dim: get('--dim'),
    fg: get('--fg'),
    line: get('--line'),
  }
  return cache
}

/** `#rrggbb` に不透明度を付ける */
export function alpha(hex: string, a: number): string {
  const v = Math.round(Math.max(0, Math.min(1, a)) * 255)
  return `${hex}${v.toString(16).padStart(2, '0')}`
}

/** 目盛りの間隔 [s]。窓の右端から数える（「10秒前」「20秒前」に線が来る） */
function xStep(spanS: number): number {
  return spanS <= 30 ? 5 : spanS <= 60 ? 10 : 30
}

/** 横軸（スケールと軸）。`frame()` は刻みごとに差し替わる最新の窓を返す */
export function xScale(frame: () => Frame): uPlot.Scale {
  return {
    time: false,
    range: () => {
      const f = frame()
      return [f.end - f.spanS, f.end]
    },
  }
}

export function xAxis(frame: () => Frame, show = true): uPlot.Axis {
  const c = tones()
  return {
    show,
    stroke: c.dim,
    grid: { stroke: c.line, width: 1 },
    ticks: { stroke: c.line, size: 4 },
    size: 24,
    font: '10px ui-monospace, Menlo, monospace',
    splits: () => {
      const f = frame()
      const step = xStep(f.spanS)
      const out: number[] = []
      for (let s = Math.floor(f.spanS / step) * step; s >= 0; s -= step) out.push(f.end - s)
      return out
    },
    // 絶対時刻（ページを開いてからの秒）に意味は無い。「何秒前か」で読む
    values: (_u, vals) => {
      const end = frame().end
      return vals.map((v) => {
        const d = Math.round(v - end)
        return d === 0 ? '今' : `${d}s`
      })
    },
  }
}

export function cursorOpts(): uPlot.Cursor {
  return {
    drag: { x: false, y: false },
    y: false,
    sync: { key: SYNC_KEY, scales: ['x', null] },
    points: { size: 6 },
  }
}

/** `mask` が立っていた区間を、描画領域いっぱいの縦帯で塗る（`y0`〜`y1` は Canvas の px） */
export function fillSpans(
  u: uPlot,
  track: FlagTrack,
  mask: number,
  color: string,
  y0 = u.bbox.top,
  y1 = u.bbox.top + u.bbox.height,
): void {
  const { ctx, bbox } = u
  const left = bbox.left
  const right = bbox.left + bbox.width
  ctx.fillStyle = color
  for (const [a, b] of flagSpans(track, mask)) {
    const xa = Math.max(left, u.valToPos(a, 'x', true))
    const xb = Math.min(right, u.valToPos(b, 'x', true))
    if (xb < left || xa > right) continue
    // 一瞬の介入（1サンプル）でも見えるよう、最低 2px は塗る
    ctx.fillRect(xa, y0, Math.max(2 * devicePixelRatio, xb - xa), y1 - y0)
  }
}

/** カーソル位置が窓の右端から何秒前か。カーソルが乗っていなければ null */
export function cursorAgo(u: uPlot, end: number): number | null {
  const idx = u.cursor.idx
  if (idx == null) return null
  const x = u.data[0]?.[idx]
  return x == null ? null : end - x
}
