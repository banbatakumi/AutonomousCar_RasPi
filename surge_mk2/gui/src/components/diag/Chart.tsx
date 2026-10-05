/**
 * 時系列グラフ1枚（uPlot）。
 *
 * - データは `bus/history.ts` のリングバッファ。**常時貯まっている**ので、
 *   異常に気づいてから診断タブを開いても直前の3分が見える
 * - 再描画は `view.ts` の共通の刻み（4Hz）。窓もカーソルも全グラフで共有する
 * - 凡例の数値は、カーソルが乗っていればその時刻の値、乗っていなければ最新値。
 *   **React の state を通さず DOM を直接書き換える**（カーソル移動のたびに再レンダリングしない）
 * - 幅はコンテナ追従（`ResizeObserver`）。uPlot は自分でリサイズしない
 */
import { useEffect, useRef } from 'react'
import uPlot from 'uplot'
import 'uplot/dist/uPlot.min.css'
import { EV, readHistory } from '../../bus/history'
import type { ChartDef } from './chartDefs'
import { Y_AXIS_PX, alpha, cursorAgo, cursorOpts, fillSpans, tones, xAxis, xScale } from './uplotBase'
import { currentFrame, onFrame, type Frame } from './view'

const HEIGHT = 150
/** どのグラフにも重ねる異常系（その瞬間に何が起きていたかを、どのグラフを見ていても読めるように） */
const TROUBLE = EV.ESTOP | EV.FAULT | EV.LINK | EV.UART | EV.GAP

/** 平らな区間でも軸が暴れないレンジ関数。データが無い間も潰れない */
function yRange(def: ChartDef) {
  return (_u: uPlot, dmin: number | null, dmax: number | null): [number, number] => {
    let lo = dmin
    let hi = dmax
    const floor = def.floor
    for (const v of floor == null ? (def.include ?? []) : [...(def.include ?? []), floor]) {
      lo = lo == null ? v : Math.min(lo, v)
      hi = hi == null ? v : Math.max(hi, v)
    }
    if (lo == null || hi == null) return [0, def.minSpan]
    const span = hi - lo
    let a: number
    let b: number
    if (span < def.minSpan) {
      const mid = (lo + hi) / 2
      a = mid - def.minSpan / 2
      b = mid + def.minSpan / 2
    } else {
      const pad = span * 0.08
      a = lo - pad
      b = hi + pad
    }
    // 下限より下へはみ出したぶんは上へずらす（幅は保つ）
    if (floor != null && a < floor) {
      b += floor - a
      a = floor
    }
    return [a, b]
  }
}

export function Chart({ def }: { def: ChartDef }) {
  const host = useRef<HTMLDivElement>(null)
  const values = useRef<(HTMLElement | null)[]>([])
  const when = useRef<HTMLSpanElement>(null)

  useEffect(() => {
    const el = host.current
    if (!el) return
    const c = tones()
    const digits = def.digits ?? 2
    const keys = def.series.map((s) => s.key)
    let frame: Frame = currentFrame()

    /** 凡例の数値を書く。カーソルが無ければ、各系列の最後の有効値 */
    const writeLegend = (u: uPlot) => {
      const idx = u.cursor.idx
      for (let s = 0; s < keys.length; s++) {
        const col = u.data[s + 1]
        let v: number | null | undefined = null
        if (col) {
          if (idx != null) v = col[idx]
          else for (let k = col.length - 1; k >= 0 && k >= col.length - 30 && v == null; k--) v = col[k]
        }
        const cell = values.current[s]
        if (cell) cell.textContent = v == null ? '—' : v.toFixed(digits)
      }
      const ago = cursorAgo(u, frame.end)
      if (when.current) when.current.textContent = ago == null ? '' : `${ago.toFixed(1)}秒前`
    }

    /** 系列の下に、フラグの帯としきい値の線を描く */
    const drawUnder = (u: uPlot) => {
      const { ctx, bbox } = u
      ctx.save()
      ctx.beginPath()
      ctx.rect(bbox.left, bbox.top, bbox.width, bbox.height)
      ctx.clip()
      if (def.bands) fillSpans(u, frame.flags, def.bands, alpha(c.live, 0.13))
      fillSpans(u, frame.flags, TROUBLE, alpha(c.bad, 0.16))
      ctx.lineWidth = devicePixelRatio
      ctx.font = `${10 * devicePixelRatio}px ui-monospace, Menlo, monospace`
      ctx.textBaseline = 'bottom'
      // uPlot は軸の目盛りを右寄せで描いたままにしている。戻さないと文字が左へはみ出して切れる
      ctx.textAlign = 'left'
      for (const ln of def.lines ?? []) {
        const y = Math.round(u.valToPos(ln.v, 'y', true))
        if (y < bbox.top || y > bbox.top + bbox.height) continue
        ctx.strokeStyle = alpha(c[ln.tone], 0.75)
        ctx.setLineDash([4 * devicePixelRatio, 4 * devicePixelRatio])
        ctx.beginPath()
        ctx.moveTo(bbox.left, y)
        ctx.lineTo(bbox.left + bbox.width, y)
        ctx.stroke()
        ctx.fillStyle = alpha(c[ln.tone], 0.9)
        ctx.fillText(ln.label, bbox.left + 4 * devicePixelRatio, y - 2 * devicePixelRatio)
      }
      ctx.restore()
    }

    const opts: uPlot.Options = {
      width: el.clientWidth || 400,
      height: HEIGHT,
      // 凡例は見出しに自前で出す（uPlot の既定は明るいテーマ向けで浮く）
      legend: { show: false },
      padding: [8, 10, 0, 0],
      cursor: cursorOpts(),
      scales: { x: xScale(() => frame), y: { range: yRange(def) } },
      axes: [
        xAxis(() => frame),
        {
          stroke: c.dim,
          grid: { stroke: c.line, width: 1 },
          ticks: { stroke: c.line, size: 4 },
          size: Y_AXIS_PX,
          font: '10px ui-monospace, Menlo, monospace',
        },
      ],
      series: [
        {},
        ...def.series.map((s) => ({
          label: s.label,
          stroke: c[s.tone],
          width: 1.4,
          dash: s.dash ? [5, 4] : undefined,
          points: { show: false },
          // 欠測を線でつながない。**欠測は欠測として見せる**
          spanGaps: false,
        })),
      ],
      hooks: { drawAxes: [drawUnder], setCursor: [writeLegend] },
    }

    const read = () => readHistory(keys, frame.spanS, frame.end) as uPlot.AlignedData
    const plot = new uPlot(opts, read(), el)
    writeLegend(plot)

    const off = onFrame((f) => {
      frame = f
      plot.setData(read())
      writeLegend(plot)
    })
    const ro = new ResizeObserver(() => {
      if (el.clientWidth > 0) plot.setSize({ width: el.clientWidth, height: HEIGHT })
    })
    ro.observe(el)

    return () => {
      off()
      ro.disconnect()
      plot.destroy()
    }
  }, [def])

  return (
    <section className="dg-chart">
      <header>
        <h4>{def.title}</h4>
        <span className="dg-unit">{def.unit}</span>
        <span ref={when} className="dg-when" />
        <div className="dg-legend">
          {def.series.map((s, i) => (
            <span key={s.key}>
              <i className={`dg-swatch tone-${s.tone}${s.dash ? ' dash' : ''}`} />
              {s.label}
              <b
                ref={(node) => {
                  values.current[i] = node
                }}
              >
                —
              </b>
            </span>
          ))}
        </div>
      </header>
      <div ref={host} className="dg-chart-host" />
    </section>
  )
}

export function ChartGrid({ defs }: { defs: ChartDef[] }) {
  return (
    <div className="dg-charts">
      {defs.map((d) => (
        <Chart key={d.title} def={d} />
      ))}
    </div>
  )
}
