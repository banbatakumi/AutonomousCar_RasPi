/**
 * イベント帯 — フラグの履歴を時間軸のレーンで見せる（全サブページ共通）。
 *
 * 現在値のピル（StatusBar）は「今」しか言わない。**いつ起きて、何と同時だったか**は
 * 並べた帯でしか読めない。グラフと同じ窓・同じカーソルを使うので、帯の上に
 * カーソルを当てれば下のグラフ全部の同じ時刻に縦線が出る。
 *
 * 中身は線を1本も描かない uPlot（横軸とカーソル同期をグラフと同じ実装で済ませるため）。
 */
import { useEffect, useRef } from 'react'
import uPlot from 'uplot'
import 'uplot/dist/uPlot.min.css'
import { EV, readHistory } from '../../bus/history'
import { Y_AXIS_PX, alpha, cursorAgo, cursorOpts, fillSpans, tones, xAxis, xScale, type Tone } from './uplotBase'
import { currentFrame, onFrame, type Frame } from './view'

/** 上から順に並ぶ。色は意味（StatusBar のピルと同じ割り当て） */
const LANES: { label: string; mask: number; tone: Tone }[] = [
  { label: 'ARM', mask: EV.ARM, tone: 'warn' },
  { label: 'AUTO', mask: EV.AUTO, tone: 'accent' },
  { label: 'E-STOP', mask: EV.ESTOP, tone: 'bad' },
  { label: 'フォルト', mask: EV.FAULT, tone: 'bad' },
  // 途絶（届いていない区間）と STM32 側の COMMAND 途絶も「通信がおかしい」として同じ段に出す
  { label: '通信', mask: EV.LINK | EV.UART | EV.GAP, tone: 'bad' },
  { label: 'TC', mask: EV.TC | EV.LIFT, tone: 'live' },
  { label: 'ABS', mask: EV.ABS, tone: 'live' },
  { label: 'TV', mask: EV.TV, tone: 'live' },
  { label: '自動停止', mask: EV.AUTOSTOP, tone: 'warn' },
]
const LANE_PX = 15
const HEIGHT = LANES.length * LANE_PX + 28

export function EventTimeline() {
  const host = useRef<HTMLDivElement>(null)
  const when = useRef<HTMLSpanElement>(null)

  useEffect(() => {
    const el = host.current
    if (!el) return
    const c = tones()
    const n = LANES.length
    let frame: Frame = currentFrame()

    const drawLanes = (u: uPlot) => {
      const { ctx, bbox } = u
      ctx.save()
      ctx.beginPath()
      ctx.rect(bbox.left, bbox.top, bbox.width, bbox.height)
      ctx.clip()
      const gap = 2 * devicePixelRatio
      for (let i = 0; i < n; i++) {
        const lane = LANES[i]!
        const y0 = u.valToPos(n - i, 'y', true) + gap
        const y1 = u.valToPos(n - i - 1, 'y', true) - gap
        ctx.fillStyle = alpha(c.line, 0.45)
        ctx.fillRect(bbox.left, y0, bbox.width, y1 - y0)
        fillSpans(u, frame.flags, lane.mask, c[lane.tone], y0, y1)
      }
      ctx.restore()
    }

    const writeWhen = (u: uPlot) => {
      const ago = cursorAgo(u, frame.end)
      if (when.current) when.current.textContent = ago == null ? '' : `${ago.toFixed(1)}秒前`
    }

    const opts: uPlot.Options = {
      width: el.clientWidth || 600,
      height: HEIGHT,
      legend: { show: false },
      padding: [2, 10, 0, 0],
      cursor: { ...cursorOpts(), points: { show: false } },
      scales: { x: xScale(() => frame), y: { range: () => [0, n] } },
      axes: [
        xAxis(() => frame),
        {
          stroke: c.dim,
          grid: { show: false },
          ticks: { show: false },
          size: Y_AXIS_PX,
          font: '10px ui-monospace, Menlo, monospace',
          splits: () => LANES.map((_, i) => n - i - 0.5),
          values: (_u, vals) => vals.map((v) => LANES[Math.round(n - v - 0.5)]?.label ?? ''),
        },
      ],
      // 線は描かない。横軸の値（カーソルが時刻を引くのに要る）だけ速度の系列から借りる
      series: [{}, { show: false }],
      hooks: { draw: [drawLanes], setCursor: [writeWhen] },
    }

    const read = () => readHistory(['speed'], frame.spanS, frame.end) as uPlot.AlignedData
    const plot = new uPlot(opts, read(), el)

    const off = onFrame((f) => {
      frame = f
      plot.setData(read())
      writeWhen(plot)
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
  }, [])

  return (
    <section className="dg-timeline">
      <span ref={when} className="dg-when" />
      <div ref={host} className="dg-chart-host" />
    </section>
  )
}
