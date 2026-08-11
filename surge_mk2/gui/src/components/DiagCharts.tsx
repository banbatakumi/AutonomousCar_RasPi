/**
 * 時系列グラフ（uPlot）— 診断タブ。
 *
 * ## なぜグラフが要るか
 *
 * 現在値のグリッド（`DiagGrid`）は「今おかしいか」しか答えられない。
 * **「いつからおかしいか」「何と同時に起きたか」は形でしか読めない。**
 * 温度がじわじわ上がっているのか、電圧が加速のたびに落ちるのか、
 * RTT が跳ねた瞬間にロスが出ているのか——これらは並べた線でしか分からない。
 *
 * ## 実装の要点
 *
 * - データは `bus/history.ts` のリングバッファ。**常時貯まっている**ので、
 *   異常に気づいてからこのタブを開いても直前の3分が見える
 * - 再描画は 4Hz。人間がグラフの形を読むにはこれで足り、20Hz で回すと
 *   uPlot の再描画が積み上がる
 * - 幅はコンテナ追従（`ResizeObserver`）。uPlot は自分でリサイズしない
 */
import { useEffect, useRef } from 'react'
import uPlot from 'uplot'
import 'uplot/dist/uPlot.min.css'
import { HISTORY_SPAN_S, readHistory, type SeriesKey } from '../bus/history'

const REDRAW_HZ = 4

/** CSS 変数と同じ意味色を使う（`styles.css` 冒頭の規約。装飾で色を増やさない） */
const C = {
  accent: '#4aa8ff',
  live: '#ffc63f',
  ok: '#3ac26a',
  bad: '#f2544b',
  warn: '#e8ab21',
  dim: '#6a7683',
  line: '#232b34',
}

type ChartDef = {
  title: string
  note: string
  keys: SeriesKey[]
  labels: string[]
  colors: string[]
  /** 系列ごとの表示単位（凡例に出す） */
  unit: string
  /**
   * 縦軸の最小の幅。**停車中は値が平らになり、uPlot の既定では軸が 0〜100 になる。**
   * 0 m/s の速度計に「100」の目盛りが付くと、一瞬あり得ない値が出ているように見える。
   * その物理量として意味のある最小レンジをここで決めておく。
   */
  minSpan: number
}

/** 平らな区間でも軸が暴れないレンジ関数。データが無い間も潰れない */
function yRange(minSpan: number) {
  return (_u: uPlot, dmin: number | null, dmax: number | null): [number, number] => {
    if (dmin == null || dmax == null) return [0, minSpan]
    const span = dmax - dmin
    if (span < minSpan) {
      const mid = (dmin + dmax) / 2
      return [mid - minSpan / 2, mid + minSpan / 2]
    }
    const pad = span * 0.1
    return [dmin - pad, dmax + pad]
  }
}

const CHARTS: ChartDef[] = [
  {
    title: '速度',
    note: '指令と実測の開きが速度制御の応答。トルクモード中は指令が途切れる',
    keys: ['speed', 'speedCmd'],
    labels: ['実測', '指令'],
    colors: [C.accent, C.live],
    unit: 'm/s',
    minSpan: 0.6,
  },
  {
    title: '舵角',
    note: 'この差がステアリングの遅れそのもの（推定ではなく実測）',
    keys: ['steer', 'steerCmd'],
    labels: ['実測', '指令'],
    colors: [C.accent, C.live],
    unit: '°',
    minSpan: 20,
  },
  {
    title: '温度',
    note: '通信断（null）は線が切れる。0℃ に落ちて見えることはない',
    keys: ['temp0', 'temp1', 'temp2', 'temp3'],
    labels: ['MD後左', 'MD後右', 'MDステア', 'MCU'],
    colors: [C.accent, C.live, C.ok, C.dim],
    unit: '℃',
    minSpan: 15,
  },
  {
    title: '電源・モータ電流',
    note: '加速のたびに電圧が落ちるなら内部抵抗。電流と見比べる',
    keys: ['battDrive', 'currDrive', 'motorRL', 'motorRR', 'motorST'],
    labels: ['駆動電圧V', '駆動電流A', '後左A', '後右A', 'ステアA'],
    colors: [C.warn, C.accent, C.ok, C.live, C.dim],
    unit: 'V / A',
    minSpan: 4,
  },
  {
    title: '加速度',
    note: '前後 G と横 G の生値 [m/s²]。ラジコンの G メータと同じ値',
    keys: ['accelX', 'accelY'],
    labels: ['前後', '左右'],
    colors: [C.accent, C.live],
    unit: 'm/s²',
    minSpan: 4,
  },
  {
    title: 'リンク',
    note: 'CRC とロスは増分（累積のままでは右上がりの直線になって読めない）',
    keys: ['rttMs', 'rxHz', 'crcDelta', 'lossDelta'],
    labels: ['UART往復ms', '受信Hz', 'CRC', 'loss'],
    colors: [C.accent, C.dim, C.bad, C.warn],
    unit: 'ms / Hz / 件',
    minSpan: 25,
  },
]

export function DiagCharts() {
  return (
    <div className="charts">
      {CHARTS.map((c) => (
        <Chart key={c.title} def={c} />
      ))}
    </div>
  )
}

function Chart({ def }: { def: ChartDef }) {
  const host = useRef<HTMLDivElement>(null)

  useEffect(() => {
    const el = host.current
    if (!el) return

    const opts: uPlot.Options = {
      width: el.clientWidth || 400,
      height: 150,
      // 凡例は自前のラベルで出す（uPlot の既定は明るいテーマ向けで浮く）
      legend: { show: false },
      cursor: { drag: { x: false, y: false } },
      scales: { y: { range: yRange(def.minSpan) } },
      axes: [
        {
          stroke: C.dim,
          grid: { stroke: C.line, width: 1 },
          ticks: { stroke: C.line },
          // 経過時間は「何秒前か」で読みたい。絶対時刻（ページを開いてからの秒）に意味は無い
          values: (_u, vals) => {
            const last = vals.length ? vals[vals.length - 1] : 0
            return vals.map((v) => `${Math.round(v - last)}s`)
          },
        },
        {
          stroke: C.dim,
          grid: { stroke: C.line, width: 1 },
          ticks: { stroke: C.line },
          size: 46,
        },
      ],
      series: [
        {},
        ...def.keys.map((_, i) => ({
          label: def.labels[i],
          stroke: def.colors[i],
          width: 1.4,
          points: { show: false },
          // NaN を線でつながない。**欠測は欠測として見せる**
          spanGaps: false,
        })),
      ],
    }

    const plot = new uPlot(opts, readHistory(def.keys) as uPlot.AlignedData, el)

    const timer = window.setInterval(() => {
      plot.setData(readHistory(def.keys) as uPlot.AlignedData)
    }, 1000 / REDRAW_HZ)

    const ro = new ResizeObserver(() => {
      plot.setSize({ width: el.clientWidth, height: 150 })
    })
    ro.observe(el)

    return () => {
      window.clearInterval(timer)
      ro.disconnect()
      plot.destroy()
    }
  }, [def])

  return (
    <section className="chart">
      <div className="chart-head">
        <h4>{def.title}</h4>
        <span className="unit">{def.unit}</span>
        <div className="chart-legend">
          {def.labels.map((l, i) => (
            <span key={l}>
              <i style={{ background: def.colors[i] }} />
              {l}
            </span>
          ))}
        </div>
        <span className="dim">直近 {HISTORY_SPAN_S}s</span>
      </div>
      <div ref={host} className="chart-host" />
      <span className="note">{def.note}</span>
    </section>
  )
}
