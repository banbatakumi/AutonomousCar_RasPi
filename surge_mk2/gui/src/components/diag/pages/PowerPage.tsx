/**
 * 電源・熱 — 2系統のバッテリーと、MD×3・MCU・RasPi の温度。
 *
 * 現在値は経路図（`PowerFlow`）が全部持つ（電圧・電流・電力・直近の最低電圧・各負荷の温度と上昇）。
 * 同じ数字をカードでもう一度並べない。推移はグラフで見る。
 *
 * 電流はどちらの系統も単方向の計測で、**回生中は 0 に張り付く**（その間の値は信用しない）。
 */
import { ChartGrid } from '../Chart'
import { CH } from '../chartDefs'
import { PowerFlow } from '../PowerFlow'
import type { PageProps } from './types'

const CHARTS = [CH.voltage, CH.battCurrent, CH.temp, CH.motorCurrent]

export function PowerPage({ n, st }: PageProps) {
  if (!n.vs) return null
  return (
    <>
      <PowerFlow n={n} st={st} />
      <ChartGrid defs={CHARTS} />
    </>
  )
}
