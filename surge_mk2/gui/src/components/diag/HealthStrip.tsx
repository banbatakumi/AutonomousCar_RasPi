/**
 * 常設サマリ — 全系統の状態を1行で（どのサブページを開いていても見える）。
 *
 * タイル1枚 = 1系統。代表値を1つ大きく、補足を1つ小さく出し、状態色は左の罫線と数値に付ける。
 * 押すと詳しい数字があるサブページへ飛ぶ。
 */
import type { Numbers } from '../../bus/live'
import {
  battLevel,
  cpuLevel,
  healthLevel,
  memLevel,
  nodeLevel,
  num,
  rttLevel,
  tempLevel,
  worst,
  type Level,
} from '../../format'
import { MD, MD_LABEL } from '../../bus/history'
import type { ControlStatus } from '../../types'
import type { Tint } from './Card'
import { throttledText, type Verdict } from './checks'
import { useDiagView, type DiagPage } from './view'

type Tile = { title: string; value: string; unit?: string; sub: string; level: Tint; page: DiagPage }

function tiles(n: Numbers, st: ControlStatus | null, v: Verdict): Tile[] {
  const { vs, link } = n
  const out: Tile[] = [
    {
      title: '走行可否',
      value: v.text,
      sub: v.bad || v.warn ? `異常 ${v.bad}・注意 ${v.warn}` : '全項目 正常',
      level: v.level,
      page: '概要',
    },
  ]
  if (!vs) return out

  out.push({
    title: '駆動電源',
    value: num(vs.batt_voltage[0], 2),
    unit: 'V',
    sub: `${num(vs.batt_current[0], 2)} A`,
    level: battLevel(vs.batt_voltage[0]),
    page: '電源・熱',
  })
  out.push({
    title: '信号電源',
    value: num(vs.batt_voltage[1], 2),
    unit: 'V',
    sub: `${num(vs.batt_current[1], 2)} A`,
    level: battLevel(vs.batt_voltage[1]),
    page: '電源・熱',
  })

  // いちばん熱いところを出す。MD が無言（null）なら、熱いかどうか以前に信用できない
  const temps: [string, number | null][] = [
    [MD_LABEL[0], vs.temp[0]],
    [MD_LABEL[1], vs.temp[1]],
    [MD_LABEL[2], vs.temp[2]],
    ['MCU', vs.temp[3]],
  ]
  const silent = temps.filter(([, t]) => t == null)
  const hottest = temps.reduce<[string, number | null]>((a, b) => ((b[1] ?? -Infinity) > (a[1] ?? -Infinity) ? b : a), ['', null])
  out.push({
    title: '温度',
    value: num(hottest[1], 0),
    unit: '℃',
    sub: silent.length ? `${silent.map(([l]) => l).join('・')} 通信断` : `最高 ${hottest[0]}`,
    level: worst(...temps.map(([, t]) => tempLevel(t))),
    page: '電源・熱',
  })

  out.push({
    title: 'リンク',
    value: n.stale ? '途絶' : (link?.health ?? '—'),
    sub: `${num(link?.cmd_rtt_ms, 0)} ms・${num(n.rxHz, 0)} Hz`,
    level: n.stale ? 'bad' : worst(healthLevel(link?.health), rttLevel(link?.cmd_rtt_ms) === 'bad' ? 'warn' : 'ok'),
    page: '通信',
  })

  const mdOk = vs.md_status.filter((s) => s & MD.COMM_OK).length
  const mdBad = vs.md_status.some((s) => !(s & MD.COMM_OK) || s & (MD.OVERHEAT | MD.OVERCURRENT))
  out.push({
    title: 'STM32・MD',
    value: `${mdOk}/3`,
    sub: link?.protocol_match === false ? 'プロトコル不一致' : vs.faults.length ? 'フォルトあり' : 'MD 応答',
    level: mdBad || link?.protocol_match === false || vs.faults.length ? 'bad' : 'ok',
    page: '通信',
  })

  const sensorBad = [!vs.imu_ok && 'IMU', !vs.lidar_ok && 'LiDAR'].filter(Boolean) as string[]
  out.push({
    title: 'センサ',
    value: sensorBad.length ? sensorBad.join('・') : '正常',
    sub: `LiDAR ${num(n.scanHz, 1)} Hz`,
    level: !vs.imu_ok ? 'bad' : !vs.lidar_ok ? 'warn' : 'ok',
    page: 'センサ・Pi',
  })

  const pi = st?.pi
  const th = throttledText(pi?.throttled)
  const piLevel: Level = worst(
    n.piTempC == null ? 'ok' : tempLevel(n.piTempC),
    cpuLevel(pi?.cpu_pct),
    memLevel(pi?.mem_used_pct),
    th.level === 'na' ? 'ok' : th.level,
  )
  out.push({
    title: 'RasPi',
    value: num(n.piTempC, 0),
    unit: '℃',
    sub: pi?.available ? `CPU ${num(pi.cpu_pct, 0)}%・メモリ ${num(pi.mem_used_pct, 0)}%` : '本体情報なし',
    level: n.piTempC == null && !pi?.available ? 'na' : piLevel,
    page: 'センサ・Pi',
  })

  const nodes = st?.nodes ?? []
  const alive = nodes.filter((x) => nodeLevel(x.age_ms) === 'ok').length
  out.push({
    title: 'ノード',
    value: nodes.length ? `${alive}/${nodes.length}` : '—',
    sub: nodes.length ? (alive === nodes.length ? '全て稼働' : '無応答あり') : '情報なし',
    level: nodes.length === 0 ? 'na' : alive === nodes.length ? 'ok' : 'warn',
    page: 'センサ・Pi',
  })
  return out
}

export function HealthStrip({ n, st, v }: { n: Numbers; st: ControlStatus | null; v: Verdict }) {
  const setPage = useDiagView((s) => s.setPage)
  return (
    <div className="dg-strip">
      {tiles(n, st, v).map((t) => (
        <button key={t.title} className={`dg-tile is-${t.level}`} onClick={() => setPage(t.page)}>
          <span className="dg-tile-title">{t.title}</span>
          <span className="dg-tile-value">
            {t.value}
            {t.unit && <small>{t.unit}</small>}
          </span>
          <span className="dg-tile-sub">{t.sub}</span>
        </button>
      ))}
    </div>
  )
}
