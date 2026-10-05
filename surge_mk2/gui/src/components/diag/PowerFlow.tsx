/**
 * 電源の経路図 — 2系統のバッテリーから、どの負荷へどれだけ流れているか。
 *
 *   駆動系 → モータドライバ×3（後左・後右・ステア）   過電流 5.0A でハード遮断（ラッチ）
 *   信号系 → STM32・RasPi・LiDAR（とライト類）        過電流 3.0A
 *
 * 系統の分け方は `docs/stm32_interface.md` §8.3。**片方だけ落ちる**（駆動を切っても
 * Pi と STM32 は生きている）ので、2本を並べて出す。
 *
 * このページの現在値はこの図だけが持つ（カードで同じ数字を繰り返さない）。図に無い情報のうち
 * 走行前に効くものだけを箱に足してある: バッテリーは直近の最低電圧（負荷を掛けたときの落ち込み）、
 * 温度は上がっている最中のときだけ上昇率。
 *
 * 電流を測っているのはバッテリーの出口（2系統）と各モータだけ。信号系の負荷ごとの
 * 電流は測っていないので、そちらの枝には数字を書かず、箱に状態と温度を出す。
 */
import { HISTORY_SPAN_S, MD_LABEL, seriesStats, type SeriesKey } from '../../bus/history'
import type { Numbers } from '../../bus/live'
import {
  DRIVE_CUTOFF_A,
  SIGNAL_CUTOFF_A,
  battLevel,
  cpuLevel,
  currentLevel,
  num,
  scanAgeLevel,
  tempLevel,
  worst,
} from '../../format'
import type { ControlStatus } from '../../types'
import type { Tint } from './Card'
import { faultText, mdText } from './checks'
import { Branch, Fan, Hop, Node } from './Route'

/** 温度の上昇率を出す窓 [s]。短いと 1℃ 刻みの量子化で暴れる */
const RATE_WINDOW_S = 60
/** これ以上の速さで上がっているときだけ箱に出す [℃/分] */
const RATE_SHOW = 2

/** 上がっている最中なら「 ↑3.2/分」。窓がまだ短い間は出さない（数秒ぶんの差を60倍すると嘘の数字になる） */
function rising(key: SeriesKey): string {
  const s = seriesStats(key, RATE_WINDOW_S)
  if (!s || s.dtS < RATE_WINDOW_S / 2) return ''
  const rate = ((s.last - s.first) / s.dtS) * 60
  return rate >= RATE_SHOW ? ` ↑${rate.toFixed(1)}/分` : ''
}

/** バッテリーの箱の補足: フォルトがあればそれ、無ければ直近の最低電圧 */
function battSub(key: SeriesKey, faults: string[]): string {
  if (faults.length) return faults.join('・')
  const low = seriesStats(key, HISTORY_SPAN_S)
  return `最低 ${num(low?.min, 2)} V（${HISTORY_SPAN_S}秒）`
}

const MD_TEMP_KEY = ['temp0', 'temp1', 'temp2'] as const

export function PowerFlow({ n, st }: { n: Numbers; st: ControlStatus | null }) {
  const vs = n.vs
  if (!vs) return null
  const [vDrive, vSignal] = vs.batt_voltage
  const [aDrive, aSignal] = vs.batt_current

  const driveFault = vs.faults.filter((f) => f.startsWith('drive_')).map(faultText)
  const signalFault = vs.faults.filter((f) => f.startsWith('signal_')).map(faultText)

  // 駆動電源は ARM で入り、DISARM で切れる。切れている間の「0 A」は異常ではない
  const driveState = vs.drive_power_locked ? 'ラッチ遮断' : vs.armed ? '通電中' : 'OFF（DISARM）'
  const driveLine: Tint = vs.drive_power_locked
    ? 'bad'
    : !vs.armed
      ? 'na'
      : worst(currentLevel(aDrive, DRIVE_CUTOFF_A), driveFault.length ? 'bad' : 'ok')
  const signalLine: Tint = worst(currentLevel(aSignal, SIGNAL_CUTOFF_A), signalFault.length ? 'bad' : 'ok')

  const pi = st?.pi
  const piLevel = worst(n.piTempC == null ? 'ok' : tempLevel(n.piTempC), cpuLevel(pi?.cpu_pct))
  const lidarLevel: Tint = !vs.lidar_ok ? 'bad' : scanAgeLevel(n.scanAgeMs) === 'ok' ? 'ok' : 'warn'

  return (
    <div className="dg-flows">
      <div className="dg-route">
        <Node
          name="駆動バッテリー"
          value={num(vDrive, 2)}
          unit="V"
          sub={battSub('battDrive', driveFault)}
          level={worst(battLevel(vDrive), driveFault.length ? 'bad' : 'ok')}
        />
        <Hop level={driveLine} lines={[`${num(aDrive, 2)} / ${num(DRIVE_CUTOFF_A, 1)} A・${num(vDrive * aDrive, 1)} W`, driveState]} />
        <Fan>
          {[0, 1, 2].map((i) => {
            const k = i as 0 | 1 | 2
            const m = mdText(vs.md_status[k])
            const t = vs.temp[k]
            const level = worst(m.level, tempLevel(t))
            return (
              <Branch key={i} level={!vs.armed && level === 'ok' ? 'na' : level} lines={[`${num(vs.motor_current[k], 2)} A`]}>
                <Node
                  name={MD_LABEL[k]}
                  sub={`${m.text}・${t == null ? '温度不明' : `${num(t, 0)}℃${rising(MD_TEMP_KEY[k])}`}`}
                  level={level === 'ok' ? undefined : level}
                />
              </Branch>
            )
          })}
        </Fan>
      </div>

      <div className="dg-route">
        <Node
          name="信号バッテリー"
          value={num(vSignal, 2)}
          unit="V"
          sub={battSub('battSignal', signalFault)}
          level={worst(battLevel(vSignal), signalFault.length ? 'bad' : 'ok')}
        />
        <Hop level={signalLine} lines={[`${num(aSignal, 2)} / ${num(SIGNAL_CUTOFF_A, 1)} A・${num(vSignal * aSignal, 1)} W`, '常時通電']} />
        <Fan>
          <Branch level={tempLevel(vs.temp[3])}>
            <Node
              name="STM32"
              sub={`MCU ${num(vs.temp[3], 0)}℃${rising('temp3')}`}
              level={tempLevel(vs.temp[3]) === 'ok' ? undefined : tempLevel(vs.temp[3])}
            />
          </Branch>
          <Branch level={piLevel}>
            <Node
              name="RasPi"
              sub={
                n.piTempC == null
                  ? '本体情報なし'
                  : `${num(n.piTempC, 0)}℃${rising('tempPi')}${pi?.available ? `・CPU ${num(pi.cpu_pct, 0)}%` : ''}`
              }
              level={piLevel === 'ok' ? undefined : piLevel}
            />
          </Branch>
          <Branch level={lidarLevel}>
            <Node
              name="LiDAR"
              sub={vs.lidar_ok ? `${num(n.scanHz, 1)} Hz` : '異常'}
              level={lidarLevel === 'ok' ? undefined : lidarLevel}
            />
          </Branch>
        </Fan>
      </div>
    </div>
  )
}
