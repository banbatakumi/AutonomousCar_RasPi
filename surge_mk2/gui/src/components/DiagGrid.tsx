/**
 * 診断の現在値グリッド — 温度・電源・モータ・スリップ・リンク・MD バス・時刻同期。
 *
 * 2箇所から使う。
 *
 *   - `DiagStrip`（自動運転ビュー）— 平常時は畳み、閾値超えで自動展開
 *   - `DiagView`（診断タブ）— 常時展開。時系列グラフと並べる
 *
 * **閾値を超えたかどうかの判定は持たない。** 自動展開の条件は `DiagStrip` の
 * 1箇所に集めてある（散らすと「なぜ開いたのか」が追えなくなる）。ここは表示だけ。
 */
import type { Numbers } from '../bus/live'
import { battLevel, ms, tempLevel } from '../format'

export const TEMP_LABEL = ['MD後左', 'MD後右', 'MDステア', 'MCU']

export function DiagGrid({ n }: { n: Numbers }) {
  const vs = n.vs
  const link = n.link
  if (!vs) return null

  return (
    <div className="diag-grid">
      <section>
        <h4>温度</h4>
        {vs.temp.map((t, i) => (
          <div className="kv" key={i}>
            <span>{TEMP_LABEL[i]}</span>
            <b className={`lv-${tempLevel(t)}`}>{t == null ? '通信断' : `${t}℃`}</b>
          </div>
        ))}
      </section>

      <section>
        <h4>電源（2系統）</h4>
        <div className="kv">
          <span>駆動</span>
          <b className={`lv-${battLevel(vs.batt_voltage[0])}`}>{vs.batt_voltage[0].toFixed(2)}V</b>
          <b>{vs.batt_current[0].toFixed(2)}A</b>
        </div>
        <div className="kv">
          <span>信号</span>
          <b className={`lv-${battLevel(vs.batt_voltage[1])}`}>{vs.batt_voltage[1].toFixed(2)}V</b>
          <b>{vs.batt_current[1].toFixed(2)}A</b>
        </div>
      </section>

      <section>
        <h4>モータ</h4>
        {['後左', '後右', 'ステア'].map((lbl, i) => (
          <div className="kv" key={lbl}>
            <span>{lbl}</span>
            <b>{vs.motor_current[i].toFixed(2)}A</b>
            {i < 2 && <b className="dim">{vs.torque_cmd[i].toFixed(3)}N·m</b>}
          </div>
        ))}
      </section>

      <section>
        <h4>スリップ</h4>
        <div className="kv">
          <span>前 FL/FR</span>
          <b>{vs.slip_front.map((v) => v.toFixed(2)).join(' / ')}</b>
        </div>
        <div className="kv">
          <span>後 RL/RR</span>
          <b>{vs.slip_rear.map((v) => v.toFixed(2)).join(' / ')}</b>
        </div>
      </section>

      <section>
        <h4>リンク（Pi 側 / STM32 側）</h4>
        <div className="kv">
          <span>frame_ok</span>
          <b>{link?.rx?.frame_ok ?? 0}</b>
          <b className="dim">{link?.stm_rx?.rx_frame_ok ?? '—'}</b>
        </div>
        <div className="kv">
          <span>crc_error</span>
          <b className={(link?.rx?.crc_error ?? 0) > 0 ? 'lv-bad' : ''}>{link?.rx?.crc_error ?? 0}</b>
          <b className="dim">{link?.stm_rx?.rx_crc_error ?? '—'}</b>
        </div>
        <div className="kv">
          <span>loss / tx_drop</span>
          <b>{link?.rx?.packet_loss ?? 0}</b>
          <b className="dim">{link?.stm_rx?.tx_drop ?? '—'}</b>
        </div>
      </section>

      <section>
        <h4>MD バス（STM32 ⇄ MD・累積）</h4>
        {['後左', '後右', 'ステア'].map((lbl, i) => {
          const ok = link?.md_rx_count?.[i]
          const ng = link?.md_rx_error?.[i]
          if (ok == null || ng == null) return null
          const pct = ok + ng > 0 ? (ng / (ok + ng)) * 100 : null
          return (
            <div className="kv" key={lbl}>
              <span>{lbl}</span>
              <b className={ok === 0 ? 'lv-bad' : ''}>{ok}</b>
              <b className={pct != null && pct > 5 ? 'lv-warn' : 'dim'}>
                {pct == null ? '—' : `err ${pct.toFixed(1)}%`}
              </b>
            </div>
          )
        })}
      </section>

      <section>
        <h4>時刻同期・IMU</h4>
        <div className="kv">
          <span>offset</span>
          <b>{link?.sync_offset_ns == null ? '—' : `${(link.sync_offset_ns / 1000).toFixed(1)}μs`}</b>
        </div>
        <div className="kv">
          <span>drift</span>
          <b>{link?.sync_drift_ppm == null ? '—' : `${link.sync_drift_ppm.toFixed(1)}ppm`}</b>
        </div>
        <div className="kv">
          <span>片道遅延</span>
          <b>{ms(link?.sync_delay_ns == null ? null : link.sync_delay_ns / 1e6, 2)}</b>
        </div>
        <div className="kv">
          <span>pitch / roll</span>
          <b>
            {((vs.pitch * 180) / Math.PI).toFixed(1)}° / {((vs.roll * 180) / Math.PI).toFixed(1)}°
          </b>
        </div>
      </section>
    </div>
  )
}
