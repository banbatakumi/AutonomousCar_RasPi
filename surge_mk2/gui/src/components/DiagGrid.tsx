/**
 * 診断の現在値グリッド — 温度・電源・モータ・スリップ・リンク・MD バス・時刻同期。
 *
 * `DiagView`（診断タブ）から使う。ラジコン・自動運転の両走行画面には出さない
 * （2026-08-20: 自動運転ビューにあった `DiagStrip` は廃止し、診断タブに一本化した。
 * 走行中の異常は `StatusBar` が全タブ共通で伝える）。
 */
import type { Numbers } from '../bus/live'
import { battLevel, ms, tempLevel, wifiLevel } from '../format'
import { useUi } from '../store/ui'

export const TEMP_LABEL = ['MD後左', 'MD後右', 'MDステア', 'MCU']

export function DiagGrid({ n }: { n: Numbers }) {
  const vs = n.vs
  const link = n.link
  const wifi = useUi((s) => s.status?.wifi)
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
        {/* **添字を変数で回さず、値を並べて書く。** `motor_current` は3要素で
            `torque_cmd` は2要素（ステア軸のトルクは測っていない）と長さが違い、
            同じ `i` で両方を引くと 3 本目で範囲外を踏む。型（タプル）に
            守らせるには添字がリテラルである必要がある */}
        {([
          ['後左', vs.motor_current[0], vs.torque_cmd[0]],
          ['後右', vs.motor_current[1], vs.torque_cmd[1]],
          ['ステア', vs.motor_current[2], null],
        ] as const).map(([lbl, cur, trq]) => (
          <div className="kv" key={lbl}>
            <span>{lbl}</span>
            <b>{cur.toFixed(2)}A</b>
            {trq !== null && <b className="dim">{trq.toFixed(3)}N·m</b>}
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
        <h4>Wi-Fi</h4>
        <div className="kv">
          <span>SSID</span>
          <b>{wifi?.available === false ? '取得不可' : (wifi?.ssid ?? '未接続')}</b>
        </div>
        <div className="kv">
          <span>電波強度</span>
          <b className={`lv-${wifiLevel(wifi?.rssi_dbm ?? null)}`}>
            {wifi?.rssi_dbm == null ? '—' : `${wifi.rssi_dbm}dBm`}
          </b>
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
        {/* ★v0.11。GUI のスライダ上限がここより先に進んでいないか切り分けるための表示。
            null なら「まだ LIMITS を受信していない」ので、GUI 側は静的な既定値
            （`PI_MAX_SPEED_CAP` 等）にフォールバックしている（`store/ui.ts` の `effectiveRange`） */}
        <h4>車両上限（LIMITS・★v0.11）</h4>
        <div className="kv">
          <span>速度</span>
          <b className={link?.max_speed_m_s == null ? 'lv-warn' : ''}>
            {link?.max_speed_m_s == null ? '未受信' : `${link.max_speed_m_s.toFixed(2)}m/s`}
          </b>
        </div>
        <div className="kv">
          <span>加速度</span>
          <b className={link?.max_accel_m_s2 == null ? 'lv-warn' : ''}>
            {link?.max_accel_m_s2 == null ? '未受信' : `${link.max_accel_m_s2.toFixed(2)}m/s²`}
          </b>
        </div>
        <div className="kv">
          <span>トルク</span>
          <b className={link?.max_torque_nm == null ? 'lv-warn' : ''}>
            {link?.max_torque_nm == null ? '未受信' : `${link.max_torque_nm.toFixed(3)}N·m`}
          </b>
        </div>
        <div className="kv">
          <span>舵角</span>
          <b className={link?.max_steer_rad == null ? 'lv-warn' : ''}>
            {link?.max_steer_rad == null ? '未受信' : `${((link.max_steer_rad * 180) / Math.PI).toFixed(1)}°`}
          </b>
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
