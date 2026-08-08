/**
 * 層 C（異常時のみ）— 温度・電圧・電流・リンク統計。
 *
 * **平常時は1行に畳み、しきい値を超えたものだけが自動的に前に出る**
 * （`architecture.md` §10.1）。正常な温度を常時大きく出しても情報量はゼロで、
 * その面積は層 A（カメラと点群）に回すべき。
 *
 * 自動展開の条件はここ1箇所に集める。散らすと「なぜ開いたのか」が追えなくなる。
 */
import { useNumbers } from '../bus/live'
import { battLevel, ms, tempLevel } from '../format'
import { useUi } from '../store/ui'

const TEMP_LABEL = ['MD後左', 'MD後右', 'MDステア', 'MCU']

export function DiagStrip() {
  const n = useNumbers()
  const { diagOpen, set } = useUi()
  const vs = n.vs
  const link = n.link

  // ── 「今まさに異常」と「これまでに何回あった」を分ける ──
  //
  // **累積カウンタを自動展開の条件にしてはいけない。** CRC エラーが起動直後に
  // 1回出ただけで、以降ずっとパネルが開いたままになり、層 C を畳んだ意味が消える。
  // 自動展開は「現在の状態」だけ、累積は数字として並べるだけにする。
  const now: string[] = [] // 今まさに異常 → 自動展開する
  const past: string[] = [] // 累積 → バッジで出すが開かない

  if (vs) {
    vs.temp.forEach((t, i) => {
      const lv = tempLevel(t)
      if (lv !== 'ok') now.push(`${TEMP_LABEL[i]} ${t == null ? '通信断' : `${t}℃`}`)
    })
    if (battLevel(vs.batt_voltage[0]) !== 'ok') now.push(`駆動 ${vs.batt_voltage[0].toFixed(1)}V`)
    if (battLevel(vs.batt_voltage[1]) !== 'ok') now.push(`信号 ${vs.batt_voltage[1].toFixed(1)}V`)
    for (const f of vs.faults) now.push(f)
    if (!vs.imu_ok) now.push('IMU 異常')
    if (!vs.lidar_ok) now.push('LiDAR 受信断')
  }
  if (link) {
    if (link.health !== 'OK' && link.health !== 'INIT') now.push(`リンク ${link.health}`)
    if (link.hb_alive === false) now.push('GPIO6 ハートビート停止')
    if ((link.rx?.crc_error ?? 0) > 0) past.push(`CRC 累計 ${link.rx.crc_error}`)
    if ((link.rx?.packet_loss ?? 0) > 0) past.push(`loss 累計 ${link.rx.packet_loss}`)
    if (link.lidar_sectors_lost > 0) past.push(`セクタ落ち 累計 ${link.lidar_sectors_lost}`)
    if (link.hb_stalls > 0) past.push(`hb 停止 ${link.hb_stalls}回`)
  }

  // **今まさに異常なら人間の操作を待たずに開く。** 畳んだまま見逃すのが一番困る
  const open = diagOpen || now.length > 0

  return (
    <div className={`diag ${open ? 'open' : ''}`}>
      <button className="diag-toggle" onClick={() => set({ diagOpen: !diagOpen })}>
        {open ? '▼' : '▶'} 温度・電圧・リンク
        {now.length > 0 && <span className="badge-bad">{now.length}</span>}
        {now.length === 0 && past.length === 0 && <span className="dim"> 正常</span>}
        {now.length === 0 && past.length > 0 && <span className="dim"> 現在は正常</span>}
      </button>

      {(now.length > 0 || past.length > 0) && (
        <div className="diag-alerts">
          {now.map((a) => (
            <span className="badge-bad" key={a}>
              {a}
            </span>
          ))}
          {past.map((a) => (
            <span className="badge-warn" key={a}>
              {a}
            </span>
          ))}
        </div>
      )}

      {open && vs && (
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
              <b className={`lv-${battLevel(vs.batt_voltage[0])}`}>
                {vs.batt_voltage[0].toFixed(2)}V
              </b>
              <b>{vs.batt_current[0].toFixed(2)}A</b>
            </div>
            <div className="kv">
              <span>信号</span>
              <b className={`lv-${battLevel(vs.batt_voltage[1])}`}>
                {vs.batt_voltage[1].toFixed(2)}V
              </b>
              <b>{vs.batt_current[1].toFixed(2)}A</b>
            </div>
            <span className="note">電流は単方向センサ。回生中は 0 に張り付く</span>
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
            <span className="note">トルクは指令値。実測ではない（Kt 未実測）</span>
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
            <span className="note">前輪は射影後の値。低速では前輪エンコーダが当てにならない</span>
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
              <b className={(link?.rx?.crc_error ?? 0) > 0 ? 'lv-bad' : ''}>
                {link?.rx?.crc_error ?? 0}
              </b>
              <b className="dim">{link?.stm_rx?.rx_crc_error ?? '—'}</b>
            </div>
            <div className="kv">
              <span>loss / tx_drop</span>
              <b>{link?.rx?.packet_loss ?? 0}</b>
              <b className="dim">{link?.stm_rx?.tx_drop ?? '—'}</b>
            </div>
            <span className="note">片方向だけ増えるか両方かで原因の切り分けが変わる</span>
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
            <span className="note">
              受信数が 0 の台は起動待ちか配線。駆動電源を入れた直後は後右が約5秒遅れる
            </span>
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
                {((vs.pitch * 180) / Math.PI).toFixed(1)}° /{' '}
                {((vs.roll * 180) / Math.PI).toFixed(1)}°
              </b>
            </div>
            <span className="note">yaw は出ない（地磁気センサ無し）</span>
          </section>
        </div>
      )}
    </div>
  )
}
