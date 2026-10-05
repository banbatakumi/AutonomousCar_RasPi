/**
 * 通信 — ブラウザ ⇄ RasPi ⇄ STM32 ⇄ MD の経路を、区間ごとの数字で見る。
 *
 * 「遅い・切れる」は、どの区間が悪いかで原因が変わる。**上りと下りを分けて出す**のも
 * 同じ理由（Pi が受けた側のエラーと STM32 が受けた側のエラーは別の配線・別の向き）。
 * MD バスは3台を合計にしない（1台だけ黙っているのが埋もれる）。
 */
import { MD_LABEL } from '../../../bus/history'
import {
  healthLevel,
  hbLateLevel,
  hex32,
  loopLevel,
  microsFromNs,
  ms,
  msFromNs,
  num,
  rttLevel,
  wifiLevel,
  worst,
  type Level,
} from '../../../format'
import { useUi } from '../../../store/ui'
import { ChartGrid } from '../Chart'
import { Card, Dot, Row, type Tint } from '../Card'
import { WifiIcon } from '../../WifiIcon'
import { CH } from '../chartDefs'
import { mdText } from '../checks'
import { Branch, Fan, Hop, Node } from '../Route'
import type { PageProps } from './types'

const CHARTS = [CH.delay, CH.rate, CH.errors, CH.loop, CH.wifi]

/** 累積カウンタの行。0 でなければ色を付ける */
function Count({ label, value, level = 'warn' }: { label: string; value: number | null | undefined; level?: Level }) {
  return (
    <Row label={label} level={value ? level : undefined}>
      {value ?? '—'}
    </Row>
  )
}

export function LinkPage({ n, st }: PageProps) {
  const { vs, link } = n
  const wsRttMs = useUi((s) => s.wsRttMs)
  if (!vs) return null
  const rx = link?.rx ?? {}
  const stm = link?.stm_rx ?? null
  const wifi = st?.wifi
  const uartErr = (rx.crc_error ?? 0) + (rx.packet_loss ?? 0) + (stm?.rx_crc_error ?? 0)
  const uartLevel = worst(healthLevel(link?.health), rttLevel(link?.cmd_rtt_ms), uartErr ? 'warn' : 'ok')
  const wsLevel: Tint = n.stale ? 'bad' : worst(rttLevel(wsRttMs), wifi?.available && wifi.ssid ? wifiLevel(wifi.rssi_dbm) : 'ok')
  const counts = Object.entries(link?.counts ?? {}).sort((a, b) => b[1] - a[1])
  return (
    <>
      <div className="dg-route">
        <Node name="ブラウザ" sub={`受信 ${num(n.rxHz, 0)} Hz`} />
        <Hop
          level={wsLevel}
          lines={[
            `往復 ${ms(wsRttMs, 0)}`,
            wifi?.available === false ? 'Wi-Fi 取得不可' : wifi?.ssid ? `${wifi.ssid} ${wifi.rssi_dbm ?? '—'} dBm` : 'Wi-Fi 未接続',
          ]}
        />
        <Node name="RasPi" sub={`指令元 ${link?.cmd_source || 'なし'}`} level={link?.cmd_stale ? 'warn' : undefined} />
        <Hop
          level={uartLevel}
          lines={[`往復 ${ms(link?.cmd_rtt_ms, 1)}`, `${link?.health ?? '—'}・エラー ${uartErr}`]}
        />
        <Node
          name="STM32"
          sub={link?.fw_id == null ? 'fw 未受信' : `fw ${hex32(link.fw_id)}`}
          level={vs.uart_timeout ? 'bad' : undefined}
        />
        <Fan>
          {[0, 1, 2].map((i) => {
            const m = mdText(vs.md_status[i as 0 | 1 | 2])
            const ok = link?.md_rx_count?.[i] ?? null
            const err = link?.md_rx_error?.[i] ?? null
            const pct = ok != null && err != null && ok + err > 0 ? (100 * err) / (ok + err) : null
            // MD 自身が「通信できている」と言っているのに受信数が 0 なのは、数を出さない相手（シム）。
            // 悪いとは言わず、判定しない
            const level: Tint =
              ok === 0 && m.level === 'ok' ? 'na' : worst(m.level, pct != null && pct > 5 ? 'warn' : 'ok')
            return (
              <Branch key={i} level={level} lines={[`${ok ?? '—'} 件・err ${num(pct, 1)}%`]}>
                <Node name={MD_LABEL[i as 0 | 1 | 2]} sub={m.text} level={m.level === 'ok' ? undefined : m.level} />
              </Branch>
            )
          })}
        </Fan>
      </div>

      <div className="dg-cards">
        <Card title="UART 受信（Pi 側）">
          <Row label="正常フレーム">{rx.frame_ok ?? '—'}</Row>
          <Count label="CRC エラー" value={rx.crc_error} level="bad" />
          <Count label="パケットロス" value={rx.packet_loss} />
          <Count label="長さエラー" value={rx.len_error} />
          <Count label="順序入れ替わり" value={rx.reordered} />
          <Count label="重複" value={rx.duplicate} />
          <Count label="再同期で捨てた byte" value={rx.resync_bytes} />
        </Card>
        <Card title="UART 受信（STM32 側）" level={stm ? undefined : 'na'}>
          <Row label="正常フレーム">{stm?.rx_frame_ok ?? '—'}</Row>
          <Count label="CRC エラー" value={stm?.rx_crc_error} level="bad" />
          <Count label="長さエラー" value={stm?.rx_len_error} />
          <Count label="未知の種別" value={stm?.rx_unknown_type} />
          <Count label="送信の取りこぼし" value={stm?.tx_drop} />
          <Row label="COMMAND 途絶" level={vs.uart_timeout ? 'bad' : undefined}>
            {vs.uart_timeout ? '自動ブレーキ中' : 'なし'}
          </Row>
        </Card>
        <Card title="ハートビート・ループ" level={link?.hb_alive === false ? 'bad' : undefined}>
          <Row label="GPIO6 ハートビート" level={link?.hb_alive == null ? 'na' : link.hb_alive ? undefined : 'bad'}>
            {link?.hb_alive == null ? 'GPIO なし' : link.hb_alive ? '出力中' : '停止'}
          </Row>
          <Row label="最大遅れ" level={hbLateLevel(link?.hb_max_late_ms)}>
            {ms(link?.hb_max_late_ms, 2)}
          </Row>
          <Count label="停滞した回数" value={link?.hb_stalls} level="bad" />
          <Row label="io ループ最大" level={loopLevel(link?.loop_max_ms)}>
            {ms(link?.loop_max_ms, 2)}
          </Row>
          <Count label="STM32 再起動の検出" value={link?.stm_resets} level="bad" />
        </Card>
        <Card title="指令の経路">
          <Row label="指令元">{link?.cmd_source || 'なし'}</Row>
          <Row label="cmd" level={link?.cmd_stale ? 'warn' : undefined}>
            {link?.cmd_stale ? '途絶（DISARM 送出中）' : '届いている'}
          </Row>
          <Count label="cmd 途絶の回数" value={link?.cmd_timeouts} />
          <Count label="デッドマン発動" value={st?.deadman_trips} />
          <Count label="自律指令の途絶" value={st?.auto.stalls} />
          <Count label="壊れた cmd" value={st?.bad_cmds} />
          <Count label="認証で拒否" value={(st?.auth_rejects ?? 0) + (st?.origin_rejects ?? 0)} />
        </Card>
        <Card title="時刻同期">
          <Row label="offset">{microsFromNs(link?.sync_offset_ns)}</Row>
          <Row label="drift">{link?.sync_drift_ppm == null ? '—' : `${num(link.sync_drift_ppm, 1)} ppm`}</Row>
          <Row label="片道遅延">{msFromNs(link?.sync_delay_ns)}</Row>
          <Row label="標本数">{link?.sync_n ?? '—'}</Row>
        </Card>
        <Card title="Wi-Fi・ブラウザ" level={wifi?.available && wifi.ssid ? wifiLevel(wifi.rssi_dbm) : 'na'}>
          <Row label="SSID" level={wifi?.available === false ? 'na' : undefined}>
            {wifi?.available === false ? '取得不可' : (wifi?.ssid ?? '未接続')}
          </Row>
          <Row label="電波強度" level={wifi?.available && wifi.ssid ? wifiLevel(wifi.rssi_dbm) : 'na'}>
            <WifiIcon dbm={wifi?.rssi_dbm ?? null} /> {wifi?.rssi_dbm == null ? '—' : `${wifi.rssi_dbm} dBm`}
          </Row>
          <Row label="ブラウザ ⇄ Pi 往復" level={rttLevel(wsRttMs)}>
            {ms(wsRttMs, 0)}
          </Row>
          <Row label="テレメトリ受信" level={n.stale ? 'bad' : undefined}>
            {n.stale ? '途絶' : `${num(n.rxHz, 1)} Hz`}
          </Row>
        </Card>
        <Card title="受信パケット数">
          {counts.length === 0 && <p className="dim dg-empty">未受信</p>}
          {counts.map(([name, c]) => (
            <Row key={name} label={name}>
              {c}
            </Row>
          ))}
        </Card>
        <Card title="接続中の画面">
          <Row label="テレメトリ">{st?.clients.telemetry ?? '—'}</Row>
          <Row label="操作">{st?.clients.control ?? '—'}</Row>
          <Row label="操縦権">
            <Dot level={st?.has_controller ? 'ok' : 'na'} /> {st?.has_controller ? st.controller || 'あり' : 'なし'}
          </Row>
        </Card>
      </div>
      <ChartGrid defs={CHARTS} />
    </>
  )
}
