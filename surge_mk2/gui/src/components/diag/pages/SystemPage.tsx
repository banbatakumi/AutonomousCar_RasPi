/**
 * センサ・Pi — センサが生きているか、RasPi 本体に余裕があるか、どのノードが動いているか。
 *
 * RasPi 本体の項目（CPU・メモリ・スロットリング）は実機でだけ出る。シムや再生では
 * `status.pi.available` が false で、カードごと「取得不可」になる。
 */
import {
  cpuLevel,
  deg,
  diskLevel,
  formatDateTime,
  gb,
  ghz,
  hex32,
  memLevel,
  metres,
  nodeLevel,
  num,
  revPerSec,
  scanAgeLevel,
  tempLevel,
} from '../../../format'
import { MSGS_SCHEMA } from '../../../generated/msgs'
import { ChartGrid } from '../Chart'
import { Card, Dot, Row } from '../Card'
import { CH } from '../chartDefs'
import { throttledText } from '../checks'
import type { PageProps } from './types'

const CHARTS = [CH.piCpu, CH.temp, CH.rate, CH.ultrasonic]

export function SystemPage({ n, st }: PageProps) {
  const { vs, link } = n
  if (!vs) return null
  const pi = st?.pi
  const th = throttledText(pi?.throttled)
  const nodes = st?.nodes ?? []
  const jpeg = st?.camera_jpeg
  const fan = st?.fan
  const scanLv = scanAgeLevel(n.scanAgeMs)
  return (
    <>
      <div className="dg-cards">
        <Card title="LiDAR" level={vs.lidar_ok ? (scanLv === 'ok' ? undefined : 'warn') : 'bad'}>
          <Row label="状態" level={vs.lidar_ok ? undefined : 'bad'}>
            {vs.lidar_ok ? '正常' : '異常'}
          </Row>
          <Row label="周期">{num(n.scanHz, 1)} Hz</Row>
          <Row label="回転数">{revPerSec(n.scanRotDps)} 回/秒</Row>
          <Row label="最後の周から" level={scanLv}>
            {Number.isFinite(n.scanAgeMs) ? `${num(n.scanAgeMs, 0)} ms` : '未受信'}
          </Row>
          <Row label="欠けたセクタ（直近の周）" level={n.scanMissing ? 'warn' : undefined}>
            {n.scanMissing} / 12
          </Row>
          <Row label="欠けたセクタ（累積）">
            {link?.lidar_sectors_lost ?? '—'} / {link ? link.lidar_scans * 12 : '—'}
          </Row>
        </Card>
        <Card title="IMU" level={vs.imu_ok ? undefined : 'bad'}>
          <Row label="状態" level={vs.imu_ok ? undefined : 'bad'}>
            {vs.imu_ok ? '正常' : '異常'}
          </Row>
          <Row label="pitch / roll">
            {deg(vs.pitch)} / {deg(vs.roll)}
          </Row>
          <Row label="ヨーレート">{num(vs.yaw_rate, 2)} rad/s</Row>
          <Row label="加速度 前後 / 左右">
            {num(vs.accel[0], 2)} / {num(vs.accel[1], 2)}
          </Row>
          <Row label="加速度 上下">{num(vs.accel[2], 2)} m/s²</Row>
        </Card>
        <Card title="車輪・超音波">
          <Row label="超音波 前 / 後">
            {metres(vs.us_front)} / {metres(vs.us_rear)}
          </Row>
          <Row label="前輪 左 / 右">
            {num(vs.wheel_speed[0], 2)} / {num(vs.wheel_speed[1], 2)} m/s
          </Row>
          <Row label="後輪 左 / 右">
            {num(vs.wheel_speed[2], 2)} / {num(vs.wheel_speed[3], 2)} m/s
          </Row>
          <Row label="走行距離">{num(vs.odom_center, 1)} m</Row>
          <Row label="オドメトリの跳び" level={link?.odom_jumps ? 'warn' : undefined}>
            {link?.odom_jumps ?? '—'}
          </Row>
        </Card>
        <Card title="RasPi 本体" level={pi?.available ? (th.level === 'ok' ? undefined : th.level) : 'na'}>
          <Row label="CPU 温度" level={n.piTempC == null ? 'na' : tempLevel(n.piTempC)}>
            {n.piTempC == null ? '取得不可' : `${num(n.piTempC, 1)} ℃`}
          </Row>
          {pi?.available ? (
            <>
              <Row label="CPU 平均 / 最大コア" level={cpuLevel(pi.cpu_pct)}>
                {num(pi.cpu_pct, 0)} / {num(pi.cpu_max_pct, 0)} %
              </Row>
              <Row label="周波数 / 上限">
                {ghz(pi.cpu_khz)} / {ghz(pi.cpu_max_khz)} GHz
              </Row>
              <Row label="ロードアベレージ">{num(pi.load1, 2)}</Row>
              <Row label="メモリ使用" level={memLevel(pi.mem_used_pct)}>
                {num(pi.mem_used_pct, 0)} %（全 {gb(pi.mem_total_mb)} GB）
              </Row>
              <Row label="電源・スロットリング" level={th.level}>
                {th.text}
              </Row>
            </>
          ) : (
            <Row label="CPU・メモリ" level="na">
              取得不可
            </Row>
          )}
          <Row label="ディスク空き" level={link?.disk_free_pct == null ? 'na' : diskLevel(link.disk_free_pct)}>
            {link?.disk_free_pct == null ? '未計測' : `${num(link.disk_free_pct, 0)} %`}
          </Row>
          <Row label="ログ書き込み失敗" level={link?.log_errors ? 'warn' : undefined}>
            {link?.log_errors ?? '—'}
          </Row>
          {fan && (
            <Row label="ファン" level={fan.available ? undefined : 'na'}>
              {fan.rpm == null ? '—' : `${fan.rpm} rpm`}
              {fan.available ? `（${fan.mode === 'auto' ? '自動' : '手動'}）` : ''}
            </Row>
          )}
        </Card>
        <Card title="ノード" wide level={nodes.length ? undefined : 'na'}>
          {nodes.length === 0 ? (
            <p className="dim dg-empty">生存申告を受けていない</p>
          ) : (
            <table className="dg-table">
              <thead>
                <tr>
                  <th />
                  <th>名前</th>
                  <th>最後の申告</th>
                  <th>pid</th>
                  <th>状態</th>
                </tr>
              </thead>
              <tbody>
                {nodes.map((x) => {
                  const lv = nodeLevel(x.age_ms)
                  return (
                    <tr key={x.node}>
                      <td>
                        <Dot level={lv} />
                      </td>
                      <td>{x.node}</td>
                      <td className={lv === 'ok' ? undefined : `lv-${lv}`}>
                        {x.age_ms < 1000 ? `${x.age_ms} ms 前` : `${num(x.age_ms / 1000, 1)} 秒前`}
                      </td>
                      <td className="dim">{x.pid || '—'}</td>
                      <td>{x.detail || '—'}</td>
                    </tr>
                  )
                })}
              </tbody>
            </table>
          )}
        </Card>
        <Card title="カメラ配信" level={jpeg ? (jpeg.errors ? 'warn' : undefined) : 'na'}>
          <Row label="エンコーダ">{st?.camera_encoder ?? '—'}</Row>
          <Row label="1枚あたり CPU">{jpeg ? `${num(jpeg.cpu_per_frame_ms, 2)} ms` : '—'}</Row>
          <Row label="焼いた枚数">{jpeg?.encoded ?? '—'}</Row>
          <Row label="捨てた枚数" level={jpeg?.torn ? 'warn' : undefined}>
            {jpeg?.torn ?? '—'}
          </Row>
          <Row label="失敗" level={jpeg?.errors ? 'warn' : undefined}>
            {jpeg?.errors ?? '—'}
          </Row>
          {jpeg?.last_error && <p className="dg-note lv-warn">{jpeg.last_error}</p>}
        </Card>
        <Card title="版数" level={link?.protocol_match === false ? 'bad' : undefined}>
          <Row label="プロトコル" level={link?.protocol_match === false ? 'bad' : undefined}>
            {link?.protocol_version == null ? '未受信' : `v${link.protocol_version}${link.protocol_match === false ? '（不一致）' : ''}`}
          </Row>
          <Row label="ファーム ID">
            {hex32(link?.fw_id)}
          </Row>
          <Row label="ファームのビルド">{link?.fw_build_epoch ? formatDateTime(link.fw_build_epoch) : '—'}</Row>
          <Row label="型定義の札">{hex32(MSGS_SCHEMA)}</Row>
          <Row label="相手">{link?.sim ? 'シミュレータ' : '実機'}</Row>
        </Card>
      </div>
      <ChartGrid defs={CHARTS} />
    </>
  )
}
