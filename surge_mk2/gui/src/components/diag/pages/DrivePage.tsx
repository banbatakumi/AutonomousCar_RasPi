/**
 * 駆動・制御 — 速度・舵角の追従と、TC・ABS・TV がどれだけ介入しているか。
 *
 * トルクのグラフは「要求（絞る前）」と「実際（送った指令）」を左右1輪ずつ重ねる。
 * **2本が離れている区間が、制御が絞っている区間**（帯が同じ時刻に重なる）。
 */
import { deg, num } from '../../../format'
import { ChartGrid } from '../Chart'
import { Card, Row, type Tint } from '../Card'
import { CH } from '../chartDefs'
import { paramsText } from '../checks'
import type { PageProps } from './types'

const CHARTS = [
  CH.speed,
  CH.steer,
  CH.wheels,
  CH.slip,
  CH.torqueRL,
  CH.torqueRR,
  CH.torqueLimit,
  CH.yawRate,
  CH.tvRatio,
  CH.yawMoment,
  CH.accel,
]

/** 機能の ON/OFF（STM32 が `CONFIG_ACK` で答えた値）と、今まさに介入しているか */
function Assist({ label, enabled, active }: { label: string; enabled: boolean | null | undefined; active: boolean }) {
  const level: Tint | undefined = enabled == null ? 'na' : active ? 'warn' : undefined
  return (
    <Row label={label} level={level}>
      {enabled == null ? '未確認' : !enabled ? 'OFF' : active ? '介入中' : 'ON'}
    </Row>
  )
}

export function DrivePage({ n }: PageProps) {
  const { vs, link } = n
  if (!vs) return null
  const pp = paramsText(link?.control_params_status)
  return (
    <>
      <div className="dg-cards">
        <Card title="制御機能">
          <Assist label="TC" enabled={link?.tc_enabled} active={vs.tc_active} />
          <Assist label="ABS" enabled={link?.abs_enabled} active={vs.abs_active} />
          <Assist label="TV" enabled={link?.tv_enabled} active={vs.tv_active} />
          <Assist label="片輪浮き対策" enabled={link?.wheel_lift_guard_enabled} active={vs.wheel_lift_active} />
          <Row label="ローンチコントロール">{vs.launch_active ? '発進中' : '—'}</Row>
          <Row label="自動停止" level={vs.auto_stop_active ? 'bad' : link?.auto_stop_margin_cm == null ? 'na' : undefined}>
            {vs.auto_stop_active ? '作動中' : link?.auto_stop_margin_cm == null ? '未確認' : `余裕 ${num(link.auto_stop_margin_cm, 0)} cm`}
          </Row>
        </Card>
        <Card title="制御パラメータ" level={pp.level}>
          <Row label="STM32 との同期" level={pp.level}>
            {pp.text}
          </Row>
          <Row label="再送した回数" level={link?.control_params_drift ? 'warn' : undefined}>
            {link?.control_params_drift ?? 0}
          </Row>
          {(link?.control_params_problems ?? []).map((p) => (
            <p key={p} className="dg-note lv-warn">
              {p}
            </p>
          ))}
        </Card>
        <Card title="車両上限" level={link?.max_speed_m_s == null ? 'warn' : undefined}>
          <Row label="速度">{num(link?.max_speed_m_s, 2)} m/s</Row>
          <Row label="加速度">{num(link?.max_accel_m_s2, 2)} m/s²</Row>
          <Row label="トルク">{num(link?.max_torque_nm, 3)} N·m</Row>
          <Row label="舵角">{link?.max_steer_rad == null ? '—' : deg(link.max_steer_rad)}</Row>
        </Card>
        <Card title="いまの指令">
          <Row label="舵角 実測 / 指令">
            {deg(vs.steer_actual)} / {deg(vs.steer_cmd_echo)}
          </Row>
          <Row label="トルク 左 / 右">
            {num(vs.torque_cmd[0], 3)} / {num(vs.torque_cmd[1], 3)} N·m
          </Row>
          <Row label="ステア原点" level={vs.steer_center_valid ? undefined : 'bad'}>
            {vs.steer_center_valid ? '保存済み' : '未保存'}
          </Row>
          <Row label="サイドブレーキ">{vs.side_brake_active ? '固定中' : '—'}</Row>
        </Card>
      </div>
      <ChartGrid defs={CHARTS} />
    </>
  )
}
