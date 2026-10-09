/**
 * 走行前チェック — 「走ってよいか」を1箇所で判定する（常設サマリの「走行可否」と概要ページの一覧）。
 *
 * **判定はここだけに置く。** サマリと一覧が別々に判定すると、片方だけ直して食い違う。
 * しきい値そのものは `format.ts`（表示の色分けと共通）。
 */
import { MD, MD_LABEL } from '../../bus/history'
import type { Numbers } from '../../bus/live'
import {
  battLevel,
  diskLevel,
  hbLateLevel,
  healthLevel,
  nodeLevel,
  num,
  scanAgeLevel,
  tempLevel,
  type Level,
} from '../../format'
import type { ControlStatus } from '../../types'
import type { DiagPage } from './view'

/** `na` は「この機体・この状態では判定しない」（シムに GPIO は無い、など）。集計に入れない */
export type CheckLevel = Level | 'na'

export type Check = {
  group: string
  label: string
  value: string
  level: CheckLevel
  /** 詳しい数字があるサブページ */
  page: DiagPage
}

/** `get_throttled` のビット（`raspi/io/pihealth.py` の `THROTTLED_BITS`） */
export const THROTTLED = {
  UNDERVOLTAGE: 0x1,
  FREQ_CAPPED: 0x2,
  THROTTLED: 0x4,
  SOFT_TEMP: 0x8,
  /** 起動してから一度でも（上の4つを 16bit 左へ） */
  OCCURRED_SHIFT: 16,
} as const

/** スロットリングの状態を1語にする。`null` は読めない機体 */
export function throttledText(t: number | null | undefined): { text: string; level: CheckLevel } {
  if (t == null) return { text: '取得不可', level: 'na' }
  const now = t & 0xf
  const past = (t >> THROTTLED.OCCURRED_SHIFT) & 0xf
  if (now & THROTTLED.UNDERVOLTAGE) return { text: '低電圧', level: 'bad' }
  if (now & (THROTTLED.THROTTLED | THROTTLED.FREQ_CAPPED | THROTTLED.SOFT_TEMP)) {
    return { text: '制限中', level: 'warn' }
  }
  if (past & THROTTLED.UNDERVOLTAGE) return { text: '起動後に低電圧あり', level: 'warn' }
  if (past) return { text: '起動後に制限あり', level: 'warn' }
  return { text: '正常', level: 'ok' }
}

const FAULT_TEXT: Record<string, string> = {
  drive_overcurrent: '駆動 過電流',
  signal_overcurrent: '信号 過電流',
  drive_undervoltage: '駆動 低電圧',
  md_fault: 'MD 異常で停止',
  signal_undervoltage: '信号 低電圧',
}

export function faultText(name: string): string {
  return FAULT_TEXT[name] ?? name
}

/** MD 1台の状態を1語にする */
export function mdText(status: number): { text: string; level: Level } {
  if (!(status & MD.COMM_OK)) return { text: '通信断', level: 'bad' }
  if (status & MD.OVERHEAT) return { text: '過熱', level: 'bad' }
  if (status & MD.OVERCURRENT) return { text: '過電流', level: 'bad' }
  if (status & MD.VOLTAGE_OOR) return { text: '電圧異常', level: 'warn' }
  if (!(status & MD.LIMIT_SYNCED)) return { text: '上限 未同期', level: 'warn' }
  return { text: status & MD.RUNNING ? '駆動中' : '待機', level: 'ok' }
}

const PARAMS_TEXT: Record<string, { text: string; level: Level }> = {
  none: { text: '送るもの無し', level: 'ok' },
  ok: { text: '一致', level: 'ok' },
  pending: { text: '送信中', level: 'warn' },
  mismatch: { text: 'STM32 が丸めた', level: 'warn' },
  unsupported: { text: 'ファームが古い', level: 'warn' },
  invalid: { text: '[control] が読めない', level: 'bad' },
}

export function paramsText(status: string | undefined): { text: string; level: Level } {
  return PARAMS_TEXT[status ?? ''] ?? { text: status || '—', level: 'warn' }
}

export function runChecks(n: Numbers, st: ControlStatus | null): Check[] {
  const { vs, link } = n
  if (!vs) {
    return [{ group: '通信', label: 'テレメトリ', value: '未受信', level: 'bad', page: '通信' }]
  }
  const out: Check[] = []
  const add = (group: string, label: string, value: string, level: CheckLevel, page: DiagPage) =>
    out.push({ group, label, value, level, page })

  // ── ラッチ・フォルト（解除に人手が要るもの） ──
  add('安全', 'E-STOP', vs.estop_active ? '発動中' : '解除', vs.estop_active ? 'bad' : 'ok', '概要')
  add('安全', '駆動電源', vs.drive_power_locked ? 'ラッチ遮断' : '正常', vs.drive_power_locked ? 'bad' : 'ok', '電源・熱')
  add('安全', 'フォルト', vs.faults.length ? vs.faults.map(faultText).join('・') : 'なし', vs.faults.length ? 'bad' : 'ok', '電源・熱')
  add('安全', 'ステア原点', vs.steer_center_valid ? '保存済み' : '未保存', vs.steer_center_valid ? 'ok' : 'bad', '駆動・制御')

  // ── 電源・熱 ──
  add('電源', '駆動バッテリー', `${num(vs.batt_voltage[0], 2)} V`, battLevel(vs.batt_voltage[0]), '電源・熱')
  add('電源', '信号バッテリー', `${num(vs.batt_voltage[1], 2)} V`, battLevel(vs.batt_voltage[1]), '電源・熱')
  const temps: [string, number | null][] = [
    [MD_LABEL[0], vs.temp[0]],
    [MD_LABEL[1], vs.temp[1]],
    [MD_LABEL[2], vs.temp[2]],
    ['MCU', vs.temp[3]],
  ]
  for (const [label, t] of temps) {
    add('温度', label, t == null ? '通信断' : `${num(t, 0)} ℃`, tempLevel(t), '電源・熱')
  }

  // ── STM32・MD ──
  for (let i = 0; i < 3; i++) {
    const m = mdText(vs.md_status[i as 0 | 1 | 2])
    add('MD', MD_LABEL[i as 0 | 1 | 2], m.text, m.level, '通信')
  }
  add(
    'STM32',
    'プロトコル',
    link?.protocol_match == null ? '未確認' : link.protocol_match ? `v${link.protocol_version} 一致` : '不一致',
    link?.protocol_match == null ? 'warn' : link.protocol_match ? 'ok' : 'bad',
    'センサ・Pi',
  )
  const pp = paramsText(link?.control_params_status)
  add('STM32', '制御パラメータ', pp.text, pp.level, '駆動・制御')
  add('STM32', '車両上限', link?.max_speed_m_s == null ? '未受信' : '受信済み', link?.max_speed_m_s == null ? 'warn' : 'ok', '駆動・制御')

  // ── 通信 ──
  add('通信', 'リンク', link?.health ?? '—', healthLevel(link?.health), '通信')
  add('通信', 'テレメトリ', n.stale ? '途絶' : `${num(n.rxHz, 0)} Hz`, n.stale ? 'bad' : 'ok', '通信')
  if (link?.hb_alive == null) {
    add('通信', 'ハートビート', 'GPIO なし', 'na', '通信')
  } else {
    add(
      '通信',
      'ハートビート',
      link.hb_alive ? `最大遅れ ${num(link.hb_max_late_ms, 2)} ms` : '停止',
      link.hb_alive ? hbLateLevel(link.hb_max_late_ms) : 'bad',
      '通信',
    )
  }

  // ── センサ ──
  add('センサ', 'IMU', vs.imu_ok ? '正常' : '異常', vs.imu_ok ? 'ok' : 'bad', 'センサ・Pi')
  add(
    'センサ',
    'LiDAR',
    vs.lidar_ok ? `${num(n.scanHz, 1)} Hz` : '異常',
    vs.lidar_ok ? (scanAgeLevel(n.scanAgeMs) === 'ok' ? 'ok' : 'warn') : 'warn',
    'センサ・Pi',
  )

  // ── Pi ──
  const pi = st?.pi
  if (pi?.available) {
    const th = throttledText(pi.throttled)
    add('Pi', '電源・スロットリング', th.text, th.level, 'センサ・Pi')
  }
  if (link?.disk_free_pct != null) {
    add('Pi', 'ディスク空き', `${num(link.disk_free_pct, 0)} %`, diskLevel(link.disk_free_pct), 'センサ・Pi')
  }
  const nodes = st?.nodes ?? []
  if (nodes.length) {
    const dead = nodes.filter((x) => nodeLevel(x.age_ms) !== 'ok')
    add(
      'Pi',
      'ノード',
      dead.length ? `${dead.map((x) => x.node).join('・')} が無応答` : `${nodes.length} 個 稼働`,
      // io が黙っているなら走れない。それ以外は機能が1つ欠けるだけ
      dead.some((x) => x.node === 'io' && nodeLevel(x.age_ms) === 'bad') ? 'bad' : dead.length ? 'warn' : 'ok',
      'センサ・Pi',
    )
  }
  return out
}

export type Verdict = { level: Level; text: string; bad: number; warn: number }

export function verdict(checks: Check[]): Verdict {
  const bad = checks.filter((c) => c.level === 'bad').length
  const warn = checks.filter((c) => c.level === 'warn').length
  if (bad) return { level: 'bad', text: '走行不可', bad, warn }
  if (warn) return { level: 'warn', text: '要確認', bad, warn }
  return { level: 'ok', text: '走行可', bad, warn }
}
