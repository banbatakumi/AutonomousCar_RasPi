/**
 * 表示のための単位変換。**ここ以外で SI から離れないこと**（`architecture.md` §5.1）。
 */

export const RAD2DEG = 180 / Math.PI

/** 標準重力 [m/s²]。加速度を「G」で読ませるためだけに持つ */
export const G = 9.80665

/** 加速度 [m/s²] → G。**ラジコンの G メータ専用**（制御には使わない） */
export function gForce(a: number): number {
  return a / G
}

export function deg(rad: number, digits = 1): string {
  return `${(rad * RAD2DEG).toFixed(digits)}°`
}

export function mps(v: number, digits = 2): string {
  return `${v.toFixed(digits)}`
}

export function kmh(v: number, digits = 1): string {
  return `${(v * 3.6).toFixed(digits)}`
}

export function metres(v: number | null, digits = 2): string {
  return v == null ? '—' : `${v.toFixed(digits)}m`
}

export function ms(v: number | null | undefined, digits = 1): string {
  return v == null ? '—' : `${v.toFixed(digits)}ms`
}

export const MODE_NAME = ['DISARM', 'MANUAL', 'AUTO', '予約']

/** 温度・電圧などの「正常か注意か危険か」。**しきい値は1箇所に集める。** */
export type Level = 'ok' | 'warn' | 'bad'

export function tempLevel(t: number | null): Level {
  if (t == null) return 'bad' // MD が無言＝値を信用できない（0 ではない）
  if (t >= 70) return 'bad'
  if (t >= 55) return 'warn'
  return 'ok'
}

/** 8セル NiMH。放電終止 8.0V / 満充電 11.2V（`uart_protocol.md` §5.3）。 */
export function battLevel(v: number): Level {
  if (v < 8.0) return 'bad'
  if (v < 8.8) return 'warn' // STM32 のヒステリシス復帰点に合わせる
  return 'ok'
}

/** エンドツーエンド遅延。50ms 超で警告（`uart_protocol.md` §5.3）。 */
export function rttLevel(v: number | null | undefined): Level {
  if (v == null) return 'warn'
  if (v > 50) return 'bad'
  if (v > 30) return 'warn'
  return 'ok'
}

export function healthLevel(h: string | undefined): Level {
  if (h === 'OK') return 'ok'
  if (h === 'DEGRADED') return 'warn'
  return 'bad'
}

/** Wi-Fi RSSI [dBm]。null は未接続/取得不可（信用できないので bad 扱い、`tempLevel` と同じ方針）。 */
export function wifiLevel(dbm: number | null): Level {
  if (dbm == null) return 'bad'
  if (dbm <= -75) return 'bad'
  if (dbm <= -60) return 'warn'
  return 'ok'
}

/** Wi-Fi RSSI [dBm] → バー本数(0-3)。`WifiIcon` の表示専用（しきい値は `wifiLevel` と共通）。 */
export function wifiBars(dbm: number | null): 0 | 1 | 2 | 3 {
  if (dbm == null) return 0
  if (dbm <= -75) return 1
  if (dbm <= -60) return 2
  return 3
}

/** バイト数 → 人が読める表記（`LogControls`/診断タブの `LogsPage` 共用）。 */
export function formatBytes(n: number): string {
  return n >= 1e6 ? `${(n / 1e6).toFixed(1)}MB` : `${(n / 1e3).toFixed(0)}KB`
}

/** 記録経過時間 [s] → `分:秒`。 */
export function formatElapsed(s: number): string {
  const m = Math.floor(s / 60)
  const sec = Math.floor(s % 60)
  return `${m}:${sec.toString().padStart(2, '0')}`
}

export function formatDateTime(unixSec: number): string {
  return new Date(unixSec * 1000).toLocaleString()
}

// ── 診断タブ ──────────────────────────────────────────────────────

/** 欠測は「—」。**0 と書かない**（値があるのと無いのを区別する） */
export function num(v: number | null | undefined, digits = 1): string {
  return v == null || !Number.isFinite(v) ? '—' : v.toFixed(digits)
}

/** 8セル NiMH の放電終止・STM32 の低電圧復帰点・満充電 [V]（`uart_protocol.md` §5.3） */
export const BATT_EMPTY_V = 8.0
export const BATT_WARN_V = 8.8
export const BATT_FULL_V = 11.2
/** 温度表示のしきい値 [℃]（`tempLevel` と同じ値。グラフの線に使う） */
export const TEMP_WARN_C = 55
export const TEMP_BAD_C = 70
/** ハード遮断の電流 [A]（駆動 / 信号。ラッチして電源入れ直しまで戻らない） */
export const DRIVE_CUTOFF_A = 5.0
export const SIGNAL_CUTOFF_A = 3.0

/** いちばん悪いものを返す */
export function worst(...levels: Level[]): Level {
  return levels.includes('bad') ? 'bad' : levels.includes('warn') ? 'warn' : 'ok'
}

/** バッテリー電流。遮断値の 80% で注意、95% で危険 */
export function currentLevel(a: number, cutoffA: number): Level {
  if (a >= cutoffA * 0.95) return 'bad'
  if (a >= cutoffA * 0.8) return 'warn'
  return 'ok'
}

/** io_node のループ1周の最大 [ms]。100ms でハートビートが止まり E-Stop になる */
export function loopLevel(v: number | null | undefined): Level {
  if (v == null) return 'ok'
  if (v > 50) return 'bad'
  if (v > 20) return 'warn'
  return 'ok'
}

/** GPIO6 ハートビートの最大遅れ [ms]。50ms 途絶で STM32 が E-Stop をラッチする */
export function hbLateLevel(v: number | null | undefined): Level {
  if (v == null) return 'ok'
  if (v > 25) return 'bad'
  if (v > 5) return 'warn'
  return 'ok'
}

export function cpuLevel(pct: number | null | undefined): Level {
  if (pct == null) return 'ok'
  if (pct > 90) return 'bad'
  if (pct > 70) return 'warn'
  return 'ok'
}

export function memLevel(usedPct: number | null | undefined): Level {
  if (usedPct == null) return 'ok'
  if (usedPct > 90) return 'bad'
  if (usedPct > 75) return 'warn'
  return 'ok'
}

export function diskLevel(freePct: number | null | undefined): Level {
  if (freePct == null) return 'ok'
  if (freePct < 10) return 'bad'
  if (freePct < 20) return 'warn'
  return 'ok'
}

/** ノードの生存申告の古さ [ms]。カメラ系は IDLE 中 2Hz まで間引くので 1秒は待つ */
export function nodeLevel(ageMs: number): Level {
  if (ageMs > 2500) return 'bad'
  if (ageMs > 1200) return 'warn'
  return 'ok'
}

/** LiDAR の1周の古さ [ms]。planner は 300ms で制動に入る（`system_overview.md`） */
export function scanAgeLevel(ageMs: number): Level {
  if (!Number.isFinite(ageMs) || ageMs > 300) return 'bad'
  if (ageMs > 200) return 'warn'
  return 'ok'
}

/** ns → μs */
export function microsFromNs(ns: number | null | undefined, digits = 1): string {
  return ns == null ? '—' : `${(ns / 1e3).toFixed(digits)} μs`
}

/** ns → ms */
export function msFromNs(ns: number | null | undefined, digits = 2): string {
  return ns == null ? '—' : ms(ns / 1e6, digits)
}

/** kHz → GHz（CPU の周波数） */
export function ghz(khz: number | null | undefined, digits = 2): string {
  return khz == null ? '—' : (khz / 1e6).toFixed(digits)
}

/** MB → GB */
export function gb(mb: number | null | undefined, digits = 1): string {
  return mb == null ? '—' : (mb / 1024).toFixed(digits)
}

/** 回転数 [°/s] → [回/秒]（LiDAR） */
export function revPerSec(dps: number | null | undefined, digits = 2): string {
  return dps == null ? '—' : (dps / 360).toFixed(digits)
}

/** `fw_id` などの u32 を 8桁の16進で */
export function hex32(v: number | null | undefined): string {
  return v == null ? '—' : (v >>> 0).toString(16).toUpperCase().padStart(8, '0')
}
