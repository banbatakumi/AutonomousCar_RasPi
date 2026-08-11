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
