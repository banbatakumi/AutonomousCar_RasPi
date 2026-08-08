/**
 * 20Hz で届くデータの置き場。**React の state には入れない。**
 *
 * `docs/architecture.md` §10.4 のとおり、20〜30Hz のデータを setState すると
 * 再レンダリングで破綻する。流量の多いものは3つに分けて扱う。
 *
 *   1. 点群・カメラ  → ここ（可変オブジェクト）に置き、Canvas が rAF で読む
 *   2. 数値表示      → 8Hz に間引いて `useNumbers()` から配る（人間は 20Hz の数字を読めない）
 *   3. 接続状態・設定 → イベント時だけ zustand（`store/ui.ts`）
 */
import { useSyncExternalStore } from 'react'
import type { LinkDiag, Scan, VehicleState } from '../types'

/** 最新値。**書き換わり続けるので、React から直接読まないこと。** */
export const live = {
  vs: null as VehicleState | null,
  scan: null as Scan | null,
  link: null as LinkDiag | null,
  /** テレメトリを最後に受けた時刻（performance.now） */
  lastRxMs: 0,
  /** 実測の受信レート [Hz]。**GUI 側が詰まっていないか**を見るために出す */
  rxHz: 0,
  /** 点群を最後に受けた時刻 */
  lastScanMs: 0,
  scanHz: 0,
  /** ARM が自動解除されるまでの残り [ms]。20Hz で減るのでここに置く */
  armRemainingMs: 0,
}

/** 自分が今送っている指令。20Hz で書き換わるのでここに置く（zustand には入れない） */
export const cmdOut = { speed: 0, steer: 0, active: false }

let rxCount = 0
let scanCount = 0
let rateT0 = 0

export function noteTelemetry(vs: VehicleState | null, link: LinkDiag | null, scan: Scan | null) {
  const now = performance.now()
  if (vs) live.vs = vs
  if (link) live.link = link
  if (scan) {
    live.scan = scan
    live.lastScanMs = now
    scanCount++
  }
  live.lastRxMs = now
  rxCount++
  if (now - rateT0 >= 1000) {
    const dt = (now - rateT0) / 1000
    live.rxHz = rxCount / dt
    live.scanHz = scanCount / dt
    rxCount = 0
    scanCount = 0
    rateT0 = now
  }
}

export function clearLive() {
  live.vs = null
  live.scan = null
  live.link = null
  live.rxHz = 0
  live.scanHz = 0
}

// ── 数値表示用の間引き（8Hz） ──────────────────────────────────────

/** 受信が途絶したとみなす時間。これを超えたら数値をグレーにする */
export const STALE_MS = 500
const NUMBERS_HZ = 8

export type Numbers = {
  rev: number
  stale: boolean
  vs: VehicleState | null
  link: LinkDiag | null
  rxHz: number
  scanHz: number
  scanAgeMs: number
  /** **直近の1周**で欠けていたセクタ数。累積ではない（累積は link 側にある） */
  scanMissing: number
  out: { speed: number; steer: number; active: boolean }
  /** ARM の自動解除までの残り [ms] */
  armRemainingMs: number
}

let snapshot: Numbers = {
  rev: 0,
  stale: true,
  vs: null,
  link: null,
  rxHz: 0,
  scanHz: 0,
  scanAgeMs: Infinity,
  scanMissing: 0,
  out: { speed: 0, steer: 0, active: false },
  armRemainingMs: 0,
}
const listeners = new Set<() => void>()

setInterval(() => {
  const now = performance.now()
  snapshot = {
    rev: snapshot.rev + 1,
    stale: live.lastRxMs === 0 || now - live.lastRxMs > STALE_MS,
    vs: live.vs,
    link: live.link,
    rxHz: live.rxHz,
    scanHz: live.scanHz,
    scanAgeMs: live.lastScanMs ? now - live.lastScanMs : Infinity,
    scanMissing: live.scan ? live.scan.sector_seen.filter((s) => !s).length : 0,
    out: { ...cmdOut },
    armRemainingMs: live.armRemainingMs,
  }
  for (const l of listeners) l()
}, 1000 / NUMBERS_HZ)

function subscribe(fn: () => void) {
  listeners.add(fn)
  return () => listeners.delete(fn)
}

/** 数値表示用のスナップショット。8Hz でしか変わらない。 */
export function useNumbers(): Numbers {
  return useSyncExternalStore(subscribe, () => snapshot)
}
