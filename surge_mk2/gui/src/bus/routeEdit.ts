/**
 * 経由点エディタの状態（`slam2d_route`） — 地図パネルのクリックで経由点・停止点を置く。
 *
 * `bus/mapPreview.ts` と同じ「React の外に置き、rAF の描画ループが毎フレーム
 * 直接読む」流儀。編集中の設定（`cfg`）は**手元の写し**で、「保存」を押すまで
 * planner には送らない（`ch.saveRoutes()`）。走行中の地図に反映されている設定は
 * `live.map.routesJson` にあり、編集を始めるとそこから写し取る。
 *
 * 形は `raspi/auto/route_config.py` と同じ（座標は map フレーム [m]、角度 [rad]）。
 *
 * ## クリックの意味
 *
 * - 編集していないとき: 自己位置探索のヒント（今までどおり、`AutoMapPanel.tsx`）
 * - グループ（A〜D）を編集中: 末尾に経由点を足す。**既存の経由点の近く
 *   （画面で12px以内）をクリックするとその点を消す**
 * - 停止点（P1〜P3）を編集中: 1回目で位置、2回目で向き（位置からクリックした方向）
 */
import { live } from './live'

export const GROUPS = ['A', 'B', 'C', 'D'] as const
export const STOPS = ['P1', 'P2', 'P3'] as const

export type Stop = { x: number; y: number; yaw: number | null; mode: 'stop' | 'park' }
export type RouteCfg = {
  groups: Record<string, number[][]>
  active: string
  stops: Record<string, Stop>
  mission: { laps: number; time_s: number; then: string }
  signal_map: Record<string, string>
}

/** グループごとの色（経路・経由点で共通）。停止経路は白 */
export const GROUP_COLOR: Record<string, string> = {
  A: '#5ec8f0',
  B: '#f0a35e',
  C: '#b98cf0',
  D: '#7fe07a',
  stop: '#e8e8e8',
}

export const routeEdit = {
  /** 編集中か。false の間はクリックは自己位置ヒントになる */
  active: false,
  /** 編集対象（"A"〜"D" か "P1"〜"P3"） */
  target: 'A' as string,
  cfg: null as RouteCfg | null,
  /** 保存していない変更があるか */
  dirty: false,
  /** 停止点の編集で、位置を置いて向き待ちか */
  awaitingYaw: false,
}

export function emptyCfg(): RouteCfg {
  return { groups: {}, active: 'A', stops: {}, mission: { laps: 0, time_s: 0, then: '' }, signal_map: {} }
}

export function parseCfg(text: string): RouteCfg {
  const c = emptyCfg()
  if (!text) return c
  try {
    const d = JSON.parse(text) as Partial<RouteCfg>
    c.groups = d.groups ?? {}
    c.active = d.active ?? 'A'
    c.stops = d.stops ?? {}
    c.mission = { laps: 0, time_s: 0, then: '', ...(d.mission ?? {}) }
    c.signal_map = d.signal_map ?? {}
  } catch {
    // 壊れた設定は空から始める（保存すれば上書きされる）
  }
  return c
}

/** 編集を始める。今の地図に載っている設定を写し取る。 */
export function beginEdit(): void {
  routeEdit.cfg = parseCfg(live.map?.routesJson ?? '')
  routeEdit.active = true
  routeEdit.dirty = false
  routeEdit.awaitingYaw = false
}

export function endEdit(): void {
  routeEdit.active = false
  routeEdit.cfg = null
  routeEdit.dirty = false
  routeEdit.awaitingYaw = false
}

export function setTarget(t: string): void {
  routeEdit.target = t
  routeEdit.awaitingYaw = false
}

/**
 * 地図パネルのクリック（map フレーム）。`pxPerM` は削除判定の距離を画面の
 * ピクセルで揃えるための今の拡大率。
 */
export function editClick(x: number, y: number, pxPerM: number): void {
  const cfg = routeEdit.cfg
  if (!cfg) return
  const t = routeEdit.target
  if ((GROUPS as readonly string[]).includes(t)) {
    const pts = cfg.groups[t] ?? []
    const near = pts.findIndex(([px, py]) => Math.hypot(px! - x, py! - y) * pxPerM < 12)
    if (near >= 0) pts.splice(near, 1)
    else pts.push([round(x), round(y)])
    if (pts.length) cfg.groups[t] = pts
    else delete cfg.groups[t]
  } else {
    const cur = cfg.stops[t]
    if (routeEdit.awaitingYaw && cur) {
      cur.yaw = round(Math.atan2(y - cur.y, x - cur.x), 4)
      routeEdit.awaitingYaw = false
    } else {
      cfg.stops[t] = { x: round(x), y: round(y), yaw: null, mode: cur?.mode ?? 'stop' }
      routeEdit.awaitingYaw = true
    }
  }
  routeEdit.dirty = true
}

export function clearTarget(): void {
  const cfg = routeEdit.cfg
  if (!cfg) return
  const t = routeEdit.target
  if ((GROUPS as readonly string[]).includes(t)) delete cfg.groups[t]
  else {
    delete cfg.stops[t]
    if (cfg.mission.then === t) cfg.mission.then = ''
  }
  routeEdit.awaitingYaw = false
  routeEdit.dirty = true
}

export function undoLast(): void {
  const cfg = routeEdit.cfg
  const pts = cfg?.groups[routeEdit.target]
  if (!cfg || !pts?.length) return
  pts.pop()
  if (!pts.length) delete cfg.groups[routeEdit.target]
  routeEdit.dirty = true
}

function round(v: number, digits = 3): number {
  const k = 10 ** digits
  return Math.round(v * k) / k
}
