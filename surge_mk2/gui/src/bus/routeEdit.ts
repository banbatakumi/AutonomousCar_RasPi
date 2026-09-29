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
 * - 「避ける」を編集中: 避ける点を足す（近くをクリックで消す）。いちばん近い道を
 *   自動経路・経由点の経路・停止経路のどれにも使わない（ルールで通れない近道など）
 */
import { live } from './live'

export const GROUPS = ['A', 'B', 'C', 'D'] as const
/** 走行中に選べる経路（自動経路＋A〜D）。**自動経路は経由点を持たないので編集対象ではない**
 *  が、切替・開始・信号の行き先には選べる（`raspi/auto/route_config.py` の `ROUTE_KEYS`） */
export const ROUTE_KEYS = ['auto', ...GROUPS] as const
export const STOPS = ['P1', 'P2', 'P3'] as const
/** 編集対象「避ける点」のキー */
export const AVOID = 'avoid'

export type Stop = { x: number; y: number; yaw: number | null; mode: 'stop' | 'park' }
export type RouteCfg = {
  groups: Record<string, number[][]>
  active: string
  stops: Record<string, Stop>
  mission: { laps: number; time_s: number; then: string }
  signal_map: Record<string, string>
  avoid: number[][]
}

/** グループごとの色（経路・経由点で共通）。停止経路は白、自動経路は灰 */
export const GROUP_COLOR: Record<string, string> = {
  auto: '#9aa7b3',
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

/** 経路のキーの表示名。`auto` は経由点が無いときに planner が地図作成の1周から作る経路
 *  （経路の設定には経由点を書かない、`slam2d_route.py`） */
export function routeLabel(key: string): string {
  if (key === 'auto') return '自動（最速の周回）'
  if (key === 'stop') return '停止点へ'
  return key
}

export function emptyCfg(): RouteCfg {
  return {
    groups: {},
    active: 'auto',
    stops: {},
    mission: { laps: 0, time_s: 0, then: '' },
    signal_map: {},
    avoid: [],
  }
}

export function parseCfg(text: string): RouteCfg {
  const c = emptyCfg()
  if (!text) return c
  try {
    const d = JSON.parse(text) as Partial<RouteCfg>
    c.groups = d.groups ?? {}
    c.active = d.active ?? 'auto'
    c.stops = d.stops ?? {}
    c.mission = { laps: 0, time_s: 0, then: '', ...(d.mission ?? {}) }
    c.signal_map = d.signal_map ?? {}
    c.avoid = d.avoid ?? []
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
  if (t === AVOID) {
    const near = cfg.avoid.findIndex(([px, py]) => Math.hypot(px! - x, py! - y) * pxPerM < 12)
    if (near >= 0) cfg.avoid.splice(near, 1)
    else cfg.avoid.push([round(x), round(y)])
  } else if ((GROUPS as readonly string[]).includes(t)) {
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
  if (t === AVOID) cfg.avoid = []
  else if ((GROUPS as readonly string[]).includes(t)) delete cfg.groups[t]
  else {
    delete cfg.stops[t]
    if (cfg.mission.then === t) cfg.mission.then = ''
  }
  routeEdit.awaitingYaw = false
  routeEdit.dirty = true
}

export function undoLast(): void {
  const cfg = routeEdit.cfg
  if (cfg && routeEdit.target === AVOID) {
    cfg.avoid.pop()
    routeEdit.dirty = true
    return
  }
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
