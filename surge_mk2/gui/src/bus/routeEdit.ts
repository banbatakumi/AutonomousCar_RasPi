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
 * - 「避ける」を編集中: **ドラッグで通ってはいけない範囲（矩形）を囲む**。範囲の中をクリックで
 *   その範囲を消す。planner は範囲を地図に壁として塗ってから道路グラフを作るので、範囲の外の
 *   道はそのまま使える（ルールで通れない近道・壁の隙間など）。古い設定の「避ける点」は
 *   表示し、近くをクリックで消せる（新しくは置かない。点は「いちばん近い道」へ吸着するので、
 *   隙間の上を指すと隣の本線ごと消えて周回が作れなくなった）
 */
import type { MapData } from '../types'
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
  /** 通ってはいけない範囲 `[x0, y0, x1, y1]`（x0≤x1・y0≤y1） */
  avoid_rects: number[][]
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
  /** 編集を始めたときの地図（`MapData` そのもの。地図が届くたびに作り直される）。
   *  **座標はこの地図のもの**なので、保存のときに今の地図と同じ格子かを確かめる */
  baseMap: null as MapData | null,
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
    avoid_rects: [],
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
    c.avoid_rects = d.avoid_rects ?? []
  } catch {
    // 壊れた設定は空から始める（保存すれば上書きされる）
  }
  return c
}

/** 編集を始める。今の地図に載っている設定を写し取る。 */
export function beginEdit(): void {
  routeEdit.cfg = parseCfg(live.map?.routesJson ?? '')
  routeEdit.baseMap = live.map
  routeEdit.active = true
  routeEdit.dirty = false
  routeEdit.awaitingYaw = false
}

export function endEdit(): void {
  routeEdit.active = false
  routeEdit.cfg = null
  routeEdit.baseMap = null
  routeEdit.dirty = false
  routeEdit.awaitingYaw = false
}

/**
 * 編集を始めてから地図が別物に替わったか（別の地図を読み込んだ・作り直した）。
 * 経路だけの更新や地図の成長では原点・解像度は変わらないので、**格子の置き方**で見分ける
 * （替わっていたら、編集中の座標は今の地図のものではない）。
 */
export function mapReplaced(): boolean {
  const a = routeEdit.baseMap
  const b = live.map
  if (!a || !b) return a !== b
  // 保存地図の名前が両方あれば、それで見分ける（確実）
  if (a.mapName && b.mapName) return a.mapName !== b.mapName
  if (a.res !== b.res || a.originX !== b.originX || a.originY !== b.originY
    || a.width !== b.width || a.height !== b.height) return true
  // 名前の無い地図（作成中・古い Pi）は、同じ置き方の別の地図への差し替えを格子だけでは
  // 見分けられない（`map_seq` も planner の作り直しで巻き戻る）。経路の選択や編集で変わらない
  // 中心線の点数・両端・中央の点で見分ける
  return centerSig(a.centerline) !== centerSig(b.centerline)
}

/** 中心線の簡易な指紋（点数・始点・中央・終点、mm 丸め） */
function centerSig(c: Float64Array): string {
  const n = c.length >> 1
  if (n === 0) return '0'
  const pt = (i: number) => `${Math.round(c[2 * i]! * 1000)},${Math.round(c[2 * i + 1]! * 1000)}`
  return `${n}:${pt(0)}:${pt(n >> 1)}:${pt(n - 1)}`
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
    // クリックは消すだけ（足すのはドラッグ、`editDrag`）。点を優先し、無ければ後に足した範囲から
    const near = cfg.avoid.findIndex(([px, py]) => Math.hypot(px! - x, py! - y) * pxPerM < 12)
    if (near >= 0) cfg.avoid.splice(near, 1)
    else {
      let k = cfg.avoid_rects.length - 1
      for (; k >= 0; k--) {
        const [x0, y0, x1, y1] = cfg.avoid_rects[k]!
        if (x >= x0! && x <= x1! && y >= y0! && y <= y1!) break
      }
      if (k < 0) return
      cfg.avoid_rects.splice(k, 1)
    }
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
      // 位置と同じ所（画面で6px以内）のクリックでは向きが決まらない（atan2(0, 0) = 0 で東向きになる）
      if (Math.hypot(x - cur.x, y - cur.y) * pxPerM < 6) return
      cur.yaw = round(Math.atan2(y - cur.y, x - cur.x), 4)
      routeEdit.awaitingYaw = false
    } else {
      cfg.stops[t] = { x: round(x), y: round(y), yaw: null, mode: cur?.mode ?? 'stop' }
      routeEdit.awaitingYaw = true
    }
  }
  routeEdit.dirty = true
}

/** 地図パネルのドラッグ（map フレームの2隅）。「避ける」の編集中だけ範囲を足す。 */
export function editDrag(x0: number, y0: number, x1: number, y1: number): void {
  const cfg = routeEdit.cfg
  if (!cfg || routeEdit.target !== AVOID) return
  cfg.avoid_rects.push([
    round(Math.min(x0, x1)), round(Math.min(y0, y1)),
    round(Math.max(x0, x1)), round(Math.max(y0, y1)),
  ])
  routeEdit.dirty = true
}

export function clearTarget(): void {
  const cfg = routeEdit.cfg
  if (!cfg) return
  const t = routeEdit.target
  if (t === AVOID) {
    cfg.avoid = []
    cfg.avoid_rects = []
  }
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
    if (cfg.avoid_rects.length) cfg.avoid_rects.pop()
    else cfg.avoid.pop()
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
