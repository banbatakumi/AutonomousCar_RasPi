/**
 * 経路パネル（`slam2d_route`、`catalog[].routes_ui`） — 走行中の経路グループ切替と、
 * 経由点・停止点・ミッションの編集。
 *
 * ## 切替ボタン（自動・A〜D）は「今すぐ乗り換え」ではない
 *
 * 「自動」は planner が地図から選ぶ最速の周回（経由点は持たない）。A〜D に経由点を置いても
 * 残るので、いつでも自動へ戻せる（2026-09-29）。
 *
 * 押すと planner は切替を**保留**し、車が新しい経路の上に乗れる地点で乗り換える
 * （`raspi/nav/route_switch.py`）。分岐の手前で押せばその分岐から、分岐を過ぎて
 * から押せば合流したところで乗り換えて次の周から新しい枝を通る。「待ち」の表示は
 * その保留中であることを示す。
 *
 * ## 編集は手元の写しを触り、「保存」で送る
 *
 * 状態は `bus/routeEdit.ts`（地図パネルのクリックと描画ループが直接読む）。
 * 保存すると planner が検証して地図と同じ名前の `.routes.json` に書き、
 * 経由点が変わっていれば経路を作り直す（走行中でもよい。作り直した経路へは
 * 切替と同じく乗れる地点で移る）。
 */
import { useState } from 'react'
import { useNumbers, live } from '../bus/live'
import {
  AVOID,
  GROUP_COLOR,
  GROUPS,
  ROUTE_KEYS,
  STOPS,
  beginEdit,
  clearTarget,
  endEdit,
  routeEdit,
  routeLabel,
  setTarget,
  undoLast,
} from '../bus/routeEdit'
import type { ControlChannel } from '../ws/control'

/** `signal_map` で編集できる信号の値。矢印信号（大会⑥）の左右 */
const SIGNALS = ['left', 'right'] as const

export function RoutePanel({ ch }: { ch: ControlChannel | null }) {
  const st = useNumbers().auto
  const [, setTick] = useState(0)
  const rerender = () => setTick((t) => t + 1)
  const ready = st?.route_groups ?? []
  const cfg = routeEdit.cfg

  const save = () => {
    if (!cfg) return
    for (const [name, s] of Object.entries(cfg.stops)) {
      if (s.mode === 'park' && s.yaw === null) {
        window.alert(`停止点 ${name} は駐車なので向きが要ります（位置のあとにもう1回クリック）`)
        return
      }
    }
    ch?.saveRoutes(JSON.stringify(cfg))
    endEdit()
    rerender()
  }

  return (
    <section className="route-panel">
      <span className="label">経路</span>
      <div className="route-switch">
        {ROUTE_KEYS.map((g) => (
          <button
            key={g}
            className={
              st?.route_active === g ? 'route-btn on' : st?.route_pending === g ? 'route-btn pending' : 'route-btn'
            }
            style={{ borderColor: GROUP_COLOR[g] }}
            disabled={!ready.includes(g) || !ch}
            title={
              ready.includes(g)
                ? `${routeLabel(g)}へ切り替える（乗れる地点で乗り換える）`
                : `${routeLabel(g)}はまだ無い`
            }
            onClick={() => ch?.selectRoute(g)}
          >
            {g === 'auto' ? '自動' : g}
          </button>
        ))}
        <span className="dim">
          {st?.route_active ? `走行 ${routeLabel(st.route_active)}` : '—'}
          {st?.route_pending ? ` → ${routeLabel(st.route_pending)} 待ち` : ''}
          {st?.route_source ? `（${st.route_source}）` : ''}
        </span>
      </div>
      {st?.mission && <div className="dim">ミッション: {st.mission}</div>}
      {st?.route_note && <div className="route-note">{st.route_note}</div>}

      {!cfg ? (
        <button
          disabled={!live.map}
          onClick={() => {
            beginEdit()
            rerender()
          }}
        >
          経由点を編集
        </button>
      ) : (
        <RouteEditor rerender={rerender} onSave={save} />
      )}
    </section>
  )
}

function RouteEditor({ rerender, onSave }: { rerender: () => void; onSave: () => void }) {
  const cfg = routeEdit.cfg!
  const t = routeEdit.target
  const isGroup = (GROUPS as readonly string[]).includes(t)
  const isAvoid = t === AVOID
  const stop = cfg.stops[t]
  const stopNames = Object.keys(cfg.stops)

  const upd = (f: () => void) => {
    f()
    routeEdit.dirty = true
    rerender()
  }

  return (
    <div className="route-edit">
      <div className="route-targets">
        {[...GROUPS, ...STOPS].map((k) => (
          <button
            key={k}
            className={t === k ? 'on' : ''}
            style={GROUP_COLOR[k] ? { borderColor: GROUP_COLOR[k] } : undefined}
            onClick={() => {
              setTarget(k)
              rerender()
            }}
          >
            {k}
            {isCount(cfg, k)}
          </button>
        ))}
        <button
          className={isAvoid ? 'on' : ''}
          style={{ borderColor: '#e0574d' }}
          title="通ってはいけない道に点を置く（自動経路・経由点の経路・停止経路のどれにも使わない）"
          onClick={() => {
            setTarget(AVOID)
            rerender()
          }}
        >
          ✕避ける{cfg.avoid.length ? `(${cfg.avoid.length})` : ''}
        </button>
      </div>
      <div className="dim route-hint">
        {isAvoid
          ? '通ってはいけない道の上をクリック（点の近くをクリックで削除）。いちばん近い道を使わなくなる'
          : isGroup
          ? `地図をクリックして経路${t}の経由点を順に置く（点の近くをクリックで削除）。最後の点から最初の点へ戻って1周になる。経由点がどのグループにも無い間は、スタートから戻る周回を全部比べた最速の経路（自動）で走る`
          : routeEdit.awaitingYaw
            ? `${t} の向き: 車の前が向く方向をクリック`
            : `${t} の位置をクリック（続けて向きをクリック）`}
      </div>
      <div className="route-row">
        {isGroup || isAvoid ? (
          <button onClick={() => upd(undoLast)}>最後の点を取消</button>
        ) : (
          stop && (
            <label>
              <select
                value={stop.mode}
                onChange={(e) => upd(() => (stop.mode = e.target.value as 'stop' | 'park'))}
              >
                <option value="stop">止まる</option>
                <option value="park">駐車（手前で止まってから）</option>
              </select>
            </label>
          )
        )}
        <button onClick={() => upd(clearTarget)}>{isAvoid ? '避ける点を全部消す' : `${t} を消す`}</button>
        <label>
          開始
          <select value={cfg.active} onChange={(e) => upd(() => (cfg.active = e.target.value))}>
            {ROUTE_KEYS.map((g) => (
              <option key={g} value={g}>
                {g === 'auto' ? '自動' : g}
              </option>
            ))}
          </select>
        </label>
      </div>
      <div className="route-row">
        <span className="dim">ミッション</span>
        <label>
          <input
            type="number"
            min={0}
            step={1}
            value={cfg.mission.laps}
            onChange={(e) => upd(() => (cfg.mission.laps = Math.max(0, Math.round(+e.target.value))))}
          />
          周
        </label>
        <label>
          <input
            type="number"
            min={0}
            step={10}
            value={cfg.mission.time_s}
            onChange={(e) => upd(() => (cfg.mission.time_s = Math.max(0, +e.target.value)))}
          />
          秒
        </label>
        <label>
          →
          <select value={cfg.mission.then} onChange={(e) => upd(() => (cfg.mission.then = e.target.value))}>
            <option value="">（走り続ける）</option>
            {stopNames.map((s) => (
              <option key={s} value={s}>
                {s}
              </option>
            ))}
          </select>
        </label>
      </div>
      <div className="route-row">
        <span className="dim">信号</span>
        {SIGNALS.map((sig) => (
          <label key={sig}>
            {sig}→
            <select
              value={cfg.signal_map[sig] ?? ''}
              onChange={(e) =>
                upd(() => {
                  if (e.target.value) cfg.signal_map[sig] = e.target.value
                  else delete cfg.signal_map[sig]
                })
              }
            >
              <option value="">—</option>
              {ROUTE_KEYS.map((g) => (
                <option key={g} value={g}>
                  {g === 'auto' ? '自動' : g}
                </option>
              ))}
            </select>
          </label>
        ))}
      </div>
      <div className="route-row">
        <button className="primary" onClick={onSave}>
          保存{routeEdit.dirty ? '（未保存あり）' : ''}
        </button>
        <button
          onClick={() => {
            if (routeEdit.dirty && !window.confirm('変更を捨てますか？')) return
            endEdit()
            rerender()
          }}
        >
          やめる
        </button>
      </div>
    </div>
  )
}

function isCount(cfg: NonNullable<typeof routeEdit.cfg>, k: string): string {
  const n = cfg.groups[k]?.length
  if (n) return `(${n})`
  return cfg.stops[k] ? '✓' : ''
}
