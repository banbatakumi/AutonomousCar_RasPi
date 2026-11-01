/**
 * 経路パネル（`slam2d_route`、`catalog[].routes_ui`） — 走行中の経路グループ切替と、
 * 経由点・停止点・ミッションの編集。
 *
 * ## 切替ボタン（自動・A〜D）は「今すぐ乗り換え」ではない
 *
 * 「自動」は planner が地図から選ぶ最速の周回（経由点は持たない）。A〜D に経由点を置いても
 * 残るので、いつでも自動へ戻せる。
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
 *
 * ## 下見中（地図を選んだだけ）でも編集を始められる
 *
 * 保存先と作り直しの対象は planner が読んでいる地図なので、下見中に「経由点を編集」を押したら
 * 先にその地図を planner に読ませ（「自己位置を特定」と同じ送信。engage しないので走らない）、
 * 読み込んだ地図が届いてから編集を始める。保存すればその場で経路を作り直して地図パネルに出る。
 */
import { useEffect, useState } from 'react'
import { useNumbers, live } from '../bus/live'
import { clearPreview, mapPreview } from '../bus/mapPreview'
import {
  AVOID,
  GROUP_COLOR,
  GROUPS,
  ROUTE_KEYS,
  STOPS,
  beginEdit,
  clearTarget,
  endEdit,
  mapReplaced,
  routeEdit,
  routeLabel,
  setTarget,
  undoLast,
} from '../bus/routeEdit'
import type { ControlChannel } from '../ws/control'
import { askConfirm, notify } from './ConfirmDialog'

/** `signal_map` で編集できる信号の値。矢印信号（大会⑥）の左右 */
const SIGNALS = ['left', 'right'] as const

export function RoutePanel({ ch, mode }: { ch: ControlChannel | null; mode: string }) {
  const st = useNumbers().auto
  const [, setTick] = useState(0)
  const rerender = () => setTick((t) => t + 1)
  const ready = st?.route_groups ?? []
  const cfg = routeEdit.cfg
  // 下見中に押して、planner に読ませた地図が届くのを待っている（その地図の名前）
  const [waitMap, setWaitMap] = useState('')

  // このパネルが消えたら（モードの切り替え等）編集も終える。残すと、地図パネルのクリックが
  // 見えない編集に吸われ続ける
  useEffect(() => endEdit, [])

  // 読ませた地図が届いたら編集を始める（`auto/state` で 10Hz に描き直されるので毎回見る）。
  // 届く前に始めると、前の地図の設定を写し取り、保存で「地図が替わった」と弾かれる。
  // 待っている間に別の地図を下見し始めたらやめる
  useEffect(() => {
    if (!waitMap) return
    if (mapPreview.data) setWaitMap('')
    else if (live.map?.mapName === waitMap) {
      setWaitMap('')
      beginEdit()
      rerender()
    }
  })

  const startEdit = () => {
    if (mapPreview.data && mapPreview.name) {
      const name = mapPreview.name
      clearPreview()
      ch?.startSlam2dLocate(mode, name)
      setWaitMap(name)
      return
    }
    beginEdit()
    rerender()
  }

  const save = () => {
    if (!cfg) return
    if (mapPreview.data) {
      notify('別の地図を下見している間は保存しません（地図パネルに出ているのは planner が読んでいる地図ではありません）。「やめる」で閉じて、選んだ地図で編集し直してください')
      return
    }
    if (mapReplaced()) {
      notify('編集を始めてから地図が替わりました。経由点の座標は前の地図のものなので保存しません。「やめる」で閉じて、今の地図で編集し直してください')
      return
    }
    for (const [name, s] of Object.entries(cfg.stops)) {
      if (s.mode === 'park' && s.yaw === null) {
        notify(`停止点 ${name} は駐車なので向きが要ります（位置のあとにもう1回クリック）`)
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
          // ★ 下見中（`mapPreview`）は、その地図を planner に読ませてから編集する（`startEdit`）。
          //   下見の地図のまま編集させると、保存は planner が今読んでいる地図（別の地図・地図なし）に
          //   書かれ、「自己位置を特定」で読み直すと消えた。名前の無い地図（作成中）は保存先が無い
          disabled={!ch || waitMap !== '' || !(mapPreview.data ? mapPreview.name : live.map?.mapName)}
          title={waitMap ? `地図 ${waitMap} を読み込んでいる`
            : mapPreview.data ? '選んだ地図を読み込んで編集する（自己位置の特定も始まる。走り出しはしない）'
            : !live.map?.mapName ? '地図を選ぶと編集できます' : undefined}
          onClick={startEdit}
        >
          {waitMap ? '地図を読み込み中…' : '経由点を編集'}
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
  const nAvoid = cfg.avoid.length + cfg.avoid_rects.length

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
          title="通ってはいけない範囲をドラッグで囲む（地図に壁として塗ってから経路を作る。自動経路・経由点の経路・停止経路のどれも通らない）"
          onClick={() => {
            setTarget(AVOID)
            rerender()
          }}
        >
          ✕避ける{nAvoid ? `(${nAvoid})` : ''}
        </button>
      </div>
      <div className="dim route-hint">
        {isAvoid
          ? '通ってはいけない範囲をドラッグで囲む（範囲の中をクリックで削除）'
          : isGroup
          ? `経路${t}の経由点を地図上に順に置く（点の近くで削除）`
          : routeEdit.awaitingYaw
            ? `${t} の向き: 車の前が向く方向をクリック`
            : `${t} の位置をクリック（続けて向きをクリック）`}
      </div>
      <div className="route-row">
        {isGroup || isAvoid ? (
          <button onClick={() => upd(undoLast)}>{isAvoid ? '最後の範囲を取消' : '最後の点を取消'}</button>
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
        <button onClick={() => upd(clearTarget)}>{isAvoid ? '避ける範囲を全部消す' : `${t} を消す`}</button>
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
            const close = () => {
              endEdit()
              rerender()
            }
            if (!routeEdit.dirty) close()
            else void askConfirm('変更を捨てますか？').then((ok) => ok && close())
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
