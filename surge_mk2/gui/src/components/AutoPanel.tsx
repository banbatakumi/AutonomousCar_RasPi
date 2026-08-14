/**
 * 自動運転の操作盤 — **どのモードで走るかを選び、engage する。**
 *
 * ## モードの一覧をここに書かない
 *
 * 選択肢もパラメータのスライダも、サーバから降ってくる `status.auto.catalog`
 * （`raspi/auto/registry.py`）だけで組み立てている。**planner を1つ足しても
 * このファイルは 1 行も変わらない。** モード名を GUI 側にも書くと、増やすたびに
 * 2箇所を直すことになり、いつか片方だけ古くなる。
 *
 * ## engage の状態を GUI 側で持たない
 *
 * ボタンを押したら `setAuto` を投げるだけで、表示は必ずサーバの `status` を見る。
 * 「押したから走っているはず」で描くと、サーバが拒否したとき（モード未選択・
 * E-Stop・操縦権の喪失）に**画面だけ走っていることになる**。
 *
 * ## engage ≠ 走る
 *
 * engage は「自律に任せてよい」という意思表示で、実際に車が動くには**人間が
 * ARM を保持している**必要がある（`Enter` または ARM ボタン）。この2つを1つの
 * ボタンにまとめないのは、自律走行の停止手段を人間側に残しておくため。
 */
import { useNumbers } from '../bus/live'
import { RAD2DEG, mps } from '../format'
import { useUi } from '../store/ui'
import type { ControlChannel } from '../ws/control'

/** planner の判断がこれ以上古ければ「planning_node が落ちている」と見なす */
const AUTO_STALE_MS = 600

export function AutoPanel({ ch }: { ch: ControlChannel | null }) {
  const ui = useUi()
  const auto = ui.auto
  const n = useNumbers()

  if (!auto) {
    return (
      <div className="autopanel">
        <span className="dim">サーバに接続していません</span>
      </div>
    )
  }

  const selected = auto.catalog.find((c) => c.id === auto.mode) ?? null
  const st = n.auto
  // **「planner が黙っている」と「planner が止まれと言っている」を区別する。**
  // 前者は planning_node が落ちている疑いで、対処が全く違う
  const silent = !isFinite(n.autoAgeMs) || n.autoAgeMs > AUTO_STALE_MS

  const setMode = (id: string) => {
    ch?.setAuto({ mode: auto.mode === id ? '' : id })
    ui.set({ autoOffReason: '' })
  }

  const toggleEngage = () => {
    const next = !auto.engaged
    ch?.setAuto({ engaged: next })
    ui.set({ autoOffReason: next ? '' : '自動運転タブから解除' })
  }

  return (
    <div className={`autopanel${auto.engaged ? ' engaged' : ''}`}>
      {/* ── モード選択 ── */}
      <section className="auto-modes">
        <span className="label">走らせ方</span>
        <div className="auto-mode-row">
          {auto.catalog.map((c) => (
            <button
              key={c.id}
              className={c.id === auto.mode ? 'on' : ''}
              onClick={() => setMode(c.id)}
              title={c.description}
            >
              {c.name}
            </button>
          ))}
          {auto.catalog.length === 0 && (
            <span className="dim">planner がありません</span>
          )}
        </div>
      </section>

      {/* ── engage ── */}
      <section className="auto-engage">
        <button
          className={auto.engaged ? 'engage on' : 'engage'}
          disabled={!auto.mode}
          onClick={toggleEngage}
        >
          {auto.engaged ? '自律走行を解除' : '自律走行を開始'}
        </button>
        {/* **engage と ARM は別物**。どちらが欠けているかを名指しで出す */}
        {auto.engaged && !ui.armRequested && (
          <span className="badge-warn">ARM してください（Enter）</span>
        )}
        {auto.engaged && ui.armRequested && !silent && (
          <span className="badge-live">走行中</span>
        )}
        {!auto.engaged && ui.autoOffReason && (
          <span className="badge-warn">{ui.autoOffReason}</span>
        )}
        {auto.stalls > 0 && (
          <span className="badge-bad">指令の途絶で制動 {auto.stalls} 回</span>
        )}
      </section>

      {/* ── planner の判断 ── */}
      <section className="auto-state">
        <span className="label">判断</span>
        {silent ? (
          <div className="badge-bad">
            planning_node から判断が届いていません
            {selected ? '（プロセスが落ちていないか確認）' : ''}
          </div>
        ) : (
          <>
            <div className={st?.ready ? 'auto-reason' : 'auto-reason lv-bad'}>
              {st?.reason || '—'}
            </div>
            <div className="auto-metrics">
              <b>{mps(st?.target_speed ?? 0)}</b>
              <i>m/s 指令</i>
              <b>{((st?.target_steer ?? 0) * RAD2DEG).toFixed(1)}</b>
              <i>° 舵</i>
              <b>{(st?.free_ahead ?? 0).toFixed(2)}</b>
              <i>m 正面</i>
              <b>{(st?.nearest ?? 0).toFixed(2)}</b>
              <i>m 最近傍</i>
              <b>
                {st ? `${st.gap_start_deg.toFixed(0)}〜${st.gap_end_deg.toFixed(0)}` : '—'}
              </b>
              <i>° ギャップ</i>
              <b>{((st?.valid_ratio ?? 0) * 100).toFixed(0)}</b>
              <i>% 有効点</i>
              <b>{(st?.plan_hz ?? 0).toFixed(1)}</b>
              <i>Hz 計画</i>
            </div>
          </>
        )}
      </section>

      {/* ── パラメータ ──
          既定で畳んである。**走行中に見たいのは上の3つ**で、ここは詰める作業を
          するときだけ開けばよい（設定ドロワーを歯車に隠しているのと同じ作法） */}
      {selected && (
        <details className="auto-params">
          <summary>パラメータ（{selected.params.length}）</summary>
          <div className="auto-param-grid">
            {selected.params.map((p) => {
              const v = auto.params[p.key] ?? p.default
              return (
                <label key={p.key} className="auto-param" title={p.note}>
                  <span className="auto-param-head">
                    {p.label}
                    <b>
                      {v.toFixed(decimalsFor(p.step))}
                      {p.unit && <i> {p.unit}</i>}
                    </b>
                  </span>
                  <input
                    type="range"
                    min={p.min}
                    max={p.max}
                    step={p.step}
                    value={v}
                    onChange={(e) =>
                      ch?.setAuto({ params: { [p.key]: Number(e.target.value) } })
                    }
                  />
                </label>
              )
            })}
          </div>
        </details>
      )}
    </div>
  )
}

/** スライダの刻みから表示桁数を決める。0.01 なら2桁、1 なら0桁 */
function decimalsFor(step: number): number {
  if (step >= 1) return 0
  return Math.min(3, Math.ceil(-Math.log10(step)))
}
