/**
 * システム同定タブ — `config/vehicle.toml` `[dynamics]` の実車試験（`docs/architecture.md` §5.3）。
 *
 * 中身は自動運転と全く同じ仕組み（`raspi/auto/sysid_*.py` も普通の `Planner`
 * サブクラスで、`category = "sysid"` が付いているだけ）に乗っている。
 * このタブが足しているのは「試験選択」の絞り込みと、**試験開始と同時に
 * mcap記録を始め、プランナーが完了合図（`AutoState.reason === "完了"`）を
 * 出したら自動で記録を止めてダウンロードする**オーケストレーションだけ。
 *
 * ## ARM は人間が保持する（省略しない）
 *
 * `engage` は「自律に任せてよい」という意思表示に過ぎず、実際に車が動くには
 * 通常の自動運転と同じく人間が ARM（Enter）を保持し続ける必要がある。
 * システム同定試験だからといってこの安全弁は外さない。
 */
import { useEffect, useRef, useState } from 'react'
import { useNumbers } from '../bus/live'
import { ParamSliders } from '../components/ParamSliders'
import { useMcapDownload } from '../hooks/useMcapDownload'
import { useUi } from '../store/ui'
import type { ControlChannel } from '../ws/control'

/** planner の判断がこれ以上古ければ「planning_node が落ちている」と見なす */
const AUTO_STALE_MS = 600

export function SysIdView({ ch }: { ch: ControlChannel | null }) {
  const ui = useUi()
  const auto = ui.auto
  const n = useNumbers()
  const [downloaded, setDownloaded] = useState(false)
  const [canceled, setCanceled] = useState(false)
  const stoppedRef = useRef(false)
  // **試験開始ボタンの多重クリック対策。** `auto.engaged`はサーバのechoを
  // 待つ必要があり、ボタンの`disabled`が実際に効くまでの往復のわずかな間に
  // 連打されると`start()`（=RecordChannel）とengageが二重に走ってしまう。
  // ローカルで即座に立てるフラグでその隙間を塞ぐ（`auto.engaged`がTrueに
  // なったら役目を終えるので下のuseEffectで下ろす）
  const startingRef = useRef(false)

  const sysidCatalog = auto?.catalog.filter((c) => c.category === 'sysid') ?? []
  const selected = sysidCatalog.find((c) => c.id === auto?.mode) ?? null
  // ファイル名を試験ごとに分かりやすくする（例: sysid_steer_20260831_220145.mcap）。
  // `selected`が変わるたびにフックの中身は作り直されるが、React
  // のフックはコンポーネント関数のトップレベルで毎回同じ順序で
  // 呼ばれていれば問題ない（条件分岐でスキップしていない）
  const { bufferedBytes, start } = useMcapDownload(ch, selected?.id ?? 'sysid')
  const st = n.auto
  const silent = !isFinite(n.autoAgeMs) || n.autoAgeMs > AUTO_STALE_MS
  const finished = !silent && st?.reason === '完了'

  // ── 完了検知: プランナーが「完了」を出したら自動で disengage → 記録停止 ──
  useEffect(() => {
    if (!auto?.engaged || !finished || stoppedRef.current) return
    stoppedRef.current = true
    ch?.setAuto({ engaged: false })
    ch?.mcapRecordStop()
  }, [auto?.engaged, finished, ch])

  // サーバが engaged を echo してきたら多重クリック対策フラグを下ろす
  useEffect(() => {
    if (auto?.engaged) startingRef.current = false
  }, [auto?.engaged])

  if (!auto) {
    return (
      <div className="settings sysid-view">
        <span className="dim">サーバに接続していません</span>
      </div>
    )
  }

  const setMode = (id: string) => {
    ch?.setAuto({ mode: id })
    setDownloaded(false)
    setCanceled(false)
  }

  const beginTest = () => {
    if (!selected || active || startingRef.current) return
    startingRef.current = true
    setDownloaded(false)
    setCanceled(false)
    stoppedRef.current = false
    start(false, () => setDownloaded(true))
    ch?.setAuto({ engaged: true })
  }

  // **中止も「記録を止めてダウンロード」までは正常終了と同じ経路を通す。**
  // 録れた範囲のログはフィッティングには使えなくても、何が起きたかを
  // 後から見返す手がかりになるので捨てない
  const cancelTest = () => {
    if (stoppedRef.current) return
    stoppedRef.current = true
    setDownloaded(false)
    setCanceled(true)
    ch?.setAuto({ engaged: false })
    ch?.mcapRecordStop()
  }

  const active = auto.engaged

  return (
    <div className="settings sysid-view">
      <div className="settings-head">
        <h2>システム同定</h2>
        <p className="dim">
          {'config/vehicle.toml の [dynamics]（実測待ちのパラメータ）を測るための試験。'}
          {'試験ごとにラズパイが自動で指令を出し、開始と同時にmcap記録を始める。'}
        </p>
      </div>

      <section className="settings-group">
        <span className="label">試験</span>
        <select value={selected?.id ?? ''} onChange={(e) => setMode(e.target.value)} disabled={active}>
          <option value="">（未選択）</option>
          {sysidCatalog.map((c) => (
            <option key={c.id} value={c.id}>
              {c.name}
            </option>
          ))}
        </select>
        {selected && <p className="auto-mode-desc dim">{selected.description}</p>}
      </section>

      {selected && (
        <section className="settings-group">
          <span className="label">パラメータ（{selected.params.length}）</span>
          <ParamSliders
            params={selected.params}
            values={auto.params}
            onChange={(key, value) => ch?.setAuto({ params: { [key]: value } })}
            disabled={active}
          />
        </section>
      )}

      <section className="settings-group">
        <div className="logs-row">
          <button className={active ? 'on' : ''} disabled={!ch || !selected || active} onClick={beginTest}>
            試験開始
          </button>
          <button disabled={!ch || !active} onClick={cancelTest}>
            試験中止
          </button>
          {active && !ui.armRequested && <span className="badge-warn">ARM してください（Enter）</span>}
          {active && ui.armRequested && !silent && <span className="badge-live">走行中</span>}
        </div>

        {active && (
          <p className="dim">
            記録中 {(bufferedBytes / 1e3).toFixed(0)}KB
          </p>
        )}

        {selected && (
          <div className="auto-state">
            {silent ? (
              <div className="badge-bad">planning_node から判断が届いていません</div>
            ) : (
              <div className={st?.ready ? 'auto-reason' : 'auto-reason lv-bad'}>{st?.reason || '—'}</div>
            )}
          </div>
        )}

        {downloaded && (
          <p className="badge-live">
            {canceled ? '試験を中止しました。' : '試験が完了しました。'}
            記録をダウンロードしました。Macの解析ツール（`tools/sysid`）でこのファイルを開いてください
          </p>
        )}
      </section>
    </div>
  )
}
