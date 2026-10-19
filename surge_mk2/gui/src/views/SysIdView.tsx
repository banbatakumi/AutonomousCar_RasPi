/**
 * システム同定タブ — 車両の校正の実車試験（`docs/calibration.md`）。
 *
 * 中身は自動運転と全く同じ仕組み（`raspi/auto/calib_*.py` も普通の `Planner`
 * サブクラスで、`category = "sysid"` が付いているだけ）に乗っている。
 * このタブが足しているのは「試験選択」の絞り込みと、**試験開始と同時に
 * mcap記録を始め、プランナーが終了合図（`AutoState.reason` が「完了」または「中止」で
 * 始まる）を出し、車が止まったら自動で記録を止めてダウンロードする**オーケストレーションだけ。
 *
 * 以前は `reason === "完了"` の完全一致だったため、「完了（後輪が滑った…）」や
 * 「中止: …」では終わらず、手動の「試験中止」が要った。止まるまで待つのは、試験が
 * 「完了」を出したあとも止まりきるまで制動するため（途中で disengage すると GUI の指令に切り替わる）。
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
  // 終了時の理由（「完了（後輪が滑った…）」など）。disengage すると reason が消えるので残す
  const [endReason, setEndReason] = useState('')
  // このビューが始めた試験が進行中でないか。**試験を始めるまでは true**——このビューはタブを
  // 替えても外されない（`App.tsx`）ので、自動運転タブで走らせている planner の「完了」を
  // 試験の終わりと取り違えて解除しないようにする
  const stoppedRef = useRef(true)
  // 試験を始めてから、サーバの engaged を一度でも見たか（外から解除されたことを見分ける）
  const sawEngagedRef = useRef(false)
  const startTimerRef = useRef<number | undefined>(undefined)
  // **試験開始ボタンの多重クリック対策。** `auto.engaged`はサーバのechoを
  // 待つ必要があり、ボタンの`disabled`が実際に効くまでの往復のわずかな間に
  // 連打されると`start()`（=RecordChannel）とengageが二重に走ってしまう。
  // ローカルで即座に立てるフラグでその隙間を塞ぐ（`auto.engaged`がTrueに
  // なったら役目を終えるので下のuseEffectで下ろす）
  const startingRef = useRef(false)

  const sysidCatalog = auto?.catalog.filter((c) => c.category === 'sysid') ?? []
  const selected = sysidCatalog.find((c) => c.id === auto?.mode) ?? null
  // ファイル名を試験ごとに分かりやすくする（例: calib_steer_20261019_220145.mcap）。
  // `selected`が変わるたびにフックの中身は作り直されるが、React
  // のフックはコンポーネント関数のトップレベルで毎回同じ順序で
  // 呼ばれていれば問題ない（条件分岐でスキップしていない）
  const { bufferedBytes, start } = useMcapDownload(ch, selected?.id ?? 'sysid')
  const st = n.auto
  const silent = !isFinite(n.autoAgeMs) || n.autoAgeMs > AUTO_STALE_MS
  const reason = st?.reason ?? ''
  const aborted = reason.startsWith('中止')
  const finished = !silent && (reason.startsWith('完了') || aborted) && n.vs?.stopped === true

  // ── 終了検知: プランナーが「完了」「中止」を出し車が止まったら自動で disengage → 記録停止 ──
  useEffect(() => {
    if (!auto?.engaged || !finished || stoppedRef.current) return
    stoppedRef.current = true
    if (aborted) setCanceled(true)
    setEndReason(reason)
    ch?.setAuto({ engaged: false })
    ch?.mcapRecordStop()
  }, [auto?.engaged, finished, aborted, reason, ch])

  // サーバが engaged を echo してきたら多重クリック対策フラグを下ろす。
  // **試験中に外から解除されたら（人の操作・E-STOP・操縦権の喪失）、記録も止める**——
  // 止めないと「試験中止」も押せないまま記録だけが続く。録れた分は中止と同じくダウンロードする
  useEffect(() => {
    if (auto?.engaged) {
      if (startingRef.current) {
        startingRef.current = false
        sawEngagedRef.current = true
        window.clearTimeout(startTimerRef.current)
      }
      return
    }
    if (stoppedRef.current || !sawEngagedRef.current) return
    stoppedRef.current = true
    setCanceled(true)
    setEndReason('自律走行が解除された')
    ch?.mcapRecordStop()
  }, [auto?.engaged, ch])

  useEffect(() => () => window.clearTimeout(startTimerRef.current), [])

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
    setEndReason('')
  }

  const beginTest = () => {
    if (!selected || active || startingRef.current) return
    startingRef.current = true
    setDownloaded(false)
    setCanceled(false)
    setEndReason('')
    stoppedRef.current = false
    sawEngagedRef.current = false
    start(false, () => setDownloaded(true))
    ch?.setAuto({ engaged: true })
    // engage は操縦権が無いと断られる（先に ARM して操縦権を取っておく必要がある）。
    // 断られたまま待ち続けると、記録だけが回り「試験開始」も押せなくなるので打ち切る
    window.clearTimeout(startTimerRef.current)
    startTimerRef.current = window.setTimeout(() => {
      if (!startingRef.current) return
      startingRef.current = false
      stoppedRef.current = true
      setCanceled(true)
      setEndReason('自律走行を開始できなかった（先に ARM して操縦権を取る）')
      ch?.mcapRecordStop()
    }, 3000)
  }

  // **中止も「記録を止めてダウンロード」までは正常終了と同じ経路を通す。**
  // 録れた範囲のログはフィッティングには使えなくても、何が起きたかを
  // 後から見返す手がかりになるので捨てない
  const cancelTest = () => {
    if (stoppedRef.current) {
      // このビューの試験ではない engage（リロード後など）。解除だけする
      ch?.setAuto({ engaged: false })
      return
    }
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
          {'車両の校正の試験（開始と同時に mcap 記録。手順・順番は docs/calibration.md）'}
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
          {/* 手順は測定に必要な条件を満たす組み合わせでプランナー側に固定してある
              （`raspi/auto/_calib_common.py`「手順は GUI から変えられない」） */}
          {selected.params.length > 0 ? (
            <>
              <span className="label">選択</span>
              <ParamSliders
                params={selected.params}
                values={auto.params}
                onChange={(key, value) => ch?.setAuto({ params: { [key]: value } })}
                disabled={active}
              />
            </>
          ) : (
            <p className="dim">手順は固定（測定に必要な条件を満たすよう決めてある）</p>
          )}
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
            {endReason && endReason !== '完了' && `（${endReason}）`}
            記録をダウンロードしました。Mac の解析ツール（launcher の「車両の校正」）でこのファイルを開いてください
          </p>
        )}
      </section>
    </div>
  )
}
