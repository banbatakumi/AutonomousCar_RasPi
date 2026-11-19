/**
 * システム同定タブ — 車両の校正の実車試験（`docs/calibration.md`）。
 *
 * 中身は自動運転と全く同じ仕組み（`raspi/auto/calib_*.py` も普通の `Planner`
 * サブクラスで、`category = "sysid"` が付いているだけ）に乗っている。
 * このタブが足しているのは「試験選択」の絞り込みと、**試験開始と同時に
 * mcap記録を始め、プランナーが終了合図（`AutoState.reason` が「完了」または「中止」で
 * 始まる）を出し、車が止まったら自動で記録を止めてダウンロードする**オーケストレーションだけ。
 *
 * 前方一致で見るのは、「完了（後輪が滑った…）」や「中止: …」のように理由が続くため。止まるまで待つのは、試験が
 * 「完了」を出したあとも止まりきるまで制動するため（途中で disengage すると GUI の指令に切り替わる）。
 *
 * ## ARM は人間が保持する（省略しない）
 *
 * `engage` は「自律に任せてよい」という意思表示に過ぎず、実際に車が動くには
 * 通常の自動運転と同じく人間が ARM（Enter）を保持し続ける必要がある。
 * システム同定試験だからといってこの安全弁は外さない。
 */
import { useEffect, useReducer } from 'react'
import { useNumbers } from '../bus/live'
import { ParamSliders } from '../components/ParamSliders'
import { useMcapDownload } from '../hooks/useMcapDownload'
import { useUi } from '../store/ui'
import { useShallow } from 'zustand/react/shallow'
import type { ControlChannel } from '../ws/control'

/** planner の判断がこれ以上古ければ「planning_node が落ちている」と見なす */
const AUTO_STALE_MS = 600
/**
 * engage の返事（サーバの engaged の echo か、`control_denied`）がこれだけ来なければ諦める [ms]。
 * 断られた場合は `control_denied` で**すぐ**分かる（`ui.autoDenial`）。これは送信そのものが届かなかった
 * （control の接続が切れていて黙って捨てられた）ときの受け皿
 */
const START_NO_REPLY_MS = 3000

/**
 * 試験の進み具合。**このビューが始めた試験だけを追う**——このビューはタブを替えても外されない
 * （`App.tsx`）ので、自動運転タブで走らせている planner の「完了」を試験の終わりと取り違えない。
 * - `idle`: 試験をしていない（始める前・終わった後）
 * - `starting`: engage を送って返事を待っている（多重クリックもここで弾く）
 * - `running`: サーバの engaged を見た（以後に外から解除されたら記録も止める）
 */
type TestState = {
  phase: 'idle' | 'starting' | 'running'
  downloaded: boolean
  canceled: boolean
  /** 終了時の理由（「完了（後輪が滑った…）」など）。disengage すると reason が消えるので残す */
  endReason: string
  /** 始めた時点の `autoDenial.seq`。これより新しい拒否だけを、この試験への返事とみなす */
  denialSeqAtStart: number
}

type TestAction =
  | { type: 'clear' }
  | { type: 'start'; denialSeq: number }
  | { type: 'engaged' }
  | { type: 'end'; canceled: boolean; reason?: string }
  | { type: 'downloaded' }

const INITIAL: TestState = { phase: 'idle', downloaded: false, canceled: false, endReason: '', denialSeqAtStart: 0 }

function reduce(st: TestState, a: TestAction): TestState {
  switch (a.type) {
    case 'clear':
      return { ...st, downloaded: false, canceled: false, endReason: '' }
    case 'start':
      return { ...st, phase: 'starting', downloaded: false, canceled: false, endReason: '', denialSeqAtStart: a.denialSeq }
    case 'engaged':
      return st.phase === 'starting' ? { ...st, phase: 'running' } : st
    case 'end':
      return {
        ...st,
        phase: 'idle',
        canceled: a.canceled,
        endReason: a.reason ?? st.endReason,
        downloaded: a.canceled && a.reason === undefined ? false : st.downloaded,
      }
    case 'downloaded':
      return { ...st, downloaded: true }
  }
}

const DENIAL_TEXT: Record<string, string> = {
  auto_engage: '自律走行を開始できなかった（先に ARM して操縦権を取る）',
  auto_busy: '自律走行を開始できなかった（他の操縦者が自律走行中）',
}

export function SysIdView({ ch }: { ch: ControlChannel | null }) {
  const ui = useUi(useShallow((s) => ({ armRequested: s.armRequested, auto: s.auto, autoDenial: s.autoDenial })))
  const auto = ui.auto
  const n = useNumbers()
  const [test, dispatch] = useReducer(reduce, INITIAL)
  const { phase, downloaded, canceled, endReason } = test

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
    if (!auto?.engaged || !finished || phase === 'idle') return
    dispatch({ type: 'end', canceled: aborted, reason })
    ch?.setAuto({ engaged: false })
    ch?.mcapRecordStop()
  }, [auto?.engaged, finished, aborted, reason, ch, phase])

  // サーバが engaged を echo してきたら走行中に進める。
  // **試験中に外から解除されたら（人の操作・E-STOP・操縦権の喪失）、記録も止める**——
  // 止めないと「試験中止」も押せないまま記録だけが続く。録れた分は中止と同じくダウンロードする
  useEffect(() => {
    if (auto?.engaged) {
      dispatch({ type: 'engaged' })
      return
    }
    if (phase !== 'running') return
    dispatch({ type: 'end', canceled: true, reason: '自律走行が解除された' })
    ch?.mcapRecordStop()
  }, [auto?.engaged, ch, phase])

  // **engage が断られたら（`control_denied`）すぐに打ち切る。** 断られたまま待ち続けると、記録だけが回り
  // 「試験開始」も押せなくなる。返事そのものが来ないときは `START_NO_REPLY_MS` で諦める
  const denial = ui.autoDenial
  useEffect(() => {
    if (phase !== 'starting') return
    const fail = (why: string) => {
      dispatch({ type: 'end', canceled: true, reason: why })
      ch?.mcapRecordStop()
    }
    if (denial && denial.seq > test.denialSeqAtStart) {
      fail(DENIAL_TEXT[denial.reason] ?? '自律走行を開始できなかった')
      return
    }
    const t = window.setTimeout(() => fail('自律走行を開始できなかった（Pi から返事が無い）'), START_NO_REPLY_MS)
    return () => window.clearTimeout(t)
  }, [phase, denial, test.denialSeqAtStart, ch])

  if (!auto) {
    return (
      <div className="settings sysid-view">
        <span className="dim">サーバに接続していません</span>
      </div>
    )
  }

  const setMode = (id: string) => {
    ch?.setAuto({ mode: id })
    dispatch({ type: 'clear' })
  }

  const beginTest = () => {
    // 多重クリック対策: `auto.engaged` はサーバの echo を待つので、返事が来るまでは `starting` で弾く
    if (!selected || active || phase !== 'idle') return
    dispatch({ type: 'start', denialSeq: denial?.seq ?? 0 })
    start(false, () => dispatch({ type: 'downloaded' }))
    // engage は操縦権が無いと断られる（先に ARM して操縦権を取っておく必要がある）
    ch?.setAuto({ engaged: true })
  }

  // **中止も「記録を止めてダウンロード」までは正常終了と同じ経路を通す。**
  // 録れた範囲のログはフィッティングには使えなくても、何が起きたかを
  // 後から見返す手がかりになるので捨てない
  const cancelTest = () => {
    if (phase === 'idle') {
      // このビューの試験ではない engage（リロード後など）。解除だけする
      ch?.setAuto({ engaged: false })
      return
    }
    dispatch({ type: 'end', canceled: true })
    ch?.setAuto({ engaged: false })
    ch?.mcapRecordStop()
  }

  const active = auto.engaged

  return (
    <div className="settings sysid-view">
      <div className="settings-head">
        <h2>
          システム同定<span className="page-sub">CALIB</span>
        </h2>
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
          <button
            className={`sysid-start ${active ? 'on' : ''}`}
            disabled={!ch || !selected || active}
            onClick={beginTest}
          >
            試験開始
          </button>
          <button className="sysid-stop" disabled={!ch || !active} onClick={cancelTest}>
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
