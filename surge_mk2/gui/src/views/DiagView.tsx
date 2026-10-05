/**
 * 診断ビュー — 層 D（走行後・調整時に開く、`architecture.md` §10.1）。
 *
 * 走行画面から外した数字の**正式な置き場**。使う場面は3つ:
 *
 *   走行前の点検   常設サマリの「走行可否」と、概要ページのチェックリスト
 *   異常の原因追跡 イベント帯（いつ・何と同時に）とイベントログ（なぜ止まったか）
 *   電源・熱の監視 電源・熱ページ（しきい値までの距離と上昇率）
 *
 * 走行中にこの画面を見る必要は無い。**異常は StatusBar が走行画面側で伝えるので、
 * ここは「原因を追う」ためだけの場所**にしてある。
 *
 * ## 構成（2026-10-05 に作り直した）
 *
 *   上段（常設）  サブページ切替・一時停止・時間幅 ／ 常設サマリ ／ イベント帯
 *   下段          サブページ（`components/diag/pages/`）。**開いているページのグラフだけ描く**
 *
 * グラフは全て同じ窓・同じカーソルを共有する（`components/diag/view.ts`）。
 * 一時停止しても記録は止まらない（`bus/history.ts` は常時貯める）。
 */
import { useMemo } from 'react'
import { useNumbers } from '../bus/live'
import { runChecks, verdict } from '../components/diag/checks'
import { EventTimeline } from '../components/diag/EventTimeline'
import { HealthStrip } from '../components/diag/HealthStrip'
import { DrivePage } from '../components/diag/pages/DrivePage'
import { LinkPage } from '../components/diag/pages/LinkPage'
import { LogsPage } from '../components/diag/pages/LogsPage'
import { OverviewPage } from '../components/diag/pages/OverviewPage'
import { PowerPage } from '../components/diag/pages/PowerPage'
import { SystemPage } from '../components/diag/pages/SystemPage'
import { PAGES, SPANS, useDiagView } from '../components/diag/view'
import { useUi } from '../store/ui'
import type { ControlChannel } from '../ws/control'

export function DiagView({ ch }: { ch: ControlChannel | null }) {
  const n = useNumbers()
  const st = useUi((s) => s.status)
  const { page, spanS, frozenAt, setPage, setSpan, togglePause } = useDiagView()

  const checks = useMemo(() => runChecks(n, st), [n, st])
  const v = verdict(checks)
  const props = { n, st, checks }

  return (
    <div className="diagview">
      <div className="dg-top">
        <div className="dg-bar">
          <div className="seg">
            {PAGES.map((p) => (
              <button key={p} className={p === page ? 'on' : ''} onClick={() => setPage(p)}>
                {p}
              </button>
            ))}
          </div>
          {n.stale && <span className="badge-bad">テレメトリ途絶中（表示は最後の値）</span>}
          <div className="spacer" />
          <button
            className={frozenAt != null ? 'on' : ''}
            onClick={togglePause}
            title="グラフとイベント帯を止めて過去を調べる。記録は止まらない"
          >
            {frozenAt != null ? '▶ 再開' : '⏸ 一時停止'}
          </button>
          <div className="seg">
            {SPANS.map((s) => (
              <button key={s} className={s === spanS ? 'on' : ''} onClick={() => setSpan(s)}>
                {s}秒
              </button>
            ))}
          </div>
        </div>
        <HealthStrip n={n} st={st} v={v} />
        <EventTimeline />
      </div>

      <div className="dg-page">
        {page === '記録' ? (
          <LogsPage ch={ch} />
        ) : !n.vs ? (
          <p className="dim dg-empty">テレメトリ未受信。Pi につながっていない。</p>
        ) : page === '概要' ? (
          <OverviewPage {...props} />
        ) : page === '電源・熱' ? (
          <PowerPage {...props} />
        ) : page === '駆動・制御' ? (
          <DrivePage {...props} />
        ) : page === '通信' ? (
          <LinkPage {...props} />
        ) : (
          <SystemPage {...props} />
        )}
      </div>
    </div>
  )
}
