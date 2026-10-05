/**
 * 概要 — 走行前チェックリストとイベントログ。
 *
 * チェックは「走ってよいか」を上から順に読めば分かる一覧。異常・注意の行を押すと
 * 詳しい数字があるサブページへ飛ぶ。イベントログは状態が変わった瞬間の記録で、
 * 「なぜ止まったか」（指令元が deadman に変わった、フォルトが立った、など）を時刻つきで追う。
 */
import { eventLog } from '../../../bus/history'
import { ChartGrid } from '../Chart'
import { Card, Dot } from '../Card'
import { CH } from '../chartDefs'
import type { Check } from '../checks'
import { useDiagView } from '../view'
import type { PageProps } from './types'

const CHARTS = [CH.speed, CH.voltage]
const LOG_ROWS = 80

function groups(checks: Check[]): [string, Check[]][] {
  const m = new Map<string, Check[]>()
  for (const c of checks) m.set(c.group, [...(m.get(c.group) ?? []), c])
  return [...m]
}

export function OverviewPage({ checks }: PageProps) {
  const setPage = useDiagView((s) => s.setPage)
  // 新しいものを上に。配列は書き換わり続けるが、この部品は 8Hz で描き直されるので追従する
  const log = eventLog().slice(-LOG_ROWS).reverse()
  return (
    <>
      <div className="dg-cards">
        <Card title="走行前チェック" wide>
          <div className="dg-checks">
            {groups(checks).map(([group, items]) => (
              <div key={group} className="dg-check-group">
                <h5>{group}</h5>
                {items.map((c) => (
                  <button key={c.label} className={`dg-check is-${c.level}`} onClick={() => setPage(c.page)}>
                    <Dot level={c.level} />
                    <span>{c.label}</span>
                    <b>{c.value}</b>
                  </button>
                ))}
              </div>
            ))}
          </div>
        </Card>
        <Card title="イベントログ" wide>
          {log.length === 0 ? (
            <p className="dim dg-empty">状態の変化はまだ無い</p>
          ) : (
            <ol className="dg-log">
              {log.map((e, i) => (
                <li key={`${e.wall}-${i}`} className={`is-${e.level}`}>
                  <time>{new Date(e.wall).toLocaleTimeString()}</time>
                  <Dot level={e.level === 'info' ? 'na' : e.level} />
                  <span>{e.text}</span>
                </li>
              ))}
            </ol>
          )}
        </Card>
      </div>
      <ChartGrid defs={CHARTS} />
    </>
  )
}
