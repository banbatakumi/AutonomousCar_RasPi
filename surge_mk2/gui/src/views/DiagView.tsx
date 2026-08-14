/**
 * 診断ビュー — 層 D（走行後・調整時に開く、`architecture.md` §10.1）。
 *
 * ラジコン・自動運転の両ビューから外した数字の**正式な置き場**。
 * ここに来れば、温度・電圧・電流・スリップ・リンク統計・MD バス・時刻同期が
 * 現在値と時系列の両方で見られる。
 *
 * 走行中にこの画面を見る必要は無い。**異常は StatusBar と DiagStrip が
 * 走行画面側で伝えるので、ここは「原因を追う」ためだけの場所**にしてある。
 */
import { useNumbers } from '../bus/live'
import { DiagCharts } from '../components/DiagCharts'
import { DiagGrid } from '../components/DiagGrid'
import { historyCount } from '../bus/history'

export function DiagView() {
  const n = useNumbers()

  return (
    <div className="diagview">
      <div className="diagview-head">
        <h2>診断</h2>
        {n.stale && <span className="badge-bad">テレメトリ途絶中（表示は最後の値）</span>}
      </div>

      {n.vs ? (
        <DiagGrid n={n} />
      ) : (
        <p className="dim">テレメトリ未受信。Pi につながっていない。</p>
      )}

      {historyCount() > 1 ? (
        <DiagCharts />
      ) : (
        <p className="dim">時系列はこれから貯まる（受信開始から数秒）。</p>
      )}
    </div>
  )
}
