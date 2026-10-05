/**
 * 記録 — `logs/` にある `.sfl`/`.mcap` の一覧・ダウンロード・削除。
 *
 * 録画の開始・停止はタブバーの `LogControls` にある（どのタブからでも押せるように）。
 * 一覧はこのページを開いたときと、録画が終わるたびに取り直す。
 */
import { useEffect } from 'react'
import { formatBytes, formatDateTime } from '../../../format'
import { useUi } from '../../../store/ui'
import type { ControlChannel } from '../../../ws/control'

export function LogsPage({ ch }: { ch: ControlChannel | null }) {
  const files = useUi((s) => s.logFiles)
  useEffect(() => {
    ch?.logsList()
  }, [ch])
  return (
    <section className="dg-card wide">
      <div className="logs-section-head">
        <h4>記録ファイル</h4>
        <button disabled={!ch} onClick={() => ch?.logsList()}>
          更新
        </button>
      </div>
      {files.length === 0 ? (
        <p className="dim">記録がありません</p>
      ) : (
        <div className="logs-table-wrap">
          <table className="logs-table">
            <thead>
              <tr>
                <th>名前</th>
                <th>種別</th>
                <th>サイズ</th>
                <th>日時</th>
                <th />
              </tr>
            </thead>
            <tbody>
              {files.map((f) => (
                <tr key={f.name}>
                  <td>{f.name}</td>
                  <td>
                    <span className="pill">{f.kind}</span>
                  </td>
                  <td>{formatBytes(f.size)}</td>
                  <td>{formatDateTime(f.mtime)}</td>
                  <td className="logs-actions">
                    <a href={`/logs/${encodeURIComponent(f.name)}`} download>
                      ダウンロード
                    </a>
                    <button
                      disabled={!ch}
                      onClick={() => {
                        if (window.confirm(`${f.name} を削除しますか？`)) ch?.logsDelete(f.name)
                      }}
                    >
                      削除
                    </button>
                  </td>
                </tr>
              ))}
            </tbody>
          </table>
        </div>
      )}
    </section>
  )
}
