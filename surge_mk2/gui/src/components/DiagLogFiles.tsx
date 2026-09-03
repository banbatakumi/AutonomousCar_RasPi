/**
 * 記録ファイル一覧 — 診断タブに常設（旧「ログ」タブの `FilesSection` を移設、2026-09-03）。
 *
 * 録画の開始・停止はタブバーの `LogControls` に移った。ここは `logs/` にある
 * `.sfl`/`.mcap` の一覧・ダウンロード・削除だけを扱う。
 */
import { formatBytes, formatDateTime } from '../format'
import type { LogFile } from '../types'
import type { ControlChannel } from '../ws/control'

export function DiagLogFiles({ ch, files }: { ch: ControlChannel | null; files: LogFile[] }) {
  return (
    <section className="settings-group">
      <div className="logs-section-head">
        <h3>記録ファイル</h3>
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
