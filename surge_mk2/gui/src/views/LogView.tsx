/**
 * ログビュー — `.sfl`/mcap の記録と、一覧・ダウンロード・削除（`architecture.md` §11）。
 *
 * ## 2つの記録は仕組みがまったく違う
 *
 * - **`.sfl`**: io_node が実時間ループの中で直接 Pi の SD に書く。ここでは
 *   「録ってほしい」という意思を送るだけ（`log/ctrl`）。Wi-Fi が切れても
 *   Pi 単体で記録は止まらない。ファイルは下の一覧からダウンロード・削除する
 * - **mcap**: `logger_node -o -` の標準出力を `/ws/record` でそのまま中継するだけ。
 *   **Piのディスクには一度も書かない。** ブラウザがチャンクを溜め、停止して
 *   サーバ側の中継が終わった合図（`/ws/record` の切断）でPCへダウンロードする
 *
 * ## 再生（`replay_node --bus`）はここにはない
 *
 * 実車の `surge-io` と同じバスのエンドポイントを取り合うため、Pi 上でGUI経由に
 * すると「使うには先にSSHで surge-io を止める」手順が結局要り、GUIに置く意味が
 * 薄い。**実車なしで知覚/SLAM/経路生成を開発したいだけなら**、Mac側でローカルに
 *
 *     .venv/bin/python -m raspi.nodes.replay_node logs/xxx.sfl --bus
 *     .venv/bin/python -m raspi.nodes.telemetry_node
 *
 * を直接叩けば足りる（そちらは別のバスで動くので実車と衝突しない）。
 */
import { useEffect, useState } from 'react'
import { useMcapDownload } from '../hooks/useMcapDownload'
import type { ControlStatus, LogFile } from '../types'
import { useUi } from '../store/ui'
import type { ControlChannel } from '../ws/control'

function formatBytes(n: number): string {
  return n >= 1e6 ? `${(n / 1e6).toFixed(1)}MB` : `${(n / 1e3).toFixed(0)}KB`
}

function formatElapsed(s: number): string {
  const m = Math.floor(s / 60)
  const sec = Math.floor(s % 60)
  return `${m}:${sec.toString().padStart(2, '0')}`
}

function formatDate(unixSec: number): string {
  return new Date(unixSec * 1000).toLocaleString()
}

export function LogView({ ch }: { ch: ControlChannel | null }) {
  const sfl = useUi((s) => s.sfl)
  const mcap = useUi((s) => s.mcap)
  const logFiles = useUi((s) => s.logFiles)

  // タブを開いたら一覧を取得。以降は録画が終わるたびに App.tsx 側では
  // なくここで直接叩き直す（新しいファイルが増えている可能性があるため）
  useEffect(() => {
    ch?.logsList()
  }, [ch])

  return (
    <div className="settings logs-view">
      <div className="settings-head">
        <h2>ログ</h2>
      </div>

      <SflSection ch={ch} sfl={sfl} />
      <McapSection ch={ch} mcap={mcap} onFinished={() => ch?.logsList()} />
      <FilesSection ch={ch} files={logFiles} />
    </div>
  )
}

function SflSection({ ch, sfl }: { ch: ControlChannel | null; sfl: ControlStatus['sfl'] | null }) {
  const active = sfl?.active ?? false
  return (
    <section className="settings-group logs-section">
      <h3>.sfl 記録（UART生ログ）</h3>
      <div className="logs-row">
        <span className={`pill ${active ? 'lv-warn' : 'dim'}`}>{active ? '記録中' : '停止中'}</span>
        <button className={active ? 'on' : ''} disabled={!ch} onClick={() => ch?.sflRecord(!active)}>
          {active ? '停止' : '開始'}
        </button>
      </div>
    </section>
  )
}

function McapSection({
  ch,
  mcap,
  onFinished,
}: {
  ch: ControlChannel | null
  mcap: ControlStatus['mcap'] | null
  onFinished: () => void
}) {
  const active = mcap?.active ?? false
  const [imageOn, setImageOn] = useState(true)
  const { bufferedBytes, start: startRecording } = useMcapDownload(ch)
  const start = () => startRecording(imageOn, onFinished)

  return (
    <section className="settings-group logs-section">
      <h3>mcap 記録（画像込み・Foxglove用）</h3>
      <div className="logs-row">
        <span className={`pill ${active ? 'lv-warn' : 'dim'}`}>
          {active ? `録画中 ${formatElapsed(mcap!.elapsed_s)} / ${formatBytes(bufferedBytes)}` : '停止中'}
        </span>
        <button
          className={active ? 'on' : ''}
          disabled={!ch}
          onClick={() => (active ? ch?.mcapRecordStop() : start())}
        >
          {active ? '停止' : '開始'}
        </button>
      </div>
      {!active && (
        <label className="logs-checkbox">
          <input type="checkbox" checked={imageOn} onChange={(e) => setImageOn(e.target.checked)} />
          画像を含める
        </label>
      )}
      {mcap?.error && <span className="badge-bad">{mcap.error}</span>}
    </section>
  )
}

function FilesSection({ ch, files }: { ch: ControlChannel | null; files: LogFile[] }) {
  return (
    <section className="settings-group logs-section">
      <div className="logs-section-head">
        <h3>ファイル一覧</h3>
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
                  <td>{formatDate(f.mtime)}</td>
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
