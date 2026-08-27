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
import { zipSync } from 'fflate'
import { useEffect, useRef, useState } from 'react'
import type { ControlStatus, LogFile } from '../types'
import { useUi } from '../store/ui'
import { extractFramesFromMcap } from '../util/mcapFrames'
import type { ControlChannel } from '../ws/control'
import { RecordChannel } from '../ws/record'

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
      <FrameExtractSection />
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
  const [bufferedBytes, setBufferedBytes] = useState(0)
  const recRef = useRef<RecordChannel | null>(null)
  const chunksRef = useRef<ArrayBuffer[]>([])

  const finish = () => {
    recRef.current = null
    if (chunksRef.current.length > 0) {
      const blob = new Blob(chunksRef.current, { type: 'application/octet-stream' })
      const url = URL.createObjectURL(blob)
      const a = document.createElement('a')
      const stamp = new Date().toISOString().replace(/[-:]/g, '').replace('T', '_').slice(0, 15)
      a.href = url
      a.download = `surge_${stamp}.mcap`
      document.body.appendChild(a)
      a.click()
      a.remove()
      URL.revokeObjectURL(url)
    }
    chunksRef.current = []
    setBufferedBytes(0)
    onFinished()
  }

  const start = () => {
    if (!ch) return
    chunksRef.current = []
    setBufferedBytes(0)
    // **サーバ側が `/ws/record` を切断した瞬間が「録画完結」の合図。**
    // mcap の索引は最後に書かれるので、`active` が false になった時点ではなく
    // 中継そのものが終わるまで待ってからダウンロードする
    recRef.current = new RecordChannel(
      (chunk) => {
        chunksRef.current.push(chunk)
        setBufferedBytes((b) => b + chunk.byteLength)
      },
      finish,
    )
    ch.mcapRecordStart(imageOn ? undefined : 0)
  }

  useEffect(
    () => () => {
      recRef.current?.close()
    },
    [],
  )

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

/**
 * フレーム抽出 — `.mcap` からカメラ画像だけを取り出し ZIP でダウンロードする。
 *
 * **すべてブラウザ内で完結する（Pi・サーバは一切関与しない）。** 録画済みの
 * `.mcap`（このタブの mcap セクションで録ったもの・上の一覧からダウンロード
 * したもの、どちらでもよい）をファイル選択で渡すだけで、`raspi/rec/mcap_log.py`
 * が書き込んだ形式（`foxglove.CompressedImage`、zstd圧縮）を解いてJPEGを
 * 取り出す（`../util/mcapFrames.ts`）。Python の `ml/extract_frames.py` と
 * 同じ仕事だが、**Python環境を用意しなくてもその場でできる**のが狙い。
 *
 * 取り出したフレームは `ml/annotate.py`（アノテーション）→ `ml/train.py`（学習）
 * の入力にそのまま使える。
 */
function FrameExtractSection() {
  const [cams, setCams] = useState({ front: true, rear: false })
  const [intervalMs, setIntervalMs] = useState(500)
  const [busy, setBusy] = useState(false)
  const [status, setStatus] = useState('')

  const onFile = async (file: File) => {
    setBusy(true)
    setStatus('読み込み中…')
    try {
      const selected = new Set<string>()
      if (cams.front) selected.add('front')
      if (cams.rear) selected.add('rear')

      const buf = await file.arrayBuffer()
      const minIntervalNs = BigInt(Math.max(0, Math.trunc(intervalMs))) * 1_000_000n
      const frames = extractFramesFromMcap(buf, selected, minIntervalNs)
      if (frames.length === 0) {
        setStatus('選んだカメラのフレームが見つかりませんでした')
        return
      }

      setStatus(`${frames.length}枚を ZIP に圧縮中…`)
      const entries: Record<string, Uint8Array> = {}
      for (const f of frames) entries[f.name] = f.data
      const zipped = zipSync(entries, { level: 6 })

      const blob = new Blob([zipped as BlobPart], { type: 'application/zip' })
      const url = URL.createObjectURL(blob)
      const a = document.createElement('a')
      a.href = url
      a.download = `${file.name.replace(/\.mcap$/i, '')}_frames.zip`
      document.body.appendChild(a)
      a.click()
      a.remove()
      URL.revokeObjectURL(url)
      setStatus(`${frames.length}枚を書き出しました`)
    } catch (e) {
      // zstd 以外の圧縮（古い .mcap 等）はここに落ちる。**Python版を使う旨を出す**
      setStatus(`失敗: ${e instanceof Error ? e.message : String(e)}` +
        '（zstd 圧縮以外の .mcap は ml/extract_frames.py を使ってください）')
    } finally {
      setBusy(false)
    }
  }

  return (
    <section className="settings-group logs-section">
      <h3>フレーム抽出（学習データ作成用）</h3>
      <p className="dim">
        録画した .mcap からカメラ画像だけを取り出し、ZIP でダウンロードします。
        ml/annotate.py・ml/train.py の入力にそのまま使えます。
        間引き間隔を0より大きくすると、連続フレームで似た構図が続く分を
        削れます（0でこれまで通り全件）。
      </p>
      <div className="logs-row">
        <label className="logs-checkbox">
          <input
            type="checkbox"
            checked={cams.front}
            onChange={(e) => setCams((c) => ({ ...c, front: e.target.checked }))}
          />
          前方カメラ
        </label>
        <label className="logs-checkbox">
          <input
            type="checkbox"
            checked={cams.rear}
            onChange={(e) => setCams((c) => ({ ...c, rear: e.target.checked }))}
          />
          後方カメラ
        </label>
        <label className="logs-checkbox">
          間引き間隔(ms):
          <input
            type="number"
            min={0}
            step={100}
            value={intervalMs}
            disabled={busy}
            onChange={(e) => setIntervalMs(Number(e.target.value))}
            style={{ width: '5em', marginLeft: '0.4em' }}
          />
        </label>
      </div>
      <div className="logs-row">
        <input
          type="file"
          accept=".mcap"
          disabled={busy || (!cams.front && !cams.rear)}
          onChange={(e) => {
            const f = e.target.files?.[0]
            e.target.value = ''
            if (f) void onFile(f)
          }}
        />
      </div>
      {status && <p className="dim">{status}</p>}
    </section>
  )
}
