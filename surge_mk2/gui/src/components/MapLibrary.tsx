/**
 * 保存済み地図の管理（`slam2d_raceline`選択時、`AutoPanel.tsx`から出す）。
 *
 * ## Mac⇔Pi間の受け渡しはGUIのボタン操作だけで完結させる
 *
 * このリポジトリの方針で、`ssh`/`rsync`等の手動スクリプトは使わない。
 * ダウンロードはブラウザ標準の`<a download>`（`GET /maps/<name>`、
 * `raspi/nodes/telemetry_node.py`の`_serve_map_file`）、アップロードは
 * 専用のWebSocketチャンネル（`ws/mapUpload.ts`）——このサーバのHTTP実装は
 * POSTボディを受け取れないため（`ws/mapUpload.ts`のモジュールdocstring参照）。
 *
 * **DL・UL は経路の設定（経由点・停止点・ミッション・地図作成の軌跡、`.routes.json`）も
 * npz に同梱して運ぶ**（`raspi/auto/mapstore.py` の `export_map`/`import_upload`、2026-09-29）。
 * 同梱の無い古い npz を上げると、同じ名前の古い経路の設定は消える。
 *
 * 想定運用: Piで地図を作成→保存→ダウンロードしてMac上で検証→良ければ
 * （同じ名前のまま、または別名で）アップロードしてPiに配置→GUIの一覧から選ぶ。
 *
 * ## 「保存」ボタンは別名で取り直すためのもの
 *
 * 地図と経路ができた瞬間（`DONE`段）に、日時から生成した名前で**自動保存**
 * される（`raspi/auto/slam2d_raceline.py`の`_auto_save_map()`）。この
 * 「保存」ボタンは、分かりやすい名前を付け直したい・走行中に別名で
 * スナップショットを取りたい、という場合のための手動操作（`DONE`/`RACE`
 * 段のどちらでも押せる）。
 *
 * ## 一覧はドロップダウン（2026-09-29）
 *
 * 地図が増えると縦の並びが場所を取るので、1つ選んで「DL」「削除」する形にした
 * （バンビの指示）。**削除は走行する側（telemetry_node が動いている機械）の
 * `saved_maps/` から `.npz`・`.json`・`.routes.json` を消す**（`mapstore.delete_map`）。
 * 実機なら Pi、`sim.run` なら Mac。ダウンロード済みの写しは消えない。ごみ箱は無い。
 */
import { useEffect, useRef, useState } from 'react'
import { useNumbers } from '../bus/live'
import { useUi } from '../store/ui'
import type { ControlChannel } from '../ws/control'
import { uploadMap } from '../ws/mapUpload'

export function MapLibrary({ ch }: { ch: ControlChannel | null }) {
  const mapFiles = useUi((s) => s.mapFiles)
  const saveResult = useUi((s) => s.mapSaveResult)
  const setUi = useUi((s) => s.set)
  const n = useNumbers()
  const phase = n.auto?.phase

  const [name, setName] = useState('')
  const [picked, setPicked] = useState('')
  // 選んでいた地図が一覧から消えたら（削除した等）選び直す
  const current = mapFiles.find((f) => f.name === picked) ?? mapFiles[0]
  const [uploading, setUploading] = useState(false)
  const fileRef = useRef<HTMLInputElement>(null)

  // 保存結果の表示は一過性。次に何か操作するまで出しておいて、
  // 新しい保存/アップロードを始めたら消す
  useEffect(() => {
    if (!saveResult) return
    const t = window.setTimeout(() => setUi({ mapSaveResult: null }), 6000)
    return () => window.clearTimeout(t)
  }, [saveResult, setUi])

  const save = () => {
    const n = name.trim()
    if (!n || !ch) return
    setUi({ mapSaveResult: null })
    ch.mapsSave(n)
  }

  const pickUpload = () => fileRef.current?.click()

  const onFileChosen = (file: File | undefined) => {
    if (!file || !ch) return
    const uploadName = name.trim() || file.name.replace(/\.npz$/i, '')
    setUi({ mapSaveResult: null })
    setUploading(true)
    uploadMap(uploadName, file, (ok, error) => {
      setUploading(false)
      setUi({ mapSaveResult: { ok, error: error ?? '' } })
      if (ok) ch.mapsList()
    })
  }

  return (
    <section className="map-library">
      <span className="label">保存済み地図</span>
      <div className="map-library-row">
        <input
          type="text"
          placeholder="地図の名前"
          value={name}
          onChange={(e) => setName(e.target.value)}
        />
        <button
          disabled={!ch || !name.trim() || (phase !== 'DONE' && phase !== 'RACE')}
          onClick={save}
          title={
            phase !== 'DONE' && phase !== 'RACE'
              ? '地図と経路ができてから（DONE/RACE段）保存できます'
              : undefined
          }
        >
          保存
        </button>
        <button disabled={!ch || uploading} onClick={pickUpload}>
          {uploading ? 'アップロード中…' : 'アップロード'}
        </button>
        <input
          ref={fileRef}
          type="file"
          accept=".npz"
          hidden
          onChange={(e) => {
            onFileChosen(e.target.files?.[0])
            e.target.value = ''
          }}
        />
      </div>
      {saveResult && (
        <p className={saveResult.ok ? 'dim' : 'badge-bad'}>
          {saveResult.ok ? '保存しました' : saveResult.error || '失敗しました'}
        </p>
      )}
      {mapFiles.length === 0 ? (
        <p className="dim map-library-empty">保存済みの地図はありません</p>
      ) : (
        <div className="map-library-row map-library-pick">
          <select value={current?.name ?? ''} onChange={(e) => setPicked(e.target.value)}>
            {mapFiles.map((f) => (
              <option key={f.name} value={f.name}>
                {f.name}（{new Date(f.created_at * 1000).toLocaleDateString()}）
              </option>
            ))}
          </select>
          {current && (
            <>
              <a
                href={`/maps/${encodeURIComponent(current.name)}`}
                download={`${current.name}.npz`}
              >
                DL
              </a>
              <button
                title="走行する側の saved_maps/ から消す（元に戻せない）"
                onClick={() => {
                  const msg =
                    phase === 'RACE'
                      ? `走行中です。本当に「${current.name}」を削除しますか？`
                      : `「${current.name}」を削除しますか？（元に戻せません）`
                  if (window.confirm(msg)) ch?.mapsDelete(current.name)
                }}
              >
                削除
              </button>
            </>
          )}
        </div>
      )}
    </section>
  )
}
