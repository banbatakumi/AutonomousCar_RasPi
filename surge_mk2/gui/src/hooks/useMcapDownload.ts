/**
 * mcap記録の開始〜ブラウザへのダウンロードまでをまとめたフック。
 *
 * 元は `LogView.tsx` の `McapSection` に直書きだったが、システム同定タブ
 * （`SysIdView.tsx`）も「試験開始と同時に記録を始め、試験終了で自動的に
 * ダウンロードする」という同じ仕組みを要るため切り出した。
 *
 * **記録中かどうか（`active`/`elapsed_s`）はサーバ側の状態（`ControlStatus.mcap`）
 * が真値。** このフックが持つのはブラウザ側のバッファとダウンロード処理だけ。
 *
 * **サーバ側が `/ws/record` を切断した瞬間が「録画完結」の合図。** mcap の索引は
 * 最後に書かれるので、`active` が false になった時点ではなく中継そのものが
 * 終わるまで待ってからダウンロードする（`RecordChannel` の docstring参照）。
 */
import { useEffect, useRef, useState } from 'react'
import type { ControlChannel } from '../ws/control'
import { RecordChannel } from '../ws/record'

export function useMcapDownload(ch: ControlChannel | null, filenamePrefix = 'surge') {
  const [bufferedBytes, setBufferedBytes] = useState(0)
  const recRef = useRef<RecordChannel | null>(null)
  const chunksRef = useRef<ArrayBuffer[]>([])
  const onFinishedRef = useRef<(() => void) | undefined>(undefined)

  const finish = () => {
    recRef.current = null
    if (chunksRef.current.length > 0) {
      const blob = new Blob(chunksRef.current, { type: 'application/octet-stream' })
      const url = URL.createObjectURL(blob)
      const a = document.createElement('a')
      const stamp = new Date().toISOString().replace(/[-:]/g, '').replace('T', '_').slice(0, 15)
      a.href = url
      a.download = `${filenamePrefix}_${stamp}.mcap`
      document.body.appendChild(a)
      a.click()
      a.remove()
      URL.revokeObjectURL(url)
    }
    chunksRef.current = []
    setBufferedBytes(0)
    onFinishedRef.current?.()
  }

  const start = (imageOn: boolean, onFinished?: () => void) => {
    if (!ch) return
    onFinishedRef.current = onFinished
    chunksRef.current = []
    setBufferedBytes(0)
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

  return { bufferedBytes, start }
}
