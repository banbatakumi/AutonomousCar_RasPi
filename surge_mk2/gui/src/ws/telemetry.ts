/**
 * `/ws/telemetry` — 20Hz のバイナリ(msgpack)を受けて `live` に書く。
 *
 * **React には触れない。** 描画は Canvas 側が rAF で `live` を読む。
 */
import { decode } from '@msgpack/msgpack'
import { markHistoryGap, pushHistory } from '../bus/history'
import { clearLive, noteTelemetry } from '../bus/live'
import { MSGS_SCHEMA } from '../generated/msgs'
import type { Snapshot } from '../types'
import { wsUrl } from './url'

const RECONNECT_MIN_MS = 300
const RECONNECT_MAX_MS = 3000

export function connectTelemetry(
  onOpenChange: (open: boolean) => void,
  /** 型定義の食い違いを検出したときに1回だけ呼ぶ。**画面に出すこと** */
  onSchemaMismatch?: (got: number) => void,
): () => void {
  let ws: WebSocket | null = null
  let timer: number | undefined
  let backoff = RECONNECT_MIN_MS
  let closed = false
  // 型が食い違っていることを既に伝えたか。**再接続しても直らない**ので、
  // 繋ぎ直すたびに鳴らしても意味が無い（GUI を再ビルドするまで続く）
  let mismatched = false

  const open = () => {
    ws = new WebSocket(wsUrl('/ws/telemetry'))
    ws.binaryType = 'arraybuffer'
    ws.onopen = () => {
      backoff = RECONNECT_MIN_MS
      onOpenChange(true)
    }
    ws.onmessage = (ev) => {
      if (!(ev.data instanceof ArrayBuffer)) return
      // **`as Snapshot` は TypeScript に嘘をつく操作で、実行時には何も起きない。**
      // 全フィールドを実行時に舐める必要は無いが、**札を1つ見るだけ**で
      // 「Pi 側だけ型が変わった画面を信じて走る」は防げる（レビュー 🟠3）
      const raw = decode(new Uint8Array(ev.data)) as { schema?: number }
      if (raw.schema !== MSGS_SCHEMA) {
        if (!mismatched) {
          mismatched = true          // 20Hz で流れるので通知は1回だけ
          onSchemaMismatch?.(raw.schema ?? 0)
        }
        return                       // **捨てる。** 古い型のまま描くほうが危ない
      }
      const s = raw as unknown as Snapshot
      noteTelemetry(s.vs, s.link, s.scan, s.auto, s.pi_temp_c, s.line_cam, s.track)
      // 診断タブ用の時系列。**タブを開いていなくても常時貯める**（`bus/history.ts`）
      pushHistory(s.vs, s.link)
    }
    ws.onclose = () => {
      onOpenChange(false)
      clearLive()
      // 履歴は残したまま線だけ断つ。**切れたこと自体が後から見たい事象**
      markHistoryGap()
      if (closed) return
      timer = window.setTimeout(open, backoff)
      backoff = Math.min(backoff * 2, RECONNECT_MAX_MS)
    }
    // onerror の直後に onclose が来るので、再接続はそちらに任せる
    ws.onerror = () => ws?.close()
  }
  open()

  return () => {
    closed = true
    window.clearTimeout(timer)
    ws?.close()
  }
}
