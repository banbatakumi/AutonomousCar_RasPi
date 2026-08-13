/**
 * `/ws/telemetry` — 20Hz のバイナリ(msgpack)を受けて `live` に書く。
 *
 * **React には触れない。** 描画は Canvas 側が rAF で `live` を読む。
 */
import { decode } from '@msgpack/msgpack'
import { markHistoryGap, pushHistory } from '../bus/history'
import { clearLive, noteTelemetry } from '../bus/live'
import type { Snapshot } from '../types'
import { wsUrl } from './url'

const RECONNECT_MIN_MS = 300
const RECONNECT_MAX_MS = 3000

export function connectTelemetry(onOpenChange: (open: boolean) => void): () => void {
  let ws: WebSocket | null = null
  let timer: number | undefined
  let backoff = RECONNECT_MIN_MS
  let closed = false

  const open = () => {
    ws = new WebSocket(wsUrl('/ws/telemetry'))
    ws.binaryType = 'arraybuffer'
    ws.onopen = () => {
      backoff = RECONNECT_MIN_MS
      onOpenChange(true)
    }
    ws.onmessage = (ev) => {
      if (!(ev.data instanceof ArrayBuffer)) return
      const s = decode(new Uint8Array(ev.data)) as Snapshot
      noteTelemetry(s.vs, s.link, s.scan, s.auto)
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
