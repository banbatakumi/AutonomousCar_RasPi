/**
 * `/ws/record` — mcap記録の生バイト列を受け取るだけの薄いチャンネル。
 *
 * **Piはここに来たバイトをディスクに書かず、ただ中継しているだけ。**
 * ブラウザ側がチャンクを保持し、停止時に `Blob` にまとめてダウンロードする
 * （`telemetry.ts` と違い、React/store には一切触れない。呼び出し側がチャンクを持つ）。
 *
 * `onClose` は**サーバ側が中継を終えて `/ws/record` を切ったとき**だけ呼ぶ。
 * mcap の索引は最後に書かれるので、「録画停止」を押した瞬間ではなく、
 * 中継そのものが終わるまで待ってからダウンロードを発火させたい
 * （`close()` を自分で呼んだとき＝アンマウント等では呼ばない）。
 */
import type { SocketGuard } from './guard'
import { openGuardedSocket } from './socket'
import { wsUrl } from './url'

export class RecordChannel {
  private ws: WebSocket | null = null
  private closingByUs = false
  private guard: SocketGuard

  constructor(
    private onChunk: (data: ArrayBuffer) => void,
    private onClose?: () => void,
  ) {
    // 開くまでの時間切れ（`guard.ts`）。握手の返らない接続を残すと、Safari では他の WS の張り直しまで止まる。
    // 見切ったら Pi が閉じたのと同じ扱い（呼び出し側は中継の終わりとして片付ける）
    const { ws, guard } = openGuardedSocket(wsUrl('/ws/record'), {
      binaryType: 'arraybuffer',
      onDead: () => {
        if (!this.closingByUs) this.onClose?.()
      },
    })
    this.guard = guard
    ws.onmessage = (ev) => {
      if (ev.data instanceof ArrayBuffer) this.onChunk(ev.data)
    }
    ws.onclose = () => {
      this.guard.stop()
      if (!this.closingByUs) this.onClose?.()
    }
    this.ws = ws
  }

  close() {
    this.closingByUs = true
    this.guard.stop()
    this.ws?.close()
    this.ws = null
  }
}
