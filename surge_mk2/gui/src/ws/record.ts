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
import { wsUrl } from './url'

export class RecordChannel {
  private ws: WebSocket | null = null
  private closingByUs = false

  constructor(
    private onChunk: (data: ArrayBuffer) => void,
    private onClose?: () => void,
  ) {
    const ws = new WebSocket(wsUrl('/ws/record'))
    ws.binaryType = 'arraybuffer'
    ws.onmessage = (ev) => {
      if (ev.data instanceof ArrayBuffer) this.onChunk(ev.data)
    }
    ws.onclose = () => {
      if (!this.closingByUs) this.onClose?.()
    }
    this.ws = ws
  }

  close() {
    this.closingByUs = true
    this.ws?.close()
    this.ws = null
  }
}
