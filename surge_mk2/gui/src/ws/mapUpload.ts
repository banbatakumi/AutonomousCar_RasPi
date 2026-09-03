/**
 * `/ws/map_upload/<name>` — Mac→Piの地図アップロード専用チャンネル。
 *
 * このリポジトリのHTTPサーバ（`raspi/nodes/telemetry_node.py`）は
 * `websockets`ライブラリ内蔵の最小実装で、**POSTボディを受け取れない**
 * （`Content-Length != 0`のリクエストを拒否する）。地図のダウンロードは
 * `GET /maps/<name>`（`<a download>`）で足りるが、アップロードは逆方向の
 * バイナリ中継が要るので、`/ws/record`（Pi→ブラウザ）と対称の新規チャンネル
 * をこちらに置く。
 *
 * プロトコル: バイナリフレームをチャンク分割して送り、最後にテキストフレーム
 * （終了合図、内容は見られない）を送る。サーバは検証・書き込みしてから
 * `{"ok": bool, "error"?: string}` を1通返して自分で閉じる。
 */
import { wsUrl } from './url'

//: 1フレームあたりの送信サイズ。地図は数百KB〜数MBなので、大きすぎる
//: フレームでブラウザ側のバッファリングを避けるためだけの分割（意味のある
//: チャンク境界ではない——サーバ側は届いた順にただ連結するだけ）
const CHUNK_BYTES = 256 * 1024

export function uploadMap(
  name: string,
  file: File,
  onDone: (ok: boolean, error?: string) => void,
): void {
  const ws = new WebSocket(wsUrl(`/ws/map_upload/${encodeURIComponent(name)}`))
  ws.binaryType = 'arraybuffer'

  ws.onopen = async () => {
    try {
      const buf = await file.arrayBuffer()
      for (let i = 0; i < buf.byteLength; i += CHUNK_BYTES) {
        ws.send(buf.slice(i, i + CHUNK_BYTES))
      }
      // 空ファイル（0バイト）でも最低1フレームは送っておく。サーバ側は
      // 「バイナリを1つも受け取らないまま終了合図」でも壊れないよう検証で弾くが、
      // 意図が伝わりやすいのでここで揃えておく
      ws.send(JSON.stringify({ type: 'done' }))
    } catch (e) {
      onDone(false, e instanceof Error ? e.message : String(e))
      ws.close()
    }
  }
  ws.onmessage = (ev) => {
    try {
      const r = JSON.parse(ev.data as string) as { ok: boolean; error?: string }
      onDone(r.ok, r.error)
    } catch {
      onDone(false, 'サーバの応答を解釈できませんでした')
    }
    ws.close()
  }
  ws.onerror = () => onDone(false, '接続エラー')
}
