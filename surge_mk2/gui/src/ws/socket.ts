/**
 * WebSocket を開いて番犬（`guard.ts`）を付ける共通部品と、その一段上の「切れたら張り直す」接続。
 *
 * telemetry・地図・control・カメラ（本体・マスク）・深度で「open → `guardSocket` → onclose で
 * setTimeout 再接続 → onerror で close」を手書きしていたのをまとめた（技術的負債の棚卸し 2026-11-18 高5）。
 * 張り直すまでの待ち時間の流儀はチャンネルごとに違ったので、**引数で選べる**ようにして挙動は変えていない:
 *
 * | チャンネル | 待ち時間 | 開いた後の見切り（`idleMs`） |
 * |---|---|---|
 * | telemetry・地図 | 指数バックオフ 300ms→3s（開いたら戻す） | telemetry は 2s、地図は無し |
 * | control | 500ms 固定 | 5s（pong） |
 * | カメラ・マスク・深度 | 800ms 固定 | 無し（カメラは自前の watchdog） |
 *
 * 張り直さない1回きりの接続（`/ws/record`・`/ws/map_upload`）は `openGuardedSocket` だけを使う。
 */
import { guardSocket, type SocketGuard } from './guard'

/** WebSocket を開き、番犬を付けて返す。`onDead` は番犬が見切ったとき（ハンドラは外し済み）に呼ばれる */
export function openGuardedSocket(
  url: string,
  opts: { binaryType?: BinaryType; onDead: () => void; idleMs?: number },
): { ws: WebSocket; guard: SocketGuard } {
  const ws = new WebSocket(url)
  if (opts.binaryType) ws.binaryType = opts.binaryType
  const guard = guardSocket(ws, opts.onDead, opts.idleMs)
  return { ws, guard }
}

/** カメラ系（本体・マスク・深度）の張り直しまでの待ち時間 [ms] */
export const CAM_RECONNECT_WAIT_MS = 800

/** 張り直すまでの待ち時間 [ms]。数なら固定、`{min, max}` なら指数バックオフ（開いたら `min` に戻す） */
type Backoff = number | { minMs: number; maxMs: number }

export type ReconnectingSocketOptions = {
  binaryType?: BinaryType
  /** 届いたメッセージ。番犬への「生きている」の通知はこちらで済ませてから呼ぶ */
  onMessage: (ev: MessageEvent) => void
  onOpen?: () => void
  /**
   * 1本の接続が終わった（閉じた・エラー・番犬が見切った・`drop()`）。**1本につき1回だけ**呼ぶ。
   * `close()` で自分で閉じたときも、閉じ終わった時点で呼ぶ（手書きの頃と同じ。張り直しはしない）
   */
  onLost?: () => void
  /** 番犬が見切ったときだけ、`onLost` の前に呼ぶ（telemetry が地図へ「Pi が消えた」を知らせる）。`close()` 後は呼ばない */
  onDead?: () => void
  /** 開いた後、これだけ何も届かなければ死んだとみなす [ms]（`guard.ts`）。無しなら開くまでだけ見る */
  idleMs?: number
  backoff: Backoff
}

export interface ReconnectingSocket {
  /** 今の接続（無ければ null）。送信・`readyState` の確認に使う */
  readonly ws: WebSocket | null
  /**
   * 今の接続を死んだとみなして捨て、張り直す（自分では半開きに気づけないチャンネル用）。
   * `immediate` なら待たずにすぐ開く（カメラの watchdog）。それ以外は通常の待ち時間。
   * 既に切れて張り直しを待っている間は何もしない
   */
  drop(immediate?: boolean): void
  /** 止める。以後は張り直さない */
  close(): void
}

export function reconnectingSocket(url: string, o: ReconnectingSocketOptions): ReconnectingSocket {
  const minMs = typeof o.backoff === 'number' ? o.backoff : o.backoff.minMs
  let wait = minMs
  let ws: WebSocket | null = null
  let guard: SocketGuard | null = null
  let timer: number | undefined
  let closed = false
  /** 今の接続を捨てる（ハンドラを外して閉じ、`onLost`）。既に終わっていれば false。`drop()` から呼ぶ */
  let dropCurrent: (() => boolean) | null = null

  const schedule = () => {
    if (closed) return
    timer = window.setTimeout(open, wait)
    if (typeof o.backoff !== 'number') wait = Math.min(wait * 2, o.backoff.maxMs)
  }

  const open = () => {
    timer = undefined
    // 1本の接続につき1回だけ（見切って捨てた後に、遅れた onclose が来ても二重に張らない）
    let done = false
    const lost = (reconnect: boolean) => {
      if (done) return
      done = true
      g.stop()
      o.onLost?.()
      if (reconnect) schedule()
    }
    const { ws: sock, guard: g } = openGuardedSocket(url, {
      binaryType: o.binaryType,
      idleMs: o.idleMs,
      onDead: () => {
        if (closed) return // 片付け済み（アンマウント後の半開き）。誰にも知らせない
        o.onDead?.()
        lost(true)
      },
    })
    ws = sock
    guard = g
    sock.onopen = () => {
      wait = minMs
      o.onOpen?.()
    }
    sock.onmessage = (ev) => {
      g.alive()
      o.onMessage(ev)
    }
    sock.onclose = () => lost(!closed)
    // onerror の直後に onclose が来るので、張り直しはそちらに任せる
    sock.onerror = () => sock.close()
    dropCurrent = () => {
      if (done) return false
      sock.onopen = sock.onmessage = sock.onclose = sock.onerror = null
      sock.close()
      lost(false)
      return true
    }
  }

  open()

  return {
    get ws() {
      return ws
    },
    drop(immediate = false) {
      // 既に切れて張り直しを待っている間は何もしない（待ち時間を延ばさない・二重に張らない）
      if (closed || !dropCurrent?.()) return
      if (immediate) open()
      else schedule()
    },
    close() {
      closed = true
      window.clearTimeout(timer)
      guard?.stop()
      ws?.close()
    },
  }
}
