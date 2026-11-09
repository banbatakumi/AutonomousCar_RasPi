/**
 * 死んだ WebSocket を **`onclose` を待たずに** 見切る番犬。
 *
 * ## なぜ要るか（Safari・Wi-Fi）
 *
 * Pi を再起動したあと GUI が INIT・通信途絶のまま戻らず、再読み込みで直った。Safari のログでは
 * - 再起動**前**に張った `:8000` の接続を、Pi 側にはもう無いのに **20分「ready」のまま**持っていた
 *   （telemetry はブラウザから何も送らないので、相手が消えたことに TCP が気づかない）
 * - 再起動中に始めた接続が **310秒「接続中」のまま**止まっていた
 *
 * どちらも `onclose` が来ないので、`onclose` で繋ぎ直す作りでは永久に戻らない。
 * カメラ（`render/CameraView.tsx`）は前から同じ考えで自分で張り直している。
 *
 * - **開くまで** `WS_OPEN_TIMEOUT_MS`。LAN の握手は数十 ms なので、これを超えたら見切る
 * - **開いた後**は `idleMs` を渡したチャンネルだけ、その間なにも届かなければ見切る
 *   （常に流れるもの: telemetry・control の pong。変わったときだけ届く地図には付けない）
 *
 * 見切るときはハンドラを外してから `close()` し、`onDead` を**すぐ**呼ぶ。半開きの接続の
 * `close()` は閉じる合図の返事を待つので、`onclose` はいつ来るか分からない。
 * 時刻は「最後に届いた時刻」との差で判定する（バックグラウンドのタブでタイマーが遅れても誤判定しない）。
 */

/** 開くまでの時間切れ [ms] */
export const WS_OPEN_TIMEOUT_MS = 4000
const CHECK_MS = 500

export interface SocketGuard {
  /** 何か届いたら呼ぶ（`idleMs` の起点を更新する） */
  alive(): void
  /** 普通に閉じた・自分で閉じたとき。番犬を止める */
  stop(): void
}

export function guardSocket(ws: WebSocket, onDead: () => void, idleMs?: number): SocketGuard {
  let last = performance.now()
  let opened = false
  let timer: number | undefined = window.setInterval(() => {
    const limit = opened ? idleMs : WS_OPEN_TIMEOUT_MS
    if (limit == null || performance.now() - last <= limit) return
    stop()
    ws.onopen = ws.onmessage = ws.onclose = ws.onerror = null
    try {
      ws.close()
    } catch {
      /* 既に閉じている */
    }
    onDead()
  }, CHECK_MS)
  const stop = () => {
    window.clearInterval(timer)
    timer = undefined
  }
  ws.addEventListener('open', () => {
    opened = true
    last = performance.now()
  })
  return {
    alive: () => {
      last = performance.now()
    },
    stop,
  }
}

const peerLostListeners = new Set<() => void>()

/**
 * 「Pi が消えた」の通知。**常に流れるチャンネル（telemetry）が見切ったとき**に鳴らし、
 * 変わったときだけ届くチャンネル（地図）はこれを合図に張り直す——自分では半開きに気づけないため。
 */
export function notifyPeerLost(): void {
  for (const f of peerLostListeners) f()
}

export function onPeerLost(f: () => void): () => void {
  peerLostListeners.add(f)
  return () => peerLostListeners.delete(f)
}
