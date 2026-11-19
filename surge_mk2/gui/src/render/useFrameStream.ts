/**
 * カメラ系の映像（JPEG）を WebSocket で受けて、最新の1枚を `ImageBitmap` で持つフック
 * （`CameraView.tsx` の effect から分けた。技術的負債の棚卸し 2026-11-18 高6）。
 *
 * 描画はしない。返す ref（`FrameStream`）を描画側の rAF が毎フレーム読む——**React の state に
 * 画像を入れない**（1フレームごとに再レンダリングが走る）。
 *
 * - 接続: `reconnectingSocket`（800ms 固定で張り直す。`ws/socket.ts`）
 * - デコード: `LatestFrameDecoder`（最新の1枚だけ）
 * - `watchdog`: デコードの固まりと、**繋がっているのに映像が来ない半開き**からの復帰（本体の映像だけ）
 *
 * `path` が null の間は繋がない（マスクは表示する間だけ繋ぐ）。
 */
import { useEffect, useRef } from 'react'
import { CAM_RECONNECT_WAIT_MS, reconnectingSocket } from '../ws/socket'
import { wsUrl } from '../ws/url'
import { LatestFrameDecoder } from './latestFrameDecoder'

/** 繋がっているのにフレームが来ない状態がこれ続いたら、接続を張り直す [ms]（DISARM は 1fps） */
const CAM_RECONNECT_MS = 5000
/** デコードがこれ終わらなければ固まりとみなして見捨てる [ms] */
const CAM_DECODE_STUCK_MS = 2000

export type FrameStream = {
  /** 最新の1枚（無ければ null）。このフックが解放まで持つ */
  bitmap: ImageBitmap | null
  wsOpen: boolean
  /** 最後に絵が更新された時刻（0=まだ）と、フレーム間隔の移動平均 [ms]（古さの判定に使う） */
  lastFrameMs: number
  frameIntervalMs: number
  /** 届いた枚数（fps の計測用。読んだ側が 0 に戻してよい） */
  frames: number
}

function emptyStream(): FrameStream {
  return { bitmap: null, wsOpen: false, lastFrameMs: 0, frameIntervalMs: 0, frames: 0 }
}

export function useFrameStream(
  path: string | null,
  opts: {
    /** デコード中に届いた最新の1枚を残すか（false は捨てる。マスクの従来の挙動） */
    keepPending?: boolean
    /** 固まり・半開きの監視をするか（本体の映像） */
    watchdog?: boolean
    /** 1枚デコードできるたびに呼ぶ（縦横比を親へ返す等）。ref 経由で最新を呼ぶ */
    onFrame?: (bmp: ImageBitmap) => void
  } = {},
) {
  const { keepPending = true, watchdog = false } = opts
  const stream = useRef<FrameStream>(emptyStream())
  const onFrameRef = useRef(opts.onFrame)
  useEffect(() => {
    onFrameRef.current = opts.onFrame
  })

  useEffect(() => {
    if (!path) return
    // 接続ごとに新しい状態を作る。閉じた古い接続の `onclose` が遅れて来ても、新しい接続の
    // `wsOpen` を倒さない（共有の1つを書き換えると、張り直した映像が「切断中」のまま残りうる）
    const st = emptyStream()
    stream.current = st
    /** 接続した時刻。固まりの検出に使う */
    let openedMs = 0
    const decoder = new LatestFrameDecoder((bmp) => {
      st.bitmap?.close()
      st.bitmap = bmp
      st.frames++
      const now = performance.now()
      if (st.lastFrameMs) {
        const dt = now - st.lastFrameMs
        st.frameIntervalMs = st.frameIntervalMs ? st.frameIntervalMs * 0.8 + dt * 0.2 : dt
      }
      st.lastFrameMs = now
      onFrameRef.current?.(bmp)
    }, keepPending)

    // **開くまで**の時間切れは番犬が見る（`ws/guard.ts`）。Safari は同じ相手への接続中を1本ずつしか
    // 進めないので、握手の返らない1本が残ると telemetry・control の張り直しまで後ろで止まる（2026-11-01 再現）。
    // 開いた後の半開きは下の watchdog が見る
    const sock = reconnectingSocket(wsUrl(path), {
      binaryType: 'arraybuffer',
      backoff: CAM_RECONNECT_WAIT_MS,
      onOpen: () => {
        st.wsOpen = true
        openedMs = performance.now()
      },
      onLost: () => {
        st.wsOpen = false
      },
      onMessage: (ev) => {
        if (ev.data instanceof ArrayBuffer) decoder.push(ev.data)
      },
    })

    /** 固まりからの復帰。デコードが終わらない、または繋がっているのに映像が来ない状態を直す */
    const timer = watchdog
      ? window.setInterval(() => {
          const now = performance.now()
          decoder.checkStuck(now, CAM_DECODE_STUCK_MS)
          // 半開きの接続（`onclose` が来ない）は、フレームが途絶えたら自分で切って張り直す
          const lastAlive = Math.max(st.lastFrameMs, openedMs)
          if (st.wsOpen && now - lastAlive > CAM_RECONNECT_MS) sock.drop(true)
        }, 1000)
      : undefined

    return () => {
      window.clearInterval(timer)
      decoder.close()
      sock.close()
      st.bitmap?.close()
      st.bitmap = null
      stream.current = emptyStream()
    }
  }, [path, keepPending, watchdog])

  return stream
}
