/**
 * `/ws/telemetry` — 50Hz のバイナリ(msgpack)を受けて `live` に書く。
 *
 * **React には触れない。** 描画は Canvas 側が rAF で `live` を読む。
 */
import { decode } from '@msgpack/msgpack'
import { markHistoryGap, pushHistory } from '../bus/history'
import { clearLive, noteTelemetry } from '../bus/live'
import { useUi } from '../store/ui'
import { notifyPeerLost } from './guard'
import { isSnapshot, schemaOf } from './messages'
import { reconnectingSocket } from './socket'
import { wsUrl } from './url'

const RECONNECT_MIN_MS = 300
const RECONNECT_MAX_MS = 3000
/** これだけ何も届かなければ、繋がっていても死んだとみなす [ms]。Pi は 50Hz で送り、`vehicle_state` が
 *  来ない間も周期ごとに送る（`telemetry_node._telemetry_pump`）。実測の最大間隔は約170ms（Wi-Fi） */
const IDLE_MS = 2000

export function connectTelemetry(
  onOpenChange: (open: boolean) => void,
  /** 型定義の食い違いを検出したときに1回だけ呼ぶ。**画面に出すこと** */
  onSchemaMismatch?: (got: number) => void,
): () => void {
  // 型が食い違っていることを既に伝えたか。**再接続しても直らない**ので、
  // 繋ぎ直すたびに鳴らしても意味が無い（GUI を再ビルドするまで続く）
  let mismatched = false
  // 直近に見た `LIMITS` の最高速度。**変わったとき（＝初めて届いたとき）に運転設定をクランプし直す**
  // （届く前は最高速度の上を切らずに持っている。`store/settings.ts` の `clampSettings`）
  let seenSpeedCap: number | null = null

  const sock = reconnectingSocket(wsUrl('/ws/telemetry'), {
    binaryType: 'arraybuffer',
    backoff: { minMs: RECONNECT_MIN_MS, maxMs: RECONNECT_MAX_MS },
    // 半開き・接続中のまま止まった接続を見切る（`guard.ts`）。見切ったら地図にも張り直させる
    idleMs: IDLE_MS,
    onDead: notifyPeerLost,
    onOpen: () => onOpenChange(true),
    onMessage: (ev) => {
      if (!(ev.data instanceof ArrayBuffer)) return
      // **`as Snapshot` は TypeScript に嘘をつく操作で、実行時には何も起きない。**
      // 全フィールドを実行時に舐める必要は無いが、**札を1つ見るだけ**で
      // 「Pi 側だけ型が変わった画面を信じて走る」は防げる（レビュー 🟠3。`ws/messages.ts`）
      const s: unknown = decode(new Uint8Array(ev.data))
      if (!isSnapshot(s)) {
        if (!mismatched) {
          mismatched = true          // 20Hz で流れるので通知は1回だけ
          onSchemaMismatch?.(schemaOf(s))
        }
        return                       // **捨てる。** 古い型のまま描くほうが危ない
      }
      noteTelemetry(s.vs, s.link, s.scan, s.auto, s.pi_temp_c, s.line_cam, s.track, s.signal)
      // 診断タブ用の時系列。**タブを開いていなくても常時貯める**（`bus/history.ts`）
      pushHistory(s.vs, s.link)
      const cap = s.link?.max_speed_m_s ?? null
      if (cap != null && cap !== seenSpeedCap) {
        seenSpeedCap = cap
        useUi.getState().setSettings({})
      }
    },
    onLost: () => {
      onOpenChange(false)
      clearLive()
      seenSpeedCap = null
      // 履歴は残したまま線だけ断つ。**切れたこと自体が後から見たい事象**
      markHistoryGap()
    },
  })
  return () => sock.close()
}
