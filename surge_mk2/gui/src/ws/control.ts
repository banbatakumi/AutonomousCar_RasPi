/**
 * `/ws/control` — 操縦権・走行指令・E-Stop。JSON。
 *
 * ## 送らないことが「止める」意味を持つ
 *
 * デッドマンを離したら **`speed=0` を送るのではなく、送信そのものを止める**。
 * サーバ側は 150ms 無音で DISARM に落ちる。こうしておくと、
 * ブラウザが固まった / タブが背後に回った / Wi-Fi が切れた のいずれでも
 * 同じ経路で停止する。「止める指令を送る」設計だと、その指令が届かない
 * 状況（＝一番止めたい状況）で止まらない。
 */
import type { CmdOut, ControlStatus, LogFile } from '../types'
import { wsUrl } from './url'

const RECONNECT_MS = 500

export type ControlHandlers = {
  onStatus: (s: ControlStatus) => void
  onOpenChange: (open: boolean) => void
  onDenied: (holder: string) => void
  /** 往復遅延の実測 [ms]。**GUI↔Pi 区間**（UART 区間は link.cmd_rtt_ms） */
  onRtt: (ms: number) => void
  /** `logs_list`/`logs_delete` への応答 */
  onLogs: (files: LogFile[]) => void
}

export class ControlChannel {
  private ws: WebSocket | null = null
  private timer: number | undefined
  private closed = false
  private pingId = 0
  private pingSentAt = 0

  /**
   * このタブの識別子。**サーバの `status.controller` と突き合わせて
   * 「操縦権を持っているのが自分かどうか」を判定する。**
   * 「誰かが持っている」だけでは、2枚目のタブが自分だと誤認する。
   */
  readonly id = `gui-${Math.random().toString(36).slice(2, 8)}`

  constructor(private h: ControlHandlers) {
    this.open()
  }

  private open() {
    const ws = new WebSocket(wsUrl('/ws/control'))
    this.ws = ws
    ws.onopen = () => this.h.onOpenChange(true)
    ws.onmessage = (ev) => {
      let m: any
      try {
        m = JSON.parse(ev.data as string)
      } catch {
        return
      }
      if (m.type === 'status') this.h.onStatus(m as ControlStatus)
      else if (m.type === 'control_denied') this.h.onDenied(m.holder ?? '')
      else if (m.type === 'logs') this.h.onLogs(m.files ?? [])
      else if (m.type === 'pong' && m.id === this.pingId) {
        this.h.onRtt(performance.now() - this.pingSentAt)
      }
    }
    ws.onclose = () => {
      this.h.onOpenChange(false)
      if (!this.closed) this.timer = window.setTimeout(() => this.open(), RECONNECT_MS)
    }
    ws.onerror = () => ws.close()
  }

  private send(obj: unknown) {
    if (this.ws?.readyState === WebSocket.OPEN) this.ws.send(JSON.stringify(obj))
  }

  takeControl() {
    this.send({ type: 'take_control', name: this.id })
  }

  releaseControl() {
    this.send({ type: 'release_control' })
  }

  /** 誰でも押せる。**止める操作に条件をつけない。** */
  estop() {
    this.send({ type: 'estop' })
  }

  cmd(c: Omit<CmdOut, 'type'>) {
    this.send({ type: 'cmd', ...c })
  }

  // ── 記録・再生 ──

  /** `.sfl` の記録意思を送る。実際に開閉するのは io_node（`log/ctrl` 経由） */
  sflRecord(active: boolean) {
    this.send({ type: 'sfl_record', active })
  }

  /** mcap のライブ中継を開始する。`/ws/record` を別途つないでおくこと */
  mcapRecordStart(imageHz?: number) {
    this.send({ type: 'mcap_record_start', image_hz: imageHz })
  }

  mcapRecordStop() {
    this.send({ type: 'mcap_record_stop' })
  }

  logsList() {
    this.send({ type: 'logs_list' })
  }

  logsDelete(name: string) {
    this.send({ type: 'logs_delete', name })
  }

  ping() {
    this.pingId++
    this.pingSentAt = performance.now()
    this.send({ type: 'ping', id: this.pingId })
  }

  close() {
    this.closed = true
    window.clearTimeout(this.timer)
    this.ws?.close()
  }
}
