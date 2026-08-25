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
import { authToken } from './token'
import { wsUrl } from './url'

const RECONNECT_MS = 500

/**
 * `/ws/control` でサーバから降ってくる JSON の**振り分けに要る分だけ**の形。
 *
 * 全フィールドを実行時に検証はしない（`type` で分岐したあとの中身は
 * `ControlStatus` の宣言を信じる）。ここで潰したいのは「`any` にしたせいで
 * 打ち間違いまで通る」ことのほうで、境界の実行時検証は
 * `/ws/telemetry` 側の札（`MSGS_SCHEMA`）が担っている。
 */
type ServerMsg = {
  type?: string
  holder?: string
  reason?: string
  files?: LogFile[]
  id?: number
}

export type ControlHandlers = {
  onStatus: (s: ControlStatus) => void
  onOpenChange: (open: boolean) => void
  /** `holder` は操縦権を持っている相手。`reason` は拒否の理由
   * （`bad_token` = トークン不一致、`auto_engage` = engage に操縦権が要る） */
  onDenied: (holder: string, reason?: string) => void
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
      // **`any` にしない。** `any` は以降のフィールドアクセスを全て検査対象から
      // 外すので、`m.holdr` のような打ち間違いも通る。`unknown` に寄せた形の
      // 部分型で受けて、使うところで絞る
      let m: ServerMsg
      try {
        m = JSON.parse(ev.data as string) as ServerMsg
      } catch {
        return
      }
      if (m.type === 'status') this.h.onStatus(m as unknown as ControlStatus)
      else if (m.type === 'control_denied') this.h.onDenied(m.holder ?? '', m.reason)
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
    this.send({ type: 'take_control', name: this.id, token: authToken() })
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

  // ── 自動運転 ──

  /**
   * モード選択・engage・パラメータ。**3つとも省略できる**（送った項目だけ効く）。
   *
   * `engaged` を GUI 側で覚えないのは、**サーバが真値**だから。ブラウザを2枚
   * 開いたときに片方だけ「走っているつもり」になるのを防ぐ。押した結果は
   * `status` のブロードキャストで返ってくる。
   *
   * ⚠ **モードを変えると engage は必ず落ちる**（サーバ側の約束）。
   */
  setAuto(p: { mode?: string; engaged?: boolean; params?: Record<string, number> }) {
    this.send({ type: 'auto', ...p })
  }

  /**
   * 「地図を確定」。**周回の自動判定が滑ったときの逃げ道**（`nav/slam.py`）。
   *
   * 真偽値ではなく**回数**でサーバへ渡る（`AutoCtrl.freeze_seq`）。あちらは
   * 現在の意思を繰り返し流す設計なので、`true` を立てると押していないのに
   * 何度も確定してしまう。
   */
  freezeMap() {
    this.send({ type: 'auto', freeze_map: true })
  }

  /**
   * 「地図を削除」。**全部捨てて地図作成からやり直す。**
   *
   * engage は落ちない（ARM の保持を巻き込まないため）。経路が消えるので、
   * 走行中に押しても次の周期で制動に落ちる。
   */
  clearMap() {
    this.send({ type: 'auto', clear_map: true })
  }

  // ── ファン（Pi5純正クーリング） ──

  /**
   * 自動/手動と、手動時のデューティ。**両方省略できる**（送った項目だけ効く）。
   * `auto` と同じく、状態はサーバが真値。GUI側では持たない。
   */
  setFan(p: { mode?: 'auto' | 'manual'; duty?: number }) {
    this.send({ type: 'fan', ...p })
  }

  // ── TC/TV 有効切り替え（★v0.8） ──

  /**
   * STM32側のトラクションコントロール/トルクベクタリングの有効・無効を切り替える。
   * `fan` と同じく状態はサーバ（STM32の`CONFIG_ACK`）が真値。**両方省略できる**（送った項目だけ効く）。
   */
  setTcTv(p: { tc?: boolean; tv?: boolean }) {
    this.send({ type: 'tc_tv', ...p })
  }

  // ── カメラ capture 設定（後方ON/OFF・前後FPS上限・GUI配信fps） ──

  /**
   * capture側(camera_node)のFPS上限・後方カメラの取得ON/OFF・GUIへのJPEG配信頻度。
   * `fan`/`tc_tv` と同じく状態はサーバが真値。**4つとも省略できる**（送った項目だけ効く）。
   *
   * `frontCapHz` は目安の上限にすぎない——カメラを使う自動運転モード
   * （`line_trace`/`ftg_cam`）が engage されている間は、サーバ側がこれを無視して
   * 上限まで引き上げる（`status.camera_config.auto_override` で分かる）。
   * `guiHz` はブラウザへ送るJPEGの頻度で、`frontCapHz`/`rearCapHz` とは別物
   * （前者はWi-Fi帯域、後者はcamera_nodeの消費電力が理由）。
   */
  setCamera(p: { frontCapHz?: number; rearCapHz?: number; rearEnabled?: boolean; guiHz?: number }) {
    this.send({
      type: 'camera',
      front_cap_hz: p.frontCapHz,
      rear_cap_hz: p.rearCapHz,
      rear_enabled: p.rearEnabled,
      gui_hz: p.guiHz,
    })
  }

  // ── 片輪浮き対策 有効切り替え（★v0.9） ──

  /**
   * STM32側の片輪浮き対策（後輪片浮き検知・トルク遮断）の有効・無効を切り替える。
   * TC/TV本体とは独立した別機構。`tc_tv` と同じく状態はサーバ（STM32の`CONFIG_ACK`）が真値。
   */
  setWheelLiftGuard(enabled: boolean) {
    this.send({ type: 'wheel_lift_guard', enabled })
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

  /** Pi を安全にシャットダウンする。estop と同じく誰でも実行できる（走行の操縦権とは無関係）。
   * 実行するかどうかの確認は呼び出し側（`StatusBar.tsx`）の責務 */
  shutdown() {
    this.send({ type: 'shutdown' })
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
