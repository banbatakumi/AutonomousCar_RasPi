/**
 * ゲームパッドの読み取り（`useDriving.ts` の rAF ループから分けた。技術的負債の棚卸し 2026-11-18 高6）。
 *
 * - 使うパッドの選択（標準マッピングを優先）
 * - **圏外で最後の入力のまま張り付いたパッド**の検出（`PAD_STALE_MS`。下記）
 * - ボタンの立ち上がりエッジ（トグル操作用）
 *
 * 状態はこのクラスが持ち、毎フレーム `read()` を呼ぶ。指令の計算はしない（`drivingLogic.ts`）。
 */
import { padPressed, padSignature } from './drivingLogic'

/** トリガーのデッドゾーン（圏外判定で「中立でない」とみなす閾値にも使う） */
export const TRIGGER_DZ = 0.06
/** パッドの `timestamp` がこの時間進まないのに、トリガー・スティックが中立でないなら、
 * 圏外（Bluetooth が切れかけ）で**最後の入力のまま張り付いている**とみなして入力を中立にする。
 * ブラウザは接続が切れても `connected` を落とさないことがあるため、この保険は残す。
 *
 * ⚠ **`timestamp` は「入力が変わったとき」しか進まない機種がある**（macOS/Linux の多くの
 * ブラウザ）。その機種で一定に握り続けると、正常な操作なのに timestamp が止まり、圏外と
 * 誤判定してしまう。そこで**「入力が同じなのに timestamp が進む」ことを観測できた機種
 * （＝timestamp が心拍になっている機種）でだけ**この判定を有効にする（`tsHeartbeat`）。
 * 観測できない機種では判定を無効にし、切断は `gamepaddisconnected` と `connected` で拾う。
 */
const PAD_STALE_MS = 2000

export class GamepadReader {
  private seenIndex = -1
  private seenTs = -1
  private tsChangedMs = 0
  private staleWas = false
  /** `timestamp` が「入力が変わらなくても進む」機種か（接続中に自動判定する）。
   * **これが true のときだけ** `PAD_STALE_MS` の圏外判定を使う */
  private tsHeartbeat = false
  /** 「入力が同じなのに timestamp が進んだ」の連続観測回数。1回だけでは、
   * 2フレームの間に値を動かして戻した（変化を跨いだ）だけの可能性がある */
  private tsHeartbeatHits = 0
  /** 直前フレームの入力の署名。timestamp が進んだのに入力が同じかを見る */
  private lastSig = ''
  /** ボタンごとの直前の押下状態（立ち上がりエッジ用） */
  private was = new Map<number, boolean>()

  /**
   * 今フレームのパッドを読む。圏外とみなしたら `pad` は null（入力を中立にする）。
   * `staleStarted` は圏外とみなし始めたフレームだけ true（通知用）。
   */
  read(pads: readonly (Gamepad | null)[], now: number, steerDeadzone: number): { pad: Gamepad | null; staleStarted: boolean } {
    // 標準マッピングのパッドを優先する（ボタン番号が `PAD_*` と合うのはそれだけ。ヘッドセット等が
    // ゲームパッドとして先に列挙される環境がある）。無ければ従来どおり最初の1台
    const padRaw = pads.find((p) => p && p.connected && p.mapping === 'standard')
      ?? pads.find((p) => p && p.connected) ?? null
    // 圏外で最後の入力のまま張り付いたパッドは無いものとして扱う（`PAD_STALE_MS`）。
    // **`timestamp` が心拍として進む機種でだけ判定する**（`tsHeartbeat`）。値の変化で
    // しか進まない機種では、一定に握り続ける正常な操作と圏外を区別できないため。
    const sig = padSignature(padRaw)
    let stale = false
    if (padRaw) {
      if (padRaw.index !== this.seenIndex || padRaw.timestamp !== this.seenTs) {
        // **入力の署名が同じなのに timestamp が進んだ** → このブラウザは入力が無くても
        // パッドを定期更新している。3回連続で観測できたら timestamp を生存確認に使える
        // （1回だけなら、2フレームの間に値を動かして戻しただけの可能性がある）
        if (padRaw.index === this.seenIndex) {
          if (sig === this.lastSig) {
            this.tsHeartbeatHits += 1
            if (this.tsHeartbeatHits >= 3) this.tsHeartbeat = true
          } else {
            this.tsHeartbeatHits = 0
          }
        } else {
          this.tsHeartbeatHits = 0
        }
        this.seenIndex = padRaw.index
        this.seenTs = padRaw.timestamp
        this.tsChangedMs = now
      } else if (this.tsHeartbeat && now - this.tsChangedMs > PAD_STALE_MS) {
        stale =
          (padRaw.buttons[7]?.value ?? 0) > TRIGGER_DZ ||
          (padRaw.buttons[6]?.value ?? 0) > TRIGGER_DZ ||
          Math.abs(padRaw.axes[0] ?? 0) > steerDeadzone
      }
    } else {
      this.seenIndex = -1
    }
    this.lastSig = sig
    let staleStarted = false
    if (stale !== this.staleWas) {
      this.staleWas = stale
      staleStarted = stale
    }
    return { pad: stale ? null : padRaw, staleStarted }
  }

  /** ボタン `i` が今フレームで押された瞬間か（押しっぱなしは1回だけ）。パッドが無ければ「離した」扱い */
  rose(pad: Gamepad | null, i: number): boolean {
    const p = padPressed(pad?.buttons[i])
    const r = p && !this.was.get(i)
    this.was.set(i, p)
    return r
  }

  /**
   * **接続・切断はイベントでも受ける。** `timestamp` は機種によって心拍にならないので、
   * 圏外判定だけに頼らない。切断されたら状態を捨て、次のフレームから入力を中立にする
   * （`connected` を見るだけでも同じだが、その瞬間を通知できるのと、
   *  心拍判定の基準をパッドごとに取り直せるのが利点）。
   */
  onConnected(now: number): void {
    this.seenIndex = -1
    this.seenTs = -1
    this.tsChangedMs = now
    this.staleWas = false
    this.tsHeartbeat = false
    this.tsHeartbeatHits = 0
    this.lastSig = ''
  }

  /** 切断。戻り値は「使っていたパッドが切れた」（通知するか） */
  onDisconnected(index: number): boolean {
    const wasUsed = index === this.seenIndex
    this.seenIndex = -1
    this.seenTs = -1
    this.staleWas = false
    this.tsHeartbeat = false
    this.tsHeartbeatHits = 0
    this.lastSig = ''
    return wasUsed
  }
}
