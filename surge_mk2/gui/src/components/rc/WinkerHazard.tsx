/**
 * ウィンカー・ハザードの操作とインジケータ。
 *
 * ウィンカーは**点滅表示とボタンを1つにした矢印ランプ**（`WinkerLamp`）。ラジコンの計器パネルの
 * シフトライトの両端に置く（実車のメータ上端の左右の矢印の位置。2026-11-13）。押すと要求が切り替わり、
 * 要求中は縁が緑、STM32 が実際に点滅させている間は矢印が緑に点滅する。以前は点滅表示と押すボタンを
 * 別々に置いていたが、場所を2倍食ううえ「どっちを見ればいいか」が分かれていた。
 * ハザードは実車と同じ赤い三角のボタン一つだけをパネルの斜辺の下に置く——ハザード専用の点滅表示は
 * 無く、押すと左右の矢印インジケータが両方光る（STM32 側は「左右とも
 * ON」をハザードと解釈する。`protocol.toml` flags2）。
 *
 * 要求（`ui.winkerLeftRequested`/`RightRequested`）の切り替えは、ゲームパッド
 * 十字キー左右（`useDriving.ts` の `PAD_WINKER_LEFT`/`PAD_WINKER_RIGHT`）と
 * 同じ「片側ずつ独立トグル」。ハザードボタンはその両方を一括で ON/OFF する
 * だけで、専用の状態は持たない。
 *
 * 「要求している」と「実際に点滅しているか」（`vs.winker_left_active`/
 * `right_active`、実行はSTM32側）は別物——ボタンの `on` は要求を、上の
 * インジケータの点滅は実測を表す。
 *
 * 低電圧強制ハザード（STM32側の安全設計。駆動系8.0V未満で要求内容に関わらず
 * 強制的に両方ON）の警告は、ハザードボタンの `title` に書いてある。`vs.winker_left_active`/`right_active` が
 * 両方立てば上の緑矢印が両方点滅するので、異常事態自体は視覚的に伝わる。
 */
import { useNumbers } from '../../bus/live'
import { useUi } from '../../store/ui'

export function WinkerLamp({ side }: { side: 'left' | 'right' }) {
  // 点滅は実測（STM32 が実際に点滅させているか）、縁は要求。冒頭コメント参照
  const active = useNumbers().vs?.[side === 'left' ? 'winker_left_active' : 'winker_right_active'] ?? false
  const requested = useUi((s) => (side === 'left' ? s.winkerLeftRequested : s.winkerRightRequested))
  const set = useUi((s) => s.set)
  const toggle = () => set(side === 'left' ? { winkerLeftRequested: !requested } : { winkerRightRequested: !requested })
  return (
    <button
      className={`winker-lamp ${requested ? 'req' : ''} ${active ? 'blink' : ''}`}
      onClick={toggle}
      title={`${side === 'left' ? '左' : '右'}ウィンカー（パッド十字キー${side === 'left' ? '左' : '右'}でも送れる）`}
    >
      {side === 'left' ? '◀' : '▶'}
    </button>
  )
}

export function HazardButton() {
  const ui = useUi()
  const forced = useNumbers().vs?.faults?.includes('drive_undervoltage') ?? false
  const on = ui.winkerLeftRequested && ui.winkerRightRequested
  const toggle = () =>
    ui.set(
      on
        ? { winkerLeftRequested: false, winkerRightRequested: false }
        : { winkerLeftRequested: true, winkerRightRequested: true },
    )
  return (
    <button
      className={`hazard-btn ${on ? 'on' : ''}`}
      onClick={toggle}
      title={
        forced
          ? '駆動系バッテリー電圧低下（8.0V未満）のため、STM32側の安全設計で強制的にハザードになっています。片側だけの点滅を試すにはバッテリーを充電し8.8V以上に回復させてください'
          : 'ハザード（左右同時点滅）'
      }
    >
      ▲
    </button>
  )
}
