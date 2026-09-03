/**
 * ウィンカー・ハザードの操作とインジケータ（2026-09-03、指示によりタブ行から
 * `DriveControls.tsx` の外へ移設）。
 *
 * 実車のダッシュボードに寄せ、`RcView.tsx` の `.rc-cam-gauges`（舵角計・速度計・
 * G メータの行）に組み込む——**舵角計の左に左ウィンカー、G メータの右に右
 * ウィンカー**をそれぞれ `.winker-side`（点滅表示を上段、矢印ボタンを下段に
 * 積んだ小さい列）として添える。メータ自体は縮めない（2026-09-03 に一度
 * 各メータをウィンカーごと包んで縮める方式にしたところ、実機で3つのメータの
 * 大きさが揃わなくなる不具合が出たため、隣に添えるだけの方式に直した）。
 * ハザードは実車と同じ赤い三角のボタン一つだけをメータパネル右下
 * （`.rc-cam-gauges` 自体に対して絶対配置）に置く——ハザード専用の点滅表示は
 * 無く、押すと左右の矢印インジケータが両方光る（STM32 側は元々「左右とも
 * ON」をハザードと解釈する。`protocol.toml` flags2）。
 *
 * 要求（`ui.winkerLeftRequested`/`RightRequested`）の切り替えは、ゲームパッド
 * 十字キー左右（`useDriving.ts` の `PAD_WINKER_LEFT`/`PAD_WINKER_RIGHT`）と
 * 同じ「片側ずつ独立トグル」。ハザードボタンはその両方を一括で ON/OFF する
 * だけで、専用の状態は持たない。
 *
 * 「要求している」と「実際に点滅しているか」（`vs.winker_left_active`/
 * `right_active`、実行はSTM32側）は別物——ボタンの `on` は要求を、上の
 * インジケータの点滅は実測を表す。旧 `DriveControls.tsx` にあった「点滅中/
 * 要求中…/解除」のテキスト表示は、点滅インジケータ自体が同じ情報を示すため
 * 廃止した。
 *
 * 低電圧強制ハザード（STM32側の安全設計。駆動系8.0V未満で要求内容に関わらず
 * 強制的に両方ON）の警告は、ハザードボタンの `title` に残している——旧実装の
 * `pill` 表示ほど目立たなくなるが、`vs.winker_left_active`/`right_active` が
 * 両方立てば上の緑矢印が両方点滅するので、異常事態自体は視覚的に伝わる。
 */
import { useNumbers } from '../../bus/live'
import { useUi } from '../../store/ui'

export function WinkerIndicator({ side }: { side: 'left' | 'right' }) {
  const active = useNumbers().vs?.[side === 'left' ? 'winker_left_active' : 'winker_right_active'] ?? false
  return (
    <div className={`winker-indicator ${active ? 'blink' : ''}`} aria-hidden="true">
      {side === 'left' ? '◀' : '▶'}
    </div>
  )
}

export function WinkerButton({ side }: { side: 'left' | 'right' }) {
  const ui = useUi()
  const requested = side === 'left' ? ui.winkerLeftRequested : ui.winkerRightRequested
  const toggle = () =>
    ui.set(
      side === 'left'
        ? { winkerLeftRequested: !ui.winkerLeftRequested }
        : { winkerRightRequested: !ui.winkerRightRequested },
    )
  return (
    <button
      className={`winker-btn ${requested ? 'on' : ''}`}
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
