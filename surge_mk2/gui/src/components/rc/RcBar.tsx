/**
 * ラジコンの計器パネル（右 1/3 の下半分）。実車のダッシュボードに載っているものだけを置く。
 *
 * ## 何を出さないか
 *
 * CRC / パケットロス、時刻同期のオフセット、UART・WS の RTT、pitch / roll は
 * **ここには出さない**。運転を楽しむのに使わないうえ、診断タブで見られる。
 *
 * ## 速度計・G メータは左の映像下へ移した（2026-08-17）
 *
 * 右パネルは LiDAR と駆動配分パネルの上下 1:1 に専念させたいという要望で、
 * `SpeedGauge`/`GMeter` は `RcView.tsx` の `.rc-cam-gauges`（前方映像の下）へ
 * 移動した。ここには残っていない。
 *
 * ## 駆動配分パネルに統合した（2026-08-17）
 *
 * スリップ・モータ電流・トルク・駆動と信号の電圧電流は `DrivePanel`（車体を
 * 上から見た図）にまとめてある。以前ここにあった水温計・燃料計の `TempBar`/
 * `BattBar` も、電圧・電流は `DrivePanel` の乾電池型ゲージに統合した。温度は
 * 一旦落としたが、同日中に指示で `TempPanel` として復活させた（下記）。
 * しきい値の考え方など詳細は `DrivePanel.tsx` 冒頭のコメントを参照。
 *
 * ## 温度計を車体図の下に復活（2026-08-17）
 *
 * 後輪モータ2つ・ステアモータ・STM32 MCU・RasPi 本体の CPU、計5つ
 * （`TempPanel.tsx`）。RcView 冒頭の方針どおり「温度と電圧は常時出す」に
 * 揃えた——電圧（`DrivePanel` の乾電池ゲージ）はあるのに温度が無いのは
 * 片手落ちだった。
 *
 * ## 補機もここに畳んだ（2026-08-11）
 *
 * 映像を左 2/3 の全面に使うため、下の補機帯（`AuxPanel`）は無くなった。
 * **操作するもの（灯火・自動停止）だけをここに残し、押下状態（ホーン・パッシング・
 * ブレーキ）は介入ランプの列に吸収した。** マウスで押しっぱなしにするボタンは
 * 置かない — カーソルが外れた瞬間に取り残されると、クラクションが鳴りっぱなしになる
 * （`AuxPanel.tsx` 冒頭の議論と同じ理由）。
 */
import { useNumbers } from '../../bus/live'
import { LIGHT_CYCLE, LIGHT_LABEL, useUi } from '../../store/ui'
import { AssistLamps } from './AssistLamps'
import { DrivePanel } from './DrivePanel'
import { TempPanel } from './TempPanel'

export function RcBar() {
  const n = useNumbers()
  const ui = useUi()
  const vs = n.vs

  return (
    <div className={`rcpanel ${n.stale ? 'stale' : ''}`}>
      <AssistLamps />

      <DrivePanel />

      <TempPanel />

      <div className="rc-small">
        <div className="kv">
          <span>走行距離</span>
          <b>{vs ? `${vs.odom_center.toFixed(2)}m` : '—'}</b>
        </div>
      </div>

      {/* 走行中に触るのは灯火だけ。**自動停止は設定パネルへ移した**（走りながら
          切り替えるものではなく、作業の性質で決める設定のため） */}
      <div className="rc-controls">
        <div className="rc-ctl">
          <span className="label">灯火</span>
          <div className="seg">
            {LIGHT_CYCLE.map((m) => (
              <button
                key={m}
                className={ui.lightMode === m ? 'on' : ''}
                onClick={() => ui.set({ lightMode: m })}
                title="L キー / パッド △ でも送れる。DAY は減光（duty 0.1）"
              >
                {LIGHT_LABEL[m]}
              </button>
            ))}
          </div>
        </div>
      </div>
    </div>
  )
}
