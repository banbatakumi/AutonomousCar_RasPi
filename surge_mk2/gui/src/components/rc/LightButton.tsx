/**
 * 灯火ボタン（ラジコンの計器パネル、下の帯の右端）。押すたびに 消灯→DAY→NORMAL と回る
 * （`L` キー・パッド十字キー上と同じ順、`LIGHT_CYCLE`）。点灯中は琥珀色に発光、DAY は減光なので弱く光らせる。
 * 他タブ（自動運転）では従来どおりタブ行の `DriveControls` のセグメントで操作する。
 */
import { LIGHT_CYCLE, LIGHT_LABEL, LIGHT_OFF, useUi } from '../../store/ui'

export function LightButton() {
  const mode = useUi((s) => s.lightMode)
  const set = useUi((s) => s.set)
  const next = LIGHT_CYCLE[(LIGHT_CYCLE.indexOf(mode) + 1) % LIGHT_CYCLE.length]!
  return (
    <button
      className={`light-btn ${mode === LIGHT_OFF ? '' : 'on'} lv${mode}`}
      onClick={() => set({ lightMode: next })}
      title="灯火（押すたびに 消灯→DAY→NORMAL。L キー / パッド十字キー上でも送れる。DAY は減光）"
    >
      <span className="light-ico">☼</span>
      <span>{LIGHT_LABEL[mode]}</span>
    </button>
  )
}
