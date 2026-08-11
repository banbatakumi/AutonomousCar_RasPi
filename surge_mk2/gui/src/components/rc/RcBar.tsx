/**
 * ラジコンの計器パネル（右 1/3 の下半分）。実車のダッシュボードに載っているものだけを置く。
 *
 * ## 何を出さないか
 *
 * スリップ量の数値、CRC / パケットロス、時刻同期のオフセット、UART・WS の RTT、
 * モータ電流、pitch / roll は**ここには出さない**。運転を楽しむのに使わないうえ、
 * 面積を食って速度計と G メータが小さくなる。全部いつでも診断タブで見られる。
 *
 * ## 逆に温度と電圧は出す
 *
 * 実車の水温計・燃料計に相当する。**走行を切り上げる判断に使う**ので、
 * 異常時だけ出るのでは遅い（`DiagStrip` の畳み方とは扱いを変えている）。
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
import { battLevel, tempLevel } from '../../format'
import { LIGHT_CYCLE, LIGHT_LABEL, useUi } from '../../store/ui'
import { AssistLamps } from './AssistLamps'
import { GMeter } from './GMeter'
import { SpeedGauge } from './SpeedGauge'

/** 温度バーの表示範囲。下限を 20℃ にしないと常に半分以上溜まって見える */
const TEMP_MIN = 20
const TEMP_MAX = 80
/** 8セル NiMH。放電終止 8.0V / 満充電 11.2V（`uart_protocol.md` §5.3） */
const BATT_MIN = 8.0
const BATT_MAX = 11.2

const TEMP_LABEL = ['MD後左', 'MD後右', 'MDステア', 'MCU']

export function RcBar() {
  const n = useNumbers()
  const ui = useUi()
  const vs = n.vs

  return (
    <div className={`rcpanel ${n.stale ? 'stale' : ''}`}>
      <div className="rc-gauges">
        <SpeedGauge />
        <GMeter />
      </div>

      <AssistLamps />

      <div className="rc-small">
        <TempBar />
        {/* 駆動と信号は別系統。**片方だけ落ちる**（駆動を切っても Pi と STM32 は生きている）
            ので、まとめずに2本出す。電流はバーにせず数字だけ — 上限が決まっていない */}
        <BattBar
          label="駆動"
          v={vs?.batt_voltage[0] ?? null}
          a={vs?.batt_current[0] ?? null}
          title="駆動系（8セル NiMH）。8.0V で放電終止。モータはこちらから取る"
        />
        <BattBar
          label="信号"
          v={vs?.batt_voltage[1] ?? null}
          a={vs?.batt_current[1] ?? null}
          title="信号系。STM32・センサ・LED。ここが落ちると車両が無言になる"
        />
        <span className="note">電流は単方向センサ。回生中は 0 に張り付く</span>
      </div>

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

      <span className="note">灯火・ホーン・パッシングは ARM していなくても効く</span>
    </div>
  )
}

/**
 * 水温計。**4点の最悪値**を1本にまとめる（内訳はホバーで出す）。
 *
 * ⚠ **未接続を赤で出さない。** テレメトリが来ていないだけの状態を「異常」と
 * 同じ色にすると、本当に温度が上がったときの赤が効かなくなる。
 * 値が来ていないなら灰色で「—」。MD からの通信断（`temp[i] == null`）は
 * 本物の異常なので、そちらは赤のままにする。
 */
function TempBar() {
  const vs = useNumbers().vs
  const temps = vs?.temp ?? []
  const valid = temps.filter((t): t is number => t != null)
  // 通信断（null）が1つでもあれば、最大値より先にそれを伝える
  const lost = temps.some((t) => t == null)
  const max = valid.length ? Math.max(...valid) : null
  const lv = !vs ? 'dim' : lost ? 'bad' : tempLevel(max)
  const frac = max == null ? 0 : (max - TEMP_MIN) / (TEMP_MAX - TEMP_MIN)
  const title = vs
    ? temps.map((t, i) => `${TEMP_LABEL[i]} ${t == null ? '通信断' : `${t}℃`}`).join(' / ')
    : 'テレメトリ未受信'

  return (
    <div className="bar-row" title={title}>
      <span className="label">温度</span>
      <div className="minibar">
        <div className={`minibar-fill lv-bg-${lv}`} style={{ width: `${clamp01(frac) * 100}%` }} />
      </div>
      <b className={`lv-${lv}`}>{!vs ? '—' : lost ? '通信断' : max == null ? '—' : `${max}℃`}</b>
    </div>
  )
}

/** 燃料計。電圧はバー、電流は数字だけ（消費電流に「満タン」が無いのでバーにできない） */
function BattBar({
  label,
  v,
  a,
  title,
}: {
  label: string
  v: number | null
  a: number | null
  title: string
}) {
  const lv = v == null ? 'dim' : battLevel(v)
  const frac = v == null ? 0 : (v - BATT_MIN) / (BATT_MAX - BATT_MIN)

  return (
    <div className="bar-row" title={title}>
      <span className="label">{label}</span>
      <div className="minibar">
        <div className={`minibar-fill lv-bg-${lv}`} style={{ width: `${clamp01(frac) * 100}%` }} />
      </div>
      <b className={`lv-${lv}`}>{v == null ? '—' : `${v.toFixed(2)}V`}</b>
      <b className="dim">{a == null ? '—' : `${a.toFixed(2)}A`}</b>
    </div>
  )
}

function clamp01(v: number): number {
  return Math.max(0, Math.min(1, v))
}
