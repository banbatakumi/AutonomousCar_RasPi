/**
 * 駆動配分パネル — 車体を上から見た図に、車輪ごとの情報をそのまま重ねる。
 *
 * ## 改訂履歴
 *
 * - 2026-08-17: 数値の並び（`bar-row`）からイラストに変更。
 * - 2026-08-17: 実機の画面で見ると小さすぎて読めなかったため全面的に整理。
 *   車軸・ディファレンシャルの図は装飾でしかなく情報を持っていなかったので消し、
 *   スリップはバーをやめて数字だけにし、空いた面積を後輪電流ゲージ（車輪の外側）
 *   と駆動・信号の電圧・電流（乾電池型ゲージ、車体中央）に使った。温度は一旦
 *   ここから外した（診断タブでは見られる）。車体全体はスリムに、かつ大きく
 *   表示する。
 * - 2026-08-17: `<svg>` は replaced element なので `align-items: stretch` の
 *   対象にならず、`width:100%` だけでは親の幅まで広がらなかった
 *   （実機で見ると豆粒サイズだった）。`.drivepanel-wrap`（ただの div）で幅を
 *   確定させてから中の svg に `width:100%` を効かせる形に直した。
 * - 2026-08-17: 後輪電流ゲージを車輪の外側から**タイヤの中**へ統合（指示による）。
 *   タイヤの矩形をそのままバーグラフの器として使い、車軸の高さ（0A）を
 *   境に上へ伸びれば駆動、下へ伸びれば回生。数値（電流・トルク）はタイヤの
 *   すぐ下（車体から遠い側）に置く — スリップの数字はタイヤと車体の間にあるので、
 *   反対側に出さないと重なる。ついでに前後輪とも直径を少し大きくした。
 *
 * ## バー／ゲージの色の意味
 *
 * 電流ゲージは単色（量だけを示す。安全上の判定は持たない）。満スケール
 * ±10A はバンビ指定の表示レンジで、駆動系統の過電流ラッチ 5.0A
 * （`docs/stm32_interface.md` §5.3、系統合計でモータ個別の閾値ではない）とは
 * 別物。電圧ゲージは `battLevel`（実在する放電終止電圧 8.0V 基準）で
 * ok/warn/bad に色分けする — こちらは走行を切り上げる判断に使う実際の閾値。
 *
 * ## 前輪に電流・トルクが無い理由
 *
 * 前輪は駆動モータを持たない（操舵のみ）。指令トルク・電流は後輪2輪分しか
 * 存在しない。
 */
import { useNumbers } from '../../bus/live'
import { battLevel } from '../../format'

/** 後輪電流ゲージの満スケール [A]。バンビ指定の表示レンジ */
const CURRENT_MAX = 10.0
/** 8セル NiMH。放電終止 8.0V / 満充電 11.2V（`uart_protocol.md` §5.3） */
const BATT_MIN = 8.0
const BATT_MAX = 11.2

const CX_L = 42
const CX_R = 158
const CY_FRONT = 46
const CY_REAR = 194
const WHEEL_W = 17
const WHEEL_H = 38
const BODY_X = 58
const BODY_Y = 26
const BODY_W = 84
const BODY_H = 182

export function DrivePanel() {
  const vs = useNumbers().vs
  const slipF = vs?.slip_front ?? null
  const slipR = vs?.slip_rear ?? null
  const cur = vs?.motor_current ?? null
  const trq = vs?.torque_cmd ?? null
  const bv = vs?.batt_voltage ?? null
  const ba = vs?.batt_current ?? null

  return (
    <div className="drivepanel-wrap">
      <svg viewBox="0 0 200 260" className="drivepanel">
        <rect className="dp-body" x={BODY_X} y={BODY_Y} width={BODY_W} height={BODY_H} rx={16} />

        <Wheel cx={CX_L} cy={CY_FRONT} />
        <Wheel cx={CX_R} cy={CY_FRONT} />
        <Wheel cx={CX_L} cy={CY_REAR} label="後左" current={cur ? cur[0] : null} torque={trq ? trq[0] : null} />
        <Wheel cx={CX_R} cy={CY_REAR} label="後右" current={cur ? cur[1] : null} torque={trq ? trq[1] : null} />

        <SlipText cx={CX_L} y={CY_FRONT + 30} label="FL" value={slipF ? slipF[0] : null} />
        <SlipText cx={CX_R} y={CY_FRONT + 30} label="FR" value={slipF ? slipF[1] : null} />
        <SlipText cx={CX_L} y={CY_REAR - 30} label="RL" value={slipR ? slipR[0] : null} />
        <SlipText cx={CX_R} y={CY_REAR - 30} label="RR" value={slipR ? slipR[1] : null} />

        <BatteryGauge cx={75} cy={112} label="駆動" v={bv ? bv[0] : null} a={ba ? ba[0] : null} />
        <BatteryGauge cx={125} cy={112} label="信号" v={bv ? bv[1] : null} a={ba ? ba[1] : null} />
      </svg>
    </div>
  )
}

/**
 * 車輪1つ。後輪（`current`/`torque` が渡されたとき）は**タイヤ自体を電流ゲージ**
 * にする — 車軸の高さ（y=cy）を0Aとして、駆動なら上、回生なら下へタイヤの中を塗る。
 * 前輪は駆動モータを持たないので、ただのタイヤ（トレッド柄だけ）として描く。
 */
function Wheel({
  cx,
  cy,
  label,
  current = null,
  torque = null,
}: {
  cx: number
  cy: number
  label?: string
  current?: number | null
  torque?: number | null
}) {
  const gauge = label != null
  const x = cx - WHEEL_W / 2
  const y = cy - WHEEL_H / 2
  const clipId = `wheelclip-${cx}-${cy}`

  const frac = current == null ? 0 : Math.max(-1, Math.min(1, current / CURRENT_MAX))
  const fillH = Math.abs(frac) * (WHEEL_H / 2)
  const fillY = frac >= 0 ? cy - fillH : cy

  return (
    <g>
      {gauge && (
        <title>{`${label}モータ 電流（タイヤの中＝ゲージ。上が駆動・下が回生）: ${
          current == null ? '—' : `${current.toFixed(2)}A`
        } / 指令トルク: ${torque == null ? '—' : `${torque.toFixed(3)}N·m`}`}</title>
      )}
      <defs>
        <clipPath id={clipId}>
          <rect x={x} y={y} width={WHEEL_W} height={WHEEL_H} rx={4} />
        </clipPath>
      </defs>

      <rect className="dp-wheel" x={x} y={y} width={WHEEL_W} height={WHEEL_H} rx={4} />

      {gauge && (
        <g clipPath={`url(#${clipId})`}>
          <rect className="dp-wheel-fill" x={x} y={fillY} width={WHEEL_W} height={fillH} />
          <rect className="dp-wheel-zero" x={x} y={cy - 0.75} width={WHEEL_W} height={1.5} />
        </g>
      )}

      {/* トレッド柄。ゲージの塗りの上からでも見えるよう、塗りより後に描く */}
      {[0.28, 0.5, 0.72].map((f) => (
        <line
          key={f}
          className="dp-wheel-tread"
          x1={x + 2.5}
          x2={x + WHEEL_W - 2.5}
          y1={y + WHEEL_H * f}
          y2={y + WHEEL_H * f}
        />
      ))}
      <rect className="dp-wheel-outline" x={x} y={y} width={WHEEL_W} height={WHEEL_H} rx={4} />

      {gauge && (
        <>
          <text className="dp-value" x={cx} y={cy + WHEEL_H / 2 + 13} textAnchor="middle">
            {current == null ? '—' : `${current.toFixed(2)}A`}
          </text>
          <text className="dp-label" x={cx} y={cy + WHEEL_H / 2 + 24} textAnchor="middle">
            {torque == null ? '—' : `${torque.toFixed(3)}N·m`}
          </text>
        </>
      )}
    </g>
  )
}

/** スリップは数字のみ（ok/warn/bad の判定は持たない。TC のしきい値が未実装のため） */
function SlipText({ cx, y, label, value }: { cx: number; y: number; label: string; value: number | null }) {
  return (
    <g>
      <title>{`滑り ${label}（射影後の車輪速 − 車体速度）: ${value == null ? '—' : `${value.toFixed(2)}m/s`}`}</title>
      <text className="dp-value" x={cx} y={y} textAnchor="middle">
        {value == null ? '—' : value.toFixed(2)}
      </text>
    </g>
  )
}

/** 駆動・信号系統の電圧（乾電池型ゲージ、`battLevel` で色分け）と電流（数値のみ） */
function BatteryGauge({
  cx,
  cy,
  label,
  v,
  a,
}: {
  cx: number
  cy: number
  label: string
  v: number | null
  a: number | null
}) {
  const W = 14
  const H = 32
  const lv = v == null ? 'dim' : battLevel(v)
  const frac = v == null ? 0 : Math.max(0, Math.min(1, (v - BATT_MIN) / (BATT_MAX - BATT_MIN)))
  // 放電終止未満でも frac は 0 に張り付く。**赤なのに何も見えない**のを避けるため
  // 最低でも少しだけ塗る（v == null のときだけ本当に空にする）
  const h = v == null ? 0 : Math.max(3, frac * (H - 4))

  return (
    <g>
      <title>{`${label}系: ${v == null ? '—' : `${v.toFixed(2)}V`} / ${a == null ? '—' : `${a.toFixed(2)}A`}`}</title>
      <text className="dp-label" x={cx} y={cy - H / 2 - 12} textAnchor="middle">
        {label}
      </text>
      {/* 乾電池の＋極 */}
      <rect className="dp-batt-nub" x={cx - 3} y={cy - H / 2 - 6} width={6} height={6} rx={1.5} />
      <rect className="dp-batt-body" x={cx - W / 2} y={cy - H / 2} width={W} height={H} rx={3} />
      <rect
        className={`lv-bg-${lv}`}
        x={cx - W / 2 + 2}
        y={cy + H / 2 - 2 - h}
        width={W - 4}
        height={h}
        rx={1.5}
      />
      <text className="dp-value" x={cx} y={cy + H / 2 + 12} textAnchor="middle">
        {v == null ? '—' : `${v.toFixed(2)}V`}
      </text>
      <text className="dp-label" x={cx} y={cy + H / 2 + 23} textAnchor="middle">
        {a == null ? '—' : `${a.toFixed(2)}A`}
      </text>
    </g>
  )
}
