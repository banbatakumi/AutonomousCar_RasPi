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
 * - 2026-08-24: 駆動・信号バッテリーを横並びから縦積み（信号=上/駆動=下）に変更し、
 *   駆動バッテリー→後輪2輪へ流れるフローライン（破線アニメーション）を追加。
 *   タイヤ内電流ゲージにも控えめな発光を追加した。電流の符号は前進/後退の別で
 *   あって回生ブレーキの意味は無い（この車体は回生を行わない）ため、フローラインは
 *   符号に関わらず常にバッテリー→モータの向きにのみ流す（指示による）。
 * - 2026-08-25: 車体を囲む障害物近接リング（`ProximityRing.tsx`）を追加。
 *   LiDAR 360点と `vehicle.toml` の車体外形・LiDAR取付位置から、車体まで
 *   10cm を切った方向だけ黄→赤で光る。
 * - 2026-08-25: 車輪位置をトレッド・ホイールベース・車輪径の実測値
 *   （`carLayout.ts`）から算出するよう変更。以前は見た目だけで決めた固定値
 *   だったため、実車のトレッド（0.155m）が車体幅（0.18m）より狭いことが
 *   反映されず、車輪・スリップ・トルク表示が近接リングの表示域（車体の
 *   すぐ外）に被っていた。あわせて車体矩形は車輪のある場所を切り欠く
 *   （フェンダーからタイヤが覗く見た目にする）。
 * - 2026-08-25: スリップをタイヤ脇の独立バーにしたが（車輪の切り欠きのすぐ
 *   内側に置いた）、バッテリー表示との間に挟まって見にくかった（指摘による）。
 *   **タイヤそのものを縦2分割**する形に変更——後輪は「車体側半分＝電流ゲージ
 *   （従来どおり）／外側半分＝スリップゲージ」、前輪はモータを持たず電流
 *   ゲージが要らないので**タイヤ全幅をスリップゲージ**にした。後輪の数字
 *   表示は電流・トルクの2行のみ（スリップはバーの塗りだけで示し、正確な値は
 *   ホバーの `<title>` で見る——タイヤが17svg単位しかなく、半分ずつに数字を
 *   置くと文字が重なって逆効果だったため）。前輪は空きがあるのでスリップの
 *   数字も1行添える。
 * - 2026-08-25: 駆動・信号バッテリーのラベルをタイヤ上の別行（「信号」「駆動」）
 *   からバッテリーイラスト内の1文字（S＝信号／P＝駆動）に変更（指示による）。
 *   フルネームはホバーの `<title>` に残す。あわせて、車体図内の数値はどれも
 *   小数第1位までに統一した（`docs/architecture.md` §15 のとおりスリップは
 *   元々未確定の仮表示なので、電流・トルク・電圧も診断タブほどの精度は
 *   ここでは要らない）。
 * - 2026-08-25: 上の変更への指摘を反映。(1) 後輪のスリップも数字で出す
 *   よう復活——電流・トルクと反対側（車体側＝タイヤの上）に置いて文字の
 *   重なりを避けた。(2) S/P のチップが半透明黒だったため満充電に近いと
 *   塗り（`.lv-bg-*`）の色と混ざって読みにくかった。不透明な `--panel2`
 *   に変更。(3) 駆動バッテリー→後輪のフローラインが微妙な斜め線になって
 *   いたので、両端の y を `CY_REAR` に揃えて水平にした。
 * - 2026-08-28: 後輪タイヤ内の電流ゲージをトルクゲージに変更（指示による）。
 *   満スケールはバンビ指定の仮値ではなく、STM32 `LIMITS`（★v0.11）実測の
 *   `max_torque_nm`（1輪あたりの物理的なトルク上限、`LinkDiag` 経由）を使う。
 *   `LIMITS` 未受信の間は `MAX_BRAKE_TORQUE_NM`（`store/ui.ts`）にフォールバック
 *   する——`SpeedGauge.tsx` が `max_speed_m_s`/`PI_MAX_SPEED_CAP` で行っている
 *   のと同じパターン。加えて、TC（★v0.13 `tc_limit_nm`）が動的に絞る駆動トルク
 *   上限をバー内の横線マーカーで示す——非介入時は `max_torque_nm` と同値なので
 *   マーカーはバーの上端（満スケール位置）に張り付き、TC が効くとバーの途中まで
 *   下りてくる。電流（`motor_current`）はバーからは外したが、数値表示と
 *   フローラインのアニメーション速度には引き続き使う（電流そのものが不要に
 *   なったわけではない）。
 *
 * ## バー／ゲージの色の意味
 *
 * トルクゲージは単色（量だけを示す。安全上の判定は持たない）。満スケールは
 * STM32 実測の物理的なトルク上限（`max_torque_nm`）——駆動系統の過電流ラッチ
 * 5.0A（`docs/stm32_interface.md` §5.3、系統合計でモータ個別の閾値ではない）
 * とは別物。TC の動的トルク上限（`tc_limit_nm`）は同じバーの中に横線マーカー
 * （`--live` 系の色、`SpeedGauge` の指令速度マーカーと同じ考え方）として重ね、
 * バーの塗り（実際のトルク指令）がマーカーに迫っている＝ TC が効き始めている、
 * と読めるようにしてある。スリップゲージも単色方針——満スケール ±1.0m/s は
 * バンビ指定の仮の表示レンジ（TC のしきい値が未実装で根拠になる値が無いため）。
 * 電圧ゲージは `battLevel`（実在する放電終止電圧 8.0V 基準）で ok/warn/bad
 * に色分けする — こちらは走行を切り上げる判断に使う実際の閾値。
 * フローライン（駆動バッテリー→後輪）は電流の大小で流れる速さを変える単色
 * 方針（トルクゲージとは独立——電流自体を捨てたわけではない）。向きは常に
 * バッテリー→モータの一方向で固定（電流の符号は前進/後退の別であって回生では
 * ないため、逆流させない）。
 *
 * ## 前輪に電流・トルクが無い理由
 *
 * 前輪は駆動モータを持たない（操舵のみ）。指令トルク・電流は後輪2輪分しか
 * 存在しない。
 */
import { useNumbers } from '../../bus/live'
import { battLevel } from '../../format'
import { MAX_BRAKE_TORQUE_NM } from '../../store/ui'
import { BODY_H, BODY_W, BODY_X, BODY_Y, CX_L, CX_R, CY_FRONT, CY_REAR, WHEEL_H } from './carLayout'
import { ProximityRing } from './ProximityRing'

/** フローラインのアニメーション速度の換算に使う電流の満スケール [A]。
 * バンビ指定の表示レンジ（トルクゲージの満スケールとは無関係） */
const CURRENT_MAX = 10.0
/** スリップゲージの満スケール [m/s]。バンビ指定の仮の表示レンジ（実測待ち、`docs/architecture.md` §15） */
const SLIP_MAX = 1.0
/** 8セル NiMH。放電終止 8.0V / 満充電 11.2V（`uart_protocol.md` §5.3） */
const BATT_MIN = 8.0
const BATT_MAX = 11.2

/** フローラインの速度換算。これ未満は「ほぼ0」としてアニメーションを止める */
const FLOW_MIN_A = 0.15
/** 電流が小さいときの遅い流れ [s] */
const FLOW_DUR_MAX = 2.2
/** 満スケール時の速い流れ [s] */
const FLOW_DUR_MIN = 0.35

/** タイヤの厚み（トレッド方向）。`vehicle.toml` に値が無いため見た目重視の固定値。
 * 実測トレッド（0.155m）は車体幅（0.18m）より僅かに狭いだけなので、この厚みだと
 * タイヤの外側がわずかに車体からはみ出す——フェンダーからタイヤが覗く見た目になる */
const WHEEL_W = 17
/** 車体の切り欠き（車輪の周り）に足す余白 [svg単位] */
const NOTCH_MARGIN = 3

export function DrivePanel() {
  const n = useNumbers()
  const vs = n.vs
  const slipF = vs?.slip_front ?? null
  const slipR = vs?.slip_rear ?? null
  const cur = vs?.motor_current ?? null
  const trq = vs?.torque_cmd ?? null
  const tcLim = vs?.tc_limit_nm ?? null
  const bv = vs?.batt_voltage ?? null
  const ba = vs?.batt_current ?? null
  // ★2026-08-28: ゲージの満スケールは STM32 実測の物理上限（`LIMITS.max_torque_nm`）
  // に固定する（`SpeedGauge` が `max_speed_m_s` で行っているのと同じ理由——
  // バンビ指定の仮値だと実際に出せるトルクと表示が食い違う）。`LIMITS` 未受信の
  // 間は `MAX_BRAKE_TORQUE_NM`（STM32 側の物理上限と同値）にフォールバックする
  const torqueMax = n.link?.max_torque_nm ?? MAX_BRAKE_TORQUE_NM

  return (
    <div className="drivepanel-wrap">
      {/* viewBox は実際の描画内容（車体・タイヤ・近接リング）に合わせて
       * 四辺とも同じ余白（≈7.7単位）になるよう詰めてある。0〜200のフル幅の
       * ままだと、実寸で描いているコンテンツの左右に大きな空白ができ、
       * `.drivepanel-wrap` の max-width を上げても埋まらなかった
       * （実測して気づいた——SVG 側の余白が真因）。
       *
       * 2026-08-29: 横（x=34.67〜165.33、余白7.67ずつ）は揃っていたが、縦は
       * 近接リング最前部（y≈1.4）にほぼ余白が無いのに対し最後部（y≈232.6）
       * には27ぶんも余っていて、車体図の上下の余白が均一でないという指摘を
       * 受けた。`y0`/`height` を車体図の縦の中心に合わせて引き直し、横と
       * 同じ ≈7.67 の余白に揃えた（`ProximityRing.tsx` の `RING_GAP_M`
       * を変えたら要再計算——`raspi/auto/*` と違い手計算の定数なので、
       * 変えたらこのコメントの数値ごと更新すること）。 */}
      <svg viewBox="27 -6.26 146 246.52" className="drivepanel">
        <defs>
          <mask id="dp-body-mask">
            <rect x={BODY_X} y={BODY_Y} width={BODY_W} height={BODY_H} rx={16} fill="white" />
            {/* 車輪のある場所は車体を切り欠く（フェンダーからタイヤが覗く見た目にする） */}
            <WheelNotch cx={CX_L} cy={CY_FRONT} side="L" />
            <WheelNotch cx={CX_R} cy={CY_FRONT} side="R" />
            <WheelNotch cx={CX_L} cy={CY_REAR} side="L" />
            <WheelNotch cx={CX_R} cy={CY_REAR} side="R" />
          </mask>
        </defs>
        <rect
          className="dp-body"
          x={BODY_X}
          y={BODY_Y}
          width={BODY_W}
          height={BODY_H}
          rx={16}
          mask="url(#dp-body-mask)"
        />

        <FlowLine x1={100 - 7} y1={CY_REAR} x2={CX_L + WHEEL_W / 2} y2={CY_REAR} current={cur ? cur[0] : null} />
        <FlowLine x1={100 + 7} y1={CY_REAR} x2={CX_R - WHEEL_W / 2} y2={CY_REAR} current={cur ? cur[1] : null} />

        <Wheel cx={CX_L} cy={CY_FRONT} side="L" label="前左" slip={slipF ? slipF[0] : null} />
        <Wheel cx={CX_R} cy={CY_FRONT} side="R" label="前右" slip={slipF ? slipF[1] : null} />
        <Wheel
          cx={CX_L}
          cy={CY_REAR}
          side="L"
          label="後左"
          gauge
          current={cur ? cur[0] : null}
          torque={trq ? trq[0] : null}
          torqueMax={torqueMax}
          tcLimit={tcLim ? tcLim[0] : null}
          slip={slipR ? slipR[0] : null}
        />
        <Wheel
          cx={CX_R}
          cy={CY_REAR}
          side="R"
          label="後右"
          gauge
          current={cur ? cur[1] : null}
          torque={trq ? trq[1] : null}
          torqueMax={torqueMax}
          tcLimit={tcLim ? tcLim[1] : null}
          slip={slipR ? slipR[1] : null}
        />

        <BatteryGauge cx={100} cy={72} letter="S" fullLabel="信号" v={bv ? bv[1] : null} a={ba ? ba[1] : null} />
        <BatteryGauge cx={100} cy={160} letter="P" fullLabel="駆動" v={bv ? bv[0] : null} a={ba ? ba[0] : null} />

        <ProximityRing />
      </svg>
    </div>
  )
}

/** 電流の絶対値 → フローラインのアニメーション周期 [s]。大きいほど速く流れる */
function flowDuration(current: number | null): number {
  if (current == null) return FLOW_DUR_MAX
  const frac = Math.min(1, Math.abs(current) / CURRENT_MAX)
  return FLOW_DUR_MAX - (FLOW_DUR_MAX - FLOW_DUR_MIN) * frac
}

/**
 * 駆動バッテリー→後輪モータの電流フロー。破線が流れて見える＝電流が流れている
 * ことを示す（Tesla Energy Flow 風）。色は `.dp-wheel-fill` と同じ単色
 * （安全判定は持たせない）。電流の符号は前進/後退の別であって回生ブレーキでは
 * ない（この車体は回生を行わない）ため、符号に関わらず常にバッテリー→モータの
 * 向きにのみ流す（逆流させない）。速度だけ |電流| に応じて変える。
 */
function FlowLine({
  x1,
  y1,
  x2,
  y2,
  current,
}: {
  x1: number
  y1: number
  x2: number
  y2: number
  current: number | null
}) {
  const idle = current == null || Math.abs(current) < FLOW_MIN_A

  return (
    <g>
      <line className="dp-flow-bg" x1={x1} y1={y1} x2={x2} y2={y2} />
      <line
        className={`dp-flow${idle ? ' dp-flow--idle' : ''}`}
        x1={x1}
        y1={y1}
        x2={x2}
        y2={y2}
        style={{ animationDuration: `${flowDuration(current)}s` }}
      />
    </g>
  )
}

/**
 * 車体マスク（`dp-body-mask`）に開ける穴1つぶん。車体の側面（`side` の側）から
 * 車輪の内側の縁 + 余白まで切り欠く——実測トレッドが車体幅よりわずかに狭いだけ
 * なので、車輪の外側は車体の外にはみ出す（フェンダーからタイヤが覗く見た目）。
 */
function WheelNotch({ cx, cy, side }: { cx: number; cy: number; side: 'L' | 'R' }) {
  const inner = cx + (side === 'L' ? 1 : -1) * (WHEEL_W / 2 + NOTCH_MARGIN)
  const x = side === 'L' ? BODY_X : Math.min(inner, BODY_X + BODY_W)
  const width = side === 'L' ? Math.max(0, inner - BODY_X) : Math.max(0, BODY_X + BODY_W - inner)
  return (
    <rect
      x={x}
      y={cy - WHEEL_H / 2 - NOTCH_MARGIN}
      width={width}
      height={WHEEL_H + NOTCH_MARGIN * 2}
      fill="black"
    />
  )
}

/** 値・満スケール・ゲージ中心/高さから、塗り矩形の y・高さを出す（正=上、負=下） */
function gaugeFill(value: number | null, max: number, cy: number, height: number) {
  const frac = value == null ? 0 : Math.max(-1, Math.min(1, value / max))
  const fillH = Math.abs(frac) * (height / 2)
  const fillY = frac >= 0 ? cy - fillH : cy
  return { fillH, fillY }
}

/**
 * 車輪1つ。タイヤの矩形をそのままバーグラフの器として使い、車軸の高さ
 * （y=cy＝0）を境に、正なら上・負なら下へ塗る。
 *
 * タイヤはどの車輪も**縦2分割**——外側半分は常にスリップゲージ、車体側
 * 半分は後輪だけトルクゲージ（駆動なら上、制動なら下）を持つ。前輪は駆動
 * モータを持たない（トルクゲージが要らない）ので車体側半分は空のまま——
 * 4輪とも同じ「タイヤを割った」見た目に揃えるための空白で、情報を減らして
 * いるわけではない。後輪は数字も3つ（スリップ・電流・トルク）出すが、
 * タイヤが17svg単位しかなく横に並べると文字が重なるため、**スリップだけ
 * 車体側（上）、電流・トルクは車体端側（下）**に分けている——半分ずつの
 * ゲージの向きとは逆に、数字は反対側に逃がして重なりを避ける形。
 *
 * トルクゲージの車体側半分には、TC の動的トルク上限（`tcLimit`）を横線
 * マーカーとして重ねる——バーの塗り（`torque`）がマーカーに迫るほど
 * TC が効き始めていることが視覚的に分かる。非介入時は `tcLimit` が
 * `torqueMax`（物理上限）と同値になる仕様なので、マーカーは自然とバーの
 * 上端（満スケール位置）に張り付く。
 */
function Wheel({
  cx,
  cy,
  side,
  label,
  gauge = false,
  current = null,
  torque = null,
  torqueMax = null,
  tcLimit = null,
  slip = null,
}: {
  cx: number
  cy: number
  side: 'L' | 'R'
  /** 車輪の位置名（「前左」等）。ゲージの有無に関わらず常に渡す——`<title>` の
   * 識別に使うため（gauge を持たない前輪でも、どちらの車輪か分かるようにする） */
  label: string
  /** トルクゲージを持つか（＝後輪＝駆動モータがある側）。前輪は false */
  gauge?: boolean
  current?: number | null
  torque?: number | null
  /** トルクゲージの満スケール [N·m]（`LIMITS.max_torque_nm`）。`gauge` が true のときのみ使う */
  torqueMax?: number | null
  /** TC が動的に決める駆動トルク上限 [N·m]（`tc_limit_nm`）。バー内の横線マーカーに使う */
  tcLimit?: number | null
  slip?: number | null
}) {
  const x = cx - WHEEL_W / 2
  const y = cy - WHEEL_H / 2
  const clipId = `wheelclip-${cx}-${cy}`

  // 車体側（medial）＝ 0 に近い側。左輪は+x（右）が車体側、右輪は-x（左）が車体側
  const medialDir = side === 'L' ? 1 : -1
  const halfW = WHEEL_W / 2
  const medialX = medialDir > 0 ? cx : cx - halfW
  const lateralX = medialDir > 0 ? cx - halfW : cx

  const trqGauge = gauge ? gaugeFill(torque, torqueMax || 1, cy, WHEEL_H) : null
  const tcGauge = gauge && tcLimit != null ? gaugeFill(tcLimit, torqueMax || 1, cy, WHEEL_H) : null
  const slipGauge = gaugeFill(slip, SLIP_MAX, cy, WHEEL_H)

  const title = gauge
    ? `${label}モータ 指令トルク（上が駆動・下が制動）: ${torque == null ? '—' : `${torque.toFixed(2)}N·m`} / ` +
      `TC動的上限: ${tcLimit == null ? '—' : `${tcLimit.toFixed(2)}N·m`} / ` +
      `電流: ${current == null ? '—' : `${current.toFixed(1)}A`} / ` +
      `滑り（射影後の車輪速 − 車体速度）: ${slip == null ? '—' : `${slip.toFixed(1)}m/s`}`
    : `${label} 滑り（射影後の車輪速 − 車体速度）: ${slip == null ? '—' : `${slip.toFixed(1)}m/s`}`

  return (
    <g>
      <title>{title}</title>
      <defs>
        <clipPath id={clipId}>
          <rect x={x} y={y} width={WHEEL_W} height={WHEEL_H} rx={4} />
        </clipPath>
      </defs>

      <rect className="dp-wheel" x={x} y={y} width={WHEEL_W} height={WHEEL_H} rx={4} />

      <g clipPath={`url(#${clipId})`}>
        {trqGauge && (
          <rect className="dp-wheel-fill" x={medialX} y={trqGauge.fillY} width={halfW} height={trqGauge.fillH} />
        )}
        <rect className="dp-slip-fill" x={lateralX} y={slipGauge.fillY} width={halfW} height={slipGauge.fillH} />
        <rect className="dp-wheel-zero" x={x} y={cy - 0.75} width={WHEEL_W} height={1.5} />
        {tcGauge && (
          <line
            className="dp-tc-limit"
            x1={medialX}
            y1={tcGauge.fillY}
            x2={medialX + halfW}
            y2={tcGauge.fillY}
          />
        )}
      </g>

      {/* 車体側/外側の境界線。4輪とも同じ「タイヤを割った」見た目に揃える */}
      <line className="dp-wheel-divider" x1={cx} y1={y} x2={cx} y2={y + WHEEL_H} />

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

      {gauge ? (
        <>
          {/* 車体側へ（電流・トルクの逆）——下は電流・トルクの2行で埋まっているため */}
          <text className="dp-label" x={cx} y={cy - WHEEL_H / 2 - 6} textAnchor="middle">
            {slip == null ? '—' : slip.toFixed(1)}
          </text>
          <text className="dp-value" x={cx} y={cy + WHEEL_H / 2 + 11} textAnchor="middle">
            {current == null ? '—' : `${current.toFixed(1)}A`}
          </text>
          <text className="dp-label" x={cx} y={cy + WHEEL_H / 2 + 21} textAnchor="middle">
            {torque == null ? '—' : `${torque.toFixed(1)}N·m`}
          </text>
        </>
      ) : (
        <text className="dp-label" x={cx} y={cy + WHEEL_H / 2 + 12} textAnchor="middle">
          {slip == null ? '—' : slip.toFixed(1)}
        </text>
      )}
    </g>
  )
}

/**
 * 駆動・信号系統の電圧（乾電池型ゲージ、`battLevel` で色分け）と電流（数値のみ）。
 * ラベルはバッテリーイラストの中の1文字（`letter`）——`信号`/`駆動` という
 * フルネームはホバーの `<title>` に残す。
 */
function BatteryGauge({
  cx,
  cy,
  letter,
  fullLabel,
  v,
  a,
}: {
  cx: number
  cy: number
  letter: string
  fullLabel: string
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
      <title>{`${fullLabel}系: ${v == null ? '—' : `${v.toFixed(1)}V`} / ${a == null ? '—' : `${a.toFixed(1)}A`}`}</title>
      {/* S/P の1文字。バッテリーイラストの真上、白文字のみ（背景チップは満充電時に
       * 塗りと混ざって読みにくかったので廃止） */}
      <text className="dp-batt-letter" x={cx} y={cy - H / 2 - 8} textAnchor="middle">
        {letter}
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
        {v == null ? '—' : `${v.toFixed(1)}V`}
      </text>
      <text className="dp-label" x={cx} y={cy + H / 2 + 23} textAnchor="middle">
        {a == null ? '—' : `${a.toFixed(1)}A`}
      </text>
    </g>
  )
}
