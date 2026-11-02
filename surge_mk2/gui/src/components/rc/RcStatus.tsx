/**
 * 電池と温度の小パネル — ラジコンビューの映像の左上（2026-11-03、指示による）。
 *
 * このタブの方針「温度と電圧は常時出す。走行を切り上げる判断に使う」（`RcView.tsx` 冒頭）の
 * 置き場所。以前は右端から引き出す車体図（`RcBar`/`DrivePanel`/`TempPanel`）にあったが、
 * 普段しまっていると見えないので、ここに要る物だけを常時出し、車体図は廃止した。
 *
 * - **電池**: 駆動系（P）と信号系（S）の2本。10 個のセグメントで残量、右に電圧。
 *   残量は 8セル NiMH の放電終止〜満充電（`DrivePanel.tsx` の乾電池ゲージと同じ範囲）
 * - **温度**: 後輪の左右とステアのモータ・STM32（`vs.temp[0..3]`）と RasPi の CPU（`piTempC`。
 *   STM32 のバスに乗らないので `telemetry_node` が別枠で配る）。RasPi の値は実機以外
 *   （シム・`bus_demo`）では常に null なので、**RasPi だけは null を「取得不可」として沈める**
 *   （モータ・STM32 の null は MD/STM32 が無言＝異常なので赤）
 *
 * 色分けのしきい値は `format.ts` の `battLevel`/`tempLevel` だけを使う（判定を2か所に書かない）。
 * 正常は白、注意は黄、危険は赤で発光させる。受信が途絶したら全体を沈める
 * （古い値を現在値に見せない。`.drivebar` と同じ）。
 */
import { useNumbers } from '../../bus/live'
import { battLevel, tempLevel, type Level } from '../../format'

/** 残量 0〜100% に対応させる電圧 [V]（8セル NiMH。`DrivePanel.tsx` と同じ） */
const BATT_MIN = 8.0
const BATT_MAX = 11.2
const BATT_SEGS = 10
/** 温度バーの満スケール [℃]。しきい値そのものではなく表示レンジ */
const TEMP_MAX = 90
const TEMP_SEGS = 10

export function RcStatus() {
  const n = useNumbers()
  const vs = n.vs
  const bv = vs?.batt_voltage ?? null
  const t = vs?.temp ?? null
  const pi = n.piTempC

  return (
    <div className={`rc-status ${n.stale ? 'stale' : ''}`}>
      <div className="rcs-head">BATT</div>
      <BattRow label="P" title="駆動系" v={bv ? bv[0] : null} />
      <BattRow label="S" title="信号系" v={bv ? bv[1] : null} />
      <div className="rcs-head">TEMP</div>
      <TempRow label="RL" title="後輪 左" v={t ? t[0] : null} />
      <TempRow label="RR" title="後輪 右" v={t ? t[1] : null} />
      <TempRow label="ST" title="ステア" v={t ? t[2] : null} />
      <TempRow label="MC" title="STM32" v={t ? t[3] : null} />
      <TempRow label="PI" title="RasPi" v={pi} naIfNull />
    </div>
  )
}

function Segs({ frac, n, lv }: { frac: number; n: number; lv: Level | 'na' }) {
  const lit = Math.round(frac * n)
  return (
    <span className="rcs-segs">
      {Array.from({ length: n }, (_, i) => (
        <i key={i} className={i < lit ? `on lv-${lv}` : ''} />
      ))}
    </span>
  )
}

function BattRow({ label, title, v }: { label: string; title: string; v: number | null }) {
  const lv = v == null ? 'na' : battLevel(v)
  // 放電終止未満でも最低1つは点ける（**赤なのに何も見えない**のを避ける。`DrivePanel` と同じ）
  const frac = v == null ? 0 : Math.max(1 / BATT_SEGS, Math.min(1, (v - BATT_MIN) / (BATT_MAX - BATT_MIN)))
  return (
    <div className={`rcs-row rcs-${lv}`} title={`${title}: ${v == null ? '—' : `${v.toFixed(2)}V`}`}>
      <span className="rcs-label">{label}</span>
      <Segs frac={frac} n={BATT_SEGS} lv={lv} />
      <b>{v == null ? '—' : v.toFixed(1)}</b>
      <i>V</i>
    </div>
  )
}

function TempRow({ label, title, v, naIfNull = false }: {
  label: string
  title: string
  v: number | null
  /** null を異常ではなく「取得不可」として扱う（RasPi。冒頭コメント参照） */
  naIfNull?: boolean
}) {
  // null は「MD が無言＝値を信用できない」。`tempLevel(null)` は bad を返す（以前の温度計と同じ扱い）
  const lv = v == null && naIfNull ? 'na' : tempLevel(v)
  const frac = v == null ? 0 : Math.max(0, Math.min(1, v / TEMP_MAX))
  return (
    <div className={`rcs-row rcs-${lv}`} title={`${title}: ${v == null ? '—' : `${v.toFixed(0)}℃`}`}>
      <span className="rcs-label">{label}</span>
      <Segs frac={frac} n={TEMP_SEGS} lv={lv} />
      <b>{v == null ? '—' : v.toFixed(0)}</b>
      <i>℃</i>
    </div>
  )
}
