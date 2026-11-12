/**
 * 温度の列 — ラジコンの計器パネルの液晶の右の列（2026-11-13）。
 *
 * このタブの方針「温度と電圧は常時出す。走行を切り上げる判断に使う」（`RcView.tsx` 冒頭）の温度の置き場所。
 * 電圧は同じパネルの車体図（`DrivePanel.tsx` の S/P）に出ているので、ここは温度だけ。
 * 以前は映像の左上の箱（電池と温度）にあったが、ステレオ化でパネルを左下へ移したときにその箱は消した。
 *
 * - 後輪の左右とステアのモータ・STM32（`vs.temp[0..3]`）と RasPi の CPU（`piTempC`。
 *   STM32 のバスに乗らないので `telemetry_node` が別枠で配る）を縦に5行並べる
 * - RasPi の値は実機以外（シム・`bus_demo`）では常に null なので、**RasPi だけは null を「取得不可」として沈める**
 *   （モータ・STM32 の null は MD/STM32 が無言＝異常なので赤）
 *
 * 色分けのしきい値は `format.ts` の `tempLevel` だけを使う（判定を2か所に書かない）。
 * 正常は白、注意は黄、危険は赤で発光させる。受信が途絶したら全体を沈める
 * （古い値を現在値に見せない。`.drivebar` と同じ）。
 */
import { useNumbers } from '../../bus/live'
import { tempLevel, type Level } from '../../format'

/** 温度バーの満スケール [℃]。しきい値そのものではなく表示レンジ */
const TEMP_MAX = 90
const TEMP_SEGS = 6

export function RcTemps() {
  const n = useNumbers()
  const t = n.vs?.temp ?? null

  return (
    <div className={`rc-temps ${n.stale ? 'stale' : ''}`}>
      {/* 単位は先頭に1つだけ（セルごとに付けると狭い画面で隣のラベルにかかる） */}
      <span className="rct-head">
        TEMP
        <br />℃
      </span>
      <TempCell label="RL" title="後輪 左" v={t ? t[0] : null} />
      <TempCell label="RR" title="後輪 右" v={t ? t[1] : null} />
      <TempCell label="ST" title="ステア" v={t ? t[2] : null} />
      <TempCell label="MC" title="STM32" v={t ? t[3] : null} />
      <TempCell label="PI" title="RasPi" v={n.piTempC} naIfNull />
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

function TempCell({ label, title, v, naIfNull = false }: {
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
    <div className={`rct-cell rcs-${lv}`} title={`${title}: ${v == null ? '—' : `${v.toFixed(0)}℃`}`}>
      <span className="rcs-label">{label}</span>
      <Segs frac={frac} n={TEMP_SEGS} lv={lv} />
      <b>{v == null ? '—' : v.toFixed(0)}</b>
    </div>
  )
}
