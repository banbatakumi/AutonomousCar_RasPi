/**
 * 温度計 — 車体図（`DrivePanel`）の下に常時出す（2026-08-17、指示による追加）。
 *
 * `RcView.tsx` 冒頭の方針どおり、温度は「正常でも隠さない」枠——実車の水温計と
 * 同じで、走行を切り上げる判断に使う数字だから。しきい値は `tempLevel()`
 * （`format.ts`）に一本化してあり、5つとも同じ基準で ok/warn/bad に色分けする。
 *
 * ## 5つの内訳
 *
 * 後輪モータ2つ・ステアモータ・STM32 MCU は `vs.temp`（UART テレメトリそのもの）。
 * RasPi 本体の CPU 温度だけ経路が違い、`telemetry_node._read_pi_temp_c()` が
 * `/sys/class/thermal/thermal_zone0/temp` を読んで `pi_temp_c` として別枠で
 * 配信している（STM32 のバスには乗らないため）。実機以外（Mac上のシム等）では
 * 常に null になる——これも「値が信用できない」として `tempLevel(null)` で bad 扱い。
 *
 * ## `.bar-row`/`.minibar` は既存 CSS の再利用
 *
 * 以前あった水温計・燃料計（`TempBar`/`BattBar`）を `DrivePanel` に統合した際に
 * 表示だけ無くした CSS がそのまま残っていた（`styles.css` 参照）。同じ用途なので
 * そのまま使う。
 */
import { useNumbers } from '../../bus/live'
import { tempLevel } from '../../format'

/** ミニバーの満スケール [℃]。しきい値そのものではなく表示レンジ */
const TEMP_MAX = 90

export function TempPanel() {
  const n = useNumbers()
  const t = n.vs?.temp ?? null

  return (
    <div className="rc-temps">
      <TempRow label="後左" value={t ? t[0] : null} />
      <TempRow label="後右" value={t ? t[1] : null} />
      <TempRow label="ステア" value={t ? t[2] : null} />
      <TempRow label="STM32" value={t ? t[3] : null} />
      <TempRow label="RasPi" value={n.piTempC} />
    </div>
  )
}

function TempRow({ label, value }: { label: string; value: number | null }) {
  const lv = tempLevel(value)
  const frac = value == null ? 0 : Math.max(0, Math.min(1, value / TEMP_MAX))
  return (
    <div className="bar-row">
      <span className="label">{label}</span>
      <div className="minibar">
        <div className={`minibar-fill lv-bg-${lv}`} style={{ width: `${frac * 100}%` }} />
      </div>
      <b className={lv === 'ok' ? '' : `lv-${lv}`}>{value == null ? '—' : `${value.toFixed(0)}℃`}</b>
    </div>
  )
}
