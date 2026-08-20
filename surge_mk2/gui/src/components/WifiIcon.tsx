/**
 * Wi-Fi電波強度アイコン — iPhoneのアンテナ表示と同じ考え方で、弱いほど
 * 内側からアーチが減っていく。3本とも点けば強、無点灯は未接続/取得不可。
 *
 * 色は `wifiLevel`（`DiagGrid.tsx` の数字表示と共通のしきい値）、消灯側は
 * `var(--dim)` — `lv-bg-*`（`DrivePanel.tsx` の電池ゲージ）は塗り潰し用途の
 * ほぼ黒に近い色なので、輪郭線だけのこのアイコンでは見えなくなってしまう。
 */
import { wifiBars, wifiLevel } from '../format'

export function WifiIcon({ dbm, size = 15 }: { dbm: number | null; size?: number }) {
  const bars = wifiBars(dbm)
  const level = wifiLevel(dbm)
  const arcClass = (need: number) => (bars >= need ? `lv-stroke-${level}` : 'lv-stroke-dim')

  return (
    <svg width={size} height={size} viewBox="0 0 24 24" fill="none" className="wifi-icon">
      <path
        className={arcClass(3)}
        d="M1.42 9a16 16 0 0 1 21.16 0"
        strokeWidth={2.4}
        strokeLinecap="round"
      />
      <path
        className={arcClass(2)}
        d="M5 12.55a11 11 0 0 1 14.08 0"
        strokeWidth={2.4}
        strokeLinecap="round"
      />
      <path
        className={arcClass(1)}
        d="M8.53 16.11a6 6 0 0 1 6.95 0"
        strokeWidth={2.4}
        strokeLinecap="round"
      />
      <circle cx={12} cy={20} r={1.4} className={bars >= 1 ? `lv-bg-${level}` : 'lv-bg-dim2'} />
    </svg>
  )
}
