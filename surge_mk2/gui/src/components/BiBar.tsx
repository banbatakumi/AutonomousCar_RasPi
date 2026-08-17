/** 中央 0 の両振れバー。実測を塗り、指令（あれば）を線で重ねる。 */
export function BiBar({
  value,
  target = null,
  max,
  flip = false,
}: {
  value: number
  target?: number | null
  max: number
  flip?: boolean
}) {
  const clamp = (v: number) => Math.max(-1, Math.min(1, v / max))
  const v = clamp(value) * (flip ? -1 : 1)
  const t = target == null ? null : clamp(target) * (flip ? -1 : 1)
  return (
    <div className="bibar">
      <div className="bibar-zero" />
      <div
        className="bibar-fill"
        style={{
          left: v >= 0 ? '50%' : `${50 + v * 50}%`,
          width: `${Math.abs(v) * 50}%`,
        }}
      />
      {t != null && <div className="bibar-target" style={{ left: `${50 + t * 50}%` }} />}
    </div>
  )
}
