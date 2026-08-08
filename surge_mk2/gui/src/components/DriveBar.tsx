/**
 * 速度・舵角の帯 — 層 B。
 *
 * ## 実測と指令を必ず並べる
 *
 * 舵角は `steer_actual`（実測）と `steer_cmd_echo`（STM32 が受理した指令）を
 * 重ねて出す。**この差がアクチュエータの遅れそのもの**で、
 * Phase 1 で測る「ステアリングの1次遅れ + むだ時間」が走らせながら見える
 * （`architecture.md` §14 Phase 1）。
 *
 * 速度は「自分が送っている指令」と「実測」を並べる。
 */
import { useNumbers } from '../bus/live'
import { UI_MAX_SPEED, UI_MAX_STEER } from '../store/ui'
import { deg, kmh, metres } from '../format'

export function DriveBar() {
  const n = useNumbers()
  const vs = n.vs
  const speed = vs ? (vs.stopped ? 0 : vs.speed) : 0
  const target = n.out.active ? n.out.speed : 0

  return (
    <div className={`drivebar ${n.stale ? 'stale' : ''}`}>
      <div className="gauge">
        <div className="gauge-head">
          <span className="label">速度</span>
          <span className="big">{speed.toFixed(2)}</span>
          <span className="unit">m/s</span>
          <span className="sub">{kmh(speed)} km/h</span>
          <span className="sub">
            指令 {n.out.active ? `${target.toFixed(2)}` : '—'}
          </span>
        </div>
        <BiBar value={speed} target={n.out.active ? target : null} max={UI_MAX_SPEED} />
        {vs?.stopped && <span className="note">停止判定（デッドバンド内。生値 {vs.speed.toFixed(3)}）</span>}
      </div>

      <div className="gauge">
        <div className="gauge-head">
          <span className="label">舵角</span>
          <span className="big">{vs ? deg(vs.steer_actual) : '—'}</span>
          <span className="sub">指令 {vs ? deg(vs.steer_cmd_echo) : '—'}</span>
          <span className="sub dim">← 差がステアリングの遅れ</span>
        </div>
        <BiBar
          value={vs?.steer_actual ?? 0}
          target={vs?.steer_cmd_echo ?? null}
          max={UI_MAX_STEER}
          flip
        />
      </div>

      <div className="gauge narrow">
        <div className="gauge-head">
          <span className="label">超音波</span>
        </div>
        <div className="kv">
          <span>前</span>
          <b className={near(vs?.us_front)}>{metres(vs?.us_front ?? null)}</b>
          <span>後</span>
          <b className={near(vs?.us_rear)}>{metres(vs?.us_rear ?? null)}</b>
        </div>
        <span className="note">実質 20Hz・同じ値が続く</span>
      </div>

      <div className="gauge narrow">
        <div className="gauge-head">
          <span className="label">走行距離</span>
        </div>
        <div className="kv">
          <span>中心線</span>
          <b>{vs ? `${vs.odom_center.toFixed(2)}m` : '—'}</b>
        </div>
        <span className="note">前輪を舵角で射影して積算</span>
      </div>
    </div>
  )
}

function near(d: number | null | undefined): string {
  if (d == null) return 'dim'
  if (d < 0.25) return 'lv-bad'
  if (d < 0.5) return 'lv-warn'
  return ''
}

/** 中央 0 の両振れバー。実測を塗り、指令を線で重ねる。 */
function BiBar({
  value,
  target,
  max,
  flip = false,
}: {
  value: number
  target: number | null
  max: number
  flip?: boolean
}) {
  const clamp = (v: number) => Math.max(-1, Math.min(1, v / max))
  // 舵角は「左が正」だが、バーは左右の見た目を合わせたいので反転させる
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
