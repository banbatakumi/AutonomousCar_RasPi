/**
 * 速度・舵角の帯 — 上段（前後カメラ・LiDAR）のすぐ下に置く、細い HUD ライン。
 *
 * ## 実測と指令を必ず並べる
 *
 * 舵角は `steer_actual`（実測）と `steer_cmd_echo`（STM32 が受理した指令）を
 * 重ねて出す。**この差がアクチュエータの遅れそのもの**で、
 * Phase 1 で測る「ステアリングの1次遅れ + むだ時間」が走らせながら見える
 * （`architecture.md` §14 Phase 1）。
 *
 * 速度は「自分が送っている指令」と「実測」を並べる。
 *
 * ## 超音波・走行距離は消した（2026-08-28）
 *
 * どちらも正常時に読む数字ではなく、超音波は自動停止が作動した時点で
 * `StatusBar`/`AssistLamps` 側の異常表示が拾う。走行距離は診断タブで見れば足りる。
 * 速度・舵角だけに絞り、**上のカメラ帯の下にぴったり収まる細い1本のバー**にした
 * （指示による——太い計器然としたパネルではなく「かっこいい HUD の線」を狙う）。
 */
import { useNumbers } from '../bus/live'
import { useUi } from '../store/ui'
import { deg, kmh } from '../format'
import { BiBar } from './BiBar'

export function DriveBar() {
  const n = useNumbers()
  const { maxSpeed, maxSteer } = useUi((s) => s.settings)
  const vs = n.vs
  const speed = vs ? (vs.stopped ? 0 : vs.speed) : 0
  const torqueMode = n.out.active && n.out.torqueMode
  const target = n.out.active ? n.out.speed : 0

  return (
    <div className={`drivebar ${n.stale ? 'stale' : ''}`}>
      <div className="gauge">
        <div className="gauge-head">
          <span className="label">速度</span>
          <span className="big">{speed.toFixed(2)}</span>
          <span className="unit">m/s</span>
          <span className="sub">{kmh(speed)} km/h</span>
          {torqueMode ? (
            <span className="sub">指令(トルク) {n.out.torque.toFixed(3)} N·m</span>
          ) : (
            <span className="sub">
              {/* 自律走行中の「指令」は planner が出した値（`bus/live.ts` の `cmdOut`）。
                  誰が出しているのか分からないまま数字だけ動くのを避ける */}
              {n.out.auto ? '指令(自律)' : '指令'} {n.out.active ? `${target.toFixed(2)}` : '—'}
            </span>
          )}
        </div>
        {/* トルクモード中は target_speed を送らない（0 固定）ので、速度の目標線は表示しない */}
        <BiBar value={speed} target={n.out.active && !torqueMode ? target : null} max={maxSpeed} />
        {vs?.stopped && <span className="note">停止判定（デッドバンド内。生値 {vs.speed.toFixed(3)}）</span>}
      </div>

      <div className="gauge">
        <div className="gauge-head">
          <span className="label">舵角</span>
          <span className="big">{vs ? deg(vs.steer_actual) : '—'}</span>
          <span className="sub">指令 {vs ? deg(vs.steer_cmd_echo) : '—'}</span>
        </div>
        <BiBar
          value={vs?.steer_actual ?? 0}
          target={vs?.steer_cmd_echo ?? null}
          max={maxSteer}
          flip
        />
      </div>
    </div>
  )
}
