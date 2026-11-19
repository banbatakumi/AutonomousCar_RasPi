import { describe, expect, it } from 'vitest'
import { DEFAULT_SETTINGS, MAX_TARGET_TORQUE_NM, type DrivingSettings } from '../store/settings'
import type { CmdOut } from '../types'
import {
  TxThrottle, approach, dz, expo, finishManual, gearContext, nextLight, stepLaunch, stepManual,
  type ManualInput, type ManualState,
} from './drivingLogic'

const S: DrivingSettings = { ...DEFAULT_SETTINGS, maxSpeed: 2, accel: 2, coast: 2.5, brake: 5, maxSteer: 0.4 }
const zero: ManualState = { speed: 0, steer: 0, torque: 0 }
const key = (p: Partial<ManualInput> = {}): ManualInput => ({
  source: 'keyboard', fwd: false, rev: false, left: false, right: false, accelTrig: 0, padSteer: 0, brake: false, ...p,
})
const pad = (p: Partial<ManualInput> = {}): ManualInput => key({ source: 'gamepad', ...p })
const speedMode = gearContext(S, 'D', 'N')

describe('小道具', () => {
  it('dz は切り落とさず再スケールする', () => {
    expect(dz(0.05, 0.1)).toBe(0)
    expect(dz(0.55, 0.1)).toBeCloseTo(0.5)
    expect(dz(-1, 0.1)).toBe(-1)
  })
  it('expo は端を変えない', () => {
    expect(expo(1, 0.45)).toBeCloseTo(1)
    expect(expo(0.5, 0)).toBe(0.5)
    expect(expo(0.5, 1)).toBe(0.125)
  })
  it('approach は行き過ぎない', () => {
    expect(approach(0, 1, 2, 0.1)).toBeCloseTo(0.2)
    expect(approach(0.95, 1, 2, 0.1)).toBe(1)
  })
  it('灯火は消灯→DAY→NORMAL→消灯', () => {
    expect([0, 1, 2].map((m) => nextLight([0, 1, 2], m))).toEqual([1, 2, 0])
  })
})

describe('gearContext', () => {
  it('MT の N は上限 0・減速は惰行のみ', () => {
    const g = gearContext({ ...S, driveMode: 'mt' }, 'D', 'N')
    expect(g.maxSpeed).toBe(0)
    expect(g.mtEngineBrakeRate).toBe(S.mtCoast)
  })
  it('MT は低いギアほどエンジンブレーキが強い', () => {
    const mt = { ...S, driveMode: 'mt' as const }
    const d1 = gearContext(mt, 'D', 'D1')
    const d5 = gearContext(mt, 'D', 'D5')
    expect(d1.mtEngineBrakeRate).toBeGreaterThan(d5.mtEngineBrakeRate)
    expect(d5.mtEngineBrakeRate).toBeCloseTo(S.mtCoast) // 5速は倍率 1.0 で加算なし
    expect(gearContext(mt, 'D', 'R').gearSign).toBe(-1)
  })
  it('速度・トルク制御は D/R レンジで向きを決める', () => {
    expect(gearContext(S, 'R', 'D3').gearSign).toBe(-1)
    expect(gearContext({ ...S, driveMode: 'torque' }, 'D', 'N').torqueModeOn).toBe(true)
  })
})

describe('stepManual（キーボード・速度）', () => {
  it('押している間は accel で加速し、離すと coast で惰行', () => {
    const a = stepManual(zero, key({ fwd: true }), S, speedMode, 0.1)
    expect(a.speed).toBeCloseTo(0.2)
    const b = stepManual({ ...a, speed: 1 }, key(), S, speedMode, 0.1)
    expect(b.speed).toBeCloseTo(0.75)
  })
  it('逆キーはブレーキで、0 を越えたら 0 で受け止める', () => {
    const a = stepManual({ ...zero, speed: 0.3 }, key({ rev: true }), S, speedMode, 0.1)
    expect(a.speed).toBe(0)
    const b = stepManual(a, key({ rev: true }), S, speedMode, 0.1)
    expect(b.speed).toBeCloseTo(-0.2) // 次のフレームから後退へ
  })
  it('舵は切り込みより戻し・切り返しが速い', () => {
    const inS = stepManual(zero, key({ left: true }), S, speedMode, 0.05).steer
    expect(inS).toBeCloseTo(S.steerRate * 0.05)
    const ret = stepManual({ ...zero, steer: 0.3 }, key(), S, speedMode, 0.05).steer
    expect(ret).toBeCloseTo(0.3 - S.steerReturn * 0.05)
    const counter = stepManual({ ...zero, steer: 0.3 }, key({ right: true }), S, speedMode, 0.05).steer
    expect(counter).toBeCloseTo(0.3 - S.steerCounter * 0.05)
  })
})

describe('stepManual（キーボード・MT／トルク）', () => {
  const mt = { ...S, driveMode: 'mt' as const, mtMaxSpeed: 2, mtAccel: 2 }
  it('ギアと逆のキーは何もしない（エンジンブレーキで減速）', () => {
    const g = gearContext(mt, 'D', 'D2')
    const a = stepManual({ ...zero, speed: 0.5 }, key({ rev: true }), mt, g, 0.1)
    expect(a.speed).toBeCloseTo(0.5 - g.mtEngineBrakeRate * 0.1)
  })
  it('シフトダウンで上限を超えた分はなだらかに落とす（ワープしない）', () => {
    const g = gearContext(mt, 'D', 'D1') // 上限 0.4
    const a = stepManual({ ...zero, speed: 1.5 }, key({ fwd: true }), mt, g, 0.05)
    expect(a.speed).toBeCloseTo(1.5 - g.mtEngineBrakeRate * 0.05)
    expect(finishManual(a, { phase: 'idle', target: 0, torque: 0 }, false, mt, g).speed).toBeCloseTo(a.speed)
  })
  it('トルクモードは押している間だけ driveTorque をそのまま出す', () => {
    const tq = { ...S, driveMode: 'torque' as const }
    const a = stepManual(zero, key({ rev: true }), tq, gearContext(tq, 'D', 'N'), 0.1)
    expect(a.torque).toBe(-tq.driveTorque)
    expect(a.speed).toBe(0)
  })
})

describe('stepManual（パッド）', () => {
  it('舵はランプを挟まずスティックの値（左が正）', () => {
    const a = stepManual(zero, pad({ padSteer: -1 }), S, speedMode, 0.016)
    expect(a.steer).toBeCloseTo(S.maxSteer)
  })
  it('速度はトリガーの目標へ accel で近づき、戻すと coast', () => {
    const a = stepManual(zero, pad({ accelTrig: 1 }), S, speedMode, 0.1)
    expect(a.speed).toBeCloseTo(0.2)
    const b = stepManual({ ...zero, speed: 1 }, pad({ accelTrig: 0 }), S, speedMode, 0.1)
    expect(b.speed).toBeCloseTo(0.75)
  })
})

describe('stepLaunch（ローンチコントロール）', () => {
  const idle = { phase: 'idle' as const, seen: false }
  it('止まってブレーキ＋アクセルで準備、ブレーキを離すと発進', () => {
    const r1 = stepLaunch(idle, key({ fwd: true, brake: true }), S, speedMode, 0, false)
    expect(r1.phase).toBe('ready')
    const r2 = stepLaunch(r1, key({ fwd: true }), S, speedMode, 0, false)
    expect(r2.phase).toBe('go')
    expect(r2.target).toBe(S.maxSpeed)
  })
  it('走っている間は準備に入らない', () => {
    expect(stepLaunch(idle, key({ fwd: true, brake: true }), S, speedMode, 0.5, false).phase).toBe('idle')
  })
  it('設定が OFF なら何もしない', () => {
    const off = { ...S, launchControl: false }
    expect(stepLaunch(idle, key({ fwd: true, brake: true }), off, gearContext(off, 'D', 'N'), 0, false).phase).toBe('idle')
  })
  it('パッドは 9 割以上で準備、発進後は 5 割を下回るまで続く', () => {
    expect(stepLaunch(idle, pad({ accelTrig: 0.8, brake: true }), S, speedMode, 0, false).phase).toBe('idle')
    const go = { phase: 'go' as const, seen: false }
    expect(stepLaunch(go, pad({ accelTrig: 0.6 }), S, speedMode, 0, false).phase).toBe('go')
    expect(stepLaunch(go, pad({ accelTrig: 0.4 }), S, speedMode, 0, false).phase).toBe('idle')
  })
  it('目標の手前に届く・ブレーキ・STM32 が発進を終えた、のどれかで終わる', () => {
    const go = { phase: 'go' as const, seen: false }
    expect(stepLaunch(go, key({ fwd: true }), S, speedMode, S.maxSpeed - 0.05, false).phase).toBe('idle')
    expect(stepLaunch(go, key({ fwd: true, brake: true }), S, speedMode, 0, false).phase).toBe('idle')
    const seen = stepLaunch(go, key({ fwd: true }), S, speedMode, 0.5, true)
    expect(seen).toMatchObject({ phase: 'go', seen: true })
    expect(stepLaunch(seen, key({ fwd: true }), S, speedMode, 0.6, false).phase).toBe('idle')
  })
  it('トルク制御の発進は最大トルク', () => {
    const tq = { ...S, driveMode: 'torque' as const }
    const r = stepLaunch({ phase: 'ready', seen: false }, key({ fwd: true }), tq, gearContext(tq, 'D', 'N'), 0, false)
    expect(r.phase).toBe('go')
    expect(r.torque).toBe(MAX_TARGET_TORQUE_NM)
  })
})

describe('finishManual', () => {
  it('ブレーキ中は速度・トルクを 0 に落とし、舵は生かす', () => {
    const r = finishManual({ speed: 1, steer: 0.2, torque: 0.1 }, { phase: 'idle', target: 0, torque: 0 }, true, S, speedMode)
    expect(r).toEqual({ speed: 0, steer: 0.2, torque: 0 })
  })
  it('発進中は目標を置き、上限で切る', () => {
    const r = finishManual(zero, { phase: 'go', target: 9, torque: 0 }, false, S, speedMode)
    expect(r.speed).toBe(S.maxSpeed)
  })
  it('トルクは普段 driveTorque、トルク制御の発進中だけ最大まで', () => {
    const tq = { ...S, driveMode: 'torque' as const, driveTorque: 0.05 }
    const g = gearContext(tq, 'D', 'N')
    expect(finishManual({ ...zero, torque: 0.1 }, { phase: 'idle', target: 0, torque: 0 }, false, tq, g).torque).toBe(0.05)
    expect(finishManual(zero, { phase: 'go', target: 0, torque: MAX_TARGET_TORQUE_NM }, false, tq, g).torque)
      .toBe(MAX_TARGET_TORQUE_NM)
  })
  it('舵は maxSteer で切る', () => {
    expect(finishManual({ ...zero, steer: -2 }, { phase: 'idle', target: 0, torque: 0 }, false, S, speedMode).steer).toBe(-S.maxSteer)
  })
})

describe('TxThrottle', () => {
  const c = (speed: number) => ({ speed }) as unknown as Omit<CmdOut, 'type'>
  it('中身が変われば最短間隔で、同じ中身は繰り返しの間隔で送る', () => {
    const tx = new TxThrottle(7, 20)
    expect(tx.due(100, c(0))).toBe(true)
    expect(tx.due(110, c(0))).toBe(false) // 同じ中身・20ms 未満
    expect(tx.due(110, c(1))).toBe(true) // 変わった・7ms 以上
    expect(tx.due(115, c(2))).toBe(false) // 変わったが 7ms 未満
    expect(tx.due(130, c(1))).toBe(true)
    expect(tx.due(150, c(1))).toBe(true) // 同じ中身・20ms 経過
  })
})
