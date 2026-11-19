import { afterEach, describe, expect, it } from 'vitest'
import type { LinkDiag, VehicleState } from '../types'
import { live } from './live'
import { virtualGear } from './virtualGear'
import { useUi } from '../store/ui'
import { DEFAULT_SETTINGS } from '../store/settings'

const drive = (speed: number) => {
  live.vs = { speed, stopped: Math.abs(speed) < 0.01 } as VehicleState
  return virtualGear()
}

afterEach(() => {
  live.vs = null
  live.link = null
  useUi.setState({ settings: DEFAULT_SETTINGS, gear: 'D', mtGear: 'N' })
})

describe('virtualGear（仮想 AT）', () => {
  it('加速で上がり、減速では下げの基準まで粘る（ヒステリシス）', () => {
    live.link = { max_speed_m_s: 3 } as LinkDiag
    expect(drive(0).label).toBe('1')
    // 1速の上限 0.54 の 92% ≈ 0.497 で 2速へ
    expect(drive(0.5).label).toBe('2')
    // 下げるのは 1速の上限の 70% ≈ 0.378 を下回ったとき
    expect(drive(0.4).label).toBe('2')
    expect(drive(0.37).label).toBe('1')
  })
  it('同じ入力で何度呼んでも結果が変わらない', () => {
    live.link = { max_speed_m_s: 3 } as LinkDiag
    drive(0)
    const a = drive(2.5)
    const b = virtualGear()
    expect(b.label).toBe(a.label)
    expect(b.shiftCount).toBe(a.shiftCount)
  })
  it('変速のたびに shiftCount が増える', () => {
    live.link = { max_speed_m_s: 3 } as LinkDiag
    const c0 = drive(0).shiftCount
    expect(drive(0.5).shiftCount).toBe(c0 + 1)
  })
  it('R レンジは R・上げない', () => {
    useUi.setState({ gear: 'R' })
    const g = drive(-0.2)
    expect(g.label).toBe('R')
    expect(g.noUpshift).toBe(true)
  })
})

describe('virtualGear（MT）', () => {
  it('選んだギアと、その上限に対する回転数', () => {
    useUi.setState({ settings: { ...DEFAULT_SETTINGS, driveMode: 'mt', mtMaxSpeed: 2 }, mtGear: 'D2' })
    const g = drive(0.4) // D2 の上限 = 2 × 0.4 = 0.8
    expect(g.label).toBe('2')
    expect(g.rpmFrac).toBeCloseTo(0.5)
    expect(g.noUpshift).toBe(false)
  })
  it('N は回転数 0・上げない', () => {
    useUi.setState({ settings: { ...DEFAULT_SETTINGS, driveMode: 'mt' }, mtGear: 'N' })
    const g = drive(0.4)
    expect(g.rpmFrac).toBe(0)
    expect(g.noUpshift).toBe(true)
  })
})
