import { describe, expect, it } from 'vitest'
import { MT_GEARS, mtGearAt, mtGearIndex, mtGearRatio, reverseSelected, shiftGearUpdate } from './gear'
import { DEFAULT_SETTINGS, type DrivingSettings } from './settings'

const mt: DrivingSettings = { ...DEFAULT_SETTINGS, driveMode: 'mt' }

describe('mtGearRatio', () => {
  it('N は 0、R は D1 と同じ、D1〜D5 は設定の倍率', () => {
    expect(mtGearRatio('N', mt)).toBe(0)
    expect(mtGearRatio('R', mt)).toBe(mt.mtGear1)
    expect(MT_GEARS.slice(2).map((g) => mtGearRatio(g, mt))).toEqual([
      mt.mtGear1, mt.mtGear2, mt.mtGear3, mt.mtGear4, mt.mtGear5,
    ])
  })
})

describe('mtGearAt / mtGearIndex', () => {
  it('範囲外は端で止める', () => {
    expect(mtGearAt(-3)).toBe('R')
    expect(mtGearAt(99)).toBe('D5')
    expect(mtGearAt(mtGearIndex('D2') + 1)).toBe('D3')
  })
})

describe('reverseSelected', () => {
  it('MT は mtGear、それ以外は D/R レンジで決まる', () => {
    expect(reverseSelected({ settings: mt, gear: 'R', mtGear: 'N' })).toBe(false)
    expect(reverseSelected({ settings: mt, gear: 'D', mtGear: 'R' })).toBe(true)
    expect(reverseSelected({ settings: DEFAULT_SETTINGS, gear: 'R', mtGear: 'D1' })).toBe(true)
  })
})

describe('shiftGearUpdate', () => {
  const speedMode = { settings: DEFAULT_SETTINGS, mtGear: 'N' as const }
  it('D/R は停止中だけ切り替わる', () => {
    expect(shiftGearUpdate({ ...speedMode, gear: 'D' }, -1, true)).toEqual({ gear: 'R' })
    expect(shiftGearUpdate({ ...speedMode, gear: 'D' }, -1, false)).toBeNull()
    expect(shiftGearUpdate({ ...speedMode, gear: 'D' }, 1, true)).toBeNull() // 既に D
  })
  it('MT は1段ずつ。N〜D5 は走行中も動かせる', () => {
    expect(shiftGearUpdate({ settings: mt, gear: 'D', mtGear: 'D2' }, 1, false)).toEqual({ mtGear: 'D3' })
    expect(shiftGearUpdate({ settings: mt, gear: 'D', mtGear: 'D1' }, -1, false)).toEqual({ mtGear: 'N' })
  })
  it('MT で R をまたぐのは停止中だけ', () => {
    expect(shiftGearUpdate({ settings: mt, gear: 'D', mtGear: 'N' }, -1, false)).toBeNull()
    expect(shiftGearUpdate({ settings: mt, gear: 'D', mtGear: 'N' }, -1, true)).toEqual({ mtGear: 'R' })
    expect(shiftGearUpdate({ settings: mt, gear: 'D', mtGear: 'R' }, 1, false)).toBeNull()
  })
  it('端ではそれ以上動かない', () => {
    expect(shiftGearUpdate({ settings: mt, gear: 'D', mtGear: 'D5' }, 1, true)).toBeNull()
    expect(shiftGearUpdate({ settings: mt, gear: 'D', mtGear: 'R' }, -1, true)).toBeNull()
  })
})
