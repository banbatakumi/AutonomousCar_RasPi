import { describe, expect, it } from 'vitest'
import { GamepadReader } from './gamepad'

const btn = (value = 0): GamepadButton => ({ pressed: value > 0.5, touched: false, value })
const mkPad = (p: { index?: number; ts: number; r2?: number; mapping?: GamepadMappingType; b0?: number }): Gamepad =>
  ({
    index: p.index ?? 0,
    connected: true,
    mapping: p.mapping ?? 'standard',
    timestamp: p.ts,
    axes: [0, 0, 0, 0],
    buttons: Array.from({ length: 16 }, (_, i) => btn(i === 7 ? (p.r2 ?? 0) : i === 0 ? (p.b0 ?? 0) : 0)),
  }) as unknown as Gamepad

describe('GamepadReader', () => {
  it('標準マッピングのパッドを優先する', () => {
    const r = new GamepadReader()
    const other = mkPad({ index: 0, ts: 1, mapping: '' })
    const std = mkPad({ index: 1, ts: 1 })
    expect(r.read([other, std], 0, 0.1).pad).toBe(std)
  })

  it('timestamp が心拍でない機種では、握り続けても圏外にしない', () => {
    const r = new GamepadReader()
    const p = mkPad({ ts: 5, r2: 1 })
    for (let t = 0; t <= 5000; t += 100) expect(r.read([p], t, 0.1).pad).toBe(p)
  })

  it('心拍の機種で timestamp が止まり入力が残っていたら、中立にして1回だけ知らせる', () => {
    const r = new GamepadReader()
    // 入力が同じのまま timestamp が3回進む → 心拍と判定
    for (let i = 1; i <= 4; i++) r.read([mkPad({ ts: i, r2: 1 })], i * 10, 0.1)
    const stuck = mkPad({ ts: 4, r2: 1 })
    expect(r.read([stuck], 1000, 0.1).pad).toBe(stuck)
    const a = r.read([stuck], 2100, 0.1)
    expect(a).toEqual({ pad: null, staleStarted: true })
    expect(r.read([stuck], 2200, 0.1)).toEqual({ pad: null, staleStarted: false })
    // 入力が中立なら圏外とはみなさない
    expect(r.read([mkPad({ ts: 4, r2: 0 })], 2300, 0.1).pad).not.toBeNull()
  })

  it('rose は押した瞬間だけ true（パッドが消えたら離した扱い）', () => {
    const r = new GamepadReader()
    const down = mkPad({ ts: 1, b0: 1 })
    expect(r.rose(down, 0)).toBe(true)
    expect(r.rose(down, 0)).toBe(false)
    expect(r.rose(null, 0)).toBe(false)
    expect(r.rose(down, 0)).toBe(true)
  })

  it('使っていたパッドの切断だけ知らせる', () => {
    const r = new GamepadReader()
    r.read([mkPad({ index: 2, ts: 1 })], 0, 0.1)
    expect(r.onDisconnected(0)).toBe(false)
    r.read([mkPad({ index: 2, ts: 1 })], 0, 0.1)
    expect(r.onDisconnected(2)).toBe(true)
  })
})
