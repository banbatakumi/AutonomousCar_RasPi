import { describe, expect, it } from 'vitest'
import { MSGS_SCHEMA } from '../generated/msgs'
import { isControlStatus, isSnapshot, parseServerMsg, schemaOf } from './messages'

describe('isSnapshot', () => {
  it('札が一致するときだけ通す', () => {
    expect(isSnapshot({ schema: MSGS_SCHEMA, vs: null })).toBe(true)
    expect(isSnapshot({ schema: MSGS_SCHEMA + 1 })).toBe(false)
    expect(isSnapshot(null)).toBe(false)
    expect(isSnapshot([MSGS_SCHEMA])).toBe(false)
  })
  it('食い違いの通知には札の値（無ければ 0）', () => {
    expect(schemaOf({ schema: 7 })).toBe(7)
    expect(schemaOf('x')).toBe(0)
  })
})

describe('parseServerMsg', () => {
  it('壊れた JSON・文字列以外・オブジェクト以外は null', () => {
    expect(parseServerMsg('{')).toBeNull()
    expect(parseServerMsg(new ArrayBuffer(2))).toBeNull()
    expect(parseServerMsg('[1,2]')).toBeNull()
    expect(parseServerMsg('{"type":3}')).toBeNull()
  })
  it('振り分けの札を読める', () => {
    expect(parseServerMsg('{"type":"pong","id":4}')).toEqual({ type: 'pong', id: 4 })
  })
})

describe('isControlStatus', () => {
  const ok = { type: 'status', has_controller: false, controller: '', arm_inhibited: true, estop_active: false }
  it('必須の札が揃った status だけ通す', () => {
    expect(isControlStatus(ok)).toBe(true)
    const { arm_inhibited: _, ...missing } = ok
    expect(isControlStatus(missing)).toBe(false)
    expect(isControlStatus({ ...ok, type: 'logs' })).toBe(false)
  })
})
