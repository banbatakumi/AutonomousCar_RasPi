import { beforeEach, describe, expect, it } from 'vitest'
import { AVOID, clearTarget, editClick, editDrag, endEdit, parseCfg, routeEdit, setTarget, undoLast, emptyCfg } from './routeEdit'

/** 1m = 100px の拡大率。削除判定（画面で12px以内）が 0.12m になる */
const PX = 100

beforeEach(() => {
  endEdit()
  routeEdit.cfg = emptyCfg()
  routeEdit.active = true
})

describe('parseCfg', () => {
  it('空・壊れた JSON は空の設定', () => {
    expect(parseCfg('')).toEqual(emptyCfg())
    expect(parseCfg('{broken')).toEqual(emptyCfg())
  })
  it('欠けた項目は既定で埋める', () => {
    const c = parseCfg(JSON.stringify({ groups: { A: [[1, 2]] }, mission: { laps: 3 } }))
    expect(c.groups.A).toEqual([[1, 2]])
    expect(c.mission).toEqual({ laps: 3, time_s: 0, then: '' })
    expect(c.active).toBe('auto')
    expect(c.avoid_rects).toEqual([])
  })
})

describe('editClick（グループ）', () => {
  it('末尾に足し、近くのクリックで消す。空になったらグループごと消す', () => {
    setTarget('A')
    editClick(1.00011, 2, PX)
    expect(routeEdit.cfg!.groups.A).toEqual([[1, 2]]) // mm に丸める
    editClick(3, 4, PX)
    editClick(1.05, 2, PX) // 5px 先 → 消す
    expect(routeEdit.cfg!.groups.A).toEqual([[3, 4]])
    expect(routeEdit.dirty).toBe(true)
    undoLast()
    expect(routeEdit.cfg!.groups.A).toBeUndefined()
  })
})

describe('editClick（停止点）', () => {
  it('1回目で位置、2回目で向き。位置と同じ所では向きを決めない', () => {
    setTarget('P1')
    editClick(1, 1, PX)
    expect(routeEdit.cfg!.stops.P1).toEqual({ x: 1, y: 1, yaw: null, mode: 'stop' })
    expect(routeEdit.awaitingYaw).toBe(true)
    editClick(1.01, 1, PX) // 1px → 無視
    expect(routeEdit.awaitingYaw).toBe(true)
    editClick(1, 2, PX)
    expect(routeEdit.cfg!.stops.P1!.yaw).toBeCloseTo(Math.PI / 2, 3)
    expect(routeEdit.awaitingYaw).toBe(false)
  })
  it('消すと mission.then の参照も外す', () => {
    setTarget('P2')
    editClick(0, 0, PX)
    routeEdit.cfg!.mission.then = 'P2'
    clearTarget()
    expect(routeEdit.cfg!.stops.P2).toBeUndefined()
    expect(routeEdit.cfg!.mission.then).toBe('')
  })
})

describe('避ける範囲', () => {
  it('ドラッグで正規化した矩形を足し、中のクリックで後に足したものから消す', () => {
    setTarget(AVOID)
    editDrag(2, 2, 0, 0)
    editDrag(1, 1, 3, 3)
    expect(routeEdit.cfg!.avoid_rects).toEqual([[0, 0, 2, 2], [1, 1, 3, 3]])
    editClick(1.5, 1.5, PX)
    expect(routeEdit.cfg!.avoid_rects).toEqual([[0, 0, 2, 2]])
    editClick(9, 9, PX) // どこにも当たらない
    expect(routeEdit.cfg!.avoid_rects).toHaveLength(1)
  })
  it('グループの編集中はドラッグを無視する', () => {
    setTarget('A')
    editDrag(0, 0, 1, 1)
    expect(routeEdit.cfg!.avoid_rects).toEqual([])
  })
})
