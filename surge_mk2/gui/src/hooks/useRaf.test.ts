import { afterEach, beforeEach, describe, expect, it, vi } from 'vitest'
import { rafLoop } from './useRaf'

let hidden = false
let queue: FrameRequestCallback[] = []

beforeEach(() => {
  hidden = false
  queue = []
  vi.spyOn(document, 'hidden', 'get').mockImplementation(() => hidden)
  vi.stubGlobal('requestAnimationFrame', (f: FrameRequestCallback) => queue.push(f))
  vi.stubGlobal('cancelAnimationFrame', (id: number) => {
    if (id > 0) queue[id - 1] = () => {}
  })
})
afterEach(() => {
  vi.restoreAllMocks()
  vi.unstubAllGlobals()
})

/** 予約済みのフレームを1つ回す */
const step = () => {
  const q = queue
  queue = []
  q.forEach((f) => f(performance.now()))
}

describe('rafLoop', () => {
  it('毎フレーム呼び、止めたら呼ばない', () => {
    const cb = vi.fn()
    const stop = rafLoop(cb)
    step()
    step()
    expect(cb).toHaveBeenCalledTimes(2)
    stop()
    step()
    expect(cb).toHaveBeenCalledTimes(2)
  })
  it('隠れている間は回さず、見えたら再開する', () => {
    const cb = vi.fn()
    const stop = rafLoop(cb)
    step()
    hidden = true
    document.dispatchEvent(new Event('visibilitychange'))
    step()
    step()
    expect(cb).toHaveBeenCalledTimes(1)
    hidden = false
    document.dispatchEvent(new Event('visibilitychange'))
    step()
    expect(cb).toHaveBeenCalledTimes(2)
    stop()
  })
  it('1フレームで例外が出てもループは続く', () => {
    let n = 0
    const stop = rafLoop(() => {
      n++
      if (n === 1) throw new Error('x')
    })
    expect(() => step()).toThrow()
    step()
    expect(n).toBe(2)
    stop()
  })
})
