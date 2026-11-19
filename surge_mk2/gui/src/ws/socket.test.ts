import { afterEach, beforeEach, describe, expect, it, vi } from 'vitest'
import { WS_OPEN_TIMEOUT_MS_FOR_TEST } from './guard'
import { reconnectingSocket } from './socket'

/** 手で開閉を起こせる WebSocket の代わり */
class FakeWs {
  static all: FakeWs[] = []
  static readonly OPEN = 1
  binaryType = 'blob'
  readyState = 0
  onopen: ((e: Event) => void) | null = null
  onmessage: ((e: MessageEvent) => void) | null = null
  onclose: ((e: CloseEvent) => void) | null = null
  onerror: ((e: Event) => void) | null = null
  private listeners: Record<string, (() => void)[]> = {}
  closeCalls = 0
  constructor(readonly url: string) {
    FakeWs.all.push(this)
  }
  addEventListener(t: string, f: () => void) {
    ;(this.listeners[t] ??= []).push(f)
  }
  open() {
    this.readyState = 1
    this.listeners.open?.forEach((f) => f())
    this.onopen?.(new Event('open'))
  }
  message(data: unknown) {
    this.onmessage?.({ data } as MessageEvent)
  }
  close() {
    this.closeCalls++
    this.readyState = 3
  }
  /** サーバ側から閉じられた（onclose が来る） */
  serverClose() {
    this.readyState = 3
    this.onclose?.({} as CloseEvent)
  }
}

const last = () => FakeWs.all[FakeWs.all.length - 1]!

beforeEach(() => {
  FakeWs.all = []
  vi.useFakeTimers()
  vi.stubGlobal('WebSocket', FakeWs)
})
afterEach(() => {
  vi.useRealTimers()
  vi.unstubAllGlobals()
})

describe('reconnectingSocket', () => {
  it('固定の待ち時間で張り直し、onLost は1本につき1回', () => {
    const onLost = vi.fn()
    const s = reconnectingSocket('ws://x', { backoff: 500, onMessage: () => {}, onLost })
    last().open()
    last().serverClose()
    last().serverClose() // 遅れた二重の close
    expect(onLost).toHaveBeenCalledTimes(1)
    vi.advanceTimersByTime(499)
    expect(FakeWs.all).toHaveLength(1)
    vi.advanceTimersByTime(1)
    expect(FakeWs.all).toHaveLength(2)
    s.close()
  })

  it('指数バックオフは倍々で上限まで伸び、開けたら最小に戻る', () => {
    const s = reconnectingSocket('ws://x', { backoff: { minMs: 300, maxMs: 1000 }, onMessage: () => {} })
    const waits: number[] = []
    for (let i = 0; i < 4; i++) {
      const n = FakeWs.all.length
      last().serverClose()
      let t = 0
      while (FakeWs.all.length === n) {
        vi.advanceTimersByTime(10)
        t += 10
      }
      waits.push(t)
    }
    expect(waits).toEqual([300, 600, 1000, 1000])
    last().open()
    const n = FakeWs.all.length
    last().serverClose()
    vi.advanceTimersByTime(300)
    expect(FakeWs.all).toHaveLength(n + 1)
    s.close()
  })

  it('close() の後は張り直さないが、閉じ終わりの onLost は呼ぶ', () => {
    const onLost = vi.fn()
    const s = reconnectingSocket('ws://x', { backoff: 100, onMessage: () => {}, onLost })
    last().open()
    s.close()
    expect(last().closeCalls).toBe(1)
    last().serverClose()
    expect(onLost).toHaveBeenCalledTimes(1)
    vi.advanceTimersByTime(1000)
    expect(FakeWs.all).toHaveLength(1)
  })

  it('開いた後に idleMs 何も届かなければ見切り、onDead → onLost → 張り直し', () => {
    const order: string[] = []
    const s = reconnectingSocket('ws://x', {
      backoff: 100,
      idleMs: 2000,
      onMessage: () => order.push('msg'),
      onDead: () => order.push('dead'),
      onLost: () => order.push('lost'),
    })
    last().open()
    vi.advanceTimersByTime(1500)
    last().message('a') // 生きている
    vi.advanceTimersByTime(1500)
    expect(order).toEqual(['msg'])
    vi.advanceTimersByTime(1000)
    expect(order).toEqual(['msg', 'dead', 'lost'])
    expect(FakeWs.all[0]!.onclose).toBeNull() // ハンドラは外してある
    vi.advanceTimersByTime(100)
    expect(FakeWs.all).toHaveLength(2)
    s.close()
  })

  it('開かないまま時間切れなら見切る', () => {
    const onLost = vi.fn()
    const s = reconnectingSocket('ws://x', { backoff: 100, onMessage: () => {}, onLost })
    vi.advanceTimersByTime(WS_OPEN_TIMEOUT_MS_FOR_TEST + 600)
    expect(onLost).toHaveBeenCalledTimes(1)
    s.close()
  })

  it('drop() は今の接続を捨てて張り直す。immediate なら待たない。切れている間は何もしない', () => {
    const onLost = vi.fn()
    const s = reconnectingSocket('ws://x', { backoff: 800, onMessage: () => {}, onLost })
    last().open()
    s.drop(true)
    expect(onLost).toHaveBeenCalledTimes(1)
    expect(FakeWs.all).toHaveLength(2)
    last().serverClose()
    s.drop() // 既に切れて待っている → 何もしない
    expect(onLost).toHaveBeenCalledTimes(2)
    vi.advanceTimersByTime(800)
    expect(FakeWs.all).toHaveLength(3)
    s.close()
  })
})
