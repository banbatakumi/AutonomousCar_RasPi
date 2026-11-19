import { beforeEach, describe, expect, it } from 'vitest'
import { LIGHT_DAYTIME, LIGHT_NORMAL, LIGHT_OFF, useUi } from './ui'

beforeEach(() => {
  useUi.setState({ armRequested: false, disarmReason: '', lightMode: LIGHT_OFF })
})

describe('arm / disarm（ARM と灯火）', () => {
  it('ARM で消灯なら DAY を点け、理由を消す', () => {
    useUi.setState({ disarmReason: '前の理由' })
    useUi.getState().arm()
    const s = useUi.getState()
    expect(s.armRequested).toBe(true)
    expect(s.lightMode).toBe(LIGHT_DAYTIME)
    expect(s.disarmReason).toBe('')
  })
  it('ARM のとき NORMAL を選んでいたら下げない', () => {
    useUi.setState({ lightMode: LIGHT_NORMAL })
    useUi.getState().arm()
    expect(useUi.getState().lightMode).toBe(LIGHT_NORMAL)
  })
  it('DISARM で消灯に戻し、理由を残す', () => {
    useUi.getState().arm()
    useUi.getState().disarm('無操作')
    const s = useUi.getState()
    expect(s.armRequested).toBe(false)
    expect(s.lightMode).toBe(LIGHT_OFF)
    expect(s.disarmReason).toBe('無操作')
  })
  it('ARM していなければ DISARM は何もしない（理由も灯火も書き換えない）', () => {
    useUi.setState({ lightMode: LIGHT_NORMAL, disarmReason: '前の理由' })
    useUi.getState().disarm('別の理由')
    const s = useUi.getState()
    expect(s.lightMode).toBe(LIGHT_NORMAL)
    expect(s.disarmReason).toBe('前の理由')
  })
  it('DISARM 中に手で点けた灯火はそのまま（変化のときだけ書き換える）', () => {
    useUi.getState().set({ lightMode: LIGHT_NORMAL })
    expect(useUi.getState().lightMode).toBe(LIGHT_NORMAL)
  })
  it('ARM 中に二度押しても状態は変わらない', () => {
    useUi.getState().arm()
    useUi.getState().set({ lightMode: LIGHT_OFF }) // ARM 中に手で消した
    useUi.getState().arm()
    expect(useUi.getState().lightMode).toBe(LIGHT_OFF)
  })
})
