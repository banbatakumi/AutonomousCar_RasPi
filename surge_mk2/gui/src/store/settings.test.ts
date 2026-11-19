import { afterEach, beforeEach, describe, expect, it, vi } from 'vitest'
import { live } from '../bus/live'
import type { LinkDiag } from '../types'
import { migrateSettings } from './settings'
import { DEFAULT_SETTINGS, PI_MAX_SPEED_CAP, SETTINGS_RANGE, clampSettings, effectiveRange, restoreSettings, type DrivingSettings } from './settings'

const withLimits = (l: Partial<LinkDiag> | null) => {
  live.link = l as LinkDiag | null
}

afterEach(() => withLimits(null))

describe('effectiveRange', () => {
  it('LIMITS 未受信なら静的なレンジ', () => {
    expect(effectiveRange(null).maxSpeed).toEqual(SETTINGS_RANGE.maxSpeed)
  })
  it('LIMITS を受けたら上限をそちらに差し替える（下限は残す）', () => {
    const r = effectiveRange({ max_speed_m_s: 5, max_accel_m_s2: 4, max_torque_nm: 0.1, max_steer_rad: 0.3 })
    expect(r.maxSpeed.max).toBe(5)
    expect(r.mtMaxSpeed.max).toBe(5)
    expect(r.coast).toEqual({ ...r.accel, min: SETTINGS_RANGE.coast.min })
    expect(r.brakeTorque.max).toBe(0.1)
    expect(r.maxSteer.max).toBe(0.3)
  })
  it('0・非有限の LIMITS は無視する', () => {
    expect(effectiveRange({ max_speed_m_s: 0, max_accel_m_s2: NaN, max_torque_nm: null, max_steer_rad: -1 }).maxSpeed)
      .toEqual(SETTINGS_RANGE.maxSpeed)
  })
})

describe('clampSettings', () => {
  it('範囲外を範囲に収め、壊れた数値は既定値に戻す', () => {
    const s = clampSettings({ ...DEFAULT_SETTINGS, accel: 99, coast: -1, brake: NaN })
    expect(s.accel).toBe(SETTINGS_RANGE.accel.max)
    expect(s.coast).toBe(SETTINGS_RANGE.coast.min)
    expect(s.brake).toBe(DEFAULT_SETTINGS.brake)
  })
  it('LIMITS が届く前は最高速度の上を切らない（保存値を仮値で巻き戻さない）', () => {
    const s = clampSettings({ ...DEFAULT_SETTINGS, maxSpeed: PI_MAX_SPEED_CAP + 2 })
    expect(s.maxSpeed).toBe(PI_MAX_SPEED_CAP + 2)
  })
  it('LIMITS が届いたらその上限で切る', () => {
    withLimits({ max_speed_m_s: 4 })
    expect(clampSettings({ ...DEFAULT_SETTINGS, maxSpeed: 9 }).maxSpeed).toBe(4)
  })
  it('capUpper=false は上を切らず下限だけ直す（保存する意思の値）', () => {
    withLimits({ max_speed_m_s: 4 })
    const s = clampSettings({ ...DEFAULT_SETTINGS, maxSpeed: 9, accel: 0 }, false)
    expect(s.maxSpeed).toBe(9)
    expect(s.accel).toBe(SETTINGS_RANGE.accel.min)
  })
  it('enum・真偽値の不正値は既定に倒す', () => {
    const bad = { ...DEFAULT_SETTINGS, driveMode: 'x', autoStop: 'yes', speedUnit: 'mph', engineSoundType: 1 }
    const s = clampSettings(bad as unknown as DrivingSettings)
    expect(s.driveMode).toBe('speed')
    expect(s.autoStop).toBe(DEFAULT_SETTINGS.autoStop)
    expect(s.speedUnit).toBe('ms')
    expect(s.engineSoundType).toBe('combustion')
  })
})

describe('migrateSettings / restoreSettings', () => {
  it('版 0 は旧 torqueMode を driveMode に直す（driveMode があればそのまま）', () => {
    const m = migrateSettings(
      { settingsIntent: { torqueMode: true }, settingsDefault: { torqueMode: false, driveMode: 'mt' } },
      0,
    ) as { settingsIntent: Record<string, unknown>; settingsDefault: Record<string, unknown> }
    expect(m.settingsIntent.driveMode).toBe('torque')
    expect(m.settingsDefault.driveMode).toBe('mt')
  })
  it('今の版では torqueMode を見ない', () => {
    const m = migrateSettings({ settingsIntent: { torqueMode: true } }, 1) as { settingsIntent: Record<string, unknown> }
    expect(m.settingsIntent.driveMode).toBeUndefined()
  })
  it('壊れた保存値・知らない値は既定に倒す', () => {
    const r = restoreSettings({ settingsIntent: 'broken', settingsDefault: null, winkerMode: 7 })
    expect(r.settingsIntent).toEqual(clampSettings(DEFAULT_SETTINGS, false))
    expect(r.winkerMode).toBe(2)
  })
  it('後から足した項目は既定値で埋まる', () => {
    const { launchControl: _, ...old } = DEFAULT_SETTINGS
    const r = restoreSettings({ settingsIntent: { ...old, accel: 3 } })
    expect(r.settingsIntent.launchControl).toBe(DEFAULT_SETTINGS.launchControl)
    expect(r.settingsIntent.accel).toBe(3)
  })
})

describe('ストアの保存（persist）', () => {
  beforeEach(() => {
    localStorage.clear()
    vi.resetModules()
  })

  it('persist 導入前の旧キーから設定を引き継ぐ', async () => {
    localStorage.setItem('surge.driveSettings.v1', JSON.stringify({ ...DEFAULT_SETTINGS, accel: 3.3, torqueMode: true, driveMode: undefined }))
    localStorage.setItem('surge.driveSettingsDefault.v1', JSON.stringify({ ...DEFAULT_SETTINGS, coast: 1.1 }))
    localStorage.setItem('surge.winkerMode', '3')
    const { useUi } = await import('./ui')
    const s = useUi.getState()
    expect(s.settings.accel).toBe(3.3)
    expect(s.settings.driveMode).toBe('torque')
    expect(s.settingsDefault.coast).toBe(1.1)
    expect(s.winkerMode).toBe(3)
    // 新しいキーへ書き移している（旧キーは残す）
    expect(JSON.parse(localStorage.getItem('surge.ui')!).version).toBe(1)
    expect(localStorage.getItem('surge.driveSettings.v1')).not.toBeNull()
  })

  it('変更は保存され、読み直しても残る。ARM・ギアなどは保存しない', async () => {
    let { useUi } = await import('./ui')
    useUi.getState().setSettings({ accel: 4.2 })
    useUi.getState().set({ winkerMode: 1, gear: 'R', sideBrakeRequested: true })
    vi.resetModules()
    ;({ useUi } = await import('./ui'))
    const s = useUi.getState()
    expect(s.settings.accel).toBe(4.2)
    expect(s.winkerMode).toBe(1)
    expect(s.gear).toBe('D')
    expect(s.sideBrakeRequested).toBe(false)
  })

  it('保存が無ければ既定値', async () => {
    const { useUi } = await import('./ui')
    expect(useUi.getState().settings).toEqual(clampSettings(DEFAULT_SETTINGS))
    expect(useUi.getState().winkerMode).toBe(2)
  })
})
