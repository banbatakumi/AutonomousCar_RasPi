/** 設定パネルの「走行（制御方式・速度orトルク・MT・舵・ブレーキ）」タブ。`SettingsPanel.tsx` から分けた。 */
import type { ControlChannel } from '../../ws/control'
import { SettingGroup, useDriveSettingsEditor, SPEED_FIELDS, STEER_FIELDS, BRAKE_FIELDS, TORQUE_FIELDS, MT_FIELDS, MT_SPEED_FIELDS, MT_FEEL_FIELDS } from './fields'

export function DriveTab({ ch }: { ch: ControlChannel | null }) {
  const { settings, settingsDefault, setDriveSettings, range } = useDriveSettingsEditor(ch)

  return (
    <>
      {/*
        制御方式を先頭に置く。**これがスロットルの意味そのものを変える**ので、
        下に並ぶ項目のどれが効くかもここで決まる。チェックボックスだと
        「今どちらなのか」がラベルを読まないと分からないので、2択のセグメントにした。
      */}
      <section className="settings-group">
        <h3>制御方式</h3>
        <div className="seg">
          {(
            [
              ['speed', '速度制御'],
              ['torque', 'トルク制御'],
              ['mt', 'MT（擬似変速）'],
            ] as const
          ).map(([m, label]) => (
            <button
              key={m}
              className={settings.driveMode === m ? 'on' : ''}
              onClick={() => setDriveSettings({ driveMode: m })}
            >
              {label}
            </button>
          ))}
        </div>
      </section>

      {settings.driveMode === 'torque' ? (
        <SettingGroup title="トルク制御" fields={TORQUE_FIELDS} settings={settings} range={range} defaults={settingsDefault} onChange={setDriveSettings} />
      ) : settings.driveMode === 'mt' ? (
        <>
          {/* MT は 'speed'/'torque' とパラメータを一切共有しない。
              `MT_SPEED_FIELDS`（`mtMaxSpeed`/`mtAccel`）は独立したキーで、
              減速は `MT_FEEL_FIELDS`（`mtCoast`/`mtEngineBrake`）に一本化した。
              逆キーブレーキ・発進キックに相当する項目は無い（MT には無い機能） */}
          <SettingGroup title="速度制御" fields={MT_SPEED_FIELDS} settings={settings} range={range} defaults={settingsDefault} onChange={setDriveSettings} />
          <SettingGroup title="MT 減速フィール（惰行・エンジンブレーキ）" fields={MT_FEEL_FIELDS} settings={settings} range={range} defaults={settingsDefault} onChange={setDriveSettings} />
          <SettingGroup title="MT ギア比（1速〜5速。R-N-1-2-3-4-5、L1/← =シフトダウン、R1/→ =シフトアップ）" fields={MT_FIELDS} settings={settings} range={range} defaults={settingsDefault} onChange={setDriveSettings} />
        </>
      ) : (
        <SettingGroup title="速度制御" fields={SPEED_FIELDS} settings={settings} range={range} defaults={settingsDefault} onChange={setDriveSettings} />
      )}

      <SettingGroup title="舵" fields={STEER_FIELDS} settings={settings} range={range} defaults={settingsDefault} onChange={setDriveSettings} />
      <SettingGroup title="ブレーキ" fields={BRAKE_FIELDS} settings={settings} range={range} defaults={settingsDefault} onChange={setDriveSettings} />
    </>
  )
}
