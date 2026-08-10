/**
 * 設定ビュー — 層 D（走行後・調整時に開く）。
 *
 * ここにあるのは**すべて GUI 側だけで完結する調整値**（`store/ui.ts` の
 * `DrivingSettings`）。`localStorage` に保存され、次回起動時も引き継がれる。
 *
 * ⚠ **Pi 側の実機設定（TC/TV ゲインなど、`CONFIG_SET`/`CONFIG_GET` で
 * STM32 と同期するもの）はまだここに無い。** io_node 側にその WS 経路が
 * 無いため（`architecture.md` §10.5 の「Pi を single source of truth に
 * する」設計はまだ未実装）。ここで扱うのは、あくまで GUI が指令を作る
 * ときのランプ・上限だけ。
 */
import type { DrivingSettings } from '../store/ui'
import {
  DEFAULT_SETTINGS, MAX_BRAKE_TORQUE_NM, PI_MAX_SPEED_CAP, PI_MAX_STEER_CAP, SETTINGS_RANGE, useUi,
} from '../store/ui'
import { RAD2DEG } from '../format'

type Field = {
  key: keyof DrivingSettings
  label: string
  unit: string
  note?: string
  /** 表示だけ変換する（保存値は SI のまま）。deg 表示用 */
  toDisplay?: (v: number) => number
  toStored?: (v: number) => number
  digits?: number
}

const SPEED_FIELDS: Field[] = [
  {
    key: 'maxSpeed',
    label: '最高速度',
    unit: 'm/s',
    note: `Pi 側ハード上限 ${PI_MAX_SPEED_CAP.toFixed(2)} m/s を超えては設定できない（超えても切り捨てられるだけ）`,
    digits: 2,
  },
  { key: 'cruiseScale', label: '巡航レンジ', unit: '×', note: '既定（低速側）。最高速度に対する倍率', digits: 2 },
  { key: 'boostScale', label: 'ブーストレンジ', unit: '×', note: 'Shift / パッド R1 を押している間。最高速度に対する倍率', digits: 2 },
  { key: 'accel', label: '加速', unit: 'm/s²', note: '押している間の加速度', digits: 1 },
  { key: 'coast', label: '惰行減速', unit: 'm/s²', note: 'キーを離したときの減速度', digits: 1 },
  { key: 'brake', label: '逆キーブレーキ', unit: 'm/s²', note: '進行方向と逆を押したときの減速度。加速より強くすること', digits: 1 },
  { key: 'kickSpeed', label: '発進キック', unit: 'm/s', note: '停止から押した瞬間に跳ばす速度。実車が転がり始める値に合わせる', digits: 2 },
]

const STEER_FIELDS: Field[] = [
  {
    key: 'maxSteer',
    label: '最大舵角',
    unit: '°',
    note: `Pi 側ハード上限 ${(PI_MAX_STEER_CAP * RAD2DEG).toFixed(0)}° を超えては設定できない`,
    toDisplay: (v) => v * RAD2DEG,
    toStored: (v) => v / RAD2DEG,
    digits: 0,
  },
  { key: 'steerRate', label: '切り込み速度', unit: 'rad/s', note: '押している間の切り込みの速さ', digits: 1 },
  { key: 'steerReturn', label: '戻り速度', unit: 'rad/s', note: '離したときのセンター戻り。切り込みより速くすること', digits: 1 },
  { key: 'steerCounter', label: '切り返し速度', unit: 'rad/s', note: '逆キーでの切り戻し', digits: 1 },
]

const BRAKE_FIELDS: Field[] = [
  {
    key: 'brakeTorque',
    label: 'ブレーキ強さ',
    unit: 'N·m/輪',
    note: `Space / パッド L1 を押している間に掛かるトルク。上限 ${MAX_BRAKE_TORQUE_NM} N·m（STM32 側の上限。超えても丸められる）`,
    digits: 3,
  },
]

const MISC_FIELDS: Field[] = [
  {
    key: 'armIdleTimeoutMs',
    label: 'ARM 無操作タイムアウト',
    unit: '秒',
    note: 'これを過ぎると自動 DISARM。実質的な停止手段ではなく、放置対策の最後の受け皿',
    toDisplay: (v) => v / 1000,
    toStored: (v) => v * 1000,
    digits: 0,
  },
]

export function SettingsView() {
  const settings = useUi((s) => s.settings)
  const setSettings = useUi((s) => s.setSettings)
  const resetSettings = useUi((s) => s.resetSettings)

  return (
    <div className="settings">
      <div className="settings-head">
        <h2>設定</h2>
        <p className="dim">
          ラジコン操作の速度・舵の応答を調整する。ブラウザの <code>localStorage</code>{' '}
          に保存され、このブラウザでは次回以降も引き継がれる。
        </p>
        <button onClick={resetSettings}>既定値に戻す</button>
      </div>

      <SettingGroup title="速度" fields={SPEED_FIELDS} settings={settings} onChange={setSettings} />
      <SettingGroup title="舵" fields={STEER_FIELDS} settings={settings} onChange={setSettings} />
      <SettingGroup title="ブレーキ" fields={BRAKE_FIELDS} settings={settings} onChange={setSettings} />
      <SettingGroup title="操作" fields={MISC_FIELDS} settings={settings} onChange={setSettings} />
    </div>
  )
}

function SettingGroup({
  title,
  fields,
  settings,
  onChange,
}: {
  title: string
  fields: Field[]
  settings: DrivingSettings
  onChange: (p: Partial<DrivingSettings>) => void
}) {
  return (
    <section className="settings-group">
      <h3>{title}</h3>
      {fields.map((f) => (
        <SettingRow key={f.key} field={f} settings={settings} onChange={onChange} />
      ))}
    </section>
  )
}

function SettingRow({
  field: f,
  settings,
  onChange,
}: {
  field: Field
  settings: DrivingSettings
  onChange: (p: Partial<DrivingSettings>) => void
}) {
  const range = SETTINGS_RANGE[f.key]
  const toDisplay = f.toDisplay ?? ((v: number) => v)
  const toStored = f.toStored ?? ((v: number) => v)
  const digits = f.digits ?? 2
  const stored = settings[f.key]
  const display = toDisplay(stored)
  const isDefault = stored === DEFAULT_SETTINGS[f.key]

  return (
    <div className="settings-row">
      <div className="settings-row-head">
        <span className="label">{f.label}</span>
        <b>{display.toFixed(digits)}</b>
        <span className="unit">{f.unit}</span>
        {!isDefault && (
          <button className="settings-reset" onClick={() => onChange({ [f.key]: DEFAULT_SETTINGS[f.key] })}>
            既定値 {toDisplay(DEFAULT_SETTINGS[f.key]).toFixed(digits)}
          </button>
        )}
      </div>
      <input
        type="range"
        min={toDisplay(range.min)}
        max={toDisplay(range.max)}
        step={toDisplay(range.step) - toDisplay(0)}
        value={display}
        onChange={(e) => onChange({ [f.key]: toStored(Number(e.target.value)) })}
      />
      {f.note && <span className="note">{f.note}</span>}
    </div>
  )
}
