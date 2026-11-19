/**
 * 設定パネルの共通部品: 運転設定のスライダ行（`SettingGroup`・`SettingRow`）と項目の定義、
 * 運転設定を編集するフック（`useDriveSettingsEditor`）。`SettingsPanel.tsx` から分けた。
 */
import { useNumbers } from '../../bus/live'
import { RAD2DEG } from '../../format'
import {
  MAX_BRAKE_TORQUE_NM, MAX_TARGET_TORQUE_NM, PI_MAX_SPEED_CAP, PI_MAX_STEER_CAP, effectiveRange,
  type DrivingSettings, type NumericSettingKey, type SettingRange,
} from '../../store/settings'
import { useUi } from '../../store/ui'
import type { ControlChannel } from '../../ws/control'

/**
 * 運転設定の編集。`speedUnit`/`engineSoundType` を除く運転設定はPi側にも保存する（`_on_drive_settings`
 * 参照）。ローカル反映はこれまで通り即座、サーバ側もサーバが真値の他設定
 * （camera/signal 等）と同じく送った項目だけ即保存する。
 *
 * スライダのレンジは車両の物理的な上限値（`LIMITS` パケット由来。★v0.11）で決まる。`link`（LinkDiag、
 * `/ws/telemetry` 8Hz）から読む——`/ws/control` の status はイベント発生時にしか broadcast されず、
 * リロードするまで反映されないように見えるため
 */
export function useDriveSettingsEditor(ch: ControlChannel | null) {
  const settings = useUi((s) => s.settings)
  const settingsDefault = useUi((s) => s.settingsDefault)
  const setSettings = useUi((s) => s.setSettings)
  const setDriveSettings = (p: Partial<DrivingSettings>) => {
    setSettings(p)
    ch?.setDriveSettings(p)
  }
  const { link } = useNumbers()
  const range = effectiveRange({
    max_speed_m_s: link?.max_speed_m_s ?? null,
    max_accel_m_s2: link?.max_accel_m_s2 ?? null,
    max_torque_nm: link?.max_torque_nm ?? null,
    max_steer_rad: link?.max_steer_rad ?? null,
  })
  return { settings, settingsDefault, setDriveSettings, range }
}

export type Field = {
  key: NumericSettingKey
  label: string
  unit: string
  /** レンジ（動的に決まる `effectiveRange` の結果）を受け取って説明文を作る。
   * 固定文言でよいものは string のままでよい */
  note?: string | ((range: SettingRange) => string)
  /** 表示だけ変換する（保存値は SI のまま）。deg 表示用 */
  toDisplay?: (v: number) => number
  toStored?: (v: number) => number
  digits?: number
}

export const SPEED_FIELDS: Field[] = [
  {
    key: 'maxSpeed',
    label: '最高速度',
    unit: 'm/s',
    note: (r) =>
      `上限 ${r.max.toFixed(2)} m/s を超えては設定できない（超えても切り捨てられるだけ）。` +
      `STM32 実測（LIMITS）が届いていればそれを使う。未接続時のみ既定 ${PI_MAX_SPEED_CAP.toFixed(2)} m/s`,
    digits: 2,
  },
  { key: 'accel', label: '加速', unit: 'm/s²', note: '押している間の加速度', digits: 1 },
  { key: 'coast', label: '惰行減速', unit: 'm/s²', note: 'キーを離したときの減速度', digits: 1 },
  { key: 'brake', label: '逆キーブレーキ', unit: 'm/s²', note: '進行方向と逆を押したときの減速度。加速より強くすること', digits: 1 },
]

export const STEER_FIELDS: Field[] = [
  {
    key: 'maxSteer',
    label: '最大舵角',
    unit: '°',
    note: (r) =>
      `上限 ${(r.max * RAD2DEG).toFixed(0)}° を超えては設定できない。` +
      `STM32 実測（LIMITS）が届いていればそれを使う。未接続時のみ既定 ${(PI_MAX_STEER_CAP * RAD2DEG).toFixed(0)}°`,
    toDisplay: (v) => v * RAD2DEG,
    toStored: (v) => v / RAD2DEG,
    digits: 0,
  },
  { key: 'steerRate', label: '切り込み速度', unit: 'rad/s', note: '押している間の切り込みの速さ（キーボードのみ）', digits: 1 },
  { key: 'steerReturn', label: '戻り速度', unit: 'rad/s', note: '離したときのセンター戻り。切り込みより速くすること（キーボードのみ）', digits: 1 },
  { key: 'steerCounter', label: '切り返し速度', unit: 'rad/s', note: '逆キーでの切り戻し（キーボードのみ）', digits: 1 },
  {
    key: 'gamepadSteerExpo',
    label: 'パッド舵カーブ（EXPO）',
    unit: '',
    note: '左スティック中央付近の反応を緩める強さ。0=リニア、1=最大限緩める。'
      + '上げるほど直進や小舵角の細かい操作がしやすくなる代わり、フルロック側は敏感になる（パッドのみ）',
    digits: 2,
  },
  {
    key: 'gamepadSteerDeadzone',
    label: 'パッド舵の遊び',
    unit: '',
    note: '左スティック中央のデッドゾーン。ここより内側の傾きは無視する（パッドのみ）',
    digits: 2,
  },
]

export const BRAKE_FIELDS: Field[] = [
  {
    key: 'brakeTorque',
    label: 'ブレーキ強さ',
    unit: 'N·m/輪',
    note: (r) =>
      `Space / パッド L2 を押している間に掛かるトルク。上限 ${r.max.toFixed(3)} N·m` +
      `（STM32 実測 max_torque_nm が届いていればそれを使う。未接続時のみ既定 ${MAX_BRAKE_TORQUE_NM} N·m。超えても丸められる）`,
    digits: 3,
  },
]

export const TORQUE_FIELDS: Field[] = [
  {
    key: 'driveTorque',
    label: 'トルク強さ',
    unit: 'N·m',
    note: (r) =>
      `スロットル全開で出す駆動トルク。上限 ${r.max.toFixed(3)} N·m` +
      `（STM32 実測 max_torque_nm が届いていればそれを使う。未接続時のみ既定 ${MAX_TARGET_TORQUE_NM} N·m。超えても丸められる）`,
    digits: 3,
  },
  // 速度メータ（`SpeedGauge.tsx`）の目盛りは、速度制御・トルク制御どちらでも常に
  // STM32 実測 `LIMITS.max_speed_m_s` に固定なので、ここに目盛りの調整項目は無い
]

/**
 * MT（擬似変速）モードのギア刻み。表示は「MT専用の最高速度（`mtMaxSpeed`、
 * 下の `MT_SPEED_FIELDS`）に対する％」——絶対速度にすると `mtMaxSpeed` を変えるたびに
 * 再計算しないと意味が読めなくなるため、倍率のまま見せる（`store/settings.ts` の
 * `mtGear1`〜`mtGear5` は 0-1 の倍率）。
 */
export const MT_FIELDS: Field[] = [
  { key: 'mtGear1', label: '1速', unit: '%', note: 'MT専用の最高速度に対する割合。後退（R）もこの値を使う。ニュートラル（N）は常に0%固定でここでは調整できない', toDisplay: (v) => v * 100, toStored: (v) => v / 100, digits: 0 },
  { key: 'mtGear2', label: '2速', unit: '%', note: 'MT専用の最高速度に対する割合', toDisplay: (v) => v * 100, toStored: (v) => v / 100, digits: 0 },
  { key: 'mtGear3', label: '3速', unit: '%', note: 'MT専用の最高速度に対する割合', toDisplay: (v) => v * 100, toStored: (v) => v / 100, digits: 0 },
  { key: 'mtGear4', label: '4速', unit: '%', note: 'MT専用の最高速度に対する割合', toDisplay: (v) => v * 100, toStored: (v) => v / 100, digits: 0 },
  { key: 'mtGear5', label: '5速', unit: '%', note: 'MT専用の最高速度に対する割合。既定は100%＝トップギアで mtMaxSpeed そのまま', toDisplay: (v) => v * 100, toStored: (v) => v / 100, digits: 0 },
]

/**
 * MT モード専用の「速度制御」欄。'speed' モードの `SPEED_FIELDS` とは
 * キーを一切共有しない独立した項目——`maxSpeed`/`accel` を変えても MT には影響しない。
 * `coast`/`brake`/`kickSpeed` に相当するものは無い（MT には逆キーブレーキも発進キックも
 * 無く、減速はすべて `MT_FEEL_FIELDS` の `mtCoast`/`mtEngineBrake` に一本化した）。
 */
export const MT_SPEED_FIELDS: Field[] = [
  {
    key: 'mtMaxSpeed',
    label: '最高速度',
    unit: 'm/s',
    note: (r) =>
      `MT 専用の最高速度（5速でこの値そのもの）。上限 ${r.max.toFixed(2)} m/s を超えては設定できない。` +
      `STM32 実測（LIMITS）が届いていればそれを使う`,
    digits: 2,
  },
  { key: 'mtAccel', label: '加速', unit: 'm/s²', note: 'アクセルを踏んでいる（踏み込んでいる）間の加速度', digits: 1 },
]

/** MT のアクセルオフ時の減速フィール。実車の惰行・エンジンブレーキ相当 */
export const MT_FEEL_FIELDS: Field[] = [
  { key: 'mtCoast', label: '惰行減速', unit: 'm/s²', note: 'アクセルを離したときの減速度（ブレーキではなく慣性）。ニュートラル（N）でもこの値が使われる', digits: 1 },
  { key: 'mtEngineBrake', label: 'エンジンブレーキ', unit: 'm/s²', note: '低いギアほど加算される追加の減速度。1速で全量、5速では加算なし（惰行減速のみ）', digits: 1 },
]

export const MISC_FIELDS: Field[] = [
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

/** `LinkDiag.control_params_status`（`raspi/core/control_params.py`）の表示。 */
export function controlParamsLabel(status: string | undefined): string {
  switch (status) {
    case 'ok':
      return 'STM32 に適用済み ✓'
    case 'pending':
      return '送信中…'
    case 'mismatch':
      return '⚠ STM32 が範囲外として丸めた値があります'
    case 'unsupported':
      return '⚠ STM32 のファームが古く、対応していません'
    case 'invalid':
      return '⚠ vehicle.toml の [control] を読めません（STM32 の既定値で走行中）'
    case 'none':
      return '未設定（STM32 の既定値）'
    default:
      return '未確認'
  }
}

export function SettingGroup({
  title,
  fields,
  settings,
  range,
  defaults,
  onChange,
}: {
  title: string
  fields: Field[]
  settings: DrivingSettings
  range: Record<NumericSettingKey, SettingRange>
  defaults: DrivingSettings
  onChange: (p: Partial<DrivingSettings>) => void
}) {
  return (
    <section className="settings-group">
      <h3>{title}</h3>
      {fields.map((f) => (
        <SettingRow key={f.key} field={f} settings={settings} range={range} defaults={defaults} onChange={onChange} />
      ))}
    </section>
  )
}

function SettingRow({
  field: f,
  settings,
  range: fullRange,
  defaults,
  onChange,
}: {
  field: Field
  settings: DrivingSettings
  range: Record<NumericSettingKey, SettingRange>
  defaults: DrivingSettings
  onChange: (p: Partial<DrivingSettings>) => void
}) {
  const range = fullRange[f.key]
  const toDisplay = f.toDisplay ?? ((v: number) => v)
  const toStored = f.toStored ?? ((v: number) => v)
  const digits = f.digits ?? 2
  const stored = settings[f.key]
  const display = toDisplay(stored)
  // **`DEFAULT_SETTINGS`（ソースコード固定値）ではなく `defaults`（`settingsDefault`、
  // ユーザーが「現在の値を既定値として保存」で上書きできる）を指すこと。**
  const isDefault = stored === defaults[f.key]
  const note = typeof f.note === 'function' ? f.note(range) : f.note

  return (
    <div className="settings-row" title={note}>
      <div className="settings-row-head">
        <span className="label">{f.label}</span>
        <b>{display.toFixed(digits)}</b>
        <span className="unit">{f.unit}</span>
        {!isDefault && (
          <button className="settings-reset" onClick={() => onChange({ [f.key]: defaults[f.key] })}>
            既定値 {toDisplay(defaults[f.key]).toFixed(digits)}
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
    </div>
  )
}
