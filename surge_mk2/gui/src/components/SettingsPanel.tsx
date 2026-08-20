/**
 * 設定パネル — 走行タブ（ラジコン／自動運転）の歯車から開くドロワーの中身
 * （`components/SettingsDrawer.tsx`）。
 *
 * ## なぜ独立したタブをやめたのか
 *
 * ここの項目は**すべて手動（キーボード/パッド）操作のためのもの**で、診断でも
 * ログでも使わない。タブを1枚使って画面上部を占有する価値がなく、かつ
 * 「走らせて → 加速が鈍い → 直す → また走らせる」の往復でタブを切り替えるのは
 * 手間でしかない。走行タブの中に畳んで、走りながら開けるようにした。
 *
 * 自動運転タブでも中身は同じもの（2026-08-20）——engage 中でも人間がキーボードで
 * 位置を微調整する場面があり、そこで効く速度・舵の調整値はラジコンタブと共通
 * （`store/ui.ts` の `DrivingSettings` は1つしかなく、タブごとに分かれていない）。
 *
 * ここにあるのは**すべて GUI 側だけで完結する調整値**（`store/ui.ts` の
 * `DrivingSettings`）。`localStorage` に保存され、次回起動時も引き継がれる。
 *
 * ⚠ **STM32 実機の `CONFIG_SET`/`CONFIG_GET` で同期する設定は、
 * TC/TV の有効・無効切り替え（★v0.8）だけ対応済み。** ゲイン等の連続値
 * パラメータ（`TC_SLIP_THRESH`/`TV_GAIN`/`SPEED_KP`/`SPEED_KI`）は
 * STM32 ファームウェア側が未実装のため、まだここに無い
 * （`architecture.md` §10.5 の「Pi を single source of truth にする」
 * 設計は部分的にしか実現していない）。他はすべて GUI が指令を作る
 * ときのランプ・上限だけ。
 */
import type { DrivingSettings, NumericSettingKey } from '../store/ui'
import {
  AUTO_STOP_DISTANCE_M, DEFAULT_SETTINGS, MAX_BRAKE_TORQUE_NM, MAX_TARGET_TORQUE_NM,
  PI_MAX_SPEED_CAP, PI_MAX_STEER_CAP, SETTINGS_RANGE, useUi,
} from '../store/ui'
import { RAD2DEG } from '../format'
import { useNumbers } from '../bus/live'
import type { ControlChannel } from '../ws/control'

type Field = {
  key: NumericSettingKey
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

const TORQUE_FIELDS: Field[] = [
  {
    key: 'driveTorque',
    label: 'トルク強さ',
    unit: 'N·m',
    note: `スロットル全開で出す駆動トルク。上限 ${MAX_TARGET_TORQUE_NM} N·m（STM32 側の上限。超えても丸められる）`,
    digits: 3,
  },
  /*
   * トルク制御では速度指令を送らないので、`maxSpeed` は走りに影響しない。
   * **それでも出しておく** — 速度メータの目盛りがこの値で決まるため、
   * ここを触れないと「針が振り切ったまま」または「まったく動かない」計器になる。
   */
  {
    key: 'maxSpeed',
    label: '速度メータの目盛り',
    unit: 'm/s',
    note: 'トルク制御では速度指令を送らないので、走りには影響しない（メータの上限だけを決める）',
    digits: 2,
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

export function SettingsPanel({ ch }: { ch: ControlChannel | null }) {
  const settings = useUi((s) => s.settings)
  const setSettings = useUi((s) => s.setSettings)
  const resetSettings = useUi((s) => s.resetSettings)
  // estop_active/drive_power_locked と同じく、これは `/ws/control` の status
  // （イベント発生時にしかbroadcastされない）ではなく `/ws/telemetry` 経由で
  // 継続更新される LinkDiag（8Hz、`bus/live.ts`）から読む。status から読むと
  // 実際は変わっているのにチェックボックスが更新されず、リロードするまで
  // 反映されないように見える（2026-08-19 実機で発覚）
  const { link } = useNumbers()
  const tcEnabled = link?.tc_enabled ?? null
  const tvEnabled = link?.tv_enabled ?? null
  const wheelLiftGuardEnabled = link?.wheel_lift_guard_enabled ?? null

  return (
    <div className="settings">
      <div className="settings-head">
        <button onClick={resetSettings}>既定値に戻す</button>
      </div>

      {/*
        制御方式を先頭に置く。**これがスロットルの意味そのものを変える**ので、
        下に並ぶ項目のどれが効くかもここで決まる。チェックボックスだと
        「今どちらなのか」がラベルを読まないと分からないので、2択のセグメントにした。
      */}
      <section className="settings-group">
        <h3>制御方式</h3>
        <div className="seg">
          <button
            className={settings.torqueMode ? '' : 'on'}
            onClick={() => setSettings({ torqueMode: false })}
          >
            速度制御
          </button>
          <button
            className={settings.torqueMode ? 'on' : ''}
            onClick={() => setSettings({ torqueMode: true })}
          >
            トルク制御
          </button>
        </div>
      </section>

      {settings.torqueMode ? (
        <SettingGroup title="トルク制御" fields={TORQUE_FIELDS} settings={settings} onChange={setSettings} />
      ) : (
        <SettingGroup title="速度制御" fields={SPEED_FIELDS} settings={settings} onChange={setSettings} />
      )}

      <SettingGroup title="舵" fields={STEER_FIELDS} settings={settings} onChange={setSettings} />
      <SettingGroup title="ブレーキ" fields={BRAKE_FIELDS} settings={settings} onChange={setSettings} />

      {/* ここだけは GUI の調整値ではなく「STM32 の機能を許可するかどうか」。
          距離も制動力も STM32 側の固定値で、GUI からは変えられない（v0.7） */}
      <section className="settings-group">
        <h3>自動停止（超音波）</h3>
        <label className="settings-checkbox">
          <input
            type="checkbox"
            checked={settings.autoStop}
            onChange={(e) => setSettings({ autoStop: e.target.checked })}
          />
          進行方向 {(AUTO_STOP_DISTANCE_M * 100).toFixed(0)}cm 未満で STM32 に自動停止させる
        </label>
      </section>

      {/* TC/TV・片輪浮き対策 も自動停止と同じく「STM32の機能を許可するかどうか」
          （TC/TVは★v0.8、片輪浮き対策は★v0.9。片輪浮き対策はTC/TV本体とは独立した別機構）。
          表示値は STM32 の CONFIG_ACK を経由したサーバ真値
          （`status.tc_enabled`/`tv_enabled`/`wheel_lift_guard_enabled`）。
          未確定（起動直後でまだ CONFIG_ACK が届いていない）間は null になる */}
      <section className="settings-group">
        <h3>走行アシスト（STM32）</h3>
        <label className="settings-checkbox">
          <input
            type="checkbox"
            checked={tcEnabled ?? true}
            disabled={tcEnabled === null}
            onChange={(e) => ch?.setTcTv({ tc: e.target.checked })}
          />
          トラクションコントロール（TC）{tcEnabled === null && '（未確認）'}
        </label>
        <label className="settings-checkbox">
          <input
            type="checkbox"
            checked={tvEnabled ?? true}
            disabled={tvEnabled === null}
            onChange={(e) => ch?.setTcTv({ tv: e.target.checked })}
          />
          トルクベクタリング（TV）{tvEnabled === null && '（未確認）'}
        </label>
        {/* 片輪浮き対策はTC/TV本体とは独立した別機構（★v0.9） */}
        <label className="settings-checkbox">
          <input
            type="checkbox"
            checked={wheelLiftGuardEnabled ?? true}
            disabled={wheelLiftGuardEnabled === null}
            onChange={(e) => ch?.setWheelLiftGuard(e.target.checked)}
          />
          片輪浮き対策{wheelLiftGuardEnabled === null && '（未確認）'}
        </label>
      </section>

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
    <div className="settings-row" title={f.note}>
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
    </div>
  )
}
