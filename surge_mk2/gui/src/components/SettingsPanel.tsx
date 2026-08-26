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
 *
 * ## 中身のタブ分け（2026-08-24）
 *
 * 上の「独立したページタブをやめた」話とは別物——ここは**このドロワーの中だけ**
 * で完結する軽いタブ（`SETTINGS_TABS`）。項目が増えて縦に長くなりすぎたので、
 * 「走行」（制御方式・速度orトルク・舵・ブレーキ）「安全」（自動停止・TC/TV・
 * 片輪浮き対策・ARMタイムアウト）「カメラ」（capture FPS上限・後方ON/OFF・
 * GUI配信fps・進路ガイド校正）の3つに分けた。「既定値に戻す」等のヘッダは
 * タブの外に置いてある——`DrivingSettings` は1つしかなく、どのタブの値も
 * まとめて戻す/保存するので、特定のタブだけの操作ではないため。
 */
import { useState } from 'react'
import type { DrivingSettings, NumericSettingKey, SettingRange } from '../store/ui'
import {
  MAX_BRAKE_TORQUE_NM, MAX_TARGET_TORQUE_NM,
  PI_MAX_SPEED_CAP, PI_MAX_STEER_CAP, effectiveRange, useUi,
} from '../store/ui'
import { RAD2DEG } from '../format'
import { useNumbers } from '../bus/live'
import type { ControlChannel } from '../ws/control'

type Field = {
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

const SPEED_FIELDS: Field[] = [
  {
    key: 'maxSpeed',
    label: '最高速度',
    unit: 'm/s',
    note: (r) =>
      `上限 ${r.max.toFixed(2)} m/s を超えては設定できない（超えても切り捨てられるだけ）。` +
      `STM32 実測（LIMITS）が届いていればそれを使う。未接続時のみ既定 ${PI_MAX_SPEED_CAP.toFixed(2)} m/s`,
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
    note: (r) =>
      `上限 ${(r.max * RAD2DEG).toFixed(0)}° を超えては設定できない。` +
      `STM32 実測（LIMITS）が届いていればそれを使う。未接続時のみ既定 ${(PI_MAX_STEER_CAP * RAD2DEG).toFixed(0)}°`,
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
    note: (r) =>
      `Space / パッド L1 を押している間に掛かるトルク。上限 ${r.max.toFixed(3)} N·m` +
      `（STM32 実測 max_torque_nm が届いていればそれを使う。未接続時のみ既定 ${MAX_BRAKE_TORQUE_NM} N·m。超えても丸められる）`,
    digits: 3,
  },
]

const TORQUE_FIELDS: Field[] = [
  {
    key: 'driveTorque',
    label: 'トルク強さ',
    unit: 'N·m',
    note: (r) =>
      `スロットル全開で出す駆動トルク。上限 ${r.max.toFixed(3)} N·m` +
      `（STM32 実測 max_torque_nm が届いていればそれを使う。未接続時のみ既定 ${MAX_TARGET_TORQUE_NM} N·m。超えても丸められる）`,
    digits: 3,
  },
  // ★2026-08-22: 「速度メータの目盛り」フィールドは廃止。速度メータ（`SpeedGauge.tsx`）の
  // 目盛りは、速度制御・トルク制御どちらでも常に STM32 実測 `LIMITS.max_speed_m_s` に
  // 固定されるようになったため、`maxSpeed` を経由してユーザーが調整する意味が無くなった
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

const CAMERA_FIELDS: Field[] = [
  {
    key: 'camHeight',
    label: '取付高さ',
    unit: 'm',
    note: '前カメラの路面からの高さ。既定値は vehicle.toml の実測位置。レンズ光学中心とはズレうるので映像のガイド線を見ながら追い込む',
    digits: 3,
  },
  // 俯角（取付角度）はここに無い。一度ネジ止めしたら変わらない固定値なので
  // vehicle.toml の sensors.cam_front.pitch を直接使う（store/ui.ts 参照）
]

/**
 * 項目が増えて縦に長くなりすぎたので、テーマごとにタブで分けてある（2026-08-24）。
 * 「既定値に戻す」等のヘッダはタブの外——`DrivingSettings` 全体（走行・安全・
 * カメラ校正のどのタブの値も含む）に効くので、特定のタブだけの操作ではない。
 */
const SETTINGS_TABS = [
  { id: 'drive', label: '走行' },
  { id: 'safety', label: '安全' },
  { id: 'camera', label: 'カメラ' },
] as const
type SettingsTab = (typeof SETTINGS_TABS)[number]['id']

export function SettingsPanel({ ch }: { ch: ControlChannel | null }) {
  const [tab, setTab] = useState<SettingsTab>('drive')
  const settings = useUi((s) => s.settings)
  const settingsDefault = useUi((s) => s.settingsDefault)
  const setSettings = useUi((s) => s.setSettings)
  const resetSettings = useUi((s) => s.resetSettings)
  const saveCurrentAsDefault = useUi((s) => s.saveCurrentAsDefault)
  const pathGuide = useUi((s) => s.pathGuide)
  const set = useUi((s) => s.set)
  // estop_active/drive_power_locked と同じく、これは `/ws/control` の status
  // （イベント発生時にしかbroadcastされない）ではなく `/ws/telemetry` 経由で
  // 継続更新される LinkDiag（8Hz、`bus/live.ts`）から読む。status から読むと
  // 実際は変わっているのにチェックボックスが更新されず、リロードするまで
  // 反映されないように見える（2026-08-19 実機で発覚）
  const { link } = useNumbers()
  const tcEnabled = link?.tc_enabled ?? null
  const tvEnabled = link?.tv_enabled ?? null
  const wheelLiftGuardEnabled = link?.wheel_lift_guard_enabled ?? null
  const autoStopMarginCm = link?.auto_stop_margin_cm ?? null
  // fan と違い `/ws/control` の status（イベント発生時のブロードキャスト）から読む。
  // 20Hz の `/ws/telemetry` に載せるほど頻繁に変わらない値なので tc_enabled 等とは事情が違う
  const cam = useUi((s) => s.cameraConfig)
  // 車両の物理的な上限値（`LIMITS` パケット由来。★v0.11）。`link`（LinkDiag、
  // `/ws/telemetry` 8Hz）から読む理由は tc_enabled 等と同じ（上のコメント参照）
  const range = effectiveRange({
    max_speed_m_s: link?.max_speed_m_s ?? null,
    max_accel_m_s2: link?.max_accel_m_s2 ?? null,
    max_torque_nm: link?.max_torque_nm ?? null,
    max_steer_rad: link?.max_steer_rad ?? null,
  })

  return (
    <div className="settings">
      <div className="settings-head">
        <button onClick={resetSettings}>既定値に戻す</button>
        {/* ★2026-08-22: 「既定値」はソースコードの固定値ではなく、ここで上書きできる
            （`store/ui.ts` の `settingsDefault`、`localStorage` に保存）。以前は
            「今使っている値を既定値にしたい」と言われるたびにソースを直接書き換えて
            いたが、実機の現在値を見られないまま推測することになっていた */}
        <button onClick={saveCurrentAsDefault} title="今の設定値を「既定値に戻す」の戻り先として保存する">
          現在の値を既定値として保存
        </button>
      </div>

      <div className="seg settings-tabs">
        {SETTINGS_TABS.map((t) => (
          <button key={t.id} className={tab === t.id ? 'on' : ''} onClick={() => setTab(t.id)}>
            {t.label}
          </button>
        ))}
      </div>

      {tab === 'drive' && (
        <>
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
            <SettingGroup title="トルク制御" fields={TORQUE_FIELDS} settings={settings} range={range} defaults={settingsDefault} onChange={setSettings} />
          ) : (
            <SettingGroup title="速度制御" fields={SPEED_FIELDS} settings={settings} range={range} defaults={settingsDefault} onChange={setSettings} />
          )}

          <SettingGroup title="舵" fields={STEER_FIELDS} settings={settings} range={range} defaults={settingsDefault} onChange={setSettings} />
          <SettingGroup title="ブレーキ" fields={BRAKE_FIELDS} settings={settings} range={range} defaults={settingsDefault} onChange={setSettings} />
        </>
      )}

      {tab === 'safety' && (
        <>
          {/* ここだけは GUI の調整値ではなく「STM32 の機能を許可するかどうか」。
              制動力自体は STM32 側の固定値で、GUI からは変えられない（v0.7）。
              ★v0.12: 停止距離はもう固定値ではなく `v・t_delay + v²/(2・a_max) + margin`
              （速度が上がるほど早い段階で作動する）。GUI から変えられるのは
              margin をcm単位で直接指定する連続値だけ（`CONFIG_SET` param_id=0x0060）。
              当初は3段階enumの予定だったが実機投入前にSTM32側がcm直接指定へ変更した。
              表示値は STM32 の CONFIG_ACK 経由のサーバ真値（`link.auto_stop_margin_cm`）。
              未確定（起動直後でまだ CONFIG_ACK が届いていない）間は null になる */}
          <section className="settings-group">
            <h3>自動停止（超音波+LiDAR）</h3>
            <label className="settings-checkbox">
              <input
                type="checkbox"
                checked={settings.autoStop}
                onChange={(e) => setSettings({ autoStop: e.target.checked })}
              />
              走行速度に応じた停止距離＋余裕（margin）未満で STM32 に自動停止させる
            </label>
            <div className="settings-row" title="停止距離の余裕（margin）。速度に応じて伸びる物理項とは別に、この分だけ手前で止める">
              <div className="settings-row-head">
                <span className="label">停止余裕（margin）{autoStopMarginCm === null && '（未確認）'}</span>
                <b>{(autoStopMarginCm ?? 15).toFixed(0)}</b>
                <span className="unit">cm</span>
              </div>
              <input
                type="range"
                min={0}
                max={100}
                step={1}
                value={autoStopMarginCm ?? 15}
                disabled={autoStopMarginCm === null}
                onChange={(e) => ch?.setAutoStopMargin(Number(e.target.value))}
              />
            </div>
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

          <SettingGroup title="操作" fields={MISC_FIELDS} settings={settings} range={range} defaults={settingsDefault} onChange={setSettings} />
        </>
      )}

      {tab === 'camera' && (
        <>
          {/* capture側(camera_node)のFPS上限・後方カメラON/OFF・GUI配信頻度。
              TC/TVと同じく状態はサーバ真値（`status.camera_config`）。
              前カメラの上限は目安——カメラを使う自動運転（`line_trace`/`ftg_cam`）が
              engage 中はサーバ側が無視して最大まで引き上げる（`auto_override`） */}
          <section className="settings-group">
            <h3>カメラ</h3>
            <label className="settings-checkbox">
              <input
                type="checkbox"
                checked={cam?.rear_enabled ?? true}
                disabled={cam === null}
                onChange={(e) => ch?.setCamera({ rearEnabled: e.target.checked })}
              />
              後方カメラの映像取得{cam === null && '（未確認）'}
            </label>
            <div
              className="settings-row"
              title={
                cam?.auto_override
                  ? '自動運転（ライントレース等）が走行中は、この上限を無視して最大まで引き上がっている'
                  : '自動運転がカメラを使うモードで engage されている間は、この上限を無視して最大まで引き上がる'
              }
            >
              <div className="settings-row-head">
                <span className="label">前カメラ 取得上限</span>
                <b>{(cam?.front_cap_hz ?? 30).toFixed(0)}</b>
                <span className="unit">fps</span>
              </div>
              <input
                type="range"
                min={1}
                max={30}
                step={1}
                value={cam?.front_cap_hz ?? 30}
                disabled={cam === null}
                onChange={(e) => ch?.setCamera({ frontCapHz: Number(e.target.value) })}
              />
              {cam?.auto_override && (
                <span className="unit">実効 {cam.front_fps_effective.toFixed(0)}fps（自動運転中）</span>
              )}
            </div>
            <div className="settings-row" title="後方カメラは自動運転では使わない（GUI表示とロギング専用）ので上書きは無い">
              <div className="settings-row-head">
                <span className="label">後カメラ 取得上限</span>
                <b>{(cam?.rear_cap_hz ?? 10).toFixed(0)}</b>
                <span className="unit">fps</span>
              </div>
              <input
                type="range"
                min={1}
                max={30}
                step={1}
                value={cam?.rear_cap_hz ?? 10}
                disabled={cam === null || cam?.rear_enabled === false}
                onChange={(e) => ch?.setCamera({ rearCapHz: Number(e.target.value) })}
              />
            </div>
            <div className="settings-row" title="ブラウザへ送るJPEGの頻度。上げるとWi-Fi帯域を余計に使う（2026-08-24 実機で前後同時30fpsでも問題ないことを確認済み）">
              <div className="settings-row-head">
                <span className="label">GUIへの配信</span>
                <b>{(cam?.gui_hz ?? 30).toFixed(0)}</b>
                <span className="unit">fps</span>
              </div>
              <input
                type="range"
                min={1}
                max={30}
                step={1}
                value={cam?.gui_hz ?? 30}
                disabled={cam === null}
                onChange={(e) => ch?.setCamera({ guiHz: Number(e.target.value) })}
              />
            </div>
          </section>

          {/* 進路ガイド（前後カメラ映像への重ね描き）の校正。CameraView.tsx の drawGuide が
              height/pitch を使う。hfov はレンズ公称値で固定なのでここには出さない。
              高さのスライダ（CAMERA_FIELDS）は前カメラの camHeight のみ——後カメラは
              固定値（VEHICLE.camRear.height）を使い調整UIを持たない（2026-08-21） */}
          <section className="settings-group">
            <h3>進路ガイド校正</h3>
            <label className="settings-checkbox">
              <input type="checkbox" checked={pathGuide} onChange={(e) => set({ pathGuide: e.target.checked })} />
              前後カメラに進路ガイドを重ねる（校正前・暫定）
            </label>
            {CAMERA_FIELDS.map((f) => (
              <SettingRow key={f.key} field={f} settings={settings} range={range} defaults={settingsDefault} onChange={setSettings} />
            ))}
          </section>
        </>
      )}
    </div>
  )
}

function SettingGroup({
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
