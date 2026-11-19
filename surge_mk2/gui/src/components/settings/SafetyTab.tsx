/** 設定パネルの「安全（自動停止・TC/TV/ABS・片輪浮き対策・ファン・ARM タイムアウト）」タブ。`SettingsPanel.tsx` から分けた。 */
import type { ControlChannel } from '../../ws/control'
import { SettingGroup, controlParamsLabel, useDriveSettingsEditor, MISC_FIELDS } from './fields'
import { useNumbers } from '../../bus/live'
import { useUi } from '../../store/ui'

export function SafetyTab({ ch }: { ch: ControlChannel | null }) {
  const { settings, settingsDefault, setDriveSettings, range } = useDriveSettingsEditor(ch)
  const fan = useUi((s) => s.fan)
  // estop_active/drive_power_locked と同じく、これは `/ws/control` の status
  // （イベント発生時にしかbroadcastされない）ではなく `/ws/telemetry` 経由で
  // 継続更新される LinkDiag（8Hz、`bus/live.ts`）から読む。status から読むと
  // 実際は変わっているのにチェックボックスが更新されず、リロードするまで
  // 反映されないように見える
  const { link } = useNumbers()
  const tcEnabled = link?.tc_enabled ?? null
  const tvEnabled = link?.tv_enabled ?? null
  const wheelLiftGuardEnabled = link?.wheel_lift_guard_enabled ?? null
  const absEnabled = link?.abs_enabled ?? null
  const brakeHoldEnabled = link?.brake_hold_enabled ?? null
  const autoStopMarginCm = link?.auto_stop_margin_cm ?? null

  return (
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
        <h3>ローンチコントロール</h3>
        <label className="settings-checkbox">
          <input
            type="checkbox"
            checked={settings.launchControl}
            onChange={(e) => setDriveSettings({ launchControl: e.target.checked })}
          />
          止まった状態でブレーキとアクセルを両方踏み、ブレーキを離すと全開で発進する
        </label>
      </section>
      <section className="settings-group">
        <h3>自動停止（超音波+LiDAR）</h3>
        <label className="settings-checkbox">
          <input
            type="checkbox"
            checked={settings.autoStop}
            onChange={(e) => setDriveSettings({ autoStop: e.target.checked })}
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
          （TC/TVは★v0.8、片輪浮き対策は★v0.9、ABSは★v0.15。片輪浮き対策はTC/TV本体とは独立した別機構）。
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
        {/* ABS（★v0.15）。制動モード（ブレーキ・自動停止・フェイルセーフ）だけに効く */}
        <label className="settings-checkbox">
          <input
            type="checkbox"
            checked={absEnabled ?? true}
            disabled={absEnabled === null}
            onChange={(e) => ch?.setAbs(e.target.checked)}
          />
          ABS{absEnabled === null && '（未確認）'}
        </label>
        {/* ブレーキホールド（★v0.19）。制動して止まったら後輪の位置保持へ移り、坂でも動かない。
            ブレーキ・自動停止・フェイルセーフのどの制動でも働く */}
        <label className="settings-checkbox">
          <input
            type="checkbox"
            checked={brakeHoldEnabled ?? true}
            disabled={brakeHoldEnabled === null}
            onChange={(e) => ch?.setBrakeHold(e.target.checked)}
          />
          ブレーキホールド（停止後に後輪を位置保持）{brakeHoldEnabled === null && '（未確認）'}
        </label>
        {/* ★v0.16: TC・ABS・TV の調整パラメータ（vehicle.toml [control]）が STM32 に入っているか。
            io_node が CONFIG_ACK の「実際に入った値」と突き合わせた結果をそのまま出す */}
        <p className="dim">
          制御パラメータ: {controlParamsLabel(link?.control_params_status)}
          {(link?.control_params_drift ?? 0) > 0 && `（送り直し ${link?.control_params_drift}回）`}
          {link?.control_params_problems?.map((t) => (
            <span key={t} className="badge-bad">
              <br />
              {t}
            </span>
          ))}
        </p>
      </section>

      {/* 運転中に頻繁に触る
          ものではなく、走行アシストと同じ「STM32/RasPi本体の保護機構」の
          仲間としてここに置く */}
      <section className="settings-group">
        <h3>ファン（RasPi5冷却）</h3>
        <div className="seg">
          {(['auto', 'manual'] as const).map((m) => (
            <button
              key={m}
              className={fan?.mode === m ? 'on' : ''}
              disabled={fan === null || (m === 'manual' && !fan.available)}
              onClick={() => ch?.setFan({ mode: m })}
              title={
                m === 'manual' && fan && !fan.available
                  ? 'この機体では手動調整に対応していません'
                  : undefined
              }
            >
              {m === 'auto' ? '自動' : '手動'}
            </button>
          ))}
        </div>
        {fan?.mode === 'manual' && fan.available && (
          <div className="settings-row">
            <div className="settings-row-head">
              <span className="label">デューティ</span>
              <b>{Math.round(fan.duty * 100)}</b>
              <span className="unit">%</span>
            </div>
            <input
              type="range"
              min={0}
              max={1}
              step={0.05}
              value={fan.duty}
              onChange={(e) => ch?.setFan({ duty: Number(e.target.value) })}
            />
          </div>
        )}
        {fan?.rpm != null && (
          <p
            className="dim"
            title="実測回転数。高温で保護動作が入ると、指定値と食い違ってここだけ上がる"
          >
            実測 {fan.rpm}rpm
          </p>
        )}
      </section>

      <SettingGroup title="操作" fields={MISC_FIELDS} settings={settings} range={range} defaults={settingsDefault} onChange={setDriveSettings} />
    </>
  )
}
