/**
 * 運転中に共通の操作 — 灯火・サイドブレーキ。**ラジコンタブでも自動運転
 * タブでも同じものを操作するので、タブごとに複製せずタブ行に1つだけ置く。**
 *
 * 「運転できるタブのときだけ表示」（`App.tsx` の `DRIVING_TABS`）。
 *
 * ファンは `SettingsPanel.tsx`（安全タブ）、ウィンカー・ハザードは
 * `components/rc/WinkerHazard.tsx`（ラジコンビュー専用）にある。
 *
 * ## サイドブレーキ
 *
 * `braking`/`horning` のような「押している間だけ」ではなく、**ON/OFF のトグル**
 * （駐車ブレーキと同じ「かけたら手を離せる」操作感、`useDriving.ts`/`store/ui.ts`
 * 参照）。STM32 は**速度に関わらず即座に**後輪を位置制御へ切り替えて固定するので、
 * 走行中に ON にするとロックし得る（ハンドブレーキターン等の用途のため制限はしない）。
 *
 * ## D/R レンジ・MT ギア
 *
 * 画面ボタンは無い。現在ギアの表示は速度メータ中央のバッジ（`SpeedGauge.tsx`）。操作は
 * パッドの L1（ダウン）/R1（アップ）とキーボードの←（ダウン）/→（アップ）のみ
 * （`useDriving.ts` の `shiftGear()`、状態は `ui.gear`/`ui.mtGear`）。
 */
import { useNumbers } from '../bus/live'
import { LIGHT_CYCLE, LIGHT_LABEL, useUi } from '../store/ui'

export function DriveControls() {
  const ui = useUi()
  const vs = useNumbers().vs

  return (
    <div className="drive-controls">
      <div className="rc-ctl">
        <span className="label">灯火</span>
        <div className="seg">
          {LIGHT_CYCLE.map((m) => (
            <button
              key={m}
              className={ui.lightMode === m ? 'on' : ''}
              onClick={() => ui.set({ lightMode: m })}
              title="L キー / パッド十字キー上 でも送れる。DAY は減光（duty 0.1）"
            >
              {LIGHT_LABEL[m]}
            </button>
          ))}
        </div>
      </div>

      <div className="rc-ctl">
        <span className="label">サイドブレーキ</span>
        <div className="seg">
          <button
            className={ui.sideBrakeRequested ? 'on' : ''}
            onClick={() => ui.set({ sideBrakeRequested: true })}
          >
            ON
          </button>
          <button className={ui.sideBrakeRequested ? '' : 'on'}
                  onClick={() => ui.set({ sideBrakeRequested: false })}>OFF</button>
        </div>
        {/* 「要求している」と「今まさに固定できているか」は別物。実際に位置制御へ
            切り替わって固定できたときだけ `TELEMETRY.flags` bit17 が立つ */}
        <span className={`pill ${vs?.side_brake_active ? 'lv-warn' : 'dim'}`}>
          {vs?.side_brake_active ? '固定中' : ui.sideBrakeRequested ? '要求中…' : '解除'}
        </span>
      </div>
    </div>
  )
}
