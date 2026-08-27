/**
 * 運転中に共通の操作 — 灯火・ファン・サイドブレーキ。**ラジコンタブでも自動運転
 * タブでも同じものを操作するので、タブごとに複製せずタブ行に1つだけ置く。**
 *
 * 2026-08-17: `RcBar.tsx`（ラジコンタブ）と `AuxPanel.tsx`（自動運転タブ）に
 * それぞれ独立していた灯火トグルを統合し、ファンの自動/手動＋デューティを新設した。
 * `DriveHint` と同じ「運転できるタブのときだけ表示」（`App.tsx` の `DRIVING_TABS`）。
 *
 * ## 実測 rpm を必ず出す（2026-08-17）
 *
 * **Pi5純正ファンは、温度が危険域（実機で約75°C付近を確認）を超えると、
 * 手動指定を無視してハードウェア側が強制的に全開へ上書きする**（サーマル
 * プロテクション。安全機構なのでソフトウェアからは無効化しない・すべきでない）。
 * この上書きは `duty`（指定値）には出ず sysfs の実際の PWM だけが変わるので、
 * 実測 `rpm` を表示しないと「手動なのに勝手に速くなった」が原因不明に見える。
 *
 * ## サイドブレーキ（v0.13、2026-08-28 追加）
 *
 * `braking`/`horning` のような「押している間だけ」ではなく、**ON/OFF のトグル**
 * （駐車ブレーキと同じ「かけたら手を離せる」操作感、`useDriving.ts`/`store/ui.ts`
 * 参照）。STM32 は**速度に関わらず即座に**後輪を位置制御へ切り替えて固定するため、
 * 走行中の誤操作を防ぐ目的で、停止中（`vs.stopped`）以外は ON ボタンを無効化する。
 */
import { useNumbers } from '../bus/live'
import { LIGHT_CYCLE, LIGHT_LABEL, useUi } from '../store/ui'
import type { ControlChannel } from '../ws/control'

export function DriveControls({ ch }: { ch: ControlChannel | null }) {
  const ui = useUi()
  const fan = ui.fan
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
              title="L キー / パッド △ でも送れる。DAY は減光（duty 0.1）"
            >
              {LIGHT_LABEL[m]}
            </button>
          ))}
        </div>
      </div>

      <div className="rc-ctl">
        <span className="label">ファン</span>
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
          <div className="rc-fan-duty">
            <input
              type="range"
              min={0}
              max={1}
              step={0.05}
              value={fan.duty}
              onChange={(e) => ch?.setFan({ duty: Number(e.target.value) })}
            />
            <b>{Math.round(fan.duty * 100)}%</b>
          </div>
        )}
        {fan?.rpm != null && (
          <span
            className="rc-fan-rpm"
            title="実測回転数。高温で保護動作が入ると、指定値と食い違ってここだけ上がる"
          >
            {fan.rpm}rpm
          </span>
        )}
      </div>

      <div className="rc-ctl">
        <span className="label">サイドブレーキ</span>
        <div className="seg">
          <button
            className={ui.sideBrakeRequested ? 'on' : ''}
            disabled={!ui.sideBrakeRequested && vs != null && !vs.stopped}
            title={vs != null && !vs.stopped ? '走行中はONにできません（停止してから）' : ''}
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
