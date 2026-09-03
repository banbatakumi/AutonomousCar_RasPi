/**
 * 運転中に共通の操作 — 灯火・サイドブレーキ。**ラジコンタブでも自動運転
 * タブでも同じものを操作するので、タブごとに複製せずタブ行に1つだけ置く。**
 *
 * 2026-08-17: `RcBar.tsx`（ラジコンタブ）と `AuxPanel.tsx`（自動運転タブ）に
 * それぞれ独立していた灯火トグルを統合し、ファンの自動/手動＋デューティを新設した。
 * `DriveHint` と同じ「運転できるタブのときだけ表示」（`App.tsx` の `DRIVING_TABS`）。
 *
 * ## ファン・ウィンカー・ハザードを移設した（2026-09-03）
 *
 * タブ行がボタンで過密になったため、他の画面へ移した。
 * **ファン**は運転中に頻繁に操作するものではないので `SettingsPanel.tsx`
 * （安全タブ）へ。**ウィンカー・ハザード**は実車のダッシュボードに近い位置——
 * ラジコンビューの舵角計・G メータ（`RcView.tsx` の `.rc-cam-gauges`）の
 * それぞれ横に、点滅時だけ光る緑矢印（上段）と矢印ボタン（下段）を積んだ
 * 小さい列（`.winker-side`）、右下に赤い三角のハザードボタン
 * （`components/rc/WinkerHazard.tsx`）——に置き直した。自動運転タブには
 * 灯火と同様この移設後のウィンカー操作は無い（運転はラジコンビューでしか
 * 楽しまないため、`.rc-cam-gauges` はラジコンビュー専用）。
 *
 * ## サイドブレーキ（v0.13、2026-08-28 追加）
 *
 * `braking`/`horning` のような「押している間だけ」ではなく、**ON/OFF のトグル**
 * （駐車ブレーキと同じ「かけたら手を離せる」操作感、`useDriving.ts`/`store/ui.ts`
 * 参照）。STM32 は**速度に関わらず即座に**後輪を位置制御へ切り替えて固定するため、
 * 走行中の誤操作を防ぐ目的で、停止中（`vs.stopped`）以外は ON ボタンを無効化する。
 *
 * ## D/R レンジ・MT ギア（v0.14〜v0.17）は画面ボタンを廃止した（v0.18）
 *
 * ゲームパッド（DUALSHOCK4）の R2/L2 を実車ペダル配置（R2=アクセル、L2=ブレーキ）に
 * したことで、R2 単体では前進/後退の向きを表せなくなり、その向きを選ぶ手段として
 * `ui.gear`/`ui.mtGear`（`useDriving.ts` の `gearSign`）を導入した。以前はここに
 * 画面ボタン（D/R の2択、MT モード中は R,1〜5 の6択）があったが、**v0.18 で廃止し、
 * 現在ギアの表示は速度メータ中央のバッジ（`SpeedGauge.tsx`）へ移した。** 操作は
 * パッドの L1（ダウン）/R1（アップ）とキーボードの←（ダウン）/→（アップ）に一本化
 * している（`useDriving.ts` の `shiftGear()`）——画面からのタップ操作は今は無い。
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
              title="L キー / パッド △ でも送れる。DAY は減光（duty 0.1）"
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
