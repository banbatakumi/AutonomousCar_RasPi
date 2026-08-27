/**
 * 補機の帯 — 層 B（走行中に横目で見る）。プロトコル v0.5 で増えた操作をまとめる。
 *
 * ## 何を操作でき、何を「見る」だけにしてあるか
 *
 * - **灯火モードはタブ行の `DriveControls` へ移設した**（2026-08-17）。ラジコンタブの
 *   `RcBar` にも同じトグルが重複していたため、1箇所（タブ行、運転できるタブ共通）に統合した
 * - **ブレーキ強さは設定パネルに移した**（2026-08-10）。最高速度などと同じ「一度決めたら
 *   基本触らない」チューニング値として扱う。ここには実際に効いている量だけ出す
 *   （設定パネルは 2026-08-11 にタブからラジコンタブの ⚙ ドロワーへ移動した）
 * - **灯火・クラクション・パッシングは ARM していなくても効く**（2026-08-11）。
 *   未 ARM の間は `mode=DISARM` / `arm=false` の cmd を送り、灯火とホーンのビットだけ
 *   載せている（`useDriving.ts`）。ブレーキだけはモータが励磁されていないと意味が無い
 * - **ブレーキ・クラクション・パッシングは表示だけ。** この3つは
 *   「押している間だけ立てる」意味論（`useDriving.ts` の表を参照）で、
 *   マウスの押しっぱなしで実現すると、ボタンの外でカーソルを離した／
 *   ドラッグしたまま画面が切り替わった場合に **`horn` が立ったまま残る**。
 *   v0.5 のクラクションは立てている間ずっと鳴るので、これは鳴りっぱなしを意味する。
 *   キーとパッドなら keyup / pressed=false が確実に来るので、そちらに寄せた
 * - **サイドブレーキ（v0.13）は `DriveControls.tsx`（タブ行）に置いた。** 灯火・
 *   ファンと同じく「ラジコンタブでも自動運転タブでも同じものを操作する」ため、
 *   ここに複製しない（2026-08-17 の灯火統合と同じ理由）
 *
 * ## 実トルクを出す理由
 *
 * v0.5 から `torque_cmd` は **正 = 駆動 / 負 = 制動**で、制動中は掛けている
 * トルクが負値で入る（v0.4 は停車保持中も 0 だった）。
 * **設定パネルの目標値と、実際に効いている制動トルクは別物**なので必ず出す。
 * ここが 0 のままならブレーキが効いていない。
 */
import { useNumbers } from '../bus/live'
import { useUi } from '../store/ui'

export function AuxPanel() {
  const n = useNumbers()
  const ui = useUi()
  const vs = n.vs

  // [RL, RR] の平均。**片輪だけ見ると LSD 的な差で誤読する**
  const tq = vs ? (vs.torque_cmd[0] + vs.torque_cmd[1]) / 2 : null
  const load = tq == null ? 'dim' : tq < -1e-6 ? '制動' : tq > 1e-6 ? '駆動' : '無負荷'

  return (
    <div className="aux">
      <section className="aux-brake">
        <span className="label">ブレーキ実トルク</span>
        <div className="aux-row">
          <b className={tq != null && tq < -1e-6 ? 'lv-warn' : ''}>
            {tq == null ? '—' : tq.toFixed(3)}
          </b>
          <span className="unit">N·m/輪</span>
          <span className="dim">{load}</span>
        </div>
      </section>

      <section className="aux-state">
        <span className="label">操作</span>
        <div className="aux-row">
          <span className={`pill ${ui.braking ? 'lv-warn' : 'dim'}`} title="Space / パッド L1">
            ブレーキ
          </span>
          <span className={`pill ${ui.horning ? 'lv-warn' : 'dim'}`} title="H / パッド A">
            ホーン
          </span>
          <span className={`pill ${ui.passing ? 'lv-warn' : 'dim'}`} title="P / パッド X">
            パッシング
          </span>
        </div>
      </section>

      {/* 自動停止（v0.7）。**灯火と同じ「状態」なのでここで切り替えてよい**
          （押しっぱなし系と違い、カーソルが外れても取り残されない）。
          設定パネルの `autoStop` と同じ値を読み書きするので、どちらで変えても保存される */}
      <section className="aux-autostop">
        <span className="label">自動停止</span>
        <div className="aux-row">
          <div className="seg">
            <button className={ui.settings.autoStop ? 'on' : ''}
                    onClick={() => ui.setSettings({ autoStop: true })}>ON</button>
            <button className={ui.settings.autoStop ? '' : 'on'}
                    onClick={() => ui.setSettings({ autoStop: false })}>OFF</button>
          </div>
          {/* 「許可している」と「今まさに止めている」は別物。**効いている間だけ赤く出す** */}
          <span className={`pill ${vs?.auto_stop_active ? 'lv-bad' : 'dim'}`}>
            {vs?.auto_stop_active ? '作動中' : '待機'}
          </span>
        </div>
      </section>
    </div>
  )
}
