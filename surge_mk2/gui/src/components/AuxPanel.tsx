/**
 * 補機の帯 — 層 B（走行中に横目で見る）。プロトコル v0.5 で増えた操作をまとめる。
 *
 * ## 何を操作でき、何を「見る」だけにしてあるか
 *
 * - **灯火モードとブレーキの強さはここで設定する。** どちらも「状態」なので、
 *   クリックで変えても取り残しが起きない
 * - **ブレーキ・クラクション・パッシングは表示だけ。** この3つは
 *   「押している間だけ立てる」意味論（`useDriving.ts` の表を参照）で、
 *   マウスの押しっぱなしで実現すると、ボタンの外でカーソルを離した／
 *   ドラッグしたまま画面が切り替わった場合に **`horn` が立ったまま残る**。
 *   v0.5 のクラクションは立てている間ずっと鳴るので、これは鳴りっぱなしを意味する。
 *   キーとパッドなら keyup / pressed=false が確実に来るので、そちらに寄せた
 *
 * ## 実トルクを並べる理由
 *
 * v0.5 から `torque_cmd` は **正 = 駆動 / 負 = 制動**で、制動中は掛けている
 * トルクが負値で入る（v0.4 は停車保持中も 0 だった）。
 * **スライダの設定値と、実際に効いている制動トルクは別物**なので必ず並べる。
 * ここが 0 のままならブレーキが効いていない。
 */
import { useNumbers } from '../bus/live'
import {
  LIGHT_CYCLE, LIGHT_LABEL, MAX_BRAKE_TORQUE_NM, MIN_BRAKE_TORQUE_NM, useUi,
} from '../store/ui'

/** スライダの刻み。STM32 の分解能（0.0001 N·m/LSB）より粗くしてある */
const STEP_NM = 0.001

export function AuxPanel() {
  const n = useNumbers()
  const ui = useUi()
  const vs = n.vs

  // [RL, RR] の平均。**片輪だけ見ると LSD 的な差で誤読する**
  const tq = vs ? (vs.torque_cmd[0] + vs.torque_cmd[1]) / 2 : null
  const load = tq == null ? 'dim' : tq < -1e-6 ? '制動' : tq > 1e-6 ? '駆動' : '無負荷'

  return (
    <div className="aux">
      <section className="aux-lights">
        <span className="label">灯火</span>
        <div className="seg">
          {LIGHT_CYCLE.map((m) => (
            <button
              key={m}
              className={ui.lightMode === m ? 'on' : ''}
              onClick={() => ui.set({ lightMode: m })}
            >
              {LIGHT_LABEL[m]}
            </button>
          ))}
        </div>
        {/* v0.4 の light=1 は全光量だったが、v0.5 の 1 は DAYTIME（減光）。
            値の意味が変わっているので画面にも書いておく */}
        <span className="note">L キー / パッド △ で送り。DAY は減光（duty 0.1）</span>
      </section>

      <section className="aux-brake">
        <span className="label">ブレーキ強さ</span>
        <div className="aux-row">
          <input
            type="range"
            min={MIN_BRAKE_TORQUE_NM}
            max={MAX_BRAKE_TORQUE_NM}
            step={STEP_NM}
            value={ui.brakeTorque}
            onChange={(e) => ui.set({ brakeTorque: Number(e.target.value) })}
          />
          <b>{ui.brakeTorque.toFixed(3)}</b>
          <span className="unit">N·m/輪</span>
        </div>
        {/* **設定値ではなく実際に効いている量。** ここが動かなければ効いていない */}
        <span className="note">
          実トルク{' '}
          <b className={tq != null && tq < -1e-6 ? 'lv-warn' : ''}>
            {tq == null ? '—' : `${tq.toFixed(3)} N·m`}
          </b>{' '}
          <span className="dim">{load}</span>（後輪 RL/RR 平均・指令値）
        </span>
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
        {/* ARM しないと cmd を送らない ＝ 灯火もホーンも効かない。
            気づかずに「壊れている」と判断するのを防ぐため明示する */}
        <span className="note">
          {ui.armRequested ? '押している間だけ効く' : 'ARM 保持中のみ有効（未 ARM）'}
        </span>
      </section>
    </div>
  )
}
