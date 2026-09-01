/**
 * 自動運転の操作盤 — **どのモードで走るかを選び、engage する。**
 *
 * ## モードの一覧をここに書かない
 *
 * 選択肢もパラメータのスライダも、サーバから降ってくる `status.auto.catalog`
 * （`raspi/auto/registry.py`）だけで組み立てている。**planner を1つ足しても
 * このファイルは 1 行も変わらない。** モード名を GUI 側にも書くと、増やすたびに
 * 2箇所を直すことになり、いつか片方だけ古くなる。
 *
 * ⚠ 例外が3つだけある——`ftg_cam`（セグメンテーション走行）と `e2e_lidar`
 * （E2E LiDAR走行）が使うモデルの選択、`follow_object`（対象追従）の
 * ROI選択状態の表示（下記）。この3つの id 文字列だけは
 * `raspi/auto/follow_the_gap_cam.py` / `raspi/auto/e2e_lidar.py` /
 * `raspi/auto/follow_object.py` と直接対応させて GUI 側に書いてある。
 *
 * ## engage の状態を GUI 側で持たない
 *
 * ボタンを押したら `setAuto` を投げるだけで、表示は必ずサーバの `status` を見る。
 * 「押したから走っているはず」で描くと、サーバが拒否したとき（モード未選択・
 * E-Stop・操縦権の喪失）に**画面だけ走っていることになる**。
 *
 * ## engage ≠ 走る
 *
 * engage は「自律に任せてよい」という意思表示で、実際に車が動くには**人間が
 * ARM を保持している**必要がある（`Enter` または ARM ボタン）。この2つを1つの
 * ボタンにまとめないのは、自律走行の停止手段を人間側に残しておくため。
 *
 * ## モデル選択もここに置く（2026-08-28）
 *
 * 以前は設定ドロワーのカメラタブに置いていたが、**「どのモードで走るか」と
 * 「そのモードが使うモデル」は同じ意思決定の一部**なので分離すると迷う。
 * `ftg_cam`/`e2e_lidar` を選んだときだけ、そのモード専用のモデル選択
 * ドロップダウンをこのパネル内に出す。一覧の取得（`camModelList`/
 * `e2eModelList`）は `LogView.tsx` と同じくマウント時に一度要求しておく。
 *
 * ## モード選択はドロップダウンに（2026-08-28）
 *
 * planner の数が増えてボタン行が窮屈になったため、1つ選べば足りるボタン行
 * ではなく `<select>` にした。未選択に戻す操作もボタンのトグルではなく
 * 「（未選択）」を選ぶ形になる。
 *
 * ## 構造を作り直した（2026-08-28、指摘による2回目の改訂）
 *
 * 車体図と横並びになって幅が2/3に減ったところ、旧来の「3列固定グリッド」
 * （走らせ方／engage／判断を横に3分割）が窮屈になり、判断の数値
 * （`auto-metrics`）が幅の途中で折り返して**数値と単位が別の行に分離する**
 * 崩れ方をした。**縦積みの1カラム構成**に作り直し、数値は「ラベル＋値」を
 * 1つのグリッドセルにまとめて絶対に分離しないようにした（`.auto-stat`）。
 * パラメータも `minmax(220px,1fr)` だと2列で頭打ちになっていたので
 * `minmax(150px,1fr)` に詰め、無理に2列に揃えず幅なりに流れるようにした。
 *
 * パネル全体に `max-height` を付け、車体図（`AutoView.tsx` の `.auto-car`）と
 * 同じ行の高さをここで決める——`.auto-car` はこの高さいっぱいまで車体図を
 * 拡大する作りなので、ここの高さが車体図の実際の大きさを決めている
 * （`styles.css` の `.auto-car .drivepanel-wrap` 参照）。パラメータを開いて
 * 縦に伸びても、上段カメラ帯の高さやこの行全体を押し広げず**パネル内だけで
 * スクロール**する。
 *
 * ## 「判断」欄も planner ごとに出し分ける（2026-08-29）
 *
 * `gap_start_deg`/`gap_end_deg`/`nearest` はギャップ探索系 planner の判断根拠で、
 * `e2e_lidar`/`line_trace` はそもそも書かない（`AutoState` の既定値0.0のまま）。
 * 無条件で出すと「0〜0° ギャップ」のような意味の無い数値が並んでしまうため
 * （指摘による）、`catalog[].stats`（`raspi/auto/base.py` の `Planner.stats`）で
 * 宣言された分だけ出す。他の宣言と同じくモード id は GUI 側に書かない。
 */
import { useEffect } from 'react'
import { useNumbers } from '../bus/live'
import { RAD2DEG, mps } from '../format'
import { useUi } from '../store/ui'
import type { ControlChannel } from '../ws/control'
import { ParamSliders } from './ParamSliders'

/** planner の判断がこれ以上古ければ「planning_node が落ちている」と見なす */
const AUTO_STALE_MS = 600

export function AutoPanel({ ch }: { ch: ControlChannel | null }) {
  const ui = useUi()
  const auto = ui.auto
  const n = useNumbers()

  const camModel = useUi((s) => s.camModel)
  const camModelFiles = useUi((s) => s.camModelFiles)
  const e2eModel = useUi((s) => s.e2eModel)
  const e2eModelFiles = useUi((s) => s.e2eModelFiles)
  // 選択中のカメラセグメンテーションモデルの備考（`ml_cam/app.py`の備考欄→エクスポート時に
  // `<name>.json`へ同梱されたもの。`e2eSelectedNote`と対称、2026-08-29追加）
  const camSelectedNote = camModelFiles.find((f) => f.name === camModel?.name)?.note
  // 選択中のE2E LiDARモデルの備考（`ml_lidar/app.py`の備考欄→エクスポート時に
  // `<name>.json`へ同梱されたもの。2026-08-29追加）
  const e2eSelectedNote = e2eModelFiles.find((f) => f.name === e2eModel?.name)?.note
  useEffect(() => {
    ch?.camModelList()
    ch?.e2eModelList()
  }, [ch])

  if (!auto) {
    return (
      <div className="autopanel">
        <span className="dim">サーバに接続していません</span>
      </div>
    )
  }

  // システム同定タブ専用のモードはここには出さない（`SysIdView.tsx` が扱う）
  const driveCatalog = auto.catalog.filter((c) => c.category !== 'sysid')
  const selected = driveCatalog.find((c) => c.id === auto.mode) ?? null
  const modeStats = selected?.stats ?? []
  const st = n.auto
  // **「planner が黙っている」と「planner が止まれと言っている」を区別する。**
  // 前者は planning_node が落ちている疑いで、対処が全く違う
  const silent = !isFinite(n.autoAgeMs) || n.autoAgeMs > AUTO_STALE_MS

  const setMode = (id: string) => {
    ch?.setAuto({ mode: id })
    ui.set({ autoOffReason: '' })
  }

  const toggleEngage = () => {
    const next = !auto.engaged
    ch?.setAuto({ engaged: next })
    ui.set({ autoOffReason: next ? '' : '自動運転タブから解除' })
  }

  return (
    <div className={`autopanel${auto.engaged ? ' engaged' : ''}`}>
      {/* ── モード選択 + engage を横並びの1行に（狭い幅では折り返す） ── */}
      <div className="auto-row1">
        <section className="auto-modes">
          <span className="label">走らせ方</span>
          <select
            className="auto-mode-select"
            value={auto.mode}
            onChange={(e) => setMode(e.target.value)}
          >
            <option value="">（未選択）</option>
            {driveCatalog.map((c) => (
              <option key={c.id} value={c.id}>
                {c.name}
              </option>
            ))}
          </select>
          {driveCatalog.length === 0 && (
            <span className="dim">planner がありません</span>
          )}
        </section>

        <section className="auto-engage">
          <button
            className={auto.engaged ? 'engage on' : 'engage'}
            disabled={!auto.mode}
            onClick={toggleEngage}
          >
            {auto.engaged ? '自律走行を解除' : '自律走行を開始'}
          </button>
          {/* **engage と ARM は別物**。どちらが欠けているかを名指しで出す */}
          {auto.engaged && !ui.armRequested && (
            <span className="badge-warn">ARM してください（Enter）</span>
          )}
          {auto.engaged && ui.armRequested && !silent && (
            <span className="badge-live">走行中</span>
          )}
          {!auto.engaged && ui.autoOffReason && (
            <span className="badge-warn">{ui.autoOffReason}</span>
          )}
          {auto.stalls > 0 && (
            <span className="badge-bad">指令の途絶で制動 {auto.stalls} 回</span>
          )}
        </section>
      </div>

      {selected && <p className="auto-mode-desc dim">{selected.description}</p>}

      {/* セグメンテーション走行（`ftg_cam`）専用のモデル選択。
          engage 中に選び直すと `_on_auto` のモード変更と同じ理由で engage が必ず落ちる */}
      {selected?.id === 'ftg_cam' && (
        <div className="auto-model-row">
          <span className="auto-model-label">セグメンテーションモデル</span>
          <select
            value={camModel?.name ?? ''}
            disabled={camModel === null}
            onChange={(e) => ch?.camModelSelect(e.target.value)}
          >
            <option value="">（未選択）</option>
            {camModelFiles.map((f) => (
              <option key={f.name} value={f.name}>
                {f.name}
                {!f.has_config && '（前処理設定なし）'}
              </option>
            ))}
          </select>
          <button onClick={() => ch?.camModelList()}>更新</button>
          {camModelFiles.length === 0 && (
            <span className="dim">models/ に .onnx がありません</span>
          )}
          {camSelectedNote && <div className="auto-model-note">{camSelectedNote}</div>}
        </div>
      )}

      {/* E2E LiDAR走行（`e2e_lidar`）専用のモデル選択。上と全く同じ形 */}
      {selected?.id === 'e2e_lidar' && (
        <div className="auto-model-row">
          <span className="auto-model-label">E2E LiDARモデル</span>
          <select
            value={e2eModel?.name ?? ''}
            disabled={e2eModel === null}
            onChange={(e) => ch?.e2eModelSelect(e.target.value)}
          >
            <option value="">（未選択）</option>
            {e2eModelFiles.map((f) => (
              <option key={f.name} value={f.name}>
                {f.name}
                {!f.has_config && '（前処理設定なし）'}
              </option>
            ))}
          </select>
          <button onClick={() => ch?.e2eModelList()}>更新</button>
          {e2eModelFiles.length === 0 && (
            <span className="dim">models/e2e_lidar/ に .onnx がありません</span>
          )}
          {e2eSelectedNote && <div className="auto-model-note">{e2eSelectedNote}</div>}
        </div>
      )}

      {/* 対象追従（`follow_object`）専用UI。対象の選択自体は前方カメラの映像上で
          ドラッグして行う（`CameraView.tsx`）——ここには選択解除ボタンと状態表示だけ置く */}
      {selected?.id === 'follow_object' && (
        <div className="auto-model-row">
          <span className="auto-model-label">対象追従</span>
          <button onClick={() => ch?.clearTrackRoi()}>選択を解除</button>
          {st?.target_locked ? (
            <span className="badge-live">
              ロック中 {st.target_distance.toFixed(2)}m / {st.target_bearing_deg.toFixed(0)}°
            </span>
          ) : st?.target_lost ? (
            <span className="badge-warn">見失い中（{(st.target_lost_ms / 1000).toFixed(1)}s）</span>
          ) : (
            <span className="dim">前方カメラの映像上でドラッグして対象を選択してください</span>
          )}
        </div>
      )}

      {/* ── planner の判断 ── */}
      <section className="auto-state">
        <span className="label">判断</span>
        {silent ? (
          <div className="badge-bad">
            planning_node から判断が届いていません
            {selected ? '（プロセスが落ちていないか確認）' : ''}
          </div>
        ) : (
          <>
            <div className={st?.ready ? 'auto-reason' : 'auto-reason lv-bad'}>
              {st?.reason || '—'}
            </div>
            <div className="auto-stats">
              <Stat value={mps(st?.target_speed ?? 0)} unit="m/s 指令" />
              <Stat value={((st?.target_steer ?? 0) * RAD2DEG).toFixed(1)} unit="° 舵" />
              {/* この先の4つは planner が書く場合だけ出す（`selected.stats`、
                  `raspi/auto/base.py` の `Planner.stats` 参照）——書かないフィールドは
                  `AutoState` の既定値0.0のままなので、出しても意味の無い数値になる */}
              {modeStats.includes('free_ahead') && (
                <Stat value={(st?.free_ahead ?? 0).toFixed(2)} unit="m 正面" />
              )}
              {modeStats.includes('nearest') && (
                <Stat value={(st?.nearest ?? 0).toFixed(2)} unit="m 最近傍" />
              )}
              {modeStats.includes('gap') && (
                <Stat
                  value={st ? `${st.gap_start_deg.toFixed(0)}〜${st.gap_end_deg.toFixed(0)}` : '—'}
                  unit="° ギャップ"
                />
              )}
              {modeStats.includes('valid_ratio') && (
                <Stat value={((st?.valid_ratio ?? 0) * 100).toFixed(0)} unit="% 有効点" />
              )}
              <Stat value={(st?.plan_hz ?? 0).toFixed(1)} unit="Hz 計画" />
            </div>
          </>
        )}
      </section>

      {/* ── パラメータ ──
          2026-08-29: 常時表示に変更（指示による）。以前は `<details>` で畳んで
          いたが、車体図を横並びにした分パネルの縦の余白に余裕ができたので、
          開閉の手間なく常に見える方が走行中の微調整に向く */}
      {selected && (
        <section className="auto-params">
          <span className="label">パラメータ（{selected.params.length}）</span>
          <ParamSliders
            params={selected.params}
            values={auto.params}
            onChange={(key, value) => ch?.setAuto({ params: { [key]: value } })}
          />
        </section>
      )}
    </div>
  )
}

/** 判断メトリクス1つぶん。**値と単位ラベルを1つのセルにまとめる**——
 * 折り返しで値と単位が別の行に分離しないようにするため（`.auto-stats` 参照） */
function Stat({ value, unit }: { value: string; unit: string }) {
  return (
    <div className="auto-stat">
      <b>{value}</b>
      <span>{unit}</span>
    </div>
  )
}
