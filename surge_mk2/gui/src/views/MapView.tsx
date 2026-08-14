/**
 * 地図生成ビュー — **世界座標のものを全部ここに集める。**
 *
 * `architecture.md` §10.1 は「見る頻度で分類する」と言っている。地図は
 * **世界座標・低頻度・大きなキャンバス**で、自動運転ビュー（車両基準・高頻度・
 * カメラと数値）とは性格が正反対なので、同じ画面に同居させると必ずどちらかが潰れる。
 *
 * ## 「地図を作っている間だけの画面」にはしない
 *
 * 自車がレーシングラインのどこに居るか・動的障害物がどこに出たかは、
 * **走っている最中にこそ見たい**情報で、しかも全部世界座標。EXPLORE 専用に
 * すると、いちばん必要な場面で見られなくなる。だから全段で同じ画面を使う。
 *
 * ## 操作はここに置かない（1つを除いて）
 *
 * モード選択と engage は自動運転ビューの `AutoPanel` **1箇所だけ**にしてある。
 * 操作の入口が2つあると「どちらで engage したか」を人間が覚える羽目になる。
 * ここに置くのは「地図を確定」だけで、これは地図そのものに対する操作だから。
 */
import { useNumbers } from '../bus/live'
import { live } from '../bus/live'
import { MapCanvas, clearTrail } from '../render/MapCanvas'
import type { ControlChannel } from '../ws/control'

export function MapView({ ch }: { ch: ControlChannel | null }) {
  return (
    <div className="mapview">
      <div className="map-main">
        <MapCanvas />
        <MapBadge />
      </div>
      <MapPanel ch={ch} />
    </div>
  )
}

/** 段と周回数。**画面を切り替えずに今どこに居るかが分かる**ようにする。 */
function MapBadge() {
  const a = live.auto
  const phase = a?.phase || '—'
  const label =
    phase === 'EXPLORE' ? '地図作成中' : phase === 'BUILD' ? '経路生成中' : phase === 'RACE' ? '走行中' : '待機'
  return (
    <div className="map-badge">
      <span className={phase === 'RACE' ? 'lv-ok' : 'dim'}>{label}</span>
      {a?.engaged && <span className="badge-warn">ENGAGED</span>}
    </div>
  )
}

/**
 * 地図生成の進み具合。**数値は 8Hz に間引いた `useNumbers` から取らない**
 * （ここは 10Hz の `auto` そのものを見たいので `live.auto` を直接読む）。
 * 再レンダリングは `useNumbers` の間引きに乗せて起こす。
 */
function MapPanel({ ch }: { ch: ControlChannel | null }) {
  useNumbers() // 8Hz で再レンダリングを起こすためだけに呼ぶ
  const a = live.auto
  const m = live.map

  const cells = m ? `${m.width}×${m.height} @ ${(m.res * 100).toFixed(1)}cm` : '—'
  const laps = a ? `${a.laps} 周（回頭 ${a.lap_progress.toFixed(2)} 周ぶん）` : '—'
  const score = a?.match_score ?? 0

  return (
    <div className="map-panel">
      <h3>地図生成</h3>
      <Row label="段" value={a?.phase || '—'} />
      <Row label="周回" value={laps} />
      {/* ★一致度が低いのに走っているなら疑う。**色で目に入るようにする** */}
      <Row
        label="一致度"
        value={score.toFixed(2)}
        cls={score < 0.35 ? 'lv-bad' : score < 0.6 ? 'lv-warn' : 'lv-ok'}
      />
      <Row
        label="自己位置"
        value={a ? `${a.pose_x.toFixed(2)}, ${a.pose_y.toFixed(2)} m  ${((a.pose_yaw * 180) / Math.PI).toFixed(0)}°` : '—'}
      />
      <Row label="格子" value={cells} />

      <h3>経路</h3>
      <Row label="点数" value={m && m.raceline.length ? `${m.raceline.length / 2}` : '未生成'} />
      <Row
        label="速度域"
        value={m && m.racelineV.length ? `${Math.min(...m.racelineV).toFixed(2)}〜${Math.max(...m.racelineV).toFixed(2)} m/s` : '—'}
      />
      <Row
        label="横偏差"
        value={a && a.phase === 'RACE' ? `${(a.cross_track * 100).toFixed(0)} cm` : '—'}
        cls={a && Math.abs(a.cross_track) > 0.3 ? 'lv-warn' : undefined}
      />
      <Row
        label="動的障害物"
        value={a && a.obstacles?.length ? `${a.obstacles.length / 3} 個` : 'なし'}
        cls={a && a.obstacles?.length ? 'lv-warn' : undefined}
      />

      <div className="map-actions">
        {/* 自動判定（累積回頭 330° + 始点近傍）が滑ったときの逃げ道。
            **EXPLORE → BUILD は不可逆**なので、人間が押せる経路を残す */}
        <button disabled={!ch || a?.phase !== 'EXPLORE'} onClick={() => ch?.freezeMap()}>
          地図を確定
        </button>
        {/* **engage は落とさない。** 地図だけ捨てて EXPLORE からやり直す。
            経路が消えるので、走行中に押しても次の周期で制動に落ちる */}
        <button
          disabled={!ch}
          onClick={() => {
            ch?.clearMap()
            clearTrail()
          }}
        >
          地図を削除
        </button>
      </div>
      <p className="hint">{a?.reason || '自動運転タブでモードを選ぶと動き出す'}</p>
    </div>
  )
}

function Row({ label, value, cls }: { label: string; value: string; cls?: string }) {
  return (
    <div className="map-row">
      <span className="dim">{label}</span>
      <span className={cls}>{value}</span>
    </div>
  )
}
