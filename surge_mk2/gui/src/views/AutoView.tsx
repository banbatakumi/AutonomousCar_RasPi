/**
 * 自動運転ビュー — 自律走行を**監視・デバッグする**ための画面（`architecture.md` §10.2 / §10.3）。
 *
 * 上段に**前方・後方・LiDAR を同じ幅で3分割**、下に速度・舵角の帯。
 * 温度・電圧の類は畳んであり、**しきい値を超えたときだけ自動で前に出る**。
 *
 * ラジコンビューと違い、前方を特別扱いしない。自律走行の検証では
 * 「後方に何が映っているか」も「点群がどう見えているか」も同じ重さで見比べる。
 *
 * ## ラジコンビューとの違い
 *
 * 出しているデータはほぼ同じだが、選び方の基準が違う。こちらは指令と実測を数値で
 * 並べ、遅れやスリップをそのまま読ませる**開発者向けの画面**。運転を楽しむための
 * 画面は `RcView.tsx`（メータ主体、介入はランプのみ）。
 *
 * 経路・占有格子の重畳は Phase 3 以降で、ここに足していく。
 *
 * ## 自動運転の操作もここに置く
 *
 * `AutoPanel` でモードを選び engage する。**モードの一覧は GUI に書いていない**
 * （サーバの `status.auto.catalog` から生える）ので、`raspi/auto/` に planner を
 * 足せばこの画面に勝手に出る。LiDAR ビューには planner が選んだギャップを重ねる。
 */
import { useNumbers } from '../bus/live'
import { AutoPanel } from '../components/AutoPanel'
import { AuxPanel } from '../components/AuxPanel'
import { DiagStrip } from '../components/DiagStrip'
import { DriveBar } from '../components/DriveBar'
import { CameraView } from '../render/CameraView'
import { LidarView } from '../render/LidarView'
import { useUi } from '../store/ui'
import type { ControlChannel } from '../ws/control'

export function AutoView({ ch }: { ch: ControlChannel | null }) {
  const ui = useUi()

  return (
    <div className="drive">
      {/* 前方・後方・LiDAR を**同じ幅で横並び**にする。デバッグ用の画面なので、
          「前方が主で後方が従」ではなく3つを対等に見比べられる方が使いやすい */}
      <div className="cam-front">
        <CameraView cam="front" label="前方" />
        <div className="cam-controls">
          <label>
            <input
              type="checkbox"
              checked={ui.pathGuide}
              onChange={(e) => ui.set({ pathGuide: e.target.checked })}
            />
            進路ガイド
          </label>
        </div>
      </div>

      <div className="cam-rear">
        <CameraView cam="rear" label="後方" />
      </div>

      <div className="lidar">
        <LidarView />
        <ScanBadge />
        <div className="lidar-controls">
          <button onClick={() => ui.set({ lidarZoom: Math.min(12, ui.lidarZoom * 1.5) })}>
            −
          </button>
          <span>{ui.lidarZoom.toFixed(1)}m</span>
          <button onClick={() => ui.set({ lidarZoom: Math.max(1, ui.lidarZoom / 1.5) })}>
            ＋
          </button>
        </div>
      </div>

      <AutoPanel ch={ch} />
      <DriveBar />
      <AuxPanel />
      <DiagStrip />
    </div>
  )
}

/** 点群の鮮度と欠測。**「点が無い」と「受信できていない」を区別する。** */
function ScanBadge() {
  const n = useNumbers()
  const stale = n.scanAgeMs > 400
  return (
    <div className="scan-badge">
      <span className={stale ? 'lv-bad' : 'dim'}>
        {isFinite(n.scanAgeMs) ? `${(n.scanAgeMs / 1000).toFixed(1)}s前` : '未受信'}
      </span>
      <span className="dim">{n.scanHz.toFixed(1)}Hz</span>
      {/* **この1周**の欠測。累積を出すと走るほど増えて常時点灯になり意味を失う */}
      {n.scanMissing > 0 && (
        <span className="badge-warn">この周の欠測 {n.scanMissing}/12</span>
      )}
    </div>
  )
}
