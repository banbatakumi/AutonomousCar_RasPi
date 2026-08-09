/**
 * 運転ビュー — 走行中に見る唯一の画面（`architecture.md` §10.2 / §10.3）。
 *
 * 左に前方カメラ（＋後方は小窓）、右に LiDAR 俯瞰、下に速度・舵角の帯。
 * 温度・電圧の類は畳んであり、**しきい値を超えたときだけ自動で前に出る**。
 */
import { useNumbers } from '../bus/live'
import { AuxPanel } from '../components/AuxPanel'
import { DiagStrip } from '../components/DiagStrip'
import { DriveBar } from '../components/DriveBar'
import { CameraView } from '../render/CameraView'
import { LidarView } from '../render/LidarView'
import { useUi } from '../store/ui'

export function DriveView() {
  const ui = useUi()

  return (
    <div className="drive">
      <div className={`cams ${ui.rearBig ? 'rear-big' : ''}`}>
        <CameraView cam="front" label="前方" />
        <CameraView cam="rear" label="後方" />
        <div className="cam-controls">
          <label>
            <input
              type="checkbox"
              checked={ui.pathGuide}
              onChange={(e) => ui.set({ pathGuide: e.target.checked })}
            />
            進路ガイド
          </label>
          <button onClick={() => ui.set({ rearBig: !ui.rearBig })}>
            {ui.rearBig ? '前方を大きく' : '後方を大きく'}
          </button>
        </div>
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
