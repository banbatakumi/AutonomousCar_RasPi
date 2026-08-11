/**
 * ラジコンビュー — **運転を楽しむための画面**。
 *
 * 自動運転ビュー（`AutoView.tsx`）と同じデータを見ているが、選び方の基準がまるで違う。
 * あちらは「開発者が異常を切り分ける」ための画面で、こちらは「運転者が走らせる」ための画面。
 *
 *   - 数値ではなくメータで出す（速度・G・舵角）。**周辺視で読めることが最優先**
 *   - 介入は数値ではなくランプ（TC / TV / 自動停止）
 *   - 通信の健全性・時刻同期・スリップ量は出さない。全部 診断タブにある
 *   - 温度と電圧だけは常時出す。実車と同じで、**走行を切り上げる判断に使う**
 *
 * 隠すのは「正常時の数字」だけで、異常は必ず出る。E-STOP・ARM・ラッチ警告は
 * `StatusBar` が全タブ共通で出しており、ここでも一切省いていない。
 *
 * ## レイアウト（2026-08-11 改訂）
 *
 * ```
 *   ┌───────────────────┬─────────┐
 *   │ [後方PIP]         │  LiDAR  │
 *   │                   ├─────────┤
 *   │   前方カメラ       │  計器   │
 *   │   （左 2/3 全面）  │ ＋補機  │
 *   └───────────────────┴─────────┘
 * ```
 *
 * **操縦は前方の映像を見て行う。** 帯を下に積むと映像が縦に潰れるので、
 * 左 2/3 を映像だけに使い、計器は右 1/3 の下半分にまとめた。
 * 後方は左上の PIP。**消すと WS ごと閉じる**ので、前方に帯域を寄せられる。
 */
import { useNumbers } from '../bus/live'
import { RcBar } from '../components/rc/RcBar'
import { SettingsDrawer } from '../components/SettingsDrawer'
import { CameraView } from '../render/CameraView'
import { LidarView } from '../render/LidarView'
import { useUi } from '../store/ui'

export function RcView() {
  const ui = useUi()

  return (
    <div className="rc">
      <div className="rc-cams">
        <CameraView cam="front" label="前方" />

        {/* バックミラー。**前方の邪魔にならない位置と大きさ**に留める */}
        {ui.rearPip && (
          <div className="rc-pip">
            <CameraView cam="rear" label="後方" />
          </div>
        )}

        <div className="cam-controls">
          <label>
            <input
              type="checkbox"
              checked={ui.pathGuide}
              onChange={(e) => ui.set({ pathGuide: e.target.checked })}
            />
            進路ガイド
          </label>
          {/* 消している間は後方の WS を張らない（前方のフレームレートに回る） */}
          <button
            onClick={() => ui.set({ rearPip: !ui.rearPip })}
            title={ui.rearPip ? '後方の配信を止める（前方に帯域が回る）' : '後方を左上に小さく出す'}
          >
            {ui.rearPip ? '後方を隠す' : '後方を表示'}
          </button>
        </div>
      </div>

      {/* LiDAR は「ぶつけないため」に置く。自動運転ビューより小さくてよい */}
      <div className="rc-lidar">
        <LidarView />
        <LidarBadge />
        <div className="lidar-controls">
          <button onClick={() => ui.set({ lidarZoom: Math.min(12, ui.lidarZoom * 1.5) })}>−</button>
          <span>{ui.lidarZoom.toFixed(1)}m</span>
          <button onClick={() => ui.set({ lidarZoom: Math.max(1, ui.lidarZoom / 1.5) })}>＋</button>
        </div>
      </div>

      <RcBar />
      <SettingsDrawer />
    </div>
  )
}

/**
 * 点群が生きているかだけ。**Hz や欠測セクタ数は出さない**（自動運転ビューの
 * `ScanBadge` はそれを出すが、ここでは「見えていない」ことだけ伝われば足りる）。
 */
function LidarBadge() {
  const n = useNumbers()
  const dead = n.scanAgeMs > 400
  if (!dead) return null
  return (
    <div className="scan-badge">
      <span className="lv-bad">{isFinite(n.scanAgeMs) ? 'LiDAR 遅延' : 'LiDAR 未受信'}</span>
    </div>
  )
}
