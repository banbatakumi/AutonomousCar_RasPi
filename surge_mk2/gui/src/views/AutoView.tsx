/**
 * 自動運転ビュー — 自律走行を**監視・デバッグする**ための画面（`architecture.md` §10.2 / §10.3）。
 *
 * 上段に**前方・後方・LiDAR を同じ幅で3分割**、その下に速度・舵角の細い帯、
 * 一番下は「モード選択・engage・判断」（左2/3）と「車体図」（右1/3）。
 * 温度・電圧・リンク統計は出さない（2026-08-20）——正常時に読む数字ではなく、
 * 異常は `StatusBar` が全タブ共通で必ず伝える。詳しい数字は診断タブ（`DiagView`）で見る。
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
 * モードごとに必要なモデル選択（`ftg_cam`/`e2e_lidar`）も `AutoPanel` の中で
 * 選んだモードに応じて出し分ける（2026-08-28、`AutoPanel.tsx` 参照）。
 *
 * ## 車体図をラジコンビューと共通化（2026-08-28）
 *
 * `DrivePanel`（`components/rc/DrivePanel.tsx`）はスリップ・トルク・電圧・
 * 障害物近接リングを一望できる車体図で、自律走行の監視にもそのまま使える
 * （むしろ近接リングは自律走行の異常検知にこそ効く）。ラジコンタブ専用に
 * 作った部品だが実体は `useNumbers()` だけで完結しており画面に依存しないので、
 * そのまま import して右下に置くだけで足りた（指示による）。
 *
 * ## 補機帯（ブレーキ実トルク・操作ランプ・自動停止）を削除（2026-08-28）
 *
 * `AuxPanel` は走行を楽しむための情報（ラジコンタブ由来）で、自律走行の監視には
 * 不要と判断して外した（指示による）。ブレーキの実効きは診断タブ、自動停止の
 * 有効/無効は設定ドロワーの安全タブで見られる。
 *
 * ## 操作設定の歯車もここに置く（2026-08-20）
 *
 * `SettingsDrawer` はラジコンタブ専用だったが、自律走行中でもキーボードで
 * 位置を微調整する場面がある（`DRIVING_TABS` に自動運転タブも含まれ、
 * `useDriving` は常時生きている）。速度・舵の調整値はどちらのタブで
 * 操作しても同じ意味を持つので、ラジコンタブと同じドロワーをここにも出す。
 */
import { useNumbers } from '../bus/live'
import { AutoPanel } from '../components/AutoPanel'
import { DriveBar } from '../components/DriveBar'
import { SettingsDrawer } from '../components/SettingsDrawer'
import { DrivePanel } from '../components/rc/DrivePanel'
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
        <CameraView cam="front" label="前方" ch={ch} />
        <div className="cam-controls">
          {/* 前後どちらのカメラにも同じ `pathGuide` が効く（`CameraView.tsx`）ので、
              チェックボックスは1つだけ置けば足りる */}
          <label>
            <input
              type="checkbox"
              checked={ui.pathGuide}
              onChange={(e) => ui.set({ pathGuide: e.target.checked })}
            />
            進路ガイド（前後）
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

      <DriveBar />

      {/* `ctrl`/`car` を1つのグリッドエリア（`bottom`）にまとめ、中身は
          横並びの flex にした（2026-08-29）。以前はグリッドの3等分列を
          そのまま `car` 側に割り当てていたため、車体図（縦長の実寸比率）が
          その幅いっぱいまで**伸びない**分、左右に大きな余白ができていた
          （指摘による）。flex なら `.auto-car` は自分の縦横比＋行の高さから
          決まる幅だけを取り、余った横幅は `AutoPanel` 側（`flex:1`）に渡る */}
      <div className="auto-bottom">
        <AutoPanel ch={ch} />

        <div className="auto-car">
          <DrivePanel />
        </div>
      </div>

      <SettingsDrawer ch={ch} />
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
