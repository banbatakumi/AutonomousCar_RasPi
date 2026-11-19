/**
 * 設定パネル — 走行タブ（ラジコン／自動運転）の歯車から開くドロワーの中身
 * （`components/SettingsDrawer.tsx`）。
 *
 * ## なぜ独立したタブにしないのか
 *
 * ここの項目は**すべて手動（キーボード/パッド）操作のためのもの**で、診断でも
 * ログでも使わない。タブを1枚使って画面上部を占有する価値がなく、かつ
 * 「走らせて → 加速が鈍い → 直す → また走らせる」の往復でタブを切り替えるのは
 * 手間でしかない。走行タブの中に畳んで、走りながら開けるようにしてある。
 *
 * 自動運転タブでも中身は同じもの——engage 中でも人間がキーボードで
 * 位置を微調整する場面があり、そこで効く速度・舵の調整値はラジコンタブと共通
 * （`store/settings.ts` の `DrivingSettings` は1つしかなく、タブごとに分かれていない）。
 *
 * ここにあるのは**すべて GUI 側だけで完結する調整値**（`store/settings.ts` の
 * `DrivingSettings`）。`localStorage` に保存され、次回起動時も引き継がれる。
 *
 * ⚠ STM32 実機の `CONFIG_SET`/`CONFIG_GET` で同期する値（TC/ABS/TV のゲイン・有効無効、
 * 速度 PI 等）は、GUI ではなく `config/vehicle.toml` の `[control]` を io_node が送る
 * （Pi が single source of truth。`raspi/proto/protocol.toml` の Param 参照）。
 * ここにあるのは GUI が指令を作るときのランプ・上限だけ。
 *
 * ## 中身のタブ分け
 *
 * 上の「独立したページタブにしない」話とは別物——ここは**このドロワーの中だけ**
 * で完結する軽いタブ（`SETTINGS_TABS`）。項目が増えて縦に長くなりすぎたので、
 * 「走行」（制御方式・速度orトルク・舵・ブレーキ）「安全」（自動停止・TC/TV・
 * 片輪浮き対策・ARMタイムアウト）「カメラ」（capture FPS上限・後方ON/OFF・
 * GUI配信fps・進路ガイド校正）「サウンド」（合成エンジン音のON/OFF・音色）の4つに分けてある。「既定値に戻す」等のヘッダはタブの外に
 * 置いてある——`DrivingSettings` は1つしかなく、どのタブの値もまとめて戻す/
 * 保存するので、特定のタブだけの操作ではないため。
 *
 * タブの中身は `components/settings/` の子コンポーネント（タブごとに1つ）。スライダ行と項目の定義は
 * `settings/fields.tsx`。
 */
import { useState } from 'react'
import { useUi } from '../store/ui'
import type { ControlChannel } from '../ws/control'
import { CameraTab } from './settings/CameraTab'
import { DriveTab } from './settings/DriveTab'
import { SafetyTab } from './settings/SafetyTab'
import { SignalTab } from './settings/SignalTab'
import { SoundTab } from './settings/SoundTab'

/**
 * 項目が増えて縦に長くなりすぎたので、テーマごとにタブで分けてある。
 * 「既定値に戻す」等のヘッダはタブの外——`DrivingSettings` 全体（走行・安全・
 * カメラ校正のどのタブの値も含む）に効くので、特定のタブだけの操作ではない。
 */
const SETTINGS_TABS = [
  { id: 'drive', label: '走行' },
  { id: 'safety', label: '安全' },
  { id: 'camera', label: 'カメラ' },
  { id: 'signal', label: '矢印信号' },
  { id: 'sound', label: 'サウンド' },
] as const
type SettingsTab = (typeof SETTINGS_TABS)[number]['id']

export function SettingsPanel({ ch }: { ch: ControlChannel | null }) {
  const [tab, setTab] = useState<SettingsTab>('drive')
  const resetSettings = useUi((s) => s.resetSettings)
  // リセット後もPi側の保存値を追随させる（追随させないと、次に繋いだときの
  // 「接続直後の1回だけ取り込む」同期で古い値に巻き戻ってしまう）
  const handleResetSettings = () => {
    resetSettings()
    // 送るのは**クランプ後の値**（`resetSettings` が今の車両の上限で切り直したもの）
    ch?.setDriveSettings(useUi.getState().settings)
  }
  const saveCurrentAsDefault = useUi((s) => s.saveCurrentAsDefault)

  return (
    <div className="settings">
      <div className="settings-head">
        <button onClick={handleResetSettings}>既定値に戻す</button>
        {/* 「既定値」はソースコードの固定値ではなく、ここで上書きできる
            （`store/ui.ts` の `settingsDefault`、`localStorage` に保存）。 */}
        <button onClick={saveCurrentAsDefault} title="今の設定値を「既定値に戻す」の戻り先として保存する">
          現在の値を既定値として保存
        </button>
      </div>

      <div className="seg settings-tabs">
        {SETTINGS_TABS.map((t) => (
          <button key={t.id} className={tab === t.id ? 'on' : ''} onClick={() => setTab(t.id)}>
            {t.label}
          </button>
        ))}
      </div>

      {tab === 'drive' && <DriveTab ch={ch} />}
      {tab === 'safety' && <SafetyTab ch={ch} />}
      {tab === 'camera' && <CameraTab ch={ch} />}
      {tab === 'signal' && <SignalTab ch={ch} />}
      {tab === 'sound' && <SoundTab />}
    </div>
  )
}
