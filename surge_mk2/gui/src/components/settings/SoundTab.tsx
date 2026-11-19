/** 設定パネルの「サウンド（合成エンジン音・クリアランスソナー）」タブ。`SettingsPanel.tsx` から分けた。 */
import { useUi } from '../../store/ui'

export function SoundTab() {
  const settings = useUi((s) => s.settings)
  const setSettings = useUi((s) => s.setSettings)
  const parkBeep = useUi((s) => s.parkBeep)
  const engineSoundOn = useUi((s) => s.engineSoundOn)
  const set = useUi((s) => s.set)

  return (
    <>
      {/* 合成エンジン音（`audio/engineSound.ts`）。**GUI 演出のみで車両側には
          一切関与しない**——鳴るのはこの画面を開いているブラウザのスピーカーだけ。
          ON/OFF は毎回明示的な操作が要る非永続の値（`store/ui.ts` の
          `engineSoundOn` コメント参照）なので、他の設定と違いここだけ `set` で
          直接読み書きする。音色（`settings.engineSoundType`）は通常どおり永続化 */}
      <section className="settings-group">
        <h3>クリアランスソナー</h3>
        <label className="settings-checkbox">
          <input type="checkbox" checked={parkBeep} onChange={(e) => set({ parkBeep: e.target.checked })} />
          ARM 中、進む向きの半周（LiDAR・超音波）に 0.4m より近い物があると断続音、0.1m より近いと連続音を鳴らす（演出のみ。止めるのは自動停止）
        </label>
      </section>
      <section className="settings-group">
        <h3>エンジン音</h3>
        <label className="settings-checkbox">
          <input
            type="checkbox"
            checked={engineSoundOn}
            onChange={(e) => set({ engineSoundOn: e.target.checked })}
          />
          速度・ギアに連動した合成音をこの画面のスピーカーから鳴らす（演出のみ、車両には無関係）
        </label>
        <div className="seg">
          {(
            [
              ['combustion', 'ガソリン車風'],
              ['ev', 'EV/ハイブリッド風'],
            ] as const
          ).map(([t, label]) => (
            <button
              key={t}
              className={settings.engineSoundType === t ? 'on' : ''}
              onClick={() => setSettings({ engineSoundType: t })}
            >
              {label}
            </button>
          ))}
        </div>
      </section>
    </>
  )
}
