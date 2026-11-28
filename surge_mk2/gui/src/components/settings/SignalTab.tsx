/** 設定パネルの「矢印信号（ウィンカーの点滅・矢印信号認識のしきい値）」タブ。`SettingsPanel.tsx` から分けた。 */
import { useEffect } from 'react'
import type { ControlChannel } from '../../ws/control'
import { useNumbers } from '../../bus/live'
import { useUi } from '../../store/ui'

export function SignalTab({ ch }: { ch: ControlChannel | null }) {
  const { signal: signalLive } = useNumbers()
  const sig = useUi((s) => s.signalConfig)
  const winkerMode = useUi((s) => s.winkerMode)
  const modelFiles = useUi((s) => s.signalModelFiles)
  const method = sig?.method ?? 'classic'
  // タブを開いたときにモデルの一覧を取りに行く（AutoPanel のモデル選択と同じ流儀）
  useEffect(() => {
    ch?.signalModelList()
  }, [ch])

  return (
    <>
      <section className="settings-group">
        <h3>ウィンカー・ハザードの点滅</h3>
        <p className="dim">選択したパターンはSTM32へ送られ、ウィンカーとハザードの両方に適用されます。</p>
        <div className="seg">
          {([[1, 'モード1 普通'], [2, 'モード2 滑らか'], [3, 'モード3 豆電球']] as const).map(([mode, label]) => (
            <button key={mode} className={winkerMode === mode ? 'on' : ''}
              onClick={() => {
                useUi.getState().set({ winkerMode: mode as number })
              }}>{label}</button>
          ))}
        </div>
      </section>
      {/* 矢印信号認識（`arrow_signal_node`）。カメラ設定と同じく状態はサーバ真値
          （`status.signal_config`）。会場で実物の看板を見ながらここのスライダーを
          動かし、自動運転タブの前方カメラ映像に重なるROI枠線・ラベルを見て
          追い込む運用（`CameraView.tsx` の drawArrowSignal） */}
      <section className="settings-group">
        <h3>矢印信号認識</h3>
        <label className="settings-checkbox">
          <input
            type="checkbox"
            checked={sig?.enabled ?? true}
            disabled={sig === null}
            onChange={(e) => ch?.setSignalConfig({ enabled: e.target.checked })}
          />
          矢印信号を認識する{sig === null && '（未確認）'}
          <span className="unit" title="OFFにするとPi側はフレーム読み取り自体を止める（CPU節電）">
            　OFFでCPU節電
          </span>
        </label>
        {signalLive && (
          <p className="dim">
            判定: {signalLive.confirmed_value || signalLive.value || '(非該当)'}
            　[{signalLive.method}]
            {signalLive.method === 'yolo'
              ? ` ${signalLive.label || '-'} ${signalLive.score.toFixed(2)} / ${signalLive.infer_ms.toFixed(0)}ms`
              : ` lit_frac ${signalLive.lit_frac.toFixed(3)}`}
            {signalLive.confirmed_value && '　→ route/select 送信中'}
          </p>
        )}
      </section>

      {/* 判定の方式。YOLO はモデルが読めなければ Pi 側で古典CVに戻る（status の方式表示が実際の方式） */}
      <section className="settings-group">
        <h3>認識の方式</h3>
        <div className="seg">
          {([['classic', '古典CV（HSV）'], ['yolo', 'YOLO']] as const).map(([m, label]) => (
            <button key={m} className={method === m ? 'on' : ''} disabled={sig === null}
              onClick={() => ch?.setSignalConfig({ method: m })}>{label}</button>
          ))}
        </div>
        {method === 'yolo' && (
          <>
            <div className="settings-row">
              <div className="settings-row-head">
                <span className="label">モデル</span>
                <button onClick={() => ch?.signalModelList()}>更新</button>
              </div>
              <select
                value={sig?.yolo_model ?? ''}
                disabled={sig === null}
                onChange={(e) => ch?.setSignalConfig({ yoloModel: e.target.value })}
              >
                <option value="">（未選択）</option>
                {modelFiles.map((f) => (
                  <option key={f.name} value={f.name}>{f.name}{f.note ? ` — ${f.note}` : ''}</option>
                ))}
                {sig?.yolo_model && !modelFiles.some((f) => f.name === sig.yolo_model) && (
                  <option value={sig.yolo_model}>{sig.yolo_model}</option>
                )}
              </select>
              <p className="dim">models/arrow_yolo/ の .onnx（ultralytics の書き出し）。クラス名 left/right/straight を方向として読む</p>
            </div>
            <div className="settings-row" title="これ未満のスコアの検出は捨てる">
              <div className="settings-row-head">
                <span className="label">スコア下限</span>
                <b>{(sig?.yolo_conf ?? 0.5).toFixed(2)}</b>
              </div>
              <input
                type="range" min={0.05} max={0.95} step={0.05}
                value={sig?.yolo_conf ?? 0.5}
                disabled={sig === null}
                onChange={(e) => ch?.setSignalConfig({ yoloConf: Number(e.target.value) })}
              />
            </div>
            {signalLive?.error && <p className="lv-warn">⚠ {signalLive.error}（古典CVで判定中）</p>}
            {signalLive && signalLive.method !== 'yolo' && !signalLive.error && signalLive.enabled && (
              <p className="dim">モデル未読み込み（古典CVで判定中）</p>
            )}
          </>
        )}
      </section>

      <section className="settings-group">
        <h3>看板を探すROI帯（画面高さの割合）</h3>
        <div className="settings-row">
          <div className="settings-row-head">
            <span className="label">上端</span>
            <b>{((sig?.roi_top ?? 0.05) * 100).toFixed(0)}</b>
            <span className="unit">%</span>
          </div>
          <input
            type="range" min={0} max={100} step={1}
            value={(sig?.roi_top ?? 0.05) * 100}
            disabled={sig === null}
            onChange={(e) => ch?.setSignalConfig({ roiTop: Number(e.target.value) / 100 })}
          />
        </div>
        <div className="settings-row">
          <div className="settings-row-head">
            <span className="label">下端</span>
            <b>{((sig?.roi_bottom ?? 0.55) * 100).toFixed(0)}</b>
            <span className="unit">%</span>
          </div>
          <input
            type="range" min={0} max={100} step={1}
            value={(sig?.roi_bottom ?? 0.55) * 100}
            disabled={sig === null}
            onChange={(e) => ch?.setSignalConfig({ roiBottom: Number(e.target.value) / 100 })}
          />
        </div>
      </section>

      <section className="settings-group">
        <h3>「薄い色」判定（HSV）{method === 'yolo' && <span className="dim">　※古典CVのときだけ効く</span>}</h3>
        <div className="settings-row" title="低すぎると背景のノイズまで拾い、高すぎると濃い色の表示（別の意味）まで拾う">
          <div className="settings-row-head">
            <span className="label">彩度 下限</span>
            <b>{sig?.sat_min ?? 40}</b>
          </div>
          <input
            type="range" min={0} max={255} step={1}
            value={sig?.sat_min ?? 40}
            disabled={sig === null}
            onChange={(e) => ch?.setSignalConfig({ satMin: Number(e.target.value) })}
          />
        </div>
        <div className="settings-row" title="これを超える彩度は「濃い色」（別の意味）として無視する">
          <div className="settings-row-head">
            <span className="label">彩度 上限</span>
            <b>{sig?.sat_max ?? 180}</b>
          </div>
          <input
            type="range" min={0} max={255} step={1}
            value={sig?.sat_max ?? 180}
            disabled={sig === null}
            onChange={(e) => ch?.setSignalConfig({ satMax: Number(e.target.value) })}
          />
        </div>
        <div className="settings-row">
          <div className="settings-row-head">
            <span className="label">明度 下限</span>
            <b>{sig?.val_min ?? 150}</b>
          </div>
          <input
            type="range" min={0} max={255} step={1}
            value={sig?.val_min ?? 150}
            disabled={sig === null}
            onChange={(e) => ch?.setSignalConfig({ valMin: Number(e.target.value) })}
          />
        </div>
        <div className="settings-row" title="ROI内でこの割合以上の画素が点灯していないと「非点灯」扱いにする">
          <div className="settings-row-head">
            <span className="label">点灯とみなす最小割合</span>
            <b>{((sig?.min_lit_frac ?? 0.02) * 100).toFixed(1)}</b>
            <span className="unit">%</span>
          </div>
          <input
            type="range" min={0} max={20} step={0.5}
            value={(sig?.min_lit_frac ?? 0.02) * 100}
            disabled={sig === null}
            onChange={(e) => ch?.setSignalConfig({ minLitFrac: Number(e.target.value) / 100 })}
          />
        </div>
      </section>
    </>
  )
}
