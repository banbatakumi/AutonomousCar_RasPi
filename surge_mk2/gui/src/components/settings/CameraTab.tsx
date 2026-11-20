/** 設定パネルの「カメラ（capture FPS 上限・GUI 配信・補正映像・映像への重畳）」タブ。`SettingsPanel.tsx` から分けた。 */
import type { ControlChannel } from '../../ws/control'
import { CAM_LABEL } from '../../render/cameraLayout'
import { useUi } from '../../store/ui'

export function CameraTab({ ch }: { ch: ControlChannel | null }) {
  const pathGuide = useUi((s) => s.pathGuide)
  const obstacleOverlay = useUi((s) => s.obstacleOverlay)
  const planOverlay = useUi((s) => s.planOverlay)
  const set = useUi((s) => s.set)
  // fan と違い `/ws/control` の status（イベント発生時のブロードキャスト）から読む。
  // 50Hz の `/ws/telemetry` に載せるほど頻繁に変わらない値なので tc_enabled 等とは事情が違う
  const cam = useUi((s) => s.cameraConfig)

  return (
    <>
      {/* capture側(camera_node)の前カメラのFPS上限・GUI配信頻度。
          TC/TVと同じく状態はサーバ真値（`status.camera_config`）。
          ARM中/DISARM中で別々の値を持つ（駐車中の節電のため）。
          前カメラの上限は目安——カメラを使う自動運転（`line_trace`・`cam_e2e` など）が
          engage 中はサーバ側が無視して最大まで引き上げる（`auto_override`） */}
      <section className="settings-group">
        <h3>カメラ（ARM中＝走行できる状態）</h3>
        {/* 左カメラは設定を持たない（Pi 側が要るときだけ主カメラと同期して回す） */}
        <div
          className="settings-row"
          title={
            cam?.auto_override
              ? '自動運転（ライントレース等）が走行中は、この上限を無視して最大まで引き上がっている'
              : '自動運転がカメラを使うモードで engage されている間は、この上限を無視して最大まで引き上がる'
          }
        >
          <div className="settings-row-head">
            <span className="label">前カメラ 取得上限</span>
            <b>{(cam?.front_fps_armed ?? 30).toFixed(0)}</b>
            <span className="unit">fps</span>
          </div>
          <input
            type="range"
            min={1}
            max={30}
            step={1}
            value={cam?.front_fps_armed ?? 30}
            disabled={cam === null}
            onChange={(e) => ch?.setCamera({ frontFpsArmed: Number(e.target.value) })}
          />
          {cam?.armed && cam?.auto_override && (
            <span className="unit">実効 {cam.front_fps_effective.toFixed(0)}fps（自動運転中）</span>
          )}
        </div>
      </section>

      <section className="settings-group">
        <h3>映像の魚眼補正</h3>
        <label
          className="settings-checkbox"
          title="Pi側で魚眼映像を仮想ピンホール（vehicle.toml の undistort_hfov）に変換して配信する。端は引き伸ばされ画角は狭くなる。記録（mcap）・📷撮影・自動運転の認識は常に魚眼のまま"
        >
          <input
            type="checkbox"
            checked={cam?.undistort ?? false}
            disabled={cam === null || !(cam.undistort_available?.front || cam.undistort_available?.left)}
            onChange={(e) => ch?.setCamera({ undistort: e.target.checked })}
          />
          補正した映像を表示する
          {cam !== null && !(cam.undistort_available?.front || cam.undistort_available?.left)
            && '（未校正。ランチャーの「カメラ校正」で校正すると使える）'}
          {cam?.undistort_available && !!cam.undistort_available.front !== !!cam.undistort_available.left
            && `（${cam.undistort_available.front ? CAM_LABEL.front : CAM_LABEL.left}カメラのみ校正済み）`}
        </label>
      </section>

      <section className="settings-group">
        <h3>
          カメラ（DISARM中＝駐車中）
          {cam !== null && (
            <span className="unit">{cam.armed ? '　現在ARM中' : '　現在DISARM中・下記を適用中'}</span>
          )}
        </h3>
        <div className="settings-row" title="駐車中は走行の応答性が要らないので、通常より低いfpsで十分">
          <div className="settings-row-head">
            <span className="label">前カメラ 取得上限</span>
            <b>{(cam?.front_fps_disarm ?? 1).toFixed(0)}</b>
            <span className="unit">fps</span>
          </div>
          <input
            type="range"
            min={1}
            max={30}
            step={1}
            value={cam?.front_fps_disarm ?? 1}
            disabled={cam === null}
            onChange={(e) => ch?.setCamera({ frontFpsDisarm: Number(e.target.value) })}
          />
        </div>
      </section>

      <section className="settings-group">
        <h3>カメラ（共通）</h3>
        <div className="settings-row" title="ブラウザへ送るJPEGの頻度。上げるとWi-Fi帯域を余計に使う（2026-08-24 実機で前後同時30fpsでも問題ないことを確認済み）。ARM/DISARMと無関係">
          <div className="settings-row-head">
            <span className="label">GUIへの配信</span>
            <b>{(cam?.gui_hz ?? 30).toFixed(0)}</b>
            <span className="unit">fps</span>
          </div>
          <input
            type="range"
            min={1}
            max={30}
            step={1}
            value={cam?.gui_hz ?? 30}
            disabled={cam === null}
            onChange={(e) => ch?.setCamera({ guiHz: Number(e.target.value) })}
          />
        </div>
      </section>

      {/* E2E LiDARモデル（`e2e_lidar`）などの
          選択は自動運転タブの `AutoPanel` にある——「どのモードで
          走るか」と「そのモードが使うモデル」は同じ意思決定の一部なので、ここには置かない */}

      {/* 進路ガイド（前方カメラ映像への重ね描き）の表示設定 */}
      <section className="settings-group">
        <h3>進路ガイド校正</h3>
        <label className="settings-checkbox">
          <input type="checkbox" checked={pathGuide} onChange={(e) => set({ pathGuide: e.target.checked })} />
          前方カメラに進路ガイドを重ねる（校正前・暫定）
        </label>
        <label className="settings-checkbox">
          <input
            type="checkbox"
            checked={obstacleOverlay}
            disabled={!pathGuide}
            onChange={(e) => set({ obstacleOverlay: e.target.checked })}
          />
          進路上の障害物（LiDAR）と当たる位置を重ねる（近づく速さで 白→琥珀→赤）
        </label>
        <label className="settings-checkbox">
          <input
            type="checkbox"
            checked={planOverlay}
            disabled={!pathGuide}
            onChange={(e) => set({ planOverlay: e.target.checked })}
          />
          自動運転の計画（指令の進路・経路・狙う点）を前方カメラに重ねる
        </label>
      </section>
    </>
  )
}
