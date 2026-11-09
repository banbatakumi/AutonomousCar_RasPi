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
 * ## レイアウト（指示による）
 *
 * ```
 *   ┌──────────────────────────────────────┐
 *   │ [電池・温度] ┌──バックミラー──┐ [LiDAR○] │ ⚙
 *   │             └──────────────┘          │
 *   │            メインカメラ（全画面）         │
 *   │   ⟨舵角⟩      ⟨速度⟩      ⟨G⟩           │ ← ボンネットの映る下端に HUD として重ねる
 *   └──────────────────────────────────────┘
 * ```
 *
 * **操縦はメインの映像を見て行う**ので、映像をタブいっぱいに出し、他は全部その上に重ねる。
 *
 * - **バックミラー**: 今メインに出ていない方のカメラを映像の上中央に重ねる。描くのは
 *   **センサー全体の縦の真ん中 1/3**（`MIRROR_SENSOR_Y`）。配信画像は下端を `bottomCrop`
 *   だけ切ってあるので、配信画像の座標に直して `CameraView` の `srcCrop` に渡す
 *   （後方カメラなら配信画像の下半分ほどになる）。後方カメラは**左右反転**して実車の
 *   ミラーと同じ左右で見せる。クリックでメインと入れ替わる（`ui.mainCam`、パッドの ○ でも可）
 * - **計器（舵角・速度・G）**: 前方映像の下端＝ボンネットが映っている帯に HUD として重ねる。
 *   高さは映像の高さの `HUD_FRAC`。背景に濃いグレーの台形（底角 `TRAP_ANGLE_DEG`）を
 *   ボンネットに重ねて敷き、映像の上でも読めるようにする。見た目はレース用ダッシュ風
 *   （F1・AEM CD-7・Ford GT のトラックモードを参考に）: 台形はカーボン調の筐体、メータの後ろに
 *   黒いガラス調の液晶（`.rc-hud-screen`）、上辺の上にシフトライト（`ShiftLights`、仮想変速に従う）
 *   介入ランプ（`AssistLamps`）は台形の左端、ハザードは右端に入れる。映像は上に詰め、余りは下に回す
 * - **電池と温度**（`RcStatus`）: 映像の左上に常時出す（温度と電圧は常時出す方針のため）
 *
 * ## メイン映像の箱は「前方カメラ」のサイズに固定
 *
 * 入れ替え後（後方がメインに出ているとき）も、**箱の形は前方カメラの実測値の
 * まま変えない**（指示による）。`onAspect` は常に「今 front を映している方の
 * `CameraView`」から受け取る（`frontAspect`）。箱はタブの中に contain-fit で収める
 * ——cover（はみ出しを切る）にしないのは、HUD をボンネットの位置に正しく重ねるには
 * 映像の下端が見えている必要があるのと、左右の端（死角になりやすい）を削らないため。
 *
 * **メイン映像のうちミラー・LiDAR・HUD 以外をクリックすると、LiDAR・進路ガイドが
 * まとめて表示/非表示になる**（不要なときに一瞬で消せる非常用スイッチ）。
 * ミラーはこの切り替えに含めない。
 *
 * **進路ガイドはメインの映像にしか出さない**（ミラー側は `srcCrop` 指定中なので出ない）。
 *
 * ## 実測ベースのサイズ計算
 *
 * 映像は `useContainFit`、計器のダイヤルは `useElementSize` で HUD の実測高さから
 * 逆算した px を渡す。⚠ CSS の `aspect-ratio` 単体には頼らない。**箱の中に
 * `<canvas>`/`<svg>` のような replaced element がいると、intrinsic サイズ
 * （既定 300px 相当）に負けて縮んだままになる罠を実機で踏んだ**
 * （`hooks/useContainFit.ts` 冒頭のコメント）。
 *
 * 映像の比率も実際に届いた画像から決める（実機ではセンサーが要求どおりのモードを
 * 取らないことがある）。フレームが来るまでは `FALLBACK_ASPECT` で仮組みしておく。
 */
import { useEffect, useRef, useState, type CSSProperties } from 'react'
import { reverseSelected, useUi } from '../store/ui'
import { AssistLamps } from '../components/rc/AssistLamps'
import { GMeter } from '../components/rc/GMeter'
import { LidarMini } from '../components/rc/LidarMini'
import { ParkSensor } from '../components/rc/ParkSensor'
import { RcStatus } from '../components/rc/RcStatus'
import { ShiftLights } from '../components/rc/ShiftLights'
import { SpeedGauge } from '../components/rc/SpeedGauge'
import { SteerGauge } from '../components/rc/SteerGauge'
import { HazardButton, WinkerButton, WinkerIndicator } from '../components/rc/WinkerHazard'
import { SettingsDrawer } from '../components/SettingsDrawer'
import { useContainFit } from '../hooks/useContainFit'
import { useElementSize } from '../hooks/useElementSize'
import { CameraView } from '../render/CameraView'
import { VEHICLE as VEHICLE_GEOM } from '../generated/vehicle'
import type { ControlChannel } from '../ws/control'

/** センサー自体のネイティブ比率（4:3、`camera_node.py --size` の既定 640x480）。
 * 配信される画像はここから `bottomCrop` 分だけ下端を切ったもの */
const SENSOR_ASPECT = 4 / 3
/** 最初のフレームが届くまでの仮の比率。前方カメラは下端を `bottom_crop` だけ切って
 * 配信するので 4:3 より横長になる。実測が来次第 `frontAspect` に置き換わる */
const FALLBACK_ASPECT = SENSOR_ASPECT / (1 - VEHICLE_GEOM.camFront.bottomCrop)
/** バックミラーに出す範囲。**センサー全体（切る前）の縦の比率**で持つ（指示: 真ん中 1/3） */
const MIRROR_SENSOR_Y = { y0: 1 / 3, y1: 2 / 3 }
/** バックミラーの幅（メイン映像の幅に対する比） */
const MIRROR_WIDTH_FRAC = 0.36
/** HUD（計器の帯）の高さ（メイン映像の高さに対する比）と下限 [px]。
 * 0.3 ではボンネットからはみ出す（実機で確認）。0.21 は液晶の上の余白を含む */
const HUD_FRAC = 0.21
const HUD_MIN_PX = 90
/** シフトライト（台形の上辺の上に載る）の高さ [px]（`.shift-lights` の `height` と同じ値）。
 * **ライトを含めた全体の高さが `HUD_FRAC` になるよう、台形をこの分だけ低くする**（指示による） */
const SHIFT_LIGHTS_H_PX = 17
/** 台形の上辺から計器までの余白 [px] */
const HUD_PAD_TOP_PX = 12
/** ダイヤルの下に数字が入る分の高さ [px]。G メータには数字が無いが、**3つの高さを
 * 揃えるためあえて同じだけ差し引く** */
const DIAL_TEXT_RESERVE_PX = 34
/** 計器の背景の台形の、底辺の角（鋭角）[deg]（指示: 30°） */
const TRAP_ANGLE_DEG = 30
/** HUD の両端の列（ランプ・ハザード）の幅と、列の間隔 [px]（`.rc-cam-gauges` の `gap` の横と同じ）。
 * この2つの分だけ台形の左右の padding を減らし、両端の列を斜辺の下に入れる */
const HUD_EDGE_COL_PX = 100
const HUD_COL_GAP_PX = 18
/** 台形の縁の太さ [px]（`.rc-hud-trap-in` の `top` と同じ値） */
const TRAP_BORDER_PX = 2

const CAM_LABEL: Record<'front' | 'rear', string> = { front: '前方', rear: '後方' }

/** センサー座標の縦の範囲 → 配信画像（下端 `bottomCrop` 切り済み）の座標 */
function sensorToDelivered(cam: 'front' | 'rear', r: { y0: number; y1: number }) {
  const keep = 1 - (cam === 'front' ? VEHICLE_GEOM.camFront : VEHICLE_GEOM.camRear).bottomCrop
  return { y0: Math.min(1, r.y0 / keep), y1: Math.min(1, r.y1 / keep) }
}

export function RcView({ ch }: { ch: ControlChannel | null }) {
  // セレクタ無しの `useUi()` だと高頻度で変わるフィールドでも全体が再描画される。使う物だけ取る
  const lidarVisible = useUi((s) => s.lidarVisible)
  const pathGuide = useUi((s) => s.pathGuide)
  const lidarExpanded = useUi((s) => s.lidarExpanded)
  const rearPip = useUi((s) => s.rearPip)
  const set = useUi((s) => s.set)
  /* ミラー（`rearPip`）はここに含めない。LiDAR・進路ガイドの2つだけを一括切り替えする */
  const overlaysOn = lidarVisible && pathGuide
  const mainCam = useUi((s) => s.mainCam)
  const [frontAspect, setFrontAspect] = useState(FALLBACK_ASPECT)

  // R に入れたらメインを後方カメラへ、抜けたら前方へ戻す（`autoRearCam`）。**戻すのは自分で
  // 切り替えたときだけ**——R の間に人が前方へ戻したなら、抜けたときに触らない
  const reverse = useUi((s) => reverseSelected(s))
  const autoRearCam = useUi((s) => s.autoRearCam)
  const switchedRef = useRef(false)
  useEffect(() => {
    if (!autoRearCam) return
    const ui = useUi.getState()
    if (reverse) {
      if (ui.mainCam !== 'rear') {
        ui.set({ mainCam: 'rear' })
        switchedRef.current = true
      }
    } else if (switchedRef.current) {
      switchedRef.current = false
      if (ui.mainCam === 'rear') ui.set({ mainCam: 'front' })
    }
  }, [reverse, autoRearCam])

  const { ref: rcRef, size: videoSize } = useContainFit<HTMLDivElement>(frontAspect)
  const { ref: hudRef, size: hudSize } = useElementSize<HTMLDivElement>()
  const dialH = hudSize ? Math.max(40, hudSize.height - HUD_PAD_TOP_PX - DIAL_TEXT_RESERVE_PX) : null
  const mirrorCam = mainCam === 'front' ? 'rear' : 'front'
  const mirrorCrop = sensorToDelivered(mirrorCam, MIRROR_SENSOR_Y)
  // ミラーの箱の縦横比。センサーの横幅 : 切り出す縦幅（4:3 の 1/3 なら 4:1）
  const mirrorAspect = SENSOR_ASPECT / (MIRROR_SENSOR_Y.y1 - MIRROR_SENSOR_Y.y0)
  const mirrorW = videoSize ? videoSize.width * MIRROR_WIDTH_FRAC : 0
  const hudH = (videoSize ? Math.max(HUD_MIN_PX, videoSize.height * HUD_FRAC) : HUD_MIN_PX) - SHIFT_LIGHTS_H_PX
  // 台形の斜辺の水平ぶん。外側は高さ/tan(角)、内側は縁の太さぶん平行にずらした分
  // （上辺が縁の太さだけ下がり、斜辺は水平に 縁/sin(角) 内へ入る）
  const trapA = (TRAP_ANGLE_DEG * Math.PI) / 180
  const trapInset = hudH / Math.tan(trapA)
  const trapFootIn = TRAP_BORDER_PX / Math.sin(trapA)
  const trapInsetIn = (hudH - TRAP_BORDER_PX) / Math.tan(trapA) + trapFootIn

  const toggleOverlays = () => {
    const next = !overlaysOn
    set({ lidarVisible: next, pathGuide: next })
  }

  return (
    <div className="rc" ref={rcRef}>
      <div
        className="rc-cam-video"
        style={videoSize ? { width: videoSize.width, height: videoSize.height } : undefined}
        onClick={toggleOverlays}
        title={overlaysOn ? 'クリックで LiDAR・進路ガイドを隠す' : 'クリックで LiDAR・進路ガイドを表示'}
      >
        {/* 箱のサイズは常に front 側が決める。後方がメインのときは onAspect を渡さない */}
        <CameraView
          cam={mainCam}
          label={CAM_LABEL[mainCam]}
          variant="minimal"
          onAspect={mainCam === 'front' ? setFrontAspect : undefined}
          overlayBottomInset={hudH + SHIFT_LIGHTS_H_PX}
        />

        {rearPip && videoSize && <div className="rc-mirror-stay" />}
        {rearPip && videoSize && (
          <div
            className="rc-mirror"
            style={{ width: mirrorW, height: mirrorW / mirrorAspect }}
            onClick={(e) => {
              e.stopPropagation()
              set({ mainCam: mirrorCam })
            }}
            title="クリックで前後を入れ替え（パッドの ○ でも可）"
          >
            <CameraView
              cam={mirrorCam}
              label={CAM_LABEL[mirrorCam]}
              variant="minimal"
              onAspect={mirrorCam === 'front' ? setFrontAspect : undefined}
              guideDisabled
              srcCrop={mirrorCrop}
              mirror={mirrorCam === 'rear'}
            />
          </div>
        )}

        {/* 左上の列: 電池と温度（走行を切り上げる判断に使うので常時）と、その下にクリアランスソナー
            （後退中か、ゆっくり動いていて何かが近いときだけ）。映像の下側は運転に要る視界なので空けておく */}
        <div className="rc-left" onClick={(e) => e.stopPropagation()}>
          <RcStatus />
          <ParkSensor />
        </div>

        {/* LiDAR ミニマップ。ゲームのマップ表示のように丸く重ねる */}
        {lidarVisible && (
          <div
            className={`rc-lidar-mini ${lidarExpanded ? 'expanded' : ''}`}
            onClick={(e) => {
              e.stopPropagation()
              set({ lidarExpanded: !lidarExpanded })
            }}
            title={lidarExpanded ? 'クリックで縮小' : 'クリックで拡大'}
          >
            <LidarMini />
          </div>
        )}

        {/* 計器の HUD。ボンネットが映る下端の帯に重ねる。速度計は中央固定、
            舵角計・G メータはウィンカーごと左右に寄せる。ハザードは右下 */}
        <div className="rc-hud" ref={hudRef} style={{ height: hudH }} onClick={(e) => e.stopPropagation()}>
          <div
            className="rc-cam-gauges rc-hud-panel"
            style={
              {
                padding: `${HUD_PAD_TOP_PX}px ${Math.max(0, trapInset - HUD_EDGE_COL_PX - HUD_COL_GAP_PX)}px 0`,
                '--hud-edge-col': `${HUD_EDGE_COL_PX}px`,
                '--trap-inset': `${trapInset}px`,
                '--trap-inset-in': `${trapInsetIn}px`,
                '--trap-foot-in': `${trapFootIn}px`,
              } as CSSProperties
            }
          >
            <div className="rc-hud-trap" />
            <div className="rc-hud-trap-in" />
            <div className="rc-hud-screen" />
            <ShiftLights />
            <AssistLamps />
            <div className="gauge-group gauge-group-left">
              <div className="winker-side">
                <WinkerIndicator side="left" />
                <WinkerButton side="left" />
              </div>
              <SteerGauge dialHeight={dialH} />
            </div>
            <SpeedGauge dialHeight={dialH} />
            <div className="gauge-group gauge-group-right">
              <GMeter dialHeight={dialH} />
              <div className="winker-side">
                <WinkerIndicator side="right" />
                <WinkerButton side="right" />
              </div>
            </div>
            <HazardButton />
          </div>
        </div>
      </div>

      <SettingsDrawer ch={ch} />
    </div>
  )
}

