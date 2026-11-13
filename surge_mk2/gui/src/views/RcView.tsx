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
 * ## レイアウト（2026-11-13 ステレオ化に合わせて変更。指示による）
 *
 * ```
 *   ┌──────────────────────────────────────────────┐
 *   │                                              │ ⚙   ← 映像の上 1/4（天井）は切って出さない
 *   │            メインカメラ（右＝主カメラ）           │
 *   ├─────┬[◀]シフトライト[▶][⚠]┐ ← 映像の幅の 1/2 で折れる（内角 135°）
 *   │     │ ┌速度┐  舵角        │ ╲
 *   │車体図│ │(大)│  G           │ ○ ╲                  │ ← 高さは映像の 1/2。ボンネットを覆う
 *   │     │ 温度×5  ランプ       │LiDAR╲                │
 *   └─────┴─────────────────────┴───────┴──────────────┘
 * ```
 *
 * **操縦はメインの映像を見て行う**ので、映像をタブいっぱいに出し、他は全部その上に重ねる。
 *
 * - **メイン映像**は前方ステレオの右（主）カメラ。後ろを見るカメラは無いのでバックミラーは無い。
 *   上 `RC_CROP_TOP` は天井しか映らないので切る（`CameraView` の `cropTop`。投影は元画像の座標のままなので
 *   進路ガイド等の重畳はずれない）
 * - **計器パネル**: 右カメラでは自分の車体（ボンネット）が映像の左下に映るので、そこを覆う（指示による）。
 *   形は固定: 左端に接し、高さは映像の 1/2、上辺は映像の幅の 1/2 で折れて内角 135°（45°）で下る。
 *   中身はこの箱に詰める（`panelLayout`）:
 *   - 左: 車体図（`DrivePanel`。電圧 S/P・トルク・スリップ・近接リング）
 *   - 上段: シフトライト（`ShiftLights`、仮想変速に従う）を、ウィンカー（`WinkerLamp`、点滅表示とボタンを兼ねる）で挟み、
 *     右端にハザード
 *   - 液晶: 大きい速度計の右に舵角計・G メータを縦に積む。箱は縦横比が近いので、横一列に並べると縦が余る
 *   - 下の帯（液晶の下）: 温度（`RcTemps`）・介入ランプ（`AssistLamps`）
 *   - LiDAR ミニマップ（`LidarMini`）: 斜辺の下の三角に内接する丸（以前は映像の右上。進む先の景色を空けるために移した）。
 *     クリック（パッドは R3）で映像の中央に大きく出す——パネルの中では広がれないため
 *   見た目はレース用ダッシュ風（カーボン調の筐体＋黒いガラス調の液晶）
 * - 電池・温度の左上の箱とクリアランスソナーの表示はやめた（電圧は車体図、温度は下の帯、
 *   ソナーは車体図の近接リングと音 `useParkBeep` で足りる）
 *
 * ## メイン映像の箱は「前方カメラのうち見えている部分」のサイズ
 *
 * 箱の形は前方カメラの実測値（`onAspect` → `frontAspect`。上を切った後の比率）。箱はタブの中に contain-fit で収める
 * ——cover（はみ出しを切る）にしないのは、パネルを車体の映る位置に正しく重ねるには
 * 映像の下端が見えている必要があるのと、左右の端（死角になりやすい）を削らないため。
 *
 * **メイン映像のうちパネル以外をクリックすると、進路ガイド（と障害物・計画の重畳）が
 * 表示/非表示になる**（不要なときに一瞬で消せる非常用スイッチ）。以前は LiDAR も一緒に消えたが、
 * LiDAR はパネルの部品になったので消さない。
 *
 * ## サイズ計算
 *
 * 映像は `useContainFit` で実測し、計器パネルの寸法とダイヤルの大きさはその映像の大きさから `panelLayout` で
 * 計算して px で渡す（液晶の実測は使わない——中身から液晶の幅を決めるので、実測すると循環する）。⚠ CSS の `aspect-ratio` 単体には頼らない。**箱の中に
 * `<canvas>`/`<svg>` のような replaced element がいると、intrinsic サイズ
 * （既定 300px 相当）に負けて縮んだままになる罠を実機で踏んだ**
 * （`hooks/useContainFit.ts` 冒頭のコメント）。
 *
 * 映像の比率も実際に届いた画像から決める（実機ではセンサーが要求どおりのモードを
 * 取らないことがある）。フレームが来るまでは `FALLBACK_ASPECT` で仮組みしておく。
 */
import { useState, type CSSProperties } from 'react'
import { useUi } from '../store/ui'
import { AssistLamps } from '../components/rc/AssistLamps'
import { DrivePanel } from '../components/rc/DrivePanel'
import { GMeter } from '../components/rc/GMeter'
import { LidarMini } from '../components/rc/LidarMini'
import { RcTemps } from '../components/rc/RcTemps'
import { LightButton } from '../components/rc/LightButton'
import { ShiftLights } from '../components/rc/ShiftLights'
import { SpeedGauge } from '../components/rc/SpeedGauge'
import { SteerGauge } from '../components/rc/SteerGauge'
import { HazardButton, WinkerLamp } from '../components/rc/WinkerHazard'
import { SettingsDrawer } from '../components/SettingsDrawer'
import { useContainFit } from '../hooks/useContainFit'
import { CameraView } from '../render/CameraView'
import { CAM_LABEL } from '../render/cameraLayout'
import { VEHICLE as VEHICLE_GEOM } from '../generated/vehicle'
import type { ControlChannel } from '../ws/control'

/** センサー自体のネイティブ比率（4:3、`camera_node.py --size` の既定 640x480）。
 * 配信される画像はここから `bottomCrop` 分だけ下端を切ったもの */
const SENSOR_ASPECT = 4 / 3
/** 映像の上端から切って出さない割合（天井しか映らない。指示: 上 1/4） */
const RC_CROP_TOP = 0.25
/** 最初のフレームが届くまでの仮の比率（配信の下端カットと、ここで切る上端の両方を引いた比率）。
 * 実測が来次第 `frontAspect` に置き換わる */
const FALLBACK_ASPECT = SENSOR_ASPECT / ((1 - VEHICLE_GEOM.camFront.bottomCrop) * (1 - RC_CROP_TOP))

// ── 計器パネルの形（指示: ボンネットを覆う。2026-11-13） ──
//
// 形は固定: 高さは映像の 1/2、上辺は映像の幅の 1/2 で折れ、内角 135°（斜辺は 45° で下る）。
// 中身はこの箱に合わせて詰める（`panelLayout`）。箱は縦横比が近いので、ダイヤルを横一列に並べると縦が余る——
// 液晶は「大きい速度計＋右に舵角・G の縦積み」の2列にし、温度・ランプ・ハザードの帯は斜辺の下の三角まで伸ばす
/** パネルの高さ（映像の高さに対する比） */
const PANEL_H_FRAC = 0.5
/** 上辺の長さ（映像の幅に対する比）＝斜辺の折れる位置 */
const PANEL_TOP_FRAC = 0.5
/** 上辺と斜辺の内角 [deg]（指示: 135°）。斜辺は水平から 180−135 = 45° で下る */
const PANEL_INNER_ANGLE_DEG = 135
/** 台形の縁の太さ [px]（`.rc-hud-trap-in` の `top` と同じ値） */
const TRAP_BORDER_PX = 2
/** パネルの内側の余白・部品の間隔 [px] */
const PANEL_PAD_PX = 8
const PANEL_GAP_PX = 8
/** 車体図の viewBox の幅/高さ（`DrivePanel.tsx` の viewBox と `.drivepanel-wrap` と同じ） */
const CAR_AR = 146 / 246.52
/** 上段（ウィンカー・シフトライト）の高さと、下との間隔 [px]（`.rc-hud-top` の `height`、`.rc-hud-main` の gap と同じ値） */
const TOP_ROW_H_PX = 22
const ROW_GAP_PX = 4
/** 下の帯（温度・介入ランプ）の高さ [px] */
const FOOT_H_PX = 44
/** LiDAR の丸の、斜辺の下の三角の内接円からの余白 [px] */
const LIDAR_MARGIN_PX = 10

// ── 液晶の中: [速度（大）][舵角 / G の縦積み] ──
/** 液晶の上下・左右の余白と列・行の間隔 [px]（`.rc-hud-screen>.rc-cam-gauges` の padding・gap と同じ値） */
const SCREEN_PAD_Y_PX = 6
const SCREEN_PAD_X_PX = 10
const SCREEN_GAP_PX = 12
/** 速度・舵角の下の数字がダイヤルにかからずに食う高さ [px]（数字 30px − 食い込み 6px） */
const DIAL_TEXT_RESERVE_PX = 26
/** 舵角計・G メータの大きさの、速度計に対する比 */
const SUB_DIAL_RATIO = 0.45
/** 舵角・速度のダイヤルの幅/高さ（viewBox 120×92。G は正方形） */
const DIAL_AR = 120 / 92
/** 舵角・G のダイヤルの高さの下限 [px] */
const DIAL_MIN_PX = 36


export function RcView({ ch }: { ch: ControlChannel | null }) {
  // セレクタ無しの `useUi()` だと高頻度で変わるフィールドでも全体が再描画される。使う物だけ取る
  const pathGuide = useUi((s) => s.pathGuide)
  const lidarExpanded = useUi((s) => s.lidarExpanded)
  const set = useUi((s) => s.set)
  const [frontAspect, setFrontAspect] = useState(FALLBACK_ASPECT)

  const { ref: rcRef, size: videoSize } = useContainFit<HTMLDivElement>(frontAspect)
  const L = panelLayout(videoSize?.width ?? 0, videoSize?.height ?? 0)

  // LiDAR はパネルの部品になったので、映像のクリックで消すのは進路ガイド（と障害物・計画の重畳）だけ
  const toggleGuide = () => set({ pathGuide: !pathGuide })
  const toggleLidar = (e: { stopPropagation: () => void }) => {
    e.stopPropagation()
    set({ lidarExpanded: !lidarExpanded })
  }

  return (
    <div className="rc" ref={rcRef}>
      <div
        className="rc-cam-video"
        style={videoSize ? { width: videoSize.width, height: videoSize.height } : undefined}
        onClick={toggleGuide}
        title={pathGuide ? 'クリックで進路ガイドを隠す' : 'クリックで進路ガイドを表示'}
      >
        <CameraView
          cam="front"
          label={CAM_LABEL.front}
          variant="minimal"
          onAspect={setFrontAspect}
          cropTop={RC_CROP_TOP}
          overlayBottomInset={L.panelH}
        />

        {/* LiDAR の拡大表示。パネルの中では広がれないので、拡大中だけ映像の中央に大きく出す */}
        {lidarExpanded && (
          <div className="rc-lidar-mini expanded" onClick={toggleLidar} title="クリックで縮小（パネルに戻す）">
            <LidarMini />
          </div>
        )}

        {/* 計器パネル。ボンネット（自分の車体が映る左下）を覆う（冒頭のレイアウト図） */}
        {videoSize && (
          <div
            className="rc-hud"
            style={
              {
                width: L.botW,
                height: L.panelH,
                '--trap-top': `${L.topW}px`,
                '--trap-top-in': `${L.topWIn}px`,
                '--trap-foot-in': `${L.footIn}px`,
              } as CSSProperties
            }
            onClick={(e) => e.stopPropagation()}
          >
            <div className="rc-hud-trap" />
            <div className="rc-hud-trap-in" />

            <div className="rc-hud-car" style={{ left: PANEL_PAD_PX, top: PANEL_PAD_PX, bottom: PANEL_PAD_PX, width: L.carW }}>
              <DrivePanel />
            </div>

            <div className="rc-hud-main" style={{ left: L.mainLeft, top: PANEL_PAD_PX, width: L.mainW, height: L.mainH }}>
              {/* 上段: ウィンカー（押せる矢印ランプ）でシフトライトを挟む */}
              <div className="rc-hud-top">
                <WinkerLamp side="left" />
                <ShiftLights />
                <WinkerLamp side="right" />
                <HazardButton />
              </div>
              <div className="rc-hud-screen">
                <div className="rc-cam-gauges">
                  <SpeedGauge dialHeight={L.speedH} />
                  <SteerGauge dialHeight={L.subH} />
                  <GMeter dialHeight={L.subH} />
                </div>
              </div>
            </div>

            {/* 下の帯（液晶の下） */}
            <div className="rc-hud-foot" style={{ left: L.mainLeft, bottom: PANEL_PAD_PX, width: L.mainW, height: FOOT_H_PX }}>
              <RcTemps />
              <AssistLamps />
              <LightButton />
            </div>

            {/* LiDAR ミニマップ。斜辺の下の三角に内接させる（ゲームのマップ表示のような丸）。拡大中は映像の中央に出すので空ける */}
            {!lidarExpanded && L.lidarR > 0 && (
              <div
                className="rc-lidar-mini in-panel"
                style={{ left: L.lidarCx - L.lidarR, top: L.lidarCy - L.lidarR, width: 2 * L.lidarR, height: 2 * L.lidarR }}
                onClick={toggleLidar}
                title="クリックで拡大"
              >
                <LidarMini />
              </div>
            )}
          </div>
        )}
      </div>

      <SettingsDrawer ch={ch} />
    </div>
  )
}

/** 計器パネルの寸法 [px]。形は映像の大きさで決まり（定数の冒頭コメント）、中身はそこに詰める */
function panelLayout(vw: number, vh: number) {
  const panelH = vh * PANEL_H_FRAC
  const topW = vw * PANEL_TOP_FRAC
  const slope = (180 - PANEL_INNER_ANGLE_DEG) * (Math.PI / 180) // 斜辺の水平からの角
  const run = 1 / Math.tan(slope) // 斜辺が 1px 下がる間に右へ進む量（45° なら 1）
  const botW = topW + panelH * run
  // 内側（地）の斜辺は、縁の太さぶん外側と平行に内へずらす（水平に 縁/sin(角)）
  const footIn = TRAP_BORDER_PX / Math.sin(slope)
  const topWIn = topW + TRAP_BORDER_PX * run - footIn

  // 左: 車体図（高さいっぱい）。右: 上段＋液晶（矩形の部分）と、その下の帯（三角まで）
  const carW = Math.max(0, panelH - 2 * PANEL_PAD_PX) * CAR_AR
  const mainLeft = PANEL_PAD_PX + carW + PANEL_GAP_PX
  const mainW = Math.max(0, topW - mainLeft - PANEL_PAD_PX)
  const mainH = Math.max(0, panelH - 2 * PANEL_PAD_PX - FOOT_H_PX - ROW_GAP_PX)
  // 斜辺の下の三角（直角が (topW, panelH)、縦の辺 panelH・横の辺 panelH·run）の内接円に LiDAR を置く。
  // 直角三角形の内接円の半径 = (縦 + 横 − 斜辺) / 2、中心は直角の頂点から縦横に半径ずつ入った所
  const legV = panelH
  const legH = panelH * run
  const inR = (legV + legH - Math.hypot(legV, legH)) / 2
  const lidarR = Math.max(0, inR - LIDAR_MARGIN_PX)
  const lidarCx = topW + inR
  const lidarCy = panelH - inR

  // 液晶: 速度計（高さ speedH）の右に舵角・G（高さ subH）を縦に積む
  const screenW = mainW
  const screenH = mainH - TOP_ROW_H_PX - ROW_GAP_PX
  const availW = screenW - 2 * SCREEN_PAD_X_PX - SCREEN_GAP_PX
  const availH = screenH - 2 * SCREEN_PAD_Y_PX
  // 速度計: 高さに収まり、かつ右の列（舵角の幅 = subH·DIAL_AR）と並べて幅に収まる大きさ
  const speedH = Math.max(0, Math.min(availH - DIAL_TEXT_RESERVE_PX, availW / (DIAL_AR * (1 + SUB_DIAL_RATIO))))
  // 舵角・G: 速度計の比で、縦に2つ（舵角の下の数字・行の間隔を含む）積んで高さに収まる大きさ
  const subH = Math.max(
    DIAL_MIN_PX,
    Math.min(speedH * SUB_DIAL_RATIO, (availH - DIAL_TEXT_RESERVE_PX - SCREEN_GAP_PX) / 2),
  )
  return { panelH, topW, botW, footIn, topWIn, carW, mainLeft, mainW, mainH, speedH, subH, lidarR, lidarCx, lidarCy }
}
