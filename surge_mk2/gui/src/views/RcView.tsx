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
 * ## レイアウト（2026-08-17 全面改訂 — GR 風ミニマル HUD）
 *
 * ```
 *   ┌─────────────────────────────────┬───┐
 *   │                        [LiDAR○] │車体│
 *   │        メインカメラ (fps 右下)     │図 │
 *   ├──────┬──────────────────────────┤   │
 *   │ [PIP]│      速度計 / G メータ     │   │
 *   └──────┴──────────────────────────┴───┘
 *          映像 : 車体図 = 4 : 1
 * ```
 *
 * **操縦はメインの映像を見て行う。** 右パネルは車体図（`RcBar`／`DrivePanel`）専任にし、
 * LiDAR はゲームのミニマップのように映像の右上へ丸く重ねた（`LidarMini`）。
 * LiDAR は**クリックで拡大縮小**（縮小＝映像の1/3高さ、拡大＝82%高さ）。
 *
 * ## PIP は映像の中ではなく「メータの横」に（2026-08-17）
 *
 * 以前は映像の左下に重ねていたが、**指示によりメータ行の左隣へ移動**した
 * （`.rc-cam-bottom`）。常に「今メインに出ていない方」のカメラを映し、
 * **クリックするとメインと入れ替わる**（`mainCam` state）。
 *
 * `.rc-cam-bottom`（PIP＋メータ行の器）は**幅をメイン映像の実測幅に固定**する
 * （`videoSize.width`）。メイン映像は列の左上に詰めているので、両者の左端も
 * 右端も揃う。**メータ（`.rc-cam-gauges`、`flex:1`）はこの固定幅のうち PIP の
 * 右側の残りいっぱいに広がる**ので、結果として速度計は「PIP の右端」と
 * 「映像の右端」のちょうど中間に来る——指示どおりの配置を、特別な座標計算を
 * せずに幅の制約だけで実現している。
 *
 * ## メイン映像の箱は「前方カメラ」のサイズに固定（2026-08-17）
 *
 * 入れ替え後（後方がメインに出ているとき）も、**箱の形は前方カメラの実測値の
 * まま変えない**（指示による）。`onAspect` は常に「今 front を映している方の
 * `CameraView`」（メインか PIP かは問わない）から受け取る（`frontAspect`）。
 * 後方の画像はこの固定サイズの箱の中に**中央合わせで収まる**——`CameraView`
 * の描画ループが元々やっている contain-fit（`Math.min(w/bw,h/bh)` で中央に
 * 描く）がそのまま効くので、ここでは何もしなくてよい。
 *
 * **メイン映像のうち PIP・LiDAR 以外をクリックすると、PIP・LiDAR・進路ガイドが
 * まとめて表示/非表示になる**（不要なときに一瞬で消せる非常用スイッチ。
 * 前後入れ替え自体は PIP を再表示すればまた使える）。
 * fps だけはどちらの操作でも消えない。
 *
 * 文字は極力出さない方針にした。「前方」「後方」ラベルとガイドの校正前バッジは
 * 消し（`CameraView` の `variant="minimal"`）、fps だけを画像隅に残している。
 *
 * ## 実測ベースのサイズ計算（2026-08-17）
 *
 * 映像は `useContainFit`、PIP とメータのダイヤルは `useElementSize` で
 * `.rc-cam-bottom` の実測高さから逆算した px を、それぞれ実測ベースで求めている。
 *
 * ⚠ どちらも CSS の `aspect-ratio` 単体には頼っていない。**箱の中に
 * `<canvas>`/`<svg>` のような replaced element がいると、intrinsic サイズ
 * （既定 300px 相当）に負けて縮んだままになる罠を実機で踏んだ**
 * （`hooks/useContainFit.ts` 冒頭のコメント）。
 *
 * 映像の比率も実際に届いた画像から決める。最初は `camera_node.py --size` の
 * 既定値から決め打ちしていたが、**実機ではセンサーが要求どおりのモードを
 * 取らず、箱の形が画像とズレて余白ができた**。フレームが来るまでは
 * `FALLBACK_ASPECT`（4:3）で仮組みしておく。
 *
 * 映像は列の**上に詰める**（`align-self: flex-start`、`styles.css`）。
 * 余った縦幅はすべて `.rc-cam-bottom`（`flex: 1`）に渡る。
 */
import { useState } from 'react'
import { useUi } from '../store/ui'
import { GMeter } from '../components/rc/GMeter'
import { LidarMini } from '../components/rc/LidarMini'
import { RcBar } from '../components/rc/RcBar'
import { SpeedGauge } from '../components/rc/SpeedGauge'
import { SteerGauge } from '../components/rc/SteerGauge'
import { SettingsDrawer } from '../components/SettingsDrawer'
import { useContainFit } from '../hooks/useContainFit'
import { useElementSize } from '../hooks/useElementSize'
import { CameraView } from '../render/CameraView'

/** 最初のフレームが届くまでの仮の比率（4:3）。実測が来次第、下記 `frontAspect` に置き換わる */
const FALLBACK_ASPECT = 4 / 3
/** PIP の縦横比。前方/後方とも同じセンサーなので固定でよい */
const PIP_ASPECT = 4 / 3
/** 下段（PIP＋メータ）に最低限残す高さ [px]。映像を最大化する計算はこの分を先に引く
 * （`useContainFit.ts` の `reservePx`。無いと下段が0になりうる） */
const BOTTOM_RESERVE_PX = 150
/** ダイヤルの下に数字が入る分の高さ [px]。速度計・舵角計はここを引いた分がダイヤルの高さになる。
 * G メータには数字が無いが、**3つの高さを揃えるためあえて同じだけ差し引く**
 * （`GMeter` 側は使わなかった分だけ余白が残るが、大きさが不揃いになるよりましという判断） */
const DIAL_TEXT_RESERVE_PX = 34

const CAM_LABEL: Record<'front' | 'rear', string> = { front: '前方', rear: '後方' }

export function RcView() {
  const ui = useUi()
  const overlaysOn = ui.rearPip && ui.lidarVisible && ui.pathGuide
  const [mainCam, setMainCam] = useState<'front' | 'rear'>('front')
  const [frontAspect, setFrontAspect] = useState(FALLBACK_ASPECT)
  const { ref: camsRef, size: videoSize } = useContainFit<HTMLDivElement>(frontAspect, BOTTOM_RESERVE_PX)
  const { ref: bottomRef, size: bottomSize } = useElementSize<HTMLDivElement>()
  const dialH = bottomSize ? Math.max(40, bottomSize.height - DIAL_TEXT_RESERVE_PX) : null
  const pipCam = mainCam === 'front' ? 'rear' : 'front'

  const toggleOverlays = () => {
    const next = !overlaysOn
    ui.set({ rearPip: next, lidarVisible: next, pathGuide: next })
  }

  return (
    <div className="rc">
      <div className="rc-cams" ref={camsRef}>
        <div
          className="rc-cam-video"
          style={videoSize ? { width: videoSize.width, height: videoSize.height } : undefined}
          onClick={toggleOverlays}
          title={overlaysOn ? 'クリックで後方・LiDAR・進路ガイドを隠す' : 'クリックで後方・LiDAR・進路ガイドを表示'}
        >
          {/* 箱のサイズは常に front 側が決める。後方がメインのときは onAspect を渡さない */}
          <CameraView
            cam={mainCam}
            label={CAM_LABEL[mainCam]}
            variant="minimal"
            onAspect={mainCam === 'front' ? setFrontAspect : undefined}
          />

          {/* LiDAR ミニマップ。ゲームのマップ表示のように丸く重ねる（車体イラスト・軌道なし） */}
          {ui.lidarVisible && (
            <div
              className={`rc-lidar-mini ${ui.lidarExpanded ? 'expanded' : ''}`}
              onClick={(e) => {
                e.stopPropagation()
                ui.set({ lidarExpanded: !ui.lidarExpanded })
              }}
              title={ui.lidarExpanded ? 'クリックで縮小' : 'クリックで拡大'}
            >
              <LidarMini />
            </div>
          )}
        </div>

        {/* PIP とメータ。幅を映像の実測幅に固定する（コメント参照） */}
        <div
          className="rc-cam-bottom"
          ref={bottomRef}
          style={videoSize ? { width: videoSize.width } : undefined}
        >
          {ui.rearPip && (
            <div
              className="rc-pip"
              /* PIP には速度計・舵角計のような下の数字が無いので、`dialH`（テキスト分を
               * 差し引いた高さ）ではなく行の実測高さそのまま使う——じゃないと下に
               * 使われない余白が残る（指示による修正） */
              style={bottomSize ? { width: bottomSize.height * PIP_ASPECT, height: bottomSize.height } : undefined}
              onClick={() => setMainCam(pipCam)}
              title="クリックで前後を入れ替え"
            >
              <CameraView
                cam={pipCam}
                label={CAM_LABEL[pipCam]}
                variant="minimal"
                onAspect={pipCam === 'front' ? setFrontAspect : undefined}
              />
            </div>
          )}

          <div className="rc-cam-gauges">
            <SteerGauge dialHeight={dialH} />
            <SpeedGauge dialHeight={dialH} />
            <GMeter dialHeight={dialH} />
          </div>
        </div>
      </div>

      <RcBar />
      <SettingsDrawer />
    </div>
  )
}
