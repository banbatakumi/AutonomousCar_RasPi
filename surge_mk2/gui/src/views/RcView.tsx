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
 *          映像 : 車体図 = 4 : 1（下段が最低高さのときは車体図側が広がる。後述）
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
 * **メイン映像のうち PIP・LiDAR 以外をクリックすると、LiDAR・進路ガイドが
 * まとめて表示/非表示になる**（不要なときに一瞬で消せる非常用スイッチ）。
 * PIP（小さい映像）はこの切り替えに含めない（2026-08-20）——同じクリックで
 * 一緒に消えると「LiDAR を消したつもりが映像まで消えた」と紛らわしいため、
 * PIP は独立させ常時表示のままにしてある。
 * fps だけはどちらの操作でも消えない。
 *
 * **進路ガイドは大きい方（メイン）の映像にしか出さない**（2026-08-21）。
 * ガイドは前後どちらのカメラにも描けるようになったが（`CameraView.tsx`）、
 * PIP は前後の入れ替え用サムネイルなので小さくガイドを重ねても読めないし、
 * メインと2重に出ると紛らわしい。PIP 側の `CameraView` には常に
 * `guideDisabled` を渡し、on/off の切り替えは従来どおりメイン映像クリック
 * （`toggleOverlays`）だけが担う。
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
 *
 * ## 下段が最低高さのとき、車体図の列を横に広げて隙間を消す（2026-08-17）
 *
 * 下段（PIP＋メータ）が `BOTTOM_RESERVE_PX` の最低値まで縮む＝映像が
 * **高さ律速**で決まっているとき、映像の幅は列の本来の幅（`4fr` 分）より
 * 狭くなる。何もしないと映像の右側・車体図の左側の間に縦長の隙間ができる。
 *
 * `naturalCamsWidth`（`.rc` 自体の実測幅から `4:1` 比率で逆算した「列の本来の
 * 幅」）と実際の映像幅の差分だけ、`.rc` の `grid-template-columns` を
 * `${映像の実測幅}px minmax(0,1fr)` に上書きし、車体図の列（`grid-area:
 * meters`）に丸ごと渡す。横幅律速（隙間が無い）のときは上書きせず CSS 既定の
 * `4fr 1fr` に任せる——常に固定 px で上書きすると、ウィンドウ幅が変わっても
 * 映像列が追従しなくなるため。
 *
 * `naturalCamsWidth` は `.rc-cams` ではなく `.rc` 自体から測る。`.rc-cams` を
 * 測ると、上書き後の幅を拾って「上書き→再測定→再上書き」の循環に陥る
 * （`hooks/useContainFit.ts` の `widthOverride` コメント参照）。
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
import { VEHICLE as VEHICLE_GEOM } from '../generated/vehicle'
import type { ControlChannel } from '../ws/control'

/** センサー自体のネイティブ比率（4:3、`camera_node.py --size` の既定 640x480）。
 * 配信される画像はここから `bottomCrop` 分だけ下端を切ったもの（後述）。 */
const SENSOR_ASPECT = 4 / 3
/** 最初のフレームが届くまでの仮の比率。前方カメラは下端 25%（`vehicle.toml`
 * `sensors.cam_front.bottom_crop`）を切って配信するため、実際の映像は 4:3 より
 * 横長（≈16:9）になる。**ここを単純な 4:3 のままにしていたところ、映像を
 * 受信できていない間（起動直後・接続断中）だけ実際より縦長の箱として計算され、
 * `.rc-cams` が高さ律速に陥って幅が縮み、その分だけ車体図の列（`grid-area:
 * meters`）が異常に広がる不具合があった**（下の `camsGap` 参照）。フォールバックを
 * 実際の配信比率に合わせておけば、映像が来ていない間もほぼ同じ計算結果になり
 * 発生しない。実測が来次第、下記 `frontAspect` に置き換わる */
const FALLBACK_ASPECT = SENSOR_ASPECT / (1 - VEHICLE_GEOM.camFront.bottomCrop)
/** PIP の縦横比。前方/後方とも同じセンサーなので固定でよい */
const PIP_ASPECT = 4 / 3
/** 下段（PIP＋メータ）に最低限残す高さ [px]。映像を最大化する計算はこの分を先に引く
 * （`useContainFit.ts` の `reservePx`。無いと下段が0になりうる） */
const BOTTOM_RESERVE_PX = 150
/** ダイヤルの下に数字が入る分の高さ [px]。速度計・舵角計はここを引いた分がダイヤルの高さになる。
 * G メータには数字が無いが、**3つの高さを揃えるためあえて同じだけ差し引く**
 * （`GMeter` 側は使わなかった分だけ余白が残るが、大きさが不揃いになるよりましという判断） */
const DIAL_TEXT_RESERVE_PX = 34

/** 以下3つは `.rc` の CSS（`styles.css`）と値を合わせている。高さ律速で映像列に
 * 余白ができたとき、その分だけ車体図の列（`grid-area: meters`）を広げて隙間を
 * 埋めるために「列の本来の幅」を逆算するのに使う（`naturalCamsWidth` 参照）。 */
const RC_PADDING_PX = 6 // `.rc { padding: 6px }`
const RC_GAP_PX = 6 // `.rc { gap: 6px }`（列は2つなのでgapは1回分だけ引く）
const RC_CAMS_RATIO = 4 / 5 // `.rc { grid-template-columns: minmax(0,4fr) minmax(0,1fr) }`

const CAM_LABEL: Record<'front' | 'rear', string> = { front: '前方', rear: '後方' }

export function RcView({ ch }: { ch: ControlChannel | null }) {
  const ui = useUi()
  /* PIP（`rearPip`）はここに含めない（2026-08-20、コメント参照）。LiDAR・進路ガイドの
   * 2つだけをメイン映像クリックで一括切り替えする */
  const overlaysOn = ui.lidarVisible && ui.pathGuide
  const [mainCam, setMainCam] = useState<'front' | 'rear'>('front')
  const [frontAspect, setFrontAspect] = useState(FALLBACK_ASPECT)

  /* `.rc-cams` 自身ではなく `.rc` 自体の幅を測る。`.rc-cams` を測ると、下で
   * 列幅を上書きしたときにその上書き後の値を拾ってしまい循環する
   * （`useContainFit.ts` の `widthOverride` コメント参照）。`.rc` の幅は
   * 内部の列比率と無関係に親から決まるので、循環しない安定した基準になる。 */
  const { ref: rcRef, size: rcSize } = useElementSize<HTMLDivElement>()
  const naturalCamsWidth = rcSize
    ? Math.floor((rcSize.width - RC_PADDING_PX * 2 - RC_GAP_PX) * RC_CAMS_RATIO)
    : undefined
  const { ref: camsRef, size: videoSize } = useContainFit<HTMLDivElement>(
    frontAspect,
    BOTTOM_RESERVE_PX,
    naturalCamsWidth,
  )
  const { ref: bottomRef, size: bottomSize } = useElementSize<HTMLDivElement>()
  const dialH = bottomSize ? Math.max(40, bottomSize.height - DIAL_TEXT_RESERVE_PX) : null
  const pipCam = mainCam === 'front' ? 'rear' : 'front'

  /* 下段が高さ律速で最低値（`BOTTOM_RESERVE_PX`）に達すると、映像は列の本来の
   * 幅（`naturalCamsWidth`）より狭くなる。その差分だけ映像列を実測幅に縮め、
   * 空いた分は車体図の列（`grid-area: meters`）が丸ごと受け取る（指示による）。
   * 差が無い（横幅律速）ときは undefined を返し、CSS 既定の `4fr 1fr` に任せる
   * ——固定 px で上書きし続けると、ウィンドウ幅が変わっても映像列が追従
   * しなくなるため。 */
  const camsGap = videoSize && naturalCamsWidth != null ? naturalCamsWidth - videoSize.width : 0
  const rcGridStyle =
    camsGap > 0 && videoSize ? { gridTemplateColumns: `${videoSize.width}px minmax(0, 1fr)` } : undefined

  const toggleOverlays = () => {
    const next = !overlaysOn
    ui.set({ lidarVisible: next, pathGuide: next })
  }

  return (
    <div className="rc" ref={rcRef} style={rcGridStyle}>
      <div className="rc-cams" ref={camsRef}>
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
                guideDisabled
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
      <SettingsDrawer ch={ch} />
    </div>
  )
}
