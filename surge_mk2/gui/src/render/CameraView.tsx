/**
 * カメラ映像。`/ws/camera/{front,left}` から JPEG が流れてくる（役割は `cameraLayout.ts`）。
 *
 * `createImageBitmap` でデコードして Canvas に描く。**React の state に
 * 画像を入れない**（1フレームごとに再レンダリングが走る）。
 *
 * ## レンズモデルと「暫定」表示（2026-10 魚眼化）
 *
 * IMX219 160° 広角レンズに替えたので、内部パラメータはチェッカーボード校正
 * （`tools/cam_calib/`）で求めた魚眼モデル（`VEHICLE.cam*.fisheye`、式は
 * `cameraModel.ts` ＝ `raspi/core/camera_model.py`）を使う。**校正値が無いカメラ**は
 * 従来どおり `hfov` のピンホールで近似するので、そのときだけ「校正前・暫定」の印を
 * 出す——ただし出すのは自動運転ビュー（`variant="full"`）のみ。
 *
 * 映像は「魚眼のまま」と「補正映像」（Pi の `telemetry_node` が仮想ピンホールに
 * remap したもの。設定パネル／タブバーで切替）の2通りが届きうる。どちらを
 * 表示しているかはサーバ真値（`status.camera_config`）で判断し、`lensFor()` が
 * 投影モデルを切り替える。外部パラメータ（高さ・俯角）は平面路面を仮定した近似の
 * ままで、実測で詰めるのは別作業。ラジコンビューは
 * 運転を楽しむ画面なので文字を極力出さない方針（`RcView.tsx`）で、下記
 * `variant="minimal"` では出さない。
 *
 * 高さ・俯角・レンズは車体の固定値として `vehicle.toml` から使う。GUIの調整項目は持たない。
 * （変わらない物理値にスライダーは要らない）。
 *
 * ## 上下クロップぶんの主点補正
 *
 * カメラは ISP の ScalerCrop で天井・ボンネット等が映る上下を最初から読み出さないことがある
 * （`camera_node.py` の `CAM_CROP`、値は `vehicle.toml` の `top_crop`・`bottom_crop`）。
 * そのとき光軸中心（センサー中心）は配信画像の中央ではなく、**上端から
 * (1/2 − topCrop)/(1 − topCrop − bottomCrop) の位置**にある（下1/4だけカットなら 66.7%）。
 * ここを画像中央（h/2）のまま投影するとガイド線がずれる。`cameraModel.ts` の `cropFrame()` で補正する。
 *
 * ## 走行中の車体姿勢（IMU）でも補正する
 *
 * カメラは車体に固定されているので、走行中に車体が pitch/roll すると
 * カメラの向きも一緒に傾く。`vs.pitch`/`vs.roll`（`TELEMETRY` の IMU 実測値。
 * 重力ベクトルで補正済みでドリフトしない）で毎フレーム補う。符号の向きは
 * 実車で確認済み（指示による）: **前が沈む（前輪側）ほど `pitch` は負**、
 * **右が沈む（右輪側）ほど `roll` は正**（2026-10-01のファーム修正で反転。それ以前は負）。
 *
 * - pitch: 固定取付角に `−vs.pitch` を足す。前のめり（pitch 負）になるほど
 *   カメラも一緒に下を向く（実効的な俯角が増える）ので符号を反転して足す。
 * - roll: 光軸まわりの回転として扱う。カメラが右に傾く（roll 正）と、写真の
 *   一般則どおり像は反時計回りに回って見える——回転角は `+vs.roll`。
 *   魚眼では「画像を主点中心に回す」近似が成り立たないので、**カメラ座標の
 *   光線を回してからレンズに通す**（ピンホールなら従来の画像回転と同じ結果）。
 *   **pitch/roll は独立に合成する近似**（厳密な3D外部パラメータではない）。
 * - IMU が無効（`imu_ok=false`）な間は 0 として扱い、固定値だけで描く。
 *
 * ## `variant`：ラジコンビューは文字を出さない
 *
 * `full`（既定・自動運転ビュー）は左上に「前方」などのラベルと fps を
 * 並べたタグを出し、校正前バッジも出す。`minimal`（ラジコンビュー）は
 * ラベルもバッジも出さず、fps だけを右下に小さく残す。**fps だけは
 * 消さない** — 配信が生きているかを判断する最後の手がかりのため。
 *
 * ## モード連動のデバッグ重畳
 *
 * 前カメラだけ、選ばれている自動運転モードに応じて2種類の重畳が自動で出る
 * （バンビの「セグメンテーションマスクを見たい」「ライントレースがどの線を
 * 認識しているか見たい」という要望より）。トグルは無く、モードを選んだ時点で出る
 * ——LiDAR ビューのギャップ重畳（`LidarView.tsx`）と同じ「見せたい情報は
 * 自動で出す」方針。
 *
 * - **`ftg_cam`/`cam_centerline`**: `cam_perception_node.py` の走行可否マスクを
 *   `/ws/camera/mask`（メイン映像とは別の専用 WS。`showMask` の間だけ繋ぐ）
 *   から受け、薄い半透明でそのまま重ねる（白＝走行可）。どちらのモードでも
 *   同じ推論結果を使っているので重畳も共通（`_CAM_MODES` 参照）。
 * - **`line_trace`**: `LineScan`（`/ws/telemetry` の `line_cam`）が持つ
 *   近傍・遠方の目標点を `makeProjector()`（`drawGuide` と共用）で画像へ
 *   逆投影し、シアン色のマーカーと線で重ねる。
 *
 * どちらも**メイン映像の WebSocket 接続とは独立**させてある——モードを
 * 切り替えるたびにメイン映像まで再接続されるのを避けるため。
 *
 * ## `onAspect`：実際に届いた画像の縦横比を親へ返す
 *
 * `RcView.tsx` は映像の箱を「取得したサイズのまま」表示したい（指示による）。
 * `camera_node.py --size` の既定値から比率を決め打ちしていたら、実機では
 * センサーが要求どおりのモードを取らないことがあり（冒頭コメント参照するまでも
 * なく `camera_node.py` 自身が「センサーネイティブモードを選ぶことがある」と
 * 警告している）、**箱の形が実際の画像とズレて余白ができた**。デコードした
 * `ImageBitmap` の実寸こそが正なので、フレームが来るたびにその比率を
 * `onAspect` で返す。比率が変わったときだけ呼ぶ（毎フレーム呼ぶと 15Hz で
 * 親を再レンダリングしてしまう）。
 *
 * ## 部品（2026-11-18 に分けた）
 *
 * - 受信・デコード・固まりの監視: `useFrameStream.ts`（`LatestFrameDecoder` で最新の1枚だけ）。マスクも同じ部品
 * - 重ねる描画: `cameraOverlays.ts`
 * - ここ: 毎フレームの描画（`useRaf`）と、ROI 選択の操作
 */
import { useEffect, useRef } from 'react'
import type { PointerEvent as ReactPointerEvent } from 'react'
import { live } from '../bus/live'
import { VEHICLE as VEHICLE_GEOM } from '../generated/vehicle'
import { useRaf } from '../hooks/useRaf'
import { useUi } from '../store/ui'
import type { ControlChannel } from '../ws/control'
import { camGeom, type CamId } from './cameraLayout'
import {
  AUTO_FRESH_MS, CAM_STALE_INTERVALS, CAM_STALE_MIN_MS, SCAN_FRESH_MS,
  drawArrowSignal, drawGuide, drawLag, drawLineTarget, drawObstacles, drawPlan, drawRoiDisarmedHint, drawRoiDrag,
  drawStale, drawTrackBox, guideFacing, isLagging,
} from './cameraOverlays'
import { useFrameStream } from './useFrameStream'

export function CameraView({
  cam,
  label,
  variant = 'full',
  onAspect,
  guideDisabled = false,
  ch = null,
  overlayBottomInset = 0,
  cropTop = 0,
}: {
  cam: CamId
  label: string
  variant?: 'full' | 'minimal'
  /** デコードした画像の実際の幅/高さ比。変わったときだけ呼ばれる */
  onAspect?: (aspect: number) => void
  /** true の間は `pathGuide` が ON でもこのインスタンスには描かない。
   * ラジコンビューの PIP（小さい方の映像）専用——**大きい方の映像だけに
   * ガイドを出す**ため */
  guideDisabled?: boolean
  /** `follow_object` のROI選択（ドラッグ矩形）に使う。前方カメラ以外・
   * 未指定の間はポインタ操作を一切拾わない */
  ch?: ControlChannel | null
  /** 映像の下端のうち、上に何か（ラジコンの HUD）が重なって見えない高さ [CSS px]。
   * 進路上の障害物の線がここに隠れるときは、見える下端に寄せて描く */
  overlayBottomInset?: number
  /** 映像の上端から切って出さない割合（0〜1）。ラジコンは天井が要らないので上 1/4 を切る。
   * 元画像全体の矩形を上へずらして置き、はみ出しをキャンバスで切るだけなので、
   * 投影（ガイド・障害物・計画の重畳）は元画像の座標のまま正しい位置に出る */
  cropTop?: number
}) {
  const ref = useRef<HTMLCanvasElement>(null)
  const guide = useUi((s) => s.pathGuide) && !guideDisabled
  const camHeight = VEHICLE_GEOM.camFront.height
  const statusRef = useRef<HTMLSpanElement>(null)
  // 描画ループから参照する値は ref 経由にする。
  const camHeightRef = useRef(camHeight)
  camHeightRef.current = camHeight
  // `guide` も同じ理由で ref 経由に。進路ガイドの ON/OFF のたびに
  // メイン映像の WS が再接続され、映像が瞬断していた（`camHeightRef` の
  // 配慮からここだけ漏れていた）
  const guideRef = useRef(guide)
  guideRef.current = guide
  // 進路上の障害物（LiDAR）と自動運転の計画の重畳。どちらも進路ガイドに従う
  // （ラジコンのメイン映像クリックでまとめて消せるように）。
  const obstacleOn = useUi((s) => s.obstacleOverlay) && guide
  const obstacleRef = useRef(obstacleOn)
  obstacleRef.current = obstacleOn
  // planner はモードを選んだだけで動いている（engage しなくても判断は出る）。ラジコン（`minimal`）で
  // 手動運転中に計画が重なると邪魔なので、そちらでは engage 中だけ出す。自動運転ビューでは常に出す
  // （engage 前に「何をしようとしているか」を見られる）
  const engaged = useUi((s) => !!s.auto?.engaged)
  const planOn = useUi((s) => s.planOverlay) && guide && cam === 'front' && (variant === 'full' || engaged)
  const planRef = useRef(planOn)
  planRef.current = planOn
  const insetRef = useRef(overlayBottomInset)
  insetRef.current = overlayBottomInset
  // 0 ≤ c < 1 に収める（1 にすると高さ 0 で割る）
  const cropRef = useRef(0)
  cropRef.current = Math.min(0.9, Math.max(0, cropTop))

  // ── モード連動のデバッグ重畳 ──
  //
  // `ftg_cam`/`cam_centerline` はセグメンテーションマスク、`line_trace` は
  // 認識した白線の目標点を前カメラにだけ重ねる。**どちらもメイン映像の
  // WS（`open()`）とは無関係**
  // ——モードを切り替えるたびにメイン映像まで再接続されると困るので、
  // ref 経由で `draw()` に渡す（`camHeightRef` と同じ理由）
  const auto = useUi((s) => s.auto)
  //: 補正映像（魚眼 → 仮想ピンホール、Pi 側で remap 済み）を表示中か。
  //: マスク・追跡枠・ROI 選択は**生画像（魚眼）の座標**で作られているので、
  //: 補正映像の上には重ねられない——その間は出さない（ROI も選ばせない）
  const undistorted = useUi(
    (s) => !!(s.cameraConfig?.undistort && s.cameraConfig.undistort_available?.[cam]),
  )
  //: `ftg_cam`/`cam_centerline` はどちらも `cam_perception_node.py` の同じ
  //: 走行可否マスクを publish する（`cam/mask`。`_CAM_MODES` 参照）ので同じ扱い
  const showMask = cam === 'front' && !undistorted
    && (auto?.mode === 'ftg_cam' || auto?.mode === 'cam_centerline')
  const showLineTarget = cam === 'front' && auto?.mode === 'line_trace'
  const showMaskRef = useRef(showMask)
  showMaskRef.current = showMask
  const showLineTargetRef = useRef(showLineTarget)
  showLineTargetRef.current = showLineTarget
  //: 矢印信号認識は`auto.mode`に紐づかない横断的な入力（`arrow_signal_node.py`の
  //: モジュールdocstring参照）なので、`showMask`/`showLineTarget`と違いモード判定を
  //: 挟まない——`drawArrowSignal()`側が`live.signal?.enabled`を見て出し分ける
  const showSignal = cam === 'front'
  const showSignalRef = useRef(showSignal)
  showSignalRef.current = showSignal

  // ── 対象追従（`follow_object`）のROI選択 ──
  //
  // ドラッグでROI矩形を選ぶ操作は前方カメラの `follow_object` 選択中だけ有効。
  // `showMask`/`showLineTarget` と同じ ref 経由（draw ループの useEffect 依存に
  // 入れない）。追跡結果のbbox重畳（`drawTrackBox()`）も同じ条件で出す
  const trackingMode = cam === 'front' && !undistorted && auto?.mode === 'follow_object'
  const trackingModeRef = useRef(trackingMode)
  trackingModeRef.current = trackingMode
  const chRef = useRef(ch)
  chRef.current = ch
  //: 直近の `draw()` が計算した映像の実描画矩形（CSS px、canvas基準）。
  //: ポインタ座標 → 正規化画像座標（0〜1）の変換に使う
  const imgBoxRef = useRef({ dx: 0, dy: 0, dw: 0, dh: 0 })
  //: ドラッグ中の選択矩形（正規化画像座標）。React state を介さず
  //: `draw()` が rAF で直接参照する（他の重畳と同じ方針）
  const dragRef = useRef<{ x0: number; y0: number; x1: number; y1: number } | null>(null)

  const clampNorm = (v: number) => Math.max(0, Math.min(1, v))
  const toNorm = (cv: HTMLCanvasElement, clientX: number, clientY: number) => {
    const rect = cv.getBoundingClientRect()
    const { dx, dy, dw, dh } = imgBoxRef.current
    if (dw <= 0 || dh <= 0) return null
    return {
      x: clampNorm((clientX - rect.left - dx) / dw),
      y: clampNorm((clientY - rect.top - dy) / dh),
    }
  }

  const onRoiPointerDown = (e: ReactPointerEvent<HTMLCanvasElement>) => {
    if (!trackingModeRef.current) return
    // DISARM 中は選択させない（cam_track_node は DISARM 中は追跡しない。サーバも断る）
    if (!live.vs?.armed) return
    const p = toNorm(e.currentTarget, e.clientX, e.clientY)
    if (!p) return
    e.currentTarget.setPointerCapture(e.pointerId)
    dragRef.current = { x0: p.x, y0: p.y, x1: p.x, y1: p.y }
  }
  const onRoiPointerMove = (e: ReactPointerEvent<HTMLCanvasElement>) => {
    const drag = dragRef.current
    if (!drag) return
    const p = toNorm(e.currentTarget, e.clientX, e.clientY)
    if (!p) return
    dragRef.current = { ...drag, x1: p.x, y1: p.y }
  }
  const onRoiPointerUp = () => {
    const drag = dragRef.current
    dragRef.current = null
    if (!drag) return
    const x0 = Math.min(drag.x0, drag.x1)
    const x1 = Math.max(drag.x0, drag.x1)
    const y0 = Math.min(drag.y0, drag.y1)
    const y1 = Math.max(drag.y0, drag.y1)
    // 小さすぎる矩形（クリックだけ・手ぶれ）は選択として送らない
    if (x1 - x0 < 0.02 || y1 - y0 < 0.02) return
    chRef.current?.setTrackRoi({ x0, y0, x1, y1 })
  }

  // ── 映像の受信 ──
  //
  // 親に返すのは上を切った後の（実際に見えている）比率。比率が変わったときだけ呼ぶ
  // （毎フレーム呼ぶと 15Hz で親を再レンダリングしてしまう）。カメラ・受け手が替わったら返し直す
  const lastAspectRef = useRef(0)
  useEffect(() => {
    lastAspectRef.current = 0
  }, [cam, onAspect])
  const main = useFrameStream(`/ws/camera/${cam}`, {
    watchdog: true,
    onFrame: (bmp) => {
      if (!onAspect) return
      const ar = bmp.width / (bmp.height * (1 - cropRef.current))
      if (Math.abs(ar - lastAspectRef.current) > 0.001) {
        lastAspectRef.current = ar
        onAspect(ar)
      }
    },
  })
  // `ftg_cam` の走行可否マスク（`/ws/camera/mask`）。**メイン映像の WS とは独立**——`showMask` が
  // 変わってもメイン映像は再接続しない。表示する間だけ繋ぎ、デコード中に届いた分は捨てる（従来どおり）
  const mask = useFrameStream(showMask ? '/ws/camera/mask' : null, { keepPending: false })

  // ── 毎フレームの描画 ──
  const rate = useRef({ last: performance.now() })
  useRaf(() => {
    const cv = ref.current
    const ctx = cv?.getContext('2d')
    if (!cv || !ctx) return
    const st = main.current
    const bitmap = st.bitmap
    const wsOpen = st.wsOpen
    const lastFrameMs = st.lastFrameMs
    const frameIntervalMs = st.frameIntervalMs
    const dpr = window.devicePixelRatio || 1
    const w = cv.clientWidth
    const h = cv.clientHeight
    if (!w || !h) return
    // 画素数は整数に丸めて比べる（拡大率が整数でないと `w * dpr` が小数になり、
    // 毎フレーム「違う」と判定してバッキングストアを作り直してしまう）
    const pw = Math.round(w * dpr)
    const ph = Math.round(h * dpr)
    if (cv.width !== pw || cv.height !== ph) {
      cv.width = pw
      cv.height = ph
    }
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0)
    ctx.fillStyle = '#07090b'
    ctx.fillRect(0, 0, w, h)

    if (bitmap) {
      // アスペクト比を保って収める。上端を `crop` だけ切るときは、見えている部分（高さ `dh·(1−crop)`）を
      // 収め、元画像全体の矩形（`dx,dy,dw,dh`）はその分だけ上へはみ出させる（はみ出しはキャンバスが切る）
      const crop = cropRef.current
      const s = Math.min(w / bitmap.width, h / (bitmap.height * (1 - crop)))
      const dw = bitmap.width * s
      const dh = bitmap.height * s
      const visH = dh * (1 - crop)
      const dx = (w - dw) / 2
      const dy = (h - visH) / 2 - crop * dh
      // 見えている部分の上端（キャンバス座標）。古さ・遅れの枠や文字はここを基準に描く
      const vy = dy + crop * dh
      imgBoxRef.current = { dx, dy, dw, dh }
      ctx.drawImage(bitmap, dx, dy, dw, dh)
      // 走行可否マスク（`ftg_cam`）。**元フレームと同じ画角から単純リサイズ
      // しただけ**（`cam_perception_node.py` の `_resize_nearest`）なので、
      // 映像と同じ矩形にそのまま重ねれば位置は揃う。薄く重ねるだけの
      // 最小実装（白＝走行可で明るく浮く、黒＝不可でそのまま暗い）
      if (showMaskRef.current && mask.current.bitmap) {
        ctx.save()
        ctx.globalAlpha = 0.4
        ctx.drawImage(mask.current.bitmap, dx, dy, dw, dh)
        ctx.restore()
      }
      // ガイドは映像そのもの（`dx,dy,dw,dh`）を基準に投影する。**箱の w/h では
      // 主点・焦点距離が箱基準になってしまう**——箱の形は CSS グリッドの行の
      // 高さで決まるだけで、カメラの実アスペクト比（`topCrop`/`bottomCrop` で変わる）とは
      // 一致せず、レターボックスが入る。箱基準のまま計算すると、その余白ぶんだけ
      // ガイドがズレる（前後配置のころ、後方カメラのガイドが上に大きくズレた）
      const nowMs = performance.now()
      const obstacles = obstacleRef.current && guideFacing(cam) && nowMs - live.lastScanMs < SCAN_FRESH_MS
      const plan = planRef.current && !!live.auto?.planner && nowMs - live.lastAutoMs < AUTO_FRESH_MS
      if ((guideRef.current && guideFacing(cam)) || obstacles || plan || showLineTargetRef.current || showSignalRef.current) {
        ctx.save()
        ctx.translate(dx, dy)
        // 映像の外（レターボックス）にはみ出して描かない
        ctx.beginPath()
        ctx.rect(0, crop * dh, dw, visH)
        ctx.clip()
        if (plan) drawPlan(ctx, dw, dh, camHeightRef.current)
        if (guideRef.current && guideFacing(cam)) drawGuide(ctx, dw, dh, camHeightRef.current, cam)
        if (obstacles) drawObstacles(ctx, dw, dh, camHeightRef.current, cam, Math.max(0, Math.min(dh, h - dy - insetRef.current)))
        if (showLineTargetRef.current) drawLineTarget(ctx, dw, dh, camHeightRef.current)
        if (showSignalRef.current) drawArrowSignal(ctx, dw, dh)
        ctx.restore()
      }
      // 対象追従（`follow_object`）のROI選択・追跡結果の重畳。
      // **メイン映像とは独立**（ドラッグ操作もbboxもここでしか意味を持たない）
      if (trackingModeRef.current) {
        if (dragRef.current) drawRoiDrag(ctx, dx, dy, dw, dh, dragRef.current)
        drawTrackBox(ctx, dx, dy, dw, dh)
        if (!live.vs?.armed) drawRoiDisarmedHint(ctx, dx, vy)
      }
      // **止まった映像を生きた映像に見せない。** 切断・受信の途絶では最後の1枚が残るので、
      // 暗くして古さを重ねる。DISARM 中は 1fps まで落とすので、しきい値は間隔に合わせて伸ばす
      const age = performance.now() - lastFrameMs
      const staleMs = Math.max(CAM_STALE_MIN_MS, frameIntervalMs * CAM_STALE_INTERVALS)
      if (!wsOpen || age > staleMs) drawStale(ctx, dx, vy, dw, visH, wsOpen ? age : null)
      else if (isLagging(age, frameIntervalMs)) drawLag(ctx, dx, vy, dw, visH, age)
    } else {
      ctx.fillStyle = '#3a444e'
      ctx.font = '12px ui-monospace, monospace'
      ctx.fillText('映像なし', 12, h / 2)
    }

    const now = performance.now()
    if (now - rate.current.last >= 1000) {
      const hz = (st.frames * 1000) / (now - rate.current.last)
      st.frames = 0
      rate.current.last = now
      if (statusRef.current) statusRef.current.textContent = `${hz.toFixed(0)} fps`
    }
  })

  return (
    <div className="camera" title={label}>
      <canvas
        ref={ref}
        onPointerDown={onRoiPointerDown}
        onPointerMove={onRoiPointerMove}
        onPointerUp={onRoiPointerUp}
        onPointerCancel={() => {
          dragRef.current = null
        }}
        style={trackingMode ? { cursor: 'crosshair', touchAction: 'none' } : undefined}
      />
      {variant === 'full' ? (
        <div className="camera-tag">
          {label}
          <span ref={statusRef} className="dim" />
          {guide && guideFacing(cam) && !camGeom(cam).fisheye && (
            <span className="badge-warn">ガイドは校正前・暫定</span>
          )}
          {undistorted && cam === 'front'
            && (auto?.mode === 'ftg_cam' || auto?.mode === 'cam_centerline' || auto?.mode === 'follow_object') && (
            <span className="badge-warn">補正映像中はマスク/追跡枠を出さない</span>
          )}
        </div>
      ) : (
        <div className="camera-fps">
          <span ref={statusRef} />
        </div>
      )}
    </div>
  )
}
