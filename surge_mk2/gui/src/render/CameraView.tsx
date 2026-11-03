/**
 * カメラ映像。`/ws/camera/{front,rear}` から JPEG が流れてくる。
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
 * 高さ（設定パネルの `camHeight`）は `config/vehicle.toml` の `sensors.cam_front.z`
 * を初期値に、実写を見ながら追い込める——定規で測れる筐体の位置とレンズ光学中心が
 * ズレうるため。**俯角（取付角度）・レンズは調整UIを持たない**。一度ネジ止めしたら
 * 変わらない物理値で、`VEHICLE.camFront.pitch`/`fisheye`/`hfov`（`vehicle.toml` 直）をそのまま使う
 * （変わらない物理値にスライダーは要らない）。
 *
 * ## 下端クロップぶんの主点補正
 *
 * 前カメラは ISP の ScalerCrop でボンネット等が映る下1/4を最初から読み出さない
 * （`camera_node.py` の `CAM_BOTTOM_CROP`）。配信される画像はセンサー上側3/4だけなので、
 * 光軸中心（センサー中心）は配信画像の中央ではなく、**上端から 1/(2·(1−bottomCrop)) の
 * 位置**にある（下1/4カットなら 66.7%）。ここを画像中央（h/2）のまま投影すると、
 * ガイド線が実際より上に描かれる。`VEHICLE.camFront.bottomCrop` で補正する。
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
 * ## 後カメラにも同じガイドを出す
 *
 * 後カメラは車体に対し180°反対向きに付いている（`vehicle.toml` の
 * `sensors.cam_rear.yaw`）。そのため前カメラ用にそのまま流用できない点が3つある：
 *
 * 1. **奥行き・左右が反転する。** 後ろ向きのカメラにとっての「奥（画面奥）」は
 *    車体座標では −x（後方）、「左」は車体座標の −y（後ろを振り返ると左右が
 *    入れ替わるのと同じ）。`project()` へは `(−x, −y)` を渡す。
 * 2. **姿勢補正の符号が反転する。** 前が沈む（`pitch` 負）と前カメラは俯角が
 *    増えるが、後ろは持ち上がるので後カメラは俯角が減る——`dPitch` の符号を
 *    反転してから同じ式に通す。左右ロールも見ている向きが逆なので同様に反転する。
 * 3. **軌跡は「後退した場合」を描く。** 自転車モデルの曲率式 dθ/ds = tanδ/L は
 *    弧長 s の符号に依存しない（前進・後退で式の形が変わらない）ので、
 *    ループの刻み `ds` を負にするだけで「今の舵角のまま下がったら」の軌跡になる。
 *
 * ここも校正前の近似であることに変わりはない（冒頭コメント参照）。
 *
 * ## `variant`：ラジコンビューは文字を出さない
 *
 * `full`（既定・自動運転ビュー）は左上に「前方/後方」ラベルと fps を
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
 */
import { useEffect, useRef } from 'react'
import type { PointerEvent as ReactPointerEvent } from 'react'
import { live } from '../bus/live'
import { reverseSelected, useUi } from '../store/ui'
import { guardSocket, type SocketGuard } from '../ws/guard'
import { wsUrl } from '../ws/url'
import { VEHICLE as VEHICLE_GEOM } from '../generated/vehicle'
import { fisheyeLens, groundProjector, pinholeLens, type Lens } from './cameraModel'
import { BUMPER_X, HALF_WIDTH, findPathObstacle, scanPointsNow, type ObstacleLevel } from '../bus/pathObstacles'
import type { ControlChannel } from '../ws/control'

export function CameraView({
  cam,
  label,
  variant = 'full',
  onAspect,
  guideDisabled = false,
  ch = null,
  srcCrop,
  mirror = false,
  overlayBottomInset = 0,
}: {
  cam: 'front' | 'rear'
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
  /** 配信画像のうち描く縦の範囲（0〜1、上端基準）。ラジコンのバックミラー用。
   * 指定中・`mirror` 中は進路ガイド等の重畳を出さない（投影が画像全体を前提にしているため） */
  srcCrop?: { y0: number; y1: number }
  /** 左右反転して描く（バックミラー。後方カメラを鏡と同じ左右で見せる） */
  mirror?: boolean
  /** 映像の下端のうち、上に何か（ラジコンの HUD）が重なって見えない高さ [CSS px]。
   * 進路上の障害物の線がここに隠れるときは、見える下端に寄せて描く */
  overlayBottomInset?: number
}) {
  const ref = useRef<HTMLCanvasElement>(null)
  const guide = useUi((s) => s.pathGuide) && !guideDisabled
  const camHeight = useUi((s) => s.settings.camHeight)
  const statusRef = useRef<HTMLSpanElement>(null)
  // スライダー操作のたびに WS を再接続させないよう ref 経由で最新値を渡す
  // （下の draw ループの useEffect 依存に camHeight を入れない）
  const camHeightRef = useRef(camHeight)
  camHeightRef.current = camHeight
  // `guide` も同じ理由で ref 経由に。進路ガイドの ON/OFF のたびに
  // メイン映像の WS が再接続され、映像が瞬断していた（`camHeightRef` の
  // 配慮からここだけ漏れていた）
  const guideRef = useRef(guide)
  guideRef.current = guide
  // 進路上の障害物（LiDAR）と自動運転の計画の重畳。どちらも進路ガイドに従う
  // （ラジコンのメイン映像クリックでまとめて消せるように）。切り出し・反転中（ミラー）は出さない
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
  // 切り出し・反転も ref 経由（窓の大きさで値が変わっても WS を張り直さない）
  const srcCropRef = useRef(srcCrop)
  srcCropRef.current = srcCrop
  const mirrorRef = useRef(mirror)
  mirrorRef.current = mirror
  const insetRef = useRef(overlayBottomInset)
  insetRef.current = overlayBottomInset

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
  //: `ftg_cam` の走行可否マスク（`/ws/camera/mask`）。専用の別 `useEffect`
  //: （下記）で `showMask` の間だけ繋ぎ、`draw()` はここを読むだけ
  const maskBitmapRef = useRef<ImageBitmap | null>(null)

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

  useEffect(() => {
    const cv = ref.current
    if (!cv) return
    const ctx = cv.getContext('2d')
    if (!ctx) return

    let bitmap: ImageBitmap | null = null
    let frames = 0
    let lastRate = performance.now()
    let hz = 0
    let closed = false
    let ws: WebSocket | null = null
    let timer: number | undefined
    let raf = 0
    let decoding = false
    /** デコード中に届いた最新の1枚。**届いた側を捨てると、次の到着まで1フレーム分遅れる** */
    let pendingBuf: ArrayBuffer | null = null
    let lastAspect = 0
    /** 最後に絵が更新された時刻と、フレーム間隔の移動平均 [ms]（古さの判定に使う） */
    let lastFrameMs = 0
    let frameIntervalMs = 0
    let wsOpen = false
    /** 接続した時刻と、デコードを始めた時刻（0=していない）。固まりの検出に使う */
    let openedMs = 0
    let decodeStartMs = 0
    let decodeGen = 0
    let guard: SocketGuard | null = null

    const open = () => {
      const sock = new WebSocket(wsUrl(`/ws/camera/${cam}`))
      ws = sock
      sock.binaryType = 'arraybuffer'
      // **開くまで**の時間切れ（`ws/guard.ts`）。Safari は同じ相手への接続中を1本ずつしか進めないので、
      // 握手の返らない1本が残ると telemetry・control の張り直しまで後ろで止まる（2026-11-01 再現）。
      // 開いた後の半開きは下の watchdog が見る
      guard?.stop()
      guard = guardSocket(sock, () => {
        if (sock !== ws || closed) return
        ws = null
        timer = window.setTimeout(open, 800)
      })
      sock.onopen = () => {
        wsOpen = true
        openedMs = performance.now()
      }
      sock.onmessage = (ev) => {
        if (sock !== ws) return
        if (!(ev.data instanceof ArrayBuffer)) return
        // デコードが追いつかないときは**最新の1枚だけ残して古いものを捨てる**。溜めると
        // 遅延が伸び続け、届いた側を捨てると次の到着まで待つぶん遅れる
        if (decoding) {
          pendingBuf = ev.data
          return
        }
        void decode(ev.data)
      }
      sock.onclose = () => {
        // 監視が張り直したあとに、捨てた古い接続の close が遅れて来ても無視する
        if (sock !== ws) return
        guard?.stop()
        wsOpen = false
        if (!closed) timer = window.setTimeout(open, 800)
      }
      sock.onerror = () => sock.close()
    }
    const decode = async (buf: ArrayBuffer) => {
      decoding = true
      decodeStartMs = performance.now()
      const gen = ++decodeGen
      try {
        const bmp = await createImageBitmap(new Blob([buf], { type: 'image/jpeg' }))
        // アンマウント後に終わったデコードの絵は誰も使わない（解放しないとリークする）
        if (closed) {
          bmp.close()
          return
        }
        bitmap?.close()
        bitmap = bmp
        frames++
        const now = performance.now()
        if (lastFrameMs) {
          const dt = now - lastFrameMs
          frameIntervalMs = frameIntervalMs ? frameIntervalMs * 0.8 + dt * 0.2 : dt
        }
        lastFrameMs = now
        if (onAspect) {
          const ar = bmp.width / bmp.height
          if (Math.abs(ar - lastAspect) > 0.001) {
            lastAspect = ar
            onAspect(ar)
          }
        }
      } catch {
        /* 壊れたフレームは捨てる */
      } finally {
        // 監視が固まりとみなして先へ進めたあとに終わった古いデコードは、状態に触らない
        if (gen === decodeGen) decoding = false
      }
      if (gen !== decodeGen) return
      const next = pendingBuf
      pendingBuf = null
      if (next && !closed) void decode(next)
    }
    open()

    /** 固まりからの復帰。デコードが終わらない、または繋がっているのに映像が来ない状態を直す */
    const watchdog = window.setInterval(() => {
      if (closed) return
      const now = performance.now()
      if (decoding && now - decodeStartMs > CAM_DECODE_STUCK_MS) {
        decodeGen++ // 終わらないデコードは見捨てる
        decoding = false
        const next = pendingBuf
        pendingBuf = null
        if (next) void decode(next)
      }
      // 半開きの接続（`onclose` が来ない）は、フレームが途絶えたら自分で切って張り直す
      const lastAlive = Math.max(lastFrameMs, openedMs)
      if (wsOpen && now - lastAlive > CAM_RECONNECT_MS && ws) {
        const old = ws
        wsOpen = false
        ws = null
        old.close()
        window.clearTimeout(timer)
        open()
      }
    }, 1000)

    const draw = () => {
      raf = requestAnimationFrame(draw)
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

      if (bitmap && (srcCropRef.current || mirrorRef.current)) {
        // バックミラー: 縦の一部だけを切り出し、必要なら左右反転して収める。重畳は出さない
        const crop = srcCropRef.current ?? { y0: 0, y1: 1 }
        const sy = bitmap.height * crop.y0
        const sh = Math.max(1, bitmap.height * (crop.y1 - crop.y0))
        const s = Math.min(w / bitmap.width, h / sh)
        const dw = bitmap.width * s
        const dh = sh * s
        const dx = (w - dw) / 2
        const dy = (h - dh) / 2
        ctx.save()
        if (mirrorRef.current) {
          ctx.translate(w, 0)
          ctx.scale(-1, 1)
        }
        ctx.drawImage(bitmap, 0, sy, bitmap.width, sh, dx, dy, dw, dh)
        ctx.restore()
        const age = performance.now() - lastFrameMs
        const staleMs = Math.max(CAM_STALE_MIN_MS, frameIntervalMs * CAM_STALE_INTERVALS)
        if (!wsOpen || age > staleMs) drawStale(ctx, dx, dy, dw, dh, wsOpen ? age : null)
        else if (isLagging(age, frameIntervalMs)) drawLag(ctx, dx, dy, dw, dh, age)
      } else if (bitmap) {
        // アスペクト比を保って収める
        const s = Math.min(w / bitmap.width, h / bitmap.height)
        const dw = bitmap.width * s
        const dh = bitmap.height * s
        const dx = (w - dw) / 2
        const dy = (h - dh) / 2
        imgBoxRef.current = { dx, dy, dw, dh }
        ctx.drawImage(bitmap, dx, dy, dw, dh)
        // 走行可否マスク（`ftg_cam`）。**元フレームと同じ画角から単純リサイズ
        // しただけ**（`cam_perception_node.py` の `_resize_nearest`）なので、
        // 映像と同じ矩形にそのまま重ねれば位置は揃う。薄く重ねるだけの
        // 最小実装（白＝走行可で明るく浮く、黒＝不可でそのまま暗い）
        if (showMaskRef.current && maskBitmapRef.current) {
          ctx.save()
          ctx.globalAlpha = 0.4
          ctx.drawImage(maskBitmapRef.current, dx, dy, dw, dh)
          ctx.restore()
        }
        // ガイドは映像そのもの（`dx,dy,dw,dh`）を基準に投影する。**箱の w/h では
        // 主点・焦点距離が箱基準になってしまう**——箱の形は CSS グリッドの行の
        // 高さで決まるだけで、前後カメラそれぞれの実アスペクト比（`bottomCrop`
        // が違うため異なる）とは一致せず、レターボックス量が前後で変わる。
        // 箱基準のまま計算すると、その余白ぶんだけカメラごとに違う量で
        // ガイドがズレる（後方カメラのガイドが上に大きくズレる不具合になる）
        const nowMs = performance.now()
        const obstacles = obstacleRef.current && guideFacing(cam) && nowMs - live.lastScanMs < SCAN_FRESH_MS
        const plan = planRef.current && !!live.auto?.planner && nowMs - live.lastAutoMs < AUTO_FRESH_MS
        if ((guideRef.current && guideFacing(cam)) || obstacles || plan || showLineTargetRef.current || showSignalRef.current) {
          ctx.save()
          ctx.translate(dx, dy)
          // 映像の外（レターボックス）にはみ出して描かない
          ctx.beginPath()
          ctx.rect(0, 0, dw, dh)
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
          if (!live.vs?.armed) drawRoiDisarmedHint(ctx, dx, dy)
        }
        // **止まった映像を生きた映像に見せない。** 切断・受信の途絶では最後の1枚が残るので、
        // 暗くして古さを重ねる。DISARM 中は 1fps まで落とすので、しきい値は間隔に合わせて伸ばす
        const age = performance.now() - lastFrameMs
        const staleMs = Math.max(CAM_STALE_MIN_MS, frameIntervalMs * CAM_STALE_INTERVALS)
        if (!wsOpen || age > staleMs) drawStale(ctx, dx, dy, dw, dh, wsOpen ? age : null)
        else if (isLagging(age, frameIntervalMs)) drawLag(ctx, dx, dy, dw, dh, age)
      } else {
        ctx.fillStyle = '#3a444e'
        ctx.font = '12px ui-monospace, monospace'
        ctx.fillText('映像なし', 12, h / 2)
      }

      const now = performance.now()
      if (now - lastRate >= 1000) {
        hz = (frames * 1000) / (now - lastRate)
        frames = 0
        lastRate = now
        if (statusRef.current) statusRef.current.textContent = `${hz.toFixed(0)} fps`
      }
    }
    raf = requestAnimationFrame(draw)

    return () => {
      closed = true
      window.clearTimeout(timer)
      window.clearInterval(watchdog)
      cancelAnimationFrame(raf)
      guard?.stop()
      ws?.close()
      bitmap?.close()
    }
  }, [cam, onAspect])

  // `ftg_cam` の走行可否マスク（`/ws/camera/mask`）。**メイン映像の WS とは
  // 独立**——`showMask` が変わっても上の effect（メイン映像）は再接続しない。
  // 開閉・デコードのパターンはメイン映像の `open()` と同じ
  useEffect(() => {
    if (!showMask) return
    let ws: WebSocket | null = null
    let timer: number | undefined
    let closed = false
    let decoding = false
    let guard: SocketGuard | null = null

    const open = () => {
      const sock = new WebSocket(wsUrl('/ws/camera/mask'))
      ws = sock
      ws.binaryType = 'arraybuffer'
      // 開くまでの時間切れ（メイン映像と同じ理由）
      guard?.stop()
      guard = guardSocket(sock, () => {
        if (!closed) timer = window.setTimeout(open, 800)
      })
      ws.onmessage = async (ev) => {
        if (!(ev.data instanceof ArrayBuffer)) return
        if (decoding) return
        decoding = true
        try {
          const bmp = await createImageBitmap(new Blob([ev.data], { type: 'image/jpeg' }))
          if (closed) {
            bmp.close()
            return
          }
          maskBitmapRef.current?.close()
          maskBitmapRef.current = bmp
        } catch {
          /* 壊れたフレームは捨てる */
        } finally {
          decoding = false
        }
      }
      ws.onclose = () => {
        guard?.stop()
        if (!closed) timer = window.setTimeout(open, 800)
      }
      ws.onerror = () => sock.close()
    }
    open()

    return () => {
      closed = true
      window.clearTimeout(timer)
      guard?.stop()
      ws?.close()
      maskBitmapRef.current?.close()
      maskBitmapRef.current = null
    }
  }, [showMask])

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
          {guide && guideFacing(cam) && !(cam === 'front' ? VEHICLE_GEOM.camFront : VEHICLE_GEOM.camRear).fisheye && (
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

/** 進路ガイドを出すカメラか。進行方向（走行中は実測速度の符号、停止中は D/R レンジ）と
 * 同じ向きのカメラにだけ出す（前進中に後方カメラへ出しても意味が無い） */
function guideFacing(cam: 'front' | 'rear'): boolean {
  const speed = live.vs?.speed ?? 0
  // 止まっているときは選んでいるレンジ（MT は R 段）で決める
  const reverse = Math.abs(speed) > 0.05 ? speed < 0 : reverseSelected(useUi.getState())
  return (cam === 'rear') === reverse
}

/** 実測舵角から予測進路を路面平面に置き、カメラモデル（`lensFor()`）で画像に落とす。
 * `camHeightSetting` は設定パネルで校正できる値（`store/ui.ts` の `camHeight`。
 * **前カメラにしか適用しない**——後カメラは高さの調整UIを持たないので
 * `VEHICLE.camRear.height` を固定で使う）。`pitch`/レンズ（`fisheye`/`hfov`）/`bottomCrop` は
 * `vehicle.toml` 由来の固定値（`cam` に応じて `camFront`/`camRear` を選ぶ）。
 * `vs.pitch`/`vs.roll`（IMU 実測）で走行中の車体姿勢ぶんを毎フレーム補正する
 * （符号の向きはファイル冒頭コメント参照。後カメラは反転して使う）。 */
function drawGuide(
  ctx: CanvasRenderingContext2D,
  w: number,
  h: number,
  camHeightSetting: number,
  cam: 'front' | 'rear',
) {
  const vs = live.vs
  if (!vs) return
  const isFront = cam === 'front'
  const project = makeProjector(w, h, camHeightSetting, cam)

  // 自転車モデルの曲率式は弧長の符号によらないので、後カメラは ds を負にして
  // 「今の舵角のまま後退したら」の軌跡を積分する（ファイル冒頭コメント参照）
  const ds = isFront ? 0.08 : -0.08

  // 車体の左右端をなぞる2本
  for (const off of [-VEHICLE_HALF_WIDTH, VEHICLE_HALF_WIDTH]) {
    ctx.beginPath()
    let x = 0
    let y = 0
    let th = 0
    let started = false
    for (let i = 0; i < 45; i++) {
      th += (ds / VEHICLE_GEOM.wheelbase) * Math.tan(vs.steer_actual)
      // 舵いっぱいだと弧が横〜後ろへ回り込み、カメラに映らない上に魚眼で暴れる。
      // 進行方向が90°を超えた先は進路ガイドとして意味が無いので打ち切る
      if (Math.abs(th) > Math.PI / 2) break
      x += ds * Math.cos(th)
      y += ds * Math.sin(th)
      // 車体端は進行方向の法線（左）へ off ずらす。車両座標の y 方向へずらすと、
      // 旋回して車体が向きを変えたぶん幅が潰れて（または広がって）崩れる。
      // 車両座標 (x=前, y=左) をそのまま渡す（後カメラの反転は投影器の中）
      const p = project(x - off * Math.sin(th), y + off * Math.cos(th))
      // 写らない点は線を繋がずに分断する（繋ぐと画面を横切る長い直線になる）
      if (!p) {
        started = false
        continue
      }
      if (!started) {
        ctx.moveTo(p[0], p[1])
        started = true
      } else ctx.lineTo(p[0], p[1])
    }
    ctx.strokeStyle = 'rgba(255,198,63,0.75)'
    ctx.lineWidth = 2
    ctx.stroke()
  }
}

/** どのレンズモデルで描くか。**補正映像を表示中なら仮想ピンホール**（Pi が remap 済み）、
 * そうでなければ校正値があれば魚眼、無ければ `hfov` のピンホール近似。
 * 補正映像かどうかはサーバ真値（`status.camera_config.undistort` と、そのカメラが
 * 校正済みで実際に補正されているか `undistort_available`）で決める。 */
export function lensFor(cam: 'front' | 'rear', w: number, h: number): Lens {
  const geom = cam === 'front' ? VEHICLE_GEOM.camFront : VEHICLE_GEOM.camRear
  const cfg = useUi.getState().cameraConfig
  if (cfg?.undistort && cfg.undistort_available?.[cam]) {
    return pinholeLens(w, h, geom.undistortHfov, geom.bottomCrop, 'undistorted')
  }
  if (geom.fisheye) return fisheyeLens(w, h, geom.fisheye, geom.bottomCrop)
  return pinholeLens(w, h, geom.hfov, geom.bottomCrop)
}

/** 路面座標（base_link、x=前 y=左）→ 画像座標の投影器を作る。IMU による姿勢（pitch/roll）補正込み。
 * `drawGuide`（進路ガイド）と `drawLineTarget`（認識ラインの重畳）が共用する
 * ——同じカメラ内部・外部パラメータ・IMU補正を2箇所に書くと片方だけ直し忘れる。
 *
 * レンズ（魚眼/補正映像/未校正ピンホール）は `lensFor()` が選ぶ。姿勢は
 * **カメラ座標の光線を回してから**レンズに通す——魚眼では「画像を主点まわりに
 * 回す」近似が成り立たない（端ほど像の回り方が違う）ため。ピンホールでは
 * 従来の画像面回転と同じ結果になる。 */
function makeProjector(w: number, h: number, camHeightSetting: number, cam: 'front' | 'rear') {
  const vs = live.vs
  const isFront = cam === 'front'
  const geom = isFront ? VEHICLE_GEOM.camFront : VEHICLE_GEOM.camRear
  const height = isFront ? camHeightSetting : geom.height
  const lens = lensFor(cam, w, h)

  // IMU が無効・未受信の間は姿勢ぶんの補正をかけない（0 扱い＝固定値のみ）。
  // 後カメラは前カメラと逆向きを見ているので、姿勢補正の符号を反転する
  // （前が沈む＝後ろは持ち上がる。左右も振り返った側から見るので鏡像になる）
  const dPitch = (vs?.imu_ok ? vs.pitch : 0) * (isFront ? 1 : -1)
  const dRoll = (vs?.imu_ok ? vs.roll : 0) * (isFront ? 1 : -1)

  // 前が沈む（dPitch 負）ほどカメラも一緒に下を向くので、符号を反転して足す。
  // roll: 光軸まわりの回転。右が沈む（dRoll 正）とカメラも右に傾き、
  // 写真の一般則どおり像は光軸を中心に反時計回りへ回って見える。
  // **返す関数は base_link 基準の地面点 `(x=前, y=左)` を受け取る**（カメラ取付位置の
  // 差し引き・後カメラの反転は `groundProjector` がやる）
  return groundProjector(
    lens,
    { x: geom.x, y: geom.y, height, pitch: geom.pitch - dPitch },
    cam,
    dRoll,
  )
}

/**
 * `line_trace` が認識している白線の近傍・遠方点（`LineScan`、既に base_link
 * 座標に逆投影済み）を、進路ガイドと同じ路面→画像投影でカメラ映像に重ねる。
 * **前カメラでしか意味を持たない**（`line_perception_node.py` は前方カメラ
 * しか見ていない）ので `cam` は取らず、常に前カメラの内部・外部パラメータで
 * 投影する。進路ガイドの琥珀色と混同しないよう別の色（シアン系）にする。
 * 見えていない点は描かない——「見失っている」こと自体は判断欄の `reason` で分かる。
 */
function drawLineTarget(ctx: CanvasRenderingContext2D, w: number, h: number, camHeightSetting: number) {
  const line = live.lineCam
  if (!line || (!line.near_seen && !line.far_seen)) return
  const project = makeProjector(w, h, camHeightSetting, 'front')

  const nearPt = line.near_seen ? project(line.near_x, line.near_y) : null
  const farPt = line.far_seen ? project(line.far_x, line.far_y) : null
  const pts = [nearPt, farPt].filter((p): p is [number, number] => p != null)
  if (pts.length === 0) return

  ctx.strokeStyle = 'rgba(80,220,255,0.85)'
  ctx.fillStyle = 'rgba(80,220,255,0.85)'
  if (nearPt && farPt) {
    ctx.beginPath()
    ctx.moveTo(nearPt[0], nearPt[1])
    ctx.lineTo(farPt[0], farPt[1])
    ctx.lineWidth = 2
    ctx.stroke()
  }
  for (const [x, y] of pts) {
    ctx.beginPath()
    ctx.arc(x, y, 5, 0, Math.PI * 2)
    ctx.fill()
  }
}

/**
 * `arrow_signal_node.py` の直近判定（`live.signal`）を前方カメラ映像に重畳する。
 * 専用スペースは使わず、ROI帯の枠線＋方向ラベルを映像へ直接描くだけ
 * （会場でのHSVライブチューニング中に見る想定）。`enabled=false`（設定パネルで
 * OFF）の間は何も描かない。確定（`confirmed_value`、`route/select`として実際に
 * 送信中）していれば目立つ色・大きな文字で、confirm前の生判定（`value`）は
 * 控えめに出す——「今まさに見えている」と「確定して経路が切り替わった」を
 * 見た目で区別できるようにする。
 */
function drawArrowSignal(ctx: CanvasRenderingContext2D, w: number, h: number) {
  const sig = live.signal
  if (!sig || !sig.enabled) return
  const y0 = sig.roi_top * h
  const y1 = sig.roi_bottom * h
  const confirmed = sig.confirmed_value
  const lit = sig.lit_frac > 0.005

  ctx.save()
  ctx.strokeStyle = confirmed ? 'rgba(255,204,0,0.9)' : 'rgba(255,255,255,0.25)'
  ctx.lineWidth = confirmed ? 3 : 1
  ctx.strokeRect(0, y0, w, y1 - y0)

  const label = confirmed
    ? { left: '◀ 左折', right: '右折 ▶' }[confirmed]
    : (lit ? (sig.value || '判定中') : '')
  if (label) {
    ctx.font = confirmed ? 'bold 22px sans-serif' : '13px sans-serif'
    ctx.fillStyle = confirmed ? 'rgba(255,204,0,0.95)' : 'rgba(255,255,255,0.6)'
    ctx.textAlign = 'center'
    ctx.fillText(label, w / 2, Math.max(16, y0 - 8))
  }
  ctx.restore()
}

/** ドラッグ中のROI選択矩形（正規化画像座標）を映像に重ねる。
 * `drag` は始点・終点で、順序（左上/右下）は問わない——描く直前に整える。 */
function drawRoiDrag(
  ctx: CanvasRenderingContext2D,
  dx: number,
  dy: number,
  dw: number,
  dh: number,
  drag: { x0: number; y0: number; x1: number; y1: number },
) {
  const x0 = Math.min(drag.x0, drag.x1)
  const x1 = Math.max(drag.x0, drag.x1)
  const y0 = Math.min(drag.y0, drag.y1)
  const y1 = Math.max(drag.y0, drag.y1)
  ctx.save()
  ctx.strokeStyle = 'rgba(255,210,80,0.9)'
  ctx.lineWidth = 2
  ctx.setLineDash([6, 4])
  ctx.strokeRect(dx + x0 * dw, dy + y0 * dh, (x1 - x0) * dw, (y1 - y0) * dh)
  ctx.restore()
}

/** 点群・判断がこれより古ければ重ねない [ms]（古い位置に障害物を描かない） */
const SCAN_FRESH_MS = 400
const AUTO_FRESH_MS = 600

/** 進路上の障害物の段ごとの色（`bus/pathObstacles.ts`）。赤・琥珀は意味色、白は「あるだけ」 */
const OB_COLOR: Record<ObstacleLevel, string> = {
  danger: '#f2544b',
  caution: '#e8ab21',
  info: 'rgba(255,255,255,0.55)',
}

/**
 * 進路上の障害物（LiDAR）を映像に重ねる（2026-11-03）。今の舵角のまま進んだら最初に
 * 当たる点群に印を付け、**車体の前端（後退なら後端）が当たる位置に横線**（止まる線）を引く。
 * 色は当たるまでの時間で決まる（近づいていなければ白）。距離は線の上に出す。
 * 計算は `bus/pathObstacles.ts`（点群を取得から今までの車の動きぶん戻してから掃く）
 */
function drawObstacles(
  ctx: CanvasRenderingContext2D, w: number, h: number, camHeightSetting: number, cam: 'front' | 'rear',
  /** 映像のうち見えている下端の y（これより下は HUD に隠れる） */
  visibleBottom: number,
) {
  const scan = live.scan
  const vs = live.vs
  if (!scan || !vs) return
  const ob = findPathObstacle(scanPointsNow(scan, vs), vs.steer_actual, vs.speed, cam === 'rear')
  if (!ob) return
  const project = makeProjector(w, h, camHeightSetting, cam)
  const col = OB_COLOR[ob.level]
  const lit = ob.level !== 'info'

  ctx.save()
  // 当たる点群（足元＝路面に落とした位置）
  ctx.fillStyle = col
  for (const [x, y] of ob.hits) {
    const p = project(x, y)
    if (p) ctx.fillRect(p[0] - 2.5, p[1] - 2.5, 5, 5)
  }
  // 止まる線。車体の端の位置で、車体の幅いっぱいに
  const { x, y, th } = ob.pose
  const bx = ob.reverse ? BUMPER_X.rear : BUMPER_X.front
  const ex = x + bx * Math.cos(th)
  const ey = y + bx * Math.sin(th)
  const nx = -Math.sin(th) * HALF_WIDTH
  const ny = Math.cos(th) * HALF_WIDTH
  const pa = project(ex + nx, ey + ny)
  const pb = project(ex - nx, ey - ny)
  let a: [number, number]
  let b: [number, number]
  // **近いほど路面の手前＝映像の下端（HUD の裏・画面の外）に入る。** 一番大事な近距離が
  // 見えなくなるので、見える下端に寄せて描く（横の位置は投影できた方を使い、無ければ中央）
  const edgeY = visibleBottom - 10
  if (pa && pb && Math.max(pa[1], pb[1]) <= edgeY) {
    a = pa
    b = pb
  } else {
    const xs = [pa, pb].filter((p): p is [number, number] => p != null).map((p) => p[0])
    const x0 = xs.length === 2 ? Math.min(...xs) : w * 0.35
    const x1 = xs.length === 2 ? Math.max(...xs) : w * 0.65
    const span = Math.max(x1 - x0, w * 0.18)
    const mid = (x0 + x1) / 2
    a = [Math.max(4, mid - span / 2), edgeY]
    b = [Math.min(w - 4, mid + span / 2), edgeY]
  }
  ctx.strokeStyle = col
  ctx.lineCap = 'round'
  ctx.lineWidth = ob.level === 'danger' ? 6 : lit ? 4 : 2
  if (lit) {
    ctx.shadowColor = col
    ctx.shadowBlur = 12
  }
  ctx.beginPath()
  ctx.moveTo(a[0], a[1])
  ctx.lineTo(b[0], b[1])
  ctx.stroke()
  ctx.shadowBlur = 0
  // 距離。線の真ん中の上に、読める地の上で
  const mx = (a[0] + b[0]) / 2
  const my = Math.min(a[1], b[1]) - 10
  const text = `${ob.distM.toFixed(2)} m`
  ctx.font = "bold 15px 'DIN Alternate', 'Hiragino Sans', sans-serif"
  ctx.textAlign = 'center'
  ctx.textBaseline = 'bottom'
  const tw = ctx.measureText(text).width
  ctx.fillStyle = 'rgba(0,0,0,0.6)'
  ctx.fillRect(mx - tw / 2 - 6, my - 18, tw + 12, 20)
  ctx.fillStyle = lit ? col : '#fff'
  ctx.fillText(text, mx, my)
  ctx.restore()
}

/** 自動運転の計画の色。計測値の線と同じ青（`--trace`）を指令の進路に、経路は紫 */
const PLAN_CMD = '#4aa8ff'
const PLAN_ROUTE = 'rgba(200,120,255,0.85)'
/** 経路を描く範囲 [m]（base_link の前方） */
const PLAN_ROUTE_MIN_X = 0.15
const PLAN_ROUTE_MAX_X = 6
const PLAN_ROUTE_MAX_Y = 3

/**
 * 自動運転の計画を前方映像に重ねる（2026-11-03）。**planner が何をしようとしているか**と
 * **実際に見えているもの**のずれを映像の上で読むため。出すもの:
 *
 * - 指令の進路（青の破線）: `target_steer` のまま進んだ弧。実測舵角の進路ガイド（琥珀）と
 *   並べると、舵の遅れや「指令と違う向きに行っている」が分かる。長さは `target_speed` の 1.5 秒ぶん
 * - 地図を持つ planner（`phase` がある＝slam2d 系）のとき: 今走っている経路（紫）、
 *   Pure Pursuit が狙っている点（白い菱形）、検出した動的障害物（赤い輪）。どれも map
 *   フレームなので、自己位置（`pose_*`）で base_link に直してから投影する
 */
function drawPlan(ctx: CanvasRenderingContext2D, w: number, h: number, camHeightSetting: number) {
  const a = live.auto
  // 判断を出せていない（モデル未選択など）間は描かない。指令 0 の線は情報にならない
  if (!a || !a.ready) return
  const project = makeProjector(w, h, camHeightSetting, 'front')
  ctx.save()

  // 指令の進路
  const len = Math.max(0.8, Math.min(4, Math.abs(a.target_speed) * 1.5))
  if (a.target_speed >= 0) {
    const k = Math.tan(a.target_steer) / VEHICLE_GEOM.wheelbase
    ctx.beginPath()
    let x = 0
    let y = 0
    let th = 0
    let started = false
    for (let s = 0; s < len; s += 0.05) {
      th += 0.05 * k
      if (Math.abs(th) > Math.PI / 2) break
      x += 0.05 * Math.cos(th)
      y += 0.05 * Math.sin(th)
      const p = project(x, y)
      if (!p) {
        started = false
        continue
      }
      if (started) ctx.lineTo(p[0], p[1])
      else ctx.moveTo(p[0], p[1])
      started = true
    }
    ctx.setLineDash([10, 7])
    ctx.strokeStyle = PLAN_CMD
    ctx.lineWidth = 3
    ctx.shadowColor = PLAN_CMD
    ctx.shadowBlur = 6
    ctx.stroke()
    ctx.setLineDash([])
    ctx.shadowBlur = 0
  }

  // 地図を持つ planner のときだけ（自己位置が意味を持つ）
  if (a.phase) {
    const c = Math.cos(a.pose_yaw)
    const sn = Math.sin(a.pose_yaw)
    const toBase = (mx: number, my: number): [number, number] => {
      const dx = mx - a.pose_x
      const dy = my - a.pose_y
      return [dx * c + dy * sn, -dx * sn + dy * c]
    }
    const inView = (x: number, y: number) =>
      x > PLAN_ROUTE_MIN_X && x < PLAN_ROUTE_MAX_X && Math.abs(y) < PLAN_ROUTE_MAX_Y

    const line = live.map?.raceline
    if (line && line.length >= 4) {
      ctx.beginPath()
      let started = false
      let prev: [number, number] | null = null
      for (let i = 0; i + 1 < line.length; i += 2) {
        const [x, y] = toBase(line[i]!, line[i + 1]!)
        const p = inView(x, y) ? project(x, y) : null
        // 範囲外・写らない点・離れた点（経路の継ぎ目）では線を切る
        const jump = prev ? Math.hypot(x - prev[0], y - prev[1]) > 0.5 : false
        prev = [x, y]
        if (!p || jump) {
          started = false
          if (!p) continue
        }
        if (started) ctx.lineTo(p[0], p[1])
        else ctx.moveTo(p[0], p[1])
        started = true
      }
      ctx.strokeStyle = PLAN_ROUTE
      ctx.lineWidth = 3
      ctx.stroke()
    }

    // 狙っている点
    const [tx, ty] = toBase(a.target_x, a.target_y)
    const tp = inView(tx, ty) ? project(tx, ty) : null
    if (tp) {
      ctx.beginPath()
      ctx.moveTo(tp[0], tp[1] - 9)
      ctx.lineTo(tp[0] + 7, tp[1])
      ctx.lineTo(tp[0], tp[1] + 9)
      ctx.lineTo(tp[0] - 7, tp[1])
      ctx.closePath()
      ctx.fillStyle = '#fff'
      ctx.strokeStyle = PLAN_CMD
      ctx.lineWidth = 2
      ctx.fill()
      ctx.stroke()
    }

    // 検出した動的障害物 [x, y, r, ...]
    const obs = a.obstacles ?? []
    ctx.strokeStyle = '#f2544b'
    ctx.lineWidth = 2
    for (let i = 0; i + 2 < obs.length; i += 3) {
      const [ox, oy] = toBase(obs[i]!, obs[i + 1]!)
      const r = obs[i + 2]!
      if (!inView(ox, oy)) continue
      ctx.beginPath()
      let started = false
      for (let j = 0; j <= 16; j++) {
        const t = (j / 16) * Math.PI * 2
        const p = project(ox + r * Math.cos(t), oy + r * Math.sin(t))
        if (!p) {
          started = false
          continue
        }
        if (started) ctx.lineTo(p[0], p[1])
        else ctx.moveTo(p[0], p[1])
        started = true
      }
      ctx.stroke()
    }
  }
  ctx.restore()
}

/** 映像が古いとみなす最短の時間 [ms]。ARM 中の 15〜30fps なら数フレーム分 */
const CAM_STALE_MIN_MS = 500
/** フレーム間隔（移動平均）の何倍届かなければ古いとみなすか。DISARM 中の低い fps 用 */
const CAM_STALE_INTERVALS = 3
/** 繋がっているのにフレームが来ない状態がこれ続いたら、接続を張り直す [ms]（DISARM は 1fps） */
const CAM_RECONNECT_MS = 5000
/** デコードがこれ終わらなければ固まりとみなして見捨てる [ms] */
const CAM_DECODE_STUCK_MS = 2000

/**
 * 映像の「遅れ」（2026-11-03）。止まった（`drawStale`）とまでは言えないが、いつもの間隔より
 * 明らかに来ていない状態。**ARM 中だけ**見る（DISARM 中は 1fps に落としているので常に遅れて見える）。
 * カメラ配信は JPEG だけで撮影時刻が付いていないので、測れるのは「届く間隔」だけ——撮ってから
 * 届くまでの遅れそのものは測れない
 */
const CAM_LAG_MIN_MS = 180
const CAM_LAG_INTERVALS = 2.5

function isLagging(ageMs: number, intervalMs: number): boolean {
  return !!live.vs?.armed && ageMs > Math.max(CAM_LAG_MIN_MS, intervalMs * CAM_LAG_INTERVALS)
}

/** 遅れ: 映像の縁を琥珀に光らせ、左上に古さを出す（映像そのものは暗くしない） */
function drawLag(ctx: CanvasRenderingContext2D, dx: number, dy: number, dw: number, dh: number, ageMs: number) {
  ctx.save()
  ctx.strokeStyle = '#e8ab21'
  ctx.lineWidth = 4
  ctx.shadowColor = '#e8ab21'
  ctx.shadowBlur = 10
  ctx.strokeRect(dx + 2, dy + 2, dw - 4, dh - 4)
  ctx.shadowBlur = 0
  drawTag(ctx, dx + 12, dy + 12, `映像 遅れ ${(ageMs / 1000).toFixed(1)}s`, '#e8ab21', '#1a1305')
  ctx.restore()
}

/** 止まった映像を暗くし、中央に古さを警告灯の札で出す。`ageMs=null` は切断中 */
function drawStale(
  ctx: CanvasRenderingContext2D, dx: number, dy: number, dw: number, dh: number, ageMs: number | null,
) {
  const text = ageMs == null ? '映像 切断' : `映像停止 ${(ageMs / 1000).toFixed(1)}s`
  ctx.save()
  ctx.fillStyle = 'rgba(0, 0, 0, 0.6)'
  ctx.fillRect(dx, dy, dw, dh)
  ctx.font = "bold 16px 'Hiragino Sans', sans-serif"
  const tw = ctx.measureText(text).width
  drawTag(ctx, dx + dw / 2 - tw / 2 - 12, dy + dh / 2 - 15, text, '#e2433b', '#fff', 16)
  ctx.restore()
}

/** 警告灯の札（地を塗った角丸の帯に文字）。左上の位置で受け取る */
function drawTag(ctx: CanvasRenderingContext2D, x: number, y: number, text: string, bg: string, fg: string, size = 13) {
  ctx.font = `bold ${size}px 'Hiragino Sans', sans-serif`
  const tw = ctx.measureText(text).width
  const hgt = size + 12
  ctx.fillStyle = bg
  ctx.beginPath()
  ctx.roundRect(x, y, tw + 24, hgt, 4)
  ctx.fill()
  ctx.fillStyle = fg
  ctx.textAlign = 'left'
  ctx.textBaseline = 'middle'
  ctx.fillText(text, x + 12, y + hgt / 2 + 1)
}

/** DISARM 中は対象を選べないことを映像の左上に出す（`onRoiPointerDown` が無視する理由）。 */
function drawRoiDisarmedHint(ctx: CanvasRenderingContext2D, dx: number, dy: number) {
  const text = 'DISARM中は対象を選べません（ARMしてから選択）'
  ctx.save()
  ctx.font = '12px ui-monospace, monospace'
  const w = ctx.measureText(text).width
  ctx.fillStyle = 'rgba(0, 0, 0, 0.6)'
  ctx.fillRect(dx + 8, dy + 8, w + 12, 20)
  ctx.fillStyle = '#ffcc66'
  ctx.fillText(text, dx + 14, dy + 22)
  ctx.restore()
}

/** `cam_track_node.py` の追跡結果（`live.track`）のbboxを映像に重ねる。
 * `tracking=False`（未選択）の間は何も描かない。見失っている（`lost`）間は
 * 色を変えて区別する——直前のbboxを保持したまま描き続けるので、
 * 「まだ追ってはいるが自信が無い」ことが見た目で分かる。 */
function drawTrackBox(ctx: CanvasRenderingContext2D, dx: number, dy: number, dw: number, dh: number) {
  const t = live.track
  if (!t || !t.tracking) return
  const x = dx + (t.bbox_cx - t.bbox_w / 2) * dw
  const y = dy + (t.bbox_cy - t.bbox_h / 2) * dh
  ctx.save()
  ctx.strokeStyle = t.lost ? 'rgba(255,90,90,0.9)' : 'rgba(80,220,255,0.9)'
  ctx.lineWidth = 2
  ctx.strokeRect(x, y, t.bbox_w * dw, t.bbox_h * dh)
  ctx.restore()
}

const VEHICLE_HALF_WIDTH = Math.max(...VEHICLE_GEOM.footprint.map(([, y]) => Math.abs(y)))
