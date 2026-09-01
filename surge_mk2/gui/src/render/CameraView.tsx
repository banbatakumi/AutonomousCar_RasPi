/**
 * カメラ映像。`/ws/camera/{front,rear}` から JPEG が流れてくる。
 *
 * `createImageBitmap` でデコードして Canvas に描く。**React の state に
 * 画像を入れない**（1フレームごとに再レンダリングが走る）。
 *
 * ## 進路ガイドは「暫定」と明示する
 *
 * 画像上に予測進路を重ねるには、カメラの内部・外部パラメータ（校正）が要る。
 * **これは Phase 1 の作業でまだ済んでいない。** ここで描いているのは、
 * 平面路面と取り付け姿勢を仮定した近似であり、**実測値ではない**。
 * 校正前のガイドを信じて経路の当たり判定に使わないよう、画面にも印を出す
 * ——ただし出すのは自動運転ビュー（`variant="full"`）のみ。ラジコンビューは
 * 運転を楽しむ画面なので文字を極力出さない方針（`RcView.tsx`）で、下記
 * `variant="minimal"` では出さない。
 *
 * 高さ（設定パネルの `camHeight`）は `config/vehicle.toml` の `sensors.cam_front.z`
 * を初期値に、実写を見ながら追い込める——定規で測れる筐体の位置とレンズ光学中心が
 * ズレうるため。**俯角（取付角度）・水平画角は調整UIを持たない**。一度ネジ止めしたら
 * 変わらない物理値で、`VEHICLE.camFront.pitch`/`hfov`（`vehicle.toml` 直）をそのまま使う
 * （2026-08-21、指示による——「絶対変わらないものにスライダーは要らない」）。
 *
 * ## 下端クロップぶんの主点補正
 *
 * 前カメラは ISP の ScalerCrop でボンネット等が映る下1/4を最初から読み出さない
 * （`camera_node.py` の `CAM_BOTTOM_CROP`）。配信される画像はセンサー上側3/4だけなので、
 * 光軸中心（センサー中心）は配信画像の中央ではなく、**上端から 1/(2·(1−bottomCrop)) の
 * 位置**にある（下1/4カットなら 66.7%）。ここを画像中央（h/2）のまま投影すると、
 * ガイド線が実際より上に描かれる。`VEHICLE.camFront.bottomCrop` で補正する。
 *
 * ## 走行中の車体姿勢（IMU）でも補正する（2026-08-21）
 *
 * カメラは車体に固定されているので、走行中に車体が pitch/roll すると
 * カメラの向きも一緒に傾く。`vs.pitch`/`vs.roll`（`TELEMETRY` の IMU 実測値。
 * 重力ベクトルで補正済みでドリフトしない）で毎フレーム補う。符号の向きは
 * 実車で確認済み（指示による）: **前が沈む（前輪側）ほど `pitch` は負**、
 * **右が沈む（右輪側）ほど `roll` は負**。
 *
 * - pitch: 固定取付角に `−vs.pitch` を足す。前のめり（pitch 負）になるほど
 *   カメラも一緒に下を向く（実効的な俯角が増える）ので符号を反転して足す。
 * - roll: 画像面内の回転として扱う（ロールはカメラの光軸まわりの回転にほぼ
 *   一致するため、画像を principal point 中心に回すだけで近似できる）。
 *   カメラが右に傾く（roll 負）と、写真の一般則どおり像は反時計回りに回って
 *   見える——回転角は `−vs.roll`。**pitch/roll は独立に合成する近似**
 *   （厳密な3D外部パラメータではない。もともと未校正の暫定ガイドなので、
 *   このクラスの近似で十分と判断している）。
 * - IMU が無効（`imu_ok=false`）な間は 0 として扱い、固定値だけで描く。
 *
 * ## 後カメラにも同じガイドを出す（2026-08-21）
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
 * ## `variant`：ラジコンビューは文字を出さない（2026-08-17）
 *
 * `full`（既定・自動運転ビュー）は左上に「前方/後方」ラベルと fps を
 * 並べたタグを出し、校正前バッジも出す。`minimal`（ラジコンビュー）は
 * ラベルもバッジも出さず、fps だけを右下に小さく残す。**fps だけは
 * 消さない** — 配信が生きているかを判断する最後の手がかりのため。
 *
 * ## モード連動のデバッグ重畳（2026-08-28）
 *
 * 前カメラだけ、選ばれている自動運転モードに応じて2種類の重畳が自動で出る
 * （バンビの「セグメンテーションマスクを見たい」「ライントレースがどの線を
 * 認識しているか見たい」という要望より）。トグルは無く、モードを選んだ時点で出る
 * ——LiDAR ビューのギャップ重畳（`LidarView.tsx`）と同じ「見せたい情報は
 * 自動で出す」方針。
 *
 * - **`ftg_cam`**: `cam_perception_node.py` の走行可否マスクを
 *   `/ws/camera/mask`（メイン映像とは別の専用 WS。`showMask` の間だけ繋ぐ）
 *   から受け、薄い半透明でそのまま重ねる（白＝走行可）。
 * - **`line_trace`**: `LineScan`（`/ws/telemetry` の `line_cam`）が持つ
 *   近傍・遠方の目標点を `makeProjector()`（`drawGuide` と共用）で画像へ
 *   逆投影し、シアン色のマーカーと線で重ねる。
 *
 * どちらも**メイン映像の WebSocket 接続とは独立**させてある——モードを
 * 切り替えるたびにメイン映像まで再接続されるのを避けるため。
 *
 * ## `onAspect`：実際に届いた画像の縦横比を親へ返す（2026-08-17）
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
import { useUi } from '../store/ui'
import { wsUrl } from '../ws/url'
import { VEHICLE as VEHICLE_GEOM } from '../generated/vehicle'
import type { ControlChannel } from '../ws/control'

export function CameraView({
  cam,
  label,
  variant = 'full',
  onAspect,
  guideDisabled = false,
  ch = null,
}: {
  cam: 'front' | 'rear'
  label: string
  variant?: 'full' | 'minimal'
  /** デコードした画像の実際の幅/高さ比。変わったときだけ呼ばれる */
  onAspect?: (aspect: number) => void
  /** true の間は `pathGuide` が ON でもこのインスタンスには描かない。
   * ラジコンビューの PIP（小さい方の映像）専用——**大きい方の映像だけに
   * ガイドを出す**ため（`RcView.tsx`、指示による。2026-08-21） */
  guideDisabled?: boolean
  /** `follow_object` のROI選択（ドラッグ矩形）に使う。前方カメラ以外・
   * 未指定の間はポインタ操作を一切拾わない（2026-09-01） */
  ch?: ControlChannel | null
}) {
  const ref = useRef<HTMLCanvasElement>(null)
  const guide = useUi((s) => s.pathGuide) && !guideDisabled
  const camHeight = useUi((s) => s.settings.camHeight)
  const statusRef = useRef<HTMLSpanElement>(null)
  // スライダー操作のたびに WS を再接続させないよう ref 経由で最新値を渡す
  // （下の draw ループの useEffect 依存に camHeight を入れない）
  const camHeightRef = useRef(camHeight)
  camHeightRef.current = camHeight

  // ── モード連動のデバッグ重畳（2026-08-28） ──
  //
  // `ftg_cam` はセグメンテーションマスク、`line_trace` は認識した白線の目標点を
  // 前カメラにだけ重ねる。**どちらもメイン映像の WS（`open()`）とは無関係**
  // ——モードを切り替えるたびにメイン映像まで再接続されると困るので、
  // ref 経由で `draw()` に渡す（`camHeightRef` と同じ理由）
  const auto = useUi((s) => s.auto)
  const showMask = cam === 'front' && auto?.mode === 'ftg_cam'
  const showLineTarget = cam === 'front' && auto?.mode === 'line_trace'
  const showMaskRef = useRef(showMask)
  showMaskRef.current = showMask
  const showLineTargetRef = useRef(showLineTarget)
  showLineTargetRef.current = showLineTarget
  //: `ftg_cam` の走行可否マスク（`/ws/camera/mask`）。専用の別 `useEffect`
  //: （下記）で `showMask` の間だけ繋ぎ、`draw()` はここを読むだけ
  const maskBitmapRef = useRef<ImageBitmap | null>(null)

  // ── 対象追従（`follow_object`）のROI選択（2026-09-01） ──
  //
  // ドラッグでROI矩形を選ぶ操作は前方カメラの `follow_object` 選択中だけ有効。
  // `showMask`/`showLineTarget` と同じ ref 経由（draw ループの useEffect 依存に
  // 入れない）。追跡結果のbbox重畳（`drawTrackBox()`）も同じ条件で出す
  const trackingMode = cam === 'front' && auto?.mode === 'follow_object'
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
    let lastAspect = 0

    const open = () => {
      ws = new WebSocket(wsUrl(`/ws/camera/${cam}`))
      ws.binaryType = 'arraybuffer'
      ws.onmessage = async (ev) => {
        if (!(ev.data instanceof ArrayBuffer)) return
        // デコードが追いつかないときは**捨てる**。溜めると遅延が伸び続ける
        if (decoding) return
        decoding = true
        try {
          const bmp = await createImageBitmap(new Blob([ev.data], { type: 'image/jpeg' }))
          bitmap?.close()
          bitmap = bmp
          frames++
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
          decoding = false
        }
      }
      ws.onclose = () => {
        if (!closed) timer = window.setTimeout(open, 800)
      }
      ws.onerror = () => ws?.close()
    }
    open()

    const draw = () => {
      raf = requestAnimationFrame(draw)
      const dpr = window.devicePixelRatio || 1
      const w = cv.clientWidth
      const h = cv.clientHeight
      if (!w || !h) return
      if (cv.width !== w * dpr || cv.height !== h * dpr) {
        cv.width = w * dpr
        cv.height = h * dpr
      }
      ctx.setTransform(dpr, 0, 0, dpr, 0, 0)
      ctx.fillStyle = '#07090b'
      ctx.fillRect(0, 0, w, h)

      if (bitmap) {
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
        // ガイドがズレる（2026-08-31、後方カメラのガイドが上に大きくズレる
        // 不具合として発見）
        if (guide || showLineTargetRef.current) {
          ctx.save()
          ctx.translate(dx, dy)
          if (guide) drawGuide(ctx, dw, dh, camHeightRef.current, cam)
          if (showLineTargetRef.current) drawLineTarget(ctx, dw, dh, camHeightRef.current)
          ctx.restore()
        }
        // 対象追従（`follow_object`）のROI選択・追跡結果の重畳。
        // **メイン映像とは独立**（ドラッグ操作もbboxもここでしか意味を持たない）
        if (trackingModeRef.current) {
          if (dragRef.current) drawRoiDrag(ctx, dx, dy, dw, dh, dragRef.current)
          drawTrackBox(ctx, dx, dy, dw, dh)
        }
      } else {
        ctx.fillStyle = '#3a444e'
        ctx.font = '12px ui-monospace, monospace'
        ctx.fillText('映像なし（camera_node が動いているか確認）', 12, h / 2)
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
      cancelAnimationFrame(raf)
      ws?.close()
      bitmap?.close()
    }
  }, [cam, guide, onAspect])

  // `ftg_cam` の走行可否マスク（`/ws/camera/mask`）。**メイン映像の WS とは
  // 独立**——`showMask` が変わっても上の effect（メイン映像）は再接続しない。
  // 開閉・デコードのパターンはメイン映像の `open()` と同じ
  useEffect(() => {
    if (!showMask) return
    let ws: WebSocket | null = null
    let timer: number | undefined
    let closed = false
    let decoding = false

    const open = () => {
      ws = new WebSocket(wsUrl('/ws/camera/mask'))
      ws.binaryType = 'arraybuffer'
      ws.onmessage = async (ev) => {
        if (!(ev.data instanceof ArrayBuffer)) return
        if (decoding) return
        decoding = true
        try {
          const bmp = await createImageBitmap(new Blob([ev.data], { type: 'image/jpeg' }))
          maskBitmapRef.current?.close()
          maskBitmapRef.current = bmp
        } catch {
          /* 壊れたフレームは捨てる */
        } finally {
          decoding = false
        }
      }
      ws.onclose = () => {
        if (!closed) timer = window.setTimeout(open, 800)
      }
      ws.onerror = () => ws?.close()
    }
    open()

    return () => {
      closed = true
      window.clearTimeout(timer)
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
          {guide && <span className="badge-warn">ガイドは校正前・暫定</span>}
        </div>
      ) : (
        <div className="camera-fps">
          <span ref={statusRef} />
        </div>
      )}
    </div>
  )
}

/** 実測舵角から予測進路を路面平面に置き、仮のピンホールで画像に落とす。
 * `camHeightSetting` は設定パネルで校正できる値（`store/ui.ts` の `camHeight`。
 * **前カメラにしか適用しない**——後カメラは高さの調整UIを持たないので
 * `VEHICLE.camRear.height` を固定で使う）。`pitch`/`hfov`/`bottomCrop` は
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
  const { project, rotateRoll } = makeProjector(w, h, camHeightSetting, cam)

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
      x += ds * Math.cos(th)
      y += ds * Math.sin(th)
      // 車両座標 (x=前, y=左) → カメラ座標。後カメラは奥行き・左右とも反転する
      // （振り返った視点なので、車体の後方＝カメラの奥、車体の左＝カメラの右）
      const [depth, lateral] = isFront ? [x, y + off] : [-x, -(y + off)]
      const projected = project(depth, lateral)
      const p = projected && rotateRoll(projected)
      if (!p) continue
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

/** 路面座標（base_link、x=前 y=左）→ 画像座標の投影器を作る（2026-08-28、
 * `drawGuide` から切り出し）。IMU による姿勢（pitch/roll）補正込み。
 * `drawGuide`（進路ガイド）と `drawLineTarget`（認識ラインの重畳）が共用する
 * ——同じカメラ内部・外部パラメータ・IMU補正を2箇所に書くと片方だけ直し忘れる。 */
function makeProjector(w: number, h: number, camHeightSetting: number, cam: 'front' | 'rear') {
  const vs = live.vs
  const isFront = cam === 'front'
  const geom = isFront ? VEHICLE_GEOM.camFront : VEHICLE_GEOM.camRear
  const { pitch: pitchFixed, hfov, bottomCrop } = geom
  const height = isFront ? camHeightSetting : geom.height
  const f = w / 2 / Math.tan(hfov / 2)
  const cx = w / 2
  // 下端クロップぶん、光軸（センサー中心）は配信画像の中央より下にずれる
  const principalY = h / (2 * (1 - bottomCrop))

  // IMU が無効・未受信の間は姿勢ぶんの補正をかけない（0 扱い＝固定値のみ）。
  // 後カメラは前カメラと逆向きを見ているので、姿勢補正の符号を反転する
  // （前が沈む＝後ろは持ち上がる。左右も振り返った側から見るので鏡像になる）
  const dPitch = (vs?.imu_ok ? vs.pitch : 0) * (isFront ? 1 : -1)
  const dRoll = (vs?.imu_ok ? vs.roll : 0) * (isFront ? 1 : -1)

  // 前が沈む（dPitch 負）ほどカメラも一緒に下を向くので、符号を反転して足す
  const pitch = pitchFixed - dPitch
  const cp = Math.cos(pitch)
  const sp = Math.sin(pitch)

  const project = (x: number, y: number): [number, number] | null => {
    // カメラ座標 (x=奥行き, y=左, z=上) → 画像。pitch だけ傾いている前提
    const zc = x * cp + height * sp // 光軸方向
    const yc = -x * sp + height * cp // 下向きが正
    if (zc < 0.15) return null
    return [cx - (y * f) / zc, principalY + (yc * f) / zc]
  }

  // roll: 画像面内の回転として近似する。右が沈む（dRoll 負）とカメラも右に傾き、
  // 写真の一般則どおり像は principal point を中心に反時計回りへ回って見える
  const rollImg = -dRoll
  const cr = Math.cos(rollImg)
  const sr = Math.sin(rollImg)
  const rotateRoll = (p: [number, number]): [number, number] => {
    const dx = p[0] - cx
    const dy = p[1] - principalY
    return [cx + dx * cr + dy * sr, principalY - dx * sr + dy * cr]
  }

  return { project, rotateRoll }
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
  const { project, rotateRoll } = makeProjector(w, h, camHeightSetting, 'front')

  const near = line.near_seen ? project(line.near_x, line.near_y) : null
  const far = line.far_seen ? project(line.far_x, line.far_y) : null
  const nearPt = near && rotateRoll(near)
  const farPt = far && rotateRoll(far)
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
