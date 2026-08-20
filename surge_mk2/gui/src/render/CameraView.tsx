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
 * ## `variant`：ラジコンビューは文字を出さない（2026-08-17）
 *
 * `full`（既定・自動運転ビュー）は左上に「前方/後方」ラベルと fps を
 * 並べたタグを出し、校正前バッジも出す。`minimal`（ラジコンビュー）は
 * ラベルもバッジも出さず、fps だけを右下に小さく残す。**fps だけは
 * 消さない** — 配信が生きているかを判断する最後の手がかりのため。
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
import { live } from '../bus/live'
import { useUi } from '../store/ui'
import { wsUrl } from '../ws/url'
import { VEHICLE as VEHICLE_GEOM } from '../generated/vehicle'

export function CameraView({
  cam,
  label,
  variant = 'full',
  onAspect,
}: {
  cam: 'front' | 'rear'
  label: string
  variant?: 'full' | 'minimal'
  /** デコードした画像の実際の幅/高さ比。変わったときだけ呼ばれる */
  onAspect?: (aspect: number) => void
}) {
  const ref = useRef<HTMLCanvasElement>(null)
  const guide = useUi((s) => s.pathGuide)
  const camHeight = useUi((s) => s.settings.camHeight)
  const statusRef = useRef<HTMLSpanElement>(null)
  // スライダー操作のたびに WS を再接続させないよう ref 経由で最新値を渡す
  // （下の draw ループの useEffect 依存に camHeight を入れない）
  const camHeightRef = useRef(camHeight)
  camHeightRef.current = camHeight

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
        ctx.drawImage(bitmap, (w - dw) / 2, (h - dh) / 2, dw, dh)
        if (guide && cam === 'front') drawGuide(ctx, w, h, camHeightRef.current)
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

  return (
    <div className="camera" title={label}>
      <canvas ref={ref} />
      {variant === 'full' ? (
        <div className="camera-tag">
          {label}
          <span ref={statusRef} className="dim" />
          {cam === 'front' && guide && <span className="badge-warn">ガイドは校正前・暫定</span>}
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
 * `height` は設定パネルで校正できる値（`store/ui.ts` の `camHeight`）。
 * `pitch`/`hfov`/`bottomCrop` は `vehicle.toml` 由来の固定値（`VEHICLE.camFront`）。
 * `vs.pitch`/`vs.roll`（IMU 実測）で走行中の車体姿勢ぶんを毎フレーム補正する
 * （符号の向きはファイル冒頭コメント参照）。 */
function drawGuide(ctx: CanvasRenderingContext2D, w: number, h: number, height: number) {
  const vs = live.vs
  if (!vs) return
  const { pitch: pitchFixed, hfov, bottomCrop } = VEHICLE_GEOM.camFront
  const f = w / 2 / Math.tan(hfov / 2)
  const cx = w / 2
  // 下端クロップぶん、光軸（センサー中心）は配信画像の中央より下にずれる
  const principalY = h / (2 * (1 - bottomCrop))

  // IMU が無効な間は姿勢ぶんの補正をかけない（0 扱い＝固定値のみ）
  const dPitch = vs.imu_ok ? vs.pitch : 0
  const dRoll = vs.imu_ok ? vs.roll : 0

  // 前が沈む（dPitch 負）ほどカメラも一緒に下を向くので、符号を反転して足す
  const pitch = pitchFixed - dPitch
  const cp = Math.cos(pitch)
  const sp = Math.sin(pitch)

  const project = (x: number, y: number): [number, number] | null => {
    // 車両座標 (x=前, y=左, z=上) → カメラ座標。pitch だけ傾いている前提
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

  // 車体の左右端をなぞる2本
  for (const off of [-VEHICLE_HALF_WIDTH, VEHICLE_HALF_WIDTH]) {
    ctx.beginPath()
    let x = 0
    let y = 0
    let th = 0
    let started = false
    const ds = 0.08
    for (let i = 0; i < 45; i++) {
      th += (ds / VEHICLE_GEOM.wheelbase) * Math.tan(vs.steer_actual)
      x += ds * Math.cos(th)
      y += ds * Math.sin(th)
      const projected = project(x, y + off)
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

const VEHICLE_HALF_WIDTH = Math.max(...VEHICLE_GEOM.footprint.map(([, y]) => Math.abs(y)))
