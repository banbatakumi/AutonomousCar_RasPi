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
 * 平面路面と仮の取り付け姿勢を仮定した近似であり、**実測値ではない**。
 * 校正前のガイドを信じて経路の当たり判定に使わないよう、画面にも印を出す
 * ——ただし出すのは自動運転ビュー（`variant="full"`）のみ。ラジコンビューは
 * 運転を楽しむ画面なので文字を極力出さない方針（`RcView.tsx`）で、下記
 * `variant="minimal"` では出さない。
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

/** 仮のカメラ姿勢。**校正前の暫定値**（`architecture.md` §14 Phase 1）。 */
const CAM = {
  height: 0.14, // m 路面からの高さ
  pitch: 0.12, // rad 下向きが正
  hfovDeg: 66, // 水平画角（imx219 の公称値）
}
const WHEELBASE = 0.25

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
  const statusRef = useRef<HTMLSpanElement>(null)

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
        if (guide && cam === 'front') drawGuide(ctx, w, h)
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

/** 実測舵角から予測進路を路面平面に置き、仮のピンホールで画像に落とす。 */
function drawGuide(ctx: CanvasRenderingContext2D, w: number, h: number) {
  const vs = live.vs
  if (!vs) return
  const f = w / 2 / Math.tan((CAM.hfovDeg * Math.PI) / 360)

  const project = (x: number, y: number): [number, number] | null => {
    // 車両座標 (x=前, y=左, z=上) → カメラ座標。pitch だけ傾いている前提
    const cp = Math.cos(CAM.pitch)
    const sp = Math.sin(CAM.pitch)
    const zc = x * cp + CAM.height * sp // 光軸方向
    const yc = -x * sp + CAM.height * cp // 下向きが正
    if (zc < 0.15) return null
    return [w / 2 - (y * f) / zc, h / 2 + (yc * f) / zc]
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
      th += (ds / WHEELBASE) * Math.tan(vs.steer_actual)
      x += ds * Math.cos(th)
      y += ds * Math.sin(th)
      const p = project(x, y + off)
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

const VEHICLE_HALF_WIDTH = 0.095
