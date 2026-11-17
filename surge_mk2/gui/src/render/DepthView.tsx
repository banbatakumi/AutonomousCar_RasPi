/**
 * ステレオの深度映像（`/ws/camera/depth`）。`stereo_node` が視差を色にした JPEG を作り、telemetry_node が
 * そのまま中継する（`_depth_pump`）。**繋いでいる間だけ Pi が照合する**（`_desired_depth`。左カメラも
 * そのときだけ回る）ので、繋いだ直後の数秒は左右が揃うのを待って何も来ない。
 *
 * 映っているのは照合した行の帯だけ（既定は平行化画像の 150〜260 行。下は自分の車体が写るので照合しない。`stereo_node.py`）。
 * 色は視差に比例: 赤 = 近い（約 0.2m）→ 青 = 遠い、黒 = 照合できなかった所（模様の無い面・左端の探索幅）。
 * 各フレームの前に `{"near_m": …}` のテキストが来る（赤の端 = 視差が探索幅の距離）。カーソルを合わせた画素の
 * 色を JET の逆引きで視差に戻し、距離 = near_m × 255 / v（v は色の値 0〜255）で表示する。
 * JPEG の劣化と 8bit の量子化があるので**目安**（遠いほど粗い）。黒は「測れない」。
 * 開閉・再接続は `CameraView` の `/ws/camera/mask` と同じ作法。
 */
import { useEffect, useRef, useState } from 'react'
import { guardSocket, type SocketGuard } from '../ws/guard'
import { wsUrl } from '../ws/url'

/** OpenCV の COLORMAP_JET（v = 0〜255 → RGB）。`colorize_disparity` の逆引き表に使う。 */
const JET = (() => {
  const clamp = (x: number) => Math.max(0, Math.min(1, x))
  return Array.from({ length: 256 }, (_, v) => {
    const x = v / 255
    return [
      clamp(1.5 - Math.abs(4 * x - 3)) * 255,
      clamp(1.5 - Math.abs(4 * x - 2)) * 255,
      clamp(1.5 - Math.abs(4 * x - 1)) * 255,
    ]
  })
})()

/** 画素の色 → 視差の値 v（1〜255）。黒（照合できなかった所）は null。 */
function jetValue(r: number, g: number, b: number): number | null {
  if (Math.max(r, g, b) < 40) return null
  let best = 1
  let bestD = Infinity
  for (let v = 1; v < 256; v++) {
    const [jr, jg, jb] = JET[v]
    const d = (jr - r) ** 2 + (jg - g) ** 2 + (jb - b) ** 2
    if (d < bestD) {
      bestD = d
      best = v
    }
  }
  return best
}

type Probe = { x: number; y: number; text: string }

export function DepthView({ label = '深度' }: { label?: string }) {
  const imgRef = useRef<HTMLImageElement>(null)
  const [status, setStatus] = useState('接続中…')
  const nearRef = useRef(0)
  const canvasRef = useRef<HTMLCanvasElement | null>(null)
  const [probe, setProbe] = useState<Probe | null>(null)

  useEffect(() => {
    let ws: WebSocket | null = null
    let timer: number | undefined
    let closed = false
    let guard: SocketGuard | null = null
    let url: string | null = null
    let frames = 0
    let last = performance.now()
    const tick = window.setInterval(() => {
      const now = performance.now()
      const fps = (frames * 1000) / (now - last)
      frames = 0
      last = now
      setStatus(fps > 0 ? `${fps.toFixed(1)}fps` : '待機中（左右が揃うまで数秒）')
    }, 1000)

    const open = () => {
      const sock = new WebSocket(wsUrl('/ws/camera/depth'))
      ws = sock
      sock.binaryType = 'arraybuffer'
      guard?.stop()
      guard = guardSocket(sock, () => {
        if (!closed) timer = window.setTimeout(open, 800)
      })
      sock.onmessage = (ev) => {
        if (typeof ev.data === 'string') {
          try {
            nearRef.current = Number(JSON.parse(ev.data).near_m) || 0
          } catch {
            /* 壊れたヘッダは捨てる */
          }
          return
        }
        if (!(ev.data instanceof ArrayBuffer) || !imgRef.current) return
        const next = URL.createObjectURL(new Blob([ev.data], { type: 'image/jpeg' }))
        imgRef.current.src = next
        if (url) URL.revokeObjectURL(url)
        url = next
        frames += 1
      }
      sock.onclose = () => {
        guard?.stop()
        if (!closed) timer = window.setTimeout(open, 800)
      }
      sock.onerror = () => sock.close()
    }
    open()

    return () => {
      closed = true
      window.clearTimeout(timer)
      window.clearInterval(tick)
      guard?.stop()
      ws?.close()
      if (url) URL.revokeObjectURL(url)
    }
  }, [])

  /** カーソル位置（`object-fit: contain` の余白を除いた画像座標）の距離を読む。 */
  const onMove = (e: React.MouseEvent<HTMLImageElement>) => {
    const img = e.currentTarget
    const nw = img.naturalWidth
    const nh = img.naturalHeight
    if (!nw || !nh) return setProbe(null)
    const box = img.getBoundingClientRect()
    const k = Math.min(box.width / nw, box.height / nh)
    const ox = (box.width - nw * k) / 2
    const oy = (box.height - nh * k) / 2
    const px = Math.floor((e.clientX - box.left - ox) / k)
    const py = Math.floor((e.clientY - box.top - oy) / k)
    if (px < 0 || py < 0 || px >= nw || py >= nh) return setProbe(null)
    const cv = (canvasRef.current ??= document.createElement('canvas'))
    cv.width = nw
    cv.height = nh
    const ctx = cv.getContext('2d', { willReadFrequently: true })
    if (!ctx) return
    ctx.drawImage(img, 0, 0)
    const [r, g, b] = ctx.getImageData(px, py, 1, 1).data
    const v = jetValue(r, g, b)
    const near = nearRef.current
    const text = v === null ? '測れない' : near > 0 ? `約 ${((near * 255) / v).toFixed(2)} m` : '…'
    const host = img.parentElement!.getBoundingClientRect()
    setProbe({ x: e.clientX - host.left, y: e.clientY - host.top, text })
  }

  return (
    <div className="camera" title="ステレオの深度（赤 = 近い → 青 = 遠い、黒 = 照合できない）">
      <img
        ref={imgRef}
        alt=""
        className="depth-img"
        onMouseMove={onMove}
        onMouseLeave={() => setProbe(null)}
      />
      {probe && (
        <div className="depth-probe" style={{ left: probe.x, top: probe.y }}>
          {probe.text}
        </div>
      )}
      <DepthScale />
      <div className="camera-tag">
        {label}
        <span className="dim">{status}</span>
        <span className="dim">赤 近い → 青 遠い</span>
      </div>
    </div>
  )
}

/** `colorize_disparity()` の色順に合わせた距離の凡例。距離は視差に反比例する。 */
function DepthScale() {
  const marks = [
    ['0.2m', 0],
    ['0.5m', 25],
    ['1m', 50],
    ['2m', 70],
    ['5m', 85],
    ['15m+', 100],
  ] as const

  return (
    <div className="depth-scale" aria-label="深度の距離スケール">
      <div className="depth-scale-bar" />
      <div className="depth-scale-labels">
        {marks.map(([label, left]) => (
          <span key={label} style={{ left: `${left}%` }}>
            {label}
          </span>
        ))}
      </div>
    </div>
  )
}
