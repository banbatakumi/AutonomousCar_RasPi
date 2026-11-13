/**
 * ステレオの深度映像（`/ws/camera/depth`）。`stereo_node` が視差を色にした JPEG を作り、telemetry_node が
 * そのまま中継する（`_depth_pump`）。**繋いでいる間だけ Pi が照合する**（`_desired_depth`。左カメラも
 * そのときだけ回る）ので、繋いだ直後の数秒は左右が揃うのを待って何も来ない。
 *
 * 映っているのは照合した行の帯だけ（既定は平行化画像の 150〜260 行。下は自分の車体が写るので照合しない。`stereo_node.py`）。
 * 色は視差に比例: 赤 = 近い（約 0.2m）→ 青 = 遠い、黒 = 照合できなかった所（模様の無い面・左端の探索幅）。
 * 開閉・再接続は `CameraView` の `/ws/camera/mask` と同じ作法。
 */
import { useEffect, useRef, useState } from 'react'
import { guardSocket, type SocketGuard } from '../ws/guard'
import { wsUrl } from '../ws/url'

export function DepthView({ label = '深度' }: { label?: string }) {
  const imgRef = useRef<HTMLImageElement>(null)
  const [status, setStatus] = useState('接続中…')

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

  return (
    <div className="camera" title="ステレオの深度（赤 = 近い → 青 = 遠い、黒 = 照合できない）">
      <img ref={imgRef} alt="" className="depth-img" />
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
