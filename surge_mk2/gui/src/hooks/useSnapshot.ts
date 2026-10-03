/**
 * 📷 撮影ボタン — Pi のカメラ映像を1枚撮って Mac（ブラウザのダウンロード先）に保存する。
 *
 * `GET /snapshot/<front|rear>.png`（`telemetry_node._serve_snapshot`）が共有メモリの
 * 最新フレームを**無劣化 PNG・魚眼のまま**返す。主な用途はチェッカーボード校正
 * （Mac 側 `tools/cam_calib/`、ランチャーの「カメラ校正」）で、表示中の JPEG
 * （画質を落とし、補正映像なら remap 済み）を保存するのでは校正に使えないため。
 *
 * ファイル名はサーバが付ける（`X-Surge-Filename`）。解像度と下端クロップ率が入っており、
 * 校正ツールはそこからクロップ前のフル画像の大きさを復元する。
 *
 * ダウンロードの作法（Blob → `<a download>` → 一呼吸置いて revoke）は
 * `useMcapDownload.ts` と同じ。
 */
import { useState } from 'react'

export type SnapCam = 'front' | 'rear'

async function snapOne(cam: SnapCam): Promise<string> {
  const res = await fetch(`/snapshot/${cam}.png`, { cache: 'no-store' })
  if (!res.ok) throw new Error(`${cam}: ${(await res.text()) || res.status}`)
  const blob = await res.blob()
  const stamp = new Date().toISOString().replace(/[-:]/g, '').replace('T', '_').slice(0, 15)
  const name = res.headers.get('X-Surge-Filename') ?? `surge_${cam}_${stamp}.png`
  const url = URL.createObjectURL(blob)
  const a = document.createElement('a')
  a.href = url
  a.download = name
  document.body.appendChild(a)
  a.click()
  a.remove()
  window.setTimeout(() => URL.revokeObjectURL(url), 0)
  return name
}

export function useSnapshot() {
  const [busy, setBusy] = useState(false)
  const [error, setError] = useState<string | null>(null)
  /** 直近に保存したファイル名（ボタンの title に出す） */
  const [last, setLast] = useState<string | null>(null)

  /** `cams` を順に撮って保存する。1台失敗しても残りは撮る */
  const snap = async (cams: SnapCam[]) => {
    if (busy) return
    setBusy(true)
    setError(null)
    const errors: string[] = []
    for (const cam of cams) {
      try {
        setLast(await snapOne(cam))
      } catch (e) {
        errors.push(e instanceof Error ? e.message : String(e))
      }
    }
    if (errors.length) setError(errors.join(' / '))
    setBusy(false)
  }

  return { snap, busy, error, last }
}
