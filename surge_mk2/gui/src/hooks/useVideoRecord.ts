/**
 * 🎥 録画ボタン — 前後カメラの動画（mp4, 30fps）を録って Mac にダウンロードする。
 *
 * mcap（`useMcapDownload`）と違い、**Pi の `logs/` に mp4 を書く**（mp4 は最後に索引を
 * 書くので、バイト列のまま中継できない）。止めると `status.video.files` に書き終えた
 * ファイル名が載るので、それを `GET /logs/<name>` で取る。
 *
 * **押したタブだけがダウンロードする**（別タブで止められても二重に落とさない）。
 * 記録中かどうか・経過時間・エラーはサーバ側の状態（`ControlStatus.video`）が真値。
 */
import { useEffect, useRef } from 'react'
import { useUi } from '../store/ui'
import type { ControlChannel } from '../ws/control'

/** 続けて複数ファイルを落とすとブラウザが後続を止めることがあるので、少し間を空ける */
const DOWNLOAD_GAP_MS = 400

function download(name: string) {
  const a = document.createElement('a')
  a.href = `/logs/${encodeURIComponent(name)}`
  a.download = name
  document.body.appendChild(a)
  a.click()
  a.remove()
}

export function useVideoRecord(ch: ControlChannel | null) {
  const video = useUi((s) => s.video)
  const active = video?.active ?? false
  const startedHere = useRef(false)
  const wasActive = useRef(false)

  useEffect(() => {
    // 録画中 → 停止の変化で、押したタブだけが取りに行く
    if (wasActive.current && !active && startedHere.current) {
      startedHere.current = false
      ;(video?.files ?? []).forEach((name, i) => {
        window.setTimeout(() => download(name), i * DOWNLOAD_GAP_MS)
      })
    }
    wasActive.current = active
  }, [active, video?.files])

  const toggle = () => {
    if (!ch) return
    if (active) {
      ch.videoRecordStop()
    } else {
      startedHere.current = true
      ch.videoRecordStart()
    }
  }

  return {
    active,
    elapsedS: video?.elapsed_s ?? 0,
    error: video?.error ?? null,
    toggle,
  }
}
