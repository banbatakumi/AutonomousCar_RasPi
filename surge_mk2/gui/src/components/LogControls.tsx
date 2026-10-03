/**
 * 記録ボタン — タブバー常設（旧「ログ」タブは廃止、2026-09-03）。
 *
 * `.sfl` と mcap は仕組みがまったく違う（旧 `LogView.tsx` の docstring 参照）——
 * `.sfl` は io_node への「録ってほしい」という意思の送信だけで、Wi-Fi が
 * 切れても Pi 単体で記録が続く。mcap は `/ws/record` の中継をブラウザが
 * 直接バッファし、**停止と同時に**（中継が終わった合図で）Mac へダウンロードする。
 *
 * ファイル一覧・ダウンロード・削除は診断タブ（`DiagLogFiles`）に移した——
 * ここはタブバーという狭い場所なので、「録る/録らない」の意思表示に絞る。
 *
 * 同じ並びに **📷 撮影**（`useSnapshot`。前カメラと、取得中なら後カメラの生画像を
 * PNG で Mac に保存。魚眼校正の写真撮りに使う）と、**配信映像の魚眼補正の切替**
 * （`status.camera_config.undistort`、校正済みのカメラがあるときだけ出す）を置く。
 */
import { formatBytes, formatElapsed } from '../format'
import { useMcapDownload } from '../hooks/useMcapDownload'
import { useSnapshot, type SnapCam } from '../hooks/useSnapshot'
import { useUi } from '../store/ui'
import type { ControlChannel } from '../ws/control'

export function LogControls({ ch }: { ch: ControlChannel | null }) {
  const sfl = useUi((s) => s.sfl)
  const mcap = useUi((s) => s.mcap)
  const sflActive = sfl?.active ?? false
  const mcapActive = mcap?.active ?? false
  const { bufferedBytes, start } = useMcapDownload(ch)
  const cam = useUi((s) => s.cameraConfig)
  const snapshot = useSnapshot()
  // 取得を止めているカメラ（DISARM 中で映像を表示していない等）はフレームが無いので撮らない
  const snapCams: SnapCam[] = (['front', 'rear'] as SnapCam[]).filter((c) =>
    (c === 'front' ? cam?.front_enabled_effective : cam?.rear_enabled_effective) !== false)
  const canUndistort = !!(cam?.undistort_available?.front || cam?.undistort_available?.rear)

  return (
    <div className="log-controls">
      <button
        className={`recbtn ${sflActive ? 'recording' : ''}`}
        disabled={!ch}
        onClick={() => ch?.sflRecord(!sflActive)}
        title=".sfl 記録（UART生ログ）。開始するとWi-Fiが切れてもPi単体で継続する。ファイルは診断タブから確認・ダウンロード"
      >
        <span className="rec-dot" />
        sfl
      </button>
      <button
        className={`recbtn ${mcapActive ? 'recording' : ''}`}
        disabled={!ch}
        onClick={() => (mcapActive ? ch?.mcapRecordStop() : start(true, () => ch?.logsList()))}
        title="mcap 記録（画像込み・Foxglove用）。停止と同時にMacへダウンロードする"
      >
        <span className="rec-dot" />
        MCAP{mcapActive ? ` ${formatElapsed(mcap!.elapsed_s)} / ${formatBytes(bufferedBytes)}` : ''}
      </button>
      {mcap?.error && <span className="badge-bad">{mcap.error}</span>}
      <button
        className={`recbtn snapbtn ${snapshot.busy ? 'busy' : ''}`}
        disabled={!ch || snapshot.busy}
        onClick={() => snapshot.snap(snapCams)}
        title={
          'カメラ映像を撮影してMacに保存する（無劣化PNG・魚眼のまま。'
          + `${snapCams.length === 2 ? '前後' : '前'}カメラ）。`
          + 'チェッカーボード校正はランチャーの「カメラ校正」で'
          + (snapshot.last ? `\n直近: ${snapshot.last}` : '')
        }
      >
        📷
      </button>
      {canUndistort && (
        <button
          className={`recbtn ${cam?.undistort ? 'active' : ''}`}
          disabled={!ch}
          onClick={() => ch?.setCamera({ undistort: !cam?.undistort })}
          title="配信映像の魚眼補正（Pi側で仮想ピンホールに変換）。記録・撮影は常に魚眼のまま"
        >
          {cam?.undistort ? '補正' : '魚眼'}
        </button>
      )}
      {snapshot.error && <span className="badge-bad" title={snapshot.error}>撮影失敗</span>}
    </div>
  )
}
