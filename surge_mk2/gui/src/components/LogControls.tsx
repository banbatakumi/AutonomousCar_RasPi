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
 */
import { formatBytes, formatElapsed } from '../format'
import { useMcapDownload } from '../hooks/useMcapDownload'
import { useUi } from '../store/ui'
import type { ControlChannel } from '../ws/control'

export function LogControls({ ch }: { ch: ControlChannel | null }) {
  const sfl = useUi((s) => s.sfl)
  const mcap = useUi((s) => s.mcap)
  const sflActive = sfl?.active ?? false
  const mcapActive = mcap?.active ?? false
  const { bufferedBytes, start } = useMcapDownload(ch)

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
    </div>
  )
}
