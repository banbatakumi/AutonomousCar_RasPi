/**
 * 自動運転ビューの地図パネル — `slam2d_raceline`選択時だけ、車体図
 * （`AutoView.tsx`の`.auto-car`）の左に挟む正方形パネル（2026-09-03新設）。
 *
 * 旧・独立タブ「地図生成」（`views/MapView.tsx`、廃止）と違い、**地図と
 * グリッドだけ**を見せる。段（EXPLORE/BUILD/LOCATE/RACE）のラベルだけは
 * 「今どこに居るか」を一目で分かるよう残すが、一致度・自己位置・格子サイズ
 * 等の数値パネルは持たない——要求により一旦すべて非表示にした（数値を
 * 見たければ診断タブ、または将来復活させる場合はここに足す）。
 *
 * ## クリックで自己位置探索のヒントを送る
 *
 * `LOCATE`段（保存済み地図から自己位置を復元中）でパネルをクリックすると、
 * その近傍だけに探索を絞り込める（`slam2d.core.localize.GlobalLocalizer`の
 * `hint`）。他の段でクリックしても`ch.setLocateHint()`は送られるが、
 * plannerが`LOCATE`段でないときは`request_locate_hint`が記録するだけで
 * 実害は無い（次回`LOCATE`に入ったときに使われる）。
 *
 * **「レーシングライン走行」を押す前（`SlamRaceButtons`でドロップダウンから
 * 地図を選んだだけの状態）でもクリックできる。** この場合`ch.setLocateHint()`
 * が今のplannerインスタンスに意思を記録しつつ、`setPreviewHint()`
 * （`bus/mapPreview.ts`）が地図パネル上に十字マーカーを描く——押す前から
 * 「ここに置いたつもり」が見えないと、地図内の任意の場所からレーシング
 * ライン走行を始められる意味が無い（バンビの指示、2026-09-03）。
 *
 * ## 「地図を削除」は隅のアイコンボタンに畳む
 *
 * 数値パネルが無くなったので、旧`MapPanel`の`.map-actions`のような専用の
 * ボタン列を置く場所が無い。地図に対する操作なので地図パネル自身の上に
 * 置くのが自然、という判断（ユーザー指示により配置は設計者判断）。
 */
import { live } from '../bus/live'
import { useNumbers } from '../bus/live'
import { mapPreview, setPreviewHint } from '../bus/mapPreview'
import { clearTrail, MapCanvas } from '../render/MapCanvas'
import type { ControlChannel } from '../ws/control'

const PHASE_LABEL: Record<string, string> = {
  EXPLORE: '地図作成中',
  BUILD: '経路生成中',
  //: 地図・経路ができて自動保存済み。「レーシングライン走行」ボタンが
  //: 押されるまでここで待つ（バンビの指示、2026-09-03）
  DONE: '保存済み・待機中',
  LOCATE: '自己位置を復元中',
  RACE: '走行中',
}

export function AutoMapPanel({ ch }: { ch: ControlChannel | null }) {
  useNumbers() // 8Hzで再レンダリングを起こすためだけに呼ぶ（バッジ・プレビュー名の更新用）
  const phase = live.auto?.phase || ''
  const previewing = mapPreview.data !== null
  const label = previewing ? `下見中: ${mapPreview.name}` : (PHASE_LABEL[phase] ?? '待機')

  const onWorldClick = (x: number, y: number) => {
    ch?.setLocateHint(x, y)
    setPreviewHint(x, y)
  }

  return (
    <div className="auto-map">
      <MapCanvas onWorldClick={onWorldClick} />
      <div className="auto-map-badge">
        <span className={!previewing && phase === 'RACE' ? 'lv-ok' : 'dim'}>{label}</span>
      </div>
      <button
        className="auto-map-clear"
        title="地図を削除"
        disabled={!ch}
        onClick={() => {
          const msg =
            phase === 'RACE'
              ? '走行中です。本当に地図を削除しますか？（次の周期で制動がかかります）'
              : '地図を削除しますか？'
          if (!window.confirm(msg)) return
          ch?.clearMap()
          clearTrail()
        }}
      >
        🗑
      </button>
    </div>
  )
}
