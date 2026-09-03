/**
 * 保存済み地図の下見（プレビュー） — 「レーシングライン走行」を押す前に、
 * ドロップダウンで選んだ地図を表示し、自己位置ヒントをクリックで置けるように
 * する（2026-09-03、バンビの指示）。
 *
 * `bus/live.ts`の`live.map`/`live.auto`と同じ「Reactの外に置く、rAFの描画
 * ループが毎フレーム直接読む」流儀。プレビューは`/ws/map`のライブ更新とは
 * 別物（`request_load()`する前の、まだplannerに読み込まれていない地図を
 * 見るためのもの）なので、`live.map`とは混ぜずに独立させてある。
 *
 * `MapCanvas.tsx`は`mapPreview.data`が非nullの間はそれを描き、`live.auto`
 * （自車位置・障害物・軌跡）は出さない——プレビュー中の地図には対応する
 * 実測の自己位置が無いため。
 */
import { decode } from '@msgpack/msgpack'
import type { AutoMapMsg, MapData } from '../types'
import { build } from '../ws/map'

export const mapPreview = {
  /** プレビュー中の地図名。空文字 = プレビューしていない */
  name: '',
  data: null as MapData | null,
  /** クリックで置いた自己位置ヒント（mapフレーム座標）。地図を切り替えたら消す */
  hint: null as { x: number; y: number } | null,
  /** 取得中かどうか（連打で古い応答が新しい選択を上書きしないようにするための世代カウンタ） */
  loading: false,
}

let requestGen = 0

/** ドロップダウンで地図を選んだ／外した。空文字を渡すとプレビューを消す。 */
export async function selectPreviewMap(name: string): Promise<void> {
  const gen = ++requestGen
  mapPreview.name = name
  mapPreview.hint = null
  mapPreview.data?.bitmap?.close()
  mapPreview.data = null
  if (!name) return

  mapPreview.loading = true
  try {
    const res = await fetch(`/maps/${encodeURIComponent(name)}/preview`)
    if (!res.ok) return
    const buf = new Uint8Array(await res.arrayBuffer())
    const msg = decode(buf) as AutoMapMsg
    const built = await build(msg)
    // **古い選択の応答が新しい選択を上書きしないように。** 連続してドロップダウンを
    // 変えると fetch の完了順が入れ替わりうる（`ws/map.ts`のseqガードと同じ理由）
    if (gen !== requestGen) {
      built?.bitmap?.close()
      return
    }
    mapPreview.data = built
  } catch {
    // 壊れた/届かない応答は「プレビュー無し」のまま。地図一覧自体は生きている
  } finally {
    if (gen === requestGen) mapPreview.loading = false
  }
}

/** 地図パネルのクリックで自己位置ヒントを置く（表示用。サーバへの送信は別途`ch.setLocateHint()`）。 */
export function setPreviewHint(x: number, y: number): void {
  mapPreview.hint = { x, y }
}

/** 「地図を作成」「レーシングライン走行」を押した等、プレビューの役目が終わったとき。 */
export function clearPreview(): void {
  requestGen++
  mapPreview.name = ''
  mapPreview.hint = null
  mapPreview.data?.bitmap?.close()
  mapPreview.data = null
  mapPreview.loading = false
}
