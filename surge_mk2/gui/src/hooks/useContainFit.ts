/**
 * 「コンテナに収まる最大サイズ」を実測して返す（`RcView.tsx` の映像パネル用）。
 *
 * ## なぜ CSS の aspect-ratio だけでは足りないか
 *
 * `aspect-ratio` + `max-width`/`max-height` + `width/height: auto` は本来
 * `object-fit: contain` と同じ計算になるはずだが、箱の中に `flex:1` の子
 * （`<canvas>`）がいると、**その子の intrinsic サイズ（canvas 既定の 300×150）に
 * 引っ張られて縮んだまま**になる挙動を実機のブラウザで確認した。CSS だけで
 * 確実に「常に最大」を出すのが難しいため、`ResizeObserver` で実測して
 * 明示的な px を計算する。
 *
 * ## `reservePx` — 兄弟要素ぶんの高さを先に引く
 *
 * 映像の下にメータ行があり、**メータ行にも最低限の高さを残したい**
 * （映像を最大化する計算だけだと、映像がコンテナの全高を食ってメータが
 * 0 になりうる）。呼び出し側で「メータ行に確保したい最小高さ」を渡す。
 * 映像がその最小分より小さくて済むとき（横長のコンテナなど）は、余った分は
 * 自動的にメータ行（`flex: 1` 側）に回る——ここでは映像の上限だけを決める。
 *
 * ## `widthOverride` — 幅だけ呼び出し側から渡す（2026-08-17）
 *
 * 呼び出し側（`RcView.tsx`）が、この結果（映像幅）を使って**測定対象の要素
 * 自身の幅を変える**ことがある（高さ律速で映像が狭くなった分、隣の列を
 * 広げて隙間を消す）。その場合 `el.clientWidth` をそのまま使うと
 * 「上書き→再測定→再上書き…」で循環してしまうため、幅だけは呼び出し側が
 * 別の（上書きの影響を受けない）要素から測った値を渡せるようにしている。
 * 未指定なら従来どおり `el.clientWidth` を使う。高さは対象要素からの実測
 * のままでよい（列の高さは幅の上書きでは変わらないため）。
 */
import { useEffect, useRef, useState } from 'react'

export function useContainFit<T extends HTMLElement>(aspect: number, reservePx = 0, widthOverride?: number) {
  const ref = useRef<T>(null)
  const [size, setSize] = useState<{ width: number; height: number } | null>(null)

  useEffect(() => {
    const el = ref.current
    if (!el) return

    const measure = () => {
      const cw = widthOverride ?? el.clientWidth
      const ch = Math.max(0, el.clientHeight - reservePx)
      if (!cw || !ch) return
      let w = cw
      let h = cw / aspect
      if (h > ch) {
        h = ch
        w = ch * aspect
      }
      setSize({ width: Math.floor(w), height: Math.floor(h) })
    }

    measure()
    const ro = new ResizeObserver(measure)
    ro.observe(el)
    return () => ro.disconnect()
  }, [aspect, reservePx, widthOverride])

  return { ref, size }
}
