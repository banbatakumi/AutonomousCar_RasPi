/**
 * 要素の実寸（`clientWidth`/`clientHeight`）を `ResizeObserver` で追う。
 *
 * `useContainFit.ts` の狭い版——アスペクト比の計算はせず、生の実測値だけ返す。
 * `RcView.tsx` がメータ行（`.rc-cam-gauges`）の高さを測って各メータへ配るのに使う。
 */
import { useEffect, useRef, useState } from 'react'

export function useElementSize<T extends HTMLElement>() {
  const ref = useRef<T>(null)
  const [size, setSize] = useState<{ width: number; height: number } | null>(null)

  useEffect(() => {
    const el = ref.current
    if (!el) return
    const measure = () =>
      setSize((prev) => {
        const w = el.clientWidth
        const h = el.clientHeight
        if (prev && prev.width === w && prev.height === h) return prev
        return { width: w, height: h }
      })
    measure()
    const ro = new ResizeObserver(measure)
    ro.observe(el)
    return () => ro.disconnect()
  }, [])

  return { ref, size }
}
