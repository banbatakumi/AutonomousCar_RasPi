/**
 * 診断タブのカード部品。**見た目はここだけで決める**（ページ側は中身を並べるだけ）。
 *
 *   Card   枠と見出し。`level` を渡すと左の罫線が状態色になる
 *   Row    ラベルと値の1行
 *   Dot    状態の点
 */
import type { ReactNode } from 'react'
import type { Level } from '../../format'

export type Tint = Level | 'na'

export function Card({
  title,
  level,
  wide,
  children,
}: {
  title: string
  level?: Tint
  /** 横2枠ぶん使う（表など） */
  wide?: boolean
  children: ReactNode
}) {
  return (
    <section className={`dg-card${level ? ` is-${level}` : ''}${wide ? ' wide' : ''}`}>
      <h4>{title}</h4>
      {children}
    </section>
  )
}

export function Row({ label, level, children }: { label: string; level?: Tint; children: ReactNode }) {
  return (
    <div className="dg-row">
      <span>{label}</span>
      <b className={level && level !== 'ok' ? `lv-${level}` : undefined}>{children}</b>
    </div>
  )
}

export function Dot({ level }: { level: Tint }) {
  return <i className={`dg-dot is-${level}`} />
}
