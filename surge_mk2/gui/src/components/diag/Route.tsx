/**
 * 経路図の部品 — 箱（`Node`）を線（`Hop`）でつなぎ、線の上下にその区間の数字を書く。
 *
 * 通信（ブラウザ ⇄ RasPi ⇄ STM32 ⇄ MD）と電源（バッテリー → 負荷）の両方で使う。
 * **どの区間が悪いかを、線の色で見せる**のが狙い（数字の表では区間の対応が読めない）。
 */
import type { ReactNode } from 'react'
import type { Tint } from './Card'

/** 区間。`lines` は線の上と下に1行ずつ（1つなら上だけ、空なら線だけ） */
export function Hop({ level, lines = [] }: { level: Tint; lines?: string[] }) {
  return (
    <div className={`dg-hop is-${level}`}>
      {lines.map((l) => (
        <span key={l}>{l}</span>
      ))}
    </div>
  )
}

export function Node({
  name,
  sub,
  level,
  value,
  unit,
}: {
  name: string
  sub?: string
  level?: Tint
  /** 箱の主役になる大きい数字（バッテリーの電圧など） */
  value?: string
  unit?: string
}) {
  return (
    <div className={`dg-node${level ? ` is-${level}` : ''}`}>
      <b>{name}</b>
      {value != null && (
        <span className="dg-node-value">
          {value}
          {unit && <small>{unit}</small>}
        </span>
      )}
      {sub && <small>{sub}</small>}
    </div>
  )
}

/** 1つの箱から複数の箱へ分かれる部分。子は `Branch` */
export function Fan({ children }: { children: ReactNode }) {
  return <div className="dg-fan">{children}</div>
}

export function Branch({ level, lines, children }: { level: Tint; lines?: string[]; children: ReactNode }) {
  return (
    <div className="dg-fan-row">
      <Hop level={level} lines={lines} />
      {children}
    </div>
  )
}
