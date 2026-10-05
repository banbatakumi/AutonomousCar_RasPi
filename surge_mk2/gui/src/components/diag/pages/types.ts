import type { Numbers } from '../../../bus/live'
import type { ControlStatus } from '../../../types'
import type { Check } from '../checks'

/** サブページが受け取るもの。8Hz で差し替わる（`useNumbers`） */
export type PageProps = {
  n: Numbers
  st: ControlStatus | null
  checks: Check[]
}
