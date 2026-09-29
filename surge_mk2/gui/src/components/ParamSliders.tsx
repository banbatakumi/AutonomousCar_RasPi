/**
 * `Planner.params`（`ParamSpec` の宣言）をスライダの並びに描く共通部品。
 *
 * 元は `AutoPanel.tsx` の `auto-param-grid` だったが、システム同定タブ
 * （`SysIdView.tsx`）も同じ「パラメータ宣言→スライダ」を要るため切り出した。
 * **どのモードのパラメータかは呼び出し側が決める**——このコンポーネントは
 * `params`/`values`/`onChange` を渡されるだけで `auto.mode` を意識しない。
 */
import type { AutoParamSpec } from '../types'

/**
 * `group`（`ParamSpec.group`）ごとに見出しを付けて並べる。組の順は最初に出てきた順
 * （宣言の途中で同じ組が再び出てきても、その組の中へまとめる）。組の無い宣言は従来どおり
 * 1つの並びで出す——地図作成（Follow the Gap）と本番走行のパラメータを見分けやすくするため
 * （バンビの指示、2026-09-29）
 */
function byGroup(params: AutoParamSpec[]): [string, AutoParamSpec[]][] {
  const out = new Map<string, AutoParamSpec[]>()
  for (const p of params) {
    const g = p.group ?? ''
    if (!out.has(g)) out.set(g, [])
    out.get(g)!.push(p)
  }
  return [...out.entries()]
}

export function ParamSliders({
  params,
  values,
  onChange,
  disabled,
}: {
  params: AutoParamSpec[]
  values: Record<string, number>
  onChange: (key: string, value: number) => void
  /** trueの間はスライダを操作不可にする。値は`merged_params()`で毎周期
   * 読み直されるので、途中で動かすと進行中の試験の意味が変わってしまう
   * （`SysIdView.tsx`が試験実行中に渡す）。省略時は従来通り常に操作可 */
  disabled?: boolean
}) {
  const groups = byGroup(params)
  if (groups.length > 1 || (groups.length === 1 && groups[0]?.[0])) {
    return (
      <div className="auto-param-groups">
        {groups.map(([g, ps]) => (
          <div key={g} className="auto-param-group">
            {g && <div className="auto-param-group-head">{g}</div>}
            <ParamGrid params={ps} values={values} onChange={onChange} disabled={disabled} />
          </div>
        ))}
      </div>
    )
  }
  return <ParamGrid params={params} values={values} onChange={onChange} disabled={disabled} />
}

function ParamGrid({
  params,
  values,
  onChange,
  disabled,
}: {
  params: AutoParamSpec[]
  values: Record<string, number>
  onChange: (key: string, value: number) => void
  disabled?: boolean
}) {
  return (
    <div className="auto-param-grid">
      {params.map((p) => {
        const v = values[p.key] ?? p.default
        return (
          <label key={p.key} className="auto-param" title={p.note}>
            <span className="auto-param-head">
              {p.label}
              <b>
                {v.toFixed(decimalsFor(p.step))}
                {p.unit && <i> {p.unit}</i>}
              </b>
            </span>
            <input
              type="range"
              min={p.min}
              max={p.max}
              step={p.step}
              value={v}
              disabled={disabled}
              onChange={(e) => onChange(p.key, Number(e.target.value))}
            />
          </label>
        )
      })}
    </div>
  )
}

/** スライダの刻みから表示桁数を決める。0.01 なら2桁、1 なら0桁 */
function decimalsFor(step: number): number {
  if (step >= 1) return 0
  return Math.min(3, Math.ceil(-Math.log10(step)))
}
