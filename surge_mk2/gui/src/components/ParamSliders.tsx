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
 *
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

/**
 * `show_if`（`ParamSpec.show_if`）を満たすか。条件のキーが宣言に無い（隠し項目など）ときは、
 * そのキーの既定値で判定する。値は浮動小数なので刻みの半分の幅で比べる
 */
export function shown(
  p: AutoParamSpec,
  params: AutoParamSpec[],
  values: Record<string, number>,
): boolean {
  const cond = p.show_if
  if (!cond) return true
  for (const [key, allowed] of Object.entries(cond)) {
    const spec = params.find((q) => q.key === key)
    const v = values[key] ?? spec?.default
    if (v === undefined) continue
    const tol = (spec?.step ?? 1) / 2
    if (!allowed.some((a) => Math.abs(a - v) < tol)) return false
  }
  return true
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
  // 条件（`show_if`）を満たさない項目は出さない。組が空になれば見出しごと消える
  const groups = byGroup(params.filter((p) => shown(p, params, values)))
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
  return (
    <ParamGrid
      params={params.filter((p) => shown(p, params, values))}
      values={values}
      onChange={onChange}
      disabled={disabled}
    />
  )
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
        if (p.kind === 'bool') {
          return (
            <label key={p.key} className="auto-param auto-param-bool" title={p.note}>
              <input
                type="checkbox"
                checked={v >= 0.5}
                disabled={disabled}
                onChange={(e) => onChange(p.key, e.target.checked ? p.max : p.min)}
              />
              <span>{p.label}</span>
            </label>
          )
        }
        if (p.kind === 'choice' && p.choices && p.choices.length > 0) {
          return (
            <div key={p.key} className="auto-param" title={p.note}>
              <span className="auto-param-head">{p.label}</span>
              <div className="auto-param-choice">
                {p.choices.map((name, i) => {
                  const val = p.min + i * p.step
                  return (
                    <button
                      key={name}
                      type="button"
                      className={Math.abs(v - val) < p.step / 2 ? 'active' : ''}
                      disabled={disabled}
                      onClick={() => onChange(p.key, val)}
                    >
                      {name}
                    </button>
                  )
                })}
              </div>
            </div>
          )
        }
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
