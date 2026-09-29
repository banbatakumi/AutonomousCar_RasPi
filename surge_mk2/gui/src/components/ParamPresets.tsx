/**
 * パラメータのプリセット（名前を付けて保存・読込・削除）と「初期値に戻す」（`AutoPanel.tsx` のパラメータ欄）。
 *
 * 保存先は telemetry_node が動いている機械の `config/auto_presets.json`（実機なら Pi、`sim.run`
 * なら Mac）で、**モードごと**に分けて持つ（planner ごとにパラメータの顔ぶれが違うため）。
 * 読み込むとサーバがクランプし直し、知らないキーは捨て、足りないキーは既定値で埋める
 * （`raspi/nodes/telemetry_node.py` の `_on_auto_presets`）。
 *
 * 今の値そのもの（スライダを動かした結果）は、これとは別に `config/auto.json` へ毎回保存され、
 * 次の起動で戻る。プリセットは「覚えておきたい組み合わせ」を名前で呼び戻すためのもの
 * （バンビの指示、2026-09-29）。
 */
import { useState } from 'react'
import type { ControlChannel } from '../ws/control'

export function ParamPresets({ ch, presets }: { ch: ControlChannel | null; presets: string[] }) {
  const [picked, setPicked] = useState('')
  const [name, setName] = useState('')
  // 選んでいたプリセットが消えたら（削除した等）未選択に戻す
  const current = presets.includes(picked) ? picked : ''

  const save = () => {
    const n = name.trim()
    if (!n || !ch) return
    if (presets.includes(n) && !window.confirm(`「${n}」を今の値で上書きしますか？`)) return
    ch.saveAutoPreset(n)
    setPicked(n)
    setName('')
  }

  return (
    <div className="param-presets">
      <select value={current} onChange={(e) => setPicked(e.target.value)}>
        <option value="">（プリセットを選択）</option>
        {presets.map((p) => (
          <option key={p} value={p}>
            {p}
          </option>
        ))}
      </select>
      <button
        disabled={!ch || !current}
        onClick={() => {
          if (window.confirm(`「${current}」を読み込みますか？（今の値は置き換わります）`)) {
            ch?.loadAutoPreset(current)
          }
        }}
      >
        読込
      </button>
      <button
        disabled={!ch || !current}
        onClick={() => {
          if (window.confirm(`プリセット「${current}」を削除しますか？`)) ch?.deleteAutoPreset(current)
        }}
      >
        削除
      </button>
      <input
        type="text"
        placeholder="名前を付けて保存"
        maxLength={40}
        value={name}
        onChange={(e) => setName(e.target.value)}
        onKeyDown={(e) => {
          if (e.key === 'Enter') save()
        }}
      />
      <button disabled={!ch || !name.trim()} onClick={save}>
        保存
      </button>
      <button
        className="param-presets-reset"
        disabled={!ch}
        onClick={() => {
          if (window.confirm('このモードのパラメータを初期値に戻しますか？')) ch?.resetAutoParams()
        }}
      >
        初期値に戻す
      </button>
    </div>
  )
}
