/**
 * MT（擬似変速）のギア段と、D/R レンジ・ギアの切り替え規則。`store/ui.ts` から分けた純粋な計算だけを置く
 * （状態そのもの＝`ui.gear`・`ui.mtGear` はストアが持つ）。
 */
import type { DrivingSettings } from './settings'

/**
 * MT（擬似変速）モードのギア段。R と D1 の間に N（ニュートラル）
 * を挟んだ7段の並びで、実車のシフトと同じく「1つ隣へ」動かす前提の配列にしてある
 * （`useDriving.ts` の L1=シフトダウン/R1=シフトアップが `mtGearIndex`/`mtGearAt` で
 * 1段ずつ動かす）。D/R レンジ（`ui.gear`）とは独立——`driveMode==='mt'` のときだけ意味を持つ。
 */
export const MT_GEARS = ['R', 'N', 'D1', 'D2', 'D3', 'D4', 'D5'] as const
export type MtGear = (typeof MT_GEARS)[number]

export function mtGearIndex(g: MtGear): number {
  return MT_GEARS.indexOf(g)
}

/** 範囲外は端で止める（クランプ）。シフトダウンし過ぎて配列外に落ちない */
export function mtGearAt(i: number): MtGear {
  const clamped = Math.max(0, Math.min(MT_GEARS.length - 1, i))
  return MT_GEARS[clamped]! // clamp 済みなので必ず範囲内
}

/**
 * そのギアでの `mtMaxSpeed` に対する倍率。R は D1 と同じ刻みを流用する
 * （`DrivingSettings.mtGear1` 参照）。**N は 0**——上限が 0 になるので、
 * N のときはスロットルを踏んでも指令速度は 0 へ収れんする（実車のニュートラルと同じ）。
 * この収れんは即座のクランプではなく `mtCoast`（惰行減速）のレートでなだらかに行われる
 * ——エンジンブレーキ（`mtEngineBrake`）は乗らない（`useDriving.ts` の `tick()` 参照）
 */
export function mtGearRatio(g: MtGear, s: DrivingSettings): number {
  switch (g) {
    case 'N': return 0
    case 'R': case 'D1': return s.mtGear1
    case 'D2': return s.mtGear2
    case 'D3': return s.mtGear3
    case 'D4': return s.mtGear4
    case 'D5': return s.mtGear5
  }
}

/** 後退を選んでいるか。MT モードは `mtGear`、それ以外は D/R レンジ（`gear`）で決まる */
export function reverseSelected(ui: { settings: DrivingSettings; gear: 'D' | 'R'; mtGear: MtGear }): boolean {
  return ui.settings.driveMode === 'mt' ? ui.mtGear === 'R' : ui.gear === 'R'
}

/**
 * ギアを1段シフトしたときの変更分。`delta=-1` でダウン、`+1` でアップ。変えないときは null。
 *
 * MT モードでは `mtGear` を `MT_GEARS` 上で1段動かし（R をまたぐときだけ走行中は拒否）、
 * それ以外は `gear`（D/R）を direct-set する（down=R、up=D。走行中は拒否）。
 * パッドの L1/R1 とキーボードの←/→の両方から呼ぶ共通の規則（`input/useKeyboardInput.ts` の `shiftGear()`）。
 * `stopped` を外から渡すのは、テレメトリが届いていない間の判定を呼び出し側が持つため。
 */
export function shiftGearUpdate(
  ui: { settings: DrivingSettings; gear: 'D' | 'R'; mtGear: MtGear },
  delta: number,
  stopped: boolean,
): { mtGear: MtGear } | { gear: 'D' | 'R' } | null {
  if (ui.settings.driveMode === 'mt') {
    const next = mtGearAt(mtGearIndex(ui.mtGear) + delta)
    if (next === ui.mtGear) return null
    const crossesReverse = next === 'R' || ui.mtGear === 'R'
    if (crossesReverse && !stopped) return null
    return { mtGear: next }
  }
  const target = delta < 0 ? 'R' : 'D'
  if (ui.gear === target || !stopped) return null
  return { gear: target }
}
