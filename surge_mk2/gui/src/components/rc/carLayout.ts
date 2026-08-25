/**
 * `DrivePanel`（車体を上から見た図）と `ProximityRing`（障害物近接リング）が
 * 共有する、実寸 [m]（`vehicle.toml` 由来）→ svg 座標の変換と車輪配置。
 *
 * 元々 `DrivePanel` は車輪位置を見た目だけで決めた固定値（車体の外側に車輪が
 * はみ出す、いわゆるデフォルメ絵）を持っていた。`ProximityRing` は実寸の
 * `footprint` からリングを描くため、実車のトレッド（0.155m）が車体幅
 * （0.18m）より狭いことがデフォルメ絵の車輪配置に反映されず、車輪・スリップ・
 * トルクの表示がリングの表示域（車体のすぐ外）に被っていた（2026-08-25）。
 * 座標変換を1箇所にまとめ、車輪位置もトレッド・ホイールベース・車輪径の
 * 実測値から算出することで、両者が同じ実寸比率に揃うようにする。
 */
import { VEHICLE } from '../../generated/vehicle'

/** 車体矩形（svg座標）。`footprint` を実寸比率のままここに収めるスケールで
 * `SCALE_X`/`SCALE_Y`/`SVG_CX`/`SVG_CY` を決めるので、この4値が基準 */
export const BODY_X = 58
export const BODY_Y = 26
export const BODY_W = 84
export const BODY_H = 182

const xs = VEHICLE.footprint.map((p) => p[0])
const ys = VEHICLE.footprint.map((p) => p[1])
export const FOOT_MIN_X = Math.min(...xs)
export const FOOT_MAX_X = Math.max(...xs)
export const FOOT_MIN_Y = Math.min(...ys)
export const FOOT_MAX_Y = Math.max(...ys)

/** 実寸 [m] → svg 座標のスケール。横方向・縦方向で別スケール（車体矩形が正方形でないため） */
export const SCALE_X = BODY_W / (FOOT_MAX_Y - FOOT_MIN_Y)
export const SCALE_Y = BODY_H / (FOOT_MAX_X - FOOT_MIN_X)
export const SVG_CX = BODY_X + BODY_W / 2
export const SVG_CY = BODY_Y + FOOT_MAX_X * SCALE_Y

/** 実寸 (x=前, y=左) [m] → svg 座標 */
export function toSvg(xPhys: number, yPhys: number): [number, number] {
  return [SVG_CX - yPhys * SCALE_X, SVG_CY - xPhys * SCALE_Y]
}

// ── 車輪配置。トレッド・ホイールベース・車輪径は vehicle.toml の実測値そのまま反映する ──

const halfTrackSvg = (VEHICLE.track / 2) * SCALE_X
export const CX_L = SVG_CX - halfTrackSvg
export const CX_R = SVG_CX + halfTrackSvg

/** base_link（後軸中心）が x=0、前軸は x=wheelbase */
const [, cyRear] = toSvg(0, 0)
const [, cyFront] = toSvg(VEHICLE.wheelbase, 0)
export const CY_REAR = cyRear
export const CY_FRONT = cyFront

/** 車輪の直径（転動方向）。トレッド方向の厚みは `vehicle.toml` に値が無いので
 * `DrivePanel` 側で見た目重視の固定値を持つ */
export const WHEEL_H = 2 * VEHICLE.wheelRadius * SCALE_Y
