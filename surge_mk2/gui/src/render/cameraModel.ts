/**
 * カメラのレンズモデル（光線 → 画素）。**`raspi/core/camera_model.py` と同じ式。**
 * 片方だけ直すと、進路ガイド（GUI）と IPM（Pi）が別のカメラを見ていることになる。
 *
 * 3種類を同じ形（`Lens`）で扱う:
 *
 * - **魚眼**（校正済み、`VEHICLE.cam*.fisheye`）: Kannala-Brandt（OpenCV `cv2.fisheye`）。
 *   θd = θ(1 + k1θ² + k2θ⁴ + k3θ⁶ + k4θ⁸)
 * - **補正映像の仮想ピンホール**（`telemetry_node` が remap した映像を表示中）:
 *   hfov = `undistortHfov` のピンホール。Pi 側 `virtual_pinhole_maps()` が
 *   未校正時と**同じ式**で作っているので、ピンホールの式に画角を入れ替えるだけでよい
 * - **未校正ピンホール**: `hfov` からの近似（従来どおり）
 *
 * カメラ座標は OpenCV と同じ（x=右, y=下, z=光軸）。
 */
import type { FisheyeCalib } from '../generated/vehicle'

export type LensKind = 'fisheye' | 'undistorted' | 'pinhole'

export interface Lens {
  kind: LensKind
  /** カメラ座標の光線 → 表示画像の画素。写らない（背後など）なら null */
  project(x: number, y: number, z: number): [number, number] | null
  /** 光軸の画素位置（roll の回転中心などに使う） */
  cx: number
  cy: number
}

/** 光軸方向の距離がこれ未満は「真下・背後」として捨てる（`ipm.py` の `_MIN_ZC` と同じ） */
export const MIN_ZC = 0.15

/** 表示画像（幅 `w`・高さ `h`、下端 `bottomCrop` 切り落とし済み）のピンホール。
 * `ipm.camera_intrinsics()` と同一の式 */
export function pinholeLens(w: number, h: number, hfov: number, bottomCrop: number,
  kind: LensKind = 'pinhole'): Lens {
  const f = w / 2 / Math.tan(hfov / 2)
  const cx = w / 2
  // 下端クロップぶん、光軸（センサー中心）は配信画像の中央より下にずれる
  const cy = h / (2 * (1 - bottomCrop))
  return {
    kind,
    cx,
    cy,
    project: (x, y, z) => (z < 1e-9 ? null : [cx + (x * f) / z, cy + (y * f) / z]),
  }
}

/** 表示画像に縮尺した魚眼。**校正値はフル画角・クロップ前の画素**なので、
 * クロップ後の高さからクロップ前の高さに戻して倍率を出す（`FisheyeCalib.scaled()` と同じ） */
export function fisheyeLens(w: number, h: number, calib: FisheyeCalib, bottomCrop: number): Lens {
  const sx = w / calib.width
  const sy = h / (1 - bottomCrop) / calib.height
  const fx = calib.fx * sx
  const fy = calib.fy * sy
  const cx = calib.cx * sx
  const cy = calib.cy * sy
  const [k1, k2, k3, k4] = calib.k
  return {
    kind: 'fisheye',
    cx,
    cy,
    project: (x, y, z) => {
      const r = Math.hypot(x, y)
      if (r < 1e-12) return z > 0 ? [cx, cy] : null
      const th = Math.atan2(r, z)
      const t2 = th * th
      const thd = th * (1 + t2 * (k1 + t2 * (k2 + t2 * (k3 + t2 * k4))))
      const s = thd / r
      return [cx + fx * x * s, cy + fy * y * s]
    },
  }
}
