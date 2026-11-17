/**
 * カメラの役割の対応（GUI 側）。カメラは前方ステレオの左右2台。
 *
 * **`front` は右の主カメラ**、2台目は `left`。後ろを見るカメラは無い。自動運転画面の2台目の枠は
 * 深度映像（`DepthView`、`stereo_node` の `/ws/camera/depth`。`docs/stereo_plan.md` 段階4）。
 *
 * 対応そのものは Pi 側の `raspi/core/vehicle.py`（`CAMERA_DEVICES`）が正。
 */
import { VEHICLE as VEHICLE_GEOM } from '../generated/vehicle'

/** カメラの役割（`/ws/camera/<役割>`・`/snapshot/<役割>.png`・`image/<役割>`） */
export type CamId = 'front' | 'left'

export const CAM_LABEL: Record<CamId, string> = { front: '前方', left: '左' }

/** その役割のカメラの取付・レンズ（`vehicle.toml` 由来） */
export function camGeom(cam: CamId) {
  return cam === 'front' ? VEHICLE_GEOM.camFront : VEHICLE_GEOM.camLeft
}

/** センサー自体のネイティブ比率（4:3、`camera_node.py --size` の既定 640x480） */
const SENSOR_ASPECT = 4 / 3

/** 配信される画像の縦横比の見込み（センサーの 4:3 から `topCrop`・`bottomCrop` を切った比率）。
 * **最初のフレームが届くまでの仮の値**——実機ではセンサーが要求どおりのモードを取らないことがあるので、
 * 届いたら `CameraView` の `onAspect`（デコードした画像の実寸）に置き換える */
export function expectedCamAspect(cam: CamId): number {
  const g = camGeom(cam)
  return SENSOR_ASPECT / (1 - g.topCrop - g.bottomCrop)
}
