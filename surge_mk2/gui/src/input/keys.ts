/** 操縦キーの割り当て（`useKeyboardInput.ts` と送信ループで共有する）。操作表は `useDriving.ts` 冒頭 */
export const SPEED_KEYS_FWD = ['KeyW', 'ArrowUp']
export const SPEED_KEYS_REV = ['KeyS', 'ArrowDown']
// ←/→ はギアチェンジに使うので、キーボードの舵は A/D だけ（下の GEAR_KEY_DOWN/UP 参照）
export const STEER_KEYS_LEFT = ['KeyA']
export const STEER_KEYS_RIGHT = ['KeyD']
export const BRAKE_KEY = 'Space'
export const HORN_KEY = 'KeyH'
export const PASSING_KEY = 'KeyP'
export const LIGHT_KEY = 'KeyL'
/** ギアチェンジ。パッドの L1/R1 と同じ役割——←＝シフトダウン、→＝シフトアップ
 * （`shiftGear()` 参照）。灯火と同じくキーの立ち上がりエッジで1段だけ動く、押しっぱなし無効 */
export const GEAR_KEY_DOWN = 'ArrowLeft'
export const GEAR_KEY_UP = 'ArrowRight'
export const DRIVING_KEYS = [
  ...SPEED_KEYS_FWD, ...SPEED_KEYS_REV, ...STEER_KEYS_LEFT, ...STEER_KEYS_RIGHT,
  BRAKE_KEY, HORN_KEY, PASSING_KEY,
]
