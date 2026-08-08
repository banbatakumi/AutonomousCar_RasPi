/**
 * イベント駆動の状態だけを持つ store（`architecture.md` §10.4 の3層目）。
 *
 * **20Hz で変わるものはここに入れない。** 接続状態・操縦権・設定の類だけ。
 */
import { create } from 'zustand'
import type { ControlStatus } from '../types'

export type InputSource = 'none' | 'keyboard' | 'gamepad' | 'slider'

/** ラジコンの上限。**表示用の見た目の上限であって安全装置ではない。**
 * 本当の上限は io_node の `--max-speed` / `--max-steer`（Pi 側）と STM32。 */
export const UI_MAX_SPEED = 1.0 // m/s
/** ±60°。**大舵角では前輪オドメトリの射影誤差が効く**（60° で 1/cos = 2倍）。
 * 据え切りを続けるとステア MD が過熱するので `temp[2]` を見ておくこと */
export const UI_MAX_STEER = 1.047 // rad = 60°

/**
 * ARM を保持したまま無操作でいられる時間。これを過ぎたら自動 DISARM。
 *
 * **「キーを離したら即停止」ではない。** 握り続ける方式より操作性を優先した
 * 選択で、手を離しても最大この時間は armed のままモータが励磁される
 * （速度指令は 0 に落ちるので進みはしないが、完全な停止ではない）。
 *
 * 20秒は**この保険がほぼ効かない長さ**である。実質的な停止手段は
 * `Esc` / E-STOP ボタン / パッドの B / フォーカス喪失 / 通信断の方であり、
 * このタイマーは「操作を忘れて放置した」場合の最後の受け皿でしかない。
 */
export const ARM_IDLE_TIMEOUT_MS = 20_000

type UiState = {
  telemetryOpen: boolean
  controlOpen: boolean
  status: ControlStatus | null
  /** 自分が操縦権を持っているか */
  hasControl: boolean
  deniedBy: string | null
  /** GUI ↔ Pi の往復 [ms]。UART 区間の遅延とは別物 */
  wsRttMs: number | null

  /** ARM ボタンで保持中か。**これが true の間だけ `cmd` を送る** */
  armRequested: boolean
  /** 実際に指令を送れているか（操縦権があり armRequested）。表示用 */
  deadman: boolean
  inputSource: InputSource
  /** 直前に DISARM した理由。**「なぜ止まったか」が分からないのが一番消耗する** */
  disarmReason: string

  /** 異常時のみ出す層(C)を人間が明示的に開いた状態 */
  diagOpen: boolean
  /** LiDAR ビューの設定 */
  lidarZoom: number
  lidarFollow: boolean
  /** カメラの進路ガイド。**カメラ校正前なので暫定表示** */
  pathGuide: boolean
  rearBig: boolean

  set: (p: Partial<UiState>) => void
}

export const useUi = create<UiState>((set) => ({
  telemetryOpen: false,
  controlOpen: false,
  status: null,
  hasControl: false,
  deniedBy: null,
  wsRttMs: null,

  armRequested: false,
  deadman: false,
  inputSource: 'none',
  disarmReason: '',

  diagOpen: false,
  lidarZoom: 4, // 画面半径が何メートルぶんか
  lidarFollow: true,
  pathGuide: true,
  rearBig: false,

  set: (p) => set(p),
}))
