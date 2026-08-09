/**
 * イベント駆動の状態だけを持つ store（`architecture.md` §10.4 の3層目）。
 *
 * **20Hz で変わるものはここに入れない。** 接続状態・操縦権・設定の類だけ。
 */
import { create } from 'zustand'
import type { ControlStatus } from '../types'

export type InputSource = 'none' | 'keyboard' | 'gamepad' | 'slider'

/** ラジコンの上限。**表示用の見た目の上限であって安全装置ではない。**
 * 本当の上限は io_node の `--max-speed` / `--max-steer`（Pi 側）と STM32。
 *
 * ⚠ **io_node 側の `--max-speed` と手で合わせること。** ずれていると
 * GUI が「0.33 m/s 出している」と表示しているのに Pi 側で切られて実際は出ない、
 * という状態になる。合わせ先は `raspi/setup/install_services.sh` の `ARM=` 行。 */
export const UI_MAX_SPEED = 0.6 // m/s（io_node の --max-speed 0.6 と一致させてある）
/** ±45°。**大舵角では前輪オドメトリの射影誤差が効く**（45° で 1/cos ≒ 1.41倍）。
 * 据え切りを続けるとステア MD が過熱するので `temp[2]` を見ておくこと */
export const UI_MAX_STEER = 0.785 // rad = 45°

/**
 * 速度レンジ。`UI_MAX_SPEED` に対する倍率。
 *
 * **既定は低速側**。`Shift`（パッドは R1）を押している間だけ全開になる。
 * 「いつでも全開が出せる」より「全開にするには意識的な操作が要る」方が、
 * 狭い場所で壁に刺さる回数が減る。
 */
export const CRUISE_SCALE = 0.55
export const BOOST_SCALE = 1.0

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

/**
 * 灯火モード（`COMMAND.flags` bit3-4）。
 *
 * ⚠ **v0.4 の 1ビット `light` とは値の意味が違う。** 旧 `light=1` は全光量だったが、
 * v0.5 の `1` は DAYTIME（減光）である。旧 GUI の値をそのまま送ると暗いまま走る。
 */
export const LIGHT_OFF = 0
export const LIGHT_DAYTIME = 1
export const LIGHT_NORMAL = 2
export const LIGHT_LABEL = ['消灯', 'DAY', 'NORMAL']
/** `L` キー / パッド △ で回す順。**消灯を含む**（v0.4 では消灯にできなかった） */
export const LIGHT_CYCLE = [LIGHT_OFF, LIGHT_DAYTIME, LIGHT_NORMAL]

/**
 * 後輪1輪あたりの制動トルクの上限 [N·m]。**STM32 の `DRIVE_MAX_BRAKE_TORQUE_NM` と
 * `raspi/msgs/convert.py` の `MAX_BRAKE_TORQUE_NM` に合わせること。**
 * 超えた値を送っても黙って丸められ、「スライダを上げても効きが変わらない」に見える。
 */
export const MAX_BRAKE_TORQUE_NM = 0.075
/** スライダの最小。**0 は「未指定 ＝ 最大制動」を意味するので選ばせない** */
export const MIN_BRAKE_TORQUE_NM = 0.005
/**
 * 制動トルクの既定値。**上限そのもの**にしてある。
 * 「効きすぎて驚く」より「踏んだのに止まらない」方が危険なので、
 * 弱める操作を人間の明示的な意思にする。低速の詰め作業で弱めたいときだけ下げる。
 */
export const DEFAULT_BRAKE_TORQUE_NM = MAX_BRAKE_TORQUE_NM

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
  /** Shift / パッド R1 を押している間だけ true。速度レンジが上がる */
  boost: boolean
  /** 直前に DISARM した理由。**「なぜ止まったか」が分からないのが一番消耗する** */
  disarmReason: string

  // ── 補機（v0.5） ──
  //
  // **押している間だけ立つものは `boost` と同じ扱い**で、rAF ループが値の変化時だけ
  // ここへ書き戻す。毎フレーム set すると 60Hz で再レンダリングが走る。
  /** Space / パッド L1 を押している間。`brake_torque` を直接掛けている */
  braking: boolean
  /** H / パッド A を押している間。**鳴りっぱなしになる** */
  horning: boolean
  /** P / パッド X を押している間。前照灯だけが全光量 */
  passing: boolean
  /** 0=消灯 1=DAYTIME 2=NORMAL。**ARM 中しか反映されない**（§下） */
  lightMode: number
  /** ブレーキの強さ [N·m/輪]。0 は送らない（未指定＝最大制動になってしまう） */
  brakeTorque: number

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
  boost: false,
  disarmReason: '',

  braking: false,
  horning: false,
  passing: false,
  lightMode: LIGHT_OFF,
  brakeTorque: DEFAULT_BRAKE_TORQUE_NM,

  diagOpen: false,
  lidarZoom: 4, // 画面半径が何メートルぶんか
  lidarFollow: true,
  pathGuide: true,
  rearBig: false,

  set: (p) => set(p),
}))
