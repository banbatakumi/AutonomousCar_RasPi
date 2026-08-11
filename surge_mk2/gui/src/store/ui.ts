/**
 * イベント駆動の状態だけを持つ store（`architecture.md` §10.4 の3層目）。
 *
 * **20Hz で変わるものはここに入れない。** 接続状態・操縦権・設定の類だけ。
 */
import { create } from 'zustand'
import type { ControlStatus, LogFile } from '../types'

export type InputSource = 'none' | 'keyboard' | 'gamepad' | 'slider'

/**
 * ラジコン操作の調整値。**設定パネルから変更でき、`localStorage` に保存される。**
 *
 * ⚠ `maxSpeed` / `maxSteer` は io_node の `--max-speed` / `--max-steer`
 * （Pi 側のハード上限。`raspi/setup/install_services.sh` の `ARM=` 行）を
 * **超えて設定してはいけない**。超えた分は Pi 側で黙って切り捨てられ、
 * 「GUI は 0.9 m/s と表示しているのに実際は 0.6 m/s しか出ない」という
 * 食い違いが起きる（`architecture.md` §10.5）。そのためレンジ上限を
 * ハード上限そのものにしてあり、**それより上には振れない**。
 *
 * ⚠ **速度の上限は3段ある。** ここと Pi を上げても、3段目で頭打ちになりうる。
 *
 *   1. ここ `PI_MAX_SPEED_CAP` …………… スライダの上限
 *   2. io_node `--max-speed` …………… 超えた分を黙って切り捨てる
 *   3. STM32 `PARAM_MAX_SPEED`（0x0001）… **Pi 側が `CONFIG_SET` 未実装で読み書きできない**
 *
 * 3 の値はこのリポジトリからは分からない（ファームウェア側）。上限を上げても実車が
 * そこまで出ないなら、まず 3 を疑うこと。
 */
export type DrivingSettings = {
  /** 見た目の最高速度 [m/s]。既定値 0.6。Pi 側ハード上限（`PI_MAX_SPEED_CAP`）を超えられない */
  maxSpeed: number
  /** 見た目の最大舵角 [rad]。既定値 0.785(45°)。Pi 側ハード上限（`PI_MAX_STEER_CAP`）を超えられない */
  maxSteer: number
  /**
   * 既定（巡航）レンジ。`maxSpeed` に対する倍率。
   * **ブースト側は倍率を持たない** — Shift / R1 を押している間は常に `maxSpeed` そのもの。
   * 「全開が最高速度」でないと、最高速度という名前が何を指すのか分からなくなる。
   */
  cruiseScale: number
  /** 押している間の加速 [m/s²] */
  accel: number
  /** キーを離したときの惰行減速 [m/s²] */
  coast: number
  /** 逆キーを押したときのブレーキ [m/s²]（`accel` より強くすること） */
  brake: number
  /** 発進キック [m/s]。停止から押した瞬間にここまで跳ばす */
  kickSpeed: number
  /** 押している間の切り込み [rad/s] */
  steerRate: number
  /** 離したときのセンター戻り [rad/s]（`steerRate` より速くすること） */
  steerReturn: number
  /** 逆キーでの切り戻し [rad/s] */
  steerCounter: number
  /** ARM を保持したまま無操作でいられる時間 [ms]。超えたら自動 DISARM */
  armIdleTimeoutMs: number
  /** ブレーキの強さ [N·m/輪]。0 は送らない（未指定＝最大制動になってしまう） */
  brakeTorque: number
  /** 駆動をトルク直接指令にするか（v0.6）。設定パネルの永続トグル。true の間、
   * スロットル入力はすべて `target_speed` ではなく `target_torque` になる */
  torqueMode: boolean
  /** トルクモード中にスロットル全開で出す駆動トルク [N·m]。上限 `MAX_TARGET_TORQUE_NM` */
  driveTorque: number
  /** 超音波の自動停止を STM32 に許可するか（v0.7）。**判定も制動も STM32 側で完結する**ので、
   * GUI は `COMMAND.flags` bit7 を立てるかどうかだけを決める。既定 ON */
  autoStop: boolean
}

/** `DrivingSettings` のうち数値項目のキーだけ。スライダのレンジは数値項目にしか無い
 * （`torqueMode` / `autoStop` は boolean なので対象外） */
export type NumericSettingKey = {
  [K in keyof DrivingSettings]: DrivingSettings[K] extends number ? K : never
}[keyof DrivingSettings]

/**
 * io_node 側のハード上限。**GUI からはこれを超えられない**（超えても切り捨てられるだけ）。
 *
 * ⚠ **60° は一度 45° に落とした値を戻したもの**（2026-08-10、指示による）。
 * 大舵角では前輪オドメトリの射影誤差が効き（60° で 1/cos ≒ 2倍）、据え切りを
 * 続けるとステア MD が過熱するので `temp[2]` を見ておくこと。
 *
 * ⚠ 速度は 2.0 → **3.0 m/s ＝ 10.8 km/h** に引き上げた（2026-08-11、指示による）。
 * `DEFAULT_SETTINGS.maxSpeed`（0.6）の5倍で、ミニカーの大きさに対してはかなり速い。
 * **既定値はあえて上げていない**ので、この上限まで使うのは設定パネルで人間が
 * 明示的に上げたときだけになる。
 */
export const PI_MAX_SPEED_CAP = 3.0 // m/s（install_services.sh の --max-speed と一致させてある）
export const PI_MAX_STEER_CAP = 1.047 // rad ≒ 60°（同 --max-steer）

/** STM32 に渡すレートリミット（安全側の保険）。設定パネルの `accel`/`steerRate` はこれより
 * 十分遅く保つこと。**これ自体はユーザー設定にしない**（`useDriving.ts` 参照） */
export const ACCEL_SAFETY_LIMIT = 6.0 // m/s²
export const STEER_RATE_SAFETY_LIMIT = 7.0 // rad/s

/**
 * 後輪1輪あたりの制動トルクの上限 [N·m]。**STM32 の `DRIVE_MAX_BRAKE_TORQUE_NM` と
 * `raspi/msgs/convert.py` の `MAX_BRAKE_TORQUE_NM` に合わせること。**
 * 超えた値を送っても黙って丸められ、「スライダを上げても効きが変わらない」に見える。
 */
export const MAX_BRAKE_TORQUE_NM = 0.125
/** スライダの最小。**0 は「未指定 ＝ 最大制動」を意味するので選ばせない** */
export const MIN_BRAKE_TORQUE_NM = 0.005
/**
 * 制動トルクの既定値。**上限そのもの**にしてある。
 * 「効きすぎて驚く」より「踏んだのに止まらない」方が危険なので、
 * 弱める操作を人間の明示的な意思にする。低速の詰め作業で弱めたいときだけ下げる。
 */
export const DEFAULT_BRAKE_TORQUE_NM = MAX_BRAKE_TORQUE_NM

/**
 * 駆動トルク直接指令の上限 [N·m]（v0.6）。**`raspi/msgs/convert.py` の
 * `MAX_TARGET_TORQUE_NM` と合わせること。** 超えた値を送っても黙って丸められる。
 */
export const MAX_TARGET_TORQUE_NM = 0.125
/**
 * トルクモードの既定の強さ。**上限より控えめにしてある。**
 * ブレーキと違い「強すぎて空転・飛び出す」方が「弱くて動かない」より危険なため、
 * 弱めから始めて設定パネルで上限まで上げてもらう方針（ブレーキとは逆）。
 */
export const DEFAULT_DRIVE_TORQUE_NM = 0.05

/**
 * STM32 が自動停止に入る距離 [m]（v0.7）。**GUI 側では変更できない**（STM32 の固定値）。
 * 表示に使うためだけに持つ。`CONFIG_SET` の param もまだ無い。
 */
export const AUTO_STOP_DISTANCE_M = 0.2
/**
 * 自動停止の既定。**ON にしてある。** 「効きすぎて止まる」より「気づかず当てる」方が
 * 損害が大きいため。⚠ STM32 側にヒステリシスが無いので、20cm 前後では
 * 効いたり切れたりする（チャタリング）。低速で壁に詰める作業では設定パネルで切ること。
 */
export const DEFAULT_AUTO_STOP = true

export const DEFAULT_SETTINGS: DrivingSettings = {
  maxSpeed: 0.6,
  maxSteer: 0.785,
  cruiseScale: 0.55,
  accel: 2.0,
  coast: 2.5,
  brake: 5.0,
  kickSpeed: 0.12,
  steerRate: 2.6,
  steerReturn: 5.2,
  steerCounter: 5.2,
  armIdleTimeoutMs: 20_000,
  brakeTorque: DEFAULT_BRAKE_TORQUE_NM,
  torqueMode: false,
  driveTorque: DEFAULT_DRIVE_TORQUE_NM,
  autoStop: DEFAULT_AUTO_STOP,
}

/** 設定パネルのスライダのレンジ。`min`/`max`/`step` の3つ組。
 * **数値項目だけ。** boolean 項目（`torqueMode`）はスライダを持たないのでここに含めない */
export const SETTINGS_RANGE: Record<NumericSettingKey, { min: number; max: number; step: number }> = {
  maxSpeed: { min: 0.1, max: PI_MAX_SPEED_CAP, step: 0.01 },
  maxSteer: { min: 0.175, max: PI_MAX_STEER_CAP, step: 0.005 }, // 0.175rad ≒ 10°
  cruiseScale: { min: 0.1, max: 1.0, step: 0.05 },
  accel: { min: 0.5, max: 5.5, step: 0.1 },
  coast: { min: 0.5, max: 5.5, step: 0.1 },
  brake: { min: 0.5, max: 5.5, step: 0.1 },
  kickSpeed: { min: 0, max: 0.3, step: 0.01 },
  steerRate: { min: 0.5, max: 6.5, step: 0.1 },
  steerReturn: { min: 0.5, max: 6.5, step: 0.1 },
  steerCounter: { min: 0.5, max: 6.5, step: 0.1 },
  armIdleTimeoutMs: { min: 5_000, max: 60_000, step: 1_000 },
  brakeTorque: { min: MIN_BRAKE_TORQUE_NM, max: MAX_BRAKE_TORQUE_NM, step: 0.001 },
  driveTorque: { min: 0.01, max: MAX_TARGET_TORQUE_NM, step: 0.001 },
}

const SETTINGS_KEY = 'surge.driveSettings.v1'

function clampSettings(s: DrivingSettings): DrivingSettings {
  const out = { ...s }
  for (const k of Object.keys(SETTINGS_RANGE) as NumericSettingKey[]) {
    const { min, max } = SETTINGS_RANGE[k]
    const v = out[k]
    out[k] = typeof v === 'number' && isFinite(v) ? Math.min(max, Math.max(min, v)) : DEFAULT_SETTINGS[k]
  }
  out.torqueMode = typeof out.torqueMode === 'boolean' ? out.torqueMode : DEFAULT_SETTINGS.torqueMode
  // 古い localStorage（v0.6 以前）にはこのキーが無い。**既定の ON に倒す**
  out.autoStop = typeof out.autoStop === 'boolean' ? out.autoStop : DEFAULT_SETTINGS.autoStop
  return out
}

/** 壊れた値・古いスキーマの値で走り出さないよう、読み込み時に必ずクランプする */
function loadSettings(): DrivingSettings {
  try {
    const raw = localStorage.getItem(SETTINGS_KEY)
    if (!raw) return DEFAULT_SETTINGS
    return clampSettings({ ...DEFAULT_SETTINGS, ...JSON.parse(raw) })
  } catch {
    return DEFAULT_SETTINGS
  }
}

function saveSettings(s: DrivingSettings) {
  try {
    localStorage.setItem(SETTINGS_KEY, JSON.stringify(s))
  } catch {
    // 保存できなくても走行は続けられるべきなので無視する
  }
}

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

  /** 異常時のみ出す層(C)を人間が明示的に開いた状態 */
  diagOpen: boolean
  /**
   * ラジコンタブの設定ドロワーが開いているか。
   *
   * **`localStorage` には保存しない。** 開きっぱなしで起動すると層 A（カメラ）が
   * 狭いまま走り出すことになる。開くのは調整したいと思ったときだけでよい。
   */
  settingsOpen: boolean
  /** LiDAR ビューの設定 */
  lidarZoom: number
  lidarFollow: boolean
  /** カメラの進路ガイド。**カメラ校正前なので暫定表示** */
  pathGuide: boolean
  /**
   * ラジコンビューで後方カメラを PIP 表示するか。
   *
   * **切ると `CameraView` ごと外れて WS が閉じる。** 後方の JPEG が流れなくなるので、
   * 前方のフレームレートに回せる（カメラ2台ぶんの帯域が効いている環境向け）。
   */
  rearPip: boolean

  /** ラジコン操作の調整値。設定パネルで変更、`localStorage` に自動保存 */
  settings: DrivingSettings

  // ── 記録・再生（ログタブ） ──
  /** `.sfl` を録ってほしいという意思。null はまだ status を受け取っていない */
  sfl: ControlStatus['sfl'] | null
  /** mcap のライブ中継。Piには保存されない */
  mcap: ControlStatus['mcap'] | null
  /** `logs/` にある `.sfl`/`.mcap` の一覧 */
  logFiles: LogFile[]

  set: (p: Partial<UiState>) => void
  /** 変更分だけ渡せば良い。クランプしてから保存＆反映する */
  setSettings: (p: Partial<DrivingSettings>) => void
  resetSettings: () => void
}

export const useUi = create<UiState>((set, get) => ({
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

  diagOpen: false,
  settingsOpen: false,
  lidarZoom: 4, // 画面半径が何メートルぶんか
  lidarFollow: true,
  pathGuide: true,
  rearPip: true,

  settings: loadSettings(),

  sfl: null,
  mcap: null,
  logFiles: [],

  set: (p) => set(p),
  setSettings: (p) => {
    const next = clampSettings({ ...get().settings, ...p })
    saveSettings(next)
    set({ settings: next })
  },
  resetSettings: () => {
    saveSettings(DEFAULT_SETTINGS)
    set({ settings: DEFAULT_SETTINGS })
  },
}))
