/**
 * イベント駆動の状態だけを持つ store（`architecture.md` §10.4 の3層目）。
 *
 * **20Hz で変わるものはここに入れない。** 接続状態・操縦権・設定の類だけ。
 */
import { create } from 'zustand'
import { live } from '../bus/live'
import type { AutoStatus, CameraConfigStatus, ControlStatus, FanStatus, LogFile } from '../types'
import { VEHICLE } from '../generated/vehicle'

export type InputSource = 'none' | 'keyboard' | 'gamepad' | 'slider' | 'auto'

/**
 * ラジコン操作の調整値。**設定パネルから変更でき、`localStorage` に保存される。**
 *
 * ⚠ `maxSpeed` / `maxSteer` のスライダ上限は `effectiveRange()` が動的に決める
 * （★v0.11。STM32/仮想STM32の `LIMITS` を受信済みならそれを使う。§`effectiveRange` 参照）。
 * 超えた分は `io_node` の `_send_command`（`--max-speed`/`--max-steer` と `LIMITS` の
 * 小さい方でクランプ）で黙って切り捨てられるので、GUI 側のレンジが実際の上限より
 * 緩いと「表示上は出せるはずなのに実際は出ない」食い違いが起きる
 * （`architecture.md` §10.5）。**GUI 側の `--max-speed` 相当の運用上限
 * （`raspi/setup/install_services.sh` の deploy 引数）を上げ忘れていないか確認すること。**
 *
 * ⚠ **速度の上限は今も2段ある。**
 *
 *   1. io_node `--max-speed` …………… Pi 側の運用上限。超えた分を黙って切り捨てる
 *   2. STM32 の物理的な上限 …………… ★v0.11 `LIMITS` で Pi/GUI が取得できるようになった
 *
 * 以前は 2 がこのリポジトリから分からず、GUI 側の `PI_MAX_SPEED_CAP` を手で
 * 静的に書いていたが、**今は `LIMITS` を受信済みならそちらを使う**（`effectiveRange`）。
 */
export type DrivingSettings = {
  /** RC の速度ダイヤル（速度制御モードの実際の上限）[m/s]。既定値 3.0（2026-08-22、
   * それまで手で使っていた値に更新）。範囲は `LIMITS` 受信済みならそちらが上限
   * （`effectiveRange`）。**速度メータの目盛りはこれとは独立**（`SpeedGauge.tsx` 参照、
   * 常に `LIMITS.max_speed_m_s` 固定） */
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
  /**
   * 前カメラの進路ガイド（`CameraView.tsx` の `drawGuide`）が使う取付高さ [m]。
   * 既定値は `config/vehicle.toml` の `sensors.cam_front.z`（実測済みのセンサ位置）だが、
   * レンズ光学中心とはズレがありうるため、実写を見ながらここで追い込む前提の値。
   *
   * ⚠ 俯角（取付角度）はここに無い。**一度ネジ止めしたら変わらない物理値**なので
   * `vehicle.toml` の `sensors.cam_front.pitch` を直接使う（`VEHICLE.camFront.pitch`）。
   * 高さだけスライダを残すのは、こちらはレンズ光学中心と `z` の実測点がズレうる
   * （定規で測れるのは筐体の位置で、レンズ内部の光学中心そのものではない）ため。
   */
  camHeight: number
}

/** `DrivingSettings` のうち数値項目のキーだけ。スライダのレンジは数値項目にしか無い
 * （`torqueMode` / `autoStop` は boolean なので対象外） */
export type NumericSettingKey = {
  [K in keyof DrivingSettings]: DrivingSettings[K] extends number ? K : never
}[keyof DrivingSettings]

/**
 * **`LIMITS` 未受信の間だけ使う起動時プレースホルダ。** WS 接続前や STM32/仮想STM32が
 * まだ `LIMITS` を返していない一瞬だけこの値でスライダを描き、届き次第
 * {@link effectiveRange} が実測値に置き換える（★v0.11。届いた後はこの値は使われない
 * ——`SETTINGS_RANGE` の「小さい方」ではなく実測値をそのまま採用する）。
 *
 * ⚠ 60°（1.047 rad）は一度 45° に落とした値を戻したもの（2026-08-10、指示による）が、
 * これは**モータ機械角の可動域**であって路面舵角ではなかった。リンク比 0.5
 * （2026-08-20 実測確定）によりタイヤは半分しか切れないため、路面舵角の上限は
 * **30°（0.524 rad）**に修正した。据え切りを続けるとステア MD が過熱するので
 * `temp[2]` を見ておくこと。**`PI_MAX_STEER_CAP` は手で書かず `config/vehicle.toml` の
 * `max_steer` から生成する**（`config/generate.py`）。値を直すには toml を直して
 * 再生成すること。
 *
 * ⚠ 速度は 2.0 → **3.0 m/s ＝ 10.8 km/h** に引き上げた（2026-08-11、指示による）。
 * **`DEFAULT_SETTINGS.maxSpeed` も 2026-08-22 に 0.6 → 3.0（当時この上限いっぱいで
 * 使っていた値）へ更新した**ので、この2つは今は同じ 3.0 m/s。速度は `vehicle.toml`
 * に持たせる値ではない（運用上の速度制限であって車両の物理諸元ではない）ため、
 * こちらは引き続き直書き。
 *
 * ⚠ **速度の上限は今も2段ある。** `effectiveRange` はスライダの見た目の話でしかなく、
 * 実際にモータへ効く安全側のクランプは別（`io_node._send_command` 参照）。
 *
 *   1. io_node `--max-speed` …………… Pi 側の運用上限。超えた分を黙って切り捨てる
 *   2. STM32 の物理的な上限 …………… ★v0.11 `LIMITS` パケットで Pi/GUI が取得できる
 *      ようになった。以前はこのリポジトリから分からなかった
 */
export const PI_MAX_SPEED_CAP = 3.0 // m/s（install_services.sh の --max-speed と一致させてある）
export const PI_MAX_STEER_CAP: number = VEHICLE.maxSteer // rad ≒ 30°（同 --max-steer。路面舵角の実機上限）

/** STM32 に渡すレートリミット（安全側の保険）。設定パネルの `accel`/`steerRate` はこれより
 * 十分遅く保つこと。**これ自体はユーザー設定にしない**（`useDriving.ts` 参照） */
export const ACCEL_SAFETY_LIMIT = 6.0 // m/s²
export const STEER_RATE_SAFETY_LIMIT = 7.0 // rad/s

/**
 * 後輪1輪あたりの制動トルクの上限 [N·m]。**STM32 の `DRIVE_MAX_BRAKE_TORQUE_NM` と
 * `raspi/msgs/convert.py` の `MAX_BRAKE_TORQUE_NM` に合わせること。**
 * 超えた値を送っても黙って丸められ、「スライダを上げても効きが変わらない」に見える。
 */
export const MAX_BRAKE_TORQUE_NM = 0.15
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
export const MAX_TARGET_TORQUE_NM = 0.15
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
  maxSpeed: 3.0,
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
  camHeight: VEHICLE.camFront.height,
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
  camHeight: { min: 0.03, max: 0.25, step: 0.005 },
}

export type SettingRange = { min: number; max: number; step: number }

/**
 * 車両の物理的な上限値（`LIMITS` パケット由来。★v0.11）。`LinkDiag`（`/ws/telemetry`
 * 8Hz、`bus/live.ts`）から読む値の形。まだ受け取っていなければ全フィールド null
 * （`ControlStatus`＝`/ws/control` の status には乗せない。`tc_enabled` 等と同じ理由で
 * status はイベント発生時にしかbroadcastされないため、リロードするまで反映されずに
 * 見えるバグを避ける。`components/SettingsPanel.tsx` 参照）
 */
export type VehicleLimits = {
  max_speed_m_s: number | null
  max_accel_m_s2: number | null
  max_torque_nm: number | null
  max_steer_rad: number | null
}

/**
 * `SETTINGS_RANGE` に STM32（または `sim/stm32.py` の仮想STM32）実測の `LIMITS`
 * （★v0.11）を重ねた、**実際にスライダへ使うべきレンジ**。
 *
 * **`LIMITS` を受信済みならその値を無条件に使う（`SETTINGS_RANGE` の静的値より優先）。**
 * `SETTINGS_RANGE` はあくまで**未接続でまだ `LIMITS` を受け取っていない間だけ使う
 * 起動時のプレースホルダ**であって、車両の実際の上限のつもりで書いた値ではない
 * （実機・シムどちらも `LIMITS`/`LIMITS_REQ` に対応しており、接続していれば
 * 数百ms 以内に本物の値へ置き換わる）。
 *
 * ⚠ これはスライダの**表示・入力レンジ**の話であって、安全側のクランプではない。
 * 実際にモータへ効く上限は `raspi/nodes/io_node.py` の `_send_command` が
 * （`--max-speed`/`--max-steer` という Pi 側の運用上限と）別に持っており、
 * そちらは意図的に「小さい方を使う」多層防御のまま変えていない。
 *
 * - `maxSpeed` ← `max_speed_m_s`
 * - `maxSteer` ← `max_steer_rad`
 * - `accel`/`coast`/`brake` ← `max_accel_m_s2`（GUI 側のランプ速度。実際の `COMMAND.accel_limit`
 *   はこれとは別に固定値 `ACCEL_SAFETY_LIMIT` を送っているが、STM32 側でどのみち
 *   `max_accel_m_s2` に丸められるため、ここより上に振ってもスライダが「上げても
 *   効かない」ものになるだけ。丸められる前に GUI 側で上限を揃えておく）
 * - `brakeTorque`/`driveTorque` ← `max_torque_nm`
 */
export function effectiveRange(limits: VehicleLimits | null | undefined): Record<NumericSettingKey, SettingRange> {
  const withLive = (r: SettingRange, live: number | null | undefined): SettingRange =>
    typeof live === 'number' && isFinite(live) && live > 0 ? { ...r, max: live } : r
  const accelCap = withLive(SETTINGS_RANGE.accel, limits?.max_accel_m_s2)
  const torqueCap = withLive(SETTINGS_RANGE.brakeTorque, limits?.max_torque_nm)
  return {
    ...SETTINGS_RANGE,
    maxSpeed: withLive(SETTINGS_RANGE.maxSpeed, limits?.max_speed_m_s),
    maxSteer: withLive(SETTINGS_RANGE.maxSteer, limits?.max_steer_rad),
    accel: accelCap,
    coast: { ...accelCap, min: SETTINGS_RANGE.coast.min },
    brake: { ...accelCap, min: SETTINGS_RANGE.brake.min },
    brakeTorque: torqueCap,
    driveTorque: withLive(SETTINGS_RANGE.driveTorque, limits?.max_torque_nm),
  }
}

const SETTINGS_KEY = 'surge.driveSettings.v1'

function clampSettings(s: DrivingSettings): DrivingSettings {
  // **`SETTINGS_RANGE`（静的）ではなく `effectiveRange`（`LIMITS` 受信済みならそちら
  // 優先）で必ずクランプすること。** ここを静的値のままにすると、スライダの DOM 要素
  // 自体は動的な `max` まで動くのに、`setSettings` を呼んだ瞬間ここで静的値まで
  // 巻き戻され「ドラッグしても弾かれる」バグになる（2026-08-22、実機で発覚）。
  // `live` はフックではない素の可変オブジェクトなので、コンポーネント外のここからでも読める
  const range = effectiveRange(live.link)
  const out = { ...s }
  for (const k of Object.keys(range) as NumericSettingKey[]) {
    const { min, max } = range[k]
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
 * 「既定値に戻す」ボタン・各行の「既定値 X」ボタンが指す先。**`DEFAULT_SETTINGS`
 * （ソースコードに焼き込まれた工場出荷値）とは別に、`localStorage` に持つ。
 *
 * 2026-08-22: 「今使っている値を既定値にしたい」という要望への対応。以前は
 * `DEFAULT_SETTINGS` をソースコードで直接書き換えるしかなく、**Claude が実機の
 * 現在値を見られないまま数値を推測して書く**という壊れ方をした（スクリーンショットの
 * 値が古くなっていた）。設定パネルの「現在の値を既定値として保存」ボタン
 * （`saveCurrentAsDefault`）で、ユーザー自身がいつでも更新できるようにする。
 */
const DEFAULT_OVERRIDE_KEY = 'surge.driveSettingsDefault.v1'

/** 保存されていなければソースコードの `DEFAULT_SETTINGS`（工場出荷値）を返す。 */
function loadDefaultSettings(): DrivingSettings {
  try {
    const raw = localStorage.getItem(DEFAULT_OVERRIDE_KEY)
    if (!raw) return DEFAULT_SETTINGS
    return clampSettings({ ...DEFAULT_SETTINGS, ...JSON.parse(raw) })
  } catch {
    return DEFAULT_SETTINGS
  }
}

function saveDefaultSettings(s: DrivingSettings) {
  try {
    localStorage.setItem(DEFAULT_OVERRIDE_KEY, JSON.stringify(s))
  } catch {
    // 同上、保存できなくても走行は続けられるべき
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
  /** `/ws/telemetry` の型定義の札が GUI 側と食い違っているか。
   *
   * **true の間はテレメトリを一切描かない**（`ws/telemetry.ts` がフレームを
   * 捨てている）。GUI を再ビルドするまで直らないので、切断とは別に出す——
   * 「切断」と表示すると Wi-Fi を疑って時間を溶かす。 */
  schemaMismatch: boolean
  controlOpen: boolean
  /** `/ws/map` が繋がっているか。**地図が更新されないのが「切れている」のか
   *  「凍結して変わらない」のかを区別するために要る** */
  mapOpen: boolean
  status: ControlStatus | null
  /** 自分が操縦権を持っているか */
  hasControl: boolean
  deniedBy: string | null
  /** 操縦権を取れなかった理由。**`bad_token` は「誰かが持っている」ではなく
   *  「トークンが違う」** なので、表示を分けないと原因を探して時間を溶かす */
  deniedReason: string | null
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
  /** 直前に自律走行が解除された理由。同上（engage したときに消す） */
  autoOffReason: string

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
   * ラジコンビューで左下に PIP（今メインに出ていない方のカメラ）を表示するか。
   *
   * **切ると PIP 側の `CameraView` が外れて WS が閉じる。** 流れなくなった分、
   * メイン映像のフレームレートに回せる（カメラ2台ぶんの帯域が効いている環境向け）。
   *
   * 2026-08-17: 個別ボタンは廃止し、メイン映像の空いた場所のクリックで
   * `lidarVisible`/`pathGuide` と一緒に切り替える方式にした。しかし
   * **2026-08-20 に外した**——LiDAR を消したいだけのクリックで小さい映像
   * まで一緒に消えるのが紛らわしいと指摘されたため。今は常時 true のまま
   * （切り替える操作が無い）。PIP 自体のクリックは表示/非表示ではなく
   * 「メインと入れ替え」（`RcView.tsx` の `mainCam`）。
   */
  rearPip: boolean
  /** ラジコンビューで LiDAR ミニマップ（映像右上の丸）を出すか。メイン映像クリックで `pathGuide` と連動する
   * （2026-08-20: `rearPip` はこの連動から外した） */
  lidarVisible: boolean
  /** LiDAR ミニマップを拡大表示中か。false＝映像の1/3高さ、true＝82%高さ。ミニマップ自体のクリックで切り替える */
  lidarExpanded: boolean

  /** ラジコン操作の調整値。設定パネルで変更、`localStorage` に自動保存 */
  settings: DrivingSettings
  /** 「既定値に戻す」の戻り先。既定はソースコードの `DEFAULT_SETTINGS` だが、
   * `saveCurrentAsDefault` で `settings` の現在値に上書きできる（★2026-08-22） */
  settingsDefault: DrivingSettings

  // ── 記録・再生（ログタブ） ──
  /** `.sfl` を録ってほしいという意思。null はまだ status を受け取っていない */
  sfl: ControlStatus['sfl'] | null
  /** mcap のライブ中継。Piには保存されない */
  mcap: ControlStatus['mcap'] | null
  /** `logs/` にある `.sfl`/`.mcap` の一覧 */
  logFiles: LogFile[]

  /**
   * 自動運転の意思と、選べるモードの宣言。**サーバが真値なのでここでは編集しない。**
   *
   * 押した結果は `status` のブロードキャストで返ってくる（`ws/control.ts` の
   * `setAuto`）。GUI 側に「engage したつもり」の状態を持つと、2枚目のタブや
   * サーバ側の拒否（モード未選択・E-Stop）と食い違う。
   */
  auto: AutoStatus | null

  /**
   * Pi5純正ファンの意思。**サーバが真値なのでここでは編集しない。**
   * 押した結果は `status` のブロードキャストで返ってくる（`ws/control.ts` の `setFan`）。
   */
  fan: FanStatus | null

  /**
   * capture側(camera_node)のFPS上限・後方カメラON/OFF・GUI配信頻度。
   * **サーバが真値なのでここでは編集しない。** 押した結果は `status` の
   * ブロードキャストで返ってくる（`ws/control.ts` の `setCamera`）。
   */
  cameraConfig: CameraConfigStatus | null

  set: (p: Partial<UiState>) => void
  /** 変更分だけ渡せば良い。クランプしてから保存＆反映する */
  setSettings: (p: Partial<DrivingSettings>) => void
  /** `settingsDefault`（＝「既定値に戻す」の戻り先）へ戻す。値そのものは変えない */
  resetSettings: () => void
  /** 今の `settings` を新しい既定値として保存する（★2026-08-22）。
   * 以後の「既定値に戻す」・各行の「既定値 X」表示はこの値を指す */
  saveCurrentAsDefault: () => void
}

export const useUi = create<UiState>((set, get) => ({
  mapOpen: false,
  telemetryOpen: false,
  schemaMismatch: false,
  controlOpen: false,
  status: null,
  hasControl: false,
  deniedBy: null,
  deniedReason: null,
  wsRttMs: null,

  armRequested: false,
  deadman: false,
  inputSource: 'none',
  boost: false,
  disarmReason: '',
  autoOffReason: '',

  braking: false,
  horning: false,
  passing: false,
  lightMode: LIGHT_OFF,

  settingsOpen: false,
  lidarZoom: 4, // 画面半径が何メートルぶんか
  lidarFollow: true,
  pathGuide: true,
  rearPip: true,
  lidarVisible: true,
  lidarExpanded: false,

  settings: loadSettings(),
  settingsDefault: loadDefaultSettings(),

  sfl: null,
  mcap: null,
  logFiles: [],
  auto: null,
  fan: null,
  cameraConfig: null,

  set: (p) => set(p),
  setSettings: (p) => {
    const next = clampSettings({ ...get().settings, ...p })
    saveSettings(next)
    set({ settings: next })
  },
  resetSettings: () => {
    const def = get().settingsDefault
    saveSettings(def)
    set({ settings: def })
  },
  saveCurrentAsDefault: () => {
    const cur = get().settings
    saveDefaultSettings(cur)
    set({ settingsDefault: cur })
  },
}))
