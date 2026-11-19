/**
 * 運転設定（`DrivingSettings`）の型・値・計算・保存。`store/ui.ts` から分けた。
 *
 * - 型と既定値（`DrivingSettings`・`DEFAULT_SETTINGS`）、スライダのレンジ（`SETTINGS_RANGE`・`effectiveRange`）
 * - 安全系の上限（`MAX_BRAKE_TORQUE_NM` 等。数値は `generated/vehicle.ts` の生成値で、正は Pi 側）
 * - クランプ（`clampSettings`）、自動運転パラメータの上限（`capAutoParams`）
 * - 旧版の補正（`migrateSettings`）と、`persist` 導入前の旧キーの読み込み・読み込み時の直し（`restoreSettings`）
 *
 * **このモジュールは `store/ui.ts` を import しない。** ストアは作る瞬間（保存値の読み込み）にここの
 * 関数と値を使うので、互いに import し合うと読み込み順によって初期化前の値を読んで落ちる（vitest で再現）。
 */
import type { EngineSoundType } from '../audio/engineSound'
import { live } from '../bus/live'
import { DRIVE_FALLBACK, SAFETY, VEHICLE } from '../generated/vehicle'
import type { AutoParamSpec } from '../types'

/**
 * ラジコン操作の調整値。**設定パネルから変更でき、`localStorage` に保存される。**
 *
 * ⚠ `maxSpeed` / `maxSteer` のスライダ上限は `effectiveRange()` が動的に決める
 * （STM32/仮想STM32の `LIMITS` を受信済みならそれを使う。§`effectiveRange` 参照）。
 * 超えた分は `io_node` の `_send_command`（`--max-speed`/`--max-steer` と `LIMITS` の
 * 小さい方でクランプ）で黙って切り捨てられるので、GUI 側のレンジが実際の上限より
 * 緩いと「表示上は出せるはずなのに実際は出ない」食い違いが起きる
 * （`system_overview.md` §10.5）。**GUI 側の `--max-speed` 相当の運用上限
 * （`raspi/setup/install_services.sh` の deploy 引数）を上げ忘れていないか確認すること。**
 *
 * ⚠ **実際に効く上限は `LIMITS`（STM32 の物理的な上限）。** io_node は `LIMITS` を受け取った後は
 * それだけでクランプし、`--max-speed`/`--max-steer` は受け取るまでの仮の上限でしかない。GUI も同じで、
 * `LIMITS` を受信済みならそちらを使う（`effectiveRange`）。`PI_MAX_SPEED_CAP` は未受信時の仮値。
 */
export type DrivingSettings = {
  /** RC の速度ダイヤル（速度制御モードの実際の上限）[m/s]。既定値 3.0。範囲は `LIMITS` 受信済みならそちらが上限
   * （`effectiveRange`）。**速度メータの目盛りはこれとは独立**（`SpeedGauge.tsx` 参照、
   * 常に `LIMITS.max_speed_m_s` 固定）。
   *
   * 常に `maxSpeed` そのものが上限（「全開が最高速度」。巡航レンジやブーストは無い）。
   */
  maxSpeed: number
  /** 手動操作で切る最大の舵角 [rad]（路面舵角）。既定値は車両の上限（`PI_MAX_STEER_CAP`）で、それを超えられない */
  maxSteer: number
  /** 押している間の加速 [m/s²] */
  accel: number
  /** キーを離したときの惰行減速 [m/s²] */
  coast: number
  /** 逆キーを押したときのブレーキ [m/s²]（`accel` より強くすること）。**MT では使わない**
   * （MT に逆キーブレーキは無い——ギアと逆のキーは何もしない、`useDriving.ts` 参照） */
  brake: number
  /** 押している間の切り込み [rad/s] */
  steerRate: number
  /** 離したときのセンター戻り [rad/s]（`steerRate` より速くすること） */
  steerReturn: number
  /** 逆キーでの切り戻し [rad/s] */
  steerCounter: number
  /**
   * ゲームパッド左スティックの舵カーブ（EXPO）[0-1]。**パッドの舵にだけ効く**
   * （キーボードは `steerRate`/`steerReturn`/`steerCounter` のランプ方式なので無関係）。
   * 0=リニア、1=完全3乗で中央付近をいちばん緩める。中央を大きく緩めるほど
   * 直進・小舵角での細かい操作がしやすくなる代わり、フルロックまでの
   * ストローク後半が敏感になる。既定 0.45
   */
  gamepadSteerExpo: number
  /**
   * ゲームパッド左スティックのデッドゾーン（中央の遊び）[0-1]。**パッドの舵にだけ効く。**
   * `dz()`（`useDriving.ts`）で切り落とすのではなく再スケールするので、ここを広げても
   * 「動き始めで飛ぶ」段差は出ない。既定 0.12
   */
  gamepadSteerDeadzone: number
  /** ARM を保持したまま無操作でいられる時間 [ms]。超えたら自動 DISARM */
  armIdleTimeoutMs: number
  /** ブレーキの強さ [N·m/輪]。0 は送らない（未指定＝最大制動になってしまう） */
  brakeTorque: number
  /**
   * 制御方式（速度 / トルク / MT の3択）。
   * 'speed' ＝ 速度制御（`maxSpeed` が上限）。
   * 'torque' ＝ トルク直接指令（スロットル入力はすべて `target_speed` ではなく `target_torque`）。
   * 'mt' ＝ 擬似変速（速度制御の一種。上限は `maxSpeed` ではなく専用の `mtMaxSpeed` に
   * `mtGear1`〜`mtGear5` の倍率を掛けたもの。ワイヤプロトコルの `torque_mode` ビットは
   * 'speed' と同じく false のまま——STM32 からは普通の速度指令にしか見えない）。
   *
   * ⚠ **MT は 'speed' と一切パラメータを共有しない**（共有すると 'speed' の乗り味の
   * 調整が MT に波及する）。MT 専用の `mtMaxSpeed`/`mtAccel`/`mtCoast`/`mtEngineBrake` を
   * 独立に持つ（下記）。`brakeTorque`（実ブレーキの強さ、STM32 の N·m）だけは車両物理量
   * なので共有する。
   */
  driveMode: 'speed' | 'torque' | 'mt'
  /** トルクモード中にスロットル全開で出す駆動トルク [N·m]。上限 `MAX_TARGET_TORQUE_NM` */
  driveTorque: number
  /**
   * MT モード専用の最高速度 [m/s]。5速（倍率100%）で
   * この値そのものが上限になる。'speed' モードの `maxSpeed` とは完全に独立——
   * どちらかを変えてももう一方には影響しない。
   */
  mtMaxSpeed: number
  /** MT モード専用の加速度 [m/s²]。押している間／
   * トリガーを踏み込んだ方向へ近づくときのレート。'speed' モードの `accel` とは独立 */
  mtAccel: number
  /**
   * MT（擬似変速）モードの D1〜D5 それぞれの上限速度 [`mtMaxSpeed` に対する倍率、0-1]。
   * R はここでは持たない——`mtGear1`（D1 と同じ）を流用する。後退はどのみち
   * 低速域でしか使わないので専用の刻みを持つ必要が薄い。
   * `SETTINGS_RANGE`/`effectiveRange` は他の項目と違い `LIMITS` で動かさない
   * （絶対速度ではなく倍率なので、`mtMaxSpeed` 側がすでに `LIMITS` に追従している）
   */
  mtGear1: number
  mtGear2: number
  mtGear3: number
  mtGear4: number
  mtGear5: number
  /**
   * MT モードでアクセルを離したとき、または現在のギアと逆のキー（実車に無い
   * 「逆ペダル」相当、`useDriving.ts` 参照）を押したときの減速度 [m/s²]
   * （実車の惰行＝慣性走行に相当）。STM32 からは緩やかに下がっていく `target_speed`
   * にしか見えない（速度PIがそれを追従するだけ）。ニュートラル（N）でもこの値が使われる
   * （エンジンブレーキが乗らない＝いちばん緩い減速というのが実車の感覚に近いため）。
   */
  mtCoast: number
  /**
   * MT モードでギアが下がるほど加算されるエンジンブレーキ相当の減速度 [m/s²]。
   * 実効減速度は `mtCoast + mtEngineBrake * (1 - mtGearRatio(gear))` — 5速
   * （比率1.0）では加算なしで `mtCoast` のみ、1速（比率0.2）ではほぼ全量が乗る。
   * ギアごとに個別の値は持たない（既存の `mtGear1`〜`mtGear5` の比率をそのまま
   * 重みに流用し、設定項目を増やしすぎないため）。シフトダウンでこの上限を
   * 超えたときも、瞬間移動ではなくこのレートでなだらかに新しい上限まで落ちる
   * （`useDriving.ts` の `tick()` 参照）。N では使われない（`mtCoast` 参照）。
   */
  mtEngineBrake: number
  /** 超音波の自動停止を STM32 に許可するか（v0.7）。**判定も制動も STM32 側で完結する**ので、
   * GUI は `COMMAND.flags` bit7 を立てるかどうかだけを決める。既定 ON */
  autoStop: boolean
  /** 手動操作のローンチコントロール（★プロトコル v0.20）を使うか。止まった状態でブレーキとアクセルを
   * 両方踏むと準備、ブレーキを離すと発進——目標速度をいきなり上限まで上げ、`COMMAND.flags2` の
   * `launch` を立てる（STM32 がランプと速度PIを迂回して全開＋TC で加速する）。トルク制御では
   * 設定の駆動トルクに関わらず最大トルクを出す。手順は `useDriving.ts` */
  launchControl: boolean
  /** ラジコンの速度計（`SpeedGauge.tsx`）の表示単位。既定 'ms'。クリックで切替、`localStorage` に保存 */
  speedUnit: 'ms' | 'kmh'
  /**
   * 合成エンジン音（`audio/engineSound.ts`）の音色。
   * 'combustion' ＝ 内燃機関風（ノコギリ波の唸り）、'ev' ＝ EV/ハイブリッド風
   * （澄んだトーン＋高い倍音のシマー）。鳴らすかどうか自体は `UiState.engineSoundOn`
   * （こちらは意図的に非永続——起動直後に前回 ON のまま音が鳴り出すと驚くため）で、
   * こちらは「鳴らすときにどの音色か」という嗜好なので通常どおり永続化する。
   */
  engineSoundType: EngineSoundType
}

/** `DrivingSettings` のうち数値項目のキーだけ。スライダのレンジは数値項目にしか無い
 * （`driveMode` / `autoStop` は boolean/enum なので対象外） */
export type NumericSettingKey = {
  [K in keyof DrivingSettings]: DrivingSettings[K] extends number ? K : never
}[keyof DrivingSettings]

/**
 * **`LIMITS` 未受信の間だけ使う起動時プレースホルダ。** WS 接続前や STM32/仮想STM32が
 * まだ `LIMITS` を返していない一瞬だけこの値でスライダを描き、届き次第
 * {@link effectiveRange} が実測値に置き換える（届いた後はこの値は使われない
 * ——`SETTINGS_RANGE` の「小さい方」ではなく実測値をそのまま採用する）。
 *
 * ⚠ 舵角の上限は**路面舵角**（`config/vehicle.toml` の `max_steer`）。モータ機械角の可動域では
 * ない（STM32 がリンクの換算を掛ける）。据え切りを続けるとステア MD が過熱するので `temp[2]` を
 * 見ておくこと。**`PI_MAX_STEER_CAP` は手で書かず `max_steer` から生成する**（`config/generate.py`）。
 * 値を直すには toml を直して再生成すること。
 *
 * ⚠ 速度の仮値は **3.0 m/s**（`DEFAULT_SETTINGS.maxSpeed` と同じ）。`LIMITS` の上限はこれより
 * 高いことがある。**保存してある最高速度をこの仮値で切り詰めてはいけない**——`LIMITS` が届く前に
 * 読み込んだ値は上を切らずに持ち、届いた時点で `LIMITS` でクランプする（`clampSettings`）。
 *
 * ⚠ `effectiveRange` はスライダの見た目の話でしかなく、実際にモータへ効く安全側のクランプは
 * io_node（`_send_command`。`LIMITS` を受け取るまでは `--max-speed`/`--max-steer`）と STM32 が持つ。
 */
// 数値は手で書かない。`config/generate.py` が `raspi/msgs/convert.py`（正）から `DRIVE_FALLBACK` を生成する
export const PI_MAX_SPEED_CAP: number = DRIVE_FALLBACK.maxSpeedMS // m/s（io_node の --max-speed と同じ値）
export const PI_MAX_STEER_CAP: number = VEHICLE.maxSteer // rad（同 --max-steer。路面舵角の実機上限）

/** STM32 に渡すレートリミット（安全側の保険）。設定パネルの `accel`/`steerRate` はこれより
 * 十分遅く保つこと。**これ自体はユーザー設定にしない**（`useDriving.ts` 参照）。
 * 値は `config/vehicle.toml` の `[safety]`（シムも同じ値で実機の舵のランプを再現する） */
export const ACCEL_SAFETY_LIMIT: number = SAFETY.cmdAccelLimit // m/s²
export const STEER_RATE_SAFETY_LIMIT: number = SAFETY.cmdSteerRateLimit // rad/s

/**
 * 後輪1輪あたりの制動トルクの上限 [N·m]（STM32 の `DRIVE_MAX_BRAKE_TORQUE_NM`）。
 * **値は `raspi/msgs/convert.py` の `MAX_BRAKE_TORQUE_NM` から生成する**（`generated/vehicle.ts` の `DRIVE_FALLBACK`）。
 * 超えた値を送っても黙って丸められ、「スライダを上げても効きが変わらない」に見える。
 */
export const MAX_BRAKE_TORQUE_NM: number = DRIVE_FALLBACK.maxBrakeTorqueNm
/** スライダの最小。**0 は「未指定 ＝ 最大制動」を意味するので選ばせない** */
const MIN_BRAKE_TORQUE_NM = 0.005
/**
 * 制動トルクの既定値。**上限そのもの**にしてある。
 * 「効きすぎて驚く」より「踏んだのに止まらない」方が危険なので、
 * 弱める操作を人間の明示的な意思にする。低速の詰め作業で弱めたいときだけ下げる。
 */
const DEFAULT_BRAKE_TORQUE_NM = MAX_BRAKE_TORQUE_NM

/**
 * 駆動トルク直接指令の上限 [N·m]（v0.6）。**値は `raspi/msgs/convert.py` の
 * `MAX_TARGET_TORQUE_NM` から生成する**（`DRIVE_FALLBACK`）。超えた値を送っても黙って丸められる。
 */
export const MAX_TARGET_TORQUE_NM: number = DRIVE_FALLBACK.maxTargetTorqueNm
/**
 * トルクモードの既定の強さ。**上限より控えめにしてある。**
 * ブレーキと違い「強すぎて空転・飛び出す」方が「弱くて動かない」より危険なため、
 * 弱めから始めて設定パネルで上限まで上げてもらう方針（ブレーキとは逆）。
 */
const DEFAULT_DRIVE_TORQUE_NM = 0.05

/**
 * 自動停止の既定。**ON にしてある。** 「効きすぎて止まる」より「気づかず当てる」方が
 * 損害が大きいため。⚠ STM32 側にヒステリシスが無いので、20cm 前後では
 * 効いたり切れたりする（チャタリング）。低速で壁に詰める作業では設定パネルで切ること。
 */
const DEFAULT_AUTO_STOP = true
/**
 * ローンチコントロール（手動操作）の既定。**ON にしてある**——働くのは「止まった状態でブレーキと
 * アクセルを両方踏み、ブレーキを離す」という手順を踏んだときだけで、普通に走り出すぶんには何も変わらない。
 */
const DEFAULT_LAUNCH_CONTROL = true
/** 手動のローンチコントロールの状態（`useDriving.ts`）。ready = ブレーキ＋アクセルで準備中、go = 発進中 */
export type LaunchPhase = 'idle' | 'ready' | 'go'

export const DEFAULT_SETTINGS: DrivingSettings = {
  maxSpeed: 3.0,
  // 車両の上限そのもの（路面舵角）。これより大きい値は io_node が切り捨てるだけで、
  // スティックの後半・キーの切り終わりが不感帯になる
  maxSteer: PI_MAX_STEER_CAP,
  accel: 2.0,
  coast: 2.5,
  brake: 5.0,
  steerRate: 2.6,
  steerReturn: 5.2,
  steerCounter: 5.2,
  gamepadSteerExpo: 0.45,
  gamepadSteerDeadzone: 0.12,
  armIdleTimeoutMs: 20_000,
  brakeTorque: DEFAULT_BRAKE_TORQUE_NM,
  driveMode: 'speed',
  driveTorque: DEFAULT_DRIVE_TORQUE_NM,
  // 'speed' の maxSpeed/accel とは独立。まずは同じ値から始め、
  // 実車に乗ってから設定パネルで別々に詰めること
  mtMaxSpeed: 3.0,
  mtAccel: 2.0,
  // 1速ごとに最高速度の20%ずつ刻む素直な等差。詰めたい場合は設定パネルから
  mtGear1: 0.2,
  mtGear2: 0.4,
  mtGear3: 0.6,
  mtGear4: 0.8,
  mtGear5: 1.0,
  // 5速/Nでは0.8のみ、1速では0.8+3.0*0.8=3.2 m/s² 相当のエンジンブレーキになる暫定値。
  // 実車に乗ってから設定パネルで詰めること
  mtCoast: 0.8,
  mtEngineBrake: 3.0,
  autoStop: DEFAULT_AUTO_STOP,
  launchControl: DEFAULT_LAUNCH_CONTROL,
  speedUnit: 'ms',
  engineSoundType: 'combustion',
}

/** 設定パネルのスライダのレンジ。`min`/`max`/`step` の3つ組。
 * ⚠ **ここの範囲を変えたら、Pi 側の `telemetry_node._DRIVE_SETTINGS_RANGE`（保存時の天井・床）も
 * 合わせること。** あちらが狭いと、GUI で出せる値が保存時に切り詰められて次の接続で別の値に戻る
 * **数値項目だけ。** boolean/enum 項目（`driveMode`/`autoStop`）はスライダを持たないのでここに含めない */
export const SETTINGS_RANGE: Record<NumericSettingKey, { min: number; max: number; step: number }> = {
  maxSpeed: { min: 0.1, max: PI_MAX_SPEED_CAP, step: 0.01 },
  maxSteer: { min: 0.175, max: PI_MAX_STEER_CAP, step: 0.005 }, // 0.175rad ≒ 10°
  accel: { min: 0.5, max: 5.5, step: 0.1 },
  coast: { min: 0.5, max: 5.5, step: 0.1 },
  brake: { min: 0.5, max: 5.5, step: 0.1 },
  steerRate: { min: 0.5, max: 6.5, step: 0.1 },
  steerReturn: { min: 0.5, max: 6.5, step: 0.1 },
  steerCounter: { min: 0.5, max: 6.5, step: 0.1 },
  gamepadSteerExpo: { min: 0, max: 1, step: 0.05 },
  gamepadSteerDeadzone: { min: 0, max: 0.3, step: 0.01 },
  armIdleTimeoutMs: { min: 5_000, max: 60_000, step: 1_000 },
  brakeTorque: { min: MIN_BRAKE_TORQUE_NM, max: MAX_BRAKE_TORQUE_NM, step: 0.001 },
  driveTorque: { min: 0.01, max: MAX_TARGET_TORQUE_NM, step: 0.001 },
  mtMaxSpeed: { min: 0.1, max: PI_MAX_SPEED_CAP, step: 0.01 },
  mtAccel: { min: 0.5, max: 5.5, step: 0.1 },
  mtGear1: { min: 0.05, max: 1, step: 0.05 },
  mtGear2: { min: 0.05, max: 1, step: 0.05 },
  mtGear3: { min: 0.05, max: 1, step: 0.05 },
  mtGear4: { min: 0.05, max: 1, step: 0.05 },
  mtGear5: { min: 0.05, max: 1, step: 0.05 },
  mtCoast: { min: 0.2, max: 5.5, step: 0.1 },
  mtEngineBrake: { min: 0, max: 6.0, step: 0.1 },
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
 * - `maxSpeed`/`mtMaxSpeed` ← `max_speed_m_s`
 * - `maxSteer` ← `max_steer_rad`
 * - `accel`/`coast`/`brake`/`mtAccel`/`mtCoast` ← `max_accel_m_s2`（GUI 側のランプ速度。
 *   実際の `COMMAND.accel_limit` はこれとは別に固定値 `ACCEL_SAFETY_LIMIT` を送っているが、
 *   STM32 側でどのみち `max_accel_m_s2` に丸められるため、ここより上に振ってもスライダが
 *   「上げても効かない」ものになるだけ。丸められる前に GUI 側で上限を揃えておく）
 * - `brakeTorque`/`driveTorque` ← `max_torque_nm`
 */
export function effectiveRange(limits: VehicleLimits | null | undefined): Record<NumericSettingKey, SettingRange> {
  const withLive = (r: SettingRange, live: number | null | undefined): SettingRange =>
    typeof live === 'number' && isFinite(live) && live > 0 ? { ...r, max: live } : r
  const accelCap = withLive(SETTINGS_RANGE.accel, limits?.max_accel_m_s2)
  const mtAccelCap = withLive(SETTINGS_RANGE.mtAccel, limits?.max_accel_m_s2)
  const torqueCap = withLive(SETTINGS_RANGE.brakeTorque, limits?.max_torque_nm)
  return {
    ...SETTINGS_RANGE,
    maxSpeed: withLive(SETTINGS_RANGE.maxSpeed, limits?.max_speed_m_s),
    mtMaxSpeed: withLive(SETTINGS_RANGE.mtMaxSpeed, limits?.max_speed_m_s),
    maxSteer: withLive(SETTINGS_RANGE.maxSteer, limits?.max_steer_rad),
    accel: accelCap,
    coast: { ...accelCap, min: SETTINGS_RANGE.coast.min },
    brake: { ...accelCap, min: SETTINGS_RANGE.brake.min },
    mtAccel: mtAccelCap,
    mtCoast: { ...mtAccelCap, min: SETTINGS_RANGE.mtCoast.min },
    brakeTorque: torqueCap,
    driveTorque: withLive(SETTINGS_RANGE.driveTorque, limits?.max_torque_nm),
  }
}

/**
 * `capUpper=false` は**保存する意思の値**用。下限と型だけ直し、上は切らない。車両の上限
 * （`LIMITS`。io_node の `--max-speed` 等で一時的に下がる）で切った値を保存すると、上限を
 * 戻しても設定が下がったまま戻らない。上で切るのは画面と送信に使う値（`settings`）だけ
 */
export function clampSettings(s: DrivingSettings, capUpper = true): DrivingSettings {
  // **`SETTINGS_RANGE`（静的）ではなく `effectiveRange`（`LIMITS` 受信済みならそちら
  // 優先）で必ずクランプすること。** ここを静的値のままにすると、スライダの DOM 要素
  // 自体は動的な `max` まで動くのに、`setSettings` を呼んだ瞬間ここで静的値まで
  // 巻き戻され「ドラッグしても弾かれる」バグになる。
  // `live` はフックではない素の可変オブジェクトなので、コンポーネント外のここからでも読める
  const range = effectiveRange(live.link)
  // **`LIMITS` が届く前は、最高速度の上を切らない。** 届く前のレンジは仮値（`PI_MAX_SPEED_CAP`）で、
  // そこで切ると、`LIMITS` の範囲で設定して保存した値がリロードのたびに仮値へ巻き戻る。
  // 届いた時点で `ws/telemetry.ts` が `setSettings({})` を呼び、ここをもう一度通す
  const speedCapKnown = typeof live.link?.max_speed_m_s === 'number'
  const out = { ...s }
  for (const k of Object.keys(range) as NumericSettingKey[]) {
    const { min, max } = range[k]
    const v = out[k]
    const upper = !capUpper || (!speedCapKnown && (k === 'maxSpeed' || k === 'mtMaxSpeed')) ? Infinity : max
    out[k] = typeof v === 'number' && isFinite(v) ? Math.min(upper, Math.max(min, v)) : DEFAULT_SETTINGS[k]
  }
  out.driveMode = out.driveMode === 'torque' || out.driveMode === 'mt' ? out.driveMode : 'speed'
  // 古い localStorage（v0.6 以前）にはこのキーが無い。**既定の ON に倒す**
  out.autoStop = typeof out.autoStop === 'boolean' ? out.autoStop : DEFAULT_SETTINGS.autoStop
  out.launchControl = typeof out.launchControl === 'boolean' ? out.launchControl : DEFAULT_SETTINGS.launchControl
  // 古い localStorage にはこのキーが無い、または不正値の場合は既定（m/s）に倒す
  out.speedUnit = out.speedUnit === 'kmh' ? 'kmh' : 'ms'
  // 古い localStorage（音色追加前）にはこのキーが無い。既定は内燃機関風に倒す
  out.engineSoundType = out.engineSoundType === 'ev' ? 'ev' : 'combustion'
  return out
}

/**
 * 保存してあった値を今の型に合わせて直す（`persist` の `merge`）。**版を上げなくても毎回通す**:
 * 後から足した項目は既定値で埋まり、壊れた値・範囲外の値はここで直る（旧キーの時代の読み込みと同じ）。
 */
export function restoreSettings(persisted: unknown): PersistedSettings & { settings: DrivingSettings } {
  const p = (isRecord(persisted) ? persisted : {}) as Record<string, unknown>
  const asObj = (v: unknown): Partial<DrivingSettings> => (isRecord(v) ? (v as Partial<DrivingSettings>) : {})
  const settingsIntent = clampSettings({ ...DEFAULT_SETTINGS, ...asObj(p.settingsIntent) }, false)
  const w = Number(p.winkerMode)
  return {
    settingsIntent,
    settings: clampSettings(settingsIntent),
    settingsDefault: clampSettings({ ...DEFAULT_SETTINGS, ...asObj(p.settingsDefault) }),
    winkerMode: [1, 2, 3].includes(w) ? w : 2,
  }
}

/**
 * 自動運転パラメータ（`raspi/auto/*.py` の `ParamSpec`）のうち「最高速度」に当たるキーの
 * スライダ上限。**ラジコンの`effectiveRange`と
 * 同じ理由・同じ方針**——`LIMITS`受信済みなら
 * 静的な`ParamSpec.max`より優先して無条件にそちらを使う。各`ParamSpec`側の
 * 静的`max`も車体の物理上限（現状3.0m/s）に合わせてあるので、通常時は
 * このcapが実際に効くのは`LIMITS`未受信の間の初期表示くらい。
 *
 * 加速度は対象にしていない。slam2d 系の前後の加速度は設定項目ではなくなり（同定値か走行中の
 * 学習で決まる。`raspi/nav/grip.py`）、`LIMITS.max_accel_m_s2` で縛るキーが無い。
 * 絶対値の前後加速度を設定に持つ planner を足すならここへ戻す。
 * 横加速度（`a_lat_max` 等）は物理量として別物なので、元から対象外。
 *
 * ⚠ これも`effectiveRange`と同じく**スライダの表示・入力レンジの話**。
 * 実際にモータへ効く上限は`io_node`の`_send_command`が`LIMITS`で無条件に
 * クランプする（`raspi/auto/`はSTM32の存在を知らない）。ここはGUI側の
 * 表示レンジをそれに追従させているだけ。
 */

const AUTO_SPEED_PARAM_KEYS = new Set(['max_speed', 'v_max', 'explore_speed'])

export function capAutoParams(
  params: AutoParamSpec[],
  limits: VehicleLimits | null | undefined,
): AutoParamSpec[] {
  const withLive = (p: AutoParamSpec, live: number | null | undefined): AutoParamSpec =>
    typeof live === 'number' && isFinite(live) && live > 0 ? { ...p, max: live } : p
  return params.map((p) => {
    if (AUTO_SPEED_PARAM_KEYS.has(p.key)) return withLive(p, limits?.max_speed_m_s)
    return p
  })
}

/**
 * v0.17 で `torqueMode: boolean` → `driveMode`（3択）に変えた。古い localStorage
 * （v0.6〜v0.16）は `torqueMode` だけを持ち `driveMode` が無い。**`DEFAULT_SETTINGS`
 * とマージする前に**変換すること——マージ後だと `driveMode` はすでに既定値 'speed'
 * で埋まっていて「未指定だったか」が区別できなくなる。
 */
function migrateDriveMode(raw: Record<string, unknown>): Record<string, unknown> {
  if (raw.driveMode == null && typeof raw.torqueMode === 'boolean') {
    raw.driveMode = raw.torqueMode ? 'torque' : 'speed'
  }
  return raw
}

// ── 保存（zustand `persist`） ───────────────────────────────────────

/** 保存する運転設定の部分。`settings`（今の上限で切った値）は保存せず、読み込み時に作り直す */
export type PersistedSettings = {
  /** 人が選んだ値（上は切っていない）。`settings` はこれを今の車両の上限で切ったもの（`clampSettings`） */
  settingsIntent: DrivingSettings
  settingsDefault: DrivingSettings
  winkerMode: number
}

/**
 * `persist` 導入前（〜2026-11）の保存形式を読む。キーごとに別々の生の値だった:
 * - `surge.driveSettings.v1`: 意思の値（JSON）
 * - `surge.driveSettingsDefault.v1`: 「既定値に戻す」の戻り先（JSON）
 * - `surge.winkerMode`: ウィンカーの点滅モード（数字の文字列）
 *
 * どれも無ければ null（初めて開いた端末）。中身は補正前の生の値で、`migrateSettings`（版 0）が直す。
 * 旧キーは消さない（古い GUI に戻しても前の設定で動くように）。
 */
export function readLegacySettings(): Partial<Record<keyof PersistedSettings, unknown>> | null {
  const read = (key: string): string | null => {
    try {
      return localStorage.getItem(key)
    } catch {
      return null
    }
  }
  const parse = (raw: string | null): unknown => {
    if (raw == null) return undefined
    try {
      return JSON.parse(raw)
    } catch {
      return undefined // 壊れた値は無いものとして扱う（既定値に倒れる）
    }
  }
  const intent = read('surge.driveSettings.v1')
  const def = read('surge.driveSettingsDefault.v1')
  const winker = read('surge.winkerMode')
  if (intent == null && def == null && winker == null) return null
  return { settingsIntent: parse(intent), settingsDefault: parse(def), winkerMode: winker == null ? undefined : Number(winker) }
}

/** 旧版の補正。`version` は保存されていた版（0 = `persist` 導入前の旧キー。`readLegacySettings`） */
export function migrateSettings(state: unknown, version: number): unknown {
  const s = (isRecord(state) ? { ...state } : {}) as Record<string, unknown>
  if (version < 1) {
    // v0.17 の `torqueMode` → `driveMode`。旧キーの時代はこれを読み込みのたびに掛けていた
    for (const k of ['settingsIntent', 'settingsDefault']) {
      if (isRecord(s[k])) s[k] = migrateDriveMode({ ...s[k] })
    }
  }
  return s
}

function isRecord(v: unknown): v is Record<string, unknown> {
  return typeof v === 'object' && v !== null && !Array.isArray(v)
}
