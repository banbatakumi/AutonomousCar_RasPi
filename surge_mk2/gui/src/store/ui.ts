/**
 * イベント駆動の状態だけを持つ store（`architecture.md` §10.4 の3層目）。
 *
 * **20Hz で変わるものはここに入れない。** 接続状態・操縦権・設定の類だけ。
 */
import { create } from 'zustand'
import type { EngineSoundType } from '../audio/engineSound'
import { live } from '../bus/live'
import type {
  AutoStatus,
  CamModelFile,
  CamModelStatus,
  CameraConfigStatus,
  ControlStatus,
  E2EModelFile,
  E2EModelStatus,
  FanStatus,
  LogFile,
} from '../types'
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
   * 常に `LIMITS.max_speed_m_s` 固定）。
   *
   * ⚠ v0.16 まではここに `cruiseScale`（既定巡航レンジ）と `boost`（Shift / パッド R1 で
   * `maxSpeed` まで一時的に伸ばす機能）があったが、指示により削除した。今は常に
   * `maxSpeed` そのものが上限——「全開が最高速度」というシンプルな1段仕様に戻した。
   */
  maxSpeed: number
  /** 見た目の最大舵角 [rad]。既定値 0.785(45°)。Pi 側ハード上限（`PI_MAX_STEER_CAP`）を超えられない */
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
   * ストローク後半が敏感になる。既定 0.45（v0.14 まで `STEER_EXPO` としてコード直書きだった値）
   */
  gamepadSteerExpo: number
  /**
   * ゲームパッド左スティックのデッドゾーン（中央の遊び）[0-1]。**パッドの舵にだけ効く。**
   * `dz()`（`useDriving.ts`）で切り落とすのではなく再スケールするので、ここを広げても
   * 「動き始めで飛ぶ」段差は出ない。既定 0.12（v0.14 まで `STICK_DZ` としてコード直書き）
   */
  gamepadSteerDeadzone: number
  /** ARM を保持したまま無操作でいられる時間 [ms]。超えたら自動 DISARM */
  armIdleTimeoutMs: number
  /** ブレーキの強さ [N·m/輪]。0 は送らない（未指定＝最大制動になってしまう） */
  brakeTorque: number
  /**
   * 制御方式（v0.6 で速度⇄トルクの2択として導入、v0.17 で MT を加えた3択に）。
   * 'speed' ＝ 速度制御（従来どおり `maxSpeed` が上限）。
   * 'torque' ＝ トルク直接指令（スロットル入力はすべて `target_speed` ではなく `target_torque`）。
   * 'mt' ＝ 擬似変速（速度制御の一種。上限は `maxSpeed` ではなく専用の `mtMaxSpeed` に
   * `mtGear1`〜`mtGear5` の倍率を掛けたもの。ワイヤプロトコルの `torque_mode` ビットは
   * 'speed' と同じく false のまま——STM32 からは普通の速度指令にしか見えない）。
   * 以前は `torqueMode: boolean` だった。
   *
   * ⚠ **2026-08-30: MT は 'speed' と一切パラメータを共有しない。** 以前は `maxSpeed`/
   * `accel`/`coast`/`kickSpeed` を流用していたため、'speed' モードの乗り味を詰めると
   * 意図せず MT の挙動まで変わってしまっていた。今は MT 専用の `mtMaxSpeed`/`mtAccel`/
   * `mtCoast`/`mtEngineBrake` を独立に持つ（下記）。`brakeTorque`（実ブレーキの強さ、
   * STM32 の N·m）だけは車両物理量なので引き続き共有——これは「乗り味」ではない。
   */
  driveMode: 'speed' | 'torque' | 'mt'
  /** トルクモード中にスロットル全開で出す駆動トルク [N·m]。上限 `MAX_TARGET_TORQUE_NM` */
  driveTorque: number
  /**
   * MT モード専用の最高速度 [m/s]（2026-08-30、`maxSpeed` から分離）。5速（倍率100%）で
   * この値そのものが上限になる。'speed' モードの `maxSpeed` とは完全に独立——
   * どちらかを変えてももう一方には影響しない。
   */
  mtMaxSpeed: number
  /** MT モード専用の加速度 [m/s²]（2026-08-30、`accel` から分離）。押している間／
   * トリガーを踏み込んだ方向へ近づくときのレート。'speed' モードの `accel` とは独立 */
  mtAccel: number
  /**
   * MT（擬似変速）モードの D1〜D5 それぞれの上限速度 [`mtMaxSpeed` に対する倍率、0-1]（v0.17）。
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
  /** ラジコンの速度計（`SpeedGauge.tsx`）の表示単位。既定 'ms'。クリックで切替、`localStorage` に保存 */
  speedUnit: 'ms' | 'kmh'
  /**
   * 合成エンジン音（`audio/engineSound.ts`）の音色（2026-08-30 追加）。
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
 * ★v0.12 で廃止。STM32 の自動停止距離は固定20cmではなく
 * `v・t_delay + v²/(2・a_max) + margin`（速度に応じて伸びる）に変わった。
 * `margin` だけが `CONFIG_SET`（`param_id=0x0060`, `AUTO_STOP_MARGIN_CM`）で
 * cm単位の連続値として直接指定できる（範囲0-100cm、既定15cm。当初は3段階enumの
 * 予定だったが実機投入前にSTM32側がcm直接指定へ変更した）。表示は
 * `link.auto_stop_margin_cm`（STM32 の `CONFIG_ACK` 由来のサーバ真値）を使うこと。
 * `pi_uart_protocol_v0.12_delta.md` 参照
 */
/**
 * 自動停止の既定。**ON にしてある。** 「効きすぎて止まる」より「気づかず当てる」方が
 * 損害が大きいため。⚠ STM32 側にヒステリシスが無いので、20cm 前後では
 * 効いたり切れたりする（チャタリング）。低速で壁に詰める作業では設定パネルで切ること。
 */
export const DEFAULT_AUTO_STOP = true

export const DEFAULT_SETTINGS: DrivingSettings = {
  maxSpeed: 3.0,
  maxSteer: 0.785,
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
  // 'speed' の maxSpeed/accel とは独立（2026-08-30）。まずは同じ値から始め、
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
  camHeight: VEHICLE.camFront.height,
  speedUnit: 'ms',
  engineSoundType: 'combustion',
}

/** 設定パネルのスライダのレンジ。`min`/`max`/`step` の3つ組。
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
  out.driveMode = out.driveMode === 'torque' || out.driveMode === 'mt' ? out.driveMode : 'speed'
  // 古い localStorage（v0.6 以前）にはこのキーが無い。**既定の ON に倒す**
  out.autoStop = typeof out.autoStop === 'boolean' ? out.autoStop : DEFAULT_SETTINGS.autoStop
  // 古い localStorage にはこのキーが無い、または不正値の場合は既定（m/s）に倒す
  out.speedUnit = out.speedUnit === 'kmh' ? 'kmh' : 'ms'
  // 古い localStorage（音色追加前）にはこのキーが無い。既定は内燃機関風に倒す
  out.engineSoundType = out.engineSoundType === 'ev' ? 'ev' : 'combustion'
  return out
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

/** 壊れた値・古いスキーマの値で走り出さないよう、読み込み時に必ずクランプする */
function loadSettings(): DrivingSettings {
  try {
    const raw = localStorage.getItem(SETTINGS_KEY)
    if (!raw) return DEFAULT_SETTINGS
    return clampSettings({ ...DEFAULT_SETTINGS, ...migrateDriveMode(JSON.parse(raw)) })
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
    return clampSettings({ ...DEFAULT_SETTINGS, ...migrateDriveMode(JSON.parse(raw)) })
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
 * 「リセット間走行距離」（`RcBar.tsx`）の起点。`vs.odom_center`（STM32 の
 * 累積走行距離、リセット不可）から引く値をここに持つ。リセットボタンで
 * その時点の `odom_center` を保存し、以後は差分をトリップ距離として表示する。
 */
const TRIP_ODOM_KEY = 'surge.tripOdomBase.v1'

function loadTripOdomBase(): number {
  try {
    const v = Number(localStorage.getItem(TRIP_ODOM_KEY))
    return isFinite(v) ? v : 0
  } catch {
    return 0
  }
}

function saveTripOdomBase(v: number) {
  try {
    localStorage.setItem(TRIP_ODOM_KEY, String(v))
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

/**
 * MT（擬似変速）モードのギア段（v0.17、v0.18 で N を追加）。R と D1 の間に N（ニュートラル）
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
  /** 直前に DISARM した理由。**「なぜ止まったか」が分からないのが一番消耗する** */
  disarmReason: string
  /** 直前に自律走行が解除された理由。同上（engage したときに消す） */
  autoOffReason: string

  // ── 補機（v0.5） ──
  //
  // **押している間だけ立つものは、rAF ループが値の変化時だけここへ書き戻す。**
  // 毎フレーム set すると 60Hz で再レンダリングが走る。
  /** Space / パッド L2（力加減） を押している間。`brake_torque` を直接掛けている */
  braking: boolean
  /** H / パッド □ を押している間。**鳴りっぱなしになる** */
  horning: boolean
  /** P / パッド ○ を押している間。前照灯だけが全光量 */
  passing: boolean
  /** 0=消灯 1=DAYTIME 2=NORMAL。**ARM 中しか反映されない**（§下） */
  lightMode: number

  /**
   * サイドブレーキの意思（`COMMAND.flags2` bit0、★v0.13）。
   *
   * **`braking`/`horning`/`passing` とは性質が違い、押しっぱなしではなく
   * トグル。** ON にしたら明示的に OFF にするまで送り続ける——駐車ブレーキと
   * 同じ操作感（かけたらハンドルから手を離せる）にしてある。STM32 は
   * **速度に関わらず即座に**後輪を位置制御へ切り替えて固定するため、走行中に
   * 誤って ON にしないよう、UI 側は停止中（`vs.stopped`）以外では ON を拒否する
   * （`AuxPanel.tsx`）。ページ再読み込みでは false に戻す（`localStorage` に
   * 保存しない）——保存すると再読み込み直後に前回の状態のまま送られてしまう。
   * 未 ARM のときは送らない（`useDriving.ts`、`brake` と同じ理由）
   */
  sideBrakeRequested: boolean

  /**
   * 左/右ウィンカーの意思（`COMMAND.flags2` bit1/2、v0.14）。
   *
   * `sideBrakeRequested` と同じくトグル（押している間だけではない）で、
   * **両方 true にするとハザード**として送られる（`command_from_cmd`）。
   * ページ再読み込みでは false に戻す（`localStorage` に保存しない。理由は
   * `sideBrakeRequested` と同じ——保存すると再読み込み直後に前回の状態のまま
   * 送られてしまう）。未 ARM でも送る（灯火と同じ扱い。`useDriving.ts`）
   */
  winkerLeftRequested: boolean
  winkerRightRequested: boolean

  /**
   * ゲームパッドの D/R レンジ（実車のシフトレバー相当、v0.14）。
   *
   * DUALSHOCK4 の R2/L2 を実車ペダル配置（R2=アクセル固定、L2=ブレーキ固定）に
   * したことで、R2 単体では前進/後退の向きを表せなくなった。その向きをここで選ぶ
   * （`useDriving.ts` の `gearSign`）。**キーボードの W/S は元々別キーなので影響しない。**
   *
   * `sideBrakeRequested` と違い、これは「今どちらへ踏み込むか」という持続的な選択
   * （実車のシフトと同じ）なので、DISARM で自動的に 'D' へは戻さない——切り返し
   * （バックで詰めて戻る等）の途中で毎回 D に戻ると操作が壊れる。**ページ再読み込みでは
   * 'D' に戻る**（`localStorage` に保存しない。理由は `sideBrakeRequested` と同じ）。
   * 走行中の誤操作を防ぐため、UI 側は停止中（`vs.stopped`）以外では切り替えを拒否する。
   *
   * 操作口はパッドの L1（R）/R1（D）とキーボードの←（R）/→（D）（v0.18、
   * `useDriving.ts` の `shiftGear()`。**2026-08-30 に D/R を入れ替えた**——
   * それまでは L1/←＝D、R1/→＝R だった）——どちらも同じ `ui.gear` を更新するので、
   * 二重に状態を持たない。**画面ボタンは v0.18 で廃止した**（現在値は速度メータ
   * 中央のバッジで表示だけする、`SpeedGauge.tsx`）。
   */
  gear: 'D' | 'R'

  /**
   * MT（擬似変速）モードのギア位置（v0.17）。`driveMode==='mt'` のときだけ意味を持つ
   * ——速度制御・トルク制御では従来どおり `gear`（'D'|'R'）を使う。
   *
   * `gear` と同じ操作口（パッドの L1/R1、キーボードの←/→）だが意味が違う。**下＝
   * シフトダウン、上＝シフトアップの相対操作**（direct-set の `gear` と違い、
   * 押すたびに `MT_GEARS` 上を1段動く。`useDriving.ts` の `shiftGear()`）。
   * R への出入りだけ `gear` と同じガード（走行中は拒否）を掛ける——N〜D5間は
   * 実車の MT と同じく走行中のシフトアップ/ダウンを許可する。
   * **既定・ページ再読み込みは N**（`localStorage` に保存しない、`gear` と同じ理由）。
   * **速度制御/トルク制御から MT に切り替えた直後も必ず N に戻す**（`setSettings`
   * 参照、2026-08-31 指示）——前回どのギアで MT を抜けたかを持ち越すと、
   * 切り替えた瞬間に前のギアの上限でいきなり動き出しかねないため。
   * 画面ボタンは無い（`gear` と同じ、v0.18 で廃止）——現在値は速度メータ中央の
   * バッジで表示だけする
   */
  mtGear: MtGear

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
  /**
   * ラジコンビューのメイン映像に出しているカメラ（`RcView.tsx`）。PIP には
   * 常にこの逆（`pipCam`）が出る。PIP クリックで入れ替わるほか、
   * ゲームパッドの R3（右スティック押し込み）でも切替可能（v0.16、`useDriving.ts`）——
   * レーシングゲームの「リアビュー確認」に近い操作なので R3 を割り当てた。
   * 以前は `RcView.tsx` のローカル `useState` だったが、パッドの rAF ループ
   * （React ツリーの外）から切り替えられるよう store に上げた。
   * **ページ再読み込みでは 'front' に戻る**（`localStorage` に保存しない。
   * `lidarExpanded` 等と同じ——起動直後に予期せず後方カメラがメインの状態は避けたい）
   */
  mainCam: 'front' | 'rear'

  /**
   * 合成エンジン音（`audio/engineSound.ts`）を鳴らすか。**GUI 演出のみで
   * 車両側には一切関与しない**——鳴るのはこの画面を開いているブラウザの
   * スピーカーだけ。`AudioContext` はブラウザの自動再生制限があるため、
   * ON トグルのクリック（ユーザー操作）の中で作る必要がある
   * （設定ドロワーの「サウンド」タブ、`SettingsPanel.tsx`／`hooks/useEngineSound.ts` 参照）。
   * **既定 OFF・`localStorage` には保存しない**（次回開いたときに勝手に
   * 音が鳴り出すと驚くため、毎回明示的に ON してもらう）。音色の選択
   * （`settings.engineSoundType`）は永続化する——こちらは驚きにつながらない嗜好のため。
   */
  engineSoundOn: boolean

  /** ラジコン操作の調整値。設定パネルで変更、`localStorage` に自動保存 */
  settings: DrivingSettings
  /** 「既定値に戻す」の戻り先。既定はソースコードの `DEFAULT_SETTINGS` だが、
   * `saveCurrentAsDefault` で `settings` の現在値に上書きできる（★2026-08-22） */
  settingsDefault: DrivingSettings

  /** 「リセット間走行距離」の起点（`vs.odom_center` の値）。`resetTripOdom` で更新、
   * `localStorage` に保存してリロードをまたいで維持する。`RcBar.tsx` が
   * `odom_center - tripOdomBase` を表示する */
  tripOdomBase: number

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

  /**
   * cam_perception_node が使うセグメンテーションモデルの選択。
   * **サーバが真値なのでここでは編集しない。** 押した結果は `status` の
   * ブロードキャストで返ってくる（`ws/control.ts` の `camModelSelect`）。
   */
  camModel: CamModelStatus | null
  /** `models/` にある `.onnx` の一覧（`camModelList` の応答） */
  camModelFiles: CamModelFile[]

  /**
   * e2e_lidar（強化学習）が使うモデルの選択。**サーバが真値なのでここでは編集しない。**
   * 押した結果は `status` のブロードキャストで返ってくる（`ws/control.ts` の
   * `e2eModelSelect`）。
   */
  e2eModel: E2EModelStatus | null
  /** `models/e2e_lidar/` にある `.onnx` の一覧（`e2eModelList` の応答） */
  e2eModelFiles: E2EModelFile[]

  set: (p: Partial<UiState>) => void
  /** 変更分だけ渡せば良い。クランプしてから保存＆反映する */
  setSettings: (p: Partial<DrivingSettings>) => void
  /** `settingsDefault`（＝「既定値に戻す」の戻り先）へ戻す。値そのものは変えない */
  resetSettings: () => void
  /** 今の `settings` を新しい既定値として保存する（★2026-08-22）。
   * 以後の「既定値に戻す」・各行の「既定値 X」表示はこの値を指す */
  saveCurrentAsDefault: () => void
  /** 「リセット間走行距離」を今の `vs.odom_center` で 0 に踏み直す */
  resetTripOdom: () => void
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
  disarmReason: '',
  autoOffReason: '',

  braking: false,
  horning: false,
  passing: false,
  lightMode: LIGHT_OFF,
  sideBrakeRequested: false,
  winkerLeftRequested: false,
  winkerRightRequested: false,
  gear: 'D',
  mtGear: 'N',

  settingsOpen: false,
  lidarZoom: 4, // 画面半径が何メートルぶんか
  lidarFollow: true,
  pathGuide: true,
  rearPip: true,
  lidarVisible: true,
  lidarExpanded: false,
  mainCam: 'front',
  engineSoundOn: false,

  settings: loadSettings(),
  settingsDefault: loadDefaultSettings(),
  tripOdomBase: loadTripOdomBase(),

  sfl: null,
  mcap: null,
  logFiles: [],
  auto: null,
  fan: null,
  cameraConfig: null,
  camModel: null,
  camModelFiles: [],
  e2eModel: null,
  e2eModelFiles: [],

  set: (p) => set(p),
  setSettings: (p) => {
    const prev = get().settings
    const next = clampSettings({ ...prev, ...p })
    saveSettings(next)
    // 速度制御/トルク制御から MT に切り替えた瞬間は必ず N から始める
    // （`mtGear` の型コメント参照）。前回 MT を抜けたときのギアを持ち越すと、
    // 切り替えた瞬間に前のギアの上限でいきなり動き出しかねないため
    const enteringMt = next.driveMode === 'mt' && prev.driveMode !== 'mt'
    set(enteringMt ? { settings: next, mtGear: 'N' } : { settings: next })
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
  resetTripOdom: () => {
    const base = live.vs?.odom_center ?? 0
    saveTripOdomBase(base)
    set({ tripOdomBase: base })
  },
}))
