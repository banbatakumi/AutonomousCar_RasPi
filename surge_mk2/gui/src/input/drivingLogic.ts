/**
 * ラジコン入力の**純粋な計算**（`useDriving.ts` から分けた。技術的負債の棚卸し 2026-11-18 高6）。
 *
 * 入力（キー・パッドの値）と設定から、速度・舵・トルクの指令と、ローンチコントロールの状態を作る。
 * React・ストア・WebSocket には触れないので単体テストで固められる（`drivingLogic.test.ts`）。
 * 方式・操作の説明は `useDriving.ts` 冒頭のコメントが正。**式を変えたら乗り味が変わる**ので、
 * テストを先に直すこと。
 */
import type { MtGear } from '../store/gear'
import { mtGearRatio } from '../store/gear'
import { MAX_TARGET_TORQUE_NM, type DrivingSettings, type LaunchPhase } from '../store/settings'
import type { CmdOut } from '../types'

/** 中央付近を緩くする度合い（0=リニア, 1=完全3乗）。スロットルは控えめのまま固定
 * （左右対称なペダルではなく踏み込み量なので、舵ほど中央を緩める必要が無い） */
const THROTTLE_EXPO = 0.25
// ── ローンチコントロール（手動操作、プロトコル v0.20〜。手順は `useDriving.ts` 冒頭の「ローンチコントロール」）──
/** パッドのアクセル（R2）をこれ以上踏んでいれば「全開のつもり」と見なして準備に入る */
const LAUNCH_TRIG = 0.9
/** 発進してからは、これを下回るまで続ける（踏み直しの揺れで切れないように） */
const LAUNCH_HOLD_TRIG = 0.5
/** 準備に入れるのは車速がこれ未満（止まっている）ときだけ [m/s] */
const LAUNCH_ARM_SPEED = 0.3
/** 車速が目標のこれだけ手前に届いたら発進は終わり [m/s] */
const LAUNCH_DONE_GAP = 0.1

/**
 * ボタンが押されているか。**`.pressed` だけでなく `.value` も見る**。
 * Bluetooth 接続のコントローラでは、ブラウザ／OS の組み合わせによって
 * デジタルボタンの `.pressed` が正しく反映されない個体・環境があり得るため
 * （アナログの `.value` は生きているのに `.pressed` だけ立たない、という報告があった）、
 * どちらか一方が閾値を超えていれば「押されている」とみなす保険を入れてある。
 */
export function padPressed(b: GamepadButton | undefined): boolean {
  if (!b) return false
  return b.pressed || b.value > 0.5
}

/**
 * パッドの入力内容の署名。**timestamp が進んだのに入力が同じ**かを見るために使う。
 *
 * 値の変化でしか `timestamp` が進まない機種では、この署名が同じなのに timestamp が
 * 進むことはないので「timestamp は心拍」という判定が誤って立たない。
 */
export function padSignature(p: Gamepad | null): string {
  if (!p) return ''
  let s = ''
  for (const b of p.buttons) s += `${b.pressed ? 1 : 0}:${b.value.toFixed(3)},`
  s += '|'
  for (const a of p.axes) s += `${a.toFixed(3)},`
  return s
}

/** デッドゾーンを**切り落とすのではなく再スケールする**。段差が出ない */
export function dz(v: number, zone: number): number {
  const a = Math.abs(v)
  if (a < zone) return 0
  return Math.sign(v) * ((a - zone) / (1 - zone))
}

/** 中央付近を緩く、端は素直に。`e=0` でリニア */
export function expo(v: number, e: number): number {
  return (1 - e) * v + e * v * v * v
}

export function approach(cur: number, target: number, rate: number, dt: number): number {
  const d = rate * dt
  if (Math.abs(target - cur) <= d) return target
  return cur + Math.sign(target - cur) * d
}

/** `L` キー / パッドの十字キー上で灯火を1つ進める（`LIGHT_CYCLE` を回す） */
export function nextLight(cycle: readonly number[], mode: number): number {
  const i = cycle.indexOf(mode)
  return cycle[(i + 1) % cycle.length]!
}

/** 制御方式とギアから決まる、その瞬間の上限・向き・減速のレート */
export type GearContext = {
  torqueModeOn: boolean
  mtModeOn: boolean
  /** 前進 +1・後退 -1。MT は `mtGear`、それ以外は D/R レンジで決まる */
  gearSign: 1 | -1
  /** 速度の上限 [m/s]。'speed' は `maxSpeed`、MT は `mtMaxSpeed × ギアの倍率`（N は 0） */
  maxSpeed: number
  /** MT のエンジンブレーキ相当の減速レート [m/s²]。ギア比が低いほど強く、N は `mtCoast` のみ */
  mtEngineBrakeRate: number
}

export function gearContext(s: DrivingSettings, gear: 'D' | 'R', mtGear: MtGear): GearContext {
  // 制御方式は「速度制御 / トルク制御 / MT（擬似変速）」の3択（`driveMode`）。
  // MT は速度制御の一種——ワイヤプロトコルには torque_mode=false（速度指令）で送る
  const torqueModeOn = s.driveMode === 'torque'
  const mtModeOn = s.driveMode === 'mt'
  // MT モードでは向きも `ui.mtGear`（R, N, D1〜D5）が持つ。それ以外は `ui.gear`
  const gearSign = mtModeOn ? (mtGear === 'R' ? -1 : 1) : (gear === 'R' ? -1 : 1)
  // 'speed' は常に `maxSpeed` が上限。MT は専用の `mtMaxSpeed`（'speed' と非共有）に
  // `mtGear` に応じた倍率（`mtGear1`〜`mtGear5`）を掛ける。
  const mtRatio = mtModeOn ? mtGearRatio(mtGear, s) : 0
  const maxSpeed = mtModeOn ? s.mtMaxSpeed * mtRatio : s.maxSpeed
  // MT モードのエンジンブレーキ相当レート。ギア比が低いほど強く、
  // N は加算なし（`mtCoast` のみ＝いちばん緩い）。アクセルオフの惰行と、
  // シフトダウンで上限を超えたときのなだらかな減速の両方に使う
  const mtEngineBrakeRate = mtModeOn
    ? (mtGear === 'N' ? s.mtCoast : s.mtCoast + s.mtEngineBrake * (1 - mtRatio))
    : 0
  return { torqueModeOn, mtModeOn, gearSign, maxSpeed, mtEngineBrakeRate }
}

/** 手動の指令の積分状態 */
export type ManualState = { speed: number; steer: number; torque: number }

/** 1フレームの手動入力 */
export type ManualInput = {
  /** 入力元（最後に動かした側で固定する。`useDriving.ts`） */
  source: 'keyboard' | 'gamepad'
  fwd: boolean
  rev: boolean
  left: boolean
  right: boolean
  /** R2（デッドゾーン処理済み 0〜1） */
  accelTrig: number
  /** 左スティック X（デッドゾーン処理済み -1〜1） */
  padSteer: number
  /** ブレーキ（Space か L2）を踏んでいるか */
  brake: boolean
}

/**
 * 手動の入力を1フレーム積分する（ローンチ・ブレーキ・クランプの前）。
 * パッドはランプを挟まず目標をそのまま、キーボードは加速・惰行・逆キーブレーキのランプで作る。
 */
export function stepManual(st: ManualState, inp: ManualInput, s: DrivingSettings, g: GearContext, dt: number): ManualState {
  let { speed, steer } = st
  let torque: number // どの経路でも作り直す（前フレームの値は使わない）
  const { torqueModeOn, mtModeOn, gearSign, maxSpeed, mtEngineBrakeRate } = g
  if (inp.source === 'gamepad') {
    const { accelTrig, padSteer } = inp
    // **アナログ入力はランプを挟まず、目標をそのまま出す。**
    // 人の指がすでにランプになっているので、二重に鈍らせない。
    // 急な踏み込みは STM32 の accel_limit / steer_rate_limit が受け持つ
    // 左スティック左 (-1) が左旋回 (+steer)。反時計回りが正
    steer = -expo(padSteer, s.gamepadSteerExpo) * s.maxSteer
    // R2（アクセル）の踏み込み量に `gearSign`（Dレンジ/Rレンジ）で向きを付ける。
    // L2 はここでは使わない——ブレーキとして `brake`/`brakeStrength` 側で処理済み
    if (torqueModeOn) {
      // トルクモードも同じ理由でランプなし。速度指令は使わないので 0
      torque = expo(accelTrig, THROTTLE_EXPO) * s.driveTorque * gearSign
      speed = 0
    } else if (mtModeOn) {
      // MT: 目標（トリガー踏み込み量、離していれば0）へ
      // `approach()` でレート制限しながら近づける。**目標へ近づく向きで
      // レートを変える**——増える方向（加速）は `s.mtAccel`、減る方向
      // （アクセルオフの惰行・シフトダウンで上限を超えた分の解消）は
      // `mtEngineBrakeRate`。両方とも一つの式で表せるので分岐が要らない
      //
      // `accel` ではなく `mtAccel` を使う——'speed' モードとパラメータを共有しない
      const mtTarget = accelTrig > 0 ? expo(accelTrig, THROTTLE_EXPO) * maxSpeed * gearSign : 0
      const mtRate = Math.abs(mtTarget) > Math.abs(speed) ? s.mtAccel : mtEngineBrakeRate
      speed = approach(speed, mtTarget, mtRate, dt)
      torque = 0
    } else {
      // 速度モードもトリガーの目標へ `accel`（加速）／`coast`（惰行・逆向きの解消）でランプする。
      // ランプが無いと STM32 の `accel_limit` 上限（約3m/s²）が実効の加減速になり、設定が効かない
      const target = expo(accelTrig, THROTTLE_EXPO) * maxSpeed * gearSign
      const accelerating = target * speed >= 0 && Math.abs(target) > Math.abs(speed)
      speed = approach(speed, target, accelerating ? s.accel : s.coast, dt)
      torque = 0
    }
  } else {
    const dir = inp.fwd ? 1 : inp.rev ? -1 : 0
    if (torqueModeOn) {
      // **トルクモードはランプを挟まない。** 0.1N・m 程度と値域が小さく、
      // 速度用の加速度ランプ（m/s²）はそのままでは単位が合わない。
      // 押している間だけ `driveTorque` をそのまま出す
      torque = dir * s.driveTorque
      speed = 0
    } else if (mtModeOn) {
      // MT: 実車にアクセルペダルは1つしか無い——ギアの向きと
      // 逆のキーを押しても何もしない（`brake` を使った強制ブレーキはしない。
      // 押していないのと同じ扱いになり、下でエンジンブレーキ／惰行が効く）。
      // 発進キックも廃止——`s.mtAccel` で 0 からそのままランプする
      const effDir = dir === gearSign ? dir : 0
      if (Math.abs(speed) > maxSpeed) {
        speed = approach(speed, Math.sign(speed) * maxSpeed, mtEngineBrakeRate, dt)
      } else if (effDir !== 0 && maxSpeed > 0) {
        // 上限ちょうどで止める（足すだけだと上限を1刻み越えては戻すのを毎フレーム繰り返す）
        speed = approach(speed, effDir * maxSpeed, s.mtAccel, dt)
      } else {
        speed = approach(speed, 0, mtEngineBrakeRate, dt)
      }
      torque = 0
    } else {
      // ── 速度：押している間は加速、逆キーはブレーキ ──
      if (dir === 0) {
        speed = approach(speed, 0, s.coast, dt)
      } else if (speed * dir < 0) {
        // 進行方向と逆を押している ＝ ブレーキ。0 を行き過ぎたら 0 で受け止め、
        // 次のフレームから加速で逆走に移る
        const next = speed + dir * s.brake * dt
        speed = next * dir > 0 ? 0 : next
      } else {
        speed += dir * s.accel * dt
      }
      torque = 0
    }

    // ── 舵：切り込みより戻しを速く、切り返しはさらに速く ──（速度/トルク共通）
    const sdir = inp.left ? 1 : inp.right ? -1 : 0
    if (sdir === 0) {
      steer = approach(steer, 0, s.steerReturn, dt)
    } else {
      const rate = steer * sdir < 0 ? s.steerCounter : s.steerRate
      steer = approach(steer, sdir * s.maxSteer, rate, dt)
    }
  }
  return { speed, steer, torque }
}

/** ローンチコントロールの状態（`launchSeen` = 発進してから STM32 の `launch_active` を一度でも見たか） */
export type LaunchState = { phase: LaunchPhase; seen: boolean }

/**
 * ローンチコントロールを1フレーム進める（手順は `useDriving.ts` 冒頭）。
 * 戻り値の `target`/`torque` は発進中（`go`）に置く目標速度・トルク。
 */
export function stepLaunch(
  ls: LaunchState,
  inp: ManualInput,
  s: DrivingSettings,
  g: GearContext,
  vNow: number,
  launchActive: boolean,
): LaunchState & { target: number; torque: number } {
  const { torqueModeOn, mtModeOn, gearSign, maxSpeed } = g
  // アクセルの向き（踏んでいなければ 0）。パッドは「ほぼ全開」だけを発進の意思と見なす
  const kdir = inp.fwd ? 1 : inp.rev ? -1 : 0
  const launchDir =
    !s.launchControl || (!torqueModeOn && maxSpeed <= 0)
      ? 0
      : inp.source === 'gamepad'
        ? (inp.accelTrig >= (ls.phase === 'go' ? LAUNCH_HOLD_TRIG : LAUNCH_TRIG) ? gearSign : 0)
        : (mtModeOn ? (kdir === gearSign ? kdir : 0) : kdir)
  // トルク制御の発進は、設定の `driveTorque`・踏み込み量に関わらず STM32 の最大トルクを出す
  // （空転は TC が絞る）。目標速度が無いので、終わるのはアクセルを離すかブレーキを踏んだとき
  const torque = launchDir * MAX_TARGET_TORQUE_NM
  const target = launchDir * maxSpeed * (inp.source === 'gamepad' ? expo(inp.accelTrig, THROTTLE_EXPO) : 1)
  let phase = ls.phase
  let seen = ls.seen
  if (launchDir === 0) {
    phase = 'idle'
  } else if (phase === 'idle') {
    if (inp.brake && Math.abs(vNow) < LAUNCH_ARM_SPEED) phase = 'ready'
  } else if (phase === 'ready') {
    if (!inp.brake) {
      phase = 'go'
      seen = false
    }
  } else {
    // 車速が目標に届いたか、STM32 が発進を終えた（目標は io_node の `--max-speed` で切り詰められる
    // ことがあり、そのときは GUI の目標には届かない）
    if (launchActive) seen = true
    const done = !torqueModeOn && vNow * launchDir >= Math.abs(target) - LAUNCH_DONE_GAP
    if (inp.brake || done || (!torqueModeOn && seen && !launchActive)) phase = 'idle'
  }
  return { phase, seen, target, torque }
}

/**
 * ローンチ・ブレーキ・上限を掛けて、送る指令値に仕上げる。
 * - 発進中は GUI 側のランプを飛ばして目標を上限へ置く（STM32 が目標との差を見て全開にする）。トルク制御は最大トルク
 * - **ブレーキ中は速度・トルク指令を 0 に落とす。** STM32 は `brake` の間 `target_speed`/`target_torque` を無視し、
 *   離すと 0 から `accel_limit` に従って加速し直す。GUI 側のランプを走らせたままにすると、「画面は 0.4 m/s と
 *   出ているのに車は止まっている」という嘘の表示になる。舵はブレーキ中も生かす（曲がりながら止めたい場面がある）
 * - MT は速度を即クランプしない（エンジンブレーキのランプで既に上限内へなだらかに収れんさせてある。
 *   ここで切ると、シフトダウンした瞬間に速度がワープする）
 */
export function finishManual(
  st: ManualState,
  launch: { phase: LaunchPhase; target: number; torque: number },
  brake: boolean,
  s: DrivingSettings,
  g: GearContext,
): ManualState {
  let { speed, steer, torque } = st
  if (launch.phase === 'go') {
    if (g.torqueModeOn) torque = launch.torque
    else speed = launch.target
  }
  if (brake) {
    speed = 0
    torque = 0
  }
  if (!g.mtModeOn) speed = Math.max(-g.maxSpeed, Math.min(g.maxSpeed, speed))
  // トルク制御のローンチ中だけは設定の `driveTorque` を超えて最大トルクまで出す
  const torqueCap = launch.phase === 'go' && g.torqueModeOn ? MAX_TARGET_TORQUE_NM : s.driveTorque
  torque = Math.max(-torqueCap, Math.min(torqueCap, torque))
  steer = Math.max(-s.maxSteer, Math.min(s.maxSteer, steer))
  return { speed, steer, torque }
}

/**
 * いま送るか。**中身が変わっていれば次のフレームで送り、同じ中身の繰り返しは
 * `repeatMs` に間引く**（デッドマンを生かしておくだけなので）。
 *
 * 一律に 20ms で間引いていた頃は、60Hz の画面では rAF の 16.7ms が 20ms に届かず
 * **1フレームおきにしか送れなかった**（実効 30Hz。操作が指令に載るまで最大 33ms）。
 */
export class TxThrottle {
  private lastMs = 0
  private lastKey = ''
  constructor(
    /** 中身が変わった指令を続けて送るときの最短間隔 [ms] */
    private readonly minMs: number,
    /** 同じ中身を繰り返すときの間隔 [ms] */
    private readonly repeatMs: number,
  ) {}

  due(now: number, c: Omit<CmdOut, 'type'>): boolean {
    const key = JSON.stringify(c)
    const minMs = key !== this.lastKey ? this.minMs : this.repeatMs
    if (now - this.lastMs < minMs) return false
    this.lastMs = now
    this.lastKey = key
    return true
  }
}
