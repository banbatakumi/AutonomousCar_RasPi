/**
 * rAF で入力を積分し、変化した指令はそのフレームで送るループ（`useDriving.ts` から分けた）。
 *
 * 方式（ARM・デッドマン・補機・自律走行・ローンチ）の説明は `useDriving.ts` 冒頭が正。
 * 指令の式は `drivingLogic.ts`、パッドの読み取りは `gamepad.ts`、キーの押下状態は `useKeyboardInput.ts`。
 * ここに残したのは「どの経路で何を送るか」（未 ARM の灯火・操縦権の取得・自律走行・手動）と、
 * その途中で ARM を下ろす判断（E-Stop・封印・無操作）。**送信の条件と周期を変えないこと。**
 */
import { useEffect, useRef, type RefObject } from 'react'
import { cmdOut, live } from '../bus/live'
import { SAFETY } from '../generated/vehicle'
import { rafLoop } from '../hooks/useRaf'
import { notify } from '../store/notice'
import { ACCEL_SAFETY_LIMIT, STEER_RATE_SAFETY_LIMIT, type LaunchPhase } from '../store/settings'
import { LIGHT_CYCLE, LIGHT_OFF, useUi } from '../store/ui'
import type { CmdOut } from '../types'
import type { ControlChannel } from '../ws/control'
import {
  TxThrottle, dz, finishManual, gearContext, nextLight, padPressed, stepLaunch, stepManual,
  type LaunchState, type ManualInput, type ManualState,
} from './drivingLogic'
import { GamepadReader, TRIGGER_DZ } from './gamepad'
import {
  BRAKE_KEY, HORN_KEY, PASSING_KEY, SPEED_KEYS_FWD, SPEED_KEYS_REV, STEER_KEYS_LEFT, STEER_KEYS_RIGHT,
} from './keys'
import { shiftGear } from './useKeyboardInput'

/** **同じ中身を繰り返すとき**の送信レート。telemetry_node の `CMD_PUB_HZ` に合わせる
 * （rAF の刻みに丸められるので、60Hz の画面では実際は 30Hz。デッドマンには十分） */
const TX_HZ = 50
const TX_INTERVAL_MS = 1000 / TX_HZ
/** 中身が変わった指令を続けて送るときの最短間隔 [ms]。rAF ごと（60Hz で 16.7ms、120Hz で 8.3ms）に
 * 送れるようにしつつ、それより速い画面で Pi へ流し込みすぎない */
const TX_MIN_INTERVAL_MS = 7

/**
 * 送信が途絶えてから車が止まるまで [ms]。
 *
 * **数字を GUI に手書きしない。** `config/vehicle.toml` の `[safety]` が正で、
 * `config/generate.py` がここへ配り、Pi 側は `raspi/core/vehicle.py` から
 * 同じ値を読む（数値を複数箇所に手書きすると一致を保証できない）。
 */
const DEADMAN_MS = SAFETY.cmdDeadmanMs

// **送信周期がデッドマンに近づいたら設計が破綻している。** 1発落としただけで
// 停止する状態になるので、開発中に気づけるようにここで見ておく
if (import.meta.env.DEV && TX_INTERVAL_MS * 3 > DEADMAN_MS) {
  console.warn(
    `送信周期 ${TX_INTERVAL_MS}ms に対しデッドマンが ${DEADMAN_MS}ms しかない。` +
      '取りこぼし数発で DISARM に落ちる',
  )
}
/** 1フレームの積分幅の上限。タブ復帰やカクつきで一気に飛ぶのを防ぐ */
const DT_MAX = 0.05

// ── STM32 に渡すレートリミット ───────────────────────────────────
/** GUI のランプ（設定パネルの `accel`/`brake`）より速い値。**通常は効かず、保険として残る**。
 * ユーザー設定にはしない（`ACCEL_SAFETY_LIMIT` 参照） */
const ACCEL_LIMIT = ACCEL_SAFETY_LIMIT // m/s²
const STEER_RATE_LIMIT = STEER_RATE_SAFETY_LIMIT // rad/s

/** 右スティック（LiDAR ミニマップ操作用）のデッドゾーン */
const RSTICK_DZ = 0.15
/** 右スティック上下いっぱいで 1 秒あたり何 m ズームが変わるか */
const MINI_ZOOM_RATE_M_S = 4
const MINI_ZOOM_MIN_M = 1.5
const MINI_ZOOM_MAX_M = 8

// ── ゲームパッドのボタン番号（standard mapping） ──
//
// 面ボタン（×○△□）は「安全系＋頻度の低いトグル」、十字キーは「合図・灯火系」
// （上＝灯火、下＝パッシング、左＝左ウィンカー、右＝右ウィンカー）。
// L1/R1 はギアチェンジ専任。
// 標準マッピングでない実機・OS の組み合わせでは番号がズレることがあるので、
// 繋いだら一度 GUI 上で反応を確認すること
// （`import.meta.env.DEV` では押されたボタン番号をコンソールに出す。下記参照）。
/** × — ARM/DISARM トグル（Enter キーと同じ）。armed→unarmed の遷移だけ `ch.estop()`
 * も呼ぶ（下記 `tick()` 参照） */
const PAD_ARM_TOGGLE = 0
/** □ — クラクション */
const PAD_HORN = 2
/** △ — サイドブレーキの ON/OFF トグル。走行中でも ON にできる（`AuxPanel.tsx` の ON/OFF ボタンと同じ）。
 * **押した瞬間だけ** 反転する */
const PAD_SIDE_BRAKE = 3
/** L1 — Rレンジ（パッドのブレーキは L2（アナログ、下記）だけ）。
 * **MT モードでは意味が変わり、シフトダウンになる**（下記 `tick()` 参照） */
const PAD_GEAR_L1 = 4
/** R1 — Dレンジ。
 * **MT モードではシフトアップになる**（`PAD_GEAR_L1` と同じ理由） */
const PAD_GEAR_R1 = 5
/** R3（右スティック押し込み） — LiDAR ミニマップの拡大表示切替 */
const PAD_LIDAR_EXPAND = 11
/** 十字キー上 — 灯火モードを1つ進める。**押した瞬間だけ** */
const PAD_LIGHT = 12
/** 十字キー下 — パッシング */
const PAD_PASSING = 13
/** 十字キー左 — 左ウィンカーの ON/OFF トグル。
 * **押した瞬間だけ** 反転する。右ウィンカーと両方 ON でハザード
 * （`PAD_WINKER_RIGHT` 参照、STM32 側の解釈は `protocol.toml` flags2） */
const PAD_WINKER_LEFT = 14
/** 十字キー右 — 右ウィンカーの ON/OFF トグル。**押した瞬間だけ** 反転する */
const PAD_WINKER_RIGHT = 15

/** 操縦権の再要求はこの間隔まで。rAF ごとに投げると 60発/秒になる */
const TAKE_CONTROL_MS = 250

export function useCommandLoop(
  ch: ControlChannel | null,
  keys: RefObject<Set<string>>,
  /** 最後に人が操作した時刻（無操作タイムアウトの起点。ARM した瞬間も `useDriving.ts` が書く） */
  lastActivity: RefObject<number>,
) {
  const manual = useRef<ManualState>({ speed: 0, steer: 0, torque: 0 })
  const sourceRef = useRef<'keyboard' | 'gamepad'>('keyboard')
  const launch = useRef<LaunchState>({ phase: 'idle', seen: false })
  /** 自律解除を投げた時刻。rAF ごとに投げると 60発/秒になる（`TAKE_CONTROL_MS` と同じ理由） */
  const lastAutoCancel = useRef(0)

  useEffect(() => {
    if (!ch) return
    let prevMs = performance.now()
    let lastTakeMs = 0
    const pads = new GamepadReader()
    const tx = new TxThrottle(TX_MIN_INTERVAL_MS, TX_INTERVAL_MS)
    /** DEV 診断用（下記の `padPressed` ログ）。ボタンごとの直前の押下状態 */
    const padDebugButtons: Record<string, boolean> = {}

    /**
     * 補機の押下状態を store に写す。**変化したときだけ**書く（毎フレーム書くと
     * 60Hz で再レンダリングが走る）。指令を送らない経路では必ず false を写すこと。
     * **画面に「鳴っている」と出したまま実際は送っていない**のが一番混乱する。
     */
    // `launch` を渡さない経路（未 ARM・操縦権なし・自律走行）では、ローンチの状態も一緒に捨てる
    const mirrorAux = (brake: boolean, horn: boolean, passing: boolean, phase: LaunchPhase = 'idle') => {
      launch.current.phase = phase
      const u = useUi.getState()
      if (u.braking !== brake || u.horning !== horn || u.passing !== passing || u.launchPhase !== phase) {
        u.set({ braking: brake, horning: horn, passing, launchPhase: phase })
      }
    }

    /** 手動側の積分と計器の指令を 0 に戻す（ARM していない・操縦権が無い・自律走行中） */
    const resetManual = () => {
      manual.current = { speed: 0, steer: 0, torque: 0 }
    }

    const tick = () => {
      const now = performance.now()
      const dt = Math.min((now - prevMs) / 1000, DT_MAX)
      prevMs = now
      if (dt <= 0) return

      const ui = useUi.getState()
      const s = ui.settings
      const { pad, staleStarted } = pads.read(navigator.getGamepads?.() ?? [], now, s.gamepadSteerDeadzone)
      if (staleStarted) notify('パッドの入力が更新されていません（圏外？）— 入力を中立にしました', 'bad')

      // ── 入力を読む ──
      const k = keys.current
      const fwd = SPEED_KEYS_FWD.some((c) => k.has(c))
      const rev = SPEED_KEYS_REV.some((c) => k.has(c))
      const left = STEER_KEYS_LEFT.some((c) => k.has(c))
      const right = STEER_KEYS_RIGHT.some((c) => k.has(c))
      const keyActive = fwd || rev || left || right

      // R2＝アクセル、L2＝ブレーキ（DUALSHOCK4 の実車ペダル配置）。
      // 前進/後退は `ui.gear`（画面の Dレンジ/Rレンジ）が決める。R2/L2 単体では
      // 向きを持たない——実車のアクセル/ブレーキペダルと同じ
      const accelTrig = dz(pad?.buttons[7]?.value ?? 0, TRIGGER_DZ)
      const brakeTrig = dz(pad?.buttons[6]?.value ?? 0, TRIGGER_DZ)
      const padSteer = dz(pad?.axes[0] ?? 0, s.gamepadSteerDeadzone)
      const padActive = accelTrig > 0 || brakeTrig > 0 || padSteer !== 0

      const g = gearContext(s, ui.gear, ui.mtGear)

      // ── 補機（すべて「押している間だけ」） ──
      //
      // ブレーキはキーボードの Space（フル、デジタル）と L2（力加減、アナログ）の合成。
      // パッドの L1 はギアチェンジ用なので、パッドのブレーキは L2 のみ。
      // **L2 は踏み込み量をそのまま `brake_torque` の倍率にする**（`s.brakeTorque` が
      // 上限）。`_brake_torque_raw`（`msgs/convert.py`）は正の値を 0 に丸めないので、
      // 軽く踏んだときに「未指定＝最大制動」に化けることはない
      const brakeStrength = Math.max(k.has(BRAKE_KEY) ? 1 : 0, brakeTrig)
      const brake = brakeStrength > 0
      const horn = k.has(HORN_KEY) || padPressed(pad?.buttons[PAD_HORN])
      const passing = k.has(PASSING_KEY) || padPressed(pad?.buttons[PAD_PASSING])

      // 灯火切替だけはトグル。**押した瞬間のエッジで1つ進める**
      // （rAF で読むので、キーの `e.repeat` に相当する処理を自前で持つ）
      if (pads.rose(pad, PAD_LIGHT)) ui.set({ lightMode: nextLight(LIGHT_CYCLE, ui.lightMode) })

      // × は Enter と同じ ARM/DISARM トグル。**押した瞬間のエッジで反転**
      // （灯火と同じ理由——押しっぱなしで何度も切り替わっては困る）
      //
      // armed→unarmed の遷移で `ch.estop()` も呼ぶ。E-STOP は「即 DISARM（権限も条件も
      // 要らない）」以上のものではない（`estop_active` はハードウェアの物理ボタン専用の
      // ラッチで、GUI/パッドからは立てられない——`raspi/core/link_tracker.py` 参照）。`ch.estop()` は**操縦権を持っていなくても通る**
      // ので、他のタブ/PCが操縦している状態からでもこの端末の × で止められる
      // （ローカルの `ui.armRequested` を false にするだけの経路より強い）。
      if (pads.rose(pad, PAD_ARM_TOGGLE)) {
        if (ui.armRequested) {
          ch.estop()
          // 自分の操作なので理由は出さない（ARM ボタン・Enter と同じ。指示による）
          ui.disarm('')
        } else {
          ui.arm()
        }
      }

      // Rレンジ/Dレンジの切替（L1=R・R1=D。MT モードでは「シフトダウン/アップ」という
      // 相対操作になる。キーボードの←/→も同じ操作で、分岐ロジックは `shiftGear()` に共通化してある）
      if (pads.rose(pad, PAD_GEAR_L1)) shiftGear(-1)
      if (pads.rose(pad, PAD_GEAR_R1)) shiftGear(1)

      // 右スティック＝ LiDAR ミニマップの操作。走行には関与しないので
      // ARM の有無に関係なく効く。上下（軸3）でズーム、R3 で拡大表示切替
      const padZoomY = dz(pad?.axes[3] ?? 0, RSTICK_DZ)
      if (padZoomY !== 0) {
        const z = live.lidarMiniZoomM + padZoomY * MINI_ZOOM_RATE_M_S * dt
        live.lidarMiniZoomM = Math.max(MINI_ZOOM_MIN_M, Math.min(MINI_ZOOM_MAX_M, z))
      }
      if (pads.rose(pad, PAD_LIDAR_EXPAND)) ui.set({ lidarExpanded: !ui.lidarExpanded })

      // △＝サイドブレーキの ON/OFF トグル。
      // **走行中でも ON にできる**（`AuxPanel.tsx` の画面ボタンと同じ。ハンドブレーキ
      // ターンのような使い方を妨げない）。STM32 側は速度に関わらず
      // 即座に後輪を位置制御へ切り替えて固定するので（`protocol.toml` flags2）、
      // 高速走行中に踏むとロックしてスピン/スリップする点は使う側の判断に委ねる
      if (pads.rose(pad, PAD_SIDE_BRAKE)) ui.set({ sideBrakeRequested: !ui.sideBrakeRequested })

      // 十字キー左/右＝ウィンカーの ON/OFF トグル。灯火系なので
      // ARM の有無に関係なく効く（`side_brake` と違い未 ARM でも送る。下記参照）。
      // 両方 ON にすればハザード（STM32 側の解釈、`protocol.toml` flags2 参照）
      if (pads.rose(pad, PAD_WINKER_LEFT)) ui.set({ winkerLeftRequested: !ui.winkerLeftRequested })
      if (pads.rose(pad, PAD_WINKER_RIGHT)) ui.set({ winkerRightRequested: !ui.winkerRightRequested })

      // ── DEV 診断：どのボタン/軸が実際に反応しているかをコンソールに出す ──
      //
      // Bluetooth 接続のコントローラは OS・ブラウザの組み合わせでボタン番号が
      // ズレることがあるので、繋いだコントローラの実際の番号を確認したいときは
      // devtools のコンソールを開いて押し、上の `PAD_*` 定数と見比べること。
      if (import.meta.env.DEV && pad) {
        pad.buttons.forEach((b, i) => {
          const p = padPressed(b)
          const key = `pad-btn-${i}`
          if (p !== padDebugButtons[key]) {
            console.log(`[gamepad] button[${i}] ${p ? 'DOWN' : 'up'} (pressed=${b.pressed}, value=${b.value.toFixed(2)})`)
            padDebugButtons[key] = p
          }
        })
      }

      // **押しっぱなしも「操作中」に数える。** keydown だけを見ると、
      // W を握り続けているのにタイムアウトで切れる。
      // 補機も操作のうち（人間が目の前にいる証拠なので ARM を維持してよい）
      if (keyActive || padActive || brake || horn || passing) lastActivity.current = now

      // ── 車が ARM を受け付けない間は ARM を保持しない ──
      //
      // E-STOP（車両の物理ボタン・ハートビート途絶）や io_node の封印（`arm_inhibited`。STM32 の
      // 再起動・版の不一致など）の間、ARM を保持し続けても車は動かない。Pi 側は解除後も
      // `arm=false` を一度受けるまで ARM を通さないので、ここで下ろして DISARM を送り
      // （`useDriving.ts` の購読）、理由を出す。走らせるには原因を解いてから人が ARM し直す
      const link = live.link
      if (ui.armRequested && link && (link.estop_active || link.arm_inhibited)) {
        ui.disarm(
          link.estop_active
            ? 'E-STOP 発動中（車両のボタンで解除してから ARM）'
            : `ARM できない: ${link.arm_inhibit_reason || 'io_node が封印中'}`,
        )
        mirrorAux(false, false, false)
        return
      }

      // ── 自律走行中に人が操作したら、自律を解除して手動に戻す ──
      //
      // クルーズコントロールと同じ作法。**「人の操作が勝つ」を無条件にする**。
      // どちらが舵を出しているのか分からない状態を作らないための解除であって、
      // 停止手段ではない（止めるのは Esc / E-STOP / ブレーキ）。
      // ★ 走行中に解除されると、サーバ（`telemetry_node._handover`）が**止まるかアクセルを入れるまで
      //   最大制動**する（舵は人が切ればその値、切らなければ自律の最後の値）。無操作の手動指令
      //   （舵 0・トルク 0 = 惰行）へいきなり替わると壁に当たるため。運転の設定に依らない
      //
      // ⚠ ブレーキは解除の往復を待たずに**そのまま送り続ける**。サーバ側の
      // 中継も `gui.brake or auto.brake` にしてあるので、解除が届くまでの
      // 数十 ms にブレーキが効かない時間は生まれない。
      const engaged = ui.auto?.engaged ?? false
      if (engaged && (keyActive || padActive || brake)) {
        if (now - lastAutoCancel.current >= TAKE_CONTROL_MS) {
          lastAutoCancel.current = now
          ch.setAuto({ engaged: false })
          ui.set({ autoOffReason: brake ? 'ブレーキ操作で解除' : '手動操作で解除' })
        }
      }

      if (!ui.armRequested) {
        resetManual()
        cmdOut.speed = 0
        cmdOut.steer = 0
        cmdOut.active = false
        cmdOut.torqueMode = false
        cmdOut.torque = 0
        cmdOut.auto = false
        live.armRemainingMs = 0
        if (ui.deadman) ui.set({ deadman: false, inputSource: 'none' })

        // ── 未 ARM でも灯火・ホーン・パッシングは通す──
        //
        // 「停めたまま前照灯だけ点ける」「人をどかすのにクラクションを鳴らす」は
        // ARM とは無関係の操作なので、ARM を条件にしていたのは筋が悪かった。
        //
        // **速度・舵は 0、`arm=false`、`mode=DISARM` で送る。** `command_from_cmd` は
        // 灯火/ホーンのビットを `arm` とは独立に組む（`msgs/convert.py`）ので、
        // ARM ビットを立てないまま灯火だけが STM32 に届く。モータが回る余地は無い。
        //
        // ⚠ **出したいものが無ければ送らない。** 送り続けると操縦権を握りっぱなしになり、
        // 2枚目のタブや別の PC が操縦できなくなる（操縦権は同時に1人）。
        // 消灯に戻したあとは送信が止まり、`DEADMAN_MS` で io_node が DISARM_COMMAND
        //（`flags=0`）に落とすので、灯火はそのまま消える。
        const wantAux = horn || passing || ui.lightMode !== LIGHT_OFF ||
          ui.winkerLeftRequested || ui.winkerRightRequested
        if (!wantAux) {
          mirrorAux(false, false, false)
          return
        }
        if (!ui.hasControl) {
          if (now - lastTakeMs >= TAKE_CONTROL_MS) {
            ch.takeControl()
            lastTakeMs = now
          }
          mirrorAux(false, false, false)
          return
        }
        const c: Omit<CmdOut, 'type'> = {
          mode: 0, // DISARM
          arm: false,
          brake: false,
          horn,
          light_mode: ui.lightMode,
          passing,
          speed: 0,
          steer: 0,
          accel_limit: ACCEL_LIMIT,
          steer_rate_limit: STEER_RATE_LIMIT,
          brake_torque: s.brakeTorque,
          torque_mode: false,
          target_torque: 0,
          auto_stop: s.autoStop,
          side_brake: false,
          // ウィンカーは純粋な灯火系（後輪モータの励磁と無関係）なので、
          // side_brake と違い未 ARM でも送る（灯火・ホーンと同じ理由）
          winker_left: ui.winkerLeftRequested,
          winker_right: ui.winkerRightRequested,
          winker_mode: ui.winkerMode,
          launch: false,
        }
        if (!tx.due(now, c)) return
        // ブレーキ・サイドブレーキは未 ARM では意味を持たない（モータが励磁されていない）
        mirrorAux(false, horn, passing)
        ch.cmd(c)
        return
      }

      // ⚠ **自律走行中は無操作タイムアウトを掛けない。** 人が何も触らないのが
      // 正常な状態なので、20秒で勝手に止まったら自律走行にならない。
      // 代わりの受け皿は従来どおり全部生きている（Esc / E-STOP /
      // タブが背後に回る / 送信が止まれば `DEADMAN_MS` で DISARM）。
      // **画面を見ていられる間だけ走る**という前提は変わっていない
      if (engaged) lastActivity.current = now

      const idle = now - lastActivity.current
      if (idle > s.armIdleTimeoutMs) {
        ui.disarm(`${s.armIdleTimeoutMs / 1000}秒 無操作で自動解除`)
        mirrorAux(false, false, false)
        return
      }
      live.armRemainingMs = s.armIdleTimeoutMs - idle

      // 操縦権が無ければ取りに行く（サーバが拒否したら denied が来る）。
      // **その間、手動側の積分は 0 に戻す。** 残すと、取れた瞬間に前の速度・舵から再開する
      if (!ui.hasControl) {
        if (now - lastTakeMs >= TAKE_CONTROL_MS) {
          ch.takeControl()
          lastTakeMs = now
        }
        resetManual()
        cmdOut.speed = 0
        cmdOut.steer = 0
        cmdOut.torque = 0
        cmdOut.active = false
        cmdOut.auto = false
        if (ui.deadman) ui.set({ deadman: false, inputSource: 'none' })
        mirrorAux(false, false, false)
        return
      }

      // ── 自律走行（AUTO） ──
      //
      // **速度と舵は送らない。** `mode=2` で送ると telemetry_node が
      // `auto/cmd`（planning_node）の値に差し替える。GUI が 0 を送っているのは
      // 「ここは planner が埋める」という意思表示で、届かなければ 0 のまま
      // ＝ 止まる方向に転ぶ（`raspi/nodes/telemetry_node.py` の `_merge_auto`）。
      //
      // 灯火・ホーン・パッシング・`auto_stop` は手動と同じに効く。自律走行中に
      // 前照灯の操作だけ効かなくなる理由が無い。
      if (engaged) {
        // 手動側の積分は 0 に戻しておく。**解除した瞬間に前の速度から再開しない**
        resetManual()

        // 計器には**実際に車へ向かっている値**（planner の指令）を出す。
        // GUI が送っている生の 0 を出すと、走っているのに計器が 0 のままになる
        const a = live.auto
        cmdOut.speed = a?.target_speed ?? 0
        cmdOut.steer = a?.target_steer ?? 0
        cmdOut.active = true
        cmdOut.auto = true
        cmdOut.torqueMode = false
        cmdOut.torque = 0
        if (!ui.deadman || ui.inputSource !== 'auto') {
          ui.set({ deadman: true, inputSource: 'auto' })
        }

        const c: Omit<CmdOut, 'type'> = {
          mode: 2, // AUTO
          arm: true,
          // **人間のブレーキはそのまま通す**（解除の往復を待たない）
          brake,
          horn,
          light_mode: ui.lightMode,
          passing,
          speed: 0,
          steer: 0,
          accel_limit: ACCEL_LIMIT,
          steer_rate_limit: STEER_RATE_LIMIT,
          // L2 の踏み込み量（`brakeStrength`）で力を加減する。Space はフル（=1）
          brake_torque: s.brakeTorque * brakeStrength,
          // 自律走行はトルク直接指令を使わない。サーバ側でも落としてある
          torque_mode: false,
          target_torque: 0,
          auto_stop: s.autoStop,
          // **人間のサイドブレーキもそのまま通す**（brake と同じ理由）
          side_brake: ui.sideBrakeRequested,
          winker_left: ui.winkerLeftRequested,
          winker_right: ui.winkerRightRequested,
          winker_mode: ui.winkerMode,
          // 自律走行のローンチは planner が決める（サーバ側も GUI の値を使わない）
          launch: false,
        }
        if (!tx.due(now, c)) return
        mirrorAux(brake, horn, passing)
        ch.cmd(c)
        return
      }

      // ── 指令値を作る ──
      //
      // **入力元は「最後に動かした側」で固定する。** 「今どちらが動いているか」で
      // 選ぶと、パッドのトリガーを戻した瞬間だけキーボード側の惰行カーブに落ちて、
      // 離し方によって減速の効きが変わってしまう
      if (padActive) sourceRef.current = 'gamepad'
      else if (keyActive) sourceRef.current = 'keyboard'
      else if (!pad) sourceRef.current = 'keyboard'
      const source = sourceRef.current

      const inp: ManualInput = { source, fwd, rev, left, right, accelTrig, padSteer, brake }
      const stepped = stepManual(manual.current, inp, s, g, dt)
      // ── ローンチコントロール（手順は `useDriving.ts` 冒頭）──
      const ls = stepLaunch(launch.current, inp, s, g, live.vs?.speed ?? 0, live.vs?.launch_active ?? false)
      launch.current = { phase: ls.phase, seen: ls.seen }
      const out = finishManual(stepped, ls, brake, s, g)
      manual.current = out

      cmdOut.speed = out.speed
      cmdOut.steer = out.steer
      cmdOut.active = true
      cmdOut.auto = false
      cmdOut.torqueMode = g.torqueModeOn
      cmdOut.torque = out.torque
      if (!ui.deadman || ui.inputSource !== source) {
        ui.set({ deadman: true, inputSource: source })
      }

      // ── 送信（変化していればこのフレームで。同じ中身は間引く。`TxThrottle`）──
      const c: Omit<CmdOut, 'type'> = {
        mode: 1, // MANUAL
        arm: true,
        brake,
        horn,
        light_mode: ui.lightMode,
        passing,
        speed: out.speed,
        steer: out.steer,
        // **0（STM32 の既定に任せる）ではなく明示的に送る。** 操作感を作るのは
        // GUI 側ランプで、こちらはそれより速い＝通常は効かない保険
        accel_limit: ACCEL_LIMIT,
        steer_rate_limit: STEER_RATE_LIMIT,
        // **こちらは逆に 0 を送ってはいけない。** `brake_torque` の 0 は
        // 「未指定 ＝ STM32 の最大制動」を意味する。スライダの下限は 0 より上にしてある。
        // L2 の踏み込み量（`brakeStrength`）で力を加減する。L1/Space はフル（=1）で、
        // `_brake_torque_raw` は正の値を 0 に丸めないので軽く踏んでも「未指定」化けはしない
        brake_torque: s.brakeTorque * brakeStrength,
        // 立っている間 `speed` は無視され `target_torque` が駆動トルクとして直接掛かる
        // MT モードは `torqueModeOn===false` なので、ここは自動的に速度指令になる
        torque_mode: g.torqueModeOn,
        target_torque: out.torque,
        // 進行方向の超音波が 20cm 未満なら STM32 が単独で最大制動する。
        // **GUI は許可を出すだけで、判定にも制動にも関与しない**（二重制御にしない）
        auto_stop: s.autoStop,
        // `braking`等と違いトグル。`AuxPanel.tsx` の ON/OFF で切り替える
        side_brake: ui.sideBrakeRequested,
        // ウィンカーも `side_brake` と同じくトグル。灯火系なので未 ARM でも送る
        winker_left: ui.winkerLeftRequested,
        winker_right: ui.winkerRightRequested,
        winker_mode: ui.winkerMode,
        // ローンチコントロール。手順を踏んだ発進の間だけ立つ
        launch: ls.phase === 'go' && !g.torqueModeOn,
      }
      if (!tx.due(now, c)) return
      mirrorAux(brake, horn, passing, ls.phase)
      ch.cmd(c)
    }

    const onPadConnected = () => pads.onConnected(performance.now())
    const onPadDisconnected = (e: GamepadEvent) => {
      if (pads.onDisconnected(e.gamepad.index)) {
        notify('ゲームパッドが切断されました — 入力を中立にしました', 'bad')
      }
    }
    window.addEventListener('gamepadconnected', onPadConnected)
    window.addEventListener('gamepaddisconnected', onPadDisconnected)

    // rAF はタブが隠れると止まる ＝ 送信も止まる ＝ `DEADMAN_MS` でサーバが DISARM（安全側。`useRaf.ts`）
    const stop = rafLoop(tick)
    return () => {
      stop()
      window.removeEventListener('gamepadconnected', onPadConnected)
      window.removeEventListener('gamepaddisconnected', onPadDisconnected)
    }
  }, [ch, keys, lastActivity])
}
