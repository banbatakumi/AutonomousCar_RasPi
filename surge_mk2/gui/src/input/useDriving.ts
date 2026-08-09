/**
 * ラジコン入力（ゲームパッド + キーボード）→ 50Hz の `cmd` 送信。
 *
 * ## 方式：ARM ボタン保持 ＋ 無操作タイムアウト
 *
 * 以前は「Space を押している間だけ送る」デッドマン方式だったが、
 * **押しながら WASD を操作するのが難しい**ため、明示的な ARM に変えた。
 *
 * | | ARM | 速度 | 舵 | ブースト | 停止 |
 * |---|---|---|---|---|---|
 * | キーボード | `Enter` または画面の ARM ボタン | W,↑ / S,↓ | A,← / D,→ | `Shift` | `Esc` |
 * | ゲームパッド | 同上 | R2 前進 / L2 後退 | 左スティック X | R1 | B/○ |
 *
 * 補機（v0.5 で追加）
 *
 * | | ブレーキ | クラクション | パッシング | 灯火切替 |
 * |---|---|---|---|---|
 * | キーボード | `Space` | `H` | `P` | `L`（消灯→DAY→NORMAL→…） |
 * | ゲームパッド | L1 | A/× | X/□ | Y/△ |
 *
 * **灯火切替以外はすべて「押している間だけ」。** STM32 側の意味論
 * （`horn` / `passing` は立てている間ずっと効く）に素直に対応させてある。
 * GUI 側でタイマーを持つと、DISARM やタブ切替と競合したときに消し忘れが起きる。
 *
 * ⚠ **補機は ARM 保持中しか効かない。** `cmd` を送るのが armed のときだけで、
 * 途絶すれば io_node が DISARM（`flags = 0`）に落とすため、灯火も消える。
 * 「停めたまま前照灯だけ点ける」には cmd の送信条件そのものを変える必要があり、
 * それは「送らないことが停止を意味する」設計に手を入れることになるので今はしない。
 *
 * ## この方式で失ったもの・残したもの
 *
 * **失った**: 「手を離した瞬間に止まる」性質。無操作でも `ARM_IDLE_TIMEOUT_MS`
 * （現在 20秒）は armed のまま（速度指令は 0 に落ちるので進みはしないが、
 * モータは励磁されたまま）。**20秒はこの保険がほぼ効かない長さ**なので、
 * 実質的な停止手段は下の即時系だと考えること。
 *
 * **残した**（ここは操作性と衝突しないので削らない）:
 *
 * - `Esc` / E-STOP ボタン / パッドの B … 即 DISARM。**権限も条件も要らない**
 * - ウィンドウのフォーカス喪失・タブが背後に回る … 即 DISARM
 * - 送信が止まれば 150ms でサーバが DISARM、さらに io_node が 150ms で DISARM
 * - 無操作 `ARM_IDLE_TIMEOUT_MS` … 自動 DISARM（放置した場合の最後の受け皿）
 *
 * つまり「ブラウザが固まる」「Wi-Fi が切れる」「PC を閉じる」は従来どおり停止する。
 * 弱くなったのは**人間が意図して手を離した場合だけ**。
 *
 * ## 応答設計（2026-08-09 改訂）
 *
 * ### 1. ループは rAF、送信は 50Hz
 *
 * 以前は `setInterval` の 20Hz で積分と送信を兼ねていた。下流は
 * telemetry_node が 50Hz、io_node が 100Hz で回っているので、
 * **20Hz だった GUI だけがボトルネック**だった。積分は rAF（≈60Hz、実 dt）で回し、
 * 送信は 50Hz に間引く（telemetry_node の publish レートに合わせる。それ以上出しても
 * 途中で捨てられるだけ）。パッドのスティックも rAF で読むのでカクつかない。
 *
 * rAF はタブが隠れると止まる ＝ 送信も止まる ＝ 150ms でサーバが DISARM。
 * **止まる方向に転ぶので安全側**（`visibilitychange` の即 DISARM と二重）。
 *
 * ### 2. レートリミットは「GUI で作り、STM32 は保険」に一元化
 *
 * 以前は `accel_limit=0`（STM32 の既定に任せる）を送っていたので、GUI 側のランプと
 * STM32 側のリミットが二重にかかり、**どちらが操作感を決めているのか分からなかった**。
 * 今は GUI から明示的に `ACCEL_LIMIT` / `STEER_RATE_LIMIT` を送る。値は GUI のランプより
 * わざと速くしてあり、通常走行では GUI のランプが支配的になる。STM32 側は
 * 「指令が飛んだ／GUI が壊れた」ときにメカを守る保険として残る。
 *
 * ### 3. 逆キーはブレーキ
 *
 * 前進中の S は「後退を積分し始める」のではなく**強い減速**。0 を跨いだら後退に移る。
 * 止めたいときに止まらないのが一番怖い。
 *
 * ### 4. 発進キック
 *
 * 停止からじわじわ指令を上げると、STM32 の速度 PI ループが動き出すまで無駄時間が出て
 * 「押した→無反応→急に出る」になる。押した瞬間に `KICK_SPEED` へ跳ばしてから積分する。
 * **`KICK_SPEED` は実車が実際に転がり始める指令値に合わせて調整すること**
 * （DriveBar の実測速度を見ながら詰める。低速では `wheel_speed` が使えないので
 * `speed` の生値と目視で判断する）。
 */
import { useEffect, useRef } from 'react'
import { cmdOut, live } from '../bus/live'
import {
  ARM_IDLE_TIMEOUT_MS, BOOST_SCALE, CRUISE_SCALE, LIGHT_CYCLE, UI_MAX_SPEED, UI_MAX_STEER,
  useUi,
} from '../store/ui'
import type { ControlChannel } from '../ws/control'

/** 送信レート。telemetry_node の `CMD_PUB_HZ` に合わせる */
const TX_HZ = 50
const TX_INTERVAL_MS = 1000 / TX_HZ
/** 1フレームの積分幅の上限。タブ復帰やカクつきで一気に飛ぶのを防ぐ */
const DT_MAX = 0.05

// ── 速度の応答 [m/s²] ────────────────────────────────────────────
/** 押している間の加速。0→1.0 m/s を 0.5 秒 */
const ACCEL = 2.0
/** キーを離したときの惰行減速 */
const COAST = 2.5
/** **逆キーを押したときのブレーキ。** 加速より強くする */
const BRAKE = 5.0
/**
 * 発進キック [m/s]。停止から押した瞬間にここまで跳ばす。
 * **実車が転がり始める値に合わせて調整すること**（大きすぎると飛び出す）
 */
const KICK_SPEED = 0.12

// ── 舵の応答 [rad/s] ─────────────────────────────────────────────
/** 押している間の切り込み。60° まで 0.40 秒 */
const STEER_RATE = 2.6
/** **離したときのセンター戻り。切り込みより速くする**（直進が安定する） */
const STEER_RETURN = 5.2
/** 逆キーでの切り戻し。センターを跨ぐまではこの速さ、跨いだら `STEER_RATE` */
const STEER_COUNTER = 5.2

// ── STM32 に渡すレートリミット ───────────────────────────────────
/** GUI のランプ（`ACCEL`/`BRAKE`）より速い値。**通常は効かず、保険として残る** */
const ACCEL_LIMIT = 6.0 // m/s²
const STEER_RATE_LIMIT = 7.0 // rad/s

// ── ゲームパッド ─────────────────────────────────────────────────
const STICK_DZ = 0.12
const TRIGGER_DZ = 0.06
/** 中央付近を緩くする度合い（0=リニア, 1=完全3乗）。舵は強め、スロットルは控えめ */
const STEER_EXPO = 0.45
const THROTTLE_EXPO = 0.25

const SPEED_KEYS_FWD = ['KeyW', 'ArrowUp']
const SPEED_KEYS_REV = ['KeyS', 'ArrowDown']
const STEER_KEYS_LEFT = ['KeyA', 'ArrowLeft']
const STEER_KEYS_RIGHT = ['KeyD', 'ArrowRight']
const BOOST_KEYS = ['ShiftLeft', 'ShiftRight']
const BRAKE_KEY = 'Space'
const HORN_KEY = 'KeyH'
const PASSING_KEY = 'KeyP'
const LIGHT_KEY = 'KeyL'
const DRIVING_KEYS = [
  ...SPEED_KEYS_FWD, ...SPEED_KEYS_REV, ...STEER_KEYS_LEFT, ...STEER_KEYS_RIGHT,
  BRAKE_KEY, HORN_KEY, PASSING_KEY,
]

// ── ゲームパッドのボタン番号（standard mapping） ──
/** A/× — クラクション */
const PAD_HORN = 0
/** X/□ — パッシング */
const PAD_PASSING = 2
/** Y/△ — 灯火モードを1つ進める（**押した瞬間だけ**） */
const PAD_LIGHT = 3
/** L1 — ブレーキ */
const PAD_BRAKE = 4

/** 操縦権の再要求はこの間隔まで。rAF ごとに投げると 60発/秒になる */
const TAKE_CONTROL_MS = 250
/** E-STOP をパッドで押しっぱなしにしたときの再送間隔 */
const ESTOP_REPEAT_MS = 100

/** デッドゾーンを**切り落とすのではなく再スケールする**。段差が出ない */
function dz(v: number, zone: number): number {
  const a = Math.abs(v)
  if (a < zone) return 0
  return Math.sign(v) * ((a - zone) / (1 - zone))
}

/** 中央付近を緩く、端は素直に。`e=0` でリニア */
function expo(v: number, e: number): number {
  return (1 - e) * v + e * v * v * v
}

function approach(cur: number, target: number, rate: number, dt: number): number {
  const d = rate * dt
  if (Math.abs(target - cur) <= d) return target
  return cur + Math.sign(target - cur) * d
}

export function useDriving(ch: ControlChannel | null) {
  const keys = useRef(new Set<string>())
  const speed = useRef(0)
  const steer = useRef(0)
  const lastActivity = useRef(0)
  const sourceRef = useRef<'keyboard' | 'gamepad'>('keyboard')

  // ── キーボード ──
  useEffect(() => {
    const disarm = (why: string) => {
      keys.current.clear()
      const ui = useUi.getState()
      if (ui.boost) ui.set({ boost: false })
      if (ui.armRequested) ui.set({ armRequested: false, disarmReason: why })
    }

    const down = (e: KeyboardEvent) => {
      if (e.repeat) return
      const ui = useUi.getState()
      if (e.code === 'Escape') {
        ch?.estop()
        disarm('Esc で E-STOP')
        return
      }
      if (e.code === 'Enter') {
        // ARM のトグル。**入れるのも切るのも同じキー**
        if (ui.armRequested) disarm('Enter で解除')
        else {
          lastActivity.current = performance.now()
          ui.set({ armRequested: true, disarmReason: '' })
        }
        return
      }
      if (BOOST_KEYS.includes(e.code)) {
        keys.current.add(e.code)
        return
      }
      if (e.code === LIGHT_KEY) {
        // **押した瞬間に1つ進めるだけ。** 押しっぱなしで回り続けると
        // どのモードで手を離したのか分からなくなる（`e.repeat` は上で弾いてある）
        const i = LIGHT_CYCLE.indexOf(ui.lightMode)
        ui.set({ lightMode: LIGHT_CYCLE[(i + 1) % LIGHT_CYCLE.length] })
        return
      }
      if (DRIVING_KEYS.includes(e.code)) {
        e.preventDefault() // 矢印キーでページがスクロールしないように
        keys.current.add(e.code)
      }
    }
    const up = (e: KeyboardEvent) => keys.current.delete(e.code)

    // **フォーカスを失ったら即 DISARM。** 押しっぱなし扱いのまま裏に回ると、
    // 戻ってきたときに突然走り出す
    const onBlur = () => disarm('ウィンドウのフォーカスが外れた')
    const onHide = () => {
      if (document.hidden) disarm('タブが背後に回った')
    }

    window.addEventListener('keydown', down)
    window.addEventListener('keyup', up)
    window.addEventListener('blur', onBlur)
    document.addEventListener('visibilitychange', onHide)
    return () => {
      window.removeEventListener('keydown', down)
      window.removeEventListener('keyup', up)
      window.removeEventListener('blur', onBlur)
      document.removeEventListener('visibilitychange', onHide)
    }
  }, [ch])

  // ── rAF で積分し、50Hz で送る ──
  useEffect(() => {
    if (!ch) return
    let raf = 0
    let prevMs = performance.now()
    let lastTxMs = 0
    let lastTakeMs = 0
    let lastEstopMs = 0
    let padEstopWas = false
    let padLightWas = false

    /**
     * 補機の押下状態を store に写す。**変化したときだけ**書く（毎フレーム書くと
     * 60Hz で再レンダリングが走る）。指令を送らない経路では必ず false を写すこと。
     * **画面に「鳴っている」と出したまま実際は送っていない**のが一番混乱する。
     */
    const mirrorAux = (brake: boolean, horn: boolean, passing: boolean) => {
      const u = useUi.getState()
      if (u.braking !== brake || u.horning !== horn || u.passing !== passing) {
        u.set({ braking: brake, horning: horn, passing })
      }
    }

    const tick = () => {
      raf = requestAnimationFrame(tick)

      const now = performance.now()
      const dt = Math.min((now - prevMs) / 1000, DT_MAX)
      prevMs = now
      if (dt <= 0) return

      const ui = useUi.getState()
      const pad = navigator.getGamepads?.().find((p) => p && p.connected) ?? null

      // ゲームパッドの B/○ はいつでも E-STOP（ARM していなくても効く）
      const padEstop = pad?.buttons[1]?.pressed ?? false
      if (padEstop) {
        // 押した瞬間に1発、握り続けている間は 100ms ごとに再送
        if (!padEstopWas || now - lastEstopMs >= ESTOP_REPEAT_MS) {
          ch.estop()
          lastEstopMs = now
        }
        padEstopWas = true
        if (ui.armRequested) ui.set({ armRequested: false, disarmReason: 'パッドの B で E-STOP' })
        mirrorAux(false, false, false)
        return
      }
      padEstopWas = false

      // ── 入力を読む ──
      const k = keys.current
      const fwd = SPEED_KEYS_FWD.some((c) => k.has(c))
      const rev = SPEED_KEYS_REV.some((c) => k.has(c))
      const left = STEER_KEYS_LEFT.some((c) => k.has(c))
      const right = STEER_KEYS_RIGHT.some((c) => k.has(c))
      const keyActive = fwd || rev || left || right
      const keyBoost = BOOST_KEYS.some((c) => k.has(c))

      const rt = dz(pad?.buttons[7]?.value ?? 0, TRIGGER_DZ)
      const lt = dz(pad?.buttons[6]?.value ?? 0, TRIGGER_DZ)
      const padSteer = dz(pad?.axes[0] ?? 0, STICK_DZ)
      const padBoost = pad?.buttons[5]?.pressed ?? false
      const padActive = rt > 0 || lt > 0 || padSteer !== 0

      const boost = keyBoost || padBoost
      if (ui.boost !== boost) ui.set({ boost })
      const maxSpeed = UI_MAX_SPEED * (boost ? BOOST_SCALE : CRUISE_SCALE)

      // ── 補機（すべて「押している間だけ」） ──
      const brake = k.has(BRAKE_KEY) || (pad?.buttons[PAD_BRAKE]?.pressed ?? false)
      const horn = k.has(HORN_KEY) || (pad?.buttons[PAD_HORN]?.pressed ?? false)
      const passing = k.has(PASSING_KEY) || (pad?.buttons[PAD_PASSING]?.pressed ?? false)

      // 灯火切替だけはトグル。**押した瞬間のエッジで1つ進める**
      // （rAF で読むので、キーの `e.repeat` に相当する処理を自前で持つ）
      const padLight = pad?.buttons[PAD_LIGHT]?.pressed ?? false
      if (padLight && !padLightWas) {
        const i = LIGHT_CYCLE.indexOf(ui.lightMode)
        ui.set({ lightMode: LIGHT_CYCLE[(i + 1) % LIGHT_CYCLE.length] })
      }
      padLightWas = padLight

      // **押しっぱなしも「操作中」に数える。** keydown だけを見ると、
      // W を握り続けているのにタイムアウトで切れる。
      // 補機も操作のうち（人間が目の前にいる証拠なので ARM を維持してよい）
      if (keyActive || padActive || brake || horn || passing) lastActivity.current = now

      if (!ui.armRequested) {
        speed.current = 0
        steer.current = 0
        cmdOut.speed = 0
        cmdOut.steer = 0
        cmdOut.active = false
        live.armRemainingMs = 0
        if (ui.deadman) ui.set({ deadman: false, inputSource: 'none' })
        mirrorAux(false, false, false)
        return
      }

      const idle = now - lastActivity.current
      if (idle > ARM_IDLE_TIMEOUT_MS) {
        ui.set({
          armRequested: false,
          disarmReason: `${ARM_IDLE_TIMEOUT_MS / 1000}秒 無操作で自動解除`,
        })
        mirrorAux(false, false, false)
        return
      }
      live.armRemainingMs = ARM_IDLE_TIMEOUT_MS - idle

      // 操縦権が無ければ取りに行く（サーバが拒否したら denied が来る）
      if (!ui.hasControl) {
        if (now - lastTakeMs >= TAKE_CONTROL_MS) {
          ch.takeControl()
          lastTakeMs = now
        }
        mirrorAux(false, false, false)
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

      if (source === 'gamepad') {
        // **アナログ入力はランプを挟まず、目標をそのまま出す。**
        // 人の指がすでにランプになっているので、二重に鈍らせない。
        // 急な踏み込みは STM32 の accel_limit / steer_rate_limit が受け持つ
        // 左スティック左 (-1) が左旋回 (+steer)。反時計回りが正
        steer.current = -expo(padSteer, STEER_EXPO) * UI_MAX_STEER
        speed.current = expo(rt - lt, THROTTLE_EXPO) * maxSpeed
      } else {
        // ── 速度：押している間は加速、逆キーはブレーキ ──
        const dir = fwd ? 1 : rev ? -1 : 0
        if (dir === 0) {
          speed.current = approach(speed.current, 0, COAST, dt)
        } else if (speed.current * dir < 0) {
          // 進行方向と逆を押している ＝ ブレーキ。0 を行き過ぎたら 0 で受け止め、
          // 次のフレームからキック＋加速で逆走に移る
          const next = speed.current + dir * BRAKE * dt
          speed.current = next * dir > 0 ? 0 : next
        } else if (Math.abs(speed.current) < KICK_SPEED) {
          speed.current = dir * KICK_SPEED // 発進キック
        } else {
          speed.current += dir * ACCEL * dt
        }

        // ── 舵：切り込みより戻しを速く、切り返しはさらに速く ──
        const sdir = left ? 1 : right ? -1 : 0
        if (sdir === 0) {
          steer.current = approach(steer.current, 0, STEER_RETURN, dt)
        } else {
          const rate = steer.current * sdir < 0 ? STEER_COUNTER : STEER_RATE
          steer.current = approach(steer.current, sdir * UI_MAX_STEER, rate, dt)
        }
      }
      // **ブレーキ中は速度指令を 0 に落とす。**
      // STM32 は `brake` の間 `target_speed` を無視し、離すと 0 から
      // `accel_limit` に従って加速し直す。GUI 側のランプを走らせたままにすると、
      // 「画面は 0.4 m/s と出ているのに車は止まっている」という嘘の表示になる。
      // 舵はブレーキ中も生かす（曲がりながら止めたい場面がある）
      if (brake) speed.current = 0

      speed.current = Math.max(-maxSpeed, Math.min(maxSpeed, speed.current))
      steer.current = Math.max(-UI_MAX_STEER, Math.min(UI_MAX_STEER, steer.current))

      cmdOut.speed = speed.current
      cmdOut.steer = steer.current
      cmdOut.active = true
      if (!ui.deadman || ui.inputSource !== source) {
        ui.set({ deadman: true, inputSource: source })
      }

      // ── 送信は 50Hz に間引く ──
      if (now - lastTxMs < TX_INTERVAL_MS) return
      lastTxMs = now
      mirrorAux(brake, horn, passing)
      ch.cmd({
        mode: 1, // MANUAL
        arm: true,
        brake,
        horn,
        light_mode: ui.lightMode,
        passing,
        speed: speed.current,
        steer: steer.current,
        // **0（STM32 の既定に任せる）ではなく明示的に送る。** 操作感を作るのは
        // 上の GUI 側ランプで、こちらはそれより速い＝通常は効かない保険
        accel_limit: ACCEL_LIMIT,
        steer_rate_limit: STEER_RATE_LIMIT,
        // **こちらは逆に 0 を送ってはいけない。** `brake_torque` の 0 は
        // 「未指定 ＝ STM32 の最大制動」を意味する。スライダの下限は 0 より上にしてある
        brake_torque: ui.brakeTorque,
      })
    }

    raf = requestAnimationFrame(tick)
    return () => cancelAnimationFrame(raf)
  }, [ch])
}
