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
 * **灯火・ホーン・パッシングは未 ARM でも効く**（2026-08-11 に変更）。
 * ARM していない間は `mode=DISARM` / `arm=false` / 速度・舵 0 の cmd を送り、
 * 灯火とホーンのビットだけを載せる。`command_from_cmd`（`msgs/convert.py`）は
 * 灯火/ホーンを `arm` とは独立に組むので、**モータが回る余地は無いまま灯火だけ届く**。
 *
 * ⚠ 出したいものが無い（消灯・ホーン離し）ときは送らない。送り続けると
 * **操縦権を握りっぱなしになり、2枚目のタブが操縦できなくなる**（操縦権は同時に1人）。
 *
 * ⚠ **ブレーキだけは未 ARM で送らない。** モータが励磁されていないので効かず、
 * 「押しているのに効かない」体験になるだけ。
 *
 * ## この方式で失ったもの・残したもの
 *
 * **失った**: 「手を離した瞬間に止まる」性質。無操作でも `settings.armIdleTimeoutMs`
 * （既定 20秒。設定パネルで変更可）は armed のまま（速度指令は 0 に落ちるので進みはしないが、
 * モータは励磁されたまま）。**既定 20秒はこの保険がほぼ効かない長さ**なので、
 * 実質的な停止手段は下の即時系だと考えること。
 *
 * **残した**（ここは操作性と衝突しないので削らない）:
 *
 * - `Esc` / E-STOP ボタン / パッドの B … 即 DISARM。**権限も条件も要らない**
 * - ウィンドウのフォーカス喪失・タブが背後に回る … 即 DISARM
 * - 送信が止まれば 150ms でサーバが DISARM、さらに io_node が 150ms で DISARM
 * - 無操作 `settings.armIdleTimeoutMs` … 自動 DISARM（放置した場合の最後の受け皿）
 * - `settings.autoStop`（v0.7、既定 ON）… **進行方向の超音波が 20cm 未満なら STM32 が単独で
 *   最大制動**。GUI は `auto_stop` を立てるだけで判定には関与しない。DISARM はしないので、
 *   離れれば自動的に解除されて再び走れる（**停止手段ではなく衝突緩和**として数えること）
 *
 * つまり「ブラウザが固まる」「Wi-Fi が切れる」「PC を閉じる」は従来どおり停止する。
 * 弱くなったのは**人間が意図して手を離した場合だけ**。
 *
 * ## 自律走行（AUTO）でも、このループが車を生かしている
 *
 * 自動運転タブで engage しても、**指令を 50Hz で送り続けているのはここ**。
 * 送るのは `mode=2` と補機だけで、速度・舵は telemetry_node が planning_node の
 * `auto/cmd` に差し替える。上のデッドマン（ARM 保持・フォーカス喪失・150ms 途絶）が
 * そのまま自律走行の停止手段になるのはこのため。**planning_node は arm を立てられない。**
 *
 * 手動と違うのは2点だけ:
 *
 * - **人が舵・スロットル・ブレーキを触ったら自律を解除する**（クルコンと同じ作法）
 * - **無操作タイムアウトを掛けない**（人が触らないのが正常な状態なので）
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
 * 今は GUI から明示的に `ACCEL_LIMIT` / `STEER_RATE_LIMIT` を送る。値は設定パネルの
 * `accel`/`steerRate` より意図的に速くしてあり（ユーザー設定にはしない）、
 * 通常走行では GUI のランプが支配的になる。STM32 側は
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
 * 「押した→無反応→急に出る」になる。押した瞬間に `settings.kickSpeed` へ跳ばしてから積分する。
 * **`kickSpeed` は設定パネルで、実車が実際に転がり始める指令値に合わせて調整すること**
 * （DriveBar の実測速度を見ながら詰める。低速では `wheel_speed` が使えないので
 * `speed` の生値と目視で判断する）。
 */
import { useEffect, useRef } from 'react'
import { cmdOut, live } from '../bus/live'
import {
  ACCEL_SAFETY_LIMIT, LIGHT_CYCLE, LIGHT_OFF, STEER_RATE_SAFETY_LIMIT, useUi,
} from '../store/ui'
import type { ControlChannel } from '../ws/control'

/** 送信レート。telemetry_node の `CMD_PUB_HZ` に合わせる */
const TX_HZ = 50
const TX_INTERVAL_MS = 1000 / TX_HZ
/** 1フレームの積分幅の上限。タブ復帰やカクつきで一気に飛ぶのを防ぐ */
const DT_MAX = 0.05

// ── 速度・舵の応答は「設定パネル」から調整する ──────────────────────
//
// 加速・惰行・ブレーキ・発進キック・切り込み/戻し/切り返し速度は
// `store/ui.ts` の `DrivingSettings`（`ui.settings`）に移した。
// ここでは毎フレーム `useUi.getState().settings` から読む。

// ── STM32 に渡すレートリミット ───────────────────────────────────
/** GUI のランプ（設定パネルの `accel`/`brake`）より速い値。**通常は効かず、保険として残る**。
 * ユーザー設定にはしない（`ACCEL_SAFETY_LIMIT` 参照） */
const ACCEL_LIMIT = ACCEL_SAFETY_LIMIT // m/s²
const STEER_RATE_LIMIT = STEER_RATE_SAFETY_LIMIT // rad/s

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
  const torque = useRef(0)
  const lastActivity = useRef(0)
  const sourceRef = useRef<'keyboard' | 'gamepad'>('keyboard')
  /** 自律解除を投げた時刻。rAF ごとに投げると 60発/秒になる（`TAKE_CONTROL_MS` と同じ理由） */
  const lastAutoCancel = useRef(0)

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
      const s = ui.settings
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
      // **ブーストは「最高速度そのもの」。** 倍率は持たない（`cruiseScale` の注記参照）
      const maxSpeed = boost ? s.maxSpeed : s.maxSpeed * s.cruiseScale

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

      // ── 自律走行中に人が操作したら、自律を解除して手動に戻す ──
      //
      // クルーズコントロールと同じ作法。**「人の操作が勝つ」を無条件にする**。
      // どちらが舵を出しているのか分からない状態を作らないための解除であって、
      // 停止手段ではない（止めるのは Esc / E-STOP / ブレーキ）。
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
        speed.current = 0
        steer.current = 0
        torque.current = 0
        cmdOut.speed = 0
        cmdOut.steer = 0
        cmdOut.active = false
        cmdOut.torqueMode = false
        cmdOut.torque = 0
        cmdOut.auto = false
        live.armRemainingMs = 0
        if (ui.deadman) ui.set({ deadman: false, inputSource: 'none' })

        // ── 未 ARM でも灯火・ホーン・パッシングは通す（2026-08-11） ──
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
        // 消灯に戻したあとは送信が止まり、150ms で io_node が DISARM_COMMAND
        //（`flags=0`）に落とすので、灯火はそのまま消える。
        const wantAux = horn || passing || ui.lightMode !== LIGHT_OFF
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
        if (now - lastTxMs < TX_INTERVAL_MS) return
        lastTxMs = now
        // ブレーキは未 ARM では意味を持たない（モータが励磁されていない）
        mirrorAux(false, horn, passing)
        ch.cmd({
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
        })
        return
      }

      // ⚠ **自律走行中は無操作タイムアウトを掛けない。** 人が何も触らないのが
      // 正常な状態なので、20秒で勝手に止まったら自律走行にならない。
      // 代わりの受け皿は従来どおり全部生きている（Esc / E-STOP / フォーカス喪失 /
      // タブが背後に回る / 送信が止まれば 150ms で DISARM）。
      // **画面を見ていられる間だけ走る**という前提は変わっていない
      if (engaged) lastActivity.current = now

      const idle = now - lastActivity.current
      if (idle > s.armIdleTimeoutMs) {
        ui.set({
          armRequested: false,
          disarmReason: `${s.armIdleTimeoutMs / 1000}秒 無操作で自動解除`,
        })
        mirrorAux(false, false, false)
        return
      }
      live.armRemainingMs = s.armIdleTimeoutMs - idle

      // 操縦権が無ければ取りに行く（サーバが拒否したら denied が来る）
      if (!ui.hasControl) {
        if (now - lastTakeMs >= TAKE_CONTROL_MS) {
          ch.takeControl()
          lastTakeMs = now
        }
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
        speed.current = 0
        steer.current = 0
        torque.current = 0

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

        if (now - lastTxMs < TX_INTERVAL_MS) return
        lastTxMs = now
        mirrorAux(brake, horn, passing)
        ch.cmd({
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
          brake_torque: s.brakeTorque,
          // 自律走行はトルク直接指令を使わない。サーバ側でも落としてある
          torque_mode: false,
          target_torque: 0,
          auto_stop: s.autoStop,
        })
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
        steer.current = -expo(padSteer, STEER_EXPO) * s.maxSteer
        if (s.torqueMode) {
          // トルクモード（v0.6）も同じ理由でランプなし。速度指令は使わないので 0
          torque.current = expo(rt - lt, THROTTLE_EXPO) * s.driveTorque
          speed.current = 0
        } else {
          speed.current = expo(rt - lt, THROTTLE_EXPO) * maxSpeed
          torque.current = 0
        }
      } else {
        const dir = fwd ? 1 : rev ? -1 : 0
        if (s.torqueMode) {
          // **トルクモードはランプを挟まない。** 0.1N・m 程度と値域が小さく、
          // 速度用の加速度ランプ（m/s²）はそのままでは単位が合わない。
          // 押している間だけ `driveTorque` をそのまま出す（v0.6）
          torque.current = dir * s.driveTorque
          speed.current = 0
        } else {
          // ── 速度：押している間は加速、逆キーはブレーキ ──
          if (dir === 0) {
            speed.current = approach(speed.current, 0, s.coast, dt)
          } else if (speed.current * dir < 0) {
            // 進行方向と逆を押している ＝ ブレーキ。0 を行き過ぎたら 0 で受け止め、
            // 次のフレームからキック＋加速で逆走に移る
            const next = speed.current + dir * s.brake * dt
            speed.current = next * dir > 0 ? 0 : next
          } else if (Math.abs(speed.current) < s.kickSpeed) {
            speed.current = dir * s.kickSpeed // 発進キック
          } else {
            speed.current += dir * s.accel * dt
          }
          torque.current = 0
        }

        // ── 舵：切り込みより戻しを速く、切り返しはさらに速く ──（速度/トルク共通）
        const sdir = left ? 1 : right ? -1 : 0
        if (sdir === 0) {
          steer.current = approach(steer.current, 0, s.steerReturn, dt)
        } else {
          const rate = steer.current * sdir < 0 ? s.steerCounter : s.steerRate
          steer.current = approach(steer.current, sdir * s.maxSteer, rate, dt)
        }
      }
      // **ブレーキ中は速度・トルク指令を 0 に落とす。**
      // STM32 は `brake` の間 `target_speed`/`target_torque` を無視し、離すと 0 から
      // `accel_limit` に従って加速し直す。GUI 側のランプを走らせたままにすると、
      // 「画面は 0.4 m/s と出ているのに車は止まっている」という嘘の表示になる。
      // 舵はブレーキ中も生かす（曲がりながら止めたい場面がある）
      if (brake) {
        speed.current = 0
        torque.current = 0
      }

      speed.current = Math.max(-maxSpeed, Math.min(maxSpeed, speed.current))
      torque.current = Math.max(-s.driveTorque, Math.min(s.driveTorque, torque.current))
      steer.current = Math.max(-s.maxSteer, Math.min(s.maxSteer, steer.current))

      cmdOut.speed = speed.current
      cmdOut.steer = steer.current
      cmdOut.active = true
      cmdOut.auto = false
      cmdOut.torqueMode = s.torqueMode
      cmdOut.torque = torque.current
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
        brake_torque: s.brakeTorque,
        // v0.6: 立っている間 `speed` は無視され `target_torque` が駆動トルクとして直接掛かる
        torque_mode: s.torqueMode,
        target_torque: torque.current,
        // v0.7: 進行方向の超音波が 20cm 未満なら STM32 が単独で最大制動する。
        // **GUI は許可を出すだけで、判定にも制動にも関与しない**（二重制御にしない）
        auto_stop: s.autoStop,
      })
    }

    raf = requestAnimationFrame(tick)
    return () => cancelAnimationFrame(raf)
  }, [ch])
}
