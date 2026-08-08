/**
 * ラジコン入力（ゲームパッド + キーボード）→ 20Hz の `cmd` 送信。
 *
 * ## 方式：ARM ボタン保持 ＋ 無操作タイムアウト
 *
 * 以前は「Space を押している間だけ送る」デッドマン方式だったが、
 * **押しながら WASD を操作するのが難しい**ため、明示的な ARM に変えた。
 *
 * | | ARM | 速度 | 舵 | 停止 |
 * |---|---|---|---|---|
 * | キーボード | `Enter` または画面の ARM ボタン | W,↑ / S,↓ | A,← / D,→ | `Esc` |
 * | ゲームパッド | 同上 | R2 前進 / L2 後退 | 左スティック X | B/○ |
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
 * ## キーボードは押下時間でランプさせる
 *
 * キーは on/off の2値なので、そのまま最大速度に叩き込むと乗り物として扱えない。
 * 押している間だけ加速度を積分し、離したら戻す。
 */
import { useEffect, useRef } from 'react'
import { cmdOut, live } from '../bus/live'
import { ARM_IDLE_TIMEOUT_MS, UI_MAX_SPEED, UI_MAX_STEER, useUi } from '../store/ui'
import type { ControlChannel } from '../ws/control'

const TX_HZ = 20
const DT = 1 / TX_HZ

/** キーボードのランプ速度。実車の accel_limit とは別物（あくまで入力の整形） */
const KEY_ACCEL = 0.8 // m/s per second
const KEY_DECEL = 1.6
const KEY_STEER_RATE = 1.2 // rad/s（±60° まで約 0.9 秒）
const KEY_STEER_RETURN = 1.8
const DEADZONE = 0.12

const SPEED_KEYS_FWD = ['KeyW', 'ArrowUp']
const SPEED_KEYS_REV = ['KeyS', 'ArrowDown']
const STEER_KEYS_LEFT = ['KeyA', 'ArrowLeft']
const STEER_KEYS_RIGHT = ['KeyD', 'ArrowRight']
const DRIVING_KEYS = [
  ...SPEED_KEYS_FWD, ...SPEED_KEYS_REV, ...STEER_KEYS_LEFT, ...STEER_KEYS_RIGHT,
]

function dz(v: number): number {
  return Math.abs(v) < DEADZONE ? 0 : v
}

export function useDriving(ch: ControlChannel | null) {
  const keys = useRef(new Set<string>())
  const speed = useRef(0)
  const steer = useRef(0)
  const lastActivity = useRef(0)

  // ── キーボード ──
  useEffect(() => {
    const disarm = (why: string) => {
      keys.current.clear()
      const ui = useUi.getState()
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

  // ── 20Hz の送信ループ ──
  useEffect(() => {
    if (!ch) return
    const id = window.setInterval(() => {
      const ui = useUi.getState()
      const now = performance.now()
      const pad = navigator.getGamepads?.().find((p) => p && p.connected) ?? null

      // ゲームパッドの B/○ はいつでも E-STOP（ARM していなくても効く）
      if (pad?.buttons[1]?.pressed) {
        ch.estop()
        if (ui.armRequested) ui.set({ armRequested: false, disarmReason: 'パッドの B で E-STOP' })
        return
      }

      // ── 入力を読む ──
      const k = keys.current
      const fwd = SPEED_KEYS_FWD.some((c) => k.has(c))
      const rev = SPEED_KEYS_REV.some((c) => k.has(c))
      const left = STEER_KEYS_LEFT.some((c) => k.has(c))
      const right = STEER_KEYS_RIGHT.some((c) => k.has(c))
      const keyActive = fwd || rev || left || right

      const rt = pad?.buttons[7]?.value ?? 0
      const lt = pad?.buttons[6]?.value ?? 0
      const padSteer = dz(pad?.axes[0] ?? 0)
      const padActive = rt > 0.08 || lt > 0.08 || padSteer !== 0

      // **押しっぱなしも「操作中」に数える。** keydown だけを見ると、
      // W を握り続けているのにタイムアウトで切れる
      if (keyActive || padActive) lastActivity.current = now

      if (!ui.armRequested) {
        speed.current = 0
        steer.current = 0
        cmdOut.speed = 0
        cmdOut.steer = 0
        cmdOut.active = false
        live.armRemainingMs = 0
        if (ui.deadman) ui.set({ deadman: false, inputSource: 'none' })
        return
      }

      const idle = now - lastActivity.current
      if (idle > ARM_IDLE_TIMEOUT_MS) {
        ui.set({
          armRequested: false,
          disarmReason: `${ARM_IDLE_TIMEOUT_MS / 1000}秒 無操作で自動解除`,
        })
        return
      }
      live.armRemainingMs = ARM_IDLE_TIMEOUT_MS - idle

      // 操縦権が無ければ取りに行く（サーバが拒否したら denied が来る）
      if (!ui.hasControl) {
        ch.takeControl()
        return
      }

      // ── 指令値を作る ──
      let source: 'keyboard' | 'gamepad' = 'keyboard'
      if (padActive) {
        source = 'gamepad'
        // 左スティック左 (-1) が左旋回 (+steer)。反時計回りが正
        steer.current = -padSteer * UI_MAX_STEER
        speed.current = (rt - lt) * UI_MAX_SPEED
      } else {
        if (fwd) speed.current += KEY_ACCEL * DT
        else if (rev) speed.current -= KEY_ACCEL * DT
        else
          speed.current -=
            Math.sign(speed.current) * Math.min(Math.abs(speed.current), KEY_DECEL * DT)

        if (left) steer.current += KEY_STEER_RATE * DT
        else if (right) steer.current -= KEY_STEER_RATE * DT
        else
          steer.current -=
            Math.sign(steer.current) * Math.min(Math.abs(steer.current), KEY_STEER_RETURN * DT)
      }
      speed.current = Math.max(-UI_MAX_SPEED, Math.min(UI_MAX_SPEED, speed.current))
      steer.current = Math.max(-UI_MAX_STEER, Math.min(UI_MAX_STEER, steer.current))

      ch.cmd({
        mode: 1, // MANUAL
        arm: true,
        brake: false,
        horn: false,
        light: false,
        speed: speed.current,
        steer: steer.current,
        // 0 は「STM32 の既定に任せる」の意。ここで固めない
        accel_limit: 0,
        steer_rate_limit: 0,
      })
      cmdOut.speed = speed.current
      cmdOut.steer = steer.current
      cmdOut.active = true
      if (!ui.deadman || ui.inputSource !== source) {
        ui.set({ deadman: true, inputSource: source })
      }
    }, 1000 / TX_HZ)

    return () => window.clearInterval(id)
  }, [ch])
}
