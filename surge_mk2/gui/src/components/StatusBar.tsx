/**
 * ステータスバー — 層 B（常時周辺視）。**横目で追える情報だけ**を置く。
 *
 * ここに数字を増やすと読めなくなるので、載せるのは
 * 「状態が変わったら即座に気づきたいもの」に限る。
 */
import { useNumbers } from '../bus/live'
import { MODE_NAME, healthLevel, ms, rttLevel } from '../format'
import { BOOST_SCALE, CRUISE_SCALE, UI_MAX_SPEED, useUi } from '../store/ui'

export function StatusBar({ onEstop }: { onEstop: () => void }) {
  const n = useNumbers()
  const ui = useUi()
  const vs = n.vs
  const link = n.link

  const health = link?.health ?? 'INIT'
  const mode = MODE_NAME[vs?.mode ?? 0]

  // ラッチ系は health より目立たせる。**人間が物理操作しないと戻らない**ため
  const latches: string[] = []
  if (link?.estop_active) latches.push('E-STOP 発動中 — 車両のボタン2を押すまで解除されません')
  if (link?.drive_power_locked) latches.push('駆動電源ラッチ — 電源を入れ直してください')
  if (vs && !vs.steer_center_valid) latches.push('ステア原点が未保存 — 走行禁止')
  if (link?.protocol_match === false) latches.push('プロトコル版数が不一致')

  return (
    <>
      <header className="statusbar">
        <div className="brand">SURGE&nbsp;Mk.2</div>

        <span className={`pill lv-${healthLevel(health)}`}>{health}</span>
        <span className="pill">{mode}</span>
        <span className={`pill ${vs?.armed ? 'lv-warn' : 'dim'}`}>
          {vs?.armed ? 'ARMED' : 'DISARM'}
        </span>
        {link?.arm_inhibited && (
          <span className="pill dim" title="io_node に --allow-arm が無い。GUI からは解禁できない">
            arm 封印中
          </span>
        )}
        {/* **今どこまで出るか**は握る前に知りたい。armed かどうかに関係なく出す */}
        <span
          className={`pill ${ui.boost ? 'lv-warn' : 'dim'}`}
          title="Shift（パッドは R1）を押している間だけ全開。離すと低速に戻る"
        >
          {ui.boost ? 'BOOST' : '低速'}{' '}
          {(UI_MAX_SPEED * (ui.boost ? BOOST_SCALE : CRUISE_SCALE)).toFixed(2)}
        </span>
        {vs?.tc_active && <span className="pill lv-warn">TC</span>}
        {vs?.tv_active && <span className="pill lv-warn">TV</span>}

        <div className="spacer" />

        <span className="metric" title="COMMAND を送ってから cmd_seq_echo で返るまで（UART 往復）">
          <b className={`lv-${rttLevel(link?.cmd_rtt_ms)}`}>{ms(link?.cmd_rtt_ms)}</b>
          <i>uart</i>
        </span>
        <span className="metric" title="GUI ↔ Pi の往復（WebSocket）">
          <b>{ms(ui.wsRttMs)}</b>
          <i>ws</i>
        </span>
        <span className="metric" title="テレメトリの実受信レート。20Hz を大きく割ったら GUI 側が詰まっている">
          <b className={n.stale ? 'lv-bad' : ''}>{n.rxHz.toFixed(0)}</b>
          <i>Hz</i>
        </span>
        <span className="metric" title="Pi 側で数えた CRC エラー / パケットロス">
          <b className={(link?.rx?.crc_error ?? 0) + (link?.rx?.packet_loss ?? 0) > 0 ? 'lv-warn' : ''}>
            {link?.rx?.crc_error ?? 0}/{link?.rx?.packet_loss ?? 0}
          </b>
          <i>crc/loss</i>
        </span>

        <span className={`pill ${ui.telemetryOpen ? 'lv-ok' : 'lv-bad'}`}>
          {ui.telemetryOpen ? '接続' : '切断'}
        </span>
        <ArmButton />
        <button className="estop" onClick={onEstop} title="Esc キーでも同じ">
          E-STOP
        </button>
      </header>

      {latches.map((t) => (
        <div className="latch" key={t}>
          {t}
        </div>
      ))}
    </>
  )
}

/**
 * ARM の保持ボタン。**押している間ではなく、押したら保持される。**
 *
 * 保持中は無操作の残り時間を出す。これが見えていないと
 * 「いつの間にか切れていた／切れていなかった」が分からない。
 */
function ArmButton() {
  const n = useNumbers()
  const ui = useUi()
  const inhibited = n.link?.arm_inhibited ?? true
  const left = n.armRemainingMs

  const toggle = () => {
    if (ui.armRequested) ui.set({ armRequested: false, disarmReason: 'ボタンで解除' })
    else ui.set({ armRequested: true, disarmReason: '' })
  }

  return (
    <button
      className={`armbtn ${ui.armRequested ? 'on' : ''}`}
      onClick={toggle}
      disabled={inhibited}
      title={
        inhibited
          ? 'io_node に --allow-arm が無い。GUI からは解禁できない'
          : 'Enter キーでも同じ。無操作で自動解除される'
      }
    >
      {/* 残り 10秒を切ったら 0.1秒刻みにする。長いうちは秒刻みの方が落ち着いて読める */}
      {ui.armRequested
        ? `ARM 保持中 ${left >= 10_000 ? Math.ceil(left / 1000) : (left / 1000).toFixed(1)}s`
        : 'ARM'}
    </button>
  )
}
