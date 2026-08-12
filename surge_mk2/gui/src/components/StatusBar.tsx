/**
 * ステータスバー — 層 B（常時周辺視）。**横目で追える情報だけ**を置く。
 *
 * ここに数字を増やすと読めなくなるので、載せるのは
 * 「状態が変わったら即座に気づきたいもの」に限る。
 *
 * ## variant='rc'（ラジコンタブ）
 *
 * 通信の生の数字（UART/WS の RTT、受信 Hz、CRC/loss）を**平常時は畳む**。
 * 運転を楽しんでいる最中に読む数字ではなく、常時4つ並んでいると
 * ARM 状態や介入ピルが埋もれる。
 *
 * **隠すのは正常時だけ。** リンクが劣化したら1つのピルにまとめて出し、
 * E-STOP・ARM・ラッチ警告・接続断は variant に関係なく必ず出す。
 * 詳しい数字は診断タブに全部ある。
 */
import { useNumbers } from '../bus/live'
import { MODE_NAME, healthLevel, ms, rttLevel } from '../format'
import { useUi } from '../store/ui'

export type StatusVariant = 'full' | 'rc'

export function StatusBar({
  onEstop,
  variant = 'full',
}: {
  onEstop: () => void
  variant?: StatusVariant
}) {
  const n = useNumbers()
  const ui = useUi()
  const vs = n.vs
  const link = n.link

  const health = link?.health ?? 'INIT'
  const mode = MODE_NAME[vs?.mode ?? 0]
  const rtt = rttLevel(link?.cmd_rtt_ms)
  // ラジコンでも「通信が怪しい」ことだけは伝える。数字ではなく1つのピルで
  const linkBad = health !== 'OK' && health !== 'INIT'
  const linkPoor = linkBad || rtt !== 'ok' || n.stale

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

        {/* **実機かシムかは何よりも先に分かる必要がある。** variant で省かない。
            これを見落としたまま --allow-arm すると本物の車が動く。
            GUI がシミュレータについて知っているのはこの1ビットだけで、
            コース切替やノイズ量の操作は sim.gui（pygame）側にある */}
        {link?.sim && (
          <span
            className="pill sim"
            title="UART の相手は実 STM32 ではなくシミュレータ（sim/link.py）。コース切替やノイズ量の調整はシミュレータGUI 側で行う"
          >
            SIM
          </span>
        )}

        {/* ラジコンでは health が OK の間だけ省く。**異常なら variant に関係なく出す** */}
        {(variant === 'full' || health !== 'OK') && (
          <span className={`pill lv-${healthLevel(health)}`}>{health}</span>
        )}
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
          {(ui.boost ? ui.settings.maxSpeed : ui.settings.maxSpeed * ui.settings.cruiseScale).toFixed(2)}
        </span>
        {/* ラジコンでは介入をランプ（`components/rc/AssistLamps.tsx`）で出すので重複させない。
            あちらは一瞬の介入もラッチして光る */}
        {variant === 'full' && vs?.tc_active && <span className="pill lv-warn">TC</span>}
        {variant === 'full' && vs?.tv_active && <span className="pill lv-warn">TV</span>}
        {/* v0.7 自動停止。**「許可しているか」と「今まさに効いているか」は別物**なので
            両方を1つのピルで出し分ける。効いている間は急減速の理由がこれだと即分かるように
            lv-bad まで上げる（TC/TV より強い介入で、指令が完全に無視されるため） */}
        {variant === 'full' && (
          <span
            className={`pill ${vs?.auto_stop_active ? 'lv-bad' : 'dim'}`}
            title={
              ui.settings.autoStop
                ? '進行方向の超音波が 20cm 未満なら STM32 が指令を無視して最大制動する（ラジコンタブの設定で OFF にできる）'
                : '自動停止は無効。STM32 は超音波を見ても止めない（ラジコンタブの設定で ON にできる）'
            }
          >
            {vs?.auto_stop_active ? '自動停止 作動中' : ui.settings.autoStop ? '自動停止' : '自動停止 OFF'}
          </span>
        )}

        <div className="spacer" />

        {variant === 'full' ? (
          <>
            <span className="metric" title="COMMAND を送ってから cmd_seq_echo で返るまで（UART 往復）">
              <b className={`lv-${rtt}`}>{ms(link?.cmd_rtt_ms)}</b>
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
          </>
        ) : (
          // 平常時は何も出さない。**遅い・切れかけのときだけ**1つのピルで割り込む
          linkPoor && (
            <span
              className={`pill ${n.stale || linkBad ? 'lv-bad' : 'lv-warn'}`}
              title="通信の詳しい数字（RTT・受信 Hz・CRC・パケットロス）は診断タブにある"
            >
              {n.stale ? '通信 途絶' : linkBad ? `通信 ${health}` : `通信 遅延 ${ms(link?.cmd_rtt_ms, 0)}`}
            </span>
          )
        )}

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
