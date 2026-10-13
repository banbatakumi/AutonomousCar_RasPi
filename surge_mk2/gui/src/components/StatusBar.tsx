/**
 * ステータスバー — 層 B（常時周辺視）。**横目で追える情報だけ**を置く。
 *
 * ここに数字を増やすと読めなくなるので、載せるのは
 * 「状態が変わったら即座に気づきたいもの」に限る。
 *
 * 通信の生の数字（UART/WS の RTT、受信 Hz、CRC/loss）は**平常時はどのタブでも畳む**
 * （診断タブに同じ数字が全部あるので重複させない）。リンクが劣化したときだけ1つのピルにまとめて出す。
 *
 * health（OK/DEGRADED等）は**variantに関係なく常に出す**（正常なことも一目で分かる方が良い）。
 *
 * ## variant='rc'（ラジコンタブ）でも残っている違い
 *
 * TC/TVのピルは引き続き `variant === 'full'` のときだけ出す。ラジコンでは同じ介入を
 * `AssistLamps`（映像下のランプ）が別の見た目で常時出しているので、ピルまで出すと重複する。
 *
 * E-STOP・ARM・ラッチ警告・接続断・Wi-Fi・health・自動停止 は variant に関係なく必ず出す
 * （自動停止はタブごとに場所が違うと分かりにくいので統一している）。
 */
import { useNumbers } from '../bus/live'
import { MODE_NAME, healthLevel, ms, rttLevel } from '../format'
import { useUi } from '../store/ui'
import { askConfirm } from './ConfirmDialog'
import { WifiIcon } from './WifiIcon'

export type StatusVariant = 'full' | 'rc'

export function StatusBar({
  onEstop,
  onShutdown,
  variant = 'full',
}: {
  onEstop: () => void
  onShutdown: () => void
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
  // STM32 が MD の異常で走行を止めている（後輪を制動→位置保持「固定中」。ARM を外すまで戻らない）。
  // どの MD がなぜ、は STM32 の LOG（`link.stm_log`）にしか出ない
  if (vs?.faults.includes('md_fault')) {
    latches.push(`MD 異常で停止中 — ARM し直すまで解除されません${link?.stm_log?.length ? `（${link.stm_log.join(' / ')}）` : ''}`)
  }

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

        <span className={`pill lv-${healthLevel(health)}`}>{health}</span>
        <span className="pill">{mode}</span>
        <span className={`pill ${vs?.armed ? 'lv-warn' : 'dim'}`}>
          {vs?.armed ? 'ARMED' : 'DISARM'}
        </span>
        {link?.arm_inhibited && (
          <span className="pill dim" title={link.arm_inhibit_reason || 'io_node が ARM を通さない。GUI からは解禁できない'}>
            arm 封印中
          </span>
        )}
        {/* ラジコンでは TC/TV の介入をランプ（`components/rc/AssistLamps.tsx`）で出すので
            重複させない。あちらは一瞬の介入もラッチして光る */}
        {variant === 'full' && vs?.tc_active && <span className="pill lv-warn">TC</span>}
        {variant === 'full' && vs?.tv_active && <span className="pill lv-warn">TV</span>}
        {variant === 'full' && vs?.abs_active && <span className="pill lv-warn">ABS</span>}
        {variant === 'full' && vs?.launch_active && <span className="pill lv-warn">LAUNCH</span>}
        {variant === 'full' && vs?.wheel_lift_active && <span className="pill lv-warn">片輪浮き</span>}
        {/* 自動停止。**「許可しているか」と「今まさに効いているか」は別物**なので
            両方を1つのピルで出し分ける。効いている間は急減速の理由がこれだと即分かるように
            lv-bad まで上げる（TC/TV より強い介入で、指令が完全に無視されるため）。
            **variantに関係なく常に出す**（`AssistLamps` ではなくこちらに一本化している） */}
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

        <div className="spacer" />

        {/* 平常時はどのタブでも何も出さない。**遅い・切れかけのときだけ**1つのピルで割り込む。
            詳しい数字（RTT・受信Hz・CRC・パケットロス）は診断タブに全部ある */}
        {linkPoor && (
          <span
            className={`pill ${n.stale || linkBad ? 'lv-bad' : 'lv-warn'}`}
            title="通信の詳しい数字（RTT・受信 Hz・CRC・パケットロス）は診断タブにある"
          >
            {n.stale ? '通信 途絶' : linkBad ? `通信 ${health}` : `通信 遅延 ${ms(link?.cmd_rtt_ms, 0)}`}
          </span>
        )}

        {/* Pi側の物理Wi-Fiリンク。**「接続/切断」（WSが繋がっているか）とは別物**——
            電波は繋がっていてもPi側プロセスが落ちていればWSは切れるし、逆にWSが
            繋がっていても電波が弱ければ切れる予兆として先に読める */}
        <span
          className="metric"
          title={
            ui.status?.wifi?.available === false
              ? 'この機体ではWi-Fi状態を取得できない'
              : ui.status?.wifi?.ssid == null
                ? 'Wi-Fi 未接続'
                : `SSID: ${ui.status.wifi.ssid} / ${ui.status.wifi.rssi_dbm ?? '—'}dBm`
          }
        >
          <WifiIcon dbm={ui.status?.wifi?.rssi_dbm ?? null} />
        </span>

        {/* **型の食い違いは「切断」と別に出す。** 同じ表示にすると Wi-Fi を
            疑って時間を溶かす。実際には繋がっていて、GUI が古いだけ */}
        <span
          className={`pill ${ui.schemaMismatch ? 'lv-bad' : ui.telemetryOpen ? 'lv-ok' : 'lv-bad'}`}
          title={
            ui.schemaMismatch
              ? 'Pi 側とテレメトリの型定義が食い違っています。gui を再ビルドしてください（npm run build）'
              : undefined
          }
        >
          {ui.schemaMismatch ? '型不一致' : ui.telemetryOpen ? '接続' : '切断'}
        </span>
        {/* 直前に ARM が下りた理由（E-STOP・操縦権の喪失・無操作など）。**なぜ止まったかを
            必ず見せる**——出さないと、押していないのに ARM が外れた理由が分からない */}
        {!ui.armRequested && ui.disarmReason && (
          <span className="pill lv-warn" title="直前に ARM が解除された理由">
            {ui.disarmReason}
          </span>
        )}
        <ArmButton />
        <button className="estop" onClick={onEstop} title="Esc キーでも同じ">
          E-STOP
        </button>
        <button
          onClick={() => {
            void askConfirm('ラズパイをシャットダウンしますか？').then((ok) => ok && onShutdown())
          }}
          title="Pi を安全にシャットダウンする"
        >
          シャットダウン
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
  // **解除はいつでも押せる。** 封印中・テレメトリ未受信で止めるのは「入れる」方だけ
  const blocked = inhibited && !ui.armRequested

  const toggle = () => {
    if (ui.armRequested) ui.set({ armRequested: false, disarmReason: 'ボタンで解除' })
    else ui.set({ armRequested: true, disarmReason: '' })
  }

  return (
    <button
      className={`armbtn ${ui.armRequested ? 'on' : ''}`}
      onClick={toggle}
      disabled={blocked}
      title={
        blocked
          ? n.link?.arm_inhibit_reason || 'io_node が ARM を通さない（テレメトリ未受信・--allow-arm が無い等）。GUI からは解禁できない'
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
