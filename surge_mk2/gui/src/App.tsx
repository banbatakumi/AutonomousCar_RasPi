/**
 * 画面の骨格。運転・設定・ログの3ビューは作り込んである。
 * 残る「地図」「診断」は `architecture.md` §10.2 の枠だけ置いてある。
 */
import { useEffect, useRef, useState } from 'react'
import { StatusBar } from './components/StatusBar'
import { useDriving } from './input/useDriving'
import { useUi } from './store/ui'
import { DriveView } from './views/DriveView'
import { LogView } from './views/LogView'
import { SettingsView } from './views/SettingsView'
import { ControlChannel } from './ws/control'
import { connectTelemetry } from './ws/telemetry'

const TABS = ['運転', '地図', '診断', '設定', 'ログ'] as const
type Tab = (typeof TABS)[number]

export function App() {
  const [tab, setTab] = useState<Tab>('運転')
  const [ch, setCh] = useState<ControlChannel | null>(null)
  const chRef = useRef<ControlChannel | null>(null)
  const set = useUi((s) => s.set)

  useEffect(() => {
    const stopTelemetry = connectTelemetry((open) => set({ telemetryOpen: open }))

    const c: ControlChannel = new ControlChannel({
      onOpenChange: (open) => set({ controlOpen: open, ...(open ? {} : { hasControl: false }) }),
      // **操縦権が自分かどうかは名前の一致で判定する。**
      // 「誰かが持っている」だけだと2枚目のタブが自分だと誤認する
      onStatus: (s) =>
        set({
          status: s,
          hasControl: s.has_controller && s.controller === c.id,
          deniedBy: s.has_controller && s.controller !== c.id ? s.controller : null,
          sfl: s.sfl,
          mcap: s.mcap,
        }),
      onDenied: (holder) => set({ deniedBy: holder, hasControl: false }),
      onRtt: (v) => set({ wsRttMs: v }),
      onLogs: (files) => set({ logFiles: files }),
    })
    chRef.current = c
    setCh(c)

    const ping = window.setInterval(() => c.ping(), 1000)
    return () => {
      window.clearInterval(ping)
      c.close()
      stopTelemetry()
    }
  }, [set])

  useDriving(ch)

  return (
    <div className="app">
      <StatusBar onEstop={() => ch?.estop()} />
      <nav className="tabs">
        {TABS.map((t) => (
          <button key={t} className={t === tab ? 'on' : ''} onClick={() => setTab(t)}>
            {t}
          </button>
        ))}
        <div className="spacer" />
        <DriveHint />
      </nav>

      {tab === '運転' ? (
        <DriveView />
      ) : tab === '設定' ? (
        <SettingsView />
      ) : tab === 'ログ' ? (
        <LogView ch={ch} />
      ) : (
        <Placeholder tab={tab} />
      )}
    </div>
  )
}

/** 操縦の状態と操作方法。**ARM していないことが一目で分かる**必要がある。 */
function DriveHint() {
  const ui = useUi()
  return (
    <div className="hint">
      {ui.deniedBy && <span className="badge-bad">操縦権は {ui.deniedBy} が保持中</span>}
      <span className={ui.deadman ? 'badge-live' : 'dim'}>
        {ui.deadman
          ? `操縦中（${ui.inputSource === 'gamepad' ? 'パッド' : 'キーボード'}）`
          : 'ARM していません'}
      </span>
      {/* **なぜ止まったかを必ず出す。** 分からないのがデバッグを最も消耗させる */}
      {!ui.armRequested && ui.disarmReason && (
        <span className="badge-warn">{ui.disarmReason}</span>
      )}
      <span className="dim">
        Enter で ARM ／ W A S D ／ パッド R2 + 左スティック ／ Esc で E-STOP
      </span>
    </div>
  )
}

function Placeholder({ tab }: { tab: Tab }) {
  const what: Record<string, string> = {
    地図: 'SLAM・占有格子・生成経路のデバッグ（Phase 3 で中身が入る）',
    診断: '時系列グラフ（uPlot）、パケットロスの推移、STATS の突き合わせ',
  }
  return (
    <div className="placeholder">
      <h2>{tab}</h2>
      <p>{what[tab]}</p>
      <p className="dim">
        今回のスコープは運転ビュー1枚。WS の形が固まったので、ここは同じ
        <code>/ws/telemetry</code> の購読を足すだけで作れる。
      </p>
    </div>
  )
}
