/**
 * 画面の骨格。タブは4枚（`architecture.md` §10.2）。
 *
 *   ラジコン   運転を楽しむための画面。メータ主体、設定は歯車から
 *   自動運転   自律走行の監視・デバッグ。指令と実測を数値で並べる（Phase 3 で中身が育つ）
 *   診断       現在値と時系列。走行画面から外した数字はすべてここ
 *   ログ       記録の開始・停止・ダウンロード
 *
 * **既定はラジコン。** 自動運転は Phase 3 以降にならないと中身が無く、
 * 今開いて意味があるのはラジコンのため。
 *
 * 「地図」タブは削除した（プレースホルダのままだった）。SLAM を実装する
 * Phase 3 で、自動運転ビューの中に入れるか独立させるかを決め直す。
 */
import { useEffect, useRef, useState } from 'react'
import { DriveControls } from './components/DriveControls'
import { SplashEmblem } from './components/SplashEmblem'
import { StatusBar } from './components/StatusBar'
import { useDriving } from './input/useDriving'
import { useUi } from './store/ui'
import { AutoView } from './views/AutoView'
import { DiagView } from './views/DiagView'
import { LogView } from './views/LogView'
import { MapView } from './views/MapView'
import { RcView } from './views/RcView'
import { ControlChannel } from './ws/control'
import { connectMap } from './ws/map'
import { connectTelemetry } from './ws/telemetry'

//: **地図は自動運転の隣**。世界座標の情報（自己位置・経路・障害物）は
//: 走行中にも見たいので、EXPLORE 専用の画面にはしない（`views/MapView.tsx`）
const TABS = ['ラジコン', '自動運転', '地図生成', '診断', 'ログ'] as const
type Tab = (typeof TABS)[number]

/** 操縦しうるタブ。ここでだけ操作ヒントを出す（診断・ログでは邪魔になる） */
const DRIVING_TABS: Tab[] = ['ラジコン', '自動運転']

export function App() {
  const [tab, setTab] = useState<Tab>('ラジコン')
  const [showSplash, setShowSplash] = useState(true)
  const [ch, setCh] = useState<ControlChannel | null>(null)
  const chRef = useRef<ControlChannel | null>(null)
  const set = useUi((s) => s.set)

  useEffect(() => {
    const stopTelemetry = connectTelemetry(
      (open) => set({ telemetryOpen: open }),
      // **Pi 側だけ新しくなった状態。** 描き続けるほうが危ないので、
      // `ws/telemetry.ts` はフレームを捨てている。ここは知らせるだけ
      (got) => {
        console.error(
          `テレメトリの型定義が食い違っています（Pi: 0x${got.toString(16)}）。` +
            'gui を再ビルドしてください: npm run build',
        )
        set({ schemaMismatch: true })
      },
    )
    // 地図は**変わったときだけ**届く別チャンネル。タブを開いていなくても繋いでおく
    // （後から地図タブを開いた人に、その時点の1枚が出ている必要がある）
    const stopMap = connectMap((open) => set({ mapOpen: open }))

    const c: ControlChannel = new ControlChannel({
      onOpenChange: (open) => set({ controlOpen: open, ...(open ? {} : { hasControl: false }) }),
      // **操縦権が自分かどうかは名前の一致で判定する。**
      // 「誰かが持っている」だけだと2枚目のタブが自分だと誤認する
      onStatus: (s) =>
        set({
          status: s,
          hasControl: s.has_controller && s.controller === c.id,
          deniedBy: s.has_controller && s.controller !== c.id ? s.controller : null,
          ...(s.has_controller && s.controller === c.id ? { deniedReason: null } : {}),
          sfl: s.sfl,
          mcap: s.mcap,
          auto: s.auto,
          fan: s.fan,
          cameraConfig: s.camera_config,
        }),
      onDenied: (holder, reason) =>
        set({ deniedBy: holder, deniedReason: reason ?? null, hasControl: false }),
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
      stopMap()
    }
  }, [set])

  // **タブに関係なく操縦は生きている。** 診断タブを見ている間に急に止まると事故になる
  useDriving(ch)

  return (
    <div className="app">
      {showSplash && <SplashEmblem onDone={() => setShowSplash(false)} />}
      <StatusBar onEstop={() => ch?.estop()} variant={tab === 'ラジコン' ? 'rc' : 'full'} />
      <nav className="tabs">
        {TABS.map((t) => (
          <button key={t} className={t === tab ? 'on' : ''} onClick={() => setTab(t)}>
            {t}
          </button>
        ))}
        <div className="spacer" />
        {DRIVING_TABS.includes(tab) && (
          <>
            <DriveControls ch={ch} />
            <DriveHint />
          </>
        )}
      </nav>

      {tab === 'ラジコン' ? (
        <RcView ch={ch} />
      ) : tab === '自動運転' ? (
        <AutoView ch={ch} />
      ) : tab === '地図生成' ? (
        <MapView ch={ch} />
      ) : tab === '診断' ? (
        <DiagView />
      ) : (
        <LogView ch={ch} />
      )}
    </div>
  )
}

/** 操縦の状態と操作方法。**ARM していないことが一目で分かる**必要がある。 */
function DriveHint() {
  const ui = useUi()
  return (
    <div className="hint">
      {ui.deniedReason === 'bad_token' ? (
        <span className="badge-bad">
          トークンが違います（<code>?token=…</code> 付きの URL で開き直してください）
        </span>
      ) : ui.deniedReason === 'auto_engage' ? (
        <span className="badge-bad">自律走行の開始には操縦権が要ります</span>
      ) : ui.deniedBy ? (
        <span className="badge-bad">操縦権は {ui.deniedBy} が保持中</span>
      ) : null}
      <span className={ui.deadman ? 'badge-live' : 'dim'}>
        {!ui.deadman
          ? 'ARM していません'
          : ui.inputSource === 'auto'
            ? '自律走行中'
            : `操縦中（${ui.inputSource === 'gamepad' ? 'パッド' : 'キーボード'}）`}
      </span>
      {/* **なぜ止まったかを必ず出す。** 分からないのがデバッグを最も消耗させる */}
      {!ui.armRequested && ui.disarmReason && (
        <span className="badge-warn">{ui.disarmReason}</span>
      )}
    </div>
  )
}
