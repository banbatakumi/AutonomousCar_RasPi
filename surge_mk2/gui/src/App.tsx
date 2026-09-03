/**
 * 画面の骨格。タブは4枚（`architecture.md` §10.2）。
 *
 *   ラジコン   運転を楽しむための画面。メータ主体、設定は歯車から
 *   自動運転   自律走行の監視・デバッグ。指令と実測を数値で並べる（Phase 3 で中身が育つ）
 *   診断       現在値と時系列。走行画面から外した数字と記録ファイル一覧はすべてここ
 *   システム同定  実車試験
 *
 * **既定はラジコン。** 自動運転は Phase 3 以降にならないと中身が無く、
 * 今開いて意味があるのはラジコンのため。
 *
 * 「地図生成」タブは削除した（2026-09-03）。世界座標の地図は自動運転タブの
 * `AutoMapPanel`（車体図の左、SLAMモード選択時だけ表示）に統合し、独立タブと
 * しては持たない——モード選択・engageの入口を1箇所に保つ（`AutoPanel.tsx`）
 * のと同じ理由で、地図の操作もその画面に集約する。
 *
 * 「ログ」タブも同じ理由で廃止した（2026-09-03）。録画の開始・停止は
 * モードタブの隣（`LogControls`）に常設し、ファイル一覧は診断タブ
 * （`DiagLogFiles`）に統合した——録るかどうかはどのタブを見ていても
 * 意思表示できる方が自然で、一覧はもともと診断タブと同じ「後から追う」場所のため。
 */
import { useEffect, useRef, useState } from 'react'
import { DriveControls } from './components/DriveControls'
import { LogControls } from './components/LogControls'
import { SplashEmblem } from './components/SplashEmblem'
import { StatusBar } from './components/StatusBar'
import { useEngineSound } from './hooks/useEngineSound'
import { useDriving } from './input/useDriving'
import { useUi } from './store/ui'
import { AutoView } from './views/AutoView'
import { DiagView } from './views/DiagView'
import { RcView } from './views/RcView'
import { SysIdView } from './views/SysIdView'
import { ControlChannel } from './ws/control'
import { connectMap } from './ws/map'
import { connectTelemetry } from './ws/telemetry'

const TABS = ['ラジコン', '自動運転', '診断', 'システム同定'] as const
type Tab = (typeof TABS)[number]

/** 操縦しうるタブ。ここでだけ灯火・ファン操作（`DriveControls`）を出す（診断では邪魔になる） */
const DRIVING_TABS: Tab[] = ['ラジコン', '自動運転', 'システム同定']

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
          camModel: s.cam_model,
          e2eModel: s.e2e_model,
        }),
      onDenied: (holder, reason) =>
        set({ deniedBy: holder, deniedReason: reason ?? null, hasControl: false }),
      onRtt: (v) => set({ wsRttMs: v }),
      onLogs: (files) => set({ logFiles: files }),
      onCamModels: (files) => set({ camModelFiles: files }),
      onE2EModels: (files) => set({ e2eModelFiles: files }),
      onMaps: (files) => set({ mapFiles: files }),
      onMapsSaveResult: (ok, error) => set({ mapSaveResult: { ok, error } }),
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
  // GUI 演出のみ（車両側には無関係）。`ui.engineSoundOn` の間だけ鳴る
  useEngineSound()

  return (
    <div className="app">
      {showSplash && <SplashEmblem onDone={() => setShowSplash(false)} />}
      <StatusBar
        onEstop={() => ch?.estop()}
        onShutdown={() => ch?.shutdown()}
        variant={tab === 'ラジコン' ? 'rc' : 'full'}
      />
      <nav className="tabs">
        {TABS.map((t) => (
          <button key={t} className={t === tab ? 'on' : ''} onClick={() => setTab(t)}>
            {t}
          </button>
        ))}
        <LogControls ch={ch} />
        <div className="spacer" />
        {DRIVING_TABS.includes(tab) && <DriveControls />}
      </nav>

      {tab === 'ラジコン' ? (
        <RcView ch={ch} />
      ) : tab === '自動運転' ? (
        <AutoView ch={ch} />
      ) : tab === '診断' ? (
        <DiagView ch={ch} />
      ) : (
        <SysIdView ch={ch} />
      )}
    </div>
  )
}
