/**
 * イベント駆動の状態だけを持つ store（`system_overview.md` §10.3 の3層目）。
 *
 * **20Hz で変わるものはここに入れない。** 接続状態・操縦権・設定の類だけ。
 */
import { create } from 'zustand'
import { persist } from 'zustand/middleware'
import type {
  AutoStatus,
  CameraConfigStatus,
  ControlStatus,
  CamE2EModelFile,
  CamE2EModelStatus,
  E2EFusionModelFile,
  E2EFusionModelStatus,
  E2EModelFile,
  E2EModelStatus,
  FanStatus,
  LogFile,
  MapFile,
  SignalConfigStatus,
} from '../types'
import type { MtGear } from './gear'
import { localPersistStorage } from './persistStorage'
import {
  DEFAULT_SETTINGS,
  clampSettings,
  migrateSettings,
  readLegacySettings,
  restoreSettings,
  type DrivingSettings,
  type LaunchPhase,
  type PersistedSettings,
} from './settings'

export type InputSource = 'none' | 'keyboard' | 'gamepad' | 'slider' | 'auto'

/**
 * 灯火モード（`COMMAND.flags` bit3-4）。
 *
 * ⚠ **v0.4 の 1ビット `light` とは値の意味が違う。** 旧 `light=1` は全光量だったが、
 * v0.5 の `1` は DAYTIME（減光）である。旧 GUI の値をそのまま送ると暗いまま走る。
 */
export const LIGHT_OFF = 0
export const LIGHT_DAYTIME = 1
export const LIGHT_NORMAL = 2
export const LIGHT_LABEL = ['消灯', 'DAY', 'NORMAL']
/** `L` キー / パッド △ で回す順。**消灯を含む** */
export const LIGHT_CYCLE = [LIGHT_OFF, LIGHT_DAYTIME, LIGHT_NORMAL]

type UiState = {
  telemetryOpen: boolean
  /** `/ws/telemetry` の型定義の札が GUI 側と食い違っているか。
   *
   * **true の間はテレメトリを一切描かない**（`ws/telemetry.ts` がフレームを
   * 捨てている）。GUI を再ビルドするまで直らないので、切断とは別に出す——
   * 「切断」と表示すると Wi-Fi を疑って時間を溶かす。 */
  schemaMismatch: boolean
  controlOpen: boolean
  /** `/ws/map` が繋がっているか。**地図が更新されないのが「切れている」のか
   *  「凍結して変わらない」のかを区別するために要る** */
  mapOpen: boolean
  status: ControlStatus | null
  /** 自分が操縦権を持っているか */
  hasControl: boolean
  deniedBy: string | null
  /** 操縦権を取れなかった理由。**`bad_token` は「誰かが持っている」ではなく
   *  「トークンが違う」** なので、表示を分けないと原因を探して時間を溶かす */
  deniedReason: string | null
  /** 画面に数秒だけ出す通知（操縦権の拒否・削除失敗・シャットダウン拒否など）。
   * `notify()` で出す。ARM の解除理由（`disarmReason`）のような継続状態ではない */
  notice: { text: string; level: 'warn' | 'bad'; id: number } | null
  /** GUI ↔ Pi の往復 [ms]。UART 区間の遅延とは別物 */
  wsRttMs: number | null
  /** 自律走行の操作が断られた（`control_denied` の `auto_engage`＝engage に操縦権が要る・`auto_busy`＝
   * 他の操縦者が走らせている）。`seq` は断られるたびに増える（同じ理由が続いても変化として見えるように）。
   * システム同定タブが「試験を始められなかった」をタイマーを待たずに知るのに使う */
  autoDenial: { reason: string; seq: number } | null

  /** ARM ボタンで保持中か。**これが true の間だけ `cmd` を送る** */
  armRequested: boolean
  /** 実際に指令を送れているか（操縦権があり armRequested）。表示用 */
  deadman: boolean
  inputSource: InputSource
  /** 直前に DISARM した理由。**「なぜ止まったか」が分からないのが一番消耗する** */
  disarmReason: string
  /** 直前に自律走行が解除された理由。同上（engage したときに消す） */
  autoOffReason: string

  // ── 補機（v0.5） ──
  //
  // **押している間だけ立つものは、rAF ループが値の変化時だけここへ書き戻す。**
  // 毎フレーム set すると 60Hz で再レンダリングが走る。
  /** Space / パッド L2（力加減） を押している間。`brake_torque` を直接掛けている */
  braking: boolean
  /** H / パッド □ を押している間。**鳴りっぱなしになる** */
  horning: boolean
  /** P / パッド ○ を押している間。前照灯だけが全光量 */
  passing: boolean
  /** 手動のローンチコントロールの状態（ランプの表示用。`useDriving.ts` が変化時だけ書く） */
  launchPhase: LaunchPhase
  /** 0=消灯 1=DAYTIME 2=NORMAL。**ARM 中しか反映されない**（§下） */
  lightMode: number

  /**
   * サイドブレーキの意思（`COMMAND.flags2` bit0、★v0.13）。
   *
   * **`braking`/`horning`/`passing` とは性質が違い、押しっぱなしではなく
   * トグル。** ON にしたら明示的に OFF にするまで送り続ける——駐車ブレーキと
   * 同じ操作感（かけたらハンドルから手を離せる）にしてある。STM32 は
   * **速度に関わらず即座に**後輪を位置制御へ切り替えて固定する。**走行中でも ON にできる**
   * （ハンドブレーキターンのような使い方を妨げない。`DriveControls.tsx`・`useDriving.ts`）。
   * ページ再読み込みでは false に戻す（`localStorage` に
   * 保存しない）——保存すると再読み込み直後に前回の状態のまま送られてしまう。
   * 未 ARM のときは送らない（`useDriving.ts`、`brake` と同じ理由）
   */
  sideBrakeRequested: boolean

  /**
   * 左/右ウィンカーの意思（`COMMAND.flags2` bit1/2、v0.14）。
   *
   * `sideBrakeRequested` と同じくトグル（押している間だけではない）で、
   * **両方 true にするとハザード**として送られる（`command_from_cmd`）。
   * ページ再読み込みでは false に戻す（`localStorage` に保存しない。理由は
   * `sideBrakeRequested` と同じ——保存すると再読み込み直後に前回の状態のまま
   * 送られてしまう）。未 ARM でも送る（灯火と同じ扱い。`useDriving.ts`）
   */
  winkerLeftRequested: boolean
  winkerRightRequested: boolean
  /** STM32 のウィンカー点滅モード（1=通常、2=滑らか、3=豆電球風） */
  winkerMode: number

  /**
   * ゲームパッドの D/R レンジ（実車のシフトレバー相当）。
   *
   * DUALSHOCK4 の R2/L2 を実車ペダル配置（R2=アクセル固定、L2=ブレーキ固定）に
   * したことで、R2 単体では前進/後退の向きを表せなくなった。その向きをここで選ぶ
   * （`useDriving.ts` の `gearSign`）。**キーボードの W/S は別キーなので影響しない。**
   *
   * `sideBrakeRequested` と違い、これは「今どちらへ踏み込むか」という持続的な選択
   * （実車のシフトと同じ）なので、DISARM で自動的に 'D' へは戻さない——切り返し
   * （バックで詰めて戻る等）の途中で毎回 D に戻ると操作が壊れる。**ページ再読み込みでは
   * 'D' に戻る**（`localStorage` に保存しない。理由は `sideBrakeRequested` と同じ）。
   * 走行中の誤操作を防ぐため、UI 側は停止中（`vs.stopped`）以外では切り替えを拒否する。
   *
   * 操作口はパッドの L1（R）/R1（D）とキーボードの←（R）/→（D）（`useDriving.ts` の `shiftGear()`）——どちらも同じ `ui.gear` を更新するので、
   * 二重に状態を持たない。**画面ボタンは無い**（現在値は速度メータ
   * 中央のバッジで表示だけする、`SpeedGauge.tsx`）。
   */
  gear: 'D' | 'R'

  /**
   * MT（擬似変速）モードのギア位置。`driveMode==='mt'` のときだけ意味を持つ
   * ——速度制御・トルク制御では従来どおり `gear`（'D'|'R'）を使う。
   *
   * `gear` と同じ操作口（パッドの L1/R1、キーボードの←/→）だが意味が違う。**下＝
   * シフトダウン、上＝シフトアップの相対操作**（direct-set の `gear` と違い、
   * 押すたびに `MT_GEARS` 上を1段動く。`useDriving.ts` の `shiftGear()`）。
   * R への出入りだけ `gear` と同じガード（走行中は拒否）を掛ける——N〜D5間は
   * 実車の MT と同じく走行中のシフトアップ/ダウンを許可する。
   * **既定・ページ再読み込みは N**（`localStorage` に保存しない、`gear` と同じ理由）。
   * **速度制御/トルク制御から MT に切り替えた直後も必ず N に戻す**（`setSettings`
   * 参照）——前回どのギアで MT を抜けたかを持ち越すと、
   * 切り替えた瞬間に前のギアの上限でいきなり動き出しかねないため。
   * 画面ボタンは無い（`gear` と同じ）——現在値は速度メータ中央の
   * バッジで表示だけする
   */
  mtGear: MtGear

  /**
   * ラジコンタブの設定ドロワーが開いているか。
   *
   * **`localStorage` には保存しない。** 開きっぱなしで起動すると層 A（カメラ）が
   * 狭いまま走り出すことになる。開くのは調整したいと思ったときだけでよい。
   */
  settingsOpen: boolean
  /** LiDAR ビューの設定 */
  lidarZoom: number
  lidarFollow: boolean
  /** カメラの進路ガイド。**カメラ校正前なので暫定表示** */
  pathGuide: boolean
  /**
   * 進路上の障害物（LiDAR）を映像に重ねるか（`bus/pathObstacles.ts`）。`pathGuide` が OFF の
   * 間は出さない（ラジコンのメイン映像クリックで重畳をまとめて消せるように）。既定 ON
   */
  obstacleOverlay: boolean
  /**
   * 自動運転の計画（指令の進路・経路・狙っている点・検出した障害物）を前方映像に重ねるか。
   * planner が動いている間だけ出る。`obstacleOverlay` と同じく `pathGuide` に従う。既定 ON
   */
  planOverlay: boolean
  /**
   * クリアランスソナーの音を鳴らすか（`hooks/useParkBeep.ts`）。ARM 中、進む向きの半周
   * （LiDAR・超音波）に近い物があるときだけ鳴る。**GUI の演出で、自動停止とは独立**。既定 ON
   */
  parkBeep: boolean
  /** LiDAR ミニマップを拡大表示中か。false＝計器パネルの中の丸、true＝映像の中央に 82% 高さ。
   * ミニマップ自体のクリック（パッドは R3）で切り替える */
  lidarExpanded: boolean

  /**
   * 合成エンジン音（`audio/engineSound.ts`）を鳴らすか。**GUI 演出のみで
   * 車両側には一切関与しない**——鳴るのはこの画面を開いているブラウザの
   * スピーカーだけ。`AudioContext` はブラウザの自動再生制限があるため、
   * ON トグルのクリック（ユーザー操作）の中で作る必要がある
   * （設定ドロワーの「サウンド」タブ、`SettingsPanel.tsx`／`hooks/useEngineSound.ts` 参照）。
   * **既定 OFF・`localStorage` には保存しない**（次回開いたときに勝手に
   * 音が鳴り出すと驚くため、毎回明示的に ON してもらう）。音色の選択
   * （`settings.engineSoundType`）は永続化する——こちらは驚きにつながらない嗜好のため。
   */
  engineSoundOn: boolean

  /** ラジコン操作の調整値。設定パネルで変更、`localStorage` に自動保存 */
  settings: DrivingSettings
  /** 「既定値に戻す」の戻り先。既定はソースコードの `DEFAULT_SETTINGS` だが、
   * `saveCurrentAsDefault` で `settings` の現在値に上書きできる */
  settingsDefault: DrivingSettings


  // ── 記録・再生（ログタブ） ──
  /** `.sfl` を録ってほしいという意思。null はまだ status を受け取っていない */
  sfl: ControlStatus['sfl'] | null
  /** mcap のライブ中継。Piには保存されない */
  mcap: ControlStatus['mcap'] | null
  /** 🎥 録画（mp4）。Pi の `logs/` に書かれ、止めるとブラウザへダウンロードする */
  video: ControlStatus['video'] | null
  /** `logs/` にある `.sfl`/`.mcap`/`.mp4` の一覧 */
  logFiles: LogFile[]

  /**
   * 自動運転の意思と、選べるモードの宣言。**サーバが真値なのでここでは編集しない。**
   *
   * 押した結果は `status` のブロードキャストで返ってくる（`ws/control.ts` の
   * `setAuto`）。GUI 側に「engage したつもり」の状態を持つと、2枚目のタブや
   * サーバ側の拒否（モード未選択・E-Stop）と食い違う。
   */
  auto: AutoStatus | null

  /**
   * Pi5純正ファンの意思。**サーバが真値なのでここでは編集しない。**
   * 押した結果は `status` のブロードキャストで返ってくる（`ws/control.ts` の `setFan`）。
   */
  fan: FanStatus | null

  /**
   * capture側(camera_node)の前カメラのFPS上限・GUI配信頻度。
   * **サーバが真値なのでここでは編集しない。** 押した結果は `status` の
   * ブロードキャストで返ってくる（`ws/control.ts` の `setCamera`）。
   */
  cameraConfig: CameraConfigStatus | null

  /**
   * 矢印信号認識（`arrow_signal_node`）のON/OFF・HSVしきい値。
   * **サーバが真値なのでここでは編集しない。** 押した結果は `status` の
   * ブロードキャストで返ってくる（`ws/control.ts` の `setSignalConfig`）。
   */
  signalConfig: SignalConfigStatus | null

  /**
   * e2e_lidar（強化学習）が使うモデルの選択。**サーバが真値なのでここでは編集しない。**
   * 押した結果は `status` のブロードキャストで返ってくる（`ws/control.ts` の
   * `e2eModelSelect`）。
   */
  e2eModel: E2EModelStatus | null
  /** `models/e2e_lidar/` にある `.onnx` の一覧（`e2eModelList` の応答） */
  e2eModelFiles: E2EModelFile[]

  /**
   * cam_e2e（カメラE2E・模倣学習）が使うモデルの選択。**サーバが真値なのでここでは
   * 編集しない。** 押した結果は `status` のブロードキャストで返ってくる
   * （`ws/control.ts` の `camE2eModelSelect`）。
   */
  camE2eModel: CamE2EModelStatus | null
  /** `models/cam_e2e/` にある `.onnx` の一覧（`camE2eModelList` の応答） */
  camE2eModelFiles: CamE2EModelFile[]

  /** e2e_fusion が使うモデルの選択。**サーバが真値**（`ws/control.ts` の `e2eFusionModelSelect`）。空文字は「自動」 */
  e2eFusionModel: E2EFusionModelStatus | null
  /** `models/e2e_fusion/` にある `.onnx` の一覧（`e2eFusionModelList` の応答） */
  e2eFusionModelFiles: E2EFusionModelFile[]
  /** stereo_e2e が使うモデルの選択（`ws/control.ts` の `stereoE2eModelSelect`）。空文字は「自動」 */
  stereoE2eModel: E2EFusionModelStatus | null
  /** `models/stereo_e2e/` にある `.onnx` の一覧 */
  stereoE2eModelFiles: E2EFusionModelFile[]

  /** `saved_maps/` にある保存済み地図の一覧（`mapsList`/`mapsSave`/`mapsDelete` の応答） */
  mapFiles: MapFile[]
  /** 直近の `mapsSave()` の結果。表示したら呼び出し側が `null` に戻す
   * （トースト的な一過性の通知——`sfl`/`mcap` のような継続状態ではない） */
  mapSaveResult: { ok: boolean; error: string } | null

  /** 人が選んだ運転設定（上は切っていない）。`settings` はこれを今の車両の上限で切ったもの。
   * **保存するのはこちら**（`store/settings.ts` の `PersistedSettings`） */
  settingsIntent: DrivingSettings

  set: (p: Partial<UiState>) => void
  /**
   * ARM する（`armRequested` を立てる）。**灯火が消灯なら DAY を点ける**（実車のデイライトと同じく
   * 「走れる状態か」が外から分かる。NORMAL を手で選んでいたら下げない）。
   * 入れ方（Enter・画面のボタン・パッドの ×）によらずここを通す
   */
  arm: () => void
  /**
   * ARM を下ろす。`reason` は「なぜ止まったか」の表示（自分で切ったときは空文字）。
   * **ARM していなければ何もしない**（理由も書き換えない）。灯火は消灯に戻す。
   * 切り方（Esc・パッド・無操作・E-STOP・切断…）によらずここを通す
   */
  disarm: (reason: string) => void
  /** 変更分だけ渡せば良い。クランプしてから保存＆反映する */
  setSettings: (p: Partial<DrivingSettings>) => void
  /** `settingsDefault`（＝「既定値に戻す」の戻り先）へ戻す。値そのものは変えない */
  resetSettings: () => void
  /** 今の `settings` を新しい既定値として保存する。
   * 以後の「既定値に戻す」・各行の「既定値 X」表示はこの値を指す */
  saveCurrentAsDefault: () => void
}

const UI_PERSIST_KEY = 'surge.ui'

export const useUi = create<UiState>()(
  persist(
    (set, get) => ({
      mapOpen: false,
      telemetryOpen: false,
      schemaMismatch: false,
      controlOpen: false,
      status: null,
      hasControl: false,
      deniedBy: null,
      deniedReason: null,
      notice: null,
      wsRttMs: null,
      autoDenial: null,

      armRequested: false,
      deadman: false,
      inputSource: 'none',
      disarmReason: '',
      autoOffReason: '',

      braking: false,
      horning: false,
      passing: false,
      launchPhase: 'idle',
      lightMode: LIGHT_OFF,
      sideBrakeRequested: false,
      winkerLeftRequested: false,
      winkerRightRequested: false,
      winkerMode: 2, // 保存してあれば読み込みで上書きされる（`restoreSettings`）
      gear: 'D',
      mtGear: 'N',

      settingsOpen: false,
      lidarZoom: 4, // 画面半径が何メートルぶんか
      lidarFollow: true,
      pathGuide: true,
      obstacleOverlay: true,
      planOverlay: true,
      parkBeep: true,
      lidarExpanded: false,
      engineSoundOn: false,

      // 保存してある値は `persist` の読み込み（`restoreSettings`）で上書きされる
      settingsIntent: clampSettings(DEFAULT_SETTINGS, false),
      settings: clampSettings(DEFAULT_SETTINGS),
      settingsDefault: clampSettings(DEFAULT_SETTINGS),

      sfl: null,
      mcap: null,
      video: null,
      logFiles: [],
      auto: null,
      fan: null,
      cameraConfig: null,
      signalConfig: null,
      e2eModel: null,
      e2eModelFiles: [],
      camE2eModel: null,
      camE2eModelFiles: [],
      e2eFusionModel: null,
      e2eFusionModelFiles: [],
      stereoE2eModel: null,
      stereoE2eModelFiles: [],
      mapFiles: [],
      mapSaveResult: null,

      set: (p) => set(p),
      arm: () => {
        const s = get()
        if (s.armRequested) return
        set({ armRequested: true, disarmReason: '', ...(s.lightMode === LIGHT_OFF ? { lightMode: LIGHT_DAYTIME } : {}) })
      },
      disarm: (reason) => {
        const s = get()
        if (!s.armRequested) return
        set({ armRequested: false, disarmReason: reason, ...(s.lightMode !== LIGHT_OFF ? { lightMode: LIGHT_OFF } : {}) })
      },
      setSettings: (p) => {
        const prev = get().settings
        // 変えた項目だけを意思の値に重ねる。`setSettings({})`（上限の到着）では保存値は変わらない
        const settingsIntent = clampSettings({ ...get().settingsIntent, ...p }, false)
        const next = clampSettings(settingsIntent)
        // 速度制御/トルク制御から MT に切り替えた瞬間は必ず N から始める
        // （`mtGear` の型コメント参照）。前回 MT を抜けたときのギアを持ち越すと、
        // 切り替えた瞬間に前のギアの上限でいきなり動き出しかねないため
        const enteringMt = next.driveMode === 'mt' && prev.driveMode !== 'mt'
        set(enteringMt ? { settingsIntent, settings: next, mtGear: 'N' } : { settingsIntent, settings: next })
      },
      resetSettings: () => {
        // 既定値も必ずクランプを通す（保存してある既定値が今の車両の上限を超えていることがある）
        const settingsIntent = clampSettings(get().settingsDefault, false)
        set({ settingsIntent, settings: clampSettings(settingsIntent) })
      },
      saveCurrentAsDefault: () => {
        set({ settingsDefault: get().settings })
      },
    }),
    {
      // 保存するのは人が選んだ値だけ（運転設定・その既定値・ウィンカーの点滅モード）。
      // ARM・ギア・サイドブレーキ・ウィンカーの ON/OFF・エンジン音の ON は**保存しない**
      // （再読み込み直後に前回の状態のまま送られる・鳴り出すのを避ける。各項目のコメント参照）
      name: UI_PERSIST_KEY,
      version: 1,
      storage: localPersistStorage<PersistedSettings>(readLegacySettings),
      partialize: (s): PersistedSettings => ({
        settingsIntent: s.settingsIntent,
        settingsDefault: s.settingsDefault,
        winkerMode: s.winkerMode,
      }),
      migrate: (state, version) => migrateSettings(state, version) as PersistedSettings,
      merge: (persisted, current) => ({ ...current, ...restoreSettings(persisted) }),
    },
  ),
)
