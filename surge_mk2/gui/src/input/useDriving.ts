/**
 * ラジコン入力（ゲームパッド + キーボード）→ 50Hz の `cmd` 送信。
 *
 * ## 方式：ARM ボタン保持 ＋ 無操作タイムアウト
 *
 * 以前は「Space を押している間だけ送る」デッドマン方式だったが、
 * **押しながら WASD を操作するのが難しい**ため、明示的な ARM に変えた。
 *
 * | | ARM/DISARM（＋E-STOP） | 速度 | 舵 | ギア |
 * |---|---|---|---|---|
 * | キーボード | `Enter` または画面の ARM ボタン、`Esc` は即 DISARM | W,↑ / S,↓ | `A` / `D` | ←＝ダウン／→＝アップ |
 * | ゲームパッド | 同上 ／ × | R2（アクセル、DUALSHOCK4） | 左スティック X | L1＝ダウン／R1＝アップ |
 *
 * ⚠ **v0.18 で舵のキーボード割り当てから ←/→ を外した。** 元々 A/D と重複していた
 * （どちらでも同じ舵になっていた）ので、空いた ←/→ をギアチェンジに回した——
 * パッドの L1/R1 と方向の意味を揃えてある（`shiftGear()` 参照）。
 *
 * **ゲームパッドの速度は DUALSHOCK4 の実車ペダルに合わせた R2=アクセル／L2=ブレーキ
 * にしてある（v0.14）。** 以前は `R2 - L2` の差で前進/後退を作っていたが、それだと
 * L2 を「ブレーキ」として力加減できず（トリガーの押し込み量＝後退速度になってしまう）、
 * ペダルらしい操作感にならなかった。R2/L2 をそれぞれ独立したアナログ量として使う
 * 代わりに、前進/後退の向きは下の **Dレンジ/Rレンジ**（パッドの L1/R1、キーボードの
 * ←/→、実車のシフトレバー相当）で選ぶ。キーボードは元々 W/S が別キーなので
 * 影響なし（レンジはゲームパッドの R2 の向きだけを決める）。
 *
 * **× は Enter と同じ ARM/DISARM トグル（v0.15）。** それまでゲームパッドだけで
 * ARM する手段が無く、必ず一度キーボードか画面ボタンに触る必要があった。
 *
 * ⚠ **v0.19 で ○ 専用の E-STOP ボタンを廃止し、× の ARM/DISARM トグルに統合した。**
 * v0.15〜v0.18 は日本の SCE 標準（○＝決定／×＝キャンセル）に合わせ、○ を常時有効の
 * 専用 E-STOP ボタンにしていたが、実際には E-STOP は「即 DISARM（権限も条件も
 * 要らない）」以上のことをしていなかった——`estop_active`（車両側の物理ボタンだけが
 * 立てられるハードウェアラッチ、`raspi/core/link_tracker.py` 参照）は GUI/パッドの
 * どちらからも立てられず、ソフト側の「E-STOP」は実質ただの強い DISARM だったため、
 * ボタンを分ける意味が無いと判断した。今は **× で armed→unarmed に落とすとき
 * `ch.estop()` も呼ぶ**（下記 `tick()` の `padArm` 参照）——`ch.estop()` は
 * 操縦権を持っていなくても通るので、他のタブ/PC が操縦している状態からでも
 * この端末の × で強制的に止められる、という E-STOP の実用上の価値はそのまま残した。
 * 空いた ○ はパッシングに使う（下記）。
 *
 * ⚠ **v0.16 でブースト機能（Shift / パッド R1 を押している間だけ `maxSpeed` まで
 * 速度レンジが伸びる機能）を削除した。** 常に `maxSpeed` そのものが上限になる
 * （`store/ui.ts` の `maxSpeed` 注記参照）。空いた R1 は L1 と対でギアチェンジに使う
 * （下記）。
 *
 * ## 制御方式は3択：速度制御 / トルク制御 / MT（擬似変速、v0.17）
 *
 * `settings.driveMode`（`store/ui.ts`）で選ぶ。速度制御・トルク制御は従来どおり
 * （設定パネルの「制御方式」）。**MT は速度制御の一種**——スロットルは相変わらず
 * `target_speed` を出すが、そのときの上限（専用の `mtMaxSpeed`。固定値の `maxSpeed`
 * ではない）を `ui.mtGear`（R, N, D1〜D5 の7段）に応じた倍率（`settings.mtGear1`〜
 * `mtGear5`、N だけは 0 固定）にする。ワイヤプロトコルには `torque_mode=false` で
 * 送るので、**STM32 から見ればただの速度指令**で MT かどうかは分からない
 * （GUI 側だけで完結する演出）。**N（ニュートラル、v0.18）は上限が 0 になるだけ**
 * ——スロットルを踏んでも指令速度は 0 へ収れんする（実車のニュートラルと同じ、
 * N 専用の分岐は要らない）。
 *
 * ⚠ **2026-08-30: MT は 'speed'/'torque' と一切パラメータを共有しない
 * （`mtMaxSpeed`/`mtAccel`/`mtCoast`/`mtEngineBrake`、いずれも `store/ui.ts`）。**
 * 以前は `maxSpeed`/`accel`/`coast`/`kickSpeed` をそのまま流用していたため、
 * 「'speed' モードの乗り味を詰めたら MT の挙動まで変わった」という混乱があった。
 * 加えて、実車の MT に合わせて次の2点も変えた:
 *
 * - **アクセルオフは即ブレーキではなく惰行**。離した瞬間に 0 へスナップ／
 *   `coast` で減速するのではなく、`mtEngineBrakeRate`（下記）でなだらかに減速する。
 * - **逆キーブレーキと発進キックを廃止**。実車のアクセルペダルは1つしか無いので、
 *   ギアと逆のキーは何もしない（上記「### 3」参照）。発進キックも `mtAccel` に一本化した
 *   （下のキーボード分岐 `dir === gearSign` 参照）。
 *
 * ギアが下がるほど強くなる「エンジンブレーキ」相当のレート（`mtEngineBrakeRate`）を、
 * アクセルオフの惰行と、シフトダウンで上限速度を超えたときの減速の両方に使う
 * （`tick()` 内、瞬間移動ではなくなだらかに新しい上限まで落ちる）。
 *
 * ギアは実車の MT と同じ「1段ずつシフト」——パッドの L1＝シフトダウン／R1＝
 * シフトアップ、キーボードの←/→も同じ意味（v0.18。共通ロジックは `shiftGear()`
 * に集約してある）。既存の Dレンジ/Rレンジと同じボタンだが、MT モード中だけ
 * 意味が相対操作に変わる。R をまたぐとき（R↔N）だけ `gear` と同じガード
 * （走行中は拒否）を掛け、N〜D5間は走行中のシフトも許可する——実車で
 * 加速しながらシフトアップするのと同じ操作感を狙った。
 *
 * ⚠ v0.18 で画面（`DriveControls.tsx`）のギア表示ボタンは廃止した。現在ギアは
 * 速度メータ中央のバッジ（`SpeedGauge.tsx`）に表示だけする——操作はパッド/
 * キーボードに一本化し、タッチ操作は今のところ提供していない。
 *
 * 補機（v0.5 で追加、v0.15/v0.16/v0.19 でボタン配置を変更）
 *
 * | | ブレーキ | クラクション | パッシング | 灯火切替 |
 * |---|---|---|---|---|
 * | キーボード | `Space` | `H` | `P` | `L`（消灯→DAY→NORMAL→…） |
 * | ゲームパッド | L2（力加減のみ、v0.16） | □ | ○（v0.19） | △ |
 *
 * **灯火切替以外はすべて「押している間だけ」。** STM32 側の意味論
 * （`horn` / `passing` は立てている間ずっと効く）に素直に対応させてある。
 * GUI 側でタイマーを持つと、DISARM やタブ切替と競合したときに消し忘れが起きる。
 *
 * ⚠ **パッシングのボタンは v0.14: □、v0.15〜v0.18: 十字キー右、v0.19: ○ と
 * 移ってきた。** v0.19 で ○ 専用の E-STOP ボタンを廃止した（上記）ことで空いた
 * ので、十字キーより押しやすい面ボタンへ寄せた。クラクションは v0.15 から
 * 変わらず □。面ボタン（×○△□）は「安全系＋常用トグル」、十字キー（残るのは
 * 左のみ）は「LiDAR ミニマップの拡大表示切替」という役割で整理した。
 *
 * ⚠ **パッドのブレーキ全開（L1、デジタル）は v0.16 で廃止した。** L1 は
 * ギアチェンジ（当時は Dレンジ、2026-08-30 に R レンジへ変更）に譲ったので、
 * パッドのブレーキは L2（アナログ）だけになる。
 * L2 を奥まで踏み込めば `brakeTrig` がほぼ 1.0 になり実質フルブレーキと変わらないため、
 * 機能としての欠落は無い。キーボードの `Space` は従来どおりフル（デジタル）のまま。
 *
 * ゲームパッドの右スティック（v0.15 まで未使用）は、ラジコンタブの表示調整に使う。
 * 上下（軸3）で LiDAR ミニマップのズームイン/アウト（`live.lidarMiniZoomM`）、
 * 十字キー左でミニマップの拡大表示切替（`ui.lidarExpanded`、画面でミニマップを
 * クリックするのと同じ）。**R3（右スティック押し込み）は前後カメラの入れ替え**
 * （`ui.mainCam`、`RcView.tsx` の PIP クリックと同じ、v0.16）——レーシングゲームの
 * 「リアビュー確認」に近い操作なのでここに割り当てた（十字キー左は v0.15 まで R3
 * だった LiDAR 拡大表示切替に譲った）。**いずれも ARM の有無に関係なく効く**——
 * 走行の入力ではなく画面表示の調整なので、灯火切替と同じ扱い。
 *
 * **サイドブレーキ（v0.13）だけは例外でトグル。** `ui.sideBrakeRequested` を
 * `AuxPanel.tsx` の ON/OFF ボタンで切り替え、ここでは毎フレームその値をそのまま
 * 送るだけ（`brake`/`horn` のようにキー・パッドの押下状態からは作らない）。
 * 駐車ブレーキと同じ「かけたら手を離せる」操作感にするため。未 ARM では送らない。
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
 * - `Esc` / 画面の E-STOP ボタン / armed 中のパッドの × … 即 DISARM。**権限も条件も要らない**
 * - ウィンドウのフォーカス喪失・タブが背後に回る … 即 DISARM
 * - 送信が止まれば `SAFETY.cmdDeadmanMs` でサーバが DISARM、さらに io_node が同じだけで DISARM
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
 * `auto/cmd` に差し替える。上のデッドマン（ARM 保持・フォーカス喪失・送信途絶）が
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
 * rAF はタブが隠れると止まる ＝ 送信も止まる ＝ `DEADMAN_MS` でサーバが DISARM。
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
 * ### 3. 逆キーはブレーキ（'speed'/'torque' のみ）
 *
 * 前進中の S は「後退を積分し始める」のではなく**強い減速**。0 を跨いだら後退に移る。
 * 止めたいときに止まらないのが一番怖い。
 *
 * ⚠ **MT では逆キーブレーキを使わない（2026-08-30）。** 実車のアクセルペダルは
 * 1つしか無く、「逆に踏むペダル」は存在しないため、MT ではギアの向き（`ui.mtGear`）と
 * 逆のキーは**何もしない**——押していないのと同じ扱いになり、`mtEngineBrakeRate`
 * （惰行／エンジンブレーキ）で減速する。止めたいときは Space/L2 の実ブレーキを使う。
 */
import { useEffect, useRef } from 'react'
import { cmdOut, live } from '../bus/live'
import {
  ACCEL_SAFETY_LIMIT, LIGHT_CYCLE, LIGHT_OFF, STEER_RATE_SAFETY_LIMIT,
  mtGearAt, mtGearIndex, mtGearRatio, useUi,
} from '../store/ui'
import { SAFETY } from '../generated/vehicle'
import type { ControlChannel } from '../ws/control'

/** 送信レート。telemetry_node の `CMD_PUB_HZ` に合わせる */
const TX_HZ = 50
const TX_INTERVAL_MS = 1000 / TX_HZ

/**
 * 送信が途絶えてから車が止まるまで [ms]。
 *
 * **数字を GUI に手書きしない。** `config/vehicle.toml` の `[safety]` が正で、
 * `config/generate.py` がここへ配り、Pi 側は `raspi/core/vehicle.py` から
 * 同じ値を読む。以前は Pi の2つの定数と GUI のコメントに 150 が3回書いてあり、
 * 一致していることを保証する仕組みが無かった（2026-08-21 のレビュー 🟢11）。
 */
const DEADMAN_MS = SAFETY.cmdDeadmanMs

// **送信周期がデッドマンに近づいたら設計が破綻している。** 1発落としただけで
// 停止する状態になるので、開発中に気づけるようにここで見ておく
if (import.meta.env.DEV && TX_INTERVAL_MS * 3 > DEADMAN_MS) {
  console.warn(
    `送信周期 ${TX_INTERVAL_MS}ms に対しデッドマンが ${DEADMAN_MS}ms しかない。` +
      '取りこぼし数発で DISARM に落ちる',
  )
}
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
// **左スティック（舵）のデッドゾーン／EXPO は設定パネルから調整する
// （`s.gamepadSteerDeadzone`/`s.gamepadSteerExpo`、`store/ui.ts`）。** v0.14 までは
// ここに `STICK_DZ`/`STEER_EXPO` として直書きしていたが、実車ごとに詰めたい値なので
// 他の速度・舵パラメータと同じく設定パネル行きにした（v0.15）。
const TRIGGER_DZ = 0.06
/** 中央付近を緩くする度合い（0=リニア, 1=完全3乗）。スロットルは控えめのまま固定
 * （左右対称なペダルではなく踏み込み量なので、舵ほど中央を緩める必要が無い） */
const THROTTLE_EXPO = 0.25
/** 右スティック（LiDAR ミニマップ操作用）のデッドゾーン */
const RSTICK_DZ = 0.15
/** 右スティック上下いっぱいで 1 秒あたり何 m ズームが変わるか */
const MINI_ZOOM_RATE_M_S = 4
const MINI_ZOOM_MIN_M = 1.5
const MINI_ZOOM_MAX_M = 8

const SPEED_KEYS_FWD = ['KeyW', 'ArrowUp']
const SPEED_KEYS_REV = ['KeyS', 'ArrowDown']
// v0.18: ←/→ はギアチェンジに譲ったので、キーボードの舵は A/D だけになった
// （下の GEAR_KEY_DOWN/UP 参照）
const STEER_KEYS_LEFT = ['KeyA']
const STEER_KEYS_RIGHT = ['KeyD']
const BRAKE_KEY = 'Space'
const HORN_KEY = 'KeyH'
const PASSING_KEY = 'KeyP'
const LIGHT_KEY = 'KeyL'
/** ギアチェンジ（v0.18）。パッドの L1/R1 と同じ役割——←＝シフトダウン、→＝シフトアップ
 * （`shiftGear()` 参照）。灯火と同じくキーの立ち上がりエッジで1段だけ動く、押しっぱなし無効 */
const GEAR_KEY_DOWN = 'ArrowLeft'
const GEAR_KEY_UP = 'ArrowRight'
const DRIVING_KEYS = [
  ...SPEED_KEYS_FWD, ...SPEED_KEYS_REV, ...STEER_KEYS_LEFT, ...STEER_KEYS_RIGHT,
  BRAKE_KEY, HORN_KEY, PASSING_KEY,
]

// ── ゲームパッドのボタン番号（standard mapping） ──
//
// v0.15 で日本の SCE 標準（○＝決定/×＝キャンセル）に合わせて再配置した。
// v0.19: ○ 専用の E-STOP ボタンを廃止し、× の ARM/DISARM トグルに統合した
// （下記 `tick()` の `padArm` 参照。理由は下のブロックコメント）。空いた ○ は
// パッシングに割り当てている。面ボタン（×○△□）は「安全系＋常用トグル」、
// L1/R1 はギアチェンジ専任（v0.16）、十字キーは表示系の切替に役割を分けてある。
// 標準マッピングでない実機・OS の組み合わせでは番号がズレることがあるので、
// 繋いだら一度 GUI 上で反応を確認すること
// （`import.meta.env.DEV` では押されたボタン番号をコンソールに出す。下記参照）。
/** × — ARM/DISARM トグル（Enter キーと同じ）。armed→unarmed の遷移だけ `ch.estop()`
 * も呼ぶ（v0.19、下記 `tick()` 参照） */
const PAD_ARM_TOGGLE = 0
/** ○ — パッシング（v0.14 までは □、v0.15〜v0.18 は十字キー右。v0.19 までは
 * E-STOP 専用ボタンだった。廃止の経緯は上のブロックコメント参照） */
const PAD_PASSING = 1
/** □ — クラクション（v0.14 までは ×） */
const PAD_HORN = 2
/** △ — 灯火モードを1つ進める（**押した瞬間だけ**） */
const PAD_LIGHT = 3
/** L1 — Rレンジ（v0.16 まではブレーキのデジタル全開。ブースト廃止で空いた R1 と対にして
 * ギアチェンジに寄せた。パッドのブレーキは L2（アナログ、下記）だけになる）。
 * **2026-08-30: D/R を入れ替えた**（それまでは D レンジだった）。
 * **MT モード（v0.17）では意味が変わり、シフトダウンになる**（下記 `tick()` 参照） */
const PAD_GEAR_L1 = 4
/** R1 — Dレンジ（v0.16 まではブースト。機能ごと削除したので空いた）。
 * **2026-08-30: D/R を入れ替えた**（それまでは R レンジだった）。
 * **MT モードではシフトアップになる**（`PAD_GEAR_L1` と同じ理由） */
const PAD_GEAR_R1 = 5
/** R3（右スティック押し込み） — メイン映像の前後カメラ入れ替え（v0.16）。
 * レーシングゲームの「リアビュー確認」に近い操作なのでここに割り当てた */
const PAD_CAM_SWAP = 11
/** 十字キー左 — LiDAR ミニマップの拡大表示切替 */
const PAD_LIDAR_EXPAND = 14

/** 操縦権の再要求はこの間隔まで。rAF ごとに投げると 60発/秒になる */
const TAKE_CONTROL_MS = 250

/**
 * ボタンが押されているか。**`.pressed` だけでなく `.value` も見る**（v0.16）。
 * Bluetooth 接続のコントローラでは、ブラウザ／OS の組み合わせによって
 * デジタルボタンの `.pressed` が正しく反映されない個体・環境があり得るため
 * （アナログの `.value` は生きているのに `.pressed` だけ立たない、という報告があった）、
 * どちらか一方が閾値を超えていれば「押されている」とみなす保険を入れてある。
 */
function padPressed(b: GamepadButton | undefined): boolean {
  if (!b) return false
  return b.pressed || b.value > 0.5
}

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

/**
 * ギアを1段シフトする（v0.18）。`delta=-1` でダウン、`+1` でアップ。
 * MT モードでは `ui.mtGear` を `MT_GEARS` 上で1段動かし（R をまたぐときだけ
 * 走行中は拒否）、それ以外は従来どおり `ui.gear`（D/R）を direct-set する
 * （down=R、up=D。**2026-08-30 に D/R を入れ替えた**——それまでは down=D、up=R
 * だった）。パッドの L1/R1（`tick()` 内）とキーボードの←/→（`down` ハンドラ）
 * の両方から呼ぶ共通ロジック——以前は2箇所に同じ分岐を書いていたのをここへ集約した。
 */
function shiftGear(delta: number) {
  const ui = useUi.getState()
  const stopped = live.vs == null || live.vs.stopped
  if (ui.settings.driveMode === 'mt') {
    const next = mtGearAt(mtGearIndex(ui.mtGear) + delta)
    if (next === ui.mtGear) return
    const crossesReverse = next === 'R' || ui.mtGear === 'R'
    if (crossesReverse && !stopped) return
    ui.set({ mtGear: next })
  } else {
    const target = delta < 0 ? 'R' : 'D'
    if (ui.gear === target || !stopped) return
    ui.set({ gear: target })
  }
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
      if (e.code === LIGHT_KEY) {
        // **押した瞬間に1つ進めるだけ。** 押しっぱなしで回り続けると
        // どのモードで手を離したのか分からなくなる（`e.repeat` は上で弾いてある）
        const i = LIGHT_CYCLE.indexOf(ui.lightMode)
        ui.set({ lightMode: LIGHT_CYCLE[(i + 1) % LIGHT_CYCLE.length] })
        return
      }
      if (e.code === GEAR_KEY_DOWN || e.code === GEAR_KEY_UP) {
        e.preventDefault() // 矢印キーでページがスクロールしないように
        shiftGear(e.code === GEAR_KEY_DOWN ? -1 : 1)
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
    let padLightWas = false
    let padArmWas = false
    let padGearL1Was = false
    let padGearR1Was = false
    let padLidarExpandWas = false
    let padCamSwapWas = false
    /** DEV 診断用（下記の `padPressed` ログ）。ボタンごとの直前の押下状態 */
    const padDebugButtons: Record<string, boolean> = {}

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

      // ── 入力を読む ──
      const k = keys.current
      const fwd = SPEED_KEYS_FWD.some((c) => k.has(c))
      const rev = SPEED_KEYS_REV.some((c) => k.has(c))
      const left = STEER_KEYS_LEFT.some((c) => k.has(c))
      const right = STEER_KEYS_RIGHT.some((c) => k.has(c))
      const keyActive = fwd || rev || left || right

      // R2＝アクセル、L2＝ブレーキ（DUALSHOCK4 の実車ペダル配置、v0.14）。
      // 前進/後退は `ui.gear`（画面の Dレンジ/Rレンジ）が決める。R2/L2 単体では
      // 向きを持たない——実車のアクセル/ブレーキペダルと同じ
      const accelTrig = dz(pad?.buttons[7]?.value ?? 0, TRIGGER_DZ)
      const brakeTrig = dz(pad?.buttons[6]?.value ?? 0, TRIGGER_DZ)
      const padSteer = dz(pad?.axes[0] ?? 0, s.gamepadSteerDeadzone)
      const padActive = accelTrig > 0 || brakeTrig > 0 || padSteer !== 0

      // v0.17: 制御方式は「速度制御 / トルク制御 / MT（擬似変速）」の3択（`driveMode`）。
      // MT は速度制御の一種——ワイヤプロトコルには torque_mode=false（速度指令）で送る
      const torqueModeOn = s.driveMode === 'torque'
      const mtModeOn = s.driveMode === 'mt'
      // MT モードでは向きも `ui.mtGear`（R, N, D1〜D5）が持つ。それ以外は従来どおり `ui.gear`
      const gearSign = mtModeOn ? (ui.mtGear === 'R' ? -1 : 1) : (ui.gear === 'R' ? -1 : 1)

      // v0.16: ブースト機能を削除した。常に `maxSpeed` そのものが上限。
      // v0.17: MT モードだけ `mtGear` に応じた倍率（`mtGear1`〜`mtGear5`）を掛ける。
      // 2026-08-30: MT は `maxSpeed` ではなく専用の `mtMaxSpeed` を使う（'speed' と非共有）
      const mtRatio = mtModeOn ? mtGearRatio(ui.mtGear, s) : 0
      const maxSpeed = mtModeOn ? s.mtMaxSpeed * mtRatio : s.maxSpeed
      // MT モードのエンジンブレーキ相当レート（2026-08-30）。ギア比が低いほど強く、
      // N は加算なし（`mtCoast` のみ＝いちばん緩い）。アクセルオフの惰行と、
      // シフトダウンで上限を超えたときのなだらかな減速の両方に使う（下記 `tick()` 内）
      const mtEngineBrakeRate = mtModeOn
        ? (ui.mtGear === 'N' ? s.mtCoast : s.mtCoast + s.mtEngineBrake * (1 - mtRatio))
        : 0

      // ── 補機（すべて「押している間だけ」） ──
      //
      // ブレーキはキーボードの Space（フル、デジタル）と L2（力加減、アナログ）の合成。
      // パッドの L1 は v0.16 でギアチェンジに譲ったので、パッドのブレーキは L2 のみ。
      // **L2 は踏み込み量をそのまま `brake_torque` の倍率にする**（`s.brakeTorque` が
      // 上限）。`_brake_torque_raw`（`msgs/convert.py`）は正の値を 0 に丸めないので、
      // 軽く踏んだときに「未指定＝最大制動」に化けることはない
      const brakeStrength = Math.max(k.has(BRAKE_KEY) ? 1 : 0, brakeTrig)
      const brake = brakeStrength > 0
      const horn = k.has(HORN_KEY) || padPressed(pad?.buttons[PAD_HORN])
      const passing = k.has(PASSING_KEY) || padPressed(pad?.buttons[PAD_PASSING])

      // 灯火切替だけはトグル。**押した瞬間のエッジで1つ進める**
      // （rAF で読むので、キーの `e.repeat` に相当する処理を自前で持つ）
      const padLight = padPressed(pad?.buttons[PAD_LIGHT])
      if (padLight && !padLightWas) {
        const i = LIGHT_CYCLE.indexOf(ui.lightMode)
        ui.set({ lightMode: LIGHT_CYCLE[(i + 1) % LIGHT_CYCLE.length] })
      }
      padLightWas = padLight

      // × は Enter と同じ ARM/DISARM トグル（v0.15）。**押した瞬間のエッジで反転**
      // （灯火と同じ理由——押しっぱなしで何度も切り替わっては困る）
      //
      // v0.19: ○ 専用の E-STOP ボタンを廃止し、armed→unarmed の遷移にここで
      // `ch.estop()` を吸収させた。E-STOP は元々「即 DISARM（権限も条件も要らない）」
      // 以上のことをしていなかった（`estop_active` はハードウェアの物理ボタン専用の
      // ラッチで、GUI/パッドからは立てられない——`raspi/core/link_tracker.py` 参照）ので、
      // ボタンを分ける意味が無かった。`ch.estop()` は**操縦権を持っていなくても通る**
      // ので、他のタブ/PCが操縦している状態からでもこの端末の × で止められる
      // （ローカルの `ui.armRequested` を false にするだけの経路より強い）。
      const padArm = padPressed(pad?.buttons[PAD_ARM_TOGGLE])
      if (padArm && !padArmWas) {
        if (ui.armRequested) {
          ch.estop()
          ui.set({ armRequested: false, disarmReason: 'パッドの × で解除' })
        } else {
          lastActivity.current = now
          ui.set({ armRequested: true, disarmReason: '' })
        }
      }
      padArmWas = padArm

      // Rレンジ/Dレンジの切替（L1/R1、v0.16。v0.15 までは十字キー上/下だったが、
      // ブースト廃止で空いた L1/R1 に寄せた。2026-08-30 に D/R を入れ替え、
      // L1=R・R1=D になった）。v0.17 で MT モードでは「シフトダウン/アップ」という
      // 相対操作に変わった。v0.18 でキーボードの←/→にも同じ操作を割り当てたため、
      // 分岐ロジックは `shiftGear()` に共通化してある（`down` ハンドラ参照）
      const padGearL1 = padPressed(pad?.buttons[PAD_GEAR_L1])
      const padGearR1 = padPressed(pad?.buttons[PAD_GEAR_R1])
      if (padGearL1 && !padGearL1Was) shiftGear(-1)
      if (padGearR1 && !padGearR1Was) shiftGear(1)
      padGearL1Was = padGearL1
      padGearR1Was = padGearR1

      // 右スティック＝ LiDAR ミニマップの操作（v0.15）。走行には関与しないので
      // ARM の有無に関係なく効く。上下（軸3）でズーム、十字キー左で拡大表示切替
      const padZoomY = dz(pad?.axes[3] ?? 0, RSTICK_DZ)
      if (padZoomY !== 0) {
        const z = live.lidarMiniZoomM + padZoomY * MINI_ZOOM_RATE_M_S * dt
        live.lidarMiniZoomM = Math.max(MINI_ZOOM_MIN_M, Math.min(MINI_ZOOM_MAX_M, z))
      }
      const padLidarExpand = padPressed(pad?.buttons[PAD_LIDAR_EXPAND])
      if (padLidarExpand && !padLidarExpandWas) ui.set({ lidarExpanded: !ui.lidarExpanded })
      padLidarExpandWas = padLidarExpand

      // R3＝前後カメラ入れ替え（v0.16）。`RcView.tsx` の PIP クリックと同じ
      // `ui.mainCam` を更新するだけ。走行には関与しないので ARM の有無に関係なく効く
      const padCamSwap = padPressed(pad?.buttons[PAD_CAM_SWAP])
      if (padCamSwap && !padCamSwapWas) {
        ui.set({ mainCam: ui.mainCam === 'front' ? 'rear' : 'front' })
      }
      padCamSwapWas = padCamSwap

      // ── DEV 診断：どのボタン/軸が実際に反応しているかをコンソールに出す ──
      //
      // Bluetooth 接続のコントローラは OS・ブラウザの組み合わせでボタン番号が
      // ズレることがあるので、繋いだコントローラの実際の番号を確認したいときは
      // devtools のコンソールを開いて押し、上の `PAD_*` 定数と見比べること。
      //
      // ⚠ v0.16 の再配線直後に「ARM はできるが E-STOP は効かない」という報告が
      // あったが、これはボタン番号のズレではなく、E-STOP が `estop_active`
      // （車両側の物理ボタンだけが立てられるハードウェアラッチ）を立てるものだと
      // 誤解していたことが原因だった（実際は単なる強い DISARM）。v0.19 で
      // ○ 専用の E-STOP ボタン自体を廃止したので、この非対称は起こり得ない。
      if (import.meta.env.DEV && pad) {
        pad.buttons.forEach((b, i) => {
          const p = padPressed(b)
          const key = `pad-btn-${i}`
          if (p !== padDebugButtons[key]) {
            console.log(`[gamepad] button[${i}] ${p ? 'DOWN' : 'up'} (pressed=${b.pressed}, value=${b.value.toFixed(2)})`)
            padDebugButtons[key] = p
          }
        })
      }

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
        // 消灯に戻したあとは送信が止まり、`DEADMAN_MS` で io_node が DISARM_COMMAND
        //（`flags=0`）に落とすので、灯火はそのまま消える。
        const wantAux = horn || passing || ui.lightMode !== LIGHT_OFF ||
          ui.winkerLeftRequested || ui.winkerRightRequested
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
        // ブレーキ・サイドブレーキは未 ARM では意味を持たない（モータが励磁されていない）
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
          side_brake: false,
          // ウィンカーは純粋な灯火系（後輪モータの励磁と無関係）なので、
          // side_brake と違い未 ARM でも送る（灯火・ホーンと同じ理由）
          winker_left: ui.winkerLeftRequested,
          winker_right: ui.winkerRightRequested,
        })
        return
      }

      // ⚠ **自律走行中は無操作タイムアウトを掛けない。** 人が何も触らないのが
      // 正常な状態なので、20秒で勝手に止まったら自律走行にならない。
      // 代わりの受け皿は従来どおり全部生きている（Esc / E-STOP / フォーカス喪失 /
      // タブが背後に回る / 送信が止まれば `DEADMAN_MS` で DISARM）。
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
          // L2 の踏み込み量（`brakeStrength`）で力を加減する。L1/Space はフル（=1）
          brake_torque: s.brakeTorque * brakeStrength,
          // 自律走行はトルク直接指令を使わない。サーバ側でも落としてある
          torque_mode: false,
          target_torque: 0,
          auto_stop: s.autoStop,
          // **人間のサイドブレーキもそのまま通す**（brake と同じ理由）
          side_brake: ui.sideBrakeRequested,
          winker_left: ui.winkerLeftRequested,
          winker_right: ui.winkerRightRequested,
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
        steer.current = -expo(padSteer, s.gamepadSteerExpo) * s.maxSteer
        // R2（アクセル）の踏み込み量に `gearSign`（Dレンジ/Rレンジ）で向きを付ける。
        // L2 はここでは使わない——ブレーキとして `brake`/`brakeStrength` 側で処理済み
        if (torqueModeOn) {
          // トルクモード（v0.6）も同じ理由でランプなし。速度指令は使わないので 0
          torque.current = expo(accelTrig, THROTTLE_EXPO) * s.driveTorque * gearSign
          speed.current = 0
        } else if (mtModeOn) {
          // MT（2026-08-30）: 目標（トリガー踏み込み量、離していれば0）へ
          // `approach()` でレート制限しながら近づける。**目標へ近づく向きで
          // レートを変える**——増える方向（加速）は `s.mtAccel`、減る方向
          // （アクセルオフの惰行・シフトダウンで上限を超えた分の解消）は
          // `mtEngineBrakeRate`。両方とも一つの式で表せるので分岐が要らない
          // （以前は「アクセルON/OFF」「上限超過」を別々の分岐にしていたが、
          // 「加速側は直接代入でランプが無い」バグが混ざっていた）。
          // `accel` ではなく `mtAccel` を使う——'speed' モードとパラメータを共有しない
          const mtTarget = accelTrig > 0 ? expo(accelTrig, THROTTLE_EXPO) * maxSpeed * gearSign : 0
          const mtRate = Math.abs(mtTarget) > Math.abs(speed.current) ? s.mtAccel : mtEngineBrakeRate
          speed.current = approach(speed.current, mtTarget, mtRate, dt)
          torque.current = 0
        } else {
          speed.current = expo(accelTrig, THROTTLE_EXPO) * maxSpeed * gearSign
          torque.current = 0
        }
      } else {
        const dir = fwd ? 1 : rev ? -1 : 0
        if (torqueModeOn) {
          // **トルクモードはランプを挟まない。** 0.1N・m 程度と値域が小さく、
          // 速度用の加速度ランプ（m/s²）はそのままでは単位が合わない。
          // 押している間だけ `driveTorque` をそのまま出す（v0.6）
          torque.current = dir * s.driveTorque
          speed.current = 0
        } else if (mtModeOn) {
          // MT（2026-08-30）: 実車にアクセルペダルは1つしか無い——ギアの向きと
          // 逆のキーを押しても何もしない（`brake` を使った強制ブレーキはしない。
          // 押していないのと同じ扱いになり、下でエンジンブレーキ／惰行が効く）。
          // 発進キックも廃止——`s.mtAccel` で 0 からそのままランプする
          const effDir = dir === gearSign ? dir : 0
          if (Math.abs(speed.current) > maxSpeed) {
            speed.current = approach(speed.current, Math.sign(speed.current) * maxSpeed, mtEngineBrakeRate, dt)
          } else if (effDir !== 0 && maxSpeed > 0) {
            speed.current += effDir * s.mtAccel * dt
          } else {
            speed.current = approach(speed.current, 0, mtEngineBrakeRate, dt)
          }
          torque.current = 0
        } else {
          // ── 速度：押している間は加速、逆キーはブレーキ ──
          if (dir === 0) {
            speed.current = approach(speed.current, 0, s.coast, dt)
          } else if (speed.current * dir < 0) {
            // 進行方向と逆を押している ＝ ブレーキ。0 を行き過ぎたら 0 で受け止め、
            // 次のフレームから加速で逆走に移る
            const next = speed.current + dir * s.brake * dt
            speed.current = next * dir > 0 ? 0 : next
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

      // MT は上のエンジンブレーキのランプで既に上限内へなだらかに収れんさせてある。
      // ここで即クランプすると、シフトダウンした瞬間に速度がワープしてしまう
      if (!mtModeOn) {
        speed.current = Math.max(-maxSpeed, Math.min(maxSpeed, speed.current))
      }
      torque.current = Math.max(-s.driveTorque, Math.min(s.driveTorque, torque.current))
      steer.current = Math.max(-s.maxSteer, Math.min(s.maxSteer, steer.current))

      cmdOut.speed = speed.current
      cmdOut.steer = steer.current
      cmdOut.active = true
      cmdOut.auto = false
      cmdOut.torqueMode = torqueModeOn
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
        // 「未指定 ＝ STM32 の最大制動」を意味する。スライダの下限は 0 より上にしてある。
        // L2 の踏み込み量（`brakeStrength`）で力を加減する。L1/Space はフル（=1）で、
        // `_brake_torque_raw` は正の値を 0 に丸めないので軽く踏んでも「未指定」化けはしない
        brake_torque: s.brakeTorque * brakeStrength,
        // v0.6: 立っている間 `speed` は無視され `target_torque` が駆動トルクとして直接掛かる
        // v0.17: MT モードは `torqueModeOn===false` なので、ここは自動的に速度指令になる
        torque_mode: torqueModeOn,
        target_torque: torque.current,
        // v0.7: 進行方向の超音波が 20cm 未満なら STM32 が単独で最大制動する。
        // **GUI は許可を出すだけで、判定にも制動にも関与しない**（二重制御にしない）
        auto_stop: s.autoStop,
        // v0.13: `braking`等と違いトグル。`AuxPanel.tsx` の ON/OFF で切り替える
        side_brake: ui.sideBrakeRequested,
        // v0.14: ウィンカーも `side_brake` と同じくトグル。灯火系なので未 ARM でも送る
        winker_left: ui.winkerLeftRequested,
        winker_right: ui.winkerRightRequested,
      })
    }

    raf = requestAnimationFrame(tick)
    return () => cancelAnimationFrame(raf)
  }, [ch])
}
