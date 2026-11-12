# SURGE Mark.2 — STM32 側 実装仕様書

**バージョン**: v0.30（`uart_protocol.md` v0.30 に対応。`protocol_version = 0x001E`）
**対象読者**: STM32 ファームウェアを読む・直す人
**関連文書**: [`uart_protocol.md`](uart_protocol.md)（プロトコルの説明）, [`system_overview.md`](system_overview.md)（全体設計）

> **本書の位置づけ**
> 数値・バイト配置・ビット定義の唯一の定義は `raspi/proto/protocol.toml`、その説明は `uart_protocol.md`。本書は
> 「STM32 側が何をどう実装しているか」の要点だけを書き、パケットのレイアウト・ビット定義・パラメータ表・数値は**複製しない**。
> 食い違ったら toml → `uart_protocol.md` → ファームのコードの順に確かめ、本書を直す。
> ファームのソースは `AutonomousCar/Program/MainF446RE_V3/`（通信は `src/comm/ras_link.[ch]`、車両の状態は `src/vehicle/vehicle.[ch]`、制御は `src/control/`）。

---

## 1. STM32 側の責務

| 分類 | 内容 |
|---|---|
| **制御** | 後輪左右モータのトルク制御、ステアモータの位置制御、車速 PI |
| **車両制御** | TC、ABS、片輪浮き対策、TV、ローンチコントロール、ブレーキホールド |
| **センサ** | LD06 のパース・1°ビニング、超音波、前輪エンコーダ（ADC）、IMU、温度、電圧・電流、車速のカルマンフィルタ |
| **単位換算** | モータ機械角 ⇄ 路面舵角、車輪角速度 → 周速 [m/s]、エンコーダ角 → 累積距離 [0.1mm]、`speed` の車体中心線方向への射影 |
| **保護・安全** | 電流・温度リミッタ、低電圧警告、過電流ラッチ、`COMMAND` 途絶時の自動ブレーキ、ハートビート監視（PB12）、自動停止 |

やらないこと: 経路生成・自己位置推定・地図生成、障害物の意味判断、ステア原点の自動探索（§9）。

**Pi は車輪半径・ギア比を知らない。** UART 上の物理量はすべて SI 単位で、換算は STM32 が行う。
例外はステアのリンク換算の係数で、Pi が `CONFIG_SET` で送る（`uart_protocol.md` §5.8.4）。

---

## 2. UART の設定

ボーレート・フォーマット・配線は `uart_protocol.md` §1。USART1（APB2 45MHz）で USARTDIV = 2.8125、分周誤差 0%。

| 用途 | サイズ | 備考 |
|---|---|---|
| RX 循環 DMA バッファ | 2048 B（`RAS_SERIAL_RX_BUF_SIZE`、`src/app/app.c`） | 1Mbps で約 20ms 分。これより長くポーリングが空くと上書きとみなして読み捨てる（`Serial_Available`） |
| TX キュー | 優先度ごとに PONG 64 / TELEMETRY 256 / LIDAR 512 / INFO 1024 / LOG 512 B | `ras_link.h` の `tx_*_storage` |

---

## 3. フレーム構造と CRC

フレーム構造・CRC-16/CCITT-FALSE は `uart_protocol.md` §3。実装は `lib/crc16/crc16.h`・`ras_link.c`（変えたら検査値 `0x29B1` で確かめる）。

- `SEQ` と CRC は**DMA 起動の直前**に確定する（キューへ積むときではない）。送出順に SEQ が単調増加し、`t_pong_tx_us` も同じ場所で書ける
- MD との通信の CRC-8/AUTOSAR とは別物（STM32 内部で完結）

---

## 4. パケット定義

### 4.1 パケット一覧とレイアウト

TYPE・LEN・方向・オフセット・スケールは `protocol.toml`（説明は `uart_protocol.md` §4・§5）。
`raspi/proto/generate.py` は C ヘッダ `raspi/proto/generated/surge_proto.h` も生成するが、**ファームはこれを取り込まず**、
`ras_link.h`・`ras_link.c` に TYPE・LEN・ビット定義・`RAS_PROTOCOL_VERSION` を手で写している。
**toml を変えたらファームの `RAS_LEN_*`・`RAS_FLAG_*`・`RAS_CMD_FLAG*`・`RAS_PROTOCOL_VERSION` も同時に直し、`protocol_version` を上げる**
（Pi は版数が完全に一致しない STM32 には ARM を通さない。手順は `raspi/proto/README.md`）。

### 4.2 構造体の注意

ペイロードはフレームの 5バイト目から始まるので `uint32_t` / `int32_t` は 4バイト境界に乗らない。
構造体で扱うなら `#pragma pack(push, 1)` と `_Static_assert(sizeof(...) == LEN)`（LEN は toml）を付け、受信フレームからは `memcpy` で取り出す
（非 packed 構造体へのキャストは未定義動作で、ARM ではハードフォールトしうる）。現行の `ras_link.c` はバイト列へ順に `memcpy` して組み立てている。

### 4.3 実装上の注意（TELEMETRY・各パケット）

フィールドの意味は `uart_protocol.md` §5.3。STM32 側で守っていること:

- **`speed` は車体中心線方向に射影済み、`wheel_speed[FL/FR]`・`odom_dist` は射影なしの生値。** Pi がスリップ計算・オドメトリの射影に生値を使う
- `speed` は走行距離と加速度計のカルマンフィルタ（`speed_kf.[ch]`）の車速。`wheel_speed` は各輪の周速を `DRIVE_LPF_TAU_FRONT_S` の1次ローパスに通した報告用の値
  （ADC を1制御周期ぶん平均した角度から微分する。車速の計算には使わない）
- `odom_dist` はローパスを掛けず、前輪エンコーダの鏡像（`encoder_direction.c`）と折り返しを直した増分（車速 KF に入れるのと同じ値。前進が正）を積算する
- `torque_cmd` は MD に出した最終指令値で実測ではない。制動中・停車保持中も制動トルクを負で入れる
- `steer_actual` は MD が返す機械角から換算した路面舵角で、指令のコピーにしない（Pi が遅延を実測するため）
- `t_us` は中身のスナップショットを取った時刻（送信時刻ではない）。`cmd_seq_echo` だけは送信時点で最後に受けた `COMMAND` の SEQ
- `VERSION`・`LIMITS` は起動直後に 3回（100ms 間隔）自発送信し、`*_REQ` にも返す。`LIMITS` はステアのリンク換算を適用したときにも送り直す
- `STATS` は起動からの累積値で 0 クリアしない
- `LOG` は本文を 127 文字で打ち切る（`RAS_LOG_MAX_LEN` = 128 = severity + 本文）。TX が逼迫したら破棄して `STATS.tx_drop` に数える
- `CONFIG_SET` / `CONFIG_GET` には必ず `CONFIG_ACK` を返す（不明な `param_id` でも `result=1`）

---

## 5. 送信の実装

### 5.1 送信スケジュール

| パケット | 送信タイミング |
|---|---|
| `PONG` | `PING` 受信時に即座（最優先キュー） |
| `TELEMETRY` | 10ms 周期（`RAS_TELEMETRY_INTERVAL_US`） |
| `LIDAR_SECTOR` | セクタ（30°）が揃うたび（10Hz 回転で約 8.3ms ごと） |
| `CONFIG_ACK` | `CONFIG_SET` / `CONFIG_GET` 受信時 |
| `VERSION` / `LIMITS` | 起動直後 3回 + 各 `*_REQ` 受信時 |
| `STATS` | 1Hz |
| `LOG` | 任意（キューに空きがあるときのみ） |

### 5.2 送信優先度と TX キュー + DMA

優先度は `uart_protocol.md` §7（PONG > TELEMETRY > LIDAR > INFO（CONFIG_ACK・VERSION・LIMITS・STATS）> LOG）。各キューが満杯なら積まずに破棄する（`tx_drop`）。

**ブロッキング送信（`HAL_UART_Transmit`）は使わない。** 1 フレームに約 1ms かかり、その間 2kHz の制御ループが止まる。

1. 送信は優先度ごとのキューへ積むだけで即座に戻る
2. DMA が動いていなければ、優先度の高いキューから1フレーム取り出して DMA 起動（このとき SEQ・`t_pong_tx_us`・CRC を確定。`PumpTx`）
3. DMA 完了（UART の TC 割り込み）で次のフレームを起動する

送信中の DMA を途中で中断しないので、`TELEMETRY` の最大待ちは送信中の1フレーム分（約 1ms）。

---

## 6. 受信の実装

### 6.1 DMA + IDLE ライン検出

`HAL_UART_Receive_DMA` の循環受信（`lib/serial`）で受け、メインループでフレームを切り出す。**割り込みハンドラ内でフレーム処理まではしない。**
UART の IDLE 割り込みは別に有効にし（`RasLink_Init`）、時刻と DMA の書き込み位置だけを記録する（`RasLink_OnUartIdle`）。

**`t_ping_rx_us` はこの IDLE 時刻を使う**: フレームの末尾が記録位置と一致すれば IDLE 時刻＝最終バイトの到着時刻、一致しなければ（後続バイトが続いている）現在時刻で代用する。
メインループで取るとループ周期分のジッタがクロックオフセットのバイアスになる。

### 6.2 フレーム抽出

- `0xAA 0xAA 0x55` を取りこぼさない（SYNC1 待ちで `0xAA` を受けたら SYNC1 待ちに留まる）
- **`LEN = 0`（`VERSION_REQ`・`LIMITS_REQ`）でペイロード 0 バイトのまま通過する経路を持つ**
- 期待 LEN は TYPE ごとに一意（`ras_link.c` の `ExpectedPayloadLen`。値は toml）。LEN 不一致・未知の TYPE（方向規約違反を含む）は破棄し、
  **`LEN` を信用せずその場で SYNC の探し直しに戻る**（化けた LEN ぶん最大 257B を読み飛ばすと、その間の `COMMAND` を失い途絶の自動ブレーキに入りうる）

### 6.3 エラー処理と統計

| 事象 | 対応 |
|---|---|
| CRC 不一致 | 破棄。`rx_crc_error++` |
| `LEN` が TYPE の期待値と不一致 | 破棄。`rx_len_error++` |
| 未知の `TYPE` | 破棄。`rx_unknown_type++` |
| USART の ORE / FE / NE | エラー割り込み（`PEIE`・`EIE`）を切り、受信 DMA を止めさせない（`RasLink_Init`）。化けたフレームは CRC で落ちる |

**HAL はエラー割り込みで受信 DMA を中断する**ので、オーバーランで受信が止まったまま気づかない、が最も多い失敗。統計は `STATS`（1Hz、累積値）で Pi へ送る。

---

## 7. LiDAR のビニング処理

実装は `src/sensing/lidar.[ch]`。LD06 の生データ（約 450点/周、約 0.8° 刻み）を **1° 刻みの 360点**に整形し、30点（= 30°）ごとにセクタとして送る。

- **各ビン中心（整数度）の距離は、前後2点から線形補間する**（角度の丸め誤差を無くすため）。2点の距離差が `LIDAR_INTERP_MAX_DIFF_MM`（50mm）を超える・
  どちらかが無効（0）・角度の間隔が `LIDAR_INTERP_MAX_GAP_DEG`（2°）を超えるときは、実在しない点を作らないよう**ビン中心に近い方の点**をそのまま採る
- **最小距離は採らない。** 衝突判定には安全側でも、地図用のデータを近距離側へ系統的に歪ませ、平均化で消えないバイアスとしてスキャンマッチングの精度を落とす。
  自動停止は送るのとは別の永続配列（`point_distance_mm`・`sector_update_us`）で STM32 内部で判定する
- **角度はセンサ基準のまま送る。** 左右の鏡像反転は Pi 側（`uart_protocol.md` §5.1）。STM32 で直すと 0°/180° がセクタ境界に乗って1セクタが分裂するうえ、
  試したときに原因不明の通信エラー増発が再現したため
- `sector_idx = bin / 30`。角度が次のセクタへ進んだ時点で完了したセクタを確定する。`t_start_us` はセクタ最初の点の時刻、`duration_us` は最後の点までの経過時間
  （点の時刻は小さな逆行を直してから使う）、`rot_speed_dps` は LD06 が報告する回転速度（1 deg/s 単位）
- LD06 のデータが 300ms（`LIDAR_TIMEOUT_US`）途絶したら `lidar_ok` を落とす

---

## 8. 安全要件

### 8.0 安全層の全体像

`uart_protocol.md` §10（ハートビート断 → `COMMAND` 途絶 → 過電流ラッチ → 低電圧警告）。
**層3（過電流ラッチ）と層4（低電圧警告）を混同しない。** 層3はラッチして走行不能、層4は警告のみ。

### 8.1 ハートビート監視（第1安全層）

挙動は `uart_protocol.md` §9。実装は `src/comm/heartbeat.[ch]`（`HEARTBEAT_MIN_EDGES`・`HEARTBEAT_TIMEOUT_US`）と `vehicle.c` の `UpdateEstop`。
PB12（`RAS_SIG`）を 2kHz の制御ループでポーリングする。**EXTI ライン 12 は前方超音波の ECHO（PA12）に割り当てる**（μs 精度のパルス幅測定が要る。ハートビートは ms 精度で足りる）。

**`estop_active` からの自動復帰はしない**: ハートビートが復活している状態でボタン2が押されて解除し、さらに `COMMAND` の `arm` が一度 0 になって再び 1 になるまで再武装しない。

### 8.2 UART タイムアウト（第2安全層）

`COMMAND` が 100ms（`RAS_COMMAND_TIMEOUT_US`）途絶したら、最大制動 + `uart_timeout` + `armed = 0`。

- 惰行ではなくブレーキ。最大制動トルク（`DRIVE_MAX_BRAKE_TORQUE_NM`）を直接掛ける。舵角は最後の指令のまま。`horn`・`passing`・ウィンカー要求は解除
- 止まったら後輪の位置保持へ移り、止まったまま 30 秒（`VEHICLE_FAILSAFE_POWER_OFF_S`）経つと駆動電源を切る（過熱・電池消耗の防止）
- `COMMAND` が再開したら `uart_timeout` は消えるが、`armed` は `arm` が一度 0 になってから立ち上がるまで戻さない
- 走行中に `arm` が外れたときは、停車（または `DRIVE_MAX_SPEED_M_S ÷ BRAKE_DECEL_M_S2 × VEHICLE_STOP_TIME_MARGIN` 秒）まで駆動電源を入れたまま最大制動を続ける
  （いきなり切ると MD ごと落ちて惰行する。`UpdateDisarmBraking`）

### 8.3 電源保護（第3・第4安全層）

| 系統 | 内容 | 過電流しきい値 | 動作 |
|---|---|---|---|
| 駆動系 (Primary) | モータードライバ電源 | `POWER_DRIVE_OVERCURRENT_THRESHOLD_A` | **DRIVE_POWER をハード遮断してラッチ** |
| シグナル系 (Secondary) | マイコン・Pi・LiDAR・ライト類 | `POWER_SIGNAL_OVERCURRENT_THRESHOLD_A` | LiDAR / 駆動電源を遮断してラッチ |

どちらも 8セル Ni-MH。しきい値・継続時間・低電圧の判定と復帰（ヒステリシス付き）は `src/power/power.h`（値は `uart_protocol.md` §5.3・§5.4）。

**過電流ラッチ中は `drive_power_locked` を立て、`arm` を要求されても `armed` を立てない。** これは仕様で、Pi は FAULT 扱いしない。
立て忘れると Pi からは「原因不明の arm 失敗」に見える。

### 8.4 その他の保護

| 保護 | 動作 | 反映先 |
|---|---|---|
| MD 過熱 | 出力制限 or 遮断。**MD ステア（`temp[2]`）は据え切りで過熱しやすい** | `md_status[2].overheat` |
| MD 通信断 | 直近 100ms で状態フレームを受信できていなければ `comm_ok = 0`（MD が無言になると status は最後の値で固まるため必須） | `md_status[i].comm_ok` |
| MD 異常（`md_fault`） | 駆動電源を入れて `VEHICLE_MD_BOOT_TIMEOUT_S` 待っても3台が揃わない、または走行中に使えない状態が `VEHICLE_MD_FAULT_DEBOUNCE_S` 続いたら、最大制動して**駆動電源が切れるまでラッチ** | `flags` bit23、理由は `LOG` |

**据え切り（停車中の操舵）による MD ステアの過熱**は、この車両で最も起こりやすい故障モード。
Pi 側の planner は停車を確認して 2 秒経ったら舵を中立へ戻す（`raspi/auto/base.py` の `StandstillSteer`）。
STM32 はフェイルセーフで停車して 30 秒経つと駆動電源を切る。**ファーム側に操舵時間の上限は無い。**

### 8.5 `flags` / `md_status` のビット

定義は `uart_protocol.md` §5.4・§5.5（コードは `ras_link.h` の `RAS_FLAG_*`・`RAS_MD_STATUS_*`）。STM32 側の約束:

- `*_ACTIVE` 系（TC・TV・自動停止・ABS・片輪浮き・ローンチ・位置保持・ウィンカー）は「有効/無効」ではなく**今まさに介入・動作しているときだけ**立てる
- 要求したモードを拒否しても、`flags` bit0-1 には実際のモードを入れる
- `md_status` は MD の status バイトをそのまま転送し、`COMM_OK`・`LIMIT_SYNCED` を STM32 が付加する

---

## 9. モードと原点合わせ

- 電源投入時は `DISARM`。`MANUAL` / `AUTO` に入って `armed` が立つには、`steer_center_valid` が立っていることと3台の MD が使えることが要る
- E-Stop・`COMMAND` 途絶のあとの復帰は `arm` を一度下ろしてから（§8.1・§8.2）
- `mode = 3`（予約）を受けたら無視して現在のモードを維持する

**ステア原点合わせは組み立て・整備時の作業**で、走行中のシーケンスは持たない: タイヤを手で真っ直ぐにし、**ボタン1を押しながら電源投入**すると、
そのときのモータ機械角を中心点として Flash に保存する。両端に当てる自動較正は MD ステアの過熱と機構破損のリスクがあるので実装しない。
IMU の静止キャリブレーションは**ボタン2を押しながら起動**（Flash に保存）。

- 起動時に Flash から中心点を読み、有効なら `steer_center_valid` を立てる（無ければ Pi は走行に進まず、GUI に整備手順を出す）
- 可動範囲はモータ角で `STEERING_MAX_ANGLE_RAD` にクランプ
- モータ角 → 路面舵角は `路面舵角 = gain·x + cubic·x³`、x = モータ角 × `STEERING_LINKAGE_RATIO`（`src/control/steering.c`）。gain・cubic は Pi が送る（既定は恒等の 1・0）

---

## 10. パラメータの扱い

`param_id` の一覧と意味は `uart_protocol.md` §5.8。調整パラメータ（TC・ABS・片輪浮き・TV・車速PI・ステアのリンク・ローンチ・FF・前輪エンコーダ校正・車速 KF・
ジャイロ補正・制動の見積もり）は **`src/control/control_params.h` の表が名前・`param_id`・範囲・仮置きの唯一の定義**。
ON/OFF 系（`0x0010`/`0x0020`/`0x0050`/`0x0070`/`0x0090`）・`0x0040`・`0x0060` は `ras_link.h` の `RasParamId`。

1. **`CONFIG_SET` / `CONFIG_GET` には必ず `CONFIG_ACK` を返す。** 不明な ID（欠番を含む）は `result=1`
2. **範囲外はクランプし、`applied` にクランプ後の値を入れて `result=2`**（非数は仮置きの値）。Pi の GUI が実機の真の値を表示できる
3. `CONFIG_GET` は現在値を `applied` に入れて `result=0` で返す
4. `0x0040`（LiDAR フォーマット）は走行中も切り替えを許す
5. ステアのリンク（`0x0081`/`0x0082`）は受けた値を保持するだけで、**停車して DISARM のときに適用**し `LIMITS` を送り直す（`vehicle.c` の `UpdateSteeringLinkage`）
6. `0x0101`・`0x0102`（制動の減速度・MD のトルクの遅れ）はファームが値を持たない（仮置きは範囲の下限）

**Flash には保存しない（揮発）。** 起動直後は仮置き（調整パラメータは各範囲の下限＝何もしない値、ステアのリンクは 1・0、TC/TV は無効）で、
Pi が全項目を送って `CONFIG_ACK` で確かめるまで ARM を通さない。設定値の唯一の定義を Pi の `config/vehicle.toml`（git 管理）に置くため。
**Flash を使ってよいのはステア原点と IMU キャリブレーションだけ**（STM32 でしか測れない値）。

---

## 11. 時刻同期で STM32 がやること

Pi が4タイムスタンプ方式でオフセットを推定する（`uart_protocol.md` §6）。STM32 は T2・T3 を正確に取って返す。

1. **T2（`t_ping_rx_us`）は UART の IDLE 割り込みで取った時刻を使う**（§6.1）
2. **T3（`t_pong_tx_us`）は1バイト目が線に出る瞬間**: PONG を最優先キューから送り、DMA 起動の直前に埋めて CRC を計算し直す（`PumpTx`）。
   送信キュー投入時に取ると、送信中のフレームの待ちがオフセットのバイアスになる
3. **μs カウンタは 32bit・起動時 0 スタート（約 71.6 分でラップ）。** DWT->CYCCNT は 180MHz で約 24 秒でラップするので、`Micros()`（`lib/timer/timer.c`）は
   呼ばれるたびに経過サイクルを μs へ積み上げて伸ばしている（24 秒以上呼ばれない期間があると飛ぶ）。ラップの扱いは Pi 側が持つ

---

## 12. 再立ち上げ時の確認

ファームを大きく作り直した・基板を替えたときに、段階的に確かめる要点。いきなり全部を繋がない。

1. **CRC**: `crc16_ccitt("123456789") == 0x29B1`
2. **STM32 → Pi**: 起動時の `VERSION` が Pi でデコードできる。続いて Pi から `VERSION_REQ`（LEN=0）を送って応答が返る（LEN=0 の受信経路の確認）
3. **TELEMETRY のレイアウト**: 各フィールドに識別できる値を入れ、`odom_dist`（i32）の上位バイト・末尾の `front_angle` まで Pi で合っている
4. **PING/PONG**: `delay` が数 ms に収まり、LiDAR を動かしても `delay` のばらつきが増えない（T3 の取得タイミングの確認）
5. **安全機構（モータ未接続で）**: ハートビート停止で `estop_active`、`COMMAND` 停止で 100ms 後に `uart_timeout`、自動復帰しない、
   `steer_center_valid` が無いと `MANUAL` に入らない。Pi 側の確認ツールは `raspi/tools/estop_test.py`
6. **LiDAR**: 360点の極座標プロットで部屋の形が見える（回転の向き・鏡像はここで分かる）。`duration_us` が約 8300 μs、`rot_speed_dps` が約 3600
7. **長時間**: モータを回した状態で `STATS` と Pi の統計のエラーが増えない。71.6 分を超えて μs カウンタのラップをまたいでも時刻同期が崩れない

未確定事項（トルク定数 Kt・前輪エンコーダの実分解能など）は `uart_protocol.md` §14。
