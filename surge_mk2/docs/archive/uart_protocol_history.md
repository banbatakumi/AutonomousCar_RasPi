# UART プロトコル変更履歴（アーカイブ）

> **陳腐化している。参照しない。** 現行仕様は `docs/uart_protocol.md` と `raspi/proto/protocol.toml`。
> 旧 `uart_protocol.md` の各版の変更の語り・v0.4 決定事項・変更履歴表を退避したもの。

## 版ごとの変更の語り（v0.14〜v0.21）

**バージョン**: v0.21
**最終更新**: 2026-10-11（v0.21: 車速PIの加速度フィードフォワード（`param_id` `0x00B1`・`0x00B2`）。2026-10-09 v0.20: ローンチコントロール（`flags2` bit3・`flags` bit24・`param_id` `0x00A1`）。v0.19:「止める」の意味を Pi・STM32・MD で揃えた。`flags` bit2 の意味の変更と bit22/23・`md_status` bit6 の追加。2026-10-08 v0.18: 舵角を実際の路面舵角に、v0.17: TV を荷重比例の配分へ）
**対象**: Raspberry Pi 5 ⇄ STM32F446RE 間の通信
**状態**: v0.21 は Pi・STM32 とも実装済み・ビルド済み（`pi_uart_protocol_v0.21_delta.md`）、v0.20 は Pi・STM32 とも実装済み（`pi_uart_protocol_v0.20_delta.md`）、v0.19 は Pi・STM32・MD とも実装済み（`pi_uart_protocol_v0.19_delta.md`）。**STM32・MD は未書き込み、実機での動作検証は未了**

> **v0.21 は車速PIの加速度フィードフォワードの追加。ワイヤ形式（LEN・オフセット）は同じ**（`param_id` を 2 つ足すだけ。
> 2026-10-11、STM32 側 `pi_uart_protocol_v0.21_delta.md`）。
>
> - **`param_id` `0x00B1` `SPEED_FF_NM_PER_M_S2`・`0x00B2` `SPEED_FF_RR_NM` を新設**（調整パラメータ。§5.8.4）: STM32 は
>   車速PIの出力に「`0x00B1` × 目標車速の傾き [m/s²] ＋ `0x00B2` × 進む向き」（後輪 2 輪の合計トルク）を足す。
>   どちらも 0（既定）なら v0.20 までと同じ。値は `[control]` の `speed_ff_nm_per_m_s2`・`speed_ff_rr_nm`
>   （`[dynamics]` の `speed_plant_gain`・`rolling_resistance` から。同定を適用すると `tools/sysid` が書き直す）
> - **`COMMAND.accel_limit` の使い方が増える**（意味は同じ: 目標車速を動かす傾きの上限）。フィードフォワードは
>   「目標車速の傾き」に効くので、slam2d 系の planner は経路の速度プロファイルの加速度をここへ入れる
>   （`raspi/nav/speedctl.py` の `ref_command`）。0 は従来どおり「指定なし＝STM32 の上限 3.0m/s²」
>
> **v0.20 はローンチコントロールの追加。ワイヤ形式（LEN・オフセット）は同じ**（既存の空きビットを使う。2026-10-09、
> STM32 側 `pi_uart_protocol_v0.20_delta.md`）。
>
> - **`COMMAND.flags2` bit3 `launch` を新設**（§5.6.7）: 立っている間、**車速指令の目標が車速より 0.5m/s 以上速い加速（発進）**で、
>   STM32 が目標車速のランプ（3.0m/s²）と車速PIを迂回して全開のトルクを要求し、TC にグリップの限界へ絞らせる。
>   `target_speed` に届いたら車速PIへ戻る。車速PIの発進は偏差が溜まるまでトルクが出ず、グリップの限界に届くまで
>   約0.3秒かかる（トルク指令の全開に対して、最初の1秒に進む距離が約3/4）
> - **`TELEMETRY.flags` bit24 `launch_active` を新設**: ローンチコントロールで発進している最中
> - **`param_id` `0x00A1` `LAUNCH_EXIT_TORQUE_NM` を新設**（調整パラメータ。§5.8.4）: 発進を終えて車速PIへ戻すときの
>   積分の初期値（その車速を保つトルク）。`[control]` の `launch_exit_torque_nm`
> - `protocol_version` を `0x0013`→`0x0014` に上げる。**Pi と STM32 を一緒に入れ替えること**（MD は変更なし）

> **v0.19 で変わったのは「止める」の振る舞いと、`flags`・`md_status` のビット。ワイヤ形式（LEN・オフセット）は同じ。**
> （2026-10-09。3リポジトリ横断の調査 `archive/cross_project_audit_2026-10.md` への対応。STM32 側 `pi_uart_protocol_v0.19_delta.md`）
>
> - **`flags` bit2 `armed` は「駆動電源 ON」から「STM32 が上位の指令で走れる状態」になった。** 駆動電源は
>   新設の bit22 `drive_power_on`。ARM を要求しても MD の起動待ち（約1秒）の間は `armed` = 0
> - **bit23 `md_fault` を新設**: MD（3台のいずれか）が無応答・異常・トルク上限の不一致で使えず、STM32 が
>   両後輪を最大制動して走行を止めている。駆動電源が切れる（ARM を外す）まで立ち続ける。理由は `LOG`
> - **`md_status` bit6 `uncalibrated` を新設**（MD のエンコーダ未校正。MD は動かない）
> - **走行中に ARM が外れても、STM32 は停車するまで駆動電源を切らず最大制動する**（上限 3 秒）。v0.18 までは
>   ARM ビットがそのまま駆動電源で、Pi が通信途絶・デッドマンで DISARM を送ると MD ごと電源が落ちて惰行していた。
>   **Pi（io_node）も、異常系ではまず `arm`+`brake`（最大）を送り、停止を確認してから DISARM に落とす**
> - **ステア中心が未較正の間、STM32 は ARM されても駆動電源を入れない**
> - **ブレーキホールドを追加**（`param_id` `0x0090` `BRAKE_HOLD_ENABLE`、既定は有効。GUI の設定から ON/OFF）:
>   制動して止まったら後輪の位置保持へ自動で移る。brake・自動停止・フェイルセーフのどの制動でも働く
> - **LiDAR スキャンのセクタ時刻（`Scan.sector_t_ns`）は、受信時刻ではなく STM32 の `t_start_us` を時刻同期で
>   換算した値になった**（Pi 側だけの変更。約 9ms 早くなる。`control_latency_s` は取り直すこと）
> - **io_node は `protocol_version` が一致し、`[control]` が STM32 に入るまで ARM を通さない**
>   （`diag/link` の `arm_inhibited`・`arm_inhibit_reason`）
> - `steer_rate_limit` = 0 のときの STM32 の既定は 0.5 rad/s（値はほぼ同じ。専用の定数にしただけ）
> - `protocol_version` を `0x0012`→`0x0013` に上げる。**Pi・STM32・MD を一緒に入れ替えること**

> **v0.18 で変わったのは舵角の意味。ワイヤ形式（LEN・オフセット）は同じ。**
>
> - **舵角（`COMMAND.target_steer`・`TELEMETRY.steer_actual`/`steer_cmd_echo`・`LIMITS.max_steer_rad`）は
>   実際の路面舵角になった。** v0.17 までは「ステアモータの角度×0.5」で、リンクが線形でないため大舵角で
>   実際より大きかった（29.1° と報告しているとき前輪は約 26.5°）。STM32 が
>   `路面舵角 = STEER_LINK_GAIN·x + STEER_LINK_CUBIC·x³`（x = モータ角×0.5）で指令と報告を換算する
> - **`param_id` `0x0081`（`STEER_LINK_GAIN`、既定 1）・`0x0082`（`STEER_LINK_CUBIC`、既定 0）を新設**（§5.8.4）。
>   値は Pi のシステム同定が `[control]` に書き、io_node が接続のたびに送る。STM32 は入れた瞬間に反映し、
>   **`LIMITS` を送り直す**（`max_steer_rad` が変わる。今の同定値では 30° → 26.56°）
> - **STM32 の車速（`TELEMETRY.speed`）も変わる**: 前輪の速度×cos(路面舵角) なので、v0.17 までは舵いっぱいで
>   2〜6% 小さく出ていた（TC のスリップ率・オドメトリ・同定の曲率に波及していた）
> - `protocol_version` を `0x0011`→`0x0012` に上げる。**本書が最終版**。**v0.17 以前のファームと v0.18 の Pi を
>   つなぐと、Pi は舵角を路面舵角のつもりで送るが STM32 は「モータ角×0.5」と解釈する**（大舵角で切れ足りない
>   向きに最大約 2.4° ずれる——26.56° の指令で前輪は約 24.1°。`control_params_status` が `unsupported` になるので、その間は走らせないこと）

> **v0.17 で変わったのは TV（トルクベクタリング）だけ。ワイヤ形式（LEN・オフセット）は同じ。**
>
> - **TV の中身**: ヨーレートの PI（規範ヨーレートとの偏差を左右差で埋める）をやめ、**総駆動トルクを
>   左右の荷重の比で配る**フィードフォワードにした。左右差 = 総駆動トルク × clamp(`TV_LOAD_GAIN_S2_PER_M`
>   × 車速 × ヨーレート, ±`TV_MAX_RATIO`)。理由は §5.8.4
> - **`param_id`**: `0x0021`-`0x0029`（PI の調整パラメータ）を廃止（STM32 は `UNKNOWN_ID` を返す）。
>   `0x002B`（`TV_LOAD_GAIN_S2_PER_M`）・`0x002C`（`TV_MAX_RATIO`）を新設。`0x002A` は残る
> - **`TELEMETRY`**: オフセット 66 の `yaw_rate_target`（0.001 rad/s）を **`tv_ratio`**（0.0001、配分の比率）に
>   置き換えた。`tv_moment_nm` は「TV が要求した左右差のヨーモーメント換算」
> - v0.17 で `protocol_version` を `0x0010`→`0x0011` に上げた。`protocol_version = 0x0011` は
>   その内容を指す。v0.16 のファームと v0.17 の Pi（またはその逆）をつなぐと、`control_params_status` が
>   `unsupported` になり、TV の調整値が入らない（走行はできる）

> **v0.16 で変わったのは `CONFIG_SET`/`CONFIG_GET` の `param_id` の追加（§5.8・§5.8.4）と、`TELEMETRY` の
> LEN 74→**84**・`flags` bit21（§5.3・§5.4）。** STM32 側の制御（TC・ABS・片輪浮き対策・TV）の中身も変わった。
>
> - **`TELEMETRY` に制御の介入量を追加**（`tc_limit_nm` の直後に10バイト）: `torque_req[2]`（絞る前の要求トルク）・
>   `abs_limit_nm`（ABS の上限）・`yaw_rate_target`（TV の規範ヨーレート）・`tv_moment_nm`（TV が要求した
>   ヨーモーメント）。`flags` bit21=`WHEEL_LIFT_ACTIVE`。GUI の診断タブのグラフと mcap（`/vehicle_state`）に出る
>
> - **調整パラメータ**: `0x0011`-`0x0015`（TC・スリップの測り方）、`0x0021`-`0x002A`（TV）、`0x0051`
>   （片輪浮き対策）、`0x0071`-`0x0073`（ABS）。名前・範囲・既定値の唯一の定義は STM32 側
>   `src/control/control_params.h` の表。値の唯一の定義は Pi 側 `config/vehicle.toml` の `[control]` で、
>   io_node が接続のたびに送り、`CONFIG_ACK.applied` と突き合わせ、その後も読み戻し続ける（§5.8.4）
> - **TC・ABS**: 油圧 ABS を写したルールベース（滑り始めのトルクを覚えて0.7倍…）から、スリップ率を
>   目標に保つ連続の PI へ。スリップ率の分母に下限を置き、**0.25m/s 未満でも働く**（発進の空転）。
>   TC は**車速指令の減速にも掛かる**（以前は ABS の対象外で、後輪がロックし得た）
> - **片輪を絞った分を反対の輪へ載せない**（総駆動力が減る）。片輪浮き対策は TC と同じ上限を共有する
>   見張りになった（ON/OFF は従来どおり独立）
> - **TV**: 左右差を保ったまま上限に収める配分（全開でも差が付く）、規範ヨーレートの1次遅れ、同定用の
>   ヨーモーメント注入（`0x002A`）。規範の舵の非線形・横加速度の上限は Pi が `[dynamics]` から送る
> - `TELEMETRY` の `slip[2]` は低速でも 0 にならない（分母の下限つきのスリップ率）。`tc_limit_nm[2]` は
>   片輪浮き対策の絞りも含む
>
> v0.16 で `protocol_version` を `0x000F`→`0x0010` に上げた。

---

## v0.14 の要約（経緯）

**状態（v0.14 時点）**: Pi 側 v0.14 実装済み。STM32 側は後に実装（ファームの `CLAUDE.md`）

> **v0.14 で変わったのは `COMMAND.flags2` に2ビット（bit1/2）、
> `TELEMETRY.flags` に2ビット（bit18/19）を追加したことだけ**（§5.6.6/§5.4）。
> **ワイヤ形式・LEN の変更は無い**（`flags2`/`flags` とも既存の空きビットを使う）ため、
> 片側が未対応でも通信は継続する（ビットを立てなければ v0.13 と同じ挙動）。
>
> - **背景**: 自律走行（AUTO）中に右左折・車線変更の意思表示ができなかった
>   （STM32 側に `Lighting_SetWinker()` 相当の実装はあるが、`ApplyRasCommand()` から
>   一度も呼ばれていない実装漏れ）。
> - **`COMMAND.flags2`**: bit1 = `WINKER_LEFT`、bit2 = `WINKER_RIGHT` を新設。
>   両方立てるとハザード（左右同時点滅）。点滅の実行（周期・位相）は STM32 側の責務、
>   Pi は点灯要求を送るだけ（§5.6.6）。
> - **`TELEMETRY.flags`**: bit18 = `WINKER_LEFT_ACTIVE`、bit19 = `WINKER_RIGHT_ACTIVE`
>   を新設。他の「介入中」ビット（`tc_active` 等）と同じく、要求ではなく**実際に
>   点滅しているか**を返す（§5.4）。
>
> `protocol_version` を `0x000D`→`0x000E` に上げる。
> **本書が最終版**であり、`protocol_version = 0x000E` はこの内容を指す。
> **STM32 側ファームウェアは未実装。** この節は Pi 側からの提案であり、
> STM32 側との合意・実装が済むまでは `flags2` bit1/2 を立てても無反応
> （未知ビットとして無視される想定）。


---

## 0. v0.4 の決定事項

### 0.1 v0.3 から変更しないもの

- SYNC(2B) + TYPE + SEQ + LEN + PAYLOAD + CRC16 の可変長フレーム
- CRC-16/CCITT-FALSE（poly=0x1021, init=0xFFFF, 反転なし, xorout=0x0000）、検査値 `0x29B1`
- 全フィールド リトルエンディアン
- LiDAR を 1周12セクタに分割して送る方式
- `PING` / `PONG` による 4タイムスタンプ方式の時刻同期
- 設定は個別 ID + ack、`applied` に実適用値を返す
- GPIO ハートビート方式の E-Stop
- **TYPE の方向規約: `0x01-0x0F` = STM32 → Pi、`0x10-0x1F` = Pi → STM32**

### 0.2 Pi 側が決めたこと（STM32 側「§8 決めてほしいこと」への回答）

1. **`mode = 3 (CALIB)` は廃止する。**
   ステアリング原点合わせは「タイヤを手で直進にしてボタンを押しながら起動 → Flash 保存」という
   **組み立て・整備時の作業**と位置づける。走行中のコマンドシーケンスは実装しない。
   → Pi 側の状態機械からも CALIB 状態を削除（§13）。`mode` の値 3 は**予約**。

2. **累積オドメトリを追加する。** `odom_dist[2] : i32×2`（前輪 FL / FR、**0.1 mm 単位**）。
   `speed` の時間積分ではパケットロス分の移動距離が永久に失われる。
   累積値なら**何発落ちても次の1発で回復する**。

3. **`accel_z` を追加する。** `i16`、0.001 m/s²。
   段差検出に加え、**静止時に `|a| ≈ 9.8` になるかで IMU の健全性を検証**できる。

### 0.3 STM32 側の再回答で確定した内容（Pi 側提案 A〜D の結末）

| # | Pi 側の当初提案 | 確定内容 |
|---|---|---|
| A | `TELEMETRY` 途絶を 100ms=DEGRADED / 200ms=FAULT の2段階に | **採用。ただし理由は訂正**。「LiDAR バーストで滞留する」は誤りで、待ち時間は 3.04ms で有界（§8）。正しい理由は**単発 CRC エラーを即 FAULT にしないため** |
| B | `VERSION` (0x07) を双方向にし、要求は LEN=0 | **不採用**。要求は **`VERSION_REQ` (0x14)** に分離。同一 TYPE で LEN が変わる問題を自ら踏んでおり、方向規約も崩れる（§5.9） |
| C | `STATS` は「直近1秒の増分」 | **不採用**。**累積 `u32`** とする。増分だと `STATS` が1個落ちた分のエラー件数が永久に失われる。`odom_dist` を累積で送るのとまったく同じ理屈（§5.10） |
| D | `dist` 圧縮は専用 TYPE `0x09` に | **採用**。加えて `param 0x0040` を **bool から enum に変更**し、`0x0041` は廃止（§5.8） |

**B と C はどちらも Pi 側の設計ミスであり、STM32 側の指摘が正しい。**
特に C は、自分で「累積で送ればロスに強い」と書いておきながら `STATS` を増分にしていた。

### 0.4 Pi 側からの訂正と追加指摘

1. **`odom_dist` のラップ値は 429m ではなく 429.5 km。**
   Pi 側資料の記載が 1000倍の誤りだった。`0.1mm × 2^32 = 4.295×10^5 m`。
   ミニカーの走行距離では**ラップに到達しない**（§12.4）。

2. **前輪データの射影は「累積値」ではなく「差分」に対して行う（重要）。**
   STM32 側 §4.3 の式 `d_center ≈ mean(odom_dist) × cos(steer_actual)` は、
   **累積値にそのまま適用すると誤りになる。** 舵角は時々刻々変わるため、
   過去の走行分に現在の `steer_actual` を掛けることはできない。
   **必ず1周期分の差分に対して、その時点の舵角で射影する**（§5.3）。

3. **前輪のスリップ量を見るときも射影が必要。**
   `wheel_speed[FL] − speed` をそのまま取ると、舵角 30° で 15.5% のスリップが
   常時出ているように見える。後輪（`RL`/`RR`）は非操舵輪なので射影不要。

4. **`STATS` に LD06 のパースエラー数を入れたい**（任意）。
   `lidar_ok` フラグだけでは「たまにフレームを落としている」状態が見えない。
   `u32 lidar_frame_error` を1つ足して LEN = 52 にすることを提案する。
   **必須ではないので、STM32 側の実装負荷が気になるなら見送ってよい。**

---

---

## 15. 変更履歴

| バージョン | 日付 | 内容 |
|---|---|---|
| v0.1 | 2026-08-06 | 初版ドラフト。TYPE 0x01-0x05 / 0x10-0x12 を定義 |
| v0.2 | 2026-08-06 | 車両構成の確定を反映。`motor_current[2]→[3]`、`temp[4]` の対象を確定、`odom_ticks` を左右2本化。`TELEMETRY` LEN 47→53 |
| v0.3 | 2026-08-06 | 時刻同期を `PING`/`PONG` の4タイムスタンプ方式に分離（`PONG` 0x06 を追加）。`COMMAND` から `t_pi_us` を削除し LEN 14→10。`flags` に `calib_running`/`calib_done` を追加 |
| **v0.4** | 2026-08-06 | **STM32 側と2往復の議論を経て確定。** `TELEMETRY` を全面改訂（LEN 53→**66**）: 電源2系統化、`odom_ticks` → `wheel_speed[4]` + `odom_dist[2]`、`torque_cmd` / `accel_z` / `md_status[3]` を追加、電圧・電流・超音波を1バイト化、`flags` を u32 に拡張し `mode` を bit0-1 へ移動。`LIDAR_SECTOR` の `angle_start`/`angle_step` を `duration_us`/`rot_speed_dps` に置換、ビニングを min → 代表点に変更。**`VERSION`(0x07) / `STATS`(0x08) / `LIDAR_SECTOR_C`(0x09) / `CONFIG_GET`(0x13) / `VERSION_REQ`(0x14) を追加**、`LOG` に `severity` を追加。`param 0x0040` を LiDAR 出力フォーマットの enum に変更（`0x0041` 廃止）。`COMMAND` タイムアウト 200→100ms、`TELEMETRY` 途絶を 100/200ms の2段階に。**CALIB モード廃止**（`mode=3` は予約、`calib_done` → `steer_center_valid` に改名）。`CONFIG_SET` は揮発（Flash 保存なし）。前輪オドメトリの射影（§5.3）を明記 |
| **v0.5** | 2026-08-09 | **STM32 側発。`COMMAND` (0x10) のみの変更。** LEN 10→**12**、末尾に `brake_torque : u16`（0.0001 N·m/輪、`0` = 未指定 = 最大制動）を追加。`flags` bit3 の1ビット `light` を bit3-4 の2ビット `light_mode`（OFF/DAYTIME/NORMAL）へ拡張し、**`light=1`（旧 NORMAL）と `light_mode=1`（DAYTIME）で値の意味が変わった**。bit5 `passing` を新設。`horn` を単発ビープから「立てている間ずっと鳴る」に変更。`brake` 中は車速 PI を迂回して指定トルクを直接掛ける（`target_speed` は無視）。緊急停止・`COMMAND` 途絶時の制動を最大トルクに変更。起動時に駆動電源が一瞬 ON になる問題を修正（`armed` の観測が変わる）。`torque_cmd` の符号を「正=駆動 / 負=制動」と明確化（レイアウト変更なし） |
| v0.4 | 2026-08-07 | **記述誤りの訂正のみ（ワイヤ形式・`protocol_version` とも変更なし）。** `LIDAR_SECTOR` / `LIDAR_SECTOR_I` / `LIDAR_SECTOR_C` の `rot_speed_dps` のスケールを `0.01 deg/s` → **`1 deg/s`**（§5.1 参照）。u16×0.01 では実回転 3600 deg/s を表現できず、実機ログで生値がそのまま deg/s であることを確認した。**STM32 側の変更は不要** |
| **v0.6** | 2026-08-11 | **`COMMAND` (0x10) のみの変更。** LEN 12→**14**、末尾に `target_torque : i16`（0.0001 N·m、負=後退方向）を追加。`flags` bit6 に `torque_mode` を新設。**ラジコン（MANUAL）モードで駆動トルクを直接指令できるようにするため。** 立っている間は `brake` と同様に車速 PI を迂回し `target_torque` を各輪へ直接掛ける（`target_speed` は無視、`brake` が同時に立っていれば `brake` が優先）。上限は一旦 **0.1 N·m**（モータ物理上限 0.1557 N·m 未満）とし Pi 側でクランプ。`target_torque` の `0` に `brake_torque` のような特別な意味はない。TC/TV との関係は未確定（§14 #8）。**同日、実車確認後に上限を `target_torque`/`brake_torque` とも 0.125 N·m へ引き上げ**（ワイヤ形式は変更なし、Pi/GUI/STM32 のクランプ値のみの変更） |
| **v0.7** | 2026-08-11 | **STM32 側発。ビットの追加のみで LEN は全パケット変更なし**（`flags` の空きビットを使う）。`COMMAND` (0x10) の `flags` bit7 に **`auto_stop`**、`TELEMETRY` (0x02) の `flags` bit16 に **`auto_stop_active`** を新設。`auto_stop` が立っている間、**進行方向**（`torque_mode` なら `target_torque`、そうでなければ `target_speed` の符号で決まる）の超音波が **20cm 未満**になると STM32 が指令を無視して**最大制動**する（§5.6.4）。逆方向のセンサは見ないので後退で抜けられる。検知不能時は作動せず、ラッチもしない（ヒステリシス無し＝境界でチャタリングし得る）。`brake` が同時に立っていれば `brake` が優先。**v0.6 とワイヤ非互換ではない**ため、片側が未対応でも通信は継続する |
| **v0.8** | 2026-08-19 | **STM32 側発。ワイヤ形式・LEN の変更なし。** `CONFIG_SET`/`CONFIG_GET` の `param_id = 0x0010`（TC 有効）/ `0x0020`（TV 有効）が STM32 ファームウェアで実際に機能するようになった（以前は `RAS_CONFIG_UNKNOWN_PARAM` を返していた）。Pi 側は `io_node` にハンドシェイク直後の `CONFIG_GET` 初期同期と、GUI トグル操作に応じた `CONFIG_SET` 送信を実装（§5.8）。`MAX_SPEED`/`MAX_ACCEL`/`MAX_STEER`（`0x0001`-`0x0003`）は STM32 側で既にクランプに使われているが、Flash 非永続化かつ動的変更のユースケースが無いため Pi からは意図的に未送信のまま（§5.8.1） |
| **v0.9** | 2026-08-20 | **STM32 側発。ワイヤ形式・LEN の変更なし。** 後輪片浮き（接地荷重ゼロ）でモータが無負荷空転する問題への対策として「片輪浮き対策（Wheel Lift Guard）」を追加、`CONFIG_SET`/`CONFIG_GET` の `param_id = 0x0050` で ON/OFF できるようになった（§5.8.2）。既存の TC 本体（`0x0010`）とは独立に動作し、個別に有効/無効を切り替えられる。Pi 側は `io_node` に3つ目の `CONFIG_GET` 初期同期と、GUI トグル（`SettingsPanel`）操作に応じた `CONFIG_SET` 送信を実装。**STM32 側も実機で動作確認済み（2026-08-20）**（`pi_uart_protocol_v0.9_delta.md`） |
| **v0.12** | 2026-08-25（`param_id = 0x0060` の値を3段階enumからcm直接指定へ改訂） | **STM32 側発。`CONFIG_SET`/`CONFIG_GET` の `param_id` が増えたのと `auto_stop` の下位判定ロジックの変更のみ（`COMMAND`/`TELEMETRY` を含む既存パケットの構造・LEN は変更なし）。** 固定20cmの自動停止判定を、走行速度に応じて伸びる動的停止距離 `d_stop = v・t_delay + v²/(2・a_max) + margin` と、LiDAR（進行方向±20°の角度窓・直近300ms・3点以上）を主センサ、超音波をフォールバック（LiDAR死角時のみ）と近接フェイルセーフ（5cm未満で常時作動）に使う判定へ置き換えた（§5.6.4）。Pi から変更できるのは安全マージン `margin`（`param_id = 0x0060`, `auto_stop_margin_cm`: cm単位の連続値、範囲0-100、既定15）のみ（§5.8.3）。**当初は `AutoStopLevel`（NEAR=5cm/STANDARD=15cm/FAR=30cmの3段階enum）として実装されていたが、実機投入前に STM32 側がcm直接指定へ変更**（本表はこの改訂を反映）。`t_delay`/`a_max` は STM32 側の固定定数。`protocol_version` を `0x000B`→`0x000C` に上げる。Pi 側は `io_node` に4つ目の `CONFIG_GET` 初期同期と、GUI 操作（`SettingsPanel` のスライダー）に応じた `CONFIG_SET` 送信を実装。**STM32 側実装済み、実機での動作検証は未了**（`pi_uart_protocol_v0.12_delta.md`） |
| **v0.13** | 2026-08-28 | **STM32 側発。`TELEMETRY`(0x02) の LEN・末尾に追加した4バイトと、`COMMAND`(0x10) の LEN・新設 `flags2` バイトのみ（他パケットに変更なし）。** TC（トラクションコントロール）のゲイン（`DRIVE_TC_CUT_GAIN`/`DRIVE_TC_RECOVER_RATE`）を実機で追い込むにあたり、`flags` bit5（`tc_active`、介入中か否かの1bit）だけでは内部状態が分からなかった問題への対応。`TELEMETRY` に `slip[2]`（後輪スリップ率、無次元）・`tc_limit_nm[2]`（TCが動的に決めるトルク上限）を追加（LEN 66→**74**、§5.3）。加えて、後輪モータを位置制御へ切り替えて機械的に固定する**サイドブレーキ**を新設。`COMMAND` に `flags2`（bit0=`SIDE_BRAKE`）を追加（LEN 14→**15**、§5.6.5）。速度に関わらず即座に切り替わり、既存の `flags` bit1（`BRAKE`）より優先される。実際に固定できたかは `TELEMETRY.flags` bit17（`SIDE_BRAKE_ACTIVE`、§5.4）で確認する。`protocol_version` を `0x000C`→`0x000D` に上げる。Pi 側は `protocol.toml`/`raspi/msgs`/`sim/stm32.py` を対応済み。**GUI 側にサイドブレーキを送出する操作は未実装**（`DriveCmd.side_brake` はバス上に存在するが、ドライバー向け操作パネルへの結線はまだ行っていない）。**STM32 側実装済み、実機での動作検証は未了**（`pi_uart_protocol_v0.13_delta.md`） |
| **v0.11** | 2026-08-22 | **STM32 側発。`LIMITS`(0x0A)/`LIMITS_REQ`(0x15) を新設（`COMMAND`/`TELEMETRY`/`CONFIG_SET`/`CONFIG_GET` を含む既存パケットの構造・LEN は変更なし）。** v0.10 で `MAX_SPEED`/`MAX_ACCEL`/`MAX_STEER` を廃止した結果 Pi が STM32 側の固定上限値を知る手段が無くなっていた問題への対応。`LIMITS` は `max_speed_m_s`/`max_accel_m_s2`/`max_torque_nm`/`max_steer_rad`（f32×4・LEN16・読み取り専用）を返す。STM32 は起動直後に `VERSION` と同じタイミングで3回自発送信、Pi は `LIMITS_REQ` でいつでも再取得できる（§5.12）。`protocol_version` を `0x000A`→`0x000B` に上げる。Pi 側は `io_node.handshake()` で `VERSION_REQ`/`LIMITS_REQ` を併せて送り、未受信なら1秒おきに再送する。`IoNode._send_command` は `LIMITS` を受信済みなら**無条件にそちらでクランプ**し（RC/AUTO 問わず）、`--max-speed`/`--max-steer`（Pi 側設定）は未受信時のみのフォールバックにした（`raspi/msgs/convert.py` の `command_from_cmd` に `max_accel`/`max_torque` 引数を追加。2026-08-22、当初は「小さい方」だったが GUI 表示との食い違いを避けるため「STM32 実測を優先」に変更）。GUI（`gui/src/store/ui.ts` の `effectiveRange()`）も同じ方針。`sim/stm32.py` にも `LIMITS`/`LIMITS_REQ` を実装済み。**STM32 側実装済み、実機での動作検証は未了**（`pi_uart_protocol_v0.11_delta.md`） |
| **v0.10** | 2026-08-21 | **STM32 側発。`CONFIG_SET`/`CONFIG_GET` の `param_id` のみの変更（`COMMAND`/`TELEMETRY` を含むパケット構造・LEN は変更なし）。** `RAS_PARAM_MAX_SPEED`(`0x0001`)/`RAS_PARAM_MAX_ACCEL`(`0x0002`)/`RAS_PARAM_MAX_STEER`(`0x0003`) を廃止し、STM32 側の固定定数（`DRIVE_MAX_SPEED_M_S` = 5.0 m/s、`DRIVE_MAX_ACCEL_M_S2` = 3.0 m/s²、路面舵角 ±30°）に一本化。この3つの `param_id` は以後常に `RAS_CONFIG_UNKNOWN_PARAM` を返す（§5.8.1）。`protocol_version` を `0x0009`→`0x000A` に上げる。Pi はこの3つの `param_id` を元々送信していなかったため、`protocol.toml`/`packets.py`/`surge_proto.h` の `protocol_version` 更新のみで Pi 側の実質的な挙動変化はない。**STM32 側実装・単体テスト済み、実機での動作検証は未了**（`pi_uart_protocol_v0.10_delta.md`） |
| **v0.14** | 2026-08-30 | **Pi 側発の提案。`COMMAND.flags2` bit1/2（`WINKER_LEFT`/`WINKER_RIGHT`）、`TELEMETRY.flags` bit18/19（`WINKER_LEFT_ACTIVE`/`WINKER_RIGHT_ACTIVE`）を新設（ワイヤ形式・LEN の変更は無し。§5.6.6/§5.4）。** STM32 側に `Lighting_SetWinker()` 相当の実装があるのに `ApplyRasCommand()` から呼ばれておらず、自律走行中に右左折の意思表示ができない実装漏れへの対応。両ビットを両方立てるとハザード（左右同時点滅）。点滅の周期・位相は STM32 側の責務、Pi は点灯要求を送るだけ。`side_brake` と同じくトグルで送り、`brake`/`side_brake` と違って灯火系なので未 ARM でも効く。`protocol_version` を `0x000D`→`0x000E` に上げる。Pi 側は `protocol.toml`/`raspi/msgs`/`sim/stm32.py`/GUI（`DriveControls.tsx` に操作パネルを追加、キー・パッド割り当ては無し）を実装済み。**STM32 側は未実装**（§14 #11） |
| **v0.15** | 2026-09-27 | **STM32 側発。`CONFIG_SET`/`CONFIG_GET` の `param_id = 0x0070`（`ABS_ENABLE`）と `TELEMETRY.flags` bit20（`ABS_ACTIVE`）を新設（ワイヤ形式・LEN の変更なし）。** 制動時の後輪ロックを防ぐ ABS（スリップ率 -0.2 で制動トルクを削る select-low、0.25m/s 未満は無効、要求の20%以下に0.2s張り付いたらブレーキ解除までフォールバック）。`protocol_version` を `0x000E`→`0x000F` に上げる。Pi 側は `protocol.toml`・`VehicleState.abs_active`・`LinkDiag.abs_enabled`・`io_node` の起動時 `CONFIG_GET` と GUI トグル・介入ランプ・`sim/stm32.py`（制動がグリップで頭打ちのとき旗だけ立てる）・同定の解析（ABS の介入を制動の頭打ちの証拠に使う）を対応。**実機での動作検証は未了**（`pi_uart_protocol_v0.15_delta.md`） |
| **v0.16** | 2026-10-05 | **`CONFIG_SET`/`CONFIG_GET` の `param_id` を追加、`TELEMETRY` に制御の介入量を追加（LEN 74→**84**: `torque_req[2]`・`abs_limit_nm`・`yaw_rate_target`・`tv_moment_nm`、`flags` bit21=`WHEEL_LIFT_ACTIVE`。§5.3）。** 足回りの制御（TC・ABS・片輪浮き対策・TV）の調整パラメータ `0x0011`-`0x0015`・`0x0021`-`0x002A`・`0x0051`・`0x0071`-`0x0073`（§5.8.4。定義は STM32 側 `control_params.h` の表、値は Pi 側 `vehicle.toml` の `[control]`）。STM32 の制御は TC・ABS をスリップ率の連続 PI へ（低速でも働く、TC は車速指令の減速にも掛かる、片輪を絞った分を反対輪へ載せない）、TV は左右差を保つ配分・規範の1次遅れ・同定用のヨーモーメント注入。`protocol_version` を `0x000F`→`0x0010` に上げる。Pi 側は `protocol.toml`・`raspi/core/control_params.py`（送信・照合・読み戻し・一時上書き）・`LinkDiag.control_params_*`・`DriveCmd.fw_overrides`・GUI・`tools/ctrl_tune`（同定と最適化）を対応。**STM32 は未書き込み、実機での動作検証は未了**（`pi_uart_protocol_v0.16_delta.md`） |
| **v0.17** | 2026-10-08 | **TV をヨーレートの PI から、後輪左右の荷重に比例した配分（フィードフォワード）へ置き換えた（ワイヤ形式・LEN の変更なし）。** `param_id` `0x0021`-`0x0029` を廃止、`0x002B`（`TV_LOAD_GAIN_S2_PER_M`）・`0x002C`（`TV_MAX_RATIO`）を新設。`TELEMETRY` のオフセット 66 を `yaw_rate_target` → `tv_ratio`（0.0001）に置き換え。`protocol_version` `0x0010`→`0x0011`（`pi_uart_protocol_v0.17_delta.md`）。**STM32 は未書き込み、実機での動作検証は未了** |
| **v0.18** | 2026-10-08 | **舵角を実際の路面舵角にした（ワイヤ形式・LEN の変更なし）。** STM32 がステアのリンクの換算（`路面舵角 = gain·x + cubic·x³`、x = モータ角×0.5）で指令・報告・`LIMITS.max_steer_rad`・車速の射影を行う。`param_id` `0x0081`（`STEER_LINK_GAIN`）・`0x0082`（`STEER_LINK_CUBIC`）を新設、設定時に `LIMITS` を送り直す。`protocol_version` `0x0011`→`0x0012`（`pi_uart_protocol_v0.18_delta.md`）。**STM32 は未書き込み、実機での動作検証は未了** |
| **v0.19** | 2026-10-09 | **「止める」の意味を Pi・STM32・MD で揃えた（ワイヤ形式・LEN の変更なし）。** `TELEMETRY.flags` bit2 `armed` を「駆動電源 ON」から「上位の指令で走れる状態」へ変更し、bit22 `drive_power_on`・bit23 `md_fault` を新設。`md_status` bit6 `uncalibrated` を新設。STM32: 惰行は送信停止ではなくトルク 0 を送る／走行中に ARM が外れても停車まで駆動電源を切らず最大制動（上限3秒）／3台の MD が揃うまで `armed` にせず、走行中に崩れたら両後輪を最大制動してラッチ／ステア未較正の間は駆動電源を入れない／フェイルセーフは停止後に後輪の位置保持へ移る／`steer_rate_limit`=0 の既定を 0.5 rad/s の定数に／ブレーキホールド（`param_id` `0x0090`、既定は有効）を新設。Pi: LiDAR セクタの時刻を STM32 の `t_start_us` から換算／異常系（`cmd` 途絶・`TELEMETRY` 途絶・デッドマン）は制動してから DISARM／`protocol_version` の一致と `[control]` の同期が済むまで ARM を通さない／`auto/cmd` 途絶時の制動を最大にし舵を保持。`protocol_version` を `0x0012`→`0x0013` に上げる。**STM32・MD は未書き込み、実機での動作検証は未了**（`pi_uart_protocol_v0.19_delta.md`、`archive/cross_project_audit_2026-10.md`） |
| **v0.21** | 2026-10-11 | **車速PIの加速度フィードフォワードを追加（ワイヤ形式・LEN の変更なし）。** `param_id` `0x00B1` `SPEED_FF_NM_PER_M_S2`・`0x00B2` `SPEED_FF_RR_NM`（車速PIの出力に「目標車速の傾き×利得＋転がり抵抗ぶん」を足す。0 で無し＝従来と同じ。§5.8.4）。`protocol_version` `0x0014`→`0x0015` |
| **v0.20** | 2026-10-09 | **ローンチコントロールを追加（ワイヤ形式・LEN の変更なし）。** `COMMAND.flags2` bit3 `launch`（車速指令の、目標が車速より大きく離れた加速＝発進を、目標車速のランプと車速PIを迂回して全開＋TC で行う。§5.6.7）、`TELEMETRY.flags` bit24 `launch_active`、`param_id` `0x00A1` `LAUNCH_EXIT_TORQUE_NM`（PIへ戻すときの積分の初期値）を新設。`protocol_version` `0x0014`。STM32 側 `pi_uart_protocol_v0.20_delta.md` |

### v0.4 内での差分（初回ドラフト → 確定版）

Pi 側が先に書いた v0.4 ドラフトから、STM32 側の指摘で以下が変わった。
**実装は本書（確定版）に従うこと。**

| 項目 | ドラフト | 確定版 |
|---|---|---|
| `TELEMETRY` のフィールド順 | accel/pitch/roll → odom_dist | **odom_dist(24) → accel/pitch/roll(32-41)** |
| `STATS` | 増分 u16、LEN 22 | **累積 u32、LEN 48** |
| `VERSION` 要求 | `0x07` を双方向（LEN 0/10） | **`VERSION_REQ` (0x14) に分離** |
| `0x14` の割り当て | `CONFIG_SAVE` | **`VERSION_REQ`**（`CONFIG_SAVE` は不採用） |
| LiDAR フォーマット切替 | `0x0040`/`0x0041` の bool 2本 | **`0x0040` の enum 1本**（`0x0041` 廃止） |
| `flags` bit9/10 | `calib_running` / `calib_done` | **予約 / `steer_center_valid`** |
| `odom_dist` ラップ | 429 m（**誤記**） | **429.5 km** |
| 2段階タイムアウトの根拠 | LiDAR バーストによる滞留（**誤り**） | **単発 CRC エラーの吸収** |


---

## v0.22（2026-10-11）

目標車速のランプの固定上限（`DRIVE_MAX_ACCEL_M_S2` 3.0）を廃止し、`COMMAND.accel_limit` をそのまま使う
（0 のときの既定は 3.0m/s²）。`LIMITS.max_accel_m_s2` は 65.535（上限なし）。自動停止の減速度は
`VEHICLE_AUTO_STOP_DECEL_M_S2` に分離。ワイヤ形式・LEN の変更なし。`protocol_version` `0x0015`→`0x0016`。
