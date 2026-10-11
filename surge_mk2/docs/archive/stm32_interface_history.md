# STM32 側実装仕様書 変更履歴（アーカイブ）

> **陳腐化している。参照しない。** 現行仕様は `docs/stm32_interface.md` と `docs/uart_protocol.md`、定義は `raspi/proto/protocol.toml`。
> 旧 `stm32_interface.md` の各版の変更の語りと変更履歴表を退避したもの。

## 版ごとの変更の語り（v0.3〜v0.22）

> **v0.22 での変更（★実装済み・ビルド済み。未書き込み、実機での動作検証は未了。2026-10-11。`pi_uart_protocol_v0.22_delta.md`）**
>
> - **目標車速のランプの固定上限を廃止**（`drive.h` の `DRIVE_MAX_ACCEL_M_S2` 3.0 を削除）。`COMMAND.accel_limit` が
>   正ならその値でランプする（下限 0.01）。0 なら既定の 3.0m/s²（`DRIVE_DEFAULT_ACCEL_M_S2`）。ランプそのものは
>   残る（加速度フィードフォワードがその傾きを使う）。上限は上位が `vehicle.toml` の `[safety] cmd_accel_limit_m_s2`
>   で決めて毎回送る（同じ動きになるよう 6.0→3.0 にしてある）
> - `LIMITS.max_accel_m_s2` はワイヤで表せる最大 65.535 を返す（「上限なし」）
> - 自動停止の停止距離の減速度は `VEHICLE_AUTO_STOP_DECEL_M_S2`（3.0、値は同じ）として独立させた
> - `protocol_version` を `0x0015`→**`0x0016`** に上げる。ワイヤ形式・LEN の変更なし
>
> **v0.21 での変更（★実装済み・ビルド済み。未書き込み、実機での動作検証は未了。2026-10-11。`pi_uart_protocol_v0.21_delta.md`）**
>
> - 車速PIの加速度フィードフォワード（`src/control/drive.c` の `SpeedFeedforward()`、`drive.h` の同名の節）:
>   車速PIの出力に `params.speed_ff_nm_per_m_s2` × 目標車速の傾き ＋ `params.speed_ff_rr_nm` × 進む向き を足す
>   （後輪 2 輪の合計トルク）。傾きは「その周期に目標車速を動かした量 ÷ dt」を `DRIVE_SPEED_FF_TAU_S`（20ms）で
>   ならしたもの。足した後の合計を ±最大トルクに収める。制動・トルク指令・ローンチ・停車の惰行では足さない
> - `param_id = 0x00B1`（`speed_ff_nm_per_m_s2`、0〜0.2）・`0x00B2`（`speed_ff_rr_nm`、0〜0.05）。既定はどちらも 0
>   （＝v0.20 までと同じ動き）。上位が `[control]` の値を接続のたびに送る
> - `protocol_version` を `0x0014`→**`0x0015`** に上げる
>
> **v0.20 での変更（★実装済み・ビルド済み。未書き込み、実機での動作検証は未了。2026-10-09。`pi_uart_protocol_v0.20_delta.md`）**
> - ローンチコントロール（`src/control/drive.c` の `UpdateLaunch`、条件と定数は `drive.h` の同名の節）:
>   `COMMAND.flags2` bit3 `LAUNCH` が立っている間、車速指令の目標車速が車速より 0.5m/s 以上速い加速（発進。
>   車速は問わない）で、目標車速のランプと車速PIを迂回して全開のトルクを要求し、TC に絞らせる。目標の 0.05m/s 手前で
>   PIへ戻す（目標車速は発進中ずっと実車速に合わせてあり、積分は `launch_exit_torque_nm` から始める）。
>   1回の要求で1回、TC が無効なら始めない、3秒で打ち切る
> - `flags` bit24 `LAUNCH_ACTIVE` = `Drive_IsLaunchActive()`、`param_id = 0x00A1`（`launch_exit_torque_nm`、
>   `control_params.h` の表。0〜0.05、既定 0）
> - `protocol_version` を `0x0013`→**`0x0014`** に上げる

> **v0.19 での変更（★実装済み・ビルド済み。未書き込み、実機での動作検証は未了。2026-10-09。`pi_uart_protocol_v0.19_delta.md`）**
> - `flags` bit2 `ARMED` = `Vehicle_IsArmed()`（以前は `Power_IsDriveOn()`）。bit22 `DRIVE_POWER_ON`・bit23 `MD_FAULT` を新設。
>   `md_status` bit6 `UNCALIBRATED` を新設
> - `Coast()`（`src/control/drive.c`）は送信を止めず、トルク 0 を送り続ける（MD は無通信 0.5 秒まで直前のトルクを出す）
> - 走行中に ARM が外れたら、停車するまで駆動電源を切らず最大制動する（`UpdateDisarmBraking`、上限 3 秒）
> - 3台の MD の健全性を毎周期見る（`UpdateMdHealth`・`Motors_NotReadyReason`）。揃うまで `armed` にせず、走行中に
>   崩れたら両後輪を最大制動して駆動電源が切れるまでラッチする
> - ステア中心が未較正の間は駆動電源を入れず、ステアへ何も指令しない
> - ブレーキホールド: 制動（brake・自動停止・強制制動・フェイルセーフ）して止まったら後輪の位置保持へ移る。
>   `param_id = 0x0090`（`BRAKE_HOLD_ENABLE`、0/1、既定は有効）で ON/OFF
> - IMU の I2C 復旧をブロッキングしないステートマシンに、ジャイロのレンジを ±500 deg/s に
> - `protocol_version` を `0x0012`→**`0x0013`** に上げる

> **v0.18 での変更（★実装済み・ビルド済み。未書き込み、実機での動作検証は未了。2026-10-08。`pi_uart_protocol_v0.18_delta.md`）**
> - ステアのリンクの換算（`src/control/steering.c`）: `路面舵角 = steer_link_gain·x + steer_link_cubic·x³`、
>   x = モータ角×`STEERING_LINKAGE_RATIO`(0.5)。`Steering_SetRoadWheelAngleRad()`（逆変換はニュートン法）・
>   `Steering_GetRoadWheelAngleRad()`・`Steering_GetMaxRoadWheelAngleRad()` のすべてに効く。既定は 1 と 0
> - `param_id` `0x0081`・`0x0082` を `control_params.h` の表に追加。`CONFIG_SET` で入れた瞬間に
>   `Steering_SetLinkage()` へ渡し、`LIMITS` を送り直す（`src/comm/ras_link.c` の `ApplyConfig`）
> - 舵角を使う箇所（車速の射影 → TC のスリップ・TV の横加速度、舵角のレート制限）は路面舵角のまま変更なし
> - `protocol_version` を `0x0011`→**`0x0012`** に上げる

> **v0.17 での変更（★実装済み・ビルド済み。未書き込み、実機での動作検証は未了。2026-10-08。`pi_uart_protocol_v0.17_delta.md`）**
> - TV（`src/control/torque_vectoring.c`）: ヨーレートの PI をやめ、総駆動トルクを左右の荷重の比で配る。
>   `左右差 = 総駆動トルク × clamp(tv_load_gain_s2_per_m × 車速 × 実測ヨーレート, ±tv_max_ratio)`。
>   各輪は個別に ±最大トルクへ収める（外輪が上限でも内輪へ載せない）。積分・規範モデルは無い
> - `param_id` `0x0021`-`0x0029` を廃止（`UNKNOWN_ID`）、`0x002B`・`0x002C` を新設。`0x002A` は残る
> - `TELEMETRY` のオフセット 66 を `yaw_rate_target` → `tv_ratio`（i16、0.0001）に置き換え（LEN 84 のまま）
> - `protocol_version` を `0x0010`→**`0x0011`** に上げる

> **v0.16 での変更（★実装済み・ビルド済み。未書き込み、実機での動作検証は未了。2026-10-05。`pi_uart_protocol_v0.16_delta.md`）**
> - 足回りの制御の調整パラメータを `CONFIG_SET`/`CONFIG_GET` で読み書きする（`param_id` `0x0011`-`0x0015`・
>   `0x0021`-`0x002A`・`0x0051`・`0x0071`-`0x0073`）。**表（名前・ID・既定値・範囲）は
>   `src/control/control_params.h` が唯一の定義**。範囲外は丸めて `result = 2`、Flash には保存しない
> - TC・ABS: スリップ率を目標に保つ連続の PI（`SlipLimiter`、`src/control/drive.c`）。分母に下限を置いた
>   スリップ率で低速でも働く。TC は車速PIの減速にも掛かる。片輪浮き対策は TC と同じ上限を共有する
> - 片輪を絞った分を反対の輪へ載せない。TV は左右差を保つ配分・規範の1次遅れ・同定用のヨーモーメント注入
> - `host/`（`host_sim.c`＋`shim/`）で `src/control` をホストでコンパイルできる（Pi 側 `tools/ctrl_tune` が
>   車両モデルと閉ループにして比較・最適化に使う）
> - `TELEMETRY` (0x02) の `tc_limit_nm[2]` の直後に `torque_req[2]`・`abs_limit_nm`・`yaw_rate_target`・
>   `tv_moment_nm`（いずれも i16）を追加。LEN 74→**84**。`flags` に bit21=`WHEEL_LIFT_ACTIVE`
> - `protocol_version` を `0x000F`→**`0x0010`** に上げる

> **v0.15 での変更（★STM32 側発・実装済み。実機での動作検証は未了。2026-09-27。`pi_uart_protocol_v0.15_delta.md`）**
> - 制動時の後輪ロックを防ぐ ABS（`ApplyAbs()`、`src/control/drive.c`）。制動モード（brake・自動停止・
>   フェイルセーフの最大制動）だけに効き、車速PIの減速・`torque_mode` の負トルクには掛からない
> - `CONFIG_SET`/`CONFIG_GET` の `param_id = 0x0070`（`ABS_ENABLE`、0/1、既定は有効）
> - `TELEMETRY` (0x02) の `flags` に bit20=`ABS_ACTIVE`（削っている最中。フォールバック中は偽）
> - **ワイヤ形式・LEN の変更は無い**。`protocol_version` を `0x000E`→**`0x000F`** に上げる

> **v0.14 での変更（★Pi 側発の提案。2026-08-30。STM32 側は後に実装済み）**
> - **背景**: 自律走行（AUTO）中に右左折・車線変更の意思表示ができない。
>   `Lighting_SetWinker()` 相当の機能は既に `src/lighting/` にあるが、
>   `ApplyRasCommand()` から一度も呼ばれていない実装漏れ
> - `COMMAND` (0x10) の `flags2` に bit1=`WINKER_LEFT`・bit2=`WINKER_RIGHT` を追加。
>   **ワイヤ形式・LEN の変更は無い**（既存の空きビットを使うだけ）
> - `TELEMETRY` (0x02) の `flags` に bit18=`WINKER_LEFT_ACTIVE`・
>   bit19=`WINKER_RIGHT_ACTIVE` を追加。こちらも**ワイヤ形式・LEN の変更は無い**
> - **両方のビットを立てるとハザード**（左右同時点滅）として扱うこと。専用の
>   ハザードビットは設けていない
> - 点滅の周期・位相・既存の灯火系（`light_mode`/`passing`/ブレーキランプ）との
>   調停は STM32 側に一任する（`indicator.c` 側に既存パターンがあるはず）
> - `protocol_version` を `0x000D`→**`0x000E`** に上げる
> - **STM32 側は未実装。** `ApplyRasCommand()` に `flags2` bit1/2 の解釈を追加し、
>   `Lighting_SetWinker()` を呼ぶ配線と、`TELEMETRY.flags` bit18/19 の返却、
>   `COMMAND` 途絶時の強制解除（`horn`/`passing` と同様）が必要

> **v0.13 での変更（★STM32 側発・実装済み。実機での動作検証は未了。2026-08-28。`pi_uart_protocol_v0.13_delta.md`）**
> - `TELEMETRY` (0x02) の末尾（`torque_cmd[2]` の直後）に `slip[2]`・`tc_limit_nm[2]`
>   （いずれも i16×0.0001）を追加。LEN 66→**74**。TC のゲイン（`DRIVE_TC_CUT_GAIN`/
>   `DRIVE_TC_RECOVER_RATE`）を実機で追い込むための可視化で、`flags` bit5
>   （`tc_active`、介入中か否かの1bit）だけでは内部状態が分からなかった問題への対応
> - `COMMAND` (0x10) の末尾に `flags2`（u8、bit0=`SIDE_BRAKE`）を追加。LEN 14→**15**。
>   立っている間、速度に関わらず即座に後輪を機械的な位置制御へ切り替えて固定する
>   （サイドブレーキ）。既存の `flags` bit1（`BRAKE`）より優先。**Pi 側が明示的に
>   下ろさない限り有効なまま**
> - `TELEMETRY.flags` に bit17（`SIDE_BRAKE_ACTIVE`）を追加。サイドブレーキが実際に
>   位置制御へ切り替わり固定中かを返す（MDデータ無効等でフォールバック中は立たない）
> - **固定長パケットの LEN が変わるため、旧14/66バイトのままでは LEN エラーで破棄される。**
>   `TELEMETRY`/`COMMAND` とも必ず新しい LEN に追従すること
> - `protocol_version` を `0x000C`→**`0x000D`** に上げる
> - **実機での動作検証は未実施。** `DRIVE_TC_CUT_GAIN`/`DRIVE_TC_RECOVER_RATE` は
>   実測前の暫定値であり、このテレメトリを使って発振（`tc_limit_nm` の周期的な増減）が
>   無いか、介入が速すぎ/遅すぎないかを確認しながら追い込む想定。サイドブレーキも
>   実機での動作検証は未了——特に「速度に関わらず即座に位置制御へ切り替わる」仕様の
>   ため、走行中に誤って `flags2` bit0 を立てないよう Pi 側の指令生成ロジックで注意する

> **v0.12 での変更（★STM32 側発・実装済み。実機での動作検証は未了。2026-08-25。`pi_uart_protocol_v0.12_delta.md`）**
> - **ワイヤ形式・LEN の変更は無い。** 変わったのは `CONFIG_SET`/`CONFIG_GET` で
>   受け付ける `param_id` が増えたことと、`auto_stop`（`COMMAND.flags` bit7）の
>   下位側の判定ロジックだけなので、この版に未対応でも既存の通信はそのまま継続する
> - `auto_stop` の検知距離を、固定20cmから **`d_stop = v・t_delay + v²/(2・a_max) +
>   margin`**（進行方向の実走行速度 `v` に応じて伸びる）へ変更。`t_delay`/`a_max` は
>   STM32 側の固定定数（`VEHICLE_AUTO_STOP_DELAY_S`/`DRIVE_MAX_ACCEL_M_S2`）。
>   Pi から変えられるのは `margin` のcm単位の値そのもの（下記 `param_id = 0x0060`）
> - 判定ロジックを**超音波単独**から**LiDAR主・超音波補助**へ変更: LiDAR（進行方向
>   ±20°、直近300ms以内、3点以上で確定）／超音波フォールバック（LiDARの当該角度窓に
>   直近300ms以内のデータが無いときだけ`d_stop`と比較）／超音波近接フェイルセーフ
>   （LiDARの結果によらず5cm未満で常時作動）のOR判定（`uart_protocol.md` §5.6.4）
> - `param_id = 0x0060`（**自動停止の安全マージン / `auto_stop_margin_cm`**）を実装。
>   f32、cm単位の連続値、範囲0.0-100.0、既定15.0
> - **改訂（2026-08-25）**: 当初 `AutoStopLevel`（1=`NEAR`(5cm)/2=`STANDARD`(15cm)/
>   3=`FAR`(30cm) の3段階enum）として実装していたが、実機投入前にcm直接指定へ変更した。
>   `param_id` は同じ `0x0060` のまま、値の意味だけが変わっている
> - `protocol_version` を `0x000B`→**`0x000C`** に上げる（enumからcm直接指定への
>   改訂時点では上げ直していない。**値の意味の変更に気づかず古い前提でクランプ範囲
>   （1〜3）のまま実装しないこと**）
> - Pi 側は `io_node` に4つ目の `CONFIG_GET` 初期同期と、GUI の操作（`SettingsPanel`
>   のスライダー、0-100cm）に応じた `CONFIG_SET` 送信を実装（`uart_protocol.md` §5.8.3）
> - 旧版（v0.7〜v0.11）の固定20cmとは意味が異なる値。互換性は意図的に取らない
>   （`uart_protocol.md` §5.8.3）

> **v0.11 での変更（★STM32 側発・実装済み。実機での動作検証は未了。2026-08-21）**
> - **ワイヤ形式・LEN の変更は無い。** 新設したのは `LIMITS`(0x0A)/`LIMITS_REQ`(0x15) の
>   2つだけなので、この版に未対応でも既存の通信はそのまま継続する
> - `LIMITS` (0x0A, LEN=16): `max_speed_m_s`/`max_accel_m_s2`/`max_torque_nm`/
>   `max_steer_rad`（f32×4、読み取り専用）を STM32 → Pi へ返す。`VERSION` と同じく
>   起動直後 3回自発送信 + `LIMITS_REQ` (0x15, LEN=0) への応答（§4.3/4.4）
> - v0.10 で `MAX_SPEED`/`MAX_ACCEL`/`MAX_STEER` の `param_id` を廃止した結果、
>   Pi が STM32 側の実際の固定上限値を知る手段が無くなっていた問題への対応
> - `protocol_version` を `0x000A`→**`0x000B`** に上げる
> - Pi 側は `io_node.handshake()` で `VERSION_REQ`/`LIMITS_REQ` を併せて送り、未受信なら
>   1秒おきに再送する。受け取った `LIMITS` は RC（MANUAL）・自律走行（AUTO）どちらの
>   `COMMAND` 送信でも**無条件に採用する**（2026-08-22。当初は Pi 側設定上限との
>   「小さい方」だったが、GUI 側が実測値を見せているのに実車だけ古い設定値で頭打ちに
>   なる食い違いを避けるため変更）。`--max-speed`/`--max-steer` は `LIMITS` 未受信の
>   間だけ使うフォールバック（`uart_protocol.md` §5.12）

> **v0.10 での変更（★STM32 側発・実装済み。実機での動作検証は未了。2026-08-21）**
> - **ワイヤ形式・LEN の変更は無い。** 変わったのは `CONFIG_SET`/`CONFIG_GET` で
>   受け付ける `param_id` だけなので、この版に未対応でも通信はそのまま継続する
> - `param_id = 0x0001`（最大速度）/`0x0002`（最大加速度）/`0x0003`（最大舵角）を**廃止**。
>   以後は常に `RAS_CONFIG_UNKNOWN_PARAM` を返す
> - これらが担っていた上限は STM32 側の固定定数に一本化: `DRIVE_MAX_SPEED_M_S`
>   （5.0 m/s）、`DRIVE_MAX_ACCEL_M_S2`（3.0 m/s²）、路面舵角 ±30°
> - `COMMAND` の `accel_limit`/`steer_rate_limit`（毎指令ごとのレート制限）は今回の
>   変更と無関係で、従来通り使える
> - `protocol_version` を `0x0009`→**`0x000A`** に上げる
> - **Pi はこの3つの `param_id` を元々送信していなかった**ため（`uart_protocol.md` §5.8.1）、
>   Pi 側の対応は `protocol_version` の更新のみ

> **v0.9 での変更（★STM32 側 実装・実機動作確認済み。2026-08-20）**
> - **ワイヤ形式・LEN の変更は無い。** 変わったのは `CONFIG_SET`/`CONFIG_GET` で
>   受け付ける `param_id` だけなので、この版に未対応でも通信はそのまま継続する
> - `param_id = 0x0050`（**片輪浮き対策 / Wheel Lift Guard**）を実装。
>   後輪片浮き（接地荷重ゼロ）でモータが無負荷空転する問題への対策
> - **TC 本体（`0x0010`）とは独立した別機構**。個別に有効/無効を切り替えられる。
>   既定値は**有効**
> - Pi 側は `io_node` がハンドシェイク直後に3つ目の `CONFIG_GET` を送って初期状態を
>   同期し、以降は GUI のトグル（`SettingsPanel`）に応じて `CONFIG_SET` を送る
> - `0x0051`（しきい値・ゲイン）は**未実装**。しきい値は固定値

> **v0.8 での変更（★STM32 側 実装済み。2026-08-19）**
> - **ワイヤ形式・LEN の変更は無い。** v0.9 と同じく `param_id` の対応が増えただけ
> - `param_id = 0x0010`（**TC 有効**）/ `0x0020`（**TV 有効**）が実際に機能するように
>   なった。**それ以前は `RAS_CONFIG_UNKNOWN_PARAM`（`result=1`）を返していた**
> - Pi 側は `io_node` にハンドシェイク直後の `CONFIG_GET` 初期同期と、GUI トグル操作に
>   応じた `CONFIG_SET` 送信を実装（`uart_protocol.md` §5.8）
> - `MAX_SPEED`/`MAX_ACCEL`/`MAX_STEER`（`0x0001`-`0x0003`）は STM32 側で既に
>   クランプに使われているが、**Pi からは意図的に送っていない**（Flash 非永続化かつ
>   動的変更のユースケースが無いため。`uart_protocol.md` §5.8.1）

> **v0.7 での変更（★STM32 側 実装・ベンチ確認済み。2026-08-11）**
> - **LEN は全パケット変更なし。** `flags` の空きビットを使うだけなので、
>   片側が未対応でも通信は継続する（bit7 を立てなければ v0.6 と同じ挙動）
> - `COMMAND` (0x10) の `flags` bit7 に **`auto_stop`** を新設。立っている間、
>   **進行方向**の超音波が **20cm 未満**なら STM32 が指令を無視して最大制動する
> - `TELEMETRY` (0x02) の `flags` bit16 に **`auto_stop_active`**（今まさに介入中）を新設
> - 優先順位は **`brake` > `auto_stop` > 通常指令**。`auto_stop` 単独での作動時は
>   `brake_torque` を見ず**常に最大制動**
> - 検知距離 20cm は固定でヒステリシス無し。`CONFIG_SET` の param も無い
>   （`uart_protocol.md` §14 #9）。LiDAR による全周版は未実装（同 #10）

> **v0.6 での変更（★STM32 側も実装・実車確認済み。2026-08-11）**
> - `COMMAND` (0x10) の LEN が 12→**14**。末尾に `target_torque : int16_t`
>   （0.0001 N·m、負=後退方向）を追加
> - `flags` bit6 に `torque_mode` を新設。立っている間は `brake` と同様に車速 PI を
>   迂回し `target_torque` を各輪へ直接掛ける（`brake` が同時に立っていれば `brake` 優先）
> - 上限は `target_torque`/`brake_torque` とも **0.125 N·m**（モータ物理上限 0.1557 N·m
>   未満。実車確認後に 0.1/0.075 N·m から引き上げ）。Pi 側は送信前にクランプ済みだが、
>   STM32 側でも同じ上限でクランプすること（多層防御）
> - TC/TV が `torque_mode` 中の `target_torque` にどう関与するかは未確定
>   （`uart_protocol.md` §14 #8）。実装方針が決まったらこの節と §4.4 を更新すること

> **v0.3 での変更**（STM32 側「v0.4 合意版」を反映。**本書の v0.2 は破棄**）
> - `TELEMETRY` の**フィールド順を変更**（`odom_dist` を offset 24 へ、accel/pitch/roll を 32-41 へ）
> - `STATS` を **増分 u16 (LEN 22) → 累積 u32 (LEN 48)**
> - **`VERSION_REQ` (0x14)** を新設。`CONFIG_SAVE` は**不採用**（0x14 は VERSION_REQ が使う）
> - `param 0x0040` を **bool → enum** に変更、`0x0041` は廃止
> - `flags` bit9 を予約に、bit10 を **`steer_center_valid`** に改名
> - `CONFIG_SET` は**揮発**（Flash 保存なし）に戻した（§10.3）
>
> **v0.2 で書いた `TELEMETRY` レイアウト・`STATS`・`CONFIG_SAVE` は実装しないこと。**


---

## 変更履歴表


| バージョン | 日付 | 内容 |
|---|---|---|
| **v0.22** | 2026-10-11 | **`uart_protocol.md` v0.22 に対応。実装済み・ビルド済み、未書き込み。ワイヤ形式・LEN の変更なし。** 目標車速のランプの固定上限（`DRIVE_MAX_ACCEL_M_S2` 3.0）を廃止し、`COMMAND.accel_limit` をそのまま使う（0 のときの既定は 3.0）。`LIMITS.max_accel_m_s2` は 65.535。自動停止の減速度は `VEHICLE_AUTO_STOP_DECEL_M_S2` に分離。`protocol_version` 0x0016 |
| **v0.21** | 2026-10-11 | **`uart_protocol.md` v0.21 に対応。実装済み・ビルド済み、未書き込み。ワイヤ形式・LEN の変更なし。** 車速PIの加速度フィードフォワード（`param_id` `0x00B1`・`0x00B2`、`drive.c` の `SpeedFeedforward()`）。`protocol_version` `0x0014`→`0x0015` |
| **v0.20** | 2026-10-09 | **`uart_protocol.md` v0.20 に対応。実装済み・ビルド済み、未書き込み。ワイヤ形式・LEN の変更なし。** ローンチコントロール（`COMMAND.flags2` bit3 `LAUNCH`、`flags` bit24 `LAUNCH_ACTIVE`、`param_id` `0x00A1` `launch_exit_torque_nm`）。`protocol_version` `0x0014` |
| **v0.18** | 2026-10-08 | **`uart_protocol.md` v0.18 に対応。実装済み・ビルド済み、未書き込み。ワイヤ形式・LEN の変更なし。** ステアのリンクの換算（モータ角→路面舵角の3次式、`param_id` `0x0081`・`0x0082`、設定時に `LIMITS` を再送）。舵角の指令・報告・最大舵角が実際の路面舵角になる |
| **v0.19** | 2026-10-09 | **`uart_protocol.md` v0.19 に対応。実装済み・ビルド済み、未書き込み。ワイヤ形式・LEN の変更なし。** `flags` bit2 `ARMED` の意味を `Vehicle_IsArmed()` へ変更、bit22 `DRIVE_POWER_ON`・bit23 `MD_FAULT`・`md_status` bit6 `UNCALIBRATED` を新設。惰行はトルク 0 を送る、ARM が外れても停車まで制動、MD の健全性監視と起動待ち、ステア未較正のガード、フェイルセーフの位置保持、IMU 復旧の非ブロッキング化、ジャイロ ±500 deg/s。`protocol_version` を `0x0012`→`0x0013` に更新（`pi_uart_protocol_v0.19_delta.md`） |
| **v0.17** | 2026-10-08 | **`uart_protocol.md` v0.17 に対応。実装済み・ビルド済み、未書き込み。ワイヤ形式・LEN の変更なし。** TV をヨーレートの PI から荷重比例の配分へ置き換え（`param_id` `0x0021`-`0x0029` 廃止、`0x002B`・`0x002C` 新設、`TELEMETRY` オフセット 66 を `tv_ratio` に） |
| **v0.16** | 2026-10-05 | **`uart_protocol.md` v0.16 に対応。実装済み・ビルド済み、未書き込み。`TELEMETRY` の LEN 74→84（制御の介入量を追加）。** 足回りの制御の調整パラメータ（`src/control/control_params.h` の表）を `CONFIG_SET`/`CONFIG_GET` に追加。TC・ABS をスリップ率の連続 PI へ、片輪を絞った分を反対輪へ載せない、TV の配分・規範の1次遅れ・同定用のヨーモーメント注入。`host/` でホスト実行。`protocol_version` を `0x000F`→`0x0010` に更新 |
| **v0.15** | 2026-09-27 | **`uart_protocol.md` v0.15 に対応。STM32 側発・実装済み、実機での動作検証は未了。ワイヤ形式・LEN の変更なし。** ABS（制動時の後輪ロック防止）を追加。`param_id = 0x0070`（`ABS_ENABLE`、既定は有効）、`TELEMETRY.flags` bit20=`ABS_ACTIVE`。`protocol_version` を `0x000E`→`0x000F` に更新。Pi 側も対応済み |
| **v0.14** | 2026-08-30 | **`uart_protocol.md` v0.14 に対応。Pi 側発の提案。STM32 側は未実装。** `COMMAND`(0x10) の `flags2` に bit1=`WINKER_LEFT`・bit2=`WINKER_RIGHT` を追加、`TELEMETRY`(0x02) の `flags` に bit18=`WINKER_LEFT_ACTIVE`・bit19=`WINKER_RIGHT_ACTIVE` を追加（**ワイヤ形式・LEN の変更は無し**）。自律走行中に右左折・車線変更の意思表示ができない実装漏れへの対応（`Lighting_SetWinker()` は既存だが `ApplyRasCommand()` から呼ばれていなかった）。両ビットを両方立てるとハザード（左右同時点滅）。点滅の周期・位相・既存灯火系との調停は STM32 側に一任。`protocol_version` を `0x000D`→`0x000E` に更新。Pi 側は `protocol.toml`/`raspi/msgs`/`sim/stm32.py`/GUI を実装済み。**STM32 側は `ApplyRasCommand()` への配線・`TELEMETRY.flags` の返却・`COMMAND` 途絶時の強制解除が未実装** |
| **v0.13** | 2026-08-28 | **`uart_protocol.md` v0.13 に対応。STM32 側発・実装済み、実機での動作検証は未了。** `TELEMETRY`(0x02) の `torque_cmd[2]` 直後に `slip[2]`（TC用スリップ率、無次元）・`tc_limit_nm[2]`（TCが動的に決めるトルク上限）を追加（LEN 66→**74**）。TC のゲイン（`DRIVE_TC_CUT_GAIN`/`DRIVE_TC_RECOVER_RATE`）を実機で追い込むにあたり `flags` bit5（`tc_active`、介入中か否かの1bit）だけでは内部状態が分からなかった問題への対応。加えて `COMMAND`(0x10) に `flags2`（bit0=`SIDE_BRAKE`）を新設し、後輪を機械的な位置制御へ切り替えて固定するサイドブレーキを追加（LEN 14→**15**）。速度に関わらず即座に切り替わり `brake` より優先、実際に固定できたかは `TELEMETRY.flags` bit17（`SIDE_BRAKE_ACTIVE`）で返す。`protocol_version` を `0x000C`→`0x000D` に更新。**TC ゲインの実測・サイドブレーキとも実機での動作検証は未実施**（`pi_uart_protocol_v0.13_delta.md`） |
| **v0.12** | 2026-08-25（`param_id=0x0060`の値を3段階enumからcm直接指定へ改訂） | **`uart_protocol.md` v0.12 に対応。STM32 側発・実装済み、実機での動作検証は未了。ワイヤ形式・LEN の変更なし。** `auto_stop` の検知距離を固定20cmから `d_stop = v・t_delay + v²/(2・a_max) + margin`（速度に応じて伸びる）へ変更し、判定を超音波単独からLiDAR主・超音波補助（フォールバック＋5cm近接フェイルセーフ）へ刷新。`CONFIG_SET`/`CONFIG_GET` の `param_id = 0x0060`（`auto_stop_margin_cm`: cm単位の連続値、範囲0-100、既定15）で `margin` を直接指定可能に。**当初 `AutoStopLevel`（1=NEAR/2=STANDARD(既定)/3=FAR の3段階enum）として実装されていたが、実機投入前にSTM32側がcm直接指定へ変更**（`param_id`は据え置き、値の意味だけ変更）。`protocol_version` を `0x000B`→`0x000C` に更新（enumからcmへの改訂時点では再度は上げていない）。Pi 側は `io_node` に4つ目の `CONFIG_GET` 初期同期と、GUI（`SettingsPanel`のスライダー）操作に応じた `CONFIG_SET` 送信を実装（`pi_uart_protocol_v0.12_delta.md`） |
| **v0.11** | 2026-08-22 | **`uart_protocol.md` v0.11 に対応。STM32 側発・実装済み、実機での動作検証は未了。ワイヤ形式・LEN の変更なし。** `LIMITS`(0x0A)/`LIMITS_REQ`(0x15) を新設。`LIMITS` は `max_speed_m_s`/`max_accel_m_s2`/`max_torque_nm`/`max_steer_rad`（f32×4・LEN16・読み取り専用）を返し、`VERSION` と同じく起動直後3回自発送信＋`LIMITS_REQ`応答。v0.10 で `MAX_SPEED`/`MAX_ACCEL`/`MAX_STEER` の `param_id` を廃止した結果 Pi が STM32 の実際の固定上限値を知れなくなっていた問題への対応。`protocol_version` を `0x000A`→`0x000B` に更新。Pi 側は `io_node.handshake()` で `VERSION_REQ`/`LIMITS_REQ` を併せて送り未受信なら1秒おきに再送、`IoNode._send_command` が RC/AUTO 問わず `LIMITS` 受信済みなら無条件にそちらでクランプし `--max-speed`/`--max-steer` は未受信時のみのフォールバックにするよう実装（`raspi/msgs/convert.py` に `max_accel`/`max_torque` 引数を追加。GUI・`sim/stm32.py` も同じ方針で対応） |
| **v0.10** | 2026-08-21 | **`uart_protocol.md` v0.10 に対応。STM32 側発・実装済み、実機での動作検証は未了。ワイヤ形式・LEN の変更なし。** `CONFIG_SET`/`CONFIG_GET` の `param_id = 0x0001`（最大速度）/`0x0002`（最大加速度）/`0x0003`（最大舵角）を廃止し、以後は常に `RAS_CONFIG_UNKNOWN_PARAM` を返す。上限は STM32 側の固定定数（`DRIVE_MAX_SPEED_M_S` = 5.0 m/s、`DRIVE_MAX_ACCEL_M_S2` = 3.0 m/s²、路面舵角 ±30°）に一本化。`COMMAND.accel_limit`/`steer_rate_limit`（毎指令ごとのレート制限）は無関係で変更なし。`protocol_version` を `0x0009`→`0x000A` に更新。Pi はこの3つの `param_id` を元々送信していなかったため、Pi 側の対応は `protocol_version` の更新のみ |
| **v0.9** | 2026-08-21 | **`uart_protocol.md` v0.9 に対応。STM32 側実装・実機動作確認済み（2026-08-20）。ワイヤ形式・LEN の変更なし。** `CONFIG_SET`/`CONFIG_GET` の `param_id = 0x0050`（**片輪浮き対策 / Wheel Lift Guard**）を実装。後輪片浮き（接地荷重ゼロ）でモータが無負荷空転する問題への対策で、**TC 本体（`0x0010`）とは独立に動作し個別に切り替えられる**。既定値は有効。`0x0051`（しきい値・ゲイン）は未実装で、しきい値は固定値。Pi 側は `io_node` に3つ目の `CONFIG_GET` 初期同期と、GUI トグル（`SettingsPanel`）に応じた `CONFIG_SET` 送信を実装 |
| **v0.8** | 2026-08-21 | **`uart_protocol.md` v0.8 に対応。STM32 側実装済み（2026-08-19）。ワイヤ形式・LEN の変更なし。** `CONFIG_SET`/`CONFIG_GET` の `param_id = 0x0010`（TC 有効）/ `0x0020`（TV 有効）が実際に機能するようになった（**それ以前は `RAS_CONFIG_UNKNOWN_PARAM` = `result=1` を返していた**）。Pi 側は `io_node` にハンドシェイク直後の `CONFIG_GET` 初期同期と、GUI トグル操作に応じた `CONFIG_SET` 送信を実装（`uart_protocol.md` §5.8）。`MAX_SPEED`/`MAX_ACCEL`/`MAX_STEER`（`0x0001`-`0x0003`）は STM32 側で既にクランプに使われているが、Flash 非永続化かつ動的変更のユースケースが無いため Pi からは意図的に未送信（同 §5.8.1） |
| **v0.7** | 2026-08-11 | **`uart_protocol.md` v0.7 に対応。STM32 側実装済み。** `COMMAND.flags` bit7 に `auto_stop`、`TELEMETRY.flags` bit16 に `auto_stop_active` を新設（**LEN 変更なし**）。進行方向の超音波 20cm 未満で最大制動、逆方向のセンサは見ない、検知不能時は不作動、ラッチなし。優先順位は `brake` > `auto_stop` > 通常指令。検知距離のパラメータ化・LiDAR 全周版は未実装（`uart_protocol.md` §14 #9/#10） |
| **v0.6** | 2026-08-11 | **`uart_protocol.md` v0.6 に対応。STM32 側実装・実車確認済み。** `COMMAND` を 12→**14** バイトに拡張し `target_torque : int16_t` を追加、`flags` bit6 に `torque_mode` を新設。実車確認後、`target_torque`/`brake_torque` の上限をともに 0.1/0.075 N·m → **0.125 N·m** へ引き上げ |
| v0.1 | 2026-08-06 | 初版。`uart_protocol.md` v0.3 に対応 |
| v0.2 | 2026-08-06 | `uart_protocol.md` v0.4 ドラフトに対応。**確定版で覆された箇所があるため破棄** |
| **v0.4** | 2026-08-09 | **`uart_protocol.md` v0.5 に対応（STM32 側実装済みの内容を反映）。** `COMMAND` を 10→**12** バイトに拡張し `brake_torque : u16` を追加。`flags` bit3 の1ビット `light` を bit3-4 の2ビット `light_mode`（OFF/DAYTIME/NORMAL）へ、bit5 に `passing` を新設。`horn` を「立てている間ずっと鳴る」に変更。`brake` 中は車速 PI を迂回して指定トルクを直接掛ける。緊急停止・`COMMAND` 途絶時は最大制動トルク。起動時に駆動電源が一瞬 ON になる問題を修正。`torque_cmd` の符号を「正=駆動 / 負=制動」と明確化 |
| v0.3 | 2026-08-06 | **`uart_protocol.md` v0.4 確定版に対応。** `TELEMETRY` を全面改訂（53→66B、電源2系統・`wheel_speed[4]`・`odom_dist[2]`・`accel_z`・`torque_cmd`・`md_status[3]`）し、**フィールド順を `wheel_speed`→`odom_dist`→`accel` に確定**。`flags` を u32 化し `mode` を bit0-1 へ移動、bit9 を予約・bit10 を `steer_center_valid` に改名。`LIDAR_SECTOR` の `angle_start`/`angle_step` を `duration_us`/`rot_speed_dps` に置換、ビニングを代表点方式に変更。`VERSION`(0x07)/`STATS`(0x08、**累積 u32・LEN 48**)/`LIDAR_SECTOR_C`(0x09)/`CONFIG_GET`(0x13)/**`VERSION_REQ`(0x14)** を追加。`LOG` に `severity` 追加。`param 0x0040` を **enum 化**（`0x0041` 廃止）。`COMMAND` タイムアウト 200→100ms。`CALIB` モード廃止。ハートビートを PB12 ポーリング方式に。**`CONFIG_SAVE` は不採用**、制御パラメータは揮発。時刻同期の T2/T3 取得タイミングを厳密化。`speed` の車体中心線射影を STM32 の責務として明記 |
