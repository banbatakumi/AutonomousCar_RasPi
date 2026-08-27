# SURGE Mark.2 開発進捗

会話が圧縮されても文脈を失わないための作業ログ。**新しいセッションではまずこれを読む。**
設計の中身は `docs/` が正。ここには「今どこまでやったか」「なぜそう決めたか」の要約だけを書く。

最終更新: 2026-08-28（UART プロトコル v0.13 対応。`TELEMETRY` に TC の
スリップ率・トルク上限を追加、`COMMAND` にサイドブレーキ `flags2` を新設。
下記「2026-08-28」節）
旧: 2026-08-25（UART プロトコル v0.12 対応。`auto_stop` の判定を固定20cmから
動的停止距離＋LiDAR併用へ刷新、安全マージンを `CONFIG_SET` param_id 0x0060 で
3段階切替可能に。下記「2026-08-25」節）
旧: 2026-08-23（zbus のトピック登録漏れを修正／中心線を壁の穴に頑健化／
`lidar_only` 実験モードを追加）
旧: 2026-08-22（SLAM の未解決3点に着手）
旧: 2026-08-21（コードレビュー13項目を全修正し実機へ反映）
旧: 2026-08-17（3561行あった本ファイルを軽量版とアーカイブに分割。**情報は削除しておらず、
詳細な実験経緯・実測値はすべて `docs/progress_archive.md` に移した**。分割前と同じ内容が
アーカイブに残っているので、根拠が要るときはそちらを見ること）

方針の最新は 2026-08-14 の **★ SLAM を一旦棚上げし、非SLAM の Disparity Extender で進める**（下記）。
**2026-08-22 にバンビの指示で SLAM 側の修正を再開した**（下記「2026-08-22」節）が、
`de` を本線とする方針そのものは変わっていない。

---

## ★ 2026-08-28：UART プロトコル v0.13 対応（STM32側発の delta doc を受けて）

STM32 側から `pi_uart_protocol_v0.13_delta.md` が届き、Pi 側の対応チェックリストに
従って実装した。**STM32 側は実装済みだが実機での動作検証は未了**（TC ゲインの
実測・サイドブレーキとも）。

### 変わったこと

- `TELEMETRY`（LEN 66→**74**）: `torque_cmd[2]` の直後に `slip[2]`（TC用スリップ率、
  無次元、正=空転/負=ロック傾向）・`tc_limit_nm[2]`（TCが動的に決めるトルク上限）を
  追加。TC のゲイン（`DRIVE_TC_CUT_GAIN`/`DRIVE_TC_RECOVER_RATE`）を実機で追い込むには
  `flags` bit5（`tc_active`、介入中か否かの1bit）だけでは内部状態が見えなかったため
- `COMMAND`（LEN 14→**15**）: 末尾に `flags2`（u8、bit0=`SIDE_BRAKE`）を新設。立てている
  間、速度に関わらず即座に後輪を機械的な位置制御へ切り替えて固定する（既存の
  `flags` bit1 `BRAKE` より優先）。実際に固定できたかは `TELEMETRY.flags` bit17
  （`SIDE_BRAKE_ACTIVE`）で返る
- `protocol_version` を `0x000C`→`0x000D`

### 変更箇所

- `raspi/proto/protocol.toml`: version bump、`TELEMETRY` に `slip`/`tc_limit_nm`、
  `COMMAND` に `flags2`、`bits.FLG.SIDE_BRAKE_ACTIVE`（bit17）、
  `bits.CMD_FLG2.SIDE_BRAKE`（新設）（生成物は `generate.py` で再生成済み）
- `raspi/msgs/types.py`: `VehicleState` に `tc_slip`/`tc_limit_nm`/`side_brake_active`、
  `DriveCmd` に `side_brake` を追加
- `raspi/msgs/convert.py`: `StateBuilder.build()` で `slip`/`tc_limit_nm` をSI換算、
  `decode_flags()` に `side_brake_active`、`command_from_cmd()` で
  `cmd.side_brake` → `flags2` を組み立て
- `sim/stm32.py`: TC/TVは未実装（タイヤモデルが無い）ため `slip=[0,0]`・
  `tc_limit_nm=DRIVE_MAX_TORQUE_NM`固定で返す。`side_brake` は個別車輪の位置制御
  モデルが無いため**最大制動として近似**（`_substep` で `flags2` を見て
  `DriveInput` を上書き）。`FLG_SIDE_BRAKE_ACTIVE` を反映
- `raspi/tests/test_proto.py`: `PROTOCOL_VERSION`/LEN/バイトオフセットの期待値を更新
- `docs/uart_protocol.md`/`docs/stm32_interface.md`/`docs/README.md`:
  バージョン表記・§5.3・§5.4・§5.6・§5.6.5(新設)・changelog を更新
  （`config/check_docs.py --check` 通過）

`./tools/check.sh`（生成物一致・文書版番号・pytest 513件・tsc）は全部通した。
**実機での動作検証はまだ**（STM32側も同様）。

### 追記（同日）: GUI にサイドブレーキ操作とTCグラフを追加

- `gui/src/types.ts`: `CmdOut.side_brake` を追加
- `gui/src/store/ui.ts`: `sideBrakeRequested`（ON/OFFトグル。`braking`等の
  「押している間だけ」とは違い、駐車ブレーキと同じ「かけたら手を離せる」操作感。
  誤操作防止のため `localStorage` には保存せず、再読み込みで必ず false に戻す）
- `gui/src/input/useDriving.ts`: 全 `ch.cmd()` 呼び出しに `side_brake` を追加。
  未ARM時は `brake` と同様に常に false（モータ非励磁で意味が無い）
- `gui/src/components/DriveControls.tsx`: 灯火・ファンと同じ「タブ行に1つだけ」
  の場所にON/OFFトグルを追加（ラジコン/自動運転どちらのタブでも共通。
  `AuxPanel.tsx` は自動運転タブにしか出ないので複製しなかった）。走行中の誤操作
  防止のため、`vs.stopped` が false の間は ON ボタンを無効化
  （`title` でツールチップ表示）
- `gui/src/bus/history.ts`/`components/DiagCharts.tsx`: `tc_slip`/`tc_limit_nm`
  を時系列バッファ・診断タブのグラフに追加。**スリップ%とトルク上限N·mは値域が
  全く違うので別チャートに分けた**（同じ軸に載せると片方が潰れる）
- `gui/src/styles.css`: `.seg button:disabled` を追加（既存の `.armbtn:disabled`
  はあったが `.seg` 系トグルには無かった）

`./tools/check.sh` 再実行、`vite build` も確認。**ブラウザでの目視確認はしていない**
（環境上ブラウザ操作ができないため）。実機・シムでの動作確認は次回に持ち越し。

---

## ★ 2026-08-25：UART プロトコル v0.12 対応（STM32側発の delta doc を受けて）

STM32 側から `pi_uart_protocol_v0.12_delta.md` が届き、Pi 側の対応チェックリストに
従って実装した。**STM32 側は実装済みだが実機での動作検証は未了**なので、Pi 側も
それに合わせて未検証の前提で実装（実機で誤作動が出たら `auto_stop` を切って様子見）。

### 変わったこと

- `auto_stop`（`COMMAND.flags` bit7）の判定が、**固定20cm・超音波単独**から
  **`d_stop = v・t_delay + v²/(2・a_max) + margin`（速度に応じて伸びる）・
  LiDAR主センサ＋超音波補助（フォールバック＋5cm近接フェイルセーフ）** へ変更。
  `t_delay`/`a_max` は STM32 固定定数、Pi から変えられるのは `margin` の3段階
  （NEAR=5cm/STANDARD=15cm(既定)/FAR=30cm）のみ
- `CONFIG_SET`/`CONFIG_GET` に `param_id = 0x0060`（`AutoStopLevel`）を追加。
  `TC_ENABLE`/`TV_ENABLE`/`WHEEL_LIFT_GUARD_ENABLE` と同じ形（整数enumをf32で
  送受信、`io_node` がハンドシェイク直後に `CONFIG_GET` で初期同期、GUI操作で
  `CONFIG_SET`）
- `protocol_version` を `0x000B`→`0x000C`

### 変更箇所

- `raspi/proto/protocol.toml`: version bump、`enums.AUTO_STOP_LEVEL`、
  `params.AUTO_STOP_LEVEL = 0x0060`（生成物は `generate.py` で再生成済み）
- `raspi/core/link_tracker.py`: `CONFIG_ACK_BOOL_PARAMS` は bool 専用のままにし、
  整数enum用に `CONFIG_ACK_INT_PARAMS` を新設（`LinkState.auto_stop_level: int | None`）
- `raspi/msgs/types.py`: `LinkDiag.auto_stop_level` 追加。`UiEvent` に `int_value`
  フィールドを追加（既存の `value: bool` とは別枠——混ぜると0/1に丸まるため）
- `raspi/core/bus_bridge.py`: `LinkDiag` 組み立てに `auto_stop_level` を追加
- `raspi/nodes/io_node.py`: `ui/event` の `"auto_stop_level"` kind を処理して
  `CONFIG_SET` 送信、ハンドシェイク後に4つ目の `CONFIG_GET` を追加
- `raspi/nodes/telemetry_node.py`: `/ws/control` の `"auto_stop_level"` kind
  （`{level: 1|2|3}`）を `UiEvent` にブリッジ
- `sim/stm32.py`: `_config` の初期値に `AUTO_STOP_LEVEL: STANDARD` を追加
  （他の bool 系パラメータと違い、未設定時の 0.0 が enum のどの値でもなく
  不正値になってしまうため）
- GUI: `gui/src/ws/control.ts` に `setAutoStopLevel()`、`SettingsPanel.tsx` の
  安全タブに margin 3段階セグメント（`link.auto_stop_level` がサーバ真値）。
  `store/ui.ts` の `AUTO_STOP_DISTANCE_M`（固定20cm前提の表示用定数）は廃止
- `docs/uart_protocol.md`/`docs/stm32_interface.md`/`docs/README.md`:
  バージョン表記・§5.6.4・§5.8.3・changelog を更新（`config/check_docs.py --check` 通過）
- `raspi/tests/test_proto.py`: `PROTOCOL_VERSION` の期待値を更新

`./tools/check.sh`（生成物一致・文書版番号・pytest 504件）と `gui` の `tsc -b` は
全部通した。**実機での動作検証はまだ**（STM32側も同様）。

### 実機反映で `surge-telemetry` の再起動漏れが型不一致を起こした

`tools/deploy.sh --restart-io` で `surge-io` は再起動したが、`surge-telemetry` は
再起動していなかった。`gui/src/generated/msgs.ts` の `MSGS_SCHEMA` は
`raspi/msgs/types.py` の型構造から決定的に導かれる値（`raspi/msgs/schema.py`）で、
`LinkDiag` にフィールドを足すと変わる。ビルド済みGUIは新スキーマだが、
`surge-telemetry` プロセスは Python を再インポートしない限り古い型定義を
メモリに持ったままなので、`/ws/telemetry` で送るスキーマ値が食い違ったまま
「テレメトリの型定義が食い違っています」の警告が出続けた。
**`raspi/msgs/types.py`（`LinkDiag`/`Scan`/`VehicleState`/`AutoState`/`AutoMap`
のいずれか）を触ったときは `surge-io` だけでなく `surge-telemetry` も
再起動が要る**（`tools/deploy.sh --restart` または `--restart-io` と併用）。

### `param_id = 0x0060` を3段階enumからcm直接指定へ改訂（同日中に revert 相当）

上記の実装・実機反映の直後、STM32側から改訂版の delta doc が届いた。**実機投入前に
STM32側の判断で、`auto_stop` の安全マージンを `AutoStopLevel`（NEAR/STANDARD/FAR
の3段階enum）ではなく `auto_stop_margin_cm`（0.0-100.0cmの連続値、既定15.0）を
直接指定する方式へ変更**。`param_id`（`0x0060`）自体は変わっていないが、
**`protocol_version` は据え置き（`0x000C`のまま）で値の意味だけが変わった**——
ワイヤ上は区別できないので、Pi側の実装が新しい方に揃っているかは人間が確認するしかない
（`docs/stm32_interface.md` にその旨を明記した）。

Pi側は enum 関連のコードを一通り書き換えた: `link_tracker.py` の
`CONFIG_ACK_INT_PARAMS`→`CONFIG_ACK_FLOAT_PARAMS`（`applied` をそのままfloatで
持つ）、`LinkState`/`LinkDiag` の `auto_stop_level: int`→`auto_stop_margin_cm: float`、
`UiEvent.int_value`→`float_value`（他に使う場所が無かったのでrename）、
GUI の `SettingsPanel.tsx` は3段階の `.seg` ボタンから0-100cmの `<input type=range>`
スライダーへ（`cam?.front_cap_hz` のfpsスライダーと同じパターン）。
`sim/stm32.py` の既定値も `AutoStopLevel.STANDARD` → `15.0`(cm) に修正。
protocol_versionは変えていないので `raspi/tests/test_proto.py` の再修正は不要だった。

---

## ★ 2026-08-23：zbus トピック登録漏れ／中心線の壁穴耐性／`lidar_only` 実験モード

3件とも実機・sim.run 起動時のエラー報告や GUI スクリーンショットから見つかった。

### zbus: `line/cam` の発行元が未登録で `planning_node` が起動時に落ちる

`8ada3f0`（line tracing機能とperception nodeを追加）で `line_trace` プランナー
（`line/cam` トピック）を `raspi/auto/registry.py` に足したが、`raspi/bus/zbus.py`
の `TOPIC_OWNER` に登録し忘れていた。`planning_node.main()` は**登録されている
全プランナー**の `input_topic` をまとめて購読するため、`line_trace` を選ばなくても
起動時に `KeyError` で落ちる（`.venv/bin/python -m sim.run` で再現・報告あり）。

ついでに調べたら `scan/cam`（`ftg_cam` 用、`cam_perception_node`）にも**同種だが
気づきにくいバグ**があった。`"scan/cam"` の完全一致キーが無く、前方一致
フォールバックが `"scan"`（io ノード）に誤って一致し、例外は出ないが
`cam_perception_node` のデータが誰にも届かない状態だった。

`raspi/bus/zbus.py` の `TOPIC_OWNER` に `"scan/cam": "cam_perception"` と
`"line/cam": "line_perception"` を追加（`_NODE_TCP_PORT` とハートビート配信先にも）。
再発防止に `raspi/tests/test_zbus.py::TestTopicOwner` を追加
（登録済み全プランナーの `input_topic` が解決できることを検証）。

### 中心線が壁の小さな穴で局所的に暴れる（GUI スクリーンショットで発覚）

複雑なコース（ヘアピン・S字を含む）を走らせた GUI の地図で、中心線が特定の
場所だけ激しく乱れる症状が報告された。原因は `nav/centerline.py` の
`measure()`——法線方向のレイが、`min_hits` を満たせなかった壁の小さな穴
（`OccGrid.raycast()` の既定 `fill=1` セルでは塞ぎきれない広さの穴）を抜けて
**本来のレーンとは無関係な奥の壁**に当たり、その1点だけ幅が数十cm〜
`max_width` へ跳ね上がっていた。`build()` の「幅の中間へ寄せる」反復が
その異常値をそのまま使うので、中心線がその1点だけ大きく飛ぶ。

`_declutter()`（近傍5点の移動中央値より 0.5m 以上大きい値だけをクランプ）を
`measure()` に追加。**狭い側へは倒さない**（本当に狭い場所を誤って広げない
ため）ので、本物の広い区間（コース出口など）は連続して広い＝中央値も高く
保たれ、削られない。単体テスト `raspi/tests/test_centerline.py`（新設）で
「穴は無視する」「本物の開口部は削らない」「狭い側は広げない」を確認。

**注意: これは中心線側の対症療法であり、壁に穴が空くこと自体の根治ではない。**
`_END_BACKOFF`（`nav/grid.py`）でコーナー内側の虫食いは 2026-08-14 に
対処済みだが、実車の LiDAR ノイズ・複雑コースの狭い区間ではまだ穴が
残りうる。本丸は引き続き `_END_BACKOFF` やコース側の走らせ方の見直し。

### ★ 上の `_declutter()` だけでは足りなかった。原因は「穴」ではなく知覚エイリアシング

`_declutter()` を入れた直後、バンビから同じコース（`sim.run` 実行中の GUI
スクリーンショット）で「点線の中心線が所々破綻してる」と再報告があり、
改善はしたが直り切っていないことが分かった。`sim.bench` に `RaceLine` の
`centerline` を直接読みに行くスクリプトを書いて実測（`fuji` コース、急な
0.5m 半径のシケイン区間）したところ、**壁の穴は無関係**で、隣接する
数十点にわたって `w_left`/`w_right` が点ごとに激しく反転していた
（`0.63/1.73 → 0.43/1.63 → 2.41/0.04 → 2.26/1.95 → …`）。`_declutter()` は
孤立した1点の異常値にしか効かないので、近傍ごと汚染されるこの症状は
すり抜けていた。

原因は「穴」ではなく**急カーブでの法線方向の脆さ**（知覚エイリアシング）。
中央差分（`tangents()`）で作る法線がわずかに傾くと、レイが数十cm先の
本来の壁ではなく通路に沿って何mも先まで抜け、点ごとに拾う距離が
無関係にばらつく。何が「正しい壁」かを区別する情報が無いので、異常値
検出では直せない。代わりに `build()` の反復1回で動かせる量に上限
（`_MAX_SHIFT_M = 0.15m`）を掛けた——1回の測定がどれだけ暴れても
中心線の点が一気にテレポートしなくなり、`iters` 回の反復で緩やかに
真ん中へ寄っていく。

**効果を実測**（`sim.bench` で `fuji` コースの `centerline.w_left/w_right` を直接読む）:

| | 幅が乱れた点の数 | 症状 |
|---|---|---|
| 修正前（`_declutter()` のみ） | シケイン区間で数十点が連続で反転 | 0.5m 前後が正常のところ、隣と点ごとに 0.04〜3.0m へ跳ね回る |
| **修正後（`_MAX_SHIFT_M` 追加）** | **1〜2点の局所的な残留のみ** | 跳ね幅も 3.0m（上限）→ 1.9m 程度まで縮小 |

なお同じコースの別区間で「30点連続して `right≈1.5〜2.0m`」という広い読みも
出たが、こちらは**点ごとになめらかに変化する本物の道幅**（コース設計上の
広い区間）で、`_declutter()`/`_MAX_SHIFT_M` とも正しく削らずに残していた
（狙い通り）。

**それでも残る限界**: `_MAX_SHIFT_M` は「1回のテレポート」を防ぐだけで、
急カーブそのものの測定精度を上げるわけではない。GUI で「一直線に飛ぶ」
症状は大幅に緩和されるはずだが、タイトなシケイン内で中心線が完全に
滑らかになる保証はない。実測では上記の通り 1〜2点の残留があった。
根本的に直すなら、法線推定を接ベクトルの単純な中央差分より頑健にする
（例: より長いベースラインで平滑化する、あるいは EDT ベースの手法に
置き換える）必要があるが、今回は着手していない。

### `lidar_only` 実験パラメータを追加（IMU/オドメトリ依存の切り分け用）

バンビから「今の SLAM は IMU/オドメトリを使っているのか、LiDAR だけの SLAM も
使えるようにしてほしい」との要望。回答: **姿勢を最終的に決めるのは常に
LiDAR のスキャンマッチ**で、ジャイロ/`speed` は探索の初期値（100ms 予測）と
点群のデスキューにしか使っていない（`nav/slam.py` の docstring）。とはいえ
「ジャイロ/`speed` を一切使わない」経路は無かったので、比較・切り分け用に
`RaceLine` へパラメータ `lidar_only`（既定0）を追加した。

1 にすると `Slam.update()` へ `yaw_rate=None, speed=None` を渡し（＝ジャイロ・
`speed` を隠す）、代わりに `SlamConfig.scan_to_scan`（LiDAR のスキャン同士の
直接照合、`nav/scan2scan.py`。既存機能だが既定オフだった）を自動で有効にする。
`sim.bench --set lidar_only=1` で実走確認済み（circuit・自己位置平均 7.4cm・
地図正確さ99%。ジャイロ+speed 使用時の 4.4cm より劣るが動作はする）。

**この機能自体は当面の課題（GUI 画像の中心線の乱れ）を直すものではない。**
むしろヘアピンのような近接した平行区間では、ジャイロ/`speed` という
先行知識（prior）が無いぶん**知覚エイリアシング（違う壁に吸い寄せられる）
のリスクは上がる**可能性がある。IMU/オドメトリへの依存を切り分けたい
実験・比較の道具として使うこと。

---

## ★ 2026-08-22：SLAM の未解決3点（oval渦巻き・壁の虫食い再発懸念）に着手。**シム・実車とも未検証**

バンビから「コース走行時に壁が閉じ切らない（特に旋回時）・ループが閉じない」と報告があり、
`docs/progress_archive.md`「oval が渦巻きになる」「壁が虫食いになる」の両節を見直した。
**この症状は 2026-08-14 に一度診断済みで、SLAM の原理的な限界ではなく実装バグと特定されていた。**

- **壁の虫食い**: `OccGrid._carve()` の `_END_BACKOFF=1.5` 修正（2026-08-14）は現在も
  コードに残っている（`raspi/nav/grid.py`）。シムでは直った実測があるが**実車では未検証**。
  実車で再発するなら、実際の LiDAR ノイズ σ が想定（1cm + 距離の0.5%）より大きい疑いがある
- **oval が渦巻きになる**: 原因は `raspi/auto/raceline.py` の `_lap_ready()` が
  `laps >= explore_laps` のときしか `True` を返さず、**`close_loop()` が最終周にしか
  呼ばれていなかった**こと（`explore_laps=2` だと1周目のループ閉じが永遠に走らない）。
  今回 `_lap_ready()` を `_check_lap()` に分割し `(lap_done, explore_done)` を返すようにして、
  **周回を検出するたびに `close_loop()` を呼ぶ**よう修正（診断済みの入口①、上の「一旦棚上げ」
  節参照）
- ついでに診断済みの②③も対応: `close_loop()` 専用の探索範囲 `LOOP_CLOSE_STAGES`
  （`raspi/nav/slam.py`、毎周期の `RELOC_STAGES` より広く取れる。1周に1回しか呼ばないため）と、
  マッチ補正の異方化（`SlamConfig.match_gain_fwd_ratio`・`_blend_xy()`。進行方向
  （corridor problemで観測できない方向）への補正を弱め、横方向はそのまま寄せる）

単体テストは追加済み（`test_nav.py::TestBlendXY`・`test_raceline.py::...every_lap`）で
全459件green。

**`sim.bench` でも oval・circuit を実測して効果を確認した（2026-08-22）。**
`sim/courses/` に oval 用コースが無かった（`split` 同様、いつからか未コミットのまま
消えていた）ため、直線+180°円弧×2の「角の無いループ」を一時ファイルで作って検証:

| コース | 自己位置（平均/中央/最大） | 地図の正確さ | 周回 | 衝突 |
|---|---|---|---|---|
| oval（角無し・180°円弧×2） | 4.40 / 2.34 / **49.3cm** | **100%**（4cm以内） | 2周 | 8 |
| circuit（既定） | 4.95 / 3.42 / 57.8cm | **100%**（4cm以内） | 2周 | 1 |

修正前（2026-08-14）は oval で「地図が渦巻き・一致度 0.06 で見失う」だった。
今回は **`50.9s [EXPLORE] ... 1周目のループを閉じた（残差 1cm）` とログに出て**、
1周目で正しく閉じ、そのまま2周目も地図が壊れずに完走した。**oval が渦巻きになる
問題はシム上では解消したとみてよい。**

★ 残った疑問点（次に見るべきこと）:
- 両コースとも **BUILD→RACE の切り替え直後に自己位置誤差が一時的に 30〜58cm へ
  跳ね、2〜4秒かけて数cmまで戻る**現象がある。これは今回直した3点とは別物で、
  上の「未解決・要相談」にある**「RACE 段が circuit で開始直後に壁へ接触して
  固着する」**と同じ入口（BUILD→RACE の Pure Pursuit 遅延補償）の疑いが強い。
  今回は「固着」まではせず数秒で回復したが、衝突が oval で8件・circuit で1件出て
  おり、無視できる数ではない
- `split`（分離帯コース）はコースファイル自体が無く未検証のまま
- **これはシム。実車の LiDAR ノイズ・ジャイロ特性はシムより悪い可能性が高く、
  実車でも同じ結果になる保証はない。** `raspi/nav/` は引き続き実車未検証

---

## ★★ 方針：SLAM は一旦棚上げ。非SLAM（反射型）で進める（2026-08-14 決定）

自作 LiDAR SLAM は circuit で自己位置 2.6cm まで来たが、**oval が閉じない**
（1周目のループ閉じが走らない）、**RACE 段が開始直後に壁へ接触して固着する**が
未解決。一方で反射型の **`DisparityExtender`（id `de`）がシムで 2.2 倍速く**、
SLAM が1周もできなかった分離帯コース（split）も周回できた。

**当面は `de` を本線にする。** 詳細な実測値・切り分け経緯は
`docs/progress_archive.md` の「LiDAR SLAM と地図生成を実装した」以降の各節。

- `raspi/nav/`（SLAM 一式）と `raspi/auto/raceline.py` は**消さない。**
  「一旦」であって撤退ではない
- 新しい機能は `de` 側に載せる
- ~~SLAM に戻るときの入口は「周回を検出するたびに `close_loop()` を走らせる」（未着手）~~
  → **2026-08-22 対応済み。** 上の「2026-08-22」節参照。ただし `sim.bench`・実車とも未検証

### 非SLAM で進めるときに無くなるもの／代わりの手

| SLAM があって初めてできること | 非SLAM での代替 |
|---|---|
| レーシングライン（アウトインアウト） | **無い。** 反射型は毎周期その場で決めるだけ |
| 先読みブレーキ（次のコーナーの形） | 見通し（`ext`）で近似。**原理的に劣る** |
| ラップ計測・周回数 | **ジャイロの累積回頭 ÷ 360°** で数えられる（地図不要） |
| 「経路から落ちた」の検出 | 無い。`stop_dist` と超音波 `auto_stop` が最後の砦 |

## ★ 2026-08-21：コードレビュー13項目を全修正し、実機へ反映済み

`docs/review_2026-08-21.md` の指摘を**全項目**片付けた（各項目に「解決済」を追記済み）。
**中身と判断の根拠はレビュー文書側にある。** ここには実機に効く要点だけ書く。

### ★★ 運用が変わった点（これだけは覚えておくこと）

1. **`telemetry_node` の既定バインドが `127.0.0.1` になった。**
   外の PC から GUI を開くには `--host 0.0.0.0` が要る。
   `run_stack.sh` と `install_services.sh` の両方に明示済みなので通常運用は変わらないが、
   **手で `python -m raspi.nodes.telemetry_node` と打つと Pi 自身からしか見えない**
2. **Origin 検査が入った。** 別サイトから `/ws/*` を張ると 403。
   判定は `Host` ヘッダとの一致なので、`surge-mk2.local` でも IP 直打ちでも通る
3. **自律走行の `engage` に操縦権が要る**（解除は従来どおり誰でも通る）
4. **共有トークンは既定で無効。** 使うなら `config/secret.txt`（gitignore 済み）に書き、
   GUI を一度 `?token=…` 付きで開く
5. **検査コマンドが1本になった**: `./tools/check.sh`（`--fast` で生成物だけ）。
   同じものを GitHub Actions（`.github/workflows/ci.yml`）が Python 3.12/3.13 で回す

### 新しく増えた「正」

| 正 | 生成物 | 検査 |
|---|---|---|
| `raspi/msgs/types.py` | `gui/src/generated/msgs.ts` | `config/gen_msgs.py --check` |
| `config/vehicle.toml` の `[safety]` | `gui/src/generated/vehicle.ts` の `SAFETY` | `config/generate.py --check` |
| `raspi/proto/protocol.toml` の版番号 | （文書の見出し） | `config/check_docs.py --check` |

**メッセージ型を GUI に手書きしないこと。** `types.py` を直して `gen_msgs.py` を回す。
デッドマン 150ms も同様で、`vehicle.toml` の `[safety]` が唯一の出どころ。

### 実機で確認したこと（2026-08-21・surge-mk2）

- `pytest` **435 tests OK**（Pi 上・Python 3.13）
- Origin 検査: 同一オリジン/Vite dev/Origin無し = 通す、別サイト/ポート違い/接尾辞偽装 = **403**
- テレメトリの札 `schema=0x667639be` が Pi と手元で一致
- デッドマンが io_node / telemetry_node / GUI の3箇所とも **150ms** で一致
- `_fence()`（seqlock のメモリバリア）実機 Cortex-A76 で **169ns/回**。無視できる
- 再起動後 `health=OK` / `protocol=0x0009` / `sim=False` / `estop_active=False` /
  `hb_stalls=0` / `cmd_rtt_ms≈9.6ms` / TC・TV・片輪浮き対策すべて有効

### ★ JPEG 二重エンコードは「測って、直さないと決めた」

レビュー 🟡7。実測 **3.13ms/枚**（simplejpeg・実機）。二重ぶん（logger の
`image_hz=5` × 2台）は **Pi 5 全体の 0.8%** しかない。camera_node に寄せると
「誰が見ているか」の伝達まで作る必要があり、0.8% では割に合わない。
計測は `status.camera_jpeg` に残してあるので、`image_hz` を上げたり
カメラを増やしたときに再確認できる。

### 副産物：`noUncheckedIndexedAccess` で実バグが1つ出た

`gui/src/ws/map.ts` の `PALETTE` が3要素しかなく、圧縮データが壊れて 2bit 値 `3` が
出ると `PALETTE[3]` が `undefined` になり、分割代入の例外で**地図の描画が止まる**
形だった。4要素にして「壊れた1セルは未知として塗る」ようにした。

---

## 現在地

**★ Phase 0（通信基盤・ログ記録再生・GUI 骨格）完了。実車で走行実績あり。**
Phase 3（自律走行）は非SLAM系（FTG / Disparity Extender）と SLAM 系の両方を
**シムで**試作済みだが、**どちらも実車は未検証**。

以下の表で「詳細」「実測」とある項目の根拠は、同じ見出し・同じ日付で
`docs/progress_archive.md` を検索すれば出てくる。

| | 状態 |
|---|---|
| 設計フェーズ | 完了 |
| UART プロトコル | **v0.9 まで実車確認済み**（2026-08-20。v0.4確定→v0.5トルク直接指令の下地→v0.6トルク直接指令→v0.7超音波auto_stop→v0.8 TC/TV有効切替→v0.9片輪浮き対策。経緯はアーカイブ） |
| プロトコル実装（Python + C ヘッダ生成） | **完了** |
| **Pi 実機セットアップ** | **完了**（SSH・リポジトリ配置・venv・UART 有効化） |
| **STM32 実機との UART 疎通** | **確認済み**（双方向・エラー0） |
| **io_node 中核（シリアル+時刻同期）** | **実機で安定動作**（loss=0/crc=0） |
| **生フレームログ `.sfl` 記録＋解析ツール／`replay_node`** | **実機で記録・再生・完全一致まで確認済み** |
| **★ 2D シミュレータ（Mac 専用・`sim/`）** | **完了**（`io_node --sim` で実機と同じ制御コードのまま走る） |
| **★ 自動運転（FTG・SLAM+raceline・Disparity Extender）** | **シムで確認済み・実車未検証**（下記「未解決」） |
| **Disparity Pursuit（id `dp`）新設**（DEの段差塗り＋狙点選択 と FTGのPure Pursuit舵角を統合し、①狙点のヒステリシス ②曲率ベース速度上限 ③TTCによる早期停止 を追加。`raspi/auto/gap_pursuit.py`） | **実装完了・単体テスト58件green（2026-08-18）。シム・実車とも未検証** |
| **GPIO E-Stop ハートビート＋端から端までの動作確認** | **実機で合格**（`--allow-arm` 経路も含め2026-08-10に合格） |
| **カメラの実力測定／`camera_node` + 共有メモリリング** | **実機で動作確認済み**（2台30fps・ゼロコピー） |
| **LED 表示（緑=生存と可動性 / 赤=異常の重さ）** | **実機で目視確認済み** |
| **arm を通す最小実験** | **合格**（駆動電源維持・トルク0を確認） |
| **メッセージ型 + ZeroMQ バス／WS サーバ／GUI 運転ビュー** | **実機で端から端まで合格**（実映像・実点群含む） |
| **★ 実車を GUI（キーボード）で操縦** | **成功**（Phase 0 の達成条件の片方） |
| **操縦レスポンス（rAF+50Hz・ブレーキ・発進キック）** | **実車で確認済み** |
| **`tools/deploy.sh`／systemd 自動起動** | **動作確認済み**（電源投入20秒で全部復帰。arm 有効） |
| **mDNS（`surge-mk2.local`）運用** | **確立**（下記「Pi 実機」） |
| ~~AP モード~~ | **実装しない方針に確定**（下記） |
| **MCAP 記録（`logger_node`）／記録はPC側で受ける／GUI「ログ」タブ** | **実機で合格・反映済み** |
| **GUI 5タブ再編（ラジコン/自動運転/地図生成/診断/ログ）** | **完了** |
| **`docs/system_overview.md`・`docs/development.md` 新設** | **完了**（2026-08-16。文書と実装のズレを棚卸し。一覧は `development.md` §11） |
| Pi 純正ファン | **正常**（設定不要） |
| **起動音・GUI接続音（ブザー）** | **実機で合格**（2026-08-17。下記「GPIO18 はSTM32と共有」参照。STM32のブザー音より音質はやや粗いが許容） |
| **GUI ラジコンビュー GR風再設計**（黒基調+赤、LiDARをゲーム風ミニマップ化、映像:車体図4:1、後方PIP/LiDARの拡縮＆一括表示切替、後輪ゲージをタイヤ内へ統合、温度計5つ追加） | **実装完了・ブラウザ確認済み（2026-08-17）。実車のカメラ/LiDAR実データでの見え方は未確認** |
| **RasPi CPU温度の配信**（`telemetry_node._read_pi_temp_c()` が `/sys/class/thermal/thermal_zone0/temp` を読み `pi_temp_c` として `/ws/telemetry` に追加。GUIの温度計で表示） | **実装完了（2026-08-17）。Mac上では常にnullを返すことのみ確認済み。実車での実測値は未確認** |
| **GUI ラジコンビュー 追加調整**（舵角メータを速度計の左に追加・速度計を中央固定、速度計を-120〜120°/真上0°に、映像とメータ行を実測ベースで都度最大化——`useContainFit`/`useElementSize`。svg/canvas子要素の intrinsic サイズに負けてCSSのaspect-ratioだけでは縮む罠を2回踏んだ） | **実装完了・複数ウィンドウサイズでブラウザ確認済み（2026-08-17）。⚠ 後方PIPとLiDARミニマップを両方82%まで拡大すると重なる場合がある（未対応、実使用で気になれば要調整）** |

## Pi 実機（surge-mk2）

接続情報の詳細と機密は `docs/setup_credentials.md`（gitignore 済み）。要点だけ:

- **Raspberry Pi 5 / Debian 13 (Trixie) / Python 3.13.5**
- **接続は Ethernet 直結**。Mac の USB アダプタ ↔ Pi。**Pi eth0 = 固定 `169.254.55.2`**。
  `ssh surge-mk2` で決め打ち接続（`~/.ssh/config` 設定済み、鍵は `~/.ssh/id_surge_mk2` でパスフレーズ無し。
  `sshpass` は今は不要）
- **UART**: `/dev/serial0 -> ttyAMA0`。io_node では `/dev/serial0` を使う（名前直書き禁止）
- リポジトリは **Mac → Pi へ rsync**（`~/surge_mk2`）。git remote はあるが未 push
- venv は `~/surge_mk2/.venv`（`--system-site-packages`）
- **電源は 5V/5A（27W）USB-C PD 必須**（USB給電では不足してハングする実績あり。解決済みだが忘れないこと）

### よく使うコマンド

```bash
rsync -az --exclude __pycache__ --exclude '*.pyc' --exclude .venv --exclude docs/setup_credentials.md \
  surge_mk2/ surge-mk2:~/surge_mk2/
ssh surge-mk2 'cd ~/surge_mk2 && .venv/bin/python -m unittest discover -s raspi/tests -t .'

# 反映（毎回rsyncを手打ちしていたのをスクリプト化。何を再起動すべきかはスクリプト冒頭の表を見る）
tools/deploy.sh                # GUI をビルドして rsync（既定・いちばん安全）
tools/deploy.sh --no-gui       # Python だけ直したとき
tools/deploy.sh --services     # systemd unit を入れ直す（--max-speed 等を変えたとき）
tools/deploy.sh --restart      # surge-telemetry / surge-camera を再起動（E-Stop 無関係）
tools/deploy.sh --restart-io   # ★ surge-io を再起動（E-Stop がラッチする。車体のそばで実行）
```

### mDNS（`surge-mk2.local`）— IP を探す作業は不要

`surge-mk2.local` が直結・Wi-Fi のどちらでも生きている方を指す（`ssh surge-mk2` /
`http://surge-mk2.local:8000/` が常に通る）。運用上ハマりやすい点:

- **`~/.ssh/config` に `AddressFamily inet` が必須。** 無いと直結があるのに IPv6 リンクローカル
  （Wi-Fi 経由）へ迷い込む。curl は `-4`
- **IF を落として戻すと avahi が再広告しないことがある。**
  「直結が生きているのに遅い」ときは `ssh surge-mk2 'sudo systemctl restart avahi-daemon'`
- **素の `ping 169.254.55.2` は直結が生きていても失敗しうる。**（169.254/16 のルート重複のため）
  直結の生死は **`ping -b <IF名> 169.254.55.2`** で確認（IF名は挿し直すと変わる）
- mDNS が使えない環境では MAC のベンダ prefix `2c:cf:67`（Raspberry Pi Ltd）で `arp -an` を grep

### ⚠ 原因不明の再起動（2026-08-07・2回・未解決）

作業中に Pi が勝手に落ちて再起動していた事案が2回ある。電源・熱は異常なし（電源不足問題とは別。
電源不足自体は 5V/5A PD への交換で解決済み）。**原因は未特定のまま。**
永続ジャーナル（`sudo journalctl --list-boots` / `journalctl -b -1`）を有効化済みなので、
**再発したら必ずログを見て原因を追うこと。** 走行中に落ちるとハートビートが止まって
E-Stop になる（フェイルセーフではある）が、信頼性の問題として残っている。

### ★ 屋外に出る前に、研究室で SSID を登録しておくこと（未実施）

NetworkManager は知らない SSID に自動接続しない。**現地で登録しようとすると、
Pi に繋ぐ手段が直結 Ethernet しか無い状態になる。**

```bash
ssh surge-mk2 'sudo nmcli device wifi connect "SURGE-FIELD" password "********" ifname wlan0'
ssh surge-mk2 'sudo nmcli connection modify "SURGE-FIELD" connection.autoconnect-priority 20'
ssh surge-mk2 'sudo nmcli connection modify "tplink"      connection.autoconnect-priority 10'
```

---

## 決まっていること

### UART プロトコル — 現在 v0.11（`docs/uart_protocol.md` が正）

`raspi/proto/protocol.toml` が実装上の唯一の定義で、そこから Python パーサと STM32 側 C
ヘッダを生成する（`python3 raspi/proto/generate.py`）。v0.4確定 →
v0.5（`COMMAND` LEN10→12・ブレーキ/灯火/ホーン拡張）→ v0.6（トルク直接指令、上限0.125→0.15N·m）→
v0.7（超音波 `auto_stop`、ワイヤ非破壊）→ v0.8（TC/TV 有効切替、`param_id 0x0010`/`0x0020`）→
v0.9（片輪浮き対策、`param_id 0x0050`。TC本体とは独立、2026-08-20）→
v0.10（`MAX_SPEED`/`MAX_ACCEL`/`MAX_STEER` の `param_id 0x0001-3` を廃止し STM32 側固定定数へ
一本化）→ **v0.11（`LIMITS`(0x0A)/`LIMITS_REQ`(0x15) を新設。STM32側発、2026-08-21）**と進んだ。
**v0.8・v0.9 は実機で動作確認済み。v0.10・v0.11 は STM32側実装済みだが実機での動作検証は
まだ（STM32側ドキュメント曰く「実機での動作検証は未了」）。**
各版の変更点・実車確認の経緯はアーカイブの該当節（`UART プロトコル v0.6` / `v0.7` / `v0.5 に追従した` など）。

#### ★v0.11 `LIMITS`（車両の物理的な上限値。2026-08-21、Pi側実装済み・未コミット）

`LIMITS`（STM32→Pi、16B、`max_speed_m_s`/`max_accel_m_s2`/`max_torque_nm`/`max_steer_rad`
のf32×4・読み取り専用）を新設。v0.10でPiからパラメータの動的変更手段が失われた代わりに、
**STM32側の実際の固定上限値をPiが起動時に知れるようにする**のが目的。

- Pi側の対応（`raspi/nodes/io_node.py`）: `handshake()` で `VERSION_REQ` と同時に
  `LIMITS_REQ` を送る。**`_send_command()` が毎回 `state.limits` を読み、RC(MANUAL)・
  AUTO(自律走行) 問わず `--max-speed`/`--max-steer`（Pi側設定）と `LIMITS`（STM32実測）の
  小さい方でクランプする**（`raspi/msgs/convert.py` の `command_from_cmd` に
  `max_accel`/`max_torque` 引数を追加）。全指令がこの1箇所を通るので送信元を問わず効く
- **バグを1つ踏んで直した**: 最初の実装は `handshake()` の1秒タイムアウト内に `LIMITS` が
  届かなければ**二度と再送しない**ままだった（バンビが実機で「STM32から送っているはずの
  上限が適用されない」「同時起動しないと受け取れない」と報告して発覚）。
  `run()` のメインループで `state.limits is None` の間 1秒おきに `LIMITS_REQ` を再送する
  よう修正済み
- **GUIへの反映も追加実装した**（当初「多分ほぼ変更ないと思う」と見積もっていたが、
  バンビの「GUIの上限が3/5.5のままでSTM32実測(5/3)が反映されていない」という指摘で
  スコープに入れた）。`LinkDiag`（`raspi/msgs/types.py`）に `max_speed_m_s` 等を追加し、
  `bus_bridge.py` の `build_diag()` で伝搬
- **★2026-08-22 に「STMからの値を優先してほしい」で再修正**: 最初は
  `Math.min(静的既定, STM実測)` にしていたが、これだと STM32 実測の方が大きい場合
  （3.0固定→実際は5.0出せる、等）に古い静的値で頭打ちになったまま。
  `gui/src/store/ui.ts` の `effectiveRange()` を「**`LIMITS` 受信済みなら無条件にそれを
  使う**（静的値は WS 接続前のプレースホルダでしかない）」に変更。
  **`sim/stm32.py` にも `LIMITS`/`LIMITS_REQ` を実装した**（`_emit_limits`。実機と同じ
  `DRIVE_MAX_SPEED_M_S=5.0`/`DRIVE_MAX_ACCEL_M_S2=3.0`/`DRIVE_MAX_TORQUE_NM=0.15`/
  `spec.max_steer` を返す）ので、シムでも実機と同じ経路で動く
- **バンビに確認 → 「LIMITSを優先に変える」で回答済み（2026-08-22）。**
  `io_node.py` の `_send_command` も `min(--max-speed, LIMITS)` から「**`LIMITS` 受信済み
  なら無条件にそちらを使う**」に変更した（`--max-speed`/`--max-steer` は未受信時だけの
  フォールバック）。GUI・Pi 両方で「STM32実測を優先」に統一済み。
  `raspi/tests/test_cmd_path.py` に往復方向（LIMITSがより緩い場合／より厳しい場合）
  それぞれのテストを追加
- **★はまりどころ**: `LIMITS` は `ControlStatus`（`/ws/control` の status。イベント発生時
  にしかbroadcastされない）ではなく **`LinkDiag`（`/ws/telemetry` 8Hz、`bus/live.ts` の
  `useNumbers()`）から読むこと。** `tc_enabled`/`wheel_lift_guard_enabled` と同じ理由
  （2026-08-19に実機で発覚した「statusから読むと更新されずリロードするまで気づかない」
  バグを避けるため）。一度 `ControlStatus` 側に実装してから気づいてリバートした
- **★★実機で踏んだ本命バグ（2026-08-22）**: 実機で `--restart-io` までして確認したのに
  「最高速度スライダーが3で止まる」（診断タブの LIMITS 表示は正しく 5.00m/s）。
  原因は `SettingsPanel.tsx` ではなく **`store/ui.ts` の `setSettings` アクション**。
  `clampSettings()` が `effectiveRange()`（動的）ではなく**静的な `SETTINGS_RANGE`
  （3.0固定）でクランプしていた**ため、スライダの DOM `max` 自体は動的に5.0まで動くのに、
  `onChange` → `setSettings()` が呼ばれた瞬間に静的値へ巻き戻されていた
  （見た目は動くのにドラッグすると弾かれる、という分かりにくい壊れ方）。
  `clampSettings()` を `effectiveRange(live.link)` でクランプするよう修正
  （`live` は `bus/live.ts` の素の可変オブジェクト。フックではないので `store/ui.ts`
  という非コンポーネントのコードからでも直接読める。**これが今回のようにクランプ関数を
  複数箇所に持つ設計の落とし穴** — 表示側だけ直して、保存側の別の静的クランプを
  見落とすと再現する）
- 診断タブに「車両上限（LIMITS）」の生値表示を追加済み（`DiagGrid.tsx`）。
  今後同種の不具合は、まずここで Pi が実際に受け取っている値を確認してから
  GUI 側のバグを疑うと切り分けが速い
- **★速度メータの目盛りを `settings.maxSpeed` から切り離した（2026-08-22、バンビ指示）**。
  それまで `SpeedGauge.tsx`（RC タブの主計器）は速度制御・トルク制御どちらでも
  `settings.maxSpeed`（RC の速度ダイヤル、ユーザーが下げられる）を目盛りの満尺に
  流用していたため、ダイヤルを下げるとメータの目盛りまで一緒に縮み「実際に出せる
  速度」と食い違っていた。`SpeedGauge.tsx` の `full` を常に `LIMITS.max_speed_m_s`
  （`n.link?.max_speed_m_s`、未受信時のみ `PI_MAX_SPEED_CAP` にフォールバック）に固定。
  この役割が要らなくなったので `SettingsPanel.tsx` の `TORQUE_FIELDS` から
  「速度メータの目盛り」フィールド（`key: 'maxSpeed'` の重複エントリ）を削除。
  `DEFAULT_SETTINGS.maxSpeed` も 0.6 → **3.0**（バンビがそれまで手で設定していた値）
  に更新。`maxSpeed` は今は「速度制御モードの実際の RC 上限」のみを表す
- **★「現在の値を既定値として保存」ボタンを追加（2026-08-22、バンビ指示）**。
  上の `DEFAULT_SETTINGS.maxSpeed` 更新は Claude がスクリーンショットの値を見て
  ソースコードに決め打ちで書いただけで、**実機の現在値を確認したわけではなかった**
  （バンビに指摘されて発覚）。今後同じ往復をしないための恒久対応:
  - `store/ui.ts` に `settingsDefault`（`localStorage` キー
    `surge.driveSettingsDefault.v1`）を新設。「既定値に戻す」・各行の「既定値 X」表示は
    今後ソースコードの `DEFAULT_SETTINGS`（工場出荷値）ではなく、この値を指す
  - `saveCurrentAsDefault()` アクションで `settings` の現在値をここへコピーできる。
    設定パネル上部「既定値に戻す」の隣に「現在の値を既定値として保存」ボタンを追加
  - `DEFAULT_SETTINGS` 自体は据え置き（壊れた localStorage から復旧するときの
    最終フォールバックとして `clampSettings`/`loadSettings`/`loadDefaultSettings` が使う）
- GUIの型は自動生成なので変更したら **`python3 config/gen_msgs.py`** を忘れずに
  （`gui/src/generated/msgs.ts` の `MSGS_SCHEMA` が変わる。App.tsx が食い違いを検出する）
- **まだコミットしていない。** 実機テストで確認してから commit する予定

確定している主要仕様:
- `TELEMETRY` LEN = 66。`STATS` は累積 u32。`CONFIG_SET` は揮発（Flash はステア原点とIMUキャリブのみ）
- `COMMAND` 途絶 100ms で自動ブレーキ / `TELEMETRY` 途絶は 100ms 警告・200ms FAULT
- **`COMMAND` の DISARM 固定は `--allow-arm` で外せる（既定は今も DISARM 固定）**

### 知覚・自己位置推定の方式

2026-08-13〜14 に自作 SLAM（LiDAR主体・スキャンマッチング＋占有格子）と
Disparity Extender（非SLAM反射型）の両方をシムで試作した。**現状の方針は上記の通り
非SLAMを当面の本線とする**が、Phase 3 の最終選択としては確定していない。
議論の背景（オドメトリ単独のデッドレコニングが成立しない理由など）は
`docs/architecture.md` §15-1 とアーカイブ「LiDAR SLAM と地図生成を実装した」節。

### 前輪オドメトリの扱い（どの方式でも共通）

エンコーダは**前輪 = 操舵輪**に付いている。`odom_dist` / `wheel_speed[FL,FR]` は
車輪自身の軌跡に沿った値で、車体中心線方向ではない。**射影は累積値ではなく差分に対して
行う**（`cos(δ)` を累積距離に掛けるのは誤り）。射影済みなのは `speed` だけ（STM32側で処理）。
また **`wheel_speed[FL,FR]` は低速で使い物にならない**（静止時に±12m/s級のスパイクが
7割超のサンプルに出る実測あり。静止判定・スリップ検出には `speed` か `odom_dist` の差分を使う）。

### ★ GPIO18（ブザー）は STM32 と1本の線を共有している（2026-08-17 判明）

`raspi/io/gpio.py` の `Buzzer`（起動音・GUI接続音・E-Stop/FAULTビープ）は GPIO18 を使うが、
**この線は STM32 側のブザー出力と隔離なしで直結**されている（ダイオード等の分離回路は無い）。
両者が同時に押し出す（プッシュプル）と信号が衝突し、RasPi 側からは音が出ない。

- **STM32 ファームウェア側で「鳴らしていない間は Hi-Z（入力）にする」実装済み**（2026-08-17、
  実機で確認・解決）。このリポジトリの管轄外（STM32 ファームウェアは別プロジェクト）なので、
  **STM32 側のこの実装を壊すとまた無音に戻る**。触る人に必ず伝えること
- 音質は STM32 自身のブザー音より粗い（パチパチ・ジリジリしたノイズが乗る）。
  `Buzzer.SETTLE_S`（起動直後の初回発音前に 0.15s 待つ）で軽減したが根絶はしていない。
  共有線・ソフトウェア PWM（gpiozero/lgpio、RP1のハードウェアPWMではない）の両方が疑わしいが未特定
- 今後 STM32 と Pi が同時に鳴らそうとする状況（両方の起動タイミングが重なる等）は
  未検証。気になる症状が出たら真っ先にここを疑うこと

### ネットワーク — Pi は常に STA。AP モードは実装しない（2026-08-08 決定）

**屋外では Wi-Fi ルーターを持ち込み、Pi と PC の両方をそれに STA 接続する。**
モード切替という操作自体を無くした。理由（電波法・切断安全性・NTPなど5点）と
詳細な構成表はアーカイブおよび `docs/architecture.md` §9.1。

| 場面 | 構成 |
|---|---|
| 据置開発 | 有線 Ethernet 直結（`169.254.55.2`） |
| 研究室での走行テスト | STA（`tplink`） |
| **屋外・大会** | **STA（持込ルーター）** |
| ルーターを忘れた/壊れた | **Ethernet 直結**（AP モードではない） |

---

## ★ 未解決・要相談（新しいセッションで最初に確認すること）

### ハードウェア / STM32 側と要相談

- **`steer_actual` が指令 0° で −24.0° に張り付く（未解決）。**
  「遅れ」では説明できない定常オフセット。原点ずれか、Pi/STM32間の舵角符号規約の
  食い違いを疑っている。**Phase 1 のアクチュエータ遅延実測（`steer_cmd_echo` と
  `steer_actual` の差を測る）の前に必ず潰すこと。** 過去記録では逆符号（+22.4°）の
  張り付きも観測されており、単発の配線ではなく構造的な疑いがある
- **STM32 の時刻が +3378ppm 速い（新発見・要相談）。** HSI(内蔵RC, ±1%級)で動いている
  可能性が高い。`TimeSync` の回帰補正で実害は吸収できているが、STM32側に確認する価値あり
- **MD バスの CRC エラー率が3台とも 23〜25%（新発見・要相談）。** Pi⇄STM32 は無傷（crc=0）
  なので STM32⇄MD 側の共通要因（終端・GND・ボーレート等）を疑うべき。
  `comm_ok` のちらつき・GUIグレーアウト表示の原因もこれ。**この率が下がらないと
  `comm_ok=0` → FAULT 遷移の実装（`uart_protocol.md` §5.5 で規定・未実装）を入れられない**
- **原因不明の再起動が2026-08-07に2回（未解決）。** 上記「Pi 実機」節参照。再発時はログ必須

### 自動運転（シムのみで検証・実車一式が未検証）

- **FTG / Disparity Extender / SLAM+raceline / Disparity Pursuit とも実車未検証。** 車輪を浮かせて
  `auto/state` の `reason` と選ぶギャップ／自己位置を先に確認し、`max_speed` を
  0.2m/s 程度まで落として床に降ろす、という段階を踏むこと
- **Disparity Pursuit（`dp`）の `a_lat_max`（既定3.0 m/s²）・`ttc_min`（既定0.6s）は
  未計測の暫定値。** まずシムで衝突なしを確認し、実車で横滑り・急制動の体感から詰めること
- **RACE 段が circuit で開始2秒後に壁へ接触して固着する（SLAM側・未修正）。**
  BUILD（停止）→RACE の受け渡しで Pure Pursuit の遅延補償が怪しいところまで切り分け済み
- **oval が「渦巻き」になる＝ループ閉じが1周目に走らない（SLAM側）。**
  診断済みの3点（① 周回検出のたびに `close_loop()` を走らせる ② 探索範囲を専用に
  広げる ③ マッチの寄せ方を異方的にする）は **2026-08-22 に実装した**（上の
  「2026-08-22」節）。**`sim.bench`・実車とも未検証のまま**なので、oval で本当に
  直っているかはまだ確認できていない。直っていなければ `explore_laps=1` が回避策
- **split（分離帯コース）は raceline（SLAM）では通らない。** Disparity Extender では
  解決済み（周回できる）だが、SLAM側の自己位置は縦方向の観測が原理的に弱く未解決

### 実装の穴（小さいが未修正）

- `install_services.sh --remove` が `surge-logclean.timer` を消し損ねる
- `tools/deploy.sh --restart` に `surge-planning` が入っていない（`raspi/auto/` `raspi/nav/`
  `planning_node.py` を直したら手で再起動が必要）
- `telemetry_node._serve_log_file` が `.mcap` を全部メモリに載せる
- `LinkTracker` が STM32 の `LOG`(0x04) を数えるだけでどこにも出していない
- `logger_node` の記録対象に `auto/*` が無く、自律走行の判断根拠が `.mcap` に残らない
- テスト実行中に `telemetry_node.py` の `AttributeError: '_auto_mode'` が印字される
  （テスト414件は全部通るが、未初期化属性の疑いとして残っている）
- GUI のカメラが前後とも 14fps（`camera_node` は実機で2台30fps確認済みなので配信側で
  落ちている疑い。切り分け手順はアーカイブ「v0.5 を実車で確認した」節）

### 未実測・未着手（クリティカルパス）

- **`config/vehicle.toml` は `[dynamics]`（アクチュエータの動特性）を除き全項目を実測確定**
  （2026-08-20）。ホイールベース L=0.23m・トレッド=0.155m・車輪半径 0.03m・質量 2.0kg・
  車体外形・センサ取付位置・ステアリングのリンク比（0.5、路面舵角上限 ±30°=0.524 rad）。
  **後輪はダイレクトドライブでギア比の概念が無い**ため、`docs/architecture.md` §15 #2〜#4 はこれで確定
  （`raspi/` 各種 / GUI `PI_MAX_STEER_CAP` / `sim/` に反映済み）。**車輪半径・リンク比は
  STM32 ファームウェアの換算定数にも反映済み**（本人確認）。`speed`/`wheel_speed`/
  `odom_dist`/`steer_actual` はもう暫定スケールではない。残るクリティカルパスは
  `[dynamics]` の実測のみ
- **GUI の車体寸法定数を `config/vehicle.toml` から生成する仕組みを追加**（`config/generate.py`
  → `gui/src/generated/vehicle.ts`。`raspi/proto/generate.py` と同じ考え方）。
  toml を直したら手で GUI 側を書き換えなくてよくなった。編集後は再生成を忘れずに
- 屋外用ルーターの SSID が Pi に未登録（上記「Pi 実機」節）
- `perception_node` / `safety_node` は未実装（`architecture.md` の設計と実装のズレ。
  一覧は `docs/development.md` §11）

---

## 次にやること（優先度順）

1. **`steer_actual` の −24° 張り付きの原因特定**（原点ずれ or 符号規約。他の何より先）
2. **壁に穴が残る根本原因の対処**（`_END_BACKOFF` の再チューニング or 実車 LiDAR
   ノイズの実測。2026-08-23 の中心線の頑健化は対症療法でしかない。上の
   「2026-08-23」節参照）
3. **中心線の急カーブ耐性のさらなる改善**（`_MAX_SHIFT_M` はテレポートを
   防ぐだけで、シケイン内の測定精度そのものは上がらない。`fuji` コースで
   1〜2点の残留を実測済み。法線推定の頑健化 or EDT ベースへの置き換えが
   次の一手。上の「2026-08-23」節参照）
4. **BUILD→RACE 切り替え直後の一時的な自己位置誤差（30〜58cm）の原因調査**
   （2026-08-22 の `sim.bench` で oval・circuit 両方に出た。「RACE 段が壁へ接触
   して固着する」と同じ Pure Pursuit 遅延補償が疑わしい。oval/circuit は
   sim.bench で確認済みなので残るは split の再実測とこれ）
5. **Disparity Extender を車輪を浮かせて実車検証**（`--max-speed 0.2` 程度から）
6. GUI カメラ 14fps の切り分け
7. `[dynamics]`（操舵のむだ時間・1次遅れなど）の実測
8. 屋外用ルーターの SSID を研究室で登録

SLAM 側の課題（RACE固着・oval渦巻き・split未解決）は方針により**棚上げ中**。
再開する入口は「周回検出のたびに `close_loop()` を走らせる」変更（上記）。

---

## `docs/progress_archive.md` の節一覧（時系列）

見出しをそのまま検索語にすれば該当節に飛べる。主なもの:

- Pi 実機（接続トラブルの詳細・電源不足の原因究明）
- STM32 実機との疎通（2026-08-07）
- io_node 中核（実装方針・実証結果）
- ★★ 駆動電源が落ちる件（原因判明の全経緯）
- ★ arm を通す最小実験（`wheel_speed` 実測データ含む）
- ログ記録 `.sfl` / replay_node（形式・実証・`--verify`）
- GPIO E-Stop ハートビート（実測値・STM32側仕様・estop_test.py 詳細）
- カメラの実力測定 / camera_node と共有メモリリング（実測表・踏んだ罠）
- 内部バス・WS・GUI（ZeroMQ の罠・走行指令の安全設計）
- 実機でバスと GUI を通した／GUI から arm を通した／実車をキーボードで走らせた
- MCAP 記録を実装した／実機検証（`RemoveIPC=yes` の罠など）
- 操縦レスポンスの作り直し（もっさりの原因4つの実測）
- プロトコル v0.5 / v0.6 / v0.7 に追従した（変更内容・実車確認の詳細）
- GUI を4タブ・5タブに再編した経緯（レイアウト改訂の試行錯誤）
- Mac 上で動く 2D シミュレータを作った（設計判断・踏んだ罠・検証結果表）
- Follow the Gap による自動運転を実装した
- LiDAR SLAM と地図生成を実装した（BreezySLAM不採用の理由・実装中に踏んだ罠6つ）
- 地図生成アルゴリズムの見直し／スキャン対スキャン／ループ閉じ関連の一連の修正
- ★★ Disparity Extender を実装した（設計上の誤り3つ・実測比較表）
- 壁が虫食いになる件／oval が渦巻きになる件の切り分け
- 全体像と開発ガイドの文書を新設した（2026-08-16・文書と実装のズレの棚卸し）

---

詳細な経緯・実測値・トラブルシューティングは `docs/progress_archive.md` を参照。
