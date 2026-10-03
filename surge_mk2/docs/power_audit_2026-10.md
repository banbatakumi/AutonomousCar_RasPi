# 制御系バッテリー 省電力化調査（2026-10-04）

制御系バッテリーが駆動系より早く減る問題について、**プログラムで削れる電力**を調べた。
過去の電力測定記録は使わず、すべて測り直している。

## 1. 計測方法

- **電流値**: STM32 のテレメトリに入っている `vehicle_state.batt_current[1]` を使う。制御系の入力電流で、分解能は 20mA。
  - 電源は安定化電源 11V。100Hz で約4000点を取り、40秒間の平均をとった。標準誤差は 0.3〜0.5mA。
- **比較の仕方**: 基準 → 施策 → 基準（A-B-A）の順に測り、前後2回の基準の平均との差を効果とした。
  - 基準は時間とともに 628〜654mA の範囲でゆっくり動く（温度・ファンの影響）。このため、**±5mA 未満の差は効果とみなさない**。
- **測定条件**: DISARM・停車中、GUI は未接続、全 surge サービスが起動した状態。
  - 走行時の負荷は、カメラの seg モデル（`models/v2.onnx`、224×224）を 10Hz と 30Hz で回して模擬した。
- **基準値**: **約 640mA（7.0W）**。io_node 以外のサービスを全部止めると **509mA**。

## 2. 省電力化候補（Raspberry Pi 5）

効果は 11V 側の電流（mA）。−が削減。★は推奨度。

| # | 施策 | 実測効果 | 実装 | リスク・トレードオフ | 推奨 |
|---|---|---|---|---|---|
| P1 | **DISARM 中は CPU の上限周波数を 1.5GHz に落とす**（`scaling_max_freq`、ARM したら 2.4GHz に戻す） | **−56mA**（待機時）。1.8GHz に下げても −3mA しか変わらない。1.5GHz だけが電圧の底にあたるらしい | 小（telemetry か io が armed の変化を見て sysfs に書く。sudoers か udev で書き込み権限を与える） | 待機中に重い処理が少し遅くなる（GUI の地図読み込みなど）。走行には影響しない | ★★★ |
| P2 | **e2e_lidar の ORT セッションに SessionOptions を付ける**（intra=2、spin 無効。cam_perception と同じ設定） | **−98mA**（e2e_lidar の 10Hz 推論時。既定設定だと +100mA・CPU 31%、設定後は +3mA）。推論時間は 0.32ms→0.31ms で悪化なし | 極小（[e2e_lidar.py:78](../raspi/auto/e2e_lidar.py#L78)） | ほぼ無し | ★★★ |
| P3 | **DISARM 中は後方カメラを止める**（今の `config/camera.json` は `rear_enabled_disarm: true`。コードの既定は false） | **−39mA** | 無し（GUI の設定を変えるだけ） | 駐車中に後方映像が見られない | ★★★ |
| P4 | **カメラが要らないときは前方カメラも止める**（capture を `stop()` する。条件は、GUI の視聴者 0・カメラ系モードでない・logger 停止中） | 前方 10→5fps で −8mA、1fps で −15mA、**止めると −63mA**（カメラ系4ノード分を除いた値）。センサが動いているだけで約 45mA かかる | 中（`request_enabled` を前方にも適用し、再開時の遅延に対処する） | 再開に数百 ms かかる。ARM の直後にカメラ系モードへ入る場合は、先に起動しておく必要がある | ★★ |
| P5 | **走行中の CPU 上限を 2.0GHz にする** | seg 30Hz の負荷で **−62mA**（1008→946）。推論時間の中央値は 26.2→28.4ms（+8%）。1.8GHz なら −106mA だが p95 が 34.5ms で 33ms を超え、1.6GHz ではフレームを落とす | 小（P1 と同じ仕組み） | 推論の遅延が +2ms 増える。SLAM など他の処理が重なったときの余裕は実走で確認が必要 | ★★ |
| P6 | **カメラ系ノードが IDLE 中に空のメッセージを publish し続けるのをやめる**（状態が変わったときと 1〜2Hz の生存通知だけにし、vehicle_state を 100Hz で受けて起きる作りもやめる） | 4ノード停止で −19mA（上限値）。1ノードあたり 4〜6mA | 中（4ノード＋planning の受信側） | 生存の判定を 1〜2Hz の通知に合わせて直す必要あり | ★★ |
| P7 | **planning の `poll(2)` を、次に送る時刻までの残り時間で待つ形にする** | 停止で −12mA（上限値） | 小 | `auto/cmd` を 50Hz で送るタイミングを保つ必要あり | ★★ |
| P8 | **ファンの kick 不具合を直す**（バグ B1） | 約 −7mA。毎秒のファン起動とその音も無くなる | 小 | 元々は governor が止まる現象への対処。直し方次第で再発の恐れ | ★★★ |
| P9 | **telemetry の `_bus_pump` を 1kHz のポーリングから zmq の fd 待ち（`loop.add_reader`）に替える** | 1kHz の空ループを単独で走らせると +5mA | 中 | `auto/cmd` の即時中継の遅延を測り直す必要あり（制御経路なので慎重に） | ★ |
| P10 | **`_wifi_pump` の nmcli を、GUI 接続中だけ・10秒周期にする** | 毎秒の nmcli は +3mA | 極小 | 無し | ★ |
| P11 | telemetry の残りの消費の内訳を調べる | telemetry を止めると −49mA。上の P8・P9・P10 を足しても約 15mA にしかならず、**残り約 34mA は原因不明** | 調査（py-spy で計測） | — | ★★（次の調査） |
| P12 | governor を ondemand から schedutil に替える | 待機時 −42mA、seg 10Hz で −86mA。ただし推論時間が 26.7→35.1ms（+31%）に延び、seg 30Hz では電流が逆に増えた（温度の影響と区別できていない） | 極小 | 遅延が大きく増えるので P1＋P5 の方が良い | ☆（不採用） |
| P13 | `dtoverlay=disable-bt`、`dtparam=audio=off`、HDMI 系の無効化、`apt-daily` / `man-db` タイマーの停止 | **未測定**（再起動が必要なため見送った）。BT は HCI が既に DOWN なので、効果は小さいと見込む | 極小 | 再起動が必要。BT や音を使う予定があれば外せない | ☆（余裕があれば） |

- Wi-Fi の power_save は既に有効（dmesg で確認）なので、手を入れる余地は無し。
- **待機時に削れる量の見積もり**: P1＋P3＋P4＋P6〜P10 で **約 180mA（640→約 460mA、−28%）**。
  - 「io 以外を全停止すると −121mA」「P1 で −56mA」とつじつまは合う。
  - 実際は互いに重なる部分があるので、施策を入れた後で測り直す必要がある。
- **走行時**: e2e_lidar モードなら P2 で −98mA。カメラ系モードなら P5 で −62mA。

## 3. 省電力化候補（STM32F446RE）

STM32 には書き込んでいない。根拠はコード（`FW/MainBoard_V3` ブランチ 2ec1add）。
11V 側の分解能 20mA では差が見えないものが多く、効果は推定値。

| # | 施策 | 推定効果 | 実装 | リスク | 推奨 |
|---|---|---|---|---|---|
| S1 | **failsafe 中（Pi 未接続・通信途絶・E-stop）はブレーキ灯を 100% から尾灯レベルか点滅にする**（`vehicle.c:280` `ApplyFailsafe` → `ApplyBrakeLight(obj, true)`） | ライトの電流しだい（回路を見て確認してほしい）。**Pi を落として制御電源だけ入っている間は、ずっと 100% 点灯している** | 極小 | 停止中であることの表示が弱くなる | ★★★ |
| S2 | **超音波のトリガ停止を ARM の状態で判定する**（今は `Drive_IsEnabled` で判定しているため、failsafe 中は出しっぱなし。バグ M2） | 超音波2個のトリガ分。数 mA〜十数 mA | 極小 | 無し | ★★★ |
| S3 | **LiDAR の電源を切ったら回転用 PWM（PA8、40%）も止める**。駆動電源が OFF の間は、モータドライバ向け UART の TX 線を Low かアナログにする | 逆給電があるかどうかで大きく変わる（回路での確認が要る） | 小 | TX 線をいじる場合は、再 ARM 時に初期化をやり直す必要あり | ★★（回路を確認してから） |
| S4 | **ADC の DMA 割込み（約 9k 回/s・コールバックは空）を止める**。さらに ADC を TIM の 2kHz トリガで変換させる | 割込み処理の分（CPU は 180MHz で回り続けるので、S6 と組み合わせないと効かない） | 小 | 無し | ★★ |
| S5 | **SYSCLK を 180MHz から 168MHz に下げ、Over-drive を外す**（APB は 42MHz、1Mbps は分周できる） | 数 mA @3.3V（11V 側で 1〜2mA） | 中（全タイマ・UART の分周の確認、LiDAR PWM の周期の再計算） | ボーレートの誤差の確認が必要。CPU 負荷を先に測ること | ★ |
| S6 | **アイドル待ちを WFI にする**（500µs 周期のビジーウェイトをやめる） | CPU コア電流が下がる分。数〜十数 mA @3.3V（11V 側で 2〜5mA） | **大**。時間基準 `Micros()` / `Timer_*` が DWT CYCCNT に依存しており、WFI 中は止まる。超音波のエコー幅も CYCCNT で測っている。時間基準を 1MHz のハードウェアタイマへ移し、割込みで起きたときに周期フラグがなければ再び WFI する形にする必要がある（Copilot によるレビュー結果） | 制御周期・PID の dt・超音波距離・ハートビートの全部を検証し直す必要あり | ★（効果に対して工数が大きい） |
| S7 | 未使用ピン（PA15・PB2・PB3・PB11・PC13〜15）をアナログに設定。UART と I2C の GPIO 速度を VERY_HIGH から MEDIUM へ。printf 用の UART5 を本番では止める | µA〜数百 µA | 極小 | 無し | ★ |
| S8 | DISARM 中は IMU を 200Hz から 50Hz に、テレメトリを 100Hz から 20Hz に落とす | 小（MPU6050 は通常動作でも数 mA） | 小 | Pi 側が 100Hz を前提にしていないか確認が要る | ☆ |

## 4. バグ・最適化（見つけたもの）

### Raspberry Pi
| # | 内容 | 根拠 |
|---|---|---|
| B1 | **ファンが毎秒回っては止まる（実機で確認）**。`_fan_pump` が auto モードでも毎秒 `set_auto()` → `_kick_governor()` を呼び、低温時は `cur_state` を 0↔1 で行き来させている。pwm は 0↔75、回転数は 0↔2600rpm。auto のときは「`pwm1_enable` が 2 でなくなった場合」や「温度が変わったのに状態が変わらない場合」だけ kick すべき | [fan.py:172](../raspi/io/fan.py#L172)、[telemetry_node.py:2367](../raspi/nodes/telemetry_node.py#L2367) |
| B2 | **e2e_lidar の ORT が既定設定のまま**（4スレッド＋spin）。0.3ms の推論に CPU を 31% 使っている（P2） | [e2e_lidar.py:78](../raspi/auto/e2e_lidar.py#L78) |
| B3 | **seq による重複排除が効いていない**。`Publisher._send` は送るたびに seq を振り直すので、line_perception・cam_e2e・cam_track が同じ結果を再送すると、planning の `_replan` が「新しい周」と判断して最大約 130Hz で `plan()` を回してしまう | [zbus.py:253](../raspi/bus/zbus.py#L253)、[planning_node.py:244](../raspi/nodes/planning_node.py#L244) |
| B4 | `_bus_pump` の docstring は 200Hz と書いているが、実際は `BUS_POLL_S=0.001`（1kHz）。カメラ配信も docstring は「最大15Hz」だが、既定値は 30Hz | [telemetry_node.py:17](../raspi/nodes/telemetry_node.py#L17)、[:298](../raspi/nodes/telemetry_node.py#L298) |
| B5 | cam_perception が、GUI に視聴者がいなくても推論のたびにマスクを JPEG 化している | `cam_perception_node.py` の mask_every |
| B7 | **`hb/cam_e2e` を誰も受け取れない**。`endpoints_for_topic` がハートビートを購読する接続先の一覧に `cam_e2e` が無いため、GUI の生存表示に届かない（2026-10-04 に実機で確認。今回は未修正） | [zbus.py:158](../raspi/bus/zbus.py#L158) |
| B6 | cv2 / BLAS を使うプロセスにスレッド数の指定が無い（`OMP_NUM_THREADS` も `cv2.setNumThreads` も無し）。systemd unit で `OMP_NUM_THREADS=1` を指定するのが無難 | `install_services.sh` |

### STM32（`FW/MainBoard_V3`）
| # | 内容 | 根拠 |
|---|---|---|
| M1 | **LiDAR の PWM が設計の半分の 15kHz で出ている**。`LIDAR_PWM_PERIOD` は TIM1 のクロックを 180MHz と仮定しているが、APB2 が ÷4 なので実際のタイマクロックは 90MHz | `lidar.h:41-45`、`main.c:173` |
| M2 | **超音波が failsafe 中も止まらない**。`MainApp` の冒頭で `Drive_Enable` が呼ばれ、`ApplyFailsafe` は `Drive_Disable` を呼ばないため | `app.c:238,266`、`vehicle.c:264-285` |
| M3 | LiDAR の電源を切った後も PWM（40%）が出続ける | `lidar.c:87`、`vehicle.c:53-66` |
| M4 | DMA が書き込むバッファ（`adc_dma.h` の buffer、`serial.h` の rxBuf）に volatile が無い。`-O2` にしたときに問題が表に出る恐れがある | `adc_dma.h:11` |
| M5 | 500µs 周期を超えても検出されない（オーバーランを数えていない）。「LED2 の点灯幅がアイドル時間」というコメントがあるが、その実装が無い | `app.c:297-299` |
| M6 | `PwmOut` が相補出力の無い TIM2〜4 にも `HAL_TIMEx_PWMN_Start` を呼んでいる（実害は未確認） | `pwm_out.h:20` |
| M7 | **checkout されているのは古い `main` ブランチで、実体のファームは `FW/MainBoard_V3` にある**。作業するときは取り違えに注意 | — |

## 5. 次の手順（提案）
1. すぐできるもの: P3（GUI の設定）、P2、B1/P8、S1、S2。
2. 次に P1＋P5（armed に応じて周波数上限を切り替える）。同じ条件で測り直す。
3. P11（telemetry の残り 34mA）を py-spy で切り分ける。
4. P4・P6・P7 を実装し、全部入れた状態でもう一度 A-B-A で測る。
5. STM32 は S3 の回路確認（バンビ）を待ってから、S4・S5。

## 6. 実装結果（2026-10-04、推奨度★★以上）

測定条件は 1. と同じ（DISARM・GUI 未接続・11V）。

| 段階 | 内容 | 待機電流 |
|---|---|---|
| 実装前 | — | 約 640mA |
| 1 | P2（e2e_lidar の ORT）・P8（ファン）・P6（カメラ系の IDLE）・P7（planning の poll） | 約 549mA |
| 1' | P3（DISARM 中の後カメラ OFF。GUI の設定値 `rear_enabled_disarm=false`） | 約 509mA |
| 2 | P1・P5（CPU 上限を DISARM 1.5GHz / ARM 2.0GHz）・P4（DISARM 中は使う者がいなければ前カメラ停止） | **約 430mA（−33%）** |

- P2 の実機確認: 新しい設定（1スレッド）で 10Hz 推論の上乗せは +11mA、推論時間 p50 0.30ms（既定設定は +100mA）。
- P4 の再開は約210ms。GUI でカメラ映像を開くと 10fps で即再開し、閉じると止まる。📷 撮影は止まっているカメラだと 409 を返す。
  後カメラは DISARM 中は設定（`rear_enabled_disarm`）だけで決め、GUI で表示していても撮らない（バンビ判断）。
- P11: py-spy では Python 側に目立つ処理が無かった。telemetry を止めたときの差は −22mA まで縮み、残りは 1kHz の `_bus_pump` と定期タスクの起床（P9）。
- STM32 の S1・S2・S3（LiDAR の PWM のみ）・S4 は実装・ビルド済みで、**ST-Link が繋がっていないため未書き込み**。
  モータドライバ向け TX 線の遮断（S3 の残り）は、回路で逆給電を確かめてからにする。

## 7. 対象追跡（follow_object）の追加対策（2026-10-04）

測定は GUI を開いた状態（基準 約525mA）、CPU 1.5GHz。Pi 上で本物の NanoTrack・実カメラのフレーム（30fps）を使い、ノードの `run()` をそのまま回した。

| 条件 | 追跡回数 | 基準からの上乗せ |
|---|---|---|
| 旧 cam_track（同じフレームも推論、4スレッド） | 44.7回/s（実機は vehicle_state の起床でさらに多い。単体ベンチでは約85回/s・+217mA） | +119mA |
| 新（新しいフレームだけ、2スレッド） | 30回/s | **+70mA** |
| 新＋`--track-hz 15` | 15回/s | +43mA |

- 実装: 同じ ring_seq のフレームは追跡も publish もしない（200ms フレームが来なければフレーム無しで1周期回し、見失いの計時を進める）、`cv2.setNumThreads(2)`、`--track-hz`（撮像時刻で間引く。既定 0＝毎フレーム）。
- 前カメラ 30→15fps は GUI 表示中で約 −35mA だったが、追跡性能を優先して **30fps のまま**（バンビ判断）。
