# SURGE Mark.2 開発進捗

会話が圧縮されても文脈を失わないための作業ログ。**新しいセッションではまずこれを読む。**
設計の中身は `docs/` が正。ここには「今どこまでやったか」「なぜそう決めたか」だけを書く。

最終更新: 2026-08-08（camera_node と共有メモリリングまで完了）

---

## 現在地

**Phase 0（通信基盤・ログ記録再生・GUI 骨格）を実装中。**

| | 状態 |
|---|---|
| 設計フェーズ | 完了 |
| UART プロトコル v0.4 | **確定**（STM32 側と2往復して合意） |
| プロトコル実装（Python + C ヘッダ生成） | **完了** |
| **Pi 実機セットアップ** | **完了**（SSH・リポジトリ配置・venv・UART 有効化） |
| **STM32 実機との UART 疎通** | **確認済み**（双方向・v0.4 一致・エラー0） |
| **io_node 中核（シリアル+時刻同期）** | **実機で安定動作**（loss=0/crc=0、45秒実証） |
| **生フレームログ `.sfl` 記録＋解析ツール** | **実機で記録・読み出し確認**（20秒・3566フレーム） |
| **`replay_node`（ログ再生＝シミュレータ代わり）** | **実機ログの再生が記録と完全一致** |
| **GPIO E-Stop ハートビート（Pi 側）** | **実機で 100.0Hz・最大遅れ 0.46ms・停止動作も確認** |
| **E-Stop の端から端までの動作確認** | **合格**（4項目とも実車で確認。途絶→ラッチ 64ms） |
| **カメラの実力測定** | **完了**（2台 30fps 同時・io_node と共存・時刻基準も確認） |
| **`camera_node` + 共有メモリリング** | **実機で動作**（2台30fps・別プロセスからゼロコピー読み出し確認） |
| MCAP エクスポート | 未着手（`.sfl` → MCAP の変換ツールとして作る） |
| バス配信 | 未着手 |
| WS サーバ / GUI 骨格 | 未着手 |

## Pi 実機（surge-mk2）

接続情報の詳細と機密は `docs/setup_credentials.md`（gitignore 済み）。要点だけ:

- **Raspberry Pi 5 / Debian 13 (Trixie) / Python 3.13.5**
- **接続は Ethernet 直結**。Mac の USB アダプタ(en18) ↔ Pi。
  **Pi eth0 = 固定 `169.254.55.2`**。`ssh surge-mk2` で決め打ち接続（`~/.ssh/config` 更新済み）
- **UART**: `/dev/serial0 -> ttyAMA0`。io_node では `/dev/serial0` を使う（名前直書き禁止）
- リポジトリは **Mac → Pi へ rsync**（`~/surge_mk2`）。git remote はあるが未 push
- venv は `~/surge_mk2/.venv`（`--system-site-packages`）。pyserial 3.5。
  gpiozero/lgpio は OS プリインストール利用

### ⚠ 原因不明の再起動が 2026-08-07 に**2回**

作業中に SSH が切れ、繋ぎ直すと `up 4 min` / `up 5 min`。**Pi が勝手に再起動している。**
`vcgencmd get_throttled` は `0x0`、温度 49.4°C で、電源・熱ともに異常なし。
Ethernet 直結のリンクも一度落ちている（Mac 側 link-local アドレスが変わった、
ping ロス 33%）が、これは Pi が落ちた結果と思われる。

**原因は未特定。1回目は journald の永続ログが無く前回起動のログが取れなかった。**
→ **永続ジャーナルを有効化済み（2026-08-07）。次に再発したら原因を追える。**

```bash
sudo journalctl --list-boots        # 起動履歴
sudo journalctl -b -1 -n 50         # 前回起動の最後
```

走行中に落ちるとハートビートが止まって E-Stop になる（フェイルセーフではある）が、
**信頼性の問題として残っている。再発したら必ずログを見ること。**

### ★ 不安定だった原因＝電源不足（解決済み）

Pi が「起動 → 数十秒 → ハングして消える」を繰り返していたのは **Pi 5 の電源不足**。
USB 給電では足りず、**5V/5A（27W）USB-C PD に交換して解決**。`vcgencmd get_throttled`=`0x0`。
緑 LED が点いていても OS はハングし得るので、LED だけで健全判断しないこと。

### Mac ↔ Pi の同期（現状の運用）

秘密鍵にパスフレーズがあり agent 未ロードのため、当面 **sshpass + パスワード**で運用
（パスワードは `docs/setup_credentials.md`）。バンビが一度 `ssh-add ~/.ssh/id_ed25519` すれば
鍵認証に移行できる。

```bash
export SSHPASS='<パスワード>'
OPT='-o StrictHostKeyChecking=no -o PreferredAuthentications=password -o PubkeyAuthentication=no'
rsync -az --exclude __pycache__ --exclude '*.pyc' --exclude .venv --exclude docs/setup_credentials.md \
  -e "sshpass -e ssh $OPT" surge_mk2/ pi@169.254.55.2:~/surge_mk2/
ssh surge-mk2 'cd ~/surge_mk2 && .venv/bin/python -m unittest discover -s raspi/tests -t .'
```

---

## 決まっていること

### UART プロトコル v0.4（確定・実装済み）

`docs/uart_protocol.md` が正。`raspi/proto/protocol.toml` が実装上の唯一の定義で、
そこから Python パーサと STM32 側 C ヘッダを生成する。

策定中に **`TELEMETRY` のフィールド順が Pi 側と STM32 側で食い違ったまま2往復した**ため、
定義を1箇所に閉じて機械生成する形にした。同じ事故を防ぐのが目的。

確定した主な内容:

- `TELEMETRY` LEN = 66。順序は `wheel_speed`(16) → **`odom_dist`(24)** → `accel`(32)
- `STATS` は**累積 u32**（増分ではない）。ロスしても情報が失われないため
- `VERSION_REQ` は `0x14`（`0x07` の双方向化はやめた）
- `param 0x0040` は LiDAR 出力フォーマットの **enum**（0=通常 / 1=強度付き / 2=圧縮）
- `CONFIG_SET` は**揮発**。Flash に書くのはステア原点と IMU キャリブレーションのみ
- CALIB モードは**廃止**。`calib_done` → `steer_center_valid` に改名
- `COMMAND` 途絶 100ms で自動ブレーキ / `TELEMETRY` 途絶は 100ms 警告・200ms FAULT

### 知覚・自己位置推定の方式 — **未定**

**LiDAR 主体とは限らない。カメラベースになる可能性がある。** Phase 3 で決める。
**LiDAR SLAM 前提で先回りして作らないこと。**

議論して分かったこと（`docs/architecture.md` §15-1 に記載）:

- **オドメトリ単独のデッドレコニングは成立しない。** 地磁気センサが無く、
  方位ドリフトが 1°/min。位置誤差は時間の2乗で効くので 1 m/s なら1分後に約 50cm ずれる
- LiDAR 主体にするなら、姿勢はスキャンマッチングが決め、オドメトリと IMU は
  歪み補正とマッチング初期値（100ms スケール）を担う
- **Phase 0〜2 はこの選択に依存しない**ので先に作れる

### 前輪オドメトリの扱い（どの方式でも共通）

エンコーダは**前輪 = 操舵輪**に付いている。`odom_dist` / `wheel_speed[FL,FR]` は
車輪自身の軌跡に沿った値で、車体中心線方向ではない。

- **射影は累積値ではなく差分に対して行う。** `cos(δ)` を累積距離に掛けるのは誤り
- 射影済みなのは `speed` だけ（STM32 側で処理）
- 前輪のスリップ量を見るときも射影が要る

---

## STM32 実機との疎通（2026-08-07 確認済み）

`raspi/tools/probe_uart.py` で Pi ↔ STM32 の双方向通信を実機確認した。**ファームは v0.4 準拠。**

- 受信: `TELEMETRY` 50Hz / `STATS` 1Hz が正しい頻度で来る。**CRC・長さ・ロスすべて 0**
- 送信: `VERSION_REQ` → `VERSION` 応答（`protocol_version=0x0004` 一致、`fw_id=0x4D463303`）、
  `PING` → `PONG` 応答
- ベンチ状態の観測（正常）:
  - `accel_z ≈ +9.87 m/s²`（IMU 健全、静止で重力を検出）
  - `flags` に `fault_drive_undervoltage`(0x2000) + `armed` — **駆動バッテリー未接続**のため
  - `md_status` 全 `0x00`（`comm_ok` ビット無し）— **MD/モータ未接続**。この間 temp/current は当てにしない
  - `batt_signal` 9.55V は正常、`batt_drive` 1.55V は未接続
- 診断ツール: `.venv/bin/python raspi/tools/probe_uart.py [--passive|--ping]`

**注意: STM32 に給電・配線されていないと 0 バイトになる**（最初はそれで空振りした）。

## io_node 中核（2026-08-07・方針 A で実装）

バス（ZeroMQ）を入れる前に、シリアル送受信と時刻同期だけの単体版を作り、実機で回した。

- `raspi/core/timesync.py` — PING/PONG 4タイムスタンプ。u32 μs の unwrap、min filter、
  線形回帰でドリフト推定。**短い時間スパン（<15秒）では回帰せずオフセットのみ**にフォールバック
- `raspi/io/serial_link.py` — pyserial ラッパ。poll()/send() で T1/T4 を ns で取得
- `raspi/nodes/io_node.py` — 起動時 VERSION 照合 → PING(5Hz/warmup 20Hz) →
  **COMMAND は常に DISARM（安全ハートビート、arm は絶対しない）** → 受信集計 → ライブ表示
- テスト計 50件（proto 37 + timesync 13）。実行:
  `.venv/bin/python -m raspi.nodes.io_node --duration 10`

### 実機での実証結果（電源修正後・安定）
- VERSION 一致（0x0004、fw=0x4D463303="MF3"）、health INIT→OK
- TELEMETRY 50Hz / LIDAR 120Hz(12セクタ) / STATS 1Hz を正しい頻度で受信
- **loss=0・crc=0・resync=0B**（45秒実証）。リンクは完全にクリーン

### 確定した実機の性質（2つ）

1. **STM32 のクロックが約 -3360ppm ドリフトしている。**
   4回の実行で -3339/-3352/-3353/-3362ppm と安定。0.336% は水晶にしては大きく、
   **STM32 が外部水晶でなく内蔵 RC(HSI, ±1%級)で μs タイマを回している**とみられる。
   時刻同期の回帰補正がこれを吸収するので実害なし。**15秒未満は回帰せずオフセットのみ**の
   ガードも入れた（短窓だとジッタで傾きが暴れて -3300 のような値が出るため）。

2. **STM32 は送信優先度キューで SEQ を数個ぶん並べ替える**（TELEMETRY が LIDAR を追い越す）。
   逆行しても実フレームは全部届く（`tx_drop=0`）。
   → **framing.py の loss 計算バグを修正**（逆行を 254 ロスと誤計上していた）。
   逆行は `reordered` として別集計し、飛び計上を相殺して loss=実ロスだけにした。

### 補足: MD は未接続
`STATS.md_rx_error` が巨大・`md_rx_count=0`・`md_status=0x00`。モータドライバ未接続の
ベンチ状態なので正常。この間 temp/current/wheel_speed は当てにしない。

## ログ記録 `.sfl`（2026-08-07）

### なぜ MCAP を実時間パスに置かなかったか

設計書（`docs/architecture.md` §11）はログを MCAP と決めているが、**MCAP はトピック
（意味のあるメッセージ）の器**であり、`msgs/`（msgspec 型）と `bus/` が決まってから。
今それをやるとメッセージ型を先に凍結することになる（知覚方式が Phase 3 まで未定なのに）。
加えて Mac に pip パッケージが入らないため、`mcap` に依存すると Mac 側でテストできない。

そこで **生フレームログを先に作り、MCAP は後段のエクスポータ**にした。

- **生フレームこそが真実** — CRC エラー・SEQ 逆行・リンク断は、デコード後の
  きれいなメッセージには残らない。デバッグで要るのはそこ
- protocol.toml が変わっても、**過去のログを新しいパーサで読み直せる**
- 実時間で回る io_node に外部依存を持ち込まない（stdlib のみ）

### 形式

`raspi/rec/framelog.py` のモジュール docstring が正。要点だけ:

- 32B ヘッダ + `(kind, ptype, seq, t_ns, len, payload)` 14B の追記レコード
- `kind` = RX / TX / META(JSON) / EVENT(JSON)
- 時刻は **Pi CLOCK_MONOTONIC の絶対値**。TELEMETRY 内の STM32 時刻や
  TimeSync の推定と突き合わせるときに変換を挟まないため
- **末尾が切れていても前半は必ず読める**（追記のみ・インデックス無し）。
  正常終了したログには `close` イベントが入るので、電源断ログを判別できる
- **flush はレコードの t_ns 基準（既定1秒）＋4096件の歯止め**。
  時刻だけで判定すると、t_ns が想定外だと永久に flush されず記録が黙って消える

フレームに現れない情報は EVENT で残す:
`handshake` / `health` 遷移 / `linkstats`（1Hz・CRC エラーや resync バイト数、
時刻同期の推定値）。**捨てたフレームは RX レコードに残らない**ので、
数字として別に持たないと再生しても異常が見えない。

### 使い方

```bash
# 記録（--log は値なしで logs/ に自動命名、*.sfl でファイル指定）
.venv/bin/python -m raspi.nodes.io_node --duration 20 --log

# 要約（頻度・期待Hzとの比較・最大間隔・リンク統計・イベント）
python3 -m raspi.tools.logcat logs/surge_20260807_055014.sfl
python3 -m raspi.tools.logcat run.sfl --dump 40 --type TELEMETRY
python3 -m raspi.tools.logcat run.sfl --csv telem.csv --type TELEMETRY  # 物理量で出る
```

### 実機での実証（20秒・STM32 接続）

```
LIDAR_SECTOR 2403 (120.1Hz)  TELEMETRY 998 (49.9Hz)  PONG 144  STATS 20 (1.0Hz)
COMMAND 2001 (100.0Hz)  PING 144  VERSION_REQ 1
loss=0 crc=0 reordered=96   0.34MB / 20s ≈ 17KB/s ≈ 61MB/時
```

送受信フレーム数がログのレコード数と完全一致。Mac に持ち帰って CSV 化まで確認済み。
**記録が壊れても走行は止めない**（ログ書き込みの例外はカウントするだけでループ継続）。

ログだけで実機に触らず分かったこと（＝この仕組みの狙い通り）:

- `flags` は `0x2580`。`imu_ok` / `lidar_ok` / `steer_center_valid` が立ち、
  `armed` は立っていない（DISARM 固定が効いている）
- `uart_timeout` は**最初の1フレームだけ ON**（998 中1）。io_node 起動前は
  COMMAND が来ていないので当然で、0.034s には消えている。異常ではない

### ★ 実データで見つかった定義の誤り — `rot_speed_dps`（修正済み）

`LIDAR_SECTOR.rot_speed_dps` を **`scale = 0.01` deg/s** と定義していたが、
**実ファームは deg/s をそのまま送っていた**。**定義が誤りでファームが正しい。**

- 生値の中央値 **3594**（2403 セクタ）。同フレームの `duration_us` から逆算した
  回転速度は **3642 deg/s** → スケール 1.0 なら一致、0.01 だと 100倍ずれる
- そもそも **u16 × 0.01 は最大 655.4 deg/s** で、実回転（約 3600 deg/s）を表現できない

**2026-08-07 に `scale = 1.0` へ修正**（`protocol.toml` 3箇所 → 再生成、
`uart_protocol.md` §5.1/§5.2 と変更履歴）。

- **ワイヤ形式は無変更**。生成物の差分は Python の `META` と C のコメントだけで、
  構造体サイズもオフセットも v0.4 のまま（`cc -Werror` + `offsetof` で検証済み）
- `protocol_version` は **0x0004 のまま**。**STM32 側の変更は不要**
- **修正前に記録した `.sfl` が、新しい定義でそのまま正しく読み直せた**
  （CSV に `3589.0 deg/s` と出る）。生フレームで残す設計の狙い通り

#### 教訓

**スケールを決めたら「その型で実値が表現できるか」を必ず検算する。**
u16×0.01 の上限 655 deg/s は、LD06 の 10 rps = 3600 deg/s を最初から表現できなかった。
`protocol.toml` は単一定義にはなっているが、値域の妥当性までは見てくれない。

**他の全フィールドも同じやり方で監査済み（2026-08-07）。`rot_speed_dps` 以外は問題なし。**
電源系は 8セル NiMH（8.0〜11.2V）に対し u8×0.05 = 0〜12.75V で余裕 1.55V、
電流も過電流しきい値（駆動 5.0A / シグナル 3.0A）に対し 2.5倍 / 1.7倍 とれている。

## replay_node（2026-08-07）

`.sfl` を実時間で流して io_node の代わりをするノード。**これが事実上のシミュレータ代わり。**
実車・STM32・シリアルなしで知覚・経路生成・GUI を開発できる。pyserial にも依存しない。

### 実機と食い違わせないための作り

- **`LinkTracker`（`raspi/core/link_tracker.py`）を io_node と共有する。**
  受信ディスパッチ・時刻同期・health 判定を2箇所に書けば必ずズレる
  （プロトコル定義が2箇所にあってズレた前科がある）。io_node も載せ替えた
- **ログ時刻の座標系のまま動かす。** 「今」は壁時計ではなく再生カーソル。
  壁時計は待ち時間の計算にしか使わない。**何度流しても同じ結果になる**
- **PING の T1 は TX レコードから復元。** PONG(RX) と突き合わせるので、
  時刻同期は記録時とまったく同じ4タイムスタンプで動く
- **フレームの無い区間でもカーソルを 10ms 刻みで進めて health を更新する。**
  リンクが完全に切れた区間はレコードが空になるので、次のフレームまで待つ実装だと
  DEGRADED/FAULT を丸ごと見落とす
- CRC エラー・再同期バイト数は捨てたフレームなので復元不能 →
  1Hz の `linkstats` イベントから読み戻す（**そのために記録している**）

### 使い方

```bash
python3 -m raspi.nodes.replay_node logs/run.sfl               # 等速
python3 -m raspi.nodes.replay_node logs/run.sfl --speed 4     # 4倍速
python3 -m raspi.nodes.replay_node logs/run.sfl --speed 0     # 最速（待たない）
python3 -m raspi.nodes.replay_node logs/run.sfl --start 12 --end 18
python3 -m raspi.nodes.replay_node logs/run.sfl --verify      # 記録と突き合わせ
```

`--start` は早送りで**状態を温めてから**その位置に入る（冷えた状態から始めても役に立たない）。

### `--verify` — ログが十分かを検査する

再生結果を記録内容と突き合わせる。**合わなければ「ログに足りない情報がある」**という意味で、
再生器のバグとして片付けないこと。実機ログ（20秒 / 3566フレーム）で全項目一致:

```
✓ 受信フレーム数: 再生 3566 / 記録 frame_ok 3566
✓ クロックドリフト: 再生 -3366.35ppm / 記録 -3366.35ppm
✓ 時刻同期サンプル数: 再生 144 / 記録 144
✓ health 遷移: 再生 [('INIT','OK')] / 記録 [('INIT','OK')]
```

別の15秒ログでも `offset` が実機ライブと**完全一致**（`-10961548404 ns`）。
20秒ぶんの再生に 0.01秒。テスト計 **127件**（+40: link_tracker 20 / replay_node 20）。

## GPIO E-Stop ハートビート（2026-08-07）

`raspi/io/gpio.py`。GPIO6 に 100Hz の矩形波を出し続け、STM32 が「エッジが 50ms
途切れたら即ブレーキ」で受ける（`docs/architecture.md` §7.1）。

### 設計上ここだけは外せない2点

**1. ハードウェア PWM を使ってはいけない。**
PWM に任せると **Python プロセスが死んでも波形が出続け**、STM32 が「Pi は生きている」と
誤認して安全機構が丸ごと無効になる。波形はプロセス自身が出す。プロセスが死ねば
OS が GPIO を解放し、プルアップで High に張り付いてエッジが消える＝フェイルセーフ。

**2. スレッドが生きているだけでは不十分 → `kick()`。**
専用スレッドで叩くだけだと、**メインループが固まってもスレッドが元気に波形を出し続ける**。
それでは「Pi のフリーズ」を検出できない。メインループが毎周 `kick()` を呼び、
100ms 途切れたら**ハートビート側が意図的に波形を止める**。

```
メインループ停止 → kick 途絶(100ms) → 波形停止 → STM32 が 50ms で E-Stop
                                                  合計およそ 150ms
```

### 実機での実測（Pi 5・STM32 接続・60秒）

```
edges=12000 (100.0Hz)  最大遅れ=0.46ms  停止=0回  skip=0  {'<1ms': 12000}
同時に frames_ok=10608 crc_err=0 loss=0   ← シリアルにも影響していない
```

**12000 エッジ全部が 1ms 未満の遅れ。最悪 0.46ms は 50ms の予算に対して約 108倍の余裕。**
→ **Python のスレッドでこれを回して問題ない**と判断できる。

フリーズ検出も実機で確認（`kick` を止めると 19本＝タイムアウト分だけ出して停止、
再開すると 100Hz に復帰）。ハートビートの実測値は `.sfl` の `linkstats` と
`heartbeat_start` / `heartbeat_stop` イベントに残る（GPIO はフレームに現れないため）。

### 仕様からの逸脱を1つ（意図的）

仕様は「オープンドレイン推奨」だが**プッシュプルにした**。Pi も STM32 も 3.3V で
電位差が無く、gpiozero がオープンドレインを直接持たないため入出力切替で模擬すると
毎秒 200回の切替でジッタが増える。Pi 電源断ならピンはハイインピーダンスになり
STM32 側プルアップが High を保つので、フェイルセーフ性は失われない。

### STM32 側の確定挙動（2026-08-07 に判明）— 詳細は `uart_protocol.md` §9

STM32 側は**実装済み**（2kHz ポーリング）。当初「ハートビートを送っていないのに
`estop_active` が立たない」のを不審に思ったが、**仕様どおりだった**:

> ハートビートを**一度も見ていない間は緊急停止を発動しない**（ベンチ確認のため）。
> エッジを10回（50ms 分）数えて初めて「Pi が繋がっている」とみなす。

**Pi 側が守ること:**

- **出し始めたら出し続ける。途中でやめると E-Stop がラッチする。**
  解除には**車両のボタン2を人間が押す**必要がある（自動復帰しない）
- 逆に**一度も出さなければ発動しない**ので、ベンチ診断では `--no-heartbeat` を使う
- **`estop_active` 中は `COMMAND` が一切効かない。** リンク断（health）とは別の軸
- 発動しても**駆動電源は切られない**（切ると MD が制動できず惰行して停止距離が伸びる）

これを受けて Pi 側に入れた対応:

- `LinkState.estop_active` / `.drive_power_locked` を追加し、
  **立ち上がりを `on_latch` コールバックで捉えて `.sfl` に `latch` イベントで残す**。
  人間の物理操作でしか戻らないので、立った瞬間を逃すと原因が追えなくなる
- ステータス行に `★E-STOP発動中(車両のボタン2で解除)` を出す（16進フラグに埋もれさせない）
- LED は **E-Stop を health より優先**（赤 4Hz 点滅＋0.6秒ブザー。FAULT の 2Hz/0.2秒と区別）
- io_node 起動時と終了時に「止めるとラッチする」ことを明示表示

**オープンドレインは「でよい」と確認された**（基板に外付けプルアップあり）。
プッシュプルのままで問題ないが、切り替えたければ電気的にはどちらでも通る。

### ★ E-Stop 端から端までの動作確認 — **合格**（2026-08-07・実車）

`raspi/tools/estop_test.py`。**ファーム・配線・Pi 側のどれかを触ったら回し直すこと。**
「波形は出ている」だけでは安全機構が効いている証拠にならない。

```
.venv/bin/python -m raspi.tools.estop_test                  # 4項目を通しで検査
.venv/bin/python -m raspi.tools.estop_test --release-only   # 後片付け（解除だけ待つ）
```

| 検査項目 | 結果 |
|---|---|
| ハートビート中は正常運転（E-Stop なし） | ✓ |
| **途絶でラッチする** | ✓ **64ms**（3回測って 65 / 60 / 64ms。期待 50〜90ms） |
| **自動復帰しない**（ハートビートを戻しても解除されない） | ✓ |
| ボタン2で解除される | ✓ |

`estop_active` 発動時の `flags` は **`0x2588`**（通常 `0x2580` + bit3）。

検査は E-Stop をラッチさせるので、**開始時にラッチしていたらまず解除を待つ**構造にした。
その解除待ちがそのまま「人間の操作でしか戻らない」ことの確認になっている。
**終了時に車両をラッチしたまま残さない**（残った場合は `--release-only` で戻せる）。

判定は「安全項目の合否」と「車両をラッチしたまま終えたか」を**別に集計する**。
混ぜると「安全機構が壊れている」と「ボタンの押し忘れ」が区別できなくなる。

#### 1回目が不合格だった件（原因判明・実装の問題ではない）

初回は「ハートビートを止めても `estop_active` が立たない」で落ちた。
**原因は STM32 のファーム書き込み忘れ**（E-Stop 未実装のファームが載っていた）。
ログがそれを裏付けている: 受信が **8.01秒でぴたりと途絶**し（TELEMETRY 403本で終わり、
以降 90秒間ゼロ）、書き込み開始の瞬間と一致する。2回目は 95秒間 4744本で正常。

**検査は正しく「反応なし」を捉えていた。** 生ログを残していたから、
「ツールのバグ」ではなく「載っているファームが違う」と切り分けられた。

#### 配線の切り分け方（同じことが起きたとき用）

Pi 側がシロであることは、ソフトを疑う前にハードで確かめられる。

```bash
pinctrl set 6 ip pd && pinctrl get 6   # 内部プルダウンを掛けて読む
```

**`hi` のままなら外部プルアップが吊っている＝線が繋がっている。**
未接続のピン（GPIO5/16 で対照実験した）は `lo` になる。
出力の駆動確認は `pinctrl get 6` が `op dh|hi` / `op dl|lo` と書いた値に追従するかを見る。

### LED / ブザーは未検証

GPIO19 緑 / GPIO13 赤 / GPIO18 ブザーを health に連動させる `StatusIndicator` を
入れたが、**配線を確認していないので動作未確認**。ピンが開けなければ黙って無視する
（表示が出ないだけで走行に影響しない）。

### 使い方

```bash
.venv/bin/python -m raspi.nodes.io_node --duration 60 --log   # 既定でハートビート ON
.venv/bin/python -m raspi.nodes.io_node --no-heartbeat        # 出さない（診断用）
.venv/bin/python -m raspi.nodes.io_node --require-gpio        # GPIO が開けなければ起動しない
```

GPIO が開けない環境（Mac など）では警告を出して続行する。`--require-gpio` で止められる。

## カメラの実力測定（2026-08-08）

`raspi/tools/camera_probe.py`。**「映った」で終わらせないための道具。**
`camera_node` を書く前に、設計を変えさせ得る数字を先に取った。

### 構成

- **imx219（Camera Module v2, 8MP）×2**。別々の CSI ポートで両方検出
- 対応モード: `640x480@200fps` / `1640x1232@81fps` / `1920x1080@47fps` / `3280x2464@21fps`
- **`python3-picamera2` は apt でしか入らない**（libcamera の Python バインディングは
  C++ を meson でビルドしたもの。pip 不可）。**`--no-install-recommends` で入れること** —
  既定だと PyQt5 / Qt 一式（プレビュー GUI 用）まで付いてくる
- 導入のため **Pi を Wi-Fi に接続した**（`docs/setup_credentials.md`）。
  eth0 直結は維持される

### ★ `SensorTimestamp` は `CLOCK_MONOTONIC` 基準だった

**これが一番知りたかったこと。**

```
SensorTimestamp と CLOCK_MONOTONIC の差:               31.3 ms （＝パイプライン遅延）
                  CLOCK_REALTIME  の差:  1,786,166,048,935.7 ms
```

`t_capture` にそのまま使える。**STM32 のような時刻同期は要らない。**
`docs/architecture.md` §6.3 の「全メッセージが Pi の `CLOCK_MONOTONIC` で
`t_capture` を持つ」前提がそのまま通る。

### 実測（2台同時・カメラごとにスレッドを分けた場合）

| 解像度 | fps | 取得レイテンシ | 帯域/台 | 備考 |
|---|---|---|---|---|
| 640x480 | **30.0** | **5.9ms** | 28 MB/s | センサのネイティブモード |
| 1280x720 | 30.0 | 23.8ms | 83 MB/s | **ネイティブに無い**→縮小されて遅い |
| 1640x1232 | 30.0 | 15.7ms | 182 MB/s | ネイティブモード（2MP） |

- フレーム間隔の揺らぎは **±0.01ms**。フレーム落ちなし
- **1280x720 は 1640x1232 より遅い。** ネイティブモードに無く libcamera が縮小するため。
  **解像度を選ぶときはネイティブモードから選ぶこと**
- 640x480 の 28 MB/s は設計書の見積り 27 MB/s とほぼ一致 →
  **「画像は ZMQ に流さず共有メモリ」の判断は正しい**

### ★ io_node と共存できる

カメラ2台を 30fps で回しながら io_node を 30秒:

```
frames_ok=5326  crc_err=0  loss=0     ← リンクは完全にクリーンなまま
CPU: user 3-4% / system 1% / idle 95-96%（4コア）
```

**余裕は桁違いにある。** プロセスを分ける設計はそのままで問題ない。
ただし**取得しているだけで何も処理していない**数字である点に注意
（ISP はハードウェアが担当）。知覚処理を載せたら当然変わる。

### ★ 設計上の落とし穴 — 1スレッドで2台を順に取ってはいけない

最初の測定でカメラ1だけレイテンシが **38.1ms**（カメラ0は 5.6ms）になった。
カメラ1単独で測ると 5.5ms だったので、**カメラではなく測り方の問題**と判明。

1スレッドで2台を順にブロッキング取得すると、1台目を待つ間に2台目のキューが
1フレーム溜まり、**常に1フレーム古いものを取り続ける**（+33ms）。
カメラごとにスレッドを分けたら両方 6.0ms になった。

**`camera_node` は必ずカメラごとに独立して取得すること。**

### 補足: ネットが無いと壁時計がずれる

Wi-Fi を繋ぐまで NTP が効かず、Pi の時計が約1日ずれていた
（apt の署名検証が「まだ有効時刻でない」で失敗して発覚）。
**記録データは全部 `CLOCK_MONOTONIC` 基準なので影響を受けない** —
この設計判断が効いた形。ただし `.sfl` のファイル名と `t0_unix_ns` は壁時計依存。

## camera_node と共有メモリリング（2026-08-08）

`raspi/bus/shm_ring.py` + `raspi/nodes/camera_node.py` + `raspi/tools/shm_view.py`。
設計は `docs/architecture.md` §6.2 の「画像は ZMQ に流さず共有メモリ」そのまま。

### 構造

```
┌ ヘッダ 64B ────────────────────────────────────┐  /dev/shm/surge_cam0
│ magic / n_slots / slot_bytes / w / h / stride  │
│ format / write_seq                             │
├ スロット表 32B × n_slots ──────────────────────┤
│ seq(seqlock) / t_capture_ns / frame_id / nbytes│
├ データ領域 slot_bytes × n_slots ───────────────┤
└────────────────────────────────────────────────┘
```

- `write_seq` は累積フレーム数。書き込み先は `write_seq % n_slots` なので
  **読み手は「最新」を一意に特定できる**
- バスに流すのは `FrameDesc`（名前・スロット・seq・`t_capture_ns` など数十バイト）だけ。
  **画素は流さない**
- 640x480x3 × 8枚 = **7.4MB / カメラ**

### seqlock — 上書きを「検出できる」ようにする

8枚あれば書き手が同じスロットに戻るまで 266ms（30fps）あり、実用上は衝突しない。
問題は**衝突したときに気づけないこと**なので seqlock を入れた。

```
書き手: seq += 1（奇数=書込中） → 画素を書く → seq += 1（偶数=安定）
読み手: seq を読む → 画素を読む → seq を読み直す → 変わっていたら破棄
```

**正直に書くと、Python には明示的なメモリバリアが無い**ので厳密な保証ではない。
実効的な守りは段数を十分取ることで、seqlock はその**検出手段**。
`still_valid()` が False なら捨てて次を待つ。

### 実機での実証

カメラ2台 + io_node + 読み手2プロセスを**同時に**走らせた:

```
camera_node : cam0/cam1 とも 30.1fps 落ち0  共有メモリ書込 平均223-261μs 最大536μs
io_node     : frames_ok=3566 crc_err=0 loss=0        ← シリアルは無傷
読み手×2    : 29.7 / 29.9fps  上書き検出0  ゼロコピー確認済み
撮像→読み手 : 中央値 11-12ms
```

`shm_view --save` で PNG に落として**画素が本物であることを目視確認**した
（配管が通っただけでは画が正しい保証にならない）。PNG 書き出しは zlib のみで
外部依存なし。

### ★ 踏んだ落とし穴2つ

**1. DMA バッファは `release()` 前に共有メモリへ書くこと。**
`req.make_array()` は DMA バッファへの view であって、`release()` すると無効になる。
最初 release してから書く実装にしていた（解放済みメモリを読む）。
コピーは「DMA → 共有メモリ」の1回だけで済む。

**2. 参照を残したまま共有メモリを閉じられない。**
numpy 配列や memoryview が生きていると `BufferError`。握り潰すと後で
`SharedMemory.__del__` が「Exception ignored」を吐いて原因が追えなくなるので、
**`close_blocked` を立てて呼び出し側に見せる**ようにした。
読み手は使い終わったら参照を手放すこと。

### ★ RGB / BGR — 実測で確定し、修正済み（2026-08-08）

**libcamera の形式名はメモリ上のバイト順ではない。** 32bit ワードにパックしたときの
並びを指すので、リトルエンディアンのメモリ上では逆になる。

カメラの前に**赤いシート**を置いて撮ったところ、`format="RGB888"` で
配列の **ch2 が最大**になった（ch0=151 ch1=115 **ch2=198**）。
PNG に素直に書くと赤が青紫に写る。**メモリ上は `B, G, R` で確定。**

修正方針は「PNG 側でごまかす」ではなく **リングにメモリ上の実際の並びを記録する**:

- `camera_node` が libcamera の名前を変換して入れる
  （`RGB888 → BGR888`、`BGR888 → RGB888`。`memory_format()`）
- 下流は `FrameDesc.fmt` をそのまま信じてよい。
  **名前を信じて色が入れ替わる事故**が起きない形にした
- `shm_view` は `fmt` を見て、必要なときだけ入れ替えて PNG にする
- 対応表は `raspi/tests/test_camera_format.py` で固定（往復して元に戻ることも検査）

修正後、実機で**赤いシートが赤く写ることを確認済み**。

### カメラの取り付け向きは正しい（2026-08-08 確認）

FFC が下から出る向き（imx219 の標準）で、**画像は正立**する。
上下反転も左右反転もしていない。根拠:

- Apple ロゴが正立して写った（葉が上、かじり跡が右）
- 車体が画面**下**に写る — 前を向いたカメラが自分の車体を下端に捉える正しい位置関係

将来カメラを回して付けるなら、**picamera2 の `transform`（hflip/vflip）で
設定時に直すこと**。ISP が処理するので実質ゼロコストで、
下流でソフト的に回すより安い。

## やったこと

### 2026-08-06 〜 07

1. **プロトコル v0.4 を確定**（コミット `5dbf6a2`）
   - `docs/uart_protocol.md` を v0.4 確定版に
   - `docs/stm32_interface.md` を v0.3（プロトコル v0.4 対応）に
   - `docs/architecture.md` の Phase 3 を「方式未定」に変更

2. **プロトコル実装**（コミット `76bad1f`）
   - `raspi/proto/protocol.toml` — 唯一の定義
   - `raspi/proto/generate.py` — Python + C ヘッダ生成
   - `raspi/proto/framing.py` — CRC16・フレーム組み立て・ストリーム抽出
   - `raspi/tests/test_proto.py` — 37件
   - 生成した C ヘッダを `cc -Werror` で検証済み。
     **Python 側と C 側でオフセットが一致することを確認**（`odom_dist`=24, `accel_x`=32）

5. **GPIO E-Stop ハートビート**
   - `raspi/io/gpio.py` — `Heartbeat`（kick 方式・ジッタ実測付き）、
     `StatusIndicator`（LED/ブザー）、`FakePin`（ハード無し環境用）
   - `raspi/nodes/io_node.py` — 既定 ON、毎周 `kick()`、終了時に波形停止
   - `raspi/tests/test_gpio.py`（26件・仮想クロックで決定的に検証）
   - `raspi/tools/estop_test.py` — **実車での端から端まで検査**（4項目／後片付けモード付き）

4. **`replay_node` と `LinkTracker`**
   - `raspi/core/link_tracker.py` — `LinkState` + 受信ディスパッチ + 時刻同期 +
     health 判定。**io_node と replay_node で共有**（io_node から括り出した）
   - `raspi/nodes/replay_node.py` — 再生・区間指定・倍速・`--verify`
   - `raspi/tests/test_link_tracker.py`（20件）/ `test_replay_node.py`（20件）

3. **生フレームログ `.sfl`**
   - `raspi/rec/framelog.py` — Writer / Reader（stdlib のみ）
   - `raspi/nodes/io_node.py` — `--log` で記録。RX/TX/イベントを全部落とす
   - `raspi/io/serial_link.py` — `on_tx` フックを追加（送信 SEQ はエンコーダ内部に
     しか無く呼び出し側から見えないため。記録は横断的関心事なのでコールバックで外に出す）
   - `raspi/tools/logcat.py` — 要約 / ダンプ / CSV 書き出し
   - テスト計 87件（proto 37 + timesync 13 + framelog 30 + io_node ログ結合 7）。
     結合テストは偽リンクで io_node を回し、**COMMAND が常に DISARM であること**まで
     ログから検証している

---

## 開発環境の制約

Mac 側の python3.12 に **pyyaml も pytest も入っていない**（venv も無い）。
勝手に入れずに標準ライブラリで済ませる方針にした。

- 設定ファイルは YAML ではなく **TOML**（`tomllib` は標準ライブラリ）
- テストは **`unittest`**

```bash
# テスト
python3 -m unittest discover -s surge_mk2/raspi/tests -t surge_mk2

# プロトコル再生成（protocol.toml を編集したら必ず）
python3 surge_mk2/raspi/proto/generate.py
```

Pi 5 実機では ZeroMQ / msgspec / numpy / picamera2 / gpiozero+lgpio が要る。
**venv と requirements はまだ作っていない。**

---

## 次にやること（Phase 0 の残り）

優先度順。上ほど他が依存する。

1. ~~**`io_node`**~~ 完了
2. ~~**時刻同期**~~ 完了
3. ~~**ログ記録（生フレーム）**~~ 完了
4. ~~**`replay_node`**~~ 完了
5. ~~**GPIO E-Stop**~~ 完了。**実車で端から端まで合格**（上記）。
   これで **arm を通す前提条件は満たした**
6. ~~**`camera_node`**~~ 完了（共有メモリリングまで実機確認）
7. **メッセージ型とバス** — msgspec 型定義、ZeroMQ ラッパ。
   ここが決まったら `logger_node`（MCAP）と `.sfl` → MCAP エクスポータを作る
8. **WS サーバ + GUI 骨格** — `replay_node` を繋げば実車なしで作れる

### バスの手前で決めること

**Pi にまだ `pyzmq` も `msgspec` も入っていない。** しかも Mac には pip が無いので、
バス層のコードは Mac でテストできなくなる。`requirements.txt` を作る段階で、
「バス層は Pi でしかテストできない」を受け入れるか、抽象を挟んで中身は Mac でも
回せるようにするかを決めること。

達成条件: **PC からラジコン操縦できる / 全データが記録できる。**

### 注意

- **ラジコン操縦にはまず `COMMAND` を DISARM 固定から外す必要がある。**
  現状 io_node は安全のため常に DISARM を送る。E-Stop は実車で合格済みなので、
  前提条件は満たしている。**arm を入れたら `estop_test.py` を回し直すこと**
- 起動直後の1フレームだけ、STM32 の**前の状態が残る**（`armed` や `uart_timeout` が
  立っていることがある）。DISARM ハートビートが 20ms 以内に落とすので実害は無いが、
  GUI で「起動直後の1フレームで驚かない」ようにしておくこと
- ステアリングのリンク比と車輪半径が未実測。経路追従（Phase 4）までには必要
- ベンチ観測で `steer_actual ≈ +0.392 rad`（約22.4°）が張り付いている。
  `steer_center_valid` は立っているので、**原点は有効なまま実際に切れている**か、
  原点そのものがずれている。実車で確認すること
