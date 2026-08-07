# SURGE Mark.2 開発進捗

会話が圧縮されても文脈を失わないための作業ログ。**新しいセッションではまずこれを読む。**
設計の中身は `docs/` が正。ここには「今どこまでやったか」「なぜそう決めたか」だけを書く。

最終更新: 2026-08-07

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
| ログ記録・再生 / GPIO / バス配信 | 未着手 |
| ログ記録・再生（MCAP） | 未着手 |
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

1. **`io_node`** — UART 送受信ループ、`COMMAND` 100Hz 送信、受信ディスパッチ
2. **時刻同期** — `PING`/`PONG` の4タイムスタンプ、min filter + 線形回帰
3. **GPIO** — E-Stop ハートビート 100Hz 出力（`gpiozero` + `lgpio`）、LED、ブザー
4. **メッセージ型とバス** — msgspec 型定義、ZeroMQ ラッパ
5. **ログ記録・再生** — MCAP 記録、`replay_node`（実質シミュレータ代わり）
6. **WS サーバ + GUI 骨格**

達成条件: **PC からラジコン操縦できる / 全データが記録できる。**

### 注意

- **STM32 側のファームはまだ無い。** 実機接続前に、記録したログを流す `replay_node` か
  ダミー送信スクリプトで Pi 側だけ検証できるようにしておくと手戻りが減る
- ステアリングのリンク比と車輪半径が未実測。経路追従（Phase 4）までには必要
