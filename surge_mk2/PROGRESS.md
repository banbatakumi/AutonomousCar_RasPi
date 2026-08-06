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
| GPIO / 時刻同期 / io_node | 未着手 |
| ログ記録・再生（MCAP） | 未着手 |
| WS サーバ / GUI 骨格 | 未着手 |

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
