# Pi ⇄ STM32 の UART プロトコル

!!! note "【設計の説明】"
    定義: Pi 側 [`raspi/proto/protocol.toml`][protocol-toml]（**唯一の正**）。STM32 側は [`ras_link.h`][ras-link-h] / [`ras_link.c`][ras-link-c]。
    確認した版: プロトコル v0.23（`protocol_version = 0x0017`）。ファーム `2a2e9a8`、上位 `19291eb`。

## 物理層とフレーム

| 項目 | 値 |
|---|---|
| 物理 | UART 1,000,000 bps、8N1、フロー制御なし、3.3 V（レベル変換なし） |
| 配線 | Pi GPIO14（TX）→ STM32 RX、Pi GPIO15（RX）← STM32 TX（Pi 側のデバイスは `/dev/serial0`） |
| 実効帯域 | 100 kB/s（1 バイト = 10 ビット） |
| STM32 側 | USART1。USARTDIV = 45 MHz ÷ (16 × 1 M) = 2.8125 = 2 + 13/16 で、ボーレート誤差 **0 %** |

```text
 0xAA 0x55 | TYPE | SEQ | LEN | PAYLOAD (LEN バイト, 最大 255) | CRC16 (リトルエンディアン)
```

- **CRC-16/CCITT-FALSE**（多項式 0x1021、初期値 0xFFFF）。計算範囲は TYPE から PAYLOAD の末尾まで。
- 全フィールドはリトルエンディアン。TYPE の方向規約: `0x01〜0x0F` = STM32 → Pi、`0x10〜0x1F` = Pi → STM32。
- 1 Mbps にしているのは、同じ基板内の短い配線で、フレーム 1 つの線上時間を短くして順番待ちをほぼ無くすため。
  STM32 側の受信リングバッファは 2048 バイトで、ポーリングが空いてもよい猶予は約 20 ms です（`app.c`）。

## 主なパケット

| TYPE | 名前 | 方向 | 周期 | 長さ | 内容 |
|---|---|---|---|---|---|
| 0x10 | `COMMAND` | Pi → STM32 | **100 Hz** | 15 | モード、フラグ（ARM・ブレーキ・自動停止…）、目標車速、目標舵角、加速度上限、舵角速度上限、制動トルク |
| 0x02 | `TELEMETRY` | STM32 → Pi | **100 Hz** | 84 | 車速、ヨーレート、舵角、各輪速度、IMU、電流、スリップ率、TC 上限、TV 比率、電圧、温度、超音波… |
| 0x01 | `LIDAR_SECTOR` | STM32 → Pi | 120 Hz | 69 | LiDAR 1 周（360 点）を 12 セクタ × 30 点に分けた 1 セクタ。角度は `sector_idx*30 + i` [deg] |
| 0x11 / 0x03 / 0x13 | `CONFIG_SET` / `CONFIG_ACK` / `CONFIG_GET` | 双方向 | 随時 | 6 / 7 / 2 | 調整パラメータの書き込み・確認応答（実際に適用された値を返す）・読み出し |
| 0x12 / 0x06 | `PING` / `PONG` | 双方向 | 5 Hz | 4 / 12 | 時刻同期 |
| 0x14 / 0x07 | `VERSION_REQ` / `VERSION` | 双方向 | 起動時 | 0 / 10 | プロトコル版数とファーム ID |
| 0x15 / 0x0A | `LIMITS_REQ` / `LIMITS` | 双方向 | 起動時 | 0 / 16 | 車両の物理上限（最高速、最大トルク、最大舵角…） |
| 0x08 | `STATS` | STM32 → Pi | 1 Hz | 48 | 受信フレーム数、CRC エラー数、MD ごとの受信数・エラー数 |
| 0x04 | `LOG` | STM32 → Pi | 随時 | 可変 | 重要度つきの文字列。優先度は最低 |

LiDAR にはほかに、強度付き（`LIDAR_SECTOR_I`）と距離 1 バイト圧縮（`LIDAR_SECTOR_C`）の変種があり、パラメータ `0x0040` で切り替わります。

STM32 は送信を優先度つきキューで並べます: **PONG > TELEMETRY > LIDAR > CONFIG_ACK 等 > LOG**（`CLAUDE.md`）。
PONG は最優先です（応答を送り始める時刻 T3 が遅れると時刻同期の誤差になるため、と思われます）。

### 回線の使用率（計算）

フレーム全体の長さは「LEN + 7」バイトです。

| 流れ | 計算 | 使用率 |
|---|---|---|
| STM32 → Pi | LiDAR 76 B × 120 Hz ＋ TELEMETRY 91 B × 100 Hz ＋ STATS ≒ 18.3 kB/s | 約 18 % |
| Pi → STM32 | COMMAND 22 B × 100 Hz ＋ PING ≒ 2.3 kB/s | 約 2 % |

TELEMETRY 1 フレームの線上時間は 0.91 ms、COMMAND は 0.22 ms です。

## 起動時のやりとり（ARM の前提）

Pi の `io_node` は、次の条件を**独立に**確かめるまで ARM を通しません（[上位の安全設計](../raspi/safety.md)）。

1. **`protocol_version` の一致。** 起動時に `VERSION_REQ` を送り、`VERSION` の版数が一致しなければ ARM しない。
   **Pi と STM32 はセットで入れ替える**前提です（片方だけ新しいと、フィールドの位置がずれて「なぜか値が化ける」ことになる）。
2. **`LIMITS` を受け取る。** 速度・舵角のクランプに使う。
3. **調整パラメータの同期**（下記）。

### 調整パラメータの同期

STM32 は調整パラメータを Flash に保存しません。電源を入れ直すと仮置き値に戻るため、Pi が責任を持ちます
（[`raspi/core/control_params.py`][control-params-py]）。

1. 接続したら**全項目**を `CONFIG_SET` で送り、`CONFIG_ACK.applied`（実際に入った値）が**送った値と一致するまで再送**する。
2. 一致した後も、**1 項目ずつ順番に `CONFIG_GET` で読み戻し続ける。** 食い違いを 1 つでも見つけたら、
   STM32 だけが再起動して仮置き値に戻ったと見て、全項目を送り直す。
3. STM32 の再起動を検出したら、最初からやり直す。

状態は `pending / ok / mismatch / unsupported` で、`unsupported` は STM32 が不明な ID を返した＝**ファームが古い**ことを意味します。
**同定の試験**（TC を切る、など）では、planner が書いた一時的な上書き値を、指令が届いている間だけ入れ、
届かなくなったら元に戻します（試験が異常終了しても TC を切ったままにならない）。

## 時刻同期

STM32 の `t_us` は 32 bit のマイクロ秒カウンタで、約 71.6 分でラップします。これを Pi の `CLOCK_MONOTONIC` に換算するのに、
NTP と同じ **4 つのタイムスタンプ**を使います（[`raspi/core/timesync.py`][timesync-py]）。

| 記号 | 意味 |
|---|---|
| T1 | Pi が PING を送った時刻（Pi の単調 ns） |
| T2 | STM32 が PING を受けた時刻（`PONG.t_ping_rx_us`、UART の IDLE 割り込みで取得） |
| T3 | STM32 が PONG を送り始めた時刻（`PONG.t_pong_tx_us`、送信開始割り込みで取得） |
| T4 | Pi が PONG を受けた時刻 |

$$
\text{往復遅延} = (T_4 - T_1) - (T_3 - T_2),\qquad
\text{対応点：}\ \tfrac{T_2+T_3}{2} \leftrightarrow \tfrac{T_1+T_4}{2}
$$

- 往復遅延が小さいサンプルほど信頼できます（待ち行列で伸びていない）。直近のウィンドウから遅延の小さい点を選び、
  **線形回帰で切片（オフセット）と傾き（ドリフト）を同時に**推定します。
- 回帰は PONG が来たときにだけ計算して持ち、点が一時的に足りなくなっても最後に成立した回帰で外挿します（**傾きを捨てない**）。
- 開発記録によれば、過去に「条件しだいで最大 178 ms 未来側へずれる」不具合があり、回帰を保持する形に直しました
  （修正後 0.36 ms 以下。`PROGRESS.md` 2026-10-14）。
- シミュレータの仮想 STM32 は、**Pi と独立の時計に、実機で実測したドリフト（+3378 ppm）を持たせています。**
  Pi の時計をそのまま返すと、時刻同期のコードが仕事をしているか確認できないためです。

指令を送ってから `cmd_seq_echo`（TELEMETRY に載る）が返るまでの**往復遅延は、Pi の時計だけで測れる**ので、時刻同期が収束していなくても値が出ます。

## ハートビートは UART とは別系統

UART の COMMAND 途絶（100 ms）とは別に、Pi の **GPIO6 が 100 Hz の矩形波**を出し、STM32 の **PB12（`RAS_SIG`）** がエッジ間隔を監視します。
50 ms 途切れると緊急停止がラッチされます（[安全層](../stm32/safety.md)）。

- Pi 側は専用スレッドで、100 Hz（5 ms ごとにトグル）の遅れを常時計測しています（`raspi/io/gpio.py` の `Heartbeat`）。
- **PB12 は超音波 `ECHO_FRONT`（PA12）と同じ EXTI ライン 12** で、外部割り込みが使えません。そのため STM32 側はポーリングです。

## パケット定義を 1 か所に閉じる

パケット定義を Pi 側と STM32 側で別々に手書きすると**必ずズレます。** 実際、v0.4 の策定中に `TELEMETRY` のフィールド順
（`odom_dist` と `accel` のどちらが先か）が食い違ったまま 2 往復した、と記録にあります（`raspi/proto/README.md`）。

そこで `protocol.toml` を唯一の正にして、`generate.py` が Python のパーサ（`packets.py`）と C ヘッダ（`surge_proto.h`）を生成します。
生成時に、フィールドの合計バイト数が `len` と一致するかなどを検証し、CI（`generate.py --check`）が再生成し忘れを検出します。

!!! warning "現状の限界"
    **確認した版のファームは、生成された `surge_proto.h` を取り込んでいません。**
    STM32 側は `ras_link.h` / `ras_link.c` を手書きで保守し、`RAS_PROTOCOL_VERSION` も手で合わせています。
    つまり、Pi 側の生成の仕組みは**ファームとのずれを機械的には防いでいません。**
    防いでいるのは実行時の `protocol_version` の照合（不一致なら ARM を拒否）です。
    ファーム側にも同じ生成物を取り込む、または版数と定数の一致を検査する、というのが次の一手になります。

    また、同じ仕様が Pi 側の `docs/uart_protocol.md` と STM32 側の `docs/pi_uart_protocol_v0.XX_delta.md` の二箇所に書かれており、
    **ファーム側の `docs/README.md` は v0.22 までしか触れていません**（コードは v0.23）。仕様の正はコードです。

[protocol-toml]: https://github.com/banbatakumi/AutonomousCar_RasPi/blob/master/surge_mk2/raspi/proto/protocol.toml
[ras-link-h]: https://github.com/banbatakumi/AutonomousCar/blob/FW/MainBoard_V3/Program/MainF446RE_V3/src/comm/ras_link.h
[ras-link-c]: https://github.com/banbatakumi/AutonomousCar/blob/FW/MainBoard_V3/Program/MainF446RE_V3/src/comm/ras_link.c
[control-params-py]: https://github.com/banbatakumi/AutonomousCar_RasPi/blob/master/surge_mk2/raspi/core/control_params.py
[timesync-py]: https://github.com/banbatakumi/AutonomousCar_RasPi/blob/master/surge_mk2/raspi/core/timesync.py
