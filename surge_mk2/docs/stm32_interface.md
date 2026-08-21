# SURGE Mark.2 — STM32 側 実装仕様書

**バージョン**: v0.11（`uart_protocol.md` **v0.11** に対応）
**最終更新**: 2026-08-22
**対象読者**: STM32 ファームウェアを実装する人
**関連文書**: [`uart_protocol.md`](uart_protocol.md)（プロトコルの正）, [`architecture.md`](architecture.md)（全体設計）

> **本書の位置づけ**
> `uart_protocol.md` が仕様の**正**。本書はそれを「STM32 側で何を実装すればよいか」の形に
> 落とし込んだもの。両者が食い違った場合は `uart_protocol.md` を優先し、本書を修正すること。

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

## 目次

1. [STM32 側の責務](#1-stm32-側の責務)
2. [UART の設定](#2-uart-の設定)
3. [フレーム構造と CRC](#3-フレーム構造と-crc)
4. [パケット定義（C 構造体）](#4-パケット定義c-構造体)
5. [送信の実装](#5-送信の実装)
6. [受信の実装](#6-受信の実装)
7. [LiDAR のビニング処理](#7-lidar-のビニング処理)
8. [安全要件](#8-安全要件)
9. [モードと原点合わせ](#9-モードと原点合わせ)
10. [パラメータの扱い](#10-パラメータの扱い)
11. [時刻同期で STM32 がやること](#11-時刻同期で-stm32-がやること)
12. [実装チェックリスト](#12-実装チェックリスト)
13. [立ち上げ・テスト手順](#13-立ち上げテスト手順)

---

## 1. STM32 側の責務

### やること

| 分類 | 内容 |
|---|---|
| **制御** | 後輪左右モータのトルク制御、ステアリングモータの位置制御、車速の速度制御ループ |
| **車両制御** | TC（トラクションコントロール）、トルクベクタリング ※**実装済み** |
| **センサ** | LD06 のパース・1°ビニング、超音波測距、前輪エンコーダ（ADC）、IMU、温度、電圧・電流 |
| **単位換算** | **モータ機械角 ⇄ 路面舵角**、**車輪角速度 → 周速 [m/s]**、**エンコーダ角 → 累積距離 [0.1mm]**、**`speed` の車体中心線方向への射影**（§4.3-補足） |
| **保護** | 電流リミッタ、温度リミッタ、低電圧警告、過電流ハード遮断のラッチ管理 |
| **安全** | UART タイムアウト時の自動ブレーキ、ハートビート監視（PB12） |
| **通信** | 本書のプロトコルによる送受信 |

### やらないこと

- 経路生成・自己位置推定・地図生成（すべて Raspberry Pi 側）
- 障害物の意味判断（「壁か人か」など）
- ステアリング原点の自動探索（→ §9）

### 機械定数は STM32 に閉じる

**Pi は車輪半径・ギア比・ステアリングのリンク比を一切知らない。**
UART 上の物理量はすべて SI 単位（m/s、rad、N·m、0.1mm）で定義されており、
換算はすべて STM32 側で行う。

この設計により、**機械定数を実測して値が変わっても Pi 側の改修が不要**になる。
**ステアリングのリンク比は 0.5、車輪半径は 0.03m で実測確定し**（2026-08-20。後輪は
ダイレクトドライブでギア比の概念が無いため、値が要るのはこの2つだけ）、STM32
ファームウェアの換算定数にも反映済み。`steer_actual` / `target_steer` / `speed` の
絶対スケールはもう暫定ではない。

### Pi から受け取る指令は3つだけ

```
target_speed   目標速度      [m/s]     ← PWM デューティでもトルクでもない
accel_limit    加速度の上限   [m/s²]    ← レートリミット
target_steer   目標舵角      [rad]     ← ★路面舵角。モータ機械角ではない
```

---

## 2. UART の設定

| 項目 | 値 |
|---|---|
| ボーレート | **250000 bps** |
| フォーマット | 8N1（8データビット / パリティなし / ストップ1ビット） |
| フロー制御 | なし |
| 実効スループット | 25.0 kB/s（1バイト = 10ビット） |
| 接続 | STM32 TX → Pi GPIO15 (RX) / STM32 RX ← Pi GPIO14 (TX) |
| 論理レベル | 3.3V（両者3.3V系のためレベル変換不要） |

### 分周誤差について — **誤差 0%**

```
USARTDIV = f_ck / (16 × baud)      （OVER8 = 0）

  APB1  f_ck = 45 MHz → USARTDIV = 11.25   ✓ 1/16 刻みで厳密に表現可能
  APB2  f_ck = 90 MHz → USARTDIV = 22.5    ✓
```

**分数部が 1/16 の整数倍で表せるため誤差 0%。** フレーミングエラーの懸念はない。

### バッファサイズの目安

| 用途 | 推奨サイズ | 根拠 |
|---|---|---|
| RX 循環 DMA バッファ | **512 B** | 25 B/ms なので約 20ms 分。IDLE 割り込みで随時取り出す |
| TX リングバッファ | **1024 B** | 最大フレーム 106 B（強度付き LiDAR）× 8フレーム分以上 |

---

## 3. フレーム構造と CRC

```
┌────────┬────────┬──────┬──────┬──────┬───────────┬────────────┐
│ 0xAA   │ 0x55   │ TYPE │ SEQ  │ LEN  │ PAYLOAD   │ CRC16      │
│ 1B     │ 1B     │ 1B   │ 1B   │ 1B   │ LEN bytes │ 2B         │
└────────┴────────┴──────┴──────┴──────┴───────────┴────────────┘
  0        1        2      3      4      5           5+LEN

オーバーヘッド : 7 バイト
CRC 計算範囲   : TYPE (offset 2) から PAYLOAD 末尾まで = LEN + 3 バイト
エンディアン   : 全フィールド リトルエンディアン（STM32 ネイティブ）
```

- `SEQ` は**送信方向ごとに独立**した 0-255 のローリングカウンタ。送信するたびに +1
- `SYNC` が2バイトなのは、1バイトだとバイナリデータ中に頻出して誤同期するため

### CRC-16/CCITT-FALSE

| パラメータ | 値 |
|---|---|
| 多項式 | `0x1021` |
| 初期値 | `0xFFFF` |
| 入力反転 / 出力反転 | なし / なし |
| 最終 XOR | `0x0000` |
| **検査値** | `"123456789"` → **`0x29B1`** |

```c
uint16_t crc16_ccitt(const uint8_t *data, size_t len)
{
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (int b = 0; b < 8; b++)
            crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021) : (uint16_t)(crc << 1);
    }
    return crc;
}
```

**実装したら必ず検査値 `0x29B1` で確認すること。** ここを間違えると通信が一切成立しない。

速度が問題になる場合は 256エントリのテーブル駆動版に置き換えてよい（結果は同一）。
STM32 内蔵の CRC ペリフェラルは多項式・初期値の設定が機種により制限されるため、
**まずはソフトウェア実装で動かすことを推奨**（最大 106 バイト × 130 回/秒なので負荷は無視できる）。

> MD との通信で使っている CRC-8/AUTOSAR とは**別物**。混同しないこと。
> MD 側の CRC は STM32 内部で完結しており、Pi 側とは無関係。

### フレーム送信ヘルパ

```c
// payload を type のフレームに包んで TX リングへ積む
// 戻り値: 成功 = true / TXリング満杯 = false
bool frame_send(uint8_t type, const void *payload, uint8_t len)
{
    static uint8_t tx_seq = 0;
    uint8_t buf[7 + 255];

    buf[0] = 0xAA;
    buf[1] = 0x55;
    buf[2] = type;
    buf[3] = tx_seq++;
    buf[4] = len;
    if (len) memcpy(&buf[5], payload, len);

    uint16_t crc = crc16_ccitt(&buf[2], (size_t)len + 3);
    buf[5 + len]     = (uint8_t)(crc & 0xFF);
    buf[5 + len + 1] = (uint8_t)(crc >> 8);

    return tx_ring_push(buf, (size_t)len + 7);
}
```

---

## 4. パケット定義（C 構造体）

### 4.1 共通事項

```c
#include <stdint.h>

#pragma pack(push, 1)
/* ... 構造体定義 ... */
#pragma pack(pop)
```

**必ず `#pragma pack(push, 1)`（または `__attribute__((packed))`）を付けること。**
付け忘れるとコンパイラがパディングを挿入し、Pi 側とバイト配置がずれる。

> **アンアライン アクセスについて**
> **ペイロードはフレーム先頭から 5バイト目に始まるため、`uint32_t` / `int32_t` は
> 4バイト境界に乗らない。**
> - packed 構造体のメンバに**直接**アクセスする分には、コンパイラがバイト単位の
>   アクセスコードを生成するため安全。ただし速度が落ちるので、頻繁に触る値は
>   一度ローカル変数に取り出すこと
> - **非 packed 構造体へのポインタキャストは未定義動作。**
>   `LDM`/`STM` 系はアンアラインを許容せず、実際にハードフォールトしうる。
>   受信フレームから構造体を作るときは必ず `memcpy` を使う

### 4.2 パケット種別

```c
/* STM32 → Pi */
#define PKT_LIDAR_SECTOR    0x01   /* LEN = 69                              */
#define PKT_TELEMETRY       0x02   /* LEN = 66  ★v0.4 で 53→66             */
#define PKT_CONFIG_ACK      0x03   /* LEN = 7                               */
#define PKT_LOG             0x04   /* LEN = 可変 (2-255)  ★severity 追加    */
#define PKT_LIDAR_SECTOR_I  0x05   /* LEN = 99  （強度付き・既定OFF）        */
#define PKT_PONG            0x06   /* LEN = 12                              */
#define PKT_VERSION         0x07   /* LEN = 10  ★v0.4 追加                  */
#define PKT_STATS           0x08   /* LEN = 48  ★v0.4 追加                  */
#define PKT_LIDAR_SECTOR_C  0x09   /* LEN = 39 （圧縮・既定OFF）★v0.4 追加  */
#define PKT_LIMITS          0x0A   /* LEN = 16  ★v0.11 追加                 */

/* Pi → STM32 */
#define PKT_COMMAND         0x10   /* LEN = 14  ★v0.5 で 10→12、v0.6 で 12→14 */
#define PKT_CONFIG_SET      0x11   /* LEN = 6                               */
#define PKT_PING            0x12   /* LEN = 4                               */
#define PKT_CONFIG_GET      0x13   /* LEN = 2   ★v0.4 追加                  */
#define PKT_VERSION_REQ     0x14   /* LEN = 0   ★v0.4 追加                  */
#define PKT_LIMITS_REQ      0x15   /* LEN = 0   ★v0.11 追加                 */
```

**方向規約: `0x01-0x0F` = STM32 → Pi、`0x10-0x1F` = Pi → STM32。**
**LEN は TYPE ごとに一意**（`LOG` を除く）。この2つを守ることで、
受信側は TYPE を見た時点で方向も長さも確定でき、ロジックアナライザでの切り分けも容易になる。

**LiDAR パケットは 0x01 / 0x05 / 0x09 のいずれか1種類だけを送る**（同時送信はしない）。
どれを使うかは `param_id 0x0040`（enum）で決まる（§10.1）。

### 4.3 送信するパケット

#### `LIDAR_SECTOR` (0x01) — 69 バイト

```c
typedef struct {
    uint8_t  sector_idx;      /* 0-11                                       */
    uint32_t t_start_us;      /* このセクタ先頭点の取得時刻 [μs]             */
    uint16_t duration_us;     /* ★このセクタ30点の取得に要した時間 [μs]      */
    uint16_t rot_speed_dps;   /* ★0.01 deg/s。LD06 が報告する回転速度        */
    uint16_t dist[30];        /* mm。0 = 無効 / 範囲外                       */
} lidar_sector_t;             /* = 69 bytes */
```

**`angle_start` / `angle_step` は v0.4 で廃止した。**
1°ビンにビニングした時点で角度は 1° 格子に量子化されており、実測値を送っても精度が残らない
（v0.3 の内部矛盾）。角度は Pi 側が `sector_idx * 30 + i` [degree] で一意に決める。

代わりに `duration_us` を送ることで、**セクタを1個落としても Pi 側の点群歪み補正が壊れない**。

#### `LIDAR_SECTOR_I` (0x05) — 99 バイト（`param 0x0040` で有効化時のみ）

```c
typedef struct {
    uint8_t  sector_idx;
    uint32_t t_start_us;
    uint16_t duration_us;
    uint16_t rot_speed_dps;
    uint16_t dist[30];
    uint8_t  intensity[30];   /* LD06 の信頼度をそのまま転送 */
} lidar_sector_i_t;           /* = 99 bytes */
```

#### `LIDAR_SECTOR_C` (0x09) — 39 バイト（`param 0x0040 = 2` のときのみ）

```c
typedef struct {
    uint8_t  sector_idx;
    uint32_t t_start_us;
    uint16_t duration_us;
    uint16_t rot_speed_dps;
    uint8_t  dist[30];        /* 2 cm/LSB。0 = 無効、255 = 5.10m 以上（飽和） */
} lidar_sector_c_t;           /* = 39 bytes */
```

**`255` は「5.10m ちょうど」ではなく「5.10m 以上」**である。Pi 側は 255 を実測点として
占有格子に打たず、「5.1m まで障害物なし」としてのみ扱う（`uart_protocol.md` §5.2）。

帯域を 9.12 → 5.52 kB/s に落とす圧縮版。LD06 の測距精度が ±1.5cm なので
2cm 量子化による損失はほぼ無く、代償は 5.1m 以遠の飽和だけ。

```c
/* mm → 2cm/LSB への変換（丸めと飽和） */
static inline uint8_t compress_dist(uint16_t mm)
{
    if (mm == 0) return 0;                 /* 無効はそのまま 0 */
    uint32_t v = ((uint32_t)mm + 10) / 20; /* 四捨五入 */
    if (v == 0) v = 1;                     /* 0 に潰さない（無効と区別する） */
    return (v > 255) ? 255 : (uint8_t)v;
}
```

**`mm != 0` なのに変換結果が 0 になると「無効」と誤解される**ため、1 にクリップする。

#### `TELEMETRY` (0x02) — 66 バイト ★全面改訂

```c
typedef struct {
    uint32_t t_us;                 /* STM32 monotonic μs（71.6分でラップ）        */
    uint32_t flags;                /* §8.5 参照。★u16 → u32 に拡張               */
    int16_t  speed;                /* 0.001 m/s  ★車体中心線方向に射影済み        */
    int16_t  yaw_rate;             /* 0.001 rad/s                                */
    int16_t  steer_actual;         /* 0.0001 rad ★路面舵角に換算済み              */
    int16_t  steer_cmd_echo;       /* 0.0001 rad 最後に受理した指令値（路面舵角）  */
    int16_t  wheel_speed[4];       /* 0.001 m/s  周速 [FL, FR, RL, RR] 射影なし   */
    int32_t  odom_dist[2];         /* 0.1 mm 累積走行距離 [FL, FR] 射影なし        */
    int16_t  accel_x;              /* 0.001 m/s²                                 */
    int16_t  accel_y;              /* 0.001 m/s²                                 */
    int16_t  accel_z;              /* 0.001 m/s²                                 */
    int16_t  pitch;                /* 0.0001 rad                                 */
    int16_t  roll;                 /* 0.0001 rad                                 */
    int16_t  motor_current[3];     /* mA  [RL, RR, ST] 双方向・制動時は負          */
    int16_t  torque_cmd[2];        /* 0.0001 N·m [RL, RR] TC適用後の最終指令値     */
    uint8_t  temp[4];              /* 1 degC [RL, RR, ST, MCU] ★符号なし          */
    uint8_t  batt_voltage_drive;   /* 0.05 V/LSB  駆動系                          */
    uint8_t  batt_voltage_signal;  /* 0.05 V/LSB  シグナル系                      */
    uint8_t  batt_current_drive;   /* 0.05 A/LSB  駆動系（単方向）                 */
    uint8_t  batt_current_signal;  /* 0.02 A/LSB  シグナル系（単方向）             */
    uint8_t  us_front;             /* 2 cm/LSB。0 = 無効                          */
    uint8_t  us_rear;              /* 2 cm/LSB。0 = 無効                          */
    uint8_t  md_status[3];         /* §8.6 参照 [RL, RR, ST]                      */
    uint8_t  cmd_seq_echo;         /* 最後に受理した COMMAND の SEQ                */
} telemetry_t;                     /* = 66 bytes */
```

##### 配列インデックスの規約

**`wheel_speed` / `odom_dist` は `FL → FR → RL → RR`、モータ系は `RL → RR → ST → (MCU)`。**

| index | `wheel_speed` | `odom_dist` | `motor_current` | `torque_cmd` | `temp` | `md_status` |
|---|---|---|---|---|---|---|
| 0 | 前左 (FL) | 前左 (FL) | 後左 (RL) | 後左 (RL) | MD 後左 | MD 後左 |
| 1 | 前右 (FR) | 前右 (FR) | 後右 (RR) | 後右 (RR) | MD 後右 | MD 後右 |
| 2 | 後左 (RL) | — | ステア (ST) | — | MD ステア | MD ステア |
| 3 | 後右 (RR) | — | — | — | STM32 (MCU) | — |

**`odom_dist` は前輪のみ 2本。** `wheel_speed` の index 0/1 と対応する。

##### 射影の責任分担 ★重要

| 値 | 射影 | 担当 |
|---|---|---|
| `speed` | **車体中心線方向に射影済み** | **STM32** |
| `wheel_speed[FL], [FR]` | **射影なし**（車輪自身の軌跡に沿った周速） | Pi 側が `cos(δ)` を掛ける |
| `odom_dist[FL], [FR]` | **射影なし**（車輪自身の軌跡に沿った距離） | Pi 側が**差分に対して**射影 |
| `wheel_speed[RL], [RR]` | 後輪は非操舵輪なので射影不要 | — |

前輪は操舵輪なので、舵角 δ のとき走行距離は車体中心線距離の **1/cos(δ) 倍**になる
（δ=30° で +15.5%、δ=60° で +100%）。**射影済みなのは `speed` だけ**である。

STM32 は `speed` を作るときに `cos(steer_actual)` を掛けること。
`wheel_speed` / `odom_dist` は**生値のまま送る**（Pi 側がスリップ計算に生値を必要とするため）。

##### 実装時の注意

- **`speed` / `wheel_speed` は帯域 1〜2Hz のローパスを通した値を入れる。**
  前輪はアナログ絶対角エンコーダの ADC 読みで、角速度換算すると 1LSB ≈ 1.5 rad/s 相当の
  ノイズが乗る。生値を送ると Pi 側が使えない。
  Pi 側には「この値を微分して加速度を求めてはならない」と明記済み。
- **`odom_dist` はフィルタ前の絶対角アンラップから積算する。**
  速度と違ってローパスをかけない（積算値なので位相遅れがそのまま距離誤差になる）。
  0.1mm 単位・`int32_t` のラップは **±214.7 km**（u32 全域で 429.5 km）なので、
  ミニカーの走行距離では**到達しない**。`t_us` と同格の注意事項ではない。
- **`torque_cmd` は MD に出した指令値**（TC 適用後の最終値）であって実測ではない。
  MD が返すのは `iq [mA]` のみで、Kt が未実測のため実トルクは算出できない。
  スケールは **0.0001 N·m/LSB**（最大トルク 0.1557 N·m なので 0.001 刻みでは分解能不足）。
- **`temp` は符号なし。** MD が `uint8_t [degC]` で返すのでそのまま入れる。
  `temp[3]`（MCU 内蔵温度センサ）は**工場較正なし・±10℃級・ダイ温度**なので、
  絶対値でのしきい値判定には使えない。傾向監視のみ。
- **`us_front` / `us_rear` は実質 20Hz 更新。** 50Hz の TELEMETRY に対して遅いので、
  同じ値が連続して入る。HC-SR04 の最小測距が 2cm あるため **0 を「無効」に使って安全**。
- **`batt_current_*` は単方向**（INA180A2 + シャント、GND 基準）。回生時は 0 に張り付く。
  `motor_current`（MD の `iq`）は双方向。**この非対称は意図的**。

##### `steer_actual` / `steer_cmd_echo` を両方送る理由

Pi 側で**アクチュエータ遅延を実測**するため。この2つが同じパケットに入っていれば、
差分を取るだけで遅延特性（1次遅れ + むだ時間）が同定でき、別ログの時刻合わせが不要になる。

**`steer_actual` は MD が返す機械角から換算した路面舵角**であり、指令値のコピーであってはならない。

##### `cmd_seq_echo` の意味

**最後に「受理して制御に反映した」`COMMAND` の SEQ**（単に受信しただけのものではない）。
Pi 側はこれで往復遅延を測る。

#### `PONG` (0x06) — 12 バイト

```c
typedef struct {
    uint32_t ping_id;        /* 受信した PING の ping_id をそのままエコー          */
    uint32_t t_ping_rx_us;   /* ★UART IDLE 割り込みで、PING 最終バイト到着の瞬間  */
    uint32_t t_pong_tx_us;   /* ★送信開始割り込みで、1バイト目が線に出た瞬間      */
} pong_t;                    /* = 12 bytes */
```

**タイムスタンプの取得タイミングが精度に直結する（§11）。**

#### `VERSION` (0x07) — 10 バイト ★v0.4 追加

```c
typedef struct {
    uint16_t protocol_version;  /* 上位=メジャー 下位=マイナー。v0.5 → 0x0005 */
    uint32_t fw_id;             /* git short hash 等                        */
    uint32_t build_epoch;       /* ビルド時刻（UNIX epoch 秒）               */
} version_t;                    /* = 10 bytes */
```

- **起動直後に 3回（100ms 間隔）自発送信する。**
  ただしこれは補助。Pi の起動が遅れていれば無駄になるため、
  **Pi はリンク確立時に必ず `VERSION_REQ` (0x14) を送る**。
- `VERSION_REQ` (0x14, LEN = 0) を受信したら、即座にこれを返す。
  **要求と応答で TYPE を分けている**のは、同一 TYPE で LEN が変わるのを避けるためと、
  `0x01-0x0F` = STM32→Pi の方向規約を守るため。
- **ファーム更新と Pi 側コードの食い違いは必ず起きる。** バージョンを交換しないと
  「原因不明のバグ」に化けて何時間も溶かすので、必須。

#### `LIMITS` (0x0A) — 16 バイト ★v0.11 追加

```c
typedef struct {
    float max_speed_m_s;    /* target_speed の上限（DRIVE_MAX_SPEED_M_S）        */
    float max_accel_m_s2;   /* accel_limit に指定できる上限（DRIVE_MAX_ACCEL_M_S2）*/
    float max_torque_nm;    /* 1輪あたり。target_torque/brake_torque 共通          */
    float max_steer_rad;    /* 路面舵角の上限（片側振れ幅、Steering_GetMaxRoadWheelAngleRad()）*/
} limits_t;                  /* = 16 bytes */
```

- **読み取り専用。** `CONFIG_SET` のような書き込み経路は無い（v0.10 で
  `MAX_SPEED`/`MAX_ACCEL`/`MAX_STEER` の `param_id` 自体を廃止したのと表裏）
- `VERSION` と同じく**起動直後に 3回（100ms 間隔）自発送信**し、
  `LIMITS_REQ` (0x15, LEN = 0) を受信したら即座にこれを返す
- **実行時に変化しない値**（走行中に上限が変わる想定は無い）なので、送信頻度はこれで十分
- v0.10 で `MAX_SPEED`/`MAX_ACCEL`/`MAX_STEER` の `CONFIG_SET`/`CONFIG_GET` を廃止した
  結果、Pi が STM32 の実際の固定上限値を知る手段が無くなっていた。それを埋めるための
  パケット（`uart_protocol.md` §5.12）

#### `STATS` (0x08) — 22 バイト ★v0.4 追加

```c
typedef struct {
    uint32_t t_us;              /* このスナップショットの時刻                   */
    uint32_t rx_frame_ok;       /* Pi → STM32 で正常受理したフレーム数          */
    uint32_t rx_crc_error;      /* CRC 不一致で破棄した数                       */
    uint32_t rx_len_error;      /* LEN が TYPE の期待値と不一致                 */
    uint32_t rx_unknown_type;   /* 未知の TYPE                                 */
    uint32_t tx_drop;           /* 送信キュー溢れで破棄した数（主に LOG）        */
    uint32_t md_rx_count[3];    /* MD 状態フレームの正常受信数 [RL, RR, ST]     */
    uint32_t md_rx_error[3];    /* MD 状態フレームの CRC 不一致数 [RL, RR, ST]  */
} stats_t;                      /* = 48 bytes */
```

**すべて「起動からの累積値」。0 クリアしない。**

増分方式（本書 v0.2 の案）は **`STATS` が1個ロスするとその1秒分のエラー件数が永久に失われる**。
これは `odom_dist` を累積で送るのとまったく同じ理屈であり、Pi 側の当初案が誤りだった。
`u32` なら 1000件/s でも 49日かかるため実質ラップせず、「u16 のラップを避ける」という
当初の目的も同時に達成される。

**Pi 側は連続する2つの `STATS` の差を `t_us` の差で割ってレートを出す。**
途中でロスがあっても平均レートとしては正しい値が得られる。

**目的は「上下どちらの線が悪いか」の切り分け。** Pi 側のカウンタだけでは
STM32 → Pi の品質しか分からない。

> **Pi 側からの任意提案**: 末尾に `uint32_t lidar_frame_error`（LD06 のパース/チェックサム
> エラー数）を足して LEN = 52 にしたい。`lidar_ok` は「300ms 完全に途絶したか」しか
> 見ておらず、**「たまにフレームを落としている」状態が見えない**ため。
> 必須ではないので、実装負荷が気になるなら見送ってよい。

#### `CONFIG_ACK` (0x03) — 7 バイト

```c
typedef struct {
    uint16_t param_id;
    float    applied;        /* 実際に適用された値（クランプ後）              */
    uint8_t  result;         /* 0=OK 1=不明ID 2=範囲外 3=変更不可              */
} config_ack_t;              /* = 7 bytes */
```

`CONFIG_SET` / `CONFIG_GET` のどちらに対しても、必ずこれを返す。

#### `LOG` (0x04) — 2〜255 バイト

```c
/* [severity : uint8_t][message : char[]]  — NUL 終端なし、改行なし */
#define LOG_DEBUG  0
#define LOG_INFO   1
#define LOG_WARN   2
#define LOG_ERROR  3
```

**`severity` は v0.4 で追加。** これが無いと Pi 側でフィルタできず実用にならない。
**最低優先度**で送信し、TX リングが逼迫したら**破棄してよい**
（破棄数は `stats_t.tx_drop_count` に計上すること）。

### 4.4 受信するパケット

#### `COMMAND` (0x10) — 14 バイト ★v0.6 で 12→14（実装・実車確認済み）

```c
typedef struct {
    uint8_t  mode;              /* 0=DISARM 1=MANUAL 2=AUTO （3 は予約・来ない） */
    uint8_t  flags;             /* 下表 ★v0.6 で bit6、v0.7 で bit7 追加         */
    int16_t  target_speed;      /* 0.001 m/s  （負 = 後退）★brake/torque_mode 中は無視 */
    int16_t  target_steer;      /* 0.0001 rad ★路面舵角（反時計回り正 = 左）     */
    uint16_t accel_limit;       /* 0.001 m/s²                                    */
    uint16_t steer_rate_limit;  /* 0.001 rad/s                                   */
    uint16_t brake_torque;      /* 0.0001 N·m 後輪各輪。0=未指定(最大) ★v0.5     */
    int16_t  target_torque;     /* 0.0001 N·m 駆動トルク直接指令（負=後退方向）★v0.6 */
} command_t;                    /* = 14 bytes */
```

`flags` (u8):

| bit | 名称 | 意味 |
|---|---|---|
| 0 | `arm` | 駆動電源投入を要求 |
| 1 | `brake` | ブレーキ（強さは `brake_torque`）。**`torque_mode` より優先** |
| 2 | `horn` | クラクション。**立てている間ずっと鳴る** |
| 3-4 | `light_mode` | 0=OFF / 1=DAYTIME / 2=NORMAL（3 は予約・Pi は送らない） |
| 5 | `passing` | パッシング（前照灯のみ全光量。尾灯は連動しない） |
| 6 | `torque_mode` | 立っている間 `target_speed` を無視し `target_torque` を直接掛ける ★v0.6 |
| 7 | `auto_stop` | 立っている間、進行方向の超音波 20cm 未満で自動的に最大制動 ★v0.7 |

- `target_speed` は**目標速度**であり目標加速度ではない。`accel_limit` は
  「その目標速度へ向かうときの加減速の上限」
- **`target_steer` は路面舵角。** リンク比を掛けてモータ機械角に変換するのは STM32 の仕事
- **`mode` は Pi 側の要求であり、STM32 は拒否できる。** 実際に採用したモードは
  `TELEMETRY.flags` **bit0-1** で返る（v0.3 の bit8-9 から移動）
- `steer_rate_limit` は角度制御ループの追従限界を超える指令によるオーバーシュート・発振を防ぐ
- **`brake` 中は車速 PI を迂回し、`brake_torque` を直接掛ける**（★v0.5）。
  `brake_torque = 0` は「制動 0」ではなく「未指定 = 最大制動」。上限 **0.125 N·m/輪**
  （実車確認後に 0.075 から引き上げ）
- **`torque_mode` 中は車速 PI を迂回し、`target_torque` を各輪へ直接掛ける**（★v0.6）。
  `brake` と同時に立っていたら `brake` を優先すること。`target_torque` の `0` は
  「未指定」ではなく素直に「駆動トルク 0」。上限は **0.125 N·m**（Pi 側でもクランプ済み。
  モータ物理上限 0.1557 N·m 未満。実車確認後に 0.1 から引き上げ）。**TC/TV との関係は未確定**
  （そのまま通すか、空転抑制のため減衰させるか。`uart_protocol.md` §14 #8）
- **`auto_stop` 中は進行方向の超音波だけを見る**（★v0.7）。`torque_mode` が立っていなければ
  `target_speed`、立っていれば `target_torque` の符号で向きを決め、`>= 0` なら前方センサ、
  負なら後方センサ**だけ**を見る（逆方向は見ないので後退で抜けられる）。検知不能
  （`ULTRASONIC_NO_ECHO`・範囲外）では作動しない。`brake` が同時なら `brake` 優先で、
  `auto_stop` 単独時は `brake_torque` を見ず常に最大制動。**ラッチせず**、離れれば自動解除。
  介入中は `TELEMETRY.flags` bit16 を立てること
- **`COMMAND` が 100ms 途絶したら自動ブレーキ**（v0.3 の 200ms から短縮。§8.2）。
  ★v0.5 では最大制動トルクを直接掛け、`horn` と `passing` は強制解除する

#### `CONFIG_SET` (0x11) — 6 バイト / `CONFIG_GET` (0x13) — 2 バイト

```c
typedef struct {
    uint16_t param_id;
    float    value;
} config_set_t;                 /* = 6 bytes */

typedef struct {
    uint16_t param_id;          /* 現在値を CONFIG_ACK の applied で返す */
} config_get_t;                 /* = 2 bytes */
```

**いずれも受信したら必ず `CONFIG_ACK` を返す**（不明な `param_id` でも `result=1` で返す）。

#### `PING` (0x12) — 4 バイト

```c
typedef struct {
    uint32_t ping_id;
} ping_t;                       /* = 4 bytes */
```

#### `VERSION_REQ` (0x14) — 0 バイト

ペイロードなし（LEN = 0）。受信したら `version_t` を `PKT_VERSION` (0x07) で返す。

**受信ステートマシンが `LEN = 0` で止まらないことを必ず確認すること**（§6.2）。

#### `LIMITS_REQ` (0x15) — 0 バイト ★v0.11 追加

ペイロードなし（LEN = 0）。受信したら `limits_t` を `PKT_LIMITS` (0x0A) で返す。
`VERSION_REQ` と同じ扱いでよい（`LEN = 0` の受信経路を共有できる）。

### 4.5 コンパイル時にサイズを検証する

```c
_Static_assert(sizeof(lidar_sector_t)   == 69, "lidar_sector_t size mismatch");
_Static_assert(sizeof(lidar_sector_i_t) == 99, "lidar_sector_i_t size mismatch");
_Static_assert(sizeof(lidar_sector_c_t) == 39, "lidar_sector_c_t size mismatch");
_Static_assert(sizeof(telemetry_t)      == 66, "telemetry_t size mismatch");
_Static_assert(sizeof(pong_t)           == 12, "pong_t size mismatch");
_Static_assert(sizeof(limits_t)         == 16, "limits_t size mismatch");
_Static_assert(sizeof(version_t)        == 10, "version_t size mismatch");
_Static_assert(sizeof(stats_t)          == 48, "stats_t size mismatch");
_Static_assert(sizeof(config_ack_t)     ==  7, "config_ack_t size mismatch");
_Static_assert(sizeof(command_t)        == 10, "command_t size mismatch");
_Static_assert(sizeof(config_set_t)     ==  6, "config_set_t size mismatch");
_Static_assert(sizeof(config_get_t)     ==  2, "config_get_t size mismatch");
_Static_assert(sizeof(ping_t)           ==  4, "ping_t size mismatch");
```

**必ず入れること。** `#pragma pack` の付け忘れをビルド時に検出できる。
これが無いと「なぜか Pi 側で値が化ける」という原因の分かりにくいバグに数時間溶かす。

---

## 5. 送信の実装

### 5.1 送信スケジュール

| パケット | 送信タイミング |
|---|---|
| `PONG` | `PING` 受信時に即座（**最優先・キューの先頭に割り込ませる**） |
| `TELEMETRY` | 20ms 周期（50Hz） |
| `LIDAR_SECTOR` | セクタ（30点）が揃うたび。10Hz 回転なら約 8.3ms ごと |
| `CONFIG_ACK` | `CONFIG_SET` / `CONFIG_GET` 受信時 |
| `VERSION` | 起動直後 3回（100ms 間隔）+ `VERSION_REQ` 受信時 |
| `LIMITS` | 起動直後 3回（100ms 間隔、`VERSION` と同じタイミング）+ `LIMITS_REQ` 受信時 ★v0.11 |
| `STATS` | 1000ms 周期（1Hz）。**カウンタは 0 クリアしない**（累積値） |
| `LOG` | 任意。TX リングに空きがあるときのみ |

### 5.2 送信優先度

```
高  PONG  >  TELEMETRY  >  LIDAR_SECTOR  >  CONFIG_ACK  >  VERSION  >  LIMITS  >  STATS  >  LOG  低
```

`LOG` は TX リングの使用率が 50% を超えたら破棄する（`tx_drop_count++`）。

**`PONG` を最優先にする理由（v0.4 で訂正）**:
v0.3 は「min filter に採用されにくくなる」と説明していたが、これは理由として不十分。
実際には、**`t_pong_tx_us` を送信開始時点で取ったとしても、PONG が LiDAR セクタの後ろに
並ぶと T3 以降の経路に非対称遅延が生じ、オフセット推定そのものに最大 1.5ms の
バイアスが乗る**（LiDAR セクタ 1個の送信に 3.04ms かかるため）。

### 5.3 TX リング + DMA

**ブロッキング送信（`HAL_UART_Transmit`）を使ってはいけない。**
最大フレーム 106 B の送信に 4.2ms かかり、その間 制御ループが止まる。

```
frame_send() ──▶ [TX リングバッファ 1024B] ──▶ DMA ──▶ USART
                        ▲
              DMA 完了割り込みで次のチャンクを起動
```

実装パターン:

1. `frame_send()` はリングに積むだけで即座に戻る
2. DMA が動いていなければ、リングの連続領域を DMA 起動
3. `HAL_UART_TxCpltCallback()` でリングの読み出しポインタを進め、残りがあれば再起動

**`PONG` だけは通常のリング末尾ではなく、先頭に割り込ませる専用パスを用意する。**
（送信中の DMA を中断する必要はない。「次に送るもの」を PONG にすればよい）

> **現行の `Serial_Write` は使えない。** 送信中の DMA を中断してしまうため、
> フレームが途中で切れる。**送信キュー付きのドライバを新規に実装すること。**

#### 送信待ち時間は 3.04ms で有界

優先度キューがあり、かつ**送信中の DMA フレームを中断しない**ため、
`TELEMETRY` が LiDAR に阻まれる最大待ち時間は**送信中の1フレーム分**である。

```
76 B × 10 bit ÷ 250 kbps = 3.04 ms   （LiDAR セクタ1フレーム）
+ PONG が割り込む場合    ≈ 0.76 ms
─────────────────────────────────
最大でも約 4 ms
```

これは Pi 側の2段階タイムアウト（§8.2）の根拠に関わる。
**「LiDAR バーストで上りが数十ms滞留する」という状況は発生しない。**

### 5.4 帯域の確認

既定構成（LiDAR は `0x01`、強度・圧縮ともに OFF）:

| パケット | フレーム長 | 頻度 | 帯域 |
|---|---|---|---|
| `LIDAR_SECTOR` | 76 B | 120 Hz | 9.12 kB/s |
| `TELEMETRY` | **73 B** | 50 Hz | **3.65 kB/s** |
| `PONG` | 19 B | 5 Hz | 0.10 kB/s |
| `STATS` | 55 B | 1 Hz | 0.06 kB/s |
| `CONFIG_ACK` / `LOG` / `VERSION` | — | 散発 | 約 0.2 kB/s |
| **合計** | | | **約 13.1 kB/s（上限 25.0 kB/s の 52%）** |

**目標の 50% をわずかに超える。** 上りの 70% を LiDAR が占めるため、
テレメトリを削っても本質的な改善にはならない。

削減が必要になったら、**`LIDAR_SECTOR_C` (0x09) への切り替えが第一候補**
（13.1 → 9.5 kB/s = 38%）。`param_id 0x0040 = 2` で Pi 側から**走行中でも**切り替えられる。

強度 ON（`0x05`）は +3.6 kB/s で 16.7 kB/s（67%）になるため、地図生成の実験時など
短時間の用途に限定する。

---

## 6. 受信の実装

### 6.1 DMA + IDLE ライン検出

STM32 の定番パターンを使う。

```c
HAL_UARTEx_ReceiveToIdle_DMA(&huart, rx_dma_buf, sizeof(rx_dma_buf));
__HAL_DMA_DISABLE_IT(&hdma_rx, DMA_IT_HT);   /* Half-Transfer 割り込みは不要なら止める */
```

`HAL_UARTEx_RxEventCallback()` で受信バイト数が渡されるので、それをアプリ側の
リングバッファへ積み、メインループでフレーム抽出する。

**割り込みハンドラ内でフレーム処理まで行わないこと。** 積むだけにして、パース以降は
メインループか低優先タスクで行う。

> **例外: `t_ping_rx_us` だけは IDLE 割り込み内でタイムスタンプを取る。**
> メインループで取ると、ループ周期分のジッタがそのままクロックオフセット推定の
> バイアスになる。「IDLE 割り込みが入った時刻」を毎回記録しておき、
> パース結果が PING だったらその時刻を使う、という実装でよい。

### 6.2 フレーム抽出のステートマシン

```
    ┌──────────────┐
    │ WAIT_SYNC0   │ 0xAA を探す
    └──────┬───────┘
           ▼ 0xAA
    ┌──────────────┐
    │ WAIT_SYNC1   │ 0x55 なら次へ / 0xAA なら留まる / 他なら WAIT_SYNC0 へ戻る
    └──────┬───────┘
           ▼ 0x55
    ┌──────────────┐
    │ READ_HEADER  │ TYPE, SEQ, LEN を読む
    └──────┬───────┘   LEN が TYPE の期待値と違えば破棄して WAIT_SYNC0 へ
           ▼
    ┌──────────────┐
    │ READ_PAYLOAD │ LEN バイト読む（LEN = 0 なら即 READ_CRC へ）
    └──────┬───────┘
           ▼
    ┌──────────────┐
    │ READ_CRC     │ CRC 一致 → dispatch / 不一致 → 破棄、WAIT_SYNC0 へ
    └──────────────┘
```

**`WAIT_SYNC1` で `0xAA` を受けたときに `WAIT_SYNC1` に留まる**のがポイント。
`0xAA 0xAA 0x55` のような並びを取りこぼさない。

**`VERSION_REQ` (0x14) は LEN = 0 で来る。** `READ_PAYLOAD` を 0 バイトで通過する経路を
必ず用意すること（ここでハングする実装をよく見る）。

### 6.3 受信側の LEN 期待値

| TYPE | 期待 LEN |
|---|---|
| `0x10` `COMMAND` | 14 |
| `0x11` `CONFIG_SET` | 6 |
| `0x12` `PING` | 4 |
| `0x13` `CONFIG_GET` | 2 |
| `0x14` `VERSION_REQ` | **0** |
| `0x15` `LIMITS_REQ` | **0** ★v0.11 |

上記以外の TYPE は「未知」として `LEN` 分読み飛ばす。
**`0x01-0x0F` が受信側に来ることはない**（方向規約違反なので `unknown_type` に計上する）。

### 6.4 エラー処理と統計

| 事象 | 対応 |
|---|---|
| CRC 不一致 | 破棄。`crc_error_count++`。`WAIT_SYNC0` へ戻る |
| `LEN` が TYPE の期待値と不一致 | 破棄。`len_error_count++` |
| 未知の `TYPE` | `LEN` を信用して読み飛ばす。`unknown_type_count++` |
| USART の ORE / FE / NE エラー | フラグをクリアし DMA を再起動 |

**`HAL_UART_ErrorCallback()` で DMA 受信を必ず再起動すること。**
オーバーラン（ORE）で受信が止まったまま気づかない、というのが最も多い失敗。

統計は **`STATS` パケット (0x08) で 1Hz** で Pi へ送る（v0.3 の「将来 DIAG を追加」を実装）。
**カウンタは累積値であり、送信後に 0 クリアしない。**

---

## 7. LiDAR のビニング処理

### 7.1 やること

LD06 の生データ（約 450点/周）を **1° 刻みの 360点**に整形し、30点（= 30°）ごとに
`LIDAR_SECTOR` として送信する。

```
LD06 生パケット ──▶ 角度補間 ──▶ 1° ビンへ集約 ──▶ 30点揃ったらセクタ送信
```

**角度はセンサ基準のまま送る。左右の鏡像反転は Pi 側で行う**（`uart_protocol.md` §5.1）。
LD06 を裏向きに取り付けている都合で車両座標とは回転向きが逆になるが、STM32 側で直すと
0°/180° がセクタ境界に乗って1セクタが分裂するため、補正は受信側に寄せてある。

### 7.2 角度の補間

LD06 のパケットには `start_angle` と `end_angle` が入っており、中間の N 点は等間隔と仮定して補間する。

```c
/* N 点、start→end（0.01° 単位、ラップあり） */
uint16_t span = (uint16_t)(end_angle - start_angle);   /* 符号なし減算でラップ処理 */
for (int i = 0; i < N; i++) {
    uint16_t angle = (uint16_t)((start_angle + (uint32_t)span * i / (N - 1)) % 36000);
    ...
}
```

### 7.3 ビンへの集約 ★v0.4 で変更

**同一ビンに複数点が入った場合は「ビン中心に最も近い点」を代表点として採用する。**

```c
int bin = (angle + 50) / 100;            /* 0.01° → 1° に丸め */
if (bin >= 360) bin -= 360;

int32_t err = (int32_t)angle - (int32_t)bin * 100;   /* ビン中心からのずれ [0.01°] */
if (err >  18000) err -= 36000;
if (err < -18000) err += 36000;
uint16_t abs_err = (uint16_t)(err < 0 ? -err : err);

if (dist > 0 && (bin_dist[bin] == 0 || abs_err < bin_err[bin])) {
    bin_dist[bin] = dist;
    bin_err[bin]  = abs_err;
}
```

#### なぜ「最小距離」をやめたのか

v0.3 は「同一ビンでは最小距離を採用（安全側）」としていた。
**衝突判定としては正しいが、地図生成用のデータを近距離側に系統的に歪ませる。**
系統誤差（バイアス）はランダム誤差と違って平均化で消えないため、
**ICP / スキャンマッチングの精度が構造的に落ちる。**

**安全用途の最短距離判定は STM32 内部で完結させる。**
別途「各セクタ内の最短距離」を内部変数として保持し、緊急停止・減速の判定に使えばよい。
Pi へ送る点群にはバイアスをかけない。

```
    Pi へ送る点群   : バイアスのない代表点（地図生成・SLAM 用）
    STM32 内部の判定: セクタ内最短距離（衝突回避用）  ← 送らなくてよい
```

距離 0（無効・範囲外）はそのまま 0 として送る。Pi 側が無効値として扱う。

### 7.4 セクタの送信タイミング

```
sector_idx = bin / 30       （0-11）
```

角度が次のセクタへ進んだ時点で、完了したセクタを送信する。

- **`t_start_us`**: そのセクタの**最初の点が観測された時刻**
- **`duration_us`**: そのセクタの最初の点から最後の点までの経過時間
  （= 最後の点の観測時刻 − `t_start_us`）
- **`rot_speed_dps`**: LD06 が報告する回転速度をそのまま 0.01 deg/s 単位で入れる
- **`angle_start` / `angle_step` は廃止した。** 送らない
- 送信後、そのセクタのビン（`bin_dist` / `bin_err`）を 0 クリアする

観測時刻は LD06 パケットの受信時刻でよい（精度は数百 μs あれば十分）。

**`duration_us` があることで、Pi 側はセクタを1個落としても点群の歪み補正を続行できる。**
v0.3 のように前セクタとの `t_start_us` 差分に頼る設計だと、1個のロスで補間が破綻していた。

### 7.5 起動時の扱い

**最初の1回転は部分的にしか埋まらないため、送信をスキップする。**
1周分のビンが完全に埋まってから送信を開始する。

### 7.6 LiDAR 断検出

LD06 からのデータが 300ms 途絶したら `flags` の `lidar_ok` を落とす。
LiDAR の回転停止・ケーブル外れを Pi 側が検知できるようにする。

LD06 のパース/チェックサムエラーは `stats_t.lidar_frame_error_count` に計上する。

---

## 8. 安全要件

**この章の内容は最優先で実装すること。** 走行機能より先に、止まる機能を作る。

### 8.0 安全層の全体像

| 層 | 機構 | 反応時間 | 復帰条件 |
|---|---|---|---|
| 1 | **ハートビート断**（`estop_active`） | 50ms | 人間の明示操作 |
| 2 | **`COMMAND` 途絶**（`uart_timeout`） | **100ms** | `COMMAND` 再開で自動（ただし `armed` は落としたまま） |
| 3 | **過電流ハード遮断**（`fault_*_overcurrent`） | 即時 | **電源リセットのみ（ラッチ）** |
| 4 | **低電圧警告**（`fault_*_undervoltage`） | 1秒継続 | 電圧回復で自動 |

**層3と層4を混同しないこと。** 層3はラッチして走行不能になる。層4は警告のみで走り続けられる。

### 8.1 ハートビート監視（第1安全層）★実装済み（2026-08-07）

**STM32 側で実装完了。ポーリングは 2kHz。確定した挙動は
[`uart_protocol.md`](uart_protocol.md) §9「STM32 側の確定挙動」が正。**
要点は「エッジ10回で接続とみなす / 未接続なら発動しない / 50ms でラッチ /
車両のボタン2でのみ解除 / 制動はするが駆動電源は切らない」。**Pi 側の実装に変更は不要。**

Pi は GPIO6 に **100Hz の矩形波**を出力し続ける。STM32 は **PB12** で監視する。

```
エッジが 50ms 途切れた  →  即ブレーキ + flags.estop_active = 1
```

#### 実装は EXTI ではなくポーリング

**STM32 の EXTI はライン番号ごとに 1ポートしか選べない。**
Pi 連携用の PB12 と前方超音波の ECHO (PA12) が同じ **EXTI ライン 12** を使うため、
**ハートビートを外部割り込みで取ることはできない。**

```
1kHz 以上の制御ループで PB12 をポーリングし、レベル変化を検出する
エッジ間隔 5ms に対して 1kHz なら 5サンプル取れるので、50ms 判定には十分間に合う
```

超音波の ECHO は割り込みが必要（パルス幅測定のため μs 精度が要る）なので、
**EXTI ライン 12 は PA12 に割り当てる。** ハートビートは ms 精度で足りるのでポーリングでよい。

チャタリング・ノイズ対策として「1ms 未満の間隔の変化は無視する」フィルタを入れる。

#### なぜ単純なアクティブ Low の E-Stop ではないのか

**フェイルセーフにならないから。**
配線が断線するとプルアップで High（= 正常）に戻ってしまい、異常を検知できない。

ハートビート方式なら、以下すべてを**単一の機構で**検出できる。

1. Pi のフリーズ
2. Pi の電源断
3. 配線の断線・接触不良
4. Pi 側プロセスのクラッシュ

#### 復帰条件

**`estop_active` からの自動復帰は禁止。**

```
1. ハートビートが復活している               （必要条件）
   かつ
2. Pi から mode = DISARM を受信            （人間が明示的に解除した証拠）
   その後
3. Pi から flags.arm = 1 を受信して初めて再武装
```

原因が解消しないまま走り出すのを防ぐ。

### 8.2 UART タイムアウト（第2安全層）★200ms → 100ms

```
COMMAND が 100ms 途絶  →  自動ブレーキ + flags.uart_timeout = 1 + armed = 0
```

**v0.3 の 200ms は緩すぎた。** 100Hz 送信に対して 200ms は 20発の取りこぼしを許すことになり、
3 m/s なら 60cm 進んでからブレーキがかかる。100ms（10発）とする。

- **惰性走行（コースト）ではなくブレーキ**をかける
- `COMMAND` の受信が再開したら `uart_timeout` は自動でクリアしてよい
- ただし **`armed` は落としたままにする。** Pi 側が明示的に再武装するまで動かさない

### 8.3 電源保護（第3・第4安全層）

**電源系統は2つある。**

| 系統 | 内容 | 過電流閾値 | 動作 |
|---|---|---|---|
| 駆動系 (Primary) | モータードライバ電源 | 5.0 A | **DRIVE_POWER をハード遮断してラッチ** |
| シグナル系 (Secondary) | マイコン・Pi・LiDAR・ライト類 | 3.0 A | LiDAR / 駆動電源を遮断してラッチ |

どちらも 8セル ニッケル水素。放電終止 8.0V（1.0V/cell）、満充電 11.2V（1.4V/cell）。

```
低電圧警告: 8.0V 未満が 1秒継続で立てる
復帰      : 8.8V（1.1V/cell）で自動クリア  ← ヒステリシス必須
```

#### 過電流ラッチと `arm` 要求の関係 — **Pi 側に必ず伝えること**

過電流でラッチした後は、**リセットするまで `DRIVE_POWER` が復帰しない。**
このとき `flags.drive_power_locked` を立て、**Pi が `arm` を要求しても `armed` を立てない。**

**これは異常ではなく仕様である。** Pi 側は `drive_power_locked` が立っている間の
`arm` 拒否を FAULT 扱いしないよう実装済み（`uart_protocol.md` §5.4）。
STM32 側は必ずこのビットを立てること。**立て忘れると Pi 側からは「原因不明の arm 失敗」に見える。**

### 8.4 その他の保護

| 保護 | 動作 | 反映先 |
|---|---|---|
| MD 過熱 | 出力制限 or 遮断。**特に MD ステア（`temp[2]`）は据え切りで過熱しやすい** | `md_status[2].overheat` |
| MD 通信断 | 該当モータを停止 | `md_status[i].comm_ok = 0` |

**据え切り（停車中の操舵）による MD ステアの過熱**は、この車両構成で最も起こりやすい故障モード。
停車中の連続操舵時間に上限を設けるか、電流を絞る保護を入れること。

**`md_status[i].comm_ok` は必須。** MD が死んで無言になると status は最後の値のまま固まるため、
これが無いと Pi 側が過去の正常値を見続けることになる。
**直近 100ms で MD からの状態フレームを受信できているか**で判定する。

### 8.5 `flags` ビット定義（u32）★全面改訂

```c
/* bit0-1: STM32 が実際に採用しているモード */
#define FLG_MODE_MASK                 (3u <<  0)   /* 0=DISARM 1=MANUAL 2=AUTO 3=予約 */

#define FLG_ARMED                     (1u <<  2)   /* DRIVE_POWER 投入済み            */
#define FLG_ESTOP_ACTIVE              (1u <<  3)   /* ハートビート断で発動中           */
#define FLG_UART_TIMEOUT              (1u <<  4)   /* COMMAND 途絶で自動ブレーキ中     */
#define FLG_TC_ACTIVE                 (1u <<  5)   /* ★TC が「今まさに介入中」        */
#define FLG_TV_ACTIVE                 (1u <<  6)   /* ★TV が「今まさに介入中」        */
#define FLG_IMU_OK                    (1u <<  7)
#define FLG_LIDAR_OK                  (1u <<  8)
/* bit9 は予約（旧 FLG_CALIB_RUNNING。CALIB 廃止により削除）                        */
#define FLG_STEER_CENTER_VALID        (1u << 10)   /* ★Flash に有効な原点が保存済み   */
#define FLG_FAULT_DRIVE_OVERCURRENT   (1u << 11)   /* 駆動系過電流。★ラッチ           */
#define FLG_FAULT_SIGNAL_OVERCURRENT  (1u << 12)   /* シグナル系過電流。★ラッチ       */
#define FLG_FAULT_DRIVE_UNDERVOLTAGE  (1u << 13)   /* 駆動系低電圧。自動クリア          */
#define FLG_FAULT_SIGNAL_UNDERVOLTAGE (1u << 14)   /* シグナル系低電圧。自動クリア      */
#define FLG_DRIVE_POWER_LOCKED        (1u << 15)   /* ラッチにより arm 要求を拒否中     */
#define FLG_AUTO_STOP_ACTIVE          (1u << 16)   /* ★v0.7 自動停止が今まさに介入中   */
/* bit17-31 予約（0 を送ること） */
```

- **`mode` が bit8-9 から bit0-1 へ移動した。** v0.3 からの変更点なので注意
- **`TC_ACTIVE` / `TV_ACTIVE` / `AUTO_STOP_ACTIVE` は「有効/無効」ではなく
  「今まさに介入しているか」**（★v0.7）。`auto_stop` を許可しているかどうかは
  Pi 自身が送った `COMMAND.flags` bit7 で分かるので、テレメトリには載せない
- **bit11/12 はリセットするまで消えない。** bit13/14 は電圧が復帰すると消える
- **bit10 は旧 `calib_done` を改名したもので、意味も変わった。**
  「今この起動で実行した」ではなく「**Flash に有効な原点が保存されている**」。
  CALIB モードを廃止し、起動のたびに実行するものではなくなったため

### 8.6 `md_status[i]` ビット定義

MD が返す status バイトをそのまま転送し、上位ビットを STM32 が付加する。

```c
#define MDS_RUNNING          (1u << 0)   /* MD 由来: 停止モードでない            */
#define MDS_VOLTAGE_OOR      (1u << 1)   /* MD 由来: 電源電圧異常                */
#define MDS_OVERHEAT         (1u << 2)   /* MD 由来: 過熱                        */
#define MDS_OVERCURRENT      (1u << 3)   /* MD 由来: 過電流                      */
#define MDS_COMM_OK          (1u << 4)   /* ★STM32 が付加: 直近100ms 受信あり    */
#define MDS_LIMIT_SYNCED     (1u << 5)   /* ★STM32 が付加: トルク上限が指令と一致 */
/* bit6-7 予約 */
```

---

## 9. モードと原点合わせ

### 9.1 モード ★`CALIB` を廃止

| 値 | モード | 内容 |
|---|---|---|
| 0 | `DISARM` | 停止・待機。モータ非励磁。**電源投入時の初期状態** |
| 1 | `MANUAL` | Pi 経由のラジコン操縦 |
| 2 | `AUTO` | 自律走行 |
| 3 | — | **予約**（v0.4 で `CALIB` を廃止。Pi は送ってこない） |

### 9.2 遷移ルール

- `MANUAL` / `AUTO` へ入るには **`FLG_STEER_CENTER_VALID` が立っていること**が必要
- `ESTOP` / `FAULT` 状態からの復帰は `DISARM` を経由する
- 要求されたモードを拒否した場合も、**`TELEMETRY.flags` bit0-1 には実際のモードを入れる**
- `mode = 3` を受信したら無視して現在のモードを維持する（`LOG` に WARN を出す）

### 9.3 ステアリング原点合わせ — 走行中のシーケンスは実装しない

**v0.4 で方針を確定した（`uart_protocol.md` §0.2-1）。**

原点合わせは**組み立て・整備時の作業**として扱う。

```
1. タイヤを手で真っ直ぐにする
2. ボタンを押しながら電源投入
3. そのときのモータ機械角を中心点として Flash に保存
```

**自動キャリブレーション（両端に当てて中心を出すシーケンス）は実装しない。**
理由: 実装量が大きい割に、機構を突き当てる動作は MD ステアの過熱と機構破損のリスクがある。
組み立て時に一度決めれば足りる。

#### STM32 がやること

- 起動時に Flash から中心点を読み、**有効な値があれば `FLG_STEER_CENTER_VALID` を立てる**
- **`flags` bit9（旧 `calib_running`）は予約。常に 0 を送る**
- **Flash に有効な値が無ければ `FLG_STEER_CENTER_VALID` を立てない**
  → Pi 側は `READY` に進まず、GUI に整備手順を表示する
- 可動範囲はモータ角で **±60°** にクランプする

#### リンク比: 0.5（実測確定）

**モータ機械角 → 路面舵角の換算比は 0.5**（2026-08-20 実測確定、§14-1）。
モータ可動域 ±60° に対し、路面舵角の上限は **±30°（0.524 rad）**。
`config/vehicle.toml` の `max_steer` はこの路面舵角基準の値。

---

## 10. パラメータの扱い

### 10.1 パラメータ一覧

| param_id | 内容 | 型 | 備考 |
|---|---|---|---|
| ~~`0x0001`~~ | ~~最大速度~~ | — | **v0.10 で廃止**（`result=1` を返す）。`DRIVE_MAX_SPEED_M_S`（5.0 m/s）固定 |
| ~~`0x0002`~~ | ~~最大加速度~~ | — | **v0.10 で廃止**（同上）。`DRIVE_MAX_ACCEL_M_S2`（3.0 m/s²）固定 |
| ~~`0x0003`~~ | ~~最大舵角（路面舵角）~~ | — | **v0.10 で廃止**（同上）。路面舵角 ±30° 固定 |
| `0x0010` | TC 有効 | 0/1 | ★v0.8 で STM32 側が実装。それ以前は `result=1`（不明な param_id） |
| `0x0011` | TC スリップ率しきい値 | float | |
| `0x0020` | トルクベクタリング 有効 | 0/1 | ★v0.8 で STM32 側が実装。それ以前は `result=1` |
| `0x0021` | TV ゲイン | float | |
| `0x0030` | 速度制御 Kp | float | |
| `0x0031` | 速度制御 Ki | float | |
| `0x0040` | **LiDAR 出力フォーマット** | **enum** | 下表。★v0.4 で bool から変更 |
| `0x0050` | 片輪浮き対策 有効 | 0/1 | ★v0.9 で STM32 側が実装。TC 本体（`0x0010`）とは独立 |
| `0x0051` | 片輪浮き対策 しきい値・ゲイン | float | 未実装 |
| ~~`0x0041`~~ | — | — | **廃止**（`0x0040` の enum に統合） |

**`param_id 0x0040` — LiDAR 出力フォーマット（enum）**

| 値 | 送信する TYPE | 帯域 |
|---|---|---|
| **0（既定）** | `0x01` `LIDAR_SECTOR` | 9.12 kB/s |
| 1 | `0x05` `LIDAR_SECTOR_I` | 12.7 kB/s |
| 2 | `0x09` `LIDAR_SECTOR_C` | 5.52 kB/s |
| その他 | — | `result=2`（範囲外）で拒否し、`applied` に現在値を返す |

**bool 2本（強度 on/off + 圧縮 on/off）にしない理由**:
「圧縮 かつ 強度付き」という**第4の組み合わせが表現できてしまい、未実装のまま
指定される余地が残る**ため。enum なら不正な状態を型で排除できる。

**走行中の切り替えを許可すること**（`result=3` で拒否しない）。
帯域が苦しいときに GUI から試せることに意味がある。

### 10.2 実装ルール

1. **`CONFIG_SET` / `CONFIG_GET` を受けたら必ず `CONFIG_ACK` を返す。**
   不明な ID でも `result=1` で返す
2. **範囲外の値はクランプし、`applied` にクランプ後の値を入れる**
   これにより Pi の GUI が常に「実機の真の値」を表示できる
3. **`CONFIG_GET` は現在値を `applied` に入れて `result=0` で返す**（v0.4 追加）
   これが無いと GUI 起動時に実機の値と同期できない
4. 走行中に変更すると危険なパラメータは `result=3`（変更不可）で拒否する

### 10.3 制御パラメータは Flash に保存しない（揮発）

**`CONFIG_SET` の値は RAM 上のみ。STM32 は Flash に書かない。**

```
電源投入   →  安全側のデフォルト値で起動
           →  Pi が接続時に CONFIG_GET で現在値を読む
           →  Pi が config/params.yaml の値を CONFIG_SET で push
           →  以降、Pi の値が正
```

理由：設定値の single source of truth を git 管理下のファイルに一元化するため。
STM32 だけに値が残ると、「誰も知らない古い設定が残っていて挙動が変わる」という
再現困難な問題を生む。加えて、**暗黙に毎回 Flash へ書くと書き換え寿命を無駄に消費する**
（チューニング中は毎秒書くことになる）。

#### STM32 が Flash を使ってよい値

**ステアリング原点と IMU キャリブレーション値のみ。**
この2つは STM32 でしか測れず、Pi から push できないため Flash に持つのが正しい。
制御パラメータ（`0x0001`〜`0x0040`）とは扱いを分けること。

> **`CONFIG_SAVE` は v0.4 では実装しない。**
> 永続化が必要になった時点で別コマンドとして追加する。
> （Pi 側の初回ドラフトは `0x14` に `CONFIG_SAVE` を割り当てていたが、
> `0x14` は `VERSION_REQ` になった）

**デフォルト値は必ず安全側**（最大速度は控えめ、TC/TV は有効、`0x0040 = 0`）にすること。
Pi が設定を push する前に暴走しないように。

---

## 11. 時刻同期で STM32 がやること

Pi 側が NTP と同じ4タイムスタンプ方式でクロックオフセットを推定する。

```
    T1 ── Pi   : PING を送信した時刻
    T2 ── STM32: PING 最終バイトを受信した時刻   → PONG.t_ping_rx_us
    T3 ── STM32: PONG 1バイト目が線に出た時刻    → PONG.t_pong_tx_us
    T4 ── Pi   : PONG を受信した時刻
```

Pi 側が計算する:

```
    往復遅延   delay  = (T4 − T1) − (T3 − T2)
    オフセット offset = ((T2 − T1) + (T3 − T4)) / 2
```

### STM32 側の実装要件 ★v0.4 で厳密化

1. **`T2` は UART の IDLE 割り込みで、PING 最終バイト到着の瞬間に取る。**
   メインループで取ると、ループ周期分のジッタがそのままオフセット推定のバイアスになる。
   実装としては「IDLE 割り込みが入った時刻」を毎回記録し、
   パース結果が PING だったらその時刻を使えばよい。

2. **`T3` は「送信キュー投入時」ではなく「実際に1バイト目が線に出た瞬間」に取る。**
   **LiDAR セクタ1個の送信には 3.04ms かかる。** PONG がその後ろに並ぶと
   T3 以降に最大 3ms の非対称遅延が発生し、**4タイムスタンプ方式でも除去されず、
   オフセットに最大 1.5ms のバイアスが乗る。**

   実装: PONG を送信キューの先頭に割り込ませ、**送信開始割り込み（または DMA 転送開始直後）で
   タイムスタンプを確定させ、その値をペイロードに書き込んでから送る**。
   ペイロードを DMA に渡す前に値を確定させる必要があるため、
   「PONG 専用の小さな送信バッファを用意し、DMA 起動の直前に `t_pong_tx_us` を埋める」
   という順序にする。

3. **`PONG` は最優先で送る。** 定期送信の順番待ちに入れない。

4. **μs カウンタを新規に実装する。**
   現行のタイマは 180MHz のサイクルカウンタベースで**約 24秒でラップ**してしまい使えない。
   自由走行タイマ（例 TIM2 を 1MHz で回す）で **32bit μs、起動時 0 スタート、
   約 71.6 分でラップ**するカウンタを用意する。

### μs カウンタのラップ

- STM32 側は特に対処不要（そのまま送る）
- 差分計算は必ず符号なし演算で行えば、ラップをまたいでも正しい値になる（`(uint32_t)(a - b)`）
- Pi 側で 64bit に拡張して累積する

**`odom_dist` は同格ではない。** 0.1mm 単位の `int32_t` のラップは **±214.7 km**
（u32 全域で 429.5 km）で、ミニカーでは到達しない。Pi 側は素直な符号付き減算で扱う。

---

## 12. 実装チェックリスト

### 通信基盤

- [ ] UART 250000 bps 8N1（USARTDIV = 11.25 / 22.5、誤差 0%）
- [ ] `crc16_ccitt("123456789")` が **`0x29B1`** を返す
- [ ] 全構造体に `#pragma pack(push,1)` を付けた
- [ ] `_Static_assert` で全構造体サイズを検証している（**`telemetry_t` == 66**）
- [ ] TX は **送信キュー付き DMA**（**現行の `Serial_Write` を使っていない**）
- [ ] 送信中の DMA フレームを途中で中断していない
- [ ] RX は DMA + IDLE 検出
- [ ] `HAL_UART_ErrorCallback()` で DMA 受信を再起動している
- [ ] フレーム抽出ステートマシンが `0xAA 0xAA 0x55` を取りこぼさない
- [ ] **`LEN = 0`（`VERSION_REQ` 0x14）でハングしない**
- [ ] 受信した TYPE が `0x10-0x1F` の範囲であることを確認している（方向規約）
- [ ] SEQ を送信ごとにインクリメントしている
- [ ] 受信フレームから構造体を作るのに `memcpy` を使っている（キャストしていない）

### 安全（最優先）

- [ ] ハートビートを **PB12 のポーリング**で 50ms タイムアウト監視している
- [ ] **EXTI ライン 12 は PA12（超音波 ECHO）に割り当てている**
- [ ] `estop_active` からの**自動復帰をしない**
- [ ] **`COMMAND` 100ms 途絶**で**ブレーキ**（コーストではない）
- [ ] UART タイムアウト後、`armed` を落としたままにしている
- [ ] 電源投入時のモードが `DISARM` である
- [ ] `FLG_STEER_CENTER_VALID` が立つまで `MANUAL` / `AUTO` に入れない
- [ ] **過電流ラッチ中に `FLG_DRIVE_POWER_LOCKED` を立てている**（立て忘れ厳禁）
- [ ] 低電圧警告に**ヒステリシス**がある（8.0V で立て、8.8V で落とす）
- [ ] MD ステアの据え切り過熱保護がある
- [ ] **`md_status[i].comm_ok` を直近 100ms の受信有無で更新している**
- [ ] パラメータのデフォルト値が安全側である
- [ ] **`torque_mode`（bit6）が立っている間、車速 PI を迂回し `target_torque` を直接掛けている**（★v0.6）
- [ ] `brake` と `torque_mode` が同時に立っていたら **`brake` を優先**している（★v0.6）
- [ ] `target_torque` を **±0.125 N·m でクランプ**している（Pi 側のクランプに頼っていない）（★v0.6）
- [ ] **`auto_stop`（bit7）が立っている間、進行方向の超音波 20cm 未満で最大制動している**（★v0.7）
- [ ] `auto_stop` の進行方向判定に `torque_mode` の有無を反映している（速度 or トルクの符号）（★v0.7）
- [ ] **逆方向のセンサを見ていない**（前方に障害物があっても後退できる）（★v0.7）
- [ ] 検知不能（`ULTRASONIC_NO_ECHO`・範囲外）では `auto_stop` が作動しない（★v0.7）
- [ ] 優先順位が **`brake` > `auto_stop` > 通常指令**になっている（★v0.7）
- [ ] 介入中だけ `FLG_AUTO_STOP_ACTIVE` を立てている（許可されているかではない）（★v0.7）

### センサ・データ

- [ ] LD06 のビニングで**ビン中心最寄りの代表点**を採用している（最小距離**ではない**）
- [ ] セクタ内最短距離は**内部の衝突判定用**として別途保持している
- [ ] `duration_us` がセクタの実所要時間である
- [ ] `rot_speed_dps` が LD06 の報告値である
- [ ] **`angle_start` / `angle_step` を送っていない**（廃止済み）
- [ ] 起動後の**最初の1回転を送信していない**
- [ ] `t_start_us` がセクタ先頭点の時刻である
- [ ] LiDAR 300ms 断で `lidar_ok` を落とす
- [ ] `steer_actual` が**路面舵角に換算済み**である（モータ機械角ではない）
- [ ] `target_steer` を**路面舵角として解釈**している
- [ ] `speed` / `wheel_speed` に **1〜2Hz のローパス**を掛けている
- [ ] **`speed` を `cos(steer_actual)` で車体中心線方向に射影している**
- [ ] **`wheel_speed` / `odom_dist` は射影せず生値で送っている**
- [ ] `odom_dist` は**ローパスを掛けずに**絶対角アンラップから積算している
- [ ] `wheel_speed` が **周速 [m/s]** である（角速度ではない）
- [ ] `TELEMETRY` のフィールド順が **`wheel_speed`(16) → `odom_dist`(24) → `accel`(32)** である
- [ ] `torque_cmd` が **TC 適用後の最終指令値**で 0.0001 N·m/LSB である
- [ ] `cmd_seq_echo` が「受理して制御に反映した」SEQ である
- [ ] 配列インデックスが規約どおり（`wheel_speed`/`odom_dist` は FL,FR,RL,RR）
- [ ] `TC_ACTIVE` / `TV_ACTIVE` が「介入中」を表している
- [ ] **`flags` の `mode` が bit0-1 にある**（v0.3 の bit8-9 から移動）

### その他

- [ ] `PONG` を最優先で即座に返し、**送信開始時点で `t_pong_tx_us` を確定**している
- [ ] `t_ping_rx_us` を **IDLE 割り込み内**で取得している
- [ ] **μs カウンタが 71.6 分周期**である（24秒でラップする旧タイマを使っていない）
- [ ] `VERSION` を起動時に3回送り、`VERSION_REQ` (0x14) にも応答する
- [ ] `LIMITS` を `VERSION` と同じタイミングで起動時に3回送り、`LIMITS_REQ` (0x15) に
      も応答する（★v0.11。値は実装済みの固定定数と一致させること）
- [ ] `STATS` を 1Hz で送り、カウンタは **累積のまま 0 クリアしていない**
- [ ] `param 0x0040` が **enum (0/1/2)** として実装されている（bool ではない）
- [ ] `param 0x0040` の切り替えを**走行中も許可**している
- [ ] `CONFIG_SET` / `CONFIG_GET` に必ず `CONFIG_ACK` を返している
- [ ] 範囲外の値をクランプし `applied` に反映している
- [ ] **`CONFIG_SET` で Flash に書いていない**（制御パラメータは揮発）
- [ ] Flash に書くのは**ステアリング原点と IMU キャリブレーションのみ**
- [ ] `LOG` に `severity` を付けている
- [ ] `LOG` を TX 逼迫時に破棄し、`STATS.tx_drop` に計上している

---

## 13. 立ち上げ・テスト手順

段階的に確認する。いきなり全部を繋がない。

### Step 1: CRC 単体テスト

```
crc16_ccitt("123456789", 9) == 0x29B1
```

これが通らなければ他は何も動かない。

### Step 2: ループバック試験（STM32 単体）

TX と RX を短絡し、自分で送ったフレームを自分でパースできるか確認する。
フレーミング・CRC・ステートマシンをまとめて検証できる。

### Step 3: `VERSION` + `LOG`（Pi と接続）

STM32 → Pi の一方向から始める。`VERSION` を起動時に送り、Pi 側でデコードできるか確認。
続いて既知の ASCII 文字列を `LOG` で 1Hz 送り、化けないか見る。
**ボーレートとフレーミングの確認を兼ねる。**

続いて Pi から `VERSION_REQ` (0x14, LEN=0) を送り、応答が返るか確認する。
**`LEN = 0` の受信経路の検証を兼ねる**（ここでハングする実装が多い）。

### Step 4: `TELEMETRY` 送信

ダミーの固定値を入れて送り、Pi 側で正しくデコードできるか確認する。
**構造体レイアウトのズレはここで発見される。**

各フィールドに**それぞれ違う識別可能な値**を入れること。

```c
t.speed          = 1111;
t.yaw_rate       = 2222;
t.steer_actual   = 3333;
t.steer_cmd_echo = 4444;
t.wheel_speed[0]=41; t.wheel_speed[1]=42; t.wheel_speed[2]=43; t.wheel_speed[3]=44;
t.odom_dist[0]   = 500000; t.odom_dist[1] = 600001;  /* ★i32。境界確認に最重要 */
t.accel_x=51; t.accel_y=52; t.accel_z=53;            /* odom_dist の直後にあること */
t.pitch=61; t.roll=62;
t.temp[0]=11; t.temp[1]=22; t.temp[2]=33; t.temp[3]=44;
t.md_status[0]=0x11; t.md_status[1]=0x22; t.md_status[2]=0x33;
t.cmd_seq_echo   = 0xAB;   /* 末尾が合っていれば全体のサイズが合っている証拠 */
```

**`odom_dist` は i32 で offset 24 にあり、ここがずれると後続の全フィールドがずれる。**
値を大きめにして上位バイトまで正しく届くか確認すること。
**`accel_x` が `odom_dist` の直後（offset 32）に来ていることを必ず確認する** —
本書 v0.2 のドラフトでは accel が先、odom_dist が後だった。順序が逆だと
`accel` と `motor_current` がまとめて化ける。

### Step 5: `PING` / `PONG`

Pi から `PING` を送り、`PONG` が返るか確認する。`delay` が数 ms 程度に収まっていること。
**LiDAR を動かした状態でも `delay` のばらつきが増えないこと**（T3 の取得タイミングの検証）。

### Step 6: `COMMAND` 受信

**モータは繋がず**（または車体を浮かせて）、受信した `target_speed` / `target_steer` を
`LOG` でエコーバックし、値が正しいか確認する。

### Step 7: 安全機構の確認（モータ未接続で）

- [ ] Pi 側でハートビートを止める → `estop_active` が立つか
- [ ] Pi 側で `COMMAND` の送信を止める → **100ms 後**に `uart_timeout` が立つか
- [ ] `estop_active` から自動復帰しないか
- [ ] 過電流を模擬して `drive_power_locked` が立ち、`arm` が拒否されるか
- [ ] `steer_center_valid` が立っていない状態で `MANUAL` に入れないか

### Step 8: `LIDAR_SECTOR`

LD06 を接続。Pi 側で 360点を極座標プロットし、**部屋の形が見えるか**を目視確認する。
角度のオフセットや回転方向の逆転はここで発見する。

- `duration_us` が妥当か（10Hz 回転・30° なら約 8300 μs）
- `rot_speed_dps` が約 360000（= 3600 deg/s = 10 rps）か
- **わざと 1セクタ落として**、Pi 側の歪み補正が破綻しないか

### Step 9: `STATS` と長時間の品質確認

`STATS` を 1Hz で送り、Pi 側の統計と並べて表示する。
**モータを回した状態**でエラーが増えないかを見る（ノイズが UART に乗っていないかの確認）。

### Step 10: 実走行

上記すべてが通ってから、**低速から**。

### 長時間試験

- 30分以上の連続通信で `crc_error_count` / ロス率が増えないか
- **モータを回した状態**で通信エラーが増えないか
- **71.6 分以上**動かして μs カウンタのラップをまたいでも時刻同期が破綻しないか
- `odom_dist` が単調に増え、停止中にドリフトしないか
  （ラップは 214.7 km 先なので試験不要）

---

## 14. 未実測のまま残っている量

`uart_protocol.md` §14 と同じ内容。**STM32 側の作業項目。**

| # | 項目 | これが無いと困ること |
|---|---|---|
| ~~1~~ | ~~**ステアリングのリンク比**（モータ角 → 路面舵角）~~ | **確定**：0.5（2026-08-20 実測） |
| ~~2~~ | ~~**車輪半径**~~ | **確定**：0.03m（2026-08-20 実測）。STM32 ファームウェアの換算定数にも反映済み |
| ~~3~~ | ~~**後輪のギア比**~~ | **該当なし**：後輪はダイレクトドライブ（比=1） |
| ~~4~~ | ~~**ホイールベース・トレッド**~~ | **確定**：L=0.23m・トレッド=0.155m（2026-08-20、Pi 側で実測） |
| 5 | **トルク定数 Kt** | `torque_cmd` の物理的意味。TC のゲイン設計 |
| 6 | 前輪エンコーダの実分解能 | `odom_dist` の量子化誤差、静止デッドバンド幅 |

**残るは #5・#6。** #1・#2 は STM32 ファームウェアの換算定数への反映まで済んでおり、
Pi 側は「何 m 進んだか」「舵を何 rad 切ったか」を正しく知ることができる。

---

## 変更履歴

| バージョン | 日付 | 内容 |
|---|---|---|
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
