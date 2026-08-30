# proto — UART プロトコル（Pi 側実装）

`protocol.toml` が**唯一の定義**。ここから Pi 側 Python パーサと STM32 側 C ヘッダの
両方を生成する。仕様の文章は [`../../docs/uart_protocol.md`](../../docs/uart_protocol.md)。

```
proto/
├── protocol.toml            ← 唯一の正。編集するのはここだけ
├── generate.py              コード生成
├── framing.py               CRC・フレーム組み立て・ストリーム抽出（手書き）
└── generated/               生成物。手で編集しない
    ├── packets.py           Python の dataclass + encode/decode
    └── surge_proto.h        STM32 側へ渡す C ヘッダ
```

## なぜ生成にしているか

パケット定義を Pi 側と STM32 側で別々に手書きすると**必ずズレる**。
実際 v0.4 の策定中、`TELEMETRY` のフィールド順（`odom_dist` と `accel` のどちらが先か）が
食い違ったまま2往復した。バイト位置のズレは「なぜか値が化ける」という形で現れ、
原因にたどり着くまでに時間を溶かす。

定義を1箇所に閉じ込め、両言語を機械的に吐かせればこの種の事故は起きない。

## 使い方

```bash
# 定義を編集したら再生成
python3 raspi/proto/generate.py

# 再生成し忘れの検出（CI 用）
python3 raspi/proto/generate.py --check

# テスト
python3 -m unittest raspi.tests.test_proto
```

`generate.py` は生成時に以下を検証して落とす。

- フィールド合計バイト数が `len` と一致するか
- TYPE の重複
- 方向規約（`0x01-0x0F` = STM32→Pi、`0x10-0x1F` = Pi→STM32）と `dir` の整合
- 可変長フィールド (`bytes`) が末尾にあるか

## STM32 側への渡し方

**`generated/surge_proto.h` をそのままコピーして取り込む。編集しない。**

生成物をリポジトリにコミットしているのは、STM32 側が Python を動かさずに
ヘッダだけ取得できるようにするため。

```bash
cp raspi/proto/generated/surge_proto.h  <STM32プロジェクト>/Core/Inc/
```

ヘッダには以下が入っている。

- `PKT_*` — パケット種別
- `surge_*_t` — `#pragma pack(push,1)` 付きの構造体
- `_Static_assert(sizeof(...) == LEN)` — **pack 忘れをビルド時に検出する**
- `FLG_*` / `MDS_*` / `CMD_FLG_*` — ビット定義
- `MODE_*` / `PARAM_*` など enum

CRC とフレーム組み立ては STM32 側で実装する（`docs/stm32_interface.md` §3 に参照実装あり）。
`SURGE_CRC_CHECK` (= `0x29B1`) で必ず検証すること。

### ヘッダの健全性確認

手元で確認したいときは C コンパイラに通すだけでよい。`_Static_assert` が全部走る。

```bash
echo '#include "surge_proto.h"
int main(void){return 0;}' | cc -std=c11 -Wall -Wextra -Werror \
  -I raspi/proto/generated -x c - -o /dev/null && echo OK
```

## Python 側の使い方

```python
from raspi.proto import FrameParser, FrameEncoder, packets

# 受信 — Pi は STM32→Pi のパケットだけを受け付ける
parser = FrameParser(expect_types=packets.S2P_TYPES)
for frame in parser.feed(ser.read(max(1, ser.in_waiting))):
    msg = frame.decode()
    if isinstance(msg, packets.Telemetry):
        speed_mps = msg.speed * packets.Telemetry.META["speed"][0]

# 送信 — SEQ は FrameEncoder が管理する
enc = FrameEncoder()
ser.write(enc.encode(packets.Command(mode=packets.Mode.MANUAL,
                                     target_speed=1500)))   # 1.5 m/s
```

`parser.stats` に `crc_error` / `len_error` / `packet_loss` / `unknown_type` などが溜まる。
**GUI では STM32 側の `STATS` パケットと並べて出している**（`DiagGrid`）。
片方向だけ増えるのか両方向なのかで、原因の切り分けが変わる。

### 生値と SI 単位

**dataclass のフィールドはすべて生の整数**（ワイヤ上の値そのもの）。
SI 単位が要るときは `META` のスケールを掛ける。

```python
scale, unit = packets.Telemetry.META["steer_actual"]   # (0.0001, "rad")
```

ここで自動変換しないのは、ログには生値をそのまま残したいため
（変換係数を後から直しても記録が壊れない）。

## 未実装

- `LIDAR_SECTOR_C` の `255` は「5.10m 以上（飽和）」であり実測点ではない。
  占有格子に打たない処理は上位（perception）側で行う
- `odom_dist` の射影（`cos(steer_actual)` を**差分に**掛ける）も上位側
