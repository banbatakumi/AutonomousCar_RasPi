# Raspberry Pi 側のノード構成と内部バス

!!! note "【設計の説明】"
    確認した版: 上位 `19291eb`（2026-10-14）。コードは [`surge_mk2/raspi/`][raspi-dir]。

Pi 側は**ノードごとに別プロセス**で、ZeroMQ の PUB/SUB と共有メモリでつないでいます。
1 つのプロセスが落ちても、ほかが巻き込まれないようにするためです。

## ノード

| ノード | 入力 | 出力 | 役割 |
|---|---|---|---|
| `io_node` | UART、バスの `cmd` | `vehicle_state`（100 Hz）、`scan`（10 Hz）、`diag/link` | STM32 との送受信、時刻同期、ARM の門番、ハートビート |
| `telemetry_node` | バス、WebSocket | `cmd`、`auto/ctrl` ほか | ブラウザ GUI との橋渡し、操縦権、デッドマン、自律走行の中継 |
| `planning_node` | `scan` ほか、`vehicle_state`、`auto/ctrl` | `auto/cmd`、`auto/state`、`auto/map` | 走行モード（planner）を動かして指令を作る |
| `camera_node` | CSI カメラ 2 台 | `image/front`・`image/rear`（共有メモリへの参照） | 画像取り込み |
| `cam_perception_node` | 画像 | `scan/cam`、`path/cam`、`cam/mask` | 走行可能領域のセグメンテーション（ONNX）→ 擬似 LiDAR |
| `cam_e2e_node` | 画像 | `cam_e2e/cmd` | 画像から操舵を直接回帰（模倣学習） |
| `line_perception_node` | 画像 | `line/cam` | 白線検出（古典的な色しきい値） |
| `cam_track_node` | 画像 + `scan` | `track/target` | 対象追従用のトラッキング |
| `arrow_signal_node` | 画像 | `route/select` | 矢印信号の認識（HSV しきい値） |
| `logger_node` | バス全体 | mcap ファイル | 記録 |
| `replay_node` | `.sfl` 記録 | `io_node` と同じバスの出力 | 記録の再生（実機なしの開発用） |

`planning_node` の**アルゴリズム自体は `raspi/auto/` にあり**、ノードは配線だけを持ちます（[走行モード](modes.md)）。

## バスの設計

### ZeroMQ PUB/SUB（`raspi/bus/zbus.py`）

```text
ワイヤ形式:  b"vehicle_state" + b"\x00" + msgpack(payload)     ← 1 フレーム
```

素直には multipart（`[topic, payload]`）にしたいところですが、**ZMQ の CONFLATE は multipart に対応していません。**
制御系に CONFLATE（最新 1 件だけ残す）は必須なので、トピックとペイロードを 1 フレームに詰めています。
区切りに NUL を入れているのは、`scan` の購読が `scan_debug` にまで前方一致してしまうのを防ぐためです。

| 設計判断 | 理由 |
|---|---|
| **購読は「トピック 1 本につきソケット 1 本」** | CONFLATE は**ソケット単位**でありトピック単位ではない。1 本の SUB で `vehicle_state` と `scan` を購読して CONFLATE を掛けると、片方が来るたびにもう片方が捨てられる。**LiDAR が来るたびに車両状態が消える**のに、見た目は動いているので気づきにくい |
| **配送ポリシーは購読側が決める** | `LATEST`（CONFLATE、制御・GUI 表示）と `RELIABLE`（`RCVHWM` 大きめ、記録）。制御系が「2 秒前の点群で舵を切る」のを防ぐのが `LATEST` の目的 |
| **ノードごとに 1 本の endpoint を bind** | PUB を 1 プロセス 1 本にして、購読側の接続先を固定する |
| **endpoint の持ち主は 1 つ** | ZeroMQ の ipc は、**既に誰かが bind している経路にもう一度 bind してもエラーにならない**（ソケットファイルを消して作り直す）。2 つ目が黙って endpoint を奪い、「動いているのにデータが来ない」になる。`Publisher` は bind の前に `.lock` を排他ロックし、取れなければ起動を拒否する |

メッセージ型は `msgspec` の Struct（`raspi/msgs/types.py`）で、内部の msgpack と WebSocket が同じ形です。
GUI 用の TypeScript の型は、ここから `config/gen_msgs.py` で**生成**します（CI の `--check` が再生成し忘れを検出）。

### 画像は共有メモリ（`raspi/bus/shm_ring.py`）

640×480×3 を 30 Hz で msgpack 経由にすると **28 MB/s のメモリコピー**が発生します。そこで:

```text
camera_node     → 共有メモリの N 枚リングに書く
ZMQ に流すのは  → {name, slot, seq, t_capture_ns, ...} の数十バイトだけ
perception_node → その参照から numpy の view を作る（コピーゼロ）
```

**読んでいる最中の上書きは seqlock で検出**します（書き手は `seq` を奇数にして書き、偶数に戻す。読み手は前後の `seq` を比べる）。
リングを 8 枚取れば、書き手が同じスロットに戻るまで 8 フレーム（30 fps なら 266 ms）あります。

!!! warning "弱いメモリモデルの注意"
    Pi 5 の Cortex-A76 は**弱いメモリモデル**で、プロセス間には GIL の保護もありません。理屈の上では、安定を示す `seq` の更新が
    データ書き込みより先に他コアから見えることがあり、読み手が壊れたフレームを掴みえます。Python には明示的なメモリバリアが無いので、
    `threading.Lock` の acquire/release（CPython 内部で atomic 命令になり ARM64 では `dmb` が出る）で**代用**しています。
    ソースのコメント自身が「言語仕様の保証ではなく実装の性質に頼っている。『証明された』とは言わない」と書いています。
    実効的な守りは、リングの段数を十分取ることです。

## 再生と実機を同じコードにする

`io_node`（実機）と `replay_node`（ログ再生）は、`LinkTracker`（受信フレームからリンク状態を組む部分）と `bus_bridge`
（受信フレーム → バスへの publish）を**共有**しています。2 つ書けば必ずズレて「再生した結果が実機と違う」ことになるためです。

「今」の時刻は呼び出し側が渡します。実機は `time.monotonic_ns()`、再生はログ時刻のカーソルを渡します。
こうすると再生が記録時と同じ座標系で動き、**リンクの健全性（INIT/OK/DEGRADED/FAULT）の遷移まで再現**できます。

シミュレータも同じ考え方で、**偽装するのは STM32 だけ**（`SerialLink` の位置に `SimLink` を差す）です。
`io_node` より上流は 1 行も変えずに動きます（[検証の仕組み](../dev/workflow.md)）。

## 記録

| 形式 | 書き手 | 特徴 |
|---|---|---|
| `.sfl` | `io_node` が直接 SD へ | 送受信フレームを**生のまま**。Wi-Fi が切れても Pi 単体で記録が続く。`replay_node` で再生できる |
| `.mcap` | `logger_node` | バス上のトピックを記録。GUI からは標準出力をそのまま WebSocket へ中継し、**Pi のディスクには書かない**（ブラウザ側が保持し、停止時にダウンロード） |

## 依存と配備

- 依存は Python 3.12/3.13 の `pyserial`、`pyzmq`、`msgspec`、`websockets`、numpy など。バージョンは範囲で留め、
  実機・大会当日は `requirements.lock.txt`（`pip freeze` の結果）を使う方針です。
- 画像の前処理は OpenCV や Pillow に依存せず numpy だけで書かれ、**学習側（Mac）と推論側（Pi）が同じ関数を import**します
  （`raspi/core/cam_e2e_preproc.py`）。前処理を変えたら `PREPROC_VERSION` を上げ、古い前処理で学習したモデルは読み込みを拒否します。
- systemd の `RemoveIPC=yes`（既定）は、そのユーザーの最後のセッションが終わると共有メモリを全消しします。
  実際に起きたため、Pi 側は `RemoveIPC=no` にしてあります（[設計上の教訓](../dev/lessons.md)）。

[raspi-dir]: https://github.com/banbatakumi/AutonomousCar_RasPi/tree/master/surge_mk2/raspi
