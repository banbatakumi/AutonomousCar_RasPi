# CLAUDE.md（surge_mk2）

自律走行ミニカー **SURGE Mark.2** の Raspberry Pi 5 側プログラム。
リポジトリ全体の位置づけは [`../CLAUDE.md`](../CLAUDE.md) を参照（V1 の `robotcar/` とは独立）。

## 車体ハードウェア（一言）

- **駆動**: 後輪左右独立ダイレクトドライブ BLDC（トルクモード、TC・トルクベクタリングは STM32 内で完結）
- **ステアリング**: BLDC 1個、**位置モード**（サーボではない）。モータ角→路面舵角のリンク比 0.5（実測確定）
- **車速計測**: 前輪左右のアナログ絶対角エンコーダ（分解能粗く、加速度の微分には使えない）
- **IMU**: MPU6050（6軸、地磁気無し → 絶対方位は出ない）
- **LiDAR**: LD06（10Hz回転・裏向き実装で点群は鏡像、`ScanAssembler` が戻す）
- **カメラ**: 前後 CSI×2（`picamera2`）
- **超音波**: 前後×2（実質20Hz上限）
- **制御ボード**: STM32F446RE（100Hz〜1kHz のリアルタイム制御）
- **上位計算機**: Raspberry Pi 5（RP1 チップのため `RPi.GPIO` 不可 → `gpiozero` + `lgpio`）

詳細な仕様・制約（据え切り過熱、電源2系統、GPIO6ハートビート等）は
**必ず** [`docs/system_overview.md` §2 ハードウェア](docs/system_overview.md#2-ハードウェア) を参照する。
ここに数値を複製しない（複製すると片方が古くなる）。

## ドキュメントの索引（どれが何の正か）

| 知りたいこと | 見るファイル |
|---|---|
| システム全体をざっと把握したい | [`docs/system_overview.md`](docs/system_overview.md) |
| なぜその設計にしたか | [`docs/architecture.md`](docs/architecture.md) |
| 今どこまで進んだか・実測値 | [`PROGRESS.md`](PROGRESS.md) |
| どう直して実機/シムに反映するか | [`docs/development.md`](docs/development.md) |
| UART プロトコルの数値仕様 | [`docs/uart_protocol.md`](docs/uart_protocol.md)（唯一の定義は `raspi/proto/protocol.toml`） |
| STM32 側の実装仕様 | [`docs/stm32_interface.md`](docs/stm32_interface.md) |
| シミュレータの使い方 | [`sim/README.md`](sim/README.md) |
| GUI のコード地図 | [`gui/README.md`](gui/README.md) |
| 車両パラメータ（幾何・質量・動特性） | `config/vehicle.toml`（全ノードがここだけを見る） |

## 新規開発方針

- **設計から新規に作り直す方針。** 旧シミュレータ（`surge_sim/` `surge_sim_v2/`）は削除済み。
  タグ `v1-sim-archive` に残っているが、**ユーザーの明示的な指示がない限り参照・復元・流用しない**
  （設計をやり直すのが目的のため）。
