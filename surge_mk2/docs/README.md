# SURGE Mark.2 ドキュメント

自律走行ミニカー **SURGE Mark.2** の設計文書。

| 文書 | 内容 | 読む人 |
|---|---|---|
| [architecture.md](architecture.md) | **ソフトウェア設計書**。全体アーキテクチャ、座標系規約、内部バス設計、安全設計、GUI 方針、ロードマップ | 全員。まずこれを読む |
| [uart_protocol.md](uart_protocol.md) | **UART プロトコル仕様**（**v0.7**）。仕様の説明の**正** | STM32 / RasPi 双方の実装者 |
| [stm32_interface.md](stm32_interface.md) | **STM32 側 実装仕様書**。C 構造体、送受信の実装、安全要件、チェックリスト、立ち上げ手順 | STM32 ファームウェア実装者 |
| [../sim/README.md](../sim/README.md) | **シミュレータ**（Mac 専用）。実機と同じ制御コードのまま走らせる | 自律走行を書く人 |

**数値の正は `raspi/proto/protocol.toml`。** そこから Python パーサと C ヘッダの両方を
生成する（`python3 raspi/proto/generate.py`）。文書と食い違ったら toml が正しい。

## 文書間の関係

```
architecture.md          全体設計（何をどこでやるか）
      │
      ├── uart_protocol.md      STM32 ⇄ Pi のプロトコル仕様【正】
      │         │
      │         └── stm32_interface.md   ↑を STM32 実装の形に落としたもの
      │
      └── （今後）raspi 側の詳細設計 / gui 側の詳細設計
```

**`uart_protocol.md` と `stm32_interface.md` が食い違った場合は `uart_protocol.md` を優先し、
`stm32_interface.md` を修正すること。**

## 現在の状態

**この節は腐りやすい。詳しい現在地は `PROGRESS.md` を見ること**（そちらが正）。

- **Phase 0（通信基盤・ログ記録再生・GUI 骨格）完了。実車で走行実績あり。**
  Pi 側の実装は `raspi/`、GUI は `gui/`（React 4タブ）
- **シミュレータあり**（`sim/`、Mac 専用）。`io_node --sim` で実機と同じ制御コードのまま
  コース上を走らせられる。実車が無いときの自律走行開発はこちら
- **プロトコルは v0.7**（2026-08-11）。v0.6 からはビット追加のみで**ワイヤ互換**
  （`COMMAND.flags` bit7 = `auto_stop`、`TELEMETRY.flags` bit16 = `auto_stop_active`）。
  v0.4 → v0.5 → v0.6 の経緯は `uart_protocol.md` の改版履歴にある
- SLAM / 自己位置推定の方式は **Phase 3 着手時に決定**する方針
- **ステアリングのリンク比と車輪半径が未実測**のため、`config/vehicle.toml` は暫定値。
  この2つが残っているクリティカルパス（`architecture.md` §15）
