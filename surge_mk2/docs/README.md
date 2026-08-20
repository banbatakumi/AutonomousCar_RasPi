# SURGE Mark.2 ドキュメント

自律走行ミニカー **SURGE Mark.2** の設計文書。

| 文書 | 内容 | 読む人 |
|---|---|---|
| [system_overview.md](system_overview.md) | **システム全体像**。何をどこでやっているかを図と表で一通り。**まずこれ** | はじめて触る人 / 久しぶりに戻ってきた人 |
| [development.md](development.md) | **開発ガイド**。何を直したら何を再起動するか、コマンド集、**トラブルシューティング** | 手を動かす人。困ったらここ |
| [architecture.md](architecture.md) | **ソフトウェア設計書**。**「なぜそう設計したか」の正** | 設計の意図を知りたい人 |
| [uart_protocol.md](uart_protocol.md) | **UART プロトコル仕様**（**v0.7**）。仕様の説明の**正** | STM32 / RasPi 双方の実装者 |
| [stm32_interface.md](stm32_interface.md) | **STM32 側 実装仕様書**。C 構造体、送受信の実装、安全要件、チェックリスト、立ち上げ手順 | STM32 ファームウェア実装者 |
| [../sim/README.md](../sim/README.md) | **シミュレータ**（Mac 専用）。実機と同じ制御コードのまま走らせる | 自律走行を書く人 |
| [../gui/README.md](../gui/README.md) | GUI のコード地図 | GUI を直す人 |

> **`architecture.md` は設計時の文書で、実装が先へ進んでいる箇所がある。**
> ズレの一覧は [development.md §11](development.md#11-文書と実装のズレ2026-08-16-時点の棚卸し) にまとめてある。

**数値の正は `raspi/proto/protocol.toml`。** そこから Python パーサと C ヘッダの両方を
生成する（`python3 raspi/proto/generate.py`）。文書と食い違ったら toml が正しい。

## 文書間の関係

```
system_overview.md       システム全体像（何がどう動いているか）   ← 入口
      │
      ├── architecture.md       設計の理由【なぜそうしたかの正】
      │         │
      │         ├── uart_protocol.md      STM32 ⇄ Pi のプロトコル仕様【正】
      │         │         │
      │         │         └── stm32_interface.md   ↑を STM32 実装の形に落としたもの
      │         │
      │         └── sim/README.md / gui/README.md   各サブシステムの詳細
      │
      └── development.md        変更の反映方法・トラブルシューティング
                │
                └── PROGRESS.md   開発の経緯と実測値【現在地の正】
                          │
                          └── progress_archive.md   詳細な実験ログ・実測値（PROGRESS.mdから分離）
```

**`uart_protocol.md` と `stm32_interface.md` が食い違った場合は `uart_protocol.md` を優先し、
`stm32_interface.md` を修正すること。**

## 現在の状態

**この節は腐りやすい。詳しい現在地は `PROGRESS.md` を見ること**（そちらが正）。

- **Phase 0（通信基盤・ログ記録再生・GUI 骨格）完了。実車で走行実績あり。**
  Pi 側の実装は `raspi/`、GUI は `gui/`（React 5タブ）
- **シミュレータあり**（`sim/`、Mac 専用）。`io_node --sim` で実機と同じ制御コードのまま
  コース上を走らせられる。実車が無いときの自律走行開発はこちら
- **プロトコルは v0.7**（2026-08-11）。v0.6 からはビット追加のみで**ワイヤ互換**
  （`COMMAND.flags` bit7 = `auto_stop`、`TELEMETRY.flags` bit16 = `auto_stop_active`）。
  v0.4 → v0.5 → v0.6 の経緯は `uart_protocol.md` の改版履歴にある
- SLAM / 自己位置推定の方式は **Phase 3 着手時に決定**する方針
- `config/vehicle.toml` は `[dynamics]`（アクチュエータの動特性）を除き実測確定済み
  （2026-08-20。後輪はダイレクトドライブでギア比の概念が無い）。車輪半径 0.03m・
  リンク比 0.5 は STM32 ファームウェアの換算定数にも反映済みで、残るクリティカルパスは
  `[dynamics]` の実測のみ
