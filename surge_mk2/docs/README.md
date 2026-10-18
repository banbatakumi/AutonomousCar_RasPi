# SURGE Mark.2 ドキュメント

| 文書 | 内容 |
|---|---|
| [system_overview.md](system_overview.md) | システム全体像。**まずこれ** |
| [development.md](development.md) | 開発ガイド。何を直したら何を再起動するか、コマンド集、トラブルシューティング |
| [architecture.md](architecture.md) | ソフトウェア設計書。「なぜそう設計したか」 |
| [uart_protocol.md](uart_protocol.md) | **UART プロトコル仕様**（**v0.27**）の説明 |
| [stm32_interface.md](stm32_interface.md) | STM32 側の実装仕様（C 構造体・安全要件・立ち上げ手順） |
| [competition_plan.md](competition_plan.md) | 大会に向けた計画 |
| [../sim/README.md](../sim/README.md) | シミュレータ（Mac 専用） |
| [../gui/README.md](../gui/README.md) | GUI のコード地図 |
| [setup_credentials.md](setup_credentials.md) | Pi の SSH・Wi-Fi 等の設定値（`.gitignore` 対象・機密） |
| [archive/](archive/README.md) | **過去の記録。陳腐化しており、指示がない限り読まない** |

現在地・実測値は [`../PROGRESS.md`](../PROGRESS.md)。**プロトコルは v0.27**（`protocol.toml` の `protocol_version` が正）。

## 正の優先順位

1. **数値・バイト配置の正は `raspi/proto/protocol.toml`**（Python パーサと C ヘッダを生成: `python3 raspi/proto/generate.py`）。文書と食い違ったら toml が正しい
2. 車両パラメータの正は `config/vehicle.toml`
3. `uart_protocol.md` と `stm32_interface.md` が食い違ったら `uart_protocol.md` を優先
4. `architecture.md` は設計意図の文書。実装が先へ進んでいる箇所はコードを正とする
