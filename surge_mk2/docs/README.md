# SURGE Mark.2 ドキュメント

| 文書 | 内容 |
|---|---|
| [system_overview.md](system_overview.md) | システム概要（構成・プロセスとトピック・安全・自動運転・GUI・記録・シム）と設計理由。**まずこれ** |
| [development.md](development.md) | 開発ガイド。何を直したら何を再起動するか、コマンド集、安全ルール、トラブルシューティング、ログ、学習系の動かし方 |
| [uart_protocol.md](uart_protocol.md) | **UART プロトコル仕様**（**v0.29**）の説明 |
| [stm32_interface.md](stm32_interface.md) | STM32 側が満たすべき実装要件（送受信・安全・再立ち上げ時の確認） |
| [calibration.md](calibration.md) | 車両の校正（試験 `raspi/auto/calib_*`・解析 `tools/calib`）、カメラの校正、車速 KF の調整の設計と手順 |
| [control_constants.md](control_constants.md) | 制御定数の出どころ（物理・同定・式・方針・経験則） |
| [competition_plan.md](competition_plan.md) | 大会に向けた計画 |
| [../sim/README.md](../sim/README.md) | シミュレータ（Mac 専用）・コースの作り方・ベンチ |
| [../gui/README.md](../gui/README.md) | GUI のコード地図 |
| [../slam2d/README.md](../slam2d/README.md) | 車体非依存の 2D LiDAR SLAM ライブラリ |
| [setup_credentials.md](setup_credentials.md) | Pi の SSH・Wi-Fi 等の設定値（`.gitignore` 対象・機密） |
| [archive/](archive/README.md) | **過去の記録（旧 PROGRESS・監査・コードレビュー）。陳腐化しており、指示がない限り読まない** |

現在地・実測値は [`../PROGRESS.md`](../PROGRESS.md)。**プロトコルは v0.29**（`protocol.toml` の `protocol_version` が正）。

## 正の優先順位

1. **数値・バイト配置の正は `raspi/proto/protocol.toml`**（Python パーサと C ヘッダを生成: `python3 raspi/proto/generate.py`）。文書と食い違ったら toml が正しい
2. 車両パラメータの正は `config/vehicle.toml`
3. `uart_protocol.md` と `stm32_interface.md` が食い違ったら `uart_protocol.md` を優先
4. `system_overview.md` は概要と設計理由の文書で数値を持たない。コードと食い違ったらコードを正とする
