# SURGE Mark.2 ドキュメント

自律走行ミニカー **SURGE Mark.2** の設計文書。

| 文書 | 内容 | 読む人 |
|---|---|---|
| [architecture.md](architecture.md) | **ソフトウェア設計書**。全体アーキテクチャ、座標系規約、内部バス設計、安全設計、GUI 方針、ロードマップ | 全員。まずこれを読む |
| [uart_protocol.md](uart_protocol.md) | **UART プロトコル仕様**（**v0.5**）。パケット定義の**正** | STM32 / RasPi 双方の実装者 |
| [stm32_interface.md](stm32_interface.md) | **STM32 側 実装仕様書**（v0.4 = プロトコル v0.5 に対応）。C 構造体、送受信の実装、安全要件、チェックリスト、立ち上げ手順 | STM32 ファームウェア実装者 |

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

- 設計フェーズ完了。**Phase 0（通信基盤・ログ記録再生・GUI 骨格）に着手可能**
- 実装コードはまだ存在しない
- SLAM / 自己位置推定の方式は **Phase 3 着手時に決定**する方針
- **プロトコルは v0.5**（2026-08-09、STM32 側発の変更を反映）。
  v0.4 からの差分は **`COMMAND` (0x10) だけ**で、LEN が 10→12 に変わっている。
  **LEN が変わるため、Pi と STM32 の版数がずれている間は走行指令が一切通らない。**
  灯火の 3段切替・パッシング・クラクションの押下中継続・制動トルク指定が増えた。
  v0.4 確定の経緯（STM32 側と2往復の議論）は `uart_protocol.md` §0 と §15 に残してある
- **ステアリングのリンク比と車輪半径が未実測**のため、`config/vehicle.yaml` は暫定値。
  この2つが Phase 0 のクリティカルパス（`architecture.md` §15）
