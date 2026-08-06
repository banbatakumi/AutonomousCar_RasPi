# CLAUDE.md

このファイルは Claude Code (claude.ai/code) がこのリポジトリで作業する際のガイダンスです。

## リポジトリの位置づけ

自律走行ミニカーの Raspberry Pi 側プログラムを管理するリポジトリ。
**2つの独立したプロジェクトが並存する**ので、どちらの話かを必ず確認すること。

```
AutonomousCar_RasPi/
├── robotcar/     # V1: Raspberry Pi 3 + STM32 遠隔操縦システム
└── surge_mk2/    # V2: Raspberry Pi 5 自律走行システム（← 現在のメイン）
```

### robotcar/（V1・休眠中だが現役）

Raspberry Pi 3 + STM32F446RE の**手動遠隔操縦**システム。RasPi を Wi-Fi AP にして、
ブラウザ UI から WebSocket でコマンドを送り、UART で STM32 に転送する。

- **完成済みで動作実績あり。再開して手を入れる可能性がある**
- 削除・移動・リファクタリングをしないこと
- 詳細は `robotcar/CLAUDE.md` を参照
- 指示なくこのディレクトリを変更しない

### surge_mk2/（V2・新規開発）

自律走行ミニカー「SURGE Mark.2」の Raspberry Pi 5 側プログラム。
LiDAR による SLAM・地図生成・経路生成・自律走行を目指す。

- **設計から新規に作り直す方針**
- 旧シミュレータ（`surge_sim/` `surge_sim_v2/`）は 2026-08-06 に削除済み。
  **旧実装を参考にしたり復元したりしないこと**（設計をやり直すのが目的のため）
- 参考が必要な場合のみ、ユーザーの明示的な指示に従って `git show v1-sim-archive:<path>` で参照する

## 削除済みの旧実装について

タグ `v1-sim-archive` に旧シミュレータ（`surge_sim/`＝V1、`surge_sim_v2/`＝V2）の
最終状態が保存されている。Python + FastAPI + React による自律走行シミュレータだった。

**ユーザーから明示的に指示されない限り、この内容を参照・復元・流用しない。**

```bash
git show v1-sim-archive:surge_sim_v2/core/slam.py   # 単体表示
git checkout v1-sim-archive -- <path>               # 作業ツリーへ復元
```

## 開発方針

- 応対は日本語
- コミットメッセージは1行の日本語でシンプルに
- 機密情報を含むドキュメントを作る場合は必ず `.gitignore` に追加する

## Raspberry Pi 5 の注意点（surge_mk2 の実装時）

Pi 3/4 との差異があり、V1 のコードがそのまま動かない箇所がある。

- **GPIO**: Pi 5 は RP1 チップ採用のため `RPi.GPIO` が動作しない。
  `gpiozero` + `lgpio` バックエンドを使うこと
- **UART**: デバイス名の割り当てが Pi 4 までと異なる（`/dev/ttyAMA0` が別用途）。
  GPIO UART を直結する場合は接続先の確認が必要
- **カメラ**: `picamera2` を使用（`picamera` は非対応）
