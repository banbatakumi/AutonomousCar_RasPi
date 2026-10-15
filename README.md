# AutonomousCar_RasPi

自律走行ミニカー **SURGE Mark.2** の Raspberry Pi 側プログラム。

**技術解説サイト: https://banbatakumi.github.io/AutonomousCar_RasPi/**
制御・システム構成・自動運転（SLAM・レーシングライン）の解説です。

## 構成

| ディレクトリ | 内容 |
|---|---|
| [`surge_mk2/`](surge_mk2/) | **現行。** Raspberry Pi 5 の自律走行システム（LiDAR SLAM、レーシングライン、シミュレータ、GUI、同定ツール） |
| [`robotcar/`](robotcar/) | 旧 V1。Raspberry Pi 3 + STM32 の手動遠隔操縦。現在は使っていない |

## 関連リポジトリ

[`AutonomousCar`](https://github.com/banbatakumi/AutonomousCar) — STM32 ファームウェアと基板。
**ファームの最新は `FW/MainBoard_V3` ブランチ**です（既定ブランチ `main` のファームは古い）。

`surge_mk2/tools/ctrl_tune` は、ファームの `src/control` を直接コンパイルして使うため、
`AutonomousCar` をこのリポジトリの隣のディレクトリに置きます（別の場所なら環境変数 `SURGE_FW_DIR`）。
