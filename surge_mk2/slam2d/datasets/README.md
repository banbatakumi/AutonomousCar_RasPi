# 検証用データセット

生ログはgit管理しない（`.gitignore`対象）。ここには入手手順とライセンス上の注意だけを書く。

## CARMEN log形式（Intel Research Lab / MIT CSAIL / ACES Building）

Radish本家（`radish.sourceforge.net`）は2009年で更新停止しているため、以下のミラーを使う:

- MRPT公式データセットリポジトリ: https://www.mrpt.org/robotics_datasets （Rawlog形式）
- Bonn大学ミラー（Cyrill Stachniss研究室）: http://www.ipb.uni-bonn.de/datasets/
  （CARMEN log形式、プレーンテキスト。`slam2d/adapters/carmen.py` が読むのはこちら）

入手したログは `slam2d/datasets/raw/<データセット名>.log` に置く（gitignore対象）。
CIには `slam2d/tests/fixtures/` に先頭数百行だけを切り詰めたfixtureを同梱する
（Phase 6で追加）。

## フォーマットメモ

CARMEN logの`FLASER`行:

```
FLASER num_readings [ranges...] x y theta odom_x odom_y odom_theta timestamp
```

`num_readings`はデータセットにより異なる（180/181/361等）。詳細は
`slam2d/adapters/carmen.py`のdocstring参照（Phase 6で追加）。
