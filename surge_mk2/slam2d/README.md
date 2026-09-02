# slam2d/ — 車体非依存の汎用2D LiDAR SLAM

`raspi/nav/`（このリポジトリの自律走行ミニカー専用SLAM、2026-08-14に一旦棚上げ）を土台に、
車体・センサ構成に依存しない汎用2D LiDAR SLAMライブラリとして一から設計し直したもの。
将来的にOSSとして単独公開できる品質を目指す。

## スコープ

**含むもの**: スキャン列（距離+角度+時刻）とオドメトリ/IMU列を入力に、占有格子地図と
自己位置の軌跡を出力する2D SLAMのコア機能一式（占有格子、スキャンマッチング、
Point-to-Line ICP、運動予測、ループ検出、ポーズグラフ最適化）。

**含まないもの**: 特定車両の動力学モデル・特定LiDAR（LD06等）のプロトコル処理・
Ackermanステア固有のロジック。これらは `core/motion.py` の `MotionModel` 抽象や
`adapters/` 経由で外部から注入する。

`slam2d/` は `raspi.*` を一切 import しない。逆に `raspi/` 側が将来 `slam2d/` を
薄いアダプタ越しに使うことはあり得るが、それは任意の後続タスク。

## 開発方針

- 現行の `sim/`（車両動力学込みシミュレータ）には依存しない。検証は合成データ
  （`tests/`）と公開データセット（CARMEN log形式、`datasets/README.md` 参照）で行う
- 依存管理は `requirements.txt` 方式（このリポジトリの `ml_lidar/`・`ml_cam/` と同じ）。
  `pyproject.toml` は今は作らない
- テストは `unittest.TestCase` で書き `pytest slam2d/tests -q` で実行
- 詳細な設計判断は `/Users/banbatakumi/.claude/plans/wiggly-doodling-moth.md` を参照
  （車両依存/汎用の切り分け、g2oバインディングの選定、開発フェーズの完了条件など）
