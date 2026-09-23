# slam2d/ — 車体非依存の汎用2D LiDAR SLAM

`raspi/nav/`（このリポジトリの自律走行ミニカー専用SLAM、2026-08-14に一旦棚上げ）を土台に、
車体・センサ構成に依存しない汎用2D LiDAR SLAMライブラリとして一から設計し直したもの。
将来的にOSSとして単独公開できる品質を目指す。

## スコープ

**含むもの**: スキャン列（距離+角度+時刻）とオドメトリ/IMU列を入力に、占有格子地図と
自己位置の軌跡を出力する2D SLAMのコア機能一式（占有格子、面要素地図、点対線ICPによる
位置合わせ、運動予測、ループ検出、ポーズグラフ最適化、グローバルローカリゼーション）。

**含まないもの**: 特定車両の動力学モデル・特定LiDAR（LD06等）のプロトコル処理・
Ackermanステア固有のロジック。これらは `core/motion.py` の `MotionModel` 抽象や
`adapters/` 経由で外部から注入する。

`slam2d/` は `raspi.*` を一切 import しない。逆に `raspi/` 側が `slam2d/` を薄いアダプタ
（`raspi/auto/_slam2d_nav.py`）越しに使う。

## 構成（2026-09-23 の作り直し後）

```
core/
  types.py     SE(2)の代数・点群と姿勢の型
  deskew.py    モーションスキュー補正（点ごとの時刻で補正する TwistBuffer 付き）
  grid.py      占有格子（ヒット/ミスの回数、必要な方向にだけ自動で拡張）
  surfmap.py   面要素の地図（壁セルごとの平均位置と法線＋最寄り壁の表引き）
  register.py  点対線ICP（Gauss-Newton）。推測航法を事前分布に持つ
  localmap.py  直近のキーフレームだけで作る位置合わせ専用の地図
  motion.py    推測航法の抽象・ジャイロのゼロ点ずれ/車速の倍率の学習
  frontend.py  1周期の処理（脱スキュー→予測→位置合わせ→地図更新）
  localize.py  グローバルローカリゼーション（保存地図の上で「どこに居るか」）
backend/
  loop_detection.py  古いキーフレームだけで作った部分地図への再訪判定
  posegraph.py       g2opy のラッパー（Huber・外れ値の除去つき）
pipeline.py    SlamSystem（フロントエンド＋ループ閉じの公開API）
```

### 設計の骨格（なぜこの形か）

- **位置合わせは「距離を最小化する連続最適化」**（`register.py`）。相関スキャンマッチ
  （候補姿勢の総当たり）は初期値が悪いときに強い一方、答えが探索格子に量子化され、
  推測航法との融合が「得点−罰則」の近似になるため**推測航法の系統誤差を正せない**。
  総当たりは見失いからの復帰とループ検出の初期値作りにだけ使う
- **地図作成中は「直近数mのキーフレームだけの局所地図」に合わせる**（`localmap.py`）。
  全体の地図に合わせると、周回して戻ったとき古い壁と新しい壁が混ざる。大域的な整合は
  ループ閉じの担当（Cartographer の local SLAM / pose graph と同じ分担）
- **凍結した地図の上では全体の地図に合わせる**（追跡専用。ドリフトが溜まらない）
- **ジャイロのゼロ点ずれは静止中に較正する**（ZUPT）。走行中の回頭は自分で焼いた
  局所地図を基準に測っているので、ゼロ点ずれは原理的に観測できない
- **点ごとの脱スキュー**（`deskew.TwistBuffer`）。1周100msのあいだヨーレートが
  変わるコーナーでは、「1周のあいだ twist は一定」の近似が破綻する

## 検証

- 合成データの単体テスト: `pytest slam2d/tests -q`（105件、15秒程度）
- **シミュレータでの通し評価**: `.venv/bin/python -m sim.slam_bench`
  （`sim/slam_bench.py`。実機と同じ `VirtualLidar → ScanAssembler` 経路を通し、
  車速・ジャイロに誤差を注入して真値と突き合わせる。地図作成と凍結地図での
  追従を分けて測る）
- 公開データセット（CARMEN log形式、`datasets/README.md`）は未着手

## 開発方針

- 現行の `sim/`（車両動力学込みシミュレータ）には依存しない。`sim/slam_bench.py` は
  **シム側から slam2d を呼ぶ**評価ツールで、依存の向きは `sim → slam2d` のまま
- 依存管理は `requirements.txt` 方式。`pyproject.toml` は今は作らない
- テストは `unittest.TestCase` で書き `pytest slam2d/tests -q` で実行
- `cv2`（OpenCV）があれば距離変換・最寄りセルの表引きに使い、無ければ numpy の
  フォールバックに落ちる（Pi 5 には `opencv-contrib-python-headless` が入っている）
