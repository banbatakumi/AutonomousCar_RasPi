# SURGE Mark.2 シミュレータ

ロボットカー「SURGE Mark.2」の制御ロジック開発用シミュレータ。
最終目標は **SLAMで地図生成 → レーシングライン最適化 → 自律走行**。
実機（Raspberry Pi + STM32 + LD06 LiDAR）と**共通コード**で動作することが最重要要件。

現在 **Phase1（基盤）・Phase2（Pure Pursuit経路追従）** が完成。Phase3〜4はスタブ。

## セットアップ

```bash
cd surge_sim
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

## 起動

```bash
python main.py --mode sim                       # ローカルpygameのみ
python main.py --mode sim --course "L-Shape Course"
python main.py --mode sim --serve               # ヘッドレス＋Web UI(WebSocket)
python main.py --mode sim --serve --window      # pygame と Web を同時表示
python main.py --mode real                      # 実機（Phase以降で実装）
```

### Web UI（実機/SIM共通）
`--serve` で起動すると、ブラウザから操作・監視できます（実機/SIM共通のデータ契約）。

```bash
python main.py --mode sim --serve --host 0.0.0.0 --port 8000
# → ブラウザで http://<PCのIP>:8000/ を開く
```

- LiDARレーダー・推定姿勢の車両・経路・先読み点をCanvasに描画
- キー（↑↓←→ A F R Space）/ボタンで操作 → `CommandFrame` をWebSocket送信
- 操作権は1クライアント（`操作権を取得`で奪取）、閲覧は複数可
- 詳細設計は `docs/UNIFIED_UI_DESIGN.md`

## UIの役割分担

- **Web UI＝操作系＋運用ビュー（実機相当）**：全操作はブラウザから。表示は「ロボットが認識している世界」のみ＝LiDAR・推定姿勢・**SLAMで構築した地図**・SLAM由来の中心線/レーシングライン。**真の壁・真の姿勢は配信せず表示しない**（実機には存在しないため、SIMでも隠す）。これにより実機/SIMでWeb UIは完全に同じ見た目。
- **pygame＝SIMデバッグビューア**：上記に加えて**真の壁・真の姿勢**を重ねて表示し、「SLAM地図 vs 真値」を見比べられる。描画専用（TAB=右ペイン切替、ESC=終了 のみ）。

推奨は `python main.py --mode sim --serve --window`（Webで操作・pygameで観察）。

## 操作（Web UI／キー・ボタン）

| キー | 動作 |
|------|------|
| ↑ / ↓ | 速度（目標速度）増減 |
| ← / → | ステア（左/右）。離すと自動センタリング |
| A | 地図追従（AUTO）⇔ 手動 切替 |
| F | リアクティブ（REACTIVE, LiDAR直接）⇔ 手動 切替 |
| M | SLAM占有格子マッピング 開始/停止 |
| G | 地図からレーシングライン生成（→AUTOで追従可） |
| SPACE | 一時停止 / 再開 |
| R | リセット |
| TAB | 右ペイン切替（SLAM / Graph） |
| ESC | 終了 |

### SLAM → レーシングライン → 自律走行（Phase3-4）
1. `F` でリアクティブ走行を開始し、`M` でマッピング開始 → コースを1周探索
2. 1周したら `M` で停止 → `G` でレーシングライン生成（占有格子から中心線抽出→最小曲率最適化）
3. `A` で AUTO にすると、生成したレーシングラインを Pure Pursuit で追従

緑＝SLAM占有格子、シアン破線＝抽出中心線、黄＝レーシングライン。

**Web UI（`--serve`）でも同じ操作・表示が可能**です。占有格子はbinary配信、
レーシングラインは生成時にシーン再送でブラウザへ反映されます（`M`/`G`キー・ボタン）。

走行モードは3種類：

- **MANUAL**：手動操作
- **AUTO(map)**（`A`）：既知のコース中心線（シアン破線）を Pure Pursuit で追従。黄丸＝先読み点。
  地図と自己位置が前提（Phase2はどちらも真値を使用）。
- **REACTIVE**（`F`）：**地図・自己位置推定・SLAMを一切使わず**、その瞬間の LiDAR スキャンだけで
  進路を決める Follow the Gap。**コースが事前に分からなくても走れる**。緑線＝選んだギャップ方向。

マウス: 右パネルの **COURSE** リストでコース切替、**SIM CTRL** で ▶/⏸/↺・速度倍率切替。

## アーキテクチャ

```
BackendBase (core/interfaces.py)  ← 実機/SIM共通インターフェース
   ├── SimBackend  (物理 AckermannModel + レイキャスト LidarSimulator)
   └── RealBackend (UART 250000bps, スタブ)

Controller (50Hz, 別スレッド)
   └── Localizer を source で自動切替（cheat / slam / ekf）

SimRenderer (pygame, UIスレッド)  ← Controller を get_snapshot() で読む
```

すべての制御モジュールは `core/interfaces.py` のデータクラスにのみ依存するため、
シミュレータで開発したコードをそのまま実機で動かせる。

## Phase ロードマップ

| Phase | 内容 | 状態 |
|-------|------|------|
| 1 | シミュレータ基盤（物理/LiDAR/描画/手動操作） | ✅ 完成 |
| 2 | Pure Pursuit 経路追従（中心線は真値から生成） | ✅ 完成 |
| 3 | 占有格子マッピング・CourseAnalyzer（中心線抽出） | ✅ 完成 |
| 4 | レーシングライン最適化（最小曲率＋速度プロファイル） | ✅ 完成 |

#### Phase3-4 実装メモ
- `core/occupancy.py`：log-odds占有格子マッピング（中央値誤差2.5cm）
- `core/course_analyzer.py`：占有格子を法線レイマーチ→中心線（平均誤差6.5cm・幅0.96m）
- `core/racing_line.py`：最小曲率最適化（二階差分エネルギー最小化）＋前後パス速度プロファイル
- 自律走行：SLAM地図由来のレーシングラインをPure Pursuit追従（誤差約3cm）
- マッピング姿勢：SIMはcheat（既知姿勢マッピング）／実機はオドメトリ想定
- `SLAMLocalizer`（Hector風スキャンマッチング）も実装済みだが、長い直線通路では
  アパーチャ問題で通路方向に滑るため既定はcheat。純LiDAR-SLAMは要オドメトリ補助。

### Phase2 メモ
`CourseAnalyzer.analyze()` は SLAM由来の `OccupancyGrid`（Phase3成果物）を入力にするため、
Phase2では `course_analyzer.build_course_map()` がコース真値から中心線を生成する
（`CheatLocalizer` と同じ「カンニング」方式）。Phase3完成後、この経路を SLAM由来に差し替える。

- 経路処理: `core/path_utils.py`（等間隔リサンプル / 最近傍 / 先読み点 / 曲率）
- 追従制御: `core/planner.py` `PurePursuitPlanner`（速度比例の可変先読み＋コーナー減速）
- 中心線ウェイポイントは各 `maps/*.py` の `CENTER_LINE`（任意）で定義

### コース未知でのLiDAR直接制御（Reactive / Follow the Gap）
地図を持たない実機の探索走行向け。`core/reactive_planner.py` `ReactivePlanner.compute_command(scan)`
が `LidarScan` だけから `ControlCommand` を生成する（自己位置推定もSLAムも不要）。

1. 前方視野(±90°)を切り出し → 2. 最近傍障害物に安全バブル → 3. 最大ギャップ探索
→ 4. ギャップ最遠点へ向かう角度をステアに → 5. 前方距離と旋回量で速度決定

検証：地図なし・LiDARのみ（σ0.02mノイズ込み）で両コースを無衝突周回（壁クリアランス約0.16m）。
実機では1周目をこれで安全探索し、Phase3のSLAMで地図化 → Phase4で最適化、という流れを想定。

## 車両スペック（config/vehicle.yaml）

- ホイールベース 0.230 m / トレッド 0.155 m
- 最大ステア角 ±40°、最大速度 3.0 m/s、タイヤ半径 0.028 m
- アッカーマンジオメトリ、速度時定数 0.1s / ステア時定数 0.05s

## 実機UART送信パケット（backend/real.py）

```
[0xAA][0x01][speed_H][speed_L][steer_H][steer_L][CRC8]
  speed : mm/s        符号付き16bit ビッグエンディアン
  steer : 0.01deg単位  符号付き16bit ビッグエンディアン
  CRC8  : ヘッダ(0xAA)以外の全バイトのXOR
```
