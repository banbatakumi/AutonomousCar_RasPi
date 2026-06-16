# SURGE Mark.2 シミュレータ＆自律走行システム（V2）

ロボットカー「SURGE Mark.2」の制御ロジック開発用シミュレータと、
実機共通の操作・モニタリング WebUI。

最終目標: **SLAMで地図生成 → レーシングライン自動生成 → 自律走行**

## 設計原則
- `core/` のコードは実機・シミュレーションで完全共通
- WebUI は実機（WiFi 経由）・シミュレーション（localhost）で完全共通
- バックエンド（`backend/real.py` or `backend/sim/`）の切り替えのみで動作が変わる
- 起動時の `--mode` 引数でシミュ・実機を切り替える

## セットアップ

### Python（バックエンド）
```bash
cd surge_sim_v2
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

### Web UI
```bash
cd web
npm install
npm run build      # 本番ビルド（FastAPI が dist を配信）
# または開発時:
npm run dev        # Vite 開発サーバー（http://localhost:5173）
```

## 起動

```bash
# シミュレーション（pygame ビューア + FastAPI サーバー + 制御ループ）
python main.py --mode sim

# 実機（FastAPI + 制御ループのみ、pygame なし）
python main.py --mode real

# ログ再生
python main.py --mode replay --log logs/YYYYMMDD_HHMMSS.jsonl
```

起動後 `http://localhost:8000` で WebUI にアクセス。
（`npm run build` 済みなら FastAPI が配信。開発時は `npm run dev` の 5173 を使用。）

## アーキテクチャ

```
[Backend(sim/real)] --step--> [SharedState] <--read-- [pygame Renderer]（SIMのみ）
       ^                          ^  |
       |                          |  +--read--> [Broadcaster 20Hz] --WS--> [Web UI]
   send_command            update(50Hz)
       |                          |
       +------ [Controller 50Hz] -+
                    ^
                    | REST / WS command
              [FastAPI server]
```

- **SharedState**: 全スレッドが読み書きするスレッドセーフな中央データストア
- **Controller**: 50Hz 制御ループ（モード分岐・緊急停止・ウォッチドッグ）
- **Broadcaster**: 20Hz で SystemState を WebSocket 配信
- **pygame Renderer**: シミュの真実を表示するビューア専用（操作はブラウザで）

## 走行モード
- **Manual**: キーボードで速度・ステア操作
- **MapBuilding**: 手動走行しながら SLAM で地図構築（Phase3）
- **Autonomous**: 保存済み地図からレーシングライン生成 → ライン追従（Phase2/4）

## 実装状況（Phase）
- **Phase1 ✅**: シミュレータ基盤（物理・LiDAR・WebUI・ログ・手動操作）
- **Phase2 ✅**: 経路追従（Pure Pursuit）。コース定義の `CENTER_LINE` から
  カンニングで `CourseMap` を生成（`CourseAnalyzer.build_course_map`）、
  `PurePursuitPlanner` で追従。横ずれ平均3.3cm/最大8.9cmで両コース周回。
- **Phase3 ✅**: SLAM（占有格子マッピング）。`OccupancyGridMapper`（log-odds＋
  レイトレース）を中核に `HectorSLAM` で地図構築。`CourseAnalyzer.analyze(grid, seed_path)`
  が占有格子＋探索軌跡から中心線を抽出。`SLAMLocalizer`（Hector風スキャンマッチ）も実装
  （既定は cheat、純LiDARは直線退化のため）。地図保存/読込（.npz）対応。
  壁フィット中央値2.5cm・中心線抽出平均9.6cm。
- **Phase4 ✅**: レーシングライン最適化＋速度プロファイル。`RacingLineOptimizer` が
  最小曲率パス（循環二階差分作用素の線形最小二乗、numpyのみ）でレーシングラインを生成、
  横/縦加速度制限から速度プロファイルを算出。コース構築時に自動生成され Pure Pursuit が
  速度プロファイル付きで追従。曲率エネルギー88〜94%削減、衝突なし、平均速度1.5→2.2m/s。
  MPC（`MPCPlanner`）はスタブのまま。

**全4フェーズ実装完了。**

### 実機相当モード（真値なし・SLAM自己位置推定）★既定
`localization.mode: "slam"`（既定）では、自己位置を**LiDARのみのSLAMスキャンマッチ**で
推定する。車輪オドメトリは使わない。真値はシミュ物理とpygame比較表示・誤差評価だけに使う。

- **SLAMLocalizer**: Hector風 Gauss-Newton スキャンマッチ。占有格子の確率場に
  スキャン終点を合わせ込む。マルチ解像度（粗→細のボカし）で収束範囲を確保。
- **運動モデル事前分布**: 等速度予測（SLAM軌跡由来、オドメトリではない）をソフト拘束
  として GN に組み込み、直線で拘束不足になる縦方向（アパーチャ問題）を補う。← これが安定化の要
- **増分マッピング**: 一度確定したセルは凍結し未知領域だけ埋める＋キーフレーム間引きで、
  推定の微小誤差が地図に焼き込まれて発散するのを防ぐ。
- **半セル整合**: 占有はセル中心にあるので補間インデックスを半セルずらす（バイアス除去）。

検証（1周通し・両コース）: 自己位置推定誤差 **平均4〜15cm**、自律走行で衝突なし。

実機相当モードを切る（真値を使うデバッグ）には:
```bash
python main.py --mode sim --localization cheat
```

pygame では **青=真値 / 橙=SLAM推定** を重ねて表示し、ズレを目視できる。
`MPCPlanner` は将来拡張のスタブのまま。

### 使い方（実機相当フロー：既定の slam モード）
1. モードを **MapBuilding** にし、↑↓←→ で**コースを1周以上**手動走行
   → SLAM が自己位置推定しながら占有格子を構築（WebUI 左ペインに表示）
2. 「地図を保存」で `saved_maps/<名前>.npz` に保存（任意。占有格子＋探索軌跡）
3. モードを **Autonomous** → **スタート**
   → SLAM地図＋探索軌跡から中心線抽出 → レーシングライン生成 → SLAM自己位置のみで追従
   （保存済み地図は「読込」でロード後そのまま自律走行可能）

### デバッグ用（cheat モード：真値で即自律）
`--localization cheat` で起動すると、コース定義の中心線から即レーシングラインを生成し、
真値を自己位置として使うので探索なしで Autonomous を試せる。

- レーシングラインは**コース構築時に自動生成**される（`config/sim.yaml` の `racing_line.enabled`）
- 速度はコーナーで自動的に落ち、直線で最大速度まで上がる（横/縦加速度制限）
- 中心線抽出には**1周分の探索軌跡**が要る。1周し切る前に生成すると未探索区間の経路が荒れる
- pygame は真値ビューア（SLAM地図・経路は WebUI に表示）

## 車両スペック
- ホイールベース 0.230m / トレッド 0.155m / 最大ステア ±40° / 最大速度 3.0m/s
- ステアリング: アッカーマンジオメトリ / タイヤ半径 0.028m
- UART 250000bps（実機）/ LiDAR: LD06（360°・1°分解能・最大12m）

座標系: heading [deg] 0=East、反時計回り正。ステア正 = 左旋回（CCW）。
