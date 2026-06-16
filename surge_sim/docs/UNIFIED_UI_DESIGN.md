# 実機/SIM共通UI 設計提案（ドラフト・未実装）

方針：**テレメトリのスキーマ（データ契約）を1つに固定し、その上で運用UI(Web)と
開発UI(pygame)の二刀流**。UIは2つだが、両者が同じスキーマを読む／書くことで、
実機・SIMどちらにも同じUIが接続できる。

> 本書は方針合意後のたたき台。コードはまだ書いていない。

---

## 1. 全体アーキテクチャ

```
  共有コア (変更最小)
  ┌─────────────────────────────────────────────┐
  │ core/ : Controller(50Hz), planners, interfaces│
  │ BackendBase ┬ SimBackend (物理+レイキャスト)   │
  │             └ RealBackend (UART 250000bps)    │
  └───────────────┬─────────────────────────────┘
                  │ ControllerSnapshot
                  ▼
        TelemetryCodec（共通スキーマへ直列化/復元）
                  │  TelemetryFrame(JSON/binary)
        ┌─────────┴───────────┐
        ▼                     ▼
  TelemetryServer        in-process
  (WebSocket, RasPi/SIM)  (SIMローカル)
        │                     │
   ┌────┴─────┐          ┌────┴─────┐
   ▼          ▼          ▼          ▼
 Web UI    pygame      pygame     (将来)
 (運用)   (network)   (local sim)
```

ポイント：
- **コアとバックエンドは現状のまま再利用**。新規は「スキーマ」「サーバ」「UIのスキーマ対応」だけ。
- pygameは2通りで動く：SIMローカル（in-process）／実機・遠隔（NetworkBackend or サーバ購読）。
- Web UIは `surge_sim/ui_web/` に新規構築（FastAPI + WebSocket + ブラウザ）。

---

## 2. データ契約（テレメトリスキーマ）

`core/interfaces.py` のデータクラス（VehicleState / LidarScan / ControlCommand /
LocalizationResult / OccupancyGrid）を、UIへ送る1つのフレームに統合する。
ハードウェア由来のテレメトリ（速度・加速度・電圧 等）は STM32 → RasPi の UART で
取得する想定（詳細は RealBackend 実装時に確定）。

### 2.1 ダウンリンク `TelemetryFrame`（RasPi/SIM → UI, 10〜30Hz）

```jsonc
{
  "type": "telemetry",
  "t": 12.34,                       // [s]
  "source": "sim" | "real",         // UIの出し分けに使用
  "drive_mode": "manual|auto|reactive",
  "sim_ctrl": { "paused": false, "speed_mult": 1.0 },   // sourceがsimの時のみ意味あり

  "vehicle": { "speed": 1.49, "accel": 0.02, "steer": 5.3 }, // 実機=オドメトリ/STM32

  "pose_est":  { "x": 3.1, "y": 0.5, "heading": 12.0,        // 推定姿勢(常にある)
                 "conf": 1.0, "src": "cheat|slam|ekf" },
  "pose_truth":{ "x": 3.1, "y": 0.5, "heading": 12.0 } | null, // SIM専用デバッグ

  "lidar": { "n": 360, "max_range_mm": 12000,
             "dist_mm": [ /* 360 × uint16, 0=範囲外, 別binaryフレームでも可 */ ] },

  "command": { "target_speed": 1.5, "target_steer": 5.0 },
  "planner": { "target_point": [3.6, 0.5] | null,            // 先読み点/ギャップ方向
               "has_path": true },

  "map": null | { "res": 0.05, "ox": -1.0, "oy": -1.0,        // SLAM占有格子(別binaryフレーム)
                  "w": 200, "h": 120 },

  "hw": null | { "volt_s": 16.4, "volt_p": 7.8, "cpu_temp": 52, // 実機のみ(任意)
                 "motor_err": 0 },

  "health": { "comm_ok": true, "estop": false }
}
```

### 2.2 アップリンク `CommandFrame`（UI → RasPi/SIM）

```jsonc
{ "type": "cmd", "name": "manual_input", "speed": 1.5, "steer": 5.0 }
{ "type": "cmd", "name": "set_mode", "mode": "reactive" }
{ "type": "cmd", "name": "estop" }            // 実機の緊急停止（最優先）
{ "type": "cmd", "name": "reset" }
{ "type": "cmd", "name": "pause", "value": true }      // SIM専用
{ "type": "cmd", "name": "speed_mult", "value": 2.0 }  // SIM専用
{ "type": "cmd", "name": "set_course", "course": "L-Shape Course" } // SIM専用
```

### 2.3 シーン情報 `SceneFrame`（接続時/コース変更時に1回）

```jsonc
{ "type": "scene", "source": "sim",
  "walls": [ [[x1,y1],[x2,y2]], ... ],        // SIMのみ(真値)。実機はnull
  "center_line": [ [x,y], ... ] | null }
```

実機では壁の真値が無いので `walls=null`。代わりに `map`(SLAM地図)を継続送信する。

---

## 3. 「真値」と「推定」の扱い（最重要・2026-06-16 確定運用）

実機には真の壁も真の姿勢も無い。よって**真値はワイヤ(テレメトリ/シーン)に載せない**。

- **Web UI（運用・実機相当）**：表示は「ロボットが知る世界」のみ＝`pose_est`・LiDAR・
  **SLAM占有格子（地図）**・SLAM由来の中心線/レーシングライン。真の壁(`walls`)・真の姿勢
  (`pose_truth`)は**サーバが配信しない**（`walls=None`, `pose_truth=None`）。表示範囲も
  占有格子(無ければ推定姿勢周辺)から決める。→ 実機/SIMでWeb UIは同一。
- **pygame（SIMデバッグビューア・in-process）**：`build_view(include_truth=True)` と
  真の壁入りシーンを使い、真値を重ねて「SLAM地図 vs 真値」を確認できる。
- 実装: `build_view(..., include_truth)`、サーバの送信は `include_truth=False`、
  運用 `get_scene` は `walls=None/center_line=None`（SLAM由来のみ）。

---

## 4. リアルタイム制約と安全機構

| 機能 | SIM | 実機 |
|---|---|---|
| pause / 速度倍率 | ◯（物理を止める/早送り） | ✕（実時間で動き続ける） |
| reset | ◯（初期姿勢へ） | △（停止＋姿勢推定リセット） |
| **E-STOP** | （任意） | **必須・最優先**。即 speed=0 送信 |
| 通信断時の自動停止 | — | **必須**。一定時間 cmd 途絶で speed=0 へ |

UIは `source` を見て、SIM専用ボタン群と実機専用（E-STOP等）を出し分ける。

---

## 5. 通信レート/帯域の見積り

- LiDAR: 360 × uint16 = 720 B/frame。20Hz で 14.4 KB/s → 5GHz WiFi(802.11ac)で余裕。
- 占有格子: 200×120 = 24,000 cell。生で送ると重いので **1〜2Hz の全体送信**（決定②）。
- テレメトリ本体(JSON)は数KB/frame。カメラを使う場合は別チャネル(binary WS)で分離。

---

## 6. 段階的移行プラン（低リスク順・各ステップで動作確認）

1. ✅ **スキーマ定義** `core/telemetry.py`（2026-06-16実装済）：
   `TelemetryFrame`/`CommandFrame`/`SceneFrame`、`build_telemetry(snapshot)`、
   LiDAR(uint16 LE mm)/占有格子(int8)のbinary encode/decode、`apply_command(controller,cmd)`。
   全roundtrip検証済み。既存挙動の変更なし（まだ誰もimportしていない）。
2. ✅ **pygameをスキーマ対応に**（2026-06-16実装済）：`SimRenderer` が
   `UIView`（TelemetryFrame＋LiDAR＋Scene）だけを読んで描画する形へ。
   `get_view()` をDI（in-process は `build_view(snapshot, scene)`）。
   車両は **pose_est** で描画、真値はSIM debugオーバーレイ（ズレ時のみ）。
   壁・中心線は SceneFrame 由来。**書き込み(操作)はまだ controller 直接**（③でCommandFrame化）。
3. ✅ **TelemetryServer 追加**（2026-06-16実装済）`backend/telemetry_server.py`：
   FastAPI WebSocket `/ws`。封筒JSON＋LiDAR binary(先頭タグ0x01)を毎フレーム、
   占有格子binary(タグ0x02)は低レート。CommandFrame受信→`apply_command`。
   操作権トークン(最初の接続者→claim_controlで移譲)、接続時Scene再送、操作者切断でestop。
   起動: `python main.py --mode sim --serve [--host H --port P]`。
   依存追加: fastapi / uvicorn[standard] / websockets / wsproto。
   実装メモ: FastAPI の websocket 引数は `from __future__ import annotations` と
   ローカルimportを併用すると型解決に失敗し403になる→fastapi系importはモジュール先頭で行う。
4. ✅ **Web UI**（2026-06-16実装済）：`surge_sim/ui_web/`（index.html/style.css/app.js）。
   WebSocketで封筒JSON＋LiDAR binary(0x01)を受信しCanvasに描画（壁/車両=pose_est/LiDAR/
   中心線/先読み点/真値オーバーレイ）。操作はキー/ボタン→CommandFrame送信。操作権/claim対応。
   サーバが `static_dir` で配信、コース一覧は `/courses.json`、コース切替時はsceneを再送。
5. **RealBackend 実装**（UART）＋ RasPi でヘッドレス Controller + TelemetryServer。
6. **pygame NetworkBackend**（任意）：実機を遠隔pygameで観測。

→ Phase3(SLAM)・Phase4(最適化) はこの土台の上に `map` / `racing_line` を足すだけ。

---

## 7. 決定事項（2026-06-16 確定）

| # | 論点 | 決定 |
|---|---|---|
| ① | エンコード | **封筒JSON＋配列binary**。テレメトリ本体はJSON（可読・デバッグ容易・スキーマ拡張が楽）、LiDARと占有格子は大きい数値配列なのでbinaryフレーム（テキスト膨張とパース負荷を回避。WebSocketのtext/binary両用） |
| ② | 占有格子の送信 | **全体をbinaryで1〜2Hz**送信＋姿勢/LiDARは20Hzで別送。重くなったら後で差分化 |
| ③ | Web UIの置き場所 | **`surge_sim/ui_web/` に新規構築**。SIMのTelemetryServerと実機RasPiの両方が同じUIを配信＝sim/real共通の単一クライアント |
| ④ | 接続管理・認証 | **認証なし**（private AP）。**操作クライアント1＋閲覧N**（操作権トークン）。接続/再接続時に Scene＋現在地図を即再送。**通信断タイムアウトで安全停止** |
| ⑤ | 座標系・単位 | **ワイヤはSI統一**：位置[m]・速度[m/s]・角度[deg]、heading 0=East反時計回り（surge_simコアと一致）。**LiDARのみ uint16 mm**（LD06がmmネイティブ・720B/frameと低帯域。0=範囲外/最大12000）。**mm↔m変換は RealBackend(UART)境界に閉じ込める** |

### 次に詰める（実装前の残課題）
- 操作権トークンの具体仕様（取得/解放/タイムアウト）。
- Scene/地図の再送トリガと初期同期の手順。
- ui_web の技術選定（素のJS or 軽量フレームワーク）。
