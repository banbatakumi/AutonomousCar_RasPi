# SURGE Mk.2 GUI

React + TypeScript + Vite。`docs/architecture.md` §10 の設計に沿う。

タブは4枚。**同じテレメトリを見ているが、出す物の選び方が違う。**

| タブ | 誰のための画面か | 特徴 |
|---|---|---|
| **ラジコン**（既定） | 運転する人 | 速度・G・舵角をメータで。介入は数値ではなくランプ。設定は ⚙ ドロワー |
| **自動運転** | 開発する人 | 指令と実測を数値で並べ、遅れをそのまま読む。Phase 3 で経路・地図が入る |
| **診断** | 原因を追う人 | 現在値の全項目 + 時系列グラフ（uPlot・直近180秒） |
| **ログ** | 後で見る人 | `.sfl` 記録と mcap ライブ中継 |

## 動かす

### 1. 車体もPiも無しで（ハード無しの GUI 開発）

3つを別々のターミナルで。**この順に上げること**（publish 側が先だと購読が繋がらない
わけではないが、ログが読みやすい）。

```bash
cd surge_mk2

# ① WS サーバ（バスを購読して GUI に配る）
.venv/bin/python -m raspi.nodes.telemetry_node

# ② それらしいデータをバスに流す（cmd を購読して舵が反応する）
.venv/bin/python -m raspi.tools.bus_demo --allow-arm --faults

# ③ GUI（自動リロードが効く）
cd gui && npm install && npm run dev      # http://localhost:5173
```

`--faults` を付けると 60秒周期で E-Stop・低電圧・過熱を演出する。
**異常表示の作り込みはこれで確認する**（実車で異常を出して確かめるのは無理）。

### 2. シミュレータで走らせる（**指令に反応する。自律走行の開発台**）

`bus_demo` と違い、**UART プロトコルから下を丸ごと偽装する**ので、
`convert.py`・`ScanAssembler`・安全ゲートを含む実機と同じコードが全部動く。
LiDAR にはノイズ・欠損・遅延が入り、コース上の壁が点群として見える。

```bash
.venv/bin/python -m sim.run                  # 3つまとめて起動。Ctrl-C で全部落ちる
.venv/bin/python -m sim.run --course slalom  # コース指定
```

**この GUI にはシミュレータ用の操作を一切足していない。** 出るのは
ステータスバーの **SIM** バッジだけ（`link.sim`）。コース切替・ノイズ量・欠損率は
`sim.gui` 側にある。詳しくは [`sim/README.md`](../sim/README.md)。

### 3. 実機ログを流す（本物のセンサノイズ入り）

記録済みの `.sfl` を実時間で流す。**本物のセンサそのもの**だが、
指令には反応しない（走らせて試すならシミュレータ）。

```bash
.venv/bin/python -m raspi.nodes.replay_node logs/run.sfl --bus --loop
```

### 4. 実車

```bash
# Pi 上で
python -m raspi.nodes.io_node          # ★ --allow-arm を付けない限り DISARM 固定
python -m raspi.nodes.camera_node
python -m raspi.nodes.telemetry_node
```

ブラウザで `http://surge.local:8000/`。**GUI 本体も telemetry_node が配る**ので、
WS と同一オリジンになり接続先の書き分けが要らない。

Mac で GUI を編集しながら実車に繋ぐなら:

```bash
SURGE_HOST=surge.local npm run dev     # /ws だけ Pi に中継される
```

## ビルドして Pi に置く

`dist/` は `.gitignore` 済み。**Pi に node は要らない**。Mac でビルドして rsync する。

```bash
npm run build            # -> gui/dist
# 既存の rsync（PROGRESS.md）でそのまま Pi に載る
```

## 操作

| | デッドマン | 速度 | 舵 | 停止 |
|---|---|---|---|---|
| ゲームパッド | R2 を踏む | R2 前進 / L2 後退 | 左スティック X | B/○ |
| キーボード | **Space 長押し** | W,↑ / S,↓ | A,← / D,→ | Esc |

**離した瞬間に指令の送信そのものが止まる。** `speed=0` を送るのではない。
サーバは 150ms 無音で DISARM に落とし、io_node も 150ms で DISARM に落とす。
「止める指令を送る」設計だと、その指令が届かない状況（＝一番止めたい状況）で止まらない。

操縦権は**同時に1人**。2枚目のタブでデッドマンを握ると拒否される。
E-Stop だけは誰でも押せる。

## コードの地図

```
src/
├── bus/live.ts        20Hz のデータ置き場。**React の state には入れない**
├── bus/history.ts     診断グラフ用リングバッファ（10Hz × 180秒・常時記録）
├── ws/telemetry.ts    /ws/telemetry（msgpack バイナリ・20Hz）
├── ws/control.ts      /ws/control（JSON・操縦権・E-Stop）
├── input/useDriving.ts ゲームパッド + キーボード → 20Hz の cmd
├── render/LidarView   Canvas 2D。rAF で live を読む
├── render/CameraView  JPEG → ImageBitmap → Canvas
├── views/             RcView / AutoView / DiagView / LogView（＝タブ4枚）
├── components/        StatusBar(層B) / DriveBar(層B) / DiagStrip(層C) / DiagGrid / DiagCharts
├── components/rc/     ラジコン専用の計器（SpeedGauge / GMeter / SteerGauge / AssistLamps）
├── components/SettingsDrawer  ⚙ から出る設定ドロワー（中身は SettingsPanel）
├── store/ui.ts        イベント駆動の状態だけ（zustand）
└── format.ts          SI → 表示単位。**ここ以外で単位を変えない**
```

### メータの更新頻度は3種類ある

- **G メータ**だけ rAF で `live.vs` を直読する。軌跡を描くので 8Hz では点が飛ぶ
- **介入ランプ**も rAF。`tc_active` は1〜2フレームで落ちるため、**300ms ラッチ**して
  「介入したのに光らない」を防ぐ。class の付け外しだけで React state は使わない
- **速度計・舵角計**は 8Hz の `useNumbers()`。針の間は CSS の transition で埋める

### G メータの軸は実機で合わせる

`components/rc/GMeter.tsx` の `AX_SIGN` / `AY_SIGN`。IMU の取り付け向きに依存するので、
**前進加速で上・右旋回で右**に振れるかを実車で確認し、違えば符号を反転する。
重力は補正していない（坂では中心がずれる。見て楽しむ計器で、制御には使わないため）。

### 20Hz のデータを React state に入れない

`architecture.md` §10.4。流量で3つに分ける。

1. 点群・カメラ → `live`（可変オブジェクト）に置き、Canvas が rAF で読む
2. 数値表示 → **8Hz に間引いて** `useNumbers()` から配る（人間は 20Hz の数字を読めない）
3. 接続状態・操縦権・設定 → イベント時だけ zustand

### 型は Python 側の写し

`src/types.ts` は `raspi/msgs/types.py` の写し。**2箇所にあるのでズレうる。**
UART のパケット定義（`proto/`）のように生成器を挟むほどの規模ではないと判断している。
Python 側を変えたらここも直すこと。
