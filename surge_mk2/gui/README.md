# SURGE Mk.2 GUI

React + TypeScript + Vite。`docs/architecture.md` §10 の設計に沿う。
今回のスコープは **運転ビュー1枚**（他タブは枠のみ）。

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

### 2. 実機ログを流す（本物のセンサノイズ入り）

`bus_demo` は運動学だけのモックで、センサのばらつきは入っていない。
記録済みの `.sfl` があるならこちらが正しい（`architecture.md` §11）。

```bash
.venv/bin/python -m raspi.nodes.replay_node logs/run.sfl --bus --loop
```

### 3. 実車

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
├── ws/telemetry.ts    /ws/telemetry（msgpack バイナリ・20Hz）
├── ws/control.ts      /ws/control（JSON・操縦権・E-Stop）
├── input/useDriving.ts ゲームパッド + キーボード → 20Hz の cmd
├── render/LidarView   Canvas 2D。rAF で live を読む
├── render/CameraView  JPEG → ImageBitmap → Canvas
├── components/        StatusBar(層B) / DriveBar(層B) / DiagStrip(層C)
├── store/ui.ts        イベント駆動の状態だけ（zustand）
└── format.ts          SI → 表示単位。**ここ以外で単位を変えない**
```

### 20Hz のデータを React state に入れない

`architecture.md` §10.4。流量で3つに分ける。

1. 点群・カメラ → `live`（可変オブジェクト）に置き、Canvas が rAF で読む
2. 数値表示 → **8Hz に間引いて** `useNumbers()` から配る（人間は 20Hz の数字を読めない）
3. 接続状態・操縦権・設定 → イベント時だけ zustand

### 型は Python 側の写し

`src/types.ts` は `raspi/msgs/types.py` の写し。**2箇所にあるのでズレうる。**
UART のパケット定義（`proto/`）のように生成器を挟むほどの規模ではないと判断している。
Python 側を変えたらここも直すこと。
