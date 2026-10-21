# SURGE Mk.2 GUI

React + TypeScript + Vite。`docs/architecture.md` §10 の設計に沿う。

タブは4枚。**同じテレメトリを見ているが、出す物の選び方が違う。**

| タブ | 誰のための画面か | 特徴 |
|---|---|---|
| **ラジコン**（既定） | 運転する人 | 速度・G・舵角をメータで。介入は数値ではなくランプ。設定は ⚙ ドロワー |
| **自動運転** | 開発する人 | 指令と実測を数値で並べ、遅れをそのまま読む。経路・占有格子の重畳は Phase 3 以降 |
| **診断** | 点検する人・原因を追う人 | 常設サマリ＋イベント帯＋サブページ6枚（下の「診断タブ」） |
| **システム同定** | 車両パラメータを測る人 | 試験の実行と結果の確認 |

記録（`.sfl`・mcap）の開始・停止はタブバーの `LogControls` に常設、ファイル一覧は診断タブの「記録」ページ。

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
.venv/bin/python -m sim.run                  # 4つまとめて起動。Ctrl-C で全部落ちる
.venv/bin/python -m sim.run --course fuji    # コース指定
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

ブラウザで `http://surge-mk2.local:8000/`。**GUI 本体も telemetry_node が配る**ので、
WS と同一オリジンになり接続先の書き分けが要らない。

Mac で GUI を編集しながら実車に繋ぐなら:

```bash
SURGE_HOST=surge-mk2.local npm run dev     # /ws だけ Pi に中継される
```

## ビルドして Pi に置く

`dist/` は `.gitignore` 済み。**Pi に node は要らない**。Mac でビルドして rsync する。

```bash
tools/deploy.sh          # GUI をビルドして rsync（既定）。Python だけなら --no-gui
```

## 操作

**ARM を保持している間だけ走る。**（詳細と割り当ての全部は `src/input/useDriving.ts` の冒頭）

| | ARM / DISARM | 速度 | 舵 | ブレーキ | ギア | 即停止 |
|---|---|---|---|---|---|---|
| キーボード | `Enter`（トグル）・画面の ARM ボタン | W,↑ / S,↓ | `A` / `D` | `Space` | ← ダウン / → アップ | `Esc` |
| ゲームパッド | ×（トグル） | R2（アクセル） | 左スティック X | L2（力加減） | L1 ダウン / R1 アップ | ×（ARM 中） |

補機: 灯火 `L`／十字キー上、クラクション `H`／□、パッシング `P`／十字キー下、ウィンカー 十字キー左右、
サイドブレーキ △、前後カメラ入れ替え ○。

- **止める手段**: `Esc`・画面の E-STOP ボタン・ARM 中のパッドの ×。どれも ARM を下ろし、
  もう一度走らせるには人が ARM し直す（E-STOP は誰でも押せる。操縦権は要らない）
- **ARM が自動で下りるとき**: タブが背後に回った／操縦権を失った（他の端末の E-STOP・接続の切断）／
  車が E-STOP 中・ARM 封印中／無操作が続いた。理由は ARM ボタンの左に出る
- ウィンドウのフォーカスを失っただけ（`blur`）では DISARM しないが、**キーの押下状態は捨てる**
  （押していたキーが残って走り続けないように）
- 文字を打つ欄にフォーカスがある間だけ、操縦キーは効かない（`Esc` は常に効く）。スライダや
  ボタンにフォーカスが残っていても操縦キーは効く

送信は画面の描画（rAF）に合わせ、**中身が変わった指令はそのフレームで、同じ中身の繰り返しは
50Hz 以下**で送る。ARM を下ろした瞬間に DISARM を1発送り、あとは送信そのものを止める。
サーバは `config/vehicle.toml` の `[safety] cmd_deadman_ms`（150ms）無音で DISARM に落とし、
io_node も同じだけで落とす（自律走行中だけ、Wi-Fi の瞬断を `auto_link_grace_ms` まで待つ）。

操縦権は**同時に1人**。2枚目のタブで ARM すると拒否される。

確認のダイアログは `window.confirm` を使わない（`components/ConfirmDialog.tsx`）。ブラウザの
ダイアログは開いている間 JavaScript を止めるので、その間 E-STOP も送信も効かなくなる。

## コードの地図

```
src/
├── bus/live.ts        20Hz のデータ置き場。**React の state には入れない**
├── bus/history.ts     診断用リングバッファ（10Hz × 180秒・常時記録）。数値の系列＋フラグのビット列＋イベントログ
├── ws/telemetry.ts    /ws/telemetry（msgpack バイナリ・50Hz）
├── ws/control.ts      /ws/control?catalog=changed（JSON・操縦権・E-Stop。auto.catalog は接続時の1通目のみ → App.tsx が前の値を保つ）
├── input/useDriving.ts ゲームパッド + キーボード → cmd（rAF。変化はそのフレームで、繰り返しは 50Hz 以下）
├── render/LidarView   Canvas 2D。rAF で live を読む
├── render/CameraView  JPEG → ImageBitmap → Canvas。進路ガイドの投影は render/cameraModel.ts（魚眼/補正映像/未校正ピンホール）
├── hooks/useSnapshot  📷 撮影（GET /snapshot/<cam>.png → Mac にダウンロード。校正用の生画像）
├── render/MapCanvas   世界座標のキャンバス（地図生成タブ・低頻度）
├── views/             RcView / AutoView / DiagView / SysIdView（＝タブ4枚）
├── components/        StatusBar(層B) / DriveBar(層B) / LogControls ほか
├── components/ErrorBoundary / ConfirmDialog  タブごとの例外の受け皿 / 止まらない確認ダイアログ
├── components/diag/   診断タブの部品（下の「診断タブ」）
├── components/rc/     ラジコン専用の計器（SpeedGauge / GMeter / SteerGauge / AssistLamps）
├── components/SettingsDrawer  ⚙ から出る設定ドロワー（中身は SettingsPanel）
├── store/ui.ts        イベント駆動の状態だけ（zustand）
└── format.ts          SI → 表示単位。**ここ以外で単位を変えない**
```

### 診断タブ

用途は3つ: **走行前の点検・異常の原因追跡・電源と熱の監視**。

```
上段（常設）  [概要][電源・熱][駆動・制御][通信][センサ・Pi][記録]   ⏸  30秒|60秒|180秒
              常設サマリ（走行可否・電源2系統・温度・リンク・STM32/MD・センサ・RasPi・ノード）
              イベント帯（ARM・AUTO・E-STOP・フォルト・通信・TC・ABS・TV・自動停止）
下段          サブページ。**開いているページのグラフだけ描く**
```

| ファイル（`components/diag/`） | 役割 |
|---|---|
| `view.ts` | サブページ・時間幅・一時停止の状態と、**全グラフ共通の再描画の刻み（4Hz・1本）** |
| `checks.ts` | 走行前チェックの判定（**判定はここだけ**。サマリと一覧が同じ結果を使う） |
| `chartDefs.ts` | グラフ定義。**1枚に載せる単位は1つ**（V と A、ms と Hz を混ぜない） |
| `Chart.tsx` / `EventTimeline.tsx` / `uplotBase.ts` | uPlot。窓とカーソルを全グラフで共有、フラグの帯としきい値線を重ねる |
| `HealthStrip.tsx` / `Card.tsx` | 常設サマリのタイルとカード部品 |
| `Route.tsx` / `PowerFlow.tsx` | 経路図（箱を線でつなぎ、区間の数字と状態色を線に出す）。通信ページと電源ページで使う |
| `pages/` | サブページ6枚 |

- **グラフを足す**: `bus/history.ts` の `SERIES` と `pushHistory` に系列を足し、`chartDefs.ts` に定義を足して、
  ページの `CHARTS` に並べる
- **チェック項目を足す**: `checks.ts` の `runChecks` に1行。しきい値は `format.ts`
- **カードは4列の格子に載せる**（1枚 = 1枠、`wide` = 2枠。同じ行は高さも揃う）。ページのカードは
  枠の合計が4の倍数になるように並べる。値は折り返さない（折り返すとそのカードだけ高くなる）
- **一時停止しても記録は止まらない**（履歴は常時貯める。止めるのは表示の窓だけ）
- 色は `uplotBase.ts` の `tones()` が CSS 変数から引く。**グラフ側に色の値を書かない**
- 画面に数秒だけ出す通知（操縦権の拒否・記録削除の失敗・シャットダウン拒否など）は `store/ui.ts` の `notify()`（ステータスバーのピル）。
- パッドの `timestamp` が2秒進まず入力が中立でないときは、圏外とみなして入力を中立にする（`input/useDriving.ts`）。
  **ただし `timestamp` が「入力が変わらなくても進む」機種でだけ**判定する（値の変化でしか進まない機種では、
  一定に握る正常な操作と圏外を区別できないため）。切断は `gamepaddisconnected` でも拾う。
- `status.pi`（CPU・メモリ・スロットリング）と `status.nodes`（ノードの生存申告）は `/ws/control` で 1Hz。
  Pi 以外（シム・`bus_demo`）では `pi.available=false` で「取得不可」になる

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

### バスのメッセージ型は生成物

`src/generated/msgs.ts`（`VehicleState`・`Scan`・`LinkDiag`・`AutoState`・`AutoMapMsg`）は
`raspi/msgs/types.py` から `python3 config/gen_msgs.py` が生成する。**`types.py` が唯一の正**
（UART のパケット定義を `protocol.toml → Python/C` で生成しているのと同じ形）。

手で写すと片方だけ直してもどのツールもエラーを出さず、**画面に静かに `undefined` が出る**ため生成にしている。Python 側の型を変えたら `config/gen_msgs.py` を
再実行すること（CI は `--check` で生成物の陳腐化を検出する）。

`src/types.ts` に残っているのは、Python 側に対応物が無い GUI 固有の型だけ
（`ControlStatus`・`CmdOut`・`Snapshot`・`MapData` など）。
