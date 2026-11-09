# SURGE Mk.2 シミュレータ（Mac 専用）

実車が無くても、**実機と同じ制御コードのまま**コース上を走らせられる 2D シミュレータ。
ラジコンモードで人が運転でき、そのまま自動運転ノードの開発台にもなる。

## 設計の原則 — シミュレータが偽装するのは STM32 だけ

`SerialLink` の位置に `SimLink` を差す。これが最も深い切り口で、**`io_node` より上流は
1 行も変えずに動く**。以下はすべて実機と同一のコードが走る:

`protocol.toml` のパケット定義 / CRC-16 とフレーミング / `convert.py` の SI ↔ 整数換算と
クランプ / `ScanAssembler` の鏡像反転 / `LinkTracker` と PING-PONG 時刻同期 /
`--allow-arm` ゲート / `cmd` 150ms タイムアウト → DISARM / `bus_bridge` / `telemetry_node` / GUI。

```
ブラウザ(React GUI)  ← SIM バッジを出すだけ。それ以外シムの存在を知らない
   │ WS
telemetry_node ──────── ZMQ ──────── io_node
                                        │  link.poll() / link.send()
                            ┌───────────┴───────────┐
                       SerialLink                SimLink        ← ここだけ差し替え
                            │                        │
                        実 STM32                 仮想 STM32
                                                     │ UDP 127.0.0.1（私設チャンネル）
                                                 シミュレータGUI (pygame)
```

**シミュレーションの操作は実機 GUI に入れない。** コース切替・ノイズ量・欠損率は実機に
対応物が無い概念で、それらの UI を実機 GUI に持たせると、表示判定を 1 つ間違えた瞬間に
実車の画面にシム用コントロールが出る。GUI がシミュレータを知らなければその事故は起きない。

**例外は SIM バッジだけ**（`LinkDiag.sim` の 1 ビット）。これは利便性ではなく安全要件で、
シムと実機の画面が見分けられないと「シムのつもりで `--allow-arm` した実車が動く」が起きる。

同じ理由で **シム ↔ シミュレータGUI は ZeroMQ バスを使わず UDP ループバック**にしてある
（`channel.py`）。バスに乗せると `raspi/msgs/types.py` と `raspi/bus/zbus.py` に
シムの語彙が入り込む。

## 動かす

```bash
cd surge_mk2
.venv/bin/pip install -r sim/requirements.txt     # 初回だけ（pygame / pillow）

.venv/bin/python -m sim.run                        # ★ これだけ
```

`io_node --sim` / `telemetry_node --no-camera` / `planning_node` / `sim.gui` の 4 つを
立ち上げ、出力に印を付けて1画面に流し、ブラウザを開く。
**Ctrl-C か、俯瞰ビューの窓を閉じると全部止まる。**

**自動運転もそのまま試せる。** ブラウザの「自動運転」タブでモード（Follow the Gap）を
選び、`Enter` で ARM してから「自律走行を開始」を押す。LiDAR ビューに planner が
選んだギャップが重なる。engage しなくても判断だけは流れているので、**手動で走らせ
ながら planner の狙いを検分できる。**

俯瞰ビューが**異常終了したときだけ**は巻き添えにしない（二重起動や描画のバグで
落ちただけなら、走っている車まで止める理由が無い）。**終了コードで区別**していて、
正常終了 0 なら「人が終わりにした」、非0なら「事故」とみなす。

`planning_node` は**どう終わっても巻き添えにしない。** 自動運転のコードをいじって
いる最中に落ちても、手動で走らせている車まで止める理由が無いため（自律指令が途絶
すれば telemetry_node が制動に落とすので、危険側には転ばない）。上げ直すのは
`python -m raspi.nodes.planning_node` を別ターミナルで叩くだけでよい。

```bash
.venv/bin/python -m sim.run --course toyota2    # コース指定（名前だけでよい）
.venv/bin/python -m sim.run --no-planning       # 自動運転ノード無し
.venv/bin/python -m sim.run --list              # コース一覧
.venv/bin/python -m sim.run --no-gui            # 俯瞰ビュー無し
.venv/bin/python -m sim.run --kill-stale        # 古いプロセスを落としてから起動
.venv/bin/python -m sim.run --safe              # --allow-arm を付けない
```

`sim.run` は起動前に**詰まりどころを潰す**（[`docs/development.md` §8.3](../docs/development.md#83-mac-でシミュレータが起動しない)）:
Rosetta で起動されたら arm64 で立て直し、ポートや `io` endpoint を掴んでいる古いプロセスを
**起動時刻つきで**報告する。古い `bus_demo --faults` が動きっぱなしだと偽データが GUI に出る。

### 手で4つ起動する場合

```bash
.venv/bin/python -m raspi.nodes.io_node --sim --course sim/courses/normal.json \
    --allow-arm --max-speed 3.0 --max-steer 0.524 --no-heartbeat --no-leds
.venv/bin/python -m raspi.nodes.telemetry_node --no-camera
.venv/bin/python -m raspi.nodes.planning_node        # 自動運転タブを使うなら
.venv/bin/python -m sim.gui
# ブラウザ http://localhost:8000/
```

`io_node --sim` の `--course` は `sim.run` と同じくコース名（既定 `normal`）かパスを受ける。
`--no-heartbeat --no-leds` は Mac に GPIO が無いための指定（付けなくても警告が出るだけで動く）。
`--allow-arm` は手で起動するときは明示が要る（**実機と同じ手順を崩さない**ため）。
`sim.run` は `--sim` 専用の入口で実車に届く経路が無いので、そちらでは既定で付ける。

### シミュレータGUI の操作

**ウィンドウはリサイズできる。** コースは横長なので表示倍率は幅で頭打ちになり、
レイアウトを詰めるより窓を広げる方がずっと効く（1240px で 88px/m、1600px で 126px/m）。

右のパネルは**常時は走行の数値**（速度・舵角・位置・超音波・有効点数）を出す。
シムのパラメータは set-and-forget なので `P` で開くページに退けてある
（実機 GUI が設定を ⚙ ドロワーに隠しているのと同じ作法）。

| | |
|---|---|
| `P` | シムの設定ページ（コース一覧・LIDAR・ODOM・LINK）を開閉 |
| `M` | **コースエディタを別ウィンドウで開く**（常に新規コースで始まる。既存は `O` で開く） |
| コース名をクリック / `N` | コース切替（車両はそのコースのスタート姿勢へ） |
| `↑` `↓` | 行の選択（閉じているときは設定ページを開く） |
| `←` `→` | 選択中のパラメータを増減（`Shift` で 10 倍） |
| `R` / `E` / `T` / `D` | 姿勢リセット / E-STOP / 軌跡消去 / パラメータ既定値 |
| `Esc` / `Q` | 終了 |

**同じ操作はパネル下部のボタンにも出ている。** ショートカットを覚えなくても全機能に
届くようにしてあり、ボタンにキーを併記してあるので使ううちに覚えられる。

## モデル化してあるもの / していないもの

| 入れてある | 入れていない |
|---|---|
| 自転車モデル（後輪車軸中心） | タイヤのスリップ角・横滑りの動力学 |
| 操舵の**むだ時間 + 1次遅れ** + レート制限 + 舵リンクのガタ・不感帯 + 簡易グリップ限界（摩擦円） | TC / TV の介入（ABS・TC は旗だけ。TC の旗は駆動の加速度が上限で頭打ちの間に立てる作り物） |
| 速度はファームの PI を写した `SpeedController`（`speed_kp=0` なら1次遅れ）+ `accel_limit` | カメラ画像の生成 |
| `brake_torque` / `torque_mode` / ローンチコントロール | 路面の勾配 |
| 車体外形（多角形）を使った衝突判定 | 電流・温度・電圧の物理的な根拠 |
| 仮想 LD06（ノイズ・点欠損・**セクタ欠損**・伝送遅延・走査ゆがみ） | |
| UART の線上時間（1Mbps）による順番待ち | |
| 仮想 STM32 の独立した時計（既定 +3378ppm） | |
| `COMMAND` 100ms 途絶 → 最大制動。途絶・E-Stop・DISARM 中は**舵角を凍結**（ファームの `applied_steer_rad` と同じ。直前の参照舵角を保つ） | |
| ファームの ARM 規則: 起動直後・E-Stop 解除後・途絶（100ms）復帰後は、ARM ビットが一度 0 になるまで ARM を受け付けない（LOG WARN "ARM ignored: ..." を1回送る）。`mode=3`（予約）は現モード維持 | |
| センサの倍率誤差（`SimParams.speed_scale_err`・`gyro_scale_err`。**既定 0**）。`sim.bench --errors real\|harsh` が `sim.slam_bench` の同名プリセットと同じ車速・ジャイロの倍率とバイアスを入れる | |
| 場所・時刻で変わる路面の摩擦（`surface.py`。`sim.bench --mu/--surface/--mu-at` だけが使う。横の限界と、加速・速度指令での減速の上限に効く） | タイヤが徐々に逃げる動き・後輪の巻き込みとロック・制動モードの減速の低下 |
| 走行中の揺れ（`SimParams.vib_*`。実機で測ったヨーレート・車速・実舵角の約 11Hz の雑音。**既定は 0**、`sim.bench --real-noise` で実機の値が入る） | 揺れの原因（舵指令のばたつきに車が応える動き）そのもの。テレメトリの値に足すだけで、車の運動は変えない |

**自動停止（`auto_stop`）はファームの判定を写してある**（`sim/stm32.py` の `_auto_stop_obstacle`、定数は
`vehicle.h` の `VEHICLE_AUTO_STOP_*` と同じ）。停止距離 `v×遅延 + v²/(2a) + マージン（CONFIG_SET、既定 15cm）`
（遅延 = LiDAR 1 回転 0.1s ＋ CONFIG_SET の `MD_TORQUE_LAG_S`、a = CONFIG_SET の `BRAKE_DECEL_M_S2`）以内で、進行方向（ほぼ静止なら指令の向き）・車幅 ±15cm の矩形に LiDAR の点が 3 点以上あれば作動し、
止まるまで保持する。超音波は至近（車体先端から 5cm 未満）と、LiDAR が 300ms 来ないときの停止距離判定の代わり。
違い: LiDAR の点は UART を通さず `VirtualLidar.last_scan`（セクタを撃った時点の値）から読む。
`NO_HEARTBEAT`・`AUTO_STOP_BLIND`（TELEMETRY の予約ビット）は常に 0。

**そのほかファームに合わせたもの。** 目標車速は ±5m/s にクランプし、実車速がそれを超えたら加速させない。
ブレーキ・torque_mode の間は目標車速を実車速に合わせ続ける（離したあとに 0 から数え直さない。サイドブレーキと
DISARM は 0）。フェイルセーフ（途絶・E-Stop）で停車して 30 秒たつと駆動電源を切り、ARM を入れ直すまで入れない。
ロック中（再 ARM 待ち）の ARM は駆動電源も入れない。`steer_link_*` はシムに換算が無い（舵角は最初から路面舵角）ので
CONFIG_SET の値を持つだけ。起動待ちは `SimParams.md_boot_s`（MD が使えるまで ARM を通さない秒数、**既定 0**）と
`lidar_power_gate`（1 で LiDAR は ARM を要求している間と外れて 5 秒だけ回る、**既定 0**）で再現できる。
左右の車輪速は旋回で差が付く（ヨーレート×トレッド/2）。衝突したら壁へ向かう動きだけを止め、逆向きなら離れられる。

**LiDAR のセクタは、撃つ時刻の姿勢で撃つ**（`advance_to` が長く進んでも、溜まったセクタが終点の姿勢に
まとまらない）。

**E-Stop は自動発動しない。** GPIO6 ハートビートが Mac に無いため（`--no-heartbeat` と
同じ扱い）。シミュレータGUI の `E` で手動発動できる。

車両パラメータは `config/vehicle.toml` の `[dynamics]`（車両の校正 `docs/calibration.md` で測る）と `[sim]`（シムの車だけの値）を見る。
直せば全部が追随する。

## わざと鏡像で出している

実機の LD06 は裏向きに実装されていて、`ScanAssembler` が
`車両角 = (360 − センサ角) % 360` で戻している（`raspi/msgs/convert.py` の `ScanAssembler`）。

**シミュレータは反転した状態で `LIDAR_SECTOR` を出す。** 素直に車両座標で出すと、
反転処理が壊れていても画面が正しく見えてしまい、バグを永久に検出できない。

検算には**左右非対称なコース**を使うこと（対称だと反転処理が壊れていても画面が
正しく見えてしまう）。同梱の `toyota2.json` や `circuit_chicane_*.json` は左右の旋回が混在するので、
そのまま検算に使える。

## コースを自作する

コースは `sim/courses/` に置く（`python -m sim.run --list` で一覧。`sim.run` の既定は `normal`）。
大会のコースは `toyota2`（toyota 大会と同形）・`magp1`（コース写真から作った PNG、分岐あり）・`magp2`（magp 大会用、壁モード）。

### エディタ（`python -m sim.editor`）

```bash
.venv/bin/python -m sim.editor            # 新規
.venv/bin/python -m sim.editor normal     # 既存を開いて編集
.venv/bin/python -m sim.editor 写真.png    # 画像から作る
```

**シミュレータGUI の `M` からも開ける**（常に新規コースで始まる。既存は `O` で開く）。
俯瞰ビューと別プロセスにしてあるのは、エディタが「未保存の状態を持つ作成ツール」だから（同じプロセスだと
シムを止めた瞬間に作りかけのコースが消える）。保存したらシミュレータGUI の `N` か設定ページから選び直す。

頂点をクリック（直線）／ドラッグ（直前の直線に接する円弧）でつなぐ。モードは3つ（操作の詳細は `sim/editor.py` の docstring）:

| モード | 描くもの | 保存形式 |
|---|---|---|
| **中心線** | 中心線のループ。道幅から左右の壁を自動生成（壁は常に平行・スタート姿勢は中心線上） | `path`（直線・円弧の区間列。手書き JSON と同じ形式） |
| **壁** | 壁そのもの（外壁＋内壁など何本でも）。中心線は細線化で自動導出、スタート地点は別に置く | `"mode": "wall"`（`sim/wall_track.py`）。分岐があると中心線の導出に失敗し `centerline: null` で保存される（中心線を前提にする `slam_bench` の真値の運転手などだけが使えない） |
| **画像** | 写真・図から壁の色を拾って占有格子にする。ノイズ消し・手での消去・外形の実寸合わせ・スタートと障害物の配置 | `<名前>.png` ＋ `.json`（`sim/image_course.py`） |

プレビューは俯瞰ビューと同じ描画コード（`sim/ui.py` の `map_surface`）を通している。

### JSON を手書きする（中心線方式）

中心線を直線と円弧でつなぎ、**道幅から左右の壁を自動生成する**。同梱の `normal.json` が見本。

```json
{
  "name": "サーキット",
  "width": 1.0,                 // 道幅 [m]
  "resolution": 0.02,           // 格子の刻み [m/px]
  "origin": [1.0, 1.0, 0.0],    // 中心線の始点 x, y, 向き[rad]
  "loop": true,                 // 始点と終点のずれを検査して警告する
  "path": [
    ["straight", 4.0],          // 4m 直進
    ["arc", 1.2, 90],           // 半径 1.2m を左に 90度（反時計回りが正）
    ["straight", 2.0],
    ["arc", 1.2, 90],
    ["straight", 4.0],
    ["arc", 1.2, 90],
    ["straight", 2.0],
    ["arc", 1.2, 90]
  ]
}
```

**中心線そのものが経路追従の正解データになる**（`course.centerline`）。

### PNG 方式

PNG を描いて、同名の JSON を隣に置く。**白 = 走行可、黒 = 壁。**

```json
{
  "name": "オーバル",
  "resolution": 0.02,        // m/px
  "origin": [0.0, 0.0],      // 画像左下ピクセルの世界座標 [m]
  "start": [1.0, 1.0, 0.0],  // 初期姿勢 x[m], y[m], yaw[rad]
  "wall_threshold": 128      // 輝度がこれ未満を壁とみなす
}
```

**PNG 方式で一番やりがちな間違いはスタート姿勢が壁に埋まっていること。** 車が一切動かず、
超音波 2cm → `auto_stop` が効き続けるという分かりにくい症状になる。
`python -m sim.courses.make_courses` と `create_sim_link()` の両方で検出して警告を出す。

### 共通の欄と判別の規則

PNG コースの JSON に `"obstacles": [[x, y, 半径], ...]` を書くと、壁と別に孤立障害物を刻める。

コースの JSON に `"surface": {"mu_scale": 1.25, "mu_long_scale": 1.1}` を書くと、そのコースの床のグリップを
校正の床（`[limits]`）の何倍にするかを決められる（横・前後。`sim/surface.py`）。会場の床は校正の床と違うので、
床が分かっている会場のコースに置く。**倍率は実機の走行記録から合わせ込まない**（校正で測るか、分からなければ
`sim.bench --mu` で振って planner が破綻しないかを見る）。`sim.bench --mu` はこの床を置き換える。

判別は `sim/course.py` の `Course.load`: `"mode": "wall"` なら壁モード、`"path"` があれば中心線方式、
どちらも無ければ隣の PNG を占有格子として読む。

## ベンチ — 数値で比べる

見た目では「なんとなく速い」しか分からない。**数字で出してから採否を決める。**
**真値を読めるのはベンチだけ**（シムの真値は UDP の私設チャンネルにしか出ず、mcap にも残らない）。

### planner の閉ループ評価（`sim.bench`）

```bash
.venv/bin/python -m sim.bench --course normal
.venv/bin/python -m sim.bench --mode de --time 180 --set safety_half_width=0.12
```

**プロセスもバスも GUI も使わず、1プロセスの中で `io_node` と同じことをやる**（`ScanAssembler` の鏡像反転・
`LinkTracker`＋時刻同期・`command_from_cmd` のクランプを通る）ので、「ここで走れば `sim.run` でも走る」。
`vehicle.toml` の `[control]` を仮想 STM32 へ全部入れる。

出るのは**計画周期（平均/中央/最大 ms）・自己位置誤差・向きの誤差・横偏差・周回数・走行距離・衝突回数・
ラップタイム・地図の正確さ**。

★ベンチは**実時間**で回る。並列は 2 本まで。1 行目の「計画 … 最大」が 90ms を超えた走行と、
「指令の間隔 最大」が 150ms を超えた走行（★が付く）は捨てて走らせ直す（後者はホストが詰まって仮想 STM32 の
デッドマンが車を止めた走行で、走りの問題ではない見かけの衝突が出る）。

そのほかのオプション（全部は `--help`）:

| オプション | 中身 |
|---|---|
| `--set KEY=VALUE` | planner の設定。**GUI から隠した項目（`ParamSpec.hidden`）も上書きできる** |
| `--errors real\|harsh` | 車速・ジャイロの倍率とバイアス（`sim.slam_bench` の同名プリセットと同じ） |
| `--sim KEY=VALUE` | センサ誤差（`sim.params.SimParams`）を上書き |
| `--real-noise` | **実機で測った走行中の揺れ**（ヨーレート・車速・実舵角。約 11Hz）をテレメトリに足す（`sim.params.REAL_VIBRATION`）。★シムの既定の雑音は実機よりずっと小さいので、安全装置の判定を変えたらこれ有り・無しの両方で走らせる |
| `--vehicle KEY=VALUE` | **真値の車だけ**諸元を変える（`sim.vehicle.VehicleSpec` の欄名。例 `steer_servo_gain=0.877`）。同定値が外れていたときに走りが壊れないかを見る |
| `--assume KEY=VALUE` | **planner が信じている車**の諸元だけ変える（`--vehicle` の逆） |
| `--control KEY=VALUE` | **シムの STM32 だけ** `[control]` の値を変える（「Pi は新しいのにファームに値が入っていない」場合の走り） |
| `--obstacle X,Y,R` | 円柱の障害物を置く（繰り返し可） |
| `--mover S0,SPEED[,D,R,T0,L]` | planner の経路に沿って動く円柱（他車の代わり。速度が負なら逆走） |
| `--hijack T:DUR:STEER:SPEED` | 走行開始から T 秒後に DUR 秒だけ指令を乗っ取り、経路から外す・壁に当てる（例 `--hijack 14:1.2:0.5:1.0`） |
| `--plan-slowdown K` | `plan()` を K 倍遅く見せる（待つ間も車は進む）。Pi で回した時間との比を入れると、計算の遅れを Pi 相当にできる |
| `--obs-log` | 障害物まわりの判断（reason の移り変わり、横へ避けられなかった理由つき）を時刻つきで出す |
| `--trace CSV` | RACE 中の 100Hz の記録（真値の位置・速度、指令速度、プロファイル速度、横偏差、車体と壁の距離） |

結果の「車の使い方」: 舵の張り付き・横G p99・要求中の実加減速・経路の速度からの出過ぎ・`plan()` の p99。
「回避」: 横ずらしの回数と計算時間、Hybrid A* の回数と時間。

### 分岐のあるコースの評価（`sim.bench --mode slam2d_route`）

```bash
.venv/bin/python -m sim.bench --course toyota2 --mode slam2d_route \
    --routes sim/routes/toyota2.json --switch 200:B --signal 260:right --set explore_laps=1
```

- `--routes`: 経路の設定（`raspi/auto/route_config.py` の形）。**シムの世界座標**で書けば
  bench が SLAM の map フレームへ直して planner に渡す
- `--switch T:G` / `--signal T:V`: 時刻 T に経路グループの切替・信号の値を送る（GUI の A〜D
  ボタン・`route/select` と同じ口）
- `--map` の走行は、保存地図を真のコースへ重ねて（`slam_bench.align_map`）自己位置の誤差と、真の位置から
  経路までの距離（「真の横偏差」）を出す。planner の自己申告（「横偏差」）とは別の数字
- 地図作成が終わると bench が「走行」を押す（DONE → 保存した地図を `request_load`）。
  保存先は一時ディレクトリ（実機の `saved_maps/` を汚さない）
- 結果に**周回ごとに真値で通った道路グラフのエッジ**（真のコースから作ったグラフの番号）と、
  停止点に止まったときの誤差（真値／推定）が出る。**狙った枝を通れたかはエッジの列で見る**
- ★地図作成の Follow the Gap は分岐で内側の輪に入り込み、何周か回ってから抜けることがある
  （toyota2 で約3分）。`slam_bench` は `--route <json>:<グループ>` で真値の運転手を
  好きな経路で走らせられる（中心線の無いコースでは出発点からの最短の周回が既定）

### ラップタイムの評価（保存済み地図から走る）

```bash
# 地図作成から通しで走り、できた地図を saved_maps/sim_toyota として残す（約5分）
.venv/bin/python -m sim.bench --course toyota --mode slam2d_route --time 330 --save-map sim_toyota
# 残した地図から LOCATE → RACE だけを走る（地図作成を飛ばす。1本 約2分）
.venv/bin/python -m sim.bench --course toyota --map sim_toyota --locate-hint 0,0 \
    --mode slam2d_raceline --time 100 --set v_max=3.0 --trace /tmp/t.csv
```

- `--map NAME`: `saved_maps/NAME` の**写し**を一時ディレクトリに置いて読む（実機の地図は書き換えない）。
  `--routes` を併せて渡すと、地図と一緒に経路の設定として読ませる
- `--locate-hint X,Y`: LOCATE に渡すおおよその位置（地図座標）。★toyota2 は対称で候補を絞り込めず、
  LOCATE が 12 秒ごとにやり直して**周回に入れない回がある**。bench が作った地図は原点が出発位置なので `0,0`
- `--save-map NAME`: bench が作った地図を `saved_maps/NAME` に残す（`--map` で使い回す）
- 保存地図は `saved_maps/`（gitignore）。無ければ `--save-map NAME` で作る
- 結果の読み方:
  - **真値ラップ**: RACE の出発位置に置いたゲートを跨いだ時刻（プランナーの周回カウンタに頼らない）。
    1周目は発進を含むので、比べるのは2周目以降
  - **理想ラップ**: 速度プロファイルどおりに走った場合の1周。真値ラップとの差が「追従で失っている分」
  - **車体と壁の最小距離**: 車体外形と真の壁の距離（周ごと）。衝突の手前の余裕で、0 = 接触

### 路面を変えて走らせる（走行中の学習の検証）

`slam2d_route` の `grip_learn`（`raspi/nav/grip.py`）は、真値の路面を変えた車で確かめる。路面を変えるオプションは
**真値の車だけ**に効く（planner は `config/vehicle.toml` のまま）。横は 1 段に 1 周かかるので、収束まで見るなら `--time` は数百秒。
実機の記録での確かめと実機で試す手順は [`docs/development.md` §4.3](../docs/development.md#43-走行中の学習を確かめる)。

```bash
B="--course toyota2 --map mpc_toyota2 --locate-hint 0,0 --mode slam2d_route --set v_max=4.0"
.venv/bin/python -m sim.bench $B --time 150 --mu 0.55 --set grip_learn=1          # 一様に 0.55
.venv/bin/python -m sim.bench $B --time 170 --mu 0.55 --surface 4.38,5.68,1.0,0.30 --set grip_learn=1
.venv/bin/python -m sim.bench $B --time 190 --mu 0.55 --mu-at 90:0.44 --set grip_learn=1   # 90 秒後に下がる
.venv/bin/python -m sim.bench $B --time 80                                        # 学習なし（同定値で固定）
```

| オプション | 中身 |
|---|---|
| `--mu MU[:MU_LONG]` | 真値の摩擦係数（一様）。横の限界と、前後（加速・速度指令での減速）の上限を同じ割合で変える。`MU_LONG` で前後だけ別の値。コースの `surface` を置き換える |
| `--surface X,Y,R,MU[,MU_LONG]` | シムの世界座標の円の中だけ変える（繰り返し可） |
| `--mu-at T:MU[:MU_LONG]` | bench の時刻 `T` 秒から一様の値を変える |
| `--grip FILE` / `--save-grip FILE` | 学習した値を読んでから走る / 走り終えた値を書く |
| `--state DIR` | `--map` の地図の学習値と仕上げた経路の写しを DIR から読み、走り終えたら書く（同じ地図の 2 回目を再現） |
| `--grip-set KEY=VALUE` | 横の学習の定数（`grip.GripConfig`）を上書き |

結果の「横G学習」の行に、終わったときの値・真値・天井・収束した時刻・段の判断と採用の数・安全装置の回数と**誤検出**
（真値で限界を超えていなかった）、そのあとの横偏差と壁との余裕が出る。その下に加速・減速の値と真値・包絡。
`--trace` の CSV には `slip`（真値の超過率）・`g_level` などが入り、`<trace>.long.csv`・`<trace>.path.csv` も書く。

シムの路面（`sim/surface.py`）と TC の旗は**作り物**: 車は限界で曲率がいきなり頭打ちになり（実機のタイヤは徐々に逃げる）、
後輪が巻き込む動き・後輪のロックは無い。

### SLAM だけの評価（`sim.slam_bench`）

```bash
.venv/bin/python -m sim.slam_bench                                  # 全コース×real
.venv/bin/python -m sim.slam_bench --course toyota --errors harsh --plot
.venv/bin/python -m sim.slam_bench --course normal --seeds 3 --json out.json
```

`sim.bench` は SLAM が制御ループの中にいるので原因の切り分けができず、実時間なので遅い。こちらは車を
**真値の中心線に沿って真値の姿勢で**純追従させ（SLAM を制御から切り離す）、仮想時間で回す。LiDAR は実機と同じ
`VirtualLidar → ScanAssembler` 経路を通り、車速・ヨーレートには `--errors ideal|real|harsh` で誤差が入る
（値の根拠は `sim/slam_bench.py` の `SensorModel` docstring）。

測るのは2段:

1. **地図作成**（EXPLORE 相当、低速2周）: 推定軌跡のずれ・見失い・1周期の処理時間・ループ閉じの前後で地図がどれだけ真の壁に乗っているか
2. **凍結地図での追従**（RACE 相当、高速2周）: **レース中の自己位置精度はここで決まる**。横方向の誤差（`rc_lat`）が走りに効く量

地図全体の剛体回転・平行移動は走りに無関係なので、RACE 段の誤差は**地図を真のコースへ剛体で重ねてから**測る（`align_map`）。
ほかに `--route <json>:<グループ>`（真値の運転手を好きな経路で走らせる）・`--locate N`（保存地図の読み直し＋全域探索）・
`--speed-quant`・`--time-offset-ms`・`--car`（他車）など（`--help`）。

### 駐車の評価ベンチ（`sim.park_bench`）

```bash
.venv/bin/python -m sim.park_bench --scenario all --n 12 --errors real
.venv/bin/python -m sim.park_bench --scenario garage --n 4 --plot --verbose
```

`sim.bench`（コース周回の評価）とは評価軸が違うので別に持っている。
**成否を「plannerの申告」ではなく真値で決める**のがこのベンチの要点:

- 毎周期 `VirtualLidar` → `ScanAssembler` を通す（鏡像戻し・セクタ欠損・
  ノイズ・遅延が入る）。**`Scan`を作り置きして使い回してはいけない**
  ——障害物が車体ローカルに固定され、車と一緒に動いてしまう
- `Course.collides()` で**衝突を毎ステップ数える**。planner が「完了」と
  言っても擦っていれば失敗
- `--errors real|harsh` でジャイロバイアス・オドメトリのスケール誤差・
  エンコーダ量子化を注入する。`VehicleModel`は誤差ゼロの真値を出すので、
  **これが無いとデッドレコニングのドリフトを一度も測っていないことになる**
- `推定誤差`列は planner の自己位置推定 vs 真値。成否は許容誤差の境界で
  ばたつくので、**推定の良し悪しはこの列で見る**


### 実機の記録でシムの車を確かめる（`sim.log_check`）

```bash
.venv/bin/python -m sim.log_check ~/Downloads/surge_20261102_093957.mcap
```

記録の `/cmd` をシムの車へ開ループで入れ、舵角・車速・ヨー比（÷運動学）・摩擦円・前後加速度の差を実機と並べる。
**値は求めない・書かない**（モデルを直したときに「実機に近づいたか」を判定する物差し）。既知の差: 実機は横加速度の
限界まで徐々に飽和しながら曲がるのに、シムは `[limits] a_lat_m_s2` でいきなり頭打ちになる。

## コードの地図

```
sim/
├── link.py       SimLink — SerialLink と同じ顔をした偽リンク（**7つの約束だけ**）
├── stm32.py      仮想 STM32 ファーム（パケット解釈・生成・状態機械・送信キュー）
├── vehicle.py    自転車モデル + 操舵むだ時間/遅れ + グリップ限界 + 速度PI（`SpeedController`）
├── lidar.py      仮想 LD06（**センサ座標系＝鏡像で出す**）
├── course.py     コース読み込み → 占有格子・numpy ベクトル化レイキャスト・外形衝突判定
├── track.py      センターライン方式（直線＋円弧 → 道幅から壁を自動生成）
├── params.py     シム固有のパラメータ（実機に対応物が無い量だけ）
├── channel.py    UDP 私設チャンネル（**バスは使わない**）
├── ui.py         画面部品。**地図の描画を gui と editor で共有する**（2箇所に置くとずれる）
├── gui.py        pygame の俯瞰ビュー + 走行数値 + 設定ページ（P で開閉）
├── editor.py     コースエディタ（区間をクリックでつなぐ／画像から作る）
├── image_course.py  画像 → 占有格子（エディタの画像モードの中身）
├── run.py        4プロセスをまとめて起動するランチャ（**stdlib だけで書く**）
├── raceline.py   理想レーシングライン（最小曲率QP）と速度プロファイル（RL 報酬・評価用）
├── sketch.py / wall_track.py  エディタの頂点→中心線 / 壁モードの中心線導出
├── bench.py      planner の閉ループ評価（バス・GUI・プロセス無し）
├── surface.py    路面の摩擦を場所と時刻で変える（bench 専用。上の「路面を変えて走らせる」）
├── slam_bench.py slam2d の通し評価（誤差注入・真値比較）
├── park_world.py  駐車配置（縦列/車庫入れ/斜め/袋小路）を Course として生成
├── park_bench.py  駐車の評価ベンチ（真値の衝突判定・センサ誤差の注入）
├── routes/       経路の設定の例（`sim.bench --routes`。**シムの世界座標**で書く）
└── courses/      *.png（＋サイドカー json）と *.json（センターライン方式）
```

## 気をつけること

- **`SimLink.poll()` の `sleep` を消さないこと。** 本物の `SerialLink.poll()` は
  `read_timeout=0.005` でブロックし、io_node のループはそこで待つ前提になっている。
  即座に返すと CPU を 100% 使う暴走ループになる。さらに「毎回まるごと 5ms 寝る」のも
  駄目で、受信時刻に下駄が乗って片道遅延の推定が数倍になる（実際に 5.8ms → 1.3ms）
- **`bus_demo` / `replay_node --bus` / `io_node --sim` は同じ `io` endpoint を取り合う。**
  同時に起動できない
- レイキャストは **numpy でベクトル化**すること。360本を Python の for で回すと
  120Hz に間に合わず、LiDAR がスカスカになる
- 真値は UDP にしか流れないので **mcap ログには残らない**。将来「真値つきデータセット」が
  要るならシム側で独立したファイルに書く（バスに戻すのではない）
- **`sim.gui` の終了コードには意味がある。** `sim.run` が 0 か否かで「窓を閉じた」と
  「落ちた」を区別しているので、正常終了の経路で例外を投げて非0で死なせないこと
- シムを2つ並走させたいときは `SURGE_BUS_DIR` に加えて
  `SURGE_SIM_TRUTH_PORT` / `SURGE_SIM_CTRL_PORT` も分ける（UDP の私設チャンネル）
