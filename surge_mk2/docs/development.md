# SURGE Mark.2 開発ガイド — 変更の反映とトラブルシューティング

**対象**: このリポジトリを触る人（＝3ヶ月後の自分を含む）

「コードを直した。どこに何をすれば実機に反映されるのか」と
「動かない。何から疑えばいいのか」だけを書いた実務文書。
**設計の理由は [`architecture.md`](architecture.md)、開発の経緯は `PROGRESS.md` が正。**

---

## 目次

1. [開発の3つの舞台](#1-開発の3つの舞台)
2. [変更 → 反映の対応表（最重要）](#2-変更--反映の対応表最重要)
3. [コマンド早見表](#3-コマンド早見表)
4. [Mac だけで回す開発ループ](#4-mac-だけで回す開発ループ)
5. [実機へ反映する](#5-実機へ反映する)
6. [テスト](#6-テスト)
7. [触る前の安全ルール](#7-触る前の安全ルール)
8. [トラブルシューティング](#8-トラブルシューティング)
9. [ログの取り方・見方](#9-ログの取り方見方)
10. [新しく何かを足すときの定石](#10-新しく何かを足すときの定石)
11. [既知の穴と文書の読み方](#11-既知の穴と文書の読み方)
12. [学習ベースの自動運転を動かす（LiDAR E2E / カメラセグメンテーション / カメラE2E）](#12-学習ベースの自動運転を動かすlidar-e2e--カメラセグメンテーション--カメラe2e)

---

## 1. 開発の3つの舞台

**実車を出す前に、Mac 上で潰せるものは全部潰す。** 舞台は3つある。

| 舞台 | 何が本物か | 何を開発できるか | 起動 |
|---|---|---|---|
| **シミュレータ** | STM32 より上流のコード全部（プロトコル・換算・安全ゲート・バス・WS・GUI） | **自律走行・点群処理・操縦感**。指令に車が反応する | `python -m sim.run` |
| **モック** (`bus_demo`) | バス・WS・GUI の配管だけ（UART 層を通らない） | **GUI の見た目・異常表示**。`--faults` で E-Stop や過熱を演出できる | `python -m raspi.tools.bus_demo` |
| **ログ再生** (`replay_node`) | センサデータが**本物**（実車で録った `.sfl`） | 実データでの知覚・解析。ただし**指令には反応しない** | `python -m raspi.nodes.replay_node <file> --bus` |
| （実車） | 全部 | 最終確認・遅延やノイズの実測 | Pi 上の systemd |

```
                   速い・安全 ←───────────────────────→ 本物
   モック(bus_demo)      シミュレータ(sim.run)      ログ再生      実車
   配管と見た目          自律走行の開発台           実センサ      最終確認
```

### 選び方

- **GUI の色・配置・異常表示を直した** → モック（`--faults`）
- **planner（`raspi/auto/`）を書いた・パラメータを詰めたい** → シミュレータ、数値で比べるなら `sim.bench`
- **SLAM（`slam2d/`）を触った** → `sim.slam_bench`（真値比較・仮想時間・誤差注入。§4.4）
- **点群の解釈や換算を疑っている** → ログ再生（本物のノイズが入っている）
- **遅延・電流・温度・電波を知りたい** → 実車以外に手段は無い

---

## 2. 変更 → 反映の対応表（最重要）

**Pi 上では systemd が複数のサービスを持っている**（内訳は
[`system_overview.md` §6](system_overview.md#6-raspberry-pi-の中身--プロセスとトピック)）。
何を直したかで、必要な操作が変わる。

| 直したもの | 必要な操作 | E-Stop がラッチするか |
|---|---|---|
| `gui/` | **rsync だけ**（`tools/deploy.sh`）。telemetry_node は毎リクエストでファイルを読む | しない |
| `raspi/tools/mdns_unicast.py`（Wi-Fi 端末から `surge-mk2.local` を引けるようにする avahi の補助） | rsync ＋ `ssh surge-mk2 'sudo systemctl restart surge-mdns'`。`deploy.sh --restart` は再起動しない | しない |
| `raspi/nodes/telemetry_node.py` | `deploy.sh --restart`（surge-telemetry / surge-camera） | しない |
| `raspi/nodes/camera_node.py` | 同上 | しない |
| `raspi/nodes/cam_perception_node.py` | `deploy.sh --restart`（surge-telemetry / surge-camera / surge-cam-perception） | しない |
| `raspi/nodes/cam_e2e_node.py` | `deploy.sh --restart`（surge-telemetry / surge-camera / surge-cam-e2e）。`surge-cam-e2e` は常時起動だが `cam_e2e` が選ばれている間だけ推論する（IDLE/ACTIVE、`cam_perception_node.py` と同じ設計。§12.3） | しない |
| `raspi/nodes/line_perception_node.py` | `deploy.sh --restart`（surge-telemetry / surge-camera / surge-line-perception）。`surge-line-perception` は常時起動だが `line_trace` が選ばれている間だけ認識する（IDLE/ACTIVE、`cam_perception_node.py` と同じ） | しない |
| **`raspi/auto/` `raspi/nav/` `planning_node.py`** | `deploy.sh --restart`（surge-planning を含む。`PLANNERS` はプロセス起動時に固定されるので必須） | しない |
| `config/vehicle.toml` | 読んでいるノードを再起動（planning / io）。**先に `python3 config/generate.py`**（GUI 用 TS を再生成）してから rsync | io なら**する** |
| `config/auto.json` `auto_presets.json` `camera.json` `cam_model.json` `e2e_lidar_model.json` `odometer.json`・`saved_maps/` | **機械ごとの状態。`deploy.sh` は運ばない**（telemetry_node / io_node が動いている機械で書き、起動時に戻す。Mac＝シム、Pi＝実機で別々）。地図は GUI の DL/UL で運ぶ（経路の設定も同梱） | しない |
| `raspi/nodes/io_node.py` `raspi/io/` `raspi/msgs/` `raspi/proto/` | **`deploy.sh --restart-io`** | **★★ する** |
| `raspi/setup/install_services.sh` の引数（`--max-speed` 等） | **`--services` ＋ `--restart-io`** | **★★ する** |
| `raspi/proto/protocol.toml` | **先に `python3 raspi/proto/generate.py`**、その後 `--restart-io`。**STM32 側にもヘッダを渡す** | **★★ する** |
| `raspi/msgs/types.py` | **`gui/src/types.ts` も手で直す**（写しなのでズレる）→ rsync ＋ `--restart-io` | **★★ する** |
| `models/*.onnx`（カメラセグメンテーション用）`models/cam_e2e/*.onnx`（カメラE2E用）`models/e2e_lidar/*.onnx`（LiDAR E2E用） | **通常の `tools/deploy.sh`（rsync のみ）。** プロセス再起動は不要——`cam_perception_node`/`cam_e2e_node`/`e2e_lidar` の `reload_if_changed()` が GUI でモデル名を選んだ瞬間に読み直す | しない |
| `ml_cam/` `ml_cam_e2e/` `ml_lidar/`（学習パイプライン一式）`ml_common/`（3つの学習GUIが共有するTkinter部品） | **Mac 側だけで完結。Pi には無関係**（rsync では運ばれるが、`raspi/` 側は読まない） | しない |

> **`--services` は unit ファイルを書き換えるだけ。** 走っている `io_node` は古い引数のまま
> 動き続けるので、反映には `--restart-io` が要る。

### ★★ `--restart-io` は E-Stop をラッチさせる

`io_node` は GPIO6 に 100Hz のハートビートを出している（安全第1層）。
**止めた瞬間に STM32 が E-Stop をラッチし、車両のボタン2を物理的に押すまで戻らない。**

- **仕様どおりの動作であり、故障ではない**
- **車体に手が届く状態でだけ使う**（リモートで叩くと、現地に行くまで車が死ぬ）
- `Restart=on-failure` で io_node が再起動しても**ラッチは解除されない**。人間の操作でしか戻らない

---

## 3. コマンド早見表

### Mac 側

メイン GUI（Web）以外の補助 GUI（シミュレータ一式・コースエディタ・システム同定・SLAM 再処理・動画の書き出し・
ml_lidar / ml_cam / ml_cam_e2e）は、**`launcher.command`（surge_mk2 直下）をダブルクリック**すると
1つの窓から起動・停止できる（`tools/launcher.py`。アプリの追加は `APPS` に1行足す）。
ランチャーを閉じると、そこから起動したアプリも終了する。

```bash
cd ~/GitHub/AutonomousCar_RasPi/surge_mk2

# ── 実行 ──
.venv/bin/python -m sim.run                       # シミュレータ一式（Ctrl-C で全部落ちる）
.venv/bin/python -m sim.run --course toyota2       # コース指定
.venv/bin/python -m sim.run --list                # コース一覧
.venv/bin/python -m sim.run --kill-stale          # 古いプロセスを落としてから起動
.venv/bin/python -m sim.editor [名前]             # コースエディタ
.venv/bin/python -m sim.bench --course normal      # planner の数値評価

# ── テスト・生成 ──
./tools/check.sh                                  # 生成物一致・文書版番号・pytest・tsc を1コマンドで
./tools/check.sh --fast                           # 生成物の --check だけ（数秒。コミット前向き）
.venv/bin/python -m pytest raspi/tests -q         # テストだけ単体で回すとき
python3 raspi/proto/generate.py                   # protocol.toml を直したら必ず

# ── GUI ──
cd gui && npm run dev                             # http://localhost:5173（自動リロード）
SURGE_HOST=surge-mk2.local npm run dev            # 編集しながら実機に繋ぐ
npm run build                                     # -> gui/dist（Pi に node は不要）

# ── Pi へ ──
tools/deploy.sh                                   # GUI ビルド + rsync（既定・いちばん安全）
tools/deploy.sh --no-gui                          # Python だけ直したとき
tools/deploy.sh --test                            # 反映後に Pi 上でテスト
tools/deploy.sh --restart                         # telemetry / camera / cam-perception / cam-e2e / line-perception / planning を再起動
tools/deploy.sh --services --restart-io           # unit を入れ直して io を再起動（★E-Stop）
tools/record.sh --duration 60                     # SD に書かずに MCAP を PC へ

# ── ログ ──
.venv/bin/python -m raspi.tools.logcat logs/x.sfl          # .sfl の要約
.venv/bin/python -m raspi.tools.sfl2mcap logs/x.sfl        # .sfl → .mcap
.venv/bin/python -m raspi.tools.slam_replay x.mcap          # slam2d に通し直して自己位置を足す（x_slam.mcap）
.venv/bin/python -m raspi.tools.grip_replay x.mcap          # 走行中の学習（nav/grip.py）に流し直して前兆の判定を見る
.venv/bin/python -m raspi.tools.mcap_repair x.mcap --inplace   # 尻切れ .mcap の復旧
.venv/bin/python -m raspi.nodes.replay_node logs/x.sfl --bus --loop
```

### Pi 側（ssh surge-mk2）

```bash
systemctl is-active surge-io surge-camera surge-telemetry surge-planning
sudo systemctl restart surge-planning        # 自動運転だけ入れ直す（安全）
sudo systemctl stop surge-io                 # ★ E-Stop がラッチする
tail -F ~/surge_mk2/logs/surge-io.out        # 標準出力ログ
journalctl -u surge-io -f                    # systemd 側のログ
sudo journalctl --list-boots                 # 勝手に再起動した形跡を追う
sudo bash ~/surge_mk2/raspi/setup/install_services.sh --safe   # arm 封印で入れ直す
```

`install_services.sh` は unit を書き換えるだけで、**動いている surge-io は古い引数のまま**（`--safe` を入れても
封印はまだ効いていない）。スクリプトが最後にそう警告する。反映は `tools/deploy.sh --restart-io`（★E-Stop）。
surge-io は drop-in `surge-io.service.d/10-priority.conf` で `Nice=-10` にしてある。

**接続先は常に `surge-mk2.local`**（mDNS）。直結・Wi-Fi の生きている方を自動で選ぶので、
IP を調べる作業は要らない。GUI は `http://surge-mk2.local:8000/`。

---

## 4. Mac だけで回す開発ループ

### 4.1 シミュレータ

```bash
.venv/bin/pip install -r sim/requirements.txt     # 初回だけ（pygame / pillow）
.venv/bin/python -m sim.run
```

`io_node --sim` / `telemetry_node --no-camera` / `planning_node` / `sim.gui` の4つが上がり、
ブラウザが開く。**`Ctrl-C` か俯瞰ビューの窓を閉じると全部止まる。**

自動運転を試すときは、ブラウザの「自動運転」タブでモードを選び、
`Enter` で ARM してから「自律走行を開始」を押す。
**engage しなくても判断だけは流れている**ので、手動で走らせながら planner の狙いを見られる。

`sim.run` は起動前に「Rosetta で動いていないか」「ポート 8000 と `io` エンドポイントを
掴んでいる古いプロセスが居ないか」を**起動時刻つきで**点検する（下記 8.3 の2つを
そもそも踏ませない作り）。子プロセスの扱いは3種類:

| 子 | 死んだら |
|---|---|
| `io_node` / `telemetry_node` | **全部止める** |
| `sim.gui` | 終了コード 0（＝窓を閉じた）なら全部止める。**非0（事故）なら続行** |
| `planning_node` | **どう死んでも巻き添えにしない**（別ターミナルで上げ直せばよい） |

> コースは `sim/courses/*.json`（`python -m sim.run --list` で一覧。`sim.run` の既定は `normal`）。
> `io_node` 単体の `--course` 既定（`sim/courses/oval.png`）は現存しないので、手で起動するときは明示する。
> 増やすなら `python -m sim.editor`（下記）。

#### シミュレータ GUI（pygame）のキー操作

| キー | 動作 |
|---|---|
| `P` | シムの設定ページ（コース一覧・LiDAR・ODOM・LINK）を開閉 |
| `M` | **コースエディタを別ウィンドウで開く**（今のコースを開いた状態） |
| `N` / コース名クリック | コース切替（車両はそのコースのスタート姿勢へ） |
| `↑` `↓` | 行の選択（閉じているときは設定ページを開く） |
| `←` `→` | 選択中のパラメータを増減（`Shift` で 10 倍） |
| `R` / `E` / `T` / `D` | 姿勢リセット / E-STOP / 軌跡消去 / パラメータ既定値 |
| `Esc` / `Q` | 終了 |

**同じ操作は全部パネル下部のボタンにも出ている**（キーを併記してあるので使ううちに覚える）。

#### コースを作る（`python -m sim.editor`）

直線（0.5/1/2/3m）と円弧（R0.5〜2m × 30/45/60/90/180°）をクリックでつないでいくだけ。
**閉ループを作るための数字が3つ出る**:

| 表示 | 読み方 |
|---|---|
| **総回頭** | **360 の倍数でなければ絶対に閉じない。** 「あと 90」と出る |
| **始点まで 前 / 左** | 進行方向基準の差。「あと 1.2m 前」「0.3m 左」と読める |
| **直線で閉じる** | 回頭差と横ずれが無ければ、必要な長さの直線1本を足して閉じる（`Enter`） |

`←` 左カーブ / `→` 右カーブ / `↑` 直線 / `Backspace` 取消 / `C` 全消去（**2度押し**）/
`S` 保存 / `O` 既存を開く / `W`・`Shift+W` 道幅。
保存先は `sim/courses/<名前>.json` で、**手書き JSON と完全に同じ形式**。

### 4.2 GUI だけを直す

サーバ（`telemetry_node`）とデータ源（`bus_demo` か `sim`）を上げたまま、
`npm run dev` を使う。Vite の自動リロードが効くので rsync も再起動も要らない。

### 4.3 planner を数値で比べる

```bash
.venv/bin/python -m sim.bench --course normal
.venv/bin/python -m sim.bench --mode de --time 180 --set safety_half_width=0.12
```

**プロセスもバスも GUI も使わず、1プロセスの中で `io_node` と同じことをやる。**
`ScanAssembler` の鏡像反転も `command_from_cmd` のクランプも通るので、
「ここで走れば `sim.run` でも走る」。

出るのは**計画周期（平均/中央/最大 ms）・自己位置誤差・向きの誤差・横偏差・
周回数・走行距離・衝突回数・ラップタイム・地図の正確さ**。

見た目では「なんとなく速い」しか分からない。**数字で出してから採否を決める。**

> **ベンチだけが真値を読める。** シムの真値は UDP の私設チャンネルにしか出ないので、
> SLAM の推定と真値を毎周期突き合わせられるのはここだけ（実機では絶対に手に入らない数字）。

#### 路面を変えて走らせる（走行中の学習の検証）

`slam2d_route` の `grip_learn`（`raspi/nav/grip.py`。横・加速・減速の上限を走行中に決める）は、真値の
路面を変えた車で確かめる。横の決め方は既定で「周回のタイム」（`grip.TimeLearner`）。1 段に 2 周かかるので
`--time` は 400 秒以上にする。元の方式（前兆が出るまで上げる）と比べるときは `--set grip_by_time=0`。路面を変えるオプションは**真値の車だけ**に効く（planner は `config/vehicle.toml` のまま）。

```bash
B="--course toyota2 --map mpc_toyota2 --locate-hint 0,0 --mode slam2d_route --set v_max=4.0"
.venv/bin/python -m sim.bench $B --time 150 --mu 0.55 --set grip_learn=1          # 一様に 0.55
.venv/bin/python -m sim.bench $B --time 170 --mu 0.55 --surface 4.38,5.68,1.0,0.30 --set grip_learn=1
.venv/bin/python -m sim.bench $B --time 190 --mu 0.55 --mu-at 90:0.44 --set grip_learn=1   # 90 秒後に下がる
.venv/bin/python -m sim.bench $B --time 80                                        # 学習なし（同定値で固定）
```

| オプション | 中身 |
|---|---|
| `--mu MU[:MU_LONG]` | 真値の摩擦係数（一様）。横の限界と、**前後（加速・速度指令での減速）の上限**を同じ割合で変える。`MU_LONG` を付けると前後だけ別の値 |
| `--surface X,Y,R,MU[,MU_LONG]` | シムの世界座標の円の中だけ変える（繰り返し可） |
| `--mu-at T:MU[:MU_LONG]` | bench の時刻 `T` 秒から一様の値を変える |
| `--assume KEY=VALUE` | **planner が信じている車**の諸元だけ変える（`--vehicle` の逆） |
| `--grip FILE` / `--save-grip FILE` | 学習した値を読んでから走る / 走り終えた値を書く（保存値からの再開） |
| `--launch-ratio R` | 横の限界÷発進の加速度を知っている状態から始める |
| `--grip-per-lap` | 横を周の境目でだけ上げる（収束の速さを比べる基準版） |
| `--grip-set KEY=VALUE` | 横の学習の定数（`grip.GripConfig`）を上書き |
| `--sim KEY=VALUE` | センサ誤差（`sim.params.SimParams`）を上書き |
| `--real-noise` | **実機で測った走行中の揺れ**（ヨーレート・車速・実舵角。約 11Hz）をテレメトリに足す（`sim.params.REAL_VIBRATION`）。★学習のしきい値・上げ幅を変えたら、これ有り・無しの両方で走らせる |
| `--set KEY=VALUE` | planner の設定。**GUI から隠した項目（`ParamSpec.hidden`）も bench からは上書きできる** |

結果の「横G学習」の行に、終わったときの値と真値・収束した時刻・前兆の回数と**誤検出**・前兆のあと
1.5 秒の横偏差の最大と壁との余裕の最小が出る。その下に加速・減速の値と真値、値を下げた測定の一覧、
窓ごとの判定の集計（「限界」と測った値の真値との誤差・**取り違え**の数）、評価しなかった窓の理由が出る。
`--trace` の CSV には `slip`（真値の超過率）・`g_level`・`g_excess`・`g_cap`・`g_gate` が入り、あわせて
`<trace>.long.csv`（加速・減速の学習へ渡した値と、その瞬間の真値）・`<trace>.path.csv`（終わったときに
追っていた経路と速度）を書く。

★ベンチは**実時間**で回る。並列は 2 本まで。1 行目の「計画 … 最大」が 90ms を超えた走行と、
「指令の間隔 最大」が 150ms を超えた走行（★が付く）は捨てて走らせ直す。後者はホストが詰まって仮想 STM32 の
デッドマン（`cmd_deadman_ms`）が車を止めた走行で、直線の途中で止まり、走り直す所で壁に当たることがある
（2026-10-12 に「衝突」として 3 回見えたのはこれだった。走りの問題ではない）。

シムの路面（`sim/surface.py`）と TC の旗は**作り物**: 車は限界で曲率がいきなり頭打ちになり
（実機のタイヤは徐々に逃げる）、後輪が巻き込む動き・後輪のロックは無い。TC の旗は駆動の加速度が
上限で頭打ちの間に立てているだけ（`sim/stm32.py`）。

★**シムの既定の雑音は実機よりずっと小さい**（ジャイロ σ 0.3°/s。実機の走行中は高周波だけで 4.5〜8°/s、
舵も σ 0.016rad でばたつく）。既定のシムだけで決めたしきい値を実機に入れたら、滑っていない車で前兆が
46 回出て、横の上限が下限まで落ちた（2026-10-12、1 周 5.7s → 13.1s）。`--real-noise` を付けた走行と、
下の「実機の記録を流し直す」の両方で確かめること。

#### 実機の記録を学習に流し直す（`raspi/tools/grip_replay.py`）

横の前兆の判定は 100Hz の車両状態（車速・ヨーレート・実舵角）だけで決まるので、記録をそのまま
流せば実機で何が前兆と判定されたかが手元で再現できる。**滑っていない記録では前兆 0 が合格。**

```bash
.venv/bin/python -m raspi.tools.grip_replay ~/Downloads/surge_xxx.mcap            # 学習を入れて走った記録
.venv/bin/python -m raspi.tools.grip_replay ~/Downloads/surge_xxx.mcap --pin 4.9  # 学習なしの記録: 上限を固定して流す
.venv/bin/python -m raspi.tools.grip_replay x.mcap --set noise_k=8                # GripConfig を変えて比べる
.venv/bin/python -m raspi.tools.grip_replay x.mcap --end 100 --export x.npz       # テスト用の小さな記録に抜く
```

- 流し直しは開ループ（前兆で目標速度を落としても、記録の車速は変わらない）。**学習なしで走った記録を
  `--pin` なしで流すと、上限だけ上がって車が付いて来ないので「頭打ち」が出る**——`--pin` を使う
- 回帰テスト（`raspi/tests/test_grip.py` の `TestRealNoise`）は `raspi/tests/data/grip_real_*.npz`
  （2026-10-11・12 の実機の記録）を流す。実機で誤検出した記録が増えたら、ここへ足す
- 学習を入れて走った記録には `auto/state.grip`（学習値・不足としきい値・測った雑音の幅・試行の
  平均の不足・横偏差の増え・加速と減速の学習へ渡した計画の速度と加速度）が 10Hz で入る。
  タイムで決める方式ではさらに `t_lap`・`t_plan`（直近に数えたラップと計画のラップ [s]）・`ref_lat`・
  `ref_lap`・`ref_plan`（比べる相手の段）・`step`（今の幅）・`laps_n`（今の段で数えた周）
- ★`grip_replay` は元の方式（`GripLearner`）の判定を流し直す道具。タイムで決める方式の判断は開ループでは
  再現できない（上限を変えたときのラップは、実際に走らないと出ない）

#### 走行中の学習を実機で初めて試すとき

シムで決めたしきい値・上げ幅をそのまま使わない。安全側から順に:

1. **記録だけ取る**（`grip_learn` = 0 のまま普段どおり周回）。記録を `grip_replay --pin <走っていた
   横加速度>` に流し、前兆が 0 であることを見る（上の節）。しきい値は測った雑音の幅から自動で決まる
   （`GripConfig.noise_k`）が、車・路面・センサが変わって雑音の形が変われば誤検出しうる
2. **下げるだけ**: `grip_learn` 1・`grip_down_only` 1。出発点は同定値の 0.85 倍（横）・0.95 倍（加速）・
   0.75 倍（減速）で、そこから上げない。前兆が誤って出ないか
   （reason の「前兆 n回」が何もない所で増えないか、減速の値が下がり続けないか）を見る。
   減速は「制動区間の終わりで車速が計画より速かったぶん」で決める（記録の `grip.lag_dec`。0.10m/s を超えた周に
   1 段下がる）。加速は学習せず、上限 3.0m/s² で固定
3. **上げる**: `grip_down_only` 0・`grip_max` を今走れている値＋0.5m/s² 程度（例 5.0）・`v_max` は
   普段より下げる。壁の外側に緩衝材を置く。★横は**限界の見積もりそのものまで出す**。出してよい
   上限は `grip_max` で決める。上げ幅（既定 6%）は隠した設定 `grip_step` で、実機で小さくしたいときは
   `raspi/auto/slam2d_raceline.py` の既定値を変える
   ★減速も同時に学習される（切り離す設定は無い）。1 周に 1 段（8%）上げるので、同定値の 0.75 倍から
   上限（`cmd_accel_limit_m_s2`）まで 6 周ほど。限界を 1 段超えた周は、コーナーへ計画より速く入る
4. 値が落ち着いたら `grip_max` を上げる。本番の計測走行は、練習で収束した値を保存したうえで
   `grip_down_only` 1（保存した値の 0.92 倍から始まり、上げない）

### 4.4 SLAM だけを数値で比べる（`sim.slam_bench`）

```bash
.venv/bin/python -m sim.slam_bench                                  # 全コース×real
.venv/bin/python -m sim.slam_bench --course toyota --errors harsh --plot
.venv/bin/python -m sim.slam_bench --course normal --seeds 3 --json out.json
```

`sim.bench` は planner 全体を実時間で回すので、**SLAM の精度だけを見たいときには
向かない**（SLAM が制御ループの中にいるので原因の切り分けができない・コース×条件を
振ると何十分もかかる・`VirtualStm32` は `speed` を真値のまま送る）。

こちらは車を**真値の中心線に沿って真値の姿勢で**純追従させ（SLAM を制御から切り離す）、
仮想時間で回す。LiDAR は実機と同じ `VirtualLidar → ScanAssembler` 経路を通り、
車速・ヨーレートには `--errors ideal|real|harsh` で誤差が入る（値の根拠は
`sim/slam_bench.py` の `SensorModel` docstring。MPU6050 と LD06 のデータシート）。

測るのは2段:

1. **地図作成**（EXPLORE 相当、低速2周）: 推定軌跡のずれ・見失い・1周期の処理時間・
   ループ閉じ（`freeze()`）の前後で地図がどれだけ真の壁に乗っているか
2. **凍結地図での追従**（RACE 相当、高速2周）: **レース中の自己位置精度はここで決まる**。
   横方向の誤差（`rc_lat`）が走りに効く量

> 地図全体の剛体回転・平行移動は planner の走りに無関係（地図の中でしか動かない）なので、
> RACE 段の誤差は**地図を真のコースへ剛体で重ねてから**測る（`align_map`）。
> これをやらないと「地図が1°回っている」が位置誤差として数十cmに化ける。

### 4.5 システム同定（車両パラメータを実機で測ってシムに入れる）

`config/vehicle.toml` の `[dynamics]` は実機で測る。試験は4つ（下表）。測る量の必要性は
PROGRESS.md「システム同定で測る量の必要性」、手法の理由は `tools/sysid/fit.py` のモジュール docstring。
旧手順で録った値は系統誤差を含むので測り直すこと。

**録る前に**: STM32 に TELEMETRY の `t_us` がスナップショット時刻になったファーム
（`fw_id` 0x4D463304 以降）を書き込んでおく。古いファームでは④の制御遅延が最大+20msずれる
（`docs/uart_protocol.md` §5.3）。

**実機での録り方**: GUI の「システム同定」タブで試験を選び、「試験開始」→ ARM を
保持 → 「完了」「中止」が出て車が止まったら自動で記録を止めてダウンロード（試験ごとに1つの mcap）。

| 順 | 試験（planner） | 測るもの | 必要なスペース（シムで計測、マージン別） |
|---|---|---|---|
| ① | ステア `sysid_steer` | `tau_steer_s`・`dead_time_s`（走行中1.0m/s・±7.5°・±15°のステップ、2往復）、`steer_rate_limit_rad_s`（低速・±30°のステップ）、`steer_servo_hysteresis_rad`（止まる位置の幅。階段の3.5°以下の小さな段差から直接読む）・`steer_servo_gain`（段を保持する階段。同じ舵角に上りと下りの両方から近づく）。階段と±30°のステップは**同じ点からの往復**（舵角を保って前進→オドメトリが元の値に戻るまで後退。舵は後退の終わりに切り替える）なので、その間は約0.6m×0.25mの中から出ない | 直線2m＋車長＋約0.3m × 幅約0.8m（走行部のジグザグの分）。約1分 |
| ② | 前後運動 `sysid_accel` | `speed_plant_gain`（トルク→加速度の利得）・`rolling_resistance`・駆動加速度の上限とその速度依存・`speed_decel_m_s2`（上限はファームの目標ランプ3.0m/s²より低いときだけ見える。見えなければ0）・`brake_decel_per_nm`（制動トルク→減速度の傾き）・`brake_decel_m_s2`（グリップによる制動の頭打ち）。**最初に全開加速（TC が効く）・最大制動（ABS が効く）でこの路面の最大の加速度・減速度をその場で読み、残りの3段をその下に割り振る**（加速度は最大の3・5・7割、制動は減速度が最大の3・5.5・8割になる強さ。頭打ちしなかった段の実測で傾きを直しながら置く）。段ごとにブレーキと速度指令0で1回ずつ止まり、最後にもう一度、全開・最大制動（計9回）。抑えきれない空転だけ、すぐ止める。ファームの PI の定数 `speed_kp` 等は同定しない | 直線 `run_length_m`（既定2.0m）＋車長 × 幅約0.25m。約45秒 |
| ③ | 旋回グリップ `sysid_corner` | `mu`（舵いっぱい＝`max_steer`、0.3m/sから0.06m/s²で連続して速くする。車体が滑ったこと——**横加速度が頭打ち**か**後輪が流れた**（曲率が余裕のある区間の1.15倍超）の早い方を限界とし（後輪の空転では打ち切らない）、舵を保ったまま1.0m/s²でゆっくり止まる） | 約2m 四方。約30〜45秒 |
| ④ | 遅延 `sysid_latency` | `control_latency_s`（スキャン完了→STM32受理） | その場（据え切りで小さく反転）。約15秒 |
| ①②③ | まとめて（`fit_geometry`） | 舵の効き（`steer_gain`・`steer_gain_cubic`。**結果は STM32 のリンクの換算 `steer_link_gain`・`steer_link_cubic` へ畳み込んで出る**——下の「ステアのリンクの換算」）・`steer_offset_rad`・`understeer_gradient`・`steer_link_hysteresis_rad`・`steer_link_deadband_rad`・`yaw_relaxation_m`（タイヤの緩和長）・`yaw_rate_filter_s`（IMU のローパス）・`yaw_natural_freq_rad_s`/`yaw_damping`（振動的なヨー応答＝曲率の2次遅れ。有意なときだけ） | — |

**ステアのリンクの換算**（プロトコル v0.18〜）: ステアのリンクは線形ではなく、モータ角×0.5 が
29.1° のとき前輪は約 26.5° しか切れていない。**STM32 が
`路面舵角 = steer_link_gain·x + steer_link_cubic·x³`（x = モータ角×0.5）で換算**し、舵角の指令・報告・
`max_steer` はすべて実際の路面舵角（STM32 は前輪速度を cos(舵角) で車体速度に直すので、換算が無いと
TC のスリップ・オドメトリ・曲率に誤差が乗る）。リンクが複雑で設計から式を出せないので、値は同定で決める:

- 解析は従来どおり「報告された舵角 → 曲率」の効きを当てはめ、それを**記録したときに STM32 に入っていた
  換算の上に重ねて**新しい `steer_link_*` を出す（`analyze.fold_steer_link`。記録時の換算は mcap の
  `/diag/link` の `control_params` から読む。無い記録は換算なし）。換算が合っていれば効きは 1・0 で、
  `steer_link_*` は変わらない——**録り直すたびに残りを詰める形**になる
- 「適用」は `[control]` の `steer_link_gain`・`steer_link_cubic`、`[dynamics]` の `steer_gain`=1・
  `steer_gain_cubic`=0、最上位の `max_steer`（換算を可動範囲 ±30° に通した値）を**そろえて**書く。
  `steer_link_*` は必ず2つとも適用する（片方だけはエラー）。換算の違う記録を混ぜて解析するとエラー
- 適用したら `./tools/deploy.sh --restart-io`。GUI の「制御パラメータ: 適用済み ✓」を確かめてから走らせる
  （STM32 に入っていないと、Pi の舵角と STM32 の舵角の意味が食い違う）
- 別の換算のもとで作ったものは舵角の読み替えが要る: `e2e_lidar` のモデルは契約
  JSON の `steer_link`（無ければ換算なし）から今の路面舵角へ直す（`control_params.steer_relink`）。
  **カメラE2E の教師データ（`ml_cam_e2e`）は未対応**——v0.18 より前に録った記録と後の記録を混ぜて学習しない
- `steer_servo_hysteresis_rad`・`steer_rate_limit_rad_s` など舵角の単位の値は、換算の前後で大舵角ほど
  数%〜1割変わる。換算を入れたら①を録り直す

**速度の使い分け**: リアルタイム（プランナーの判定・E2E の観測）は STM32 の `speed`、
**同定の解析は前輪オドメトリをゼロ位相で微分した速度**（前後2サンプルずつ計5点に直線を当てはめた
傾き、`fit.odom_speed`）。`speed` は 2kHz の値にローパス（1段・9.95ms）を掛けて 50Hz に間引いた
もので、遅れがあるうえ 25Hz より上のノイズが間引きで折り返して混ざる。`odom_dist` はローパス前の
角度を積算した位置（0.1mm 単位、STM32 内部は約0.03mm で端数も繰り越し）なので、録り終えてから
前後で微分すれば遅れも折り返しも無い。20ms で1カウントも進まないのは 5mm/s 未満のときだけで、
5点の当てはめで量子化の誤差は約0.5mm/s（端数が出る条件で確認）。旧ファームの `speed`（100ms）で
解析すると、ブレーキ減速度・ガタなどが誤る。
`vehicle.toml` の `speed_filter_s`（ファームの定数、同定しない）はシムの観測にだけ使う。
**ファームの `DRIVE_LPF_K_FRONT` を変えたら直す**——解析の所見「速度センサの自己点検」が、
記録の `speed` とオドメトリを照合して食い違いを警告する（静止中のノイズも出る）。
同じく `speed_kp`・`speed_ki`・`speed_torque_max_nm` もファームの定数
（`DRIVE_SPEED_KP/KI`・`DRIVE_MAX_TORQUE_NM`×2）なので、ファームを変えたら直す。目標車速のランプの上限は STM32 には無く、
`[safety] cmd_accel_limit_m_s2`（Pi が送る値）が実効の上限。

**前輪エンコーダの周期誤差**: アナログの絶対角エンコーダは1回転の中で読みがうねる。うねりは
曲率（ヨーレート÷速度）に乗ってガタ・不感帯を取り違えさせる（検証で±3.4°の誤差で起きた）ので、
記録を読むときに定速の区間から左右それぞれの誤差（1回転に1〜3回の高調波）を推定し、有意なら
オドメトリを補正する（`fit.correct_encoder_ripple`。所見に振幅が出る）。

**所見の★**: 各解析は残差を静止中のノイズ（速度・舵角・ヨーレート）と比べ、2倍を超えると
「モデルの形から外れている」と警告する。★が出た値はそのまま使わず原因を確かめること。
GUI は★の出た試験の項目を**既定で未チェック**にし、理由を横に出す（`FitResult.warnings`）。
速度応答と加減速を合わせて解析するとき、加減速に★が出たら
合わせて当てはめ直した車体の利得・転がり抵抗を採らない（外れた分を吸収して壊れる）。遅延試験は、`t_us` が送信時刻だった古いファーム（`fw_id` < 0x4D463304）の
記録なら★。
★の基準のノイズは、止まっている間と**走っている間**の大きい方（走っている間の方がずっと大きく、
実機で曲率は舵角換算0.5〜1.0°、速度は0.7〜2.7cm/s）。
**制動**: 減速度 = 傾き×強さ×tanh(車速÷境界層)＋転がり抵抗（後輪のロックで頭打ち）。境界層0.6m/s・
不感帯0.06m/s は**MD のファームの定数**（`BLDC/ProgramV4` の BRAKE_BOUNDARY/DEADBAND_SPEED_RAD_S ×
後輪半径。一定トルクの符号切り替えはバンバン発振するので境界層で0へ落とす）で `vehicle.toml` に書く。
測るのは傾きと頭打ちだけで、制動中の**走行距離（位置）**に当てはめる（微分しないので止まる直前も使える）。
実機で傾き約35/N·m、0.12N·m 以上で後輪がロック（約3.7m/s²）。★**最大制動（0.15N·m、
フェイルセーフ・E-Stop）は後輪をロックさせる**（制動中は TC が効かない。ABS は検討中）。同定の試験の
中止・周囲クリアランスでの停止は 0.08N·m で制動し舵を保つ（`ABORT_BRAKE_TORQUE_NM`）。
**速度の比べ方**: 実測の速度はオドメトリを±45msの窓で微分したものなので、加速度が急に変わる所
（全開加速→制動の切り替えなど）で窓の幅だけ鈍る。解析はシムの応答にも同じ微分を掛けて比べる
（`fit._simulate_speed`。掛けないと、上限の無い車に約2.3m/sの最高速を読んだ）。
要素を入れるか（ヒステリシス・上限など）の尤度比は、残差の自己相関から有効な自由度を見積もる
（`fit.n_eff`）。

**安全**: 直線・旋回の試験は前輪の速度を後輪モータの周速と照合し、後輪が回っているのに前輪が
大きく遅い状態が0.3s続いたら中止して制動する（`_sysid_common.SensorGuard`。前輪エンコーダが
止まると、加減速試験が2mの直線を超えて走り続ける）。戻りのタイムアウトも短い。
後輪・ステアのモータドライバが無応答（`MDS_COMM_OK` が落ちた）になったときも、その場で中止する（STM32 は黙った MD の車輪速度を古い値のまま送るので、放っておくと上の照合が逆の原因を表示する。解析もその記録を弾く）。
中止の理由は、前輪がほぼ止まっていれば「前輪エンコーダの不調？」、前輪も動いていれば（または
TC が介入中なら）「後輪の空転か前輪エンコーダの不調」と出す。★**空転**（加速がタイヤのグリップを
超える）: 後輪の荷重配分・縦の mu は未実測で、ファームの目標ランプ3.0m/s² が出せるかは実機で
確かめるまで分からない。②は最初に全開で最大の加速度を読み、残りの段をその下に置く（2026-10-13。
固定の段——0.8→1.6→2.4m/s²→全開、制動 0.04→0.08→0.12→最大——は、路面によって限界のどちら側に
落ちるか分からない。実機では制動4段のうち3段が ABS の頭打ちで、傾きを1段だけで決めていた）。
- **後輪が滑っても止めない**。TC はスリップ率を目標（`tc_slip_target`、今は 0.2）に
  保つので、TC が働いているだけで滑り率は 0.1 を超える（滑り率でその段までにすると、全開の段と強い
  ブレーキの段＝制動の頭打ちが測れない）。滑ったかどうかは解析が記録から読む（後輪の周速の平均と車速の差 ÷ 車速（1m/s未満は
  1m/sで割る）が0.1超。TC の旗は見ない）: 滑りながら出た加速度を段ごとにまとめ、**いちばん大きい段**の
  値を加速の上限にする（グリップで決まる、この車が実際に出せる加速度。限界より下の段は指令どおりの
  加速度しか出ないので、その値は限界ではない）。滑った区間は速度制御の当てはめから外す。
  滑った段より上を走っていない記録（古いプランナー・途中で止めた）で、加速度が指令とほぼ同じなら★
- **抑えきれない空転**（後輪が前輪より 0.2m/s＋30% 以上速い状態が 0.1s 続く）ならすぐ制動する。
  最初の全開で起きたら、空転するまでに出ていた加速度を最大とみなして下の段へ進む（全開の2回目と
  最後の全開は走らない）。割り振った段でも起きたら試験を終える。解析は★を出し、加速の上限を
  空転した段の加速度で上から抑える
- **前進が短い回は遅く戻る**（戻りに2.2s以上かける）。速度 PI が落ち着く前に次の加速が始まると、
  解析の「加速の頭で制御器は定常」の仮定が外れる（ベンチで、戻りが1.1〜1.3sの回だけ残差が約6倍になり、
  速度による加速度の低下を見落とした）

TC の細かい動き（トルクを削っては戻す）はシムに入っていない（シムは空転も TC も表現せず、
グリップの限界を「加速の上限」の一定値で近似する）。TC の定数（`DRIVE_TC_*`）はファームで決まって
いるので測らない。空転が出るなら、①の走行部・③の COMMAND の `accel_limit` をその段より下げるか
検討する。

- **全部を「直線2m」と「約2m四方」の2か所で録れる**
- どの試験も開始直後の1秒は止まる（ジャイロのバイアスをそこで読む）
- 直線の試験（①②）は、置いた向きのまま `run_length_m` の直線を往復する。①は向きが
  ずれないようジャイロで積算した自己位置で元の直線へ寄せ直すが、置く向きは合わせておく
- ①の階段・±30°のステップは前進・後退を**距離（オドメトリ）**で区切る（2026-10-13。時間で区切っていた頃は、
  実機で向きの反転に約0.45sかかって前進と後退の距離が合わず、階段部だけで約0.45mずれて終わった）。
  ベンチ（`tools.sysid.bench`）は各試験の所要時間と終了位置のずれも出す
- ②のブレーキの強さはプランナーが段ごとに指定する（`AutoState.brake_torque`→`COMMAND.brake_torque`）
  ので、GUI の制動トルクの設定は関係ない。自律走行中に人（GUI）がブレーキを掛ければ、強い方が効く
- ③の左右差を見たいときは `direction` を変えてもう1回録る
- **手順（速度・振幅・時間・回数）は GUI から変えられない**。測定に必要な条件を組み合わせで
  満たすように各プランナーの `SETTINGS` に固定してある（理由は `raspi/auto/_sysid_common.py`）。
  変えるときはコードを直し、`tools/sysid/bench.py` で測れることを確かめる

### スペースの制約で測れないもの・外挿になるもの

| 項目 | 何が足りないか | 影響と対処 |
|---|---|---|
| 駆動加速度の減衰（`drive_fade_speed_m_s`・`drive_top_speed_m_s`） | 2mの直線では約2m/sまでしか出ない | 2m/sまでの加速度は正確。それより上は外挿（学習の上限2.0m/sの範囲では問題ない。最高速を上げるなら長い直線で録り直す） |
| 速度制御の応答（車体の利得・転がり抵抗） | 定速にするのは1.0〜1.5m/s（全開の段は2m/sまで） | 2m/s超の応答は外挿（利得は速度によらないと仮定） |
| 舵の効きとアンダーステア勾配の切り分け | 小舵角（5〜15°）は1.0m/sの1速度でしか測れない（大きい半径の円は描けない） | 速度による曲率の落ち方は30°の旋回からしか読めない。小舵角でも同じ勾配と仮定している |
| リンクの不感帯・ガタ | 階段は0.3m/s（中立付近の段は0.6m/s）の低速（曲率＝ヨーレート÷速度なのでノイズが大きい） | 0.5°程度より小さいと「検出されず（0）」になりうる（見逃す側で、無いものを有るとは言わない。ベンチで確認。センサのノイズが3倍だと不感帯0.6°を3回に1回見逃す） |
| 制御遅延 | STM32→Pi の時刻換算（時刻同期）を信じるしかない | 時刻同期のずれがそのまま乗る（3msずれれば3ms）。2026-09-26 より前のファームは TELEMETRY の時刻が送信時刻で、さらに最大+20ms |
| ヨーレートの大きい旋回 | ジャイロの測定範囲は ±250°/s（4.36rad/s） | 張り付いたサンプルは材料から外し、所見で知らせる（mu が大きく旋回半径が小さい車だけ） |
| グリップ限界（滑らない車） | 旋回は3.0m/s（io_node の上限）まで | 2.2m/s以上走っても滑らなければ、観測した最大の横加速度を mu の下限として返す（学習の速度域では頭打ちしないので挙動には影響しない） |
| グリップ限界 | 30°の円の旋回だけ（速度はゆっくり上げる） | 加減速しながらの限界（摩擦円）や、後輪が先に滑る挙動は測れない。後者は解析が所見で警告する |
| 縦のグリップ（空転） | ②は後輪が滑っても止めない（抑えきれない空転だけ止める） | TC が効いた加速度が読めればそれを上限に。抑えきれない空転では「空転しなかった段〜空転した段」の間としか分からず、上から抑える（所見の★）。シムは空転・TC を表現しない。★グリップを超えても後輪がほとんど滑らない車（ベンチで作った特殊な真値）では、速度PIの積分の溜まり方がシムと違い、利得・転がり抵抗が歪む（★で知らせる） |
| 制動の強さ | 最大＋その下に割り振った3段、2m の直線で止まれる速さ（1.0〜2.0m/s）から。グリップの低い路面（頭打ち約1.8m/s²）では弱い段が転がり抵抗と同程度になり、直線も約0.3m余分に使う | 強さ→減速度は直線（×MD の境界層）＋ロックの頭打ちとして読む |
| 舵モータの止まり方 | 1次遅れ＋止まる位置の幅（ヒステリシス）のモデル | 実機は止まる位置が幅の中で毎回少し違い、大きな段差では行き過ぎることもあるので、残差がノイズの数十倍になる（★）。学習中のゆっくりした舵で効く「幅」は小さな段差から直接読むので合う |
| 旋回の限界 | シムは横の限界を前輪側の頭打ち（mu）で表す | 実機（後輪駆動）は後輪が先に滑る。mu は限界の横加速度として合うが、限界を超えたときの挙動（後輪が流れる）は再現されない |

**解析**（Mac）:

```bash
.venv/bin/python -m tools.sysid.gui       # mcap を試験ごとに選んで「解析」→ 所見を見て「適用」
.venv/bin/python -m tools.sysid.bench     # 解析が正しいかの検証（真値の分かっているシムで全試験）
.venv/bin/python -m pytest tools/sysid/tests   # 通し確認: mcap→解析→vehicle.toml→シムが真値と同じ応答か
```

解析は**シムのモデルそのもの**（`sim/vehicle.py` の `SteerServo`・`SpeedController`・
`VehicleSpec.curvature()`）を実機の記録に当てはめる（出力誤差法）。所見の「残差」が
大きい項目は、実機がシムのモデル構造から外れているサイン。`tools.sysid.bench` は
`raspi/auto/sysid_*.py` を実機と同じ経路（10Hz のスキャン・50Hz の中継・100Hz の
COMMAND・センサのノイズ）で回して、解析が真値を復元できるかと各試験のスペースを出す。

**反映先**: `vehicle.toml` を直せばインタラクティブシム（`sim.run`）と学習環境
（`ml_lidar/env.py`）の両方に効く。学習環境は実ノードを通さないので、COMMAND の
レート制限（`[safety]`）と `control_latency_s` を自分で再現している。

### 4.6 カメラ校正（魚眼レンズ）

前後カメラは IMX219 **160° 広角**。画角が広いぶん像が大きく歪むので、内部パラメータを
チェッカーボードで校正して `config/vehicle.toml` に置く。
**校正値が無い（未校正の）カメラは、従来の `hfov` ピンホールで近似する**——160° では
端ほど大きくズレるので、レンズを替えたら必ずやる。

**レンズを付け替えられるように、レンズで変わる値はプロファイルに分けてある**
（`raspi/core/vehicle.py` の `resolve_lens`）:

```toml
[sensors.cam_front]
lens = "wide160"               # ← 付いているレンズ。"stock"（純正 ≒66°）に変えれば戻る

[sensors.cam_front.lenses.stock]      # x y z roll pitch yaw（取付位置・姿勢）・hfov・bottom_crop・undistort_hfov
[sensors.cam_front.lenses.wide160]
[sensors.cam_front.lenses.wide160.fisheye]   # 校正値（tools/cam_calib が書く）
```

- **取付位置・姿勢もプロファイルに持つ**（モジュールを替えるとステーも変わるため。
  替える前の実測値が残る）。`z` は路面からレンズ中心まで、`pitch` は下向きが正
- **戻すとき**: `lens` を書き換えて `tools/deploy.sh --restart`。両方のレンズの校正値・
  `bottom_crop` が残っているので、他は触らなくてよい。前後で別のレンズにしてもよい
- **別のレンズを足すとき**: `[sensors.cam_*.lenses.<名前>]` を足す（名前は英数字と `-`。
  📷 のファイル名に入るので `_` は使えない）。存在しない名前を `lens` に書くと
  `config/generate.py`（CI の `--check`）が落ちる
- 📷 の写真の名前にはレンズ名が入る（`surge_front_wide160_640x360_…png`）。校正ツールは
  別のレンズの写真を混ぜず、写真を撮ったレンズのプロファイルに書き込む

| 使うところ | 校正値の効き方 |
|---|---|
| IPM（`raspi/nav/ipm.py`）→ `line_trace`・`ftg_cam`・`cam_centerline` | 画素 → 地面座標の変換が魚眼モデルになる |
| `cam_track_node`（`follow_object`）| 画素 → 方位角 |
| GUI の進路ガイド・認識ラインの重畳（`CameraView.tsx`）| 地面 → 画素の投影が魚眼モデルになる（`config/generate.py` 経由） |
| GUI の「補正」映像（タブバーの「魚眼/補正」、設定パネルのカメラ）| `telemetry_node` が仮想ピンホール（`undistort_hfov`）に remap して配信 |

**記録（mcap）・📷 撮影・認識ノードは常に魚眼のままの生画像**を使う（補正は GUI 表示だけ）。
式（Kannala-Brandt、OpenCV `cv2.fisheye` と同じ）と「フル画角・下端クロップ前の画素で持つ」
約束は `raspi/core/camera_model.py` の docstring が正。

**手順**:

1. 印刷用ボードを作る: ランチャーの「カメラ校正」→「印刷用ボードを作る」（または
   `.venv/bin/python -m tools.cam_calib --make-board 9x6 --square 0.025 -o board.pdf`）。
   **「実際のサイズ」で印刷し、平らな板に貼り、1マスを定規で測る。**
   内側の角点は**片方を偶数・片方を奇数**（9x6 など）。両方奇数だと向きが決まらず校正が破綻する
2. 撮る: GUI のタブバーの **📷**（sfl/mcap の隣）。押すたびに前カメラ（と、取得中なら後カメラ）の
   生画像が無劣化 PNG で Mac の ~/Downloads に落ちる（`surge_front_wide160_640x360_crop0.25_….png`）。
   **20 枚前後、画面の中央・四隅・左右の端に、距離と傾きを変えて**写す。魚眼は端ほど歪むので、
   端にボードが写った写真が無いと RMS が小さくても端のガイドが曲がる（ツールの「画面カバー率」を見る）。
   ブレないよう車もボードも止めて撮る
3. 校正: ランチャーの「カメラ校正」でカメラ・ボード寸法・実測の1マスを入れ、「写真を読み込む」→「校正」。
   再投影誤差 RMS が 0.5px 未満、補正後のプレビューで直線が直線に戻っていれば「vehicle.toml に書き込む」
   （`config/generate.py` まで走る）。書き込み先は画面の「レンズ」（既定は今付いているレンズ）。CLI は
   `.venv/bin/python -m tools.cam_calib --cam front --board 9x6 --square 0.025 ~/Downloads/surge_front_*.png [--write]`
4. 反映: `tools/deploy.sh --restart`（GUI を再ビルドして rsync し、telemetry・camera・認識ノードを
   再起動する。校正値は起動時に読むので再起動が要る。io_node は無関係＝E-Stop はラッチしない）

- 校正値はカメラ×レンズごと（前後で個体差がある）。前後のカメラを入れ替えたり、
  レンズを付け直した（ピントリングを回した等）ら撮り直す
- 解像度（`camera_node --size`）を変えても校正値はそのまま使える（幅・高さで縮尺する）。
  `bottom_crop` を変えても同じ（クロップは「下を切っただけ」なので内部パラメータは変わらない）
- カメラ学習モデル（`ml_cam`・`ml_cam_e2e`）は撮ったレンズの画像に依存するので、
  **レンズを替えたら撮り直して再学習が要る**

#### 色むら補正（レンズごとの ISP チューニング）

libcamera の標準のチューニング（`imx219.json`）の色むら補正（ALSC）は**純正レンズ用**。160° 広角に
そのまま当てると、白い壁が「中央は緑・周辺はピンク」に写る（センサーの生の色むらは ±5〜8% しか
無いのに、標準の表が約 ±20% 掛ける。実測: 白壁の R/G が 0.84〜1.45 → 補正後 1.05〜1.12）。

レンズのプロファイルに `isp_tuning = "<ファイル名>"` を書くと、`config/cam_tuning/` の**差分**
（ALSC の表だけ）を標準のチューニングへ重ねて開く（`raspi/core/cam_tuning.py`）。
**書いていないレンズは標準のまま**なので、`lens = "stock"` に戻せば色も元どおりになる。

作り方（レンズを替えた・別のレンズを足したとき）:

1. 白い無地の壁にカメラを向ける。壁が**画面の中心と上の両隅**まで入ること。明るさのむらは構わないが、
   色の違う光（窓と電球）が混ざらないこと
2. Pi の上で（カメラを掴むので配信を止める）:
   ```
   sudo systemctl stop surge-camera
   .venv/bin/python -m raspi.tools.alsc_calib --cam 0 --rows 0,0.54 --out config/cam_tuning/imx219_<レンズ>.json
   sudo systemctl start surge-camera
   ```
   `--rows` は使う行の範囲（車体が写り込む下側を外す）。`残差σ` が数%なら良い。できたファイルは
   Mac へ持ち帰ってコミットする（`deploy.sh` は Mac → Pi の一方向）
3. プロファイルに `isp_tuning` を書いて `tools/deploy.sh --restart`

- **チューニングはプロセスに1つしか効かない**（libcamera が最初にカメラを数えた時点で全カメラ分を
  決める）。前後で `isp_tuning` が違うと、前カメラのものが両方に当たり、起動時に警告が出る。
  `camera_node` でカメラを開く前に `Picamera2.global_camera_info()` を呼ぶと標準に固定されて
  黙って効かなくなる（実機で確認済み）
- 表は中心からの距離だけの関数（車体で壁が隠れていても作れる・壁の光の色の偏りを焼き込まない）。
  周辺減光と、色温度ごとの表は作っていない（標準のまま・ALSC の適応補正まかせ）
- 色が変わるので、色を見る処理（矢印信号の HSV しきい値・カメラ学習モデル）は撮り直し・再調整が要る

---

### 4.7 足回りの制御を調整する（TC・ABS・片輪浮き対策・TV）

STM32 の TC・ABS・片輪浮き対策・TV のゲイン・しきい値は `config/vehicle.toml` の `[control]` が唯一の
定義で、io_node が接続のたびに STM32 へ送る（`docs/uart_protocol.md` §5.8.4。入ったかは GUI の
「設定 → 走行アシスト」の「制御パラメータ: STM32 に適用済み ✓」）。値は `tools/ctrl_tune` が決める。

**しくみ**: STM32 のファーム（隣のリポジトリ `MainF446RE_V3` の `src/control`）を**そのまま Mac で
コンパイル**し、後輪＋タイヤ＋車体のモデル（ファーム側 `host/host_sim.c`）と閉ループにする。調整するのは
実物のコードなので、Python に写した制御とのずれが起きない。ファームの場所が違うなら環境変数
`SURGE_FW_DIR`。

| 段 | やること | どこで |
|---|---|---|
| ① 録る | GUI「システム同定」タブの「制御の同定」2試験（下表）を走らせ、mcap をダウンロード | 実機 |
| ② 同定 | launcher →「制御の調整」→「① 車両モデルの同定」で mcap を開いて解析、`[control.plant]` に適用 | Mac |
| ③ 最適化 | 同「② パラメータの最適化」（数分）。調整前後の物差しを見て `[control]` に適用 | Mac |
| ④ 反映 | `./tools/deploy.sh --restart-io`。GUI で「適用済み ✓」を確かめる | Pi |
| ⑤ 確かめる | システム同定の②前後運動・③旋回を録り直す（TC・ABS が変わると加減速の上限・`mu` が変わる） | 実機 |

| 試験（planner） | 置き方 | 測るもの |
|---|---|---|
| Ⓐ 後輪の空転（`sysid_wheel`） | **後輪を両方とも浮かせる**（車体を台に載せる） | 車輪の慣性・回転の摩擦・MD のトルクの遅れ |
| Ⓑ タイヤの前後力（`sysid_tyre`） | 直線2m。TC・ABS を切って後輪を滑らせる | スリップ率→駆動力・制動力の曲線（μ・荷重移動・傾き・形） |

試験は TC・ABS の ON/OFF を自分で行い（`AutoState.fw_overrides`）、終われば
io_node が元へ戻す。Ⓑは先にⒶを適用してから解析する（力の計算に車輪の慣性を使う）。

```bash
.venv/bin/python -m tools.ctrl_tune.gui          # 同定と最適化（launcher の「制御の調整」と同じ）
.venv/bin/python -m tools.ctrl_tune.bench        # コミット済みのファーム と 作業ツリー を同じ場面で比べる
.venv/bin/python -m pytest tools/ctrl_tune/tests # 試験→解析が真値を復元するか・制御の約束
```

- **ファームの制御を変えるときは、変える前後で `bench` を回して比べる**（`--ref <コミット>` で相手を指定）。
  物差しは `tools/ctrl_tune/scenarios.py` の冒頭（効率・滑っていた割合・横グリップの残り・トルクの暴れ）
- **ローンチコントロール**（`[control]` の `launch_exit_torque_nm`、プロトコル v0.20）: 手動操作で「止まった状態で
  ブレーキ＋アクセル → ブレーキを離す」の手順を踏んだ発進（GUI の設定で ON/OFF。planner も `launch`=1 で使えるが
  既定は無効）で、STM32 が目標車速のランプと車速PIを迂回して全開＋TC で加速する（`docs/uart_protocol.md` §5.6.7）。
  空転を抑えるのは TC なので調整するものは無く、`launch_exit_torque_nm` だけ [dynamics] の同定値から決める
  （`rolling_resistance ÷ speed_plant_gain ÷ 2`。システム同定の②前後運動を録り直したら直す）。
  `bench` の `launch_pi`（車速指令）と `launch_lc`（＋ローンチ）で効きを比べられる
- **スリップ率の目標（`tc_slip_target`・`abs_slip_target`）は最適化しない。人が選ぶ。** 前後の力と
  横グリップのどちらを取るかの方針で、最適な値が無いため（画面に、同定したタイヤでの「目標 → 前後力 /
  残る横グリップ」の表が出る）。最適化が決めるのは、その目標を保つゲイン（kp・ki）だけ。
  目標まで探すと、実機のタイヤ（滑らせても前後力が落ちない）では「ABS を実質切ってロックさせる」が
  最適と出る（`tools/ctrl_tune/optimize.py` 冒頭）
- **記録の先頭が欠けた mcap**（"invalid magic"）も読める
- 最適化は「あり得る車」（路面μ・タイヤの形・慣性・MD の遅れ・ノイズ・前輪の読みの誤差を揺らした組、
  `plant.variants()`）の平均と最悪の中間で選ぶ
- **★の付いた値は既定で適用されない**: 同定は記録がモデルの形から外れたとき、最適化は探索の範囲の
  端に張り付いたとき
- **TV は対象外**（同定も最適化もしない。`tv_*` は `[control]` を手で決める）。TV は**荷重比例の配分**（左右差 = 総駆動トルク × `tv_load_gain_s2_per_m` × 車速 ×
  ヨーレート。フィードフォワードのみ。`docs/uart_protocol.md` §5.8.4）。`tv_load_gain_s2_per_m` の決め方: **旋回しながら
  加速**する走行を TV の ON/OFF（「設定 → 走行アシスト」）で比べ、診断タブの「スリップ率」で内輪と
  外輪がそろい、「TC・ABS トルク上限」で内輪だけ絞られる時間が減る値を探す（「TV 配分の比率」
  「ヨーモーメント」も見る）。定速の旋回では総トルクが小さく、差はほとんど出ない。
  車両モデル（`tools/ctrl_tune`）では 0.045〜0.09 で内外輪の滑り率がそろうが、TC が各輪を個別に絞る
  ので進む距離の差は 1% 未満
- **モデルが表していないもの**: 後輪の横滑り（スピン）、路面の凹凸、タイヤの温度・摩耗、バッテリー電圧に
  よるトルクの頭打ち。調整した値は必ず実機で確かめる

## 5. 実機へ反映する

```bash
tools/deploy.sh --test                    # ふだんはこれ（GUI ビルド + rsync + Pi 上テスト）
```

`deploy.sh` がやること:

1. `surge-mk2` への疎通確認（失敗したら直結・mDNS の直し方を出す）
2. `npm run build`（`--no-gui` で省略）
3. `rsync`（**`.venv` `node_modules` `logs/` `docs/setup_credentials.md` は運ばない**）
4. `gui/dist` だけは `--delete` 付き（ビルドごとにハッシュが変わり、古い `index-*.js` が溜まるため）
5. 指定に応じて test / services / restart
6. 最後に各サービスの `is-active` を表示

> **`.venv` を絶対に運ばない。** Pi の venv は `--system-site-packages` で作られていて
> 中身が Mac と別物（gpiozero / lgpio / picamera2 は OS 同梱を使う）。上書きすると壊れる。

### 反映後に見るところ

```bash
ssh surge-mk2 'systemctl is-active surge-io surge-camera surge-telemetry surge-planning'
```

`active` が並んでいれば良い。GUI が更新されたかは、ブラウザの読み込んでいる
`index-*.js` のハッシュがローカルビルドと一致するかで確認できる。

---

## 6. テスト

```bash
./tools/check.sh                                                  # 生成物一致・文書版番号・pytest・tsc をまとめて
.venv/bin/python -m pytest raspi/tests -q                        # テストだけ（Mac）
tools/deploy.sh --test                                           # Pi 上でも回す
```

**pytest で全部通る**（GPIO の無い環境＝ Mac では一部 skip される）。
ハードウェアもシリアルもカメラも要らない（`ipc://` はテンポラリに閉じ込めてある）。
以下は `raspi/tests/` の主なもの（全一覧は `ls raspi/tests`）。

| ファイル | 何を守っているか |
|---|---|
| `test_proto.py` | CRC のチェック値、**TELEMETRY のバイト位置**、生成物と `protocol.toml` の一致 |
| `test_msgs.py` | **スケールと射影**（単位を間違えても値は出るので「10倍おかしい」としか症状が出ない） |
| `test_zbus.py` | ワイヤ形式と PUB/SUB のポリシー（CONFLATE の罠2つを固定） |
| `test_shm_ring.py` | seqlock。**「壊れたときに壊れたと分かる」**方が要件なので意図的に上書きを起こす |
| `test_gpio.py` | E-Stop ハートビート。**止まるべきときに止まること**を厚く |
| `test_link_tracker.py` | 健全性判定と**ラッチ**（人間が触るまで戻らない） |
| `test_cmd_path.py` | GUI → WS → バス → io_node → UART の指令経路と ARM の3条件 |
| `test_framelog.py` `test_io_node_log.py` | `.sfl` の往復と、壊れたファイルへの頑健性 |
| `test_mcap.py` | 書いた MCAP を**読み直して**検査。尻切れファイルの修復も |
| `test_replay_node.py` | 再生が**実機と同じ結果**になること |
| `test_auto.py` `test_slam2d_raceline.py` | planner の安全条件と段の遷移。**「走る」より「止まる」を厚く** |
| `test_nav.py` | SLAM の精度を**数値で縛る** |
| `test_timesync.py` `test_camera_format.py` | u32 のアンラップ、画素の並び |

- テスト本体は **`unittest`**（標準ライブラリ）で書く。実行は **`pytest`**
  （CI と `tools/check.sh` が使うのに揃えてある。`unittest discover` でも動く）
- **実時間で回るコード（`io_node` / `framelog` / `shm_ring`）は stdlib だけで書く**方針。
  この制約のおかげで、Mac でも Pi でも同じテストが通る
- **プロトコルを直したら `generate.py` を回してからテストする**

---

## 7. 触る前の安全ルール

**自律走行で最初に覚えるのは走らせ方ではなく止め方。**

| 止め方 | 効く範囲 | 復帰 |
|---|---|---|
| GUI のデッドマンを離す | 指令の送信が止まる → 150ms で DISARM | 握り直すだけ |
| GUI の `Esc` / E-STOP ボタン | 即 DISARM | 画面から |
| **車両のボタン2**（E-Stop 解除） | ラッチした E-Stop の解除 | **人間が物理的に押す** |
| 駆動電源を切る | モータのみ。Pi と STM32 は生きたまま | 電源投入 |

### モータが回る条件は3つ、全部そろって初めて回る

1. `io_node` に **`--allow-arm`** が付いている（`install_services.sh` の既定で付く）
2. **`cmd` が 150ms 以内に届き続けている**（途絶＝上位が死んだ、とみなして止める）
3. GUI で**人間が ARM を保持している**

加えて、`protocol_version` の一致と `[control]` の同期が済んでいること。E-Stop・駆動電源ラッチ・STM32 の
再起動・COMMAND 途絶の後は、**ARM を一度外してから入れ直す**まで通らない（io_node と STM32 の両方が持つ。
理由は GUI の ARM ボタンの横と `diag/link` の `arm_inhibit_reason` に出る）。

**engage（自律走行の開始）は ARM ではない。** planning_node は `arm` を立てられないので、
**電源を入れただけで走り出す経路は存在しない。**

### 実機で `--allow-arm` のまま作業するときは

- **車輪を浮かせる**（台上）か、周囲に人・物が無いことを確認してから電源を入れる
- `surge-io` を止める操作は E-Stop ラッチを伴う。**車体に手が届く場所でだけ叩く**

---

## 8. トラブルシューティング

**症状 → 疑う順 → 対処。** どれも一度は実際に踏んだもの。

### 8.1 Pi に繋がらない・遅い

| 症状 | 原因 | 対処 |
|---|---|---|
| `ssh surge-mk2` が遅い（6ms のはずが 40ms） | **直結が生きているのに Wi-Fi 経由で繋がっている** | `ping -b en18 169.254.55.2` のように **IF を明示**して直結の生死を見る。**アダプタの IF 名は挿し直すと変わる**（en18→en19） |
| 直結を挿し直したら mDNS が Wi-Fi しか返さない | avahi が再広告していない | `ssh surge-mk2 'sudo systemctl restart avahi-daemon'` |
| ssh が IPv6 に行ってしまう | avahi は IPv6 リンクローカルも広告し、ssh は IPv6 を優先する | `~/.ssh/config` に **`AddressFamily inet`**（設定済み）。curl は `-4` |
| 素の `ping 169.254.55.2` が通らない | `169.254/16` のルートが複数 IF に重複して載っている | **これで「直結が死んだ」と誤診したことがある。** IF 明示で見る |
| どうしても IP が要る | — | `arp -an \| grep -i 2c:cf:67`（Raspberry Pi Ltd の MAC prefix） |
| Wi-Fi 経由で、数分おきに数秒だけ遅延が 20〜125ms に跳ねる（操縦のデッドマンは 150ms） | **NetworkManager が wpa_supplicant に定期スキャンをさせている**（`bgscan simple:30:-65:300`＝−65dBm より強ければ 5分ごと、弱ければ **30秒ごと**。1回 約3.6秒、全チャンネル） | **接続先の AP（BSSID）を固定する**と止まる: `sudo nmcli con mod <接続名> 802-11-wireless.bssid <BSSID>` → `sudo nmcli con up <接続名>`。確認は `sudo wpa_cli -i wlan0 get_network 0 bgscan`（`FAIL` なら無し）と `iw event -t`。**固定すると他の AP（メッシュの別の子機）へ移らない**。戻すのは BSSID に `""`。接続ごとの設定なので、会場の AP でもやり直す |

### 8.2 GUI が古い・表示がコードと合わない

| 症状 | 原因 | 対処 |
|---|---|---|
| 直したはずの画面が変わらない | **古いプロセスが配っている**（4日前に起動した `telemetry_node` が居座っていた実績あり） | `lsof -nP -iTCP:8000 -sTCP:LISTEN` → `ps -o lstart= -p <PID>` で**起動時刻を見る**。古ければ kill |
| 実機の GUI が古い | `gui/dist` が更新されていない | `tools/deploy.sh`（`--no-gui` を付けていないか確認）。配信中の `index-*.js` のハッシュを見る |
| 偽のデータが出る | `bus_demo` が生き残って `io` endpoint に bind している | `sim.run --kill-stale`、または `pgrep -af bus_demo` |

### 8.3 Mac でシミュレータが起動しない

| 症状 | 原因 | 対処 |
|---|---|---|
| `ImportError: incompatible architecture (have 'arm64', need 'x86_64')` | **ターミナルが Rosetta で動いている**（python.org の Python は親のアーキを継承する） | `arch` で確認（`i386` なら Rosetta）。`arch -arm64 zsh` が応急処置。恒久対応は Terminal.app の「Rosetta を使用して開く」を外す。**VSCode の統合ターミナルは arm64** |
| `OSError: [Errno 48] address already in use ('0.0.0.0', 8000)` | 古い `telemetry_node` が残っている | 上記 8.2 と同じ手順。`sim.run --kill-stale` でも落とせる |

`sim.run` はこの2つを**起動前に検査して報告する**（Rosetta なら自動で立て直す）。

### 8.4 車が動かない

**上から順に潰す。**

1. **ARM しているか。** GUI で ARM（`Enter` / パッドの ×）を保持しているか
2. **`--allow-arm` が付いているか。** `systemctl show surge-io -p ExecStart --value`
3. **駆動電源がラッチしていないか。** 過電流でハード遮断されると
   **電源を入れ直すまで復帰しない**（`drive_power_locked`）。**これは異常ではなく仕様**。
   GUI に「駆動電源ラッチ中」と出る
4. **E-Stop がラッチしていないか。** `surge-io` を止めた／Pi が落ちた後は必ずこれ。
   **車両のボタン2**を押す
5. **`cmd` が届いているか。** Wi-Fi が切れると 150ms で DISARM に落ちる

### 8.5 速度が上がらない

**上限は `LIMITS`（STM32 が報告する車両の物理上限）が正。** io_node も GUI（`effectiveRange`）も、
`LIMITS` を受信済みならそれを使う。**`LIMITS` を受け取るまで io_node は ARM を通さない。**
`--max-speed`（既定 3.0 m/s）・`--max-steer`（`vehicle.toml` の `max_steer` で頭打ち）は ARM していない間の
既定で、`--services` ＋ `--restart-io` で反映する。

| 症状 | 疑う |
|---|---|
| GUI で上限まで上げても実機が出ない | `LIMITS` 未受信（GUI のリンク表示）、または `[control]` が STM32 に入っていない |
| 指令の速度が黙って切られる | io_node の `_send_command` で `LIMITS`/`--max-speed` にクランプ。planner の `max_speed` も見る |
| 出ているが遅い | トルク上限・TC/ABS（診断タブの「TC・ABS トルク上限」）、電池の垂れ |

**据え切りを続けるとステア MD が過熱する**ので `temp[2]` を見ておくこと。

### 8.6 カメラが映らない

| 症状 | 原因 | 対処 |
|---|---|---|
| **GUI のカメラだけ映らない**（点群やメータは正常） | `RemoveIPC=yes` により、**SSH を切った瞬間に `/dev/shm/surge_cam*` が消える**。camera_node 自身はマッピングを保持したまま動き続けるので、**新しく attach するプロセスだけが失敗する** | `install_services.sh` が `RemoveIPC=no` を入れる。入っているか確認: `cat /etc/systemd/logind.conf.d/10-surge-removeipc.conf` |
| ロガーの画像だけ入らない | 同上 | 同上 |
| 前後とも 14fps しか出ない | **未調査の既知問題**（camera_node 自体は 30fps 出ている。配信側で落ちている疑い） | — |

### 8.7 ログに何も出ない

**`python -u` が付いていない。** リダイレクト先が tty でないと Python は 8KB 単位でしか
書かないので、「起動したのにログが空」になる。
`run_stack.sh` と `install_services.sh` は `-u` 付きで起動している。

### 8.8 `pkill -f raspi.nodes` で SSH ごと落ちる

**リモートシェルの cmdline にパターンと同じ文字列が入っている**ため、自分自身にマッチする。
`[r]aspi` のブラケット回避も、同じ行に実際のモジュール名があるので効かない。
→ **スクリプト経由にする**（`run_stack.sh` なら cmdline は `bash run_stack.sh` だけ）。
この事故は3回踏んでいる。

### 8.9 記録まわり

| 症状 | 原因 | 対処 |
|---|---|---|
| `.mcap` が開けない | **MCAP は `finish()` を呼んで初めて索引が書かれる。** 電源断や ssh 切断で尻切れになる | `python -m raspi.tools.mcap_repair <file> --inplace`。87MB のファイルから 10万件・画像 4869枚を救えた実績あり |
| SD が埋まる | `.sfl` 2.5MB/分 ＋ `.mcap` 10.3MB/分 ＝ **1日 18GB** | `surge-logclean.timer` が毎時「7日超」と「合計 8GB 超過分」を消す。**`.mcap` は既定で SD に書かない**（GUI か `tools/record.sh` で PC に流す）。加えて `io_node`/`logger_node` 自身が30秒ごとに空き容量[%]を見て、逼迫していれば警告・世代管理で古い順に削除する（`raspi/rec/logclean.py`。毎時タイマーより早く気づける・記録中のファイルは保護する点が異なる） |
| それでも `.sfl`/`.mcap` の書き込み中に SD が満杯（ENOSPC） | 上の掃除が間に合わなかった・他プロセスが埋めた等 | `FrameLogWriter`/`McapLog` は `OSError` を握り潰し、以後そのファイルへの書き込みを止める（`broken`/`errors`/`last_error` に理由が残る）。**記録スレッドはクラッシュしない**——走行そのものは続く |
| `.sfl` が途中で切れている | 追記のみ・索引なし | **前半は必ず読める。** これが `.sfl` を残している理由 |

**記録まわりの約束**（2026-10）:
- `.sfl` は io_node の専用スレッド（`sfl-writer`）が書き、3 秒ごとに fsync する。SD が詰まってもメインループは
  止まらない（キューが溢れたら古い順に捨て、終了時の統計に `dropped` として出す）
- `logs/<サービス>.out` は毎時の掃除で 10MB を超えたら末尾 2MB だけ残す。毎時の掃除は最終更新が 1 分以内の
  ファイル（記録中）を消さない。`mcap_repair --inplace` の `.mcap.bak` も掃除の対象
- STM32 だけが再起動したら（`t_us` の後退、または要求していない VERSION の到着で検出）、io_node は時刻同期を
  やり直し、`[control]` と GUI で設定した値（TC/TV/ABS/片輪浮き/ブレーキホールド/自動停止マージン）を送り直し、
  ARM を入れ直すまで通さない
- 同定（`tools/sysid`）の「適用」で `[control]` が変わったら、STM32 に入れるには `tools/deploy.sh --restart-io`
  （`--restart` では入らない）。ジャイロ倍率の誤差は `mu` と `steer_link_gain` にそのまま乗る
- 検証用: `sim.slam_bench --locate N`（保存地図の読み直し＋全域探索の検証）・`--speed-quant`・`--time-offset-ms`、
  `sim.bench --errors real|harsh`、`raspi.tools.grip_replay --learner time --plan-lap`

### 8.10 点群がおかしい

| 症状 | 見るところ |
|---|---|
| 左右が反転している | `ScanAssembler` が `車両角 = (360 − センサ角) % 360` で戻している（LD06 が裏向き実装のため）。**シミュレータはわざと反転した状態で出している**ので、ここが壊れると両方で崩れる |
| 実在しない壁が円状に出る | 圧縮フォーマットの `255` は「5.10m **以上**」であって実測点ではない |
| 欠測方向へ舵を切る | **`sector_seen == False` は「障害物なし」ではない。** 欠測は距離 0（侵入禁止）として扱う |
| 古い点群で舵を切る（見た目は正常に動く） | **ZMQ の `CONFLATE` は multipart 非対応**、かつ**ソケット単位**。1フレーム送信・トピック1本につきソケット1本にしてある（`test_zbus.py` が固定） |

### 8.11 Pi が勝手に再起動する

2026-08-07 に **2回**発生。`vcgencmd get_throttled` は `0x0`、温度も正常で**原因未特定**。
永続ジャーナルは有効化済みなので、**再発したら必ずログを見ること。**

```bash
sudo journalctl --list-boots
sudo journalctl -b -1 -n 50        # 前回起動の最後
```

なお「起動 → 数十秒 → ハング」を繰り返していた件は**電源不足**が原因で、
**5V/5A（27W）USB-C PD に交換して解決済み**。緑 LED が点いていても OS はハングし得る。

### 8.12 未解決として残っているもの

[`system_overview.md` §13](system_overview.md#13-現在地とロードマップ) の「未解決・要相談」表を参照（ここに複製しない）。

---

## 9. ログの取り方・見方

### 2種類あり、目的が違う

| | `.sfl` | `.mcap` |
|---|---|---|
| 書く人 | `io_node`（実時間ループの中） | `logger_node`（別プロセス） |
| 中身 | **UART を流れた生バイト**（送受信とも） | 解釈済みトピック **＋ カメラ画像** |
| 依存 | stdlib のみ | `mcap` |
| 置き場 | **Pi の SD**（Wi-Fi が切れても止まらない） | **PC**（SD には書かない） |
| 量 | 2.5MB/分 | 10.3MB/分 |
| 尻切れ耐性 | **強い**（追記のみ） | 弱い（索引が要る）→ `mcap_repair` |
| 用途 | 異常の証拠・確定的な再生 | 解析・Foxglove で目視 |

### 録り方

- **ふだんは GUI の「ログ」タブ**（開始/停止・一覧・ダウンロード・削除）
- GUI を開けない・長時間なら `tools/record.sh`（ssh パイプで PC に直接落とす）
- `.sfl` は後から `python -m raspi.tools.sfl2mcap` で MCAP に変換できる。
  **変換は `replay_node` + `BusBridge` をそのまま通す**ので、実機・再生・変換で
  同じ解釈コードが動く

### 見方

```bash
python -m raspi.tools.logcat logs/x.sfl        # 頻度・期待Hzとの比較・最大間隔・リンク統計
python -m raspi.nodes.replay_node logs/x.sfl --verify    # ログが解析に足りるかの検査
python -m raspi.nodes.replay_node logs/x.sfl --bus --loop  # バスに流し直す（Mac 上で）
```

`.mcap` は **Foxglove Studio でそのまま開ける**（自作 GUI＝ライブ用、Foxglove＝オフライン解析用）。

**カメラ画像を動画にするには**: `launcher.command` から「動画の書き出し」を起動（`tools/mcap_video_gui.py`）→ 記録を
選んで「書き出す」→ 前後それぞれの `<入力>_front.mp4` / `<入力>_rear.mp4` ができる（Foxglove の動画書き出しは
Enterprise 限定のため自前。前後は同じ時間軸なので並べて再生すれば同期する）。CLI は `python -m tools.mcap_video`。

**自己位置を見るには**: `launcher.command`（surge_mk2 直下） から「SLAM 再処理」を起動（`tools/slam_replay_gui.py`）→ 記録を
選んで「実行」→ 終わると `<入力>_slam.mcap` が Foxglove で開く。3D パネルの設定で Display frame を `odom` にし、
Topics の `/viz/scan` `/viz/odom/path` `/viz/slam2d/*` の目を点ける（既定では非表示のことがある）。
点群の Decay time を伸ばすと地図のように溜まり、地図（`/viz/slam2d/map`）は Color mode を
「RGBA (separate fields)」にする。推測航法と SLAM の差は Plot パネルで `/pose_compare.err_pos` `.err_yaw`。

> **GUI からの `.sfl` 再生は実装したが撤回した。** 再生は `surge-io` と同じ ZeroMQ
> エンドポイントを取り合うので、GUI が動いている＝`surge-io` も動いている実運用では必ず失敗する。
> **Mac 上で `replay_node --bus` + `telemetry_node` を直接叩く**のが正しい使い方。

---

## 10. 新しく何かを足すときの定石

### 自動運転モードを足す

**GUI を触らない。** `raspi/auto/` に `Planner` のサブクラスを1つ書き、
`registry.py` に1行足すだけで GUI の選択肢に出る。
パラメータのスライダも planner が宣言した `ParamSpec` から自動生成される。

```
raspi/auto/base.py          Planner / ParamSpec ← バスも WS も知らない純粋な計算
raspi/auto/<新しいの>.py     アルゴリズム本体
raspi/auto/registry.py      id → クラス（+1行）
raspi/nodes/planning_node.py 配線だけ。触らない
```

**GUI にモード名を書くと、増やすたびに2箇所直すことになり、いつか片方が古くなる。**

### UART パケットを増やす・変える

1. `raspi/proto/protocol.toml` を直す（**ここが唯一の定義**）
2. `python3 raspi/proto/generate.py` → `packets.py` と `surge_proto.h` が再生成される
3. **`surge_proto.h` を STM32 側に渡す**
4. `docs/uart_protocol.md` を更新（**仕様の説明はこれが正**。数値の正は toml）
5. テスト → `deploy.sh --restart-io`

**版数がずれている間は走行指令が一切通らない**ので、Pi と STM32 は同時に上げること。

### バスのトピックを増やす

`raspi/msgs/types.py` に型を足し、`raspi/bus/zbus.py` の `TOPIC_OWNER` に持ち主を登録する。
**GUI にも見せるなら `gui/src/types.ts` を手で写す**（生成器は挟んでいない）。

### シミュレータに機能を足す

**シムの概念を実機側に持ち込まない。** コース切替・ノイズ量・欠損率は実機に対応物が無いので、
UI は `sim/gui.py`（pygame）側に置く。シム ↔ シム GUI の通信も内部バスではなく UDP ループバック。
**唯一の例外は `LinkDiag.sim`（SIM バッジ）**で、これは利便性ではなく安全要件
（シムと実機の画面が見分けられないと「シムのつもりで `--allow-arm` した実車が動く」）。

---

## 11. 既知の穴と文書の読み方

**設計の理由（`architecture.md`）は設計時の記述を含み、実装に無い名前が残っている。**
コードが正。特に次は実装に存在しない（`perception_node`・`safety_node`・`grid/local` `pose` `path` トピック）。
Watchdog の役は GPIO6 ハートビートが担い、`hb/*` は logger_node が記録するだけ。
実際のノード構成は [`system_overview.md` §6](system_overview.md#6-raspberry-pi-の中身--プロセスとトピック) を見る。

### 直したほうがよい実装上の小さな穴

| 場所 | 内容 |
|---|---|
| `install_services.sh --remove` | **`surge-logclean.timer` を消し損ねる**。サービス本体が消えたタイマーが enable のまま残り、毎時 fail する |
| `install_services.sh` | 引数を1つしか見ないので **`--safe --with-logger` を併用できない**。`--with-logger` 単体では arm が有効のまま |

---

## 12. 学習ベースの自動運転を動かす（LiDAR E2E / カメラセグメンテーション / カメラE2E）

独立した3本の学習パイプラインがある。どれも「**Mac で学習 → ONNX 化 → `models/` に配置 →
GUI でモデル名を選ぶ**」という流れは共通だが、中身も検証手段も別物なので混同しないこと。

| | LiDAR E2E（`e2e_lidar`） | カメラセグメンテーション（`ftg_cam`/`cam_centerline`） | カメラE2E（`cam_e2e`） |
|---|---|---|---|
| 学習方法 | 強化学習（PPO、シム上で試行錯誤） | 教師あり学習（人がラベル付けした走行画像） | 教師あり学習（人の操舵指令を直接ラベルにする模倣学習） |
| 学習コード | `ml_lidar/`（Mac 専用。Pi には運ばない使い方をする） | `ml_cam/`（同左） | `ml_cam_e2e/`（同左） |
| モデルの置き場 | `models/e2e_lidar/<名前>.onnx`（＋同名 `.json`） | `models/<名前>.onnx`（＋同名 `.json`） | `models/cam_e2e/<名前>.onnx`（＋同名 `.json`。**`.json` の無いモデルは読み込まれない**） |
| 推論コード | `raspi/auto/e2e_lidar.py`（planner本体。配線は`planning_node`がやる） | `raspi/nodes/cam_perception_node.py`（**独立プロセス**。`scan/cam`へ変換）＋ `raspi/auto/follow_the_gap_cam.py`/`cam_centerline.py` | `raspi/nodes/cam_e2e_node.py`（**独立プロセス**。IPMなどの幾何変換もLiDARも使わず、操舵を直接推論。速度は planner が舵から決める）＋ `raspi/auto/cam_e2e.py` |
| シムで検証できるか | **できる**（`sim.run` は LiDAR を持つ） | **できない**（`sim.run` は `--no-camera` 固定でカメラを持たない） | **できない**（同左） |
| 実車で動かすのに要る追加操作 | 無し（`planning_node` が engage 中だけ推論する） | 無し（`surge-cam-perception` は常時 enable。`auto/ctrl` で `ftg_cam`/`cam_centerline` 選択中だけ推論する。下記 12.2） | 無し（`surge-cam-e2e` は常時 enable。`auto/ctrl` で `cam_e2e` 選択中だけ推論する。下記 12.3） |

いずれも `models/` はリポジトリの `.gitignore` 対象（機体・学習ごとに違う大容量ファイルのため）。
配布は `tools/deploy.sh` の rsync に任せる——コミットには乗らない。

### 12.1 LiDAR E2E（強化学習、`e2e_lidar`）

`disparity_extender.py`（`de`）のような**模倣学習ではない**。シム上で「コースに沿って
進めたら＋報酬・衝突したら－報酬」を頼りに、Stable-Baselines3 の PPO で方策を試行錯誤
させる。`de` を再現するのではなく**それを超えうる**代わりに、未知の点群パターンに
対する挙動は原理的に保証できない。planner 側に固定距離の停止策は持たず、
STM32 の `auto_stop` で受ける（`architecture.md`「走る」より「止まる」を先に決める）。
学習の報酬設計・バージョンごとの結果は `ml_lidar/env.py` と `train_rl.py` の docstring、`PROGRESS.md`。

```bash
# 初回だけ（torch / gymnasium / stable-baselines3 / onnxruntime 等）
.venv/bin/pip install -r ml_lidar/requirements.txt

# 1. 学習（毎エピソード形状も道幅も変えたランダムコースで回す）。出力は ml_lidar/runs/<run名>/。
#    既存の run 名には --resume（続きから）か --overwrite が要る。
#    最大舵角は config/vehicle.toml の車両物理限界を常に使う（--max-steer は無い）
.venv/bin/python -m ml_lidar.train_rl --run-name v1 --total-timesteps 2000000 --n-envs 8 --max-speed 2.0

# 学習曲線（別ターミナル。ブラウザで http://localhost:6006 ）
.venv/bin/tensorboard --logdir ml_lidar/runs/v1/tb

# 2.（任意）学習中の方策を観戦する。学習とは別プロセスで、学習を遅くしない
.venv/bin/python -m ml_lidar.watch --model v1

# 3. ONNX 化。観測・行動の契約は run の env_config.json から読む。
#    実運用のモデル選定は eval/best_model_generalized.zip（ランダムコース・DR ありの汎化評価）
.venv/bin/python -m ml_lidar.export_onnx_rl --model ml_lidar/runs/v1/eval/best_model_generalized.zip --name v1

# 4. 実車に配る（models/ は通常の deploy.sh で運ばれる。追加操作は不要）
tools/deploy.sh --no-gui
```

`ml_lidar/app.py`（または `launcher.command` から「ml_lidar」）が上記を操作する薄い Tkinter GUI
（学習・TensorBoard・観戦・エクスポートを run 名ごとに起動でき、**学習を止めずに並行して動かせる**）。
推論・学習のロジックは持たないので、中身のスクリプトを直せば GUI 側は変えなくてよい。
`watch.py` は学習出力を読み直すだけで、学習には関与しない。

GUI での使い方:

1. 設定タブ →「E2E LiDARモデル」ドロップダウンで `models/e2e_lidar/` の一覧から選ぶ
   （増えたモデルが出ないときは「更新」ボタン）
2. 自動運転タブ → モードで「E2E LiDAR」を選択
3. パラメータ `max_speed` はモデル出力をこの値でクランプするだけの安全側の上限——
   学習時の `--max-speed` より大きくしても出力レンジがそこまで届かないので意味が無い。
   **最大舵角は GUI パラメータではない**（`config/vehicle.toml` の車両物理限界を常に使う。他の planner も同様）。
   舵角の意味が別のリンク換算のもとで作られたモデルは契約 JSON の `steer_link` から読み替える（§4.5）
4. `Enter` で ARM → 自動運転タブの「自律走行を開始」で engage

★ **モデル名を切り替えると `e2e_lidar` の engage は自動的に解除される**
（`telemetry_node._on_e2e_model`）。★ モデル未選択・ロード失敗の間は「今のモデルを保持」
（存在しない名前を選んでも走行中のモデルで続行する——ただし選び間違いに気づけるよう
GUI 側にエラーは残る）。

### 12.2 カメラセグメンテーション走行（`ftg_cam`）

前方カメラで「走行可能／不可能」の2値セグメンテーションを行い、`raspi.nav.ipm`
（逆投影）と `OccGrid.raycast()`（既存のレイキャスト）で LiDAR と同じ形の擬似距離配列
に変換して `scan/cam` へ流す。ギャップ探索そのものは書いておらず、`follow_the_gap_cam.py`
が `FollowTheGap`（`ftg`）のロジックをそのまま流用する。

```bash
# 初回だけ
.venv/bin/pip install -r ml_cam/requirements.txt
# SAM のチェックポイント（annotate.py が使う。数百MB）を別途ダウンロード:
#   https://dl.fbaipublicfiles.com/segment_anything/sam_vit_b_01ec64.pth

# 0. 走行を録画する（実車の GUI「ログ」タブで .mcap 記録、「画像を含める」を ON）。
#    Pi 側には新しい記録コードは不要——.mcap をダウンロードして Mac に置くだけ

# 1. .mcap からフレームを抽出（間引きは --min-interval-ms、既定0=全件）
python3 ml_cam/extract_frames.py logs/run1.mcap --out ml_cam/data/frames --cam front

# 2. アノテーション（走行可能な床を1クリック→SAMがマスクを提案。Shift+クリックで除外点）
python3 ml_cam/annotate.py ml_cam/data/frames \
    --checkpoint sam_vit_b_01ec64.pth --model-type vit_b

# 3. 学習（毎エポック IoU を表示。過学習していないか val_iou を見ながら回す）
python3 ml_cam/train.py --frames ml_cam/data/frames --epochs 30 --out ml_cam/runs/latest

# 4. ONNX 化（入力解像度・正規化・閾値という前処理契約を同名 .json に焼く）
python3 ml_cam/export_onnx.py --checkpoint ml_cam/runs/latest/best.pt \
    --size 224x224 --out models/<好きな名前>.onnx

# 5. 実車に配る
tools/deploy.sh --no-gui
```

ターミナル操作をまとめて避けたいなら `ml_cam/app.py`（または `launcher.command`（surge_mk2 直下） から「ml_cam」を
起動）が上記1〜4をボタンとファイル選択ダイアログでサブプロセス起動するだけの
薄い Tkinter GUI。推論・学習のロジックは持たないので、中身のスクリプトを直せば
こちら側は何も変えなくてよい。

★ **実車での推論プロセス（`surge-cam-perception`）は常時 enable の systemd unit。**
`cam_perception_node.py` 自身が `auto/ctrl`（`AutoCtrl.mode`。telemetry_node が GUI の
選択を送り続けるトピック）を見て**`ftg_cam`（と`cam_centerline`）が選ばれている間だけ推論する**。手で起動する必要はない:

```bash
ssh surge-mk2 systemctl status surge-cam-perception   # 動いているか確認するだけならこれで十分
```

- 起動時の既定引数には `--model` を渡していないので、GUI（設定タブ
  「セグメンテーションモデル」）で選んだモデル名を `cam/model` トピック経由で待つ
  （プロセス再起動もSSHも要らずにモデルだけ選び直せる）。モデルのロード自体は
  `ftg_cam` が選ばれていなくても行う（選んだ瞬間から使えるように先読みしておく）
- 前方カメラの共有メモリ（`image/front`）を読むので、**`surge-camera` が動いていること
  が前提**（unit の `After=surge-camera.service` で順序は保証している）
- `auto/ctrl` で `ftg_cam` 以外が選ばれている間・推論が失敗する・モデル未選択の周期は
  全セクタ `sector_seen=False`（＝壁）を出し続けるので、`ftg_cam` は自然に
  「点群の欠測が多すぎる」で止まる側に倒れる（安全側）
- `raspi/nodes/cam_perception_node.py` を直したら `tools/deploy.sh --restart`
  で反映できる（surge-telemetry / surge-camera と一緒に再起動する）

GUI での使い方:

1. 設定タブ →「セグメンテーションモデル」で `models/` 直下の `.onnx` 一覧から選ぶ
2. 自動運転タブ → モードで「Follow the Gap（カメラ）」を選択
3. パラメータの `視野角` はカメラの実視野（hfov≈66°）より広げても見えない範囲なので
   意味が無い。既定60°

★ **シミュレータでは検証できない。** `sim.run` は `--no-camera` 固定でカメラそのものを
持たないため、`ftg_cam` を試すには実車が要る（`e2e_lidar` は LiDAR が対象なので
`sim.run` でそのまま試せるのと対照的）。

### 12.3 カメラE2E（模倣学習、`cam_e2e`）

**前方カメラの画像だけ**から操舵と速度を直接回帰する。`ftg_cam`/`cam_centerline`が
セグメンテーション→`raspi.nav.ipm`（逆投影）という幾何変換を経由するのに対し、
こちらは**幾何変換もLiDARも使わない**——カメラの取付高さが低く（実測8.5cm）
IPMの深度誤差が拡大する問題を、画像から操作を直接学習することで迂回するのが
この方式の狙い。
教師データはSAMの手動アノテーションが不要——走行記録に入っている前方カメラ画像と
人の指令（`cmd`）を時刻で組にするだけで作れる。**その代わり、悪い手本を外す作業が
モデルの出来を決める。**

#### 手順（操作パネル）

`launcher.command`（surge_mk2 直下）から「ml_cam_e2e」を起動する（`ml_cam_e2e/app.py`）。
タブは作業の順に並んでいる。

| タブ | すること |
|---|---|
| ① ペア抽出 | 録った `.mcap` を選んで (画像, 人の操作) の組を取り出す。モデル名を決める（以降のタブはその名前を使う） |
| ② 確認・選別 | 映像と舵・速度の時系列を見て、手本にしない区間（コースアウト・やり直し等）を範囲で除外する。`←/→` 1枚送り、`space` 再生、`[` `]` で範囲、`x` で除外 |
| ③ 偏り | 学習に使うコマの舵・速度の分布と、記録ごとの内訳。直進が大半なら「直進過多の補正」を上げる |
| ④ 学習 | 学習曲線の青（`val_mae`）は検証データでの舵の誤差 |
| ⑤ エクスポート | `models/cam_e2e/<名前>.onnx` と契約 `.json` を書く。解像度と正規化の基準は学習時の値を自動で使う |
| ⑥ 評価 | 予測と手本の時系列、検証データでの誤差、誤差の大きい場面の一覧。場面から「選別タブで開く」で②へ飛べる |

**②と⑥を往復するのが基本の使い方。** 誤差の大きい場面は、モデルが悪いのではなく
手本が悪い（そこだけ人が違う操作をした）ことが多い。除外して学習し直す。

#### 走行の録り方（★ここで出来が決まる）

- **画像つき・高めのレートで録る。** `tools/record.sh --image-hz 15`（Mac の `logs/` に落ちる）。
  実機GUIの MCAP ボタンは 5Hz 固定なので枚数が稼げない
- **手本は人の手動運転か、うまく走れている自動運転。** 既定は手動運転だけを使う。
  `slam2d_route` などの自動運転の走行を手本にする（LiDAR の走りをカメラで真似させる）なら、
  ①の「自動運転中の走行も手本にする」にチェックを入れる（コマンドでは `--include-auto`）。
  入れ忘れると自動運転の記録は全部落ちて0枚になる（理由はダイアログに出る）。
  `cam_e2e` 自身の走行は、チェックを入れても手本にしない
- **速度モードで運転する。** トルクモードの記録は `target_speed` が 0 のままで、抽出で全部
  落ちる（落ちた理由と枚数は①のログに出る）
- **ゆっくり、舵に集中して走ってよい。** 実車の速度は舵から決めるので（下の「仕組みの要点」）、
  手本の速さは走りに影響しない。速く走って舵が雑になるより、遅くても正確な方がよい
- **手本どおりの走行だけでなく、壁際から戻る操作も録る。** きれいな走行だけで学習すると、
  少しずれたときにどう戻すかを知らないモデルになる（ずれが積み上がってコースアウトする）
- **両方向を走る。** 片回りだけだと左右どちらかの舵しか学ばない（③で左右の枚数を確認）。
  左右反転の水増しは既定で有効だが、コースが左右対称でない（片側通行の標識など）なら④で切る
- **止まって待つ・後退する**区間は自動で学習から外れる（`ml_cam_e2e/samples.py`）。
  わざわざ切り取らなくてよい
- レンズや取付角度を変えたら**録り直して再学習**（§4.6）

#### 手順（コマンド）

```bash
.venv/bin/pip install -r ml_cam_e2e/requirements.txt       # 初回だけ

python3 ml_cam_e2e/extract_pairs.py logs/run1.mcap --out ml_cam_e2e/runs/v1/frames
python3 ml_cam_e2e/train.py --frames ml_cam_e2e/runs/v1/frames --out ml_cam_e2e/runs/v1 --epochs 30
python3 ml_cam_e2e/export_onnx.py --checkpoint ml_cam_e2e/runs/v1/best.pt --out models/cam_e2e/v1.onnx
python3 ml_cam_e2e/eval_model.py --frames ml_cam_e2e/runs/v1/frames --run ml_cam_e2e/runs/v1 \
    --model models/cam_e2e/v1.onnx                          # → runs/v1/eval.csv
tools/deploy.sh --no-gui                                    # 実車に配る
```

#### 実車で走らせる

1. 自動運転タブ → モードで「E2Eカメラ（模倣学習）」→ **「カメラE2Eモデル」欄でモデルを選ぶ**
   （選択は `config/cam_e2e_model.json` に残る。engage 中に選び直すと engage は落ちる）
2. `Enter` で ARM → 「自律走行を開始」で engage

パラメータ（`raspi/auto/cam_e2e.py`）:

| 名前 | 既定 | 意味 |
|---|---|---|
| 最高速度 | 0.30 m/s | 直線で出す速度。**初めてのモデルはここを低くして試す** |
| 最低速度 | 0.10 m/s | 舵をいっぱいに切ってもこれ以下にはしない |
| 旋回時の横加速度上限 | 3.0 m/s² | 舵の曲率 κ=tan(δ)/L から `v ≤ sqrt(これ/κ)` で減速する。★実車未計測の暫定値 |
| 舵の倍率 | 1.0 | 回帰は平均へ寄って舵が浅くなりがち。コーナーで膨らむなら上げる |
| 舵の平滑化 | 0.10 s | 1次遅れの時定数 |

★ **このモードは自分では止まらない。** LiDAR を見ないので、前に物があっても
モデルが「進め」と出せば進む。止める根拠は planner の外にある——GUI の
**自動停止（STM32 の `auto_stop`）を ON** にしておくこと、人が舵・スロットル・
ブレーキに触れれば即座に解除されること、通信が切れればデッドマンで止まること。

#### 仕組みの要点

- **学習と推論の前処理は同じ関数**（`raspi/core/cam_e2e_preproc.py`、numpy のみ）。色順（RGB）・
  縮小（面積平均）・正規化を1箇所で定義し、Pi の推論・学習・評価・操作パネルが全部これを通る。
  処理を変えたら `PREPROC_VERSION` を上げる——版の違うモデルは `cam_e2e_node` が読み込みを拒否する
  （学習・推論で色順・縮小方式がずれると精度が出ない）
- **モデルの出力は2個**（`steer_norm` -1..1、`speed_norm` 0..1。前進のみ）。物理量へ戻す基準
  （`max_steer`・`speed_ref`）は同梱 `.json` に書く。`speed_ref` は学習に使った手本の最高速度
- **速度は舵から決める。モデルの `speed_norm` は実車で使わない。**手本は舵に集中して
  ゆっくり走った方が質が上がるが、そうすると速度のラベルは「遅い一定値」で情報が無く、手本の
  遅さがそのまま実車の上限になる。学習の `--speed-weight` は既定 0（舵だけ）——このとき
  `speed_norm` は学習されていない値なので、⑥評価は速度のグラフ・誤差・予測値を出さない
  （`train_config.json` の `speed_weight` で判断する）。出力の契約は変えていない
  
- ★ **舵が切れてから遅くなる**ので、コーナー手前の減速は遅れる。舵いっぱい（30°）での速度は
  既定の 3.0 m/s² で約 1.1 m/s——**最高速度がそれ以下なら減速は一切起きない**。足りなければ
  「先の経路（数点）を出すモデルに変え、その曲率で手前から減速する」が次の手
- **検証データは5秒のかたまり単位で分ける**（隣のコマはほぼ同じ絵なので、1枚ずつ無作為に
  分けると誤差が実力より小さく出る）
- `surge-cam-e2e` は常時 enable の systemd unit。`auto/ctrl` が `cam_e2e` で、かつ ARM 中の間だけ
  推論する（既定 10Hz、`--infer-hz`）。engage 中は前カメラの fps が上限まで上がる
- モデル未選択・推論失敗・カメラ途絶の周期は `ready=False` を出すので、停止側へ倒れる
- `raspi/nodes/cam_e2e_node.py` を直したら `tools/deploy.sh --restart`

★ **シミュレータでは走行を検証できない。**`ftg_cam`と同じ理由（`sim.run`はカメラを持たない。
`sim.bench --mode cam_e2e` も対象外）。代わりに `ml_cam_e2e/tests/test_pipeline.py` が、合成した記録で
抽出→学習→エクスポート→実車側の読み込み→評価までを通し、学習したモデルが手本の向きと速度を
再現することを確かめる（`.venv/bin/python -m pytest ml_cam_e2e/tests -q`）。

---

## 付録: ディレクトリと担当

| ディレクトリ | 中身 | 直したら |
|---|---|---|
| `raspi/nodes/` | プロセス本体（io / camera / telemetry / planning / logger / replay / **cam_perception** / **cam_e2e** / **line_perception**）。`cam_perception_node`（`surge-cam-perception`）・`cam_e2e_node`（`surge-cam-e2e`）・`line_perception_node`（`surge-line-perception`）とも常時 enable、`auto/ctrl` でそれぞれ対応モード選択中だけ推論/認識する（§12.2・§12.3） | ノードごとに再起動 |
| `raspi/auto/` | 自動運転アルゴリズム（**バスも WS も知らない純粋な計算**。`e2e_lidar.py`/`follow_the_gap_cam.py`/`cam_e2e.py` もここ） | surge-planning 再起動 |
| `raspi/nav/` | 経路生成・追従・回避・局所地図などの部品（SLAM 本体は `slam2d/`）。`ipm.py`（カメラ逆投影）は `ftg_cam`/`cam_centerline` が使用（`cam_e2e` は経由しない） | surge-planning 再起動 |
| `raspi/proto/` | UART 定義（**STM32 と共有する唯一の定義**） | 再生成 ＋ `--restart-io` |
| `raspi/bus/` `raspi/msgs/` | ZeroMQ ラッパ・共有メモリ・メッセージ型 | 関係ノード全部 |
| `raspi/io/` | `SerialLink` / GPIO（**`import serial` はここ1箇所だけ**） | `--restart-io` |
| `raspi/rec/` | `.sfl` / MCAP の書き出し | io / logger |
| `raspi/tools/` | 単発の検査・変換ツール | — |
| `raspi/tests/` | pytest（`tools/check.sh`） | — |
| `gui/` | React + TypeScript | rsync だけ |
| `sim/` | Mac 専用シミュレータ（**Pi には運ぶが使わない**）。カメラは持たない | — |
| `ml_cam/` | カメラセグメンテーションの学習パイプライン（Mac 専用。§12.2） | Pi には無関係 |
| `ml_cam_e2e/` | カメラE2E（模倣学習）の学習パイプライン（Mac 専用。§12.3） | Pi には無関係 |
| `ml_lidar/` | LiDAR E2E 強化学習パイプライン（Mac 専用。§12.1） | Pi には無関係 |
| `models/` | 学習済み ONNX の置き場（`models/*.onnx`=カメラセグメンテーション用、`models/cam_e2e/*.onnx`=カメラE2E用、`models/e2e_lidar/*.onnx`=LiDAR E2E用）。`.gitignore` 対象 | `tools/deploy.sh` で運ぶだけ。再起動は不要 |
| `config/` | 車両諸元 `vehicle.toml`・自動運転パラメータ・生成器（§2） | 読んでいるノード |
| `slam2d/` | 車体非依存の汎用 2D LiDAR SLAM（`slam2d/README.md`） | surge-planning 再起動 |
| `tools/` | `deploy.sh` / `record.sh` / `sysid` / `ctrl_tune` / `cam_calib` | — |
