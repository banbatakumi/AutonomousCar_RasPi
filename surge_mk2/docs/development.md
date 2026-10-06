# SURGE Mark.2 開発ガイド — 変更の反映とトラブルシューティング

**対象**: このリポジトリを触る人（＝3ヶ月後の自分を含む）
**最終更新**: 2026-08-28

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
11. [文書と実装のズレ（棚卸し）](#11-文書と実装のズレ2026-08-16-時点の棚卸し)
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
| `config/auto.json` `auto_presets.json` `camera.json` `cam_model.json` `e2e_lidar_model.json` `odometer.json`・`saved_maps/` | **機械ごとの状態。`deploy.sh` は運ばない**（telemetry_node / io_node が動いている機械で書き、起動時に戻す。Mac＝シム、Pi＝実機で別々）。Mac の値で Pi を上書きしていたのを 2026-09-29 に止めた。地図は GUI の DL/UL で運ぶ（経路の設定も同梱） | しない |
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
.venv/bin/python -m sim.run --course slalom       # コース指定
.venv/bin/python -m sim.run --list                # コース一覧
.venv/bin/python -m sim.run --kill-stale          # 古いプロセスを落としてから起動
.venv/bin/python -m sim.editor [名前]             # コースエディタ
.venv/bin/python -m sim.bench --course circuit    # planner の数値評価

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

> ### ⚠ コースは `circuit` しか入っていない
>
> `sim/courses/` にあるのは **`circuit.json` の1本だけ**。
> `sim/README.md` や `io_node --course` の既定値が挙げている
> `oval.png` / `slalom` / `room.png` / `chicane.json` は**現存しない**
> （`make_courses.py` も PNG 生成コードを失っている）。
>
> - **`sim.run` は既定が `--course circuit` なので問題なく動く**
> - **壊れるのは `sim/README.md` の「手で3つ起動する場合」をそのまま貼ったとき**
>   （`--course sim/courses/oval.png` が無い）
> - 増やすなら `python -m sim.editor` でコースを作る（下記）

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
.venv/bin/python -m sim.bench --course circuit
.venv/bin/python -m sim.bench --mode de --time 180 --set safety_half_width=0.12
```

**プロセスもバスも GUI も使わず、1プロセスの中で `io_node` と同じことをやる。**
`ScanAssembler` の鏡像反転も `command_from_cmd` のクランプも通るので、
「ここで走れば `sim.run` でも走る」。

出るのは**計画周期（平均/中央/最大 ms）・自己位置誤差・向きの誤差・横偏差・
周回数・走行距離・衝突回数・ラップタイム・地図の正確さ**。

見た目では「なんとなく速い」しか分からない。**数字で出してから採否を決める。**
実際、この数字によって「SLAM 版より反射型（`de`）の方が 2.2 倍速い」ことが分かり、
方針が変わった。

> **ベンチだけが真値を読める。** シムの真値は UDP の私設チャンネルにしか出ないので、
> SLAM の推定と真値を毎周期突き合わせられるのはここだけ（実機では絶対に手に入らない数字）。

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

`config/vehicle.toml` の `[dynamics]` は実機で測る。**2026-09-24 に手順と解析を
全面的に作り直した**（それ以前の値は系統誤差を含むので測り直すこと。理由は
`tools/sysid/fit.py` のモジュールdocstring）。2026-09-26 に第三者検証を受けて、速度モデルを
ファームの PI に、安全ガード・周期誤差の補正・残差の警告などを加えた（PROGRESS.md 同日の節）。
2026-09-27 に**試験を4つに組み直した**（バンビ「試験はなるべくシンプルな方が正しい値が取れる」）:
速度応答と加減速を「加速度を段階的に上げる前後運動」1つに、旋回は速度を段ではなく連続して
上げる形に、ステアの掃引・小さな往復は「段を保持する階段」に。測る量は減らしていない——
どの量も、すでに許容している同定誤差と同程度以上に学習・シムに効くことを確かめた（PROGRESS.md
「システム同定で測る量の必要性」）。

**録る前に**: STM32 に 2026-09-26 以降のファーム（TELEMETRY の `t_us` がスナップショットの
時刻になったもの、`fw_id` 0x4D463304 以降）を書き込んでおく。古いファームでは④の制御遅延が最大+20msずれる
（`docs/uart_protocol.md` §5.3）。

**実機での録り方**: GUI の「システム同定」タブで試験を選び、「試験開始」→ ARM を
保持 → 「完了」「中止」が出て車が止まったら自動で記録を止めてダウンロード（試験ごとに1つの mcap）。

| 順 | 試験（planner） | 測るもの | 必要なスペース（シムで計測、マージン別） |
|---|---|---|---|
| ① | ステア `sysid_steer` | `tau_steer_s`・`dead_time_s`（走行中1.0m/s・±5〜15°のステップ）、`steer_rate_limit_rad_s`（低速・±30°のステップ）、`steer_servo_hysteresis_rad`（止まる位置の幅。階段の3.5°以下の小さな段差から直接読む）・`steer_servo_gain`（段を保持する階段。同じ舵角に上りと下りの両方から近づく） | 直線2m＋車長＋約0.3m × 幅約1.1m（階段で車の向きが回る分）。約1分30秒 |
| ② | 前後運動 `sysid_accel` | `speed_plant_gain`（トルク→加速度の利得）・`rolling_resistance`・駆動加速度の上限とその速度依存・`speed_decel_m_s2`（上限はファームの目標ランプ3.0m/s²より低いときだけ見える。見えなければ0）・`brake_decel_per_nm`（制動トルク→減速度の傾き）・`brake_decel_m_s2`（グリップによる制動の頭打ち）。加速度を 0.8→1.6→2.4m/s²→全開と上げ、段ごとにブレーキ（強さも 0.04→0.08→0.12→0.15N·m と上げる）と速度指令0で1回ずつ止まる。**後輪が滑ったら（滑り率0.1超が0.1s）その段まで、抑えきれない空転ならすぐ止めて**上の段を走らない（TC の旗は見ない）。ファームの PI の定数 `speed_kp` 等は同定しない | 直線 `run_length_m`（既定2.0m）＋車長 × 幅約0.25m。約40秒 |
| ③ | 旋回グリップ `sysid_corner` | `mu`（舵30°、0.3m/sから0.06m/s²で連続して速くする。**後輪が滑った**（滑り率0.1超）か**横加速度が頭打ち**の早い方を限界とし、舵を保ったまま1.0m/s²でゆっくり止まる） | 約2m 四方。約30〜45秒 |
| ④ | 遅延 `sysid_latency` | `control_latency_s`（スキャン完了→STM32受理） | その場（据え切りで小さく反転）。約15秒 |
| ①②③ | まとめて（`fit_geometry`） | `steer_gain`・`steer_gain_cubic`・`steer_offset_rad`・`understeer_gradient`・`steer_link_hysteresis_rad`・`steer_link_deadband_rad`・`yaw_relaxation_m`（タイヤの緩和長）・`yaw_rate_filter_s`（IMU のローパス）・`yaw_natural_freq_rad_s`/`yaw_damping`（振動的なヨー応答＝曲率の2次遅れ。有意なときだけ） | — |

**速度の使い分け**: リアルタイム（プランナーの判定・E2E の観測）は STM32 の `speed`、
**同定の解析は前輪オドメトリをゼロ位相で微分した速度**（前後2サンプルずつ計5点に直線を当てはめた
傾き、`fit.odom_speed`）。`speed` は 2kHz の値にローパス（1段・9.95ms）を掛けて 50Hz に間引いた
もので、遅れがあるうえ 25Hz より上のノイズが間引きで折り返して混ざる。`odom_dist` はローパス前の
角度を積算した位置（0.1mm 単位、STM32 内部は約0.03mm で端数も繰り越し）なので、録り終えてから
前後で微分すれば遅れも折り返しも無い。20ms で1カウントも進まないのは 5mm/s 未満のときだけで、
5点の当てはめで量子化の誤差は約0.5mm/s（端数が出る条件で確認）。旧ファームの `speed`（100ms）で
解析したときは、ベンチでブレーキ減速度・ガタなど5項目が誤った。
`vehicle.toml` の `speed_filter_s`（ファームの定数、同定しない）はシムの観測にだけ使う。
**ファームの `DRIVE_LPF_K_FRONT` を変えたら直す**——解析の所見「速度センサの自己点検」が、
記録の `speed` とオドメトリを照合して食い違いを警告する（静止中のノイズも出る）。
同じく `speed_kp`・`speed_ki`・`speed_torque_max_nm`・`speed_ramp_max_m_s2` もファームの定数
（`DRIVE_SPEED_KP/KI`・`DRIVE_MAX_TORQUE_NM`×2・`DRIVE_MAX_ACCEL_M_S2`）なので、ファームを変えたら直す。

**前輪エンコーダの周期誤差**: アナログの絶対角エンコーダは1回転の中で読みがうねる。うねりは
曲率（ヨーレート÷速度）に乗ってガタ・不感帯を取り違えさせる（検証で±3.4°の誤差で起きた）ので、
記録を読むときに定速の区間から左右それぞれの誤差（1回転に1〜3回の高調波）を推定し、有意なら
オドメトリを補正する（`fit.correct_encoder_ripple`。所見に振幅が出る）。

**所見の★**: 各解析は残差を静止中のノイズ（速度・舵角・ヨーレート）と比べ、2倍を超えると
「モデルの形から外れている」と警告する。★が出た値はそのまま使わず原因を確かめること。
GUI は★の出た試験の項目を**既定で未チェック**にし、理由を横に出す（`FitResult.warnings`）。
2026-09-26 までの2つの記録（速度応答＋加減速）を合わせて解析するときは、加減速に★が出たら
合わせて当てはめ直した車体の利得・転がり抵抗を採らない（外れた分を吸収して壊れる。空転を入れた
真値で 21.4→17.8）。遅延試験は、`t_us` が送信時刻だった古いファーム（`fw_id` < 0x4D463304）の
記録なら★。
★の基準のノイズは、止まっている間と**走っている間**の大きい方（2026-09-27。走っている間の方が
ずっと大きく、実機で曲率は舵角換算0.5〜1.0°、速度は0.7〜2.7cm/s。止まっている間だけを基準にして
いたので、残差がノイズ程度でも★「40倍」と出た）。
**制動**: 減速度 = 傾き×強さ×tanh(車速÷境界層)＋転がり抵抗（後輪のロックで頭打ち）。境界層0.6m/s・
不感帯0.06m/s は**MD のファームの定数**（`BLDC/ProgramV4` の BRAKE_BOUNDARY/DEADBAND_SPEED_RAD_S ×
後輪半径。一定トルクの符号切り替えはバンバン発振するので境界層で0へ落とす）で `vehicle.toml` に書く。
測るのは傾きと頭打ちだけで、制動中の**走行距離（位置）**に当てはめる（微分しないので止まる直前も使える）。
実機（2026-09-27）で傾き約35/N·m、0.12N·m 以上で後輪がロック（約3.7m/s²）。★**最大制動（0.15N·m、
フェイルセーフ・E-Stop）は後輪をロックさせる**（制動中は TC が効かない。ABS は検討中）。同定の試験の
中止・周囲クリアランスでの停止は 0.08N·m で制動し舵を保つ（`ABORT_BRAKE_TORQUE_NM`）。
**速度の比べ方**: 実測の速度はオドメトリを±45msの窓で微分したものなので、加速度が急に変わる所
（全開加速→制動の切り替えなど）で窓の幅だけ鈍る。解析はシムの応答にも同じ微分を掛けて比べる
（`fit._simulate_speed`。掛けないと、上限の無い車に約2.3m/sの最高速を読んだ）。
要素を入れるか（ヒステリシス・上限など）の尤度比は、残差の自己相関から有効な自由度を見積もる
（`fit.n_eff`）。

**安全**: 直線・旋回の試験は前輪の速度を後輪モータの周速と照合し、後輪が回っているのに前輪が
大きく遅い状態が0.3s続いたら中止して制動する（`_sysid_common.SensorGuard`。前輪エンコーダが
止まると、以前は加減速試験が2mの直線で35m走った）。戻りのタイムアウトも短くした。
中止の理由は、前輪がほぼ止まっていれば「前輪エンコーダの不調？」、前輪も動いていれば（または
TC が介入中なら）「後輪の空転か前輪エンコーダの不調」と出す。★**空転**（加速がタイヤのグリップを
超える）: 後輪の荷重配分・縦の mu は未実測で、ファームの目標ランプ3.0m/s² が出せるかは実機で
確かめるまで分からない。②は加速度を段階的に上げ、
- **後輪が滑ったら**（後輪の周速の平均と車速の差 ÷ 車速（1m/s未満は1m/sで割る）が0.1を超えた状態が
  0.1s。TC の旗は見ない——実機で旗が30〜50msだけ立ったのを数えて全開の段を飛ばした）その段は
  最後まで走り、上の段へは進まない。滑りながら出た加速度を、解析が加速の上限として読む（グリップで
  決まる、この車が実際に出せる加速度）。滑った区間は速度制御の当てはめから外す
- **抑えきれない空転**（後輪が前輪より 0.2m/s＋30% 以上速い状態が 0.1s 続く）ならすぐ制動して
  試験を終える。解析は★を出し、加速の上限を空転した段の加速度で上から抑える

TC の細かい動き（トルクを削っては戻す）はシムに入っていない（シムは空転も TC も表現せず、
グリップの限界を「加速の上限」の一定値で近似する）。TC の定数（`DRIVE_TC_*`）はファームで決まって
いるので測らない。空転が出るなら、①の走行部・③の COMMAND の `accel_limit` をその段より下げるか
検討する。

- **全部を「直線2m」と「約2m四方」の2か所で録れる**（旧手順は旋回が約2.0m四方・加減速が
  直線約3.2m で、しかも学習で使う速度・舵角で測っていなかった）
- どの試験も開始直後の1秒は止まる（ジャイロのバイアスをそこで読む）
- 直線の試験（①②）は、置いた向きのまま `run_length_m` の直線を往復する。①は向きが
  ずれないようジャイロで積算した自己位置で元の直線へ寄せ直すが、置く向きは合わせておく
- ②のブレーキの強さはプランナーが段ごとに指定する（`AutoState.brake_torque`→`COMMAND.brake_torque`）
  ので、GUI の制動トルクの設定は関係ない（2026-09-26 までは GUI の値の1通りで、最大にしておく必要が
  あった）。自律走行中に人（GUI）がブレーキを掛ければ、強い方が効く
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
| 縦のグリップ（空転） | ②で TC が介入・空転したら上の段を走らない | TC が効いた加速度が読めればそれを上限に。抑えきれない空転では「空転しなかった段〜空転した段」の間としか分からず、上から抑える（所見の★）。シムは空転・TC を表現しない。★グリップを超えても後輪がほとんど滑らない車（ベンチで作った特殊な真値）では、速度PIの積分の溜まり方がシムと違い、利得・転がり抵抗が歪む（★で知らせる） |
| 制動の強さ | 0.04〜0.15N·m の4段、2m の直線で止まれる速さ（1.0〜2.0m/s）から | 強さ→減速度は直線（×MD の境界層）＋ロックの頭打ちとして読む |
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
- 自動運転のカメラ学習モデル（`ml_cam`・`ml_cam_e2e`）は旧レンズの画像で学習してあるので、
  **160° レンズで撮り直して再学習が要る**

#### 色むら補正（レンズごとの ISP チューニング）

libcamera の標準のチューニング（`imx219.json`）の色むら補正（ALSC）は**純正レンズ用**。160° 広角に
そのまま当てると、白い壁が「中央は緑・周辺はピンク」に写る（センサーの生の色むらは ±5〜8% しか
無いのに、標準の表が約 ±20% 掛ける。2026-10-06 実測: 白壁の R/G が 0.84〜1.45 → 補正後 1.05〜1.12）。

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
- **スリップ率の目標（`tc_slip_target`・`abs_slip_target`）は最適化しない。人が選ぶ。** 前後の力と
  横グリップのどちらを取るかの方針で、最適な値が無いため（画面に、同定したタイヤでの「目標 → 前後力 /
  残る横グリップ」の表が出る）。最適化が決めるのは、その目標を保つゲイン（kp・ki）だけ。
  以前は目標も探していたが、実機のタイヤ（滑らせても前後力が落ちない）では「ABS を実質切って
  ロックさせる」が最適と出た（2026-10-05。`tools/ctrl_tune/optimize.py` 冒頭）
- **記録の先頭が欠けた mcap**（"invalid magic"）も読める。2026-10-05 まで、ブラウザの接続より先に
  出た先頭の塊を中継が捨てることがあった（`telemetry_node._mcap_pump` で修正済み）
- 最適化は「あり得る車」（路面μ・タイヤの形・慣性・MD の遅れ・ノイズ・前輪の読みの誤差を揺らした組、
  `plant.variants()`）の平均と最悪の中間で選ぶ
- **★の付いた値は既定で適用されない**: 同定は記録がモデルの形から外れたとき、最適化は探索の範囲の
  端に張り付いたとき
- **TV は対象外**（同定も最適化もしない。`tv_*` は `[control]` を手で決める）。2026-10-05 に一度作って
  外した: 実測でヨーモーメント 0.1N·m → 0.056rad/s、上限 0.15N·m でも旋回中のヨーレートの約3%しか
  動かせず、この車両モデルは限界での後輪の横滑りも表せない。見直すなら、TV の ON/OFF の走り比べと
  診断タブの「TV ヨーレート」「TV ヨーモーメント」から
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

**pytest で全部通る**（skip 1 件は「GPIO の無い環境」＝ Mac だから）。
ハードウェアもシリアルもカメラも要らない（`ipc://` はテンポラリに閉じ込めてある）。
`raspi/tests/` 自体は標準ライブラリの `unittest.TestCase` で書かれているが、
CI（`.github/workflows/ci.yml`）と `tools/check.sh` が pytest に統一したので、
通常はこちらを使う（`python -m unittest discover` でも同じテストは動く）。

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
| `test_auto.py` `test_raceline.py` | planner の安全条件と段の遷移。**「走る」より「止まる」を厚く** |
| `test_nav.py` | SLAM の精度を**数値で縛る** |
| `test_timesync.py` `test_camera_format.py` | u32 のアンラップ、画素の並び |

- テスト本体は **`unittest`**（標準ライブラリ）で書く。実行は **`pytest`**
  （CI と `tools/check.sh` が使うのに揃えてある。`unittest discover` でも動く）
- **実時間で回るコード（`io_node` / `framelog` / `shm_ring`）は stdlib だけで書く**方針。
  この制約のおかげで、Mac でも Pi でも同じテストが通る
- **プロトコルを直したら `generate.py` を回してからテストする**

> ⚠ 実行中に `AttributeError: '_auto_mode'` のトレースバックが混じることがあるが、
> テスト自体は通る。`telemetry_node` の非同期タスク内で出ているもので、
> **未初期化属性の疑いとして残っている**（ノイズと決めつけない方がよい）。

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

1. **ARM しているか。** GUI のデッドマン（Space / R2）を握っているか
2. **`--allow-arm` が付いているか。** `systemctl show surge-io -p ExecStart --value`
3. **駆動電源がラッチしていないか。** 過電流でハード遮断されると
   **電源を入れ直すまで復帰しない**（`drive_power_locked`）。**これは異常ではなく仕様**。
   GUI に「駆動電源ラッチ中」と出る
4. **E-Stop がラッチしていないか。** `surge-io` を止めた／Pi が落ちた後は必ずこれ。
   **車両のボタン2**を押す
5. **`cmd` が届いているか。** Wi-Fi が切れると 150ms で DISARM に落ちる

### 8.5 速度が上がらない

**速度の上限は3段ある。一番低いところで頭打ちになる。**

| # | どこ | 既定 | 直す場所 |
|---|---|---|---|
| 1 | GUI のスライダ上限 `PI_MAX_SPEED_CAP` | — | `gui/src/store/ui.ts` |
| 2 | Pi の `--max-speed`（黙って切り捨てる） | **3.0 m/s** | `raspi/setup/install_services.sh` → `--services` ＋ `--restart-io` |
| 3 | STM32 の `PARAM_MAX_SPEED` | **不明**（Pi 側が未実装で読み書きできない） | STM32 ファームウェア |

**1 と 2 は必ず一致させる。** ずれると「GUI では上限まで上げられるのに実機は出ない」という
分かりにくい状態になる。2 を上げても出ないなら **3 を疑う**。

舵角も同様で `--max-steer`（既定 0.524 rad ＝ 30°。路面舵角の実機上限。モータ機械角の
可動域 ±60° をリンク比 0.5 で割った値、2026-08-20 実測確定）と `PI_MAX_STEER_CAP` の対で
持っている。**据え切りを続けるとステア MD が過熱する**ので `temp[2]` を見ておくこと。

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

| 件 | 状態 |
|---|---|
| `steer_actual` が指令 0° で **−24.0°** に張り付く | 原点ずれの疑い。Phase 1 の遅延実測の前に潰す |
| GUI のカメラが前後とも 14fps | 未調査 |
| STM32 の時刻が **+3378 ppm 速い** | STM32 側と要相談（時刻同期で吸収はしている） |
| MD バスの CRC エラー率 **23〜25%**（3台とも） | STM32 側と要相談 |
| SLAM: `oval` でループが閉じない / RACE 段が壁に固着 | **SLAM は一旦棚上げ**。反射型（`de`）を本線にする方針（2026-08-14） |

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

## 11. 文書と実装のズレ（2026-08-16 時点の棚卸し）

**コードを読む前にここを見ておくと、文書に騙されずに済む。**
どれも「文書が古い」側で、コードが正。

| 場所 | 文書の記述 | 実際 |
|---|---|---|
| `architecture.md` §6.1 の図 | `perception_node` がある | **無い**。planner が `scan` を直接見る |
| `architecture.md` §7.3 | `safety_node` が `hb/*` を 300ms 監視して FAULT | **無い**。`hb/*` を購読しているのは logger_node（記録するだけ）。Watchdog の役は GPIO6 が代替 |
| `architecture.md` §6.3 | `grid/local` `pose` `path` トピック | 未実装。相当する情報は `auto/state` と `auto/map` |
| `architecture.md` §7 冒頭 | STM32 の UART タイムアウトが 200ms | §7.2 本文と `protocol.toml` は **100ms**（本文が正） |
| `architecture.md` §9.4 | PC → Pi のハートビートは 20Hz | 実装は **50Hz**（`CMD_PUB_HZ`） |
| `architecture.md` §11 | `.sfl` は Pi の SD に常時記録 | `install_services.sh` は `--log` を付けない。**既定では何も録らず**、GUI から開始する |
| `architecture.md` §10.2 | タブは4枚。「地図」タブは削除した | **5枚**（`地図生成` が復活している） |
| `gui/README.md` | デッドマンは Space 長押し | **Space はブレーキ。ARM は `Enter` トグル** |
| `gui/README.md` | 指令は 20Hz | **50Hz**（積分は rAF） |
| `gui/README.md` | `components/rc/SteerGauge` | **存在しない**（ラジコンタブに舵角計は無い） |
| `sim/README.md` | コースは `oval` / `slalom` / `room` / `chicane` | **`circuit.json` の1本だけ** |
| 各所の docstring | 地図は 400×400 ＝ 160KB | `raceline` の実際は **640×640**（5cm/20m 時代の記述が残っている） |

### 直したほうがよい実装上の小さな穴

| 場所 | 内容 |
|---|---|
| `tools/deploy.sh --restart` | **`surge-planning` が入っていない**（§2 の既知の穴） |
| `install_services.sh --remove` | **`surge-logclean.timer` を消し損ねる**。サービス本体が消えたタイマーが enable のまま残り、毎時 fail する |
| `install_services.sh` | 引数を1つしか見ないので **`--safe --with-logger` を併用できない**。`--with-logger` 単体では arm が有効のまま |
| `telemetry_node._serve_log_file` | `.mcap`（実測 87MB の実績）を**全部メモリに載せて**返す |
| `LinkTracker` | STM32 からの `LOG`(0x04) パケットを**受信数に数えるだけでどこにも出さない** |
| `logger_node` の記録対象 | `auto/*` を含まないので、**自律走行の判断根拠が `.mcap` に残らない** |

---

## 12. 学習ベースの自動運転を動かす（LiDAR E2E / カメラセグメンテーション / カメラE2E）

独立した3本の学習パイプラインがある。どれも「**Mac で学習 → ONNX 化 → `models/` に配置 →
GUI でモデル名を選ぶ**」という流れは共通だが、中身も検証手段も別物なので混同しないこと。

| | LiDAR E2E（`e2e_lidar`） | カメラセグメンテーション（`ftg_cam`/`cam_centerline`） | カメラE2E（`cam_e2e`） |
|---|---|---|---|
| 学習方法 | 強化学習（PPO、シム上で試行錯誤） | 教師あり学習（人がラベル付けした走行画像） | 教師あり学習（人の操舵・速度指令を直接ラベルにする模倣学習） |
| 学習コード | `ml_lidar/`（Mac 専用。Pi には運ばない使い方をする） | `ml_cam/`（同左） | `ml_cam_e2e/`（同左） |
| モデルの置き場 | `models/e2e_lidar/<名前>.onnx`（＋同名 `.json`） | `models/<名前>.onnx`（＋同名 `.json`） | `models/cam_e2e/<名前>.onnx`（＋同名 `.json`。**`.json` の無いモデルは読み込まれない**） |
| 推論コード | `raspi/auto/e2e_lidar.py`（planner本体。配線は`planning_node`がやる） | `raspi/nodes/cam_perception_node.py`（**独立プロセス**。`scan/cam`へ変換）＋ `raspi/auto/follow_the_gap_cam.py`/`cam_centerline.py` | `raspi/nodes/cam_e2e_node.py`（**独立プロセス**。IPMなどの幾何変換もLiDARも使わず、操舵と速度を直接推論）＋ `raspi/auto/cam_e2e.py` |
| シムで検証できるか | **できる**（`sim.run` は LiDAR を持つ） | **できない**（`sim.run` は `--no-camera` 固定でカメラを持たない） | **できない**（同左） |
| 実車で動かすのに要る追加操作 | 無し（`planning_node` が engage 中だけ推論する） | 無し（`surge-cam-perception` は常時 enable。`auto/ctrl` で `ftg_cam`/`cam_centerline` 選択中だけ推論する。下記 12.2） | 無し（`surge-cam-e2e` は常時 enable。`auto/ctrl` で `cam_e2e` 選択中だけ推論する。下記 12.3） |

いずれも `models/` はリポジトリの `.gitignore` 対象（機体・学習ごとに違う大容量ファイルのため）。
配布は `tools/deploy.sh` の rsync に任せる——コミットには乗らない。

### 12.1 LiDAR E2E（強化学習、`e2e_lidar`）

`disparity_extender.py`（`de`）のような**模倣学習ではない**。シム上で「コースに沿って
進めたら＋報酬・衝突したら－報酬」を頼りに、Stable-Baselines3 の PPO で方策を試行錯誤
させる。`de` を再現するのではなく**それを超えうる**代わりに、未知の点群パターンに
対する挙動は原理的に保証できない。以前は `e2e_lidar.py` が独立した `stop_dist`
（正面がこの距離を切ったらモデル出力を無視して無条件停止）を持っていたが、
STM32 の `auto_stop`（速度に応じて伸びる動的停止距離）の方が高性能なため
2026-09-12 に撤去した（`architecture.md`「走る」より「止まる」を先に決める）。

```bash
# 初回だけ（torch / gymnasium / stable-baselines3 / onnxruntime 等）
.venv/bin/pip install -r ml_lidar/requirements.txt

# 1. 学習（毎エピソード形状も道幅も変えたランダムコースで回す。数時間〜のオーダー）。
#    --max-speed を省略すると既定値(1.5 m/s)になる。変えたいときはここで明示的に
#    渡すこと（渡した値は --out/run_config.json に自動で記録され、手順3で自動的に
#    読まれる）。★最大舵角に対応する --max-steer は存在しない——config/vehicle.toml
#    の車両物理限界を常に使う（2026-08-28、GUI・訓練環境とも同じ方針に統一）
.venv/bin/python ml_lidar/train_rl.py --timesteps 2000000 --n-envs 8

# 学習曲線を見る（別ターミナル。ブラウザで http://localhost:6006 ）
.venv/bin/tensorboard --logdir ml_lidar/runs/ppo_e2e/tb

# 2.（任意）学習中の方策を複数パネルで観戦する。train_rl.py とは別プロセスで、
#    学習の SubprocVecEnv には一切触れない（学習を遅くしない）
.venv/bin/python ml_lidar/watch.py --panels 9

# 3. ONNX 化。--max-speed は省略可——手順1で書かれた ml_lidar/runs/ppo_e2e/run_config.json
#    から自動で読む（実際に使った値と出所は標準出力に表示される）。SB3 の checkpoint は
#    重みだけで行動レンジは持たないので、ここが学習時とズレると
#    models/e2e_lidar/<名前>.json に書く契約と実際の学習内容が食い違う——手順1より前に
#    学習した run_config.json の無いモデルは手で指定すること。最大舵角は常に
#    config/vehicle.toml の車両物理限界を使う（--max-steer という引数は無い）
.venv/bin/python ml_lidar/export_onnx_rl.py \
    --model ml_lidar/runs/ppo_e2e/best_model.zip \
    --out models/e2e_lidar/<好きな名前>.onnx

# 4. 実車に配る（models/ は通常の deploy.sh で運ばれる。追加操作は不要）
tools/deploy.sh --no-gui
```

ターミナル操作をまとめて避けたいなら `ml_lidar/app.py`（または `launcher.command`
をダブルクリックして「ml_lidar」を起動）が上記1〜3をボタンで操作できる薄い Tkinter GUI（`ml_cam/app.py` と対称、
2026-08-28追加）。学習前に「run名」を1つ決めるだけで、`ml_lidar/runs/<run名>` への
学習出力と `models/e2e_lidar/<run名>.onnx` へのエクスポート先が自動的に紐づく
（`v1`・`v2`…と自動採番も提案する）。学習run一覧タブから TensorBoard・観戦(`watch.py`)・
エクスポートをそれぞれ起動でき、**学習を止めずに並行して動かせる**（学習は数時間かかる
裏でTensorBoardを眺めたり、別runをエクスポートしたりできる設計）。推論・学習のロジックは
持たないので、中身のスクリプトを直せばこちら側は何も変えなくてよい。

既存のrun名で学習開始すると「続きから再開／上書きして新規／キャンセル」の3択を聞く
（`train_rl.py --resume-from`、2026-08-28追加。`best_model.zip`から`PPO.load()`する）。
「実行中のジョブ」欄で学習を選んで「選択を停止」を押すと、数時間ぶんの進捗を誤って
失わないよう確認ダイアログが出る（TensorBoard・観戦・エクスポートは確認無しで即停止）。
観戦の窓数（`watch.py --panels`）もrun一覧タブから選べる。

`train_rl.py` は既定で `circuit`/`fuji`（学習には使わない既知コース）を定期的に評価し、
更新されるたびに `ml_lidar/runs/ppo_e2e/best_model.zip` を上書きする。評価スコアが
`--early-stop-patience`（既定10）回連続で更新されなければ `--timesteps` 未達でも
学習を打ち切る（`--early-stop-patience 0` で無効化）。**`watch.py` はこの
`best_model.zip` を数秒おきに読み直すだけなので、学習を止めずに並行して眺められる。**

GUI での使い方:

1. 設定タブ →「E2E LiDARモデル」ドロップダウンで `models/e2e_lidar/` の一覧から選ぶ
   （増えたモデルが出ないときは「更新」ボタン）
2. 自動運転タブ → モードで「E2E LiDAR」を選択
3. パラメータ `max_speed` はモデル出力をこの値でクランプするだけの安全側の上限——
   学習時（`train_rl.py` の `--max-speed`）より大きくしても出力レンジがそこまで
   届かないので意味が無い。**最大舵角は GUIパラメータではない**——
   `config/vehicle.toml` の車両物理限界を常に使う
   （2026-08-28、自動運転planner全体の方針。他のplanner（`ftg`・`de`等）も同様）
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
以前は「`planning_node` が今どのモードを選んでいるかを一切知らない独立プロセス」で
前方カメラのフレームが来る限り無条件に CNN 推論を回し続けるため既定で無効にしていたが、
`cam_perception_node.py` 自身が `auto/ctrl`（`AutoCtrl.mode`。telemetry_node が GUI の
選択を送り続けるトピック）を見て**`ftg_cam` が選ばれている間だけ推論する**よう変更した
（2026-09-03、`cam_track_node.py` の待機コスト設計に揃えた）。手で起動する必要はない:

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
この方式の狙い（2026-09-04導入、2026-10-06に操舵＋速度・カメラのみへ作り直し）。
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
- **速度モードで運転する。** トルクモードの記録は `target_speed` が 0 のままで速度の手本に
  ならず、抽出で全部落ちる（落ちた理由と枚数は①のログに出る）
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
| 最高速度 | 0.30 m/s | モデルが出した速度の頭打ち。**初めてのモデルはここを低くして試す** |
| 速度の倍率 | 1.0 | モデルが出した速度に掛ける |
| 最低速度 | 0.10 m/s | これ未満でもここまでは出す（止まった絵に速度0を出すモデルが発進できるように） |
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
  （2026-10-06 以前は、学習が RGB・推論が BGR、縮小方式も別という食い違いがあった）
- **モデルの出力は2個**（`steer_norm` -1..1、`speed_norm` 0..1。前進のみ）。物理量へ戻す基準
  （`max_steer`・`speed_ref`）は同梱 `.json` に書く。`speed_ref` は学習に使った手本の最高速度
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
| `raspi/nav/` | SLAM・占有格子・経路（**一旦棚上げ中。消さない**）。`ipm.py`（カメラ逆投影）は `ftg_cam`/`cam_centerline` が使用（`cam_e2e` は経由しない） | surge-planning 再起動 |
| `raspi/proto/` | UART 定義（**STM32 と共有する唯一の定義**） | 再生成 ＋ `--restart-io` |
| `raspi/bus/` `raspi/msgs/` | ZeroMQ ラッパ・共有メモリ・メッセージ型 | 関係ノード全部 |
| `raspi/io/` | `SerialLink` / GPIO（**`import serial` はここ1箇所だけ**） | `--restart-io` |
| `raspi/rec/` | `.sfl` / MCAP の書き出し | io / logger |
| `raspi/tools/` | 単発の検査・変換ツール | — |
| `raspi/tests/` | `unittest.TestCase` で書き、実行は pytest（`tools/check.sh`） | — |
| `gui/` | React + TypeScript | rsync だけ |
| `sim/` | Mac 専用シミュレータ（**Pi には運ぶが使わない**）。カメラは持たない | — |
| `ml_cam/` | カメラセグメンテーションの学習パイプライン（Mac 専用。§12.2） | Pi には無関係 |
| `ml_cam_e2e/` | カメラE2E（模倣学習）の学習パイプライン（Mac 専用。§12.3） | Pi には無関係 |
| `ml_lidar/` | LiDAR E2E 強化学習パイプライン（Mac 専用。§12.1） | Pi には無関係 |
| `models/` | 学習済み ONNX の置き場（`models/*.onnx`=カメラセグメンテーション用、`models/cam_e2e/*.onnx`=カメラE2E用、`models/e2e_lidar/*.onnx`=LiDAR E2E用）。`.gitignore` 対象 | `tools/deploy.sh` で運ぶだけ。再起動は不要 |
| `config/` | 車両諸元・自動運転パラメータ | 読んでいるノード |
| `tools/` | `deploy.sh` / `record.sh` | — |
