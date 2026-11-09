# SURGE Mark.2 開発ガイド — 変更の反映とトラブルシューティング

「コードを直した。どこに何をすれば実機に反映されるのか」と
「動かない。何から疑えばいいのか」だけを書いた実務文書。
**設計の理由は [`system_overview.md`](system_overview.md)、現在の状態は `PROGRESS.md` が正。**
シミュレータとベンチは [`sim/README.md`](../sim/README.md)、GUI は [`gui/README.md`](../gui/README.md)、
校正は [`calibration.md`](calibration.md) が正（ここには書かない）。

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

**実車を出す前に、Mac 上で潰せるものは全部潰す。**

| 舞台 | 何が本物か | 何を開発できるか | 起動 |
|---|---|---|---|
| **シミュレータ** | STM32 より上流のコード全部（プロトコル・換算・安全ゲート・バス・WS・GUI） | **自律走行・点群処理・操縦感**。指令に車が反応する | `python -m sim.run` |
| **モック** (`bus_demo`) | バス・WS・GUI の配管だけ（UART 層を通らない） | **GUI の見た目・異常表示**。`--faults` で E-Stop や過熱を演出できる | `python -m raspi.tools.bus_demo` |
| **ログ再生** (`replay_node`) | センサデータが**本物**（実車で録った `.sfl`） | 実データでの知覚・解析。ただし**指令には反応しない** | `python -m raspi.nodes.replay_node <file> --bus` |
| （実車） | 全部 | 最終確認・遅延やノイズの実測 | Pi 上の systemd |

選び方:

- **GUI の色・配置・異常表示を直した** → モック（`--faults`）
- **planner（`raspi/auto/`）を書いた・パラメータを詰めたい** → シミュレータ、数値で比べるなら `sim.bench`
- **SLAM（`slam2d/`）を触った** → `sim.slam_bench`（真値比較・仮想時間・誤差注入）
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
| `raspi/nodes/telemetry_node.py` `camera_node.py` `cam_perception_node.py` `cam_e2e_node.py` `line_perception_node.py` | `deploy.sh --restart`（surge-telemetry / camera / cam-perception / cam-e2e / line-perception / planning をまとめて再起動） | しない |
| **`raspi/auto/` `raspi/nav/` `planning_node.py`** | `deploy.sh --restart`（`PLANNERS` はプロセス起動時に固定されるので必須。地図作成中に再起動すると未確定の地図は失われる） | しない |
| `config/vehicle.toml` | **先に `python3 config/generate.py`**（GUI 用 TS を再生成）してから rsync。読んでいるノードを再起動（planning は `--restart`、`[control]` は io が接続時に STM32 へ送るので `--restart-io`） | io なら**する** |
| `config/auto.json` `auto_presets.json` `drive_settings.json` `camera.json` `cam_model.json` `cam_e2e_model.json` `e2e_lidar_model.json` `odometer.json`・`saved_maps/` | **機械ごとの状態。`deploy.sh` は運ばない**（動いている機械で書き、起動時に戻す。Mac＝シム、Pi＝実機で別々）。地図は GUI の DL/UL で運ぶ（経路の設定も同梱） | しない |
| `raspi/nodes/io_node.py` `raspi/io/` `raspi/proto/` | **`deploy.sh --restart-io`** | **★★ する** |
| `raspi/msgs/types.py` | **先に `python3 config/gen_msgs.py`**（`gui/src/generated/msgs.ts` を再生成）→ `deploy.sh --restart --restart-io` | **★★ する** |
| `raspi/setup/install_services.sh`（unit に書く引数。`ARM=` 行の `--max-speed` 等） | **`--services` ＋ `--restart-io`** | **★★ する** |
| `raspi/proto/protocol.toml` | **先に `python3 raspi/proto/generate.py`**、その後 `--restart-io`。**ファームは生成ヘッダを取り込まず `ras_link.[ch]` に手で写している**ので、ファームも直して `protocol_version` を上げ、同時に書き込む（[`raspi/proto/README.md`](../raspi/proto/README.md)） | **★★ する** |
| `models/*.onnx`（カメラセグメンテーション）`models/cam_e2e/*.onnx`（カメラE2E）`models/e2e_lidar/*.onnx`（LiDAR E2E） | **通常の `tools/deploy.sh`（rsync のみ）。** 再起動は不要——各ノード／planner の `reload_if_changed()` が GUI でモデル名を選んだ瞬間に読み直す | しない |
| `ml_cam/` `ml_cam_e2e/` `ml_lidar/` `ml_common/`（学習パイプラインと学習 GUI の共通部品） | **Mac 側だけで完結。** rsync では運ばれるが `raspi/` 側は読まない | しない |

> **`--services` は unit ファイルを書き換えるだけ。** 走っている `io_node` は古い引数のまま
> 動き続けるので、反映には `--restart-io` が要る（`install_services.sh` も最後にそう警告する）。

### ★★ `--restart-io` は E-Stop をラッチさせる

`io_node` は GPIO6 に 100Hz のハートビートを出している（安全第1層）。
**止めた瞬間に STM32 が E-Stop をラッチし、車両のボタン2を物理的に押すまで戻らない。**

- **仕様どおりの動作であり、故障ではない**
- **車体に手が届く状態でだけ使う**（リモートで叩くと、現地に行くまで車が死ぬ）
- `Restart=on-failure` で io_node が再起動しても**ラッチは解除されない**。人間の操作でしか戻らない

---

## 3. コマンド早見表

### Mac 側

補助 GUI（シム・コースエディタ・校正・SLAM 再処理・走行サマリ・動画の書き出し・ml 系）は
**`launcher.command`（surge_mk2 直下）**から起動・停止する（`tools/launcher.py` の `APPS`。閉じると子も終了する）。

```bash
cd ~/GitHub/AutonomousCar_RasPi/surge_mk2

# ── 実行（オプションは sim/README.md）──
.venv/bin/python -m sim.run                       # シミュレータ一式（Ctrl-C で全部落ちる）
.venv/bin/python -m sim.bench --course normal      # planner の数値評価

# ── テスト・生成 ──
./tools/check.sh                                  # 生成物一致・文書版番号・pytest・tsc を1コマンドで
./tools/check.sh --fast                           # 生成物の --check だけ（数秒。コミット前向き）
.venv/bin/python -m pytest raspi/tests -q         # テストだけ単体で回すとき
python3 raspi/proto/generate.py                   # protocol.toml を直したら必ず
python3 config/generate.py                        # vehicle.toml を直したら必ず
python3 config/gen_msgs.py                        # raspi/msgs/types.py を直したら必ず

# ── GUI ──
cd gui && npm run dev                             # http://localhost:5173（自動リロード）
SURGE_HOST=surge-mk2.local npm run dev            # 編集しながら実機に繋ぐ
npm run build                                     # -> gui/dist（Pi に node は不要）

# ── Pi へ ──
tools/deploy.sh                                   # GUI ビルド + rsync（既定・いちばん安全）
tools/deploy.sh --no-gui                          # Python だけ直したとき
tools/deploy.sh --no-prune                        # Mac に無くなった .py を Pi から消さない（既定は消す）
tools/deploy.sh --test                            # 反映後に Pi 上でテスト
tools/deploy.sh --restart                         # telemetry / camera / cam-perception / cam-e2e / line-perception / planning を再起動
tools/deploy.sh --services --restart-io           # unit を入れ直して io を再起動（★E-Stop）
tools/record.sh --duration 60                     # SD に書かずに MCAP を PC へ（--image-hz N で画像の頻度）

# ── ログ ──
.venv/bin/python -m raspi.tools.logcat logs/x.sfl          # .sfl の要約
.venv/bin/python -m raspi.tools.sfl2mcap logs/x.sfl        # .sfl → .mcap
.venv/bin/python -m raspi.tools.slam_replay x.mcap          # slam2d に通し直して自己位置を足す（x_slam.mcap）
.venv/bin/python -m raspi.tools.grip_replay x.mcap          # 走行中の学習に流し直して安全装置の判定を見る（§4.3）
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
sudo bash ~/surge_mk2/raspi/setup/install_services.sh --safe   # arm 封印で入れ直す（反映は --restart-io）
```

surge-io は drop-in `surge-io.service.d/10-priority.conf` で `Nice=-10` にしてある。

**接続先は常に `surge-mk2.local`**（mDNS）。直結・Wi-Fi の生きている方を自動で選ぶ。
GUI は `http://surge-mk2.local:8000/`。**Wi-Fi だけで繋いだ Mac では名前が引けなくなることがある**（§8.1）。
そのときは launcher の「実機 GUI を開く」か `.venv/bin/python -m tools.find_pi --open` で **IP で開く**。
`tools/deploy.sh` は名前で繋がらないと自動で IP に切り替える。

---

## 4. Mac だけで回す開発ループ

### 4.1 シミュレータとベンチ

起動・キー操作・コースの作り方・ベンチ（`sim.bench`・`sim.slam_bench`・`sim.park_bench`）のオプションと
結果の読み方は [`sim/README.md`](../sim/README.md) が正。

- **engage しなくても判断だけは流れている**ので、手動で走らせながら planner の狙いを見られる
- **見た目では「なんとなく速い」しか分からない。** planner の採否は `sim.bench` の数字で決める
- ★ベンチは**実時間**で回る。並列は 2 本まで

### 4.2 GUI だけを直す

サーバ（`telemetry_node`）とデータ源（`bus_demo` か `sim`）を上げたまま `npm run dev` を使う。
Vite の自動リロードが効くので rsync も再起動も要らない（手順は [`gui/README.md`](../gui/README.md)）。

### 4.3 走行中の学習を確かめる

`slam2d_route` の `grip_learn`（`raspi/nav/grip.py`。横の上限を `GripLearner` が周回のタイムで、前後の上限を
`LongLearner` が包絡で、走行中に決める）の検証は3段。

1. **シム**: `sim.bench` の `--mu` / `--surface` / `--mu-at` で真値の路面だけ変えて走らせる
   （[`sim/README.md`「路面を変えて走らせる」](../sim/README.md#路面を変えて走らせる走行中の学習の検証)）。
   ★シムの既定の雑音は実機よりずっと小さいので、安全装置を変えたら `--real-noise` 有り・無しの両方と、下の流し直しで確かめる
2. **実機の記録を流し直す**（下）
3. **実機で段階的に試す**（下）

#### 実機の記録を学習に流し直す（`raspi/tools/grip_replay.py`）

安全装置（リアの滑り・線から外れた・横偏差の増え）は車両状態（IMU の横加速度を含む）と記録の横偏差だけで決まるので、
記録をそのまま流せば実機で何が働いたかが手元で再現できる。**滑っていない記録では 0 回が合格。**

```bash
.venv/bin/python -m raspi.tools.grip_replay x.mcap                    # 学習を入れて走った記録
.venv/bin/python -m raspi.tools.grip_replay x.mcap --pin 4.9          # 学習なしの記録: 横の上限を固定して流す
.venv/bin/python -m raspi.tools.grip_replay x.mcap --set slide_s=0.1  # GripConfig を変えて比べる
.venv/bin/python -m raspi.tools.grip_replay x.mcap --end 100 --export x.npz   # テスト用の小さな記録に抜く
```

- 流し直しは開ループ（安全装置で目標速度を落としても、記録の車速は変わらない）。タイムで決める段の採否は
  `--plan-lap` で計画のラップを与えたときだけ流れる
- 回帰テスト（`raspi/tests/test_grip.py` の `TestRealRecords`）は `raspi/tests/data/grip_real_*.npz`（実機の記録の抜粋）を流す。
  実機で誤って働いた記録が増えたら、ここへ足す

#### 走行中の学習を実機で初めて試すとき

シムで決めたしきい値・上げ幅をそのまま使わない。安全側から順に:

1. **記録だけ取る**（`grip_learn` = 0 のまま普段どおり周回）。記録を `grip_replay --pin <走っていた横加速度>` に流し、
   安全装置が 0 回であることを見る
2. **下げるだけ**: `grip_learn` 1・`grip_down_only` 1。出発点は同定値の 0.85 倍（横）、加速・減速は校正の値
   （`[limits]`）で、そこから上げない。前兆が誤って出ないか（reason の「前兆 n回」が何もない所で増えないか、
   減速の値が下がり続けないか）を見る
3. **上げる**: `grip_down_only` 0・`grip_max` を今走れている値＋0.5m/s² 程度・`v_max` は普段より下げる。
   壁の外側に緩衝材を置く。出してよい上限は `grip_max` で決める（上げ幅は `GripConfig.time_step`・`LongConfig.step`）。
   加速・減速も同時に学習される（切り離す設定は無い）
4. 値が落ち着いたら `grip_max` を上げる。本番は練習で**学習を終えた**（reason が「収束」）地図をそのまま使い、
   学習を終えた値で 1 周目から走る。床が変わったら `saved_maps/<地図名>.grip.json` を消して学習し直す

### 4.4 校正・ファームの制御

- **車両の校正**（エンコーダ・ステア・遅延・車速制御・TC/ABS）、**カメラの校正**、**車速の推定（KF）の調整**は
  [`calibration.md`](calibration.md) が正。値は `config/vehicle.toml`、反映は `tools/deploy.sh --restart-io`
  （`[control]` は接続のたびに STM32 へ送られる。`--restart` では入らない）
- **ファームの制御を変えるときは、変える前後で `.venv/bin/python -m tools.fwsim.bench` を回して比べる**（ホストでコンパイルした
  ファーム＋車のモデル。`--ref <コミット>` で相手を指定。物差しは `tools/fwsim/scenarios.py` の冒頭）
- **TV は同定も最適化もしない**（`tv_*` は `[control]` を手で決める。有効/無効は `tv_enable`）。旋回しながら加速する走行を TV の
  ON/OFF（「設定 → 走行アシスト」）で比べ、診断タブの「スリップ率」で内輪と外輪がそろう値を探す

---

## 5. 実機へ反映する

```bash
tools/deploy.sh --test                    # ふだんはこれ（GUI ビルド + rsync + Pi 上テスト）
```

`deploy.sh` がやること:

1. `surge-mk2` への疎通確認（名前で繋がらなければ IP に切り替える。だめなら直し方を出す）
2. `npm run build`（`--no-gui` で省略）
3. `rsync`（**`.venv` `node_modules` `logs/` `*.mcap` `*.sfl` `docs/setup_credentials.md`・機械ごとの状態（§2）・`ml_*/runs/` は運ばない**）。
   全体には `--delete` を掛けず、Mac で消した `.py` だけ Pi からも消す（`--no-prune` で止める）
4. `gui/dist` だけは `--delete` 付き（ビルドごとにハッシュが変わり、古い `index-*.js` が溜まるため）
5. 指定に応じて test / services / restart
6. 最後に各サービスの `is-active` を表示

> **`.venv` を絶対に運ばない。** Pi の venv は `--system-site-packages` で作られていて
> 中身が Mac と別物（gpiozero / lgpio / picamera2 は OS 同梱を使う）。上書きすると壊れる。

GUI が更新されたかは、ブラウザの読み込んでいる `index-*.js` のハッシュがローカルビルドと一致するかで確認できる。

---

## 6. テスト

```bash
./tools/check.sh                                                  # 生成物一致・文書版番号・pytest・tsc をまとめて
.venv/bin/python -m pytest raspi/tests -q                        # テストだけ（Mac）
tools/deploy.sh --test                                           # Pi 上でも回す
```

ハードウェアもシリアルもカメラも要らない（GPIO の無い Mac では一部 skip。`ipc://` はテンポラリに閉じ込めてある）。
主なもの（全一覧は `ls raspi/tests`）:

| ファイル | 何を守っているか |
|---|---|
| `test_proto.py` | CRC のチェック値、**TELEMETRY のバイト位置**、生成物と `protocol.toml` の一致 |
| `test_msgs.py` | **スケールと射影**（単位を間違えても値は出るので「10倍おかしい」としか症状が出ない） |
| `test_zbus.py` | ワイヤ形式と PUB/SUB のポリシー（CONFLATE の罠2つを固定） |
| `test_shm_ring.py` | seqlock。**「壊れたときに壊れたと分かる」**方が要件なので意図的に上書きを起こす |
| `test_gpio.py` | E-Stop ハートビート。**止まるべきときに止まること**を厚く |
| `test_link_tracker.py` | 健全性判定と**ラッチ**（人間が触るまで戻らない） |
| `test_cmd_path.py` | GUI → WS → バス → io_node → UART の指令経路と ARM の3条件 |
| `test_framelog.py` `test_io_node_log.py` `test_mcap.py` | `.sfl`・MCAP の往復と、壊れた・尻切れファイルへの頑健性 |
| `test_replay_node.py` | 再生が**実機と同じ結果**になること |
| `test_auto.py` `test_slam2d_raceline.py` | planner の安全条件と段の遷移。**「走る」より「止まる」を厚く** |
| `test_nav.py` | SLAM の精度を**数値で縛る** |
| `test_native.py` | C 版と Python 版の**ビット一致**（SLAM 側は `slam2d/tests/test_native.py`） |

- テスト本体は **`unittest`** で書き、実行は **`pytest`**（CI と `tools/check.sh` に揃えてある）
- **実時間で回るコード（`io_node` / `framelog` / `shm_ring`）は stdlib だけで書く**。Mac でも Pi でも同じテストが通る
- **プロトコルを直したら `generate.py` を回してからテストする**
- **経路・SLAM・駐車の内側の計算は C 版を持つ**（`raspi/nav/_native.c`・`slam2d/core/_native.c`）。初回の import で `cc` が組んで
  `~/.cache/surge_mk2/` に置く（組めなければ Python 版で回る＝同じ結果で遅い。`SURGE_NO_NATIVE=1` で C を使わない）。
  C 版がある関数（各 `_native.c` の冒頭と Python 側の呼び出しで分かる）の式を変えたら C も直し、`test_native.py` で一致を確かめる

---

## 7. 触る前の安全ルール

**自律走行で最初に覚えるのは走らせ方ではなく止め方。**

| 止め方 | 効く範囲 | 復帰 |
|---|---|---|
| GUI のデッドマンを離す | 指令の送信が止まる → 150ms で DISARM | 握り直すだけ |
| GUI の `Esc` / E-STOP ボタン | 即 DISARM | 画面から |
| **車両のボタン2**（E-Stop 解除） | ラッチした E-Stop の解除 | **人間が物理的に押す** |
| 駆動電源を切る | モータのみ。Pi と STM32 は生きたまま | 電源投入 |

モータが回る条件（`--allow-arm`・`cmd` が 150ms 以内に届き続ける・人間が ARM を保持）と再 ARM の規則は
[`system_overview.md` §8.3](system_overview.md#83-モータが回る条件は3つ)。E-Stop・駆動電源ラッチ・STM32 の再起動・COMMAND 途絶の後は
**ARM を一度外してから入れ直す**まで通らない（理由は GUI の ARM ボタンの横と `diag/link` の `arm_inhibit_reason` に出る）。
**engage（自律走行の開始）は ARM ではない**ので、電源を入れただけで走り出す経路は無い。

実機で `--allow-arm` のまま作業するときは:

- **車輪を浮かせる**（台上）か、周囲に人・物が無いことを確認してから電源を入れる
- `surge-io` を止める操作は E-Stop ラッチを伴う。**車体に手が届く場所でだけ叩く**

---

## 8. トラブルシューティング

**症状 → 疑う順 → 対処。** 今も起こりうるものだけを書く。

### 8.1 Pi に繋がらない・遅い

| 症状 | 原因 | 対処 |
|---|---|---|
| `ssh surge-mk2` が遅い（6ms のはずが 40ms） | **直結が生きているのに Wi-Fi 経由で繋がっている** | `ping -b en18 169.254.55.2` のように **IF を明示**して直結の生死を見る（素の `ping` は `169.254/16` のルートが複数 IF に載っていて誤診する）。**アダプタの IF 名は挿し直すと変わる** |
| 直結を挿し直したら mDNS が Wi-Fi しか返さない | avahi が再広告していない | `ssh surge-mk2 'sudo systemctl restart avahi-daemon'` |
| ssh が IPv6 に行ってしまう | avahi は IPv6 リンクローカルも広告し、ssh は IPv6 を優先する | `~/.ssh/config` に **`AddressFamily inet`**（設定済み）。curl は `-4` |
| Wi-Fi だけの Mac で GUI が「読み込み中」／INIT・切断のまま。`ssh surge-mk2` も名前解決で失敗（有線を挿すと直る） | **その Mac が Wi-Fi 上の mDNS（224.0.0.251）を受け取れない。** Pi の IP が変わると mDNSResponder が古いまま固まり、問い合わせ自体を出さなくなる | **IP で開く**: launcher「実機 GUI を開く」／`python -m tools.find_pi --open`（mDNSResponder を通さず Pi に直接問い合わせる）。固まりを解くのは Mac の Wi-Fi の入れ直しか `sudo killall -HUP mDNSResponder` |
| どうしても IP が要る | — | `.venv/bin/python -m tools.find_pi`。だめなら `arp -an \| grep -i 2c:cf:67`（Raspberry Pi Ltd の MAC prefix） |
| Wi-Fi 経由で、数分おきに数秒だけ遅延が 20〜125ms に跳ねる（操縦のデッドマンは 150ms） | **NetworkManager が wpa_supplicant に定期スキャンをさせている**（`bgscan`。電波が弱いと 30 秒ごと、1回 約3.6秒） | **接続先の AP（BSSID）を固定する**: `sudo nmcli con mod <接続名> 802-11-wireless.bssid <BSSID>` → `sudo nmcli con up <接続名>`。確認は `sudo wpa_cli -i wlan0 get_network 0 bgscan`（`FAIL` なら無し）と `iw event -t`。**固定すると他の AP へ移らない**。戻すのは BSSID に `""`。接続ごとの設定なので、会場の AP でもやり直す |

### 8.2 GUI が古い・表示がコードと合わない

| 症状 | 原因 | 対処 |
|---|---|---|
| 直したはずの画面が変わらない | **古いプロセスが配っている** | `lsof -nP -iTCP:8000 -sTCP:LISTEN` → `ps -o lstart= -p <PID>` で**起動時刻を見る**。古ければ kill |
| 実機の GUI が古い | `gui/dist` が更新されていない | `tools/deploy.sh`（`--no-gui` を付けていないか確認）。配信中の `index-*.js` のハッシュを見る |
| 偽のデータが出る | `bus_demo` が生き残って `io` endpoint に bind している | `sim.run --kill-stale`、または `pgrep -af bus_demo` |

### 8.3 Mac でシミュレータが起動しない

| 症状 | 原因 | 対処 |
|---|---|---|
| `ImportError: incompatible architecture (have 'arm64', need 'x86_64')` | **ターミナルが Rosetta で動いている**（python.org の Python は親のアーキを継承する） | `arch` で確認（`i386` なら Rosetta）。恒久対応は Terminal.app の「Rosetta を使用して開く」を外す。`sim.run` は自動で arm64 で立て直す |
| `OSError: [Errno 48] address already in use ('0.0.0.0', 8000)` | 古い `telemetry_node` が残っている | §8.2 と同じ手順。`sim.run --kill-stale` でも落とせる |

### 8.4 車が動かない

**上から順に潰す。**

1. **ARM しているか。** GUI で ARM（`Enter` / パッドの ×）を保持しているか。ラッチの後は一度外して入れ直す（§7）
2. **`--allow-arm` が付いているか。** `systemctl show surge-io -p ExecStart --value`
3. **駆動電源がラッチしていないか。** 過電流でハード遮断されると**電源を入れ直すまで復帰しない**（`drive_power_locked`。仕様）。
   GUI に「駆動電源ラッチ中」と出る
4. **E-Stop がラッチしていないか。** `surge-io` を止めた／Pi が落ちた後は必ずこれ。**車両のボタン2**を押す
5. **`LIMITS` と `[control]` が届いているか・版数が一致しているか。** どちらかが欠けると io_node は ARM を通さない（GUI のリンク表示）
6. **`cmd` が届いているか。** Wi-Fi が切れると 150ms で DISARM に落ちる

### 8.5 速度が上がらない

**上限は `LIMITS`（STM32 が報告する車両の物理上限）が正。** io_node も GUI（`effectiveRange`）も、
`LIMITS` を受信済みならそれを使う。`--max-speed`・`--max-steer`（`vehicle.toml` の `max_steer` で頭打ち）は
`LIMITS` 受信前の既定。

| 症状 | 疑う |
|---|---|
| GUI で上限まで上げても実機が出ない | `LIMITS` 未受信（GUI のリンク表示）、または `[control]` が STM32 に入っていない |
| 指令の速度が黙って切られる | io_node の `_send_command` で `LIMITS`/`--max-speed` にクランプ。planner の `max_speed` も見る |
| 出ているが遅い | トルク上限・TC/ABS（診断タブの「TC・ABS トルク上限」）、電池の垂れ |

**据え切りを続けるとステア MD が過熱する**ので `temp[2]` を見ておくこと。

### 8.6 カメラが映らない

| 症状 | 原因 | 対処 |
|---|---|---|
| **GUI のカメラだけ／ロガーの画像だけ入らない**（点群やメータは正常） | `RemoveIPC=yes` により、**SSH を切った瞬間に `/dev/shm/surge_cam*` が消える**。新しく attach するプロセスだけが失敗する | `install_services.sh` が `RemoveIPC=no` を入れる。入っているか確認: `cat /etc/systemd/logind.conf.d/10-surge-removeipc.conf` |
| 前後とも 14fps しか出ない | **未調査**（camera_node 自体は 30fps 出ている。配信側で落ちている疑い） | — |

### 8.7 ログに何も出ない

**`python -u` が付いていない。** リダイレクト先が tty でないと Python は 8KB 単位でしか書かないので、
「起動したのにログが空」になる。`run_stack.sh` と `install_services.sh` は `-u` 付きで起動している。

### 8.8 `pkill -f raspi.nodes` で SSH ごと落ちる

**リモートシェルの cmdline にパターンと同じ文字列が入っている**ため、自分自身にマッチする
（`[r]aspi` のブラケット回避も効かない）。→ **スクリプト経由にする**（`run_stack.sh` なら cmdline は `bash run_stack.sh` だけ）。

### 8.9 記録まわり

| 症状 | 原因 | 対処 |
|---|---|---|
| `.mcap` が開けない | **MCAP は `finish()` を呼んで初めて索引が書かれる。** 電源断や ssh 切断で尻切れになる | `python -m raspi.tools.mcap_repair <file> --inplace` |
| SD が埋まる | `.sfl` 2.5MB/分 ＋ `.mcap` 10.3MB/分 | `surge-logclean.timer` が毎時「7日超」と「合計の上限超過分」を消す。**`.mcap` は既定で SD に書かない**（GUI か `tools/record.sh` で PC に流す）。`io_node`/`logger_node` も30秒ごとに空き容量を見て古い順に消す（`raspi/rec/logclean.py`。記録中のファイルは保護） |
| 書き込み中に SD が満杯（ENOSPC） | 掃除が間に合わなかった | そのファイルへの書き込みだけ止まる（`broken`/`last_error`）。**走行は続く** |
| `.sfl` が途中で切れている | 追記のみ・索引なし | **前半は必ず読める。** これが `.sfl` を残している理由 |

記録まわりの約束:

- `.sfl` は io_node の専用スレッド（`sfl-writer`）が書き、数秒ごとに fsync する。SD が詰まってもメインループは止まらない
  （キューが溢れたら古い順に捨て、終了時の統計に `dropped` として出す）
- STM32 だけが再起動したら（`t_us` の後退、または要求していない VERSION の到着で検出）、io_node は時刻同期をやり直し、
  `[control]` と GUI で設定した値（TC/TV/ABS 等）を送り直し、ARM を入れ直すまで通さない

### 8.10 点群がおかしい

| 症状 | 見るところ |
|---|---|
| 左右が反転している | `ScanAssembler` が `車両角 = (360 − センサ角) % 360` で戻している（LD06 が裏向き実装のため）。**シミュレータはわざと反転した状態で出している**ので、ここが壊れると両方で崩れる |
| 実在しない壁が円状に出る | 圧縮フォーマットの `255` は「5.10m **以上**」であって実測点ではない |
| 欠測方向へ舵を切る | **`sector_seen == False` は「障害物なし」ではない。** 欠測は距離 0（侵入禁止）として扱う |
| 古い点群で舵を切る（見た目は正常に動く） | **ZMQ の `CONFLATE` は multipart 非対応**、かつ**ソケット単位**。1フレーム送信・トピック1本につきソケット1本にしてある（`test_zbus.py` が固定） |

### 8.11 Pi が勝手に再起動する・ハングする

原因が特定できていない再起動が起きたことがある（`vcgencmd get_throttled` は `0x0`、温度も正常だった）。
永続ジャーナルは有効なので、**再発したら必ずログを見る。**

```bash
sudo journalctl --list-boots
sudo journalctl -b -1 -n 50        # 前回起動の最後
```

「起動 → 数十秒 → ハング」の繰り返しは**電源不足**。**5V/5A（27W）USB-C PD** を使う。緑 LED が点いていても OS はハングし得る。

### 8.12 未解決として残っているもの

`PROGRESS.md` 冒頭の「原因が分かっていないこと」と [`system_overview.md` §14](system_overview.md#14-現在地とロードマップ)（ここに複製しない）。

---

## 9. ログの取り方・見方

### 2種類あり、目的が違う

| | `.sfl` | `.mcap` |
|---|---|---|
| 書く人 | `io_node`（実時間ループの中） | `logger_node`（別プロセス） |
| 中身 | **UART を流れた生バイト**（送受信とも） | 解釈済みトピック **＋ カメラ画像** |
| 依存 | stdlib のみ | `mcap` |
| 置き場 | **Pi の SD**（Wi-Fi が切れても止まらない） | **PC**（SD には書かない） |
| 量 | 2.5MB/分 | 10.3MB/分（画像 5Hz×2台） |
| 尻切れ耐性 | **強い**（追記のみ） | 弱い（索引が要る）→ `mcap_repair` |
| 用途 | 異常の証拠・確定的な再生 | 解析・Foxglove で目視 |

### 録り方

- **ふだんは GUI の「ログ」タブ**（開始/停止・一覧・ダウンロード・削除）
- GUI を開けない・長時間・画像を増やしたいなら `tools/record.sh`（ssh パイプで PC に直接落とす。`--image-hz`）
- `.sfl` は後から `python -m raspi.tools.sfl2mcap` で MCAP に変換できる（`replay_node` + `BusBridge` をそのまま通すので、
  実機・再生・変換で同じ解釈コードが動く）

### 見方

```bash
python -m raspi.tools.logcat logs/x.sfl        # 頻度・期待Hzとの比較・最大間隔・リンク統計
python -m raspi.nodes.replay_node logs/x.sfl --verify    # ログが解析に足りるかの検査
python -m raspi.nodes.replay_node logs/x.sfl --bus --loop  # バスに流し直す（Mac 上で）
```

- `.mcap` は **Foxglove Studio でそのまま開ける**（自作 GUI＝ライブ用、Foxglove＝オフライン解析用）
- **`.sfl` の再生は GUI からはできない。** 再生は `surge-io` と同じ ZeroMQ エンドポイントを取り合うので、
  **Mac 上で `replay_node --bus` + `telemetry_node` を直接叩く**
- **動画にする**: launcher の「動画の書き出し」（`tools/mcap_video_gui.py`、CLI は `python -m tools.mcap_video`）→
  `<入力>_front.mp4` / `<入力>_rear.mp4`（前後は同じ時間軸）
- **走行の結果を 1 枚で見る**: launcher の「走行サマリ」（`tools/run_summary_gui.py`、中身は `raspi/tools/run_summary.py`）。
  周回タイム・衝突・電圧・前兆・学習値・通信と、直前の走行との差分。CLI は
  `python3 -m raspi.tools.run_summary <mcap> [--out-dir DIR] [--prev 前回の_summary.json]`
- **自己位置を見る**: launcher の「SLAM 再処理」（`tools/slam_replay_gui.py`）→ `<入力>_slam.mcap` が Foxglove で開く。
  3D パネルは Display frame を `odom`、`/viz/*` を表示（地図は Color mode「RGBA (separate fields)」）。差は `/pose_compare.err_pos` `.err_yaw`

---

## 10. 新しく何かを足すときの定石

### 自動運転モードを足す

**GUI を触らない。** `raspi/auto/` に `Planner` のサブクラスを1つ書き、
`registry.py` に1行足すだけで GUI の選択肢に出る。
パラメータの UI も planner が宣言した `ParamSpec` から自動生成される。

```
raspi/auto/base.py          Planner / ParamSpec ← バスも WS も知らない純粋な計算
raspi/auto/<新しいの>.py     アルゴリズム本体
raspi/auto/registry.py      id → クラス（+1行）
raspi/nodes/planning_node.py 配線だけ。触らない
```

### UART パケットを増やす・変える

1. `raspi/proto/protocol.toml` を直す（**Pi 側の唯一の定義**）
2. `python3 raspi/proto/generate.py` → `generated/packets.py` と `generated/surge_proto.h` が再生成される
3. **ファームの `ras_link.[ch]`（TYPE・LEN・ビット・`RAS_PROTOCOL_VERSION`）を手で同じに直し、`protocol_version` を上げる**
   （ファームは生成ヘッダを取り込んでいない。[`raspi/proto/README.md`](../raspi/proto/README.md)）
4. `docs/uart_protocol.md` を更新（**仕様の説明はこれが正**。数値の正は toml）
5. テスト → `deploy.sh --restart-io` とファームの書き込み

**版数がずれている間は ARM が通らない**ので、Pi と STM32 は同時に上げること。

### バスのトピックを増やす

`raspi/msgs/types.py` に型を足し、**`raspi/bus/zbus.py` の `TOPIC_OWNER` に持ち主を登録する**（漏れると「動くのにデータが来ない」）。
GUI にも見せるなら `python3 config/gen_msgs.py` で `gui/src/generated/msgs.ts` を再生成する。

### シミュレータに機能を足す

**シムの概念を実機側に持ち込まない**（UI は `sim/gui.py`、通信は UDP の私設チャンネル。例外は SIM バッジだけ）。
理由は [`sim/README.md`](../sim/README.md#設計の原則--シミュレータが偽装するのは-stm32-だけ)。

---

## 11. 既知の穴と文書の読み方

**文書とコードが食い違ったらコードが正。** 実際のノード構成は
[`system_overview.md` §6](system_overview.md#6-raspberry-pi-の中身--プロセスとトピック) を見る。

### 直したほうがよい実装上の小さな穴

| 場所 | 内容 |
|---|---|
| `install_services.sh --remove` | **`surge-logclean.timer` を消し損ねる**（`surge-*.service` しか消さない）。サービス本体が消えたタイマーが enable のまま残り、毎時 fail する |
| `install_services.sh` | `$1` しか見ないので **`--safe --with-logger` を併用できない**。`--with-logger` 単体では arm が有効のまま |

---

## 12. 学習ベースの自動運転を動かす（LiDAR E2E / カメラセグメンテーション / カメラE2E）

独立した3本の学習パイプラインがある。どれも「**Mac で学習 → ONNX 化 → `models/` に配置 →
GUI でモデル名を選ぶ**」という流れは共通だが、中身も検証手段も別物。
学習・評価のロジックの詳細は各スクリプトの docstring が正（`ml_*` に README は無い）。

| | LiDAR E2E（`e2e_lidar`） | カメラセグメンテーション（`ftg_cam`/`cam_centerline`） | カメラE2E（`cam_e2e`） |
|---|---|---|---|
| 学習方法 | 強化学習（PPO、シム上で試行錯誤） | 教師あり学習（人がラベル付けした走行画像） | 模倣学習（人の操舵指令を直接ラベルにする） |
| 学習コード（Mac） | `ml_lidar/` | `ml_cam/` | `ml_cam_e2e/` |
| モデルの置き場 | `models/e2e_lidar/<名前>.onnx`（＋同名 `.json`） | `models/<名前>.onnx`（＋同名 `.json`） | `models/cam_e2e/<名前>.onnx`（＋同名 `.json`。**`.json` の無いモデルは読み込まれない**） |
| 推論コード | `raspi/auto/e2e_lidar.py`（planner 本体） | `raspi/nodes/cam_perception_node.py`（**独立プロセス**。`scan/cam` へ変換）＋ `raspi/auto/follow_the_gap_cam.py`/`cam_centerline.py` | `raspi/nodes/cam_e2e_node.py`（**独立プロセス**。操舵を直接推論）＋ `raspi/auto/cam_e2e.py` |
| シムで検証できるか | **できる**（`sim.run` は LiDAR を持つ） | **できない**（シムはカメラを持たない） | **できない**（同左） |

`models/` は `.gitignore` 対象で、`tools/deploy.sh` の rsync で配る。実車で要る追加操作は無い（推論プロセスは常時起動で、対応するモードを選んだ間だけ推論する）。
各学習パイプラインには操作用の薄い Tkinter GUI がある（`ml_lidar/app.py`・`ml_cam/app.py`・`ml_cam_e2e/app.py`、
launcher から起動）。ロジックは持たないので、中身のスクリプトを直せば GUI 側は変えなくてよい。

**共通の注意**: モデル名を GUI で選び直すと、そのモードの engage は解除される。モデル未選択・ロード失敗・推論失敗の間は
停止側に倒れる（カメラ系は全セクタ `sector_seen=False`／`ready=False`）。**レンズや取付角度を変えたら撮り直して再学習**
（カメラの校正は [`calibration.md` §7](calibration.md#7-カメラ魚眼レンズ)）。

### 12.1 LiDAR E2E（強化学習、`e2e_lidar`）

シム上で「コースに沿って進めたら＋報酬・衝突したら－報酬」を頼りに、Stable-Baselines3 の PPO で方策を試行錯誤させる。
未知の点群パターンに対する挙動は原理的に保証できないので本番の主力にはしない。planner 側に固定距離の停止策は持たず、
STM32 の `auto_stop` で受ける（[`system_overview.md` §9.3](system_overview.md#93-走るより止まるを先に決める)）。
報酬設計は `ml_lidar/env.py` と `train_rl.py` の docstring。

```bash
.venv/bin/pip install -r ml_lidar/requirements.txt          # 初回だけ

# 1. 学習（毎エピソード形状も道幅も変えたランダムコース）。出力は ml_lidar/runs/<run名>/。
#    既存の run 名には --resume（続きから）か --overwrite が要る。最大舵角は vehicle.toml を常に使う
.venv/bin/python -m ml_lidar.train_rl --run-name v1 --total-timesteps 2000000 --n-envs 8 --max-speed 2.0
.venv/bin/tensorboard --logdir ml_lidar/runs/v1/tb          # 学習曲線（http://localhost:6006）
.venv/bin/python -m ml_lidar.watch --model v1               # （任意）学習中の方策を観戦。学習を遅くしない

# 2. ONNX 化。観測・行動の契約は run の env_config.json から読む。
#    実運用のモデルは eval/best_model_generalized.zip（ランダムコース・DR ありの汎化評価）
.venv/bin/python -m ml_lidar.export_onnx_rl --model ml_lidar/runs/v1/eval/best_model_generalized.zip --name v1

# 3. 実車に配る
tools/deploy.sh --no-gui
```

GUI: 設定タブ「E2E LiDARモデル」で選ぶ（出ないときは「更新」）→ 自動運転タブで「E2E LiDAR」→ ARM → engage。
`max_speed` はモデル出力をクランプするだけなので、学習時の `--max-speed` より上げても速くならない。
舵角の意味が別のリンク換算のもとで作られたモデルは契約 JSON の `steer_link` から読み替える。

### 12.2 カメラセグメンテーション走行（`ftg_cam`）

前方カメラで「走行可能／不可能」の2値セグメンテーションを行い、`raspi.nav.ipm`（逆投影）と `OccGrid.raycast()` で
LiDAR と同じ形の擬似距離配列に変換して `scan/cam` へ流す。ギャップ探索は `FollowTheGap`（`ftg`）をそのまま使う。

```bash
.venv/bin/pip install -r ml_cam/requirements.txt             # 初回だけ
# SAM のチェックポイント（annotate.py が使う。数百MB）を別途ダウンロード:
#   https://dl.fbaipublicfiles.com/segment_anything/sam_vit_b_01ec64.pth

# 0. 実車の GUI「ログ」タブで画像つきの .mcap を録ってダウンロードする
# 1. フレームを抽出（間引きは --min-interval-ms、既定0=全件）
python3 ml_cam/extract_frames.py logs/run1.mcap --out ml_cam/data/frames --cam front
# 2. アノテーション（走行可能な床を1クリック→SAMがマスクを提案。Shift+クリックで除外点）
python3 ml_cam/annotate.py ml_cam/data/frames --checkpoint sam_vit_b_01ec64.pth --model-type vit_b
# 3. 学習（val_iou を見ながら）
python3 ml_cam/train.py --frames ml_cam/data/frames --epochs 30 --out ml_cam/runs/latest
# 4. ONNX 化（入力解像度・正規化・閾値という前処理契約を同名 .json に焼く）
python3 ml_cam/export_onnx.py --checkpoint ml_cam/runs/latest/best.pt --size 224x224 --out models/<名前>.onnx
# 5. 実車に配る
tools/deploy.sh --no-gui
```

- `surge-cam-perception` は常時 enable。`auto/ctrl`（GUI のモード選択）が `ftg_cam`/`cam_centerline` の間だけ推論する。
  モデルは GUI（設定タブ「セグメンテーションモデル」）で選んだ名前を `cam/model` で受け取り、モードを選ぶ前から先読みする
- 前方カメラの共有メモリを読むので `surge-camera` が動いていることが前提
- GUI: 自動運転タブで「Follow the Gap（カメラ）」。`視野角` はカメラの実視野より広げても意味が無い

### 12.3 カメラE2E（模倣学習、`cam_e2e`）

**前方カメラの画像だけ**から操舵を直接回帰する。IPM もLiDARも使わない——カメラの取付高さが低く
IPM の深度誤差が大きくなる問題を、画像から操作を直接学習することで迂回するのが狙い。
教師データは走行記録の前方カメラ画像と人の指令（`cmd`）を時刻で組にするだけで作れる。
**その代わり、悪い手本を外す作業がモデルの出来を決める。**

#### 手順（操作パネル）

launcher から「ml_cam_e2e」（`ml_cam_e2e/app.py`）。タブは作業の順に並ぶ。

| タブ | すること |
|---|---|
| ① ペア抽出 | 録った `.mcap` から (画像, 人の操作) の組を取り出す。モデル名を決める |
| ② 確認・選別 | 映像と舵・速度の時系列を見て、手本にしない区間を除外する。`←/→` 1枚送り、`space` 再生、`[` `]` で範囲、`x` で除外 |
| ③ 偏り | 舵・速度の分布と記録ごとの内訳。直進が大半なら「直進過多の補正」を上げる |
| ④ 学習 | 学習曲線の `val_mae` は検証データでの舵の誤差 |
| ⑤ エクスポート | `models/cam_e2e/<名前>.onnx` と契約 `.json` を書く |
| ⑥ 評価 | 予測と手本の時系列、誤差の大きい場面の一覧。場面から②へ飛べる |

**②と⑥を往復するのが基本。** 誤差の大きい場面は、手本が悪い（そこだけ人が違う操作をした）ことが多い。

#### 走行の録り方（★ここで出来が決まる）

- **画像つき・高めのレートで録る**: `tools/record.sh --image-hz 15`（GUI の MCAP 記録は 5Hz で枚数が稼げない）
- **手本は人の手動運転か、うまく走れている自動運転。** 自動運転の走行を手本にするなら①の「自動運転中の走行も手本にする」
  （`--include-auto`）。入れ忘れると自動運転の記録は全部落ちる。`cam_e2e` 自身の走行は手本にしない
- **速度モードで運転する**（トルクモードの記録は `target_speed` が 0 のままで抽出で落ちる）
- **実車で走らせる速度で録る。** 絵のブレは旋回の速さに比例するので、モデルはブレを舵の手がかりに使う。
  手本より遅く走らせると舵が足りず、速く走らせると崩れる
- **壁際から戻る操作も録る・両方向を走る**（左右反転の水増しは既定で有効。コースが左右非対称なら④で切る）
- 止まって待つ・後退する区間は自動で学習から外れる（`ml_cam_e2e/samples.py`）

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

自動運転タブで「E2Eカメラ（模倣学習）」→「カメラE2Eモデル」欄でモデルを選ぶ（`config/cam_e2e_model.json` に残る）→ ARM → engage。
パラメータは `raspi/auto/cam_e2e.py` の `ParamSpec`（GUI に説明が出る）。**初めてのモデルは最高速度を低くして試す。**

★ **このモードは自分では止まらない。** LiDAR を見ないので、前に物があってもモデルが「進め」と出せば進む。
GUI の**自動停止（STM32 の `auto_stop`）を ON** にしておくこと。

#### 仕組みの要点

- **学習と推論の前処理は同じ関数**（`raspi/core/cam_e2e_preproc.py`）。処理を変えたら `PREPROC_VERSION` を上げる
  （版の違うモデルは `cam_e2e_node` が拒否する。左右の切り落とし付きは `PREPROC_VERSION_CROP`）
- **速度は舵から決める**（曲率 κ=tan(δ)/L から `v ≤ sqrt(a_lat_max/κ)`）。モデルの `speed_norm` は実車で使わず、
  学習の `--speed-weight` は既定 0（舵だけ）。舵が切れてから遅くなるので、コーナー手前の減速は遅れる
- **ずれから戻す舵は「向きをずらした絵」で教える**（`ml_cam_e2e/yaw_aug.py`、`--yaw-aug-k`）。手本の記録には
  ずれた絵とそこで戻す舵が無いので、そのままだとずれへの応答 dδ/dψ が弱く、実車で線を外れる（舵の倍率では直らない）。
  ⑥評価のログに dδ/dψ が出る（負で大きいほど戻す力が強い）
- **舵の正解を変えない外乱を混ぜる**（`ml_cam_e2e/photo_aug.py`、`--photo-aug`）
- **手本の指令は推論の遅れの分だけ後のものを組む**（`--label-shift-ms`）。推論の遅れ（`cam_e2e/cmd` の `t_pub − t_capture`）が
  変わったら合わせて学習し直す

★ **シミュレータでは走行を検証できない**（`sim.bench --mode cam_e2e` も対象外）。代わりに `ml_cam_e2e/tests/test_pipeline.py` が
合成した記録で抽出→学習→エクスポート→実車側の読み込み→評価までを通す（`.venv/bin/python -m pytest ml_cam_e2e/tests -q`）。

ディレクトリの構成は [`system_overview.md` §13](system_overview.md#13-リポジトリ構成と技術スタック)、どこを直したら何を再起動するかは §2。
