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
12. [学習ベースの自動運転を動かす（LiDAR E2E / カメラセグメンテーション）](#12-学習ベースの自動運転を動かすlidar-e2e--カメラセグメンテーション)

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
- **点群の解釈や換算を疑っている** → ログ再生（本物のノイズが入っている）
- **遅延・電流・温度・電波を知りたい** → 実車以外に手段は無い

---

## 2. 変更 → 反映の対応表（最重要）

**Pi 上では systemd が5つのサービスを持っている。何を直したかで、必要な操作が変わる。**

| 直したもの | 必要な操作 | E-Stop がラッチするか |
|---|---|---|
| `gui/` | **rsync だけ**（`tools/deploy.sh`）。telemetry_node は毎リクエストでファイルを読む | しない |
| `raspi/nodes/telemetry_node.py` | `deploy.sh --restart`（surge-telemetry / surge-camera） | しない |
| `raspi/nodes/camera_node.py` | 同上 | しない |
| `raspi/nodes/cam_perception_node.py` | **`--restart` には入っていない**（`surge-planning`と同じ既知の穴）。`ssh surge-mk2 'sudo systemctl restart surge-cam-perception'`。**`surge-cam-perception`は既定で無効**なので、有効化していなければ何もしなくてよい | しない |
| **`raspi/auto/` `raspi/nav/` `planning_node.py`** | **`ssh surge-mk2 'sudo systemctl restart surge-planning'`**（★ `--restart` に入っていない。下記） | しない |
| `config/vehicle.toml` `config/auto.json` | 読んでいるノードを再起動（planning / io）。`vehicle.toml` は**先に `python3 config/generate.py`**（GUI 用 TS を再生成）してから rsync | io なら**する** |
| `raspi/nodes/io_node.py` `raspi/io/` `raspi/msgs/` `raspi/proto/` | **`deploy.sh --restart-io`** | **★★ する** |
| `raspi/setup/install_services.sh` の引数（`--max-speed` 等） | **`--services` ＋ `--restart-io`** | **★★ する** |
| `raspi/proto/protocol.toml` | **先に `python3 raspi/proto/generate.py`**、その後 `--restart-io`。**STM32 側にもヘッダを渡す** | **★★ する** |
| `raspi/msgs/types.py` | **`gui/src/types.ts` も手で直す**（写しなのでズレる）→ rsync ＋ `--restart-io` | **★★ する** |
| `models/*.onnx`（カメラ用）`models/e2e_lidar/*.onnx`（E2E用） | **通常の `tools/deploy.sh`（rsync のみ）。** プロセス再起動は不要——`cam_perception_node`/`e2e_lidar` の `reload_if_changed()` が GUI でモデル名を選んだ瞬間に読み直す | しない |
| `ml/` `ml_lidar/`（学習パイプライン一式） | **Mac 側だけで完結。Pi には無関係**（rsync では運ばれるが、`raspi/` 側は読まない） | しない |

> **`--services` は unit ファイルを書き換えるだけ。** 走っている `io_node` は古い引数のまま
> 動き続けるので、反映には `--restart-io` が要る。

### ★ 既知の穴：`--restart` に `surge-planning` が入っていない

`tools/deploy.sh --restart` は `surge-telemetry surge-camera` しか再起動しない。
**自動運転まわり（`raspi/auto/` `raspi/nav/` `planning_node.py`）を直したら手で再起動する。**

```bash
ssh surge-mk2 'sudo systemctl restart surge-planning'
```

planning_node は `auto/cmd` に出すだけで E-Stop に無関係なので、
`--restart` の対象に足してよい（未対応のまま）。

### ★★ `--restart-io` は E-Stop をラッチさせる

`io_node` は GPIO6 に 100Hz のハートビートを出している（安全第1層）。
**止めた瞬間に STM32 が E-Stop をラッチし、車両のボタン2を物理的に押すまで戻らない。**

- **仕様どおりの動作であり、故障ではない**
- **車体に手が届く状態でだけ使う**（リモートで叩くと、現地に行くまで車が死ぬ）
- `Restart=on-failure` で io_node が再起動しても**ラッチは解除されない**。人間の操作でしか戻らない

---

## 3. コマンド早見表

### Mac 側

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
.venv/bin/python -m unittest discover -s raspi/tests -t .
python3 raspi/proto/generate.py                   # protocol.toml を直したら必ず

# ── GUI ──
cd gui && npm run dev                             # http://localhost:5173（自動リロード）
SURGE_HOST=surge-mk2.local npm run dev            # 編集しながら実機に繋ぐ
npm run build                                     # -> gui/dist（Pi に node は不要）

# ── Pi へ ──
tools/deploy.sh                                   # GUI ビルド + rsync（既定・いちばん安全）
tools/deploy.sh --no-gui                          # Python だけ直したとき
tools/deploy.sh --test                            # 反映後に Pi 上でテスト
tools/deploy.sh --restart                         # telemetry / camera を再起動
tools/deploy.sh --services --restart-io           # unit を入れ直して io を再起動（★E-Stop）
tools/record.sh --duration 60                     # SD に書かずに MCAP を PC へ

# ── ログ ──
.venv/bin/python -m raspi.tools.logcat logs/x.sfl          # .sfl の要約
.venv/bin/python -m raspi.tools.sfl2mcap logs/x.sfl        # .sfl → .mcap
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

---

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
.venv/bin/python -m unittest discover -s raspi/tests -t .        # Mac
tools/deploy.sh --test                                           # Pi 上でも回す
```

**現状 414 件・約 12.5 秒・全部通る**（skip 1 件は「GPIO の無い環境」＝ Mac だから）。
ハードウェアもシリアルもカメラも要らない（`ipc://` はテンポラリに閉じ込めてある）。

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

- テストは **`unittest`**（標準ライブラリ）。pytest は要らない
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
| SD が埋まる | `.sfl` 2.5MB/分 ＋ `.mcap` 10.3MB/分 ＝ **1日 18GB** | `surge-logclean.timer` が毎時「7日超」と「合計 8GB 超過分」を消す。**`.mcap` は既定で SD に書かない**（GUI か `tools/record.sh` で PC に流す） |
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

## 12. 学習ベースの自動運転を動かす（LiDAR E2E / カメラセグメンテーション）

独立した2本の学習パイプラインがある。どちらも「**Mac で学習 → ONNX 化 → `models/` に配置 →
GUI でモデル名を選ぶ**」という流れは共通だが、中身も検証手段も別物なので混同しないこと。

| | LiDAR E2E（`e2e_lidar`） | カメラセグメンテーション（`ftg_cam`） |
|---|---|---|
| 学習方法 | 強化学習（PPO、シム上で試行錯誤） | 教師あり学習（人がラベル付けした走行画像） |
| 学習コード | `ml_lidar/`（Mac 専用。Pi には運ばない使い方をする） | `ml/`（同左） |
| モデルの置き場 | `models/e2e_lidar/<名前>.onnx`（＋同名 `.json`） | `models/<名前>.onnx`（＋同名 `.json`）。**E2E とは別ディレクトリ** |
| 推論コード | `raspi/auto/e2e_lidar.py`（planner本体。配線は`planning_node`がやる） | `raspi/nodes/cam_perception_node.py`（**独立プロセス**。`scan/cam`へ変換）＋ `raspi/auto/follow_the_gap_cam.py`（`FollowTheGap`をそのまま流用） |
| シムで検証できるか | **できる**（`sim.run` は LiDAR を持つ） | **できない**（`sim.run` は `--no-camera` 固定でカメラを持たない） |
| 実車で動かすのに要る追加操作 | 無し（`planning_node` が engage 中だけ推論する） | `surge-cam-perception` を有効化する必要あり（**既定で無効**。下記 12.2） |

いずれも `models/` はリポジトリの `.gitignore` 対象（機体・学習ごとに違う大容量ファイルのため）。
配布は `tools/deploy.sh` の rsync に任せる——コミットには乗らない。

### 12.1 LiDAR E2E（強化学習、`e2e_lidar`）

`disparity_extender.py`（`de`）のような**模倣学習ではない**。シム上で「コースに沿って
進めたら＋報酬・衝突したら－報酬」を頼りに、Stable-Baselines3 の PPO で方策を試行錯誤
させる。`de` を再現するのではなく**それを超えうる**代わりに、未知の点群パターンに
対する挙動は原理的に保証できない——だから `e2e_lidar.py` は独立した `stop_dist`
（正面がこの距離を切ったらモデル出力を無視して無条件停止）を必ず持つ。

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

ターミナル操作をまとめて避けたいなら `ml_lidar/app.py`（または `ml_lidar/start_app.command`
をダブルクリック）が上記1〜3をボタンで操作できる薄い Tkinter GUI（`ml/app.py` と対称、
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
   届かないので意味が無い。`stop_dist` は上記の独立安全策。**最大舵角は
   GUIパラメータではない**——`config/vehicle.toml` の車両物理限界を常に使う
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
.venv/bin/pip install -r ml/requirements.txt
# SAM のチェックポイント（annotate.py が使う。数百MB）を別途ダウンロード:
#   https://dl.fbaipublicfiles.com/segment_anything/sam_vit_b_01ec64.pth

# 0. 走行を録画する（実車の GUI「ログ」タブで .mcap 記録、「画像を含める」を ON）。
#    Pi 側には新しい記録コードは不要——.mcap をダウンロードして Mac に置くだけ

# 1. .mcap からフレームを抽出（間引きは --min-interval-ms、既定0=全件）
python3 ml/extract_frames.py logs/run1.mcap --out ml/data/frames --cam front

# 2. アノテーション（走行可能な床を1クリック→SAMがマスクを提案。Shift+クリックで除外点）
python3 ml/annotate.py ml/data/frames \
    --checkpoint sam_vit_b_01ec64.pth --model-type vit_b

# 3. 学習（毎エポック IoU を表示。過学習していないか val_iou を見ながら回す）
python3 ml/train.py --frames ml/data/frames --epochs 30 --out ml/runs/latest

# 4. ONNX 化（入力解像度・正規化・閾値という前処理契約を同名 .json に焼く）
python3 ml/export_onnx.py --checkpoint ml/runs/latest/best.pt \
    --size 224x224 --out models/<好きな名前>.onnx

# 5. 実車に配る
tools/deploy.sh --no-gui
```

ターミナル操作をまとめて避けたいなら `ml/app.py`（または `ml/start_app.command` を
ダブルクリック）が上記1〜4をボタンとファイル選択ダイアログでサブプロセス起動するだけの
薄い Tkinter GUI。推論・学習のロジックは持たないので、中身のスクリプトを直せば
こちら側は何も変えなくてよい。

★ **実車での推論プロセス（`surge-cam-perception`）は systemd unit として存在するが、
既定では無効。** `install_services.sh` は他の4ノードと違ってこの unit を
`enable --now` しない（2026-08-28、`--with-cam-perception` を追加）。理由は
`ftg_cam` が実験的なモードであることに加え、**`cam_perception_node` は
`planning_node` が今どのモードを選んでいるかを一切知らない独立プロセス**で、
前方カメラのフレームが来る限り`ftg_cam`を使う気が無くても CNN 推論を回し続ける
——常時有効にすると CPU・電力を無条件に消費し続けるため:

```bash
ssh surge-mk2
sudo systemctl start surge-cam-perception          # 一時的に（このブート限り）
sudo systemctl enable --now surge-cam-perception    # 次回起動時も自動で有効化
# または: sudo bash ~/surge_mk2/raspi/setup/install_services.sh --with-cam-perception
```

- 起動時の既定引数には `--model` を渡していないので、GUI（設定タブ
  「セグメンテーションモデル」）で選んだモデル名を `cam/model` トピック経由で待つ
  （プロセス再起動もSSHも要らずにモデルだけ選び直せる）
- 前方カメラの共有メモリ（`image/front`）を読むので、**`surge-camera` が動いていること
  が前提**（unit の `After=surge-camera.service` で順序は保証している）
- 推論が失敗する・モデル未選択の周期は全セクタ `sector_seen=False`（＝壁）を出し続ける
  ので、`ftg_cam` は自然に「点群の欠測が多すぎる」で止まる側に倒れる（安全側）
- `raspi/nodes/cam_perception_node.py` を直したら、他ノードと同じく
  `tools/deploy.sh --restart` **には入っていない**（§2 の既知の穴と同じ理由で
  `surge-planning` も未対応）。手で `ssh surge-mk2 'sudo systemctl restart surge-cam-perception'`

GUI での使い方:

1. 設定タブ →「セグメンテーションモデル」で `models/` 直下の `.onnx` 一覧から選ぶ
2. 自動運転タブ → モードで「Follow the Gap（カメラ）」を選択
3. パラメータの `視野角` はカメラの実視野（hfov≈66°）より広げても見えない範囲なので
   意味が無い。既定60°

★ **シミュレータでは検証できない。** `sim.run` は `--no-camera` 固定でカメラそのものを
持たないため、`ftg_cam` を試すには実車が要る（`e2e_lidar` は LiDAR が対象なので
`sim.run` でそのまま試せるのと対照的）。

---

## 付録: ディレクトリと担当

| ディレクトリ | 中身 | 直したら |
|---|---|---|
| `raspi/nodes/` | プロセス本体（io / camera / telemetry / planning / logger / replay / **cam_perception**）。`cam_perception_node`（`surge-cam-perception`）だけは他と違い**既定で無効**（§12.2） | ノードごとに再起動 |
| `raspi/auto/` | 自動運転アルゴリズム（**バスも WS も知らない純粋な計算**。`e2e_lidar.py`/`follow_the_gap_cam.py` もここ） | surge-planning 再起動 |
| `raspi/nav/` | SLAM・占有格子・経路（**一旦棚上げ中。消さない**）。`ipm.py`（カメラ逆投影）は `ftg_cam` が使用中 | surge-planning 再起動 |
| `raspi/proto/` | UART 定義（**STM32 と共有する唯一の定義**） | 再生成 ＋ `--restart-io` |
| `raspi/bus/` `raspi/msgs/` | ZeroMQ ラッパ・共有メモリ・メッセージ型 | 関係ノード全部 |
| `raspi/io/` | `SerialLink` / GPIO（**`import serial` はここ1箇所だけ**） | `--restart-io` |
| `raspi/rec/` | `.sfl` / MCAP の書き出し | io / logger |
| `raspi/tools/` | 単発の検査・変換ツール | — |
| `raspi/tests/` | unittest | — |
| `gui/` | React + TypeScript | rsync だけ |
| `sim/` | Mac 専用シミュレータ（**Pi には運ぶが使わない**）。カメラは持たない | — |
| `ml/` | カメラセグメンテーションの学習パイプライン（Mac 専用。§12.2） | Pi には無関係 |
| `ml_lidar/` | LiDAR E2E 強化学習パイプライン（Mac 専用。§12.1） | Pi には無関係 |
| `models/` | 学習済み ONNX の置き場（`models/*.onnx`=カメラ用、`models/e2e_lidar/*.onnx`=E2E用）。`.gitignore` 対象 | `tools/deploy.sh` で運ぶだけ。再起動は不要 |
| `config/` | 車両諸元・自動運転パラメータ | 読んでいるノード |
| `tools/` | `deploy.sh` / `record.sh` | — |
