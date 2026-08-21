# SURGE Mark.2 開発進捗

会話が圧縮されても文脈を失わないための作業ログ。**新しいセッションではまずこれを読む。**
設計の中身は `docs/` が正。ここには「今どこまでやったか」「なぜそう決めたか」の要約だけを書く。

最終更新: 2026-08-21（コードレビュー13項目を全修正し実機へ反映。下記「2026-08-21」節）
旧: 2026-08-17（3561行あった本ファイルを軽量版とアーカイブに分割。**情報は削除しておらず、
詳細な実験経緯・実測値はすべて `docs/progress_archive.md` に移した**。分割前と同じ内容が
アーカイブに残っているので、根拠が要るときはそちらを見ること）

方針の最新は 2026-08-14 の **★ SLAM を一旦棚上げし、非SLAM の Disparity Extender で進める**（下記）。

---

## ★★ 方針：SLAM は一旦棚上げ。非SLAM（反射型）で進める（2026-08-14 決定）

自作 LiDAR SLAM は circuit で自己位置 2.6cm まで来たが、**oval が閉じない**
（1周目のループ閉じが走らない）、**RACE 段が開始直後に壁へ接触して固着する**が
未解決。一方で反射型の **`DisparityExtender`（id `de`）がシムで 2.2 倍速く**、
SLAM が1周もできなかった分離帯コース（split）も周回できた。

**当面は `de` を本線にする。** 詳細な実測値・切り分け経緯は
`docs/progress_archive.md` の「LiDAR SLAM と地図生成を実装した」以降の各節。

- `raspi/nav/`（SLAM 一式）と `raspi/auto/raceline.py` は**消さない。**
  「一旦」であって撤退ではない
- 新しい機能は `de` 側に載せる
- SLAM に戻るときの入口は「周回を検出するたびに `close_loop()` を走らせる」（未着手）

### 非SLAM で進めるときに無くなるもの／代わりの手

| SLAM があって初めてできること | 非SLAM での代替 |
|---|---|
| レーシングライン（アウトインアウト） | **無い。** 反射型は毎周期その場で決めるだけ |
| 先読みブレーキ（次のコーナーの形） | 見通し（`ext`）で近似。**原理的に劣る** |
| ラップ計測・周回数 | **ジャイロの累積回頭 ÷ 360°** で数えられる（地図不要） |
| 「経路から落ちた」の検出 | 無い。`stop_dist` と超音波 `auto_stop` が最後の砦 |

## ★ 2026-08-21：コードレビュー13項目を全修正し、実機へ反映済み

`docs/review_2026-08-21.md` の指摘を**全項目**片付けた（各項目に「解決済」を追記済み）。
**中身と判断の根拠はレビュー文書側にある。** ここには実機に効く要点だけ書く。

### ★★ 運用が変わった点（これだけは覚えておくこと）

1. **`telemetry_node` の既定バインドが `127.0.0.1` になった。**
   外の PC から GUI を開くには `--host 0.0.0.0` が要る。
   `run_stack.sh` と `install_services.sh` の両方に明示済みなので通常運用は変わらないが、
   **手で `python -m raspi.nodes.telemetry_node` と打つと Pi 自身からしか見えない**
2. **Origin 検査が入った。** 別サイトから `/ws/*` を張ると 403。
   判定は `Host` ヘッダとの一致なので、`surge-mk2.local` でも IP 直打ちでも通る
3. **自律走行の `engage` に操縦権が要る**（解除は従来どおり誰でも通る）
4. **共有トークンは既定で無効。** 使うなら `config/secret.txt`（gitignore 済み）に書き、
   GUI を一度 `?token=…` 付きで開く
5. **検査コマンドが1本になった**: `./tools/check.sh`（`--fast` で生成物だけ）。
   同じものを GitHub Actions（`.github/workflows/ci.yml`）が Python 3.12/3.13 で回す

### 新しく増えた「正」

| 正 | 生成物 | 検査 |
|---|---|---|
| `raspi/msgs/types.py` | `gui/src/generated/msgs.ts` | `config/gen_msgs.py --check` |
| `config/vehicle.toml` の `[safety]` | `gui/src/generated/vehicle.ts` の `SAFETY` | `config/generate.py --check` |
| `raspi/proto/protocol.toml` の版番号 | （文書の見出し） | `config/check_docs.py --check` |

**メッセージ型を GUI に手書きしないこと。** `types.py` を直して `gen_msgs.py` を回す。
デッドマン 150ms も同様で、`vehicle.toml` の `[safety]` が唯一の出どころ。

### 実機で確認したこと（2026-08-21・surge-mk2）

- `pytest` **435 tests OK**（Pi 上・Python 3.13）
- Origin 検査: 同一オリジン/Vite dev/Origin無し = 通す、別サイト/ポート違い/接尾辞偽装 = **403**
- テレメトリの札 `schema=0x667639be` が Pi と手元で一致
- デッドマンが io_node / telemetry_node / GUI の3箇所とも **150ms** で一致
- `_fence()`（seqlock のメモリバリア）実機 Cortex-A76 で **169ns/回**。無視できる
- 再起動後 `health=OK` / `protocol=0x0009` / `sim=False` / `estop_active=False` /
  `hb_stalls=0` / `cmd_rtt_ms≈9.6ms` / TC・TV・片輪浮き対策すべて有効

### ★ JPEG 二重エンコードは「測って、直さないと決めた」

レビュー 🟡7。実測 **3.13ms/枚**（simplejpeg・実機）。二重ぶん（logger の
`image_hz=5` × 2台）は **Pi 5 全体の 0.8%** しかない。camera_node に寄せると
「誰が見ているか」の伝達まで作る必要があり、0.8% では割に合わない。
計測は `status.camera_jpeg` に残してあるので、`image_hz` を上げたり
カメラを増やしたときに再確認できる。

### 副産物：`noUncheckedIndexedAccess` で実バグが1つ出た

`gui/src/ws/map.ts` の `PALETTE` が3要素しかなく、圧縮データが壊れて 2bit 値 `3` が
出ると `PALETTE[3]` が `undefined` になり、分割代入の例外で**地図の描画が止まる**
形だった。4要素にして「壊れた1セルは未知として塗る」ようにした。

---

## 現在地

**★ Phase 0（通信基盤・ログ記録再生・GUI 骨格）完了。実車で走行実績あり。**
Phase 3（自律走行）は非SLAM系（FTG / Disparity Extender）と SLAM 系の両方を
**シムで**試作済みだが、**どちらも実車は未検証**。

以下の表で「詳細」「実測」とある項目の根拠は、同じ見出し・同じ日付で
`docs/progress_archive.md` を検索すれば出てくる。

| | 状態 |
|---|---|
| 設計フェーズ | 完了 |
| UART プロトコル | **v0.9 まで実車確認済み**（2026-08-20。v0.4確定→v0.5トルク直接指令の下地→v0.6トルク直接指令→v0.7超音波auto_stop→v0.8 TC/TV有効切替→v0.9片輪浮き対策。経緯はアーカイブ） |
| プロトコル実装（Python + C ヘッダ生成） | **完了** |
| **Pi 実機セットアップ** | **完了**（SSH・リポジトリ配置・venv・UART 有効化） |
| **STM32 実機との UART 疎通** | **確認済み**（双方向・エラー0） |
| **io_node 中核（シリアル+時刻同期）** | **実機で安定動作**（loss=0/crc=0） |
| **生フレームログ `.sfl` 記録＋解析ツール／`replay_node`** | **実機で記録・再生・完全一致まで確認済み** |
| **★ 2D シミュレータ（Mac 専用・`sim/`）** | **完了**（`io_node --sim` で実機と同じ制御コードのまま走る） |
| **★ 自動運転（FTG・SLAM+raceline・Disparity Extender）** | **シムで確認済み・実車未検証**（下記「未解決」） |
| **Disparity Pursuit（id `dp`）新設**（DEの段差塗り＋狙点選択 と FTGのPure Pursuit舵角を統合し、①狙点のヒステリシス ②曲率ベース速度上限 ③TTCによる早期停止 を追加。`raspi/auto/gap_pursuit.py`） | **実装完了・単体テスト58件green（2026-08-18）。シム・実車とも未検証** |
| **GPIO E-Stop ハートビート＋端から端までの動作確認** | **実機で合格**（`--allow-arm` 経路も含め2026-08-10に合格） |
| **カメラの実力測定／`camera_node` + 共有メモリリング** | **実機で動作確認済み**（2台30fps・ゼロコピー） |
| **LED 表示（緑=生存と可動性 / 赤=異常の重さ）** | **実機で目視確認済み** |
| **arm を通す最小実験** | **合格**（駆動電源維持・トルク0を確認） |
| **メッセージ型 + ZeroMQ バス／WS サーバ／GUI 運転ビュー** | **実機で端から端まで合格**（実映像・実点群含む） |
| **★ 実車を GUI（キーボード）で操縦** | **成功**（Phase 0 の達成条件の片方） |
| **操縦レスポンス（rAF+50Hz・ブレーキ・発進キック）** | **実車で確認済み** |
| **`tools/deploy.sh`／systemd 自動起動** | **動作確認済み**（電源投入20秒で全部復帰。arm 有効） |
| **mDNS（`surge-mk2.local`）運用** | **確立**（下記「Pi 実機」） |
| ~~AP モード~~ | **実装しない方針に確定**（下記） |
| **MCAP 記録（`logger_node`）／記録はPC側で受ける／GUI「ログ」タブ** | **実機で合格・反映済み** |
| **GUI 5タブ再編（ラジコン/自動運転/地図生成/診断/ログ）** | **完了** |
| **`docs/system_overview.md`・`docs/development.md` 新設** | **完了**（2026-08-16。文書と実装のズレを棚卸し。一覧は `development.md` §11） |
| Pi 純正ファン | **正常**（設定不要） |
| **起動音・GUI接続音（ブザー）** | **実機で合格**（2026-08-17。下記「GPIO18 はSTM32と共有」参照。STM32のブザー音より音質はやや粗いが許容） |
| **GUI ラジコンビュー GR風再設計**（黒基調+赤、LiDARをゲーム風ミニマップ化、映像:車体図4:1、後方PIP/LiDARの拡縮＆一括表示切替、後輪ゲージをタイヤ内へ統合、温度計5つ追加） | **実装完了・ブラウザ確認済み（2026-08-17）。実車のカメラ/LiDAR実データでの見え方は未確認** |
| **RasPi CPU温度の配信**（`telemetry_node._read_pi_temp_c()` が `/sys/class/thermal/thermal_zone0/temp` を読み `pi_temp_c` として `/ws/telemetry` に追加。GUIの温度計で表示） | **実装完了（2026-08-17）。Mac上では常にnullを返すことのみ確認済み。実車での実測値は未確認** |
| **GUI ラジコンビュー 追加調整**（舵角メータを速度計の左に追加・速度計を中央固定、速度計を-120〜120°/真上0°に、映像とメータ行を実測ベースで都度最大化——`useContainFit`/`useElementSize`。svg/canvas子要素の intrinsic サイズに負けてCSSのaspect-ratioだけでは縮む罠を2回踏んだ） | **実装完了・複数ウィンドウサイズでブラウザ確認済み（2026-08-17）。⚠ 後方PIPとLiDARミニマップを両方82%まで拡大すると重なる場合がある（未対応、実使用で気になれば要調整）** |

## Pi 実機（surge-mk2）

接続情報の詳細と機密は `docs/setup_credentials.md`（gitignore 済み）。要点だけ:

- **Raspberry Pi 5 / Debian 13 (Trixie) / Python 3.13.5**
- **接続は Ethernet 直結**。Mac の USB アダプタ ↔ Pi。**Pi eth0 = 固定 `169.254.55.2`**。
  `ssh surge-mk2` で決め打ち接続（`~/.ssh/config` 設定済み、鍵は `~/.ssh/id_surge_mk2` でパスフレーズ無し。
  `sshpass` は今は不要）
- **UART**: `/dev/serial0 -> ttyAMA0`。io_node では `/dev/serial0` を使う（名前直書き禁止）
- リポジトリは **Mac → Pi へ rsync**（`~/surge_mk2`）。git remote はあるが未 push
- venv は `~/surge_mk2/.venv`（`--system-site-packages`）
- **電源は 5V/5A（27W）USB-C PD 必須**（USB給電では不足してハングする実績あり。解決済みだが忘れないこと）

### よく使うコマンド

```bash
rsync -az --exclude __pycache__ --exclude '*.pyc' --exclude .venv --exclude docs/setup_credentials.md \
  surge_mk2/ surge-mk2:~/surge_mk2/
ssh surge-mk2 'cd ~/surge_mk2 && .venv/bin/python -m unittest discover -s raspi/tests -t .'

# 反映（毎回rsyncを手打ちしていたのをスクリプト化。何を再起動すべきかはスクリプト冒頭の表を見る）
tools/deploy.sh                # GUI をビルドして rsync（既定・いちばん安全）
tools/deploy.sh --no-gui       # Python だけ直したとき
tools/deploy.sh --services     # systemd unit を入れ直す（--max-speed 等を変えたとき）
tools/deploy.sh --restart      # surge-telemetry / surge-camera を再起動（E-Stop 無関係）
tools/deploy.sh --restart-io   # ★ surge-io を再起動（E-Stop がラッチする。車体のそばで実行）
```

### mDNS（`surge-mk2.local`）— IP を探す作業は不要

`surge-mk2.local` が直結・Wi-Fi のどちらでも生きている方を指す（`ssh surge-mk2` /
`http://surge-mk2.local:8000/` が常に通る）。運用上ハマりやすい点:

- **`~/.ssh/config` に `AddressFamily inet` が必須。** 無いと直結があるのに IPv6 リンクローカル
  （Wi-Fi 経由）へ迷い込む。curl は `-4`
- **IF を落として戻すと avahi が再広告しないことがある。**
  「直結が生きているのに遅い」ときは `ssh surge-mk2 'sudo systemctl restart avahi-daemon'`
- **素の `ping 169.254.55.2` は直結が生きていても失敗しうる。**（169.254/16 のルート重複のため）
  直結の生死は **`ping -b <IF名> 169.254.55.2`** で確認（IF名は挿し直すと変わる）
- mDNS が使えない環境では MAC のベンダ prefix `2c:cf:67`（Raspberry Pi Ltd）で `arp -an` を grep

### ⚠ 原因不明の再起動（2026-08-07・2回・未解決）

作業中に Pi が勝手に落ちて再起動していた事案が2回ある。電源・熱は異常なし（電源不足問題とは別。
電源不足自体は 5V/5A PD への交換で解決済み）。**原因は未特定のまま。**
永続ジャーナル（`sudo journalctl --list-boots` / `journalctl -b -1`）を有効化済みなので、
**再発したら必ずログを見て原因を追うこと。** 走行中に落ちるとハートビートが止まって
E-Stop になる（フェイルセーフではある）が、信頼性の問題として残っている。

### ★ 屋外に出る前に、研究室で SSID を登録しておくこと（未実施）

NetworkManager は知らない SSID に自動接続しない。**現地で登録しようとすると、
Pi に繋ぐ手段が直結 Ethernet しか無い状態になる。**

```bash
ssh surge-mk2 'sudo nmcli device wifi connect "SURGE-FIELD" password "********" ifname wlan0'
ssh surge-mk2 'sudo nmcli connection modify "SURGE-FIELD" connection.autoconnect-priority 20'
ssh surge-mk2 'sudo nmcli connection modify "tplink"      connection.autoconnect-priority 10'
```

---

## 決まっていること

### UART プロトコル — 現在 v0.11（`docs/uart_protocol.md` が正）

`raspi/proto/protocol.toml` が実装上の唯一の定義で、そこから Python パーサと STM32 側 C
ヘッダを生成する（`python3 raspi/proto/generate.py`）。v0.4確定 →
v0.5（`COMMAND` LEN10→12・ブレーキ/灯火/ホーン拡張）→ v0.6（トルク直接指令、上限0.125→0.15N·m）→
v0.7（超音波 `auto_stop`、ワイヤ非破壊）→ v0.8（TC/TV 有効切替、`param_id 0x0010`/`0x0020`）→
v0.9（片輪浮き対策、`param_id 0x0050`。TC本体とは独立、2026-08-20）→
v0.10（`MAX_SPEED`/`MAX_ACCEL`/`MAX_STEER` の `param_id 0x0001-3` を廃止し STM32 側固定定数へ
一本化）→ **v0.11（`LIMITS`(0x0A)/`LIMITS_REQ`(0x15) を新設。STM32側発、2026-08-21）**と進んだ。
**v0.8・v0.9 は実機で動作確認済み。v0.10・v0.11 は STM32側実装済みだが実機での動作検証は
まだ（STM32側ドキュメント曰く「実機での動作検証は未了」）。**
各版の変更点・実車確認の経緯はアーカイブの該当節（`UART プロトコル v0.6` / `v0.7` / `v0.5 に追従した` など）。

#### ★v0.11 `LIMITS`（車両の物理的な上限値。2026-08-21、Pi側実装済み・未コミット）

`LIMITS`（STM32→Pi、16B、`max_speed_m_s`/`max_accel_m_s2`/`max_torque_nm`/`max_steer_rad`
のf32×4・読み取り専用）を新設。v0.10でPiからパラメータの動的変更手段が失われた代わりに、
**STM32側の実際の固定上限値をPiが起動時に知れるようにする**のが目的。

- Pi側の対応（`raspi/nodes/io_node.py`）: `handshake()` で `VERSION_REQ` と同時に
  `LIMITS_REQ` を送る。**`_send_command()` が毎回 `state.limits` を読み、RC(MANUAL)・
  AUTO(自律走行) 問わず `--max-speed`/`--max-steer`（Pi側設定）と `LIMITS`（STM32実測）の
  小さい方でクランプする**（`raspi/msgs/convert.py` の `command_from_cmd` に
  `max_accel`/`max_torque` 引数を追加）。全指令がこの1箇所を通るので送信元を問わず効く
- **バグを1つ踏んで直した**: 最初の実装は `handshake()` の1秒タイムアウト内に `LIMITS` が
  届かなければ**二度と再送しない**ままだった（バンビが実機で「STM32から送っているはずの
  上限が適用されない」「同時起動しないと受け取れない」と報告して発覚）。
  `run()` のメインループで `state.limits is None` の間 1秒おきに `LIMITS_REQ` を再送する
  よう修正済み
- **GUIへの反映も追加実装した**（当初「多分ほぼ変更ないと思う」と見積もっていたが、
  バンビの「GUIの上限が3/5.5のままでSTM32実測(5/3)が反映されていない」という指摘で
  スコープに入れた）。`LinkDiag`（`raspi/msgs/types.py`）に `max_speed_m_s` 等を追加し、
  `bus_bridge.py` の `build_diag()` で伝搬
- **★2026-08-22 に「STMからの値を優先してほしい」で再修正**: 最初は
  `Math.min(静的既定, STM実測)` にしていたが、これだと STM32 実測の方が大きい場合
  （3.0固定→実際は5.0出せる、等）に古い静的値で頭打ちになったまま。
  `gui/src/store/ui.ts` の `effectiveRange()` を「**`LIMITS` 受信済みなら無条件にそれを
  使う**（静的値は WS 接続前のプレースホルダでしかない）」に変更。
  **`sim/stm32.py` にも `LIMITS`/`LIMITS_REQ` を実装した**（`_emit_limits`。実機と同じ
  `DRIVE_MAX_SPEED_M_S=5.0`/`DRIVE_MAX_ACCEL_M_S2=3.0`/`DRIVE_MAX_TORQUE_NM=0.15`/
  `spec.max_steer` を返す）ので、シムでも実機と同じ経路で動く
- **バンビに確認 → 「LIMITSを優先に変える」で回答済み（2026-08-22）。**
  `io_node.py` の `_send_command` も `min(--max-speed, LIMITS)` から「**`LIMITS` 受信済み
  なら無条件にそちらを使う**」に変更した（`--max-speed`/`--max-steer` は未受信時だけの
  フォールバック）。GUI・Pi 両方で「STM32実測を優先」に統一済み。
  `raspi/tests/test_cmd_path.py` に往復方向（LIMITSがより緩い場合／より厳しい場合）
  それぞれのテストを追加
- **★はまりどころ**: `LIMITS` は `ControlStatus`（`/ws/control` の status。イベント発生時
  にしかbroadcastされない）ではなく **`LinkDiag`（`/ws/telemetry` 8Hz、`bus/live.ts` の
  `useNumbers()`）から読むこと。** `tc_enabled`/`wheel_lift_guard_enabled` と同じ理由
  （2026-08-19に実機で発覚した「statusから読むと更新されずリロードするまで気づかない」
  バグを避けるため）。一度 `ControlStatus` 側に実装してから気づいてリバートした
- **★★実機で踏んだ本命バグ（2026-08-22）**: 実機で `--restart-io` までして確認したのに
  「最高速度スライダーが3で止まる」（診断タブの LIMITS 表示は正しく 5.00m/s）。
  原因は `SettingsPanel.tsx` ではなく **`store/ui.ts` の `setSettings` アクション**。
  `clampSettings()` が `effectiveRange()`（動的）ではなく**静的な `SETTINGS_RANGE`
  （3.0固定）でクランプしていた**ため、スライダの DOM `max` 自体は動的に5.0まで動くのに、
  `onChange` → `setSettings()` が呼ばれた瞬間に静的値へ巻き戻されていた
  （見た目は動くのにドラッグすると弾かれる、という分かりにくい壊れ方）。
  `clampSettings()` を `effectiveRange(live.link)` でクランプするよう修正
  （`live` は `bus/live.ts` の素の可変オブジェクト。フックではないので `store/ui.ts`
  という非コンポーネントのコードからでも直接読める。**これが今回のようにクランプ関数を
  複数箇所に持つ設計の落とし穴** — 表示側だけ直して、保存側の別の静的クランプを
  見落とすと再現する）
- 診断タブに「車両上限（LIMITS）」の生値表示を追加済み（`DiagGrid.tsx`）。
  今後同種の不具合は、まずここで Pi が実際に受け取っている値を確認してから
  GUI 側のバグを疑うと切り分けが速い
- **★速度メータの目盛りを `settings.maxSpeed` から切り離した（2026-08-22、バンビ指示）**。
  それまで `SpeedGauge.tsx`（RC タブの主計器）は速度制御・トルク制御どちらでも
  `settings.maxSpeed`（RC の速度ダイヤル、ユーザーが下げられる）を目盛りの満尺に
  流用していたため、ダイヤルを下げるとメータの目盛りまで一緒に縮み「実際に出せる
  速度」と食い違っていた。`SpeedGauge.tsx` の `full` を常に `LIMITS.max_speed_m_s`
  （`n.link?.max_speed_m_s`、未受信時のみ `PI_MAX_SPEED_CAP` にフォールバック）に固定。
  この役割が要らなくなったので `SettingsPanel.tsx` の `TORQUE_FIELDS` から
  「速度メータの目盛り」フィールド（`key: 'maxSpeed'` の重複エントリ）を削除。
  `DEFAULT_SETTINGS.maxSpeed` も 0.6 → **3.0**（バンビがそれまで手で設定していた値）
  に更新。`maxSpeed` は今は「速度制御モードの実際の RC 上限」のみを表す
- **★「現在の値を既定値として保存」ボタンを追加（2026-08-22、バンビ指示）**。
  上の `DEFAULT_SETTINGS.maxSpeed` 更新は Claude がスクリーンショットの値を見て
  ソースコードに決め打ちで書いただけで、**実機の現在値を確認したわけではなかった**
  （バンビに指摘されて発覚）。今後同じ往復をしないための恒久対応:
  - `store/ui.ts` に `settingsDefault`（`localStorage` キー
    `surge.driveSettingsDefault.v1`）を新設。「既定値に戻す」・各行の「既定値 X」表示は
    今後ソースコードの `DEFAULT_SETTINGS`（工場出荷値）ではなく、この値を指す
  - `saveCurrentAsDefault()` アクションで `settings` の現在値をここへコピーできる。
    設定パネル上部「既定値に戻す」の隣に「現在の値を既定値として保存」ボタンを追加
  - `DEFAULT_SETTINGS` 自体は据え置き（壊れた localStorage から復旧するときの
    最終フォールバックとして `clampSettings`/`loadSettings`/`loadDefaultSettings` が使う）
- GUIの型は自動生成なので変更したら **`python3 config/gen_msgs.py`** を忘れずに
  （`gui/src/generated/msgs.ts` の `MSGS_SCHEMA` が変わる。App.tsx が食い違いを検出する）
- **まだコミットしていない。** 実機テストで確認してから commit する予定

確定している主要仕様:
- `TELEMETRY` LEN = 66。`STATS` は累積 u32。`CONFIG_SET` は揮発（Flash はステア原点とIMUキャリブのみ）
- `COMMAND` 途絶 100ms で自動ブレーキ / `TELEMETRY` 途絶は 100ms 警告・200ms FAULT
- **`COMMAND` の DISARM 固定は `--allow-arm` で外せる（既定は今も DISARM 固定）**

### 知覚・自己位置推定の方式

2026-08-13〜14 に自作 SLAM（LiDAR主体・スキャンマッチング＋占有格子）と
Disparity Extender（非SLAM反射型）の両方をシムで試作した。**現状の方針は上記の通り
非SLAMを当面の本線とする**が、Phase 3 の最終選択としては確定していない。
議論の背景（オドメトリ単独のデッドレコニングが成立しない理由など）は
`docs/architecture.md` §15-1 とアーカイブ「LiDAR SLAM と地図生成を実装した」節。

### 前輪オドメトリの扱い（どの方式でも共通）

エンコーダは**前輪 = 操舵輪**に付いている。`odom_dist` / `wheel_speed[FL,FR]` は
車輪自身の軌跡に沿った値で、車体中心線方向ではない。**射影は累積値ではなく差分に対して
行う**（`cos(δ)` を累積距離に掛けるのは誤り）。射影済みなのは `speed` だけ（STM32側で処理）。
また **`wheel_speed[FL,FR]` は低速で使い物にならない**（静止時に±12m/s級のスパイクが
7割超のサンプルに出る実測あり。静止判定・スリップ検出には `speed` か `odom_dist` の差分を使う）。

### ★ GPIO18（ブザー）は STM32 と1本の線を共有している（2026-08-17 判明）

`raspi/io/gpio.py` の `Buzzer`（起動音・GUI接続音・E-Stop/FAULTビープ）は GPIO18 を使うが、
**この線は STM32 側のブザー出力と隔離なしで直結**されている（ダイオード等の分離回路は無い）。
両者が同時に押し出す（プッシュプル）と信号が衝突し、RasPi 側からは音が出ない。

- **STM32 ファームウェア側で「鳴らしていない間は Hi-Z（入力）にする」実装済み**（2026-08-17、
  実機で確認・解決）。このリポジトリの管轄外（STM32 ファームウェアは別プロジェクト）なので、
  **STM32 側のこの実装を壊すとまた無音に戻る**。触る人に必ず伝えること
- 音質は STM32 自身のブザー音より粗い（パチパチ・ジリジリしたノイズが乗る）。
  `Buzzer.SETTLE_S`（起動直後の初回発音前に 0.15s 待つ）で軽減したが根絶はしていない。
  共有線・ソフトウェア PWM（gpiozero/lgpio、RP1のハードウェアPWMではない）の両方が疑わしいが未特定
- 今後 STM32 と Pi が同時に鳴らそうとする状況（両方の起動タイミングが重なる等）は
  未検証。気になる症状が出たら真っ先にここを疑うこと

### ネットワーク — Pi は常に STA。AP モードは実装しない（2026-08-08 決定）

**屋外では Wi-Fi ルーターを持ち込み、Pi と PC の両方をそれに STA 接続する。**
モード切替という操作自体を無くした。理由（電波法・切断安全性・NTPなど5点）と
詳細な構成表はアーカイブおよび `docs/architecture.md` §9.1。

| 場面 | 構成 |
|---|---|
| 据置開発 | 有線 Ethernet 直結（`169.254.55.2`） |
| 研究室での走行テスト | STA（`tplink`） |
| **屋外・大会** | **STA（持込ルーター）** |
| ルーターを忘れた/壊れた | **Ethernet 直結**（AP モードではない） |

---

## ★ 未解決・要相談（新しいセッションで最初に確認すること）

### ハードウェア / STM32 側と要相談

- **`steer_actual` が指令 0° で −24.0° に張り付く（未解決）。**
  「遅れ」では説明できない定常オフセット。原点ずれか、Pi/STM32間の舵角符号規約の
  食い違いを疑っている。**Phase 1 のアクチュエータ遅延実測（`steer_cmd_echo` と
  `steer_actual` の差を測る）の前に必ず潰すこと。** 過去記録では逆符号（+22.4°）の
  張り付きも観測されており、単発の配線ではなく構造的な疑いがある
- **STM32 の時刻が +3378ppm 速い（新発見・要相談）。** HSI(内蔵RC, ±1%級)で動いている
  可能性が高い。`TimeSync` の回帰補正で実害は吸収できているが、STM32側に確認する価値あり
- **MD バスの CRC エラー率が3台とも 23〜25%（新発見・要相談）。** Pi⇄STM32 は無傷（crc=0）
  なので STM32⇄MD 側の共通要因（終端・GND・ボーレート等）を疑うべき。
  `comm_ok` のちらつき・GUIグレーアウト表示の原因もこれ。**この率が下がらないと
  `comm_ok=0` → FAULT 遷移の実装（`uart_protocol.md` §5.5 で規定・未実装）を入れられない**
- **原因不明の再起動が2026-08-07に2回（未解決）。** 上記「Pi 実機」節参照。再発時はログ必須

### 自動運転（シムのみで検証・実車一式が未検証）

- **FTG / Disparity Extender / SLAM+raceline / Disparity Pursuit とも実車未検証。** 車輪を浮かせて
  `auto/state` の `reason` と選ぶギャップ／自己位置を先に確認し、`max_speed` を
  0.2m/s 程度まで落として床に降ろす、という段階を踏むこと
- **Disparity Pursuit（`dp`）の `a_lat_max`（既定3.0 m/s²）・`ttc_min`（既定0.6s）は
  未計測の暫定値。** まずシムで衝突なしを確認し、実車で横滑り・急制動の体感から詰めること
- **RACE 段が circuit で開始2秒後に壁へ接触して固着する（SLAM側・未修正）。**
  BUILD（停止）→RACE の受け渡しで Pure Pursuit の遅延補償が怪しいところまで切り分け済み
- **oval が「渦巻き」になる＝ループ閉じが1周目に走らない（SLAM側・未着手）。**
  直す順（① 周回検出のたびに `close_loop()` を走らせる ② 残差の測り方を修正
  ③ マッチの寄せ方を異方的にする）まで特定済み。当面は `explore_laps=1` が回避策
- **split（分離帯コース）は raceline（SLAM）では通らない。** Disparity Extender では
  解決済み（周回できる）だが、SLAM側の自己位置は縦方向の観測が原理的に弱く未解決

### 実装の穴（小さいが未修正）

- `install_services.sh --remove` が `surge-logclean.timer` を消し損ねる
- `tools/deploy.sh --restart` に `surge-planning` が入っていない（`raspi/auto/` `raspi/nav/`
  `planning_node.py` を直したら手で再起動が必要）
- `telemetry_node._serve_log_file` が `.mcap` を全部メモリに載せる
- `LinkTracker` が STM32 の `LOG`(0x04) を数えるだけでどこにも出していない
- `logger_node` の記録対象に `auto/*` が無く、自律走行の判断根拠が `.mcap` に残らない
- テスト実行中に `telemetry_node.py` の `AttributeError: '_auto_mode'` が印字される
  （テスト414件は全部通るが、未初期化属性の疑いとして残っている）
- GUI のカメラが前後とも 14fps（`camera_node` は実機で2台30fps確認済みなので配信側で
  落ちている疑い。切り分け手順はアーカイブ「v0.5 を実車で確認した」節）

### 未実測・未着手（クリティカルパス）

- **`config/vehicle.toml` は `[dynamics]`（アクチュエータの動特性）を除き全項目を実測確定**
  （2026-08-20）。ホイールベース L=0.23m・トレッド=0.155m・車輪半径 0.03m・質量 2.0kg・
  車体外形・センサ取付位置・ステアリングのリンク比（0.5、路面舵角上限 ±30°=0.524 rad）。
  **後輪はダイレクトドライブでギア比の概念が無い**ため、`docs/architecture.md` §15 #2〜#4 はこれで確定
  （`raspi/` 各種 / GUI `PI_MAX_STEER_CAP` / `sim/` に反映済み）。**車輪半径・リンク比は
  STM32 ファームウェアの換算定数にも反映済み**（本人確認）。`speed`/`wheel_speed`/
  `odom_dist`/`steer_actual` はもう暫定スケールではない。残るクリティカルパスは
  `[dynamics]` の実測のみ
- **GUI の車体寸法定数を `config/vehicle.toml` から生成する仕組みを追加**（`config/generate.py`
  → `gui/src/generated/vehicle.ts`。`raspi/proto/generate.py` と同じ考え方）。
  toml を直したら手で GUI 側を書き換えなくてよくなった。編集後は再生成を忘れずに
- 屋外用ルーターの SSID が Pi に未登録（上記「Pi 実機」節）
- `perception_node` / `safety_node` は未実装（`architecture.md` の設計と実装のズレ。
  一覧は `docs/development.md` §11）

---

## 次にやること（優先度順）

1. **`steer_actual` の −24° 張り付きの原因特定**（原点ずれ or 符号規約。他の何より先）
2. **Disparity Extender を車輪を浮かせて実車検証**（`--max-speed 0.2` 程度から）
3. GUI カメラ 14fps の切り分け
4. `[dynamics]`（操舵のむだ時間・1次遅れなど）の実測
5. 屋外用ルーターの SSID を研究室で登録

SLAM 側の課題（RACE固着・oval渦巻き・split未解決）は方針により**棚上げ中**。
再開する入口は「周回検出のたびに `close_loop()` を走らせる」変更（上記）。

---

## `docs/progress_archive.md` の節一覧（時系列）

見出しをそのまま検索語にすれば該当節に飛べる。主なもの:

- Pi 実機（接続トラブルの詳細・電源不足の原因究明）
- STM32 実機との疎通（2026-08-07）
- io_node 中核（実装方針・実証結果）
- ★★ 駆動電源が落ちる件（原因判明の全経緯）
- ★ arm を通す最小実験（`wheel_speed` 実測データ含む）
- ログ記録 `.sfl` / replay_node（形式・実証・`--verify`）
- GPIO E-Stop ハートビート（実測値・STM32側仕様・estop_test.py 詳細）
- カメラの実力測定 / camera_node と共有メモリリング（実測表・踏んだ罠）
- 内部バス・WS・GUI（ZeroMQ の罠・走行指令の安全設計）
- 実機でバスと GUI を通した／GUI から arm を通した／実車をキーボードで走らせた
- MCAP 記録を実装した／実機検証（`RemoveIPC=yes` の罠など）
- 操縦レスポンスの作り直し（もっさりの原因4つの実測）
- プロトコル v0.5 / v0.6 / v0.7 に追従した（変更内容・実車確認の詳細）
- GUI を4タブ・5タブに再編した経緯（レイアウト改訂の試行錯誤）
- Mac 上で動く 2D シミュレータを作った（設計判断・踏んだ罠・検証結果表）
- Follow the Gap による自動運転を実装した
- LiDAR SLAM と地図生成を実装した（BreezySLAM不採用の理由・実装中に踏んだ罠6つ）
- 地図生成アルゴリズムの見直し／スキャン対スキャン／ループ閉じ関連の一連の修正
- ★★ Disparity Extender を実装した（設計上の誤り3つ・実測比較表）
- 壁が虫食いになる件／oval が渦巻きになる件の切り分け
- 全体像と開発ガイドの文書を新設した（2026-08-16・文書と実装のズレの棚卸し）

---

詳細な経緯・実測値・トラブルシューティングは `docs/progress_archive.md` を参照。
