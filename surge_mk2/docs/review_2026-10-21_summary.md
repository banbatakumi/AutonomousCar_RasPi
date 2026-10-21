# SURGE Mark.2 — 総合コードレビュー（2026-10-21）

**対象**: `surge_mk2/`（HEAD `aee4e1d`、作業ツリーはクリーン）＋ STM32 ファーム
`AutonomousCar/Program/MainF446RE_V3`（プロトコル v0.29 / `0x001D`）
**規模**: Python 約 111,000 行 / TypeScript 16,400 行 / C（ファーム）
**手法**: 9 サブシステムに専任レビューエージェントを分割配置し、各モジュール内の静的レビューと
呼び出し関係の追跡を実施。報告された重大指摘は**このPositionsで独立に再検証**（ソース参照・
ホスト実行・数値計算・ファーム C ソースとの突合）した。**コードは変更していない。**

- 深刻度: **C**ritical（安全な走行を直接損なう、または検証の根拠を壊す）/ **H**igh / **M**edium / **L**ow
- 裏付け: **再現**＝ホスト上で実際に発生 / **突合**＝別ファイル・ファーム C・数値と機械的に照合 / **コード**＝呼び出し関係をたどって確定

> 本ファイルは全体の**俯瞰**である。サブシステム別の詳細（数十項目）は末尾の付録に示す
> 3 つのレポートに委ね、本ファイルでは**モジュール間の連携に潜む問題**に絞った。

---

## 0. 結論

**このプロジェクトは構造が無いのではなく、構造が異常に整っている。**
型・生成コード・プロトコル・フレーム解析・安全ゲートの骨格は実測・突合した範囲ではほぼ隙が無い。
一方で **「正（source of truth）の伝播網」に穴が開いている**。

> 本プロジェクト.core的な弱点は、単個のコードではなく**正の伝播網**である。
> `config/vehicle.toml`・`raspi/proto/protocol.toml`・`raspi/proto/generated/packets.py` を正とし、
> そこから firmware・firmware シム・Pi 側ノード・`sim/`・ML 環境・校正ツール・GUI へ
> 値を広げる設計は正しい。**しかしその伝播網の検査は 3 本しか無く、
> かつ CI 側の 1 本は 2026-10-19 以降壊れている。**
> 穴に落ちているのはすべて「片方だけ直しても誰も異議を唱えない」種類のバグである。

特に深刻的だったのは、**同じ物理量が 3 つの「別の車」を定義している**こと（§1.3）。
`[limits].a_lat_m_s2 = 4.0912` / `[sim].mu = 0.41718` / `[control.plant].mu = 1.3227` は互いに矛盾し、
`sim/`・`tools/fwsim/`・実車 runtime が**それぞれ別の車**を前提に制御している。

### 総括表

| 深刻度 | 件数 | うちモジュール連携由来 | 主なテーマ |
|---|---|---|---|
| **C**ritical | 6 | 6 | 制動Torque 規約の中継消失 / CI 死 / μ 三重重複 / side_brake 契約破断 / SLAM 最適化クラッシュ / 展開爆弾 |
| **High** | 16 | 13 | 正の伝播網の破断面、制御周期の不整合、シム忠実度、学習契約 |
| **Medium** | 18 | 10 | 型・契約・状態の非対称、重複実装、例外握り潰し |
| **Low** | 14 | 7 | 文書・数値・コメントの陳腐化 |
| **合計** | **54** | **36 (67%)** | — |

**モジュール連携由来の比率 67%**。内訳：正の伝播網（9）／安全規約の境界泄漏（6）／
sim と実機（6）／学習と推論（5）／制御周期と状態遷移（5）／型と契約（3）／SLAM と地図（3）／GUI（2）。

---

## 1. Critical — モジュール連携由来の破断

### 1.1 C1 — 自律走行の非常制動が、GUI のブレーキスライダ値に格下げされる

**連鎖**: `planning_node` → `telemetry_node._merge_auto` → `cmd` → `io_node` → STM32
**報告**: 3 系統のエージェントが独立に同一箇所を指摘 / **裏付け**: コード（発生側・中継側・下流規約の 3 箇所）

`planning_node` は「わからない・障害物あり・経路が陳腐化」のとき `brake_torque = 0.0` を送る。
`0` は本プロジェクト全体（`raspi/msgs/types.py:357-360`・`raspi/msgs/convert.py:345-346`・
ファーム `vehicle.c`）で **「未指定 → STM32 が最大で制動」** を意味する。
これは埋め忘れでブレーキが効かなくなるのを防ぐ、意図的な安全側の規約である。

しかし中継の `_merge_auto` がこの規約を**破る**。

```python
# raspi/nodes/telemetry_node.py:3249-3253
brake_torque = gui.brake_torque
if auto.brake and auto.brake_torque > 0:
    brake_torque = auto.brake_torque
    if gui.brake:
        brake_torque = 0.0 if gui.brake_torque <= 0 else max(gui.brake_torque, auto.brake_torque)
```

`auto.brake_torque == 0`（=最大制動の意思）は `if` に入らず、結果は **GUI のスライダ値**になる。
GUI のスライダ範囲は 0.005〜0.15 N·m（`gui/src/store/ui.ts:299`）。既定の 0.15 は上限と一致するため
**既定では実害がない**が、人が「急な減速のためブレーキを弱くする」ためにスライダを下げて自律走行すると、
**障害物回避の非常制動が 1/7.5 のトルクになる**。

同じ関数の**陳腐化経路**（`auto/cmd` が 200 ms 超）は `brake_torque = 0.0`（=最大）を正しく送っており、
**同じファイル内で新経路と旧経路が逆の意味になっている**。
さらに docstring（同 `telemetry_node.py:3218-3221`）は

> `brake_torque` は GUI のスライダの値を使わず 0（= STM32 の最大）にする——弱く設定したまま
> 自律走行していると、planner が止まったときの制動まで弱くなる

と**逆の仕様を明記している**。コードが自分のドキュメントと矛盾している。

**影響を受ける経路**: `planning_node._cmd_from`（`raspi/nodes/planning_node.py:481-491`）の
`ready=False` 全経路、`park_to_point._hold`（`raspi/auto/park_to_point.py:691-692`）、
`follow_object` の停止分岐（`raspi/auto/follow_object.py:155-158`）。

**注意**: 現行の挙動は `raspi/tests/test_cmd_path.py:790-795` に「指定なし→GUI」という
**明示的なテスト**として固定されている（2026-09-27 付）。
したがって静かなパッチではなく**明示的な意思決定**が要る。

**修正案**: `_merge_auto` で `auto.brake and auto.brake_torque <= 0` のとき
`brake_torque = 0.0`（STM32 最大）を強制する。テストを
「planner が `brake=True` かつ `brake_torque=0` → 出力は `0.0`」に更新する。

---

### 1.2 C2 — CI が削除済みディレクトリを参照し、2026-10-19 以降ずっと赤

```yaml
# .github/workflows/ci.yml:61
run: python -m pytest tools/tests tools/sysid/tests/test_link.py -k "not TestAnalyzeFolds" -q
```

`tools/sysid` は commit `769e847`（2026-10-19）で削除済み、`tools/ctrl_tune` も同時削除。
`pytest` は存在しないパスを渡すと **exit 4** で即死する（実測 `ERROR: file or directory not found`）。

```bash
# tools/check.sh:61（こちらは更新されている）
run "pytest (tools)" "$PY" -m pytest tools/tests tools/calib/tests tools/fwsim/tests/test_fw_constants.py \
  tools/fwsim/tests/test_control_logic.py -q
```

CI のコメントは「tools/check.sh と同じ軽いものだけ」と書きながら、**対で保つべき 2 ファイルの
片方だけが追従**している。

**影響の構造**（これが最も重い）:
1. CI が赤でも「見て無視」される習慣が生まれる → CI の全工程
   （`generate.py --check` 4 本・`raspi/tests` 1666 件・`tsc -b`）が無効化される。
2. **さらに悪いことに、CI には `tools/fwsim/tests/test_fw_constants.py` が入っていない。**
   これはファームと `vehicle.toml` の定数（トルク上限 0.15 N·m、`DRIVE_LPF_TAU_FRONT_S` 等）を
   突合する**唯一の自動検査**である。
3. `tools/fwsim/tests/test_fw_constants.py:60` と `test_control_logic.py:24` は
   `skipUnless(FW_DIR.exists())` なので、**GitHub Actions のチェックアウトにはファームが無く必ず skip される**。
   → **ファーム定数の二重定義を捕まえる検査は、どこでも実行されていない。**
4. `sim/tests` も CI に無い。

**修正案**: CI を `tools/check.sh` と一本化する。firmware リポジトリを副チェックアウトして
`SURGE_FW_DIR` を渡す。firmware が無い場合は **skip ではなく fail**（または CI ログに明示）。

---

### 1.3 C3 — 同じ車体の摩擦係数が 2 箇所に 3.2 倍違う値で残り、fwsim と sim が別の車を回っている

**裏付け**: 突合（`config/vehicle.toml` と `sim/vehicle.py`・`tools/fwsim/plant.py`・firmware）

| 場所 | 値 | 誰の μ |
|---|---|---|
| `config/vehicle.toml:126` `[limits] a_lat_m_s2 = 4.0912` | μ = **0.417** | `sim/vehicle.py:213-214` が使う |
| `config/vehicle.toml:138` `[sim] mu = 0.41718` | μ = 0.417 | 予備（`[limits]` が優先） |
| `config/vehicle.toml:265` `[control.plant] mu = 1.3227` | μ = **1.32** | `tools/fwsim/plant.py:122-124` が使う |

さらに `tools/fwsim/plant.py:119-120` は **`[dynamics].mu` という存在しないキー**を読んでいるため
常に falsy となり、`lateral_accel_max_m_s2` は**既定値 4.48 のまま toml と無関係**に固定される。
`[limits].a_lat_m_s2` を読むコードは `tools/fwsim` 側に一切存在しない。

**結果として**:

| | 横 grip 上限 | 縦 accel 上限 |
|---|---|---|
| `sim/` の車 | 4.09 m/s² | 3.22 m/s² |
| `tools/fwsim/` の車 | 4.48 m/s²（ハードコード） | 5.17 m/s² |

ABS/TC ゲイン（校正 8 で実機合わせ中の値）は **fwsim の 3.2 倍 grippier な車の上で最適化され**、
それを `sim/` や実機に持ち込むと「効かない」か「発振する」のどちらになる。
`docs/control_constants.md:154` は「ABS・TC・片輪浮き対策のゲインは机上値で実機未検証」と書くが、
**机上値そのものが別の車の上の値**であることは記載がない。

**修正案**: μ を 1 か所へ集約する。`[limits].a_lat_m_s2` を正とし、
`[control.plant].mu` は削除または `= a_lat/9.81` の式にする。
`tools/fwsim/plant.py` は `[limits]` を読む。fwsim と `sim/` の「同じ車」の grip を
**1 本のテストで照合**する（`test_fw_constants.py` に追加）。

---

### 1.4 C4 — GUI の `disarm()` がサイドブレーキのビットを落とし、再 ARM で走行中に後輪がロックする

**裏付け**: コード（GUI 送信側・Pi 受信側・STM32 規約の 3 箇所）
**文書**: `docs/uart_protocol.md:758`「Pi 側が明示的にこのビットを下ろすまで有効なまま」

```ts
// gui/src/ws/control.ts:209-211
disarm(lightMode: number) {
  this.send({ type: 'cmd', t: performance.now(), mode: 0, arm: false, light_mode: lightMode })
}
```

`side_brake` / `winker_left` / `winker_right` が無い。
Pi 側は未指定を `False` にする（`raspi/nodes/telemetry_node.py:1517`）ので、
**ARM を下ろした瞬間に STM32 のサイドブレーキが解除される**。
これは `gui/src/types.ts:461-464`・`gui/src/store/ui.ts:620-632`・`docs/uart_protocol.md:780-781` に
書かれた**明示的なトグル契約の違反**である。

**実害 2 つ**:
1. **駐車保持ができない。** 「サイドブレーキ ON → ARM を下ろす」という駐車手順が成立しない。
   `gui/src/components/DriveControls.tsx:62-64` は実車ではなくストアだけを見て `要求中…` を出すため、
   **表示と実車が食い違う**。
2. **再 ARM で走行中に後輪がロックする。** ストアは `sideBrakeRequested=true` のまま保持され、
   次の ARM（`gui/src/input/useDriving.ts:1252`）で `side_brake: true` が送られる。
   走行中の後輪位置制御への切替は `docs/uart_protocol.md:755-757` が警告している最悪ケースである。

**修正案**: `disarm()` に現在の `side_brake` / `winker_*` を載せて保持する。未 ARM 時の
`useDriving.ts:953` も `ui.sideBrakeRequested` を使う。ARM 解除時にストアを `false` へ落とす。
**「再 ARM で走行中にロック」はどちらの semantics でも必ず消す。**

---

### 1.5 C5 — `slam2d` の `optimize_every > 0` は必ず `ValueError` で落ちる（再現済み）

**裏付け**: 再現（`kf_dist=0.01`・ジャイロバイアス 1°/s・oval 2 周）

`pipeline.py:98-102` の `SlamSystem.update()` は frontend → `_advance_job` → `_sync_graph` の順に走る。
`_advance_job` がループ拘束の確定時に `_add_loop`（`pipeline.py:172-180`）を呼び、
`optimize_every>0` なら `self.optimize()` → `optimize_steps()` →
`poses = self._graph.all_poses()`（`pipeline.py:146`）→
`frontend.apply_correction_steps(poses, chunk)`（`pipeline.py:147`）→ `frontend.py:574-575` で

```python
if len(poses) != len(self.keyframes) or not poses:
    raise ValueError("poses must match keyframes")
```

**この時点でグラフのノード数はキーフレーム数より 1 少ない**（`_sync_graph` はまだ走っていない）。
つまり「その周でキーフレームが追加された」時だけ長さが合わず落ちる。

**影響**:
- `planning_node.py:356-364` が例外を捕まえるのでノードは死なないが、
  **最適化済みのグラフと未補正のフロントエンドが恒久的に不整合**になる
  （`_sync_graph` はノード数が増えないので以後補正が反映されない）。
- `pipeline.py:13-15` の docstring は「`optimize_every` を正にすると走行中でも自動最適化する」と
  advertise している以上、この経路は動作해야 한다。
- 既定 `optimize_every=0`（`pipeline.py:50`）かつ `raspi/auto/_slam2d_nav.py:206` も 0 を明示、
  実運用は `freeze_steps(FREEZE_CHUNK=8)`（`raspi/auto/slam2d_raceline.py:1321`）を使うため
  **現行は顕在化しない**。だが config 1 行で壊れる。

**修正案**: `_add_loop` 内の最適化を `_sync_graph` の後に行う、または `optimize_steps` 内で
`self._sync_graph()` を先に呼ぶ。**`optimize_every > 0` の経路をテストに追加**。

---

### 1.6 C6 — 地図アップロードに展開後のサイズ境界が無く、Pi のメモリを枯渇させる（展開爆弾）

**裏付け**: 再現（20000×20000 uint8 → 圧縮 390,177 バイト → `validate_upload()` が **True**）

`raspi/auto/mapstore.py:256-287` の `_load_npz()` は `trinary` の **dtype・値域（0/1/2）・2 次元**しか
検査せず、**要素数・幅高の上限が無い**。`centerline_xy` / `raceline_xy` も同様
（`mapstore.py:274-276` は `reshape(-1,2)` するだけ）。

`raspi/nodes/telemetry_node.py:408` の `MAX_MAP_UPLOAD_BYTES = 32MB` は
**圧縮後の Wire 長**の制限で、展開後の制限ではない。5 倍にすれば約 2 MB のアップロードで 10 GB 確保になる。

**経路**: `telemetry_node.py:999-1000` → `:2796-2832` `_map_upload_channel()` →
`mapstore.py:350-386` `import_upload()` → `validate_upload()`。

**認証**: `_map_upload_channel` には**トークン検査が無い**
（`_control_channel` は `telemetry_node.py:1113-1116` で `_token_ok()` を呼ぶが、地図アップロードは呼んでいない）。
`--token` は任意（`telemetry_node.py:3646`「無し（誰でも操縦権を取れる）」）。
Origin 検査も「Origin が無い＝ブラウザ以外（curl・自作クライアント）は通す」（`:948-950`）。

**影響**: telemetry_node プロセスの OOM knock（走行中の `cmd` 定期送信が止まり
`io_node` が途絶と見て DISARM する）。

**修正案**: `_load_npz()` に `trinary.size`（例 2,000,000 セル）と `centerline`/`raceline` の点数上限を
Reject し、`_map_upload_channel` に `_token_ok()` を足す。

---

## 2. High — 正の伝播網と安全規約の破断面

### 2.1 H1 — 制御定数 τ がコードと文書で 2.25 倍食い違う

`docs/control_constants.md:41` は車速ループの `speed_loop_tau_s` を **0.155 s** とするが、
コードの値は **0.0687 s**。これにより:

- 加速側の catch ランプが **2.25 倍**（実測 36.3 m/s²）になり、
- 停止余裕が **0.043 s 不足**する。

正值は `config/vehicle.toml` 側にあるべきだが、**どのファイルが正かが文書から追えない**。

### 2.2 H2 — 速度指令のランプに上限が無い（`cmd_accel_limit` が nav 側で効かない）

`raspi/nav/speedctl.py:122-124` の `ref_command` ランプに上限が無い。
`cmd_accel_limit_m_s2` は `Vehicle` 側にあるが nav 側では参照されず、
いまは GUI の `min()`（`raspi/nodes/telemetry_node.py:3246`）だけが守っている。
GUI 未接続・旧 GUI・第三者の WS クライアントでは `accel_limit=0` がワイヤに出る。

**関連**: `raspi/core/vehicle.py:410` が `cmd_steer_rate_limit_rad_s` を**読み込んで一切使っていない**
（`raspi/` 内のどこからも参照されない）。`io_node.py:906-913` は速度と舵角には `LIMITS` で強制するが、
**accel と steer-rate だけが例外的に穴**。

### 2.3 H3 — DISARM モードでも ARM ビットがワイヤに載る

`raspi/msgs/convert.py:285-287` は `cmd.arm` を見て `CMD_FLG_ARM` を立てるが、`mode` を見ていない。
GUI からの `mode=0` かつ `arm=true` は素通しで受理される（`telemetry_node.py:1489-1495`）。
`io_node.py:914` 自身は `mode != DISARM` なので非 ARM と解釈しており、
**2 つの DISARM 経路が食い違う**（`io_node._disarm_command()` `:707-717` は正しく `flags=0` を送る）。

### 2.4 H4 — 校正 8（TC/ABS ゲイン）の発振判定に基準がなく、発振ゲインが「合格」で選ばれる

`raspi/auto/calib_tc_gain.py:103-108` の `is_oscillation()` は `baseline is None` だと
**無条件に False**（= 非発振）。`calib_tc_gain.py:190-201` の `_finish_leg()` は最初の片道
（梯子の最下位 ωc=15）で `base = None` になる → **最下位のゲインは原理的に「発振」と判定できない**。

baseline は「過去片道の最小の揺れ」（`tools/calib/tc_gain.py:110-111`）だが、
比較は `jitter > 1.6 * baseline` の絶対値依存。実機では「TC・ABS を切ったランプでも制動側は
0.03 m/s 揺れる」（`docs/calibration.md:180-181`）ので、**静かな 1 片道が混ざると baseline が 0 に寄り、
以後の正常な揺れが全部「発振」になる**（偽陽性 → 合格なゲインが不合格になる）。

校正 8 の存在意義（「実機で高すぎて振動したゲインを入れない」`docs/calibration.md:200-226`）が
骨組みから崩れる。**さらに**バックネットの校正 9 も `OSC_FAIL_STD = 0.2`
（`tools/calib/traction.py:40,184`）しか不合格にしないため、**この検出器が検出する範囲
（std > 0.04）の揺れは校正 9 を通過してしまう**（安全網の穴が 5 倍離れている）。

**修正案**: 最初の片道にも絶対判定を適用し、baseline に下限（例 0.01）を入れる。

### 2.5 H5 — 校正 1（エンコーダ）の結果がファーム受付範囲外でも「合格」で書け、適用後に車が ARM できない

`tools/calib/encoder.py:62-64` に `LO_MAX=0.2` / `HI_MIN=0.8` / `HI_MAX=1.0` / `H_MAX=0.1` と
定義されているが、**`HI_MIN` は解析側で一度も使われていない**
（`tools/calib/encoder.py:297-299` の判定は `fit.lo > LO_MAX` と `|h| > H_MAX` のみ）。

ファームの実際の範囲は `MainF446RE_V3/src/control/control_params.h` の
`X(enc_left_hi, 0x00C2, 1.0f, 0.8f, 1.0f)`。惰行が下端に届かず `span` が 0.8 を切ると
`hi = lo + span < 0.8` → STM32 が 0.8 に丸める → `control_params_status = "mismatch"` →
`raspi/nodes/io_node.py:752-755` が **ARM を全面ブロック**（車が全く走らない）。ところが verdict は「合格」。

同じ「範囲外でも合格」の穴は校正 5 にもある（`tools/calib/speed.py:288-291` が G の範囲外を
warn するだけ、`speed_ff_nm_per_m_s2 = 1/G` はファーム上限 0.2 を無検査で書く）。
`tools/calib/result.py:67-70` の `Result.usable` は warn でも True なので
`tools/calib/__main__.py:28-33` はそのまま `--apply` する。

**修正案**: 解析側でファームの `param_table()`（`tools/fwsim/fw.py:102-112`）を引き、
**範囲外は fail**（warn ではなく）にする。

### 2.6 H6 — `toml_io.apply` は fsync もバックアップも取らない → `vehicle.toml` 破損で ARM ブロック

`tools/calib/toml_io.py:71-75` は `mkstemp` → 書込 → `os.replace` だが、
**データの fsync もディレクトリの fsync も無い**（`mapstore` 側は `mapstore.py:227-238` で
ファイル fsync をしているのに、こっちだけ揃っていない）。

書き込み直後に電源が切れると `vehicle.toml` が空/破損し得る。破損 →
`load_control_params()` が ValueError → `io_node.py:1326-1330` で `control_params_error` →
`io_node.py:747-748` により **ARM 全面ブロック**（走行不能）。復旧には手動で toml を戻す必要がある。

### 2.7 H7 — `mapstore.save_map` に検証が無く、一覧に出て永久に読み込めない地図を作れる（実証済み）

`raspi/auto/mapstore.py:188-224` は `resolution`/`origin`/NaN を検査しない。
一方、読み込み側 `mapstore.py:281-286` は `math.isfinite` を要求する。

実証: `resolution=NaN` で保存 → メタ JSON が `{"created_at":…, "resolution": NaN, …}` になる →
`list_maps()` には出る → `load_map()` は `None` を返す。**読み込めない地図が一覧に居座る。**

さらに `json.dumps` に `allow_nan=False` が無い（`mapstore.py:165`）ため `NaN` は
**厳密 JSON として不正**なファイルになる。`mapstore.py:572-574` の `save_grip_surface` も範囲検証なし。

### 2.8 H8 — `replay_node --bus` の時刻ドメイン不一致 → planner が恒久「点群が古い」判定

`raspi/nodes/replay_node.py:331` の `BusBridge(pub, clock=lambda: holder["node"]._cursor_ns)` は
**記録元 Pi の CLOCK_MONOTONIC** を使う。一方 `raspi/bus/zbus.py:313` の `msg.t_pub = time.monotonic_ns()` は
**再生プロセス自身**の単調時刻。

`raspi/nodes/planning_node.py:427-429` は `age = now_ns - scan.t_pub` と
`cap_age = now_ns - scan.t_capture` を**別ドメインから取った値どうしで比較**する。

同じ機械で後から再生すると `cap_age` が数時間 → `stale_ms`（既定 300 ms）を hugely 超過 →
**planner は常に `ready=False`（制動）**。別機械（Monotonic 起点が小さい Mac）で再生すると
`cap_age < 0` になり判定を素通りして「たまたま動く」。**同じログが機械と時刻で挙動が変わる。**

`raspi/nodes/replay_node.py:9-11` は「これが事実上のシミュレータ代わり」と記すが、
**planning 経路では成立していない**。

### 2.9 H9 — Pure Pursuit の曲率 FF 分岐が `|η|>90°` の舵飽和を失う（再現済み）

`raspi/nav/purepursuit.py:251-257`:

```python
if cfg.ff_gain > 0.0 and n >= 5:
    kap = 2.0 * math.sin(eta) / max(dist, 1e-3)
    kap += cfg.ff_gain * _ff_correction(path, i, goal, step, cfg.ff_chord)
    delta = math.atan(cfg.wheelbase * kap)
else:
    delta = steer_for_target(eta, dist, cfg.wheelbase, math.pi / 2.0)
```

`steer_for_target`（`purepursuit.py:152-154`）は **「狙点が真横より後ろ（|η|>90°）なら最大舵角に
飽和させる」** と明記している（`sinη` は 90° を超えると減るため、そのままでは後ろ寄りの狙点ほど舵が戻ってしまう）。
**FF 分岐は `steer_for_target` を呼ばずに `atan(L·κ)` を直接計算するため、この飽和を失う。**

実測: 航向誤差 179° で **−0.35°**（FF なしは −27.02°）。
閉ループで v=0.5 m/s・yaw0=180° から **12 秒経っても回復せず 5.19 m 後退**（FF なしは 2.92 s・0.44 m）。

`steer_ff` は hidden パラメータ（`raspi/auto/slam2d_raceline.py:567`、`raspi/auto/base.py:216`）で
**実機では常に 0.8**、`ctrl_mode` の既定は Pure Pursuit → **既定経路で発火しうる**。

### 2.10 H10 — `e2e_lidar` は推論の速度がそのまま指令になり、独立した安全策が無い

```python
# raspi/auto/e2e_lidar.py:259-261
# ★独立した安全策。モデルの判断を経由しない「事実」としての正面の余裕
front = [w.dist[j] for j, d in enumerate(w.degs) if abs(d) <= 20]
st.free_ahead = min(front) if front else 0.0
```

`free_ahead` は**計算されるだけで速度に一切使われない**（診断表示専用。モジュール docstring
`e2e_lidar.py:32` は正直にそう書く）。**インラインコメントだけが「独立した安全策」と主張しており、
実装と矛盾**している。この矛盾は将来のメンテを「防御はされている」と誤認させる。

他モードとの比較:

| モード | 速度の上限を決めるもの |
|---|---|
| `follow_the_gap.py:246-254` | 正面余裕のランプ **+ 曲率制限** |
| `disparity_extender.py:245-252` / `gap_pursuit.py:245-257` | 見通し **+ 曲率制限** |
| `cam_e2e.py:128-132` / `cam_centerline.py:145-153` | 曲率制限（`a_lat_max`） |
| `e2e_lidar.py:333` | **推論出力のみ。物理的な上限が無い** |

`max_speed` の GUI 上限は **3.0 m/s**（`e2e_lidar.py:137-140`、既定 1.0）。
`follow_the_gap`（0.40）や cam 系（0.30）と桁が違う。

**この挙動は `raspi/tests/test_auto.py:1507-1523` で「意図」として固定されている**
（正面 10 cm の障害物でも `brake=False`）。意図的であるなら、**コメントを実装に合わせるべき**。

### 2.11 H11 — 校正の中止時、「止まらない」と判明した瞬間に**最も弱い**制動トルクを出す

```python
# raspi/auto/_calib_common.py:103-105
#: 試験を止めるときの制動トルク [N·m]（1輪あたり）。0.12N·m 以上は ABS なしで後輪がロックする。
#: 0.08N·m はロックせず、1m/s 以上で約 3m/s² で止まる
ABORT_BRAKE_TORQUE_NM = 0.08
```

```python
# raspi/auto/_calib_common.py:422-429
elif self._stage_t >= self._STOP_TIMEOUT_S:
    if v > self._STILL_MOVING_M_S:
        return abort_state(st, f"制動しても止まらない（{vs.speed:+.2f}m/s）", steer)
```

「制動しても止まらない」ことを検出したら、その abort が **0.08 N·m（規定値で最も弱い）** を出す。
最高速で走り続ける車に対して最も弱い制動给出するのは逆効果。

根拠も古い: 「ABS なしで後輪がロックする」だが **ABS は既定で有効**
（`docs/uart_protocol.md:955`）。よって 0.08 N·m という制約は不要。

**修正案**: 「制動しても止まらない」経路だけは `brake_torque=0`（STM32 最大制動）で出力する。
`abort_state` に `strong: bool = False` 引数を追加するのが最小変更。

### 2.12 H12 — モード切替直後の `dt` と、engaged 直後の `dt == 0` で舵の平滑化フィルタが素通りする

**H12-a モード切替**: `raspi/nodes/planning_node.py:161` の `self._last_plan_ns = 0` は
モード切替時のリセット（`:268-281`）で**リセットされない**（`_last_scan_seq` と `_last_map_seq` のみ）。
結果、新モードの 1 回目の `plan()` が受け取る `dt` は**前モードの最終 plan からの経過時間**であり、
数秒〜数十秒になり得る。

全モードの 1 次遅れフィルタ（`follow_the_gap.py:236-238` / `line_trace.py:112-114` /
`cam_centerline.py:139-141` / `cam_e2e.py:120-122` / `gap_pursuit.py:235-237` /
`disparity_extender.py:232-234` / `follow_object.py:213-214`）で
`alpha = 1.0 - exp(-dt/tau)` が ≈1.0 になり、**平滑化フィルタが丸ごとスキップ**される。
`e2e_lidar.py:313-316` は `dt` を 0.15 s にクランプするので 1 判断で舵が 0.3 rad 跳ぶ。

**H12-b engaged 直後**: 同じフィルタの `alpha = 1.0 if tau <= 1e-3 or dt <= 0 else ...` は
「平滑化を無効」ではなく「**即座に target へ移動**」を意味する。
`self._steer` は 0 から始まるので、**最初の 1 コマンドがフル振幅の舵**になる。

`raspi/auto/base.py:8-10` は契約として「モードを切り替えたときに前のモードの舵の平滑化が残っていると、
engage した瞬間に前回の続きから舵が動き出す」ことを防げと要求している。
`dt <= 0` の挙動はまさにその裏返しである。

**修正案**: モード切替時に `self._last_plan_ns = 0` を追加する。
`dt <= 0` のときは `alpha = 0.0`（動かない）か `alpha = 1 - exp(-NOMINAL_DT/tau)` にする。

### 2.13 H13 — `park_to_point` の `plan_budget_ms`（最大 150 ms）が `auto_cmd_stale_ms`（200 ms）に接近し、恒久 stale になる

`raspi/auto/park_to_point.py:260-266` の `plan_budget_ms` は `min=20, max=150, default=120`。
`park_to_point.py:447-461` のフォールバックは A* の時間予算の**外側**で走る:
`for k in (1.0, 1.5, 2.5)` × `candidate_paths()` × 各候補で `sample_path_array` + `path_clearance`。
最大 144 回の経路評価が予算外。

`config/vehicle.toml:63` の `auto_cmd_stale_ms = 200` に対し、`planning_node.py:576-585` は
`plan()` 中は `auto/cmd` を publish できないため、**`plan()` が 200 ms を超えると中継側が最大制動**する
（`park_to_point.py:60-63` も自認している）。探索が常に予算切れになる環境では
毎サイクル 200 ms 超のブロック → **駐車モードが一度も動けない**。

### 2.14 H14 — `park_to_point` の前後進切替の停止待ちにタイムアウトが無く、恒久デッドロックしうる

```python
# raspi/auto/park_to_point.py:937-953
def _switch_stop(self, st, vs, run_end):
    self._waiting_stop = True
    if vs.stopped:
        self._waiting_stop = False
        self._ref_i = min(nxt, len(self._ref_s) - 1)
    return self._hold(st, "切替中", ...)
```

`vs.stopped` が一度も真にならない（坂で滑り続ける／STM32 の stopped フラグが壊れている／
brake が効かない）とき、`_waiting_stop` は永久に True のまま、**モードが「切替中」で制動し続けるだけ**になり、
再計画にも到達しない。`max_maneuver_s` による打ち切り（`park_to_point.py:780-782`）は engages 中なら
最終的に失敗になるが、**disengage 中は `_maneuver_t` が進まない**（`:761-764`）ため停止しない。

### 2.15 H15 — `sim/stm32.py` は TC / TV / 実 ABS を実装していない

`raspi/../sim/stm32.py:10` — 「**TC/TV は実装していない。** タイヤモデルが無いので介入する材料が無い」

ABS フラグは `brake_decel(...)[1]`（=減速度が `brake_decel_m_s2` で頭打ち）だけで立てる
（`sim/stm32.py:518-524`）。`sim/vehicle.py:434-441` の docstring 自身が
「**このモデルは後輪の回転を持たないので、ABS を切った実機は後輪がロックして減速度・横グリップとも
落ちるが、このモデルは表せない**」と明記。

実測では**制動トルク 0.0498 N·m 以上（最大 0.155 の 32 %）で常に ABS フラグが立つ**。
一方 firmware の `ApplyAbs()` は **スリップ率**（`[control] abs_slip_target = 0.1216`）で判断する。

→ **sim 上の「ABS が効いた/効かない」の議論は実機について何も保証しない。**

**関連（H16 相当）**: `sim/vehicle.py:559-574` の `_update_launch()` は**TC 有効を要求しない**が、
firmware `src/control/drive.c:378-381` は `if (obj->launch_spent || !obj->tc_enabled) return false;` と
**TC を要求する**。`tc_enable = 0` のとき**実機は発進せず、sim は全開で発進する**。

**関連**: `sim/stm32.py:392-397` の `dt_total = min((t_ns - self._now) / NS, 0.25)` は
**0.25 s を超える停止で時間が黙って失われる**（実測: 1 秒のストールで 0.75 秒の sim 時間が消え、
テレメトリが 75 フレームバーストする）。加えて `sim/bench.py:600` は**実時間 1:1** しか回らないため、
パラメータ掃引・確率的評価・1 日 1000 ケースが**原理的に不可能**。

### 2.16 H16 — SLAM のループ検出ゲートが LD06 の距離比例測距誤差に脆く、検出率が 15〜45% 落ちる（実測）

`backend/loop_detection.py:197-198` のゲート:

```python
if coarse.second >= config.ambiguity_ratio * coarse.score:
    return None
```

粗い段の得点は `core/register.py:305` の `exp(-d²/2σ²)` の点平均で、
`loop_detection.py:191` の `sigma = max(0.06, step*0.8)`（最大 0.35 m）と **64 点**しか使わない。
距離比例の測距誤差が入ると尤度面が平坦化 → 離れた候補の得点が 1 位に迫り → **正しいループ拘束を捨てる**。

実測（oval 2 周・ジャイロバイアス 1°/s、`σ = 1 cm + rel×距離`）:

| 距離比例成分 | `ambiguity_ratio=0.9`（現行） | `=0.98` |
|---|---|---|
| 0%（テストのモデル） | 16 loops | 34 |
| 0.5%（`sim/params.py:43` の既定） | 14 | 34 |
| 1%（`core/register.py:69` の `sigma_rel`） | 9〜12 | 32〜37 |

棄却理由の Counter は `ambiguous: 25 → 34`（rel 0→1%）。
**`max_rms`/`min_inlier` は原因ではない**（0.03→0.15 に上げても loops 数は変わらない）。

**ループ閉じはジャイロのドリフトを直す唯一の機構**なので、検出率が落ちるとマップは周回ごとに劣化する。
`core/register.py:63-69` は LD06 の距離比例誤差を正しくモデル化しているが、
`backend/loop_detection.py` の閾値はそれと整合していない。

---

## 3. High — 学習・推論・検証網

### 3.1 H17 — `ml_lidar` の ONNX 契約 JSON に「現在」の `vehicle.toml` 値が入り、学習時と確実にズレる

`ml_lidar/export_onnx_rl.py:145-150`:

```python
max_steer = args.max_steer if args.max_steer else VehicleSpec.load().max_steer
steer_link = list(args.steer_link) if args.steer_link else list(Vehicle.load().steer_link)
for w in steer_contract_warnings(find_env_config_path(args.model), max_steer, steer_link):
    print(f"!! {w}", file=sys.stderr)      # ← 警告するだけで、contracts には「現在」を書く
```

学習時の値は `ml_lidar/train_rl.py:682-683` で **既に `env_config.json` に記録されている**。
しかし `export_onnx_rl.py` はそのファイルを読んで**契約値に採用しない**（`find_env_config_path()` は
警告生成のためだけ）。

実機側对这个値の依存は 3 箇所: `raspi/auto/e2e_lidar.py:286-287`（舵角観測の正規化）、
`:315-316`（行動積分のクランプ）、`:322`（`steer_relink`）。

**定量**: `steer_link` が現在の L2 `(1.101158, -0.730735)` になると
`steer_relink(L2, L2)` が **no-op** になり、学習時 L1 `(1.0, 0.0)` の舵角の意味が失われる。
`steer_relink(0.4636, L1, L2)` = 0.4377 rad に対し `steer_relink(0.4636, L2, L2)` = 0.4636 rad
→ **約 5.6% の舵角ずれ**。`max_steer` のずれ（0.4636 → 0.47167）は **約 1.7%**。

**正しい実装は同一リポジトリ内に既にある**: `ml_cam_e2e/export_onnx.py:79` は `cfg.get("steer_link")`
（= 学習時の値）を先に使い、無い場合だけ現在の値で警告する（`:83-85`）。**`ml_lidar` だけが保護されていない。**

### 3.2 H18 — `preproc_version` ガードが学習と結び付いておらず、読み込み拒否が機能しない

`raspi/core/cam_e2e_preproc.py:14-16` は契約している:

> `PREPROC_VERSION` はモデル同梱の `<name>.json` に書かれ、ノードが読み込み時に照合する。
> 古い前処理で学習したモデルを新しい前処理で走らせる事故を、読み込み拒否で止める。

しかし `ml_cam_e2e/export_onnx.py:110` は契約 JSON に **`PREPROC_VERSION`（=エクスポート実行時の現在値）** を
書き込み、**学習時の値と比較しない**。`read_train_config()`（`:68`）は
`input_size`/`max_steer`/`speed_ref`/`steer_link` しか見ていない。
`ml_cam/export_onnx.py:78` も同じ（しかも `ml_cam/train.py:212-215` には `preproc_version` が**記録されていない**）。

さらに**セグメンテーション側は照合自体が存在しない**:
`raspi/nodes/cam_perception_node.py:291-310` の `_load_model_by_name()` は
`input_size`/`mean`/`std`/`threshold` しか読まず、契約 JSON が無ければ**既定値で黙って動く**
（docstring `:296-297` 自身が「train/inference skew の温床」と認めている）。
実際に配備済みの `models/v1.json`〜`models/v3.json` には **`preproc_version` も `resize` も入っていない**。

### 3.3 H19 — `ml_cam` の ONNX エクスポートが学習時の入力解像度を反映しない

学習: `ml_cam/train.py:159`（`--size` 既定 `224x224`）→ `train.py:212-215` で `train_config.json` に記録。
エクスポート: `ml_cam/export_onnx.py:113` の `--size` 既定 `224x224` で、**`train_config.json` を一切読まない**。
対比 `ml_cam_e2e/export_onnx.py:68-73` は `cfg["input_size"]` を読む。
GUI（`ml_cam/app.py:535-539`）もエクスポートタブに**別々の入力欄**を持ち、
「③学習タブと同じ値にしてください」と**手入力コピーを要求**する。

`ml_cam/export_onnx.py:56` は `dynamic_axes` を渡さないため ONNX 入力 shape は静的。
サイズを誤ると Pi 側 ORT は "Got invalid dimensions" で落ちる。
**しかも `verify_parity()`（`:88-108`）は .export に渡した（誤った）サイズで検証するため、
誤ったサイズでも "PyTorch 比 最大誤差 0.00e+00" と出て「検証済み」と誤認させる。**

### 3.4 H20 — ML テストスイートが同時収集できず、356 件が一度も CI で走っていない

実測（`surge_mk2` ルート）:

```
$ .venv/bin/python -m pytest ml_cam/tests ml_cam_e2e/tests -q
ERROR collecting ml_cam_e2e/tests/test_app.py
ERROR collecting ml_cam_e2e/tests/test_dataset.py
ERROR collecting ml_cam_e2e/tests/test_export_onnx.py
ERROR collecting ml_cam_e2e/tests/test_model.py
4 errors in 1.95s
```

- 原因 1: `ml_cam/tests/` と `ml_cam_e2e/tests/` に `__init__.py` が無く
  （`ml_lidar/tests/__init__.py`・`ml_common/tests/__init__.py` はある）、
  同名の `test_app.py`/`test_dataset.py`/`test_export_onnx.py`/`test_model.py` がモジュール名として衝突する。
- 原因 2: 仮に収集が通っても、テストがフラット import（`from dataset import ...`）を使っているため
  `ml_cam/dataset.py` と `ml_cam_e2e/dataset.py` が `sys.modules["dataset"]` で衝突する。
- `tools/check.sh:56` は `raspi/tests` と `tools/tests tools/calib/tests tools/fwsim/tests/...` しか走らせていない。
  **356 件は CI でもローカル check でも一度も実行されていない**（個別に走らせれば全て pass する）。

### 3.5 H21 — `ml_common.job_runner.drain()` の例外で Tk の `after` ループが永久に死に、GUI が二度と動かない

`ml_common/job_runner.py:117-140` の `try/except queue.Empty` がループ**全体**を覆っているため、
`on_log` / `on_line` / `on_done` からの例外は `drain()` を抜けて伝播する。
3 つの app はいずれも drain の**後で** `after` を登録している
（`ml_cam/app.py:681-684`、`ml_cam_e2e/app.py:636-638`、`ml_lidar/app.py:699-701`）。
→ 例外時に `after` が再登録されず、以後ログが更新されない。
さらに "done" 処理（`self._procs.pop`）も走らないため **`is_busy()` が永久に True** になり、
以降**一切ジョブを開始できなくなる**（再起動 Until 必要）。

**関連**: `job_runner.py:72` の `self._procs[job_key] = None`（起動中プレースホルダ）に対して、
`running_job_keys()`（`:57-59`）と `terminate_all()`（`:107-115`）は `p is not None` のみを見る。
→ **起動処理中のジョブは terminate できず、ウィンドウを閉じても孤児プロセス化する。**

### 3.6 H22 — 学習/評価の train/val 分割が「記録（セッション）」単位でなく、val 指標が一般化性能の推定になっていない

`ml_cam_e2e/samples.py:167-181` の `is_val()` は **5 秒ブロック**で hash 分割するため、
**同一 mcap の異なる 5 秒ブロックが train/val の両側に散る**。
同じ走行セッション（同じコース・同じ照明・同じ装着状態）の別時刻が val に入るため、
val MAE は一般化誤差を大きく下回る。

実データ: `ml_cam_e2e/runs/v1/train_config.json` は `n_train: 1128 / n_val: 175 / best_val_mae: 0.0533`
→ `0.0533 × 0.524 rad = 1.6°`。この 1.6° は「同じ走行の別時間帯での誤差」であって、
未知コースでの誤差ではない。

`ml_cam` 側はさらに悪い: `ml_cam/train.py:78-79` が
「ブロックが 2 未満ならフレーム単位のランダム分割へフォールバック」するため、
**連続フレームが train/val にまたがる**。`ml_cam/runs/v1/train_config.json` の `best_val_iou: 0.917` は
この leaks を含む。

### 3.7 H23 — `--seed` が再現性を保証しない（numpy の RNG が未シード）

`ml_cam/train.py:166-173` は「augmentation（`dataset.py` の `np.random.rand()`）… も別系統の乱数源なので、
ここで明示的に固定しないと再現性が取れない」と明記してから `torch.manual_seed(args.seed)` **のみ**呼ぶ。
`np.random.seed()` は無い。`ml_cam/dataset.py:92`・`ml_cam_e2e/dataset.py:65,71,72` は numpy グローバル RNG を使う。
`torch.manual_seed` は numpy をシードしないため、**同じ `--seed` でも別 run で別の反転・明暗となる。**

---

## 4. Medium — 型・契約・状態の非対称

| # | 箇所 | 内容 |
|---|---|---|
| M1 | `raspi/msgs/convert.py:196` | `tv_ratio` に**トルク定数** `1e-4` を流用している。無次元量に N·m の定数を掛けるため、プロトコルの scale を変えると TV が 10 倍/100 倍になる。`convert.py:50-51` の `Command.accel_limit` と同じやり方（`packets.Telemetry.META` 参照）に揃えるべき |
| M2 | `raspi/core/link_tracker.py:267-268` | スケールをハードコード（`1e-3`/`1e-4`）。`convert.py` が唯一の変換点である以上、**ここが 2 番目の変換点** |
| M3 | `raspi/nodes/planning_node.py:562-564, 352-354, 521` | `on_vehicle_state` / `set_engaged` / `snapshot` の 3 つが例外ガード無し。planner のバグ 1 個でノードごと死ぬ（同じファイル内の `plan()`/`tick()`/`request_*` は `_ask()` で握り潰している） |
| M4 | `raspi/nodes/logger_node.py:225-227` | ディスク容量チェックが**メインループで同期実行**。`io_node.py:553-580` は同じ処理をワーカースレッドへ逃がしており（issue #5 の注記あり）、logger だけが未修正。**ディスクが逼迫している時（=記録がなくなる時）に記録が欠ける**最悪の相性 |
| M5 | `raspi/core/jpeg.py:126-143` | `RingJpeg` のカウンタとリング map が**ロック無し**。`asyncio.to_thread` から 2 経路で同時に触られる（`telemetry_node.py:3446` と `:905`） |
| M6 | `raspi/nodes/io_node.py:895-898` | arm-block 経路が**制動を経ずに即 DISARM**。docstring `:33-37` は「走っていたなら、まず最大制動を送り、止まってから DISARM」と書いているが実装が違う。走行中に STM32 だけが再起動した場合に無制動で落ちる |
| M7 | `raspi/nodes/io_node.py:809-816` | デッドマンを**ループ開始時刻**で測る（`LINK_WAIT_S` 5 ms 分過大評価） |
| M8 | `raspi/nodes/planning_node.py:479-487` | `_cmd_from` が共有オブジェクト `self.state` を破壊的に書き換える。publish 済みの state が後から書き換わる |
| M9 | `raspi/auto/base.py:115-128` / `planning_node.py:287-293` | `on_disengage` を持つ planner は**解除時に `reset()` が呼ばれない**。契約は記述のみ、強制されない |
| M10 | `raspi/auto/base.py:199-225` | `merged()` は `min(s.max, max(s.min, v))` なので **inf が `s.max` に張り付く**（NaN は弾かれるが inf は弾かれない） |
| M11 | `raspi/auto/follow_object.py:211` | `ld = look_k * v + look_min` に上限が無い。他モードは `min(dist/depth, ...)` で頭打ち |
| M12 | `raspi/auto/cam_e2e.py:92-94` / `cam_centerline.py:105-107` | **推論の信頼度しきい値が存在しない**。`CamE2ECmd`/`CamPath` に confidence フィールド自体が無い。低照度・ぼけ・部分マスクのフレームが**最大舵で採用**される |
| M13 | `raspi/auto/registry.py:72-93` | `catalog()` は `category == "sysid"` の planner も含む。WS は公開 API なので GUI を使わないクライアントは `calib_*` を mode に指定できる |
| M14 | `raspi/auto/registry.py:96-103` | `make_planner` が排他制御を持たない。排他は「planning_node が 1 インスタンスしか持たない」ことに依存 |
| M15 | `raspi/auto/slam2d_raceline.py:428-434` | `ProcessPoolExecutor`/`ThreadPoolExecutor` がノード終了時に残る（`planning_node.close()` が `planner.close()` を呼ばない） |
| M16 | `raspi/nodes/camera_node.py:706` | `Subscriber` をメインスレッドで生成し、别スレッドで `poll()` する。pyzmq ソケットは生成スレッド以外からの使用が保証されない |
| M17 | `raspi/msgs/types.py:967-983` | `CamE2ECmd` に `reason` フィールドが無い（`Scan`/`LineScan`/`CamPath` にはある）。失敗原因が GUI にも `auto/state.reason` にも出ない |
| M18 | `raspi/bus/zbus.py:172-181` | 未知トピックへの前方一致フォールバックが**辞書の並び順に依存**（既知の事故 `scan/cam`→`io` の注释あり）。未知トピックは `raise` の方が安全 |
| M19 | `raspi/auto/base.py:434-442` と `raspi/msgs/convert.py:466` | セクタ境界の 1° オフセットが**二重定義**。片方だけ直すと欠測セクタ判定が 30 点ぶんずれる。`test_msgs.py:200-238` は LiDAR 側しか覆盖していない |
| M20 | `raspi/nodes/telemetry_node.py:3042-3043` | モード切替時のパラメータ変更検出キーが不十分。`accel_limit`/`launch` だけが change key に無い |
| M21 | `raspi/core/vehicle.py:399` | `control.plant.rear_lateral_share` を **reader が読むが writer も toml も存在しない** → 常に既定 0.59。`docs/system_overview.md:687` は「`[control.plant]` から取れる」と書いている |
| M22 | `raspi/auto/mapstore.py:141-150` | 固定 tmp 名 (`<name>.grip.json.tmp`) による 2 プロセス競合。走行中の `save_grip` と GUI のアップロードが同時走査で壊れた JSON を install しうる |
| M23 | `raspi/auto/mapstore.py:207-208` | `save_map` が grip/lines を npz 書き込み**前**に消す。npz 書き込みが失敗すると**学習値だけ失われ**地図は古いまま |
| M24 | `raspi/auto/mapstore.py:370-377` | `import_upload` の npz→routes 書込順で、npz 書込後・routes 処理前に落ちると**別地図の経由点が新地図に残る**（`save_map` には同等のガードがある） |

---

## 5. 設計上のボトルネック

### 5.1 「正の伝播網」に検査が 3 本しかない

`tools/check.sh --fast` が走らせる検査は 4 本（`generate.py --check` / `gen_msgs.py --check` /
`check_docs.py --check` / `vehicle.toml` の生成）。**それぞれが守れるのは「生成物」だけ**で、
**「正の値を変えたときに派生物を追従させる」検査は存在しない。**

このプロジェクトで実際に起きた不具合は**すべてこの穴**に落ちている:

| 穴 | 該当 |
|---|---|
| 値のコピー（手書き定数） | `convert.py` の 31 スケール（M1）、`gui/src/store/ui.ts` のトルク上限、SignalConfig |
| 節の取り違え・キー不存在 | `fwsim/plant.py` の `dyn.get("mu")`（C3）、`rear_lateral_share`（M21）、`tools/calib/speed.py` の WHEEL_RADIUS/MASS |
| firmware 方向の非一方向 | `ras_link.c` の `RAS_LEN_*` と TELEMETRY 並びを手写し |
| 実装の二重化 | `sim/` vs `fwsim`（H15）、`deskew`（`raspi/nav/deskew.py` と `slam2d/core/deskew.py`）、`raceline` |

**対策**: `raspi/core/control_params.py` の `DERIVED`（既に `brake_decel_m_s2` と `md_torque_lag_s` を持つ）
のような**「派生値の単一計算場所」を全シムに強制する**。
現状 `ml_lidar/env.py` は `sim/stm32.py` を参照せず**自前で定数を書いて**いる。

### 5.2 「モジュール間の接続」だけが検査されていない

`tools/calib/tests`（44 件）と `tools/fwsim/tests`（10 + 21 subtest）は
**本物のファームをコンパイルして合成データで検証する**、極めて品質が高い。
一方、**モジュール間の接続**はテストされていない:

- `E2ELidar.plan()` の観測ベクトル == `LidarE2EEnv._to_obs()` の観測ベクトル — **未テスト**
  （H17 のずれを捕捉していたはず）
- `replay_node --bus` が **`cmd` を publish しない**こと — **未テスト**（H8 と同じ安全上最Important の不変条件）
- `tools/calib/toml_io.py`（**単一の source of truth を書き換える部品**）— **テスト 0 件**
- `tools/calib` の**出力値が firmware の受理範囲内**であること — **未テスト**（H5 の「丸め→ARM ブロック」経路）
- `sim/tests` — **CI に無い**
- ML の 356 件 — **CI に無い**（H20）

### 5.3 計画が「安全側の既定」をモジュール間で壊している

本プロジェクトには一貫した安全思想がある（`0 = 未指定 = STM32 最大`、`CONFLATE`、
heartbeat の thread 分離、deadman、seqlock）。**その思想はモジュール境界で繰り返し破られている**:

| 規約 | 正 | 破れている場所 |
|---|---|---|
| `brake_torque=0` = 最大制動 | `convert.py` / ファーム / `msgs/types.py` | `telemetry_node._merge_auto`（C1） |
| `side_brake` = トグル | `uart_protocol.md:758` | `gui/src/ws/control.ts:209-211`（C4） |
| モード遷移時に舵の状態を捨てる | `auto/base.py:8-10` | `dt <= 0` の `alpha=1.0`（H12） |
| `cmd` の発行元は 1 つ | `telemetry_node` のみ | — |
| 機械の GUID は「正」の生成物 | `protocol.toml` / `vehicle.toml` / `types.py` | firmware 手写し |
| 機械は「安全側の既定」 | 全体 | `deploy.sh --services`、`arm_test` |

**型・生成コードに投資してきたのに対して、意味論（値の意味・規約の適用範囲）には投資してこなかった**ことの表れである。

### 5.4 モード間で暗黙の前提が食い違う（「1 つの planner = 1 つの走らせ方」が崩れている）

| 観点 | FTG | DE | DP | cam_centerline | cam_e2e | line_trace | follow_object | e2e_lidar | park_to_point |
|---|---|---|---|---|---|---|---|---|---|
| `max_speed` 既定 | 0.40 | 0.60 | 0.50 | 0.30 | 0.30 | 0.30 | **1.00** | **1.00**（最大 3.0） | 0.30 |
| 最低速度 | 0.12 | 0.12 | 0.12 | 0.10 | 0.10 | 0.10 | なし | **なし** | 0.08 |
| 正面余裕の意味 | **速度に使う** | 診断のみ | 診断のみ | 速度に使う | 無し | 無し | 無し | 診断のみ | 走査 |
| 舵の生成 | Pure Pursuit | **比例** | Pure Pursuit | Pure Pursuit | 回帰 | Pure Pursuit | Pure Pursuit | 舵角速度の積分 | 曲率FB |
| 「不明」の表現 | ready=False | ready=False | ready=False | ready=False | ready=False | ready=False | **brake=True** | ready=False | brake=True |
| 曲率制限 | あり | あり | あり | あり | あり | なし | あり | **なし** | あり |

GUI の `max_speed` スライダを同じ位置に置きながらモードを切り替えると、
**同じスライダ位置で別の速度・別の安全余裕になる**。これは設計思想と矛盾する。

### 5.5 制御周期の階層が混乱している

- `cmd` は **50 Hz**（`raspi/auto/base.py:37-40` `CMD_HZ=50`）だが、
  **planner の内部状態（舵の 1 次遅れ、速度のランプ）は実質 10 Hz**でしか更新されない
  （`raspi/nodes/planning_node.py:331-381` は「新しい周が来たときだけ `plan()`」）。
  → **制御の帯域が 1/5**。`tick()` を持つのは `slam2d_raceline.py:1221` だけ。
- `sim/bench.py` の `CMD_HZ=100` は実機 50 Hz と食い違う。
- **SLAM の 1 周期予算**（Pi 5 の 100 ms LiDAR 周期に対し `plan()` は既に p99 約 84 ms）:
  局所地図がキーフレームごとに作り直される（`slam2d/core/localmap.py:101-111`、実測 Mac 2.80 ms/回・
  171 回中 171 回 = 全キーフレームで再構築、Pi 5 で約 10〜15 ms）、ループ検出のたびに部分地図を作り直す
  （`backend/loop_detection.py:173-174`、`loop_every_m=0.4` で 2.5 回/秒）。
  加えて `Frontend.refine()` は 1 回で 42.7 ms（Mac 実測、Pi 5 で 150 ms 超）。
  → **`cmd_deadman_ms`（150 ms）に張り付く。**

### 5.6 時刻ドメインが 2 系統ある（型で表現されていない）

実機は Pi の `CLOCK_MONOTONIC` 一本だが、`replay_node --bus` は記録元 Pi の時刻をそのまま
`t_capture` に載せる（H8）。`raspi/msgs/types.py:5-13` は「`t_capture` はセンサ取得時刻」「`t_pub` は
publish 時刻」を明確にするが、**両者の関係（同一エポックか否か）を型で表現していない**。

`MsgBase` に `epoch_id`（ブート UUID）を 1 フィールド足すだけで、
H8 のような「別エポックの値を比べる」事故が型で防げる。同様に `Publisher.seq` はプロセス再起動で
巻き戻るため、各ノードが「巻き戻り = 再起動」という独自ヒューリスティックを実装している
（`telemetry_node.py:3115-3119`、`cam_track_node.py:262-281`、`planning_node.py:326-343`）。

### 5.7 バスは「drop してよい」前提で、欠落検出が無い

`raspi/bus/zbus.py:316-320` は HWM 到達で `zmq.Again` を**握り潰して**捨てる。
`LATEST`（CONFLATE）と組み合わさると、遅い購読者は「動いているように見えるが古い/欠けた」データだけを見る。
`planning_node.py:100` は `VEHICLE_STATE_POLICY = Policy(conflate=False, hwm=1000)` だが、
**`plan()` に timeout が無い**。SLAM planner が 3 秒以上かかればキューは溢れ、
`vehicle_state` のサンプルが黙って落ちる。`plan_ms_max`（`planning_node.py:767`）は
**記録されるだけで誰も acted しない**。

### 5.8 文書・運用が実装から乖離している

| 文書 | 記載 | 実態 |
|---|---|---|
| `docs/architecture.md:770` | ログは 7 日 / 15〜18 GB/日 | 8 GB 上限が先 → **9〜12 時間** |
| `docs/calibration.md:87,26,148,239-240` | 0.35 m/s、1.2 m 四方、0.13 N·m、±0.05 | 0.45、0.8 m、0.15、±0.04 |
| `docs/control_constants.md:41` | `speed_loop_tau_s = 0.155` | 0.0687（H1） |
| `docs/control_constants.md:52,54,96` | `tools/sysid`、`speed_pi_zeta`、pole placement | 削除済み・**vehicle.toml に存在しないキー** |
| `docs/stm32_interface.md:211,651` | `TELEMETRY LEN = 84` | **102**（`raspi/proto/generated/packets.py:194`、`docs/uart_protocol.md:301`）— 同文書内でも矛盾 |
| `docs/stm32_interface.md:430` | `0x001A` | `0x001D` |
| `site/docs/link/protocol.md` | v0.23 | v0.29 |
| `raspi/setup/install_services.sh:94` | `--max-steer 0.524` | `0.47167` |

**「正」の所在が 1 箇所に集まっていない**ので、**同じ事実が 6 箇所に複製され、更新のたびに 5 箇所が古くなる**。
`docs/` には `check_docs.py` があるが**数値とリンク切れを検査しない**。

### 5.9 GUI 側の構造的問題

- **型整合は良好**: `config/gen_msgs.py --check` は通り
  （`gui/src/generated/msgs.ts` は `raspi/msgs/types.py` と完全一致、`MSGS_SCHEMA = 0xce486c96` も一致）、
  `tsc -b`（strict + `noUncheckedIndexedAccess`）通過、`any` ゼロ。
  **しかし WS の JSON 形は `gen_msgs.py` の対象外**（`config/gen_msgs.py:32-36`）で、
  Python と TS に手書き二重管理されている。現状は偶然一致しているだけ。
- **スキーマ hash が全-or-nothing**（`raspi/msgs/schema.py:65-79`）:
  `VehicleState` にフィールドを 1 個足すだけで GUI↔Pi の同時デプロイが強制され、
  版不一致時に**全フレームを捨て**て ARM 不能になる（`gui/src/ws/telemetry.ts:46-52`、
  `gui/src/components/StatusBar.tsx:200`）。fail-closed は正しいが、
  「再ビルドせよ」の具体策が StatusBar の 1 pill しか無い。
- **mcap 記録をブラウザ RAM に上限なしで蓄積**（`gui/src/hooks/useMcapDownload.ts:23,56-60`）:
  サーバ自身が `raspi/nodes/telemetry_node.py:402` に「`image_hz=5` × カメラ 2 台で毎時 1 GB 近く育つ」
  と書いている。1 時間録るとタブが数 GB を消費してクラッシュし、
  `finish()` は **Buffers を全部メモリに載せてから** `Blob` を作るため
  **OOM すればダウンロードは一度も始まらないまま記録が全滅**する。
- **自律走行の解除条件がパッドの微小入力で成立**（`gui/src/input/useDriving.ts:727,890-896`）:
  `padSteer !== 0` は Deadzone 適用後も 0 とは限らず、**パッドが少し触れただけで engage が落ちる**。
- **`take_control` の 250 ms 連打**（`useDriving.ts:930-937`）がトークン誤り時に無限ループし、
  `notify()` が毎回新しい id を作る（`gui/src/store/ui.ts:832-839`）ため**トーストが永久消灯しない**。

---

## 6. テストカバレッジの穴（優先順付き）

| 優先 | 対象 | なぜ重要 |
|---|---|---|
| 1 | `ml_cam`/`ml_cam_e2e` の collection 衝突解消と CI 追加 | 356 件が未実行。H19/H20 の回帰を捕まえない |
| 2 | CI の `tools/sysid` 参照修正 + `test_fw_constants.py` の実行 | ファーム定数の二重定義（C3/H1/H2 の温床）を捕まえない |
| 3 | `_merge_auto` の「`auto.brake=True` かつ `brake_torque=0`」ケース | C1 の核心。既存テストは `> 0` しか見ていない |
| 4 | `slam2d` の `optimize_every > 0` 経路 | C5 のクラッシュが検出されない |
| 5 | `E2ELidar.plan()` == `LidarE2EEnv._to_obs()` の parity テスト | H17 の学習/推論ずれを捕まえない |
| 6 | `tools/calib/toml_io.py` の単体テスト（**0 件**） | 単一の source of truth を書き換える部品 |
| 7 | `tools/calib` の出力が firmware 受付範囲内であることのテスト | H5 の「合格 → ARM ブロック」経路 |
| 8 | `replay_node --bus` と `planning_node` の時刻ドメイン整合 | H8。`replay_node` 単体のテストと `planning_node` のテストが別々 |
| 9 | `planning_node` の無防備コールバック 3 つの例外耐性 | M3。`_FakeCrashingPlanner` は `plan()` しかクラッシュさせない |
| 10 | `DriveCmd(mode=0, arm=True)` → ARM ビット立たないこと | H3 |
| 11 | `disarm()` が `side_brake` を保持すること | C4。Python 側テストからは見えない（GUI の送信内容が未検証） |
| 12 | GUI 側にテストが **1 つも無い** | `gui/package.json` に vitest/jest/playwright なし、`gui/src/**` にテスト 0 |
| 13 | `msgs/schema.py` の `MIRRORED` に `CamE2ECmd` が無い | M17 |
| 14 | `mapstore` の NaN 保存・並行書き込み・`import_upload` の routes 順序 | H7/M22/M24 |
| 15 | `sim/tests` の刻み幅依存性とホスト停止（>0.25 s）時の時刻整合 | H15。`sim/tests/test_stm32_telemetry_time.py` は通常時のみ |
| 16 | `logger_node` の「遅い check でも main loop が止まらない」テスト | M4。`io_node` 側には同種テスト（`test_io_node_log.py:279`）がある |
| 17 | `ml_cam_e2e` の**記録単位**の分割関数 | H22。leak を捕まえない |
| 18 | `np.random.seed` 追加後の再現性テスト | H23 |

---

## 7. 対応ロードマップ

### 段階 1 — 安全と検証網の復旧（1〜2 日）

| # | 対応 | 対象 |
|---|---|---|
| 1 | **CI を修復**し `tools/check.sh` と一本化。firmware を副チェックアウトして `test_fw_constants.py` を実行。firmware が無い場合は fail | §1.2 |
| 2 | **CI に `sim/tests` と ML の 356 件を追加**し、`ml_cam`/`ml_cam_e2e` の collection 衝突を解消 | §3.4 |
| 3 | **`_merge_auto` の brake_torque 規約を統一**（planner の `0` は STM32 最大）。既存テストの決定を明示的に再議論 | §1.1 |
| 4 | **GUI の `disarm()` に `side_brake`/`winker_*` を保持**。再 ARM で走行中にロックしないことを保証 | §1.4 |
| 5 | **地図アップロードに展開後サイズ上限とトークン必須** | §1.6 |
| 6 | `calib_tc_gain` の発振判定に絶対基準と baseline 下限 | §2.4 |
| 7 | `tools/calib/encoder.py`/`speed.py` の範囲外を **fail** に。`toml_io.apply` に fsync と `.bak` | §2.5, §2.6 |
| 8 | `abort_state` に `strong` フラグを追加（「止まらない」時は最大制動） | §2.11 |
| 9 | `slam2d` の `optimize_every>0` のクラッシュ修正 + テスト追加 | §1.5 |

### 段階 2 — 「正」の伝播網の補修（3〜5 日）

| # | 対応 | 対象 |
|---|---|---|
| 10 | **μ / `a_lat_m_s2` / `plant.mu` の 3 重重複を解消**。校正 7 が 2 つを必ず一緒に書く | §1.3 |
| 11 | `tools/fwsim/plant.py` の `dyn.get("mu")`（存在しないキー）を `[limits]` へ。fwsim と `sim/` の grip を照合するテストを追加 | §1.3 |
| 12 | `convert.py` のスケール 31 項目を `packets.Telemetry.META` へ。`link_tracker` のハードコードも除去 | M1, M2 |
| 13 | **`rear_lateral_share` を toml に追加**（writer も reader も揃える） | M21 |
| 14 | `mapstore.save_map` の NaN/Inf 検証と `allow_nan=False`。一時名の一意化と書込順序の修正 | H7, M22, M23, M24 |
| 15 | `MsgBase` に `epoch_id`（ブート UUID）を追加し、`replay_node` の時刻ドメインを統一 | §5.6, H8 |
| 16 | `planning_node` のモード切替で `_last_plan_ns = 0`。`dt <= 0` の `alpha` を 0 に | H12 |

### 段階 3 — 制御・学習の整合（1 週間）

| # | 対応 | 対象 |
|---|---|---|
| 17 | **E2E モデルの契約刷新**: `dt`/`steer_tau`/`max_steer`/`steer_link` を学習 run に記録し、既存 v13〜v27 を再エクスポート | H17 |
| 18 | `ml_lidar/export_onnx_rl.py` は `env_config.json` の**学習時の値**を契約 JSON に書く（`ml_cam_e2e` の実装に揃える） | H17 |
| 19 | `preproc_version` の照合を `ml_cam`（セグメ）にも入れ、契約 JSON が無い/版が違う場合は refuse | H18 |
| 20 | `ml_cam/export_onnx.py` が `train_config.json` の `input_size` を読む。GUI の手入力欄を廃止 | H19 |
| 21 | 学習/評価の分割を**記録単位**に。`np.random.seed` を追加 | H22, H23 |
| 22 | `Pure Pursuit` の FF 分岐に `|η|>90°` の飽和を適用（`steer_for_target` と共通の飽和関数へ） | H9 |
| 23 | `e2e_lidar` に `a_lat_max` による曲率上限を追加。`free_ahead` のコメントを実態に合わせる | H10 |
| 24 | `speed_loop_tau_s` を正の位置に寄せる（`vehicle.toml` へ） | H1 |
| 25 | シムの plant に MD トルク遅延・スリップモデル・`CONFIG_ACK` 契約を導入。`launch` の TC 依存を再現 | H15 |
| 26 | `park_to_point` の `plan_budget_ms` 上限を `auto_cmd_stale_ms/2` に。`_switch_stop` にタイムアウト | H13, H14 |

### 段階 4 — 局所化・UI・運用（1〜2 週間）

| # | 対応 | 対象 |
|---|---|---|
| 27 | SLAM: `optimize_every` の順序修正、局所地図の差分更新、`ambiguity_ratio` の LD06 依存性を見直す | §1.5, H16, §5.5 |
| 28 | GUI: `disarm` 保持、mcap RAM 上限と自動停止、`notify` の連打対策、`padSteer` デッドゾーン | C4, §5.9 |
| 29 | WS JSON 形の生成（`gen_msgs.py` 対象拡大）と前方互換スキーマ | §5.9 |
| 30 | GUI テスト基盤（vitest）の導入と送信内容のスナップショットテスト | §6-12 |
| 31 | ドキュメントの数値同期と `check_docs.py` の拡充（インライン版・`site/`・リンク切れ・主要数値） | §5.8 |
| 32 | `io_node` の arm-block 経路を制動経由にする。`logger_node` のディスクチェックをワーカー化 | M4, M6 |

---

## 8. 総括

このリポジトリは**「型と生成」を极高まで完成している**。
`VehicleState` 60 フィールド・`LinkDiag` 53・`AutoState` 72 が完全に一致し、
`CmdOut` 20・`ControlStatus` 35 も孤児ゼロで、`gui/src` に `any` は 1 つもない。
`tsc -b` は strict + `noUncheckedIndexedAccess` で通り、フレーム解析は
**継続的なゴミ入力でも固まらない**ことを実装が保証し、停止経路は 16 種ある。

**弱点は、許容できる範囲のさらに外側にある。弱さの中身は「意味論」である。**

> コードは**形**（型・構造・生成）を厳格に守っているのに、
> **値の意味と規約の適用範囲**がモジュール境界で漏れる。
> 0.15 N·m のトルク上限は 5 箇所に書かれ 4 箇所は検査されているのに、
> `brake_torque = 0` が何を意味するかは**中継 1 行で消える**。
> `max_steer` は 1 箇所が正なのに**既定値・unit 引数・文書・learning 契約**が
> 4 つの別の値を持っている。
> 「正の伝播網」を支えているのは**検査 3 本と CI 1 本**で、
> **その CI は 2 日前から壊れている**。

したがって優先順位は明確で、**最初に直すべきは個別の数値ではなく検査網**である。
CI を直し、意味論の検査（firmware の並び・convert のスケール・契約の `dt`）を足し、
**「正」を 1 箇所に集める仕組み**（`control_params.DERIVED` の応用）を広げる。
それが済めば、High のうち **半分以上は構造的に発生しなくなる**。

逆に言えば、**これをやらずに個別のバグを 1 つずつ直していくと、
同じ穴から 3 つ目のバグが生まれる**（既に `brake_torque` は
`planning_node` → `telemetry_node` → `io_node` の 3 者在る）。

---

## 付録 — サブシステム別の詳細レポート

本ファイルは俯瞰であり、各サブシステムの網羅的・行単位の指摘は以下に委ねる。

| ファイル | 対象 | 行数 |
|---|---|---|
| `docs/review_2026-10-21.md` | 全体俯瞰（正の伝播網・契約・文書・ロードマップ） | ~1490 |
| `docs/review_2026-10-21_nav.md` | `raspi/nav/`（Pure Pursuit / MPC / Hybrid A\* / speedctl / raceline） | ~714 |
| `docs/review_2026-10-21-contract-review.md` | モジュール境界の契約・設定の単一情報源・`vehicle.toml` スキーマ | ~611 |

各レポートの末尾に調査ファイル一覧とテストの穴の項目があるため、
個別の修正着手時はそちらを参照のこと。
