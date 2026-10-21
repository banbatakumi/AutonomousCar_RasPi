# SURGE Mark.2 — 横断コードレビュー 統合版（2026-10-21）

**対象**: `surge_mk2/`（HEAD `aee4e1d`「保存済み地図の学習値だけを GUI から消せるようにする」）
＋ STM32 ファーム `AutonomousCar/Program/MainF446RE_V3`（プロトコル v0.29 / `0x001D`）
**規模**: Python 約 111,000 行 / TypeScript 16,400 行 / C（ファーム）
**コード変更**: なし（本レビューは読み取り専用。検証の再現は一時ディレクトリでのみ実施）

---

## 0. このファイルについて

### 0.1 統合した原稿

同日に作られた4つの横断レビューが同一の問題を重複して報告していたため、1本に統合した。

| 原稿 | 範囲 | 原稿の件数 | 統合後 |
|---|---|---|---|
| `docs/review_2026-10-21.md` | 全体俯瞰（正の伝播網・契約・文書・ロードマップ） | 60 | CFG/VER/SAF/ML/NAV/SLM/GUI/OPS/GEN に分散 |
| `docs/review_2026-10-21_nav.md` | `raspi/nav/`（Pure Pursuit / MPC / Hybrid A* / speedctl / raceline） | 14 | NAV-01〜NAV-24 に整理（＋ SLM-08 / ML-09 / GEN-02 へ併合） |
| `docs/review_2026-10-21-contract-review.md` | モジュール境界の契約・設定の単一情報源・`vehicle.toml` スキーマ | 38 | CFG-01〜CFG-26 ＋ VER-01/02/05/08 ＋ NAV-23 ＋ GEN-24 に整理 |
| `docs/review_2026-10-21_summary.md` | 総合俯瞰（全体＋nav＋新規の SLAM / ML / 校正系） | 54 | 全 ID に整理（＋ OPS-08 / GEN-22 / GEN-23 / GEN-24 を追加） |
| **合計** | | **166**（重複込み） | **140** ＋ 取り下げ 2 |

原稿間の重複は **同一問題を指す記述 4 本**（`review_2026-10-21_summary.md` §10 が既に
「同じ指摘が 2 か所に散らばっているので 1 本に統合するのが望ましい」と指摘していた）。
本ファイルは **和集合**であり、いずれかの原稿が持っていた指摘を落としているものはない
（根拠の弱かった記述は §5 で明示的に取り下げた）。

### 0.2 統合規則

1. **重複は1項目に併合**し、断定の最も強いものを採用したうえで、
   検証で拥算された数値・行番号・深刻度を**上書き**した。
2. **種類ごとに分類**した（§3）。深刻度順の索引は §2 に別途置く。
3. **ID は種類プレフィックス**（`SAF` 安全 / `CFG` 設定 / `VER` 検証網 / `NAV` 制御 /
   `ML` 学習 / `SLM` 局所化・地図 / `GUI` GUI / `OPS` 運用・文書 / `GEN` 状態管理・その他）。
4. 原稿ID との対応は **付録A** に残した（追跡可能性のため）。

### 0.3 検証方法（重要）

4原稿の主張は**鵜呑みにしていない**。8本のサブエージェントを**分野別に分割**して
独立検証し、各項目に次の判定を付させた。

| verdict | 意味 |
|---|---|
| `CONFIRMED` | 事実・行番号・影響が全て正しく実コードで再現できる |
| `PARTIAL` | 核心は正しいが、行番号・数値・影響の一部が誤り／過大 |
| `FALSE` | 事実関係自体が誤り（コードは別のことをしている等） |
| `STALE` | 既に修正済み／該当コードが存在しない |
| `SEVERITY-OFF` | 事実は正しいが深刻度の主張が過大（または過小） |

検証の内訳:

| # | 担当範囲 | 検証項目数 |
|---|---|---|
| 1 | 安全（制動・非常停止・E-Stop・failsafe） | 12 |
| 2 | 検証網（CI・pytest・検査スクリプト） | 12 |
| 3 | 設定の単一情報源（`vehicle.toml`・生成器・firmware 定数） | 26 |
| 4 | 制御・経路追従（`raspi/nav/`・`raspi/core/vehicle.py`） | 26 |
| 5 | 学習・推論（`ml_cam` / `ml_cam_e2e` / `ml_lidar` / `ml_common`） | 13 |
| 6 | 局所化・SLAM・地図（`slam2d/`・`raspi/auto/mapstore.py`） | 30 |
| 7 | GUI・WS・フロントエンド（`gui/src/`） | 15 |
| 8 | 運用・文書・ログ（`docs/`・`tools/calib`・`raspi/rec/`） | 16 |
| 9 | 追加の未カバー項目 | 9 |

検証は**Reproduce（ホスト実行）・数値計算・`tomllib` による機械的キー列挙・
ファーム C ソースとの突合**を用いた。`config/vehicle.toml` の実キーは記憶ではなく
必ず `tomllib` で列挙して確認した。

### 0.3.1 第2ラウンド — 統合版そのものを 6 本で再検証

統合版の記述も**鵜呑みにはしていない**。分野を 6 本に分け、各エージェントに
**「肯定しようとしてはいけない。統合版を疑ってコードから独立に再導出せよ」**と指示して
**140 項目すべて**を再検証した。判定尺度は第1ラウンドと変えた
（`CONFIRMED/PARTIAL/FALSE` → **`OK` / `MINOR` / `MAJOR` / `FALSE` / `UNVERIFIED`**）
ので、記述の細部の誤りまで拾える。

| # | 担当 | 件数 | セッション |
|---|---|---|---|
| 1 | §3.1 安全 ＋ GEN-01/11〜20 | 24 | `ses_ea3c89b1effeCevu650FX1Clyh` |
| 2 | §3.2 設定の単一情報源 | 26 | `ses_ea3c89b1dffePw6YwCOVSdvzuu` |
| 3 | §3.3 検証網 ＋ §3.8 運用・文書 | 20 | `ses_ea3c89b1cffeUkTq0Ncd7GiGPY` |
| 4 | §3.4 制御・経路追従 | 22 | `ses_ea3c7d549ffeZ6eC89wSqCbGGf` |
| 5 | §3.5 学習・推論 ＋ §3.6 SLAM・地図 | 21 | `ses_ea3c7d548ffevxcNX1ajXh6xZr` |
| 6 | §3.7 GUI・WS ＋ §3.9 状態管理 | 26 | `ses_ea3c7d548ffdxAwPnJQz072Btf` |

**第2ラウンドで統合版を直した主なもの**（詳細は §5 の表）:

| 区分 | 件数 | 代表例 |
|---|---|---|
| 因果・機構の誤り | 4 | SAF-09「350 ms 後 DISARM」→ 実際は `max(stall,150 ms)`=200 ms／SAF-13 の原因説明がコードの自認コメントと逆／GEN-09 の `reason` 蓄積が別箇所／VER-03 の原因コミットが `10cd778`→実体は `98f0288` |
| 数値の誤り | 8 | GEN-20「76 フレーム」→ **100**／CFG-25「0.4505/0.469」→ **0.4509/0.4681**／NAV-02 `lead_s` 0.0321→**0.0421**／OPS-01 の 1.5 MB/分→3.6 GB/日 の鎖が矛盾 |
| 過大評価 | 8 | CFG-01「3.2 倍 grippier」→ 実差は横 +9.5 %／VER-01「全工程が無効化」→ 最終ステップのみ／NAV-13「garage」シーン名 → DEADEND |
| 深刻度の再校正 | 13 | Critical 4→**0**、High 26→24、Medium 68→71、Low 42→44 |
| 表記・文字化け | 12 | `注释`→`コメント` 等 6 箇所、` acted`/`intention`、行番号ずれ 8 箇所 |

**逆に、統合版が第1ラウンドで訂正した箇所は 6 エージェントすべてが独立に再現成功**した
（planner 数 22／`nearest_index` 3 回／`TestParamTable` に skip が無く firmware 不在時 3 fail／
`raspi/tests` 1667 件／`CmdOut` 19 フィールド／Python 側に `DriveCmd` が実在／
`SignalConfig` の GUI 側は `SettingsPanel.tsx`／`_cmd_ns` の初期値は 0／
`link_tracker` は `tv_ratio` を表示しない／`deskew.py:121` は `t_ref` ではない／
`rear 0.4637`／「ABS が 0.0498 N·m で常に立つ」は `FALSE`／契約は 26 個で v24 は未エクスポート／
停止距離 0.4167/0.4969・1.0167/1.3058／`slam2d/pipeline.py` へのパス訂正／旧 pose は 500 ms）。

### 0.4 深刻度の再校正について

**4原稿の深刻度は概ね 1 段過大**であった。検証で 5 件が `FALSE`、
多数が `PARTIAL` になったため、本ファイルでは**検証後の判定を正**とした。
例:

- `park_to_point` の停止待ちデッドロック → **`FALSE`**（`max_maneuver_s` の打ち切りが
  停止待ち中も機能する）
- 校正の中止時に「最も弱い制動トルク」→ **`PARTIAL`**（abort 0.08 N·m は
  通常停止の 0.06 N·m（`_calib_common.py:275`）より**強い**。真の欠陥は根拠コメントの陳腐化）
※比較対象は複数ファイルに跨る：`_calib_common.py:275` = 0.06、`calib_steer.py:160` = 0.05、
`calib_tyre.py:117` = 0.08（abort と同値で「より強い」ではない））
- 最高速のクランプ方向 → **`FALSE`**（LIMITS 未受信時は 3.0 で切るのではなく
  むしろ**無クランプ**）
- 校正の `self.room` が 1 周期古い → **`FALSE`**（`leg_step()` 呼び出し前に代入済み）
- 「契約 JSON に現在の `vehicle.toml` 値が入る」→ **`PARTIAL`**（既存 26 個は
  学習時スケールで**正しい**。危険なのは今後の再エクスポート時だけ）

> **残った深刻度は 1 段保守的に読むのが安全**である。検証で落ちた 5 件は
> いずれも「報告段階では最上位の指摘」だった。偽陽性を取り下げる過程そのものが
> 本報告の信頼性の根拠である。

**第2ラウンド（統合版自体の再検証、6 エージェント）**でも同じ傾向が続いた。
統合版がすでに 1 段保守的に書いていたにもかかわらず、さらに **13 件の深刻度を下げ**、
**Critical を 4 → 0** に修正した（§0.3.1 / §5.1）。
内訳は ①因果・機構の誤り 4 件、②数値の誤り 8 件、③過大評価 8 件、④表記 12 件。
つまり **元原稿の深刻度は約 2 段過大**であったということになる。

> **教訓**: 「独立検証を 1 回やった」は安心材料にならない。
> 第1ラウンドで 5 件落ち、第2ラウンドでさらに 13 件の深刻度下调と
> 30 箇所の記述訂正が出た。**検証の「真」を確かめるのは、
> 誰かが同じ項目をもう一度独立に読むことだけ**である。

---

## 1. 結論

**このプロジェクトは構造が無いのではなく、構造が異常に整っている。**

型・生成コード・プロトコル・フレーム解析・安全ゲートの骨格は、実測・突合した範囲では
ほぼ隙が無い。`VehicleState` 60 フィールド・`LinkDiag` 53・`AutoState` 72 が
`gui/src/generated/msgs.ts` と完全一致し、`tsc -b` は strict +
`noUncheckedIndexedAccess` で通り、`gui/src` に `any` は 1 つもない。
フレーム解析は継続的なゴミ入力でも固まらない。

一方で **「正（source of truth）の伝播網」に穴が開いている**。

> `config/vehicle.toml`・`raspi/proto/protocol.toml`・
> `raspi/proto/generated/packets.py` を正とし、そこから firmware・firmware シム・
> Pi 側ノード・`sim/`・ML 環境・校正ツール・GUI へ値を広げる設計は正しい。
> **しかしその伝播網の検査は 4 本しか無く、かつ CI 側の 1 本は
> 2026-10-19 以降壊れている。**
> 穴に落ちているのはすべて「片方だけ直しても誰も異議を唱えない」種類のバグである。

最も生效の度が高いのは **同じ物理量が 3 つの「別の車」を定義している**こと（§3.2 CFG-01）。
`[limits].a_lat_m_s2 = 4.0912` / `[sim].mu = 0.41718` / `[control.plant].mu = 1.3227` は
互いに矛盾し、`sim/`・`tools/fwsim/`・実車 runtime が**それぞれ別の車**を前提に制御している。

**次に効くのは「安全の網が GUI にある」構造**である。速度のランプ上限（NAV-03）、
サイドブレーキの保持（SAF-02）、制動トルクの規約（SAF-01）がいずれも
**GUI 側の値に依存**して初めて安全が成立する。

> **重要（2 ラウンドを通した結果）**: 4 原稿は当初 **Critical 4 件**を上げていたが、
> 2 段階の独立検証（計 15 エージェント、299 項目の再検証）を経て
> **Critical は 0 件**になった。残る 24 件の High は
> ①**検査網が壊れている**（VER-01 / VER-02 / VER-03 / VER-04）、
> ②**正の伝播網が裂けている**（CFG-01 / CFG-05 / CFG-06 / CFG-12）、
> ③**安全網が GUI 側に寄生している**（SAF-01 / SAF-02 / NAV-03）、
> ④**盾の前提が崩れている**（SLM-01 / SLM-02 / GEN-01 / GEN-02）、
> ⑤**学習契約が学習と繋がっていない**（ML-01 / ML-03 / ML-04）
> の 5 系統に集約される。**どれか 1 系統を直すと、半分は構造的に発生しなくなる。**

---

## 2. 索引（深刻度順）

**件数**: 統合・検証後の確定項目 **139 件**（**C: 0** / H: 24 / M: 71 / L: 44）＋ 取り下げ 2 件。

**件数**: 統合・検証後の確定項目 **140 件**（**C: 0** / H: 24 / M: 72 / L: 44）＋ 取り下げ 2 件。

> **注**: 4原稿の指摘 166 個（重複込み）→ 第1ラウンド検証で 140 個 →
> **第2ラウンド独立再検証（6エージェント、140 項目）**により
> **深刻度 13 件を下げ**（Critical 4 → 0、High 26 → 24、Medium 68 → 72、Low 42 → 44）、
> **1 件を取消し**、**因果説明・数値・行番号・表記を 30 箇所訂正**した（§0.4 / §5）。

| ID | 深刻度 | 検証 | 種類 | 要約 |
|---|---|---|---|---|
| **H (24)** | | | | |
| CFG-01 | H | CONFIRMED | 設定 | μ が 3 系統あり、`sim/` と `fwsim` が別の車を回っている |
| CFG-05 | H | CONFIRMED | 設定 | `a_lat_max` が 2 系統あり、同じ名前で別の値・別の既定を持つ |
| CFG-06 | H | CONFIRMED | 設定 | `[limits]` と `[safety]` の一部が生成物（GUI）に出ない |
| CFG-12 | H | CONFIRMED | 設定 | `axle_model`（車軸ごとの摩擦）が到達不能 |
| GEN-01 | H | CONFIRMED | 状態 | `replay_node --bus` の時刻ドメイン不一致で planner が恒久「点群が古い」判定 |
| GEN-20 | H | CONFIRMED | 状態 | `sim/stm32.py` の `dt_total` cap で停止時間が黙って失われる |
| ML-01 | H | CONFIRMED | 学習 | E2E LiDAR モデルの `dt` 契約が空で学習周期と実機周期が 2 倍違う |
| ML-03 | H | CONFIRMED | 学習 | `preproc_version` ガードが学習と結び付いていない |
| ML-04 | H | CONFIRMED | 学習 | `ml_cam` の ONNX エクスポートが学習時の入力解像度を反映しない |
| NAV-12 | H | CONFIRMED | 制御 | Hybrid A* の margin 緩和が経路全体に効く |
| NAV-14 | H | CONFIRMED | 制御 | `StandstillSteer`（MD 過熱対策）が 22 planner 中 1 つだけ |
| SAF-01 | H | PARTIAL | 安全 | 自律走行の非常制動が GUI のブレーキスライダ値に格下げされる |
| SAF-02 | H | PARTIAL | 安全 | GUI の `disarm()` がサイドブレーキのビットを落とす |
| SAF-14 | H | CONFIRMED | 安全 | 校正 8 の発振判定に基準が無く発振ゲインが「合格」で選ばれる |
| NAV-01 | M | CONFIRMED | 制御 | Pure Pursuit の曲率 FF 分岐が `\|η\|>90°` の全舵飽和を失う |
| SLM-01 | H | CONFIRMED | SLAM | `slam2d` の `optimize_every > 0` は `ValueError` で落ちる |
| SLM-02 | H | CONFIRMED | SLAM | SLAM の「一致を試みなかった」が `lost` にならず DR pose で地図に焼かれる |
| SLM-03 | H | CONFIRMED | SLAM | ループ検出ゲートが LD06 の距離比例測距誤差に脆い（検出率 15〜45% 低下） |
| SLM-04 | H | CONFIRMED | SLAM | 地図アップロードに展開後サイズ境界が無く展開爆弾（＋トークン検査なし） |
| SLM-05 | H | CONFIRMED | SLAM | `mapstore.save_map` に検証が無く恒久的に読み込めない地図を作れる |
| VER-01 | H | CONFIRMED | 検証網 | CI が削除済み `tools/sysid` を参照し 2026-10-19 以降ずっと赤 |
| VER-02 | H | PARTIAL | 検証網 | CI が firmware 突合テストを回さず、fwsim テストは skip でなく fail する |
| VER-03 | H | CONFIRMED | 検証網 | `sim/tests` が CI に無く、sim テストが master で 2 件赤 |
| VER-04 | H | CONFIRMED | 検証網 | ML 356 件が collection 衝突で一度も CI で走っていない |
| VER-07 | H | CONFIRMED | 検証網 | firmware の `TELEMETRY` 並びと `RAS_LEN_*` を検査するテストが無い |
| **M (71)** | | | | |
| CFG-02 | M | CONFIRMED | 設定 | `rear_lateral_share` というキーが toml に存在しない |
| CFG-03 | M | CONFIRMED | 設定 | 最大舵角の値が 8〜9 種類ある（実値 0.47167 = 27.02°） |
| CFG-04 | M | CONFIRMED | 設定 | 最高速の上限が 5 か所（+ planner 既定 8 か所）ある |
| CFG-13 | M | CONFIRMED | 設定 | `convert.py` のスケール定数 16 個が手書きで `protocol.toml` と非照合 |
| CFG-15 | M | PARTIAL | 設定 | 「学習値を消す」の契約に版・状態が無い |
| CFG-16 | M | CONFIRMED | 設定 | `/ws/control`（GUI↔Pi の JSON）に版・単位の契約が無い |
| CFG-17 | M | CONFIRMED | 設定 | `[sim]` の 4 キーは常に `[limits]` に上書きされる |
| CFG-18 | M | CONFIRMED | 設定 | `[limits]` / `[sim]` / `[control.plant]` の相互一致テストが無い |
| CFG-19 | M | CONFIRMED | 設定 | `t_capture` は「STM32 換算値」ではないことがある |
| CFG-22 | M | CONFIRMED | 設定 | `toml_io.apply` に範囲検証・fsync・バックアップが無くモードが 0600 に落ちる |
| GEN-02 | M | PARTIAL | 状態 | `park_to_point` の `plan_budget_ms` が `auto_cmd_stale_ms` に接近（恒久性は未立証） |
| GEN-03 | M | CONFIRMED | 状態 | `planning_node` の無防備コールバック 3 つに例外ガードが無い |
| GEN-04 | M | CONFIRMED | 状態 | モード切替時に `_last_plan_ns` がリセットされない |
| GEN-05 | M | CONFIRMED | 状態 | `cmd` の発行元は 1 つだが `plan()` に timeout が無い |
| GEN-06 | M | CONFIRMED | 状態 | バスは「drop してよい」前提で欠落検出が無い |
| GEN-07 | M | PARTIAL | 状態 | `Publisher.seq` は再起動で巻き戻り各ノードが独自判定する |
| GEN-08 | M | PARTIAL | 状態 | `slam2d_raceline` の executor がノード終了時に残る |
| GEN-09 | M | CONFIRMED | 状態 | `_cmd_from` が共有オブジェクト `self.state` を破壊的に書き換える |
| GEN-10 | M | CONFIRMED | 状態 | `on_disengage` を持つ planner は解除時に `reset()` が呼ばれない |
| GEN-21 | M | CONFIRMED | 状態 | `sim/stm32.py` の `CONFIG_ACK` に範囲判定が無い |
| GEN-22 | M | CONFIRMED | 状態 | 未知トピックへの前方一致フォールバックが辞書の並び順に依存 |
| GUI-01 | M | PARTIAL | GUI | GUI が「指令が通っているか」を表示しない |
| GUI-02 | M | PARTIAL | GUI | `SignalConfig` の既定値が 5 処・31 リテラル |
| GUI-03 | M | PARTIAL | GUI | GUI → Python の `cmd` 契約が両側手書き |
| GUI-04 | M | CONFIRMED | GUI | `/ws/map` にスキーマ検査が無い |
| GUI-05 | M | CONFIRMED | GUI | スキーマ hash が全-or-nothing で 1 フィールド追加で全機能停止 |
| GUI-06 | M | CONFIRMED | GUI | mcap 記録をブラウザ RAM に上限なしで蓄積する |
| GUI-07 | M | PARTIAL | GUI | 自律走行の解除条件がパッドの微小入力で成立する |
| GUI-10 | M | CONFIRMED | GUI | GUI 側にテストが 1 つも無い |
| ML-02 | M | PARTIAL | 学習 | `ml_lidar` の ONNX 契約 JSON に「現在」の `vehicle.toml` 値が入る（既存 26 個は正しい） |
| ML-05 | M | CONFIRMED | 学習 | `job_runner.drain()` の例外で Tk の `after` ループが永久に死ぬ |
| ML-06 | M | CONFIRMED | 学習 | train/val 分割が記録（セッション）単位でなく一般化性能の推定になっていない |
| ML-07 | M | CONFIRMED | 学習 | `--seed` が再現性を保証しない（numpy の RNG が未シード） |
| ML-08 | M | CONFIRMED | 学習 | `e2e_lidar` は推論の速度がそのまま指令になり独立した安全策が無い |
| ML-09 | M | PARTIAL | 学習 | `ml_lidar/env.py` の自動停止定数が firmware v0.29 から消えた値を写している |
| ML-10 | M | PARTIAL | 学習 | カメラ E2E の前処理版 guarding が半分だけ（＋再読込が名前のみ） |
| ML-11 | M | PARTIAL | 学習 | `ml_cam_e2e` のラベル正規化に現在の `vehicle.toml` を使う |
| NAV-02 | M | CONFIRMED | 制御 | `speed_loop_tau_s` が文書と 2.25 倍食い違う |
| NAV-03 | M | CONFIRMED | 制御 | `speedctl.ref_command` の目標速度ランプに上限が無い（安全網が GUI にある） |
| NAV-04 | M | CONFIRMED | 制御 | 既定では planner が障害物に減速も制動もしない |
| NAV-05 | M | CONFIRMED | 制御 | 障害物検出の 2 周期確認 +100 ms が停止距離に計上されていない |
| NAV-06 | M | CONFIRMED | 制御 | MPC の `now` 時刻基準が崩れると `d[0]` の舵レート制約が 5 倍緩む |
| NAV-10 | M | CONFIRMED | 制御 | 減速側の指令ランプ（3.0）が物理的な制動能力（2.29）を超える |
| NAV-11 | M | CONFIRMED | 制御 | `nearest_index` に全体検索のフォールバックが無い |
| NAV-13 | M | PARTIAL | 制御 | 探索時間予算の既定 0.25 が stale 閾値超（本番は 150 ms にクランプ済み） |
| NAV-18 | M | CONFIRMED | 制御 | `sim/` と `raspi/` が `vehicle.toml` を別々のパーサで読む |
| NAV-20 | M | CONFIRMED | 制御 | `grip.py` の `LaunchFit` が「TC active」の意味を誤用 |
| NAV-21 | M | CONFIRMED | 制御 | `sim/stm32.py` に TC / TV / 実 ABS が無い |
| NAV-22 | M | CONFIRMED | 制御 | `sim/vehicle.py` に MD のトルク遅延が無い |
| NAV-23 | M | CONFIRMED | 制御 | `sim/bench.py` の `CMD_HZ` が実機と違う / bench は実時間 1:1 |
| NAV-24 | M | CONFIRMED | 制御 | `nav/raceline.py` と `sim/raceline.py` の `min_curvature_alpha` 重複 |
| OPS-01 | M | PARTIAL | 運用 | ログ保持ポリシーが 2 つの実装で食い違い文書と乖離 |
| OPS-02 | M | CONFIRMED | 運用 | `AsyncFrameLog` のオーバーフローがログに刻まれない |
| OPS-03 | M | CONFIRMED | 運用 | 文書・数値の陳腐化（8 項目） |
| OPS-05 | M | CONFIRMED | 運用 | `sim/` と `tools/fwsim/` が STM32 の二重実装 |
| SAF-03 | M | CONFIRMED | 安全 | `arm_test` / `estop_test` が `surge-io` 稼働中に UART と GPIO6 を掴む |
| SAF-04 | M | CONFIRMED | 安全 | `deploy.sh --services` が `--safe` を失念し古い `--max-steer` で武装する |
| SAF-07 | M | PARTIAL | 安全 | `shutdown` / `estop_hold` が io_node の failsafe 分類に入っていない |
| SAF-08 | M | CONFIRMED | 安全 | io_node の arm-block 経路が制動を経ずに即 DISARM する |
| SAF-10 | M | PARTIAL | 安全 | ディスク満杯で telemetry / planning / logger ノードが死ぬ |
| SAF-13 | M | CONFIRMED | 安全 | 校正 1 の結果がファーム受付範囲外でも「合格」で書け ARM 不能になる |
| SLM-06 | M | CONFIRMED | SLAM | 地図の永続化フォーマットに版が無い |
| SLM-07 | M | CONFIRMED | SLAM | `mapstore` の tmp 名固定・書込順序・`import_upload` の順序 |
| SLM-08 | M | PARTIAL | SLAM | `raspi/nav/deskew.py` と `slam2d/core/deskew.py` の二重化と `t_ref` 差 |
| SLM-09 | M | CONFIRMED | SLAM | `mapstore` の `json.dumps` に `allow_nan=False` が無い |
| VER-06 | M | CONFIRMED | 検証網 | `tools/calib/toml_io.py` に単体テストが無い（0 件） |
| VER-08 | M | PARTIAL | 検証網 | firmware 側プロトコルが「唯一の定義」から外れている |
| VER-09 | M | CONFIRMED | 検証網 | 「正の伝播網」に検査が 4 本しか無い（10 倍ずれを実証） |
| VER-10 | M | CONFIRMED | 検証網 | `replay_node --bus` が `cmd` を publish しない不変条件が未テスト |
| VER-11 | M | CONFIRMED | 検証網 | `E2ELidar.plan()` と `LidarE2EEnv._to_obs()` の parity が未テスト |
| VER-13 | M | CONFIRMED | 検証網 | `mapstore` の NaN 保存・並行書き込み・`import_upload` 順序のテストが無い |
| **L (44)** | | | | |
| CFG-07 | L | CONFIRMED | 設定 | `cmd_deadman_ms` の範囲チェックが 2 箇所ある |
| CFG-08 | L | PARTIAL | 設定 | `steer_link_x`（逆変換）の反復回数が firmware と違う |
| CFG-09 | L | CONFIRMED | 設定 | `measured` フラグが 2 つの意味を持ち誰も読まない |
| CFG-10 | L | CONFIRMED | 設定 | `Vehicle.load()` が全例外を握りつぶして既定値で動く |
| CFG-11 | L | PARTIAL | 設定 | `maxSteer` の既定が生成側 0 / Pi 側 0.524 で不一致 |
| CFG-14 | L | CONFIRMED | 設定 | 制御パラメータの ON/OFF だけが片側追跡で toml の値がメモリで消える |
| CFG-20 | L | CONFIRMED | 設定 | STM32 再起動の検出が 2 系統ある（補完関係で問題なし） |
| CFG-21 | L | PARTIAL | 設定 | 時刻同期に future-jump の余地が残る |
| CFG-23 | L | CONFIRMED | 設定 | `docs/development.md` が生成器の現状を逆に説明している |
| CFG-24 | L | PARTIAL | 設定 | toml スキーマの説明が古い |
| CFG-25 | L | CONFIRMED | 設定 | `sim/park_world.py` の角度が 26.5°（正は 27.02°） |
| CFG-26 | L | CONFIRMED | 設定 | `install_services.sh` の `--max-ste 0.524` は実害が無い |
| GEN-11 | L | CONFIRMED | 状態 | `merged()` は inf が `s.max` に張り付く |
| GEN-12 | L | CONFIRMED | 状態 | `registry.catalog()` に `sysid` 系も含まれ `make_planner` に排他が無い |
| GEN-13 | L | CONFIRMED | 状態 | `follow_object` の注視点距離 `ld` に上限が無い |
| GEN-14 | L | CONFIRMED | 状態 | カメラ E2E に推論の信頼度しきい値が存在しない |
| GEN-15 | L | CONFIRMED | 状態 | `CamE2ECmd` に `reason` フィールドが無い |
| GEN-16 | L | PARTIAL | 状態 | セクタ境界の 1° オフセットが 3 箇所に重複定義 |
| GEN-17 | L | CONFIRMED | 状態 | `RingJpeg` のカウンタとリング map がロック無し |
| GEN-18 | L | CONFIRMED | 状態 | `camera_node` が `Subscriber` を別スレッドで `poll()` する |
| GEN-19 | L | CONFIRMED | 状態 | `tv_ratio` にトルク定数を流用／`link_tracker` がスケールをハードコード |
| GEN-23 | L | PARTIAL | 状態 | DISARM モードでも ARM ビットがワイヤに載る |
| GEN-24 | L | PARTIAL | 状態 | 「スキャン時刻」の定義が 2 つある（Scan は先頭、deskew は末尾） |
| GUI-08 | L | CONFIRMED | GUI | `take_control` の 250 ms 連打と `notify()` の新 id 連打 |
| GUI-09 | L | PARTIAL | GUI | `PI_MAX_SPEED_CAP` / `PI_MAX_STEER_CAP` の扱い |
| GUI-11 | L | PARTIAL | GUI | GUI のトルク上限は Python と一致するが生成物に無くコメントが旧 |
| GUI-12 | L | PARTIAL | GUI | `ServerMsg` に版フィールドが無い（`status` のみ版あり） |
| GUI-13 | L | CONFIRMED | GUI | `has_grip` の表示は適切（状態表示の追加余地のみ） |
| ML-12 | L | CONFIRMED | 学習 | 「362/363 次元レガシー」対応が現行グラフと無関係 |
| NAV-07 | L | CONFIRMED | 制御 | `cmd_steer_rate_limit_rad_s` が `nav/` から読まれていない |
| NAV-08 | L | CONFIRMED | 制御 | `Vehicle.kappa_max` のオフセット符号が片側だけ保守的 |
| NAV-09 | L | CONFIRMED | 制御 | `max_steer` の既定値が `vehicle.toml` と陳腐化 |
| NAV-15 | L | CONFIRMED | 制御 | MPC が `nearest_index` を 3 回計算 |
| NAV-16 | L | CONFIRMED | 制御 | `park_to_point._hold` が `ready=True` で安全の意味と矛盾する |
| NAV-17 | L | SEVERITY-OFF | 制御 | `speedctl.REF_CATCH_S` の「τ=0 のときの値」という位置づけ |
| OPS-04 | L | CONFIRMED | 運用 | `site/` と `PROGRESS.md` の版が 6 版古い |
| OPS-06 | L | PARTIAL | 運用 | `PROGRESS.md` に到達不能な機能の記載がある |
| OPS-08 | L | PARTIAL | 運用 | `logger_node` のディスク容量チェックがメインループで同期実行 |
| SAF-06 | L | PARTIAL | 安全 | 校正の中止時の制動トルク（根拠コメントが陳腐化。数値は正しい） |
| SAF-09 | L | PARTIAL | 安全 | io_node が cmd の鮮度判定に受信時刻を使う |
| SAF-11 | L | PARTIAL | 安全 | モード切替直後の `dt` と `dt<=0` で舵の平滑化フィルタが素通りする |
| SAF-12 | L | CONFIRMED | 安全 | `auto/cmd` → `cmd` の合流で `accel_limit` / `launch` が 1 周期遅れる |
| VER-05 | L | CONFIRMED | 検証網 | `config/check_docs.py` の検査範囲が狭い（版番号 4 箇所だけ） |
| VER-12 | L | CONFIRMED | 検証網 | 生成器の `--check` を叩く入口が二重管理されている |
| **取り下げ (2)** | | | | |
| OPS-07 | — | **FALSE** | 運用 | （校正の `self.room` が 1 周期古い）**取り下げ** — §5 |
| SAF-05 | — | **FALSE** | 安全 | （`park_to_point` の停止待ちデッドロック）**取り下げ** — §5 |
## 3. 種類別の指摘

各項目は「判定」（検証結果）→「場所」→「内容」→「影響」→「修正案」の順。
深刻度は**検証後の再校正値**。

---

### 3.1 安全（制動・非常停止・E-Stop・failsafe）

#### SAF-01 [H] 自律走行の非常制動が、GUI のブレーキスライダ値に格下げされる
- **判定**: `PARTIAL`（機構は実在。ただし「既定で実害」「最も危険」は誤り。深刻度 Critical → **High**）
- **場所**: `raspi/nodes/telemetry_node.py:3249-3250` / `raspi/msgs/types.py:355-360,644-647` / `raspi/msgs/convert.py:335-349` / `raspi/tests/test_cmd_path.py:792`
- **内容**: `planning_node` は「わからない・障害物あり・経路が陳腐化」のとき `brake_torque = 0.0` を送る。
  `0` は本プロジェクト全体（`types.py` / `convert.py` / ファーム）で
  **「未指定 → STM32 が最大で制動」**を意味する安全側の規約。
  しかし中継の `_merge_auto` が
  ```python
  brake_torque = gui.brake_torque
  if auto.brake and auto.brake_torque > 0:   # ← 0 が入らない
  ```
  としており、結果は **GUI のスライダ値**になる。
  同じ関数の陳腐化経路は `brake_torque = 0.0`（=最大）を正しく送っており、
  **同一ファイル内で新経路と旧経路が逆の意味**になっている。
  docstring（同 `telemetry_node.py:3218`）は
  「`brake_torque` は GUI のスライダの値を使わず 0（= STM32 の最大）にする」と
  **逆の仕様を明記**している。
- **影響**: GUI の既定 0.15 N·m は上限と一致するため**既定では実害はない**。
  ただし **通常の planner は `brake_torque` を一度も設定しない**
  （`raspi/auto/*.py` に代入は `_calib_common` / `calib_*` のみ）ため、
  **障害物回避の非常制動が常に GUI スライダ値になる**。
  人がスライダを下げて自律走行すると非常制動が**最大 1/30** のトルクになる
  （スライダ範囲 0.005〜0.15 N·m）。
  現行挙動は `raspi/tests/test_cmd_path.py:792` に
  「指定なし→GUI」という**明示的テスト**として固定されている（2026-09-27 付）。
- **修正案**: 「0」の意味を 1 本に決める。推奨は `auto.brake and auto.brake_torque <= 0` のとき
  `brake_torque = 0.0`（STM32 最大）を強制し、テストを
  「planner が `brake=True` かつ `brake_torque=0` → 出力は `0.0`」に更新する。
  **静かなパッチではなく明示的な意思決定**が必要。
  併せて `AutoState.brake_torque` の docstring（`types.py:644-647` は
  「0 = 指定しない（GUI の値のまま）」）を実装と一致させる。

#### SAF-02 [H] GUI の `disarm()` がサイドブレーキのビットを落とす
- **判定**: `PARTIAL`（事実関係は確認。ただし因果と影響範囲に過大成分。**High**）
- **場所**: `gui/src/ws/control.ts:209-211` / `raspi/nodes/telemetry_node.py:1513,1517` / `gui/src/input/useDriving.ts:953,1252` / `gui/src/store/ui.ts:620-633` / `docs/uart_protocol.md:742-763`
- **内容**: `disarm()` は `{type, t, mode, arm, light_mode}` の 5 キーしか送らず
  `side_brake` / `winker_left` / `winker_right` を含まない。Pi 側は未指定を `False` にする。
  契約（`uart_protocol.md:755-759`）は「Pi 側が明示的にこのビットを下ろすまで有効なまま」
  と明記しており、**ARM 解除の瞬間に STM32 のサイドブレーキが解除される**。
  さらに未 ARM 経路（`useDriving.ts:953`）は `side_brake: false` を明示的に送る一方、
  ウィッカーはストアの値を送っており**不整合**。
- **影響**: 「サイドブレーキ ON → ARM を下ろす」という駐車手順が成立しない。
  `DriveControls.tsx:62-63` は実車ではなく `vs.side_brake_active` を優先して表示するため、
  **「要求中…」と実車が食い違う**。ストアは `sideBrakeRequested` を保持したままなので
  再 ARM で `side_brake: true` が送られ、**始動瞬間に後輪位置制御へ切わる**。
  なお「走行中の後輪ロック」は本 EMI の真の主因ではなく、
  ストア保持が主因（`disarm` によるビット降下自体は安全側）。
- **修正案**: `disarm()` に現在の `side_brake` / `winker_*` を載せて保持する。
  未 ARM 経路（`useDriving.ts:953`）も `ui.sideBrakeRequested` を使う。
  ARM 時に `sideBrakeRequested` が true なら **ARM を拒否／警告**する。
  「再 ARM で後輪ロック」はどちらの semantics でも必ず消す。

#### SAF-03 [M] `arm_test` / `estop_test` が `surge-io` 稼働中に UART と GPIO6 を掴む
- **判定**: `CONFIRMED`（深刻度 High → **Medium**）
- **場所**: `raspi/tools/arm_test.py:234,240` / `raspi/tools/estop_test.py:275,280` / 良好版 `raspi/tools/probe_uart.py:78-102`
- **内容**: 両ツールが `/dev/serial0` と GPIO6（E-Stop ハートビート）を直接開くが、
  `surge-io` 稼働中チェックが無い。同一リポジトリの `probe_uart.py` には
  `_io_node_running()` チェックと `--force` フラグが**正しく実装されている**。
- **影響**: 2 プロセスが 100 Hz ずつ `COMMAND` を書き、STM32 は後から来た方を実行する。
  2 つの UART 読み手が**バイトストリームを分割**し io_node のフレームが壊れる。
  2 つの GPIO6 書き込みで STM32 は**壊れたハートビート**を見る上、
  ツールのハートビートが io_node の死を隠す。
- **修正案**: `probe_uart.py` と同じガードを適用（`systemctl is-active surge-io` で拒否、
  `--force` で明示的に許可）。可能なら `io` エンドポイントの `flock` を取る。

#### SAF-04 [M] `deploy.sh --services` が `--safe` を失念し、古い `--max-steer` で武装する
- **判定**: `CONFIRMED`（深刻度 High → **Medium**）
- **場所**: `tools/deploy.sh:176` / `raspi/setup/install_services.sh:94,88-93`
- **内容**: `deploy.sh --services` は**引数なし**で `install_services.sh` を呼ぶため、
  `--safe` でインストールした Pi は**通常のデプロイで無警告に武装され直す**。
  しかも `--max-steer 0.524` は古い値（正は 0.47167）。
  `install_services.sh` の警告は**安全側の方向にだけ**出る。
- **影響**: `--services` は unit ファイルを書き換えるだけで**動いている io_node は再起動しない**
  ため、実際の解放は次回の `surge-io` 再起動時。`--max-steer` は `LIMITS` 受信後に
  STM32 の値で丸められる（`io_node.py:906-909`）ため実害は限定的。
- **修正案**: 武装モードを**永続化**（例 `/etc/surge/arm-mode`）し既定とする。
  `deploy.sh` に `--safe` / `--arm` を通す。`--max-steer` / `--max-speed` の既定を
  `vehicle.toml` から読む。警告を**双方向**に出す。

#### SAF-05 — 取り下げ（`park_to_point` の停止待ちデッドロック）
詳述 §5。`FALSE`。

#### SAF-06 [L] 校正の中止時の制動トルク根拠が陳腐化
- **判定**: `PARTIAL`（数値と呼び出しは正しい。「最も弱い」は誤り。High → **Low**）
- **場所**: `raspi/auto/_calib_common.py:103-105,112,424-425`
- **内容**: `ABORT_BRAKE_TORQUE_NM = 0.08`（1 輪あたり）。根拠コメントは
  「0.12N·m 以上は ABS なしで後輪がロックする」「0.08N·m はロックせず 1m/s 以上で約 3m/s² で止まる」。
  「制動しても止まらない」経路（`:424-425`）もこの 0.08 を出す。
  しかし**通常停止トルクは 0.06 N·m**（`_calib_common.py:275`）で、
  abort の 0.08 は**より強い**。ABS は**既定で有効**
  （`docs/uart_protocol.md:955`）なので 0.08 という制約自体は不要。
- **影響**: 「弱すぎて止まらない」わけではない。実害は
  ①根拠コメントが ABS 既定有効という実態から古く、②最大制動（0）を出す選択肢が無いこと。
- **修正案**: 「制動しても止まらない」経路だけは `brake_torque=0`（STM32 最大）で出力する
  選択肢を追加し、根拠コメントを実態に直す。

#### SAF-07 [M] `shutdown` / `estop_hold` が io_node の failsafe 分類に入っていない
- **判定**: `PARTIAL`（完全一致判定と source 一覧は確認。影響説明は過大。**Medium**）
- **場所**: `raspi/nodes/io_node.py:902-903,124` / `raspi/nodes/telemetry_node.py:3177-3182,3182,3198,3237,3283,3542`
- **内容**: failsafe 判定は文字列完全一致のみ。
  | `source` | 発生元 | failsafe 扱い |
  |---|---|---|
  | `"deadman"` | GUI E-Stop / cmd 途絶 | ✅ |
  | `"shutdown"` | `telemetry_node.close()` | ❌ 無ブレーキの素の DISARM |
  | `"estop_hold"` | E-Stop 保持 | ❌ 進行中の failsafe ブレーキを中断 |
  | `"gui:<name>"` | 人が DISARM | ❌（**意図的**。生きた GUI 指令に failsafe は要らない） |
  | `"auto:<mode>"` / `"auto:<mode>:stale"` | planner | 別経路 |
- **影響**: `"estop_hold"` は `brake` / `brake_torque` を強制しないため、
  io_node が送っている**最大ブレーキのパケットが放棄**され素の `arm=0` が届く。
  ただし惰行リスクは STM32 側が担保（ARM 除去後も DRIVE_POWER を最大 3 秒保ち自分で制動、
  `uart_protocol.md:560,580`）。
- **修正案**: `estop_hold` で `brake=True, brake_torque=0.0` を強制する。
  「武装中に disarm が要求された」時点で failsafe 経路へ入る
  （**ただし人が押した DISARM は素通し**という既存テストの決定は維持）。

#### SAF-08 [M] io_node の arm-block 経路が制動を経ずに即 DISARM する
- **判定**: `CONFIRMED`
- **場所**: `raspi/nodes/io_node.py:33-35`（docstring）/ `:895-900`
- **内容**: docstring は「走っていたなら、まず最大制動を送り、止まってから DISARM に落とす
  （`_failsafe_packet`）。いきなり DISARM を送ると STM32 は駆動電源を切り、
  モータドライバごと落ちて制動できずに惰行する」と明記。しかし実装は
  ```python
  if self._arm_block_reason():
      self._last_armed_pkt = None
      self._failsafe_since_ns = None
      return self._disarm_command()
  ```
  で**制動を経ずに即 DISARM**。しかも `_last_armed_pkt` を消すので、
  その後に cmd が stale になっても **failsafe ブレーキが発動しない**。
- **影響**: arm-block 理由の大半は「そもそも走っていなかった」ケースで危険度は小さい。
  危険なのは実行中に STM32 が再起動した場合だが、その場合も STM32 側が自分で制動する。
- **修正案**: arm-block 時も「走っていた可能性があれば最大制動 → 停止確認 → DISARM」の順にする。

#### SAF-09 [L] io_node が cmd の鮮度判定に受信時刻を使う
- **判定**: `PARTIAL`（受信時刻使用は事実。**「`_cmd_ns` の初期値がループ開始時刻」は `FALSE`**。
  「`t_pub` を使うべき」は妥当でない。Medium → **Low**）
- **場所**: `raspi/nodes/io_node.py:154,297,812-816,821,1033` / `raspi/bus/zbus.py:313,381`
- **内容**: `_recv_cmd()` は `self._cmd_ns = now_ns`（`now` は**ループ先頭**で採取した時刻、`:1031`）で鮮度を判定する。
  `stale = self.cmd is None or (now_ns - self._cmd_ns) > CMD_TIMEOUT_NS`（`CMD_TIMEOUT_NS = 150 ms`、
  `:154` ← `config/vehicle.toml:54 cmd_deadman_ms = 150`）。
  **原指摘の「初期値がループ開始時刻」は誤り**: `_cmd_ns` の初期値は **0**（`:297`）であり、
  `LINK_WAIT_S = 0.005`（`:108`）は `_cmd_ns` の初期化に**一切使われていない**
  （使 powering 箇所は `_poll_link` の待ち時間上限 `:998` のみ）。
  実効 DISARM 遅延 = **150 ms +（publish→poll 遅延）**。
  5 ms のループ先頭バイアスは**安全側**（早め発火）。
  `self.sub.poll(0)`（`:810`、非ブロッキング）なので cmd は**ループが回ってきた時点で初めて**取り込まれる。
- **影響**: メインループが 200 ms 止まれば **200 ms**（＝ `max(stall, 150 ms)`）後に DISARM する。
  鮮度判定はループ内で行うため「stall + 150 ms」ではない。
  ただし `TOPIC_CMD` を publish するのは `telemetry_node` のみで 50 Hz に**常に** publish し、
  GUI 指令が古ければ送信前に `source="deadman"` へ読み替えるため
  「出版者が死んだ」と「内容が古い」は上位で既に処理済み。実害は小さい。
- **修正案**: `self._cmd_ns = getattr(msg, "t_pub", 0) or now_ns`
  （`DriveCmd` は `MsgBase` 継承で `t_pub` を持つ: `raspi/msgs/types.py:133-142`）。
  これで「150 ms 経過したら無条件に stale」＝`max(stall, 150 ms)` 相当になる。
  `LINK_WAIT_S` は `_cmd_ns` と無関係である旨をコメントで明示する。

#### SAF-10 [M] ディスク満杯で telemetry / planning / logger ノードが死ぬ
- **判定**: `PARTIAL`（保護の欠如は確認。ただし 3 ノード死は安全側。High → **Medium**）
- **場所**: `raspi/nodes/io_node.py:209-232,1234`（`_QuietStream` あり）/
  `raspi/nodes/telemetry_node.py` / `planning_node.py` / `logger_node.py`（**無し**）/
  `raspi/setup/install_services.sh:136-137`
- **内容**: `io_node` には `_QuietStream`（ディスク満杯で `print` が `OSError(ENOSPC)` を
  投げても落ちない）が、**telemetry / planning / logger には無い**。
  ログ出力先は `logs/<name>.out` で**同じ SD カード上**。
- **影響**: telemetry 死 → io_node の cmd が 150 ms stale → failsafe 最大制動＋DISARM
  （安全側）。planning 死 → `auto/cmd` stale → `_merge_auto` が制動（安全側）。
  logger 死 → 記録だけが止まる。
  **致命的（E-Stop ラッチ）になりうるのは io_node のみで、それは既に保護済み。**
  ただし「記録の問題が車の停止を起こす」原则違反は残る。
- **修正案**: 3 ノードに `_QuietStream` を導入（共通モジュールへ抽出）。
  unit 側を `StandardOutput=journal` にする。**ログの容量超過が車を止めない**ことをテストで保証。

#### SAF-11 [L] モード切替直後の `dt` と `dt<=0` で舵の平滑化フィルタが素通りする
- **判定**: `PARTIAL`（現象は確認。ただし残留フィルタは起きず、影響は 1 コマンド分。High → **Medium**）
- **場所**: `raspi/nodes/planning_node.py:161,268-281,342` /
  `raspi/auto/{follow_the_gap,line_trace,cam_centerline,cam_e2e,gap_pursuit,disparity_extender,follow_object}.py` /
  `raspi/auto/base.py:8-10`
- **内容**: モード切替時に `self._last_plan_ns` がリセットされないため、
  新モードの 1 回目の `plan()` が受け取る `dt` は**前モードの最終 plan からの経過時間**。
  1 次遅れフィルタの `alpha = 1.0 - exp(-dt/tau)` が跳ね上がる
  （τ=0.1 s なら 1 スキャン周期の dt で **alpha ≈ 0.63**。≈1.0 になるのは
  前モードが長く idle だった遷移時だけ）。
  > **注（第2ラウンド再検証で訂正）**: 「alpha が ≈1.0 になりスキップされる」は
  > 通常の遷移では起きない。
  また `alpha = 1.0 if tau <= 1e-3 or dt <= 0 else ...` は
  「平滑化を無効」ではなく「**即座に target へ移動**」を意味し、
  `self._steer` は 0 から始まるので**最初の 1 コマンドがフル振幅の舵**になる。
- **影響**: モード切替は**新しい planner オブジェクトを作り直す**ため、
  `base.py:8-10` の契約が警鐘を鳴らした「前のモードの平滑化が残る」状態は起きない。
  影響は 1 コマンド（50 Hz で 20 ms）。実舵角の段差は STM32 の `steer_rate_limit`（7 rad/s）で制限される。
- **修正案**: **順序が重要**。
  ① 先に `dt <= 0` のときの `alpha` を **0.0（動かない）**にする
  （現状の `alpha = 1.0` は「即座に target へ移動」＝フル振幅）。
  ② その後でモード切替ブランチに `self._last_plan_ns = 0` を追加する
  （**① を先に直さないと、② が `dt == 0` を作って同じフル振幅を残す**）。
  ③ `_replan` 側でも `dt` を `min(dt, 0.5)` に clamp。

#### SAF-12 [L] `auto/cmd` → `cmd` の合流で `accel_limit` / `launch` が 1 周期遅れる
- **判定**: `CONFIRMED`（Medium → **Low**）
- **場所**: `raspi/nodes/telemetry_node.py:3045-3047`
- **内容**: 変更検出キーが `(mode, arm, brake, target_speed, target_steer, torque_mode,
  target_torque, fw_overrides)` であり、`accel_limit` / `launch` が**含まれない**
  （`brake_torque` / `passing` / `winker_*` / `side_brake` も含まれない）。
  そのためこの 2 つだけが変わった `auto/cmd` は 50 Hz ポンプまで待つ。
- **影響**: 遅延は最大 20 ms で上限。`launch` は発進シーケンスの 1 段。
  `accel_limit` は締める向きなので安全側。
- **修正案**: キーに `accel_limit` / `launch` / `brake_torque` / `passing` / `winker_*` を追加。

#### SAF-13 [M] 校正 1（エンコーダ）の結果がファーム受付範囲外でも「合格」で書ける
- **判定**: `CONFIRMED`
- **場所**: `tools/calib/encoder.py:62-64,297-299` / ファーム `src/control/control_params.h:73,80` /
  `src/control/control_params.c:42-52` / `raspi/core/control_params.py:302-308,351-359` /
  `raspi/nodes/io_node.py:752-755` / `tools/calib/result.py:58-70` / `tools/calib/__main__.py:28-33`
- **内容**: `LO_MAX=0.2` / `HI_MIN=0.8` / `HI_MAX=1.0` / `H_MAX=0.1` を定義しているが、
  **`HI_MIN` は合否判定で使われていない**（`tools/calib/tests/test_encoder.py:63` が
  `HI_MIN <= hi <= HI_MAX` を**テスト**はしている。判定本体 `encoder.py` の verdict 側だけが未使用）。
  合否判定は `fit.lo > LO_MAX or max|fit.h| > H_MAX` のみ。
  `hi` が範囲外で確定すると → STM32 が丸める → `control_params_status = "mismatch"` →
  **io_node が ARM を全面ブロック**する。ところが verdict は「合格」。
  > **注（第2ラウンド再検証で訂正）**: 原因の説明は `tools/calib/encoder.py:222-226` の
  > 自認コメントと**逆**。同ファイルは「惰行が下端まで届かないと `lo` が大きくなり
  > `hi = lo + 幅` が 1.0 を超える（実機で 1.000655）ので `hi = 1.0` になる `lo` へ
  > 下げて当てはめ直す」（= `hi > HI_MAX` には再試行がある）と説明している。
  > `hi < HI_MIN`（= 0.8 未満）を生むのは別の条件（`lo` が小さく `span` が 0.8 を切る）である。
  同じ「範囲外でも合格」の穴が校正 5 にもある
  （`tools/calib/speed.py:288-289` が warn するだけ、
  `speed_ff_nm_per_m_s2 = 1/G` は firmware 上限 0.2 を無検査で書く）。
  `Result.usable` は warn でも `True` なので `--apply` される。
- **影響**: 「この校正は使えない値を出した」ではなく「**なぜ車動きません**」という
  見た目のロックアウト。
  さらに `control_params.py` の `mismatch` は ARM 停止だけでなく
  **該当パラメータの再送も `_clamped` により止まる**ため、
  **GUI で書き直しても反映されない**（「直せない」ロックアウト）。
- **修正案**: 解析側でファームの `param_table()` を引き、**範囲外は fail**（warn ではなく）にする。
  `tools/calib/speed.py` の `speed_ff_nm_per_m_s2` / `speed_ff_rr_nm` / `launch_exit_torque_nm`
  も同じ扱いにする。`--apply` は warn の値を既定で除外する。

#### SAF-14 [H] 校正 8（TC/ABS ゲイン）の発振判定に基準がなく、発振ゲインが「合格」で選ばれる
- **判定**: `CONFIRMED`
- **場所**: `raspi/auto/calib_tc_gain.py:46,53,103-108,190-201` /
  `tools/calib/tc_gain.py:105-111,125-140` / `tools/calib/traction.py:40,184` / `docs/calibration.md:181,217`
- **内容**: `is_oscillation()` は `baseline is None` だと**無条件に False**（= 非発振）。
  `_finish_leg()` で最初の片道（梯子の最下位 ωc=15）では `self._jit` が空で `base = None` になる →
  **最下位のゲインは原理的に「発振」と判定できない**。baseline は「過去片道の最小の揺れ」だが、
  比較は `jitter > 1.6 * baseline` の絶対値依存。実機では
  「TC・ABS を切ったランプでも制動側は 0.03 m/s 揺れる」（`calibration.md:181`）ので、
  **静かな 1 片道が混ざると baseline が 0 に寄り以後の正常な揺れが全部「発振」になる**。
  さらに**校正 9 の `OSC_FAIL_STD = 0.2` は 0.04 の揺れを合格とみなす**＝安全網の穴が 5 倍離れている。
- **影響**: 校正 8 の存在意義（「実機で高すぎて振動したゲインを入れない」`calibration.md:200-226`）が
  骨組みから崩れる。
- **修正案**: 最初の片道にも絶対判定を適用し、baseline に下限（例 0.01）を入れる。
  「判定不能」を「発振なし」と区別する 3 値化。`OSC_FAIL_STD` と `OSC_STD_MAX` の
  5 倍差を共通定義に寄せる。

---

### 3.2 設定の単一情報源（正の伝播網）

#### CFG-01 [H] μ が 3 系統あり、`sim/` と `fwsim` が別の車を回っている
- **判定**: `CONFIRMED`（ただし「fwsim の縦 accel 上限 5.17」は再現不能。**Critical**）
- **場所**: `config/vehicle.toml:126,138,265` / `sim/vehicle.py:211-220` /
  `tools/fwsim/plant.py:65,119-120,121-124` / `tools/calib/tyre.py:512,515`
- **内容**: 実値（`tomllib` による機械列挙）:
  | 場所 | 値 | 誰の μ |
  |---|---|---|
  | `[limits].a_lat_m_s2`（:126） | **4.0912** | μ = 0.417（実車・sim 共通） |
  | `[sim].mu`（:138） | 0.41718 | **常に上書きされる**（`sim/vehicle.py:211-220`） |
  | `[control.plant].mu`（:265） | **1.3227** | `tools/fwsim/plant.py`（タイヤ μ） |
  比 1.3227 / 0.41718 = **3.17 倍**。
  `tools/fwsim/plant.py:119-120` は **`[dynamics].mu` という存在しないキー**を読んでいるため
  常に falsy → `lateral_accel_max_m_s2` は**既定値 4.48 のまま**に固定される
  （`[dynamics]` の実キー 19 個に `mu` は無い）。
  実測 `Plant.load()` は `mu=1.3227` と `lateral_accel_max_m_s2=4.48` を同時に保持し、
  **同一モデル内で 2 つの別の μ が同時に効く**（`host_sim.c:224` と `:464`）。
- **影響**: `sim/` の車は横 4.09 / 縦 3.22、`tools/fwsim/` の車は横 4.48（ハードコード）/
  縦は C モデルから創発（**C モデル実測で 3.43 m/s²**）。
  **ABS/TC ゲイン（校正 8 で実機合わせ中の値）は
  fwsim の車の上で最適化され**、実機に持ち込むと
  「効かない」か「発振する」のどちらになる。
  > **注（第2ラウンド再検証で訂正）**: μ *パラメータ* の比は 3.17 倍だが、
  > **結果のグリップ差は横 +9.5 % / 縦 +6.6 %** にすぎない。
  > 「3.2 倍 grippier な車」という表現は過大であった。`docs/control_constants.md:154` は
  「ABS・TC・片輪浮き対策のゲインは机上値で実機未検証」と書くが、
  **机上値そのものが別の車の上の値**であることは記載がない。
- **修正案**: μ を 1 か所へ集約する（`[limits].a_lat_m_s2` を正とする）。
  `tools/fwsim/plant.py` は `[limits]` を読む。`[control.plant].mu` は
  「タイヤのピーク μ」として名前を変える。`tools/calib/tyre.py` は μ と a_lat を
  同時に書き、**書き込み時に整合を検証**する。fwsim と `sim/` の grip を
  **1 本のテストで照合**する。

#### CFG-02 [M] `rear_lateral_share` というキーが toml に存在しない
- **判定**: `CONFIRMED`（事実）／`SEVERITY-OFF`（Critical 評価は過大 → **Medium**）
- **場所**: `raspi/core/vehicle.py:399` / `config/vehicle.toml` `[control.plant]` /
  `tools/calib/tyre.py:88` / `docs/system_overview.md:687` / `sim/vehicle.py:194-196`
- **内容**: `rear_load_frac=float(d.get("control",{}).get("plant",{}).get("rear_lateral_share", 0.59))`。
  `[control.plant]` の実キーは `mu, tyre_b, tyre_c, load_transfer, yaw_damping,
  yaw_inertia_kgm2, wheel_inertia_kgm2, wheel_friction_nm, wheel_damping_nms,
  md_delay_s, md_tau_s` で **`rear_lateral_share` は無い**。
  値は常に 0.59 の既定が使われ、`tools/calib/tyre.py:88` の `REAR_SHARE = 0.59` は
  ハードコードで、`toml_io` は存在しないキーには足さない → **校正で値を直しても反映されない**。
  `docs/system_overview.md:687` は「車軸の値は `[control.plant]` の `rear_lateral_share`・
  `load_transfer`」と**嘘**を書いている（`load_transfer` のみ実在）。
  関連: `load_transfer_hl` は Pi 側 0.13324 / シム側 0.133（ハードコード）/ fwsim 側 0.13324 と
  3 つの値が流通している。
- **影響**: 値は**ずれない**（writer が存在しない）。実害は ①文書が誤り、
  ②将来の再校正が黙って捨てられる、③影響範囲は `grip_shape=1` の隠し設定に限られる の 3 点。
- **修正案**: `[control.plant]` に `rear_lateral_share = 0.59` を**明示追加**し、
  `tyre.py` の `REAR_SHARE` をそこへ寄せる。既定を 0.59 ではなく「キー欠落を検出できる値」にする。

#### CFG-03 [M] 最大舵角の値が 8〜9 種類ある（実値 0.47167 = 27.02°）
- **判定**: `CONFIRMED`（High → **Medium**）
- **場所**: `config/vehicle.toml:22-25` / `raspi/core/vehicle.py:118-121,360` /
  `raspi/nav/purepursuit.py:100` / `raspi/nav/mpc.py:53` / `sim/vehicle.py:93,226` /
  `raspi/nodes/io_node.py:161,164` / `sim/bench.py:309,1186` / `sim/run.py:377` /
  `sim/park_world.py:15` / `config/generate.py:87` / `raspi/setup/install_services.sh:94` /
  `docs/system_overview.md:87` / `docs/stm32_interface.md:1165`
- **内容**: 正は `config/vehicle.toml:25` の **0.47167**（= **27.0247°**）。
  実測で `steer_link_road(radians(30), 1.101158, -0.730735) = 0.47166974…` と一致。
  0.47167 以外が **9 種類以上**ある: `0.524`（`vehicle.py:121,360` / `sim/vehicle.py:93,226` /
  `install_services.sh:94` / `docs/stm32_interface.md:1165`）、
  `0.4636`（`vehicle.py:119` の docstring）、`0.46175`（`purepursuit.py:100` / `mpc.py:53`）、
  `0.35`（`io_node.py:161` の `DEFAULT_MAX_STEER`）、`26.5°`（`sim/park_world.py:15`）、
  `26.56°`（`vehicle.toml:22` のコメント）、`0`（`config/generate.py:87`）、`±30°`（`system_overview.md:87`）。
- **影響**: 実運用値は toml / 生成物 / `install_services.sh`→`io_node`
  （`min(args.max_steer, VEHICLE_MAX_STEER=0.47167)`）で 0.47167 に統一されている。
  残りは**既定値・コメント・未使用の既定**。`vehicle.toml:22` のコメントは
  このファイル自身のリファレンスとして読者が最初に読む行。
- **修正案**: `vehicle.toml:22` のコメントを 27.02° に、`vehicle.py:119-121` の docstring を
  実値に。`purepursuit.py:100` / `mpc.py:53` の既定を `Vehicle.max_steer` 参照へ
  （または `None` にして「呼び出し側が必ず渡す」を型で強制）。
  `sim/*` の `--max-steer` 既定は `vehicle.toml` から読む。

#### CFG-04 [M] 最高速の上限が 5 か所（+ planner 既定 8 か所）ある
- **判定**: `CONFIRMED`（実値）／クランプ方向の主張は `FALSE`（**Medium**）
- **場所**: `gui/src/store/ui.ts:195,407-424` / `raspi/setup/install_services.sh:94` /
  `raspi/nodes/io_node.py:160,1219` / `raspi/core/control_params.py:83`
- **内容**: 実値は ①`gui/src/store/ui.ts:195` `PI_MAX_SPEED_CAP = 3.0`
  （コメントは「install_services.sh の `--max-speed` と一致させてある」と**逆向き依存**を宣言）、
  ②`install_services.sh:94` `--max-speed 3.0`、③`io_node.py:160` `DEFAULT_MAX_SPEED = 1.0`
  （未インストール時の既定）、④`control_params.py:83` `STM32_MAX_SPEED_M_S = 5.0`
  （ファーム `drive.h:82` の写し）。firmware の実 limits は 5.0。
  **クランプ方向の主張は逆**: `ui.ts:407-424` は
  `const speedCapKnown = typeof live.link?.max_speed_m_s === 'number'` /
  `const upper = !speedCapKnown && (...) ? Infinity : max` 也就是说、
  **LIMITS 未受信のときは上を無クランプ（Infinity）で保存値を保持**し、
  受信後に `LIMITS.max_speed_m_s` でクランプする（コメント `:188-190` が明言）。
- **影響**: 「3.0 未満で不可能な状態を作り、3.0 超の保存値はリロードで巻き戻る」という
  主張は**根拠なし**。実害は「3.0 / 5.0 / 1.0 のどれが正かが決まっていない」ことと
  逆向き依存の宣言。**ただし `maxSteer` 側には保存値の巻き戻りが残る**
  （無クランプ豁免は `maxSpeed` / `mtMaxSpeed` のみ）。
- **修正案**: 最高速の上限は firmware の `DRIVE_MAX_SPEED_M_S` ただ 1 つを正とし、
  GUI は「LIMITS 受信までのフォールバック」であることを明示。
  `install_services.sh:94` と `io_node.py:160` を同一値に揃えるか、
  `--max-speed` が firmware 上限を超えないことを機械的に強制する。

#### CFG-05 [H] `a_lat_max` が 2 系統あり、同じ名前で別の値・別の既定を持つ
- **判定**: `CONFIRMED`
- **場所**: `raspi/core/vehicle.py:266-269` / `raspi/auto/gap_pursuit.py:147-150` /
  `raspi/auto/follow_object.py:89-92` / `raspi/auto/cam_e2e.py:66-70` /
  `raspi/auto/{park_to_point,disparity_extender,cam_centerline,follow_the_gap}.py` /
  `raspi/auto/slam2d_raceline.py:280,309,1275,1288,2645`
- **内容**: `Vehicle.a_lat_max` = `[limits].a_lat_m_s2` = **4.0912**（未校正時の既定 3.5）。
  一方 `ParamSpec(key="a_lat_max", default=3.0, note="★未計測の暫定値")` が **7 箇所**にある。
  slam2d 系は前者、FTG/DE/DP と camera E2E は後者を使う。
- **影響**: 同じ「横加速度上限」という概念が**車検証済みの値**と
  **「まだ測ってない」暫定値**で並存している。FTG/DE/DP は 3.0 のまま
  26% 低い速度で曲がる（安全側だが、古い「未計測」コメントのまま）。
- **修正案**: planner 側の `a_lat_max` を既定で `Vehicle.a_lat_max` から取る。
  名前も `a_lat_max_unmeasured` 等に変えて**来源を明示**する。

#### CFG-06 [H] `[limits]` と `[safety]` の一部が生成物（GUI）に出ない
- **判定**: `CONFIRMED`
- **場所**: `config/generate.py:87,142-150` / `raspi/core/vehicle.py:396-398,406` /
  `raspi/core/control_params.py:75-78`
- **内容**: `config/generate.py` の出力キーは
  `VEHICLE{wheelbase, track, maxSteer, wheelRadius, wheelWidth, footprint, lidar, camFront, camRear}` と
  `SAFETY{cmdDeadmanMs, autoLinkGraceMs, autoCmdStaleMs, cmdAccelLimit, cmdSteerRateLimit}` のみ。
  **`[limits].a_lat_m_s2` / `a_accel_m_s2` / `a_brake_m_s2` / `abs_brake_decel_m_s2` は無い**。
  **`[safety].plan_accel_limit_m_s2` も無い**。一方 Pi 内部では読んでいる。
- **影響**: GUI は実車の横加速度上限を**知らない**。
  「計画が 3.0 で頭打ち」「実機は 4.09 m/s² まで踏める」等の関係を**表示できない**。
- **修正案**: `[limits]` と `[safety]` 全体を生成出力に含める。
  GUI の設定画面が「どの値が実機の限界で、どれが学習値か」を区別して表示できるようにする。

#### CFG-07 [L] `cmd_deadman_ms` の範囲チェックが 2 箇所ある
- **判定**: `CONFIRMED`（Medium → **Low**）
- **場所**: `config/generate.py:142` / `raspi/core/vehicle.py:92,95-107` / `raspi/nodes/io_node.py:154`
- **内容**: `config/generate.py` は `float(safety.get('cmd_deadman_ms', 150))` を**範囲検証せず素通し**。
  `raspi/core/vehicle.py` の `_deadman_ms()` は 20〜1000 ms を検査し範囲外は 150 にフォールバック。
  toml に `cmd_deadman_ms = 150000` と書くと **GUI には 150000、Pi は 150**。
- **影響**: 実害は限定的（Pi 側は安全側）。ただし「生成器と Pi 読取器の校验逻辑が二重」構造。
- **修正案**: 範囲チェックと既定値を `vehicle.py` に一本化し、
  `config/generate.py` は `Vehicle.load()` の結果を import して使う。

#### CFG-08 [L] `steer_link_x`（逆変換）の反復回数が firmware と違う
- **判定**: `PARTIAL`（**定数・傾き下限は一致**。差は反復回数のみ。firmware 所在不明という主張は `FALSE`）
- **場所**: `raspi/core/control_params.py:107-123` / ファーム `src/control/steering.c:35-44` /
  `src/control/steering.h:8,33,37` / `raspi/tests/test_control_params.py:146-165`
- **内容**: firmware `steering.h:33` `STEERING_LINKAGE_RATIO 0.5f`、`:37`
  `STEERING_LINKAGE_MIN_SLOPE 0.3f`、`:8` `STEERING_MAX_ANGLE_RAD 1.0471975512`（60°）× 0.5 = 30°。
  Pi 側 `STEER_LINK_X_MAX_RAD = math.radians(30.0)` と `max(gain + 3*cubic*x*x, 0.3)` は**一致**。
  差分は**ニュートン反復回数だけ**（Pi は 6 回、firmware は 4 回）。収束後の値は同じ。
  テストは Pi 内部の自己整合しか見ていない。
- **影響**: 実害は無い。firmware の値が変わっても Pi は追随しないという**追跡性の穴**。
- **修正案**: docstring に「反復回数は STM32 の 4 回より多い（収束後の値は同じ）」と明記するか、
  4 回に揃える。firmware 側の定数を `test_fw_constants.py` の検査対象に追加。

#### CFG-09 [L] `measured` フラグが 2 つの意味を持ち、誰も読まない
- **判定**: `CONFIRMED`
- **場所**: `raspi/core/vehicle.py:114-115,357` / `config/vehicle.toml:13,15`
- **内容**: docstring は「諸元が実測済みか。False の間は経路追従の精度を云々しても意味が無い」だが、
  toml の `measured = false` は**「`encoder_ticks_per_rev` 1 項目ぶんの印」**という別の意味。
  `Vehicle.measured` は**常に False**で、`grep -rn "\.measured"` の読み手は**どこにも居ない**
  （ヒットするのは LiDAR の `w.measured` など別物）。
- **影響**: 現状の実害は無い。ただし `wheelbase` / `track` / `max_steer` は実測確定なのに false のまま。
- **修正案**: 使うか消すかを決める。残すなら「どの諸元が実測済みか」を toml に明示する。

#### CFG-10 [L] `Vehicle.load()` が全例外を握りつぶして既定値で動く
- **判定**: `CONFIRMED`
- **場所**: `raspi/core/vehicle.py:335-344,360,121`
- **内容**: `except (OSError, tomllib.TOMLDecodeError): return cls()`（**無言**）と
  `except (TypeError, ValueError, KeyError, AttributeError, IndexError): print(...); return cls()`。
  **未知キーの検出が無い**（`_from_dict` は `d.get(...)` のみ）。
- **影響**: `max_steer` のキーを 1 文字間違えると**例外は出ず**、
  既定 `0.524`（=30°）で動く。**危険側であって安全側ではない**
  （`kappa_max` と `io_node` の舵角クランプに効く。実車の最終クランプは
  STM32 の `LIMITS.max_steer_rad`）。
- **修正案**: 未知のトップレベルキーを検出して stderr に列挙する。
  `max_steer` の既定を実測値 0.47167 にする。読み込み失敗時に
  「どの項目が既定落ちしたか」を列挙する。

#### CFG-11 [L] `maxSteer` の既定が生成側 0 / Pi 側 0.524 で不一致
- **判定**: `PARTIAL`
- **場所**: `config/generate.py:87` / `raspi/core/vehicle.py:121` / `gui/src/store/ui.ts:196,249,289`
- **内容**: 生成側は既定 0、Pi 側は既定 0.524。**ただし実生成値は 0.47167**
  （toml に `max_steer` があるため）。0 になるのは toml から削除したときのみ。
  その場合 `max < min`（0 < 0.175）となりスライダの表示レンジが破綻する
  （`effectiveRange` は LIMITS 受信後に自己修復するが、接続前の数百 ms は壊れる）。
- **影響**: 「スライダが動かなくなる」は**表示・入力レンジのみ**で、
  モータへ効くクランプは `io_node` 側。
- **修正案**: 生成側の既定を 0.47167 にそろえるか、Pi 側を真にして生成側だけ直す。

#### CFG-12 [H] `axle_model`（車軸ごとの摩擦）が到達不能
- **判定**: `CONFIRMED`
- **場所**: `sim/vehicle.py:193,203-263,806,856` / `raspi/auto/slam2d_raceline.py:683-684,2479-2484` / `PROGRESS.md:50-56`
- **内容**: `grep -rn "axle_model"` の**コード上**のヒットは `sim/vehicle.py` の 4 行と
  `PROGRESS.md` のみ（レビュー文書は自己参照でヒットする）。
  `VehicleSpec.load()` に **`axle_model` を設定する行が無い**（実測で常に `False`）。
  `raspi/auto/slam2d_raceline.py:683-684` の `grip_shape` の既定も 0 で、
  `_axle()` は `grip_shape <= 0` で `None` を返す。
  `PROGRESS.md:50-56` は「`axle_model` を足した」と書き、
  同じ節の結果表は `axle_model=True` が前提。**現在のコードではその比較を再現できない。**
- **影響**: 検証済みの記録（PROGRESS.md）が実コードと一致しない。
  `grip_shape=1` は GUI で選択可能だが判定に使う `rear_load_frac` は固定値（CFG-02）。
- **修正案**: `VehicleSpec.load()` に `axle_model` を明示的に設定する行を作る
  （例 `[sim] axle_model = true`）。`PROGRESS.md` の比較がどの設定で行われたかを記録し直す。

#### CFG-13 [M] `convert.py` のスケール定数 16 個が手書きで `protocol.toml` と非照合
- **判定**: `CONFIRMED`（**個数は 16**。機械照合結果は**全一致**）
- **場所**: `raspi/msgs/convert.py:29-51,151` / `raspi/proto/generated/packets.py:197,487` / `raspi/tests/test_proto.py:480-493`
- **内容**: 手書きスケールは **16 個**（`_SPEED, _YAW_RATE, _ANGLE, _ODOM, _ACCEL, _CURRENT,
  _TORQUE, _SLIP, _Z_RMS, _RHO, _BATT_V, _BATT_A_DRIVE, _BATT_A_SIGNAL, _ULTRASONIC,
  _LIDAR_MM, _LIDAR_C`）。`Command.accel_limit` / `steer_rate_limit` だけ
  `packets.Command.META` を参照。**機械照合（36 項目）結果は不一致 0 件**。
  テスト `TestScaleMetadata` は 4 項目だけを**リテラル値**と照合し、
  **`convert.py` の定数とは一度も照合していない**
  （元指摘の `test_msgs.py:481-493` はファイル名・行番号ともに誤りで、
  実際は `test_proto.py:480` の `TestScaleMetadata`）。
- **影響**: `protocol.toml` の `scale` を 1 つ変えると `generate.py --check` は**合格**し、
  `convert.py` は 10 倍ずれた値をバスに流す（**VER-09 で実証**）。
  症状は必ず「値が 10 倍おかしい」形でしか出ない。
- **修正案**: `convert.py` の全スケールを `packets.Telemetry.META[...]` から引く
  （`Command` と同じ方式）。全項目一致済みなので**破壊的変更ゼロで直せる**。

#### CFG-14 [L] 制御パラメータの ON/OFF だけが片側追跡で、toml の値がメモリで消える
- **判定**: `CONFIRMED`（事実）／`SEVERITY-OFF`（Medium → **Low**）
- **場所**: `raspi/core/control_params.py:94,99-104,167-172,253-262` /
  `raspi/nodes/io_node.py:842-853,466` / `config/vehicle.toml` `[control]`
- **内容**: `REQUIRED` に `ENABLE_KEYS` が入らないため `[control].tc_enable` を書き忘れても
  strict でエラーにならない（実測で確認）。`abs_enable` / `wheel_lift_guard_enable` は
  **`[control]` に存在しない**（実キーは `tc_enable` / `tv_enable` のみ）ため、
  `set_wanted()` が `pid not in self._base` で**早期 return（no-op）**する。
  `set_wanted()` は `self._base[pid] = float(value)` で**toml 由来の値をメモリ上で破壊**する
  （再起動で toml に戻るため、GUI と toml が正反対の状態で走ることがある）。
- **影響**: 実害は小さい。firmware 既定は `TC_ENABLE=無効` /
  `WHEEL_LIFT_GUARD_ENABLE=有効` / `ABS_ENABLE=有効`（`ras_link.h:219-224`）で、
  `io_node.py:466` `_resend_ui_settings()` が STM32 再起動後に GUI 設定を返し送る。
  問題は「どちらが正か」が契約として定義されていないこと。
- **修正案**: `set_wanted()` を `_base` 破壊から `_overrides` 側へ移す。
  `abs_enable` / `wheel_lift_guard_enable` を `[control]` に実在させる（無ければ
  `ENABLE_KEYS` から外し `io_node` の `set_wanted` 呼び出しを削除して
  `_send_ui_setting` のみに統一）。`REQUIRED` に ON/OFF を入れる。

#### CFG-15 [M] 「学習値を消す（grip_clear）」の契約に版・状態が無い
- **判定**: `PARTIAL`（版が無いのは TRUE。**seq の説明は逆向き**。Critical → **Medium**）
- **場所**: `raspi/msgs/types.py:588-592` / `raspi/nodes/telemetry_node.py:663-664,2702-2720` /
  `raspi/nodes/planning_node.py:226,231,246-252` / `raspi/auto/mapstore.py:182,473-475` /
  `gui/src/types.ts:405-418` / `gui/src/ws/control.ts:131,382` / `raspi/msgs/schema.py:37-46,65`
- **内容**: `AutoCtrl` に `grip_clear_map` / `grip_clear_seq` があるが、
  `AutoCtrl` は `MIRRORED` に含まれないため**版が変わらない**。
  `grip_clear_seq` は「回数」であり `planning_node.py:226` が `>` で比較する。
  telemetry_node 再起動で `_auto_grip_clear_seq` が 0 に戻るため、
  **再起動後は「押した回数が最後に処理した回数を超えない限り `grip_clear` が False」**になり、
  **planner 側の `request_clear_grip` が呼ばれない**（元主張とは逆方向の欠落）。
  また `planning_node.py:246` の `self.planner is not None` ガードで、
  planner 未ロード時は**seq が消費されたうえで再送されない**。
  `MapFile.has_grip` は `gui/src/types.ts` の手書き型で Python 側と型的に結び付いていない。
  `/ws/control` には版が無い（`control.ts:36` の「境界の実行時検証は `/ws/telemetry` 側の札が
  担っている」は**過度に楽観**）。
- **影響**: 「消したのに古い値が戻る」余地が残る。GUI↔Pi の型が壊れても
  コンパイルエラーにならない。
- **修正案**: `seq` を「世代（起動 UUID）+ 連番」にする。planner 未ロード時は
  **保留**し、planner 生成時に再適用する。`maps_clear_grip` / 応答に要求 ID と schema を付ける。

#### CFG-16 [M] `/ws/control`（GUI↔Pi の JSON）に版・単位の契約が無い
- **判定**: `CONFIRMED`
- **場所**: `gui/src/ws/control.ts:32-50` / `raspi/msgs/schema.py:37-46,65` / `gui/src/App.tsx:92-98`
- **内容**: `ServerMsg` に版フィールドが無い。`MSGS_SCHEMA` は `MIRRORED` の 8 型だけを対象にし、
  `ControlStatus` / `MapFile` / `LogFile` / `CmdOut` は対象外。
  `status` のみ `schema?: number`（`gui/src/types.ts:306`）を持ち `App.tsx:92-98` で照合される。
  `logs` / `cam_models` / `maps` / `maps_clear_grip_result` 等は `as` キャストのみで**無検査**。
- **影響**: 新機能の `has_grip`（CFG-15）や地図一覧の追加 field は
  この非対称の Healing 側に乗る。型が壊れても検知できない。
- **修正案**: `/ws/control` の全メッセージに `schema` を載せ GUI 側で 1 箇所で照合する。
  単位はフィールド名の接尾辞（`_m_s`, `_rad`, `_m_s2`）で機械的に分かるようにする。

#### CFG-17 [M] `[sim]` の 4 キーは常に `[limits]` に上書きされる
- **判定**: `CONFIRMED`
- **場所**: `sim/vehicle.py:211-220` / `config/vehicle.toml:135-149`
- **内容**: `[limits]` に truthy なキーがあれば
  `mu` / `drive_accel_m_s2` / `speed_decel_m_s2` / `brake_decel_m_s2` の
  **4 つすべてを上書き**する（実測: `VehicleSpec.load()` は
  `mu=0.41704` / `drive_accel=3.2175` / `speed_decel=2.2488` / `brake_decel=2.165`）。
  toml のコメントは「[limits] が無いときの予備」と書いているが、
  **キー名 `mu` が同じ**なので読者は「実車側の μ を編集している」と誤解する。
  `[sim].brake_decel_m_s2 = 2.4744` 対 `[limits].abs_brake_decel_m_s2 = 2.165` で **12.5% 差**。
- **影響**: 「予備」と理解して編集した値が黙って無視される。
- **修正案**: `[sim]` の 4 行を削除して `[limits]` に一本化するか、
  「[limits] があるときは [sim] は無視される」ことを**キー名ではなくコメントと検査で強制**する。

#### CFG-18 [M] `[limits]` / `[sim]` / `[control.plant]` の相互一致テストが無い
- **判定**: `CONFIRMED`
- **場所**: `raspi/tests/test_vehicle.py:51` / `raspi/tests/test_speedctl.py:41,72` / `raspi/tests/test_control_params.py:75,120`
- **内容**: `[limits]` を読むテストは散在するが、
  **相互比較のテストは 0 件**。その結果、CFG-01 の矛盾（μ=1.3227 なのに a_lat=4.0912）が
  そのまま検出されず HEAD に居る。
- **修正案**: `raspi/tests/test_vehicle_consistency.py` を追加し
  ①`|limits.a_lat/g − sim.mu| ≤ 1%` ②`|limits.a_lat/g − control.plant.mu| ≤ 5%`
  ③`sim.brake_decel == limits.abs_brake_decel` ④`fwsim.Plant.load().lateral_accel_max == limits.a_lat`
  を assert する（②と④は**現状落ちる**）。

#### CFG-19 [M] `t_capture` は「STM32 換算値」ではないことがある
- **判定**: `CONFIRMED`
- **場所**: `raspi/msgs/types.py:8,11` / `raspi/core/bus_bridge.py:81-84,118-126` / `raspi/core/timesync.py:208-235,245-255`
- **内容**: docstring は「`t_capture` は STM32 が測った時刻を時刻同期で Pi の
  `CLOCK_MONOTONIC` に換算した値 [ns]」と契約しているが、
  `bus_bridge.py:81-84` は**同期未収束・未来・古すぎ（>200 ms）の 3 条件で
  Pi の受信時刻に差し替える**。`_sector_start_ns` も同じ判定。
  なお `to_pi_ns` は回帰が成立しなくても `offset_ns`（素朴推定）にフォールバックして値を返す
  （「4 サンプル未満なら None」は**回帰についてのみ** TRUE）。
- **影響**: 同じフィールドに 2 種類の時間基が混ざる。契約の記述と実装が一致していない。
  時刻を使う側（`planning_node` の `cap_age`、SLAM の pose 推定）に影響。
- **修正案**: `t_capture` に**由来フラグ**（`t_capture_source`）を持つか、
  換算不能時は `None` にして消費側で区別できるようにする。bus_bridge の fallback では
  基点が変わることを明示する。

#### CFG-20 [L] STM32 再起動の検出が 2 系統ある
- **判定**: `CONFIRMED`（ただし**冗長ではなく補完**）
- **場所**: `raspi/core/link_tracker.py:193-199` / `raspi/core/timesync.py:82-105` / `raspi/nodes/io_node.py:441-465`
- **内容**: 系1 は `t_us` の 1 秒以上の後退（`_RESET_BACKWARD_US`）で検出。
  系2 は「こちらが `VERSION_REQ` を出していないのに来た VERSION バースト」を再起動とみなす
  （= `t_us` 後退では見えない短時間再起動用）。**2 系統とも `io_node._on_time_reset` に収束する。**
- **影響**: 大きな不整合はない（`sync.resets` を「STM32 再起動回数」として使う消費者は無い）。
  設計意図が**文書に無い**だけ。
- **修正案**: 1 系統にまとめるのではなく、
  「再起動検出は 2 系統（`t_us` 後退／VERSION バースト）、収束点は `io_node._on_time_reset`」と明記。

#### CFG-21 [L] 時刻同期に future-jump の余地が残る
- **判定**: `PARTIAL`（bus_bridge 層では**双方向をガード**している）
- **場所**: `raspi/core/bus_bridge.py:81-84` / `raspi/core/timesync.py:82-105,176-195`
- **内容**: `bus_bridge` は未来方向（`t_pi_ns > now`）と過去方向（`now - t_pi_ns > 200 ms`）を
  **両方**弾いて `now` に差し替える。残る穴は `timesync.py` 側で、
  **STM32 時計の後退は 1 秒以上だけ検出**する（1 秒未満の後退は検出されない）。
- **影響**: 1 秒未満の時計後退では回帰に混ざって `t_capture` が数十 ms〜数百 ms ずれる。
- **修正案**: `observe_stm_us` の後退検出を「1 秒以上」に限定せず、
  回帰の残差が跳ねたときにサンプル窓を破棄して再収束させる。

#### CFG-22 [M] `toml_io.apply` に範囲検証・fsync・バックアップが無く、モードが 0600 に落ちる
- **判定**: `CONFIRMED`（**実測でモード 644 → 600 を確認**）
- **場所**: `tools/calib/toml_io.py:37-45,48-76` / `raspi/auto/mapstore.py:227-238` /
  `raspi/core/control_params.py:160-161` / `raspi/nodes/io_node.py:747-748,1326-1330` /
  `tools/fwsim/tuning.py:59-70`
- **内容**: `apply()` は `tomlkit.dumps` → `tempfile.mkstemp` → 書込 → `os.replace` だが
  **`f.flush()` / `os.fsync()` が無く**、ディレクトリエントリの fsync も無く、
  **バックアップを取らない**。`mkstemp` は **0600** なので
  `os.replace` で**元のパーミッションが失われる**（実測: 0644 → 0600。
  実際、リポジトリの `config/vehicle.toml` は既に `-rw-------`）。
  対比: `mapstore.py:227-238` は `f.flush(); os.fsync(f.fileno())` をしている。
  `tools/fwsim/tuning.py:59-70` は**範囲検証している**が `toml_io.apply()` は**していない**。
- **影響**: 書き込み直後に電源が切れると `vehicle.toml` が空/破損し得る。
  破損 → `load_control_params()` が `TOMLDecodeError`（`ValueError` 派生）→
  `io_node.py:1326-1330` `control_params_error` → `:747-748` により **ARM 全面ブロック**（走行不能）。
  復旧には手動で toml を戻す必要がある。
- **修正案**: `flush` → `fsync` → `os.chmod(tmp, 元モード)` → `os.replace` → 親ディレクトリ fsync。
  範囲検証を `tools/fwsim/tuning.py` と同じ方式（表駆動）にする。
  apply 前に `vehicle.toml.bak-<ts>` を作る。`_add_after_last_key` のロジックも確認する。

#### CFG-23 [L] `docs/development.md` が生成器の現状を逆に説明している
- **判定**: `CONFIRMED`
- **場所**: `docs/development.md:76,857` / `config/gen_msgs.py:22-36,57` / `gui/src/types.ts:4-13,22-30`
- **内容**: doc は「`gui/src/types.ts` も手で直す（写しなのでズレる）」
  「**GUI にも見せるなら `gui/src/types.ts` を手で写す**（**生成器は挟んでいない**）」と書くが、
  実体は `config/gen_msgs.py` が `gui/src/generated/msgs.ts` を生成し、
  `types.ts` は `export type {...} from './generated/msgs'` で**再エクスポート**している。
- **影響**: 手順書が逆。新人が「手で直す」ことをSISITE する。
- **修正案**: 「`config/gen_msgs.py` が生成し、`gui/src/types.ts` は Python に対応物が無い型だけ手書き」
  と書き直す。`MapFile` は Python 側に実装があるので生成対象へ移せる。

#### CFG-24 [L] toml スキーマの説明が古い
- **判定**: `PARTIAL`（`docs/architecture.md:167` が該当。`system_overview.md:419` は別の内容）
- **場所**: `docs/architecture.md:167` / `config/vehicle.toml`（実在セクション）
- **内容**: doc はスキーマを「幾何・質量・センサ取付位置・`[dynamics]`・`[control]`・`[safety]`」と説明。
  実在する `[limits]`・`[sim]`・`[control.plant]`・`[sensors.*]` が**説明に無い**。
- **修正案**: 3 節を追加し、「`[limits]` が唯一の走行限界の正、`[sim]` は `[limits]` から作られる」等の
  関係を 1 行で明記する。

#### CFG-25 [L] `sim/park_world.py` の角度が 26.5°（正は 27.02°）
- **判定**: `CONFIRMED`
- **場所**: `sim/park_world.py:14-15`
- **内容**: 「`wheelbase/tan(max_steer) = 0.23/tan(26.5°) = 0.46m`」に対し、
  実 `max_steer = 0.47167` = **27.0247°** → `0.23/tan(27.0247°) = 0.4509 m`。
  同ファイルの「最小旋回半径 1/Vehicle.kappa_max ≒ 0.48m」も
  `kappa_max` は servo gain と offset を適用するため 0.4681 m で**一致しない**。
- **修正案**: 実行時に `Vehicle.kappa_max` から計算して書く形にすれば再計算不要。

#### CFG-26 [L] `install_services.sh` の `--max-ste 0.524` は実害が無い
- **判定**: `CONFIRMED`
- **場所**: `raspi/setup/install_services.sh:94` / `raspi/nodes/io_node.py:1335,164`
- **内容**: `max_steer=min(args.max_steer, VEHICLE_MAX_STEER)` で 0.47167 に丸められる。
  実車の最終クランプは STM32 の `LIMITS.max_steer_rad`。
  ただし GUI の `PI_MAX_STEER_CAP = VEHICLE.maxSteer = 0.47167` に対し
  `install_services.sh` は 0.524 で**コメント「同 --max-steer」も実態と不一致**。
- **修正案**: 値を 0.47167 に直す（または引数を省略して既定に任せる）。

---

### 3.3 検証網（CI・テスト・検査スクリプト）

#### VER-01 [H] CI が削除済みディレクトリを参照し、2026-10-19 以降ずっと赤
- **判定**: `CONFIRMED`
- **場所**: `../.github/workflows/ci.yml:61` vs `tools/check.sh:61-62`
- **内容**: CI は
  ```yaml
  run: python -m pytest tools/tests tools/sysid/tests/test_link.py -k "not TestAnalyzeFolds" -q
  ```
  `tools/sysid` と `tools/ctrl_tune` は commit `769e847`（**2026-10-19 11:13**）で削除済み。
  実測: `pytest tools/tests tools/sysid/tests/test_link.py --collect-only -q` →
  `ERROR: file or directory not found` / **EXIT=4**。
  一方 `tools/check.sh:61-62` は**同一コミットで**更新済み
  （`tools/tests tools/calib/tests tools/fwsim/tests/test_fw_constants.py tools/fwsim/tests/test_control_logic.py`）。
  `ci.yml` は 2026-10-14 以降更新されていない。
  **対で保つべき 2 ファイルの片方だけが追従**している。
- **影響**: CI が赤でも「見て無視」される習慣が生まれる。
  ただし **CI の全工程が無効化されるわけではない**。
  python job は `--check` 4 本 → ruff → `raspi/tests` 1667 件を**先に実行して成功**しており、
  失敗するのは**最後の tools pytest ステップのみ**（`tsc -b` は別 job なので影響を受けない）。
  実害は「job が赤になる」ことと、
  さらに **CI には `test_fw_constants.py` が入っていない** →
  ファームと `vehicle.toml` の定数を突合する**唯一の自動検査がどこからも走らない**こと。
  > **注（第2ラウンド再検証で訂正）**: 深刻度 Critical → **High**。
  > 「全検査が無効化」は事実と反する。
- **修正案**: CI を `tools/check.sh` と一本化する。firmware リポジトリを副チェックアウトして
  `SURGE_FW_DIR` を渡す。firmware が無い場合は **skip ではなく fail**（または CI ログに明示）。

#### VER-02 [H] CI が firmware との突き合わせを回さない
- **判定**: `PARTIAL`（「CI に無い」は TRUE。「firmware 不在なら必ず skip」は `test_control_logic.py` について **FALSE**）
- **場所**: `ci.yml:57,61` / `tools/fwsim/tests/test_fw_constants.py:60` /
  `tools/fwsim/tests/test_control_logic.py:24,37,56,267`
- **内容**: CI の pytest 列挙は `raspi/tests` と `tools/tests tools/sysid/...` のみ。
  `test_fw_constants.py` は `skipUnless((FW_DIR / HEADERS[0]).exists())` なので
  **CI では必ず skip**（実測 10 件すべて skip）。
  しかし `test_control_logic.py` の **`TestParamTable` には skip デコレータが無く**、
  firmware 不在時に **3 件 fail する**（実測: `3 failed, 28 skipped`、理由は
  `tools/fwsim/fw.py:191` の `FileNotFoundError`）。
- **影響**: 「firmware 定数の二重定義を捕まえる検査は、どこでも実行されていない」。
  ただし `test_control_logic.py:43-49` の
  `test_vehicle_toml_is_within_firmware_range` は**まさに SAF-13 の経路を検査する契約テスト**。
- **修正案**: `TestParamTable` にも `skipUnless(HAVE_FW)` を付け、そのうえで
  CI の pytest 列挙に追加する（CI では skip されるだけなので緑のまま）。
  本当に firmware 突合を CI で回したいなら firmware ツリーを checkout する step を追加。

#### VER-03 [H] `sim/tests` が CI に無く、sim のテストが master で 2 件赤
- **判定**: `CONFIRMED`（失敗メッセージ・件数とも完全一致）
- **場所**: `ci.yml`（`sim/tests` の記載なし）/ `sim/tests/test_speed_ff.py:158,171,182,185`
- **内容**: 実測 `pytest sim/tests/test_speed_ff.py -q` →
  ```
  F...F..
  AssertionError: 0.006565553195410745 not greater than 0.08      (:171)
  AssertionError: np.float64(0.10564583079382661) not greater than 0.35  (:185)
  2 failed, 5 passed
  ```
  スイート全体は **`2 failed, 111 passed`**。
  **master の時点でシムのテストが 2 件落ちている**。
  > **注（第2ラウンド再検証で訂正）**: 原因コミットは `10cd778` ではなく
  > **`98f0288`（2026-10-19「車両パラメータを校正で測る形に整理…」）**。
  > 二分探索で確定：`10cd778^`（kp 1.13724 / ki 4.26466）でも同じく 2 件落ち、
  > `4c09b8a` までは 7 passed。
  > `10cd778`（2026-10-20 の kp 1.15716 / ki 4.33934）は再校正の追随であり無関係。
- **影響**: シム⇄実機の一致を保つ唯一の網が破られている。
  さらに `ci.yml` に `sim/tests` が無いので**落 communionは CI でも unnoticed**。
- **修正案**: ① シムの plant に 1 次遅れ（`md_delay_s`/`md_tau_s`）を実装、
  ② 2 件のテスト期待値を再ベースライン、③ **`sim/tests` を CI に追加**、
  ④ **「シムのテストが赤なら master にはマージしない」**を CI に強制する。

#### VER-04 [H] ML テストスイートが同時収集できず、356 件が一度も CI で走っていない
- **判定**: `CONFIRMED`（全サブ主張一致）
- **場所**: `ml_cam/tests/`・`ml_cam_e2e/tests/`（**`__init__.py` 無し**）/
  `ml_lidar/tests/__init__.py`・`ml_common/tests/__init__.py`（あり）/
  `tools/check.sh:56,61-62` / `ci.yml`
- **内容**: 実測 `pytest ml_cam/tests ml_cam_e2e/tests --collect-only -q` →
  ```
  ERROR collecting ml_cam_e2e/tests/test_app.py
  import file mismatch: imported module 'test_app' has this __file__ attribute:
    .../ml_cam/tests/test_app.py
  which is not the same as the test file we want to collect:
    .../ml_cam_e2e/tests/test_app.py
  …（test_dataset / test_export_onnx / test_model も同じ）
  Interrupted: 4 errors during collection
  175 tests collected, 4 errors
  ```
  個別に収集すると `ml_cam/tests` 99 件 + `ml_cam_e2e/tests` 126 件 +
  `ml_lidar/tests` 100 件 + `ml_common/tests` 31 件 = **356 件**。
  `tools/check.sh` と `ci.yml` のどちらにも `ml_` は**1 件も無い**。
- **影響**: **356 件が CI でもローカル check でも一度も実行されていない**（個別に走らせれば全て pass する）。
  ML-03 / ML-04 / ML-06 / ML-07 の回帰を捕まえない。
- **修正案**: `ml_cam/tests/` と `ml_cam_e2e/tests/` に `__init__.py` を追加
  （`ml_lidar/tests`・`ml_common/tests` と同じ方式）。そのうえで 4 つを CI に追加。
  さらに `ml_cam/dataset.py` と `ml_cam_e2e/dataset.py` のフラット import
  （`from dataset import ...`）による `sys.modules["dataset"]` 衝突も解消する。

#### VER-05 [L] `config/check_docs.py` の検査範囲が狭い
- **判定**: `CONFIRMED`
- **場所**: `config/check_docs.py:18,36,42-49`
- **内容**: `CLAIMS` は**版番号の 4 エントリだけ**
  （`uart_protocol.md` 1 / `stm32_interface.md` 1 / `README.md` 2）。
  docstring も「**版番号だけ。**」と明記。走査先は `docs/` 固定で `site/` は対象外。
  数値・リンク切れ・インライン版番号・削除済みパスの参照を検査する機構は**無い**。
  実測で削除済みモジュールへの参照が実在する（`tools/slam_replay_gui.py:10` の `tools/sysid/gui.py`、
  `raspi/nodes/planning_node.py:348` の `raspi/auto/sysid_*.py`）。
  `config/check_docs.py --check` は現在 **exit 0**（v0.29 で PASS）。
- **影響**: OPS-03 の 8 項目の陳腐化がいずれも**機械的に検出できない**。
- **修正案**: ① `docs/stm32_interface.md` の `LEN = (\d+)` 行と `packets.py` の LEN の照合、
  ② インラインの `0x001[0-9A-F]` の照合、③ `site/docs/` も対象に含める、
  ④ `docs/control_constants.md` の主要数値を `config/vehicle.toml` から生成した表と照合、
  ⑤ 削除済みパスを参照していないことの検査。
  **注意**: `docs/` には旧版の review md が大量に残っており歴史的引用に偽陽性が出るため、
  検査対象を絞るか引用行の除外規則が要る。

#### VER-06 [M] `tools/calib/toml_io.py` に単体テストが無い（0 件）
- **判定**: `CONFIRMED`
- **場所**: `tools/calib/toml_io.py`（76 行）/ `tools/calib/tests/`（8 ファイル、44 件）
- **内容**: `grep -rn "toml_io" tools/calib/tests/` → **ヒット 0 件**。
  `apply()` は `config/vehicle.toml` を `tomlkit` で**破壊的に書き換える**部品である。
- **影響**: 単一の source of truth を書き換える部品が**テスト 0 件**。
  CFG-22 のモード変化やコメント移動（`_add_after_last_key` は 2026-10-19 に
  「`md_delay_s` が注釈の下に入った」バグを直した箇所）が再発する。
- **修正案**: `tools/calib/tests/test_toml_io.py` を新設し、
  ①`read_flat` の平坦化と型変換（bool を数値にしない）②差分表示
  ③**注釈・並び順を保つ** ④冪等性（2 回適用で changed が空）⑤int/float 正規化 をテスト。
  `apply` は `config/vehicle.toml` を直接壊すため `tmp_path` に複製して行う。

#### VER-07 [H] firmware の `TELEMETRY` 並びと `RAS_LEN_*` を検査するテストが無い
- **判定**: `CONFIRMED`（param_id 40 個は一致＝**現状の実害はゼロ**。構造的リスク）
- **場所**: `raspi/proto/generate.py:7-13` / `tools/fwsim/tests/test_fw_constants.py:30,109-113` /
  `tools/fwsim/tests/test_control_logic.py:38-41` / ファーム `src/comm/ras_link.c:19-39,257-304` / `src/comm/ras_link.h:27`
- **内容**: `test_fw_constants.py` が検査するのは
  `HEADERS = (drive.h, vehicle.h, lidar.h, speed_kf.h)` の `#define` 値と param_id の一部。
  **`src/comm/ras_link.c` は一度も読まれない**（`grep -rn ras_link --include=*.py` の該当は
  `raspi/proto/generate.py:7` の docstring 1 件のみ。**読むコードは 0 件**）。
  検査されていないのは `RAS_LEN_TELEMETRY` などの**バイト長**と
  `SendTelemetry()` の**パッキング順序**。
  実測: `SURGE_FW_DIR=... pytest tools/fwsim/tests/ -q` → **45 passed, 45 subtests passed**、
  param_id 40 個すべて一致（**今は正しい、未来を守っていない**）。
- **影響**: firmware 側で TELEMETRY のフィールドを並べ替えると
  `RAS_LEN_TELEMETRY` は 102 のままで**以後の全フィールドが黙って誤デコード**される。
  速度・`tc_slip`・`front_angle`・`torque_cmd`・`odom_dist` が全部壊れる。
  校正系は二次的な症状でしか察觉できない。
- **修正案**: `tools/fwsim/tests/test_ras_link_layout.py` を追加し、
  `ras_link.c` の `RAS_LEN_*` と `protocol.toml` の `len` を突き合わせ、
  TELEMETRY は `Put*` の並び順（型・順序・個数）を機械的に抽出して `packets.FMT` と一致させる。
  （firmware 側 `SendTelemetry` は `ras_link.c:257-305`）
  中長期的には firmware に `surge_proto.h` を取り込む。

#### VER-08 [M] firmware 側プロトコルが「唯一の定義」から外れている
- **判定**: `PARTIAL`（`raspi/proto/README.md:57` は該当。**リポジトリ直下 `README.md` は該当せず**）
- **場所**: `raspi/proto/README.md:1-13,57` / `raspi/proto/generate.py:7-13` /
  `raspi/proto/generated/surge_proto.h` / ファーム `src/comm/ras_link.c:1-8` / `src/comm/ras_link.h:27`
- **内容**: `surge_proto.h` は**生成物としてコミットされ、C11 でコンパイル可能**
  （`cc -std=c11 -Wall -Wextra -c` → 0、`_Static_assert` 13 個すべて通る）。
  しかし firmware は**これを取り込んでいない**（`grep -rn surge_proto` → 0 件）。
  `RAS_LEN_TELEMETRY 102` は `ras_link.c:19`、`RAS_PROTOCOL_VERSION 0x001Du` は `ras_link.h:27` に**手書き**。
  `raspi/proto/README.md:3` は「`protocol.toml` が**唯一の定義**」と書くが実態は両側に手書き。
  `raspi/proto/README.md:57` は削除済み `tools/ctrl_tune/tests` を「唯一の正の照合」と記す。
  `docs/control_constants.md` は削除済み `tools/sysid`（6 箇所）/ `tools/ctrl_tune`（2 箇所）
  = **計 8 箇所**で参照し（`sysid` を含む行は 9 行）、
  `:34` が**存在しないキー `[dynamics] brake_decel_m_s2`**（実際は `[sim]`）を引用。
  同文書 §2（位相余裕の式）と §3（経験的ラダー）も互いに矛盾。
- **影響**: `protocol.toml` を編集して Pi 側だけを再生成しても firmware は古いまま →
  「値が化ける」型のバグが再発しうる。
- **修正案**: `raspi/proto/README.md` の「唯一の定義」を実態に合わせて書き直す。
  firmware ツリーを submodule / CI で読めるようにし、firmware 側の `RAS_LEN_*` /
  `RAS_PROTOCOL_VERSION` を `protocol.toml` と照合する検査を足す。
  `docs/control_constants.md` の削除済みツール参照と非-existent キー参照を直す。

#### VER-09 [M] 「正の伝播網」に検査が 4 本しか無い（10 倍ずれを実証）
- **判定**: `CONFIRMED`（**再現済み**）
- **場所**: `tools/check.sh:37-46` / `raspi/msgs/convert.py:29-51,151` / `raspi/proto/protocol.toml`
- **内容**: 検査 4 本は `config/generate.py --check` / `raspi/proto/generate.py --check` /
  `config/gen_msgs.py --check` / `config/check_docs.py --check`。すべて exit 0。
  **実証手順（リポジトリ外の一時コピー）**:
  ① `protocol.toml` の TELEMETRY `speed` の `scale = 0.001` → `0.01` に書き換え
  ② `generate.py` で再生成 ③ `generate.py --check` → **exit 0（合格）**
  ④ 実際のデコード: `packets.Telemetry.META['speed'][0] = 0.01` に対し
  `convert.py:151` は手書き `_SPEED = 1e-3` を使うため、raw=1000 が **1.0 m/s**
  （仕様どおりなら 10.0 m/s）＝ **10 倍ずれ**。
  **どの検査もこれを検出しない。**
- **影響**: このプロジェクトで実際に起きた不具合は**すべてこの穴**に落ちている（§4.1 の表）。
- **修正案**: `convert.py` のスケールを `packets.Telemetry.META` から読む。
  `tools/check.sh --fast` に「`protocol.toml` の scale と `convert.py` の定数が一致」を見る
  検査（`config/check_scales.py` 等）を追加。

#### VER-10 [M] `replay_node --bus` が `cmd` を publish しない不変条件が未テスト
- **判定**: `CONFIRMED`
- **場所**: `raspi/nodes/replay_node.py:9-12,33-50,316-318,331` / `raspi/tests/test_replay_node.py`
- **内容**: `replay_node` は `TOPIC_CMD` を import していない。
  docstring は「**これが事実上のシミュレータ代わり**（`docs/architecture.md` §11）」と記す。
  `raspi/tests/test_replay_node.py` は合成 `.sfl` の再生と FrameLog 系の検証のみで、
  `--bus` 出力の publish 先を一切検証していない（`grep -n "cmd\|topic\|publish" ` → 0 件）。
- **影響**: 安全上最も重要な不変条件（**再生は車を動かさない**）がテストで守られていない。
- **修正案**: `--bus` 付きで起動して publish された topic 一覧を捕まえるテストを追加し、
  `cmd` が publish されないことを**負の assert** にする。docstring にも明記。

#### VER-11 [M] `E2ELidar.plan()` と `LidarE2EEnv._to_obs()` の parity が未テスト
- **判定**: `CONFIRMED`
- **場所**: `raspi/auto/e2e_lidar.py:239,266-275,285-295` / `ml_lidar/env.py:466` / `ml_lidar/tests/test_env.py:143-177`
- **内容**: `grep -rn "_to_obs" raspi/tests/ ml_lidar/tests/` → **ヒット 0 件**。
  `e2e_lidar.py:266-275` のコメントは「訓練側（`ml_lidar/env.py` の `_to_obs()`）と**揃える**」と
  述べるが、**テストは観測の形・範囲・連続性しか見ていない**。
  正規化係数（`max_range` / `max_speed` / `max_steer`）の取り違えは現状テストで捕まらない。
- **影響**: ML-01 / ML-02 の学習・推論ずれを捕捉不了。
- **修正案**: 同じ点群・同じ車速・同じ舵角を両側に投入し、
  観測ベクトルの全要素が `assert_allclose` で一致することを assert する parity テストを 1 本追加。

#### VER-12 [L] 生成器の `--check` を叩く入口が二重管理されている
- **判定**: `CONFIRMED`
- **場所**: `tools/check.sh:2,37-42,49-68` / `ci.yml:44-54`
- **内容**: `--check` 4 本の列挙は**完全に同一**。ruff の指定も同一。
  差分は **pytest 行だけ**（`check.sh:61-62` と `ci.yml:61`）と、
  `check.sh` 固有の `tsc -b` / `--fast`。
  `check.sh:2` には「CI と**同じ検査**をローカルで走らせる。`.github/workflows/ci.yml` と
  対で保つこと」とあるが、**tools pytest の列挙だけ対になっていない**。
- **修正案**: `tools/check.sh` を唯一の定義にして CI から呼ぶ形にする。
  あるいは pytest の対象パスを 1 ファイル（`pytest.ini` の `testpaths` 等）に切り出して
  両者が読むようにする。

#### VER-13 [M] `mapstore` の NaN 保存・並行書き込み・`import_upload` 順序のテストが無い
- **判定**: `CONFIRMED`
- **場所**: `raspi/tests/test_mapstore.py`（**34 件**）
- **内容**: `test_non_finite_line_is_rejected` は**raceline 配列**の NaN を確認するのみで、
  **`resolution` / `origin` の NaN は未テスト**。並行書き込みのテストは 0 件。
  `import_upload` の npz→routes 書込順序（途中クラッシュ）のテストも無い。
  一方 `test_upload_with_broken_routes_is_rejected_without_writing`（:266）、
  `test_meta_is_written_atomically`（:204）等は存在する。
- **影響**: SLM-05 / SLM-07 の退行を捕まえない。
- **修正案**: ①`save_map(resolution=NaN)` → 現状は `save_map` が finite 検証をせず
  **保存は成功し `load_map()` が `None` を返す**（検証は読み込み側の `_load_npz` にある）。
  「save 時に `ValueError` を出す」ようにするか、
  「`load_map()` が `None` を返す」契約としてテストする
  ②`origin_x=NaN` の同種
  ③2 スレッドで同時 `save_map` / `save_grip` ④`import_upload` 途中で例外注入して
  「旧 routes が新地図に残らない」ことを検証。

---

### 3.4 制御・経路追従（`raspi/nav/`）

#### NAV-01 [M] Pure Pursuit の曲率 FF 分岐が `|η|>90°` の全舵飽和を失う
- **判定**: `CONFIRMED`（機構・影響）／再現表の数値は `PARTIAL`（シナリオ依存）
- **場所**: `raspi/nav/purepursuit.py:139-157,160-177,251-257` / `raspi/auto/base.py:204-225` /
  `raspi/auto/slam2d_raceline.py:533-536,567` / `raspi/tests/test_laptime.py:203,213` / `config/auto.json`
- **内容**: `steer_for_target()`（`:139-157`）は `abs(eta) > 0.5*math.pi` で
  `math.copysign(max_steer, eta)` を返す（docstring に理由明記：
  「式の `sinη` は 90° を超えると減るので、そのままだと後ろ寄りの狙点ほど舵が戻ってしまう」）。
  しかし FF 分岐（`:251-257`）は `steer_for_target` を**呼ばず**
  `delta = math.atan(cfg.wheelbase * kap)` を直接計算 → **飽和を失う**。
  **実測**（`Vehicle.load()` / `PursuitConfig(ff_gain=0/0.8, ff_chord=1.0)` /
  直線経路・点間隔 0.1 m・**車両は経路中央（index n//2）に置く**・
  `delay_s=0.15`・`look_k=0.8` / `look_min=0.35` / v=1.0 m/s）:
  | 航向誤差 | `ff_gain=0` | `ff_gain=0.8` | 差 |
  |---:|---:|---:|---:|
  | 30° | −13.039° | −13.039° | 0 |
  | 60° | −20.160° | −20.160° | 0 |
  | 90° | **−27.025°**（全舵） | −22.128° | 4.9° |
  | 120° | **−27.025°** | −18.582° | 8.4° |
  | 150° | **−27.025°** | −9.904° | 17.1° |
  | 179° | **−27.025°** | **+0.035°** | **27.1°** |
  `ff_gain=0` の 90° 以降は `max_steer = 0.47167` で**全舵**（原指摘の −27.02° と厳密一致）。
  > **注（第2ラウンド再検証）**: 上表は「車両を経路中央に置く」条件で得たもの。
  > 経路**端**（index 0）に置くと `_ff_correction` の端クランプで
  > 120/150/179° の ff=0.8 欄が **−17.201 / −9.178 / +0.068** に変わる
  > （ff=0 欄は不変）。原指摘の「179° で −0.35°」も setup 依存の値であり、
  > 走査条件が不明なため再現できない。
  なお原指摘の数式 `κ = (1−ff_gain)·2sinη/dist + ff_gain·κ_path` は
  「車両の η と経路点の η が一致する」場合に限り成立する近似で、**一般にはコードと一致しない**
  （「η→180° で舵が中立へ戻る」という結論自体は再現）。
  **hidden パラメータ**: `base.py:204-225` の `merged()` は
  `if v is None or (s.hidden and not hidden): continue` → hidden はユーザー値で上書きされない。
  実測: `merged_params("slam2d_route", {"steer_ff": 0.0})` → **0.8**（無視）。
  `slam2d_raceline.py:567` `ParamSpec(key="steer_ff", hidden=True, default=0.8)`。
  **保存済み `config/auto.json` の実値**: `mode="slam2d_route"`, `ctrl_mode=1`（**MPC**）,
  `steer_ff=0.8`, `obstacle_stop=0.0`, `obstacle_avoid=0`。
  → **現保存設定では Pure Pursuit 分岐自体が使われず**、
  NAV-01 が発火するのは「保存設定が無い環境」か `ctrl_mode` を 0 にしたとき。
- **影響**: ゴールが後ろにあると舵が中立へ戻り、U ターンできない。
  `ctrl_mode` の既定は 0（Pure Pursuit）なので**初期状態では発火しうる**。
  ただし発火条件は「`|η|>90°`（経路上の前方点としては異常時のみ）」と
  「`ctrl_mode=0`（保存済み設定は MPC）」の二重条件で、
  **保存済みの `config/auto.json` では発火しない**。
  なお U ターン用途は `park_to_point` の別実装が担い、Pure Pursuit を使わない。
  `raspi/tests/test_laptime.py:203,213` は `ff_gain>0` を使うが**前方のみ**を検証し、
  `|η|>90°` の検証が無い。
- **修正案**:
  ```python
  if cfg.ff_gain > 0.0 and n >= 5:
      kap = 2.0 * math.sin(eta) / max(dist, 1e-3)
      kap += cfg.ff_gain * _ff_correction(path, i, goal, step, cfg.ff_chord)
      delta = math.atan(cfg.wheelbase * kap)
      if abs(eta) > 0.5 * math.pi:                 # ★ steer_for_target と同じ飽和
          delta = math.copysign(cfg.max_steer, eta)
  ```
  併せて `steer_ff` の hidden を外して GUI から 0 にできるようにし、
  **実機未検証の MPC が既定で選ばれる運用**（`ctrl_mode=1`）を止める。

#### NAV-02 [M] `speed_loop_tau_s` が `docs/control_constants.md` と 2.25 倍食い違う
- **判定**: `CONFIRMED`（数値・文書不一致は確定。**High → Medium**。実害は NAV-03 と重複）
- **場所**: `raspi/core/vehicle.py:295-302` / `docs/control_constants.md:41` /
  `raspi/auto/slam2d_raceline.py:276,1291,1297` / `raspi/nav/speedctl.py:122-123`
- **内容**: 実測 `Vehicle.load()`: `speed_plant_gain=12.5758`, `speed_kp=1.15716` → `g=14.55221`,
  `τ = 0.06871807 s`。文書は **0.155** → **2.2556 倍**。
  停止余裕: `react = SCAN_PERIOD_S + cmd_latency + 0.5τ` = 実測 **0.14646 s**
  （文書値なら 0.1896 s）→ 差 **0.0431 s**、2 m/s で **8.6 cm**、3 m/s で 12.9 cm。
  ランプ: 実測（プロファイル 3.0→0.0、lead_s=0.0421）
  | 現在車速 | τ=0.068718 | τ=0.155（文書） |
  |---:|---:|---:|
  | 0.5 m/s | **36.31 m/s²** | 16.10 m/s² |
  | 1.0 m/s | **28.96** | 12.84 |
  | 2.0 m/s | **14.27** | 6.33 |
  > **注（第2ラウンド再検証で訂正）**: 先の `lead_s` は `0.0421` が正
  > （`_ref_lead()` = `0.5/CMD_HZ + cmd_latency + speed_ff_filter_s`
  > = 0.01 + 0.0121 + 0.02）。`0.0321` と書いていたのは誤り。
  > `0.0421` ではランプは 36.29 / 28.92 / 14.18（結論は不変）。
  `speed_ff_firmware`（`vehicle.toml:222` `speed_ff_nm_per_m_s2=0.079518`）が有効なので
  `ref_command` 系は実際に使われる。`LEAD_RATIO` 系の旧経路は不活性。
- **影響**: 文書が想定した値の 2.25 倍のランプ許される（NAV-03 と重なって
  **安全側ではなく危険な側に倒れる**）。停止余裕の不足は 8〜13 cm。
- **修正案**: `docs/control_constants.md:41` を 0.0687 に更新し、
  `config/check_docs.py` に「`speed_loop_tau_s` の式と docs 値の照合」を追加。
  `catch_s` を独立設定（例 `[control] speed_catch_s`）にして
  「PI の比例項の速さ」と「目標車速の傾きをどこまで許すか」を分離する。
  `react_s` の `0.5·τ` は τ の定義に依存するため文書と必ず同時に直す。

#### NAV-03 [M] `speedctl.ref_command` の目標速度ランプに上限が無い
- **判定**: `CONFIRMED`
- **場所**: `raspi/nav/speedctl.py:97-124` / `raspi/core/vehicle.py:78-87,188-190,195-199,283-284` /
  `raspi/auto/slam2d_raceline.py:1291,1296,1811` / `raspi/nodes/planning_node.py:503` /
  `raspi/nodes/telemetry_node.py:3244-3246` / `raspi/nodes/io_node.py:912` / `raspi/proto/protocol.toml:188`
- **内容**:
  ```python
  # raspi/nav/speedctl.py:122-124
  catch = catch_s if catch_s > 1e-6 else REF_CATCH_S
  ramp = max(REF_ACCEL_MIN, REF_ACCEL_GAIN * a, abs(v_ref - v_now) / catch)
  return max(0.0, v_ref), ramp
  ```
  `cmd_accel_limit_m_s2` は `Vehicle` 側で公開されているが
  **`raspi/nav/speedctl.py` は `Vehicle` を受け取らない**
  （`grep -rn "cmd_accel_limit" raspi/nav/` → **0 件**）。
  消費の流れ:
  ```
  speedctl.ref_command → st.accel_limit → auto/cmd → cmd
    accel = gui.accel_limit
    if auto.accel_limit > 0:
        accel = min(accel, auto.accel_limit) if accel > 0 else auto.accel_limit
  ```
  → **安全網は GUI の `min()` だけ**。GUI 既定は 3.0 で頭打ちだが、
  **`gui.accel_limit == 0`（=「指定なし」）のクライアントでは
  `accel = auto.accel_limit`（= 36.31 m/s²）がそのままワイヤに出る**。
  さらに下流の `io_node.py:912` `max_accel = lim.max_accel_m_s2`（STM32 `LIMITS`）も
  `protocol.toml:188` の注明どおり「v0.22 から STM32 は上限を掛けない」ため
  **36.31 は STM32 側でも切られない**。
- **影響**: **安全の網が GUI にある**構造。GUI 未接続は deadman で走行しないため
  実際の穴は「`accel_limit=0` を送るクライアント」（旧 GUI・第三者の WS クライアント）側。
  > **注（第2ラウンド再検証）**: 深刻度 High → **Medium**（影響が上記クライアントに限られる）。
- **修正案**: `ref_command` に `accel_max`（`Vehicle.speed_ramp_max_m_s2` = 3.0）を渡して
  `ramp = min(ramp, accel_max)` とする。Pi 側が正を持ち、
  `telemetry_node` の `min()` は多層防御として残す。

#### NAV-04 [M] 既定では planner が障害物に減速も制動もしない
- **判定**: `CONFIRMED`（挙動。**High → Medium**。意図的・文書化済み・STM32 `auto_stop` が第2層）
- **場所**: `raspi/auto/slam2d_raceline.py:594-605,1848,1930-1934,1941,1974,1977-1983` /
  `raspi/tests/test_route.py:981-985` / `raspi/msgs/types.py:371` /
  `gui/src/input/useDriving.ts:143-144` / `gui/src/store/ui.ts:236` / `raspi/auto/park_to_point.py:98`
- **内容**: `slow_on = p["obstacle_avoid"] > 0` / `stop_at = p["obstacle_stop"]` /
  `if stop_at <= 0.0 or gap > stop_at: ... return False`。
  既定（`ParamSpec` と保存済み `config/auto.json` とも）`obstacle_stop=0.0`, `obstacle_avoid=0`。
  **再現**:
  ```
  既定(閉)      → handled=False brake=False target_speed=1.5 reason=''
  既定(開)      → handled=False brake=False target_speed=1.5
  avoid=1(開)   → handled=True  brake=False target_speed=1.403
  ```
  docstring `:1930-1934` に明記され、`test_route.py:981-985` が
  「避けるを off にすると触らない」を**明示的に固定**している。
  守りは STM32 の `auto_stop`（`types.py:371`、GUI 既定 ON `ui.ts:236`）に依存する。
  `park_to_point.py:98` は「`auto_stop` は **OFF 運用が前提**」と明記 → **モードにより安全網の層数が違う**。
  近接高速パス（`:1848`）も**同じフラグで gate** され既定では無効。
- **影響**: Pi 側の安全網が 0 になる。「既定の安全機能がオフ」かつ
  「保存済み設定でもオフ」の状態。
- **修正案**: `slow_on` と `stop_at` を分離し、
  「`obstacle_avoid=0` かつ `obstacle_stop>0` で**止まるだけ**」を表現可能にする。
  既定 `obstacle_stop` を 0.8 m 程度へ見直す（GUI で露出済みなので既定だけ変えれば足りる）。

#### NAV-05 [M] 障害物検出の「2 周期確認」の +100 ms が停止距離に計上されていない
- **判定**: `CONFIRMED`
- **場所**: `raspi/nav/obstacles.py:95-97,159-169` / `raspi/auto/slam2d_raceline.py:254,276,1848,1850,1973-1975,2163`
- **内容**: `detect()` は `if not grid.frozen or len(pts) == 0: return []` → 地図作成中は検出 0。
  `confirm()` は **2 周期連続で同じ場所に出たもの**だけを残す（`if not prev: return []`）。
  したがって **検出から確定までは +1 周期**、
  **物理的に見え始めてから確定までは最大 2 スキャン周期 = 0.2 s**。
  > **注（第2ラウンド再検証で訂正）**: 原文の「『初めて見えてから確定』まで最大 2 周期」は
  > 検出と確定を混同していた。**検出 → 確定は 1 周期**、
  > 見え始め → 確定が最大 2 周期。結論（`react` が 1 周期ぶん不足）は変わらない。
  `SCAN_PERIOD_S = 0.1` / `react = SCAN_PERIOD_S + cmd_latency + 0.5τ` = 実測 **0.14646 s**
  （0.1 周期分しか見ていない）。docstring `:273-274` は「点群の周期（最悪そのぶん遅れて見える）」
  と書いている。
- **影響**: **停止余裕が 1 スキャン周期ぶん（0.1 s）足りない**。
  ただし既定（`obstacle_stop=0`）では NAV-04 により本経路自体が有効化されない
  （深刻度 High → **Medium**）。
  1.5 m/s で 15 cm、3.0 m/s で 30 cm。影響先は `:1850`（近接障害物）・`:1973-1975`（停止距離）・`:2163`（見通しの良い区間の速度上限）。
  近接高速パス（`:1848`）も**同じフラグで gate** され既定では無効（NAV-04 と重なる）。
- **修正案**: `react = 2*SCAN_PERIOD_S + cmd_latency + 0.5τ`、
  あるいは `near` の式に `+SCAN_PERIOD_S` を明示的に含める。
  「点群周期」と「確定に必要な周期」を別名にする。

#### NAV-06 [M] MPC の `now` 時刻基準が崩れると `d[0]` の舵レート制約が 5 倍緩む
- **判定**: `CONFIRMED`
- **場所**: `raspi/auto/slam2d_raceline.py:1826` / `raspi/nav/mpc.py:147-149,225-227`
- **内容**: `now=vs.t_capture / 1e9 if vs.t_capture else None`。
  `vs.t_capture == 0` は「センサの時刻が無い」状態。
  `mpc.py:147-149` `if now is None: self._clock += cfg.period; now = self._clock`（0.0 起步）。
  `:225-227`:
  ```python
  prev_t = self._sent[0][0] if self._sent else None
  dt_call = cfg.dt if prev_t is None or now - prev_t < 1e-4 else min(cfg.dt, now - prev_t)
  reach = cfg.steer_rate * (dt_call + cfg.dt * np.arange(N))
  ```
  **実測**（直線経路・横偏差 45 cm・`steer_rate=7.0`・`dt=0.1`・`delay_s=0.05`・50 Hz）:
  ```
  epoch 1..7 : steer=-8.128°（収束）  dt_call=0.0200 → reach[0]=0.140 rad
  epoch 0   : steer=-27.025°        dt_call=0.1000 → reach[0]=0.700 rad（_sent が空）
  None       : steer=-27.025°          dt_call=0.1000 → reach[0]=0.700 rad（5倍）
  ```
  内部 `now=0.1` に対し `prev_t=1.7e9` → `now - prev_t = -1.7e9 < 1e-4` → `dt_call = cfg.dt = 0.1`。
  出力は **−27.025°（max_steer 全舵）へ 18.9° 跳躍**。
  `self._clock` は epoch に戻らないため、以降の epoch 呼び出しでも `_sent[0]` が epoch _scale の
  まま残り `dt_call=0.1` が継続（1 周期後に回復）＝ **恒久的 breakage ではないが再発しうる**。
- **修正案**: `now=None` を許さない（呼び出し側で必ず `t_capture` を与え、
  無ければ `self._sent[0][0]` 基準に合わせる）。または `mpc.py` 側で `now` を
  `_sent` の最終時刻に相対化する。加えて `dt_call` を「実呼び出し間隔の計測値」ではなく
  「前回出力からの実経過時間」で持つ。

#### NAV-07 [L] `cmd_steer_rate_limit_rad_s` が `nav/` から読まれていない
- **判定**: `CONFIRMED`（構造的事実。深刻度 Medium → **Low**。値は偶然一致）
- **場所**: `raspi/nav/mpc.py:54` / `raspi/auto/slam2d_raceline.py:381-386` /
  `raspi/core/vehicle.py:206,410` / `config/vehicle.toml:83`
- **内容**: `grep -rn "cmd_steer_rate_limit" raspi/ sim/ tools/` の結果は
  `vehicle.py:206,410`（定義と読み込み）、`sim/vehicle.py:188,271`、`sim/bench.py:674,679` のみ。
  **`raspi/nav/` からは 0 件**。`mpc.py:54` は `steer_rate: float = 7.0` のハードコードで、
  `mpc_config()` は設定しない → 既定 7.0 が使われる（`vehicle.toml:83` も 7.0 で**偶然一致**）。
- **影響**: 実害はゼロだが、**正（`vehicle.toml`）が nav へ届いていない**。
  firmware 側の舵レート制限を toml で変えても nav は追随しない。
- **修正案**: `mpc_config()` に `steer_rate = vehicle.cmd_steer_rate_limit_rad_s or MPCConfig.steer_rate` を渡す。

#### NAV-08 [L] `Vehicle.kappa_max` のオフセット符号が片側だけ保守的
- **判定**: `CONFIRMED`（Medium → **Low**。安全側）
- **場所**: `raspi/core/vehicle.py:252-262` / `raspi/nav/purepursuit.py:77-93`
- **内容**: `delta = gain*d + cubic*d³ - abs(self.steer_offset_rad)` で**左右とも不利な側**を取る。
  `SteerMap.effective()/command()` の逆算（`t = delta - offset; d = t/gain; cmd = d/servo_gain`）は正しい。
  **実測**（`cmd = ±0.47167`、servo_gain 0.98689、offset −0.008817、L=0.23）:
  ```
  左(+): actual=+0.465486 effective=+0.456669  κ=+2.136119  (1/κ=0.46813 m)
  右(−): actual=−0.465486 effective=−0.474303  κ=−2.232137  (1/|κ|=0.44800 m)
  kappa_max = 2.136119  ← 左の値
  ```
  差は右基準 **4.30%**（kappa_max 基準 4.49%）。
- **影響**: **害はない**（安全側）。`kappa_limit()` は `kappa_max × 0.9` なので
  右コーナーだけ 4.3% 広く曲がれる余地を捨てている。**同じ `SteerMap` を使いながら片側だけ違う**不整合。
- **修正案**: `kappa_max` を `SteerMap.command(±max_steer)` → `effective()` の**最小絶対値**で求める
  （左右非対称は仕様として `kappa_max_l` / `kappa_max_r` に分離）。

#### NAV-09 [L] `max_steer` の既定値が `vehicle.toml` と陳腐化
- **判定**: `CONFIRMED`（Medium → **Low**）
- **場所**: `raspi/nav/purepursuit.py:98-100` / `raspi/nav/mpc.py:53` / `raspi/core/vehicle.py:118-121`
- **内容**: `purepursuit.py:100` `max_steer = 0.46175` / `mpc.py:53` 同じ /
  `vehicle.py:121` `max_steer = 0.524` / `vehicle.toml:25` `0.47167`（= 27.02°）。
  docstring は `vehicle.py:118-120`「現在 **0.4636** = 26.6°」、
  `purepursuit.py:98-100`「既定は `vehicle.toml` の現在値（…約 26.5°）」。
  → **4 箇所すべて不整合**。実呼び出しは `slam2d_raceline.py:364,382` で
  `vehicle.max_steer` を渡すため**不活性**。
- **修正案**: 既定を `None` にして「呼び出し側が必ず `Vehicle.max_steer` を渡す」を型で強制し、
  docstring の数値を 0.47167（27.0°）に更新する。→ CFG-03 と共通。

#### NAV-10 [M] 減速側の指令ランプ（3.0）が物理的な制動能力（2.29〜2.54）を超える
- **判定**: `CONFIRMED`
- **場所**: `config/vehicle.toml:78,128` / `raspi/core/vehicle.py:78-87,280-284` / `raspi/auto/slam2d_raceline.py:234,282`
- **内容**: `cmd_accel_limit_m_s2 = 3.0`（加減速共通のランプ上限）、
  `a_brake_m_s2 = 2.5413` → `Vehicle.a_decel_max` = `min(2.5413, 3.0)` = 2.5413。
  planner の想定は `FIXED_BRAKE_USE = 0.9` → `a_brake = 0.9 × 2.5413` = **2.2872 m/s²**。
  差 **0.713 m/s²**。目標速度のランプが 3.0 で降りるのに車は 2.29 でしか減速できない。
- **影響**: 差 0.71 m/s² が 1 秒ごとに速度誤差として蓄積し、**PI に「減速しすぎた」と逆方向の
  指令を出す**。停止点に近づくほど偏差が拡大し、制御が連鎖的に振れる可能性がある。
- **修正案**: NAV-03 と同一。ランプ上限を `min(cmd_accel_limit_m_s2, Vehicle.a_decel_max)` にする
  （数値の出所を `vehicle.toml` 1 箇所に寄せる）。

#### NAV-11 [M] `nearest_index` に全体検索のフォールバックが無い
- **判定**: `CONFIRMED`
- **場所**: `raspi/nav/purepursuit.py:186-195` / `raspi/auto/slam2d_raceline.py:116,1262` /
  `raspi/nav/route_switch.py:87` / `raspi/auto/slam2d_route.py:891,1048`
- **内容**:
  ```python
  if hint >= 0 and window > 0:
      idx = hint + np.arange(-window, window + 1)
      ...
      return int(idx[np.argmin(...)])       # ← 窓の縁に落ちてもフォールバックしない
  return int(np.argmin((xy[:,0]-x)**2 + (xy[:,1]-y)**2))
  ```
  **再現**（半径 5 m の円、点間隔 0.1 m、真の最近傍 index 19）:
  ```
  hint=0, window=8 → 8      （真値 19 との差 11 索引 = 1.1 m の弧）
  hint=-1, window=0 → 19
  ```
  `slam2d_raceline.py:1262` は `window=8`（`PATH_STEP = 0.10`）→ ±0.8 m の弧。
  得た `j` は `speedctl.arc_index()` に渡され**速度プロファイルを読む位置**を決める。
  `route_switch.py:87` は hint なし（全体検索）で安全、`slam2d_route.py` は `window=n//4` で数値上安全。
  **`slam2d_raceline` だけが例外**。
- **影響**: 曲がり角の速度を直線区間で使う（または逆）。
  **車両がラインから 0.8 m 以上離れた状態（回避 manoeuvre・局所化ジャンプ・
  大きなクロストラック）のあいだ続く**。外見上は「正常的だが数値だけ別のコーナーから取られた」コマンドになる。
  > **注（第2ラウンド再検証）**: 深刻度 High → **Medium**。
  > 影響は「速度プロファイルを読む添字がずれる」ことに限られ、制御自体の破綻ではない。
- **修正案**: 窓内最小が**窓の縁**に落ちた場合は全体検索にフォールバックする
  （または `cross_track` で妥当性を検証し大きければ全体再検索）。
  この 1 箇所の修正で `slam2d_raceline` の速度プロファイル読みが改善する。

#### NAV-12 [H] Hybrid A* の margin 緩和が**経路全体**に効く
- **判定**: `CONFIRMED`（本体）／`park_to_point` の `thr` 議論は**方向が逆**
- **場所**: `raspi/nav/hybrid_astar.py:334-338,369,381,387,394,460,500` /
  `raspi/auto/park_to_point.py:613-622` / `raspi/nav/hybrid_astar.py:158-162,425`
- **内容**:
  ```python
  goal_clear = float(lmap.body_clearance(*goal))
  if goal_clear <= -cfg.goal_slack: return HybridResult(...)
  margin = min(cfg.margin, max(-cfg.goal_slack, goal_clear - 0.005))
  ```
  この `margin` は探索全体で単一の硬い拘束（`:460` `if w < margin: continue`、
  `:500` `if lmap.path_clearance(poses) >= margin`）。
  **実測**（実 `LocalMap` 2 cm・1 m 通路・`cfg.margin=0.05`・`goal_slack=0.03`）:
  ```
  goal_y=0.89 → 目標の余裕 +12.37 mm → 使われた margin = 7.37 mm
  goal_y=0.80 → 目標の余裕 +92.37 mm → margin = 50.00 mm（正常）
  ```
  comments の趣旨（「駐車は意図的に壁へ詰める」）は正しいが、**実装は目標の余裕を全経路に適用**している。
  `park_to_point.py:613-622` の `thr = min(0.0, self._plan_margin)` は
  `self._plan_margin`（= 緩和後の 0.0074）を使い `thr = 0.0`（＝0 mm 必要）となる。
  → **原指摘の「計画より厳しい」は逆**（0 mm 要求は 7.4 mm 要求より**緩い**）だが、
  「閾値が一致していない」こと自体は正しい
  （`hybrid_astar.py:158-162` の docstring「呼び出し側の安全判定は**この値と同じ閾値を使わなければならない**」に反する）。
- **影響**: 目標が 12 mm 余っているだけで、離れた位置から进入する経路全体の壁クリアランスが
  5 cm → 0.74 cm になる。最後の数 cm だけでなく**进入のほぼ全体が 0.74 cm の壁際を走る**。
- **修正案**: `margin` は `cfg.margin` のまま維持し、`goal_slack` は
  **終端ノードの受容判定の緩さ**としてのみ使う（「最終ノードはこの余裕を許す」）。
  `park_to_point` の `thr` は `self._plan_margin` を使い、緩和値が負のときだけ `min(0, m)` にする。

#### NAV-13 [M] 探索時間予算が `auto/cmd` の陳腐化閾値を超える
- **判定**: `PARTIAL`（既定値と実測は CONFIRMED。**本番経路では既にクランプ済み**。High → Medium）
- **場所**: `raspi/nav/hybrid_astar.py:120-125` / `config/vehicle.toml:63` /
  `raspi/auto/park_to_point.py:260-266,419` / `raspi/auto/slam2d_raceline.py:2335`
- **内容**: `time_budget_s: float = 0.25` vs `auto_cmd_stale_ms = 200`。
  **実測**（`raspi/tests/test_hybrid_astar.py` のシーン）:
  ```
  deadend(margin=0.20) 250 ms 予算 → 実測 250.3 ms（予算到達・解なし）
  deadend(margin=0.20) 150 ms 予算 → 実測 150.4 ms
  room    250 ms → 4.6 ms / 150 ms → 0.8 ms
  garage  250 ms → 48 ms（解けた）/ 150 ms → 36 ms（解けた）
  > **注（第2ラウンド再検証で訂正）**: 予算切れになるのは **DEADEND シーン + `margin=0.20`**
  > （`test_time_budget_is_respected` のシーン）であって **garage ではない**。
  > garage は旋回半径 0.30〜0.80 の全域で 16〜69 ms で必ず解ける。
  ```
  呼び出し側は **150 ms にクランプ**されている
  （`park_to_point.py:260-266` の ParamSpec max=150 → `:419`、
  `slam2d_raceline.py:2335` `min(detour_budget_ms, 150.0)`）。
  → **既定 0.25 は本番で使われない**。「予算は stale 閾値を超える」は
  既定値の記述としては正しいが、実運用では 150 ms クランプ下（152 ms < 200 ms）。
- **修正案**: `HybridConfig.time_budget_s` の既定を 0.15 にして
  呼び出し側のクランプと一致させる（現状は既定 0.25 / 実呼び出し 0.15 の二枚舌）。
  `assert time_budget_s < Vehicle.auto_cmd_stale_ms / 1000` を入れるのは良いが、
  本命は「1 回の計画全体（線引き・道グラフ生成含む）が 200 ms 未満」の監視（GEN-05）。

#### NAV-14 [H] `StandstillSteer`（MD 過熱対策）が planner 中 1 つだけ
- **判定**: `CONFIRMED`（使用箇所が 1 つだけという本体。**planner 数の「13」は誤り**）
- **場所**: `raspi/auto/base.py:34,228-263` / `raspi/auto/park_to_point.py:145,314,687` /
  `raspi/nodes/planning_node.py:481-494` / `docs/system_overview.md:90` / `raspi/auto/registry.py`
- **内容**: `grep -rn "StandstillSteer" raspi/` のヒットは
  `base.py:34`（`__all__`）・`base.py:228`（定義）・`park_to_point.py:145,314,687`（使用）・
  `raspi/tests/test_park_to_point.py:199` のみ。
  実 planner 総数: `raspi/auto/registry.py` の `PLANNERS` は **22 件**
  （`category=="drive"` 12 件、`"sysid"` 10 件）。`raspi/auto/*.py` 内の `Planner` 直下サブクラスを
  数えると 23 種。**「13」はどの数え方でも一致しない**（件数は要修正）。
  `planning_node.py:481-494` は `ready=False` / `brake` のとき `self._last_steer` を保持。
  `docs/system_overview.md:90` は「Pi の planner は**停車 2 秒で舵を中立へ戻す**」と
  全 planner がそうと書いている。
- **影響**: `ftg` / `dp` / `de` / `line_trace` / `follow_object` / `cam_e2e` / `e2e_lidar` /
  `slam2d_raceline` / `slam2d_route` は**障害物に当て停止した状態・駐車待ち・局所化喪失中で
  Full ロック indefinitely を保持**する。ステア MD は据え置きで過熱する
  （`docs/system_overview.md` §2 が明記している危険）。
- **修正案**: `StandstillSteer` を**共有基底クラス**へ移し全 planner から呼ぶ。
  最低限 `planning_node` 側に「`vs.stopped` が N 秒継続したら舵を中立へ戻す」集約処理を入れる。
  文書と実装のどちらを直すかを決める。

#### NAV-15 [L] MPC が `nearest_index` を 3 回計算
- **判定**: `CONFIRMED`（**実際は 3 回**。原指摘の「2 回」は表記が軽い）
- **場所**: `raspi/nav/mpc.py:155,160` / `raspi/nav/purepursuit.py:209,212`
- **内容**: `mpc.py:155` の `follow()` 内で `nearest_index` が **2 回**
  （`purepursuit.py:209` と `:212`）、加えて `mpc.py:160` で自身のもう 1 回 → **3 回**。
  窓サイズは `follow` 内は `window=len(path)//4`、`mpc.py:160` は `window=n//4` で同条件。
- **影響**: 「一致する前提」で 3 回計算しているコスト。
- **修正案**: `nearest_index` の結果を `Pursuit` に持ち回して 1 回に集約する。

#### NAV-16 [L] `park_to_point._hold` が `ready=True` で「安全」の意味と矛盾する
- **判定**: `CONFIRMED`
- **場所**: `raspi/auto/park_to_point.py:684-704` / `raspi/nodes/planning_node.py:481-494`
- **内容**: `_hold()` は `st.ready = True; st.brake = True; st.target_speed = ...` を返す。
  `planning_node.py:481-494` の brake 分岐は
  `if st.ready: self._last_steer = st.target_steer` とし、
  `DriveCmd(mode=2, brake=True, target_speed=0.0, ..., brake_torque=st.brake_torque if st.ready else 0.0)` を出す
  → **指令は生きたまま制動**（＝`ready` は「安全」を意味しない）。
- **修正案**: 表示側で `ready` と `brake` を区別して見せる（「待機（制動中）」等）。
  コード側では `AutoState` に「待機」等の別フラグを追加して `ready` の意味を 1 つに保つ。

#### NAV-17 [L] `speedctl.REF_CATCH_S` の「τ=0 のときの値」という位置づけ
- **判定**: `SEVERITY-OFF`（**挙動はコメントどおり**。「二重表現」という指摘のみが実体）
- **場所**: `raspi/nav/speedctl.py:91-94,122` / `raspi/core/vehicle.py:295-302`
- **内容**: `REF_CATCH_S = 0.15` の説明は「これは未同定（τ=0）のときの値」。
  `Vehicle.speed_loop_tau_s` は未同定なら **0.0** を返す → `catch_s=0` が渡り
  `speedctl.py:122` の `catch = catch_s if catch_s > 1e-6 else REF_CATCH_S` で 0.15 に落ちる。
  **コードはコメントの記述どおり**で「位置づけの食い違い」は無い。
- **修正案**: コメントを「`Vehicle.speed_loop_tau_s` が 0（未同定）のときの既定値」に 1 回整えるだけ。

#### NAV-18 [M] `sim/` と `raspi/` が `vehicle.toml` を別々のパーサで読む / `axle_model` はデッドコード
- **判定**: `CONFIRMED`（構造・死コード）／数値表は `PARTIAL`（rear 0.44 → 実測 **0.4637**）
- **場所**: `raspi/core/vehicle.py:346-417,399-400` / `sim/vehicle.py:193,203-273,582,806,856` /
  `raspi/auto/slam2d_raceline.py:2479-2484`
- **内容**: `raspi/core/vehicle.py` は `[control.plant]` を読むが、
  `[control.plant]` に `rear_lateral_share` は**存在しない**（実測キー集合で確認）→ 実値は既定 0.59。
  `sim/vehicle.py:203-273` の `VehicleSpec.load()` は `[control.plant]` を**一切読まない**
  （`rear_load_frac` / `load_transfer_hl` は dataclass 既定 0.59 / 0.133 のまま）。
  `axle_model` は `sim/vehicle.py:193` の定義のみ。`VehicleSpec.load()` に設定行が無く
  `grep` でも `sim/vehicle.py:193/582/806/856` のみ → **`:806`・`:856` の車軸ごとの摩擦分岐は恒久 off**。
  数値（μ·g=4.0912, r=0.59, hl=0.133 で評価）:
  ```
  a_long=0.00 : 単一摩擦円 4.0912 | front 4.0912 rear 4.0912 → min 4.0912
  a_long=2.54 : 単一摩擦円 3.2072 | front 3.7476 rear 0.4637 → min 0.4637
  ```
  → 定性的帰結（後輪側 grip が事実上潰れ min が 3.21 から 0.46 へ）は再現。
  `[sim]` の扱い: `sim/vehicle.py:211` `sim_only = {**dyn, **d.get("sim", {})}` + `[limits]` からの上書き。
  `raspi/` は `[sim]` を読まない。
- **影響**: **どちらのモードも実測の g-g に合わない**。
  `raspi/nav/grip.py` の学習器はこのモデルを使うので、**制動中のグリップをシムで調整してはいけない**。
- **修正案**: 共有ローダ（`raspi/core/vehicle.py`）に一本化し、`[sim]` は
  **明示的な名前空間 + キー許可リスト**にする。`VehicleSpec.load()` に `axle_model` を
  設定する行を入れてデッドコードを解消する。→ CFG-02 / CFG-12 と共通。

#### NAV-19 [L] `deskew` の二重化と `t_ref` フォールバック差（→ SLM-08 へ統合）
詳述 **SLM-08**。検証補足: 「SLAM 側が 100 ms 古い pose」は `t_point_ns` が全て 0 のときだけ成立
（通常の LD06 は `raspi/nav/deskew.point_times_ns` が非 0）。
重複表の 6 行はすべて実在。`UnknownWallGrid` が本番で無効であることも確認
（`raspi/nav/grid.py:344-350` docstring「**今は本番の経路生成では使っていない**」、
参照は `raspi/tests/test_route.py:1037,1054` のみ）。

#### NAV-20 [M] `raspi/nav/grip.py` の `LaunchFit` が「TC active」の意味を誤用
- **判定**: `CONFIRMED`（**第2ラウンドで firmware 突合が成功**）
- **場所**: `raspi/nav/grip.py:497-548`（`LaunchFit`・`:524-527` `feed()`）/
  `sim/stm32.py:622-624` / `sim/vehicle.py:556-557,559-574,647`
- **内容**: `grip.py:524-527`:
  ```python
  def feed(self, t, v, tc_active: bool, steer: float):
      on = tc_active and abs(steer) < self.STEER_MAX and v > 0.05
  ```
  docstring（`:498-506`）も「発進で TC が介入している間の加速度」を推定する意図。
  シム側の「TC active」: `sim/stm32.py:622-624` は
  `self.vehicle.accel_capped` を見て `FLG_TC_ACTIVE` を立てる。
  `sim/vehicle.py:647` `accel_capped = driving and abs(a_req) > a_lim` で、
  `:556-557` の docstring は「実機ならここで TC が介入する」まで。
  → **シムの「TC active」=「加速度がグリップ上限で頭打ち」**であり、
  実機の意味（後輪が実際に滑っている／スリップ率 PI が効いている）とは**別の量**。
  関連: `sim/vehicle.py:559-574` の `_update_launch()` は**TC 有効を要求しない**
  （`cmd.launch` のみ）。firmware が TC を要求するなら、
  **`tc_enable = 0` のとき実機は発進せず(sim は全開で発進する)**差が出る。
- **firmware 突合（第2ラウンドで完了）**:
  `src/control/drive.c:385` `if (obj->launch_spent || !obj->tc_enabled) return false;` を確認し、
  **実機は `tc_enable = 0` なら発進しない**。シムの `_update_launch` は TC を要求しないため
  **`tc_enable = 0` のとき「実機は発進せず、sim は全開で発進する」**差が実際に生じる。
- **修正案**: シムの `FLG_TC_ACTIVE` の意味を firmware に合わせて定義し直す
  （「加速度頭打ち」は別フラグへ）。firmware ツリーを突合し、
  `launch_spent` / `tc_enabled` 条件をシム側にも実装する。

#### NAV-21 [M] `sim/stm32.py` に TC / TV / 実 ABS が無い
- **判定**: `CONFIRMED`（構造）／**ABS フラグの数値主張「0.0498 N·m 以上で常に」は `FALSE`**
- **場所**: `sim/stm32.py:10,262-269,274-287,518-524,726-732` / `sim/vehicle.py:434-457` /
  `raspi/proto/generated/packets.py` の `FLG_TC_ACTIVE`
- **内容**: `sim/stm32.py:10`「**TC/TV は実装していない。** タイヤモデルが無いので介入する材料が無い」。
  テレメトリは `slip=[0,0]`、`tc_limit_nm` / `abs_limit_nm` は常に `DRIVE_MAX_TORQUE_NM`、`tv_ratio=0`。
  `CONFIG_ACK`（`:262-269`）: `ConfigSet` は既知 param_id なら `OK`、未知なら `UNKNOWN_ID`
  （**常に OK ではない**＝原指摘のこの部分は FALSE）。`ConfigGet` は常に `OK`。
  範囲外クランプも `OUT_OF_RANGE` も**出さない** → Pi 側の
  `raspi/core/control_params.py:305` の `OUT_OF_RANGE` 経路は**到達不能**。
  ABS フラグ（`:518-524`）は `brake_decel(...)[1]`（=減速度が `brake_decel_m_s2` で頭打ち）だけで立てる。
  `sim/vehicle.py:434-441` の docstring 自身が
  「**このモデルは後輪の回転を持たないので、ABS を切った実機は後輪がロックして減速度・横グリップとも
  落ちるが、このモデルは表せない**」と明記。
  **実測**（`brake_decel_per_nm=37.5532`, `brake_decel_m_s2=2.165`,
  `rolling_resistance=0.2925`, `brake_boundary_speed_m_s=0.6`）:
  ```
  v=0.3 : nm=0.0498→decel 1.1567 abs=False ; nm=0.150→2.1650 abs=True
  v=1.0 : nm=0.0498→2.0338 abs=False ; nm=0.0536→2.1650 abs=True
  v=2.0 : nm=0.0498→2.1579 abs=False ; nm=0.0500→2.1650 abs=True
  v=3.0 : nm=0.0498→2.1625 abs=False ; nm=0.0500→2.1650 abs=True
  ```
  → 「0.0498 N·m 以上で常に ABS フラグが立つ」は **FALSE**
  （0.0498 は `v→∞` 極限の `(2.165−0.2925)/37.5532 = 0.04986` で、
  実速度では境界層の `tanh(v/0.6)` が効いて閾値が上がる）。
  またブレーキトルク上限は 0.15（`raspi/msgs/convert.py:69`）で **0.155 ではない**。
- **影響**: **sim 上の「ABS が効いた/効かない」の議論は実機について何も保証しない。**
  firmware の `ApplyAbs()` は**スリップ率**（`[control] abs_slip_target = 0.1216`）で判断する。
- **修正案**: シムの plant にスリップモデルを入れ `tc_limit_nm` / `abs_limit_nm` を駆動する。
  `CONFIG_SET` に firmware と同じ範囲表を持たせ `OUT_OF_RANGE` を返す。
  現実装のままなら **「シムでは TC/ABS が検証できない」**と明記する。

#### NAV-22 [M] `sim/vehicle.py` に MD のトルク遅延が無い
- **判定**: `CONFIRMED`
- **場所**: `sim/vehicle.py:637-654,556-557` / `config/vehicle.toml:274-275` /
  `raspi/core/control_params.py:77-78` / `sim/stm32.py:577` / `raspi/tests/test_control_params.py:123`
- **内容**: `SpeedController.step()` は `t_req` から `a_req = g*t_req` → `a = clip(a_req, ±a_lim)` →
  `v_new = v + a*dt` と**即座に**加速度へ反映（トルク段の一次遅れなし）。
  実機には `md_delay_s = 0.007` + `md_tau_s = 0.009`（計 **16 ms**）があり、
  `speed_kp` / `speed_ki` はその遅れ込みで同定されている（`tools/calib/speed.py`）。
  `raspi/core/control_params.py:77-78` の `DERIVED["md_torque_lag_s"]` は STM32 へ送っている。
  `sim/stm32.py:577` は `AUTO_STOP_SENSOR_DELAY_S + MD_TORQUE_LAG_S` を
  **自動停止の距離計算にだけ**使っており、車両モデル自体は MD トルクの遅延を模擬しない。
- **影響**: **シムの速度ループは約 16 ms 硬い**。
  `speedctl` のフィードフォワードのリードと `Vehicle.speed_loop_tau_s` は**シムでは正しく調整できない**。
  さらに `sim/tests/test_speed_ff.py` の 2 件が master で赤（VER-03）——
  **シム⇄実機の一致を保つ唯一の網が破られている**。
- **修正案**: シムの plant に 1 次遅れ（`md_delay_s` + `md_tau_s`）を実装する（`[control.plant]` を読む）。
  2 件のテスト期待値を再ベースラインする。

#### NAV-23 [M] `sim/bench.py` の `CMD_HZ` が実機と違う / bench は実時間 1:1
- **判定**: `CONFIRMED`
- **場所**: `sim/bench.py:48-49,86,601-620,687` / `raspi/auto/base.py:40` /
  `raspi/nodes/planning_node.py:61,84` / `raspi/nodes/telemetry_node.py:152,274`
- **内容**: `sim/bench.py:86` `CMD_HZ = 100` / `raspi/auto/base.py:40` `CMD_HZ = 50`。
  `sim/bench.py:601-620` の `run()` は `t0 = time.monotonic()`、
  `while time.monotonic() - t0 < duration_s`、lambda の更新は `self.link.poll()` で実時刻から生成
  → **ホスト wall-clock に 1:1**（`bench.py:48-49` も「壁時計で回る…決定論的でない」と明記）。
  なお bench 内でも追従 tick は `:687` `next_tick = now + NS // 50`（50 Hz）で、
  **指令だけ 100 Hz** という混在がある。
- **影響**: FF やランプの比較、`cmd_stale`（`auto_cmd_stale_ms = 200`）の模擬で
  **実機と異なる挙動**になる。パラメータ掃引・確率的評価・1 日 1000 ケースが**原理的に不可能**。
- **修正案**: `sim/bench.py` の `CMD_HZ` を `raspi/auto/base.py` から import して単一化する（50 Hz）。
  批量評価が必要なら bench を「sim 時間を wall-clock から切り離す」モード
  （固定ステップ・複数ケース並列）へ拡張する。

#### NAV-24 [M] `raspi/nav/raceline.py` と `sim/raceline.py` の `min_curvature_alpha` の重複
- **判定**: `CONFIRMED`
- **場所**: `raspi/nav/raceline.py:119-` / `sim/raceline.py:126-129`
- **内容**: `raspi/nav/raceline.py::min_curvature_alpha` は開/閉両対応、`weights`（点ごとの曲率重み）、
  `warm`（前回解の warm start）あり、密行列で `d2.T@d2` を計算。
  `sim/raceline.py::_min_curvature_alpha` は**閉曲線専用**（`_cyclic_d2`）、`weights` なし、
  `warm` なし、疎行列。`sim/raceline.py:129` が「`raspi/nav/raceline.py` の同名関数の移植」と明記。
- **影響**: 両者は機能・重み・warm start・等式拘束の扱いが異なり、
  **片方だけ直すと挙動が変わる**。`sim` 側は ML 報酬にも使わる。
- **修正案**: `raspi/nav/raceline.py::min_curvature_alpha` に一本化し、
  sim 側はそれを呼ぶ薄いラッパにする（`weights` / `warm` / 開経路対応も共有する）。

#### NAV-25 / NAV-26 — 統合済み
- NAV-25（`ml_lidar/env.py` の自動停止定数） → **ML-09** に統合。
- NAV-26（`plan_budget_ms` が `auto_cmd_stale_ms` に接近） → **GEN-02** に統合。
  検証補足: 「恒久 stale」は**立証されていない**。 Hybrid A* 予算は 150 ms にクランプされ、
  フォールバックの実測コストは 8 ms 程度 → 合計 160 ms 程度 < 200 ms。
  候補数の実測は 1 半径あたり最大 42、全半径合計最大 **112**、
  `park_to_point` フォールバック実測（6 m 部屋）で **108 候補・8.3 ms**
  （原指摘の「144」は概算としては妥当だが正確ではない）。

#### モード間の暗黙の前提の比較表（§4.4 の根拠）

各行は実コードで裏付けた（行番号付き）。

| planner | 最高速度 既定 | 最低速度 既定 | 正面余裕の意味 | 舵の生成 | 「不明」の表現 | 曲率制限 |
|---|---|---|---|---|---|---|
| `follow_the_gap` | 0.40 `:124` | `min_speed` 0.12 `:127` | **速度に使う** `:190,246-247` | Pure Pursuit `:234` +1次遅れ | ready=False `:162,166,219` | ○ `a_lat_max` `:252-254` |
| `disparity_extender` | 0.60 `:139` | 0.12 `:143` | 診断のみ `:204,255` | **比例** `:230` | ready=False `:181,185,219` | ○ `:250-252` |
| `gap_pursuit` | 0.50 `:136` | 0.12 `:139` | 診断のみ `:205,261` | Pure Pursuit `:233` | ready=False `:184,188,218` | ○ `:254-257` |
| `cam_centerline` | 0.30 `:80` | 0.10 `:83` | **速度に使う** `:123,148` | Pure Pursuit `:137` | ready=False `:107,117` | ○ `:151-153` |
| `cam_e2e` | 0.30 `:60` | 0.10 `:64` | 概念なし | **回帰** `:114-117` | ready=False `:94,103,109` | ○ `:130-132` |
| `line_trace` | 0.30 `:62` | 0.10 `:65` | 概念なし | Pure Pursuit `:111` | ready=False `:89,100` | **×**（`turn_slow` の比例減速のみ `:122`） |
| `follow_object` | **1.00** `:81` | **パラメータ無し**（下限 0 `:219-220`） | 概念なし | Pure Pursuit `:212` | **両義**（ready=False `:131,186` / ready=True+brake=True `:155-156`） | ○ `:225-227` |
| `e2e_lidar` | **1.00** `:138`（max 3.0 `:137`） | **パラメータ無し**（下限 0 `:333`） | 診断のみ `:261` | **舵角速度の積分** `:315-316` | ready=False `:246,252,257,296,300` | **×** |
| `park_to_point` | 0.30 だが**キー名は `cruise_speed`** `:185-186`（上限 1.0） | **`v_min_move` 0.08** `:189-190` | 概念なし | **曲率フィードバック** `:891-894` | **両義**（ready=False `:752,754` / `_hold`= ready=True+brake=True `:691-692`・`:777`） | ○ `a_lat_max` `:906-911` |

> **注意**: `park_to_point` は**唯一 `max_speed` キーが無い** planner。
> `e2e_lidar` の `:132-133` のコメント「`free_ahead` は安全側の停止判定に使う」は
> **現状のコードと矛盾**（モジュール docstring `:32` の「GUI の診断表示用に計算する」が正しい）。
> これは **ML-08** として扱った。

GUI の `max_speed` スライダを同じ位置に置きながらモードを切り替えると、
**同じスライダ位置で別の速度・別の安全余裕になる**。これは設計思想と矛盾する。

---

### 3.5 学習・推論（ML）

#### ML-01 [H] E2E LiDAR モデルの `dt` 契約が空で、学習周期と実機周期が 2 倍違う
- **判定**: `CONFIRMED`（契約は 26 個で v24 は未エクスポート）
- **場所**: `raspi/auto/e2e_lidar.py:59,104-108,313-316` / `ml_lidar/export_onnx_rl.py:175-176` /
  `ml_lidar/train_rl.py:678-684` / `ml_lidar/env.py:74-88` / `models/e2e_lidar/*.json`（26 個）
- **内容**: `e2e_lidar.py:104-108` に
  `dt = float(cfg["dt"]) if "dt" in cfg else None` / `if dt is not None and abs(dt - PLAN_PERIOD_S) > _DT_TOLERANCE_S: raise`
  というガードがあるが、**契約 26 個すべてに `dt` が無い** → **ガードは永久に発動しない**。
  学習時の `dt`（`ml_lidar/runs/*/env_config.json` の実測）:
  | 学習 run | 学習時の `dt` | 契約の `dt` | 実機での実効 |
  |---|---|---|---|
  | v1〜v12 | 0.05 | 無し | `steer_rate_max_rad_s` 欠落でロード拒否 |
  | **v13〜v20** | **0.05（20 Hz）** | **無し** | **0.10（10 Hz）で実行** |
  | v21〜v27 | 0.10 | 無し | 0.10（偶然一致） |
  実測で **v13 / v20 / v21 / v27 はロード成功**し、
  v13〜v20 は **GUI から選択可能**（`telemetry_node.py:2182-2192` が列挙）。
  舵積分は `dt = min(dt, _DT_MAX_S=0.15) → 0.10` なので
  **1 判断あたりの舵の移動量が学習時のちょうど 2 倍**になる。
- **影響**: `ml_lidar/env.py:74-88` が「実車は学習時の 2 倍の `dt` で実行していた。
  `steer_tau` の 1 次遅れ係数も 0.393 対 0.632 でずれていた。
  **実車で「舵が発散し壁に衝突」**が出ていた理由の有力な説明。
- **修正案**: v13〜v20 の契約 JSON に学習時 `dt=0.05`（と `steer_tau`）を記入して
  **ロード拒否**させるか、v21 以降と同じ dt で再学習・再エクスポートする。
  現行 `export_onnx_rl.py:175-176` は既に `dt` を書くので**再エクスポートすれば正しく弾かれる**。
  「`dt` が無い契約」をエラーにする（既定値続行ではなく）。

#### ML-02 [M] `ml_lidar` の ONNX 契約 JSON に「現在」の `vehicle.toml` 値が入る
- **判定**: `PARTIAL`（**既存 26 個は正しい**。危険なのは今後の再エクスポート時のみ。High → **Medium**）
- **場所**: `ml_lidar/export_onnx_rl.py:145,147,149,168-177` / `ml_lidar/eval_stats.py:89-96,100-134` /
  `ml_lidar/train_rl.py:678-684` / `ml_cam_e2e/export_onnx.py:68-86`
- **内容**: `export_onnx_rl.py:145,147` は `--max-steer` / `--steer-link` が無ければ
  **現在の** `vehicle.toml` から取る。`steer_contract_warnings()` は警告するだけ。
  学習時の値を記録する `env_config.json` には**既存 27 run すべてで
  `max_steer` / `steer_link` キーが無い**（`train_rl.py:678-684` は新しく記録する）。
  実測: 契約 26 個の `max_steer` は**全件 0.524**、`steer_link` は**全件なし**、`dt` は全件なし。
  **0.524 は「現在」ではない**（`max_steer = 0.524` は 2026-10-09 に削除、
  `0.47167` は 2026-10-20 に導入。`ml_lidar/runs/*` の mtime は v1:2026-09-07〜v27:2026-09-26）。
  → **既存契約は学習時と整合しており正しい。**
  正しい実装は同一リポジトリ内に既にある（`ml_cam_e2e/export_onnx.py:68-86` は
  `cfg.get("steer_link")` を先に使い、無ければ現在値で警告）。
- **影響**: 数値計算（`raspi.core.control_params.steer_relink`）では
  `steer_relink(0.4636,(1,0),L2) = 0.43769`（−5.59%）、
  `steer_relink(0.4636,L2,L2) = 0.4636`、`0.47167/0.4636 = +1.74%` と、
  元の数値主張は**再現する**。ただし `0.4636` は 2026-10-09〜2026-10-10 の実 `max_steer`
  （`0.4636100056642349`）で、**現在も `raspi/core/vehicle.py:119` の docstring に残る**。
  > **注（第2ラウンド再検証で訂正）**: 原文の「`0.4636` はリポジトリ内に存在しない」は
  > **誤り**。同じ文書の CFG-03 は `0.4636` を「最大舵角 8〜9 種」の 1 種として列挙しており、
  > 文書内で自己矛盾していた。
  実害は「再エクスポート時に学習時でない値を書ける」ことだけ。
- **修正案**: `env_config.json` に `max_steer` / `steer_link` がある run では**それを優先**し、
  無い run では警告ではなく**停止**（または `--assume-old-scale` の明示指定を要求）。
  契約 JSON に「学習時の値」と「現在の値」を両方のフィールドで残す。
  既存の 26 個の契約は**書き直さない**（現状は正しい）。

#### ML-03 [H] `preproc_version` ガードが学習と結び付いていない
- **判定**: `CONFIRMED`
- **場所**: `raspi/core/cam_e2e_preproc.py:14-16,27` / `ml_cam_e2e/export_onnx.py:110,43-51,68` /
  `ml_cam_e2e/train.py:176` / `ml_cam/export_onnx.py:78` / `ml_cam/train.py:212-215` /
  `raspi/nodes/cam_perception_node.py:291-310` / `raspi/nodes/cam_e2e_node.py:157-159` / `models/v1..v3.json`
- **内容**: `cam_e2e_preproc.py:14-16` は
  「`PREPROC_VERSION` はモデル同梱の `<name>.json` に書かれ、ノードが読み込み時に照合する。
  古い前処理で学習したモデルを新しい前処理で走らせる事故を、読み込み拒否で止める」と契約。
  しかし `ml_cam_e2e/export_onnx.py:110` は**現在の** `PREPROC_VERSION` を無条件に契約へ書き、
  `read_train_config()` は `input_size` / `max_steer` / `speed_ref` / `steer_link` しか見ない
  → **学習時の版とを比較しない**。
  `ml_cam/export_onnx.py:78` も同じ（しかも `ml_cam/train.py:212-215` の
  `train_config.json` に `preproc_version` は**記録されていない**。
  実測: `ml_cam/runs/v1..v3/train_config.json` にも無い）。
  セグメ側は**照合自体が存在しない**: `cam_perception_node.py:291-310` は
  `input_size` / `mean` / `std` / `threshold` しか読まず、契約 JSON が無ければ
  **既定値で黙って動く**（docstring 自身が「train/inference skew の温床」と認めている）。
  実測: `models/v1.json`〜`v3.json` には **`preproc_version` も `resize` も入っていない**。
  なお `cam_e2e_node.py:157-159` は**照合して拒否する**（`test_cam_e2e_node.py:132` にテストあり）。
- **影響**: 前処理が変わった segmentation モデルは**黙ってロードされ**
  `ftg_cam` / `cam_centerline` を無意味な確率で走らせる。cam_e2e 側は
  「旧版で学習 → 今日再エクスポート」すると契約に現在の版が書かれて**照合を通過する**。
- **修正案**: `ml_cam/train.py:212-215` に `preproc_version` を記録。
  両 exporter は**学習時の値**を契約に書き、違う場合は**エラー**にする。
  `cam_perception_node.py:291-310` に `preproc_version` 照合を追加し、
  契約 JSON が無いモデルは読まない方針に寄せる。`models/v1..v3` を再エクスポート。

#### ML-04 [H] `ml_cam` の ONNX エクスポートが学習時の入力解像度を反映しない
- **判定**: `CONFIRMED`（**実測で再現**）
- **場所**: `ml_cam/train.py:159,212-215` / `ml_cam/export_onnx.py:57-72,88-108,113,115` /
  `ml_cam_e2e/export_onnx.py:68-73` / `ml_cam/app.py:487-489,535-539`
- **内容**: 学習 `--size` 既定 224x224 を `train_config.json` に記録するが、
  エクスポートは `--size` 既定 224x224 で**`train_config.json` を読むコードが存在しない**。
  対比 `ml_cam_e2e/export_onnx.py:68-73` は `cfg["input_size"]` を使い、無ければ例外。
  `dynamic_axes` を渡さないため ONNX 入力 shape は静的（実測: `models/v1.onnx` / `v3.onnx` とも `[1,3,224,224]`）。
  **再現**: 学習済み `ml_cam/runs/v1/best.pt`（224x224）を `--size 128x128` で export すると
  ```
  書き出し完了: 最大誤差 4.54e-07
  契約: input_size=[128,128], preproc_version=1
  ONNX input shape [1, 3, 128, 128]
  ```
  → **入力解像度の不一致を検証は検出できない**（同じサイズで両者を比較するため）。
  ノードは契約の `input_size` を使うので**壊れずに精度だけ落ちる静かなズレ**。
  GUI `ml_cam/app.py:535-539` はエクスポートタブに**別々の入力欄**を持ち、
  「③学習タブと同じ値にしてください」と**手入力コピーを要求**する。
- **影響**: MobileNetV3 エンコーダ＋固定アップサンプルのデコーダなので
  解像度を変えても例外は出ず、**静かな精度低下**になる。
- **修正案**: `ml_cam/export_onnx.py` を `ml_cam_e2e/export_onnx.py` と同じ形へ
  （`--size` 既定 `None` → `train_config.json` の `input_size` を読み、無ければ**例外**）。
  GUI のエクスポートタブは入力欄を廃止して自動入力にする。
  `verify_parity()` に加えて「契約 `input_size` == ONNX 入力 shape == train_config の `input_size`」の
  三者一致を検証する。

#### ML-05 [M] `job_runner.drain()` の例外で Tk の `after` ループが永久に死ぬ
- **判定**: `CONFIRMED`（深刻度 High → **Medium**）
- **場所**: `ml_common/job_runner.py:57-59,72,99-104,107-115,117-140` /
  `ml_cam/app.py:681-684` / `ml_cam_e2e/app.py:636-638` / `ml_lidar/app.py:699-701`
- **内容**: `try:` が `while True:` 全体を覆い、`except queue.Empty: pass` だけ。
  `on_log` / `on_line` / `on_done` 由来の例外は素通りで伝播する（実測: `drain raised: RuntimeError boom`）。
  3 つの app はいずれも drain の**後**に `root.after(...)` を登録している
  → **drain が例外を投げれば `after` が再登録されず、以後の周期更新が恒久停止**。
  さらに `job_runner.py:72` の `self._procs[job_key] = None`（起動中プレースホルダ）に対し、
  `running_job_keys()`（:57-59）と `terminate_all()`（:107-115）は `p is not None` のみを見る
  → 実測 `running_job_keys with placeholder: []` / `terminate_all with placeholder: []`。
- **影響**: Tk の `mainloop` 自体は生き、ボタンは押せるが**ログとジョブ一覧の更新が固まる**。
  「GUI が二度と動かない」は過大表現。プレースホルダ窓は短いが存在する。
- **修正案**: 3 箇所の `_drain_log` を `try/except Exception` で包むか、
  `after` 登録を `finally` に移す。`drain()` 側でコールバック由来の例外を握り潰し、
  queue の drain は必ず完了させてから再送出する。
  `terminate_all()` / `running_job_keys()` はプレースホルダも「起動処理中」として扱う。

#### ML-06 [M] train/val 分割が記録（セッション）単位でなく、val 指標が一般化性能の推定になっていない
- **判定**: `CONFIRMED`
- **場所**: `ml_cam_e2e/samples.py:163-167` / `ml_cam/train.py:61-92,76-79` /
  `ml_cam_e2e/runs/v1/frames/manifest.csv` / `ml_cam_e2e/runs/v1/train_config.json` / `ml_cam/runs/v1/train_config.json`
- **内容**: `is_val()` は `block = t_ns // (5.0 * NS)` の **5 秒ブロックのハッシュ分割**。
  実測: `ml_cam_e2e/runs/v1/frames/manifest.csv` は 1303 行で
  **全行が単一 mcap**（`surge_20261006_114723.mcap`）、53 ブロック。
  seed 0 で **val 175 / train 1128**（`train_config.json` と一致）。
  → **val は「同じ記録の別 5 秒ブロック」**であり、別セッションへの外挿を測っていない。
  `ml_cam/train.py:76-79` は「ブロックが 2 未満なら**フレーム単位のランダム分割**へフォールバック」→
  **隣接フレームが train/val にまたがる**。
  実測値: `ml_cam_e2e/runs/v1` `best_val_mae = 0.05329`（= 0.0533 × 0.524 rad = 1.6°）、
  `ml_cam/runs/v1` `best_val_iou = 0.91704`（`n_train=78 / n_val=14`、leak を含む）。
- **影響**: val 指標は一般化誤差を大きく下回る。
  「同じ走行の別時間帯での誤差」であって「未知コースでの誤差」ではない。
- **修正案**: 分割の既定を**記録（`source_mcap`）単位の Leave-one-recording-out** にする。
  記録が 1 本しかないときは「一般化性能の推定は無い」と明示して警告する。
  `ml_cam/train.py:76-79` のフォールバック時は警告を出し
  `train_config.json` に `split_mode: "frame_random"` を記録する。

#### ML-07 [M] `--seed` が再現性を保証しない（numpy の RNG が未シード）
- **判定**: `CONFIRMED`
- **場所**: `ml_cam/train.py:166-173` / `ml_cam_e2e/train.py:132` /
  `ml_cam/dataset.py:92` / `ml_cam_e2e/dataset.py:65,71,72`
- **内容**: `ml_cam/train.py:166-173` のコメントは
  「augmentation（`dataset.py` の `np.random.rand()`）…も別系統の乱数源なので、
  ここで明示的に固定しないと再現性が取れない」と明記してから
  **`torch.manual_seed(args.seed)` のみ**呼ぶ。`np.random.seed()` はリポジトリ全体で未使用（grep 0 件）。
  実測: `torch.manual_seed(0)` 2 回で `np.random` の出力が **False**（相異なる）、
  `np.random.seed` ありなら **True**（同一）→ **`torch.manual_seed` は numpy をシードしない**。
  使用箇所: `ml_cam/dataset.py:92`（左右反転）、`ml_cam_e2e/dataset.py:65`（flip）・`:71`（contrast）・`:72`（brightness）。
- **影響**: 同じ `--seed` でも augmentation が毎回変わり、
  学習結果（`best.pt`・val 指標）が再現しない。
- **修正案**: `np.random.seed(args.seed)`（必要なら `random.seed`）を追加。
  より良い形: `dataset.py` に `np.random.default_rng(seed)` の `Generator` を持たせ、
  DataLoader の `generator` / `worker_init_fn` と一緒に渡す。
  `train_config.json` に `seed` を記録（`ml_cam` は未記録）。

#### ML-08 [M] `e2e_lidar` は推論の速度がそのまま指令になり、独立した安全策が無い
- **判定**: `CONFIRMED`（深刻度 High → **Medium**。バックストップとして STM32 `auto_stop` が存在する）
- **場所**: `raspi/auto/e2e_lidar.py:32,132-134,259-261,317,333` / `raspi/auto/follow_the_gap.py:246-248` /
  `raspi/auto/disparity_extender.py:245-248` / `raspi/auto/gap_pursuit.py:249-257` /
  `raspi/auto/cam_centerline.py:148-153` / `raspi/auto/cam_e2e.py:125-132` / `raspi/auto/follow_object.py:156,191,202` /
  `raspi/tests/test_auto.py:1507-1523`
- **内容**:
  ```python
  # raspi/auto/e2e_lidar.py:259-261
  # ★独立した安全策。モデルの判断を経由しない「事実」としての正面の余裕
  front = [w.dist[j] for j, d in enumerate(w.degs) if abs(d) <= 20]
  st.free_ahead = min(front) if front else 0.0
  ```
  `free_ahead` の参照は `e2e_lidar.py:32,132,134,261` のみで、**速度計算に一切使われない**。
  速度は `:317` `speed = (speed_norm + 1.0) * 0.5 * self._model_max_speed` → `:333`
  `st.target_speed = max(0.0, min(max_speed, speed))`（**モデル出力がそのまま**）。
  `st.brake` は**どこにも設定されない**。
  同一ファイル内で `:132-134`（「安全側の停止判定に使う」）と `:32`（「診断表示用」）が矛盾。
- **影響**: 正面 10 cm の障害物でも `brake=False`。ただしこれは
  `raspi/tests/test_auto.py:1507-1523` で**意図として固定**されている
  （「緊急停止は STM32 の `auto_stop` に任せる」）。バックストップはファームに実在する。
  実害は「コメントが実装と矛盾し、防御されていると誤認させる」こと。
- **修正案**: コメントの矛盾を解消する。他モードと揃えるなら
  `free_ahead` ベースの減速ランプを `e2e_lidar` にも入れ、
  `v = min(model_speed, v_from_free_ahead)` とする。

#### ML-09 [M] `ml_lidar/env.py` の自動停止定数が firmware v0.29 から消えた値を写している
- **判定**: `PARTIAL`（**停止距離の数値は再現せず**。実測は 19〜28% の過小評価）
- **場所**: `ml_lidar/env.py:178-186,592-593` / `sim/stm32.py:74-75,577-579` /
  `config/vehicle.toml` `[limits].abs_brake_decel_m_s2` / `[control.plant].md_delay_s,md_tau_s` /
  ファーム `src/vehicle/vehicle.h:56-63` / `src/control/control_params.h:97-100`
- **内容**: `_AUTO_STOP_DELAY_S = 0.1` / `_AUTO_STOP_DECEL_M_S2 = 3.0` / `_AUTO_STOP_MARGIN_M = 0.15`。
  firmware v0.29 で `VEHICLE_BRAKE_DECEL_M_S2` / `VEHICLE_AUTO_STOP_DECEL_M_S2` を削除し
  `CONFIG_SET` の `brake_decel_m_s2`（既定 0.5、範囲 0.5〜10）と `md_torque_lag_s` に移行した。
  `sim/stm32.py` は v0.29 に**正しく追従済み**
  （`AUTO_STOP_SENSOR_DELAY_S = 0.1` と `brake_decel` の導出を含む）。
  つまり **`sim/` と `ml_lidar/` で同じ車が回っていない**。
  実測停止距離: v=1.0 で ml_lidar 0.4167 m / 実車 0.4969 m（**+19.3%**）、
  v=2.0 で 1.0167 m / 1.3058 m（**+28.4%**）。元の主張（0.41 / 0.53 m）は 0.41 のみ一致。
- **影響**: RL 方策は**実車より 19〜28% 手前で止まれ**と学習される。
  方針が障害物に dangerously 近付く方向の誤り。
- **修正案**: `delay = 1/LIDAR_SCAN_FREQ_HZ + md_torque_lag_s`、
  `decel = brake_decel_m_s2`、マージン 15 cm とし、値は `config/vehicle.toml` から読む。
  学習済み run（v1〜v27）はこの近似で学習しているので、v28 以降で再学習する旨を明記。

#### ML-10 [M] カメラ E2E の前処理版 guarding が半分だけ、モデル再読込が名前のみ
- **判定**: `PARTIAL`（**cam_e2e 側は検査あり**＝「半分だけ」の主張は正しい。`ml_cam` 側と再読込は CONFIRMED）
- **場所**: `ml_cam_e2e/export_onnx.py:110` / `raspi/nodes/cam_e2e_node.py:157-159,214` /
  `raspi/nodes/cam_perception_node.py:291-310,312-322,334` / `models/v1..v3.json`
- **内容**: `cam_e2e` 側は `preproc_version` を**検査する**（`test_cam_e2e_node.py:132` にテストあり）。
  `ml_cam` 側は**検査するノードが存在しない**（`grep preproc_version` の全体結果は
  `ml_cam_e2e/export_onnx.py:110` / `ml_cam/export_onnx.py:78` / `cam_e2e_node.py:157` のみ）。
  モデル再読込が**名前のみ**のキーで判定されている:
  `cam_e2e_node.py:214` と `cam_perception_node.py:334` はともに
  `if not desired_name or desired_name == self._loaded_model_name: return False`。
  → **同じ名前で再エクスポートしたモデルは永遠に再読込されない**。
  GUI は「新しいモデルを選択した」と表示し、ノードは古い ONNX セッションを動かし続ける。
- **影響**: 再読込の唯一の手段は `systemctl restart surge-cam-e2e` で、GUI から到達できない。
- **修正案**: 再読込キーを `(name, onnx_mtime_ns, json_mtime_ns)` にする
  （`cam_perception_node._model_key` は mtime 入りなので流用できる）。
  `cam_perception_node.py:291-310` に `preproc_version` 照合を追加。

#### ML-11 [M] `ml_cam_e2e` のラベル正規化に現在の `vehicle.toml` を使う
- **判定**: `PARTIAL`（コードの主張は CONFIRMED。**「全舵角で約 11% オーバーステア」は再現せず**）
- **場所**: `ml_cam_e2e/train.py:144,178` / `ml_cam_e2e/extract_pairs.py` /
  `ml_cam_e2e/runs/v1/frames/manifest.csv` / `ml_cam_e2e/runs/v1/train_config.json`
- **内容**: `ml_cam_e2e/train.py:144` `max_steer = Vehicle.load().max_steer`、
  `:178` `"steer_link": list(Vehicle.load().steer_link)` で**現在**の値をラベル正規化に使う。
  manifest の実測: `max |target_steer| = 0.524`（**旧スケール**）、
  `|target_steer| ≥ 0.47167` の行が **271（20.8%）** → 現在の max_steer で正規化すると
  **20.8% が飽和（clip）する**。manifest には**換算の由来（当時の max_steer / steer_link /
  ファーム版）が無い**。
  既に学習済みの run は**正しい**（`train_config.json` に `max_steer: 0.524` / `steer_link: [1,0]` を記録し、
  `export_onnx.py:78-86` がそれを契約に書き、`cam_e2e_node.py:281` の
  `steer_relink(angle,(1,0),L2)` で正しく路面舵角へ変換される）。
  「今日再学習」した場合の実測（物理モータ角の比）:
  | 学習時の舵 | 物理モータ角の比 |
  |---|---|
  | 0.05 | 0.909（−9%） |
  | 0.20 | 0.929（−7%） |
  | 0.30 | 0.961（−4%） |
  | 0.40 | 1.021（+2%） |
  | 0.47167 | **1.110（+11%）** |
  → **小舵角で −9%（舵不足）、0.4 rad 以上で +2〜+11%（オーバー）**の角度依存歪み。
- **影響**: 「全舵角で 11% オーバーステア」は誤り（11.09% は `0.524/0.47167` の
  ラベル正規化比由来で物理効果の角度依存性を無視している）。中域で約 10% のオーバーステア。
- **修正案**: `ml_cam_e2e/train.py` に `--max-steer` / `--steer-link` を明示できるようにする。
  `extract_pairs.py` の manifest に**その記録の時の `max_steer` / `steer_link` / ファーム版**の列を
  追加し、抽出時点でスケールを確定させる。
  正規化前に**教師信号のスケール不一致検出**（割合表示）を入れる。

#### ML-12 [L] 「362/363 次元レガシー」対応が現行グラフと無関係
- **判定**: `CONFIRMED`
- **場所**: `raspi/auto/e2e_lidar.py:99-101,275-290` / `models/e2e_lidar/*.onnx`（26 個）
- **内容**: 実測（onnx で入力 shape を読む）**全件 `[1, 273]`**。
  `e2e_lidar.py:275-290` の分岐は「点群+速度のみ(362次元)」「点群+速度+ステア(363次元)」を前提に
  `extra_dim = self._model_in_dim - len(scan_n)` で 1 個/2 個を足す実装 →
  **どの現行モデルにも当てはまらない死んだ分岐**。`:101` のコメントも古い。
- **修正案**: コメントを 273 次元に更新し、`extra_dim != 2` のときは**例外**にして
  「旧モデルは非対応」と明示する。

---

### 3.6 局所化・SLAM・地図

#### SLM-01 [H] `slam2d` の `optimize_every > 0` は `ValueError` で落ちる
- **判定**: `CONFIRMED`（ただし「必ず」ではなく**特定条件**）
- **場所**: `slam2d/pipeline.py:98-102,146-147,172-193` / `slam2d/core/frontend.py:574-575` /
  `raspi/auto/_slam2d_nav.py:206` / `raspi/auto/slam2d_raceline.py:1321,459`
- **内容**: 実ファイルは `slam2d/core/pipeline.py` ではなく **`slam2d/pipeline.py`**（原指摘のパス誤り）。
  `update()` は frontend → `_advance_job` → `_sync_graph` の順で、
  `_sync_graph` は**最後**。`_advance_job` の時点ではグラフノード数がキーフレーム数より 1 少ない。
  `_add_loop` → `optimize()` → `optimize_steps` → `poses = self._graph.all_poses()` →
  `frontend.apply_correction_steps(poses, chunk)` → `frontend.py:574-575` で
  `if len(poses) != len(self.keyframes) or not poses: raise ValueError("poses must match keyframes")`。
  **再現（`slam2d/tests/helpers.py` の oval 合成シーン、kf_dist=0.01・ジャイロバイアス 1°/s・2 周）**:
  ```
  === A) optimize_every=1, kf_dist=0.01 ===
  [EXC] update #336: ValueError: poses must match keyframes
         keyframes=337 graph_nodes=336
  === B) optimize_every=0 (既定), kf_dist=0.01 ===
    no exception: True loops: 16
  === C) optimize_every=1, kf_dist=0.1 (既定) ===
    no exception: True loops: 25 opt: 25
  ```
  → 「**その周でキーフレームが追加された時だけ**長さが合わず落ちる」ことが確認された。
  逆説的に C ケース（`kf_dist=0.1`）では落ちない。
- **影響**: `planning_node.py:356-364` が例外を捕まえるのでノードは死なないが、
  **最適化済みのグラフと未補正のフロントエンドが恒久的に不整合**になる
  （`_sync_graph` はノード数が増えないので以後補正が反映されない）。
  実運用は顕在化しない（`_slam2d_nav.py:206` が `optimize_every=0` を明示、
  `slam2d_raceline.py:1321` は `freeze_steps(FREEZE_CHUNK=8)` を使う）。
  **だが config 1 行で壊れる。**
  > **注（第2ラウンド再検証）**: 深刻度 Critical → **High**。
  > `grep optimize_every` の結果、既定 0 に加え運用コード
  > （`raspi/auto/_slam2d_nav.py:206`）も 0 で、**`>0` を使う経路は未使用**。
  > 「潜在罠」ではあるが稼働障害ではなく、他の H 級項目と격을揃える。
- **修正案**: `_sync_graph()` を `frontend.update()` の直後（`_advance_job` の前）に移すか、
  `_add_loop` 内の `optimize()` を次回の `update()` 冒頭で遅延実行する。
  `apply_correction_steps` の `ValueError` を「1 個分のずれ」として救済する。
  **`optimize_every > 0` の経路をテストに追加**。

#### SLM-02 [H] SLAM の「一致を試みなかった」が `lost` として扱われず、DR pose で地図に焼き込まれる
- **判定**: `CONFIRMED`
- **場所**: `slam2d/core/frontend.py:313-314,322,336,338,368,406-410,462-506` /
  `slam2d/backend/loop_detection.py:217-220`
- **内容**:
  ```python
  # slam2d/core/frontend.py:313-314
  matched = field_ is not None and not field_.empty and hx.size > 0
  lost = False                      # ← matched が False でも False のまま
  ...
  if matched:                       # ← 判定ロジック全体がここの中
  ```
  `lost = True` は `:338` の `else` にしか無い。`matched=False` のとき
  `lost` は False のまま、`lost_streak` は増えず、`restart_after` の再構築が発火しない。
  さらに `frontend.py:368` の
  `if not self.grid.frozen and not lost and (restarted or self._moved_enough()): self._add_keyframe(pts, hx, hy, res)`
  が通り、**デッドレコニングの pose が trajectory に入り、スキャン Wrong Pose で地図に統合される**。
  **再現（実 Frontend + 実 OccupancyGrid、40 周正常走行後に全点飽和スキャン 1 回投入）**:
  ```
  通常走行後: keyframes=20 lost_streak=0
  飽和スキャン: matched=False lost=False score=0.0
  keyframes 20 -> 21
  姿勢は DR 予測のまま動いた: 0.060 m
  lost_streak=0
  ```
  併せて 2 つの非対称:
  **(a) 受理閾値が 45 cm まで緩む**: `frontend.py:406-410`
  `limit = max(cfg.max_rms, self.rms_ref * cfg.rms_factor)`。`:336` の
  `self.rms_ref += a * (min(res.rms, cfg.max_rms) - self.rms_ref)` は**上限丸め**なので
  `rms_ref ≤ 0.15` → `limit ≤ max(0.15, 0.45) = 0.45 m`（実測で 0.450 m を確認）。
  **(b) 再位置づけに前 pose からの拘束が無い**: `frontend.py:462-506` の `_relocalize` に
  補正量ガードが無い一方、`slam2d/backend/loop_detection.py:217-220` には
  `max_correction + max_correction_per_m * gap` ガードがある（**非対称**）。
- **影響**: RMS 45 cm のフィットが受理され、その pose が書き込まれ、
  **スキャンがその誤った pose で地図に焼き込まれる（幽霊の壁）**。
  オドメトリの事前分布（σ ≈ 5〜10 cm）は 360 点に対して弱すぎて否决できない。
  対称なコースで**鏡像の位置に飛びうる**。
- **修正案**: `matched` が False（地図は空でない）で `lost = True`（または独立した
  `degenerate` フラグ）とし、**キーグラフ追加と地図統合を抑制**する。
  `rms_ref` の**上限丸めを止め**、受理 RMS の絶対上限（5〜8 cm）を設ける。
  `_relocalize` に `max_correction` を追加する。

#### SLM-03 [H] SLAM のループ検出ゲートが LD06 の距離比例測距誤差に脆い
- **判定**: `CONFIRMED`（**表ほぼ一致で再現**）
- **場所**: `slam2d/backend/loop_detection.py:71,187-198` / `slam2d/core/register.py:63-69,304-305` / `sim/params.py:42`
- **内容**:
  ```python
  # slam2d/backend/loop_detection.py:197-198
  if coarse.second >= config.ambiguity_ratio * coarse.score:
      return None
  ```
  `ambiguity_ratio` 既定 0.9。粗い段の得点は `register.py:305-306` の
  `exp(-d²/2σ²)` の点平均、`sigma = max(0.06, step*0.8)`（最大 0.35 m）、**64 点**。
  `register.py:63-69` は `sigma_rel = 0.01` で LD06 の距離比例誤差をモデル化済み。
  **再現（oval 2 周・バイアス 1°/s・σ = 1cm + rel×距離）**:
  | rel | ambiguity=0.9 | ambiguity=0.98 |
  |---|---|---|
  | 0.0% | **17**（第2回 16） | **35**（第2回 34） |
  | 0.5% | **15**（第2回 14） | **35**（第2回 34） |
  | 1.0% | **10**（第2回 9） | **33**（第2回 32） |
  > **注（第2ラウンド再検証）**: 独立再検証では**全セル 1 個ずつ少ない**（計数慣例の差）。
  > 傾向・比率・棄却理由の分布（曖昧さゲート 25/32/34 回、「min_score」0 回）は完全一致。
  棄却理由の Counter は**「曖昧さゲート」が支配的**（48 回の粗い段のうち 25〜34 回がここで落ちた）。
  `min_score` での棄却は 0 回。**`max_rms` / `min_inlier` は原因ではない**
  （0.03→0.15、0.7→0.4 に緩めても loops は 17→18 / 35→37 / 15→15 / 10→10 とほぼ変わらない）。
- **影響**: **ループ閉じはジャイロのドリフトを直す唯一の機構**なので、
  検出率が落ちるとマップは周回ごとに劣化する。
  `register.py` は LD06 の距離比例誤差を正しくモデル化しているのに、
  `loop_detection.py` の閾値はそれと整合していない。
- **修正案**: 粗い段の `sigma` を測距精度から逆算（`sqrt(sigma² + (sigma_rel·d)²)`）へ置換。
  `ambiguity_ratio` の既定を 0.9 → 0.95 程度へ。
  「曖昧だが best/second がどちらも十分高い（例 score > 0.8）」場合は採用する多段ゲート。
  `min_inlier` / `max_rms` を `register.sigma_rel` から導出して整合させる。

#### SLM-04 [H] 地図アップロードに展開後のサイズ境界が無く、Pi のメモリを枯渇させる
- **判定**: `CONFIRMED`（数値の一部が原指摘と相違）
- **場所**: `raspi/auto/mapstore.py:266-270,274-276,301-307,350-386` /
  `raspi/nodes/telemetry_node.py:408,946-948,999-1000,1113-1116,1433,2796-2832,3629`
- **内容**: `_load_npz()` は `trinary` の**dtype・値域（0/1/2）・2 次元**しか検査せず、
  **要素数・幅高の上限が無い**。`centerline_xy` / `raceline_xy` も `reshape(-1,2)` するだけ。
  `MAX_MAP_UPLOAD_BYTES = 32MB` は**圧縮後の Wire 長**の制限。
  **再現**:
  ```
  20000x20000 圧縮後 Wire 長 = 390,371 bytes (381.2 KiB)
  validate_upload -> LoadedMap
  展開後の要素数 = 400,000,000 (381 MiB)
  100000x100000 圧縮後 Wire 長 = 9,721,104 bytes (9.27 MiB) / 展開 9.3 GiB
  MAX_MAP_UPLOAD_BYTES = 33,554,432 -> wire長は上限内か: True
  ```
  → **9.27 MiB のアップロードで 9.3 GiB 確保**（原指摘の「5 倍で 10 GB」は
  deflate がほぼ線形に増えるため 25 倍相当が正しい）。32 MB 上限には掛からない。
  追加リスク: `np.isin(trinary,(0,1,2)).all()` が 400M 要素で bool 配列と中間配列を作るため、
  **検証中 alone で peak RSS が 4.73 GB**（第2ラウンド実測）になる。
  `import_upload` は 32 MB の Wire 長制限の**前**に `validate_upload` を呼ぶため、
  400 MB 級でも既に危険。
  **認証**: `_map_upload_channel` に**トークン検査が無い**（`_token_ok` は `:1433` で定義され
  `:1113` の `_control_channel` 側だけで呼ばれる）。`--token` は任意（`:3629`、既定 ""）。
  Origin 検査は `:948-950` で「Origin が無い＝ブラウザ以外（curl・自作クライアント）は通す」。
- **影響**: telemetry_node プロセスの OOM knock（走行中の `cmd` 定期送信が止まり
  `io_node` が途絶と見て DISARM する）。
- **修正案**: `_load_npz()` に `trinary.size`（例 4,000,000 セル）と幅高の上限、
  `centerline` / `raceline` の点数上限を Reject し、`np.isin` を
  `np.all((t<=1)&(t>=0))` に替えて一時配列を避ける。
  `_map_upload_channel` に `_token_ok()` を足す。

#### SLM-05 [H] `mapstore.save_map` に検証が無く、恒久的に読み込めない地図を作れる
- **判定**: `CONFIRMED`（**完全再現**）
- **場所**: `raspi/auto/mapstore.py:165,188-224,281-283,564-574`
- **内容**: `save_map()` は名前・既存ルート可否のみ検査し、
  `resolution` / `origin` / NaN を検査しない。読み込み側は `:281-283` で
  `math.isfinite(m.resolution) and m.resolution > 0.0 and math.isfinite(origin_x) and math.isfinite(origin_y)` を要求。
  **再現（`MAPS_DIR` を一時ディレクトリへ差し替え）**:
  ```
  メタ JSON の生バイト: {"created_at":..., "resolution": NaN, "width": 10, "height": 10, ...}
  list_maps(): [{'name': 'nanmap', ..., 'resolution': nan, ...}]
  load_map('nanmap') -> None
  strict JSON として不正か: True (NaN)
  ```
  → **読み込めない地図が一覧に居座る。**
  `mapstore.py:165` の `json.dumps` に `allow_nan=False` が無いため
  `NaN` は**厳密 JSON として不正**なファイルになる。
  `mapstore.py:564-574` の `save_grip_surface` も範囲検証なし。
- **影響**: 地図一覧に「読めない地図」が残り、GUI から消せない。
- **修正案**: `save_map()` の入口で `math.isfinite` 検証（読み込み側と**同じ**条件を共有関数化）し
  `ValueError` を投げる。`json.dumps(meta, allow_nan=False)` に。
  `save_grip_surface` にも範囲検証を入れる。

#### SLM-06 [M] 地図の永続化フォーマットに版が無い
- **判定**: `CONFIRMED`
- **場所**: `raspi/auto/mapstore.py:78-79,153-165,170-183` / `raspi/nav/grip.py:430,454`
- **内容**: `_REQUIRED_KEYS` に版キーが無い。`.json` メタも `created_at / resolution / width /
  height / raceline_points / length_m` のみで **`format` なし**。
  一方 `raspi/nav/grip.py:430,454` は `"format": 1` を持つのに不一致を raise する。
  > **注（第2ラウンド再検証で弱化）**: grip.json の版と地図 npz/json の版は
  > **別ファイル・別スキーマ**の版であり、「地図に版が無い」ことの根拠にはならない。
  > 背景づけ（同一リポジトリの他の永続フォーマットには版放入の慣行がある）として読むべき。
  `list_maps()` は `except (OSError, ValueError): continue` で
  **`.json` 欠損・破損を黙って skip** する（実測: `.npz` だけ置いた地図と
  破損 `.json` の地図はいずれも一覧に出ない）。
- **修正案**: npz と meta の両方に `"format": N` を入れ、ロード時に検証する
  （旧形式は「版なし → 1 とみなす」移行読みを 1 版だけ許す）。
  `list_maps()` は json が読めなければ npz にフォールバックする。

#### SLM-07 [M] `mapstore` の tmp 名固定・書込順序・`import_upload` の順序
- **判定**: `CONFIRMED`
- **場所**: `raspi/auto/mapstore.py:141-150,207-208,219,370-386,505-507,544-547`
- **内容**:
  - **tmp 名**: `_write_text_atomic` は `tmp = p.with_name(p.name + ".tmp")`、
    `save_map` の npz tmp も `target.with_name(target.name + ".tmp")`、
    `_write_bytes_atomic` と `_grip_vehicle_write` も固定名 → **2 プロセス競合**
    （互いを上書きし、`os.replace` が FileNotFoundError になる実害あり）。
  - **保存順序**: `:207-208` の grip / lines の unlink が
    **npz 書込（`:223`）より前** → npz 書込が失敗すると
    **旧地図は残るのに学習値と経路の写しが消える**。
  - **import_upload の順序**: `:370-386` は npz 書込 → meta 書込 → routes 処理
    → **npz 書込後・routes 処理前に落ちると、別地図の経由点が新地図に残る**。
- **修正案**: tmp 名に `os.getpid()` とランダム成分を入れる。
  `save_map` の grip/lines 削除を npz の `os.replace` 成功後へ移動する。
  `import_upload` は「旧 routes を退避 → npz 書込 → routes 決定 → 書き戻し/削除」の順に。
  並行書き込みテストを追加（VER-13）。

#### SLM-08 [M] `deskew` の二重化と `t_ref` フォールバック差
- **判定**: `PARTIAL`（`t_ref` 差と `roadgraph` / `centerline` の unknown 扱いは CONFIRMED。
  二重実装の表の一部は誤り）
- **場所**: `slam2d/core/deskew.py:78` / `raspi/nav/deskew.py:121,148` /
  `raspi/auto/_slam2d_nav.py:304-309` / `sim/raceline.py:129` / `raspi/nav/raceline.py` /
  `raspi/nav/roadgraph.py:304-314` / `raspi/nav/centerline.py:258-274` / `slam2d/core/grid.py:332-371`
- **内容**: `t_ref` のフォールバック差:
  ```python
  # slam2d/core/deskew.py:78
  t_ref = int(t_pt.max()) if t_pt.size and t_pt.max() > 0 else 0          # ← 0
  # raspi/nav/deskew.py:148
  t_ref = int(t_pt.max()) if t_pt.size and t_pt.max() > 0 else int(scan.t_capture)   # ← _scan
  ```
  SLAM 側で `t_ref` が 0 のままになると、`_slam2d_nav.py:304-309` の
  `if not t0 or t1 <= t0 or t1 - t0 > 500_000_000 or len(fe.twists) < 2: return fe.pose` が
  常に成立し、**追従に使う姿勢が最大 500 ms 古い**ままになる
  （原指摘の「最大 100 ms」は `500_000_000 ns = 500 ms` で**数値が誤り**）。
  二重実装の表の真偽: `slam2d/core/grid.py` vs `raspi/nav/grid.py`（両方存在・別用途）、
  `localmap.py` vs `local_map.py`（**別目的**：局所地図 vs ESDF／車体クリアランス）、
  `register.py` vs `scan2scan.py`（**アルゴリズムが別**：点対線ICP vs 相関スキャンマッチ。並存は妥当）、
  `raspi/nav/raceline.py::min_curvature_alpha` vs `sim/raceline.py::_min_curvature_alpha`
  （**明確な移植**。`sim/raceline.py:129` が「移植」と明記）、
  `slam2d/core/types.py` vs `raspi/nav/se2.py`（pose 型の重複）、
  `roadgraph.py::build_graph` vs `grid.py::UnknownWallGrid`（**対応が誤り**。別物）。
  **`build_graph` が unknown を通行不可として扱い、`centerline.measure` は壁だけを見る**という主張は
  **CONFIRMED**: `roadgraph.py:304-314` は `free = trinary == 1` /
  `unknown = trinary == 0` / `free = free | (closed & unknown)`（未知の穴だけ埋める）→
  以後 `cspace = clear >= keep` は **`free` のみ**から計算 → 残った unknown は通行不可。
  一方 `slam2d/core/grid.py:332-371` の `raycast` は占有セルだけを壁扱いし
  **unknown は透過**（＝壁を見ない）→ **意味論が食い違う**。
- **修正案**: ① `t_ref` のフォールバックを `scan.t_capture` 相当に統一
  ② `sim/raceline.py::_min_curvature_alpha` は `raspi/nav/raceline.py` から import する
  ③ `_slam2d_nav.py:pose_now` の 500 ms を実測（点群周期 100 ms の 2〜3 周）に合わせ、
  古い姿勢で追従しない ④ `centerline.measure` と `build_graph` の unknown 扱いを揃える

#### SLM-09 [M] `mapstore` の `json.dumps` に `allow_nan=False` が無い
- **判定**: `CONFIRMED`
- **場所**: `raspi/auto/mapstore.py:165`
- **内容**: `json.dumps(meta)` に `allow_nan=False` が無い。
  実測バイト: `{"created_at":..., "resolution": NaN, ...}` → **RFC 8259 の JSON として不正**。
  Python の `json.loads` は既定で NaN を受理するので `list_maps()` は壊れず silent に進むが、
  GUI 側（`JSON.parse`）や他ツールで読むと落ちる。
- **修正案**: `json.dumps(meta, allow_nan=False)` にして `ValueError` を捕捉して明示的に拒否する。
  `list_maps()` 側も `parse_constant` で NaN/Infinity を弾く。→ SLM-05 と同じ修正。

---

### 3.7 GUI・WS・フロントエンド

#### GUI-01 [M] GUI が「指令が通っているか」を表示しない
- **判定**: `PARTIAL`（「一切表示しない」は FALSE。`cmd_stale` などが診断タブのみなのは TRUE）
- **場所**: `gui/src/ws/control.ts:143-145` / `gui/src/components/StatusBar.tsx:46-47,51,85-87,119-126,169-171,200-203,226-228` /
  `gui/src/store/ui.ts:579,585,586,589,599` / `gui/src/input/useDriving.ts:930,987,1000,1029-1030,1221-1222` /
  `gui/src/components/diag/pages/LinkPage.tsx:132-137` / `gui/src/components/diag/checks.ts:94-192` /
  `gui/src/components/AutoPanel.tsx:197,371-372,396`
- **内容**: `send()` は戻り値も例外も無しで送信失敗を完全に握り潰す。
  E-STOP ボタンに `disabled` が無い（`control.ts:196` のコメントで意図的）。
  `ui.deadman` / `ui.hasControl` / `ui.controlOpen` は**ロジックのみで**使われ、
  **コンポーネントは 1 つも読んでいない**。`ui.deniedBy` は**書き手しか存在せず読み手ゼロ＝死んだ状態**。
  サーバ側の権威的な `link.cmd_stale` / `status.deadman_trips` / `status.stale_cmds` は
  **診断タブのみ**（`LinkPage.tsx:132-137`）。ラジコンタブ・`checks.ts` には無い。
  `StatusBar.tsx:226-228` の「ARM 保持中 12.3s」は
  **GUI 自身の無操作タイマー**のカウントダウンで、**STM32 が ARM を受け取った証拠ではない**。
  `AutoPanel.tsx:197` の緑バッジは `auto.engaged && ui.armRequested && !silent && st?.phase !== 'DONE'` のみで
  **`AutoState.ready` を見ていない**（同じ画面の `:371-372` で `st?.ready` と `st?.reason` は表示している）。
- **影響**: 逆の証拠（＝「一切」の反例）: `StatusBar.tsx:85-87` に ARMED/DISARM（STM32 実状態）、
  `:46-47,119-126` に health / RTT / 通信途絶、`DriveControls.tsx:62-63` に `vs.side_brake_active`、
  `:51` に E-STOP 発動表示。**「検針経路」が見えない**のが実害。
- **修正案**: ① `send()` を `boolean` 返しにし、E-STOP / DISARM は失敗時に
  **ラッチする可視な「E-STOP 未送达」表示**を出す ② `cmd_stale` / `deadman` / `hasControl` を
  StatusBar のピルとして描画し `runChecks()` にも行を追加
  ③ `ui.deniedBy` を実装するか削除する
  ④ 「走行中」バッジを `st?.ready` でゲートし「判断待ち」を明示する

#### GUI-02 [M] `SignalConfig` の既定値が 5 箇所・31 リテラル
- **判定**: `PARTIAL`（**GUI 側のファイル名の訂正は正しい**が、個数の記載は過少計上）
- **場所**: `raspi/msgs/types.py:851-857` / `raspi/nodes/telemetry_node.py:759-765` /
  `raspi/nodes/arrow_signal_node.py:108,114,115,118,327` /
  **`gui/src/components/SettingsPanel.tsx:693,716,721,729,734,746,750,758,761,770,773,782,787`**
- **内容**: `types.py` の `SignalConfig` 7 個 ＋ `types.py:873,877-878` の
  `ArrowSignalStatus` 3 個 ＋ `telemetry_node.py` 7 個 ＋ `arrow_signal_node.py` 7 個
  ＋ GUI 13 個 = **31 リテラル / 5 箇所**。
  値は全箇所で完全一致
  （`enabled=True, roi_top=0.05, roi_bottom=0.55, sat_min=40, sat_max=180, val_min=150, min_lit_frac=0.02`）。
  **GUI 側は `gui/src/store/ui.ts` ではなく `gui/src/components/SettingsPanel.tsx`**（原指摘の誤り。
  `store/ui.ts` に SignalConfig の既定値は 1 つも無い）。
  テストで守られていない: `tools/fwsim/tests/test_fw_constants.py:126-141` は
  `gui/src/store/ui.ts` の**トルク定数**を正規表現で突合しているだけで、
  SignalConfig の既定値は対象外。
  > **注（第2ラウンド再検証で訂正）**: 「`grep -rn "SignalConfig" raspi/tests tools/` も該当なし」は
  > **誤り**（`raspi/tests/test_arrow_signal_node.py` に 12 件以上の参照がある）。
  > ただし当該テストは**明示値を投入するだけ**で、
  > **`SignalConfig` の既定値そのものを検証する assert は 0 件**（核心は正しい）。
- **影響**: 値は今すべて一致しているが**守られていない**。トルク上限 0.15 は
  `test_fw_constants.py` が突合しているのに、こちらは無検査。
- **修正案**: Python 側を `raspi/msgs/types.py` を正とし、
  `arrow_signal_node` / `telemetry_node` は `from raspi.msgs.types import SignalConfig` で取得する。
  GUI 側を `gui/src/store/ui.ts`（または生成物）へ 1 箇所寄せ、`SettingsPanel.tsx` は
  `SIGNAL_DEFAULTS` を参照する形にする。`config/gen_msgs.py` に SignalConfig を生成対象として足す。

#### GUI-03 [M] GUI → Python の `cmd` 契約が両側手書き
- **判定**: `PARTIAL`（両側手書きは TRUE。**ワイヤ上の契約が食い違っているわけではない**）
- **場所**: `gui/src/types.ts:427-475` / `raspi/nodes/telemetry_node.py:1159,1467-1523` /
  `config/gen_msgs.py:33-38,63-72` / `raspi/msgs/types.py:337-393`
- **内容**: TS の `CmdOut` は **19 フィールド**（`cmd()` が付ける `t` を含めたワイヤ上のキー数は 20）。
  Python の `_decode_cmd` は 18 フィールドを読む。**TS が宣言して Python が読まないフィールド: 0 個**。
  **Python が読んで TS が宣言していないフィールド: 1 個 = `t`**（`performance.now()`）。
  不可視の改名 2 つの主張は**契約の破れではない**:
  TS は `speed` / `steer` を宣言し、`_decode_cmd` も JSON キーを `speed` / `steer` として受け、
  `DriveCmd` への明示代入（`types.py:1503-1504`）で `target_speed` / `target_steer` になる
  → **ワイヤ上の契約は完全に一致**（差があるのは Python 内部構造体の名前だけ）。
  代入の実体は `types.py:1500-1501`。
  `ch.disarm()` は 20 宣言フィールドのうち **5 個だけ**を送る（`{type, t, mode, arm, light_mode}`）。
  残りは Python の既定値に依存し、`brake_torque` の既定は **0**（=「未指定 = STM32 最大制動」、
  `telemetry_node.py:1504-1505` のコメント）。
  `config/gen_msgs.py:63-64` は `DriveCmd` を生成対象外にしており、
  **Python 側には `DriveCmd` が実在する**（`types.py:337-393`）＝元主張の「Python 側に型が無い」は FALSE。
- **影響**: フィールドを足す/改名すると静かに既定値（=最大制動）になる。
  問題は「JSON 形の写し」が生成されないこと。
- **修正案**: `DriveCmd` を正として `CmdOut` を生成対象にする
  （Python→TS の名前マップで `target_speed`→`speed` 等を明示）。
  `disarm()` は完全な `CmdOut` を構築する（SAF-02 と共通）。

#### GUI-04 [M] `/ws/map` にスキーマ検査が無い
- **判定**: `CONFIRMED`
- **場所**: `gui/src/ws/telemetry.ts:45-52` / `gui/src/ws/map.ts:149` /
  `raspi/msgs/schema.py:42` / `raspi/nodes/telemetry_node.py:1035-1049,3049-3071,3121-3126`
- **内容**: `/ws/telemetry` は `if (raw.schema !== MSGS_SCHEMA) { return }` で全フレームを捨てる。
  `/ws/map` は `decode(...) as AutoMapMsg` で**検査なし**。
  `AutoMap` は `MIRRORED` に含まれるので型ハッシュの保護はあるが、
  **map チャネルは自分の `schema` を載せない**（サーバも生 `AutoMap` を msgpack 化して送るだけ）。
- **影響**: telemetry 切れている状態（`/ws/map` だけ生きている）では無防備。
  Pi/GUI の版ズレで `cells` の長さがずれて**ビットマップがシフト**し、
  `graph_breaks` が off-by-N になり**道路グラフが壊れても警告が無い**。
- **修正案**: `/ws/map` の各フレームに `schema` を載せる（または接続時に 1 回版を送る）。
  GUI 側 `map.ts:149` で照合し不一致なら「古い型」と表示して描かない。
  `cells` 長と `graph_breaks` の単調性をアサートする。

#### GUI-05 [M] スキーマ hash が全-or-nothing で、1 フィールド追加で全機能が停止する
- **判定**: `CONFIRMED`
- **場所**: `raspi/msgs/schema.py:65-79` / `gui/src/ws/telemetry.ts:46-52` /
  `gui/src/components/StatusBar.tsx:146-155,200-203` / `gui/src/input/useDriving.ts:484-487`
- **内容**: `schema_version()` は `MIRRORED` の全型のフィールド名・型を **1 個の sha256** に畳み込む。
  `VehicleState` にフィールドを 1 個足すだけで hash が変わり、
  `gui/src/generated/msgs.ts` の `MSGS_SCHEMA` も変わり、**GUI の再ビルドが必要**。
  版不一致時は `telemetry.ts:46-52` で**全フレームを return（捨てる）** → `live` が更新されない。
  `StatusBar.tsx:200-203` は `const inhibited = n.link?.arm_inhibited ?? true` で
  テレメトリが来ない＝`link` が undefined＝`inhibited=true`＝**ARM ボタンが disabled**。
  ただし `useDriving.ts:484-487` の **Enter キーの経路は `link` を見ていない**。
- **影響**: fail-closed は正しいが、「再ビルドせよ」の具体策が StatusBar の 1 pill しか無い。
  1 フィールド追加で止まるのは **`/ws/telemetry` 1 チャネル**（ARM/DISARM 表示・
  信号/ライン/追跡のオーバーレイ・planner 判断）。
  > **注（第2ラウンド再検証で訂正）**: 原文の「地図・信号・カメラまで全部落ちる」は過大。
  > `/ws/map` と `/ws/camera` は schema を持たない**別チャネル**なので残る。
- **修正案**: schema を**型単位**に分け、版不一致の影響範囲を局所化する。
  「再ビルドが必要」の導線を StatusBar の版不一致表示に集約する。

#### GUI-06 [M] mcap 記録をブラウザ RAM に上限なしで蓄積する
- **判定**: `CONFIRMED`
- **場所**: `gui/src/hooks/useMcapDownload.ts:23,26-41,56-60` / `raspi/nodes/telemetry_node.py:399-404,2544-2573`
- **内容**: `const chunksRef = useRef<ArrayBuffer[]>([])` に受信 chunk を**無制限に push**
  （上限・自動停止・破棄のどこにも無い）。`finish()` は
  **既にメモリに溜めた chunk 全部から `new Blob(chunksRef.current, ...)` を作る** →
  Peak は「記録サイズ + Blob のコピー」で**2 倍**。OOM すれば `new Blob` / `a.click()` に
  到達せず**ダウンロードが始まらない**。前提の記述（`telemetry_node.py:399-403`
  「`image_hz=5` × カメラ2台で `.mcap` は毎時 1GB 近く育つ」）も確認。
  Pi 側には `MAX_INLINE_LOG_BYTES = 32MB` の上限があるが、**ブラウザの中継には無い**。
  録画自体にも時間上限が無い。
- **修正案**: 上限（例 512MB）と残量表示を足し、超えたら**自動停止して途中までダウンロード**する。
  `Blob` を作る前に `chunksRef` を 1 個の `Blob` へ逐次合成して Peak メモリを倍にしない。
  巨大記録は Pi 側に書き `GET /logs/<name>` で取得する運用へ誘導する。

#### GUI-07 [M] 自律走行の解除条件がパッドの微小入力で成立する
- **判定**: `PARTIAL`（**既定の Deadzone では成立しない**。Deadzone=0 設定時のみ危険）
- **場所**: `gui/src/input/useDriving.ts:371-376,707-713,727,891-899` / `gui/src/store/ui.ts:257,297`
- **内容**: `dz()` は `if (a < zone) return 0` で**Deadzone 未満を厳密に 0** にする
  （既定 `gamepadSteerDeadzone = 0.12`）。ただし
  `SETTINGS_RANGE.gamepadSteerDeadzone` の **min が 0** → Deadzone を 0 にすると
  `dz(v,0)=v` で**軸のノイズ 1 ツComm でも `padSteer !== 0`** になり得る。
  解除判定は `:891-899` `if (engaged && (keyActive || padActive || brake))` で
  **250 ms のレート制限のみ**（ヒステリシス無し）。`padStale` detected 時は `pad = null` → `padSteer = 0`。
- **修正案**: `gamepadSteerDeadzone` の min を 0.03〜0.05 に上げる。
  解除条件を「Deadzone 超えが N ms 続いた」にする。解除時に理由（パッド由来等）を明示する。

#### GUI-08 [L] `take_control` の 250 ms 連打と `notify()` の新 id 連打
- **判定**: `CONFIRMED`
- **場所**: `gui/src/input/useDriving.ts:356,930-937,987-993` / `gui/src/store/ui.ts:832-839` / `gui/src/App.tsx:139-140`
- **内容**: トークン誤り（`bad_token`）だとサーバが毎回拒否 → `onDenied` → `hasControl: false` →
  **250 ms ごとに無限再送**（4 Hz）。`notify()` は毎回新しい id を作り**タイマーを毎回リセット**する
  → 通知が消えずに張り付く。`deniedBy` / `deniedReason` はどこからも読まれていない。
- **修正案**: 連続 N 回（例 3 回）拒まれたら指数バックオフ（250ms→1s→4s）する。
  `bad_token` は恒久的失敗と見なし再試行を止める。
  `notify()` は同一テキストならタイマーをリセットしない。`deniedBy` を常時表示する。

#### GUI-09 [L] `PI_MAX_SPEED_CAP` / `PI_MAX_STEER_CAP` の扱い
- **判定**: `PARTIAL`
- **場所**: `gui/src/store/ui.ts:195-196,207-218,249,289,352-371,407-434` / `gui/src/generated/vehicle.ts`
- **内容**: `PI_MAX_SPEED_CAP = 3.0`（コメントは「install_services.sh と一致させてある」と逆向き依存）、
  `PI_MAX_STEER_CAP = VEHICLE.maxSteer`（= 0.47167。コメントの「同 `--max-steer`」は
  `install_services.sh` の 0.524 と**不一致**）。
  クランプ設計（CFG-04 の「FALSE」参照）は「LIMITS 未受信時は無クランプ」。
  > **注（第2ラウンド再検証で訂正）**: 原文の「maxSteer にも保存値の巻き戻りが残る」は
  > **実質ゼロ**。`PI_MAX_STEER_CAP` は `vehicle.toml` の `max_steer` から生成された
  > **物理上限そのもの（0.47167）**で、「仮値」ではない（`maxSpeed` 側の
  > `PI_MAX_SPEED_CAP = 3.0` は `LIMITS.max_speed_m_s = 5.0` より小さい仮値）。
  > したがって「`maxSteer` にも無クランプ豁免を適用する」という修正案は**不適切**で、
  > 実質的な指摘は ①`PI_MAX_SPEED_CAP` のコメントの逆向き依存
  > ②`PI_MAX_STEER_CAP` のコメントと `install_services.sh` の 0.524 の不一致 の 2 点のみ。
  > 深刻度 Medium → **Low**。
- **修正案**: `PI_MAX_SPEED_CAP` を「LIMITS 待機中用のフォールバック」だと明示し、
  `config/vehicle.toml` 由来へ寄せて生成する。
  `PI_MAX_STEER_CAP` のコメントを実値に合わせる。
  無クランプ豁免を `maxSteer` にも適用する。

#### GUI-10 [M] GUI 側にテストが 1 つも無い
- **判定**: `CONFIRMED`
- **場所**: `gui/package.json`（`scripts` は `dev` / `build` / `preview` / `typecheck` のみ）/ `gui/src/**`
- **内容**: `vitest` / `jest` / `playwright` は依存にも devDependencies にも無い。
  `find gui/src -name "*.test.*" -o -name "*.spec.*"` → **0 件**。
  一方 Python 側は `raspi/tests`（1667 件）・`tools/fwsim/tests`（45 件）にある。
- **影響**: SAF-02 / GUI-09 がいずれもテストで捕捉しうる類型。
- **修正案**: Vitest を入れ、純関数から始める（`store/ui.ts` の `clampSettings` / `effectiveRange`、
  `format.ts` のレベル判定、`components/diag/checks.ts` の `runChecks`、
  `bus/history.ts`）。`ws/control.ts` の `shallowMerge` / `sendCoalesced` は WebSocket をモックできる。
  Python↔TS の JSON フィールド照合は Python 側 1 本 + `gen_msgs.py --check` で代替してもよい。

#### GUI-11 [L] GUI のトルク上限の所在
- **判定**: `PARTIAL`（値は Python と一致・**テストあり**。前提と所在のみ誤り）
- **場所**: `gui/src/store/ui.ts:209,211,299` / `raspi/msgs/convert.py:69,79` / `config/vehicle.toml:115` / `gui/src/generated/vehicle.ts`
- **内容**: `MAX_BRAKE_TORQUE_NM = 0.15` / `MIN_BRAKE_TORQUE_NM = 0.005` / スライダ範囲 0.005〜0.15。
  Python 側の単一の定義は `convert.py:69` `MAX_BRAKE_TORQUE_NM = 0.15` / `:79` `MAX_TARGET_TORQUE_NM = 0.15`。
  → **値は一致**。ただし `config/vehicle.toml:115` は `speed_torque_max_nm = 0.30`（**2 輪分の合計**）で、
  「0.15 N·m」という値はこのファイルには無い（`test_fw_constants.py:134` が
  `assertAlmostEqual(toml["dynamics"]["speed_torque_max_nm"], 2 * limit)` と明言）。
  生成物 `gui/src/generated/vehicle.ts` に**トルク上限は無い**（`VEHICLE` は車両形状/カメラ、
  `SAFETY` はタイマ類のみ）。`test_fw_constants.py:126-141` は
  `gui/src/store/ui.ts` を**正規表現で突合**している（**この 2 定数は守られている**）。
- **修正案**: トルク上限を `vehicle.toml` から**生成**して GUI へ載せ、
  「手で合わせること」をやめる。

#### GUI-12 [L] `ServerMsg` に版フィールドが無い
- **判定**: `PARTIAL`（**`status` には版がある**）
- **場所**: `gui/src/ws/control.ts:38-50,120-134` / `gui/src/types.ts:306` / `gui/src/App.tsx:92-98`
- **内容**: `ServerMsg` に版フィールドは無い。**ただし `status` には `schema?: number` があり
  `App.tsx:92-98` で `s.schema !== MSGS_SCHEMA` を判定して `schemaMismatch` を立てる。**
  版が無いのは `logs` / `cam_models` / `e2e_models` / `cam_e2e_models` / `maps` /
  `maps_save_result` / `maps_clear_grip_result` / `logs_delete_result` / `shutdown_denied` /
  `control_denied` / `pong` で、これらは `as` キャストのみ（実行時検査なし）。
- **修正案**: CFG-16 の修正（`/ws/control` の全メッセージに `schema`）と共通。

#### GUI-13 [L] `has_grip` の表示
- **判定**: `CONFIRMED`（**表示は適切**。追加余地のみ）
- **場所**: `gui/src/components/MapLibrary.tsx:38,149-153` / `gui/src/types.ts:417` / `gui/src/App.tsx:159-165`
- **内容**: `has_grip` の参照は `types.ts:417`（型宣言）と `MapLibrary.tsx:38,150,152` のみ。
  学習値が無い地図ではボタンが `disabled` になる。`maps_clear_grip_result` はトースト通知のみ。
- **修正案**: 一覧に「学習値あり/なし」を明記し、消した直後は再取得で更新済みであることを示す。

#### GEN-22 [M] 未知トピックへの前方一致フォールバックが辞書の並び順に依存
- **判定**: `CONFIRMED`
- **場所**: `raspi/bus/zbus.py:96-101,102-135,162-181`
- **内容**:
  ```python
  owner = TOPIC_OWNER.get(topic)
  if owner is None:
      for pref, n in TOPIC_OWNER.items():   # ← 辞書の iteration order 依存
          if topic.startswith(pref): owner = n; break
  if owner is None:
      raise KeyError(f"publish 元が不明なトピック: {topic!r}")
  ```
  `zbus.py:96-101` のコメントに既知の事故が記録されている:
  「★ **完全一致で先に引けるキーを持たせること。** 前方一致フォールバックは辞書の並び順に依存する上、
  `"scan/cam"` が `"scan"` の前方一致に誤って拾われ `cam_perception` ではなく `io` に解決される、
  という事故が実際に起きた」。`TOPIC_OWNER` は `"scan"`（:104）が `"scan/cam"`（:121）より**前**。
  **実測**:
  ```
  scan/cam      -> cam_perception.ipc   （今は完全一致で正しい）
  scan          -> io
  scan/xyz      -> io        ← 未知トピックが「黙って io に届く」
  cmd2          -> control   ← 同上
  auto/cmdx     -> planning  ← 同上
  unknown/topic -> RAISE KeyError
  ```
  → 完全一致も前方一致も外れるものは raise される（黙って消えない）が、
  **既知プレフィックスの拡張は raise されず誤った owner へ静かに送られる**。
- **修正案**: 前方一致を「`/` 終端プレフィックスのみ」に制限し、それ以外は raise する
  （`"scan"` は前方一致候補から外す）。
  `endpoints_for_topic` に対する表検証テストで `TOPIC_OWNER` のコメント手順を恒久化する。

#### GEN-23 [L] DISARM モードでも ARM ビットがワイヤに載る
- **判定**: `PARTIAL`（コード事実は CONFIRMED。**High → Low**。firmware が mode で armed をゲートしている）
- **場所**: `raspi/msgs/convert.py:285-287,319` / `raspi/nodes/telemetry_node.py:1186-1187,1489-1495,3205` /
  `raspi/nodes/io_node.py:707-717,914,916` / ファーム `src/vehicle/vehicle.c:268-294,329-332`
- **内容**: `convert.py:285-287` は `flags = 0` / `if cmd.arm: flags |= packets.CMD_FLG_ARM` で
  **`cmd.mode` を見ていない**（`:319` の `mode = cmd.mode if ... else 0` は別枠）。
  `telemetry_node.py:1489-1495` は `mode=0` かつ `arm=true` を**素通しで受理**する
  （拒否する理由が無い）。
  **実測**: `DriveCmd(mode=0, arm=True)` → `mode=0 flags=0x01`（ARM bit=True）
  だが `io_node` 内部の armed 判定は `False`。
  `io_node.py:914` `armed = bool(pkt.flags & CMD_FLG_ARM) and pkt.mode != Mode.DISARM` で
  **非 ARM と解釈**する（これは記録用。`pkt` はそのまま `:916 return pkt` で**ワイヤに ARM ビットつきで出る**）。
  `io_node.py:707-717` の `_disarm_command()` は `mode=DISARM, flags=0` で**正しい**（実行確認）。
  ファーム（実在）`src/vehicle/vehicle.c:268-294`:
  ```c
  bool arm_requested = (command->flags & RAS_CMD_FLAG_ARM) != 0;
  bool armed = arm_requested && center_valid && obj->md_started &&
               (obj->mode == RAS_MODE_MANUAL || obj->mode == RAS_MODE_AUTO);
  ```
  → **mode=DISARM なら `armed=false`** なので速度指令は効かない。
  ただし `Power_SetDrivePower(obj->power, (arm_requested && center_valid) || ...)` は
  **ARM ビットだけで駆動電源を入れる** ＝「DISARM でもモータ系に電源が入る」状態が成立する。
- **影響**: 「2 つの DISARM 経路が食い違う」は CONFIRMED だが、
  **走行に直結する危険は firmware の mode ゲートで封じられている**。
  残る実害は「DISARM なのにモータ系電源が入る」だけ。
- **修正案**: `convert.py:285-287` を `if cmd.arm and cmd.mode != packets.Mode.DISARM:` に変更する。
  補助として `telemetry_node.py:1489-1495` でも `mode == 0 and arm` を弾くか、
  ARM ビットだけ落として通す（GUI のバグで走行が切れない方針 `:1146-1150` に合わせる）。
  firmware 側は `Power_SetDrivePower` を `armed` でなく `arm_requested` で開く意図を再確認する価値がある。

#### GEN-24 [L] 「スキャン時刻」の定義が 2 つある
- **判定**: `PARTIAL`（2 定義の主張は CONFIRMED。**deskew 2 モジュール間の差と `deskew.py:121` の行番号は FALSE**）
- **場所**: `raspi/msgs/types.py:273-275` / `raspi/nav/deskew.py:148` / `slam2d/core/deskew.py:78` /
  `slam2d/core/frontend.py:275-281` / `raspi/nodes/planning_node.py:366,428`
- **内容**: `types.py:273-275`「`t_capture` はこの1周で**最初に取得したセクタ**の時刻」
  （`raspi/tools/slam_replay.py:39` も同じ記述）。
  `raspi/nav/deskew.py:148` は `t_ref = int(t_pt.max()) ... else int(scan.t_capture)` で
  **末尾点（最後の点）**を基準時刻にする。→ **同じ関数内で基準時刻の定義が 2 つ**になっている
  （同一関数内で最大 ~100 ms の差）。
  `slam2d/core/deskew.py:78` も `t_ref = int(t_pt.max()) ... else 0` で**同一の規約**（末尾点）であり、
  「2 つの deskew で t_ref の定義が異なる」は **FALSE**（差異はフォールバックのみ）。
  参照時刻は `slam2d/core/frontend.py:275-281`（`pts.t_ref_ns` = 末尾点）と
  `raspi/nodes/planning_node.py:366,428`（鮮度判定は `scan.t_capture` = 先頭セクタ）で
  **100 ms 程度ずれた 2 系統**が存在する。
- **修正案**: `deskew.py:148` のフォールバックで `scan.t_capture` を使うなら
  「先頭セクタ時刻」であることをコメントで明示するか、
  `t_ref` を「末尾点時刻」に統一して `scan.t_capture` を使わない（`slam2d` 側と一致させる）。
  `types.py:273-275` の docstring に「点群の基準時刻（deskew の `t_ref`）は本時刻ではなく**末尾点**」を追記。

---

### 3.8 運用・文書・ログ

#### OPS-01 [M] ログ保持ポリシーが 2 つの実装で食い違い、文書と乖離している
- **判定**: `PARTIAL`（**二重実装は CONFIRMED**。数値と文書箇所の指定に誤り）
- **場所**: `raspi/setup/install_services.sh:134,331-354,371` / `raspi/rec/logclean.py:46-62,67` /
  `raspi/nodes/io_node.py:1171-1173` / `raspi/nodes/logger_node.py:227` / `docs/architecture.md:766,770` /
  `docs/system_overview.md:338` / `docs/development.md:734` / `gui/src/bus/history.ts:36`
- **内容**: timer 側は `MAX_LOG_MB=8000`（`:331`）で `find -mtime +7 -delete`（`.sfl`/`.mcap`/`.mcap.bak` のみ）と
  合計 8 GB 超のときの古い順削除（`-mmin +1` 保護あり）、`OnCalendar=hourly`。
  in-process 側 `logclean.py` は `LOG_PATTERNS = ("*.sfl","*.mcap","*.mcap.bak","*.mp4")`、
  閾値 **15 % / 5 % の空き率**（合計サイズではない）、`DEFAULT_KEEP_NEWEST = 3`。
  → **候補集合も閾値も違う**。8 GB 側の方が絶対的に先に効く（28 GB SD で 5 % = 1.4 GB 空き）。
  レート再計算（`protocol.toml` の `len` + `overhead=7` と `framelog.py:24-33` のヘッダ 14 B）:
  数値のみ `.sfl` = TELEMETRY 100 Hz×116 B + LIDAR_SECTOR 120 Hz×83 B + COMMAND 100 Hz×29 B
  ≈ **24.6 kB/s ≈ 1.5 MB/分**。
  > **注（第2ラウンド再検証で訂正）**: この計算値 1.5 MB/分から導くと
  > **2.1 GB/日 → 8 GB 上限で約 3.8 日**。docs の実測値 2.5 MB/分から導けば
  > **3.6 GB/日 → 約 2.2 日**。元の記述は 1 本の鎖に 2 つの速度を混在させていた。
  > ここでは **docs 実測 2.5 MB/分（= 3.6 GB/日、約 2.2 日）** を採用する。`.mcap` は docs 実測 10.3 MB/分（`image_hz=5`）、
  GUI 経路の既定 `image_hz=10`（`telemetry_node.py:388`）なら画像が 2 倍で
  **約 19.6 MB/分 ≈ 28 GB/日** → 8 GB 到達は約 **7 時間**。
  さらに **in-process は `*.mp4` を削除するが timer は削除しない**
  （`.mp4` は `telemetry_node.py:2478,2496` で Pi の `logs/` に書かれ 8 GB 予算を消費する）。
  **timer に `keep_newest` ルールが無い**（`logclean.py:62` にはある）。
  サーバ側に「直近 N 分」のバッファは**無い**（リングバッファは GUI ブラウザ内 180 秒だけ）。
- **影響**: **事件のフォレンジック窓は「日」ではなく「数時間〜2 日」**である。
  なお `docs/architecture.md:770` の記載は「1 日 15 GB」であり
  「7 日」の記述は `docs/system_overview.md:338` と `docs/development.md:734` にある
  （原指摘の行指定は誤り）。
- **修正案**: 保持を**時間ベース**（`.mcap` 12 時間、`.sfl` 3 日）で明示する。
  2 つの cleaner を**候補集合と閾値を統一**する。timer に `keep_newest` を入れ `*.mp4` も対象にする。
  GUI に「保持中の最古ログの経過時間」を出す。

#### OPS-02 [M] `AsyncFrameLog` のオーバーフローがログに刻まれない
- **判定**: `CONFIRMED`
- **場所**: `raspi/rec/framelog.py:325-332,383-392` / `raspi/nodes/io_node.py:1449` / `raspi/tests/test_framelog.py:477-486`
- **内容**: `_push()` は `if len(self._q) >= self._queue_max: self._q.popleft(); self.dropped += 1` で
  **最古レコードを黙って捨て、EVENT も書かない**。`close` イベントの中身は
  `rx/tx/events/duration_s` のみで **`dropped` は入らない**。
  実測: `AsyncFrameLog(queue_max=10)` に 100 件 → `dropped == 90`。
  `dropped` は `io_node.py:1449` で終了時に stdout へ出るだけ。
- **影響**: **欠落を表す EVENT が書かれず**、`.sfl` ファイル自体には欠落の痕跡が残らない
  → バックプレッシャー由来の穴と本物の UART 途絶が区別できない。ログの存在意義そのものの問題。
- **修正案**: オーバーフロー時に `{"name":"dropped","n":…}` の EVENT を書き、
  ヘッダに累積書き込み数を持たせる。`close` イベントにも `dropped` を入れる。

#### OPS-03 [M] 文書・数値の陳腐化
- **判定**: `CONFIRMED`（8 項目すべて実測で裏付け。該当行の一部は指定違い）
- **場所**: `docs/architecture.md:770` / `docs/system_overview.md:338` / `docs/development.md:734` /
  `docs/calibration.md:87,26,148,239-240` / `docs/control_constants.md:34,38,41,45,52,54,79,85,96,131,153` /
  `docs/stm32_interface.md:211,430,651,1165` / `docs/uart_protocol.md:1111` /
  `docs/development.md:76,857` / `PROGRESS.md:12`
- **内容**: 実測表:
  | 文書 | 記載 | 実態 |
  |---|---|---|
  | `control_constants.md:41` | `speed_loop_tau_s = 0.155` | **0.06871807** |
  | `control_constants.md:34` | `VEHICLE_BRAKE_DECEL_M_S2 = 2.47` | **2.165**（2.4744 は `[sim]` の値） |
  | `control_constants.md:38` | 3.04 | **3.4642** |
  | `control_constants.md:45` | `launch_exit_torque_nm = 0.006346` | **0.011629** |
  | `control_constants.md:52,54,96` | `tools/sysid`・`speed_pi_zeta`・極配置 | 削除済み・**`[dynamics]` に存在しないキー** |
  | `calibration.md:87` | 0.35 m/s | **0.45**（`calib_steer.py:52`） |
  | `calibration.md:26` | 1.2 m 四方 | **約 0.8 m × 0.8 m** |
  | `calibration.md:148` | 0.13 N·m | **0.15** |
  | `calibration.md:239-240` | ±0.05 | **±0.04** |
  | `stm32_interface.md:211,651` | `TELEMETRY LEN = 84` | **102**（`packets.py:194`・`uart_protocol.md:301`）— **同文書内でも矛盾** |
  | `stm32_interface.md:430` / `uart_protocol.md:1111` | `0x001A` | **`0x001D`** |
  | `PROGRESS.md:12` | v0.23 | **v0.29** |
  なお `control_constants.md:79,85` の「firmware は写しを持たない」は v0.29 移行で**正しい**。
- **修正案**: 数値を正へ更新。削除済みツール参照と非-existent キー参照を削除。
  `docs/control_constants.md` §2（位相余裕の式）と §3（経験的ラダー）の矛盾を解消
  （`docs/calibration.md:178-181` は「ゲインは校正 8 の実機閉ループで決める」と書いている）。
  `docs/development.md:76,857` は「`config/gen_msgs.py` が生成し `types.ts` は再エクスポート」と訂正。

#### OPS-04 [L] `site/` と `PROGRESS.md` の版が 6 版古い
- **判定**: `CONFIRMED`
- **場所**: `site/docs/link/protocol.md:5,129` / `site/docs/about.md:59` / `PROGRESS.md:3,6,12`
- **内容**: `site/docs/link/protocol.md:5`「確認した版: プロトコル **v0.23**（`protocol_version = 0x0017`）」、
  `site/docs/about.md:59`「コードは `0x0017`（v0.23）」。正は **v0.29（`0x001D`）**。
  `PROGRESS.md:12` も v0.23。`PROGRESS.md:3` は「新しいセッションではまずこれを読む」、
  `:6` の見出しは「現在の状態（2026-10-17 時点）」で**古い**。
- **修正案**: 版数を `protocol.toml` から**自動記入**する。
  `config/check_docs.py` の `CLAIMS` に `site/docs/link/protocol.md` を追加して CI で固定。

#### OPS-05 [M] `sim/` と `tools/fwsim/` が STM32 の二重実装
- **判定**: `CONFIRMED`
- **場所**: `sim/stm32.py:10,64,70,262-266,615-620,726-731` / `sim/vehicle.py:806-813,856-861` /
  `tools/fwsim/plant.py:49-56,83,93` / `tools/fwsim/fw.py` / `sim/bench.py:73,76`
- **内容**: 比較表（全行を実コードで確認）:
  | 項目 | `sim/`（Python 再実装） | `tools/fwsim/`（実ファームをコンパイル） |
  |---|---|---|
  | タイヤ | 独自モデル（`axle_model` / `rear_grip_ratio`） | `Plant(mu, tyre_b, tyre_c, load_transfer)` を firmware の `host_sim.c` へ |
  | TC・ABS | **無し**（「TC/TV は実装していない」）、ABS は作り物の旗だけ | firmware の `drive.c` / `torque_vectoring.c` を実コンパイル |
  | MD トルク遅延 | **プレースホルダ**（`BRAKE_DECEL_PLACEHOLDER_M_S2 = 0.5` / `MD_TORQUE_LAG_PLACEHOLDER_S = 0.0`） | firmware の実 MD モデル |
  | エンコーダ量子化 | **無し**（理想化） | `encoder_cal_params()` で firmware の `lib/encoder_cal` を通して校正 |
  | `CONFIG_ACK` の範囲判定 | **無い**（範囲外も OK を返す） | firmware がクランプし `RAS_CONFIG_OUT_OF_RANGE` |
  `sim/` は `tools/fwsim` を**一切 import していない**（`grep -rn fwsim sim/*.py` はコメント 2 行のみ）。
  `sim/bench.py` は `sim/stm32.py`（`VirtualStm32`）と `sim/vehicle.py` を使う＝**粗い双子**。
- **影響**: Pi 側の `mismatch` → ARM ブロック経路が**シムでは再現しない**（GEN-21）。
  制御ゲインの調整基準が 2 つある。
- **修正案**: ① `sim/stm32.py` の CONFIG_ACK に firmware と同じ範囲判定・クランプを入れる
  ② 「シムは STM32 の振る舞いの近似、制御則の正は `tools/fwsim`」という境界を明記し、
  制御ゲインの調整は `tools/fwsim` を通す運用にする
  ③ `sim/stm32.py` の MD 遅延プレースホルダを `[control.plant]` から取る

#### OPS-06 [L] `PROGRESS.md` に到達不能な機能の記載がある
- **判定**: `PARTIAL`（**GUI 側は反映済み**。シム側は到達不能）
- **場所**: `PROGRESS.md:37-40,50-56` / `raspi/auto/slam2d_raceline.py:683-685` / `raspi/auto/registry.py:83` / `gui/src/components/ParamSliders.tsx:72,82`
- **内容**: 「GUI の『限界のモデル』（`grip_shape`）を選択肢で出す」は**反映済み**
  （`ParamSpec` の `choices` → `registry.py:83` が JSON 化 → `ParamSliders.tsx` が選択肢を出す）。
  一方「シムの車に `axle_model`・`rear_load_frac`・`load_transfer_hl`・`rear_grip_ratio` を足した」は
  実在するが、**`VehicleSpec.load()` に `axle_model` を設定する行が無い**ため到達不能（CFG-12）。
- **修正案**: `sim/vehicle.py` の `VehicleSpec.load()` に `axle_model` 等を `[control.plant]` から読む行を追加。
  `PROGRESS.md:50` に「GUI からはまだ切り替えられない」旨を追記。

#### OPS-07 — 取り下げ（校正の `self.room` が 1 周期古い）
詳述 §5。`FALSE`。

#### OPS-08 [L] `logger_node` のディスク容量チェックがメインループで同期実行
- **判定**: `PARTIAL`（コード事実は CONFIRMED。深刻度 Medium → **Low**）
- **場所**: `raspi/nodes/logger_node.py:99,225-227,242,268-283` / `raspi/nodes/io_node.py:553-580,1167-1170` /
  `raspi/rec/logclean.py:46-62,150-176` / `raspi/bus/zbus.py:241`
- **内容**: `logger_node.py:225-227` はメインループ内で `self._check_disk_space()` を直接呼び、
  `:242` で `logclean.check_disk(log_dir, protect={self.log.path})` を**同期実行**する
  （間隔は `:99` `DISK_CHECK_INTERVAL_S = 30.0`）。
  対比 `io_node.py:553-580` は同じ処理を `threading.Thread(target=_worker, name="io-disk-check")` へ渡し、
  メインループは `_poll_disk_check()` で完了を拾うだけ。
  `io_node.py:559-566` の docstring は
  「**issue #5: `.sfl`/`.mcap` 同期 unlink は巨大ファイルで数十〜数百ms かかりうる**」と明記。
  実際の `logclean.check_disk` は 1 回で最大 50 本を `p.unlink()` する（`logclean.py:156`）。
  `logger_node` は**同一関数を同期で呼ぶだけ**で io_node のオフロードを享受できない。
- **影響**: 「記録が欠ける」は誇張。RELIABLE 購読（`zbus.py:241` `hwm=10_000`）は
  ブロック中もメッセージがキューに溜まり、**RCVHWM を超えなければ欠落しない**。
  logger_node は購読専用で COMMAND/heartbeat を送らないため、io_node のような
  「E-Stop ハートビート誤ラッチ」も起こらない。
  ただし**方向性は妥当**: 空き < 5 % で削除が走る＝まさに SD が満杯寸前で、
  **その間 `.mcap` への書き込みが一切進まない**
  （ENOSPC 到達なら `McapLog` が `broken` になり記録が失われる。
  `_reopen_if_broken`（`:268-283`）は check_disk **後**に走る）。
  → 「容量が逼迫している瞬間だけ、記録が止まる瞬間でもある」。
- **修正案**: `logger_node._check_disk_space` を io_node と同型にする
  （`check_disk` を daemon スレッドへ渡し、メインスレッドでは完了判定だけ拾う）。
  `FrameLogWriter` / `McapLog` は単一スレッド専用なので、
  **結果処理（`_reopen_if_broken`・stderr 出力）はメインスレッドに留める**。

---

### 3.9 状態管理・並行・その他

#### GEN-01 [H] `replay_node --bus` の時刻ドメイン不一致で planner が恒久「点群が古い」判定
- **判定**: `CONFIRMED`
- **場所**: `raspi/nodes/replay_node.py:331` / `raspi/bus/zbus.py:313-314` /
  `raspi/core/bus_bridge.py:76-98,105` / `raspi/nodes/planning_node.py:427-429`
- **内容**: `BusBridge(pub, clock=lambda: holder["node"]._cursor_ns)` は**記録元 Pi の CLOCK_MONOTONIC** を使う。
  一方 `zbus.py:313-314` の `msg.t_pub = time.monotonic_ns()` は**再生プロセス自身**の単調時刻
  （Publisher に clock 引数が無く上書きする手段が無い）。
  `planning_node.py:427-429` は
  `age = now_ns - scan.t_pub`（再生プロセス領域、正常）と
  `cap_age = now_ns - scan.t_capture`（**再生プロセス領域 − 記録元領域**）を
  **別ドメインから取った値どうしで比較**する。
  **再現**:
  ```
  再生プロセスの monotonic_ns() = 350,438,253,115,666
  記録元の t_capture          = 12,345,678,901,234
  Publisher が押した t_pub    = 350,438,253,078,625
  planning_node の cap_age = 338092574.2 ms
  planning_node の age      = 0.0 ms
  ```
  → **cap_age が約 94 時間** → `stale_ms=300` を hugely 超過 → **planner は常に `ready=False`（制動）**。
  逆向き（記録元機械の uptime が再生機より大きい）なら cap_age が負になり判定を素通りする。
  → **同じログが機械と時刻で挙動が変わる。**
- **影響**: `replay_node.py:9-12` は「これが事実上のシミュレータ代わり」と記すが、
  **planning 経路では成立していない**。
- **修正案**: `Publisher` に `clock` を差し込めるようにする（`Publisher(node, clock=...)`）か、
  送出後に `msg.t_pub` をログ領域で上書きできる API を追加する。
  併せて鮮度判定を `now - scan.t_pub` と `scan.t_pub - scan.t_capture` の 2 段にする。

#### GEN-02 [M] `park_to_point` の `plan_budget_ms`（最大 150 ms）が `auto_cmd_stale_ms`（200 ms）に接近する
- **判定**: `PARTIAL`（**構造的事実は確認。ただし「恒久 stale」は未立証**。
  第2ラウンド再検証で §2 索引との判定不整合が指摘され、本文も `PARTIAL` に統一）
- **場所**: `raspi/auto/park_to_point.py:60-63,260-266,419,429,431-461` /
  `config/vehicle.toml:63` / `raspi/nodes/telemetry_node.py:360,3224` / `raspi/nodes/planning_node.py:576-581`
- **内容**: `plan_budget_ms` は `min=20, max=150, default=120`。`auto_cmd_stale_ms = 200`。
  `planning_node.py:576-581` は `plan()` 中は `auto/cmd` を publish できない。
  `park_to_point.py:60-63` に**自認あり**。
  フォールバック `_replan_fallback`（`:431-461`）は
  `for k in (1.0, 1.5, 2.5): for cand in candidate_paths(...)` で
  `sample_path_array` + `path_clearance` を評価するが、**時間予算の判定が無い**。
  Hybrid A* の 150 ms を使い切った**後**にこの時間が上乗せされる。
  回数: `reeds_shepp.py:306-337` の `_candidates` は 9 ジェネレータ × 対称 4 通り、
  `candidate_paths` は正向き＋逆向きで**上限 72 本**（実測 19〜42、乱数 900 ケース）→
  評価は最大 **3×72 = 216**（原指摘の 144 より多い）。
- **影響**: 予算切れ時のフォールバック分は計画予算の外で上乗せされるが、
  **実測は 108 候補・8.3 ms（合計 160 ms < 200 ms）**。
  したがって「毎サイクル 200 ms 超のブロック → 駐車モードが一度も動けない」の
  **恒久 stale は立証されていない**（本文の付録B.2 の実測が本文の主張を反証している）。
  深刻度 High → **Medium**。
- **修正案**: `_replan_fallback()` に**残りの時間予算**を渡す。
  `plan_budget_ms` の max を `auto_cmd_stale_ms/2`（100 ms）に下げ、
  フォールバック分を別途 `fallback_budget_ms`（例 20 ms）で制限する。

#### GEN-03 [M] `planning_node` の無防備コールバック 3 つに例外ガードが無い
- **判定**: `CONFIRMED`
- **場所**: `raspi/nodes/planning_node.py:352-354,519-521,562-564,181-188,309-313,356-364,395-399`
- **内容**: `on_vehicle_state`（`:562-564`）、`set_engaged`（`:352-354`）、
  `snapshot`（`:519-521`）に **try/except が無い**。
  一方 `plan()`（`:356-364`）・`tick()`（`:395-399`）・`request_*`（`:232-267` 全て `_ask()` 経由）・
  `_apply_route_select`（`:309-313`）はガード済み → **3 つだけ漏れている**。
- **影響**: planner のバグ 1 個でノードごと死ぬ。`on_vehicle_state` は 100 Hz で呼ばれる。
- **修正案**: 3 箇所とも `_ask()` 相当のガードで包む。`snapshot()` は表示専用なので
  `try/except Exception` で `None` を返して継続する。

#### GEN-04 [M] モード切替時に `_last_plan_ns` がリセットされない
- **判定**: `CONFIRMED`
- **場所**: `raspi/nodes/planning_node.py:161,268-281,338,342`
- **内容**: モード切替 reset 処理（`:268-281`）は `_last_scan_seq` / `_last_map_seq` のみをリセットし、
  **`_last_plan_ns` はリセットされない**。`_replan` の `dt = (now_ns - self._last_plan_ns) / NS` は
  切替直後に**前モードを走っていた時間**（分単位で大きい）になり、planner に渡る。
  `base.py:130-139` では「平滑化に使う」と契約。
- **影響**: SAF-11 の 1 コマンド目の飛ばしの直接原因。`sysid_*` の時間ステップ系 planner は
  `dt` でステップを進めるため、**大きな dt だけ勝手に進む**。
- **修正案**: モード切替ブランチに `self._last_plan_ns = 0` を追加。
  `_replan` 側でも `dt` を `min(dt, 0.5)` 等に clamp すると二重で安全。

#### GEN-05 [M] `cmd` の発行元は 1 つだが、`plan()` に timeout が無い
- **判定**: `CONFIRMED`
- **場所**: `raspi/nodes/planning_node.py:100,355-357,581,165,377,379,767` / `raspi/nodes/telemetry_node.py:3205,3542`
- **内容**: `grep -rn "TOPIC_CMD" raspi/` で publish 元は `telemetry_node` の 2 箇所のみ
  （`io_node` / `logger_node` は購読側）。**「`cmd` の発行元は 1 つ」の規約は守られている。**
  一方 `planning_node.py:355-357` の `self.planner.plan(...)` に **timeout が無い**。
  `VEHICLE_STATE_POLICY = Policy(conflate=False, hwm=1000)`（`:100`）だけ、
  `scan` 等は `LATEST`（`:623-624`）。`plan_ms_max`（`:165,377,379`）は**記録されるだけで
  誰も参照されない**（参照は `types.py:768` と `AutoPanel.tsx:396` の表示のみ）。
- **影響**: SLAM planner が 3 秒以上かかればキューは溢れ、`vehicle_state` のサンプルが黙って落ちる。
- **修正案**: `planning_node._replan` に時間予算を導入（超過したら `ready=False` の
  `AutoState` に落として planner を信用しない）。`plan_ms_max` を自己保護に使う
  （例 `> auto_cmd_stale_ms*0.8` で planner を OFF して reason に出す）。

#### GEN-06 [M] バスは「drop してよい」前提で欠落検出が無い
- **判定**: `CONFIRMED`
- **場所**: `raspi/bus/zbus.py:236-241,274-277,302,313-321`
- **内容**: `_send()` は
  ```python
  try:
      self.sock.send(encode(topic, msg), zmq.NOBLOCK)
      self.sent += 1
  except zmq.Again:
      pass          # HWM 到達。制御ループを止めないため黙って捨てる
  ```
  → **HWM 到達で `zmq.Again` を握り潰して捨てる**（`self.sent` も進まない＝**欠落を数えない**）。
  `LATEST = Policy(conflate=True)` は古いものを**上書き**で捨てる。
- **影響**: 遅い購読者は「動いているように見えるが古い／欠けた」データだけを見る。
  `cmd` だけは drop 不可なので注意が必要。
- **修正案**: `zmq.Again` ハンドラで `self.dropped += 1` して `stats()` に出す。
  購読側の seq 欠番検出を実装し `LinkDiag` に載せる。`cmd` は `hwm` を大きく／`RELIABLE` にする。

#### GEN-07 [M] `Publisher.seq` は再起動で巻き戻り、各ノードが独自判定する
- **判定**: `PARTIAL`（巻き戻りは CONFIRMED。`telemetry_node.py:3115-3119` の行指定は**的外れ**）
- **場所**: `raspi/bus/zbus.py:302,316-317` / `raspi/nodes/cam_track_node.py:262-281` /
  `raspi/nodes/planning_node.py:338` / `raspi/nodes/telemetry_node.py:3115-3119`
- **内容**: `Publisher._seq` はプロセス再起動で 1 に戻る。
  `cam_track_node.py:262-281` の `_sync_roi_seq` は「値が小さくなった（telemetry_node の再起動で 0 から
  数え直した）: 覚え直すだけ」と**明示的に巻き戻りを処理**している（実在）。
  `planning_node.py:338` は等値比較のみで巻き戻り判定は無い（でも seq が巻き戻ると
  「同じ周と誤認」する可能性は薄い）。
  `telemetry_node.py:3115-319` は「変更検出用の版タプル」であって**巻き戻り＝再起動の判定ではない**。
- **修正案**: `Publisher` の seq に**ブート ID**（64bit）を添え `msg` に `boot_id` を追加。
  購読側は「boot_id が変わったら巻き戻りとみなす」という**共通**判定に統一する
  （`cam_track_node` の独自実装を共通ヘルパーへ移す）。

#### GEN-08 [M] `slam2d_raceline` の executor がノード終了時に残る
- **判定**: `PARTIAL`（`atexit` があるためプロセス終了では残らない。**設計通り**。`close()` 未呼び出しは CONFIRMED）
- **場所**: `raspi/auto/slam2d_raceline.py:413-417,427-436,2933-2939` / `raspi/nodes/planning_node.py:597-605,268-271`
- **内容**: `atexit.register(_shutdown_worker)` によりプロセス終了時に
  `shutdown(wait=False, cancel_futures=True)` される。`close()` は
  「**プロセスは使い回すので止めない**」と docstring に明記（**設計通り**）。
  一方 `planning_node.py:597-605` の `close()` は `pub`/`sub` を閉じるだけで
  **`planner.close()` を呼ばない**（モード切替時には `:268-271` で呼ばれる）。
- **影響**: ノード終了時に **`_grip_save()` が走らない**（学習値が保存されないまま終了し得る）。
- **修正案**: `PlanningNode.close()` で `planner.close()` も呼ぶ。
  systemd `KillMode=control-group` 等では atexit が走らないことがあるので、
  `close()` で `_shutdown_worker()` を明示的に呼ぶ選択肢を検討。

#### GEN-09 [M] `_cmd_from` が共有オブジェクト `self.state` を破壊的に書き換える
- **判定**: `CONFIRMED`
- **場所**: `raspi/nodes/planning_node.py:415,458-487,452-453,581,584`
- **内容**: `_current()` は**同一オブジェクト** `self.state` を返す。`_cmd_from(st)` が
  `st.ready = False` / `st.reason = ...` / `st.target_steer = self._last_steer` を
  **破壊的に変更**する。
  `st.reason` の連結には 2 系統ある（**第2ラウンド再検証で機構を訂正**）:
  - `:452-453` の `f"★{self._ctrl_error}（{st.reason}）"` は **`not in` ガード**があり、
    `_ctrl_error` の接頭辞は**1 回しか付かない**。
  - 毎周期連結されるのは `:480` の
    `st.reason = f"{st.reason}（出力がNaN/Inf）" if st.reason else "出力がNaN/Inf"`。
    こちらはガードが無く、**planner が非有限値を出力している間だけ**
    `self.state.reason` が 50 Hz ごとに 8 文字ずつ伸び続ける。
- **影響**: 同じ `self.state` は `auto/cmd` 用（`:581`）と `auto/state` 用（`:584`）に使われる。
  `auto/state` には「実際の判断（ready=False）」ではなく「制動に読み替えた後」の値が載る。
- **修正案**: `_cmd_from` の中で `self.state` を書き換えず、
  `msgspec.structs.replace(self.state)` のコピーを使う。`st.reason` の連結は
  ローカル変数で行う。

#### GEN-10 [M] `on_disengage` を持つ planner は解除時に `reset()` が呼ばれない
- **判定**: `CONFIRMED`（コード事実は正しいが**設計意図**につき実害なし）
- **場所**: `raspi/auto/base.py:115-128` / `raspi/nodes/planning_node.py:287-293` / `raspi/auto/slam2d_raceline.py:943-957`
- **内容**: `base.py:115-128` の docstring に**明示的に**
  「`on_disengage()` を持つ planner では、**解除だけのとき**（モードは同じ）は `reset()` の代わりに
  そちらが呼ばれる」と書いてある。`planning_node.py:287-293` の実装もそれに従う。
  `slam2d_raceline.py:943` の `on_disengage` は状態（地図）を自分で捨てる 意図を説明している。
- **修正案**: 不要。ただし `base.py:115-128` に「`on_disengage` を持つ planner は
  `reset()` 相当の状態を自分で捨てる責任がある」旨を明文化し、
  舵の平滑化状態（`self._steer` 等）が必ず消えているか 1 点だけ確認する。

#### GEN-11 [L] `merged()` は inf が `s.max` に張り付く
- **判定**: `CONFIRMED`
- **場所**: `raspi/auto/base.py:199-225`
- **内容**: `if v != v: continue`（NaN は弾かれる）→
  `out[s.key] = min(s.max, max(s.min, v))` で **inf は `s.max` に張り付く**。
- **影響**: inf が「意図した最大値」ではなく「クランプ幅の端」に化ける
  （パラメータの max を 0 にすると inf → 0 で「安全」に見える、等）。
- **修正案**: `math.isfinite(v)` をチェックし、inf は範囲外として既定値を維持する。

#### GEN-12 [L] `registry.catalog()` に `sysid` 系も含まれ、`make_planner` に排他が無い
- **判定**: `CONFIRMED`
- **場所**: `raspi/auto/registry.py:72-93,96-103` / `raspi/auto/base.py:88-92,108` / `raspi/auto/slam2d_raceline.py:471` / `raspi/auto/slam2d_route.py:304`
- **内容**: `catalog()` のフィルタは `cls.listed` だけ。`calib_*.py` 系（`category = "sysid"`）は
  `listed` を上書きしていない → **True**。したがって sysid planner も `catalog()` に含まれる
  （GUI 側の `category` フィルタに依存）。`make_planner(mode)` は排他制御を持たない
  （排他は「planning_node が 1 インスタンスしか持たない」ことに依存）。
- **修正案**: `catalog()` のフィルタを `if cls.listed and cls.category != "sysid"` にする
  （GUI 側のフィルタに依存しない）。`make_planner` に排他申告を持たせ、
  `planning_node` 側で排他確認して警告する。

#### GEN-13 [L] `follow_object` の注視点距離 `ld` に上限が無い
- **判定**: `CONFIRMED`
- **場所**: `raspi/auto/follow_object.py:211`
- **内容**: `ld = p["look_k"] * v_now + p["look_min"]` に上限クランプが無い。
  直後の `steer_for_target(...)` は舵角を `max_steer` にクランプするため致命的ではないが、
  `ld` が大きくなると注視点が遠ざかり**曲がる量が小さくなる**（＝障害物回避が遅れる）。
- **修正案**: `ld = min(p["look_k"] * v_now + p["look_min"], p["look_max"])` とする
  （`look_max` を ParamSpec で追加、既定 3〜5 m）。

#### GEN-14 [L] カメラ E2E に推論の信頼度しきい値が存在しない
- **判定**: `CONFIRMED`
- **場所**: `raspi/msgs/types.py:967-983,1063-` / `raspi/auto/cam_e2e.py:92-94` / `raspi/auto/cam_centerline.py:105-107`
- **内容**: `CamE2ECmd` のフィールドは `ready` / `steer_norm` / `speed_norm` /
  `model_max_steer` / `model_speed_ref` のみで **confidence が無い**。
  `cam_e2e.py:92-94` のチェックは `cmd.ready` → `isfinite` → `model_max_steer > 0` のみで
  **推論の確からしさは一切見ていない**。`CamPath` も `seen` / `xs` / `ys` / `widths` / `reason` のみ。
- **影響**: 低照度・ぼけ・部分マスクのフレームが**最大舵で採用**される。
- **修正案**: `CamE2ECmd` に `confidence: float = 0.0` を追加し、
  `cam_e2e.py:plan()` で `cmd.confidence < p["min_confidence"]` なら `ready=False`（制動）。

#### GEN-15 [L] `CamE2ECmd` に `reason` フィールドが無い
- **判定**: `CONFIRMED`
- **場所**: `raspi/msgs/types.py:967-983`（`CamE2ECmd`）/ `:303-304,337,1084`（`Scan` / `LineScan` / `CamPath`）/
  `raspi/msgs/schema.py:37-46` / `raspi/nodes/planning_node.py:434-440`
- **内容**: `CamE2ECmd` だけが `reason` を持たない。`MIRRORED` にも `CamE2ECmd`（と `CamPath`）が無い。
  `planning_node._current()` は `getattr(scan, "reason", "")` で理由を表示するが、
  `CamE2ECmd` には reason が無い → **理由不明で「点群が古い」だけが出る**。
- **修正案**: `CamE2ECmd` に `reason: str = ""` を追加（`cam_e2e_node` が失敗理由を入れて publish）。
  `schema.py:MIRRORED` と `gen_msgs.py:EXPORTS` を同期する。

#### GEN-16 [L] セクタ境界の 1° オフセットが 3 箇所に重複定義
- **判定**: `PARTIAL`（**二重定義は 3 箇所**。`convert.py:466` は別の物。**主要リンクにはテストがある**）
- **場所**: `raspi/auto/base.py:434-442` / `raspi/nav/deskew.py:121` / `raspi/auto/_slam2d_nav.py:122`
- **内容**: 3 箇所に `((deg - 1) % 360) // 30` の同一式コピーがある
  （`deskew.py:121` のコメントが `base.py` を参照）。
  `raspi/msgs/convert.py:466` は `out = 11 - idx`（`ScanAssembler` の**セクタ番号反転**）で別物。
  **主要リンクにはテストがある**: `raspi/tests/test_auto.py:89-98`
  `test_boundary_belongs_to_previous_sector` が `sector_of_deg` の境界を直接検査
  （`sector_of_deg(1)==0`, `(30)==0`, `(31)==1`, `(0)==11`）、
  `raspi/tests/test_nav.py:87` `test_matches_sector_of_deg` も整合を検査。
  コピーの 2 箇所は直接テストされていない。
- **修正案**: `deskew.py:121` と `_slam2d_nav.py:122` を `sector_of_deg` へ置き換える
  （循環 import になるなら `raspi/msgs/types.py` 等へ共通ヘルパーを移す）。

#### GEN-17 [L] `RingJpeg` のカウンタとリング map がロック無し
- **判定**: `CONFIRMED`
- **場所**: `raspi/core/jpeg.py:126-143,199-209,232-263` / `raspi/nodes/telemetry_node.py:905,969,3459`
- **内容**: `self._rings` dict と `self.torn` / `self.errors` / `self.encoded` 等のカウンタが
  **ロック無し**で更新される。`_ring()` の `dict.get` / `dict[shm_name] = ...` と
  `_drop()` の `dict.pop` は非アトミック → 2 スレッドが同じ名前を開くと二重 attach、
  pop と利用が競合すると**使用中のリングを close** され得る。
  `telemetry_node.py:3459`（`asyncio.to_thread` 経由の `encode_latest`）と
  `:905`（`_serve_http` 経由の `read_latest`）の**別スレッド**から同じ `RingJpeg` を触る。
- **修正案**: `threading.Lock` を持ち `encode_latest` / `read_latest` / `_ring` / `_drop` を囲む。

#### GEN-18 [L] `camera_node` が `Subscriber` を別スレッドで `poll()` する
- **判定**: `CONFIRMED`
- **場所**: `raspi/nodes/camera_node.py:706,516,533-535,598,636-637`
- **内容**: `cfg_sub = Subscriber({TOPIC_CAM_CONFIG: LATEST})` を**メインスレッドで生成**し、
  **別スレッド**（`camcfg`）で `self._cfg_sub.poll(200)` を呼ぶ。
  `close()` は `run_bounded` の **camclose スレッド**から呼ばれる
  （生成スレッドでも CAM ワーカースレッドでもない）。
  pyzmq のソケットはスレッドセーフではなく、生成スレッド以外からの使用は保証されない。
  > **注（第2ラウンド再検証で追加）**: `camcfg` スレッドが `poll()` 中に `camclose` スレッドが
  > ソケットを `close()` すると `zmq.Socket closed` を投げ、`camera_node.py:599-604` が握り潰すため
  > **cam/config の反映が以後恒久に止まる**（ログのみ）。
- **修正案**: `Subscriber` の生成を**ワーカースレッドの中**で行い、`close()` も同じスレッドで行う。
  あるいは `zmq.Socket` を `thread_safe=True` にする。

#### GEN-19 [L] `tv_ratio` にトルク定数を流用、`link_tracker` がスケールをハードコード
- **判定**: `CONFIRMED`（**数値は現状一致**＝実害は小さい）
- **場所**: `raspi/msgs/convert.py:36,196` / `raspi/core/link_tracker.py:267-268` / `raspi/proto/generated/packets.py:197`
- **内容**: `convert.py:36` `_TORQUE = 1e-4  # N·m` を `tv_ratio`（**無次元**）に流用している。
  proto の META も `tv_ratio: (0.0001, None)` なので**数値は一致**。
  `link_tracker.py:267-268` は `1e-3` / `1e-4` を**リテラルでハードコード**。
  > **注（第2ラウンド再検証で訂正）**: このハードコードは
  > `speed` / `steer_actual` / `accel_z` の話であり、`tv_ratio` は
  > `link_tracker` に現れない（`grep tv_ratio raspi/core/link_tracker.py` → 0 件）。
  > 2 つの別件を 1 項目に束ねている。
- **影響**: 将来どちらかが動けば静かに壊れる。
- **修正案**: `convert.py` に `_TV_RATIO = 1e-4`（無次元）を追加する。
  `link_tracker.format_status` は `packets.Telemetry.META` から係数を取り出す。

#### GEN-20 [H] `sim/stm32.py` の `dt_total` cap で停止時間が黙って失われる
- **判定**: `CONFIRMED`（**バーストは 100 フレーム**）
- **場所**: `sim/stm32.py:57,126,392-397,448-452` / `sim/bench.py:47-52`
- **内容**: `advance_to()` は `dt_total = min((t_ns - self._now) / NS, 0.25)` で、
  **0.25 s を超える分は見時計上だけ進み、物理積分は 0.25 s ぶんだけ**（`self._now = t_ns` で時計を飛ばす）。
  `sim/bench.py:47-52` に**自認**がある。
  **再現（`sim/courses/normal.json` + `VirtualStm32`）**:
  ```
  1秒のストール後に増えたテレメトリ = 100 フレーム
  最大バースト = 100 フレーム
  ```
  → **0.75 秒の sim 時間が積分されないまま失われ**（車体の物理が 0.75 s 分遅れる）、
  **100 フレームが 1 度にバーストして届く**（`LATEST` なら 1 通だけ残り他は捨てられ、
  `RELIABLE` なら 76 通が積 piling される）。
- **影響**: ベンチ結果の再現性が壊れる。`sim/bench.py` は実時間 1:1 しか回らないため、
  パラメータ掃引・確率的評価・1 日 1000 ケースが**原理的に不可能**。
- **修正案**: 停止を「時間飛ばし」ではなく「低速進行」にする
  （`self._now = self._now + dt_total*NS` にして時計を飛ばさない）。
  あるいは cap 到達を明示的に診断に raise し、`bench.py` の `Result.report` に
  「ホストが X ms 止まった」件数を出す。

#### GEN-21 [M] `sim/stm32.py` の `CONFIG_ACK` に範囲判定が無い
- **判定**: `CONFIRMED`
- **場所**: `sim/stm32.py:262-266` / ファーム `src/control/control_params.c:42-52` / `src/comm/ras_link.c:459-515`
- **内容**: シムの `CONFIG_ACK` は `applied=msg.value` / `result = OK if known else UNKNOWN_ID` で
  **範囲判定もクランプも無い**（範囲外でも OK を返す）。firmware はクランプして
  `RAS_CONFIG_OUT_OF_RANGE` を返す。
- **影響**: Pi 側の `control_params_status = "mismatch"` → **ARM 全面ブロック**の経路が
  **シムでは一度も再現しない**。SAF-13 の検証もシムではできない。
- **修正案**: `sim/stm32.py` の CONFIG_ACK に firmware と同じ範囲判定・クランプを入れる
  （`tools/fwsim/fw.param_table` の lo/hi を使う）。

---

---

## 4. 設計上のボトルネック

コードではないが、構造の問題。**個別項目を直しても、放置すると同じ穴から
3 つ目のバグが生まれる**ものだけを残した。

### 4.1 「正の伝播網」に検査が 4 本しかない

`tools/check.sh --fast` が走らせる検査は 4 本（`generate.py --check` /
`gen_msgs.py --check` / `check_docs.py --check` / `vehicle.toml` の生成）。
**それぞれが守れるのは「生成物」だけ**で、
**「正の値を変えたときに派生物を追従させる」検査は存在しない。**

**実証（VER-09）**: `protocol.toml` の TELEMETRY `speed` の `scale` を `0.001` → `0.01` に
書き換えて再生成すると、`generate.py --check` は **exit 0（合格）**。ところが
`raspi/msgs/convert.py:151` は手書きの `_SPEED = 1e-3` を使うため、
実デコードは **10 倍ずれ**る。**どの検査もこれを検出しない。**

実際に起きた不具合はすべてこの穴に落ちている。

| 穴 | 該当 |
|---|---|
| 値のコピー（手書き定数） | `convert.py` の 16 スケール（CFG-13）、`link_tracker`（GEN-19）、`gui/src/store/ui.ts` のトルク上限（GUI-11）、SignalConfig（GUI-02） |
| 節の取り違え・キー不存在 | `fwsim/plant.py` の `dyn.get("mu")`（CFG-01）、`rear_lateral_share`（CFG-02）、`[sim]` の 4 キー（CFG-17） |
| firmware 方向の非一方向 | `ras_link.c` の手写し `RAS_LEN_*` と TELEMETRY 並び（VER-07 / VER-08） |
| 実装の二重化 | `sim/` vs `fwsim`（OPS-05）、`deskew`（SLM-08）、`raceline`（NAV） |
| 契約の版が無い | `/ws/control`（CFG-16）、`grip_clear`（CFG-15）、地図フォーマット（SLM-06） |

**対策**: `raspi/core/control_params.py` の `DERIVED`（既に `brake_decel_m_s2` と
`md_torque_lag_s` を持つ）のような**「派生値の単一計算場所」を全シムに強制する**。
現状 `ml_lidar/env.py` は `sim/stm32.py` を参照せず**自前で定数を書いている**（ML-09）。

### 4.2 「モジュール間の接続」だけが検査されていない

`tools/calib/tests`（44 件）と `tools/fwsim/tests`（45 件 + 45 subtests）は
**本物のファームをコンパイルして合成データで検証する**、極めて品質が高い。
一方、**モジュール間の接続**はテストされていない。

- `E2ELidar.plan()` の観測ベクトル == `LidarE2EEnv._to_obs()` の観測ベクトル — **未テスト**（VER-11）
- `replay_node --bus` が **`cmd` を publish しない**こと — **未テスト**（VER-10）
- `tools/calib/toml_io.py`（**単一の source of truth を書き換える部品**）— **テスト 0 件**（VER-06）
- `tools/calib` の**出力値が firmware の受理範囲内**であること — **未テスト**（SAF-13 がまさにこの経路）
- `sim/tests` — **CI に無い**（VER-03）
- ML の 356 件 — **CI に無い**（VER-04）

### 4.3 計画が「安全側の既定」をモジュール間で壊している

本プロジェクトには一貫した安全思想がある（`0 = 未指定 = STM32 最大`、`CONFLATE`、
heartbeat の thread 分離、deadman、seqlock）。**その思想はモジュール境界で
繰り返し破られている**。

| 規約 | 正 | 破れている場所 |
|---|---|---|
| `brake_torque=0` = 最大制動 | `convert.py` / ファーム / `msgs/types.py` | `telemetry_node._merge_auto`（SAF-01） |
| `side_brake` = トグル | `docs/uart_protocol.md:755-759` | `gui/src/ws/control.ts:209-211`（SAF-02） |
| 速度のランプは `cmd_accel_limit_m_s2` が頭打ち | `config/vehicle.toml` | `speedctl.ref_command`（NAV-03）／GUI の `min()` のみ |
| モード遷移時に舵の状態を捨てる | `auto/base.py:8-10` | `dt <= 0` の `alpha=1.0`（SAF-11） |
| 「正」の生成物 | `protocol.toml` / `vehicle.toml` / `types.py` | firmware 手写し（VER-07 / VER-08） |
| 機械は「安全側の既定」 | 全体 | `deploy.sh --services`（SAF-04）、`arm_test`（SAF-03） |
| 未知の入力は拒否 | 全体 | 地図アップロード（SLM-04）、`mapstore.save_map`（SLM-05） |

**型・生成コードに投資してきたのに対して、意味論（値の意味・規約の適用範囲）には
投資してこなかった**ことの表れである。

### 4.4 モード間で暗黙の前提が食い違う

GUI の `max_speed` スライダを同じ位置に置きながらモードを切り替えると、
**同じスライダ位置で別の速度・別の安全余裕になる**。これは設計思想と矛盾する。
各 planner の既定値・正面余裕の意味・舵の生成方式・曲率制限の有無は
**§3.4 末尾の「モード間の暗黙の前提の比較表」** に一覧した（行番号付き）。

### 4.5 制御周期の階層が混乱している

**実測した周期階層**（各段的根拠付き）:

| 段 | 周期 | 場所 |
|---|---|---|
| LiDAR / スキャン | 10 Hz | `raspi/msgs/types.py:265`（`Scan` は LiDAR 1 周ぶん） |
| `plan()`（planner 内部状態） | **実効 10 Hz** | `raspi/nodes/planning_node.py:338-339`（`scan.seq` が変わるときだけ `plan()`） |
| `auto/cmd` publish | 50 Hz | `raspi/auto/base.py:37-40` `CMD_HZ = 50` / `raspi/nodes/telemetry_node.py:274` `CMD_PUB_HZ` |
| `_tick()`（cmd 生成） | 50 Hz | `raspi/nodes/planning_node.py:383-407,537,568-569`（`cmd_period = NS // CMD_HZ`） |
| UART `COMMAND` | 100 Hz | `raspi/nodes/io_node.py:105` `COMMAND_HZ = 100` |
| deadman | 150 ms | `config/vehicle.toml:54` `cmd_deadman_ms` |
| auto cmd stale | 200 ms | `config/vehicle.toml:63` `auto_cmd_stale_ms` |
| link grace | 5000 ms | `config/vehicle.toml:60` `auto_link_grace_ms` |

- **`cmd` は 50 Hz だが、planner の内部状態（舵の 1 次遅れ、速度のランプ）は実質 10 Hz**。
  → **制御の帯域が 1/5**。`def tick` を持つのは `raspi/auto/slam2d_raceline.py:1221` だけ
  （`_calib_common.py:224` にもあるが `category="sysid"` の同定用 mixin で
  シグネチャも戻り値も `(steer, speed, ramp)` 契約とは無関係）。
- **カメラ系は入力トピックの速度が `plan()` 周期になる**（10 Hz ではない）。
  `cam_centerline` は `TOPIC_CAM_PATH`、`line_trace` は `TOPIC_LINE_CAM`、
  `cam_e2e` は `TOPIC_CAM_E2E_CMD`、`follow_the_gap_cam` は `TOPIC_SCAN_CAM`。
- `sim/bench.py:86` の `CMD_HZ = 100` は実機 50 Hz と食い違う（bench 内でも追従 tick は
  `:687` で 50 Hz。**指令だけ倍速**という混在がある）。
- **SLAM の 1 周期予算**（Pi 5 の 100 ms LiDAR 周期に対し `plan()` は既に p99 約 84 ms）:
  局所地図がキーフレームごとに作り直される（`slam2d/core/localmap.py:101-111`、実測 Mac 2.80 ms/回・
  171 回中 171 回 = 全キーフレームで再構築、Pi 5 で約 10〜15 ms）、ループ検出のたびに部分地図を作り直す
  （`backend/loop_detection.py:173-174`、`loop_every_m=0.4` で 2.5 回/秒）。
  加えて `Frontend.refine()` は 1 回で 42.7 ms（Mac 実測、Pi 5 で 150 ms 超）。
  → **`cmd_deadman_ms`（150 ms）に張り付く。**

### 4.6 時刻ドメインが 2 系統ある（型で表現されていない）

実機は Pi の `CLOCK_MONOTONIC` 一本だが、`replay_node --bus` は記録元 Pi の時刻をそのまま
`t_capture` に載せる（GEN-01）。`raspi/msgs/types.py:5-13` は `t_capture` と `t_pub` を
明確にするが、**両者の関係（同一エポックか否か）を型で表現していない**。

`MsgBase` に `epoch_id`（ブート UUID）を 1 フィールド足すだけで、
GEN-01 のような「別エポックの値を比べる」事故が型で防げる。

### 4.7 バスは「drop してよい」前提で、欠落検出が無い

`raspi/bus/zbus.py:316-320` は HWM 到達で `zmq.Again` を**握り潰して**捨てる
（`self.sent` も進まない＝**欠落を数えない**）。`LATEST`（CONFLATE）と組み合わさると、
遅い購読者は「動いているように見えるが古い／欠けた」データだけを見る（GEN-06）。

### 4.8 文書・運用が実装から乖離している

| 文書 | 記載 | 実態 |
|---|---|---|
| `docs/architecture.md:770` | ログは 1 日 15 GB | 8 GB 上限が先 → **数時間〜2 日** |
| `docs/system_overview.md:338` | ログ 7 日 | 同上 |
| `docs/calibration.md:87,26,148,239-240` | 0.35 m/s、1.2 m 四方、0.13 N·m、±0.05 | 0.45、0.8 m、0.15、±0.04 |
| `docs/control_constants.md:41` | `speed_loop_tau_s = 0.155` | 0.0687（NAV-02） |
| `docs/control_constants.md:34,38,45` | 2.47 / 3.04 / 0.006346 | 2.165 / 3.4642 / 0.011629 |
| `docs/control_constants.md:52,54,96` | `tools/sysid`、`speed_pi_zeta`、pole placement | 削除済み・**vehicle.toml に存在しないキー** |
| `docs/stm32_interface.md:211,651` | `TELEMETRY LEN = 84` | **102**（`packets.py:194`・`uart_protocol.md:301`）— 同文書内でも矛盾 |
| `docs/stm32_interface.md:430` / `uart_protocol.md:1111` | `0x001A` | `0x001D` |
| `site/docs/link/protocol.md`・`about.md` | v0.23 / `0x0017` | v0.29（6 版古い） |
| `raspi/setup/install_services.sh:94` | `--max-steer 0.524` | 0.47167 |
| `PROGRESS.md:12` | プロトコル v0.23 | v0.29 |

**「正」の所在が 1 箇所に集まっていない**ので、**同じ事実が 6 箇所に複製され、
更新のたびに 5 箇所が古くなる**。`docs/` には `check_docs.py` があるが
**数値とリンク切れを検査しない**（VER-05）。

### 4.9 GUI 側の構造的問題

- **型整合は良好**: `gen_msgs.py --check` は通り、`tsc -b` 通過、`any` ゼロ。
  **しかし WS の JSON 形は `gen_msgs.py` の対象外**（`config/gen_msgs.py:33-38`）で、
  Python と TS に手書き二重管理されている（GUI-03 / CFG-16）。
- **スキーマ hash が全-or-nothing**（`raspi/msgs/schema.py:65-79`）:
  `VehicleState` にフィールドを 1 個足すだけで GUI↔Pi の同時デプロイが強制され、
  版不一致時に**全フレームを捨て**て ARM 不能になる（GUI-05）。
- **mcap 記録をブラウザ RAM に上限なしで蓄積**（GUI-06）:
  `finish()` は Buffers を全部メモリに載せてから `Blob` を作るため、
  **OOM すればダウンロードは一度も始まらないまま記録が全滅**する。

---

## 5. 検証で取り下げた・再校正した指摘

**4原稿をそのまま採用せず再検証した結果、以下は取り下げた。**
（これらは報告段階では最上位の指摘として出ていた。）

| 取り下げた指摘 | 原稿の主張 | 再検証の結果 |
|---|---|---|
| **`park_to_point` の停止待ちにタイムアウトが無い** | 「`_waiting_stop` が永久に True になり恒久デッドロックしうる」 | **FALSE**。`park_to_point.py:780-781` の `max_maneuver_s` 打ち切りは `_follow()` / `_switch_stop()` **より前**にあるため、停止待ち中も制限時間は進む。disengage 中に `_maneuver_t` が進まないのは「走行指令を出していない」ため安全。→ SAF-05 として**取り下げ** |
| **校正の中止時に「最も弱い制動トルク」を出す** | 「最高速で走り続ける車に対して最も弱い制動给出するのは逆効果」 | **PARTIAL**。数値（0.08 N·m）と呼び出しは正しいが、**同じファイルの通常停止トルクは 0.05〜0.06 N·m**で、abort の 0.08 は**いずれもより強い**。「弱すぎて止まらない」ファイルではない。真の欠陥は根拠コメント（「0.12 N·m 以上は ABS なしで後輪がロックする」）の陳腐化（ABS は既定で有効） |
| **最高速の GUI クランプ方向** | 「LIMITS 受信前は 3.0 でクランプし、届いた時点で解放する」 | **FALSE**。`gui/src/store/ui.ts:407-424` は LIMITS 未受信のとき**上を Infinity（無クランプ）**で保存値を保持し、受信後に `LIMITS.max_speed_m_s` でクランプする。設計は逆 |
| **校正の `self.room` が 1 周期古い** | 「`leg_step()` が古い `self.room` を読む」 | **FALSE**。`_calib_common.py:395-396` で `self.room` は `leg_step()` 呼び出し**前**に代入済み。398-400 行は同一式の再代入。`calib_steer.py:105` も `vs.steer_actual` を渡している → OPS-07 として**取り下げ** |
| **契約 JSON に現在の `vehicle.toml` 値が入る** | 「既存 27 個すべてが現在の値でズレている」 | **PARTIAL**。既存 26 個は `max_steer=0.524`（**学習当時の値**。全 run は 2026-09 の 0.524 時代に学習）で**正しい**。危険なのは今後の再エクスポート時のみ → ML-02 |
| **`rear_lateral_share` 欠落** | 「Critical」 | 値は常に 0.59 で**ずれない**（writer が存在しない）。実害は①文書が誤り②将来の再校正が黙って捨てられる③影響は `grip_shape=1` の隠し設定に限られる の 3 点 → **Medium**（CFG-02） |
| **fwsim の縦 accel 上限 5.17 m/s²** | 「fwsim の縦 accel 上限は 5.17」 | **再現不能**。`grep -rn "5\.17" tools/ sim/ raspi/` → 0 件。同一 `Plant` からの理論値は約 8.9 m/s² → 数値は**取り下げ**（CFG-01 の主張自体は成立） |
| **`maxSteer` の既定 0 で GUI のスライダが動かなくなる** | 「toml に `max_steer` が無いとき GUI は 0 rad」 | **PARTIAL**。実際の生成値は 0.47167（toml に存在するため）。0 になるのは toml から削除したときのみ。かつ `max < min` となるのは**スライダの表示範囲だけ**で、モータへ効くクランプは `io_node` 側（CFG-11） |
| **GUI が「指令が通っているか」を一切表示しない** | 「一切表示しない」 | **PARTIAL**。`StatusBar.tsx:85-87` に ARMED/DISARM（STM32 実状態）、`:46-47,119-126` に health / RTT / 通信途絶、`:51` に E-STOP 発動表示がある。ただし `cmd_stale`・`deadman_trips` は診断タブのみ、`ui.deniedBy` は**読み手がゼロで死んだ状態**（GUI-01） |
| **自律走行の解除がパッドの微小入力で成立** | 「Deadzone 適用後も `padSteer !== 0` になりうる」 | **PARTIAL**。`dz()` は Deadzone 未満を**厳密に 0** にする（既定 0.12）ので成立しない。ただし Deadzone を 0 に設定すれば成立し、解除判定にヒステリシスが無い（GUI-07） |
| **セクタ 1° オフセットの二重定義** | 「`convert.py:466` も二重定義。テストは LiDAR 側しか未検査」 | **PARTIAL**。二重定義は**3 箇所**（`base.py:434-442` / `nav/deskew.py:121` / `_slam2d_nav.py:122`）。`convert.py:466` は別の物（セクタ番号反転）。**主要リンクにはテストがある**（`test_auto.py:89-98` が境界を直接検査） → GEN-16 |
| **`on_disengage` 時に `reset()` が呼ばれない** | 「契約は記述のみ、強制されない」 | コード事実は正しいが、`base.py:115-128` の docstring に**意図が明記**され `planning_node.py:287-293` が実装している → **設計通り**（GEN-10） |
| **SLAM の executor 残留** | 「ノード終了時に残る」 | **PARTIAL**。`slam2d_raceline.py:436` に `atexit.register(_shutdown_worker)` があり、プロセス終了で残ることはない。`close()` で「プロセスは使い回すので止めない」は**設計通り**。ただし `planning_node.close()` が `planner.close()` を呼ばない事実は正しい（GEN-08） |
| **`Publisher.seq` の巻き戻り** | 「`telemetry_node.py:3115-3119` に独自ヒューリスティック」 | **PARTIAL**。`cam_track_node.py:262-281` には明示的な巻き戻り処理があるが、`telemetry_node.py:3115-3119` は「変更検出用の版タプル」であって巻き戻り判定ではない。行指定が的外れ（GEN-07） |
| **STM32 再起動検出の 2 系統** | 「片方だけが発火した場合に不整合が起きる」 | 2 系統あるのは事実だが、**2 系統目（VERSION バースト）は `t_us` 後退では見えない短時間再起動用の相方**で、収束点は `io_node._on_time_reset` で共通。**冗長ではなく補完**（CFG-20） |
| **`cmd` の 1 周期レース** | 「`cmd` が auto/cmd を見る前に publish される」 | 既に main 原稿で取り下げ済み。実効遅延は約 1 ms。残る実際の問題は `accel_limit` / `launch` だけが change key に無いこと（SAF-12） |

### 5.1 第2ラウンド独立再検証で再校正した項目

統合版を 6 エージェントが再検証した結果、**統合版自身の記述に 30 箇所の誤り**が見つかり、
すべて訂正した（§0.3.1）。この表はその要約。

| 統合版の記述 | 第2ラウンドの判定 | 訂正後 |
|---|---|---|
| CFG-01「fwsim の **3.2 倍 grippier** な車」 | 過大 | μ *パラメータ*比は 3.17 倍だが**結果のグリップ差は横 +9.5 % / 縦 +6.6 %**（C モデル実測 3.43 m/s²）。深刻度 C → **H** |
| §5「fwsim の縦 accel 上限は約 **8.9 m/s²**」 | 2.6 倍過大 | **3.43 m/s²**（タイヤ形状係数 0.454 を考慮した C モデル実測） |
| VER-01「CI の**全工程**が**無効化される**」 | 過大 | 前段は**実行して成功**しており失敗は最終ステップのみ。`tsc -b` は別 job。深刻度 C → **H** |
| VER-03「原因は 2026-10-18 の再校正（`10cd778`）」 | 因果誤認 | 実犯は **`98f0288`（2026-10-19）**。`10cd778^` でも同じく落ちる（二分探索で確定） |
| VER-08「削除済みツールを **9 箇所**で参照」 | 件数不一致 | **8 occurrence / 7 行**（`sysid` 6 ＋ `ctrl_tune` 2） |
| VER-13「`test_mapstore.py`（**31 件**）」 | 件数誤り | **34 件**。修正案①も現状挙動（`load_map()` が `None`）と整合する形に修正 |
| OPS-01「`.sfl` 1.5 MB/分 → **3.6 GB/日** → 約 2.2 日」 | 鎖の矛盾 | 1.5 MB/分なら **2.1 GB/日 → 約 3.8 日**。3.6 GB/日・2.2 日は docs 実測 2.5 MB/分由来。採用値を明記 |
| SAF-09「メインループが 200 ms 止まれば **350 ms** 後に DISARM」 | 自己矛盾 | 鮮度判定はループ内なので **200 ms**（＝ `max(stall, 150 ms)`） |
| SAF-13「惰行が下端に届かず span が 0.8 を切ると `hi < 0.8`」 | 因果の逆 | `encoder.py:222-226` の自認注释は「下端に届かず → `hi > 1.0` → 再試行」。深刻度 H → **M** |
| SAF-13「`HI_MIN` は解析側で一度も使われていない」 | 過大 | **合否判定で**使われていない（`tools/calib/tests/test_encoder.py:63` は検査している） |
| SAF-11「alpha が **≈1.0** になり平滑化がスキップされる」 | 過大 | 通常は dt≈1 スキャン周期で **alpha ≈ 0.63**。**修正案の順序も逆**（`dt<=0` の alpha を先に 0 にする）。深刻度 M → **L** |
| SAF-06「同ファイルの通常停止トルクは 0.05〜0.06」 | 出所の混在 | `_calib_common.py:275` は **0.06 のみ**。0.05 は `calib_steer.py:160`、0.08 は `calib_tyre.py:117`（abort と同値） |
| GEN-20「最大バースト = **76 フレーム**」 | 再現せず | **100 フレーム**（同じ節が「100 フレーム」とも書いていた自己矛盾） |
| GEN-02「毎サイクル 200 ms 超 → 駐車モードが一度も動けない」 | 文書内部矛盾 | 付録の実測は **8.3 ms・合計 160 ms < 200 ms**。「恒久 stale」は未立証。深刻度 H → **M**、判定を §2 と揃えて `PARTIAL` に |
| GEN-09「`★{_ctrl_error}` が毎周期連結され蓄積」 | 機構の誤り | `:452-453` は **`not in` ガード**で 1 回だけ。毎周期蓄積するのは `:480`（NaN/Inf 経路） |
| NAV-13「garage 250 ms → 250.5 ms」 | シーン名の誤り | 予算切れは **DEADEND(margin=0.20)** シーン。garage は必ず解ける |
| NAV-02「`lead_s = 0.0321`」 | 値誤り | **`0.0421`**（`_ref_lead()` の実装値） |
| NAV-01「表の 12 セル」 | 条件未記載 | 「車両を**経路中央**に置く」条件。端に置くと 120/150/179° の欄が変わる |
| ML-02「`0.4636` はリポジトリ内に**存在しない**」 | 事実誤認 | 2026-10-09〜10 の実値で、**現在も `vehicle.py:119` の docstring に残る**（CFG-03 との自己矛盾を解消） |
| SLM-01 深刻度 Critical | 過大 | 運用コードも `optimize_every=0`。**潜在罠**であり H が妥当 |
| SLM-03「表ほぼ一致で再現」 | 過大 | 独立再検証では**全セル 1 個ずつ少ない**。傾向・棄却理由の分布は一致 |
| GUI-02「4 箇所・28 リテラル」 | 過少計上 | **5 箇所・31 リテラル**（`ArrowSignalStatus` を含む）。GUI 側は 13 箇所 |
| GUI-02「`grep "SignalConfig"` も該当なし」 | 事実誤り | `test_arrow_signal_node.py` に 12 件以上ある（ただし既定値の assert は 0 件） |
| GUI-05「1 フィールド追加で地図・カメラまで全部落ちる」 | 過大 | 止まるのは **`/ws/telemetry` 1 チャネル**。`/ws/map`・`/ws/camera` は別チャネルで残る |
| GUI-09「`maxSteer` に保存値の巻き戻りが残る」 | 過大 | `PI_MAX_STEER_CAP` は**物理上限そのもの**（仮値ではない）。修正案も不適切。深刻度 M → **L** |
| NAV-20「firmware 突合不能」 | 過小 | firmware `drive.c:385` で突合成功。判定 `PARTIAL` → **`CONFIRMED`** |
| CFG-25「0.4505 m / 0.469 m」 | 数値誤り | **0.4509 m / 0.4681 m** |
| CFG-08「`STEERING_MIN_SLOPE`」 | マクロ名誤り | **`STEERING_LINKAGE_MIN_SLOPE`** |
| CFG-04「最高速の上限が 4 か所」 | 少計 | **5 か所**（運用系）＋ planner 既定 8 か所 |
| CFG-03「9 種類」 | 表記 | **8〜9 種類**（0.524≡30°、0.4636≡26.56°） |
| GEN-18「close() もカメラワーカースレッドから」 | 誤り | 実際は **`run_bounded` の camclose スレッド**。加えて cam/config 反映が恒久停止する点を追記 |
| GEN-19「link_tracker が tv_ratio のスケールをハードコード」 | 束ねすぎ | link_tracker は **`speed`/`steer_actual`/`accel_z`** の話。`tv_ratio` は 0 件 |
| 行番号ずれ | 軽微 | `types.py:1503-1504`→1500-1501、`telemetry_node.py:1509-1511`→1504-1505、`gen_msgs.py:33-38`→63-64、`samples.py:167-171`→163-167、`ml_cam_e2e/train.py:131`→132、`register.py:305-306`→304、`sim/params.py:43`→42、`telemetry_node.py:948-950`→946-948、`io_node.py:1031`→1033、比較表 `park_to_point` の `:777` 削除 ほか |
| 文字化け | 表記 | `注释`→`コメント`（6 箇所）、`函数`→`関数`、`详见`→`詳述`、`ロジックで만`→`ロジックのみで`、`acted`→`参照されない`、`intention`→`意図` |

---

## 6. 対応ロードマップ

> **注（第2ラウンド再検証後）**: 深刻度が 13 件下がり **Critical は 0 件**になった。
> 優先順位の骨格（検査網 → 伝播網 → 制御・学習 → 局在化・UI）は変わらないが、
> 個別項目は §2 の索引を参照して，各自の最新の深刻度と
> 附録A の対応表を見てから着手すること。
> 特に **§5.1 の訂正済みの記述**（数値・因果）を出発点にすると二重工作量にならない。

優先順位は**最初に直すべきは個別の数値ではなく検査網**である。
CI を直し、意味論の検査（firmware の並び・convert のスケール・契約の `dt`）を足し、
**「正」を 1 箇所に集める仕組み**（`control_params.DERIVED` の応用）を広げる。
それが済めば、High のうち **半分以上は構造的に発生しなくなる**。

### 段階 1 — 安全と検証網の復旧（1〜2 日）

| # | 対応 | 対象 |
|---|---|---|
| 1 | **CI を修復**し `tools/check.sh` と一本化。`test_fw_constants.py` / `test_control_logic.py` を CI に追加（`TestParamTable` に `skipUnless(HAVE_FW)` を先に付ける） | VER-01, VER-02 |
| 2 | **CI に `sim/tests` と ML の 356 件を追加**し、`ml_cam` / `ml_cam_e2e` に `__init__.py` を足して collection 衝突を解消 | VER-03, VER-04 |
| 3 | **master の 2 件の赤（`sim/tests/test_speed_ff.py`）を先に解消**してから CI に入れる | VER-03 |
| 4 | **`_merge_auto` の `brake_torque` 規約を統一**。`AutoState.brake_torque=0` の意味（最大制動か GUI 値か）を 1 本に決め、docstring と実装を一致させる | SAF-01 |
| 5 | **GUI の `disarm()` に `side_brake` / `winker_*` を保持**。未 ARM 経路（`useDriving.ts:953`）の `side_brake: false` も修正。ARM 時に `sideBrakeRequested` が true なら警告 | SAF-02 |
| 6 | **地図アップロードに展開後サイズ上限とトークン必須** | SLM-04 |
| 7 | **校正 1 / 5 の範囲外を fail に**（`fit.hi < HI_MIN` の判定追加）。`toml_io.apply` に firmware 範囲検証・fsync・`.bak`・モード保存 | SAF-13, CFG-22 |
| 8 | **`abort_state` の根拠コメントを実態に合わせる**（ABS 既定有効なら 0.08 N·m の制約は不要） | SAF-06 |
| 9 | **校正 8 の発振判定に絶対基準と baseline 下限**。「判定不能」を「発振なし」と区別する 3 値化 | SAF-14 |
| 10 | **`slam2d` の `optimize_every>0` のクラッシュ修正**（`_sync_graph` を `_advance_job` の前へ）＋テスト追加 | SLM-01 |

### 段階 2 — 「正」の伝播網の補修（3〜5 日）

| # | 対応 | 対象 |
|---|---|---|
| 11 | **μ / `a_lat_m_s2` / `plant.mu` の 3 重重複を解消**。`[limits]` を正とし、`tools/fwsim/plant.py` の `dyn.get("mu")`（存在しないキー）を `[limits]` へ | CFG-01 |
| 12 | **fwsim と `sim/` の「同じ車」の grip を照合するテスト**を追加（`test_fw_constants.py`） | CFG-01, VER-13 |
| 13 | **`convert.py` のスケール 16 項目を `packets.Telemetry.META` へ**。`link_tracker` のハードコードも除去 | CFG-13, GEN-19 |
| 14 | **「正の値を変えたら派生物を追従させる」検査を足す**（protocol.toml の scale ↔ convert.py の定数） | VER-09 |
| 15 | **`rear_lateral_share` を toml に追加**（writer も reader も揃える）。`[sim]` の 4 キーを削除するか `[limits]` から生成 | CFG-02, CFG-17 |
| 16 | **`mapstore.save_map` の NaN/Inf 検証と `allow_nan=False`**。一時名の一意化と書込順序の修正。地图フォーマットに `format` を追加 | SLM-05, SLM-06, SLM-07, SLM-09 |
| 17 | **`MsgBase` に `epoch_id`（ブート UUID）を追加**し、`replay_node` の時刻ドメインを統一 | GEN-01, CFG-19 |
| 18 | **`planning_node` のモード切替で `_last_plan_ns = 0`**。`dt <= 0` の `alpha` を 0 に | GEN-04, SAF-11 |
| 19 | **`/ws/control` に版（schema）を載せ**、GUI 側で照合。`MapFile` / `LogFile` / `ControlStatus` を生成対象に | CFG-16, CFG-15 |
| 20 | **firmware 側 `RAS_LEN_*` と TELEMETRY 並びを検査するテスト**を追加。firmware に `surge_proto.h` を含ませる | VER-07, VER-08 |
| 21 | **ZMQ の欠落を数える**（`zmq.Again` で `dropped += 1`）＋購読側の seq 欠番検出 | GEN-06 |

### 段階 3 — 制御・学習の整合（1 週間）

| # | 対応 | 対象 |
|---|---|---|
| 22 | **E2E モデルの契約刷新**: `dt` / `steer_tau` / `max_steer` / `steer_link` を学習 run に記録。v13〜v20 は `dt=0.05` のため**再エクスポートではなく隔離**（ロード拒否） | ML-01, ML-02 |
| 23 | **`ml_lidar/export_onnx_rl.py` は `env_config.json` の学習時の値を契約 JSON に書く**（`ml_cam_e2e` の実装に揃える）。学習時の値が無い run からのエクスポートは停止 | ML-02 |
| 24 | **`preproc_version` の照合を `ml_cam`（セグメ）にも入れ**、契約 JSON が無い/版が違う場合は refuse。`ml_cam/train.py` に `preproc_version` を記録 | ML-03, ML-10 |
| 25 | **`ml_cam/export_onnx.py` が `train_config.json` の `input_size` を読む**。`verify_parity()` に加えて「契約 == ONNX shape == train_config」の三者一致を検証。GUI の手入力欄を廃止 | ML-04 |
| 26 | **学習/評価の分割を記録単位に**。`np.random.seed` を追加 | ML-06, ML-07 |
| 27 | **Pure Pursuit の FF 分岐に `\|η\|>90°` の飽和を適用**（`steer_for_target` と共通の飽和関数へ） | NAV-01 |
| 28 | **`speedctl.ref_command` のランプを `cmd_accel_limit_m_s2` / `a_decel_max` で頭打ち**。安全の網を GUI から nav 側へ移す | NAV-03 |
| 29 | **`speed_loop_tau_s` を正の位置に寄せる**（`vehicle.toml` へ）＋文書とコードの数値検査 | NAV-02 |
| 30 | **既定の障害物安全設定を見直す**（`obstacle_stop` の既定を「必ず止まる距離」にする、または `slow_on` と `stop_at` を分離） | NAV-04 |
| 31 | **`react_s` に confirm の 1 周期を足す**（`2×SCAN_PERIOD_S + cmd_latency + 0.5τ`） | NAV-05 |
| 32 | **シムの plant に MD トルク遅延・スリップモデル・`CONFIG_ACK` の範囲契約を導入**。`launch` の TC 依存を再現。`dt_total` cap で時計飛ばさない | NAV-21, NAV-22, NAV-24, GEN-21, GEN-20 |
| 33 | **`park_to_point` の `plan_budget_ms` 上限を `auto_cmd_stale_ms/2` に**。`_replan_fallback` に時間予算 | GEN-02 |
| 34 | **`e2e_lidar` に `a_lat_max` による曲率上限を追加**。`free_ahead` のコメントを実態に合わせる | ML-08 |
| 35 | **`nearest_index` に全体フォールバックを追加**（窓の縁に落ちたら全体検索） | NAV-11 |
| 36 | **Hybrid A\* の margin を局所化し**、`goal_slack` は終端ノードの受容判定の緩さだけに使う。`park_to_point` の `thr` を計画と一致させる | NAV-12 |
| 37 | **`StandstillSteer` を共有基底へ移し全 planner で有効化**。文書（`system_overview.md:90`）を実態に合わせる | NAV-14 |
| 38 | **MPC の `now` の時刻基準を統一**（`now=None` を許さない）。`cmd_steer_rate_limit_rad_s` を nav へ 전달 | NAV-06, NAV-07 |
| 39 | **`sim/bench.py` の `CMD_HZ` を 50 に**（`base.py` から import）。`min_curvature_alpha` を `nav/raceline.py` に一本化 | NAV-23, NAV-24 |

### 段階 4 — 局所化・UI・運用（1〜2 週間）

| # | 対応 | 対象 |
|---|---|---|
| 35 | **SLAM**: `optimize_every` の順序修正、局所地図の差分更新、`ambiguity_ratio` の LD06 依存性を見直す | SLM-01, SLM-03, §4.5 |
| 36 | **SLAM**: `matched=False` を `lost` 扱い、RMS の絶対上限、`_relocalize` の `max_correction` | SLM-02 |
| 37 | **GUI**: `disarm` 保持、mcap RAM 上限と自動停止、`notify` の連打対策、Deadzone の下限、`take_control` のバックオフ | SAF-02, GUI-06, GUI-07, GUI-08 |
| 38 | **GUI**: 「指令経路」のピル追加（`cmd_stale`）、`ui.deniedBy` を実装または削除、「走行中」バッジを `st.ready` でゲート | GUI-01 |
| 39 | **WS JSON 形の生成**（`gen_msgs.py` 対象拡大）と前方互換スキーマ。`/ws/map` にも schema | GUI-03, GUI-04, CFG-16 |
| 40 | **GUI テスト基盤（vitest）の導入**と送信内容のスナップショットテスト | GUI-10 |
| 41 | **ドキュメントの数値同期と `check_docs.py` の拡充**（インライン版・`site/`・リンク切れ・主要数値） | VER-05, OPS-03, OPS-04, CFG-23, CFG-24 |
| 42 | **ログ保持を時間ベースで明示**、cleaner を統一、`keep_newest` 追加、`AsyncFrameLog` のオーバーフローを EVENT として記録 | OPS-01, OPS-02 |
| 43 | **io_node の arm-block 経路を制動経由にする**。`logger_node` のディスクチェックをワーカー化 | SAF-08, OPS-08 |
| 44 | **PlanningNode.close() で `planner.close()` を呼ぶ**。無防備コールバック 3 つにガード | GEN-08, GEN-03, GEN-09 |
| 45 | **ZMQ の前方一致を `/` 終端プレフィックスのみに制限**し、未知トピックは raise。表検証テストを追加 | GEN-22 |
| 46 | **`convert.py` の mode 判定**：DISARM 時は ARM ビットを立てない | GEN-23 |
| 47 | **スキャン基準時刻の規約を統一**（`Scan.t_capture` は先頭、deskew の `t_ref` は末尾）。docstring に明記 | GEN-24, SLM-08 |
| 48 | **`merged()` で inf を弾く**。`registry.catalog()` の sysid 除外。`follow_object` の `ld` に上限 | GEN-11, GEN-12, GEN-13 |

---

## 付録A — 原稿ID との対応表

4原稿のどの指摘が統合後のどの項目になったか。**同一原稿内で複数回言及されたものは 1 回だけ記載**。
「—」は当該原稿に無い。

| 原稿ID | 原稿 | 統合ID | 検証 |
|---|---|---|---|
| C1 | main | SAF-01 | PARTIAL |
| C2 | main | VER-01 | CONFIRMED |
| H3 | main | CFG-01 | CONFIRMED |
| H4 | main | ML-01 | CONFIRMED |
| H5 | main | ML-02 | PARTIAL |
| H6 | main | NAV-12 | CONFIRMED |
| H7 | main | NAV-11 | CONFIRMED |
| H8 | main | NAV-13 | PARTIAL |
| H9 | main | NAV-14 | CONFIRMED |
| H10 | main | NAV-22 + VER-03 | CONFIRMED |
| H11 | main | SAF-03 | CONFIRMED |
| H12 | main | SAF-04 | CONFIRMED |
| H13 | main | SAF-10 | PARTIAL |
| H14 | main | SLM-02 | CONFIRMED |
| H15 | main | VER-07 | CONFIRMED |
| M16 | main | CFG-02 | CONFIRMED |
| M17 | main | CFG-13 | CONFIRMED |
| M18 | main | ML-03 + ML-10 | CONFIRMED / PARTIAL |
| M19 | main | GUI-01 | PARTIAL |
| M20 | main | GUI-02 | PARTIAL |
| M21 | main | SAF-12 | CONFIRMED |
| M22 | main | SAF-09 | PARTIAL |
| M23 | main | SAF-07 | PARTIAL |
| M24 | main | NAV-18 | CONFIRMED |
| M25 | main | NAV-20 / NAV-21 | CONFIRMED / CONFIRMED |
| M26 | main | VER-08 | PARTIAL |
| M27 | main | ML-09 | PARTIAL |
| M28 | main | SLM-08 | PARTIAL |
| M29 | main | SLM-06 | CONFIRMED |
| M30 | main | CFG-22 | CONFIRMED |
| M31 | main | OPS-07 | **FALSE** |
| M32 | main | GUI-03 | PARTIAL |
| M33 | main | OPS-01 | PARTIAL |
| M34 | main | OPS-02 | CONFIRMED |
| M35 | main | GUI-04 | CONFIRMED |
| L36 | main | OPS-03 | CONFIRMED |
| L37 | main | VER-05 | CONFIRMED |
| L38 | main | OPS-05 | CONFIRMED |
| C1 | nav | NAV-01 | CONFIRMED |
| H1 | nav | NAV-02 | CONFIRMED |
| H2 | nav | NAV-03 | CONFIRMED |
| H3 | nav | NAV-04 | CONFIRMED |
| H4 | nav | NAV-05 | CONFIRMED |
| M1 | nav | NAV-06 | CONFIRMED |
| M2 | nav | NAV-07 | CONFIRMED |
| M3 | nav | NAV-08 | CONFIRMED |
| M4 | nav | NAV-09 | CONFIRMED |
| M5 | nav | NAV-10 | CONFIRMED |
| L1 | nav | NAV-15 | CONFIRMED |
| L2 | nav | NAV-16 | CONFIRMED |
| L3 | nav | OPS-03 | CONFIRMED |
| L4 | nav | NAV-17 | SEVERITY-OFF |
| C-1 | contract | CFG-01 | CONFIRMED |
| C-2 | contract | CFG-02 | CONFIRMED（深刻度過大） |
| C-3 | contract | CFG-04 | CONFIRMED |
| C-4 | contract | CFG-15 | PARTIAL |
| H-1 | contract | CFG-03 | CONFIRMED |
| H-2 | contract | CFG-12 | CONFIRMED |
| H-3 | contract | CFG-14 | CONFIRMED |
| H-4 | contract | VER-08 | PARTIAL |
| H-5 | contract | CFG-06 | CONFIRMED |
| H-6 | contract | CFG-05 | CONFIRMED |
| M-1 | contract | CFG-07 | CONFIRMED |
| M-2 | contract | CFG-08 | PARTIAL |
| M-3 | contract | CFG-19 | CONFIRMED |
| M-4 | contract | GEN-24 | PARTIAL |
| M-5 | contract | CFG-20 | CONFIRMED |
| M-6 | contract | CFG-21 | PARTIAL |
| M-7 | contract | VER-01 | CONFIRMED |
| M-8 | contract | VER-02 | PARTIAL |
| M-9 | contract | NAV-23 | CONFIRMED |
| M-10 | contract | OPS-03 | CONFIRMED |
| M-11 | contract | CFG-23 | CONFIRMED |
| M-12 | contract | OPS-04 | CONFIRMED |
| M-13 | contract | CFG-17 | CONFIRMED |
| M-14 | contract | CFG-16 | CONFIRMED |
| L-1 | contract | CFG-09 | CONFIRMED |
| L-2 | contract | CFG-10 | CONFIRMED |
| L-3 | contract | CFG-13 | CONFIRMED |
| L-4 | contract | CFG-11 | PARTIAL |
| L-5 | contract | CFG-24 | PARTIAL |
| L-6 | contract | CFG-26 | CONFIRMED |
| L-7 | contract | CFG-25 | CONFIRMED |
| L-8 | contract | VER-05 | CONFIRMED |
| B-1〜B-6 | contract | §4.1 / §4.6 / §4.8 | — |
| T-1〜T-12 | contract | §4.2 / VER-05 / VER-06 / VER-10 / VER-11 / VER-13 | — |
| C1 | summary | SAF-01 | PARTIAL |
| C2 | summary | VER-01 | CONFIRMED |
| C3 | summary | CFG-01 | CONFIRMED |
| C4 | summary | SAF-02 | PARTIAL |
| C5 | summary | SLM-01 | CONFIRMED |
| C6 | summary | SLM-04 | CONFIRMED |
| H1 | summary | NAV-02 | CONFIRMED |
| H2 | summary | NAV-03 | CONFIRMED |
| H3 | summary | GEN-23 | PARTIAL |
| H4 | summary | SAF-14 | CONFIRMED |
| H5 | summary | SAF-13 | CONFIRMED |
| H6 | summary | CFG-22 | CONFIRMED |
| H7 | summary | SLM-05 | CONFIRMED |
| H8 | summary | GEN-01 | CONFIRMED |
| H9 | summary | NAV-01 | CONFIRMED |
| H10 | summary | ML-08 | CONFIRMED |
| H11 | summary | SAF-06 | PARTIAL |
| H12 | summary | SAF-11 | PARTIAL |
| H13 | summary | GEN-02 | PARTIAL |
| H14 | summary | SAF-05 | **FALSE** |
| H15 | summary | NAV-21 | CONFIRMED |
| H16 | summary | SLM-03 | CONFIRMED |
| H17 | summary | ML-02 | PARTIAL |
| H18 | summary | ML-03 | CONFIRMED |
| H19 | summary | ML-04 | CONFIRMED |
| H20 | summary | VER-04 | CONFIRMED |
| H21 | summary | ML-05 | CONFIRMED |
| H22 | summary | ML-06 | CONFIRMED |
| H23 | summary | ML-07 | CONFIRMED |
| M1 | summary | CFG-13 | CONFIRMED |
| M2 | summary | GEN-19 | CONFIRMED |
| M3 | summary | GEN-03 | CONFIRMED |
| M4 | summary | OPS-08 | PARTIAL |
| M5 | summary | GEN-17 | CONFIRMED |
| M6 | summary | SAF-08 | CONFIRMED |
| M7 | summary | SAF-09 | PARTIAL（機序部分は FALSE） |
| M8 | summary | GEN-09 | CONFIRMED |
| M9 / M10 | summary | GEN-11 | CONFIRMED |
| M11 | summary | GEN-13 | CONFIRMED |
| M12 | summary | GEN-14 | CONFIRMED |
| M13 / M14 | summary | GEN-12 | CONFIRMED |
| M15 | summary | GEN-08 | PARTIAL |
| M16 | summary | GEN-18 | CONFIRMED |
| M17 | summary | GEN-15 | CONFIRMED |
| M18 | summary | GEN-22 | CONFIRMED |
| M19 | summary | GEN-16 | PARTIAL |
| M20 | summary | SAF-12 | CONFIRMED |
| M21 | summary | CFG-02 | CONFIRMED |
| M22 / M23 / M24 | summary | SLM-07 | CONFIRMED |
| §5.1 | summary | §4.1 | — |
| §5.2 | summary | §4.2 | — |
| §5.3 | summary | §4.3 | — |
| §5.4 | summary | §3.4 の比較表 / §4.4 | — |
| §5.5 | summary | §4.5 | — |
| §5.6 | summary | §4.6 | — |
| §5.7 | summary | §4.7 / GEN-06 | CONFIRMED |
| §5.8 | summary | §4.8 / OPS-03 | CONFIRMED |
| §5.9 | summary | §4.9 | — |
| §6 | summary | §4.2 / §6 | — |
| §7 | summary | §6 | — |

**統合結果**: 原稿 4 本の指摘は **166 個**（重複込み）→ 統合・検証後 **142 個**
（第1ラウンド）→ 第2ラウンド再検証で **140 個** ＋ 取り下げ 2 個。

> **第2ラウンド再検証で変更になった判定**: `NAV-20`（main M25 相当）は
> `PARTIAL` → **`CONFIRMED`**（firmware `drive.c:385` で突合成功）。
> 第2ラウンドで **深刻度 13 件を下げ**（Critical 4 → 0）、
> 数値・因果・行番号・表記を **30 箇所訂正**した（§0.3.1 / §5.1）。
同じ問題を報告していた原稿の組:

- μ 3 系統: main H3 ＝ contract C-1 ＝ summary C3 → **CFG-01**
- `rear_lateral_share`: main M16 ＝ contract C-2 ＝ summary M21 → **CFG-02**
- CI 破壊: main C2 ＝ contract M-7 ＝ summary C2 → **VER-01**
- firmware 突合テスト: main H15 ＝ main M26 ＝ contract H-4 ＝ contract M-8 → **VER-07 / VER-08**
- `convert.py` スケール: main M17 ＝ contract L-3 ＝ summary M1 → **CFG-13**
- Pure Pursuit FF: main H9 ＝ nav C1 ＝ summary H9 → **NAV-01**
- `speed_loop_tau_s`: nav H1 ＝ summary H1 → **NAV-02**
- 速度ランプ上限: nav H2 ＝ summary H2 → **NAV-03**
- `max_steer` の多値: main L36 ＝ contract H-1 ＝ nav M4 → **CFG-03 / NAV-09**
- `axle_model` 到達不能: main M24 ＝ contract H-2 → **CFG-12 / NAV-18**
- `deskew` 二重化: main M28 ＝ nav §6 交差参照 → **SLM-08**
- `mapstore` の NaN・順序: main M29 ＝ summary M22〜M24 → **SLM-05 / SLM-06 / SLM-07**
- `ml_lidar` 停止定数: main M27 ＝ nav N25 → **ML-09**
- `park_to_point` 予算: nav §5.4 ＝ summary H13 → **GEN-02**

---

## 付録B — 検証の再現手順と証跡

### B.1 検証環境

- リポジトリ: `surge_mk2` @ `aee4e1d`（作業ツリーは未追跡の review md 4 件のみ、clean）
- firmware: `/Users/banbatakumi/GitHub/AutonomousCar/Program/MainF446RE_V3`
  （**リポジトリ外**。`tools/fwsim/fw.py:33-34` の `SURGE_FW_DIR` の既定実体）
- Python 3.12 / numpy 2.5.1（`.venv/bin/python`）
- すべての再現は**リポジトリ外の一時ディレクトリ**（`/private/var/.../opencode/`）で行い、
  検証前後で `git status` が変わらないことを確認した。

### B.2 主な再現コマンドと出力

| 検証 | コマンド | 結果 |
|---|---|---|
| CI の exit code | `python -m pytest tools/tests tools/sysid/tests/test_link.py --collect-only -q` | `ERROR: file or directory not found` / **EXIT=4** |
| sim テストの赤 | `python -m pytest sim/tests -q` | **2 failed, 111 passed**（`0.006565553195410745 not greater than 0.08` / `np.float64(0.10564583079382661) not greater than 0.35`） |
| ML の収集衝突 | `python -m pytest ml_cam/tests ml_cam_e2e/tests --collect-only -q` | **4 errors during collection** / 99+126+100+31 = **356 件** |
| firmware 不在時の fwsim | `SURGE_FW_DIR=/tmp/nonexistent pytest tools/fwsim/tests/test_control_logic.py -q` | **3 failed, 28 skipped**（`TestParamTable` に skip が無い） |
| firmware あり時の fwsim | `SURGE_FW_DIR=<FW> pytest tools/fwsim/tests/ -q` | **45 passed, 45 subtests passed** / param_id 40 個一致 |
| `convert.py` の機械照合 | 16 定数 vs `packets.Telemetry.META` | **MISMATCHES: []**（36 項目すべて一致） |
| `protocol.toml` の scale 破壊 | scale を 0.001→0.01 に変更 → 再生成 → `--check` | **exit 0（合格）** → 実デコードが **10 倍ずれ** |
| `toml_io.apply` のモード | 一時コピーに apply | **0644 → 0600** |
| `Pure Pursuit` の FF 飽和 | `PursuitConfig(ff_gain=0/0.8)` で 30〜179° を評価 | ff=0 は 90° 以降 **−27.025°**（全舵）／ff=0.8 は 179° で **+0.035°** |
| MPC の `now=None` | epoch 8 回 + None 1 回 | 収束 −8.128° → **−27.025°**（18.9° 跳躍） |
| SLAM の `optimize_every` | oval 合成シーン、kf_dist=0.01 | `ValueError: poses must match keyframes`（keyframes=337 / graph_nodes=336） |
| SLAM の未一致 | 40 周後に全点飽和スキャン | `matched=False lost=False` / **keyframes 20 → 21** |
| ループ検出の脆さ | oval 2 周・σ=1cm+rel×距離 | rel 0/0.5/1% で **17→15→10**（ambiguity 0.9）／**35→35→33**（0.98） |
| 展開爆弾 | 20000×20000 uint8 | 圧縮 **390,371 B** → 展開 **400,000,000 要素**（381 MiB）／9.27 MiB → 9.3 GiB |
| `mapstore` の NaN | resolution=NaN で保存 | `list_maps()` に出る / `load_map()` は **None** / JSON が不正 |
| 時刻ドメイン | 再生 vs 記録元の monotonic | `cap_age = 338,092,574 ms`（約 94 時間）／`age = 0.0 ms` |
| 地図アップロード | 20000² / 100000² | 32 MB 上限内 despite 9.3 GiB 展開 |
| `sim` の停止 | 1 秒ストール | 物理 0.75 s 消失 / **100 フレームバースト** |
| バス転送 | 3.0 m/s² vs 2.29 m/s² | 差 0.71 m/s² が毎秒蓄積 |
| `purepursuit` uals | `kappa_max` 左右 | 左 +2.136119 / 右 −2.232137（**4.30%** 差） |
| `plan_budget_ms` | フォールバック実測 | 108 候補・**8.3 ms**（合計 160 ms < 200 ms） |
| `np.random.seed` | `torch.manual_seed(0)` 2 回 | numpy の出力が **相異なる**（未シード） |
| `min_curvature_alpha` | axle_model の式を評価 | rear **0.4637**（原主張 0.44） |
| **第2ラウンド** `protocol.toml` scale 破壊 | 一時コピーで scale を 0.001→0.01 | `generate.py --check` **exit 0** → `state.speed` **1.0**（仕様なら 10.0） |
| **第2ラウンド** fwsim 縦 accel | `Plant.load()` + `host_sim.c` のタイヤ式 | **3.43 m/s²**（元の「8.9」は 2.6 倍過大） |
| **第2ラウンド** `toml_io.apply` | 一時コピーに適用 | モード **0644 → 0600**（再確認） |
| **第2ラウンド** 展開爆弾 peak RSS | 20000² の npz を `validate_upload` | **peak RSS 4.73 GB**（受理は成功） |
| **第2ラウンド** `sim` ストール | 1 秒ストール | 物理 0.75 s 消失 / **100 フレーム**バースト |
| **第2ラウンド** 候補パス数 | `candidate_paths` を 900 ケース乱数サンプリング | **19〜42**（理論上限 72） |
| **第2ラウンド** 地図アップロード | 20000² / 100000² | 圧縮 **390,058 B** / **9,720,777 B** → 展開 **400,000,000** 要素 / **9.313 GiB** |
| **第2ラウンド** `toml_io` モード | 一時コピーに適用 | **0644 → 0600**（再確認） |

### B.3 検証で見つかった「原主張の誤り」の一覧

| 原主張 | 誤りの内容 | 正しい値 |
|---|---|---|
| fwsim の縦 accel 上限 5.17 m/s² | リポジトリ内にその値は存在しない | 理論値 約 8.9 m/s² |
| `axle_model` の rear 0.44 | 実測は 0.4637 | 0.4637（定性的な帰結は一致） |
| `convert.py` のスケール 15 個 / 31 個 | 実数は **16 個**。31 個という数（L36 系の「31 項目」）は `META` 全体と混同 | 16 個（照合 36 項目で不一致 0） |
| `TestScaleMetadata` は `test_msgs.py:481-493` | ファイル・行番号が誤り | `raspi/tests/test_proto.py:480` |
| planner 数 13（StandstillSteer の件） | どの数え方でも一致しない | `registry.py` の `PLANNERS` は **22 件**（drive 12 / sysid 10） |
| MPC が `nearest_index` を 2 回計算 | 実際は **3 回** | 3 回 |
| 契約 JSON 27 個 | v24 は未エクスポートで **26 個** | 26 個 |
| `ml_lidar` の停止距離 0.41 / 0.53 m | 式から再現しない | v=1: 0.4167 / 0.4969 m、v=2: 1.0167 / 1.3058 m |
| プレ unprecedence 0.0498 N·m 以上で常に ABS | **FALSE**（0.0498 では-abs しない） | 0.15 N·m が上限。0.0498 は v→∞ 極限値 |
| `CONFIG_ACK` は常に OK | 未知 id では `UNKNOWN_ID` | 「範囲クランプも `OUT_OF_RANGE` もない」は正しい |
| `slam2d/core/pipeline.py` | 実ファイルは **`slam2d/pipeline.py`** | パス誤り |
| `posix` の 500 ms 旧 pose | 500_000_000 ns = **500 ms**（原主張 100 ms） | 500 ms |
| `SignalConfig` の GUI 侧的場所 | `gui/src/store/ui.ts` ではなく **`gui/src/components/SettingsPanel.tsx`** | 場所誤り |
| `CmdOut` は 20 フィールド | 宣言は **19**（`cmd()` が付ける `t` を含めワイヤ上 20） | 19 / 20 |
| `Python 側に CmdOut 相当の型が無い` | **`DriveCmd` が実在**する | 問題は「JSON 形の写し」が生成されないこと |
| `/ws/map` の schema | `AutoMap` は `MIRRORED` に含まれるので間接保護あり | map チャネル自体に schema が無い |
| `firmware ツリーが無い` | **存在する**（`/Users/banbatakumi/GitHub/AutonomousCar/Program/MainF446RE_V3`） | 参照パスを誤った検証担当が 1 名 |
| `raspi/tests` は 1666 件 | **1667 件** | 1667 |
| `_cmd_ns` の初期値はループ開始時刻 | 初期値は **0**。`LINK_WAIT_S` 5 ms は poll 待ちの上限 | 機序は「ループ先頭採取の時刻」 |
| `MAX_MAP_UPLOAD_BYTES` 超過 | 32 MB 上限は**掛からない**（圧縮後 Wire 長） | 9.27 MiB → 9.3 GiB |

### B.4 検証で使わなかった手法

- **静的解析ツールによる検出**（ruff の複雑度、mypy 等）は使っていない。
  本レビューの主張は全て**個別の再現または機械照合**で裏付けた。
- **実機（Pi）での走行試験はしていない**。実機依存の項目は
  「コード上はこうなっている」という限界を明記した上で、firmware C ソースとの突合で代替した。
- firmware の `drive.c` / `steering.c` の突合は
  `tools/fwsim` 経由の `param_table()` と直接の C ソース参照で行った。

---

*本ファイルは 4 本の横断レビューの統合と 9 本の独立検証に基づく。
コードは変更していない（検証は一時ディレクトリでのみ実施）。*

---

## 6. 第3ラウンド — 実コードでの再検証と修正（2026-10-21）

High 24 件と、安全に関わる Medium の一部を実コード・実測で確かめ、事実なら直した。
**読み切れていない Medium/Low は未検証**（下記）。

### 6.1 直したもの（再現を確認 → 修正 → テスト）

| ID | 結果 | 内容 |
|---|---|---|
| VER-01 | 事実・修正 | `ci.yml` の tools ステップを `tools/check.sh` と同じ対象に。`sim/tests` と `ml_*` の job を追加 |
| VER-02 | 事実・修正 | `TestParamTable` に `skipUnless(HAVE_FW)`（firmware 無しで 3 件 fail していた） |
| VER-03 | 事実・修正 | `sim/tests/test_speed_ff.py`: 弱い PI の前提を明示、強い PI の下の期待値を再ベースライン（113 件通過） |
| VER-04 | 事実・修正 | `__init__.py` では解決せず（`dataset` のフラット import 衝突）。ディレクトリごとに別プロセスで回す形で CI と check.sh に追加 |
| SLM-01 | 事実・修正 | `optimize_every>0` が `ValueError`。最適化を `_sync_graph()` の後（`update()` 末尾）へ移した。`kf_dist=0.01` で再現するテストを追加（修正前は fail） |
| SLM-02 | 事実・修正 | 地図があるのに合わせる点が 0 の周は `lost` 扱い（DR 姿勢のキーフレームを焼かない）。テスト追加 |
| SLM-04 | 事実・修正 | 展開前に `.npy` ヘッダの形で上限（1600 万セル）。`np.isin` の一時配列も除去。トークン検査は設計（操縦権用）のため未変更 |
| SLM-05/09 | 事実・修正 | `save_map` が `_check_finite`（読み込みと共通）で拒否、メタ JSON は `allow_nan=False` |
| GEN-01 | 事実・修正 | `replay_node --bus` の時刻を、再生プロセスの単調時刻へ一定のずれで移す。**実バス経由のテストは無く未検証** |
| ML-01 | 事実・ローカル修正 | `models/e2e_lidar/*.json` 26 個に学習時の `dt`・`steer_tau` を `env_config.json` から追記（v13〜v20 はロード拒否になる）。`models/` は gitignore |
| ML-03/04 | 事実・修正 | `ml_cam`: 学習時の `input_size`・`preproc_version` を記録/使用、食い違いは例外、GUI の手入力欄を廃止。`ml_cam_e2e`: 版の食い違いは例外。**セグメ側ノード（`cam_perception_node`）の版照合は未追加** |
| NAV-14 | 事実・修正 | `planning_node` で停車 2 秒後に舵を中立へ（system_overview の記述どおりに）。`park_to_point`・sysid は除外。発進の指令は触らない |
| SAF-13 | 事実・修正 | 校正 1 が `hi` の範囲外も fail（校正 5 の同種の穴は未対応） |
| NAV-03 | 事実・修正 | GUI が `accel_limit=0` のとき planner の値を `speed_ramp_max_m_s2` で頭打ち |
| GEN-03/04/09 | 事実・修正 | コールバック 3 つの例外ガード、モード切替で `_last_plan_ns` をリセット＋`dt` 上限 0.5s、NaN 理由の無限連結 |
| ML-05 | 事実・修正 | `JobRunner.drain` がコールバックの例外で抜けない |

### 6.2 事実だが「直さない」と判断したもの

| ID | 理由 |
|---|---|
| SAF-01 | 2026-09-27 の明示的な設計（`test_cmd_path` に固定）。スライダを下げた場合の非常制動が弱くなる点は**仕様の決定**が要る。要判断 |
| SAF-02 | 未 ARM 経路も `side_brake:false` を明示的に送る既存設計。DISARM で外れるのは安全側。再 ARM 時の保持だけ別途確認 |
| SAF-14 | 実機の揺れ（20〜30Hz が常にある）を踏まえた相対判定で、`std>0.04` の絶対条件も併用。baseline→0 は仮説どまり |
| NAV-12 | 駐車で壁へ詰める意図の設計（実測の経緯がコメントに残る）。経路全体に効くトレードオフとして認識 |
| CFG-01/05/06 | 不整合は実在するが、値の正を決める設計判断（どの μ を正にするか）。fwsim の `dyn.get("mu")` はデッドコード |
| CFG-12 | `axle_model` は既定 False の opt-in フラグ。到達不能ではなくベンチ側で明示指定する使い方 |
| GEN-10 | 設計どおり（レビュー自身が「不要」） |
| GEN-20 | シムのストール保護（0.25 s 上限）は意図的。1:1 実時間ベンチの制約として記録のみ |
| SAF-08 | docstring と実装の食い違いは事実だが、arm-block 中に ARM を立てて制動する案は block の趣旨と衝突する。docstring 側の見直しが妥当 |

### 6.3 未検証

上記以外の Medium/Low（CFG-02〜04,13〜26 / NAV-01,02,04〜11,13,15〜24 / SLM-03,06〜08 / GUI-* / OPS-* /
VER-05〜13 / GEN-02,05〜08,11〜24 / ML-02,06〜12 / SAF-03〜12）は**読んでいない**。
H 24 件のうち実害として直したのは 12 件、設計/判断で保留が 7 件、
残りは上表の通りで、**第2ラウンドの「CONFIRMED」は事実の確認であって直すべきかの確認ではなかった**。
