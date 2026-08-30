# SURGE Mark.2 開発進捗

会話が圧縮されても文脈を失わないための作業ログ。**新しいセッションではまずこれを読む。**
設計の中身は `docs/` が正。ここには「今どこまでやったか」「なぜそう決めたか」の要約だけを書く。

最終更新: 2026-08-29（`ml/`を`ml_cam/`へ改名し、リポジトリ内の全参照
（`raspi/`のコメント・`docs/`・`.gitignore`・`gui/src/types.ts`等）を統一。
**`ml_cam/app.py`をモデル名ベースのGUIに作り替え**——①タブで決めた
「モデル名」1つを②③④タブが自動で使い回し、フレーム抽出→アノテーション→
学習→エクスポートの各タブでパスを手入力・参照する操作をほぼ無くした
（`ml_lidar/app.py`の「run名」と対称。既存の`ml/runs/latest/`は自動で
`ml_cam/runs/v1/`へ移行済み）。**⑤プレビュータブのモデル選択・
`ml_lidar/app.py`のrun一覧をどちらもドロップダウンに変更**（ログ欄の
縦幅確保が目的、バンビの要望）。**`ml_cam`にも備考機能を追加**——
`ml_lidar`と同じ`note.txt`→エクスポート時`<名前>.json`同梱→
実車GUI（`AutoPanel.tsx`のカメラモデル選択）表示、まで一気通貫。
`ml_cam`・`ml_lidar`のGUIを同時に起動できない制約は無いことを実測で確認
（元から無かった）。下記「2026-08-29（続き3）」節）
旧: 2026-08-29（学習runに自由記述の備考を付けられるように——
`ml_lidar/app.py`の備考欄→`note.txt`→エクスポート時に`<名前>.json`へ同梱→
実車のGUI（`AutoPanel.tsx`のE2E LiDARモデル選択）にも表示、まで一気通貫。
下記「2026-08-29（続き）」節）
旧: 2026-08-29（`e2e_lidar.py`に他planner（`de`/`ftg`）と同じ`steer_tau`
舵平滑化フィルタを追加。v1のTensorBoard学習曲線が早期収束後は改善していなかった
ことも判明。下記「2026-08-29」節）
旧: 2026-08-28（`train_rl.py`のバグ修正: 新規学習を同じrun名で繰り返すと
TensorBoardのサブフォルダ(`PPO_1`・`PPO_2`…)が積み上がっていたのを、
`--resume-from`無しなら`--out`を空にしてから作り直すよう修正。GUIのrun一覧から
`max_steer`表示も削除。`docs/progress_archive.md`の「2026-08-28（続き9）」節）
旧: 2026-08-28（`train_rl.py`に`--resume-from`（PPO.loadで途中から再開）を
追加し、`ml_lidar/app.py`から「続きから再開／上書き／キャンセル」を選べるように。
学習の「選択を停止」に確認ダイアログを追加、観戦の窓数もGUIから選べるように。
`docs/progress_archive.md`の「2026-08-28（続き8）」節）
旧: 2026-08-28（E2E LiDARの学習・観戦・エクスポート一式からも`max_steer`を
完全に排除し、`config/vehicle.toml`の車両物理限界だけを参照するよう統一。
`train_rl.py`/`ml_lidar/app.py`の`--max-steer`を削除。`docs/progress_archive.md`の「2026-08-28（続き7）」節）
旧: 2026-08-28（自動運転plannerの`max_steer`ParamSpecを全廃し、
`config/vehicle.toml`の車両物理限界を直接参照するように統一。RCモードだけは
従来通りGUIから調整可能なまま。`docs/progress_archive.md`の「2026-08-28（続き6）」節）
旧: 2026-08-28（`ml_lidar/app.py`——LiDAR E2Eの学習・観戦・エクスポートを
ターミナル無しで操作するTkinter GUIを新規追加。`ml/app.py`と対称。
`docs/progress_archive.md`の「2026-08-28（続き5）」節）
旧: 2026-08-28（`train_rl.py`が`--max-speed`/`--max-steer`を`run_config.json`に
記録するようにし、`export_onnx_rl.py`はそれを省略時に自動で読むように変更。
「エクスポート時にどの値を渡したか忘れる」事故を防ぐ。`docs/progress_archive.md`の「2026-08-28（続き4）」節）
旧: 2026-08-28（シム車両モデルに横方向グリップ限界(`mu*g`)を追加。
コーナーで速すぎるとアンダーステアで曲がりきれなくなる、という物理的な
必然性を持たせた。`docs/progress_archive.md`の「2026-08-28（続き3）」節）
旧: 2026-08-28（E2E LiDARの報酬設計を見直し、コーナーでのレーシングライン
（外→内→外）を阻害していた中心線ペナルティを道幅の余白付きに変更。
`docs/progress_archive.md`の「2026-08-28（続き2）」節）
旧: 2026-08-28（`ml/export_onnx.py` の `external_data` 未指定バグを修正し、
`cam_perception_node` を systemd 化（既定は無効）。`docs/progress_archive.md`の「2026-08-28（続き）」節）
旧: 2026-08-28（`steer_actual` −24°張り付き問題が解決。原因はSTM32側の
ステアモータ較正オフセットが駆動電源OFF時に乗っていたこと。`docs/progress_archive.md`の「2026-08-28」節）
旧: 2026-08-28（UART プロトコル v0.13 対応。`TELEMETRY` に TC の
スリップ率・トルク上限を追加、`COMMAND` にサイドブレーキ `flags2` を新設。
`docs/progress_archive.md`の「2026-08-28：UART プロトコル v0.13 対応」節）
旧: 2026-08-25（UART プロトコル v0.12 対応。`auto_stop` の判定を固定20cmから
動的停止距離＋LiDAR併用へ刷新、安全マージンを `CONFIG_SET` param_id 0x0060 で
3段階切替可能に。`docs/progress_archive.md`の「2026-08-25」節）
旧: 2026-08-23（zbus のトピック登録漏れを修正／中心線を壁の穴に頑健化／
`lidar_only` 実験モードを追加。`docs/progress_archive.md`の「2026-08-23」節）
旧: 2026-08-22（SLAM の未解決3点に着手。`docs/progress_archive.md`の「2026-08-22」節）
旧: 2026-08-21（コードレビュー13項目を全修正し実機へ反映。`docs/progress_archive.md`の「2026-08-21」節）
旧: 2026-08-30（本ファイルが1406行まで再肥大化したため、2026-08-21〜2026-08-28分の
詳細ログを`docs/progress_archive.md`へ再度移動。**情報は削除しておらず移動のみ**。
2026-08-14のSLAM棚上げ方針ブロックは現在地判断に直結するためそのまま本体に残した）
旧: 2026-08-17（3561行あった本ファイルを軽量版とアーカイブに分割。**情報は削除しておらず、
詳細な実験経緯・実測値はすべて `docs/progress_archive.md` に移した**。分割前と同じ内容が
アーカイブに残っているので、根拠が要るときはそちらを見ること）

方針の最新は 2026-08-14 の **★ SLAM を一旦棚上げし、非SLAM の Disparity Extender で進める**（下記）。
**2026-08-22 にバンビの指示で SLAM 側の修正を再開した**（`docs/progress_archive.md`の「2026-08-22」節）が、
`de` を本線とする方針そのものは変わっていない。

---

## ★ 2026-08-29（続き3）：`ml/`→`ml_cam/`改名・モデル名ベースGUI化・run一覧ドロップダウン化・カメラ備考機能

バンビからの4件の要望に対応:

1. `/ml`を`/ml_cam`という名前に変更。他に`ml`だけを指している箇所も統一
2. `/ml_cam`と`/ml_lidar`のGUIアプリが同時に開けないように見える件の調査
3. `/ml_cam`のモデル作成時に`/ml_lidar`のようにモデルごとに名前をつけ、パス指定の
   操作を減らす（フレーム抽出から作るモデルは基本1個、という前提で実装できるはず）
4. `/ml_lidar`のrun一覧をリストからドロップダウンに変更（ログ欄の縦幅確保のため）。
   `/ml_cam`でモデルを選ぶGUIを実装するときも同じくドロップダウンにする
5. `/ml_cam`の各モデルにも`/ml_lidar`と同じく備考を書けるようにする

**1. 改名**: `git mv ml ml_cam`後、`ml_cam/`配下の全`.py`/`.command`/`.txt`内の
`ml/`表記を`ml_cam/`へ一括置換。加えて`raspi/nodes/cam_perception_node.py`・
`telemetry_node.py`・`msgs/types.py`・`docs/development.md`・
`gui/src/types.ts`・`.gitignore`（`surge_mk2/ml/data/`・`surge_mk2/ml/runs/`）
のコメント・パスも全部`ml_cam/`に統一。**`.gitignore`のパターン自体がズレて
`ml_cam/data/`・`ml_cam/runs/`が追跡対象になりかけた**のを`git check-ignore -v`で
検出・修正（過去に同じ理由で事故った経緯が`.gitignore`のコメントに残っている）。

**2. 同時起動できない件**: 実際に両GUIを同時起動して確認したが、**コード上に
単一インスタンス制約は無く、問題なく同時に動いた**（プロセスもポートも独立、
共有ロックファイルの類も無し）。バンビの体感した「開けない」は再現せず、
現状のまま特に対処不要と判断。

**3. モデル名ベースのGUI**（`ml_cam/app.py`を大幅書き換え）: `ml_lidar/app.py`の
「run名がそのまま学習出力先とモデル名になる」設計を移植。

```
ml_cam/runs/<モデル名>/frames/     … ①フレーム抽出の出力・②③の入力
ml_cam/runs/<モデル名>/best.pt     … ③学習の出力・④エクスポートの入力
ml_cam/runs/<モデル名>/note.txt    … 備考（①タブで書ける）
models/<モデル名>.onnx             … ④エクスポートの出力（実車GUIが選ぶ場所。
                                      cam_perception_nodeがフラットに探す前提
                                      に合わせ、ml_lidarのようなサブディレクトリは切らない）
```

①タブの「モデル名」欄（`ttk.Combobox`、既存名を選ぶか新規名を打てる）を
②③④タブが`tk.StringVar`のトレース経由で自動的に使い回す。同じ名前で
フレーム抽出を再実行すれば既存フレームに追加される（`extract_frames.py`は
元々`manifest.csv`に追記していく設計なので変更不要だった）。既存の
`ml_cam/runs/latest/`（改名前の共有フレーム置き場からの唯一の学習成果、
val_iou=0.874）は`ml_cam/runs/v1/`へ自動で移行した（壊れていた旧
`model.onnx`——`external_data`未指定バグ修正前にエクスポートされたもの
——は引き継がず、④タブから再エクスポートする前提でnote.txtに書き残した）。

**4. ドロップダウン化**: `ml_lidar/app.py`のrun一覧を`tk.Listbox`から
`ttk.Combobox(state="readonly")`に変更（表示文字列は`format_run_row()`のまま、
選択中run名から`self._runs`を逆引き）。`ml_cam/app.py`⑤プレビュータブの
モデル選択も、`models/`直下の`.onnx`一覧を`ttk.Combobox`（`postcommand`で
開く直前に最新化）にし、パスの手入力・参照を無くした。

**5. カメラモデルの備考機能**: `ml_lidar`の備考機能（2026-08-29の1つ前の節）と
全く同じ配線をカメラ側にも通した。

```
ml_cam/app.py（①タブの備考欄・保存ボタン）
  → ml_cam/runs/<モデル名>/note.txt
  → export_onnx.py（--checkpointと同じディレクトリのnote.txtを自動で読む）
  → models/<名前>.json の "note" フィールド
  → raspi/nodes/telemetry_node.py の _cam_models_list()（cam_model_files に note を追加）
  → gui/src/components/AutoPanel.tsx（カメラモデル選択の下に選択中モデルの備考を表示）
```

**変更したファイル**（1〜5まとめて）:
- `ml_cam/app.py`: モデル名ベースのGUIに書き換え。`list_model_names`/
  `next_model_name`/`describe_model_status`/`read_note`/`write_note`を追加、
  `new_run_dir_str`は削除
- `ml_cam/export_onnx.py`: `export()`に`note`引数、`main()`が
  `--checkpoint`と同じディレクトリの`note.txt`を自動で読む（`ml_lidar`と同型）
- `ml_lidar/app.py`: run一覧をCombobox化
- `raspi/nodes/telemetry_node.py`: `_cam_models_list()`が`note`を返すように
  （`_e2e_models_list()`と対称）
- `gui/src/types.ts`・`AutoPanel.tsx`: `CamModelFile.note`追加、
  カメラモデル選択の下にも備考を表示
- `.gitignore`・`docs/development.md`・`raspi/`各所のコメント: `ml/`→`ml_cam/`

**テスト**: `ml_cam/tests/test_app.py`を全面書き換え（モデル名・備考の
新規テスト追加、`new_run_dir_str`関連は削除）、`test_export_onnx.py`に
備考往復テスト追加、`raspi/tests/test_telemetry_node.py`に
`TestCamModelsListNote`（4件、`TestE2eModelsListNote`と対称）を追加。
`./tools/check.sh`（生成物一致・`raspi/tests`536件・`tsc -b`）、
`ml_cam/tests`83件、`ml_lidar/tests`68件、全部green。

---

## ★ 2026-08-29（続き2）：ヘアピン/S字での衝突を診断→学習コース生成器にヘアピン対応を追加

バンビ「v3(E2E LiDAR)は長方形のようなコースでは壁に衝突せずに走れるが、
ヘアピンやS字があるコース（シムの`fuji`）でだけ壁に衝突する」と相談。

**前提（このセッションより前、`docs/progress_archive.md`未反映だがmemoryに記録済み
の一連の作業）**: v2の学習曲線を実測解析し方策崩壊パターンを発見→PPO安定化
（`target_kl`・`n_epochs`減・学習率減衰）・`steer_tau`/`steer_rate_weight`追加・
`sim.bench --model`の穴埋め・`watch.py`を学習環境に同期、まで済ませ、続けて
`sim/random_course.py`を書き直して**生成コースが車両の物理限界(R_min≈0.40m)を
超えるコーナーを作っていた**バグを修正し、コミット`b2fd326`で**生成コースの
周回方向が反時計回り固定**だったバグも修正した（v3学習時点ではこれらは未反映）。
その際「ヘアピン対応は次回」と保留していた。

**診断結果**: `sim/random_course.py`の`_filleted_polygon_xy`（コース生成の主関数）
は「1つの中心から見た半径の関数」として多角形を作る方式で、**頂点の方位変化が
原理的に最大60°程度で頭打ちになる**ことを数値実験で確認した（半径をどれだけ
深くえぐっても、隣接頂点の角度間隔で決まる上限を超えられない——極座標表現の
構造的限界）。つまり**学習コースには一度もヘアピンが存在しなかった**——
`fuji`の`arc,1.0,-90`×2連続のような急な切り返しは分布外（未経験）の形状であり、
v3が汎化できなかったのは必然だった。S字（`_add_chicanes`のシケイン）も実在は
していたが、実際は片側にしか曲がらない「C字」で、docstringの「S字」という
記述と実装が食い違っていた上、`fuji`の実際のシケイン（半径0.5m）より緩かった。

**実装した対策**（`sim/random_course.py`、詳細はモジュールdocstring参照）:
1. `_hairpin_polygon_xy()`を新設。ヘアピン部分だけは fuji と同じ「90°ずつの
   アークを直線を挟まず2つ連続」で直接組み立て（1頂点で180°近くを一気に
   折ろうとすると接線長`R*tan(|δ|/2)`が発散し現実的な辺長では作れないため）。
   残りの頂点は「turnを先に決め、辺長を最小二乗で解いて閉じる」新方式
   （`_filleted_polygon_xy`の「極座標だから自動的に閉じる」保証を使えないため）。
   自己交差検査(`_polygon_is_simple`)・外接ボックスの異常サイズ検査つき。
   単発成功率は実測1.5%と低いため`_HAIRPIN_MAX_ATTEMPTS=300`で98%まで確保
   （1回あたり約0.03msと軽いので許容）
2. `_add_chicanes()`を書き直し、`sin(pi*t)`（片側だけ膨らむC字）から
   `sin(2*pi*t)`（両側に膨らむ真のS字）に変更。目標半径をfuji並み
   （`min_radius_m*1.05`倍まで）許容するようタイト化
3. **★実装中に踏んだ罠（重要）**: 上記2の変更後、生成コースの9割以上が
   最小旋回半径違反になる回帰が発生。原因は「窓幅の安全マージン公式は
   まっすぐな基準線に蛇行を乗せる前提で導出したが、実際には既にフィレットで
   丸めた頂点の近傍に蛇行を挿入すると**base曲線側の既存曲率と蛇行の曲率が
   加算されて**合成半径が目標の半分以下に落ち込む」こと（複数シケインの窓が
   重なる場合も同じ理由で加算される）。`_add_chicanes`は挿入前にbase曲線側の
   既存曲率を調べ、実質直線とみなせる区間（半径`3*min_radius_m`超）にだけ
   蛇行を置くよう修正——窓の重複も避けるようにした。修正後は300コースで
   違反0件（修正前は96.7%が違反）

**検証方法**: matplotlibが`.venv`に無かったため、PIL（Pillow、既存依存）で
占有格子とセンターラインをPNGにレンダリングして目視確認した（コース全体図＋
ヘアピン単体のズーム図）。ヘアピンは半径を通常コーナーと同じレンジ
（`min_radius_m`の1.0〜2.5倍）にすると、コース全体のスケールに対して
「大きく優雅に曲がる」だけに見えてヘアピンらしい鋭さが視覚的に消える問題も
見つけ、`_HAIRPIN_RADIUS_MULT=(1.0, 1.3)`とタイトな専用レンジに絞って解決した。
プログラムでも「3m以内の弧長で総回頭角100°超」を検出する関数で確認し、
`hairpin_prob=1.0`のとき20コース中ほとんどで130〜170°の急旋回を確認できた。

**テスト**: `ml_lidar/tests/test_random_course.py`に4クラス追加
（`TestHairpinPolygon`・`TestGenerateRandomCourseHairpin`・
`TestGenerateRandomCourseChicaneRadius`——上記3のバグの回帰テストも含む）、
計12件全green。`ml_lidar/tests`全ファイル・`raspi/tests`533件も確認済み
（既知のフレーキーテスト`test_cam_perception_node`1件を除き全green、
今回の変更と無関係）。生成コストは1コースあたり平均5〜20ms程度
（実測、`_MAX_GEN_ATTEMPTS`retryの発生頻度による）。

**How to apply**: これも推論側（`raspi/auto/e2e_lidar.py`）は無関係——学習コース
生成側だけの変更なので、**v5として再学習しないと効果が出ない**（v3/v4の
`.onnx`のまま試しても改善しない）。`hairpin_prob`（既定0.35）は
`generate_random_course()`の新しい引数——`train_rl.py`・`watch.py`とも
デフォルト値をそのまま使うので追加のCLI変更は不要。

---

## ★ 2026-08-29（続き）：学習runに自由記述の備考をつけられるように（Pi実車のGUIまで到達）

バンビ「学習のモデルにGUIから備考をつけられるようにしたい。どんな変更をしたとか、
どのコースかとか書きたい」→「E2EのGUIからも書けるってこと？全体のGUIからその
備考を見ることができるようにできる？」という要望に対応。**ml_lidar側の入力から
実車のGUIでの表示まで一気通貫**で作った。

**データの流れ**:
```
ml_lidar/app.py（run一覧タブの備考欄・保存ボタン）
  → ml_lidar/runs/<run名>/note.txt（プレーンテキスト。run_config.jsonとは別ファイル）
  → export_onnx_rl.py（--modelと同じディレクトリのnote.txtを自動で読む。新規フラグ不要）
  → models/e2e_lidar/<名前>.json の "note" フィールド
  → raspi/nodes/telemetry_node.py の _e2e_models_list()（e2e_model_files に note を追加）
  → gui/src/components/AutoPanel.tsx（E2E LiDARモデル選択の下に選択中モデルの備考を表示）
```

**変更したファイル**:
- `ml_lidar/app.py`: `read_note()`/`write_note()`（`<run_dir>/note.txt`の読み書き）。
  run一覧タブに複数行テキスト欄＋「備考を保存」ボタンを追加。runを選ぶたびに
  読み込み直す（学習前でも後でも、いつでも書ける）
- `ml_lidar/export_onnx_rl.py`: `export()`に`note`引数を追加、`.json`へ書く。
  `main()`は`--model`と同じディレクトリの`note.txt`を自動で読む
  （`run_config.json`の自動読み込みと同じ流儀。新しいCLIフラグは無し）
- `raspi/nodes/telemetry_node.py`: `_e2e_models_list()`が各モデルの`.json`から
  `note`を読んで`e2e_model_files`に含める。`.json`が壊れていても一覧全体は壊さない
- `gui/src/types.ts`: `E2EModelFile`に`note: string`を追加
- `gui/src/components/AutoPanel.tsx`・`styles.css`: E2E LiDARモデル選択の下に、
  選択中モデルの備考を表示する行を追加（`.auto-model-note`）

**テスト**: `ml_lidar/tests/test_app.py`に`TestNote`（3件）、
`ml_lidar/tests/test_export_onnx_rl.py`に備考の往復テスト（1件）、
`raspi/tests/test_telemetry_node.py`（新規ファイル、4件——`_e2e_models_list()`は
`self`を使わないメソッドなので`TelemetryServer._e2e_models_list(None)`のように
未束縛のまま軽量にテストできる。既存の`TelemetryServer`本体には他にテストが無い
ので、初めての専用テストファイル）。GUI側は`tsc -b`と`vite build`が通ることを
確認（テストフレームワークが無いプロジェクトのため）。`raspi/tests`533件・
`ml_lidar/tests`（軽量分）とも全green。

---

## ★ 2026-08-29：`e2e_lidar.py`に舵の平滑化フィルタを追加（v1の不安定挙動の診断）

バンビから「v1の学習も終わったが、舵角の決定が少し不安定なように感じる。学習不足・
学習時の遅延固定・報酬設計、何が原因か」と相談があった。`ml_lidar/runs/v1/`の
`evaluations.npz`を実際に見て診断した。

**わかったこと**:
- `run_config.json`: `max_speed=2.0`（bambiが既定1.5から引き上げていた）・
  `max_steer=0.524`・`timesteps=2000000`
- 学習は`timesteps=1,260,000`（目標の63%）で早期終了していた
- 評価reward（circuit/fuji）は最初の数万stepで急上昇したあと、以降63回の評価まで
  **30〜120の間でノイジーに上下するだけで、明確な右肩上がりの改善が無い**
  （早期終了は「10回連続で更新されなかった」ため作動——これと整合する）
- ep_lengthも350〜1500の間で評価ごとに大きくブレる＝**同じモデルでも試行によって
  挙動が大きく揺れる**状態だった

**原因（構造的な見落とし、修正した）**: `raspi/auto/e2e_lidar.py`は他の全planner
（`de`・`ftg`・`dp`・`line_trace`）が持っている`steer_tau`（舵指令の1次遅れによる
平滑化フィルタ）を**一つも持っていなかった**（`reset()`に「平滑化などの内部状態は
持たない」と明記されていた）。LiDARスキャンのフレーム間の微小な揺らぎが、フィルタ
無しでそのままステア出力の揺らぎになっていた。他plannerと同じ`ParamSpec(key=
"steer_tau", ...)`・`self._steer`保持・1次遅れ適用を追加した（`__init__`/`reset()`/
`plan()`）。**再学習不要、推論側だけの修正**。

**まだ残っている、報酬設計とSim2Realの2つの懸念（保留・対応せず）**:
- 報酬に舵の変化量（ジャーク）を罰する項が無いため、モデル自身に「滑らかにする
  動機」が無い——`steer_tau`はあくまで事後的なフィルタで根治ではない
- `config/vehicle.toml`の`[dynamics]`（`tau_steer_s`/`dead_time_s`）は未実測の
  固定値で、道幅・LiDARノイズと違いドメインランダム化されていない。実車の実際の
  遅延とズレていれば、実車でだけ不安定に見える可能性がある（シム/`watch.py`で
  見えている不安定さとは別原因）——ただし`watch.py`は`raspi/auto/e2e_lidar.py`を
  経由せずPPOモデルを直接叩くので、**今回追加した平滑化フィルタの効果は
  `watch.py`では確認できない**。`sim.bench --mode e2e_lidar`か実車でのみ効く

**テスト**: `raspi/tests/test_auto.py`の`TestE2ELidar`に3件追加
（`test_steer_is_smoothed_over_time`・`test_reset_clears_the_steering_state`・
`test_steer_tau_zero_disables_smoothing`）。入力に関わらず一定出力を返す
ダミーONNXモデル（`_make_constant_steer_e2e_model`）でフィルタの時間応答だけを
検証。`raspi/tests`529件、全green。

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
- ~~SLAM に戻るときの入口は「周回を検出するたびに `close_loop()` を走らせる」（未着手）~~
  → **2026-08-22 対応済み。** 上の「2026-08-22」節参照。ただし `sim.bench`・実車とも未検証

### 非SLAM で進めるときに無くなるもの／代わりの手

| SLAM があって初めてできること | 非SLAM での代替 |
|---|---|
| レーシングライン（アウトインアウト） | **無い。** 反射型は毎周期その場で決めるだけ |
| 先読みブレーキ（次のコーナーの形） | 見通し（`ext`）で近似。**原理的に劣る** |
| ラップ計測・周回数 | **ジャイロの累積回頭 ÷ 360°** で数えられる（地図不要） |
| 「経路から落ちた」の検出 | 無い。`stop_dist` と超音波 `auto_stop` が最後の砦 |

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

### UART プロトコル — 現在 v0.13（`docs/uart_protocol.md` が正）

`raspi/proto/protocol.toml` が実装上の唯一の定義で、そこから Python パーサと STM32 側 C
ヘッダを生成する（`python3 raspi/proto/generate.py`）。v0.4確定 →
v0.5（`COMMAND` LEN10→12・ブレーキ/灯火/ホーン拡張）→ v0.6（トルク直接指令、上限0.125→0.15N·m）→
v0.7（超音波 `auto_stop`、ワイヤ非破壊）→ v0.8（TC/TV 有効切替、`param_id 0x0010`/`0x0020`）→
v0.9（片輪浮き対策、`param_id 0x0050`。TC本体とは独立、2026-08-20）→
v0.10（`MAX_SPEED`/`MAX_ACCEL`/`MAX_STEER` の `param_id 0x0001-3` を廃止し STM32 側固定定数へ
一本化）→ v0.11（`LIMITS`(0x0A)/`LIMITS_REQ`(0x15) を新設。STM32側発、2026-08-21）→
v0.12（`auto_stop` を固定20cmから動的停止距離＋LiDAR併用へ刷新、安全マージンを
`param_id 0x0060` で調整可能に、2026-08-25）→ **v0.13（`TELEMETRY` に TC スリップ率・
トルク上限、`COMMAND` にサイドブレーキ `flags2` を追加、2026-08-28）**と進んだ。
**v0.8・v0.9 は実機で動作確認済み。v0.10〜v0.13 は STM32側実装済みだが実機での動作検証は
まだ（STM32側ドキュメント曰く「実機での動作検証は未了」）。**
各版の変更点・実車確認の経緯はアーカイブの該当節（`UART プロトコル v0.6` / `v0.7` /
`v0.12` / `v0.13` など）。

#### ★v0.11 `LIMITS`（車両の物理的な上限値。2026-08-21、`f10ed4a`でコミット済み）

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
- `f10ed4a`でコミット済み

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
- **oval が「渦巻き」になる＝ループ閉じが1周目に走らない（SLAM側）。**
  診断済みの3点（① 周回検出のたびに `close_loop()` を走らせる ② 探索範囲を専用に
  広げる ③ マッチの寄せ方を異方的にする）は **2026-08-22 に実装した**（上の
  「2026-08-22」節）。**`sim.bench`・実車とも未検証のまま**なので、oval で本当に
  直っているかはまだ確認できていない。直っていなければ `explore_laps=1` が回避策
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

1. **壁に穴が残る根本原因の対処**（`_END_BACKOFF` の再チューニング or 実車 LiDAR
   ノイズの実測。2026-08-23 の中心線の頑健化は対症療法でしかない。上の
   「2026-08-23」節参照）
2. **中心線の急カーブ耐性のさらなる改善**（`_MAX_SHIFT_M` はテレポートを
   防ぐだけで、シケイン内の測定精度そのものは上がらない。`fuji` コースで
   1〜2点の残留を実測済み。法線推定の頑健化 or EDT ベースへの置き換えが
   次の一手。上の「2026-08-23」節参照）
3. **BUILD→RACE 切り替え直後の一時的な自己位置誤差（30〜58cm）の原因調査**
   （2026-08-22 の `sim.bench` で oval・circuit 両方に出た。「RACE 段が壁へ接触
   して固着する」と同じ Pure Pursuit 遅延補償が疑わしい。oval/circuit は
   sim.bench で確認済みなので残るは split の再実測とこれ）
4. **Disparity Extender を車輪を浮かせて実車検証**（`--max-speed 0.2` 程度から）
5. GUI カメラ 14fps の切り分け
6. `[dynamics]`（操舵のむだ時間・1次遅れなど）の実測
7. 屋外用ルーターの SSID を研究室で登録

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
