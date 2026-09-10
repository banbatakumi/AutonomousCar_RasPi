# SURGE Mark.2 開発進捗

会話が圧縮されても文脈を失わないための作業ログ。**新しいセッションではまずこれを読む。**
設計の中身は `docs/` が正。ここには「今どこまでやったか」「なぜそう決めたか」の要約だけを書く。

最終更新: 2026-09-10（ml_lidar: v12(gSDE+log_std_init=-3)を検証——学習崩壊は
解消したが舵の発振自体は未解決。行動空間を舵角→舵角速度に変える方針へ）
（v12は`collision_rate`0〜1%・`mean_reward`349〜365と全run中最良（v11の崩壊は
完全解消）だが、`vehicle.steer_actual`平均|Δ|は0.030〜0.042radでv7(0.023〜0.034)・
v9(0.029〜0.041)とほぼ同水準——劇的な改善にはならなかった。バンビが実際に
シムで観戦し「v9と同じくらいの性能に戻った感じだが、まだ発振が気になる」と
確認（2026-09-10）——**報酬に罰則を足す(v8〜v10)・探索ノイズの構造を変える
(v11〜v12)という2つの間接的アプローチはどちらも決定論的方策の滑らかさを
実質的に改善できなかった**、という結論に至った。

**次の方針: 行動空間を「舵角そのもの」から「舵角の変化率」への再パラメータ化**。
方策の出力`a[0]`を目標舵角ではなく舵角速度とし、積分して舵角にすることで、
罰則や探索ノイズのように「学習がその方向に向かうことを期待する」間接的手段では
なく、**1ステップで動ける量そのものに構造的な上限をかける**。調査中に判明した
副産物: `sim/vehicle.py`の`VehicleSpec.steer_rate_limit_rad_s`
（`config/vehicle.toml`実測値10.8rad/s、`VehicleModel.step()`内で既に適用済み）
という「アクチュエータの物理的な最大角速度」の仕組みが既存だが、1ステップ
(0.05s)で最大0.54rad(≈全可動域)動けてしまうほど緩く、今回観測している
0.03〜0.08rad/step程度の振動を抑える効果は無い（既にこの制限込みでの実測値が
上記の数字）。新設する行動空間側のレート上限はこれよりずっとタイトな
RL専用パラメータになる見込み。

**実装スコープ（未着手）**: `ml_lidar/env.py`（行動→物理量変換の作り直し）・
`raspi/auto/e2e_lidar.py`（実車推論、対になる後処理を揃える必要あり、
`plan()`内`steer_norm→steer`変換部分）・`export_onnx_rl.py`（モデル契約に
新パラメータを追加）を揃えて変更する必要がある、これまでより大きめの変更。
**行動空間の意味が変わるため既存モデル(v1〜v12)からの再開は出来ずゼロから
再学習が必要**。バンビへ実装着手の可否を確認した直後にセッション終了となった
ため、**次セッションはここから再開すること**——まず具体的な設計（レート上限の
値・積分後の扱い・既存`steer_tau`フィルタとの関係・観測`steer_norm`が何を
指すか）を合意してから実装に入る。
（v11(`--use-sde --sde-sample-freq 4`、他はv7と同条件)を検証したところ、
`vehicle.steer_actual`平均|Δ|が0.057〜0.082rad——**v7(0.023〜0.034)はおろか
最悪だったL1罰則v10(0.048〜0.082)すら上回る最悪の結果**。`collision_rate`
8〜21%（v7は2〜5%）・`mean_reward`280〜310（v7は340〜360）と学習の収束自体も
悪化。TensorBoardで`train/approx_kl`が学習序盤(2万step時点)で**1.62**
（v7は0.0046、350倍超）・`train/clip_fraction`が**0.95**（ほぼ全更新がクリップ）
という異常値を確認し原因を特定: SB3のgSDE実装は探索ノイズを
`exploration_matrix`（方策側MLPの潜在層 latent_dim_pi、既定64次元 × 行動次元、
各要素`N(0,exp(log_std_init)²)`）と方策特徴量の内積で作る。`log_std_init`の
既定値0.0（gSDE無しの2次元行動ノイズ用に想定された値）のまま使うと、
64項の独立な和で実効標準偏差が理論上`sqrt(64)=8倍`に膨れ上がり、行動空間
`[-1,1]`に対して学習序盤から桁違いに大きい探索ノイズがかかって方策が
崩壊したと判明（gSDE論文自身が指摘する「元祖SDEは状態次元が大きいと分散が
膨らむ」問題が、状態の代わりに使う特徴量側で再発した形）。**「gSDEが筋悪い」
ではなくチューニング不足だった**——`train_rl.py`に`--log-std-init`
（既定0.0、gSDE有効時は`exp(log_std_init)*sqrt(latent_dim_pi)≈1`となるよう
-3前後を推奨）を追加し`policy_kwargs`へ配線、`ml_lidar/app.py`にも
「探索ノイズ初期標準偏差(log)」入力欄を追加（既定-3）。実際に
`model.policy.log_std`が意図通り-3で初期化されることをスモークテストで確認、
`ml_lidar/tests`33件green。**未実施**: `--log-std-init -3`込みでのv12本学習・
検証）
旧: 2026-09-10（ml_lidar: `steer_rate_weight`はweight=0/0.03/0.1/0.2の
4点で単調悪化と確定、gSDEを`train_rl.py`に実装）（v9(0.03)・v10(0.2)を追加検証。
`vehicle.steer_actual`（シムが実際に描画する物理舵角）の隣接ステップ平均|Δ|は
0.023〜0.034(weight0)→0.029〜0.041(0.03)→0.037〜0.060(0.1)→0.048〜0.082(0.2)と
**4点ともきれいに単調悪化**し、`mean_speed`は全run同水準（罰則が支配的になり
保守的な方策に倒れた形跡は無い）——「重みを強めればいずれ強制的に滑らかになる」
という期待も外れ、**この方向（フィルタ前の生アクションへのL1罰則、環境報酬に
混ぜる方式）は重みに関わらず手詰まりと確定**。なお比較の途中でバンビの指摘
「v7だけsteer_tauが0.2で他と条件が違うのでは」を受けてtauを0.1/0.2両方に揃えた
対照実験も実施——v7が有利側に少し補正はされるが、weightに対する単調悪化の
順序自体は変わらないことを確認済み（v8/v9/v10はいずれも学習時steer_tau=0.1で
揃っている）。

**方針転換: gSDE（generalized State-Dependent Exploration、
[Raffin+2021, arXiv:2005.05719](https://arxiv.org/pdf/2005.05719)）を導入**。
報酬に手を入れる路線をやめ、探索ノイズの生成過程自体を変える——毎ステップ
独立サンプルのガウスノイズの代わりに、方策特徴量に依存し`sde_sample_freq`
ステップごとにしか再サンプリングしないノイズを使うことで、物理状態が連続的に
変化する限りノイズ自体も滑らかに変化する（数式:
`a_t = μ(s_t) + θ_ε・z_μ(s_t)`、`θ_ε`は`n`ステップごとに`N(0,σ²)`から再抽出）。
論文はRCカー実機を含む3種の実ロボットで直接学習に成功しており、PyBulletの
比較実験でも性能を犠牲にせず滑らかさ(continuity cost)を改善できることを
実証済み。`train_rl.py`に`--use-sde`/`--sde-sample-freq`（既定4、論文でPPOに
良好だった値）を追加しPPOへ配線、`ml_lidar/app.py`①学習タブに
「gSDEを有効化」チェックボックス＋間隔エントリを常時表示項目として追加
（`steer-rate-weight`の既定は0.03→0へ戻し、gSDE検証中は切り分けのため
0のままにするよう注記）。実際に`build_model()`→`model.learn()`のスモーク
テストで`model.use_sde`/`model.sde_sample_freq`が意図通り伝播することを確認、
`ml_lidar/tests`33件green。**未実施**: gSDE有効(`--use-sde`、`--sde-rate-weight 0`)
での本学習(v11)・v7〜v10と同じ`vehicle.steer_actual`比較での検証）
旧: 2026-09-09（ml_lidar: `steer_rate_weight=0.1`のv8がv7より舵振動が
悪化すると判明、重みを弱めて再挑戦する方針に）（バンビが実際に`steer_rate_weight
=0.1`でv8を再学習・シムで観戦し「v7より発振しているように見える」と報告。
`best_model_generalized`同士を同一シードで実測比較したところ、**生アクションの
標準偏差が全4評価シードで一貫してv7(0.41〜0.49)よりv8(0.53〜0.62)の方が大きく、
フィルタ後`steer`の平均|Δ|も約2倍(0.03→0.05〜0.07rad)に悪化**——目視観察が
正しいと数値でも確認された。**有力な原因仮説**: `steer_rate_weight*|a_t-a_{t-1}|`
というL1（絶対値）罰則は、隣接差分の総和が単調な補正では区間の刻み方に関係なく
「始点と終点の差」に等しくなる（テレスコープ和）ため、**方向転換（行き過ぎて戻る）
には効くが「一度に大きく切って据え置く」挙動は数学的に何も損しない**——実際
符号反転率(reversal rate)はv7とほぼ同じ(8〜13%)のまま標準偏差だけ増えており、
この仮説と整合する。ただしv7・v8とも学習シード1本ずつの比較なので、単一seedの
run-to-run分散である可能性は完全には排除できていない（過去のv11/v12の教訓）。

**外部調査で再確認**: RCカー規模の経路追従（arXiv:2401.05194）は`-m5|δ̇|`という
**L1**罰則を使っており、今回実装したものと同形——「二乗にすべき」という前回の
提案はCAPS(arXiv:2012.06644)の式を厳密に見るとユークリッド距離(スカラーではL1と
同次数)であり、**文献に直接の裏付けは無く、テレスコープ和の性質に基づく独自の
推論だった**と訂正済み。また「舵角そのもの」ではなく「舵角速度」をアクションに
する再パラメータ化は文献に実例があるが少数派の設計で、実車推論契約
(`raspi/auto/e2e_lidar.py`)まで変更が要る大きな構造変更のため保留。

**方針**: バンビの判断で、まず**重みだけを弱めて**（GUI既定値を0.1→0.03に変更
済み、`ml_lidar/app.py`）再挑戦する。0.03でも同様に悪化するなら、L1という罰則の
形自体（今回実装した「フィルタ前の生アクションへのL1」という具体形）に構造的な
問題がある可能性が高まり、二乗化またはアクション空間の再パラメータ化を検討する。
**未実施**: 重み0.03での再学習・検証）
旧: 2026-09-08（ml_lidar: v7の舵発散をバンビの目視指摘で再検証、v8向けに
`steer_rate_weight`を実装）（`ml_lidar/`は2026-09-06に最小構成（`progress`+
`collision_penalty`のみ）で作り直されており（`ML_LIDAR_V2_PROMPT.md`）、
v1〜v7はステア変化率への罰則を一切持たない設計だった。v7の`best_model_generalized`
を実測ロールアウトしたところ、決定論的方策でも生アクション`action[0]`が隣接
ステップ符号反転率8〜11%・平均|Δ|0.10〜0.12で常時振動しており（探索ノイズ込みだと
21〜36%・0.33〜0.42）、`steer_tau`一次遅れフィルタ後の`vehicle.steer_actual`は
滑らかに見えるだけと判明——**削除済み旧`ml_lidar`のv10〜v15で診断された「フィルタが
誤魔化せる程度まで実害を抑えるだけで根本原因（方策出力自体のノイズの荒さ）は
未解決」と全く同じパターンの再発**。外部調査でも、RCカー規模のRL経路追従
（[arXiv:2401.05194](https://arxiv.org/pdf/2401.05194)）やCAPS系
（[Mysore et al., arXiv:2012.06644](https://arxiv.org/pdf/2012.06644)、
gSDE論文[Raffin et al.](https://proceedings.mlr.press/v164/raffin22a/raffin22a.pdf)
もRCカー実機で検証済み）で、ステア変化率への罰則が標準的な対策と再確認。
ただしCAPSは「罰則が強すぎると過度に保守的な方策に倒れる」副作用も明記しており、
単一変数として重みを慎重に調整する必要がある。**v8向けに実装したもの**:
①`ml_lidar/env.py`の`EnvConfig`に`steer_rate_weight`（既定0、`steer_tau`フィルタ
**前**の生アクション`a[0]`の隣接ステップ差分に罰則、フィルタ後に掛けると同じ穴を
再度掘ることになるため）を追加、`train_rl.py`に`--steer-rate-weight`を配線。
②`eval_stats.py`の`run_episode`/`evaluate`に`mean_abs_steer_diff`・
`steer_sign_flip_rate`を追加（フィルタ後の値だけ見て「直った」と誤判定した過去の
反省を踏まえ、生アクションの滑らかさを定量的に追えるようにした）。
③`GeneralizationEvalCallback`の`generalization_evaluations.npz`にも同指標を記録
（旧run再開時の欠損キーはグレースフルに空扱い）。`ml_lidar/tests`33件green、
罰則の符号・大きさをランダム方策のスモークテストで直接確認済み。**v8学習の方針**:
`--steer-rate-weight`を0.1程度から単一変数で導入（他はv6設定に戻す——`steer_tau`は
v7の0.2ではなく既定0.10、`max_range`はv7の10.0のまま維持。`max_range`はノイズ対策と
無関係な観測レンジの整合性の話なので変更不要）。学習後は`mean_abs_steer_diff`・
`steer_sign_flip_rate`の低下と`collision_rate`/`mean_speed`の悪化有無を必ず両方
確認すること（CAPSの指摘通り、罰則が強すぎて速度が落ちるだけの保守的な方策に
倒れていないかのチェック）。**未実施**: v8本学習・検証そのもの）
旧: 2026-09-03（slam2d: 長距離ドリフト対策のフェーズ0・2（g2opy実機検証・
実車配線）完了）（Pi起動を確認しSSH接続、`.venv/bin/pip install g2opy`が即成功
（Pi5 aarch64 wheel、Python 3.13.5）、`slam2d/tools/spike_g2o.py`が実機で収束
（誤差5.51e-124）することを確認——「slam2dの依存はraspi実機に持ち込まない」
方針を転換し`raspi/requirements.txt`・`requirements.lock.txt`に`g2opy==2.3.0`
追加。`raspi/auto/_slam2d_nav.py`の`Slam2dNav`を`Frontend`直結から
`slam2d.pipeline.SlamSystem`（ループ閉じ込み）へ配線、`ENABLE_LOOP_CLOSURE`
キルスイッチ追加。**`sim.bench --course circuit`の実測で、`loop_batch_size`が
既定(10)のままだとEXPLORE走行中（10Hz呼び出し設計）に自動でバッチ最適化・
地図再構築が発火しplan()が最大1.2秒スパイクすると判明**——`1_000_000`（実質
無効化）にし、最適化・地図再構築は`freeze()`（EXPLORE→BUILD遷移、車両停止
済み）での明示的`flush()`だけに限定してスパイクを467msまで低減（残りは
広域探索自体のコストで既知の許容範囲）。`test_slam2d_raceline.py`に回帰テスト
3件追加——**追加の過程で既存テストヘルパーの円軌道の式に運動学的不整合
（進行方向と機体姿勢が90°ズレ）を発見**（短距離では影響しないが長距離だと
Frontendが早期に見失う。既存テストは変更せず、新規テストのみ正しい式に）。
`slam2d/tests/`+`raspi/tests/`合計739件green（既知の無関係な1件skip）。
**未実施**: フェーズ5（IMU・オドメトリのフル活用）、複雑コースでの一貫した
改善、実機での実走行確認）

旧: 2026-09-03（slam2d: 長距離ドリフト対策に着手。原因特定＋`core/confidence.py`
の情報行列推定を作り直し、oval基準ベンチマークで史上初めてループ閉じが
Frontend単体を上回った）（計画は
`~/.claude/plans/slam2d-slam-slam2d-imu-slam-slam2d-imu-nifty-aurora.md`。
ユーザー提示の「シケイン付き複雑コースで自己位置がズレる」画像を発端に着手。
**判明した主因**: `backend/`（g2opyポーズグラフ最適化＋ループ閉じ）は実装
済みだが、`raspi/auto/_slam2d_nav.py`が意図的に`Frontend`単体のみを使い
backendを一切呼んでいなかった（短距離ovalシムでの旧実測でループ閉じ有りの
方が悪化したため）。長いコースではこれが蓄積ヨードリフトを補正する手段の
完全な欠如になっていた。**フェーズ1（評価ハーネス、完了）**: シケイン・
ヘアピン付き複雑コースを3種`sim/courses/`に固定シード生成
（`make_random_courses.py`、procedural生成`sim/random_course.py`を流用）、
`sim/bench.py`に`--seed`とEXPLORE終了時点の誤差記録を追加、
`slam2d/tests/`に複雑コース用の高速ハーネス（`make_track_from_centerline`等）
と回帰テスト新設。**フェーズ3の診断（優先度変更）**: 当初懸念した「ループ
閉じが1周目に発火しない」問題は再現せず（3コース×3シード×4周で毎回
5〜7本検出）——PROGRESS.mdの当該記述は実は別実装(`raspi/nav/slam.py`)に
ついてのものだったと判明。**フェーズ4（情報行列の構造的改善、完了・成果
あり）**: `core/confidence.py`の旧手法（`grid.score_map()`の局所曲率、探索
安定化用に意図的にぼかした尤度場を微分）が、実測（オーバル直線区間・独立
ノイズ40回試行）で真の到達精度（位置ばらつき0.5cm）を約40〜50倍も過小評価
していると特定（差分刻み幅の問題ではなく尤度場の滑らかさ自体が頭打ちの原因）。
点対応ベースのFisher情報（点対平面＋点対点の混合、ICP共分散推定の標準手法）
に作り直し、`core/frontend.py`で「毎周期のライブブレンド用」（旧手法のまま
固定——新手法をそのまま使うと`info_pred`に対し`info_obs`が過大になり
oval3周のヨー誤差が0.85°→14〜65°へ悪化する退行を実測）と「キーフレーム＝
ポーズグラフのエッジ重み用」（新手法）を分離。結果、oval基準ベンチマーク
（3周・seed=7）でflush後ヨー誤差0.46°（Frontend単体0.85°を初めて上回る。
旧手法は常に1.3°前後で届かなかった）。複雑コースは3例中2例で改善・1例で
悪化と一貫性はまだ無い。`slam2d/tests/`全77件green。**未実施**: フェーズ0
（Pi実機でのg2opy動作検証、Piがネットワーク上に見つからず保留）、フェーズ2
（`_slam2d_nav.py`の`SlamSystem`化・実車配線）、フェーズ5（既存IMU・
オドメトリの追加活用）、複雑コースでの一貫した改善（パラメータチューニング
の余地））

旧: 2026-09-03（SLAM: 「レーシングライン走行」を押す前に保存済み地図を
下見し、クリックで自己位置ヒントを置けるように）（バンビの指示。「地図を選択
した時点で走行ボタンを押す前に地図を表示させ、自己位置を指定できるように」。
新規`GET /maps/<name>/preview`（`telemetry_node._serve_map_preview`）が
保存済み地図を`/ws/map`と同じ`AutoMap`msgpack形式で返す——`GUI`側
`ws/map.ts`の`build()`（展開・着色）をそのまま再利用できるよう`export`に
変更。新規`bus/mapPreview.ts`（`live.ts`と同じ「Reactの外の可変オブジェクトを
rAFが直接読む」流儀）が取得・保持を担当し、`MapCanvas.tsx`は
`mapPreview.data`が非nullの間はそちらを描き自車・軌跡・障害物は出さない
（対応する実測が無いため）。`SlamRaceButtons`（`AutoPanel.tsx`）のドロップ
ダウン選択・DONE段での自動選択のたびにプレビューを取得し、押した瞬間
（地図作成/レーシングライン走行どちらでも）に消して本物のライブ地図へ譲る。
クリックで置いた自己位置ヒントは`ch.setLocateHint()`（既存、plannerに記録
済みで`LOCATE`で使われる）に加え`setPreviewHint()`で地図パネル上に十字
マーカーを描画（クリックしても場所が見えないと「任意の場所から始められる」
意味が無いため）。副次的に、`vite.config.ts`の開発サーバプロキシに`/maps`が
無く`GET /maps/<name>`ダウンロードが開発中は404だったバグも発見・修正
（`/logs`と同じ扱いに揃えた）。`raspi/tests`（`test_telemetry_node.py`に
`TestServeMapPreview`追加）730件・GUI `tsc -b`/`npm run build`とも
green確認済み（`test_cam_perception_node.py`の1件のみ失敗——`cam_perception_node`
関連で今回のSLAM作業とは無関係、変更ファイルの差分もゼロなので触れていない）。
**未実施**: 実機での下見表示・クリックでのヒント配置・実際のLOCATE収束の
目視確認）

旧: 2026-09-03（ml_lidar: v14(gSDE)を停止しv15方針を決定）（v14を3M中
1.4M(47%)まで走らせて経過を追跡した結果、**gSDEは収束を大幅に遅らせるだけで
なく頭打ちしていた**——同じ1.4Mステップ時点でv13(gSDE無し)は既にreward684・
ep_len1500(完全収束)だったのに対し、v14は直近15回の評価でreward150〜280・
ep_len300〜760で上昇が止まっていた（watch.pyでバンビが「発散していそう」と
見た目視観察と一致）。残り53%のスケジュールで追いつく見込みが薄いと判断し、
学習プロセス（PID 24534）を停止した。**v15方針**: gSDEは保留し、代わりに
①`--curriculum-frac 0.3`を再導入（v13で判明したnarrow/obstacle汎化の弱さへの
対策、下記「2026-09-02（さらに続き）」節で計画済みだったもの）、②`sim/gym_env.py`
の`steer_rate_weight`（生の方策出力の振動を直接罰する既存の重み、現状0.2は
「steer_rate_weightと同程度のオーダーで見積もっただけ」の未検証値だった）を
0.2→1.0に引き上げ、生アクションの振動そのものを報酬側から直接抑える方向に
切り替える。他の設定（`--early-stop-patience 0`・`--timesteps 3000000`・
`--no-reward-norm`）はv13から変更しない。今回は2変数を同時に変える
（curriculum・steer_rate_weight）——両者は別々の問題（コース汎化 vs
出力の滑らかさ）に対する独立した対策で干渉しにくいと判断し、計算コスト
（1回3M stepの学習に十数時間）を踏まえ同時投入とした。結果が曖昧なら
次は1つずつ切り分ける）

旧: 2026-09-03（SLAM: DONE段でレーシングライン走行ボタンが消える不具合を修正）
（バンビが実機で試し、スクリーンショット付きで報告。「保存されたと出るが走行
開始ボタンが無い、自律走行解除を押すと出てくるが地図が一覧に無い」。原因は
`AutoPanel.tsx`の`auto-engage`セクションが`!auto.engaged`だけを条件に
`SlamRaceButtons`（2ボタン）を出し分けていたこと——直前に追加した`DONE`段
（バンビの指示でBUILD完了後は自動disengageせず待機するよう変更済み）は
engage済みのまま続くため、この条件では永久に単一の「解除」ボタンしか
出ない画面になっていた。`(!auto.engaged || st?.phase === 'DONE')`に条件を
広げ、`DONE`中でも2ボタン＋控えめな「解除」リンクを両方出すよう修正。
副次的に、`SlamRaceButtons`の一覧更新ロジックを「`DONE`遷移をuseRefで
エッジ検出」から「表示されるたびに（マウント時に）`mapsList()`を呼ぶ」に
単純化——前者は`DONE`中ずっとengage済みでコンポーネントが表示されない
という条件不備と絡んで、disengage後に再表示されても一覧が更新されない
（＝保存済みのはずの地図が選べない）症状の直接原因だった。ついでに
「走行中」バッジも`DONE`中は「待機中」に出し分けた（動いていないのに
「走行中」と出るのは紛らわしいとの指摘後の先回り修正）。GUI `tsc -b`/
`npm run build`のみ確認（Pythonは無変更）。**未実施**: 実機での
再現手順の再確認）

旧: 2026-09-03（SLAM: 実機フィードバックによる3件の追加修正）（バンビが
GUIで試して発覚した不具合を修正。(1)「地図名を入力しようとすると車の操縦キー
（WASD等）に奪われる」——`useDriving.ts`の`keydown`リスナーが`window`直付けで
フォーカス要素を一切見ていなかったのが原因。`INPUT`/`TEXTAREA`/`SELECT`/
`contentEditable`にフォーカス中は操縦キーを無視するガードを追加（E-STOPの
`Escape`だけは安全のため素通しのまま）。(2)「地図の保存ができない」——実は
(1)の直接の症状だった可能性が高いと判断（保存名に`w`/`a`/`s`/`d`を含むと
入力自体が奪われ`name.trim()`が空のまま保存ボタンが無効化され続ける）。
念のため`_maps_save`の保存可否ゲートも`RACE`段限定から`DONE`/`RACE`両方に
広げた（下記(3)参照）。(3)「地図作成後に自動でレーシングライン走行を
始めず、保存してボタン待ちにしたい」——`slam2d_raceline.py`に新段`DONE`を
追加し、BUILD完了時に自動ではRACEへ進まず`_auto_save_map()`
（`course_%Y%m%d_%H%M%S`名）で自動保存してから停止・待機するよう変更。
「レーシングライン走行」ボタン（既存の`request_load()`→`LOCATE`経路を
そのまま再利用）を押すまで、その場で車を動かし直してもよい。GUI側は
`SlamRaceButtons`が`DONE`遷移を検知して一覧を再取得し、できたばかりの
地図を自動選択する。既存テスト（円弧軌道でBUILDまで進める
`_drive_to_race`ヘルパ等）はRACE到達ではなくDONE到達＋
`request_load()`でLOCATEに進むことの確認に作り直した。
`raspi/tests`+`slam2d/tests`728件・GUI `tsc -b`/`npm run build`とも
全green確認済み。**未実施**: 実機での地図名入力・自動保存・
「レーシングライン走行」待機フローの目視確認）

旧: 2026-09-03（SLAM: 地図削除バグ修正・GUI再設計・地図の永続化・
グローバルローカリゼーションを実装）（バンビの指示で`slam2d_raceline`まわり
を5項目改修。(1)「地図を削除」が反映されないバグを修正——`_slam2d_nav.py`の
`Slam2dNav.reset()`が新gridの`seq`を0に戻し、GUI側の「古い版を上書きしない」
ガード（`ws/map.ts`）に弾かれていた。`raspi/nav/slam.py`と同じ「seqを単調
増加させる」パターンに直した。(2)独立タブ「地図生成」を廃止し、自動運転タブの
車体図の左に正方形パネル（`AutoMapPanel.tsx`、地図とグリッドのみ・数値非表示・
隅に削除ボタン）を新設。(3)地図データの永続化（`raspi/auto/mapstore.py`、
`saved_maps/<name>.npz`+`<name>.json`、3値のtrinaryだけ保存すれば
`OccGrid`を再現できると判明したので生のhits/missesは持たない）とMac⇔Pi間の
GUI完結アップロード/ダウンロード（`GET /maps/<name>`・専用WS
`/ws/map_upload/<name>`——このサーバのHTTP実装はPOSTボディを受け取れないと
実機コードで確認したため）。(4)「地図を作成」「レーシングライン走行」を
同一planner (`slam2d_raceline`) の2ボタンに分離（`SlamRaceButtons`）。
(5)保存済み地図から任意の場所でレーシングライン走行を始められるよう、新規
`LOCATE`段＋グローバルローカリゼーション（`slam2d/core/localize.py`の
`GlobalLocalizer`、地図の空きセル×粗い角度を全探索し尤度場でスコアリング）
を実装。左右対称コースでの誤認識対策として`ambiguous`判定を持つが、合成
テストでの実測で候補間隔やヒント半径のチューニングが誤検出率に直結すると
判明し、`spacing=0.2m`・`ambiguous_gap=0.1`・`hint_radius=0.4m`
（`ambiguous_min_dist`の半分未満に固定——ヒント後は原理的に誤検出しなくなる）
に調整した。副産物として、モノレポ直下`.gitignore`の`saved_maps/*.npz`等が
中間スラッシュを含むため`surge_mk2/`配下に効いていなかった実バグを発見・
修正（`surge_mk2/`prefix付きに変更、`git check-ignore`で検証済み）。
`raspi/tests`+`slam2d/tests`727件・GUI `tsc -b`/`npm run build`とも全green
確認済み。計画は
`~/.claude/plans/slam-slam2d-slam-mac-mac-mac-gui-groovy-waterfall.md`。
**未実施**: 実機（Pi5+実車）でのEXPLORE→BUILD→RACE→保存→DL→UL→選択→
レーシングライン走行の通し確認、LOCATE所要時間の実測、対称コース相当での
`ambiguous`検出の実地確認）

旧: 2026-09-03（ml_lidar: 「v13でも舵が発散」指摘を受け再検証しv14方針を修正）
（前回「v13でv10のbang-bang問題は解消」と判断したのは誤りだった。バンビが
watch.py・通常のシム画面の両方で発散を目視確認したのを受け衝突エピソードの
舵角トレースを追加確認したところ、`vehicle.steer_actual`（フィルタ後の実舵角）
は±0.3〜0.4rad程度に収まるが、**生の方策出力自体はcircuit/fujiも含め全ての
コースで同程度に激しく振動していた**（符号反転率37〜39%）——`steer_tau`+
`tau_steer_s`の二段フィルタがcircuit/fujiではたまたま実害を隠していただけで、
方策出力のノイズの荒さ自体はv10から変わっていなかった。narrow/obstacleでは
同じ振動が衝突・発散に直結する。対策として`train_rl.py`にPPOの`use_sde`
（gSDE、状態依存の相関探索ノイズでi.i.d.ノイズによる振動を抑える）を
`--use-sde`/`--sde-sample-freq`として追加し`app.py`のGUIにも配線
（テスト済み・green）。v14は`curriculum_frac`再導入より先に、v13設定+
`--use-sde`のみを追加した単一変数切り分けに変更。詳細は下記
「2026-09-02（さらに続き）」節の「再追記（2026-09-03、バンビ「v13でも舵が
発散しているように見える」指摘での再検証）」を参照）

旧: 2026-09-03（RasPi5節電: ARM/DISARM連動カメラ節電を実装）（バンビの
指示でラズパイ5（LiDARの電源はSTM32側で対策済み）の消費電力削減を複数
サブエージェントで調査。`raspi/setup/install_services.sh:180-234`の
2026-08-10 PMIC実測（カメラ2台1.15W=28%が最大の削減余地。CPUクロック低下は
race to idleで無効・HDMI遮断はPi5非対応（vcgencmd未実装）・Bluetooth無効化は
効果ノイズ以下、と既に検証済みで再提案しないことが分かった）と、Web外部調査
（WiFi power_saveはWi-Fi+BTレール計0.42Wしかなく監視用WebSocketのレイテンシ
悪化リスクの方が大きい）を踏まえ、ユーザー判断でWiFi省電力は見送り、カメラの
DISARM/ARM連動節電のみ実装した。既存の`cam/config`インフラ
（`_desired_front_fps`の「自動運転engage中は上限無視」という優先度上書き
パターン）がそのまま流用できると判明したため、`_vehicle_armed()`
（`vs.armed`、`VehicleState`がまだ届いていない起動直後はTrue＝ARM相当を
返す）で分岐する`_desired_front_fps`/新設`_desired_rear_enabled`に拡張。
`config/camera.json`を`front_fps_armed`/`front_fps_disarm`・
`rear_enabled_armed`/`rear_enabled_disarm`・`rear_fps_armed`のARM/DISARM
専用スキーマに変更（`camera_node.py`側は無変更で完結——FPS低下と後方カメラ
ON/OFFの既存機構をそのまま使うだけなので前方カメラの完全停止は今回スコープ
外）。GUI`SettingsPanel.tsx`にARM用/DISARM用の節電設定を別セクションで表示、
`ws/control.ts`/`types.ts`も新スキーマに追従。`raspi/tests/test_telemetry_node.py`に
armed/disarm/auto_engaged優先度のユニットテストを追加（`test_cmd_path.py`の
既存スタブも新スキーマに追従）。`raspi/tests`647件・GUI `tsc --noEmit`/
`npm run build`とも全green確認済み。計画は
`~/.claude/plans/5-lidar-stm32-5-5-disarm-arm-fps-ex-dis-peppy-feigenbaum.md`。
**未実施**: 実機でのDISARM/ARM切替時の`cam/config`配信値・GUI表示追従の
目視確認、`vcgencmd pmic_read_adc`による削減量の定量実測）

旧: 2026-09-02（SLAM再着手）（バンビの指示で、2026-08-14に棚上げした
SLAM（`raspi/nav/`）を「車体非依存の汎用2D LiDAR SLAMライブラリ」として
`slam2d/`に一から作り直した。将来のOSS公開・学習目的を兼ねる。方針決定は
複数回の外部調査（ROS2標準スタック・非ROS軽量ライブラリ・SLAM未解決問題への
対策）に基づく——結論は「フロントエンド（スキャンマッチ・占有格子・センサ
融合・ループ検出）は自作、バックエンド（ポーズグラフ最適化）は`g2opy`を借用」。
計画は`~/.claude/plans/wiggly-doodling-moth.md`。

**実装したもの**（`slam2d/`、テスト69件全green）:
- `core/`: `types.py`（Pose2D代数）・`grid.py`（占有格子）・`scanmatch.py`
  （相関スキャンマッチ）・`motion.py`（`MotionModel`抽象、車体依存センサ
  構成を差し替え可能に）・`deskew.py`・`scan2scan.py`（PLICP）・
  `confidence.py`（★新規: スキャンマッチの局所曲率を固有値分解し、oval等の
  コーナーのないループでの回転ドリフトを動的異方性ブレンドで抑制。3周の
  oval周回シムで実測: 固定比率ブレンドより一貫して誤差が小さいことを確認）・
  `frontend.py`（1周期処理パイプライン、ループ閉じは持たない）
- `backend/`: `posegraph.py`（g2opyラッパー）・`loop_detection.py`（汎用
  ループ検出、複数回の周回・8の字コースの再訪に対応）
- `pipeline.py`: `SlamSystem`（frontend+backend統合、`loop_batch_size`本
  ぶん溜めてから一括最適化。逐次処理は単一〜少数のループ拘束でむしろ
  精度が悪化することを実測で確認し、バッチ処理に変更した経緯は
  `backend/loop_detection.py`のdocstring「既知の限界」に詳しい）

**g2o-pythonではなくg2opyを採用**: 前者はEigen3等のC++依存が無いとソース
ビルドが失敗（実機確認）、後者はmacOS/Linux aarch64向けビルド済みwheelが
あり`pip install`だけで動く（Pi 5実機でのオンボード利用も見込める）。

**既存シミュレータへの統合**: `raspi/auto/slam2d_raceline.py`（新planner
`slam2d_raceline`）・`raspi/auto/_slam2d_nav.py`（`Scan`/`VehicleState`⇄
slam2d型のブリッジ）を追加、`registry.py`に登録。**既存の`raspi/nav/`
（`centerline.py`・`raceline.py`・`purepursuit.py`・`obstacles.py`)は
`OccGrid`に対するダックタイピングでコード変更ゼロのまま流用できた**。
`sim.run --course circuit`でGUIから「レーシングライン(slam2d)」を選択・
ARM・engageし、EXPLORE→BUILD→RACEまで約60秒で到達、速度1.20m/s・横偏差
-5cmで実走行することを実機シムで確認済み。ループ閉じ（`backend/`・
`pipeline.SlamSystem`）はこのplannerでは未使用（`Frontend`単体のみ、
`_slam2d_nav.py`のdocstring参照）。

**次にやること**: ①`backend/`のループ拘束がなぜ少数だと精度を悪化させるか
の根本原因（情報行列の異方性と実誤差分配パターンの不整合の疑い）をさらに
追うなら`core/confidence.py`の再設計が本命。②実車ログでの検証（Phase 7、
未着手）。③既存の`raspi/nav/`（自作SLAM、旧実装）とroom/oval等の合成
テストで精度を横並び比較すると、車体依存の細かいチューニング差分が見える
かもしれない（未着手）。旧SLAM実装・棚上げの経緯はそのまま下記に残す。）

旧: 2026-09-03（ml_lidar: v13を検証しv14方針を決定）（`--early-stop-patience 0`
（早期終了無効化）・`--timesteps 3000000`でA1+A2+reward_norm off+curriculum off
のv13を学習し直したところ、`clip_range`/`learning_rate`が最後まで線形減衰し切り、
`circuit`/`fuji`評価はreward755〜760・`ep_len`1500/1500で完全収束（v9の415〜445
から倍近い改善）。シムで`best_model`の実舵角（`vehicle.steer_actual`）を直接
確認し、v10の±1.0 bang-bang飽和が解消していることを確認——**v10の実車発散
問題はこの構成で解消したと判断**。**ただし**narrow/obstacle込みのフル分布から
ランダムに40コース生成し決定論的に評価したところcollision_rate=17.5%・
reward分散大（最悪5本は9〜48stepで即クラッシュ）——`circuit`/`fuji`固定評価
だけでは見えない、難コースへの汎化の弱さが残っている。v14は**reward_normは
引き続きOFF・curriculum_fracだけを0.3に戻す**単一変数の再導入とする。詳細は
下記「2026-09-02（さらに続き）」節末尾のv13追記を参照）

旧: 2026-09-02（さらにさらに続き）（v12の学習結果を検証。v11・v12は
`reward_norm`/`curriculum_frac`が同一設定（両方OFF）にもかかわらず結果が
食い違っており、原因は早期終了（`StopTrainingOnNoModelImprovement`）と
`learning_rate`/`clip_range`の線形減衰スケジュール（`--timesteps`固定5,000,000に
対する残り割合で計算）が噛み合っていないことと確定した。v11は5Mの16%（800k）、
v12は12%（620k）で早期終了しており、この時点で`clip_range`はまだ0.2→0.175程度
までしか減衰していない。対してv9・v10はそれぞれ72%（3.62M）・51%（2.54M）まで
回っており、`train/std`・`train/entropy_loss`の収束はその後半で起きていた。
つまりv11/v12は学習がまだ荒く探索的なフェーズで、ノイズの大きい評価reward
（30エピソードでも方策未収束の序盤は分散大）を根拠に`best_model`を選んで
しまっており、v9・v10との比較にもv11・v12同士の比較にも使えない。A1
（Conv1d初期化）・A2（むだ時間量子化）・reward_norm・curriculumのどれが
効いているかは、依然として未確定のまま。詳細・v13への具体的な方針は
下記「2026-09-02（さらに続き）」節）

旧: 2026-09-02（さらに続き）（6並列サブエージェントでコードベース全体の
バグ・最適化調査→3並列で再検証→Plan agentで実装計画を設計し、大半を実装した。

**v10発散問題への最有力候補（A1）を修正**: `ml_lidar/policy.py`の`ScanCNNExtractor`
の`Conv1d`層3枚がSB3の`init_weights`（`isinstance`が`nn.Linear`/`nn.Conv2d`限定）
によるorthogonal初期化から漏れ、PyTorchデフォルト初期化のまま学習されていた
バグを修正（`self.conv`構築直後に明示的にorthogonal_初期化を追加）。Copilot CLIの
セカンドオピニオンでも「PPO不安定性の寄与要因として有力」と裏付け済み。
**効果確認には新規学習が必要**（未実施）。

**A2（sim操舵むだ時間の量子化バグ）をユーザー合意の上で修正**: `sim/vehicle.py`の
`_pop_delayed`は「1step=1エントリ」方式で、`vehicle.step(dt)`はdt=0.1sで1回しか
呼ばれないため、`dead_time_s>0`である限り実効遅延は`dead_time_s`の値によらず
固定値に量子化されていた（ドメインランダム化`dead_time_s_range=(0.0,0.08)`が
実質無効化されていた）。`sim/gym_env.py`の`step()`内、`vehicle.step(dt)`の
1回呼び出しを`_DYNAMICS_SUBSTEP_S=0.01s`刻みのサブステップ分割に変更（`vehicle.py`
本体は無変更、衝突判定・LiDAR生成は従来通り100ms境界のまま）。**副産物の発見**:
`_pop_delayed`はwhileチェックがfor減算より前に行われる構造のため、エントリが
追加されたstep自身はpop対象にならず、反応時間には常に`+1 dt_sub`の系統的
オフセットが乗る（`dead_time_s=0`の特殊ケースのみ例外）。dt_sub=0.01なら最大10ms
のバイアスで実用上は許容範囲と判断し、`_pop_delayed`自体は今回変更していない
（気になれば次回、時刻ベースの補間キューへの置き換えを検討）。`sim.bench`は
`sim/stm32.py`経由で`VehicleModel`を直接使う別経路のため今回の変更の影響を
受けないことを実行確認済み。**既存の学習済みモデル（v9・v10以前）は「常に
固定遅延」の環境で学習されているため、この修正を適用した環境では前提が変わる
——次回の新規学習は必ずこの修正を含めてゼロから行うこと**。

**実車安全・堅牢性・最適化を多数修正**（詳細はコミット参照）: `cam_perception_node`/
`camera_node`/`planning_node`のノード停止・リトライ・例外処理バグ4件（B1-B4）、
`mcap_log`/`framelog`のfdリーク・`shm_view`のtorn画像保存・設定JSON非アトミック
書き込み・`raceline.py`の`free_ahead`表示逆転・`link_tracker`のping破棄ロジック・
`cleanup.py`のロックなし競合の7件（C1-C7）、`sim/gym_env.py`の`_CenterlineProgress`
窓探索化・`zbus.py`の`mkdir`削減・`camera_node`の`gaps_ms`上限・`purepursuit.py`の
死んだ分岐削除の4件（D2-D5）、GUIの`guide`依存WS再接続・`useMcapDownload`の
リーク2件・不要な再レンダリング3件（F1,F2,F3,F5,F6,F7）、`ml_cam`の`seed`固定・
IoU計算のmicro→macro-average化（E2,E3）。全項目、既存テスト＋新規テストで
検証済み（`raspi/tests`・`ml_lidar/tests`・`ml_cam/tests`全パス。既存の
`test_vehicle_grip.py`1件のみ今回の変更と無関係な既存の失敗と確認済み）。

**見送り**: D1（`centerline.py`のO(N*M)、BUILD相1回きりで緊急性低）、F4
（`makeProjector`メモ化、fps問題は環境要因で解決済みのため優先度低）、E1
（ImageNet正規化、3ファイル同時変更+再学習必須でコスト高）。

**次にやること（実施済み・結果は上記「最終更新」参照）**: ①A1+A2を反映した
新規学習(v11)を実行、②A4の2実験（`--no-reward-norm`単独・`--curriculum-frac 0`
単独）をA1+A2適用後の同一ベースで実行——のはずが、実際にはv11・v12とも
`reward_norm`/`curriculum_frac`が同一設定になっており②の切り分けは未達。
かつ早期終了がスケジュールと噛み合わず学習途中で打ち切られていたと判明
（詳細・v13方針は下記「2026-09-02（さらに続き）」節）。

旧: 2026-09-02（続き）（バンビの実車確認「v10は舵が大きく発散し壁に衝突する。
v9以前のモデルはOBS_DIM不一致で実行不能」を受けて診断。①OBS_DIM不一致は
`raspi/auto/e2e_lidar.py`のバグと確定・修正済み（ONNXグラフ自身の入力shapeを見て
後方互換）。②性能低下は、TensorBoard実測でv10がv9より学習初期からclip_fraction
が高くtrain/stdの収束も遅いことを確認——`VecNormalize(norm_reward=True)`が
疑わしい候補。実際にv10モデルをシムでロールアウトさせるとステア出力が
±1.0付近を毎ステップ行き来する激しい振動を確認、curriculum ramp開始
（~500kステップ）と同時に評価成績が悪化し以後回復しないパターンも実測。
切り分け用に`--no-reward-norm`を追加。次のセッションでの検証手順を含め詳細は
下記「2026-09-02（続き）」節）
旧: 2026-09-02（v9実測（早期終了3.62M/5M、`raceline/mean_cross_dev`が
400k step以降0.16〜0.21mで頭打ち、実測fps~103）の診断を受け、ml_lidar学習の
効率化・精度・安定性の改善5点をagy調査ベースで実装——①reset()のraceline計算の
非同期先読み・固定コースのメモ化キャッシュ、②観測へのステア状態追加、
③1D-CNN特徴抽出器（既定に）、④カリキュラム学習、⑤clip_range線形減衰＋
報酬正規化(VecNormalize, norm_obs=False)。観測空間が変わった（OBS_DIM 362→363）
ためv10はスクラッチ学習必須。詳細は下記「2026-09-02」節）
旧: 2026-09-01（`watch.py`で理想ラインがcircuit/fujiでギザギザに見えた不具合を
2件修正——①`sim/track.py`の閉ループ最終点重複による曲率スパイク、②Adam→L-BFGS
（`offset=max_offset*tanh(z)`の滑らかな再パラメータ化）+ 0.1m間引き最適化。
circuit/fuji/ランダムコースいずれも滑らかなアウトインアウトのラインに。詳細は
下記「2026-09-01（さらに続き）」節）
旧: 2026-09-01（v8実車評価「衝突しないがレーシングラインが綺麗でない」を受け、
`sim/raceline.py`を新設——道幅内で曲率二乗和を最小化する理想ラインと、曲率に応じた
目標速度プロファイルをオフラインで計算する。`sim/gym_env.py`の報酬に
`raceline_weight`/`speed_match_weight`を追加し、`speed_weight`の既定を0.3→0.1に縮小。
`ml_lidar/train_rl.py`に配線しTensorBoardへ`raceline/mean_cross_dev`を記録する
コールバックを追加、`ml_lidar/watch.py`に理想ラインの重ね描きを追加。
**v9本学習はまだ開始していない**——実車での`tau_steer_s`再測定
（`tools/sysid/fit.py`の遅延二重計上バグ修正版で録り直す）が先。下記「2026-09-01
（続き）」節）
旧: 2026-09-01（`config/vehicle.toml`の`[dynamics]`実測値をシステム同定タブで
実車確認し反映（`tau_steer_s`0.12→0.539、`mu`0.8→0.454等）。これを受けて
`sim/vehicle.py`に摩擦円によるRWD連成（駆動・制動が後輪だけを通るため加減速中は
コーナリングの余力が減る）を追加し、`sim/gym_env.py`のドメインランダム化レンジを
実測値中心に再設定。ml_lidar新規学習(v8)はフルスクラッチで開始する方針。
下記「2026-09-01」節）
旧: 2026-08-29（`ml/`を`ml_cam/`へ改名し、リポジトリ内の全参照
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

## ★ 2026-09-02（さらに続き）：A1・A2適用後のv11/v12を検証→早期終了とスケジュールの不整合を確定

前節末尾の「次にやること」①②③のうち①（A1+A2反映の新規学習）を実施したのがv11・
v12。ところがrun_config.jsonを比較すると**v11・v12は`reward_norm`/`curriculum_frac`
が完全に同一設定**（両方とも診断用にOFF）で、②で計画していた「`--no-reward-norm`
単独・`--curriculum-frac 0`単独」の切り分けにはまだなっておらず、実質的に
**同一設定の2回の反復学習**になっていた。

**evaluations.npz・TensorBoard実測**:
- v11: 800kステップで早期終了（`best`はeval18=360kステップ、reward513.9）。
  終盤5回のeval全てで`ep_len_mean`が上限1500に到達——一見v10より安定して見える
- v12: 620kステップで早期終了（`best`はeval5=**わずか100kステップ**、reward448.5）。
  終盤5回の`ep_len`は912〜1500と不安定、v11ほど良く見えない
- **同一設定なのにv11とv12で結果が違う**——これは`reward_norm`/`curriculum`や
  A1/A2の効果差ではなく、**学習自体のrun-to-run分散**が主因と判断

**根本原因（確定）**: `train_rl.py`の早期終了
（`StopTrainingOnNoModelImprovement(max_no_improvement_evals=20, min_evals=10)`）と、
`learning_rate`/`clip_range`の線形減衰スケジュール（`linear_schedule()`、
`progress_remaining`は**`--timesteps`固定値5,000,000に対する残り割合**で計算）が
噛み合っていない。v11は5Mの16%（800k）、v12は12%（620k）でしか学習が回っておらず、
その時点で`clip_range`は0.2→約0.175までしか減衰していない（ほぼ初期値のまま）。
対してv9は72%（3.62M）、v10は51%（2.54M）まで回っており、`train/std`・
`train/entropy_loss`が実際に収束したのはその後半区間だった（v9: entropy_loss
最終-0.40、std 0.30。v10: 同-2.0、0.66）。**v11/v12は「学習がまだ荒く探索的な
フェーズ」で早期終了のノイズだらけの評価reward（30エピソードとはいえ、方策が
定まっていない序盤は分散が大きい）に基づいて`best_model`を選んでしまっており、
v9・v10との比較にも、v11とv12同士の比較にも使えない**。

**結論**: v10→v11で「良くなったように見えた」のは、reward_norm/curriculumを
切ったからではなく、単に学習を800kステップで打ち切ったから不安定化する前に
止まっただけの可能性が高い。A1（Conv1d初期化）・A2（むだ時間量子化）・
reward_norm・curriculumのどれが効いているかは、**スケジュールと同じ土俵で
比較できるだけの長さを学習させない限り依然として未確定**。

**次にやること（v13への方針、下記チャット回答と対応）**: ①早期終了を
無効化する（`--early-stop-patience 0`）か`min_evals`/`patience`を
timesteps比で十分大きく取り、`clip_range`/`learning_rate`のスケジュールが
実際に終盤まで進むようにしてから、reward_norm=False・curriculum_frac=0・
A1+A2ありの構成で公平な基準線を取り直す。②そのモデルを実際にシムで
ロールアウトし、v10で見えた±1.0付近のbang-bang的なステア振動が
解消しているか直接確認する（reward数値だけで判断しない）。③基準線が
安定したことを確認できてから、reward_norm・curriculumは**同時にではなく
1つずつ**再導入して初めて切り分けになる。④ステアのbang-bang自体への
直接的な対策として、PPOの`use_sde`（generalized State-Dependent
Exploration）を試す価値がある——現状`use_sde`は未使用（i.i.d.な
per-step Gaussianノイズが毎ステップ独立にサンプルされるため、連続制御で
典型的なbang-bang挙動を誘発しやすいとSB3側も明記）。reward_norm/curriculum
とは独立に効く可能性があるレバーとして並行して検討する価値がある。

### 追記（2026-09-03）：v13を検証しv14方針を決定

上記③の指示通り`--early-stop-patience 0`（早期終了を完全無効化）・
`--timesteps 3000000`（v9・v10が実際に収束した範囲に合わせ、5Mより短くして
計算時間を節約）とし、`--no-reward-norm --curriculum-frac 0`はそのまま、
A1+A2適用済みのコードでv13を学習。

**収束の質（狙い通り改善）**: `train/clip_range`・`train/learning_rate`が
最後まで線形減衰し切り（終盤`approx_kl`はほぼ0まで低下）、`circuit`/`fuji`
固定評価は終盤8回連続でreward755〜760・`ep_len`1500/1500（v9の415〜445・
ep_len時々1500未達から明確に改善）。`evaluations.npz`・TensorBoardとも
早期終了時のようなノイズの山を拾った形跡は無い。

**実舵角のロールアウト確認（v10問題の解消を確認）**: `best_model.zip`を
シムでロールアウトし、`ml_lidar/env.py`の生アクション→`SimE2EEnv._steer`
（`steer_tau`フィルタ後）→`vehicle.steer_actual`（アクチュエータのむだ時間+
一次遅れ後、実際に車輪を切る角度）の3段階を直接比較。生アクションだけ見ると
符号反転が多く一見bang-bangに見えるが、これは中央付近の微小ノイズによる
見かけ上のカウントで、**実際に車輪を切る`steer_actual`は滑らかに連続変化し、
v10で確認されたような±1.0飽和の激しい往復は再現しなかった**——v10の実車
発散問題はA1+A2+reward_norm off+curriculum off+早期終了スケジュール修正の
組み合わせで解消したと判断してよい。

**新たに判明した弱点（circuit/fuji評価だけでは見えなかった）**:
`sim.random_course.generate_diverse_course`のフル分布（organic/circuit/
corridor/narrow/obstacle、道幅0.7〜1.3m——`curriculum_frac=0`なのでv13は
学習中この分布に最初から曝されている）から乱数40コースを生成し決定論的に
評価したところ、**collision_rate=17.5%（7/40）・reward=586.7±286.4
（分散が非常に大きい）・最悪5本は9〜48stepで即クラッシュ**。`circuit`/
`fuji`という2本の固定コースだけを見ていると「ほぼ完璧」に見えるが、
narrow/obstacle寄りの難しい条件への汎化はまだ弱いことが分かる。

**v14方針（単一変数の再導入）**: `reward_norm`は引き続き**OFFのまま**
（v13で実証済みの安定挙動・実舵角の滑らかさを崩すリスクを取ってまで
戻す理由が無い）。**`curriculum_frac`だけを本来の既定値`0.3`に戻す**
——`curriculum_frac=0`（v13）は学習序盤からnarrow/obstacleを含むフル難度
分布を経験させているにもかかわらず上記の汎化の弱さが残っており、
`CurriculumCourseFn`docstringが元々想定していた「一部のコース条件でだけ
評価成績が大崩れする」症状とも一致する。他の設定（`--early-stop-patience 0`・
`--timesteps 3000000`・A1+A2・reward_norm off）はv13から変更しない。
v14完了後は、v13と同じ40ランダムコース評価（`np.random.default_rng(123)`で
同一seed列を使うと直接比較できる）でcollision_rate・reward分散が改善したか
を必ず確認すること——`circuit`/`fuji`評価だけでは今回の弱点を再現できない。

### 再追記（2026-09-03、バンビ「v13でも舵が発散しているように見える」指摘での再検証）

上の「v13でv10のbang-bang問題は解消」という判断は**誤りだった**。バンビが
`ml_lidar/watch.py`と通常のシム画面（narrow/obstacle込みのランダムコース
パネル）の両方で発散を目視確認したのを受け、40ランダムコース評価の**衝突
エピソードそのものの舵角トレース**を追加確認した。

`vehicle.steer_actual`（フィルタ・アクチュエータ後の実舵角）自体は衝突直前
でも±0.3〜0.4rad程度に収まっており、v10のような±1.0飽和ではない——ここは
前回確認の通り。**しかし生の方策出力（`action[0]`、フィルタ前）は、
衝突コースだけでなく好成績だったcircuit/fujiでも同程度に激しく振動している**
（mean|diff|≈0.28〜0.35、隣接ステップの符号反転が全体の37〜39%——例:
circuit評価の実測列 `1.0, -0.23, 0.82, -0.33, 0.68, -0.44, 0.59...`）。
**circuit/fujiでは`steer_tau`（環境側の一次遅れ、dt=tau=0.1sでα≈0.63）+
車両の`tau_steer_s`（実測0.539s、A2修正後は正しく効く）という二段のフィルタが
たまたまこの振動を実害の出ない範囲まで削り切れていただけで、方策自身が
出す信号の荒さ（探索ノイズの大きさ）そのものはv10から実質変わっていない**。
narrow/obstacleのような余裕の少ないコースでは同じ振動が姿勢を乱すのに
十分な大きさになり、衝突および「発散して見える」挙動として顕在化する。
つまり早期終了/スケジュール修正・A1・A2・reward_norm off・curriculum off
はいずれも「二段フィルタが誤魔化せる程度まで実害を抑える」効果はあったが、
**根本原因（方策出力自体のノイズの荒さ）には手を付けていなかった**。

**v14方針を修正**: `curriculum_frac`の再導入より前に、**PPOの`use_sde`
（generalized State-Dependent Exploration）を試す**——毎ステップi.i.d.に
サンプルされる既定のガウス探索ノイズが振動の直接原因と疑われるため、状態
依存の相関ノイズを`sde_sample_freq`ステップごとにしか再サンプルしない
gSDEに切り替えれば、時間的に滑らかな探索軌道になり生の出力の振動そのものが
減るとSB3側も明記している。`train_rl.py`に`--use-sde`/`--sde-sample-freq`
（既定4）を追加し、`run_config.json`記録・`ml_lidar/app.py`のGUI
（チェックボックス+エントリ）にも配線済み（スモークテストで疎通確認済み、
`ml_lidar/tests`131件green・既存の`test_vehicle_grip.py`1件のみ無関係な
既存失敗）。v14は**v13の設定（`--early-stop-patience 0`・`--timesteps
3000000`・`--no-reward-norm`・`--curriculum-frac 0`）はそのまま**、
`--use-sde --sde-sample-freq 4`だけを追加した単一変数の切り分けとする。

v14完了後は①生アクション（`action[0]`、フィルタ前）のmean|diff|・符号反転率が
v13より下がっているか、②同じ40ランダムコース評価でcollision_rateが下がって
いるか、の両方を確認すること。gSDEで振動が収まらなければ、方策の出力層の
活性化（squash_output）やreward側の`steer_rate_weight`をもっと強める方向を
次に疑う。curriculum_fracの再導入（元々の生成の狭さ・narrow/obstacleへの
汎化不足への対策）はgSDEの効果を見極めた後、引き続き一つずつ試す。

---

## ★ 2026-09-02（続き）：v10の実車確認「舵が発散し壁に衝突」を診断

バンビが実車でv10を確認し「舵が大きく発散し壁に衝突する。大きな性能低下が見られる」
「v9以前のモデルが`Got invalid dimensions...Got: 363 Expected: 362`で実行できない」
と報告。2件を切り分けて調査した。

**①OBS_DIM不一致バグ（確定・修正済み）**: `raspi/auto/e2e_lidar.py`の`plan()`が
ロード中のモデルに関係なく常に363次元（点群+速度+ステア）の観測ベクトルを組み立てて
いたため、ステア観測追加（本節の1つ前、362→363次元化）より前にエクスポートされた
v9以前のモデル（362次元）が全滅していた。`_load_from_path()`を、JSON側の`in_dim`
ではなく**ONNXグラフ自身が宣言する入力shape**（`session.get_inputs()[0].shape[-1]`。
`export_onnx_rl.py`はバッチ1固定の具体的なshapeでエクスポートしているので必ず
具体的な整数になる）を読むように変更し、`plan()`側は`self._model_in_dim -
len(scan_n)`で「速度のみ足すか（旧362次元）・速度+ステアを足すか（新363次元）」を
動的に分岐するようにした。`raspi/tests/test_auto.py`に
`test_legacy_362dim_model_still_loads_and_runs`を追加、回帰防止。

**②性能低下の診断（`ml_lidar/runs/v10/evaluations.npz`・TensorBoard実測）**:
評価reward・ep_lengthは学習序盤（~500kステップ、カリキュラム進捗~33%）までは
v9同等以上に良好（reward500超・ep_len 1500）だったが、**それ以降は評価が悪化して
以後回復しないパターン**（reward 130〜520の間で乱高下、ep_lengthも500〜1500で
不安定）。`early_stop_patience=20`が2.12Mのベスト(reward522)から20回連続未更新で
2.54Mに発火し停止。

実際に`ml_lidar/runs/v10/best_model.zip`をシム上で複数エピソード走らせたところ、
**ステア出力が毎ステップ±1.0付近を激しく行き来する**（例: +0.29,+0.16,+0.55,-0.05,
-0.66,-1.00,-0.20,-1.00...）ことを直接確認——バンビの実車報告と一致する。観測末尾の
ステア特徴を強制的に0固定してもこの振動自体は解消しなかった（振動の頻度は下がるが
飽和した±1.0張り付きは残る）ため、**ステア観測フィードバックそのものが唯一の原因
ではなく、方策の出力自体がbang-bang的に飽和しがちな、より根本的な学習不安定性**が
疑われる。

v9とのTensorBoard比較で決定的な手がかりを得た: **v10は`train/clip_fraction`が
学習ごく初期（36万step、カリキュラムの影響がまだ小さい時点）から既にv9より高く
（v10: 0.22 vs v9: 0.15）、学習全体を通して0.09→0.34まで単調増加し続ける
（v9は0.11〜0.18で安定）**。`train/std`（方策のガウス分布の標準偏差）の収束も
v10の方が明確に遅い（2.5M step時点でv10は0.66、v9は同程度のstepで0.36前後）。
これは**カリキュラムが難しい条件を導入する前から既にPPOの最適化が不安定**で
あることを示し、カリキュラムの難度上昇はその既存の不安定性を「衝突」という
目に見える形で表面化させた可能性が高いと考える。`train/explained_variance`は
逆にv10の方がv9より高め（0.6〜0.76 vs 0.45〜0.65）で価値関数自体の当てはまりは
悪くないため、**価値推定ではなく方策更新側（advantageのスケール・clip_rangeとの
相互作用）に問題がある**可能性が高い。

**最有力候補**: 今回新規追加した`VecNormalize(norm_reward=True)`（報酬正規化）。
学習ごく初期から症状が出ていること・PPOの方策更新の実効的な大きさに直接影響する
唯一の新機構であることから、CNN特徴抽出器やカリキュラムより疑わしい。ただし
確定させるには実際に無効化して再学習しないと分からない。

**対策**: `train_rl.py`に`--reward-norm`/`--no-reward-norm`
（`argparse.BooleanOptionalAction`、既定`--reward-norm`）を追加し、
`VecNormalize`の有無をCLIから切り替えられるようにした。`run_config.json`にも
`reward_norm`を記録。`ml_lidar/tests/test_train_rl.py`に
`TestTrainRlRewardNorm`（無効化しても完走し`vecnormalize.pkl`を書かないことを
確認）を追加。

**次にやること（未実施）**: `--no-reward-norm --curriculum-frac 0`
（v9のPPO力学に戻しつつCNN・ステア観測・clip_range減衰だけ残す構成）でv11を
再学習し、v9並みの安定性（clip_fraction・std収束）が戻るか確認する。戻れば
報酬正規化とカリキュラムのどちらか（または両方）が原因と確定し、個別に
再度切り分ける。戻らなければCNN特徴抽出器かステア観測自体を疑う。

**GUI対応（同日、バンビ「コマンドをさわれないのでGUIからその設定で学習できる
ようにしといて」を受けて）**: `ml_lidar/app.py`の①学習タブに
「curriculum_frac」入力欄（`ttk.Entry`）と「報酬正規化(VecNormalize)を使う」
チェックボックス（`ttk.Checkbutton`）を追加。`build_train_cmd()`に
`reward_norm`/`curriculum_frac`引数を追加し、`reward_norm=False`のとき
`--no-reward-norm`を、常に`--curriculum-frac <値>`を付与する。**原因が
確定するまでの間、GUIのウィジェット既定値そのものを診断構成
（curriculum_frac="0"・報酬正規化チェックボックスOFF）にしてある**——
`build_train_cmd()`関数自体のキーワード引数既定は`reward_norm=True,
curriculum_frac=0.3`（train_rl.py本来の意図した既定と一致）のままなので、
原因確定後はGUIのウィジェット既定だけ戻せばよい。テストは
`ml_lidar/tests/test_app.py`に`TestBuildCmds`2件追加。

**テスト**: `ml_lidar/tests`+`raspi/tests`計694件中692件green（残り2件は
本変更と無関係な既存failure——`test_vehicle_grip.py`は2026-09-01節で既述、
`test_cam_perception_node.py`の`hb/cam_perception != scan/cam`は既知の
フレーキーテスト、`git stash`不要で単体再実行でも同じ失敗を確認済み）。

---

## ★ 2026-09-02：v9実測診断→ml_lidar学習の効率化・精度・安定性 改善5点を実装

バンビ「v9の学習が終わった。走行ラインも綺麗で良い感じだが、まだ完璧ではない。
v9の正確な検証が必要であれば行って。改善案は全て実装して」→ 前回セッションで
agyスキルを使い外部調査（F1TENTH/AWS DeepRacer/学術研究）した5つの改善提案
（`~/.claude/plans/proud-fluttering-boot.md`）を全部実装した。

**v9実測診断**: `evaluations.npz`/TensorBoardを解析。早期終了は3.62M/目標5M
（ベストは2.6M）。`raceline/mean_cross_dev`が0.16〜0.21mで400k step以降ほぼ
横ばい（理想ラインへの追従精度が頭打ち）。`train/explained_variance`も
0.45〜0.65止まり（価値関数の当てはまりが甘い）。評価rewardの標準偏差が
1.1M/2.36M/3.08M stepでだけ100超まで跳ねる（一部のコース条件でだけ大きく崩れる）。
実測fps~103（`n_envs=8`。raceline導入前の素の環境fps 502〜556の約1/5）——
9.69時間で3.62Mステップ。

**①reset()のraceline計算の高速化**（`sim/gym_env.py`）: 固定`courses`
（`make_eval_env`のcircuit/fuji）向けに`(id(course), mu, drive_accel, brake_decel)`
キーのメモ化キャッシュを追加。手続き生成コース（`course_fn`、訓練用）向けには
`course`・`episode_spec`の値だけで決まる純粋関数`_compute_raceline()`を
バックグラウンドスレッドで次エピソード分先読みする仕組みを追加——
`self.rng`はメインスレッドの`_draw_course()`/`_episode_spec()`だけが触るので
競合状態は起きない設計。**実装直後に踏んだ罠**: `ml_lidar/env.py`の
`GymSurgeEnv.reset(seed=...)`は`self._env.rng`を丸ごと差し替えてから`reset()`を
呼ぶため、差し替え前の`rng`から先読みした結果をそのまま使うと決定性が壊れる
（`test_course_fn_reset_with_same_seed_gives_the_same_course`で発覚）。
先読み開始時の`rng`オブジェクト自体を`self._pending_rng`として覚えておき、
`reset()`時に今の`self.rng`と一致するときだけ使うよう修正して解決。

**②観測に「現在の平滑化後ステア角」を追加**（`OBS_DIM` 362→363）:
`sim/gym_env.py`の`_obs()`・`ml_lidar/env.py`の`_to_obs()`・
`raspi/auto/e2e_lidar.py`の`plan()`に同じ量（`steer_tau`一次遅れフィルタ後の
実ステア角、`/max_steer`で[-1,1]正規化）を追加。方策が「今どれだけ舵を切って
いる状態か」を知らないまま次の指令を出していた設計の穴への対応。
`raspi/auto/e2e_lidar.py`は既に`self._steer`（出力側の平滑化）を持っていたので
新規stateは不要——読み出しのタイミングを`self._steer`更新より前にするだけで
「前回ステップの実現値」という学習側と同じ時系列関係になる。
`ml_lidar/env.py`の`observation_space`もscalarの`Box(0,1)`から
`Box(low,high)`（steerの1個だけ[-1,1]）に修正（範囲外警告の副次的な修正）。

**③1D-CNN特徴抽出器**（新規`ml_lidar/policy.py`の`ScanCNNExtractor`、
`--features-extractor {mlp,cnn}`既定`cnn`）: 361点のスキャンだけ3層のConv1dで
圧縮してから速度・ステア角と結合するSB3の`BaseFeaturesExtractor`。角度順に
並ぶ点群の空間相関を活かす狙い。ONNXエクスポート（opset18）・PyTorch/ONNXRuntime
parityとも実測確認済み（最大差1.49e-07）。

**④カリキュラム学習**（`sim/random_course.py`の`CurriculumCourseFn`、
`sim/gym_env.py`の`SimE2EEnv.set_curriculum_progress`、`train_rl.py`の
`CurriculumCallback`・`--curriculum-frac`既定0.3）: 評価rewardの標準偏差が
一部の評価回だけ跳ねる傾向を受け、学習序盤は`mu_range`を実測中心値付近に絞り・
narrow/obstacleアーキタイプを出さず・道幅下限を1.0mに絞った易しい条件から、
`curriculum_frac * total_timesteps`かけて本来の難度分布へ線形に近づける。
`VecEnv.env_method()`で各ワーカープロセスへ配る。`eval_env`は触らない
（固定courses+`randomize_dynamics=False`のまま、v9との比較可能性を保つ）。

**⑤`clip_range`線形減衰＋報酬正規化**: `--clip-range`（既定0.2）を`learning_rate`
と同じ`linear_schedule()`で減衰（v2で踏んだ方策崩壊対策の追加分）。
`VecNormalize(vec_env, norm_obs=False, norm_reward=True)`で報酬だけ正規化——
観測は既存のmin-max正規化のまま(`raspi/auto/e2e_lidar.py`の前処理と一致させ
続けるため、`norm_obs`はfalseのまま)。**踏んだ罠**: SB3の`EvalCallback`は
訓練env側に`VecNormalize`があると無条件で`sync_envs_normalization()`を呼び、
`eval_env`も`VecNormalize`でないと`AssertionError`で落ちる。`eval_env`を
`VecNormalize(..., norm_obs=False, norm_reward=False, training=False)`で
包み、評価スコア自体は生rewardのまま・統計量も更新されないようにして解決。
`--resume-from`時は`vecnormalize.pkl`（`model.save()`とは別ファイル）を
同じ流儀（run_config.json自動読み込みと同型）で復元する。

**テスト**: `ml_lidar/tests/test_policy.py`新設（4件）。
`test_env.py`/`test_gym_env.py`/`test_random_course.py`/`test_export_onnx_rl.py`/
`test_train_rl.py`に先読み・キャッシュ・カリキュラム・CNN export・
CurriculumCallbackのテストを追加。`raspi/tests/test_auto.py`の
`_OBS_DIM`を362→363に更新（ダミーモデルの重み位置`w[1,-1]`→`w[1,-2]`も
speed位置ズレに合わせて修正）、`TestE2ELidar`にステア観測のround-tripテストを
追加。`ml_lidar/tests`・`raspi/tests`計693件中692件green（残り1件
`test_vehicle_grip.py`の`test_braking_mid_corner_at_measured_mu_can_zero_out_lateral_grip`
は本変更と無関係な既存の未解決failure——摩擦円RWD連成の加減速側実測待ち、
2026-09-01セクション参照）。`./tools/check.sh`のtsc/生成物チェックも問題なし
（protocol.tomlの版番号文書ズレは本変更と無関係の既存差分）。
`train_rl.py --timesteps 2000 --n-envs 2`のスモークテストで
`curriculum/progress`・`raceline/mean_cross_dev`のTensorBoard記録、
`vecnormalize.pkl`保存、CNN構成でのONNXエクスポート+parityまで実地確認済み。

**まだやっていないこと**: v10本学習はまだ開始していない（観測空間が変わった
ため`--resume-from`は使えず、フルスクラッチ必須）。実車での`tau_steer_s`再測定
（2026-09-01「続き」節の未着手1）も引き続き未対応のまま——v9と同じ実測待ちの
`tau_steer_s`レンジを使っているので、v10の前提条件としては変わっていない。

---

## ★ 2026-09-01（続き）：理想ライン(`sim/raceline.py`)導入・v8→v9レーシングライン品質改善

バンビ「v8を実車で走らせたら壁には衝突しないが綺麗なライン取りができていない。
前提を疑い、類似プロジェクトも調査して改良案を」→ 調査の結果、(1) 報酬に理想
レーシングラインの概念が一切無い（`cross_track_margin_frac`の余白＋一律の
`speed_weight`だけ）、(2) `tau_steer_s=0.539`が`fit.py`の遅延二重計上バグ修正前の
値のままドメインランダム化の中心に使われている、の2点が主要因と判断。
F1TENTH/AWS DeepRacer/学術研究（Trajectory-Aided Learning、Bosello et al.
arXiv:2306.07003）を調査し、E2E LiDAR方策のアーキテクチャは変えず学習時の
報酬にだけ理想ラインを組み込む方針で合意（プラン: `~/.claude/plans/ml-lidar-v8-compressed-ember.md`）。

**1. `sim/raceline.py`新設**: `compute_raceline_offsets()`（道幅内で最小曲率に
寄せた中心線オフセット）と`compute_speed_profile()`（曲率ベースの目標速度、
前進=加速度上限・後退=減速度上限で挟む3段パス）。

**★曲率最小化の実装で複数回の手戻りを踏んだ（重要な教訓）**: 最初は
`sim/random_course.py`の`_min_turn_radius_m`と同じ「隣接点の中点に寄せる
反復平滑化」を試したが、これは**曲線短縮フロー（curve shortening flow）**
そのもので曲率最小化とは逆方向に働く——閉ループ全体に一様適用すると円が一様に
収縮し半径が小さくなる（＝曲率が悪化する）ことを数値実験で確認（生成コースで
`sum(curvature^2)`が最悪10倍近く悪化）。次に線形近似`κ_path≈κ_ref+n''`で
Poisson方程式を解く方式を試したが、実際のコース（道幅に対するオフセット比が
0.3〜0.5程度）ではこの近似の前提（オフセットが曲率半径に対して小さい）が
崩れ、非線形項`n·κ_ref²`の寄与が支配的になり同様に悪化した。最終的に**厳密な
離散曲率をそのまま目的関数にし、`torch`の自動微分で正確な勾配を取ってbox制約
付きAdam勾配降下（射影付き）で最小化**する方式に落ち着いた——手で導いた線形
近似は符号や非線形項の扱いを誤りやすく実測で悪化が繰り返し確認されたため、
正確性を優先した。`torch`はSB3が要求する既存の依存で新規追加ではない。1コース
(200〜400点)・200ステップの最適化で実測20〜40ms（初回呼び出しのみtorchの
ウォームアップで数百ms）——コース生成(エピソード)ごとに1回で済む設計なので
学習速度への影響は軽微。**同じようなコード（曲率・軌道の最適化）を今後書く際は、
必ず実データ（`sim/random_course.py`の実際の生成器）で数値検証してから採用する
こと**——手計算の直感（「中点に寄せれば滑らかになるはず」）は円のような閉曲線
では容易に裏切られる。

**2. `sim/gym_env.py`の報酬統合**: `_CenterlineProgress.update()`が最近傍点の
インデックスも返すよう変更（既に内部で求めていた値を活用）。`_RacelineProgress`
クラスを追加し、そのインデックスを流用して理想ラインの参照点・目標速度を引く
（2回目のO(N)最近傍探索を避ける近似）。`reset()`で`episode_spec`（ドメイン
ランダム化後の`mu`等）を使って理想ラインを計算——固定`spec`を使うと今エピソード
のグリップと目標速度がズレるため。新しい報酬項`raceline_weight`（理想ラインから
`raceline_tolerance_m`を超えた横偏差への罰則）・`speed_match_weight`（曲率考慮の
目標速度との整合度へのボーナス）を追加。`speed_weight`の既定を0.3→0.1に縮小し
（曲率を考慮しない一律ボーナスがコーナー前の減速を妨げていた疑い）、主役を
`speed_match_weight`に譲った。

**3. `ml_lidar/train_rl.py`への配線**: `--raceline-weight`(既定0.3)・
`--raceline-tolerance-m`(既定0.08)・`--speed-match-weight`(既定0.3)を追加、
`--speed-weight`既定を0.1に変更。`make_train_env_fn`/`make_eval_env`/
`run_config.json`にも配線。`RacelineMetricsCallback`（SB3の`BaseCallback`、
`logger.record_mean`で`raceline/mean_cross_dev`をTensorBoardに記録）を新設し
`model.learn(callback=[eval_callback, raceline_metrics_callback])`に変更。

**4. `ml_lidar/watch.py`に理想ラインの重ね描き**（黄色の細線、`_RACELINE_COLOR`）
を追加。目視でアペックスを突けているか確認できるように。

**テスト**: `ml_lidar/tests/test_raceline.py`新設（7件: 道幅制約を破らない・
曲率二乗和を悪化させない・ほぼ直線な大半径円ではオフセットがほぼ0・narrow
アーキタイプで例外にならない・曲率が高いほど目標速度が下がる・max_speedで
頭打ち・実測`drive_accel_m_s2`がデフォルトフォールバックより優先される）。
`ml_lidar/tests/test_env.py`に4件追加（`raceline_cross`/`target_speed`が
infoに含まれる・`raceline_weight`が許容帯超過を罰する・`speed_match_weight`が
目標速度への一致を報酬する）。`ml_lidar/tests/`全99件green。
`ml_lidar/train_rl.py --timesteps 500 --n-envs 1`のスモークテストで
`raceline/mean_cross_dev`がTensorBoardログに出ることを確認済み。

**未着手（次にやること）**:
1. **実車でステア試験を録り直し`tau_steer_s`/`dead_time_s`を再測定**
   （`tools/sysid/fit.py`の`steer_cmd_echo`基準・遅延二重計上バグ修正版で）。
   `config/vehicle.toml`の`tau_steer_s=0.539`はまだ修正前の値のまま。
   `sim/gym_env.py`の`tau_steer_s_range`（154行目）も新しい実測値中心に
   更新すること（これが土台なので最優先——ここが歪んだままだと理想ラインの
   速度プロファイルも報酬チューニングも効果を切り分けられない）
2. v9をスクラッチ学習（`--resume-from`は使わない。報酬の形が変わるため）
3. circuit/fuji評価スコア・`raceline/mean_cross_dev`をv8と比較し、
   `raceline_weight`/`speed_match_weight`/`cross_track_margin_frac`を
   必要なら再調整してから実車確認

計画の全文は`~/.claude/plans/ml-lidar-v8-compressed-ember.md`参照。

## ★ 2026-09-01（さらに続き）：理想ラインが circuit/fuji でギザギザだったバグ2件を修正

バンビが`watch.py`の観戦画面を見て「黄色い理想ラインがcircuit/fujiで毛羽立って
ギザギザ、ランダムコースもアペックスを突けていないように見える」と指摘。数値で
検証し、独立した2つのバグを特定・修正した。

**バグ①（circuit/fuji限定）**: `sim/track.py`の`centerline()`（`path`指定コースの
ビルダー）が、閉ループの最後の点を始点と重複させたまま返していた（距離`1.3e-15m`＝
浮動小数点誤差レベルで完全一致）。`sim/raceline.py`の曲率計算は`dyaw/max(seg,1e-6)`
なのでこの1点だけセグメント長がほぼ0になり、曲率が**103万rad/m**（他のコーナーは
1〜2rad/m）に爆発し目的関数を乗っ取っていた（実測: オフセット0での損失が`1.06e12`）。
`raspi/nav/centerline.py`の`resample_loop`が同じ理由で最後の点を含めない設計に
なっているのに、`track.py`側には未適用だった。`build()`でループが綺麗に閉じている
ときだけ最後の点を落とすよう修正（`sim/track.py`）。

**バグ②（全コース共通）**: 重複点を除いても、隣接点の60%以上でオフセットの符号が
反転する高周波ノイズが残った。しかも反復回数を200→5000に増やすと悪化（境界張り付き
点が13→468に増加）——目的関数（曲率二乗和、オフセットの2階差分を含む梁のたわみ的な
性質）に対しAdamが各点をほぼ独立・等幅で動かすため、隣接点が逆方向に押し合う
チェッカーボード状のノイズを減衰できていなかった。`torch.optim.LBFGS`
（fullbatch quasi-Newton）に変更し、box制約は`clamp_`ではなく
`offset=max_offset*tanh(z)`の滑らかな再パラメータ化で埋め込んだ。さらに、
circuit/fuji（`track.py`由来、解像度そのまま2cm間隔=976〜1588点）は自由度が
多すぎてL-BFGSでも収束が遅い（実測5000反復・24秒）ことが分かったため、最適化
自体は`sim/random_course.py`と同じ0.1m間隔に間引いた点で行い、結果を弧長ベースの
周期線形補間で密な点列に戻す（`_coarse_indices`）。

**結果**: circuit/fuji/ランダムコースいずれも目視で滑らかなアウトインアウトの
理想ラインになった（PNG出力で確認済み）。`ml_lidar/tests/test_raceline.py`の
`test_near_straight_loop_keeps_offsets_small`は前提が誤っていたことが判明
（直線区間の無い完全な円は、外側へ均一に膨らむのがbox制約下で数学的に正しい
最適解——旧Adam実装が200反復で収束しきらず偶然小さい値で止まっていただけだった）
ため`test_full_circle_loop_inflates_uniformly_to_box_limit`に書き換え。
`ml_lidar/tests/test_env.py`の2件（`test_progress_reward_is_near_zero_when_stationary`・
`test_speed_weight_has_no_effect_when_stationary`）は理想ラインが実際に意味のある
オフセットを持つようになった結果、スポーン地点がコーナー付近だと静止時にも
`raceline_weight`の罰則が乗るようになったため、`raceline_weight=0.0`/
`speed_match_weight=0.0`を明示（`cross_track_weight=0.0`と同じ既存の流儀）。

`ml_lidar/tests/`139件中138件green（残り1件`test_vehicle_grip.py`の
`test_braking_mid_corner_at_measured_mu_can_zero_out_lateral_grip`は本修正と無関係な
既存の未解決failure——`git stash`で本修正前でも同じ失敗を確認済み。摩擦円RWD連成の
方の話で、`tau_steer_s`再測定と同じく別件）。

**まだやっていないこと**: v9本学習はまだ開始していない（上の「2026-09-01（続き）」
節の「未着手」1〜3がそのまま該当）。理想ラインの品質改善はその前提条件の一部が
片付いただけで、`tau_steer_s`実測し直しは依然として最優先のまま。

## ★ 2026-09-01（さらにさらに続き）：↑のL-BFGS化が原因でv9学習が実質止まっていた不具合を修正

バンビが上記修正を反映した`v9`を実際に回したところ、起動直後のwarning以降ログが
全く更新されず「学習が止まっている」状態に。`env.reset()`のたびに
`compute_raceline_offsets()`が呼ばれる設計なので、この関数自体の実行時間を
疑って計測したところ、**1回280ms前後**（旧Adam実装の目標20〜40msの7〜14倍）
かかっていることが判明——`n_envs=8`の`SubprocVecEnv`で、特に学習初期はほぼ
無作為な方策がすぐ衝突してエピソードが短く`reset()`が頻発するため、これが
学習を事実上フリーズさせていた。

原因は`_OPT_ITERATIONS=200`を`opt.step(closure)`で2回呼んでいたこと。ただし
反復回数を減らして`step()`を1回だけにすると**別の不具合**が出た——`max_iter`を
5〜200のどれにしても同じ(悪い)ロス値で頭打ちになり理想ラインが中心線からほぼ
動かなくなる。実測で判明したのは、`torch.optim.LBFGS`の`strong_wolfe`直線探索は
1回の`step()`内で「この回はもう改善できない」と判断すると`max_iter`を使い切る前に
抜けてしまい、**quasi-Newton履歴を引き継いだまま`step()`を何度も呼び直すことで
初めて先に進む**という、`max_iter`を増やすだけでは代替できない挙動を持つこと
（circuitで実測: 1回目`step()`でロス50.97のまま頭打ちに見えたが、9回呼び直すと
22.8まで下がり、そこで`max|offset|`が0.01→0.38に育ってようやくアペックスを
突くラインになった）。`sim/raceline.py`のdocstring・コメントに詳細を残した。

**修正**: `max_iter=20`の`step()`を、ロス改善が2%を下回るまで最大8回呼び直す
方式（`_OPT_MAX_CALLS`・`_OPT_REL_TOL`）に変更。実測タイミング: ランダム
コース(訓練用)平均約90ms・最大約100ms、circuit約140ms・fuji約100ms
（旧280msの約2〜3倍高速化、かつ理想ラインの質は劣化させていないことをPNG
出力と`max|offset|`の実測値で確認済み）。テストは`ml_lidar/tests/test_raceline.py`
`test_env.py``test_gym_env.py`計33件green。

**判断が必要な点（次のセッションで確認）**: 90〜140ms/resetは旧Adam実装の目安
（20〜40ms）よりまだ2〜4倍重い。`n_envs=8`での実際のsteps/sec低下がv9の
学習時間に許容範囲か、実際にしばらく走らせて確認すること。もし依然重すぎるなら、
`_OPT_MAX_CALLS`を減らす（速度優先）か、理想ライン自体をエピソードごとではなく
コース形状ごとにキャッシュする（`generate_diverse_course`は`course_fn`で
毎エピソード新規生成するため今は使えないが、キャッシュキーを工夫する余地はある）
などの追加最適化を検討する。

---

## ★ 2026-09-01：vehicle.toml実測反映・シム旋回精度の摩擦円連成・ドメインランダム化レンジ再調整

バンビ「vehicle.tomlを実測して作り直したのでml_lidarの学習を新しく始めようと思う。
これに伴ってシミュレータの精度をより改善したい。特に、旋回時の挙動などを検討して」
→ さらに「後輪駆動なのは考慮されてる？」という指摘を受けて調査・実装した。

**1. システム同定機能一式をコミット**（`b2dbb8d`）: `raspi/auto/sysid_{corner,speed,steer}.py`・
`_sysid_common.py`・`tools/sysid/`（`fit.py`/`toml_update.py`/`gui.py`）・GUIの
`SysIdView.tsx`等、実車確認済みの一連の機能を先にコミット。`config/vehicle.toml`の
`[dynamics]`が仮値から実測値へ更新された:
`tau_steer_s` 0.12→**0.539**（操舵が想定の4.5倍遅い）、`dead_time_s` 0.030→0.046、
`steer_rate_limit_rad_s`新規**11.32**、`tau_speed_s` 0.35→**0.152**、
`mu` 0.8→**0.454**（グリップが想定よりかなり低い）。

**2. 摩擦円によるRWD連成を`sim/vehicle.py`に追加**（`117145e`）: 従来の
`a_lat_max = mu*g`は舵角と速度だけで決まる固定値で、加減速中かどうかを一切
考慮していなかった。しかしこの車の駆動力・制動力は`drive_ratio`（後輪ぶん）を
通じてどちらも後輪だけが発生させる（`_next_speed()`のブレーキ・トルクmode参照）。
実車では同じ後輪タイヤが「曲がる力」と「加減速する力」を同じ摩擦の予算から
奪い合う（摩擦円）ため、コーナー立ち上がりで加速しながら曲がると定常円旋回より
横方向の余力が減る。`step()`内で
`a_lat_max = sqrt(max(0, (mu*g)^2 - accel_x^2))`として近似（前後輪別のグリップ配分・
荷重移動までは踏み込まない、単純な全体摩擦円）。定常状態（`accel_x≈0`）では
従来通り`mu*g`に一致するため、`sysid_corner.py`の測定条件（定常段階でグリップ測定）
や既存テストの前提とは矛盾しない。`slip_frac`プロパティも同じ絞られた値を参照する
よう変更（`slip_weight`報酬との整合）。

**3. `sim/gym_env.py`のドメインランダム化レンジを実測値中心に再設定**（同コミット）:
`SimE2EEnv.randomize_dynamics`の`*_range`は旧仮値（`tau_steer_s=0.12`・`mu=0.8`）
基準の「未検証の見積もり」のままで、特に`tau_steer_s_range=(0.05, 0.25)`は実測値
0.539を全く含んでいなかった（訓練中の方策が実車ほど鈍い操舵応答を一度も経験しない）。
`mu_range`(0.4,1.1)→(0.28,0.65)、`tau_steer_s_range`(0.05,0.25)→(0.35,0.75)、
`tau_speed_s_range`(0.15,0.6)→(0.08,0.30)に変更（`dead_time_s_range`は既に実測値
0.046を中心に含むため変更なし。`rolling_resistance_range`は学習ループが常に
`armed=True`速度指令モードのため反映されず変更なし）。

**テスト**: `ml_lidar/tests/test_vehicle_grip.py`に2件追加
（`test_braking_mid_corner_reduces_available_lateral_accel`——定常円旋回中に
急制動すると`mu*g`単独より横加速度上限が下がることを確認、
`test_steady_cornering_a_lat_max_matches_mu_g`——定常状態では従来通り`mu*g`に
一致することの回帰確認）。既存7件+新規2件、計9件全green。

**学習(v8)は`ml_lidar/app.py`からフルスクラッチで開始する方針**（バンビの判断。
mu・tau_steer_sの変化が大きく、v7の方策は新しい力学に対して的外れになっている
可能性が高いため`--resume-from`は使わない）。`--slip-weight`は既にCLI引数として
存在する（既定0.2）ので追加対応不要。**v8の学習自体・実車での確認は未実施。**

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
