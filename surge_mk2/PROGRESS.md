# SURGE Mark.2 開発進捗（現在の状態だけ）

**新しいセッションではまずこれを読む。** 設計の中身は `docs/` が正。ここには「今どこまで進んだか」と「まだ確かめていないこと」だけを書く。
作業ごとの経緯は書き足さない（コミットメッセージと git 履歴に残す）。2026-11-09 までの作業記録は
`git show 4921671:surge_mk2/PROGRESS.md`、それより前は `docs/archive/`（**陳腐化しているので、指示がない限り読まない・根拠にしない**）。

## 現在の状態（2026-11-09。git で確かめた値）

- **Pi 側**: 変更はすべてコミット済み（HEAD 4921671）。11-09 の変更（解除後に経路に沿って止める・経路への合流・止まった姿勢からの再特定・
  `ParamSpec` のチェックボックスと表示条件・遮られたときの自己位置・LiDAR のノイズ除去と強度付きの形式）は **Pi に反映した記録がなく、実機では確かめていない**。
  11-06 までの変更は Pi で走らせた（記録 `surge_20261106_135218` まで）
- `config/vehicle.toml` の `[limits]` は校正の値（会場で手で書き換えた値 4921671 は戻した）
- **STM32 ファーム**（`~/GitHub/AutonomousCar`、ブランチ `FW/MainBoard_V3`、HEAD 99f7c94。origin より 3 コミット先で未 push）:
  書き込みの記録があるのは `fb65b6f`（2026-11-01、控え `~/stm32_backup_20261101/`）まで。それより後の自動停止の予想進路（11-02）、
  TC の作り直し（`TractionLimiter`、止めない PI）と過速度ガードの分離（11-09）は**書き込みの記録がない**
- **MD ファーム**（`~/GitHub/BLDC/ProgramV4`、ブランチ `FW/Program_V4`）: 仕様は `ProgramV4/docs/SERIAL_PROTOCOL.md`
- **プロトコル v0.29**（`raspi/proto/protocol.toml` の `protocol_version` = 0x001D が正。ファームの `RAS_PROTOCOL_VERSION` も 0x001D）。
  Pi とファームは一緒に入れ替える（io_node は版が一致するまで ARM を通さない）
- **大会**: toyota の予選は 5 位で通過（2026-11-08）。本番コースはシムの `toyota2` と同じ形。次の大会 magp はシムの `magp2`
- **自動運転**: 主力は `slam2d_route`（`slam2d_raceline` を土台に経路選択・経由点）。障害物・他車は `nav/overtake.py`（Frenet 格子）、
  抜けない静止物は Hybrid A*。走行中の学習は `raspi/nav/grip.py`（`GripLearner`＝横の上限をタイムで、`LongLearner`＝前後の包絡）と
  `raspi/nav/understeer.py`（アンダーステア勾配 K）。GUI の自動運転モードに「レーシングライン(slam2d)」は無い
- **カメラ模倣学習**: `cam_e2e` は v9 まで（`models/cam_e2e/`）。障害物では止まらない。`ml_lidar/` はシムでの新しい学習を止めている
- **E2E 融合（`e2e_fusion`、2026-11-11 に追加・未コミット・Pi 未反映・実走していない）**: 点群＋前方カメラ＋車両状態 → 前方の経路点
  （舵・速度の直接出力は 11-11 に外した。v1・v2 は学習済みの重みからそのヘッドだけ捨てて書き出し直した。再学習はしていない）。
  学習 `ml_e2e/`、推論 `raspi/nodes/e2e_fusion_node.py`（新サービス `surge-e2e-fusion`）、走行 `raspi/auto/e2e_fusion.py`（経路点を推測航法で
  ずらして追う）。操作パネルは launcher の「ml_e2e」（`ml_e2e/app.py`）、手順は [`docs/development.md` §12.4](docs/development.md)。モデル v1（`models/e2e_fusion/v1`）は 10-31〜11-08 の記録 18 本
  （3 コース・16038 枚、うち 15 本で学習）。学習に使っていない 3 本での値: 経路の誤差は最初の 2m で 0.07〜0.17m、全 8m で 0.25〜0.49m、
  向きのずれへの応答は理想の 0.58〜0.77 倍、planner の目標速度は手本の 0.84〜0.99 倍。Pi（DISARM・1.5GHz）で前処理 10ms＋推論 8.6ms、
  10Hz で制御系 +20mA。段階の計画は `~/.claude/plans/slam-e2e-ai-lidar-imu-cosmic-kettle.md`（段階0〜1 の実走前まで済み）

## 実機で確かめること

- **11-09 の変更**: 走行中に GUI で解除して経路に沿って止まるか、解除後の再特定にかかる時間、発進の速さと壁との余裕、
  ヘアピンで止まらないか（車体のすぐ外の点を障害物にしない）、LiDAR の強度の分布（窓際・ヘアピン）と捨てた点の数、遮られたときに見失わないか
- **STM32 の TC の作り直し**: 書き込んで、校正 8（`calib_tc_gain`）を録り直す。TC のゲインは旧い数値最適化の値のまま
- **10-31〜11-01 の修正**: ARM・駆動・走行での確認（舵の見張り・PI 減速の ABS・odom の手押し・解除後の再特定・車体内の点のマスク）
- **駐車**: Hybrid A* と毎周期の局所地図を C に書き直してから、実機ではまだ走らせていない
- **舵が追従しない**（`surge_20261031_062217`・`surge_20261102_094700`）: しばらく置いた後の最初の操舵で、ステア MD が 4.8A で押しても舵が動かないか逆へ動く。
  `steer_guard` で止めて待つようにしたが、原因（ステア MD のファーム・エンコーダ）は調べていない

- **E2E 融合（`e2e_fusion`）**: Pi で `install_services.sh`（`surge-e2e-fusion` を作る）→ 低速で engage して周回できるか、
  自動運転タブのモデル欄で選んだモデルに切り替わるか（`reason` と Pi のログ `# モデル切替`）、ARM 中の制御系電流の基準値と推論中の増分、`e2e_fusion/plan` の遅れ（`reason` に出る）

## 原因が分かっていないこと

- シムで学習した K が真値 2e-3 に対して 3.4e-3 になる（11-06 の朝は 2.09e-3 だった）
- 実機の K が 1 本の走行の中で時間とともに下がる（`surge_20261106_024246`: 2.37 → 1.09e-3。加減速の標本を外す修正の後は収まった）
- シム toyota2 の速いコーナー（舵 0.1〜0.15・外へ 8cm）で 1 件当たる
- 実車のカメラ推論が撮影から配信まで 191ms・6.4Hz（設定は 10Hz）

大会に向けた残りの作業は [`docs/competition_plan.md`](docs/competition_plan.md)。
Pi への接続・反映・トラブルシューティングは [`docs/development.md`](docs/development.md)、接続情報の機密は `docs/setup_credentials.md`（gitignore 済み）。
