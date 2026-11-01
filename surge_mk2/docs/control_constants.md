# 制御定数の棚卸し — どの数字が物理・同定・経験則か

制御系（STM32 の `MainF446RE_V3/src/control`・`src/vehicle` と、Pi の速度指令・舵の追従・シム）に出てくる
定数を、**どこから来た数字か**で分けた一覧。同定は車両の校正（[`calibration.md`](calibration.md)、`tools/calib`）、
ファームのホスト検証は `tools/fwsim`。

方針: 当てずっぽうの定数・一時的な走行ログから決めた値を残さず、**物理法則（式）・データシート・
システム同定**のどれかに根拠を置く。根拠を置けないもの（方針・経験則）は、そうだと分かる名前と
コメントで残し、同定・検証の手段を書く。

| 分類 | 意味 | 扱い |
|---|---|---|
| **A 物理** | 車体寸法・質量・データシートの値・物理定数 | 維持。`config/vehicle.toml` が正 |
| **B 同定** | 車両の校正（`tools/calib`）が実機の記録から求める値 | 維持。校正し直すとツールが書き直す |
| **C 式** | A・B から式で決まる値 | **手で書かない**。式で持つか、ツールが書く |
| **D 方針** | 物理からは決まらない設計上の選択（安全の余裕・前後力と横グリップの配分など） | 名前を付けて残す。根拠をコメントに |
| **E 経験則** | シム・実機の試行で合わせた値 | 残すが、同定・検証の手段を併記。置き換え候補 |

ファームは `vehicle.toml` を読めないので、A・B の一部を `#define` に写している。写しが正と食い違わないことは
`tools/fwsim/tests/test_fw_constants.py` が確かめる（ファームのリポジトリが隣にあるときだけ走る）。

---

## 1. 現在の値の出どころ

### A 物理（`vehicle.toml` が正。ファームの写しはテストで突き合わせる）

- 幾何: `wheelbase` 0.23・`track` 0.155・`wheel_radius` 0.03・`footprint`・センサ取付位置 — 実測確定。
  ファームの `DRIVE_WHEELBASE_M`・`DRIVE_REAR_TRACK_M`・`DRIVE_*_WHEEL_RADIUS_M`・`VEHICLE_BODY_*`・`VEHICLE_*_X_M` は写し
- `mass` 2.0kg、`drive_ratio` 2（駆動輪数）、`GRAVITY` 9.81
- ステアモータの可動域 ±60°（`STEERING_MAX_ANGLE_RAD`）、リンク比 0.5（`STEERING_LINKAGE_RATIO`）
- MD の制動の境界層・不感帯（`brake_boundary_speed_m_s` 0.6・`brake_deadband_speed_m_s` 0.06）— MD ファームの定数
- トルク上限 `DRIVE_MAX_TORQUE_NM` 0.15（MD の自動校正で絶対上限が約 0.2 になったため。Kt×10A ≒ 0.144〜0.148 の頃は 0.15 を MD が受け付けない）。Pi 側の唯一の定義は `raspi/msgs/convert.py` の `MAX_TARGET_TORQUE_NM`（GUI・toml・シム・tools はここと突き合わせ/参照。`test_fw_constants.py`）。`speed_torque_max_nm` はその 2 輪分
- 前輪エンコーダ AS5600・IMU MPU6050 のデータシート値（`speed_kf.h` のコメント）

### B 同定（車両の校正 `tools/calib` が書く。どの校正がどの値かは `config/vehicle.toml` の各行と `calibration.md` §2）

- `[dynamics]` の舵・ヨーの遅れ・車体の利得・転がり抵抗・遅延（校正 2〜5）
- `[control]` の `steer_link_*`・`max_steer`（校正 2）、`enc_*`（校正 1）、`kf_*`（`tools/calib/kf.py`、検定は `raspi/tools/kf_consistency.py`）
- `[control]` の TC・ABS の目標（校正 7）とゲイン（校正 8。実機で ωc を振って決める）、`[control.plant]`（校正 6・7）
- `[limits]`（slam2d の上限。校正 7・9）
- 自動停止・DISARM 後の制動の見積もり: `[limits] abs_brake_decel_m_s2`（校正 9 の ABS の制動）と
  `[control.plant] md_delay_s + md_tau_s`（校正 6）を、io_node が CONFIG_SET（`BRAKE_DECEL_M_S2`・
  `MD_TORQUE_LAG_S`）で STM32 へ送る。**ファームは写しを持たない**（校正し直しても toml を書いて `--restart-io` するだけ）

### C 式（手で書かない）

- `speed_ff_nm_per_m_s2` = 1/`speed_plant_gain`、`speed_ff_rr_nm` = `rolling_resistance`/`speed_plant_gain`、
  `launch_exit_torque_nm`、`speed_kp`・`speed_ki`（位相余裕 60° の式。`tools/calib/loop.py`）— 校正 5（`tools/calib/speed.py`）が書く
- `max_steer`（リンクの換算を可動域に通した値）— 同上
- `Vehicle.kappa_max`・`a_lat_max`・`a_accel_max`・`a_decel_max`・`speed_loop_tau_s`・`speed_loop_reset_s`・`steer_delay_s` — `raspi/core/vehicle.py` の派生量
- `tv_load_gain_s2_per_m` 0.02（机上の式 `load_transfer` × `mass` ÷ 後輪1つの静止荷重 = 0.045 から実機の滑りを見て下げた。内外の滑りがそろうかは未確認。`tv_enable` の既定は 0）

## 2. 方針（D）と経験則（E）、置き換えの手段

### ファーム

| 定数 | 値 | 分類 | 根拠・次の手 |
|---|---|---|---|
| `DRIVE_ANTIWINDUP_TT_S` | 0.10 | E | 積分の巻き戻しの時定数。教科書の目安は Ti = kp/ki（今 0.5s）の 0.1〜1 倍。`python -m tools.fwsim.bench` で比べてから式にする |
| `DRIVE_STANDSTILL_SPEED_M_S` | 0.05 | D | 停車の判定。車速の推定の雑音（静止 σ ≒ 2mm/s）より十分上 |
| `DRIVE_LAUNCH_END_MARGIN_M_S` | 0.05 | E | ファームのコメントの根拠「前輪 LPF τ≈10ms の遅れ×全開の加速度」は古い（車速はカルマンフィルタ）。今の式なら 加速度×(加速度計の DLPF 3ms＋MD 4.5ms) ≒ 0.02。実機のローンチの記録で確かめてから直す |
| `DRIVE_LAUNCH_TIMEOUT_S`・`_MIN_STEP_M_S`・`_REVERSE_SPEED_M_S` | 3.0・0.5・0.3 | D | 発進を始める・打ち切る条件 |
| `DRIVE_ABS_FALLBACK_LIMIT_RATIO`・`_TIME_S` | 0.2・0.2 | D | 前輪エンコーダ故障時に ABS が制動を抜き続けないための見張り |
| `tc_slip_target`・`abs_slip_target` | 校正 7 が決める | D | 曲線の山の 95% の力が出るスリップ率（`tools/calib/tyre.py`。前後の力を優先し、横のための上限は付けない）。ゲインは校正 8（`tools/calib/tc_gain.py`）が実機で決める |
| `slip_speed_floor_m_s`・`wheel_lift_diff_threshold_m_s` | 0.5・0.35 | E | 低速のスリップ率の分母、片輪浮きの判定。前輪の読みの誤差（タイヤ径・周期誤差 数%）より上に置く。後者は車速に比例させる（例: 7%×車速＋下限）のが物理的 |
| `tv_max_ratio` | 0.5 | D | 内輪のトルクを残す上限 |
| `[control] kf_*`（`SPEED_KF_*` は既定値） | 0.8mm・0.6・0・100・0.05・0.15 | B（`tools/calib/kf.py` が書く。今の値は経験値のまま） | 校正 1 を入れた後の記録で推定し直す。`kf_lever_arm_m` は旋回を含む記録のときだけ推定（IMU の取付位置を実測すれば一部は A にできる） |
| `STEERING_LINKAGE_MIN_SLOPE` | 0.3 | D | リンクの換算が単調であるための下限（Pi の `SteerMap.command` は 0.2 で打ち切るが、ファームが 0.3 以上を保証するので届かない） |
| 自動停止の `VEHICLE_AUTO_STOP_LATERAL_MARGIN_M`・`_LIDAR_MIN_POINTS`・`_ULTRASONIC_NEAR_CM` | 6cm・3点・5cm | D | 誤検知と見逃しの釣り合い |
| `VEHICLE_STOP_TIME_MARGIN` | 1.5 | D | 低速で MD の制動が tanh で弱まるぶん（0.33m/s 以下で同定の減速度を下回る。延びは約 2cm で、自動停止では上位のマージンが持つ） |
| ブレーキホールド・ESS・ブレーキ灯のしきい値 | — | D | 制御ではなく表示・保持の条件（`vehicle.h` のコメント） |

### Pi

| 定数 | 値 | 分類 | 根拠・次の手 |
|---|---|---|---|
| `speedctl.LEAD_RATIO` | 0.65 | E | 減速の前倒し（ファームの FF が無効なときだけ使う方式）。FF が有効なら使われない |
| `speedctl.REF_ACCEL_MIN`・`REF_ACCEL_GAIN` | 0.3・1.1 | D | ランプの下限と、プロファイルより少し速く追う余裕 |
| `speedctl.CATCH_UP_M` | 6.0 | D | 追いつく速度を引き直す先の距離（停止から最高速までの加速距離 v²/(2a) ≒ 4.5m より長く） |
| `slam2d_raceline.FIXED_*_USE` | 0.9・0.95・0.9 | D | 路面の限界を学習しないときに同定値に掛ける余裕（学習するときの加速・減速の出発点は校正値そのまま） |
| Pure Pursuit の `look_k`・`look_min`・`speed_preview`・`steer_ff`、MPC の重み `q_y`・`q_psi`・`r_d` | GUI の設定 | E | 追従の設計値（シム `sim.bench` で比べて決めた）。遅延補償 `mpc_delay_s` は Pure Pursuit と同じ `steer_delay()`（同定した舵・ヨーの遅れ＋通信） |
| `Vehicle` の未同定時の既定（`a_lat_max` 3.5・加減速 2.5・`cmd_latency` 0.02 等） | — | D | 同定値が無いときに控えめに走るための値。同定後は使われない |

## 4. 校正し直したときの手順（ファームの写しがあるもの）

校正で変わる値のファームの写しは無い（制動の減速度・MD の遅れも CONFIG_SET で送る）。
toml を書いて `./tools/deploy.sh --restart-io` するだけ。ファームが写しを持つのは車体寸法（`DRIVE_WHEELBASE_M` ほか、
校正では変わらない）だけで、変えたら `python -m pytest tools/fwsim/tests/test_fw_constants.py` が食い違いを知らせる

## 3. 未校正・手置きで残っているもの

上ほど走りへの効きが大きい。

| 対象 | 今の値・状態 | 種類 | 効き | 手段（案） |
|---|---|---|---|---|
| ジャイロの感度（MPU6050） | データシートの 131LSB/(°/s) のまま。バイアスだけ静止中に追従 | 未校正のセンサ | 感度の公差 ±3% がそのままヨーレート・曲率の倍率になる。舵の効きの校正（校正 2）が吸ってしまい、旋回の向きの積分（SLAM の初期値・推測航法）に残る | その場で N 回転させ、LiDAR のスキャンの回転角と積分を比べる（LiDAR のスキャンが基準。試験を足す） |
| LiDAR の取付の向き（`[sensors.lidar] yaw`） | 0（未校正） | 未校正のセンサ | 点群全体が回る。SLAM の地図・自動停止の矩形が向きの誤差ぶんずれる | 直線の往復（校正 1 の記録にもスキャンは入っている）で、スキャンどうしを合わせた移動の向き atan2(dy, dx) と車体の前（推測航法）の差 |
| IMU の取付の傾き（ヨー・ピッチ） | 軸変換だけ。ずれの校正なし | 未校正のセンサ | 加速度の x に横・上の加速度が混ざる。静止の分は KF のバイアスが吸うが、旋回の横加速度は `kf_lever_arm_m` が別の形で吸う | 直進の加減速と定常旋回で accel の y・x を回帰（取付角）。`kf_tune` に取付角を足す |
| MD の左右のトルク定数 | 指令をそのままトルクとみなす（`torque_cmd` は指令） | 未校正の駆動 | 同定の G は左右の平均。左右差はトルクベクタリングの差に乗る | 校正 5 の記録で、トルク指令の左右差とヨーレートを回帰（直進中に小さな差を加える段を足す） |
| `scan2scan.PRIOR_XY` | 5mm（「倍率誤差 2%」の前提） | 一時的な前提 | 半径は実測済み・Encoder の dt の誤り（1.8%）も直ったので、倍率誤差の前提は古い | 今の走行で、照合の残差から決め直す |
| トルクベクタリングの利得 `tv_load_gain_s2_per_m` | 0.02（机上の式 0.045 から実機の滑りで下げた） | 手置き（実機で調整中） | 旋回中の左右配分 | 同定しない方針。`tools/fwsim` の旋回シナリオと実機の滑りで合わせる |
| `LIDAR_PWM_DUTY` 0.4 | 目で見て合わせた（「回転数を見ながら詰めること」） | 手置き | 回転数（10Hz からずれると 1 周の点の密度と自動停止の遅延が変わる） | LD06 が回転速度を送ってくる（`Scan.rot_speed_dps`）ので、STM32 で PI にして 3600°/s に合わせるか、記録から duty→回転数を当てはめる |
| AHRS の積分の上限・`IMU_ACCEL_LPF_K` | 仮置きの値・係数で持つ（周期を前提にした係数） | 手置き | ピッチ・ロールの推定 | 時定数で持つ形に（`LPF_UpdateTau` と同じ直し方）、静止・傾けた台での記録で確かめる |
| ESS・ブレーキ灯のしきい値 | 記録1本から | 一時的な記録（表示のみ） | 走りには効かない | 録り直すときに一緒に見る（D のまま） |
| 後ろカメラの魚眼の校正・`bottom_crop` | 前カメラの値の流用（★仮） | 未校正のセンサ | 後ろカメラを使う機能（後退の駐車・カメラE2E） | `development.md` §4.6 の手順で後ろカメラ単独で校正 |
| 電圧・電流の ADC の換算（`ADC_VREF` ほか） | 公称 3.3V・分圧比の設計値 | 未校正のセンサ | 電池の残量・過電流の判定 | テスタで2点（満充電・放電後）を測って倍率を出す |
| `wheel_width` | 概算 | 未実測の寸法 | 外形・接触（小） | ノギスで測る |
| `telemetry_node` の合図の既定値 | `arrow_signal_node` と手で同じ値に合わせている | 写し（二重定義） | 片方を変えると食い違う | 1 か所に寄せる（定数の共有） |
