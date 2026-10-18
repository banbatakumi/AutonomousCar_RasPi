# 制御定数の棚卸し — どの数字が物理・同定・経験則か

制御系（STM32 の `MainF446RE_V3/src/control`・`src/vehicle` と、Pi の速度指令・舵の追従・シム）に出てくる
定数を、**どこから来た数字か**で分けた一覧。2026-10-17 に両リポジトリを走査して作った。

方針: 当てずっぽうの定数・一時的な走行ログから決めた値を残さず、**物理法則（式）・データシート・
システム同定**のどれかに根拠を置く。根拠を置けないもの（方針・経験則）は、そうだと分かる名前と
コメントで残し、同定・検証の手段を書く。

| 分類 | 意味 | 扱い |
|---|---|---|
| **A 物理** | 車体寸法・質量・データシートの値・物理定数 | 維持。`config/vehicle.toml` が正 |
| **B 同定** | `tools/sysid`・`tools/ctrl_tune` が実機の記録から求める値 | 維持。同定し直すとツールが書き直す |
| **C 式** | A・B から式で決まる値 | **手で書かない**。式で持つか、ツールが書く |
| **D 方針** | 物理からは決まらない設計上の選択（安全の余裕・前後力と横グリップの配分など） | 名前を付けて残す。根拠をコメントに |
| **E 経験則** | シム・実機の試行で合わせた値 | 残すが、同定・検証の手段を併記。置き換え候補 |

ファームは `vehicle.toml` を読めないので、A・B の一部を `#define` に写している。写しが正と食い違わないことは
`tools/ctrl_tune/tests/test_fw_constants.py` が確かめる（ファームのリポジトリが隣にあるときだけ走る）。

---

## 1. 今回直したもの（E/手置き → C/B）

| 場所 | 前 | 後 | 根拠 |
|---|---|---|---|
| ファーム `vehicle.h` 自動停止の減速度 `VEHICLE_AUTO_STOP_DECEL_M_S2` | 3.0（手置き。コメントに「楽観的、測り直したら更新」） | `VEHICLE_BRAKE_DECEL_M_S2` = **2.47**（`[dynamics] brake_decel_m_s2`、ABS 込みの制動の同定値） | 自動停止は制動モードで止める。停止距離は 2m/s で約 15cm 伸びる（安全側） |
| 同 自動停止の遅延 `VEHICLE_AUTO_STOP_DELAY_S` | 0.1（未実測のプレースホルダー） | `1/LIDAR_SCAN_FREQ_HZ + VEHICLE_MD_TORQUE_TAU_S` = 0.1045 | 点群の各ビンは 1 回転に 1 回しか更新されない（LD06 標準 10Hz）＋MD のトルクの立ち上がり（`[control.plant] md_tau_s`）。判定は STM32 内で閉じるので通信は通らない |
| 同 センサの取付オフセット `VEHICLE_AUTO_STOP_*_OFFSET_CM` | 23・14・4・3（手で引き算した結果） | 車体外形・センサ位置（`VEHICLE_BODY_*`・`VEHICLE_*_X_M`）の差 | 値は同じ。寸法を変えたら追従する |
| 同 LiDAR の矩形の半幅 `VEHICLE_AUTO_STOP_LIDAR_HALF_WIDTH_CM` | 15（9＋6 を手計算） | `VEHICLE_BODY_HALF_WIDTH_M + VEHICLE_AUTO_STOP_LATERAL_MARGIN_M`（D: 余裕 6cm） | 値は同じ。余裕を名前の付いた方針値に分離 |
| 同 DISARM 後の制動の上限 `VEHICLE_DISARM_BRAKE_TIMEOUT_S` | 3.0（「2.5m/s² で 2 秒」の手計算） | `DRIVE_MAX_SPEED_M_S / VEHICLE_BRAKE_DECEL_M_S2 × VEHICLE_STOP_TIME_MARGIN`（D: 1.5）= 3.04 | 最高速から同定した減速度で止まり切れる時間 |
| ファーム `drive.h` 各輪の周速のローパス `DRIVE_LPF_K_FRONT/REAR` | 係数 0.951・0.90（2kHz を前提にした係数） | 時定数 `DRIVE_LPF_TAU_FRONT_S` 9.95ms・`_REAR_S` 4.75ms。係数は毎周期 `exp(-dt/τ)`（`LPF_UpdateTau`。2kHz で元の 0.951・0.90 に一致） | 周期を変えても時定数が変わらない。`ctrl_tune.bench` で HEAD と同等を確認 |
| 同 加速度FFのローパス `DRIVE_SPEED_FF_TAU_S` | 0.02（Pi・シムに同じ数字を手書き、計 3 箇所） | `DRIVE_HOST_COMMAND_PERIOD_S`（Pi の指令周期 50Hz）。Pi 側は `[dynamics] speed_ff_filter_s` 1 箇所 | 目標の刻みの周期でならすのが目的なので、時定数はその周期そのもの |
| Pi `speedctl.REF_CATCH_S` | 0.15（手置き） | `Vehicle.speed_loop_tau_s` = 1/(車体の利得×kp) = 0.155（呼び出し側が渡す） | PI が偏差 Δv を詰める初めの傾きは Δv/τ |
| Pi `speedctl.LEAD_ACC_S` | 0.05（手置き） | `cmd_delay_s` = 指令の半周期＋通信（同定の遅延試験）= 0.023 | 「指令が届くまでの遅れぶんだけ前倒し」の定義どおり。ファームのFFが無効なときだけ使う旧方式 |
| Pi `slam2d_raceline._ref_lead` の `0.01` | 50Hz の半周期を数字で | `0.5 / CMD_HZ` | 指令周期を `raspi/auto/base.py` の `CMD_HZ` 1 箇所に（planning_node・telemetry_node も読む） |
| Pi MPC の遅延補償 `mpc_delay_s` | 0.06（GUI のスライダ。シムで 0.05〜0.07 が最良） | Pure Pursuit と同じ `steer_delay()` = 同定した舵・ヨーの遅れ＋通信 ≒ **0.053** | 両者とも「今」へ進めた姿勢を使うので、見込む遅れは同じ。手で合わせた最良域と同定値が一致した |
| `[control] launch_exit_torque_nm` | 0.009（旧同定値のまま。コメントに「手で直す」） | `rolling_resistance ÷ speed_plant_gain ÷ drive_ratio` = **0.006346**。`tools/sysid` が同定の適用時に書き直す（`launch_exit_value`） | その車速を保つのに要る 1 輪あたりのトルク |
| シムの車速PIの帰還・`speed` の遅れ | `speed_filter_s` 9.95ms（v0.24 までの前輪ローパス） | `[dynamics] speed_kf_lag_s` = 0（ホストでコンパイルしたファームのカルマンフィルタを測った遅れ 0.1〜0.3ms）。`speed_filter_s` は `wheel_speed` だけ | v0.25 から実機の `speed` はカルマンフィルタ。§4 |

## 2. 維持するもの

### A 物理（`vehicle.toml` が正。ファームの写しはテストで突き合わせる）

- 幾何: `wheelbase` 0.23・`track` 0.155・`wheel_radius` 0.03・`footprint`・センサ取付位置 — 2026-08-20 実測確定。
  ファームの `DRIVE_WHEELBASE_M`・`DRIVE_REAR_TRACK_M`・`DRIVE_*_WHEEL_RADIUS_M`・`VEHICLE_BODY_*`・`VEHICLE_*_X_M` は写し
- `mass` 2.0kg、`drive_ratio` 2（駆動輪数）、`GRAVITY` 9.81
- ステアモータの可動域 ±60°（`STEERING_MAX_ANGLE_RAD`）、リンク比 0.5（`STEERING_LINKAGE_RATIO`）
- MD の制動の境界層・不感帯（`brake_boundary_speed_m_s` 0.6・`brake_deadband_speed_m_s` 0.06）— MD ファームの定数
- トルク上限 `DRIVE_MAX_TORQUE_NM` 0.13 — MD の絶対上限（Kt×10A ≒ 0.144〜0.148）より下。`speed_torque_max_nm` はその 2 輪分
- 前輪エンコーダ AS5600・IMU MPU6050 のデータシート値（`speed_kf.h` のコメント）

### B 同定（`tools/sysid`・`tools/ctrl_tune` が書く）

- `[dynamics]` の舵・ヨー・前後運動・グリップ・遅延（`tau_steer_s`・`dead_time_s`・`steer_servo_gain`・
  `yaw_natural_freq_rad_s`・`speed_plant_gain`・`rolling_resistance`・`mu`・`drive_accel_m_s2`・
  `brake_decel_m_s2`・`control_latency_s` ほか）
- `[control]` の `steer_link_*`（リンクの換算）・`max_steer`
- `[control]` の TC・ABS のゲイン（`tc_kp/ki`・`abs_kp/ki`）— 同定した車両モデル `[control.plant]` とファームの制御を閉ループにして最適化
- `[control.plant]`（タイヤ・荷重移動・MD の時定数）
- ファームの写し: `VEHICLE_BRAKE_DECEL_M_S2`（← `brake_decel_m_s2`）・`VEHICLE_MD_TORQUE_TAU_S`（← `md_tau_s`）。
  **同定し直したらファームも直す**（テストが食い違いで落ちる）

### C 式（手で書かない）

- `speed_ff_nm_per_m_s2` = 1/`speed_plant_gain`、`speed_ff_rr_nm` = `rolling_resistance`/`speed_plant_gain`、
  `launch_exit_torque_nm`（上表）— `tools/sysid/toml_update.py` が書く
- `max_steer`（リンクの換算を可動域に通した値）— 同上
- `Vehicle.kappa_max`・`a_lat_max`・`a_accel_max`・`a_decel_max`・`speed_loop_tau_s`・`speed_loop_reset_s`・`steer_delay_s` — `raspi/core/vehicle.py` の派生量
- `tv_load_gain_s2_per_m` 0.045 = `load_transfer` × `mass` ÷ 後輪1つの静止荷重（机上の式。実機未調整）

## 3. 残しているもの — 方針（D）と経験則（E）、置き換えの手段

### ファーム

| 定数 | 値 | 分類 | 根拠・次の手 |
|---|---|---|---|
| `[dynamics] speed_kp`・`speed_ki` | 0.3・0.6 | **E** | 同定しない手置きのゲイン。今の同定値では閉ループの固有角振動数 √(G·ki) = 3.6rad/s・減衰比 G·kp/(2√(G·ki)) = 0.90 に当たる。**次の候補**: 減衰比と帯域（MD の時定数・車速の推定の遅れで上限が決まる）を方針値として持ち、利得 G から極配置で求める。ただし同定（`tools/sysid/fit.py`）は記録時の kp・ki を前提に利得を当てはめるので、自動で書き直すと過去の記録の解析が狂う——ゲインを記録に残してから行うこと |
| `DRIVE_ANTIWINDUP_TT_S` | 0.10 | E | 積分の巻き戻しの時定数。教科書の目安は Ti = kp/ki（今 0.5s）の 0.1〜1 倍。`ctrl_tune.bench` で比べてから式にする |
| `DRIVE_STANDSTILL_SPEED_M_S` | 0.05 | D | 停車の判定。車速の推定の雑音（静止 σ ≒ 2mm/s）より十分上 |
| `DRIVE_LAUNCH_END_MARGIN_M_S` | 0.05 | E | コメントの根拠「前輪 LPF τ≈10ms の遅れ×全開の加速度」は車速がカルマンフィルタになって古い。今の式なら 加速度×(加速度計の DLPF 3ms＋MD 4.5ms) ≒ 0.02。実機のローンチの記録で確かめてから直す |
| `DRIVE_LAUNCH_TIMEOUT_S`・`_MIN_STEP_M_S`・`_REVERSE_SPEED_M_S` | 3.0・0.5・0.3 | D | 発進を始める・打ち切る条件 |
| `DRIVE_ABS_FALLBACK_LIMIT_RATIO`・`_TIME_S` | 0.2・0.2 | D | 前輪エンコーダ故障時に ABS が制動を抜き続けないための見張り |
| `tc_slip_target`・`abs_slip_target` | 0.2・0.25 | D | 前後力と横グリップの配分の方針（`tools/ctrl_tune/optimize.py` の冒頭。`tradeoff()` の表で人が決める） |
| `slip_speed_floor_m_s`・`wheel_lift_diff_threshold_m_s` | 0.5・0.35 | E | 低速のスリップ率の分母、片輪浮きの判定。前輪の読みの誤差（タイヤ径・周期誤差 数%）より上に置く。後者は車速に比例させる（例: 7%×車速＋下限）のが物理的 |
| `tv_max_ratio` | 0.5 | D | 内輪のトルクを残す上限 |
| `SPEED_KF_JERK_PSD`・`BIAS_WALK_PSD` | 100・0.05 | E | カルマンフィルタのプロセス雑音。**検定の手段がある**: TELEMETRY の NIS・自己相関（`raspi/tools/kf_consistency.py`）が 1・0 に近づくよう合わせる |
| `SPEED_KF_SIGMA_S_M`・`SIGMA_A_M_S2` | 0.8mm・0.6 | B（記録から）| AS5600 の INL・走行中の残差から（`speed_kf.h` のコメント）。同じ検定で見直す |
| `SPEED_KF_LEVER_ARM_M` | 0.15 | B（記録から）| 走行記録の回帰（IMU の取付位置＋横滑りの和）。IMU の取付位置を実測すれば一部は A にできる |
| `STEERING_LINKAGE_MIN_SLOPE` | 0.3 | D | リンクの換算が単調であるための下限（Pi の `SteerMap.command` は 0.2 で打ち切るが、ファームが 0.3 以上を保証するので届かない） |
| 自動停止の `VEHICLE_AUTO_STOP_LATERAL_MARGIN_M`・`_LIDAR_MIN_POINTS`・`_ULTRASONIC_NEAR_CM` | 6cm・3点・5cm | D | 誤検知と見逃しの釣り合い |
| `VEHICLE_STOP_TIME_MARGIN` | 1.5 | D | 低速で MD の制動が tanh で弱まるぶん（0.33m/s 以下で同定の減速度を下回る。延びは約 2cm で、自動停止では上位のマージンが持つ） |
| ブレーキホールド・ESS・ブレーキ灯のしきい値 | — | D | 制御ではなく表示・保持の条件（`vehicle.h` のコメント） |

### Pi

| 定数 | 値 | 分類 | 根拠・次の手 |
|---|---|---|---|
| `speedctl.LEAD_RATIO` | 0.65 | E | 減速の前倒し（ファームのFFが無効なときの旧方式だけ）。今は FF が有効なので使われない |
| `speedctl.REF_ACCEL_MIN`・`REF_ACCEL_GAIN` | 0.3・1.1 | D | ランプの下限と、プロファイルより少し速く追う余裕 |
| `speedctl.CATCH_UP_M` | 6.0 | D | 追いつく速度を引き直す先の距離（停止から最高速までの加速距離 v²/(2a) ≒ 4.5m より長く） |
| `slam2d_raceline.FIXED_*_USE`・`LEARN_BRAKE_INIT` | 0.9・0.95・0.9・0.75 | D | 路面の限界を学習しないときに同定値に掛ける余裕 |
| Pure Pursuit の `look_k`・`look_min`・`speed_preview`・`steer_ff`、MPC の重み `q_y`・`q_psi`・`r_d` | GUI の設定 | E | 追従の設計値（シム `sim.bench` で比べて決めた）。遅延補償だけは同定値から決まる（上表） |
| `Vehicle` の未同定時の既定（`a_lat_max` 3.5・加減速 2.5・`cmd_latency` 0.02 等） | — | D | 同定値が無いときに控えめに走るための値。同定後は使われない |

## 4. 食い違いと、その後の扱い

- **シムの車速の遅れ（直した）。** v0.25 から実機の `speed`（車速PIの帰還にも使う）はカルマンフィルタの車速で、
  前輪のローパス（`speed_filter_s` 9.95ms）を通るのは `wheel_speed` だけになった。シムは `speed` にも車速PIの帰還にも
  9.95ms のローパスを掛けていたので、新しく `[dynamics] speed_kf_lag_s` を設けて `speed`・帰還はそちらで遅らせ、
  `speed_filter_s` は `wheel_speed` だけにした。値はホストでコンパイルしたファームを車両モデルで回して測った遅れ
  （加減速で 0.1〜0.3ms、加速度計に 5ms の遅れを入れても）で 0。`test_fw_constants.py` が測り直して突き合わせる。
  **2026-09-28 の前後運動の同定値はローパスを帰還に使っていた旧ファームで測ったもの**なので、v0.26 のファームで
  前後運動試験を取り直すまでは、シムの速度ループがわずかに（帰還の遅れ 10ms ぶん）実機と違う。
  同定ツールのベンチ（`tools/sysid/bench.py`）も記録の `speed` をカルマンフィルタに見立てて作る
- `[control.plant] md_tau_s` 4.54ms と `VEHICLE_MD_TORQUE_TAU_S` 4.5ms は丸めの範囲で一致（テストの許容 0.1ms）

## 5. 同定し直したときの手順（ファームの写しがあるもの）

1. `tools/sysid`（前後運動試験）で `brake_decel_m_s2` が変わったら、ファームの `VEHICLE_BRAKE_DECEL_M_S2` を直す
2. `tools/ctrl_tune`（制御の同定）で `md_tau_s` が変わったら、`VEHICLE_MD_TORQUE_TAU_S` を直す
3. `python -m pytest tools/ctrl_tune/tests/test_fw_constants.py` が通ることを確かめてから書き込む
