# 制御定数の棚卸し — どの数字が物理・同定・経験則か

制御系（STM32 の `MainF446RE_V3/src/control`・`src/vehicle` と、Pi の速度指令・舵の追従・シム）に出てくる
定数を、**どこから来た数字か**で分けた一覧。2026-10-17 に両リポジトリを走査して作り、2026-10-18 に
同じ種類の事例（試験の無い手置きのゲイン・一時的な記録で決めた値・未校正のセンサ）を洗い直した（§1.2・§6）。

> **2026-10-19: 同定は車両の校正（[`calibration.md`](calibration.md)、`tools/calib`）に作り直した。** 下の表の旧ツール名の
> 読み替え: `tools/sysid`・`tools/ctrl_tune` の同定 → `tools/calib`（校正 1〜8）、`tools/ctrl_tune` のファームのシム → `tools/fwsim`、
> ⓪ → 校正 1、⑤ → 校正 5、`sysid_wheel`・`sysid_tyre` → 校正 6・7。②前後運動・③旋回は試験ごと消し、その値（`mu`・
> `drive_accel_m_s2`・`brake_decel_m_s2` など）はシム専用の `[sim]` へ、slam2d の上限は `[limits]`（校正 7・8 が書く）へ移した。
> 車速 PI のゲインは極配置（ζ・ωn）ではなく位相余裕 60° の式（`tools/calib/loop.py`）。§1〜§3 は 2026-10-17〜18 時点の記録

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

## 1. 今回直したもの（E/手置き → C/B）

| 場所 | 前 | 後 | 根拠 |
|---|---|---|---|
| ファーム `vehicle.h` 自動停止の減速度 `VEHICLE_AUTO_STOP_DECEL_M_S2` | 3.0（手置き。コメントに「楽観的、測り直したら更新」） | `VEHICLE_BRAKE_DECEL_M_S2` = **2.47**（`[dynamics] brake_decel_m_s2`、ABS 込みの制動の同定値）。★v0.29 で CONFIG_SET `BRAKE_DECEL_M_S2`（`[limits] abs_brake_decel_m_s2`）へ | 自動停止は制動モードで止める。停止距離は 2m/s で約 15cm 伸びる（安全側） |
| 同 自動停止の遅延 `VEHICLE_AUTO_STOP_DELAY_S` | 0.1（未実測のプレースホルダー） | `1/LIDAR_SCAN_FREQ_HZ + VEHICLE_MD_TORQUE_TAU_S` = 0.1045。★v0.29 で後ろは CONFIG_SET `MD_TORQUE_LAG_S`（`md_delay_s + md_tau_s`） | 点群の各ビンは 1 回転に 1 回しか更新されない（LD06 標準 10Hz）＋MD のトルクの立ち上がり（`[control.plant] md_tau_s`）。判定は STM32 内で閉じるので通信は通らない |
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

### 1.2 2026-10-18 に直したもの（試験・校正を足して B/C へ。★プロトコル v0.27。**実機未検証**、既定値は前と同じ動き）

| 場所 | 前 | 後 | 根拠・試験 |
|---|---|---|---|
| 前輪エンコーダ（AS5600）の読み | 校正なし（出力の範囲は全域と仮定、1回転の周期誤差は Pi の解析だけが記録ごとに推定） | `[control]` の `enc_*`（出力の範囲 lo・hi、周期誤差の1・2次の係数）。STM32 の `Encoder` が直してから車速・走行距離にする（`lib/encoder_cal`） | **⓪ エンコーダ校正試験**（`sysid_encoder` → `tools/sysid/encoder_cal.py`）。トルク0の惰行の区間で当てはめる（巡航は車速PIが読みの誤差に合わせてトルクを揺らし、真の動きが誤差を打ち消す） |
| 車速のカルマンフィルタの調整値 | `SPEED_KF_*`（σs 0.8mm は周期誤差を逃がす水増し、σa 0.6・jerk 100・バイアス 0.05 は 2026-10-16・17 の記録1本と経験） | `[control]` の `kf_*`（＋車速による加速度の雑音の増え方 `kf_sigma_a_per_m_s`） | `tools/sysid/kf_tune.py`: 走行の記録に STM32 と同じフィルタを通し、残差の尤度を最大にする（予測誤差分解）。結果の NIS・ρ で白色の前提を検定 |
| `[dynamics] speed_kp`・`speed_ki` | 0.3・0.6（手置き） | 狙い `speed_pi_zeta`・`speed_pi_wn_rad_s`（D）と利得 G から極配置: kp = 2ζωn/G、ki = ωn²/G（`toml_update.speed_pi_values`。G を書き直すと kp・ki も書き直る）。狙いの既定は手置きの値を今の G で割り戻した ζ 0.898・ωn 3.59 | **⑤ 車速PI試験**（`sysid_speed_pi` → `tools/sysid/speed_pi.py`）: トルク指令で往復し、PI を通さずに G・転がり抵抗（前後別）・トルクの遅れを測る。測った遅れで位相余裕 45° を保てる ωn の上限を示す。過去の記録の解析は記録時の kp・ki（`/diag/link`）で当てはめる（`Log.speed_pi`） |
| シムの STM32 の自動停止の定数（`sim/stm32.py`） | 遅延 0.1・減速度 3.0・DISARM 制動の上限 3.0s（ファームの旧値のまま） | ファームの式と同じ（0.1045・2.47・3.04s）。`test_fw_constants.py` が突き合わせる | ファームを直したときにシムの写しが置き去りになっていた |
| シムのトルク指令の加速度（`sim/vehicle.py`） | 名目の 1/(車輪半径×質量) | 車速PIと同じ車体の利得 `speed_plant_gain`×駆動輪数 | 同じトルクで車速指令とトルク指令の加速が約3割食い違っていた |
| 同定の試験の停止距離の減速度 `_sysid_common._STOP_DECEL_M_S2` | コメントの出典が廃止された `DRIVE_MAX_ACCEL_M_S2` | 値は同じ 2.4。同定した制動の減速度 2.47 より小さい＝安全側、と根拠を書き直した | 同定の前にも回すので、同定値には頼らない |

## 2. 維持するもの

### A 物理（`vehicle.toml` が正。ファームの写しはテストで突き合わせる）

- 幾何: `wheelbase` 0.23・`track` 0.155・`wheel_radius` 0.03・`footprint`・センサ取付位置 — 2026-08-20 実測確定。
  ファームの `DRIVE_WHEELBASE_M`・`DRIVE_REAR_TRACK_M`・`DRIVE_*_WHEEL_RADIUS_M`・`VEHICLE_BODY_*`・`VEHICLE_*_X_M` は写し
- `mass` 2.0kg、`drive_ratio` 2（駆動輪数）、`GRAVITY` 9.81
- ステアモータの可動域 ±60°（`STEERING_MAX_ANGLE_RAD`）、リンク比 0.5（`STEERING_LINKAGE_RATIO`）
- MD の制動の境界層・不感帯（`brake_boundary_speed_m_s` 0.6・`brake_deadband_speed_m_s` 0.06）— MD ファームの定数
- トルク上限 `DRIVE_MAX_TORQUE_NM` 0.15（2026-10-20 に 0.13 から。MD の自動校正し直し（2026-10-18）で絶対上限が約 0.2 になったため。それ以前は Kt×10A ≒ 0.144〜0.148 で 0.15 は MD が受け付けず走れなかった）。Pi 側の唯一の定義は `raspi/msgs/convert.py` の `MAX_TARGET_TORQUE_NM`（GUI・toml・シム・tools はここと突き合わせ/参照。`test_fw_constants.py`）。`speed_torque_max_nm` はその 2 輪分
- 前輪エンコーダ AS5600・IMU MPU6050 のデータシート値（`speed_kf.h` のコメント）

### B 同定（車両の校正 `tools/calib` が書く。どの校正がどの値かは `config/vehicle.toml` の各行と `calibration.md` §2）

- `[dynamics]` の舵・ヨーの遅れ・車体の利得・転がり抵抗・遅延（校正 2〜5）
- `[control]` の `steer_link_*`・`max_steer`（校正 2）、`enc_*`（校正 1）、`kf_*`（`tools/calib/kf.py`）
- `[control]` の TC・ABS の目標とゲイン（校正 7。位相余裕の式）、`[control.plant]`（校正 6・7）
- `[limits]`（slam2d の上限。校正 7・8）
- 自動停止・DISARM 後の制動の見積もり: `[limits] abs_brake_decel_m_s2`（校正 9 の ABS の制動）と
  `[control.plant] md_delay_s + md_tau_s`（校正 6）を、io_node が CONFIG_SET（★v0.29 `BRAKE_DECEL_M_S2`・
  `MD_TORQUE_LAG_S`）で STM32 へ送る。**ファームは写しを持たない**（2026-10-19 に `VEHICLE_BRAKE_DECEL_M_S2`・
  `VEHICLE_MD_TORQUE_TAU_S` を削除。校正し直しても toml を書いて `--restart-io` するだけ）

### C 式（手で書かない）

- `speed_ff_nm_per_m_s2` = 1/`speed_plant_gain`、`speed_ff_rr_nm` = `rolling_resistance`/`speed_plant_gain`、
  `launch_exit_torque_nm`、`speed_kp`・`speed_ki`（位相余裕 60° の式）— 校正 5（`tools/calib/speed.py`）が書く
- `max_steer`（リンクの換算を可動域に通した値）— 同上
- `Vehicle.kappa_max`・`a_lat_max`・`a_accel_max`・`a_decel_max`・`speed_loop_tau_s`・`speed_loop_reset_s`・`steer_delay_s` — `raspi/core/vehicle.py` の派生量
- `tv_load_gain_s2_per_m` 0.045 = `load_transfer` × `mass` ÷ 後輪1つの静止荷重（机上の式。実機未調整）

## 3. 残しているもの — 方針（D）と経験則（E）、置き換えの手段

### ファーム

| 定数 | 値 | 分類 | 根拠・次の手 |
|---|---|---|---|
| `[dynamics] speed_pi_zeta`・`speed_pi_wn_rad_s` | 0.898・3.59 | D | 車速PIの狙い（§1.2）。ωn の上限は ⑤ が測った遅れから位相余裕で示す。上げるかは追従の速さと車速の雑音（kp に比例してトルクに乗る）の釣り合いで人が決める |
| `DRIVE_ANTIWINDUP_TT_S` | 0.10 | E | 積分の巻き戻しの時定数。教科書の目安は Ti = kp/ki（今 0.5s）の 0.1〜1 倍。`ctrl_tune.bench` で比べてから式にする |
| `DRIVE_STANDSTILL_SPEED_M_S` | 0.05 | D | 停車の判定。車速の推定の雑音（静止 σ ≒ 2mm/s）より十分上 |
| `DRIVE_LAUNCH_END_MARGIN_M_S` | 0.05 | E | コメントの根拠「前輪 LPF τ≈10ms の遅れ×全開の加速度」は車速がカルマンフィルタになって古い。今の式なら 加速度×(加速度計の DLPF 3ms＋MD 4.5ms) ≒ 0.02。実機のローンチの記録で確かめてから直す |
| `DRIVE_LAUNCH_TIMEOUT_S`・`_MIN_STEP_M_S`・`_REVERSE_SPEED_M_S` | 3.0・0.5・0.3 | D | 発進を始める・打ち切る条件 |
| `DRIVE_ABS_FALLBACK_LIMIT_RATIO`・`_TIME_S` | 0.2・0.2 | D | 前輪エンコーダ故障時に ABS が制動を抜き続けないための見張り |
| `tc_slip_target`・`abs_slip_target` | 校正 7 が決める | D | 曲線の山の 95% の力が出るスリップ率（`tools/calib/tyre.py`。前後の力を優先し、横のための上限は付けない = 2026-10-19 の決定）。ゲインは校正 8（`tools/calib/tc_gain.py`）が実機で決める |
| `slip_speed_floor_m_s`・`wheel_lift_diff_threshold_m_s` | 0.5・0.35 | E | 低速のスリップ率の分母、片輪浮きの判定。前輪の読みの誤差（タイヤ径・周期誤差 数%）より上に置く。後者は車速に比例させる（例: 7%×車速＋下限）のが物理的 |
| `tv_max_ratio` | 0.5 | D | 内輪のトルクを残す上限 |
| `[control] kf_*`（`SPEED_KF_*` は既定値） | 0.8mm・0.6・0・100・0.05・0.15 | B（`kf_tune` が書く。今の値は v0.26 の経験値のまま） | §1.2。⓪ の校正を入れた後の記録で推定し直す。`kf_lever_arm_m` は旋回を含む記録のときだけ推定（IMU の取付位置を実測すれば一部は A にできる） |
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
- （2026-10-19 まで）`[control.plant] md_tau_s` とファームの `VEHICLE_MD_TORQUE_TAU_S` の写しがあった。v0.29 で
  ファームの定数を消し、`md_delay_s + md_tau_s` を CONFIG_SET で送る形にした

## 5. 校正し直したときの手順（ファームの写しがあるもの）

校正で変わる値のファームの写しは無い（2026-10-19、v0.29 で制動の減速度・MD の遅れも CONFIG_SET にした）。
toml を書いて `./tools/deploy.sh --restart-io` するだけ。ファームが写しを持つのは車体寸法（`DRIVE_WHEELBASE_M` ほか、
校正では変わらない）だけで、変えたら `python -m pytest tools/fwsim/tests/test_fw_constants.py` が食い違いを知らせる

## 6. 再調査（2026-10-18）— 同じ種類で、まだ残っているもの

§1.2 の3つ（車速PIのゲイン・KF の調整値・エンコーダの校正）と同じ形の事例を両リポジトリで探した。**直したものは §1.2**。
以下は残っているもの（値は動かしていない）。上ほど走りへの効きが大きい。

| 対象 | 今の値・状態 | 種類 | 効き | 手段（案） |
|---|---|---|---|---|
| ジャイロの感度（MPU6050） | データシートの 131LSB/(°/s) のまま。バイアスだけ静止中に追従 | 未校正のセンサ | 感度の公差 ±3% がそのままヨーレート・曲率の倍率になる。舵の効き・アンダーステア勾配の同定（`fit_geometry`）が吸ってしまい、旋回の向きの積分（SLAM の初期値・推測航法）に残る | その場で N 回転させ、LiDAR のスキャンの回転角と積分を比べる（LiDAR のスキャンが基準。試験を足す） |
| LiDAR の取付の向き（`[sensors.lidar] yaw`） | 0（未校正） | 未校正のセンサ | 点群全体が回る。SLAM の地図・自動停止の矩形が向きの誤差ぶんずれる | 直線の往復（⓪ の記録にもスキャンは入っている）で、スキャンどうしを合わせた移動の向き atan2(dy, dx) と車体の前（推測航法）の差 |
| IMU の取付の傾き（ヨー・ピッチ） | 軸変換だけ。ずれの校正なし | 未校正のセンサ | 加速度の x に横・上の加速度が混ざる。静止の分は KF のバイアスが吸うが、旋回の横加速度は `kf_lever_arm_m` が別の形で吸う | 直進の加減速と定常旋回で accel の y・x を回帰（取付角）。`kf_tune` に取付角を足す |
| MD の左右のトルク定数 | 指令をそのままトルクとみなす（`torque_cmd` は指令） | 未校正の駆動 | 同定の G は左右の平均。左右差はトルクベクタリングの差に乗る | ⑤ の記録で、トルク指令の左右差とヨーレートを回帰（直進中に小さな差を加える段を足す） |
| `scan2scan.PRIOR_XY` | 5mm（「倍率誤差 2%」の前提） | 一時的な前提 | 半径は実測済み・Encoder の dt の誤り（1.8%）も直ったので、倍率誤差の前提は古い | 今の走行で、照合の残差から決め直す |
| トルクベクタリングの利得 `tv_load_gain_s2_per_m` | 0.045（机上の式） | 手置き（式だが実機未調整） | 旋回中の左右配分 | `sysid_tyre`・`ctrl_tune` の旋回シナリオで最適化（他の TC・ABS と同じ手順） |
| ABS・TC・片輪浮き対策のしきい値・ゲイン | 車両モデル上で最適化した机上値 | 試験はあるが実機で未検証 | 限界域の挙動 | 校正 6・7・8（`calib_wheel`・`calib_tyre`・`calib_tc_gain`）を実機で録り直す（手順は `docs/calibration.md`） |
| `LIDAR_PWM_DUTY` 0.4 | 目で見て合わせた（「回転数を見ながら詰めること」） | 手置き | 回転数（10Hz からずれると 1 周の点の密度と自動停止の遅延が変わる） | LD06 が回転速度を送ってくる（`Scan.rot_speed_dps`）ので、STM32 で PI にして 3600°/s に合わせるか、記録から duty→回転数を当てはめる |
| AHRS の積分の上限・`IMU_ACCEL_LPF_K` | 仮置きの値・係数で持つ（周期を前提にした係数） | 手置き | ピッチ・ロールの推定 | 時定数で持つ形に（`LPF_UpdateTau` と同じ直し方）、静止・傾けた台での記録で確かめる |
| ESS・ブレーキ灯のしきい値 | 記録1本から | 一時的な記録（表示のみ） | 走りには効かない | 録り直すときに一緒に見る（D のまま） |
| 後ろカメラの魚眼の校正・`bottom_crop` | 前カメラの値の流用（★仮） | 未校正のセンサ | 後ろカメラを使う機能（後退の駐車・カメラE2E） | `development.md` §4.6 の手順で後ろカメラ単独で校正 |
| 電圧・電流の ADC の換算（`ADC_VREF` ほか） | 公称 3.3V・分圧比の設計値 | 未校正のセンサ | 電池の残量・過電流の判定 | テスタで2点（満充電・放電後）を測って倍率を出す |
| `wheel_width` | 概算 | 未実測の寸法 | 外形・接触（小） | ノギスで測る |
| `telemetry_node` の合図の既定値 | `arrow_signal_node` と手で同じ値に合わせている | 写し（二重定義） | 片方を変えると食い違う | 1 か所に寄せる（定数の共有） |
