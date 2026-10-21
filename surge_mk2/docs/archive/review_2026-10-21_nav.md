# コードレビュー報告 — `raspi/nav/`（経路・追従系）2026-10-21

**対象**: `surge_mk2/raspi/nav/`（purepursuit, mpc, raceline, obstacles, hybrid_astar,
reeds_shepp, route, route_switch, centerline, drivable_path, local_map, grip, ipm, grid,
scan2scan, scan_anchor, se2, deskew, signals, roadgraph, avoid）＋
`raspi/core/vehicle.py` / `raspi/core/camera_model.py` / `raspi/auto/line_trace.py` /
`raspi/auto/base.py` / `raspi/auto/slam2d_raceline.py` / `raspi/nodes/{planning,telemetry}_node.py` /
`config/vehicle.toml` / `sim/vehicle.py`

**方法**: 全 22 モジュールと呼び出し側（planner → node → STM32 経路）を読み、候補を
**ホスト上で数値再現して裏を取ったものだけ**を載せた。再現しない／推測のものは落とした。
**コードは一切変更していない（読み取り専用のレビュー）。**

- 深刻度: **C**ritical（安全な走行を直接損なう）/ **H**igh / **M**edium / **L**ow
- 「確認」列: **再現** = ホストで実際に発生させた / **コード** = 呼び出し関係をたどって確定 /
  **突合** = 別のファイル・数値計算と機械的に照合
- **既存の全体レビュー `docs/archive/review_2026-10-21.md`（H6/H7/H8/C1/M21/M24/M28）と重複するものは
  繰り返さず、参照に留めた。**本書は `nav/` 内部の点に絞っている。**

---

## 0. 概要

1. `nav/` の**数値・符号・単位は概ね一貫**している。曲率の正負（左旋回が正）、
   舵の「指令／実舵／実効」の 3 層の区別（`SteerMap`）、車体外形の footprint 表現、
   摩擦円の制約はコードとコメントが食い違っておらず、**再審査で新たな符号誤りは出なかった**。
2. 一方で**「正（source of truth）の伝播網」に 3 つの穴**がある。
   ① `docs/control_constants.md:41` の `speed_loop_tau_s`=0.155[s] は
   校正 5 で `speed_plant_gain`/`speed_kp` を書き直した結果**実測 0.0687[s]** になったのに
   文書が古く、catch ランプと停止余裕が 2.25 倍ずれている。
   ② `cmd_steer_rate_limit_rad_s` は `vehicle.toml` にあるが **`nav/` から読まれていない**
   （MPC は 7.0 をハードコード）。
   ③ 安全機能（障害物停止）の既定が **0** で、既定の `config/auto.json` にもそのままで保存されている。
3. **Critical 1 件**：`Pure Pursuit` の曲率 FF 分岐（既定 `steer_ff=0.8`）が
   **`|η|>90°` の全舵飽和を失う**。ゴールが後ろにあると舵が中立へ戻り、
   実測で**車が 12 秒経っても U ターンできず 5.2 m 後退した**（FF なしなら 2.9 秒・0.44 m）。
4. **速度系の 2 つの High**：① `speed_loop_tau_s` の文書不一致（catch ランプ 2.25 倍・
   停止余裕 0.043 s（2 m/s で 8.6 cm）不足）、② `speedctl.ref_command` が**ランプに上限が無い**
   （実測 36.3 m/s²、`cmd_accel_limit_m_s2`=3.0 の 12 倍）。後者は今は GUI の `min()` 合流が
   守っているが、**安全の網が GUI にある**構造。
5. 既存テストは 60 件すべて緑（`raspi/tests/test_nav.py test_mpc.py test_speedctl.py`）。
   **本書の指摘は「回帰」ではなく「検査の穴」**である。FF 分岐の飽和・`τ` と文書の整合・
   ランプの上限・`now=None` のフォールバックを検査するテストが無い。

---

## 1. 総合表

| ID | 深刻度 | テーマ | 場所 | 確認 |
|---|---|---|---|---|
| C1 | **Critical** | Pure Pursuit の FF 分岐が `\|η\|>90°` の全舵飽和を失う | `raspi/nav/purepursuit.py:251-257` | 再現 |
| H1 | **High** | `speed_loop_tau_s` が文書（0.155[s]）と食い違い 0.0687[s] | `raspi/core/vehicle.py:295-302` | 再現・突合 |
| H2 | **High** | `ref_command` の目標速度ランプに上限が無い（36.3 m/s²） | `raspi/nav/speedctl.py:97-124` | 再現 |
| H3 | **High** | 既定では planner が障害物に減速も制動もしない | `raspi/auto/slam2d_raceline.py:1927-1985` | 再現 |
| H4 | **High** | 障害物検出の「2 周期確認」の +100 ms が停止距離に未計上 | `raspi/nav/obstacles.py:159-169` | コード |
| M1 | **Medium** | MPC の `now` 時刻基準が崩れると `d[0]` の舵レート制約が 5 倍緩む | `raspi/nav/mpc.py:147-149, 225-227` | 再現 |
| M2 | **Medium** | `cmd_steer_rate_limit_rad_s` が `nav/` から読まれない | `raspi/nav/mpc.py:54` | コード |
| M3 | **Medium** | `Vehicle.kappa_max` のオフセット符号が片側だけ保守的 | `raspi/core/vehicle.py:252-262` | 再現 |
| M4 | **Medium** | `max_steer` の既定値が `vehicle.toml` と陳腐化 | `raspi/nav/purepursuit.py:100`, `raspi/nav/mpc.py:53` | コード |
| M5 | **Medium** | 減速側の指令ランプ（3.0）が物理的な制動能力（2.54）を超える | `config/vehicle.toml:78,128` | 再現 |
| L1 | Low | MPC が `nearest_index` を 2 回計算 | `raspi/nav/mpc.py:155,160` | コード |
| L2 | Low | `_hold` が `ready=True` で「安全」の意味と矛盾する | `raspi/auto/park_to_point.py:684-704` | コード |
| L3 | Low | `docs/control_constants.md` の数値が校正 5 の再同定で陳腐化 | `docs/control_constants.md:41` | 突合 |
| L4 | Low | `speedctl.REF_CATCH_S` は τ=0 のときの値だが τ=0 は返らない | `raspi/nav/speedctl.py:94` | コード |

---

## 2. Critical

### 2.1 C1 — Pure Pursuit の曲率 FF 分岐が `|η|>90°` の全舵飽和を失う 【Critical・再現】

**場所**: `raspi/nav/purepursuit.py:251-257`（FF 分岐）／`raspi/nav/purepursuit.py:139-157`
（飽和を持つ `steer_for_target`）／`raspi/nav/purepursuit.py:160-177`（`_ff_correction`）

`follow()` は `ff_gain > 0` のとき Pure Pursuit の標準式を **FF 補正つきで**
自前で計算し、`steer_for_target()` の飽和処理を **通さない**:

```python
# raspi/nav/purepursuit.py:251-257
if cfg.ff_gain > 0.0 and n >= 5:
    kap = 2.0 * math.sin(eta) / max(dist, 1e-3)
    kap += cfg.ff_gain * _ff_correction(path, i, goal, step, cfg.ff_chord)
    delta = math.atan(cfg.wheelbase * kap)      # ← 飽和なし
else:
    delta = steer_for_target(eta, dist, cfg.wheelbase, math.pi / 2.0)
```

`steer_for_target()` は狙点が真横より後ろ（`|η| > 90°`）なら
`math.copysign(max_steer, eta)` で**全舵**を返す（`purepursuit.py:153-156`）。
その docstring は理由を明記している——「式の `sinη` は 90° を超えると減るので、
そのままだと後ろ寄りの狙点ほど舵が戻って旋回をやめてしまう」。
**FF 分岐はこの議論をそのまま無視している。**

FF 補正が打ち消せない理由: `_ff_correction`（`purepursuit.py:160-177`）が返すのは
`k_path − k_ideal` で、`k_path` は**経路自身の曲率**、`k_ideal = 2 sinη / dist` は
「その点から見た曲率」。したがって

```
kappa = 2 sinη / dist + ff_gain · (k_path − 2 sinη / dist)
      = (1 − ff_gain) · 2 sinη / dist + ff_gain · k_path
```

となり、**η → 180° で `sinη → 0` の項が打ち消され、`kappa → ff_gain · k_path` だけになる**。
経路が直線（`k_path = 0`）なら `kappa → 0` → **舵が中立へ戻る**。

#### 再現 1: 開いた直線経路上の舵指令

`Vehicle.load('config/vehicle.toml')`、`PursuitConfig(ff_gain=0.0 / 0.8, ff_chord=1.0)`、
経路上の点（1.0, 0.0）から見た yaw 誤差に対する**指令舵角**:

| 航向誤差 | `ff_gain=0`（`steer_for_target`） | `ff_gain=0.8`（FF 分岐） | 差 |
|---:|---:|---:|---:|
| 30° | −13.49° | −13.49° | 0 |
| 60° | −20.44° | −20.44° | 0 |
| 90° | **−27.02°**（全舵） | **−22.32°** | 4.7° |
| 120° | **−27.02°**（全舵） | −18.77° | 8.3° |
| 150° | **−27.02°**（全舵） | −10.17° | 16.9° |
| 170° | **−27.02°**（全舵） | −3.47° | 23.6° |
| 179° | **−27.02°**（全舵） | **−0.35°** | **26.7°** |

`ff_gain=0.8` では **舵の絶対値が単調に 0 へ落ちていく**。`|η|>90°` で全舵を出さない。

#### 再現 2: 閉ループ（キネマティックな姿勢モデル）

操舵モデル: `actual = cmd × servo_gain(0.98689)`、
`effective = gain·actual + cubic·actual³ + offset(−0.008817)`、`kappa = tan(effective)/L`。
`yaw0 = 180°`（後ろを向く）、`v = 0.5 m/s`、`dt = 0.02 s`、
回復の判定は `|yaw 誤差| < 5°`。

| 条件 | 回復時間 | 後退量 | 12 s 後の x |
|---|---:|---:|---:|
| `ff_gain=0.0`, v=0.3 | 4.76 s | 0.445 m | +1.95 m |
| **`ff_gain=0.8`, v=0.3** | **回復せず** | **3.46 m** | −3.46 m |
| `ff_gain=0.0`, v=0.5 | 2.92 s | 0.443 m | +4.37 m |
| **`ff_gain=0.8`, v=0.5** | **回復せず** | **5.19 m** | −5.19 m |
| `ff_gain=0.0`, v=1.0 | 1.66 s | 0.438 m | +10.41 m |
| **`ff_gain=0.8`, v=1.0** | **回復せず** | **6.84 m** | −6.28 m |

FF ありでは **車が永久に後ろへ押される**（符号が反転して「後退で U ターン」を試みる）。
FF なしでは 2.9 秒で向きを戻す。

#### なぜ本番で起きるか

- `steer_ff` は **hidden** パラメータ（`raspi/auto/slam2d_raceline.py:567`
  `ParamSpec(..., key="steer_ff", hidden=True, ..., default=0.8)`）。
- `raspi/auto/base.py:204-225` `merged()` は **hidden 項目はユーザー値で上書きしない**
  （`:216` `if v is None or (s.hidden and not hidden): continue`）。
  → **保存済みの `config/auto.json` に `steer_ff` をいくら書いても反映されず、実機では常に 0.8。**
- `ctrl_mode` の既定は **0 = Pure Pursuit**（`slam2d_raceline.py:533-536`）。
  → **初期状態（`config/auto.json` の無い環境）では C1 が発火する。**
- `steer_ff` の説明（`slam2d_raceline.py:567-571`）は
  「0.5 ではチケインで内を切って衝突、0.8 で 0 回」と S 字の検証結果だけを記録しており、
  **`|η|>90°`（ゴールが後ろ）の検証記録が無い**。

#### 修正案

FF 分岐でも `steer_for_target` と同じ飽和条件を適用する。最小の差し替えは

```python
if cfg.ff_gain > 0.0 and n >= 5:
    kap = 2.0 * math.sin(eta) / max(dist, 1e-3)
    kap += cfg.ff_gain * _ff_correction(path, i, goal, step, cfg.ff_chord)
    delta = math.atan(cfg.wheelbase * kap)
    # ★ FF 分岐でも |η|>90° は全舵に飽和させる（steer_for_target と同じ扱い）
    if abs(eta) > 0.5 * math.pi:
        delta = math.copysign(cfg.max_steer, eta)
```

（本レビューは読み取り専用。上の差分は提案であり、適用はしていない。）

**補足**: 同じ飽和を MPC 側は既に持っている。`raspi/tests/test_mpc.py:46-57`
`test_target_behind_saturates_instead_of_straightening` が `steer_for_target` を
直接検査しているが、**`follow()` の FF 分岐を検査するテストは無い**
（`ff_gain > 0` を使うのは `raspi/tests/test_laptime.py:203,213` の周回テストのみで、
前方を向いている）。MPC が飽和を本質的に持つのは、`mpc.py:219-230` の箱制約が
`lo_abs/hi_abs`（実効舵角の絶対値の上限）で舵を制限するため。
Pure Pursuit の FF 分岐だけが「数式で舵を決めてそのまま出す」ため穴になる。

---

## 3. High / Medium

### 3.1 H1 — `speed_loop_tau_s` が `docs/control_constants.md` と 2.25 倍食い違う 【High・再現/突合】

**場所**: `raspi/core/vehicle.py:295-302`（定義）／`docs/control_constants.md:41`（文書）

```
Vehicle.speed_loop_tau_s = 1 / (speed_plant_gain × speed_kp)
                         = 1 / (12.5758 × 1.15716) = 0.06871807 [s]
```

`docs/control_constants.md:41` は
「`Vehicle.speed_loop_tau_s` = 1/(車体の利得×kp) = **0.155**（呼び出し側が渡す）」と書く。
0.155 は**旧同定値**（G=21.5・手置きの kp=0.3）で、校正 5（`tools/calib/speed_pi.py`）で
G と kp を極配置し直した結果、**文書だけが取り残されている**。
**コードの 0.0687 が式として正しい**（`τ = 1/(G·kp)`）。

この 1 つの τ が nav の中で **3 つの異なる意味**に使われており、そのどれかが古くなっている:

**(a) catch 項の傾き（本番で有効）** — `raspi/auto/slam2d_raceline.py:1290` と `:1296` が
`catch_s=self.vehicle.speed_loop_tau_s` を `speedctl.ref_command` に渡す。
`raspi/nav/speedctl.py:122-123` で `ramp = max(REF_ACCEL_MIN, REF_ACCEL_GAIN*a, |v_ref−v_now|/catch)`。
実測（`catch_s=τ`、`lead_s=0.0321`、プロファイル 3.0→0.0 m/s）:

| 現在車速 | `v_ref` | ランプ（τ=0.0687） | ランプ（文書値 0.155） |
|---:|---:|---:|---:|
| 0.5 m/s | 2.995 | **36.31 m/s²** | 16.10 m/s² |
| 1.0 m/s | 2.990 | **28.96 m/s²** | 12.84 m/s² |
| 2.0 m/s | 2.981 | **14.27 m/s²** | 6.33 m/s² |

→ 文書が想定した値の **2.25 倍**のランプ許される（H2 と重なって、安全側ではなく危険な側に倒れる）。

**(b) 停止の立ち上がり（停止余裕）** — `raspi/auto/slam2d_raceline.py:277`
`react = SCAN_PERIOD_S + cmd_latency + 0.5·τ` = 0.1 + 0.0121 + 0.0344 = **0.1465 s**
（文書値なら 0.1896 s）。停止余裕を **0.0431 s 少なく計算**している（相当于）
2 m/s で **8.6 cm**、3 m/s で **12.9 cm** の停止余裕不足。
影響先は `:1850`（近接障害物の減速目標）、`:1974`（障害物の停止距離）、
`:2164`（見通しの良い区間の速度上限 → `raspi/nav/raceline.py:579-587` `sight_speed`）。

**(c) 減速の前倒し `lead_s`（既定では不活性）** — `raspi/nav/speedctl.py:47,63`
`lead_s = LEAD_RATIO(0.65) × τ` = **0.0447 s**（文書値なら 0.1008 s）。
ただし `speedctl.py:44-46` のコメントが明記するとおり、
`Vehicle.speed_ff_firmware`（`vehicle.py:411`、`speed_ff_nm_per_m_s2 = 0.079518 > 0` → True、
`config/vehicle.toml:222`）のあいだ `ref_command` 経路しか使われず、
`LEAD_RATIO` は使われない。→ **ファーム FF を切る運用**（`speed_ff_nm_per_m_s2 = 0`）で
2.25 倍前倒しの減速になり、コーナー進入の出過ぎ（`LEAD_RATIO` のコメントにある
「0 倍: 出過ぎ +0.49 m/s」）程度になる。

**修正案**: ① `docs/control_constants.md:41` を 0.0687 に更新し、
`tools/calib` が `speed_kp` を書くたびに文書数値も更新する検査を `config/check_docs.py` に足す。
② `catch_s` を `τ` ではなく独立した設定（例 `[control] speed_catch_s`）にして、
「PI の比例項の速さ」と「目標車速の傾きをどこまで許すか」を分離する。
③ `react_s` の `0.5·τ` は `τ` の定義に依存する以上、文書と必ず同時に直す。

---

### 3.2 H2 — `speedctl.ref_command` の目標速度ランプに上限が無い 【High・再現】

**場所**: `raspi/nav/speedctl.py:97-124`

```python
# raspi/nav/speedctl.py:122-124
catch = catch_s if catch_s > 1e-6 else REF_CATCH_S
ramp = max(REF_ACCEL_MIN, REF_ACCEL_GAIN * a, abs(v_ref - v_now) / catch)
return max(0.0, v_ref), ramp
```

`ramp` は STM32 に送る **「目標車速を動かす傾きの上限」**（`docs/stm32_interface.md:561-562`
「`accel_limit` は『その目標速度へ向かうときの加減速の上限』」）。
**`cmd_accel_limit_m_s2`（3.0）や `a_decel_max`（2.5413）で頭打ちにされていない。**
H1(a) のとおり実測で **36.3 m/s²**。

消費の流れ:

```
speedctl.ref_command            (speedctl.py:122-124)
 → st.accel_limit               (slam2d_raceline.py:1290 / 1296)
 → auto/cmd                     (planning_node.py:407 / 503)
 → cmd                          (telemetry_node.py:3244-3246)
     accel = min(gui.accel_limit, auto.accel_limit)
```

`telemetry_node.py:3244-3246` の `min()` が **現在は GUI の 3.0 で頭打ち**にしてくれる。
GUI は `ACCEL_SAFETY_LIMIT`（`gui/src/store/ui.ts:201` ← `gui/src/generated/vehicle.ts:85`，
`config/vehicle.toml:78` と同じ 3.0）を常送している（`gui/src/input/useDriving.ts:947,1043,1237`）。

**問題は 3 つ**:

1. **安全の網が GUI にある。** GUI が無い／`accel_limit=0` で送ってきた場合、
   `telemetry_node.py:3246` は `accel = auto.accel_limit`（planner の値 36 m/s²）を採用する。
   `docs/archive/review_2026-10-21.md` H3/M32 と同じ「正が GUI にある」構造。
2. **`cmd_accel_limit_m_s2`（3.0）が `ref_command` の出口で効かない。**
   値は `raspi/core/vehicle.py:78-87` の `_ramp_max()` が読み、
   `Vehicle.speed_ramp_max_m_s2`（`vehicle.py:188-190`、実測 3.0）として公開されている。
   `raspi/core/vehicle.py:195-199,283-284` の `plan_accel_limit_m_s2` も
   `Vehicle.plan_accel_max` として公開されている。
   **しかし `raspi/nav/speedctl.py` は `Vehicle` を一切受け取らない**ため、
   `ref_command` の戻り値には**どちらの上限もかかっていない**。
   （`speedctl.py` の `SpeedFFConfig` 側は `cmd_accel_limit_m_s2` を使っていない。）
3. **減速側も同じランプを使う。** `accel_limit` は「加減速」の上限なので、
   減速時に目標が 3.0 m/s² で落ちていく一方、**タイヤが作れる減速は 2.5413 m/s²**
   （`config/vehicle.toml:128` `a_brake_m_s2`、planner 内部では `limits().a_brake`
   = 0.9 × 2.5413 = **2.287 m/s²**、`slam2d_raceline.py:270-283`）。
   → 目標速度が物理より早く下がり、**速度誤差が負に開いて PI が押し返す**。
   1 秒あたり (3.0 − 2.287) = 0.71 m/s の乖離が積み上がる（M5 として独立に記す）。

**修正案**: `ref_command` の戻り値を
`min(ramp, vehicle.speed_ramp_max_m_s2)` で頭打ち（減速方向はさらに `a_decel_max`）。
戻り値に上限の引数を渡す必要があるので、キーワード引数 `ramp_max` を追加する形が最小。

---

### 3.3 H3 — 既定では planner が障害物に減速も制動もしない 【High・再現】

**場所**: `raspi/auto/slam2d_raceline.py:1927-1985`（`_obstacle_response`）

```python
# raspi/auto/slam2d_raceline.py:1936
slow_on = p["obstacle_avoid"] > 0
...
# :1968
stop_at = p["obstacle_stop"]
if slow_on:  ...                       # :1969-1976（減速）
...
# :1983
if stop_at <= 0.0 or gap > stop_at:
    self._brake_hold = None
    if slow_on: ...                    # :1986-1990
    return False                      # ← brake も target_speed も触らない
```

既定は `obstacle_avoid = 0` / `obstacle_stop = 0.0`
（`slam2d_raceline.py:594-605` の `ParamSpec`、および保存済みの `config/auto.json`）。
このとき `slow_on = False`, `stop_at = 0.0` なので **`:1983` で即 `return False`**。

**再現（`_obstacle_response` を直接叩く）**:

```python
st = AutoState(); st.target_speed = 1.5
vs = VehicleState(); vs.speed = 1.5; vs.armed = True
hit = SimpleNamespace(x=1.0, y=0.0, r=0.1)
nav._obstacle_response(st, hit, 1.0, vs, {"obstacle_avoid":0.0,"obstacle_stop":0.0,...}, None, 0)
# → returned False, brake False, target_speed 1.5, reason ''
```

前方 1 m に障害物があっても **制動も減速もしない**。

**これは意図された振舞いである**（docstring `:1934` に明記、`raspi/tests/test_route.py:980-985`
`test_open_path_is_untouched_when_avoidance_is_off` が固定している）。したがって
「バグ」ではなく **既定の安全設定の妥当性** の問題として扱う。

**なぜ High とするか**:

- **既定の安全機能がオフ**で、既定の `config/auto.json` にもそのままで保存されている
  （`"obstacle_stop":0.0,"obstacle_avoid":0`）。
- 守りは **STM32 の `auto_stop`**（`DriveCmd.auto_stop`、`raspi/msgs/types.py:371`、
  GUI の `settings.autoStop` 既定 ON `gui/src/input/useDriving.ts:143-144`）に依存する。
  つまり **「障害物対応」が 2 層に分かれており、Pi 側をオフにすると只剩 1 層**になる。
- しかも **`raspi/auto/park_to_point.py:98` は「`auto_stop` は OFF 運用が前提」と明記**している
  → **モードによって安全網が変わる**。
- さらに `:1844-1857` の「近距離の障害物は 2 周期目を待たない」高速パスも
  **同じフラグで gate されている**（`:1848` `if p["obstacle_avoid"] > 0 or p["obstacle_stop"] > 0:`）
  → 既定では無効（H4 と重なる）。

**修正案（運用ではなくコード）**: 既定値を見直す（`obstacle_stop` の既定を
「必ず止まる距離」として 0.8 m 程度）、あるいは
「`obstacle_avoid=0` かつ `obstacle_stop=0` のときは **減速だけでも**入れる」
（`slow_on` と `stop_at` を分離する）。現状は `slow_on` が `obstacle_avoid` に縛られているため、
「避けるはしないが止まる」は表現できない。

---

### 3.4 H4 — 障害物検出の「2 周期確認」の +100 ms が停止距離に計上されていない 【High・コード】

**場所**: `raspi/nav/obstacles.py:95-97, 159-169`／`raspi/auto/slam2d_raceline.py:267-283`

- `obstacles.detect()` は `if not grid.frozen: return []`（`:95-97`）→ **地図を渲画している間は検出が 0**。
- `obstacles.confirm()`（`:159-169`）は **2 周期連続で同じ場所に出たものだけ**を残す。
  LiDAR は 10 Hz（`SCAN_PERIOD_S = 0.1`、`slam2d_raceline.py:254`）なので **+100 ms**。
- `limits().react_s`（`slam2d_raceline.py:277`）は
  `SCAN_PERIOD_S(0.1) + cmd_latency(0.0121) + 0.5·τ(0.0344) = 0.1465 s`。
  docstring（`:272-276`）は「点群の周期（**最悪そのぶん**遅れて見える）」と書いているが、
  **実際の最悪は 2 周期分（0.2 s）**。

→ **停止余裕が 1 スキャン周期ぶん（0.1 s）足りない**。
1.5 m/s なら 15 cm、3.0 m/s なら 30 cm の差になる。
これは `:1850`（近接障害物）・`:1974`（障害物停止距離）・`:2164`（見通しの良い区間の速度上限）
すべてに効く。

**近接高速パスで相殺されるか**: `:1844-1857` の近接判定は
`near = v²/(2·a_brake) + v·react_s + front_overhang + 0.6`。
1.5 m/s なら `near ≈ 1.40 m`。**1.4 m より手前の障害物**は 2 周期目を待たずに採用される。
一方 **1.4 m より先の障害物**（遠方ではそれで良いので基本は問題ないが、
角落の先などで近い情形はありうる）は +100 ms を待つ。

**修正案**: `react_s` に「確認の 1 周期ぶん」を足す、
すなわち `react = 2×SCAN_PERIOD_S + cmd_latency + 0.5·τ` とする
（あるいは近接判定の閾値を `near` の式に `+SCAN_PERIOD_S` 相当を明示的に含める）。

---

### 3.5 M1 — MPC の `now` 時刻基準が崩れると `d[0]` の舵レート制約が 5 倍緩む 【Medium・再現】

**場所**: `raspi/auto/slam2d_raceline.py:1826`／`raspi/nav/mpc.py:147-149, 225-227`

```python
# raspi/auto/slam2d_raceline.py:1826
now=vs.t_capture / 1e9 if vs.t_capture else None
```

`vs.t_capture == 0` は「センサの時刻が無い」状態（`raspi/msgs/types.py:8-11`
「失敗・未選択の結果は `t_capture=0` で出る」）。このとき `now=None` になり、
`mpc.py:147-149` で **`self._clock`（0.0 起步）が使われる**。
`self._sent` には epoch スケールの時刻が入っているため、**2 つの時刻基準が混ざる**。

```python
# raspi/nav/mpc.py:225-227
prev_t = self._sent[0][0] if self._sent else None
dt_call = cfg.dt if prev_t is None or now - prev_t < 1e-4 else min(cfg.dt, now - prev_t)
reach = cfg.steer_rate * (dt_call + cfg.dt * np.arange(N))
```

epoch → 0 へ落ちると `now - prev_t` が大きく負 → `< 1e-4` → **`dt_call = cfg.dt = 0.1 s`**
（実周期は 0.02 s）。`reach[0] = 7.0 × 0.1 = 0.7 rad`（本来 0.14 rad）、
つまり **最初の 1 歩だけ舵レート制約が 5 倍緩む**。

**再現**（直線経路、横偏差 45 cm、50 Hz 呼び出し、`steer_rate=7.0`）:

| 条件 | 出力舵角 |
|---|---:|
| 正常系（epoch 時刻で 8 回連続） | −8.13° に収束 |
| 8 回呼んだ後、**1 回だけ `now=None`** | **−27.02°** |

**18.9° の跳躍**。制約が効いていないので 1 周期で最大舵まで飛べる。
`self._clock` は `reset()`（`mpc.py:86-89`）まで epoch に戻らないため、
**一度こうなると以後は `_sent` との基準ずれが固定される**。

**修正案**: `now=None` を許さない（呼び出し側で必ず `t_capture` を与える、
無い場合は `self._sent[0][0]` を基準に合わせる）、あるいは
`mpc.py` 側で `now` の基準を `_sent` の最終時刻に相対化してから使う。

---

### 3.6 M2 — `cmd_steer_rate_limit_rad_s` が `nav/` から読まれていない 【Medium・コード】

**場所**: `raspi/nav/mpc.py:54`／`raspi/auto/slam2d_raceline.py:381-386`／
`raspi/core/vehicle.py:206,410`／`config/vehicle.toml:83`

- `MPCConfig.steer_rate` の既定は **7.0 のハードコード**（`mpc.py:54`）。
- `mpc_config()`（`slam2d_raceline.py:381-386`）は `steer_rate` を設定していない → 既定のまま。
- `vehicle.cmd_steer_rate_limit_rad_s`（`vehicle.py:206,410`、`vehicle.toml:83` = 7.0）は
  `raspi/` 内の参照が `vehicle.py` の定義箇所のみ（`sim/bench.py:674,679` が
  `sim/vehicle.py` 経由で参照）。**nav は読んでいない。**

値はたまたま一致（7.0）ので実害はないが、**正（`vehicle.toml`）が nav へ届いていない**。
firmware 側の舵レート制限（`docs/stm32_interface.md`「`steer_rate_limit` は角度制御ループの
追従限界を超える指令によるオーバーシュート・発振を防ぐ」）が toml で変わったときに
nav を追随しない。

**修正案**: `mpc_config()` で `steer_rate=vehicle.cmd_steer_rate_limit_rad_s or MPCConfig.steer_rate`。
`cmd_accel_limit_m_s2` も H2 と同じく nav 側の参照が無い。

---

### 3.7 M3 — `Vehicle.kappa_max` のオフセット符号が片側だけ保守的 【Medium・再現】

**場所**: `raspi/core/vehicle.py:252-262`／`raspi/nav/purepursuit.py:77-93`

`SteerMap` は 3 層を正しく分けており、**逆算も正しい**:

- `effective(steer_actual) = gain·d + cubic·d³ + offset`（`purepursuit.py:77-79`）
- `command(delta, max)`: `t = delta − offset` → `d = t/gain` → `cmd = d/servo_gain`
  （`:81-93`）。つまり**実効舵角→指令舵角の逆算で `offset` を引く**のは正しい。

一方 `Vehicle.kappa_max` は

```python
# raspi/core/vehicle.py:260-262
d = self.steer_servo_gain * self.max_steer
delta = self.steer_gain * d + self.steer_gain_cubic * d ** 3 - abs(self.steer_offset_rad)
return math.tan(max(delta, 1e-3)) / self.wheelbase
```

と **`- abs(offset)`**（左右とも不利な側に取る）。実測（`cmd = ±max_steer = ±0.47167`）:

| 方向 | actual | effective | 達成できる κ |
|---|---:|---:|---:|
| 左（+） | +0.46550 | +0.45668 rad | **+2.1361** |
| 右（−） | −0.46550 | −0.47432 rad | **−2.2321** |

`kappa_max = 2.13612` は**左で正確、右で 4.3% 保守**
（最小旋回半径 0.4681 m に対し、右だけ 0.4480 m まで回的せる）。
`kappa_limit()`（`slam2d_raceline.py:249-251`）は `kappa_max × kappa_use(0.9)` なので、
**右コーナーだけ 4.3% 広く曲がれる余地を捨てている**。

**害はない**（安全側）が、「同じ `SteerMap` を使っていて片側だけ違う」不整合。
**修正案**: `kappa_max` を `SteerMap.command(±max_steer, max_steer)` → `effective` の
**最小絶対値**で求める（左右非対称は仕様として `kappa_max_l`/`kappa_max_r` にするのも手）。

---

### 3.8 M4 — `max_steer` の既定値が `vehicle.toml` と陳腐化 【Medium・コード】

| 場所 | 既定値 | `config/vehicle.toml:25` |
|---|---:|---:|
| `raspi/nav/purepursuit.py:100` `PursuitConfig.max_steer` | 0.46175 | **0.47167** |
| `raspi/nav/mpc.py:53` `MPCConfig.max_steer` | 0.46175 | **0.47167** |
| `raspi/core/vehicle.py:121` `Vehicle.max_steer` | 0.524 | **0.47167** |

現状は全呼び出し側が `vehicle.max_steer` を渡すので不活性
（`slam2d_raceline.py:364`（PP）・`:382`（MPC））。
ただし `Vehicle.max_steer` の docstring（`vehicle.py:118-120`）は
「現在 **0.4636** = 26.6°」と書かれており、**実際の 0.47167 とも違う**。
`purepursuit.py:98-100` は「既定は `vehicle.toml` の現在値（…約 26.5°）に合わせた値」と書きながら
数値が 0.46175（26.42°）で、**どの値も最新ではない**。

**修正案**: `max_steer` の既定を `None` にして「呼び出し側が必ず渡す」を型で強制する
（docstring が既に「**呼び出し側が `Vehicle.max_steer` を渡すこと**」と要求している）。
`Vehicle.max_steer` の docstring の数値も更新。

---

### 3.9 M5 — 減速側の指令ランプ（3.0）が物理的な制動能力（2.29〜2.54）を超える 【Medium・再現】

**場所**: `config/vehicle.toml:78`（`cmd_accel_limit_m_s2 = 3.0`）／`:128`（`a_brake_m_s2 = 2.5413`）

`accel_limit` は「加減速の上限」（`docs/stm32_interface.md:561-562`）なので、
**減速側にも 3.0 m/s² がかかる**。一方、

| 数字 | 値 | 場所 |
|---|---:|---|
| 指令のランプ | 3.0 m/s² | `vehicle.toml:78`（GUI も同じ値 `gui/src/store/ui.ts:201`） |
| タイヤの制動（同定値） | 2.5413 m/s² | `vehicle.toml:128` |
| planner が仮定する制動 | 0.9 × 2.5413 = **2.287 m/s²** | `slam2d_raceline.py:270-283` |

→ 目標速度は 3.0 m/s² で落ちていくのに、**車は 2.29 m/s² でしか減速できない**。
差 0.71 m/s² が 1 秒ごとに速度誤差として蓄積し、**PI に「減速しすぎた」と逆方向の指令を出す**。
停止点に近づくほど偏差が拡大し、制御が連鎖的に振れる可能性がある。

H2 の修正（ランプを `a_decel_max` で頭打ち）と同じ処置で解ける。
数値の出所も 1 箇所にまとめるべき（GUI が 3.0 をハードコードしている
`gui/src/generated/vehicle.ts:85` は生成物だが、`config/vehicle.toml` を読み直していない）。

---

## 4. Low

### 4.1 L1 — MPC が `nearest_index` を 2 回計算している 【Low・コード】

`raspi/nav/mpc.py:155` で `base = follow(...)` を呼び（その中で
`purepursuit.py:198` の `follow()` が `nearest_index` を実行）、
`:160` でさらに自前の `nearest_index(path, px, py, hint, window=n//4)` を呼んでいる。
2 回目のものは `window = n//4`（広い窓）なので 1 回目（`window=0`＝全体検索）と
**理論上は違う添字を返しうる**。
円 R=0.47〜2.0 m / ヘアピン / 点間隔 0.10 m の実測では添字差分のヒストグラムは常に `{0: N}`
（**一度も乖離しなかった**）ので実害は無いが、
**「一致する前提」で 2 回計算している**コストである。
`base` を使うのは舵・速度・target だけなので、`nearest_index` の結果を 1 回にまとめられる。

### 4.2 L2 — `park_to_point._hold` が `ready=True` で「安全」の意味と矛盾する 【Low・コード】

`raspi/auto/park_to_point.py:684-704` は `_hold` で `ready=True` かつ `brake=True` を返す。
`raspi/nodes/planning_node.py:458-509` の `_cmd_from` は brake 分岐で**指令を生きたまま**にする
（「`ready` は安全の信号ではない」）。GUI が `ready` 相当的表示をするなら、
これは「待機中＝安全」を意味しない。表示側の扱いを明示する。

### 4.3 L3 — `docs/control_constants.md` の数値が校正 5 の再同定で陳腐化 【Low・突合】

`docs/control_constants.md:41`（H1 以外の行もある）。同じ表には次の記載もある:
「`[dynamics] speed_ff_filter_s`… Pi 側は 1 箇所」等の行があるが、
`docs/control_constants.md` の §1.2 は「★プロトコル v0.27。**実機未検証**、既定値は前と同じ動き」
と書きながら、`config/auto.json` の `ctrl_mode` は **1（= MPC）** が保存されている
（`slam2d_raceline.py:533-536` の note も「MPCは実機未検証」と書いている）。
**文書が「未検証」と宣言している経路が、保存済み設定では既定で選ばれている。**

### 4.4 L4 — `speedctl.REF_CATCH_S` の「τ=0 のときの値」という位置づけが実際のコードと合わない 【Low・コード】

`raspi/nav/speedctl.py:91-94` は `REF_CATCH_S = 0.15` の説明に
「**未同定（τ=0）のときの値**」と書くが、`Vehicle.speed_loop_tau_s`
（`vehicle.py:295-302`）は未同定なら **0.0 を返す**ので `ref_command` に `catch_s=0` が渡り、
`speedctl.py:122` の `catch = catch_s if catch_s > 1e-6 else REF_CATCH_S` で 0.15 に落ちる。
**挙動は意図どおり**だが、「τ=0 のときの値」という説明は
「τ が返らないとき（未同定）」という二重表現になっている。読み手の混乱を招くだけなので Low。

---

## 5. 設計ボトルネック・リスク

### 5.1 「正」が 1 箇所に無い（3 層の伝播網）

本リポジトリ全体のテーマ（`docs/archive/review_2026-10-21.md` §0）だが、nav 内部で
**3 つの独立した破断面**が確認できた。

| 破断面 | 症状 | 本書 |
|---|---|---|
| `vehicle.toml` → `docs/control_constants.md` | τ が 2.25 倍ずれ | H1 / L3 |
| `vehicle.toml` → `nav/`（`cmd_steer_rate_limit_rad_s`） | 参照が 0 件、既定 7.0 が使われる | M2 |
| `vehicle.toml` → GUI → STM32（`cmd_accel_limit_m_s2`） | `ref_command` の出口で効かない。安全の網が GUI の `min()` | H2 |

**根本原因**: `nav/` の各モジュールは **必要な値だけを引数で受け取る**設計のため、
「どこから来た値か」が引数の型（`float`）に暗黙に失われている。
`Vehicle` を丸ごと渡す形ではなく、
`SpeedFFConfig`/`PursuitConfig`/`MPCConfig` のような **派生 NamedTuple** に切り分けているのは
この方針としては筋が良いが、**「この値は vehicle.toml のどこから来たか」の追跡の手が切れる**。

**提案**: 派生 NamedTuple の各フィールドに **出所のコメント**（`# ← vehicle.toml [safety].cmd_accel_limit_m_s2`）
を付けるだけで、`grep` による追跡が可能になる。名前で担保するより確実。

### 5.2 hidden パラメータは「常に既定」で、hidden でないパラメータは「保存値が勝つ」

- `steer_ff`（hidden, 既定 0.8）: `base.py:216` により**ユーザー値を無視** → 常に 0.8 → **C1 が本番で発火**。
- `ctrl_mode`（visible, 既定 0）: `config/auto.json` に **1（MPC、実機未検証）** が保存されている。

この 2 つの組み合わせは
「**最も攻撃的な経路の FF が off にできず**、一方 **未検証の経路が保存値で選ばれる**」という
**安全側の既定が逆転した状態**になっている。
hidden は「隠す」だけでなく「**値を変えられない**」実効にするなら、
GUI に「現在 0.8 で固定」を明示して表示するのが親切。

### 5.3 安全機能が既定でオフ、かつモードによって安全網が変わる

- `obstacle_stop = 0` / `obstacle_avoid = 0`（既定）→ Pi 側は障害物を見ない（H3）。
- 守りは STM32 `auto_stop`（GUI 既定 ON）。**`park_to_point` は `auto_stop` OFF 運用が前提**
  （`raspi/auto/park_to_point.py:98`）。
- つまり **経路モードと駐車モードで安全網の層数が違う**。
  均一に「どちらのモードでも Pi 側が 1 層は持つ」設計に寄せるのが安全側。

### 5.4 周期の階層が 3 つあり、余裕が薄い

| 周期 | 値 | 場所 |
|---|---|---|
| 指令の送信 | 50 Hz | `raspi/auto/base.py:40` `CMD_HZ = 50` |
| 追従（`_tick`） | 50 Hz | `raspi/nodes/planning_node.py:383-407` |
| 経路の再計画（`_replan`） | 10 Hz | `planning_node.py:331-382` |
| 障害物の確認 | 2 × 10 Hz = 100 ms | `raspi/nav/obstacles.py:159-169` |
| LiDAR | 10 Hz | `slam2d_raceline.py:254` |

予算の階層:

```
plan_budget_ms（既定 120 ms、最大 150 ms）  ≤  cmd_deadman_ms 150 ms  <  auto_cmd_stale_ms 200 ms
```

`raspi/auto/park_to_point.py:260` の `plan_budget_ms` は max 150 ms、
`raspi/nodes/telemetry_node.py:360` `AUTO_CMD_STALE_NS = auto_cmd_stale_ms` = 200 ms。
**1 回の再計画が 150 ms かかると `auto/cmd` が 1 周期途切れ**（`docs/archive/review_2026-10-21.md` H8 参照）。
余裕は 50 ms しかない。

実測（`raspi/tests/test_hybrid_astar.py` のシーン、`time_budget_s=0.15`）:

| シーン | 予算 | 実測 | 超過 |
|---|---:|---:|---:|
| 行き止まり（解なし） | 150 ms | 150.5 ms | +0.5 ms |
| 車庫（解なし） | 150 ms | 151.4 ms | +1.4 ms |
| 部屋（解あり） | 150 ms | 16.9 ms | — |

→ **Hybrid A* の予算は十分守られている**（`hybrid_astar.py:366` の 4 展開ごとの判定で十分）。
H8 の残余リスクは `plan()` **全体**（線引き・道グラフ生成を含む）が 200 ms を超える可能性にあり、
nav 内部の問題ではない。**本書は H8 を再現できなかった**（新規の指摘にはしない）。

### 5.5 速度ループの 1 つの物理量（τ）が 3 つの役割を持つ

`Vehicle.speed_loop_tau_s`（`vehicle.py:295-302`）は
①減速の前倒し（`speedctl.py:63`）②catch 項の傾き（`speedctl.py:122` 経由）
③停止の立ち上がり（`slam2d_raceline.py:277`）の 3 役。
**校正 5 で `speed_kp` を 1 つ書き換えるだけで 3 箇所が同時に動く**（H1）。
3 つの役割は物理的に別物なので、**1 つの名前で 1 つの意味**に、あるいは
**3 つの名前で 3 つの出所**に分けるべき。

### 5.6 速度 FF の新旧 2 経路が併存し、切り替えは 1 ビット

`Vehicle.speed_ff_firmware`（`vehicle.py:411`、`speed_ff_nm_per_m_s2 > 0` で True）で
`ref_command`（新）と `SpeedFF`（旧）を切り替える（`slam2d_raceline.py:1290-1300`）。
このビットは **GUI に無い**。`vehicle.toml:218-222` を 1 つの値を変えただけで
速度の挙動が完全に別物になる。旧経路のテスト（`raspi/tests/test_speedctl.py`）は
旧経路の写しを検査している（`test_speedctl.py:27`）が、
**新経路（`ref_command`）の「ランプに上限が無い」ことを検査するテストが無い**（H2）。

---

## 6. テストの穴

既存テストは緑（回帰ではない）:

```
$ .venv/bin/python -m pytest raspi/tests/test_nav.py raspi/tests/test_mpc.py raspi/tests/test_speedctl.py -q
60 passed
```

以下は**本書の発見を捕まえられない**テストの穴。

| # | 穴 | 捕まえない理由 | 対応 |
|---|---|---|---|
| T1 | **Pure Pursuit の FF 分岐（`\|η\|>90°`）の飽和** | `steer_for_target` の飽和は `test_mpc.py:46-57` で検査されるが、`follow()` の **FF 分岐**を検査するテストが無い。`ff_gain>0` を使うのは `test_laptime.py:203,213` の周回（前方を向く）だけ | C1 に直接対応 |
| T2 | **`Vehicle.speed_loop_tau_s` と `docs/control_constants.md` の整合** | `config/check_docs.py` は値を検査しない（`docs/archive/review_2026-10-21.md` L37 参照）。校正ツールが `speed_kp` を書いても文書は直らない | H1 |
| T3 | **`ref_command` のランプが `cmd_accel_limit_m_s2` / `a_decel_max` を守ること** | `test_speedctl.py:248-273` は「プロファイルとその傾きを送る」ことと「乗っている間は最小ランプ」を検査するが、**ランプに上限が無いこと**は検査しない | H2 / M5 |
| T4 | **MPC の出力が `max_steer` と `cmd_steer_rate_limit_rad_s` を守ること** | `test_mpc.py:98` は MPCConfig 内の `steer_rate` で検査しているだけで、**`vehicle.cmd_steer_rate_limit_rad_s` との紐付け**が無い | M2 |
| T5 | **MPC の `now=None` フォールバック（`t_capture==0`）** | `test_mpc.py:127-164` は時刻正常系（`now=0.0, 0.02, …`）のみ。`t_capture==0` → `self._clock` 混入の経路が無い | M1 |
| T6 | **障害物検出の遅延（confirm 2 周期 = 100 ms）が停止距離に計上されているか** | `test_route.py:940-1005` は反応の挙動を検査するが、`limits().react_s` が**検出遅延を含む**ことを検査するテストが無い | H4 |
| T7 | **既定（`obstacle_stop=0`, `obstacle_avoid=0`）で planner が何もしないことの明示** | `test_route.py:980-985` は「避けるを off にすると触らない」ことを固定しているが、**「既定値そのものが安全か」**は誰も検査しない。意図的であるなら「意図である」ことを宣言するテストに留保する | H3 |
| T8 | **`catch_up` が入力 `path.v` を破壊しないこと** | 現状は `speedctl.py:151` で `np.array(..., copy=True)` して**正しい**（aliasing バグは無い）。**この正しさが守られていることを検査するテストが無い**（将来 `inplace` へ変えると静かに壊れる） | 予防 |
| T9 | **`Vehicle.kappa_max` の左右非対称** | `kappa_max` は「最大曲率」として 1 つの値しか返さないので、非対称が設計か欠陥かを区別できない | M3 |
| T10 | **`speedctl.REF_CATCH_S` と `τ=0` のフォールバック** | `test_speedctl.py:78-84` は「未同定なら既定値へ」を検査するが、**`catch_s=0` → `REF_CATCH_S` へ落ちる経路**（`speedctl.py:122`）は明示されていない | L4 |

**交差参照（重複ではなく参照）**: `docs/archive/review_2026-10-21.md` の
**H6**（Hybrid A* の margin 緩和が経路全体に効く）、**H7**（`nearest_index` に全体検索のフォールバックが無い）、
**H8**（探索時間予算が `auto/cmd` の陳腐化閾値を超える）、**C1**（非常制動が GUI のブレーキスライダ値に格下げされる）、
**M21**（`auto/cmd` → `cmd` の合流で `accel_limit` が 1 周期遅れる）、
**M24**（`sim/` と `raspi/` が `vehicle.toml` を別々のパーサで読む）、
**M28**（`raspi/nav/deskew.py` と `slam2d/core/deskew.py` の二重化）
は本書の対象外（nav 内部の点ではない）。**本書で再現できなかったもの**：
Hybrid A* の予算超過（H8 の残余）、MPC と Pure Pursuit の添字不一致（実測で常に一致）、
閉ループ横偏差（正しい操舵モデルでは FF 有無で差が出ない）。
**いずれも本書の主張には含めていない。**

---

## 7. 優先順位の提案

| 順 | 対応 | 影響 |
|---:|---|---|
| 1 | **C1**: FF 分岐に `\|η\|>90°` の飽和を入れる | U ターン不能・後退走行（実測 5.2 m） |
| 2 | **H3**: 既定の障害物安全設定を有効にする（減速だけでも） | Pi 側の安全網が 0 |
| 3 | **H2 + M5**: `ref_command` のランプを `cmd_accel_limit_m_s2` / `a_decel_max` で頭打ち | 目標速度の急降下・速度誤差の蓄積 |
| 4 | **H1 + L3**: τ の文書を直し、`catch_s` を独立設定にする | catch ランプ 2.25 倍・停止余裕 8〜13 cm 不足 |
| 5 | **H4**: `react_s` に confirm の 1 周期を足す | 停止余裕 15〜30 cm 不足 |
| 6 | **M1**: MPC の `now` の時刻基準を統一 | 1 周期で 18.9° の舵跳躍 |
| 7 | **M2 / M3 / M4 / L1〜L4** | 保守性・追跡可能性・性能 |

---

*本レビューは読み取り専用で、コードの変更はしていない。*
*数値はすべて `.venv/bin/python`（numpy 2.5.1 / Python 3.12）で `config/vehicle.toml` を
`Vehicle.load()` して取得した。*
