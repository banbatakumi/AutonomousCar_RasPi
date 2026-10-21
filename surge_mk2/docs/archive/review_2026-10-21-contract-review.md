# SURGE Mark.2 コードレビュー — モジュール境界の契約と設定の単一情報源

対象: `surge_mk2/`（`/Users/banbatakumi/GitHub/AutonomousCar_RasPi/surge_mk2`）
基準コミット: `aee4e1d`「保存済み地図の学習値だけを GUI から消せるようにする」（14 ファイル, +249/-2）
作業ツリー: `git status` はクリーン。未追跡の `docs/archive/review_2026-10-21.md`（86KB）が 1 つあるが、本レビューの营利とは独立している（ 後述「補足」）。

観点: **アルゴリズムの正しさではなく「A の値を B が X として読んでいる」契約の破れ**、および**設定値が複数の定義元を持ちどれが正か決まっていない状態**に絞った。

---

## 1. 概要

- `config/vehicle.toml` は单车一の仕様表のはずだが、**同じ物理量（μ・横加速度・舵角上限・最高速）が 3〜5 箇所に別の値として存在する**。特に μ は `[limits]`（4.0912 m/s²）・`[sim]`（0.41718）・`[control.plant]`（1.3227）で意味が違い、`tools/fwsim/plant.py:120` は**存在しない `[dynamics].mu` を読んでいる**ため常に既定値 4.48 のままになる。
- 2026-10-14 のレビュー §2.7「最高速の上限が 3 か所」は**未修正**（GUI 3.0 / `install_services.sh` 3.0 / io_node 既定 1.0 / firmware 5.0 の 4 か所）。§2.8「`[control]` の反映の抜け」も未修正。
- 新機能の「学習値を消す」は、**bus に新フィールド（`grip_clear_map`/`grip_clear_seq`）を足したが、GUI↔Pi の `/ws/control` には版・単位の契約が無く**、**planner が未ロードだと指示が黙って消える**。
- 契約の破れではないが、AI 支援の現物として **CI が赤**（`tools/sysid` は削除済みだが CI が参照している）。加えて firmware 定数との突き合わせテストが CI に無い。
- 実装潜意识的に「구의唯一性」を守ろうとした仕組みは 3 つある（`config/generate.py` / `raspi/proto/generate.py` / `config/gen_msgs.py`）が、**都比较の入口が無く、生成物も人手による写しと混在**している。ここが設計ボトルネックの核心。

---

## 2. Critical（契約の破れ）

### C-1. μ が 3 系統あり、それぞれ別の「車」を指している

**該当**
- `config/vehicle.toml:126` — `[limits] a_lat_m_s2 = 4.0912`
- `config/vehicle.toml:138` — `[sim] mu = 0.41718121313132595`
- `config/vehicle.toml:265` — `[control.plant] mu = 1.3227`
- `raspi/core/vehicle.py:396` → `:267-269` — `[limits] a_lat_m_s2` を `Vehicle.a_lat_max` として読む（実機系は 4.0912）
- `sim/vehicle.py:211-214` — `[limits]` があれば `sim_only["mu"] = a_lat_m_s2 / g` で **`[sim].mu` を上書き**
- `tools/fwsim/plant.py:120` — `if dyn.get("mu"): over["lateral_accel_max_m_s2"] = ...`
- `tools/fwsim/plant.py:122-125` — `[control.plant]` の値をそのまま `Plant` にコピー
- `tools/fwsim/plant.py:65` — `lateral_accel_max_m_s2: float = 4.48`
- firmware `host/host_sim.c:224`（`p->mu` はタイヤ摩擦）／`:464`（`p->lateral_accel_max_m_s2` はヨー角速度の上限）

**内容**
`[dynamics]` に `mu` キーは存在しない（実測: `dynamics` のキーは `control_latency_s, dead_time_s, drive_ratio, rolling_resistance, speed_ff_filter_s, speed_filter_order, speed_filter_s, speed_kf_lag_s, speed_ki, speed_kp, speed_plant_gain, speed_torque_max_nm, steer_gain, steer_gain_cubic, steer_offset_rad, steer_servo_gain, tau_steer_s, yaw_damping` のみ）。したがって `tools/fwsim/plant.py:120` の分岐は**常に偽**で、`lateral_accel_max_m_s2` は既定 4.48 のままになる。

fwsim 同一个 `Plant` の中で:
- `Plant.mu` = **1.3227**（`[control.plant]` からコピー、タイヤ摩擦係数。`host_sim.c:224` の `d = p->mu * mu_scale * fz` に使われる）
- `Plant.lateral_accel_max_m_s2` = **4.48**（`[dynamics].mu` を読めなかった既定値。`host_sim.c:464` のヨー角速度上限）
- 実機の横加速度上限 = **4.0912**（`[limits]`、Pi 側 `Vehicle.a_lat_max`）

つまり **fwsim は「タイヤ μ 1.32 の世界」を使いながら、横加速度は 4.48 で頭打ち**にする。実機の 4.0912 とは 9.5% 乖離し、タイヤの μ も 1.32 vs 実機の 0.417 と別物。TC/ABS/TV のゲインを fwsim で詰める基準線と、実車で観測される挙動は別の車どうしの比較になっている。

さらに `[sim].mu` は `sim/vehicle.py:213-214` で**常に `[limits]` に上書きされる**ため、`[sim].mu` を編集しても何も起きない（`config/vehicle.toml:136-137` のコメントは「`[limits]` が無いときの予備」と書いているが、キー名 `mu` が同じなので読者は「実車側の μ を編集している」と誤解する）。

**影響**
- fwsim で得た TC/ABS ゲインを実機に持っていくと、横加速度の前提が 9.5% 違う。
- `[sim]` ブロックの μ・加速・減速は「編集しても反映されない」名前innieのフィールド。

**推奨**
`[control.plant].mu`（タイヤ係数）と `[limits].a_lat_m_s2`（車全体の横加速度上限）は**別の物理量**なので同じ名前をやめる。例: `[control.plant].tyre_mu_f` / `[limits].a_lat_m_s2`。さらに `tools/fwsim/plant.py:120` の読みkeysを実在するキー（`[limits].a_lat_m_s2`）に直し、`Plant.mu` と `Plant.lateral_accel_max_m_s2` の**矛盾を起動時に検出する**（`tools/fwsim/plant.py:104` 付近でアサート）。`[sim]` の 4 キーは `[limits]` から**生成**するか、`[sim]` ではなく `[limits]` ONLY にして `[sim]` を削除する。

---

### C-2. `rear_lateral_share` というキーは toml に存在しない（`Vehicle` は常に既定値を使う）

**該当**
- `raspi/core/vehicle.py:399` — `rear_load_frac=float(d.get("control", {}).get("plant", {}).get("rear_lateral_share", 0.59))`
- `raspi/core/vehicle.py:400` — `load_transfer_hl=...get("load_transfer", 0.133)`
- `config/vehicle.toml:264-275` — `[control.plant]` の実キー（`load_transfer, md_delay_s, md_tau_s, mu, tyre_b, tyre_c, wheel_damping_nms, wheel_friction_nm, wheel_inertia_kgm2, yaw_damping, yaw_inertia_kgm2`）に **`rear_lateral_share` は無い**
- `sim/vehicle.py:194-196` — `rear_load_frac = 0.59` / `load_transfer_hl = 0.133`（`[control.plant]` を**一切読まない**）
- `raspi/auto/slam2d_raceline.py:2480-2484` — `_axle()` が `v.rear_load_frac` / `v.load_transfer_hl` を使う（`grip_shape = 1` の車軸ごとの摩擦）
- `docs/system_overview.md:687` — 「車軸の値は `[control.plant]` の `rear_lateral_share`・`load_transfer`」
- `tools/fwsim/plant.py:51,53` — `load_transfer = 0.17` / `rear_lateral_share = 0.59`（firmware 既定）

**内容**
`raspi/core/vehicle.py:399` は **存在しないキーを `.get(..., 0.59)` で読むため、常に 0.59**。値の実体は `tools/fwsim/plant.py:53` の firmware 既定 0.59 に依存している偶然の合意で、toml には記録されない。

さらに `load_transfer_hl` は Pi 側 0.13324492181757844（`[control.plant].load_transfer` 由来）、シム側 0.133（`sim/vehicle.py:196` のハードコード既定）、fwsim 側 0.13324492181757844（`tools/fwsim/plant.py:122-125` で上書き）と**3 つの値**が流通している（0.133 と 0.13324 の差は小さいが、「どこから来た値か」が 3 種類）。

`rear_lateral_share` については **`docs/system_overview.md:687` が「`[control.plant]` から取れる」と書いている**ため、読む人は必ずキー名を鵜呑みする。キー名を鵜呑みにして書くと `Vehicle.load()` は例外を握りつぶして既定値 0.59 で動く（`raspi/core/vehicle.py:339-344`）。

**影響**
`grip_shape = 1`（車軸ごとの摩擦）の実験（`PROGRESS.md:50-56`）は、`rear_load_frac` が 0.59 固定、`load_transfer_hl` は 0.1332 固定で走っている。値を toml で変えたい المستخدمはキーを足しても反映されない。

**推奨**
`[control.plant]` に `rear_lateral_share = 0.59` を**実際に追加**し、`raspi/core/vehicle.py:399` の既定値を 0.59 ではなく「キーが無いことを検出できる値（例: `None`）」にして、`Vehicle.load()` の黙示フォールバック（`raspi/core/vehicle.py:339-344`）に落とさない。`sim/vehicle.py` 側も同じ `[control.plant]` を読む実装に統一する。

---

### C-3. 最高速の上限が 4 か所にある（2026-10-14 §2.7 が未修正）

**該当**
- `gui/src/store/ui.ts:195` — `export const PI_MAX_SPEED_CAP = 3.0 // m/s（install_services.sh の --max-speed と一致させてある）`
- `raspi/setup/install_services.sh:94` — `ARM="--allow-arm --max-speed 3.0 --max-ste 0.524"`
- `raspi/nodes/io_node.py:160` — `DEFAULT_MAX_SPEED = 1.0  # m/s`
- `raspi/core/control_params.py:78` — `STM32_MAX_SPEED_M_S = 5.0`（firmware `DRIVE_MAX_SPEED_M_S` の写し）
- `gui/src/store/ui.ts:417-425` — `LIMITS` 受信前の仮値として `PI_MAX_SPEED_CAP` を使う

**内容**
`docs/archive/review_2026-10-14.md` §2.7 は「最高速の上限が 3 か所にある（GUI の静的上限 3.0・io_node の `--max-speed`・LIMITS の 5.0）。3.0 より上に設定してもリロードで 3.0 に戻りうる（`ui.ts:202, 410-422, 860`）」と指摘したが、**現時点でも 4 か所**（`install_services.sh` の 3.0、io_node の既定 1.0、GUI の 3.0、firmware の 5.0）。

`gui/src/store/ui.ts:195` のコメントは「install_services.sh の `--max-speed` と一致させてある」と**逆向きの依存**を宣言している（GUI は Pi 側の設定値を知らなくてもよい）。`gui/src/store/ui.ts:417-425` を見ると、`LIMITS` が届く前は `PI_MAX_SPEED_CAP`（3.0）でクランプし、届いた時点で `ws/telemetry.ts` が `setSettings({})` して解放する設計なので、**一時的に 3.0 未満で|France تقديم不可能な状態を作り、3.0 超の保存値はリロードで巻き戻る**。

**影響**
最高速設定は運用上重要な安全パラメータで、「3.0 」「5.0」「1.0」のどれが正かが決まっていない。2026-10-14 レビュー §2.7 と同じ指摘が 1 週後にまだ残存している。

**推奨**
最高速の上限は **firmware の `DRIVE_MAX_SPEED_M_S` ただ 1 つ**（`raspi/core/control_params.py:78`）を正とし、GUI は `LIMITS` 受信までのフォールバックを明示的に「未受信・UI 側で制限しない（または安全側の既定 1.0）」とする。`install_services.sh:94` の `--max-speed 3.0` と `io_node.py:160` の 1.0 は**同一の値に揃える**か、`--max-speed` を**運用側の任意値**として firmware 上限より大きくしないことを機械的に強制する。

---

### C-4. 新機能「学習値を消す」の契約に版・状態がなく、planner 未ロード時に黙って消える

**該当**
- `raspi/msgs/types.py:591-592` — `grip_clear_map: str = ""` / `grip_clear_seq: int = 0`（`AutoCtrl` に新フィールド）
- `raspi/nodes/telemetry_node.py:2702-2720` — `_maps_clear_grip()`（ファイル削除 → `seq` を +1 → `AutoCtrl` を publish）
- `raspi/nodes/planning_node.py:226` — `grip_clear = c.grip_clear_seq > self.ctrl.grip_clear_seq`
- `raspi/nodes/planning_node.py:246` — `if grip_clear and self.planner is not None:`
- `raspi/nodes/planning_node.py:250` — `self._ask("学習値の削除", self.planner.request_clear_grip, c.grip_clear_map)`
- `raspi/auto/mapstore.py:466-483` — `clear_grip()`
- `raspi/auto/mapstore.py:182` — `list_maps()` が `has_grip` を返す
- `gui/src/types.ts:405-419` — `MapFile.has_grip`（**手書き**。`config/gen_msgs.py` の生成対象外）
- `gui/src/ws/control.ts:131, 382` — `maps_clear_grip` / `maps_clear_grip_result`
- `raspi/msgs/schema.py:65` — `schema_version()`（**`/ws/telemetry` のみ**）

**内容**
1. `grip_clear` の処理は `planning_node.py:246` の `self.planner is not None` ガードを通る。**planner がロードされていない（走行前・地図未読込・直前の planner 再生成中）とき、指示は `_auto_grip_clear_seq` だけ incremen されて消費され、planner 側のメモリ学習値は初期化されない**。ファイルは削除済みなので、次に planner がその地図を読むと「学習値が無い」状態で始まるが、**既にメモリに学習値を持っている planner**（走行中だが `self.planner` が `None` でないはずなので通常は大丈夫）以外の経路で『消したのに古い値が戻る』余地が残る。
2. `seq` は** repressive counter ではなく「回数」**で、planning_node は `>` で比較する（`planning_node.py:226`）。`planning_node` 側の `self.ctrl` は最後の受信値なので没有问题だが、telemetry_node が**再起動**すると `_auto_grip_clear_seq` が 0 に戻る（`raspi/nodes/telemetry_node.py:663-664`）。planning_node は `c.grip_clear_seq > self.ctrl.grip_clear_seq` の比較で、**telemetry 再起動後は 0 < 大きい値**となるため、**再起動後のすべての「seq=0」が「初期化指示」として誤読される**可能性がある（`clear_seq`・`freeze_seq` と同じ構造）。
3. `/ws/control`（JSON）には**版・単位のタグが無い**（`gui/src/ws/control.ts:32-50` の `ServerMsg` に版が無い）。`/ws/telemetry`（msgpack）だけが `MSGS_SCHEMA` を持つ（`raspi/msgs/schema.py:65`）。`has_grip` と `maps_clear_grip` は**この版ayahないチャネル** quench，新增字段Meaning が壊れても**検知できない**。
4. `MapFile.has_grip` は `gui/src/types.ts` に手書きされた型で、`raspi/auto/mapstore.py:182` の実装と**型で結び付いていない**（`config/gen_msgs.py:32-36` が「バスに流れるメッセージの写し」だけ生成すると明記しているため、写しのibbには `MapFile` は含まれない）。`list_maps()` の返り値にキーが足されても TypeScript は通る。

**影響**
「保存済み地図の学習値を消す」は GUI の操作で「消えた」と表示されるが、
- planner 側のメモリ学習値が初期化されない経路がある
- GUI↔Pi の型が壊れてもコンパイルエラーにならない
- どちらの「順番」で消したか（seq の巻き戻り）が判別できない

**推奨**
- `planning_node.py:246` のガードを外し、planner 未ロード時は**保留**（`self._pending_grip_clear` に積んで、planner 生成時に適用）する。
- `seq` を `planning_node` 側でも「最後に処理した seq」を保存する方式から、**セッション ID（起動ごとに変わる UUID）**を併用する（既存の `ui/event` の `seq` + `t_pub` 方式は `raspi/nodes/io_node.py:812-828` に既にある。同じ手を使う）。
- `/ws/control` にも `/ws/telemetry` と同等の schema 版 introduction し、`has_grip` のような新フィールドは**版mafiaして受信する**。

---

## 3. High

### H-1. 最大舵角の値が 7 種類ある（実値 0.47167 = 27.02°）

**該当**
- `config/vehicle.toml:22-25` — コメント「最大路面舵角 [rad] = 26.56°」に対し `max_steer = 0.47167`（= 27.02°）。**コメントと値が食い違う**
- `raspi/core/vehicle.py:119` — 「現在 0.4636 = 26.6°」（**古い**）
- `raspi/core/vehicle.py:121` / `:360` — 既定 `0.524`
- `docs/system_overview.md:87` — 「±30° ＝ 実際は約 ±26.6°」
- `docs/stm32_interface.md:1165` — 「±30°（0.524 rad）」
- `sim/vehicle.py:93` / `:226` — 既定 `0.524`
- `sim/bench.py:309` / `:1186`、`sim/run.py:377` — `--max-steer` の既定 `0.524`
- `sim/park_world.py:15` — 「26.5°」
- `config/generate.py:87` — `maxSteer` の既定 `0`
- `raspi/setup/install_services.sh:94` — `--max-ste 0.524`

**内容**
`max_steer` の正は `config/vehicle.toml:25` の `0.47167`（`tools/calib/steer.py:179` の `steer_link_road(STEER_LINK_X_MAX_RAD, g, c)` から出る値。`raspi/core/control_params.py:108` の `STEER_LINK_X_MAX_RAD = radians(30)` を通す）。**27.02°** であり、`0.4636`（26.6°）でも `0.524`（30°）でもない。

実害の無いもの: `raspi/nodes/io_node.py:1335` の `max_steer=min(args.max_steer, VEHICLE_MAX_STEER)` により `install_services.sh:94` の 0.524 は 0.47167 に丸められる。`sim/vehicle.py:789` も `±sp.max_steer` でクランプする。

**影響**
- `config/vehicle.toml:22` のコメントは**このファイル自身のリファレンス**として読者が最初に読む行であるのに、値と 0.46°ずれている。
- `raspi/core/vehicle.py:119` は「`STM32` の `LIMITS.max_steer_rad` と同じ」と書いているが、firmware は 0.524 相当の旧値（`raspi/tests/test_link_tracker.py:88,93` でも 0.524 を偽 `LIMITS` として使っている）。
- `sim/bench.py:309,1186` / `sim/run.py:377` の 0.524 は**_sim 側でクリープする**：`sim/vehicle.py:789` がクランプするので実害は無いが、`sim/bench.py:788` の `0.98 * spec.max_steer` 判定などは 0.47167 基準なので、ログの解釈が混乱する。

**推奨**
`config/vehicle.toml:22-25` のコメントを実値（27.02°）に直し、`raspi/core/vehicle.py:119` を 0.47167 に更新、`docs/system_overview.md:87` と `docs/stm32_interface.md:1165` を更新。`sim/*` の `--max-steer` 既定は **`config/vehicle.toml` から読む**（`sim/vehicle.py:226` とтой導出）。`config/generate.py:87` の既定 0 は「キーが無い」の意味なので、既定 0.47167 ではなく `null` にするのが安全。

---

### H-2. `axle_model`（車軸ごとの摩擦）が到達不能 — PROGRESS.md が「入れた」と書いている

**該当**
- `sim/vehicle.py:193` — `axle_model: bool = False`（定義）
- `sim/vehicle.py:806`, `:856` — **これだけが使用箇所**
- `sim/vehicle.py:203-273` — `VehicleSpec.load()` に `axle_model` を設定する行が**無い**
- `raspi/auto/slam2d_raceline.py:2480-2484` — `grip_shape = 1` の `_axle()`
- `PROGRESS.md:50-56` — 「シムの車（`sim/vehicle.py`）: `axle_model`・`rear_load_frac`・`load_transfer_hl`・`rear_grip_ratio` を足した」

**内容**
`rg axle_model` の結果はこの 4 行のみ。**`axle_model=True` にする箇所が存在しない**ため、`sim/vehicle.py:806` と `:856` の車軸ごとの摩擦は永久に off。`VehicleSpec` は `frozen=False` の dataclass なので外から書き換えられるが、`sim/bench.py` 等から設定するコードも存在しない。

`PROGRESS.md:50-56` は「実機の g-g と比べた: μg 6.7・直線加速 3.3 のとき、加速の差は 0.3〜0.5、減速の差は 0.2〜0.8 m/s²」と報告しているが、**その比較が `axle_model=True` で行われたのか記録に残っていない**（`axle_model` は `False` のままではモデル差が出ない）。同じ節の「`Vehicle.rear_load_frac`・`load_transfer_hl`（`[control.plant]` から）」も C-2 のとおり実際には既定値 0.59 / 0.1332。

**影響**
`grip_shape = 1` は GUI で selectable（`raspi/auto/slam2d_raceline.py:683`）だが、**実機の Pythone の frog モデル側は `axle_model` を使う前に、`grip_shape=1` 側の `rear_load_frac` が C-2 により 0.59 固定**。検証済みの記録（PROGRESS.md）が実コードと一致しない。

**推奨**
`VehicleSpec.load()` に `axle_model` を**明示的に設定する行**（例: `[sim] axle_model = true`、または `grip_shape` と同じスイッチ）を作り、`PROGRESS.md:50-56` の比較がどの設定で行われたかを記録し直す。暫定でも `axle_model=True` にする行を入れるだけでも「到達不能」 は解消する。

---

### H-3. 制御パラメータの ON/OFF だけが片側追跡で、toml の値がメモリで消える

**該当**
- `raspi/core/control_params.py:94` — `REQUIRED = tuple(k for k in CONTROL_PARAM_IDS if k != "tv_test_moment_nm")` — **`ENABLE_KEYS` は含まれない**
- `raspi/core/control_params.py:99-103` — `ENABLE_KEYS = {tc_enable, tv_enable, wheel_lift_guard_enable, abs_enable}`
- `raspi/core/control_params.py:167-172` — `[control]` の ON/OFF だけ `out` に入る
- `raspi/nodes/io_node.py:843-855` — GUI の ON/OFF イベントで `control_sync.set_wanted(...)` と `_send_ui_setting(...)` の両方を呼ぶ
- `raspi/core/control_params.py:253-261` — `set_wanted()` が `self._base[pid] = value`（**toml 由来の値を上書き**）
- `raspi/nodes/io_node.py:783-795` — `_requery_config()` が ON/OFF 4 つを `ConfigGet` で聞き直す
- `config/vehicle.toml:171` — 「ABS・片輪浮き対策（`abs_enable`・`wheel_lift_guard_enable`）も」— **"[control]" には存在しない**（実キーは `tc_enable`・`tv_enable` のみ）

**内容**
3 つの不整合がある。

1. **`abs_enable` と `wheel_lift_guard_enable` は `[control]` に無い**。`load_control_params` は `out` に放入しないので `ControlParamSync._base` にも入らない。`io_node.py:849,852` の `set_wanted("wheel_lift_guard_enable", ...)` / `set_wanted("abs_enable", ...)` は `raspi/core/control_params.py:257` の `pid not in self._base` で**早期 return（no-op）**。GUI は ON/OFF を送る（`_send_ui_setting`）ので実機の挙動は変わるが、**`ControlParamSync` 側は「知らない値として扱わない」**。
2. **`set_wanted` が `_base` を破壊する**。`raspi/core/control_params.py:259` の `self._base[pid] = float(value)` により、**toml に書かれた値がメモリ上で静かに置き換わる**。`set_wanted` の docstring（「GUI の ON/OFF が変えた機能が、読み戻しで元へ戻らないように」`raspi/core/control_params.py:254-257`）は意図的なコメントだが、**toml を唯一の定義とするなら這是「toml を上書きする」操作**である。`[control].tc_enable = false` を書いていても GUI で ON にすると、**再起動まで toml と食い違う**（再起動時は `_base` が toml から再構築されるので OFF に戻る）。GUI の状態は `localStorage` にあるため、**GUI と toml が正反対の状態で走る**ことがある。
3. **`REQUIRED` に ON/OFF が入らない**ため、`[control]` に `tc_enable` を書き忘れた falt してもエラーにならない（`load_control_params` は `strict` でも `REQUIRED` しか見ない、`raspi/core/control_params.py:198-201`）。

**影響**
GUI の「走行アシスト」トグルと `config/vehicle.toml` の ON/OFF が**どちらも正**でありうる。どちらを「正」とするかが契約として定義されていない。

**推奨**
- ON/OFF も `toml` を正とする（GUI の切り替えは**明示的なPersist**：`[control]` を書き戻す、または「GUI 優先」を `ControlParamSync` に別枠 `_gui_override` として持ち、`set_wanted` で `_base` を破壊しない）。`raspi/core/control_params.py:253-261` を `_base` 破壊から `_overrides` 側へ移すのが最小変更。
- `abs_enable` / `wheel_lift_guard_enable` を `[control]` に実在させる（無ければ `ENABLE_KEYS` から外し、`io_node.py:849,852` の `set_wanted` 呼び出しを削除して `_send_ui_setting` のみに統一）。
- `REQUIRED` に `ENABLE_KEYS` の全部か一部（全項目必須）を明示的に入れる。

---

### H-4. STM32 側プロトコルは「唯一的定义」から外れている（`surge_proto.h` は生成されない、使われない）

**該当**
- `raspi/proto/generate.py:9-13` — 「**STM32 側は `surge_proto.h` を取り込んでいない**（`ras_link.h/.c` に手で写している）。`--check` は Pi 側の生成物しか見ないので、toml を変えたらファームの `RAS_LEN_*`・`RAS_FLAG_*`・`RAS_PROTOCOL_VERSION` も直すこと」
- `raspi/proto/README.md:1-13` — 「`protocol.toml` が**唯一の定義**。ここから Pi 側 Python パーサと STM32 側 C ヘッダの両方を生成する」← **実体と食い違う**
- firmware `src/comm/ras_link.h:27` — `#define RAS_PROTOCOL_VERSION 0x001Du`（**手書き**）
- firmware `src/comm/ras_link.c` — `surge_proto.h` の include が**無い**
- `tools/fwsim/tests/test_control_logic.py:38-41` — `control_params.h` の表と `protocol.toml` の param_id の一致だけを見る
- `docs/archive/review_2026-10-14.md` §2.7 — 「ファームは生成物 `surge_proto.h` を使っておらず、パケット定義は手書きの複製」

**内容**
Pi 側の Python パーサは `protocol.toml` から生成されるが、**STM32 側のバイト配置は手書き**。README と `generate.py` の docstring が**相反する**（「唯一的定义」と「手書き」）ため、読む側はどちらが正か分からない。

現状の実害は小さい（`TELEMETRY` のフィールド順・オフセット・スケールは `ras_link.c` と `protocol.toml` が一致することを本次レビューで数値検証済み。102 バイト一致、48 個の param_id 一致）。しかし **自動検査が無い**。

**影響**
`protocol.toml` を編集して Pi 側だけを再生成しても、firmware は古いまま → 「値が化ける」型のバグが再発しうる。2026-10-14 §2.7 が既に指摘している。

**推奨**
- 最小: `raspi/proto/README.md` の「唯一的定义」を実装に合わせて書き直し、「**Pi 側だけが生成される**」と明記する。
- 本当の解決: firmware の `ras_link.c/.h` を `surge_proto.h` を include する形に変更し、`tools/fwsim` が firmware ツリーを submodule / CI で読めるようにする（現状 `tools/fwsim/tests/test_fw_constants.py` は **CI には入っていない**、後述 M-8）。
-  Regardless に `raspi/proto/generate.py --check` で「firmware 側の `RAS_LEN_*` / `RAS_PROTOCOL_VERSION` が生成値と一致するか」を検査できるようにする（firmware ツリーが無い環境ではスキップ）。

---

### H-5. `[limits]` と `[safety]` の一部が生成物（GUI）に出ない

**該当**
- `config/generate.py:87` — `maxSteer`
- `config/generate.py:142` — `cmdDeadmanMs`（`[safety]` の一部）
- `raspi/core/vehicle.py:406` — `plan_accel_limit_m_s2`（`[safety]`、**生成物に無い**）
- `config/vehicle.toml:126-131` — `[limits]`（**生成物に無い**）
- `raspi/core/vehicle.py:396-398` — `[limits]` 3 項目（Pi 内部）

**内容**
`config/generate.py` は `[safety]` の一部（`cmd_accel_limit_m_s2`・`cmd_steer_rate_limit_rad_s`・`cmd_deadman_ms` 等）と車体諸元を GUI に出すが、
- `[limits].a_lat_m_s2` / `a_accel_m_s2` / `a_brake_m_s2` / `abs_brake_decel_m_s2` は**一切出ない**。GUI は実車の横加速度上限を**知らない**。
- `[safety].plan_accel_limit_m_s2` も出ない（Pi 側の `raspi/core/vehicle.py:406` だけが読む）。

**影響**
GUI は「計画が 3.0 で頭打ち」「実機は 4.09 m/s² まで踏める」等の関係を**表示できない**。学習値の GUI 選択（`raspi/auto/slam2d_raceline.py:683` の `grip_shape`）のように「物理限界を解釈する」操作を追加]',値は GUI に出していない。

**推奨**
`[limits]` と `[safety]` 全体を `config/generate.py` の出力に含め、GUI の設定画面が「どの値が実機の限界で、どれが学习値か」を区別して表示できるようにする。少なくとも `[limits]` を `LIMITS`（STM32 から届く `max_steer_rad` 等）と同じ GUI 画面に並べて**二重表示**にして_sources of truth_ の不一致を 目視できるようにする。

---

### H-6. `a_lat_max` が 2 系統あり、同じ名前で別の値・別の既定を持つ

**該当**
- `raspi/core/vehicle.py:267-269` — `Vehicle.a_lat_max` = `[limits] a_lat_m_s2`（**4.0912**、実測値）／既定 3.5
- `raspi/auto/gap_pursuit.py:147-151` — `ParamSpec(key="a_lat_max", default=3.0, note="★未計測の暫定値")`
- `raspi/auto/follow_object.py:89-93` — 同じく `default=3.0`、`★実車未計測の暫定値`
- `raspi/auto/cam_e2e.py:66-70` — 同じく `default=3.0`
- `raspi/auto/slam2d_raceline.py:280, 309, 1275, 1288, 2645` — `vehicle.a_lat_max`（= 4.0912）を使う
- `raspi/auto/gap_pursuit.py:28` — 「`a_lat_max`（既定 3.0 m/s²）は実車未計測のため」

**内容**
`a_lat_max` という名前が、**車体仕様から導かれる実測値（4.0912）**と、**planner ごとの手置き GUI スライダ（3.0、「未計測の暫定値」）**の両方に存在する。slam2d_raceline は前者、FTG/DE/DP（`gap_pursuit`・`follow_object`）と camera E2E は後者を使う。つまり**同じ「横加速度上限」という概念が、車検証済みの値と「まだ測ってない」値で並存**している。

**影響**
「実車の横加速度上限 4.0912」を反映した車速プロファイルは slam2d 系だけ。FTG/DE/DP は 3.0 のまま 26% 低い速度で曲がる（安全側だが、仕様上は古い「未計測」注释のまま）。

**推奨**
planner 側の `a_lat_max` を**既定で `Vehicle.a_lat_max` から取る**（`ParamSpec` の既定を動的にする仕組みが無いなら、planner 側で `p.get("a_lat_max")` が無ければ `self.vehicle.a_lat_max` を使う）。名前も `a_lat_max_user` 等に変えて**来源を明示**する。

---

## 4. Medium

### M-1. `cmd_deadman_ms` の範囲チェックが 2 箇所にある
- `config/generate.py:142` — `cmdDeadmanMs: {float(safety.get('cmd_deadman_ms', 150))}`（範囲チェック**なし**）
- `raspi/core/vehicle.py:95-107` — `_deadman_ms()` が 20〜1000 ms を検査し、範囲外は stderr に警告して 150 ms
- `raspi/nodes/io_node.py:157` — `cmd_deadman_ms` を Vehicle 経由で取得

`config/generate.py` は `vehicle.py` のロジックを**再実装していない**ので、toml に `cmd_deadman_ms = 150000` と書くと **GUI には 150000 が表示され、Pi は 150 で動く**。GUI 側スライダの `max` は別なので実害は限定的だが、「生成器和 Pi 読取器的校验逻辑が二重」構造。

**推奨**: 範囲チェック（と既定値）を `vehicle.py` に一本化し、`config/generate.py` は `Vehicle.load()` の結果を import して使う（`config/generate.py` は既に `vehicle.toml` を読んでいるので）。

### M-2. `steer_link_x`（逆変換）が firmware と定数・反復回数が違う
- `raspi/core/control_params.py:108` — `STEER_LINK_X_MAX_RAD = math.radians(30.0)`（ハードコード）
- `raspi/core/control_params.py:114-123` — ニュートン法 **6 回**、傾きの下限 0.3
- firmware `src/control/steering.c` / `steering.h` — `STEERING_MAX_ANGLE_RAD` × `STEERING_LINKAGE_RATIO`、ニュートン **4 回**（`raspi/core/control_params.py:107` のコメントがこう書いている）

`STEERING_LINKAGE_RATIO` は firmware の値（0.5）。Pi 側は `radians(30)` を直接持ち、**firmware の値が変わっても追随しない**。逆変換の結果は x（モータ角×リンク比）なので 1 度違えば指令舵角がずれる。`raspi/tests/test_control_params.py:154-163` は Pi 側の自己整合しか見ていない。

**推奨**: `STEER_LINK_X_MAX_RAD` を firmware 値の写しとして `tools/fwsim/tests/test_fw_constants.py` の対象に追加し、firmware 側と**同じ回数・同じ係数**で実装する。

### M-3. `t_capture` は「STM32 換算値」ではないことがある（型契約の違反）
- `raspi/msgs/types.py:10` — 「`t_capture` は STM32 が測った時刻を時刻同期で Pi の `CLOCK_MONOTONIC` に換算した値 [ns]」
- `raspi/core/bus_bridge.py:81-84` — `if t_pi_ns is None or t_pi_ns > now or now - t_pi_ns > _MAX_STATE_AGE_NS: capture = now`（**Pi の受信時刻**に**差し替える**）
- `raspi/core/bus_bridge.py:118-126` — `_sector_start_ns` も同じ fallback
- `raspi/core/timesync.py:245-255` — `to_pi_ns` はサンプル 4 個未満・回帰不成立で `None`
- `raspi/core/timesync.py:208-235` — `_compute_fit` は `len(samples) < 4` なら None、`span < 15 s` でも None

同じフィールドに**2 種類の時間基**が混ざる（STM32 由来 / Pi 受信時刻）。`raspi/msgs/types.py:10` の docstring は「常に換算値」と書いているため、**契約が実装と一致していない**。此外 `now`（単調時刻）と `t_pi_ns`（換算値）は同じ軸なので値は連続だが、**意味（サンプリング点の実測時刻 vs 受信時刻）が違う**。

**推奨**: `t_capture` を `int | None` にして「換算できないなら None」を表現するか、`sync_ok: bool` をフィールドに足す。`bus_bridge.py:81-84` の fallback 自体はこの設計（ Livingston には有用）なので、**契約の記述を実装に合わせる**のが最小。

### M-4. 「スキャン時刻」の定義が 2 つある
- `raspi/msgs/types.py:273-275` — `Scan.t_capture` は「この1周で最初に取得したセクタの時刻」
- `raspi/nav/deskew.py:148` — 脱スキューは `max(t_pt)` を基準時刻に使う（= **最後の点**）

`Scan.t_capture`（最初）と deskew の基準（最後）は**別の時刻**。本次レビューで `ScanAssembler` のセクタ再配置と `deskew.py:point_times_ns` が合成データで一致することは確認したが、**「Scan の基準時刻」の意味が 2 系統**ある。

**推奨**: `Scan` に `t_first` と `t_last` を別フィールドで持ち、基準時刻を明示的に選ぶ（`deskew` は `t_last` を参照する）。

### M-5. STM32 再起動の検出が 2 系統ある
- `raspi/core/link_tracker.py:193-199` — `TimeSync.observe_stm_us` の `t_us` 後退で `sync.resets` が増える → `on_time_reset`
- `raspi/nodes/io_node.py:441-453` — `_on_version()` が「VERSION バースト」を再起動として扱い `self._on_time_reset(rx_ns)` を呼ぶ

2 つの独立した検出器が同じ「STM32 再起動」イベントを表す。**片方だけが発火した場合**、時刻同期の `resets` は incremented されないが `state_builder`/`scans` は reset される（逆も同様）。`raspi/core/timesync.py:107-112` の `reset()` は `self.resets += 1` するので、`sync.resets` を「STM32 再起動回数」として使う消費者（`raspi/core/link_tracker.py:196` 以外，例如 診断表示）は** versioning がずれる**。

**推奨**: `_on_version` からの再起動も `TimeSync.reset()` を通して `resets` をvoltogically 起票し、検出を 1 系統（または明示的に 2 系統と Curry する）にする。

### M-6. 時刻同期に future-jump の余地が残る（2026-10-14 §2.3 の部分修正）
- `raspi/core/timesync.py:177-195` — `offset_ns` は「往復遅延が小さいサンプルのうち、いちばん新しいもの」から取る
- `raspi/core/timesync.py:208-235` — 回帰は 4 サンプル以上かつ 15 s スパンを満たすまで成立しない
- `raspi/core/bus_bridge.py:81-84` — `t_pi_ns > now` なら fallback（**過去方向の跳躍は検出しない**）
- `docs/archive/review_2026-10-14.md` §2.3 — 「時刻の軸」

`to_pi_ns` は過去方向の `offset` 跳躍（例: 遅延の非対称がPLIEDで `latest` サンプルが古い）に**上位落下しない**。`bus_bridge.py:82` の `t_pi_ns > now` チェックは**未来への跳躍だけを**弾く。過去への跳躍（例: 10 秒前のサンプルを picked up）は `t_capture` が 10 秒巻き戻ることを意味する。

**推奨**: `to_pi_ns` の戻り値に「現在からの age」の上限チェックを入れ（`bus_bridge.py:118-126` の `_MAX_SECTOR_AGE_NS` と同じ考え方）、`timesync` 側で弾く。

### M-7. CI が壊れている（削除済み `tools/sysid` を参照）
- `.github/workflows/ci.yml:61` — `run: python -m pytest tools/tests tools/sysid/tests/test_link.py -k "not TestAnalyzeFolds" -q`
- `tools/` には `sysid` が**無い**（`calib`, `cam_calib`, `fwsim`, `tests` のみ）
- 再現: `pytest tools/tests tools/sysid/tests/test_link.py` → `ERROR: file or directory not found: tools/sysid/tests/test_link.py` / `no tests ran`

CI の python ジョブが**必ず失敗**する（pytest の終了コード 4）。`tools/check.sh:66-67` は別のコマンド列（`tools/tests tools/calib/tests tools/fwsim/tests/test_fw_constants.py tools/fwsim/tests/test_control_logic.py`）なので、**手元は緑・CI は赤**という乖離。

**推奨**: `.github/workflows/ci.yml:61` を `tools/check.sh` の内容に合わせる、または `tools/check.sh --fast` を CI から呼ぶ形（「生成物チェック + pytest」の入口を 1 つに）。

### M-8. CI が firmware との突き合わせを回さない
- `.github/workflows/ci.yml:61` — `tools/fwsim/tests/test_control_logic.py` を含まない
- `tools/check.sh:66-67` — `tools/calib/tests`・`tools/fwsim/tests/test_fw_constants.py`・`tools/fwsim/tests/test_control_logic.py` を含む
- `tools/fwsim/tests/test_control_logic.py:42-48` — `test_vehicle_toml_is_within_firmware_range`（`[control]` の全値が firmware の範囲内かの検査）
- `tools/fwsim/tests/test_fw_constants.py:98-115` — firmware 定数と Pi の写しの照合

`test_vehicle_toml_is_within_firmware_range` は**まさに「[control] の値が firmware が受け付けない範囲に居ないか」を検査する契約テスト**だが、**CI では一度も実行されない**（`tools/fwsim` は firmware ツリーを必要とするため CI では skip される可能性はあるが、CI は `tools/fwsim` を全く列挙していない）。

**推奨**: firmware ツリーを submodule として CI に取り込むか、少なくとも `tools/fwsim/tests/test_control_logic.py` を CI の pytest に追加する（firmware 不在時は skip される設計なので harmless）。

### M-9. bench の指令周波数が実機と違う
- `sim/bench.py:86` — `CMD_HZ = 100`
- `raspi/auto/base.py:40` — `CMD_HZ = 50`
- firmware `DRIVE_HOST_COMMAND_PERIOD_S` = 0.02 s（= 50 Hz）

bench は実機の 2 倍の速度で指令を出している。FF やランプの比較、`cmd_stale`（`auto_cmd_stale_ms = 200`）の模擬、`_MAX_CMD_AGE` 系の判定で**実機と異なる挙動**になる。

**推奨**: `sim/bench.py:86` を 50 に揃える、または「bench の既定を実機に合わせる／明示的に `--cmd-hz` で変更可」とする。

### M-10. `docs/control_constants.md` の数値が実装から乖離
- `docs/control_constants.md:34` — 「`VEHICLE_BRAKE_DECEL_M_S2` = **2.47**」→ 実値 `config/vehicle.toml:131` `abs_brake_decel_m_s2 = 2.165`
- `docs/control_constants.md:38` — 「= 3.04」→ 実値 `disarm_brake_timeout_s(2.165)` = **3.4642**（`raspi/core/control_params.py:85-89`）
- `docs/control_constants.md:41` — 「`Vehicle.speed_loop_tau_s` = 0.155」→ 実値 **0.06872**（`raspi/core/vehicle.py:295` を `Vehicle.load()` で確認）
- `docs/control_constants.md:45` — 「`launch_exit_torque_nm` = **0.006346**」→ 実値 `config/vehicle.toml:215` = **0.011629**

同 doc の `:79`・`:85` は「firmware は写しを持たない」と正しく書いており、**表の値だけが古い**。

**推奨**: この doc は「定数の写し」なので、**`config/generate.py` の出力**か、`tools/fwsim/tests/test_fw_constants.py` のような検査対象にする。

### M-11. `docs/development.md` が生成器の现状を逆に説明している
- `docs/development.md:76` — 「`raspi/msgs/types.py` | **`gui/src/types.ts` も手で直す**（写しなのでズレる）」
- `docs/development.md:857` — 「**GUI にも見せるなら `gui/src/types.ts` を手で写す**（生成器は挟んでいない）」

実体は `config/gen_msgs.py` が `gui/src/generated/msgs.ts` を生成している（`config/gen_msgs.py:22-36`）。**「手で直す」と「生成器は挟んでいない」は両方誤り**。

**推奨**: 両方書き換える。加えて `gui/src/types.ts` に**残す**型（`MapData`・`Snapshot`・`ControlStatus`・`CmdOut`・`MapFile`）について、`config/gen_msgs.py:32-36` は「Python 側に対応物が無いので手書きのまま残す」と説明しているが、`MapFile` は `raspi/auto/mapstore.py:182` に**Python 側の実装が存在する**。生成対象を広げる余地がある。

### M-12. `PROGRESS.md` のプロトコル版が古い
- `PROGRESS.md:12` — 「**プロトコルは v0.23**（`raspi/proto/protocol.toml` が正）」
- 実際: `raspi/proto/protocol.toml` / firmware `ras_link.h:27` = `0x001D` = **v0.29**（`docs/uart_protocol.md:3`、`docs/README.md:8,16` と一致）

`PROGRESS.md` は冒頭に「新しいセッションではまずこれを読む」とある（`PROGRESS.md:3`）ので、**最初に読む文書の 1 行目が 6 版古い**。

**推奨**: `PROGRESS.md:12` を v0.29 に更新するか、版数を `protocol.toml` から**自動記入**する。

### M-13. `[sim]` の 4 キーは常に `[limits]` に上書きされる（編集しても効かない）
- `sim/vehicle.py:211-220` — `[limits]` があれば `sim_only["mu"]` / `["drive_accel_m_s2"]` / `["speed_decel_m_s2"]` / `["brake_decel_m_s2"]` を**上書き**
- `config/vehicle.toml:136-139` — 「★ μ・加速の上限・車速指令の減速の上限・ABS の減速度の 4 つは、**シムは [limits] から作る**。下の 4 行は [limits] が無いときの予備」
- `config/vehicle.toml:146-149` — `[sim] brake_decel_m_s2 = 2.4743645729434935` に対し `[limits].abs_brake_decel_m_s2 = 2.165`（**12.5% 違う**）

「予備」と理解して編集した値が黙って無視される。`[sim].drive_accel_m_s2 = 3.0699` も `[limits].a_accel_m_s2 + rr = 2.925 + 0.35 = 3.275` に上書きされる。

**推奨**: `[sim]` の 4 行を**削除**して `[limits]` に一本化するか、`[sim]` の値を残すなら「`[limits]` があるときは `[sim]` は無視される」ことを**キー名ではなくコメントと `config/check_docs.py` で強制**する。

### M-14. `/ws/control`（GUI↔Pi の JSON）に版・単位の契約が無い
- `gui/src/ws/control.ts:32-50` — `ServerMsg` に版フィールドが無い（`type` による分岐のみ）
- `gui/src/ws/control.ts:36` — 「境界の実行時検証は `/ws/telemetry` 側の札（`MSGS_SCHEMA`）が担っている」← **誤り**（`/ws/control` には `MSGS_SCHEMA` は届かない）
- `raspi/msgs/schema.py:65` — `schema_version()` は msgpack チャネル用
- `docs/archive/review_2026-10-14.md` §2.7 — 「契約に版と単位が入っていない」

`/ws/telemetry` には版があるのに `/ws/control` にはない、という**非対称**。新機能の `has_grip`（C-4）や `map_files` の追加 field は、この非対称の Healing 側に乗る。

**推奨**: `/ws/control` の JSON にも `schema`（または `proto_version`）を 1 個入れ、GUI 側起動時に不一致を warning 表示する。

---

## 5. Low

### L-1. `measured` フラグが 2 つの意味を持つ
- `raspi/core/vehicle.py:115` — 「諸元が実測済みか。**False の間は経路追従の精度を云々しても意味が無い**」
- `config/vehicle.toml:13` — `measured` は**存在しない**（実キーは `encoder_ticks_per_rev = 0` で、「★未確定（STM32 の実測待ち）。どのコードも読まない」）
- `raspi/core/vehicle.py:357` — `measured=bool(d.get("measured", False))` → **常に False**
- `rg "\.measured"` → 読み手は**どこにも居ない**（`ScanWindow.measured` のみ別物）

`Vehicle.measured` は**常に False** であり、誰も読まない。`config/vehicle.toml:13` の `measured=false` 相当は `encoder_ticks_per_rev = 0` という別のフィールドで、**別の意味**。

**推奨**: `Vehicle.measured` を使うか消すかを決める。残すなら「どの諸元が実測済みか」を `vehicle.toml` に**明示**して `measured = true` を付ける（現状 `wheelbase`・`track`・`max_steer` は実測確定なのに false のまま）。

### L-2. `Vehicle.load()` が全例外を握りつぶして既定値で動く
- `raspi/core/vehicle.py:326-344` — `except (TypeError, ValueError, KeyError, AttributeError, IndexError)` → `print(..., file=sys.stderr)` して `cls()`（既定値）を返す

`max_steer` の既定は 0.524（`raspi/core/vehicle.py:121`）なので、**キー名の 1 文字ミスで 30° の車が静かに Deploy される**。C-2 の `rear_lateral_share` 正是この経路で静かに 0.59 になっている。

**推奨**: 致命的でない項目は個別に fallback し、`[limits]`・`[control.plant]` のような**同定値**は「読めなかったらエラー」にする。

### L-3. `convert.py` がスケール定数を手書きしている
- `raspi/msgs/convert.py:30-45` — 15 個のスケール定数（`_SPEED`…`_LIDAR_C`）／「（uart_protocol.md §5.3。protocol.toml の META と一致させること）」
- `raspi/msgs/convert.py:47-50` — `Command.accel_limit` だけは `packets.Command.META` から取っている（**この後者だけrott防止されている**）
- `raspi/tests/test_proto.py:481-493` — META との一致をテスト（今回実測で全一致を確認）

**推奨**: `_SPEED` 等も `packets.<X>.META["<field>"][0]` から機械的に取る（`convert.py:47-50` の方式を全部に適用）。

### L-4. `maxSteer` の既定が生成側 0 / Pi 側 0.524 で不一致
- `config/generate.py:87` — `maxSteer: {float(spec.get('max_steer', 0))}`
- `raspi/core/vehicle.py:121` — `max_steer: float = 0.524`

toml に `max_steer` が無いとき、**GUI は 0 rad（舵が全く切れない）**、`Pi` は 0.524 rad。`gui/src/store/ui.ts:196` の `PI_MAX_STEER_CAP = VEHICLE.maxSteer` が 0 になり、**GUI のスライダが動かなくなる**。

**推奨**: 生成側の既定を 0.524 に揃える、または `null`（未設定）にして UI 側で「未受信」と明示する。

### L-5. toml スキーマの説明が古い
- `docs/system_overview.md:419`、`docs/architecture.md:162-168` — スキーマを「幾何・質量・センサ・`[dynamics]`・`[control]`・`[safety]`」と説明
- 実在するセクション: `[limits]`・`[sim]`・`[control.plant]` が**説明に無い**

**推奨**: 3 節を追加。

### L-6. `install_services.sh` の `--max-ste 0.524` は実害が無い（が誤解を招く）
- `raspi/setup/install_services.sh:94` — `--max-ste 0.524`
- `raspi/nodes/io_node.py:1335` — `max_steer=min(args.max_steer, VEHICLE_MAX_STEER)` で 0.47167 に丸められる

**推奨**: `install_services.sh` の値を 0.47167 に直す（または引数を省略して既定に任せる）。

### L-7. `sim/park_world.py:15` の角度が 26.5°
- `sim/park_world.py:15` — 「wheelbase/tan(max_steer) = 0.23/tan(26.5°) = 0.46m」

実値は 27.02°（0.47167）。0.46 m 対 0.453 m。

### L-8. `config/check_docs.py` の検査範囲が狭い
- `config/check_docs.py:42-49` — `CLAIMS` で 4 つの文書文字列だけ検査
- `docs/architecture.md`・`docs/control_constants.md`・`PROGRESS.md` の版数記載は**検査対象外**

M-10・M-12 が指摘した乖離が**機械的に検出できない**理由。

---

## 6. 設計ボトルネック

### B-1. 生成器が 3 つあり、比較の入口が無い
`config/generate.py`（vehicle.toml → GUI）、`raspi/proto/generate.py`（protocol.toml → packets.py / surge_proto.h）、`config/gen_msgs.py`（msgs/types.py → GUI）。**どれも `--check` を持つが、まとめて叩くコマンドが無い**。

- `tools/check.sh` は 4 つの `--check` を叩く（`tools/check.sh:37-42`）が、**CI は 4 つを個別に列挙**している（`.github/workflows/ci.yml:44-49`）。**2 箇所に同じ手順が書かれ、片方だけ古くなっている**（M-7）。
- `raspi/proto/README.md:28` 以降の「使い方」と `docs/development.md` の手順も別書き。

**推奨**: `make check`（または `tools/check.sh --fast` 1 本）を唯一の入口にし、CI・README・`docs/development.md` をそこに参照させる。

### B-2. `config/vehicle.toml` 1 ファイルが「層・出力先ごとに違う意味」を持つ
同じファイルが以下を同時にしている:
- 実車の物理諸元（`wheelbase`, `max_steer`）
- 同定で得た安全限界（`[limits]`）
- シム専用の暫定値（`[sim]`）
- firmware の plant モデル（`[control.plant]`）
- 制御ゲイン（`[control]`）
- 安全側の指令制限（`[safety]`）

**節で分けているのに「読み手が誰か」を表す記号がない**ため、C-1・C-2・M-13 のように同じ名前が別の層で別の意味を持てる。

**推奨**: 節名を「層 × 出力先 × 誰が書くか」を含む形にする（例: `[measured.limits]`＝校正が書く、`[sim.tuning]`＝人が手置き、`[fw.plant]`＝firmware 用）。**最低でも「書き手」と「読み手」を各節のコメントに 1 行ずつ書く**。

### B-3. 「生成物」と「人手による写し」が同じリポジトリに混在
- 生成物: `gui/src/generated/vehicle.ts`・`gui/src/generated/msgs.ts`・`raspi/proto/generated/packets.py`・`raspi/proto/generated/surge_proto.h`
- 人手写し: `raspi/msgs/convert.py:30-45`（スケール）、`raspi/core/control_params.py:78,87`（firmware 定数）、`raspi/core/control_params.py:108`（`STEER_LINK_X_MAX_RAD`）、`gui/src/types.ts`（`MapFile` 等）、`gui/src/store/ui.ts:195`（`PI_MAX_SPEED_CAP`）

**チェック Juliet ouis** が入るのは「生成物 vs 正」の 3 組だけ（`config/generate.py --check` 等）。**人手写し 5 箇所のうち 3 つに検査が無い**。

**推奨**: 人手写しの定数を「firmware 定数」「プロトコル定数」「GUI 運用値」に分類し、firmware 定数とプロトコル定数を `tools/fwsim/tests/test_fw_constants.py` の検査対象扩充する（M-8 の CI 修正と同時に）。

### B-4. GUI の型が 2 系統（生成物 + 手書き `types.ts`）で境界が曖昧
- 生成: `gui/src/generated/msgs.ts`（`config/gen_msgs.py`）
- 手書き: `gui/src/types.ts`（`MapData`・`Snapshot`・`ControlStatus`・`CmdOut`・`LogFile`・`MapFile` など）

`config/gen_msgs.py:32-36` は「Python 側に対応物が無いので手書きのまま残す」とするが、`MapFile` は `raspi/auto/mapstore.py:170-183` に**Python 側の実装がある**（C-4 の `has_grip` がまさにその例）。

**推奨**: 対応物が Python 側にある型は `gen_msgs.py` の生成対象へ移す（`mapstore.list_maps()` の戻り値形状を型として宣言できるようにする）。

### B-5. 時刻の基点が 3 つあり、「どの時刻か」を型の側で強制できない
- STM32 の生 u32（`t_stm_us`、`raspi/msgs/types.py:210`）
- Pi の `CLOCK_MONOTONIC`（`time.monotonic_ns`）
- UNIX epoch（`raspi/rec/mcap_log.py:285-354` の `t0_mono`/`t0_unix` ペアのみ）

**NTP / chrony はリポジトリに一切無い**（`rg -i "ntp|chrony"` でヒットなし）。epoch への変換は**ログ書き出し時だけ**、`t0_mono`/`t0_unix` のペアとして保持される。これは「取り違えない」良い設計だが、**型には現れない**ため、M-3/M-4 の混在が許されている。

**推奨**: `t_capture` / `t_pub` / `t_start_us` それぞれに「どの基点か」を表す Doc 契約を統一し、`bus_bridge.py` の fallback では**基点が変わることを明示**する（値 型を `int` → `Timestamp` の newtype にすると型で防げる）。

### B-6. 「数値の写し」は静かに古びる
`docs/control_constants.md`・`docs/system_overview.md`・`docs/stm32_interface.md`・`PROGRESS.md`・`config/vehicle.toml` のコメント — すべて**同じ数値の写し**を持ち、**更新の責任者がコードの所有者と一篇の異なる**。M-10・M-12・H-1 がその実例。

**推奨**: 定数の「正」を 1 箇所（`vehicle.toml` + `protocol.toml` + firmware の `control_params.h`）に寄せ、**写しの値を検査する**（`tools/fwsim/tests/test_fw_constants.py` の 🤔 対象に追加、または `config/check_docs.py` の `CLAIMS` を増やす）。

---

## 7. テストの穴

| # | 穴 | 影響 | 現状 |
|---|---|---|---|
| T-1 | `[limits]`・`[sim]`・`[control.plant]` の**相互一致**を検証するテストが無い | C-1 / C-2 / M-13 が検出されず Shine | `raspi/tests/` に `[limits]` を読むテストは `test_speedctl.py:41,72` と `test_vehicle.py:51`（異常系）だけ。**相互比較のテストは 0** |
| T-2 | `Vehicle`（Pi）と `sim.VehicleSpec`（シム）が**同じキーを読んでいる**ことのテストが無い | C-2 のように片方だけ欠ける | 無し。`raspi/core/vehicle.py:396-400` と `sim/vehicle.py:211-220` は**別々のキーリスト**をハードコードしている |
| T-3 | `max_steer == steer_link_road(STEER_LINK_X_MAX_RAD, gain, cubic)` のテストが無い | H-1 のコメント/値の乖離（H-1 はまさにこれ）を検出できない | `tools/calib/steer.py:179` で計算しているだけ。テストは `raspi/tests/test_control_params.py:154-163`（Pi 内部の自己整合のみ） |
| T-4 | `[control]` ⊂ firmware の許容範囲 のテストが**CI で回らない** | M-8 | `tools/fwsim/tests/test_control_logic.py:42-48` に**存在する**が CI の pytest 列挙に入っていない（`.github/workflows/ci.yml:61`） |
| T-5 | GUI の ON/OFF トグル → `ControlParamSync` → toml の関係を検証するテストが無い | H-3 | `raspi/tests/test_io_node_ui_event.py` はあるが、`set_wanted` が `_base` を破壊する（H-3-2）ことを捕まえる筈がない |
| T-6 | `grip_clear` の「planner 未ロード」経路のテストが無い | C-4-1 | `raspi/tests/test_planning_node.py:498-507` は `planner` **あり**のケースのみ |
| T-7 | `grip_clear_seq` の**巻き戻り**（telemetry_node 再起動）のテストが無い | C-4-2 | 無し。`raspi/tests/test_telemetry_node.py:405-434` は `_auto_grip_clear_seq` の増加のみ |
| T-8 | `/ws/control` の**版**のテストが無い | C-4-3 / M-14 | `/ws/telemetry` の版は `raspi/tests/test_msgs.py` にあり。`/ws/control` には版が無いため**テストすべきものが無い** |
| T-9 | `t_capture` の「STM32 由来 / Pi 受信由来」の**混在**を検出するテストが無い | M-3 | `raspi/tests/test_bus_bridge.py` はあるが、fallback 経路の**時刻の基点**は検証していない |
| T-10 | `Scan.t_capture`（最初）と deskew の基準（最後）の**規約**を固定するテストが無い | M-4 | `raspi/tests/test_scan_anchor.py` はあるが、規約の記述が無いため**規約を書けばテストできる状態** |
| T-11 | `axle_model` / `grip_shape=1` の**到達可能性**を検証するテストが無い | H-2 | `raspi/tests/test_slam2d_raceline.py` に `grip_shape` のテストは無い（`raspi/auto/slam2d_raceline.py:2480-2484` の `_axle` は既定 `grip_shape=0` のため**呼ばれない**） |
| T-12 | firmware ツリーが無い環境で**契約テストが skip される**ことの明示がない | M-8 | `tools/fwsim/tests/test_control_logic.py:56` に `@unittest.skipUnless(HAVE_FW, ...)` がある（`TestParamTable` 側は無いが） |

---

## 8. 補足：2026-10-14 レビュー / PROGRESS.md の現状

### 修正済み（コード側で確認）
| 指摘 | 現状 |
|---|---|
| §2.7 GUI の `maxSteer` 既定 0.785 rad | 修正済み。`gui/src/store/ui.ts:196` は `VEHICLE.maxSteer`（生成物）を参照 |
| §2.7 io_node のフォールバック `--max-steer 0.524` | **部分修正**。`raspi/nodes/io_node.py:1335` で `min(args.max_steer, VEHICLE_MAX_STEER)` にクランプ。ただし `raspi/setup/install_services.sh:94` は 0.524 のまま（H-1 / L-6） |
| §2.8 STM32 だけが再起動したときの `auto_stop_margin_cm` 等 | 修正済み。`raspi/nodes/io_node.py:783-795` の `_requery_config` と `:797-801` の `_resend_ui_settings` |
| §2.3 時刻の軸（`to_pi_ns` の fit キャッシュ） | **部分修正**。`raspi/core/timesync.py:197-206` に `_fit_cache` が入り、回帰不出来の間も前回の回帰を使う。過去方向の跳躍は未対応（M-6） |
| §2.4 検証器が楽観的 | **一部修正**。`raspi/core/control_params.py:94` の `REQUIRED` と `raspi/core/vehicle.py:95-107` の `_deadman_ms` 範囲検査は入った。ただし ON/OFF は `REQUIRED` に含まれない（H-3） |
| §J 生成器の `--check` を CI に置く | **修正済み**。`.github/workflows/ci.yml:44-49` に 4 つの `--check` がある。ただし pytest 側の指定が壊れている（M-7） |

### 未修正
| 指摘 | 現状 |
|---|---|
| §2.7 最高速の上限が 3 か所 | **未修正 + 1 か所増加**（C-3） |
| §2.8 `deploy.sh --restart` だけで `[control]` が STM32 に入らない | **未修正**。`[control]` は起動時に 1 回だけ読まれる（`raspi/core/control_params.py:151-206`）。`io_node.py:1358` の `_requery_config` は STM32 側の値を**聞き直す**だけで、Pi 側 `[control]` を**送り直す**のは `ControlParamSync` の役割。STM32 再起動時は `raspi/nodes/io_node.py:459-466` で `control_sync.reset()` → 再送するが、**Pi だけ再起動（`--restart-io` 以外）した経路**は未確認 |
| §2.3 時刻の軸（future-jitter） | **部分修正**（M-6） |
| §J CI と `check.sh` の乖離 | **未修正**。むしろ拡大（M-7 / M-8） |

### PROGRESS.md の未反映
- `PROGRESS.md:12` — 「プロトコルは v0.23」→ 実際 v0.29（M-12）
- `PROGRESS.md:50-56` — 「`axle_model` を足した」「`Vehicle.rear_load_frac`・`load_transfer_hl`（`[control.plant]` から）」→ 両方とも実装が到達不能／既定値（C-2 / H-2）
- `PROGRESS.md:36-41` — 「GUI の『限界のモデル』（`grip_shape`、本番走行の学習の組）を選択肢で出す」→ **UI には出ている**（`raspi/auto/slam2d_raceline.py:683`）が、判定に使う `rear_load_frac` が固定値（C-2）

---

## 9. 今回調べたファイル

**設定・生成**
`config/vehicle.toml`・`config/generate.py`・`config/gen_msgs.py`・`config/check_docs.py`

**Pi 本体**
`raspi/core/vehicle.py`・`raspi/core/control_params.py`・`raspi/core/bus_bridge.py`・`raspi/core/timesync.py`・`raspi/core/link_tracker.py`・`raspi/msgs/types.py`・`raspi/msgs/convert.py`・`raspi/msgs/schema.py`・`raspi/proto/protocol.toml`・`raspi/proto/generate.py`・`raspi/proto/README.md`・`raspi/proto/framing.py`

**ノード**
`raspi/nodes/io_node.py`・`raspi/nodes/telemetry_node.py`・`raspi/nodes/planning_node.py`

**オート・シム**
`raspi/auto/slam2d_raceline.py`・`raspi/auto/mapstore.py`・`raspi/auto/base.py`・`raspi/auto/gap_pursuit.py`・`raspi/auto/follow_object.py`・`raspi/auto/cam_e2e.py`・`sim/vehicle.py`・`sim/bench.py`・`sim/run.py`・`sim/park_world.py`

**ツール**
`tools/fwsim/plant.py`・`tools/fwsim/fw.py`・`tools/fwsim/tests/test_control_logic.py`・`tools/fwsim/tests/test_fw_constants.py`・`tools/calib/steer.py`・`tools/check.sh`

**GUI**
`gui/src/store/ui.ts`・`gui/src/types.ts`・`gui/src/ws/control.ts`・`gui/src/components/MapLibrary.tsx`・`gui/src/App.tsx`

**firmware（外部ツリー）**
`/Users/banbatakumi/GitHub/AutonomousCar/Program/MainF446RE_V3/src/comm/ras_link.h`・`src/comm/ras_link.c`・`src/control/control_params.h`・`host/host_sim.c`

**CI・文書**
`.github/workflows/ci.yml`・`PROGRESS.md`・`docs/control_constants.md`・`docs/uart_protocol.md`・`docs/stm32_interface.md`・`docs/system_overview.md`・`docs/architecture.md`・`docs/development.md`・`docs/archive/review_2026-10-14.md`・`docs/README.md`

---

## 10. 補足：作業ツリーの未追跡ファイルについて

`docs/archive/review_2026-10-21.md`（86KB, 2026-10-21 01:23）は **`git ls-files` に無い未追跡ファイル**で、本レビューとは独立に作られた**別のレビュー（先行版）**とみられる。C-1（μ・`[limits]`/`[control.plant]` の矛盾）、H-2（`axle_model` ）、C-2（`rear_lateral_share` 欠落）、M-7/M-8（CI と fwsim テスト）で**重なっている**。同じ指摘が 2 か所に散らばっている状態なので、**1 本に統合**するのが望ましい。

なお、本レビューで検証した範囲（`config/generate.py --check`・`raspi/proto/generate.py --check`・`config/gen_msgs.py --check`・`config/check_docs.py --check` すべて PASS、`raspi/tests` 1666 passed / 1 skipped、`TELEMETRY` 102 バイトのフィールド順・オフセット・スケールの一致、48 param_id の一致、`convert.py` の全スケール定数と `packets.*.META` の一致、`steer_link_road(π/6, 1.101158, -0.730735) = 0.47167` の数値一致）はいずれも**問題なし**と確認済み。**生成物と正のずれは無い**。本次見つかった問題は-Polices は「正の**定義**が 1 箇所にない」側に集中している。
