"""システム同定 — 前後運動試験（`config/vehicle.toml` `[dynamics]` の実測手順）。

直進で「加速 → 定速 → 減速 → 元の位置へ後退」を、**加速度を段階的に上げながら**繰り返す
（`STAGES`）。1つの記録から次を測る（解析は `tools/sysid/fit.py` の `fit_accel()`）：

- `speed_plant_gain`・`rolling_resistance`: ファームの速度制御（目標のランプ→PI→トルク上限、
  `sim/vehicle.py` の `SpeedController`）に記録の `/cmd` を入れた応答を当てはめる。段ごとに違う
  加速度のランプの始まり・終わりの角と、定速で落ち着くまでの応答が材料
- `drive_accel_m_s2`・`drive_fade_speed_m_s`・`drive_top_speed_m_s`: 加速度の上限（モータ・TC）
  とその速度依存。指令どおりに加速できる段と、頭打ちになる段（最後は全開）の違いで見える
- `brake_decel_per_nm`・`brake_decel_m_s2`: `brake` を立てたときの「制動トルク→減速度」の傾きと、
  タイヤのグリップによる頭打ち（各段の1回目。段ごとに制動トルクを変える）
- `speed_decel_m_s2`: 速度指令を0にしたときの減速の上限（各段の2回目）

定速の区間（舵0°・前進）は、舵の中立ずれ（`fit_geometry()`）の材料にもなる。

## 1つにまとめた理由（2026-09-27）

以前は速度応答試験（0.4→1.2→0.4m/s の段）と加減速試験（全開加速だけ）が別だった。バンビの
「試験はなるべくシンプルな方が正しい値が取れる」「加速度を段階的に変える前後運動」を受けて
まとめた。車体の利得と加速度の上限は互いに補い合うので、以前も解析は2つの記録を合わせて
当てはめていた。段階的にすると、上限が「指令どおり→頭打ち」の形で直接見え、**タイヤの
空転（加速がグリップを超える）が始まる段も分かる**。

## 段の加速度の作り方

段の加速度 `a` では、目標速度をプランナー自身が `a` で上げ、同時に `AutoState.accel_limit = a`
（`COMMAND.accel_limit`）も送る。ファームは目標を `accel_limit` でランプさせるので、10Hz の
判断の刻み（目標は0.1sごとに `a`×0.1 ずつ増える）がちょうど滑らかな一定の傾きになる。
目標だけを上げると、刻みごとにファームのランプ（3.0m/s²）で急に追いついて止まる階段になり、
`accel_limit` だけだと目標が最初から一定なので、加速中の区間を定速の区間と区別できない
（中立ずれの材料に加速中のサンプルが混ざる。全開加速中は左右モータのトルク差でもヨーが出る）。
全開の段は以前と同じく目標を `v_max` より上（`v_max+1.0`）に置く（目標に近づいて速度PIが
自分で緩める応答ではなく、上限を測るため）。減速は全段とも GUI の上限（ファームのランプ3.0）で
行う（停止距離の見積もり `stop_distance()` がその前提）。

## ブレーキは段ごとに強さを変える（2026-09-27）

各段の1回目の減速は `brake` で、制動トルク（1輪あたり、`AutoState.brake_torque`→
`COMMAND.brake_torque`）を段ごとに 0.04・0.08・0.12・0.15N·m（最大）にする。弱い方の2段で
「トルク→減速度」の傾きが、強い方でタイヤのグリップによる頭打ちが見える。以前は GUI の
制動トルク（最大にしておく手順）の1通りだけで、弱いブレーキの効きはシムの名目の式のまま
だった（自律走行で任意の強さのブレーキを使うには、この傾きが要る）。止まりきれなくなる
地点の見積もりも、その強さの保守的な減速度で行う（`brake_decel_guess()`）。

## 後輪が滑ったら上の段へ進まない（2026-09-27）

TC の旗（ファームが滑り率20%で立てる）ではなく、後輪の滑り率そのもの（`rear_slip`）で判断する。
実機の最初の記録では、TC の旗が30〜50msだけ立った（トルクもほぼ削っていない）のを「介入」と
数えて、全開の段と最大ブレーキの段を飛ばした。

- **滑った**（加速中に滑り率が `REAR_SLIP_LIMIT` を超えた状態が 0.1s 続く）: その段は最後まで走り、
  上の段へは進まない（「完了（後輪が滑った…）」）。滑りながら（TC が効きながら）出る加速度こそが
  この車の出せる加速度（シム・学習に要る値）なので、止めずに測る
- **抑えきれない空転**（後輪の周速が前輪より 0.2m/s＋30% 以上速い状態が 0.1s 続く）: その段で
  すぐ制動し、戻ってから試験を終える（「完了（空転を検出…）」）。`SensorGuard`（0.3s、前輪
  エンコーダの不調で中止）より先に効く。制動はロックしない強さ（`ABORT_BRAKE_TORQUE_NM`）

## 使える直線の長さに収める

加速・定速の途中でも、**`run_length_m` に止まりきれなくなる地点**（オドメトリで見た前進距離＋
想定の停止距離）に来たら、すぐ減速に移る（`_sysid_common.stop_distance()`）。

## 戻りフェーズ

減速して止まった後、`VehicleState.odom_dist`（前輪の累積走行距離。舵0°なので射影なし
でそのまま前後の変位）を見ながら開始位置まで後退する。`target_speed`が負で
`brake=False`なので、解析側の加速・減速の区間選別には引っかからない。オドメトリ
不調で戻りきれない場合に無限後退しないよう、前進距離からタイムアウトを計算する。

## 試験開始/中止を押すまでステップは進まない

`TestGate`（`_sysid_common.py`）参照。
"""

from __future__ import annotations

from ..msgs.types import AutoState, Scan, VehicleState
from ._sysid_common import (ABORT_BRAKE_TORQUE_NM, REAR_SLIP_LIMIT, OdomRun, SensorGuard, TestGate,
                            abort_state, brake_decel_guess, rear_slip, settle_state, stop_distance)
from .base import Planner

__all__ = ["SysIdAccel", "STAGES"]

#: io_node `--max-speed`（`install_services.sh`）。これを超える `target_speed` は切り捨てられる
_PI_MAX_SPEED = 3.0
#: 全開の段で `target_speed` を `v_max` からどれだけ上に置くか [m/s]（docstring参照）
_ACCEL_OVERSHOOT = 1.0
#: 停止距離の見積もりに使う全開加速中の加速度 [m/s²]（ファームの目標ランプの上限。測る対象
#: そのものなので `vehicle.toml` に頼らない）
_ACCEL_FOR_STOP_M_S2 = 3.0
#: 段の並び `(加速度 [m/s²], 定速にする速度 [m/s], ブレーキのサイクルの制動トルク [N·m])`。
#: 加速度 0 は全開（GUI の上限＝ファームのランプ3.0で、目標は `v_max+1.0`、定速にしない）。速度は
#: 「加速→`hold_s`の定速→停止」が2mの直線に収まる値（収まらなければ定速を切り上げる）。
#: 制動トルクの最大（0.15）は STM32 の `DRIVE_MAX_BRAKE_TORQUE_NM`
STAGES: tuple[tuple[float, float, float], ...] = (
    (0.8, 1.0, 0.04), (1.6, 1.3, 0.08), (2.4, 1.5, 0.12), (0.0, 2.0, 0.15))


class SysIdAccel(Planner):
    id = "sysid_accel"
    name = "システム同定: 前後運動"
    description = ("直線で、加速度を0.8→1.6→2.4m/s²→全開と段階的に上げながら「加速→定速→減速"
                   "（ブレーキと速度指令0を交互。ブレーキの強さも段ごとに上げる）→後退で戻る」を8回。"
                   "後輪が滑ったらその段まで、抑えきれない空転ならすぐ止める。直線2m＋車長が要る。約40秒")
    category = "sysid"
    stats = ()

    #: フェーズ番号。`_PHASE_LABELS`・`plan()`と対応
    _ACCEL, _DECEL, _HOLD, _RETURN = 0, 1, 2, 3
    _PHASE_LABELS = {_ACCEL: "加速", _DECEL: "減速", _HOLD: "停止", _RETURN: "戻り"}

    #: 戻りフェーズの「元の位置」判定の許容誤差 [m]
    _RETURN_TOLERANCE_M = 0.05
    #: 戻りフェーズの安全装置の下限時間 [s]（`OdomRun.RETURN_TIMEOUT_MIN_S` と同じ理由で短くした）
    _RETURN_TIMEOUT_MIN_S = 2.0
    #: 減速フェーズの安全装置 [s]（止まりきらなくても次へ進む）
    _DECEL_TIMEOUT_S = 4.0
    #: 停止とみなす速度 [m/s]
    _STOPPED_M_S = 0.02
    #: 空転とみなす後輪と前輪の速度差（`SensorGuard` と同じ形）と、続く時間 [s]
    _SPIN_TOL_M_S = 0.2
    _SPIN_TOL_FRAC = 0.3
    _SPIN_PERSIST_S = 0.1

    #: 試験の手順。**GUIからは変えられない**（`_sysid_common.py`「手順を固定した理由」）
    SETTINGS: dict[str, float] = {
        # 段（`STAGES`）ごとにブレーキ・速度指令0を1回ずつ。定速は hold_s（中立ずれと速度PIの
        # 落ち着き方の材料。解析は指令が0.4s以上変わらない区間の後半を定速とみなす）
        "v_max": 2.0, "run_length_m": 2.0, "cycles_per_stage": 2, "hold_s": 0.8,
        "stop_hold_s": 0.5, "return_speed": 0.6,
    }
    params = ()

    def __init__(self) -> None:
        #: 手順（`SETTINGS`の写し。テスト・ベンチだけが差し替える）
        self.settings = dict(self.SETTINGS)
        self.stages = STAGES
        self._gate = TestGate()
        #: 前輪エンコーダの不調で暴走しないための照合（`SensorGuard`）
        self._guard = SensorGuard()
        self._reset_state()

    def _reset_state(self) -> None:
        self._t = 0.0
        #: 加速フェーズで目標を上げ始めてからの時間 [s]（目標を出した周の数×dt）
        self._ramp_t = 0.0
        self._cycle_i = 0
        self._phase = self._ACCEL
        #: 試験の開始位置の前輪オドメトリ [m]
        self._origin_odom: float | None = None
        self._return_timeout_s = self._RETURN_TIMEOUT_MIN_S
        #: 抑えきれない空転の疑いが続いている時刻 [ns] と、それを検出した段（None＝未検出）
        self._spin_since_ns: int | None = None
        self.spin_stage: int | None = None
        #: 加速中に後輪が滑った（`REAR_SLIP_LIMIT`）段（None＝滑っていない）。その段は最後まで走り、
        #: 上の段へは進まない
        self.slip_stage: int | None = None
        self._slip_since_ns: int | None = None
        self._guard.reset()

    def reset(self) -> None:
        self._gate.reset()
        self._reset_state()

    def on_vehicle_state(self, vs: VehicleState) -> None:
        """`planning_node`が全サンプルで呼ぶ（ダックタイピング）。前輪と後輪の速度を照合し、
        加速中の空転を見る。"""
        self._guard.update(vs)
        if self._phase != self._ACCEL or self.spin_stage is not None or len(vs.wheel_speed) < 4:
            self._spin_since_ns = None
            return
        front = vs.speed
        rear = 0.5 * (vs.wheel_speed[2] + vs.wheel_speed[3])
        if self.slip_stage is None:
            if rear > 0.1 and rear_slip(front, rear) > REAR_SLIP_LIMIT:
                self._slip_since_ns = self._slip_since_ns if self._slip_since_ns is not None else vs.t_capture
                if vs.t_capture - self._slip_since_ns >= self._SPIN_PERSIST_S * 1e9:
                    self.slip_stage = self.stage_of(self._cycle_i)
            else:
                self._slip_since_ns = None
        # 後輪が前進方向に回っているときだけ（加速の段の頭はまだ後退中）
        spinning = rear > 0.1 and rear - front > self._SPIN_TOL_M_S + self._SPIN_TOL_FRAC * abs(front)
        if not spinning:
            self._spin_since_ns = None
        elif self._spin_since_ns is None:
            self._spin_since_ns = vs.t_capture
        elif vs.t_capture - self._spin_since_ns >= self._SPIN_PERSIST_S * 1e9:
            self.spin_stage = self.stage_of(self._cycle_i)

    def set_engaged(self, engaged: bool) -> None:
        """`planning_node.py`がplan()の直前に呼ぶ（ダックタイピング）。"""
        self._gate.set_engaged(engaged)

    @staticmethod
    def _front_odom_m(vs: VehicleState) -> float:
        return (vs.odom_dist[0] + vs.odom_dist[1]) / 2.0

    def n_cycles(self, p: dict[str, float]) -> int:
        return len(self.stages) * int(p["cycles_per_stage"])

    def stage_of(self, cycle_i: int) -> int:
        return cycle_i // int(self.settings["cycles_per_stage"])

    def uses_brake(self, cycle_i: int) -> bool:
        """このサイクルの減速を `brake` で行うか（各段の1回目）。"""
        return cycle_i % 2 == 0

    def _stage_label(self, stage: int) -> str:
        a = self.stages[stage][0]
        return f"{a:.1f}m/s² の段" if a > 0 else "全開の段"

    def _done(self, st: AutoState) -> AutoState:
        st.target_speed = 0.0
        if self.spin_stage is not None:
            st.reason = f"完了（空転を検出: {self._stage_label(self.spin_stage)}。上の段は走らない）"
        elif self.slip_stage is not None and self.slip_stage < len(self.stages) - 1:
            st.reason = f"完了（後輪が滑った: {self._stage_label(self.slip_stage)}。上の段は走らない）"
        else:
            st.reason = "完了"
        return st

    def plan(self, scan: Scan, vs: VehicleState | None,
             p: dict[str, float], dt: float) -> AutoState:
        p = {**self.settings, **p}
        st = AutoState(mode=self.id, planner=self.name)
        st.ready = True
        st.target_steer = 0.0

        engaged, armed, just_started = self._gate.tick(vs)
        if just_started:
            self._reset_state()
        if not engaged:
            st.target_speed = 0.0
            st.reason = "試験開始を押してください"
            return st
        if not armed:
            st.target_speed = 0.0
            st.reason = "ARM待ち（Enterを押してください）"
            return st
        # `armed`が真なら`TestGate.tick()`の実装上vsは必ずある
        assert vs is not None
        if self._guard.fault is not None:
            return abort_state(st, self._guard.fault)
        if self._gate.settling(dt):
            # 開始位置は最初の1回だけ（`OdomRun` と同じ理由でサイクルごとに置き直さない）
            if self._origin_odom is None:
                self._origin_odom = self._front_odom_m(vs)
            return settle_state(st)
        if self._origin_odom is None:
            self._origin_odom = self._front_odom_m(vs)

        n_cycles = self.n_cycles(p)
        if self._cycle_i >= n_cycles:
            return self._done(st)

        self._t += dt
        traveled = self._front_odom_m(vs) - self._origin_odom
        brake = self.uses_brake(self._cycle_i)
        a, v_hold, brake_nm = self.stages[self.stage_of(self._cycle_i)]
        v = vs.speed
        # 止まりきれなくなる地点の見積もりの減速度（ブレーキの強さに合わせる。速度指令0は最大制動と同じ扱い）
        stop_decel = brake_decel_guess(brake_nm if brake and self.spin_stage is None else 0.0)

        if self._phase == self._ACCEL:
            if a > 0:
                # ランプ＋定速。止まりきれなくなる地点が先に来たら切り上げる
                done = self._t >= v_hold / a + p["hold_s"]
                accel_now = a if self._t < v_hold / a else 0.0
            else:
                # 全開: v_max に達したら。1.5m/s²も出ない異常時の保険に時間でも切る
                # （以前は「v_max/1.0+1.0」（3s）で、オドメトリが止まると目標3.0m/s のまま3s走った。
                # 一次の防御は `SensorGuard`）
                done = v >= p["v_max"] or self._t >= p["v_max"] / 1.5 + 0.5
                accel_now = _ACCEL_FOR_STOP_M_S2
            done = (done or self.spin_stage is not None
                    or traveled + stop_distance(v, brake, accel_now, stop_decel) >= p["run_length_m"])
            if self.spin_stage is not None:
                brake = True
        elif self._phase == self._DECEL:
            done = abs(v) <= self._STOPPED_M_S or self._t >= self._DECEL_TIMEOUT_S
        elif self._phase == self._HOLD:
            done = self._t >= p["stop_hold_s"]
            if done:
                # ここまで進んだ距離から、余裕を持った戻りのタイムアウトを逆算する
                self._return_timeout_s = max(
                    self._RETURN_TIMEOUT_MIN_S, abs(traveled) / p["return_speed"] * 1.5 + 1.0)
        else:  # _RETURN
            done = (traveled <= self._RETURN_TOLERANCE_M + OdomRun.RETURN_LEAD_S * p["return_speed"]
                    or self._t >= self._return_timeout_s)

        if done:
            self._t = 0.0
            self._ramp_t = 0.0
            self._phase = (self._phase + 1) % 4
            if self._phase == self._ACCEL:
                if self.spin_stage is not None:
                    self._cycle_i = n_cycles
                    return self._done(st)
                self._cycle_i += 1
                # 後輪が滑った段は最後まで走り、上の段へは進まない
                if self._cycle_i >= n_cycles or (self.slip_stage is not None
                                                 and self.stage_of(self._cycle_i) > self.slip_stage):
                    self._cycle_i = n_cycles
                    return self._done(st)
                brake = self.uses_brake(self._cycle_i)
                a, v_hold, brake_nm = self.stages[self.stage_of(self._cycle_i)]

        if self._phase == self._ACCEL:
            if a > 0:
                # 1周先の目標を出す（ファームが `accel_limit` でその間を埋める）。切り替わった周に
                # 目標0を出すと、戻りの後退から0.1sだけ「目標0」が挟まり、その間に動くファームの
                # 目標のランプを解析が区間の頭で知りようがなく、段ごとに約0.15m/s ずれた当てはめに
                # なった（2026-09-27、ベンチで車体の利得を22→40と読んだ）
                self._ramp_t += dt
                st.target_speed = min(v_hold, a * self._ramp_t)
                st.accel_limit = a
            else:
                st.target_speed = min(p["v_max"] + _ACCEL_OVERSHOOT, _PI_MAX_SPEED)
        elif self._phase == self._DECEL:
            st.target_speed = 0.0
            st.brake = brake or self.spin_stage is not None
            if brake and self.spin_stage is None:
                st.brake_torque = brake_nm
            elif self.spin_stage is not None:
                st.brake_torque = ABORT_BRAKE_TORQUE_NM
        elif self._phase == self._HOLD:
            st.target_speed = 0.0
            st.brake = True
        else:
            st.target_speed = -p["return_speed"]
        label = self._PHASE_LABELS[self._phase]
        if self._phase == self._ACCEL:
            label += f"（{a:.1f}m/s²）" if a > 0 else "（全開）"
        elif self._phase == self._DECEL:
            label += (f"（ブレーキ {st.brake_torque:.2f}N·m）" if st.brake_torque > 0 else "（ブレーキ）") \
                if st.brake else "（速度指令0）"
        st.reason = f"{label} {self._cycle_i + 1}/{n_cycles}"
        return st
