"""システム同定 — 前後運動試験（`config/vehicle.toml` `[dynamics]` の実測手順）。

直進で「加速 → 定速 → 減速 → 元の位置へ後退」を繰り返す。**最初に全開加速・最大制動で、この路面で
出せる最大の加速度・減速度を測り、残りの段をその下に割り振る**（`ACCEL_FRACTIONS`・
`BRAKE_FRACTIONS`）。1つの記録から次を測る（解析は `tools/sysid/fit.py` の `fit_accel()`）：

- `speed_plant_gain`・`rolling_resistance`: ファームの速度制御（目標のランプ→PI→トルク上限、
  `sim/vehicle.py` の `SpeedController`）に記録の `/cmd` を入れた応答を当てはめる。段ごとに違う
  加速度のランプの始まり・終わりの角と、定速で落ち着くまでの応答が材料
- `drive_accel_m_s2`・`drive_fade_speed_m_s`・`drive_top_speed_m_s`: 加速度の上限（モータ・TC）
  とその速度依存。全開の段で見える
- `brake_decel_per_nm`・`brake_decel_m_s2`: `brake` を立てたときの「制動トルク→減速度」の傾きと、
  タイヤのグリップによる頭打ち（全開の回と、割り振った段の1回目。段ごとに制動トルクを変える）
- `speed_decel_m_s2`: 速度指令を0にしたときの減速の上限（割り振った段の2回目）

定速の区間（舵0°・前進）は、舵の中立ずれ（`fit_geometry()`）の材料にもなる。

## 最大を先に測ってから段を割り振る（2026-10-13）

2026-10-12 までは段を固定していた（加速度 0.8→1.6→2.4m/s²→全開、制動 0.04→0.08→0.12→最大N·m）。
限界は路面・タイヤ・TC/ABS の設定で変わるので、固定の段は限界のどちら側に落ちるか分からない。
2026-10-10 の実機の記録では、制動の頭打ち（ABS、2.47m/s²）が約0.06N·m に当たり、4段のうち3段が
頭打ちして**傾きを 0.04N·m の1段だけで決めていた**。加速も 2.4m/s² の段が滑り、車体の利得の
当てはめに使えたのは2段だけだった。段の値を選び直しても、別の路面でまた外れる。

そこで最初の段を全開加速（TC が効く）・最大制動（ABS が効く）にして、車速の傾きからその場で最大の
加速度・減速度を読み、残りの段をその割合に置く:

- 加速度: 最大の `ACCEL_FRACTIONS` 倍（3段とも限界の下＝滑らず、車体の利得の当てはめに使える）
- 制動: 減速度が最大の `BRAKE_FRACTIONS` 倍になる制動トルク。トルク→減速度の傾きは最初は保守側の
  見積もり（`_BRAKE_SLOPE_GUESS`）で、頭打ちしなかった段を走るたびに実測で直す。割り振った段で ABS が
  効いたら（＝頭打ち側だった）、次の段はその強さの `_BRAKE_BACKOFF` 倍より上にしない

読んだ値は段を置くためだけに使う（多少ずれても段の位置が動くだけ。同定値は解析が記録から決める）。

## 速度応答と加減速を1つの試験にしている理由

車体の利得と加速度の上限は互いに補い合うので、解析は両方の記録を合わせて当てはめる。

## 段の加速度の作り方

段の加速度 `a` では、目標速度をプランナー自身が `a` で上げ、同時に `AutoState.accel_limit = a`
（`COMMAND.accel_limit`）も送る。ファームは目標を `accel_limit` でランプさせるので、10Hz の
判断の刻み（目標は0.1sごとに `a`×0.1 ずつ増える）がちょうど滑らかな一定の傾きになる。
目標だけを上げると、刻みごとにファームのランプ（3.0m/s²）で急に追いついて止まる階段になり、
`accel_limit` だけだと目標が最初から一定なので、加速中の区間を定速の区間と区別できない
（中立ずれの材料に加速中のサンプルが混ざる。全開加速中は左右モータのトルク差でもヨーが出る）。
全開の段は目標を `v_max` より上（`v_max+1.0`）に置く（目標に近づいて速度PIが
自分で緩める応答ではなく、上限を測るため）。減速は全段とも GUI の上限（ファームのランプ3.0）で
行う（停止距離の見積もり `stop_distance()` がその前提）。

## 後輪が滑っても止めない

TC は「スリップ率を目標に保つ PI」（`[control]` の `tc_slip_target`、0.2）なので、**TC が正常に
働いているだけで滑り率が `REAR_SLIP_LIMIT`（0.10）を超える**。全開の段は滑りながら走るのが正常で、
滑ったかどうか・どの加速度が限界かは解析（`tools/sysid/fit.py`）が記録から読む。

- **抑えきれない空転**（後輪の周速が前輪より 0.2m/s＋30% 以上速い状態が 0.1s 続く。TC が保つ
  滑りの2倍以上＝TC が切れている・効いていない）は、すぐ制動する（ロックしない強さ
  `ABORT_BRAKE_TORQUE_NM`）。`SensorGuard`（0.3s、前輪エンコーダの不調で中止）より先に効く。
  全開の回で起きたら、残りの全開の回は走らず、空転するまでに出ていた加速度を最大とみなして下の段へ
  進む。割り振った段でも起きたら、戻ってから試験を終える（「完了（空転を検出…）」）

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

import math

from ..msgs.types import AutoState, Scan, VehicleState
from ._sysid_common import (ABORT_BRAKE_TORQUE_NM, OdomRun, SensorGuard, TestGate, abort_state,
                            brake_decel_guess, settle_state, stop_distance)
from .base import Planner

__all__ = ["SysIdAccel", "ACCEL_FRACTIONS", "BRAKE_FRACTIONS", "HOLD_SPEEDS_M_S", "N_STAGES"]

#: io_node `--max-speed`（`install_services.sh`）。これを超える `target_speed` は切り捨てられる
_PI_MAX_SPEED = 3.0
#: 全開の段で `target_speed` を `v_max` からどれだけ上に置くか [m/s]（docstring参照）
_ACCEL_OVERSHOOT = 1.0
#: 停止距離の見積もりに使う全開加速中の加速度 [m/s²]（ファームの目標ランプの上限。測る対象
#: そのものなので `vehicle.toml` に頼らない）
_ACCEL_FOR_STOP_M_S2 = 3.0
#: 割り振る段の加速度（全開の段で測った最大に対する割合）。いちばん上でも限界の7割＝滑らない
#: （実機で限界 2.8m/s² に対し 2.4m/s² の段は滑り、1.6m/s² の段は滑らなかった）
ACCEL_FRACTIONS: tuple[float, ...] = (0.3, 0.5, 0.7)
#: 割り振る段の制動の減速度（最大制動で測った最大に対する割合）。3段とも頭打ちの下に置く
BRAKE_FRACTIONS: tuple[float, ...] = (0.3, 0.55, 0.8)
#: 割り振る段で定速にする速度 [m/s]。「加速→`hold_s`の定速→停止」が2mの直線に収まる値
#: （収まらなければ定速を切り上げる）
HOLD_SPEEDS_M_S: tuple[float, ...] = (1.0, 1.3, 1.5)
#: 段の数（最初の全開・最大制動の段＋割り振る段）
N_STAGES = 1 + len(ACCEL_FRACTIONS)
#: 最大制動の指令 [N·m]（STM32 の `DRIVE_MAX_BRAKE_TORQUE_NM` = 0.13 に丸められる）と、割り振る段の下限
_BRAKE_MAX_NM = 0.15
_BRAKE_MAX_APPLIED_NM = 0.13
_BRAKE_MIN_NM = 0.01
#: 制動トルク→減速度の傾きの最初の見積もり [(m/s²)/(N·m)]（名目 約33 の保守側＝弱めに踏む側には
#: ならない値。1段走れば実測に置き換わる）と、傾きを読むときに引く転がり抵抗の見当 [m/s²]。
#: どちらも段を置くためだけの値（測る対象そのものなので `vehicle.toml` に頼らない）
_BRAKE_SLOPE_GUESS = 25.0
_ROLLING_GUESS_M_S2 = 0.25
#: 割り振った段で ABS が効いたとき、次の段の強さの上限（その強さに対する割合）
_BRAKE_BACKOFF = 0.8
#: 最大が読めなかったときの値 [m/s²]（全開が短すぎた・最大制動が走れなかった）
_ACCEL_MAX_FALLBACK = 2.0
_DECEL_MAX_FALLBACK = 2.4
#: 最大の加速度として扱う範囲 [m/s²]（上はファームの目標ランプ）
_ACCEL_MAX_RANGE = (0.5, 3.0)
#: 読むサンプルの条件: 加速は車速がこれ以上。制動は、指令してから効き始めるまでの時間を捨て、
#: 最大の減速度は MD の境界層（下）より上の車速だけ、傾きは境界層を式に入れて低い車速まで使う
#: （弱い段は 1.0m/s から踏むので、0.6m/s より上だけでは 0.1〜0.2s しか無い）
_FIT_ACCEL_MIN_V = 0.3
_FIT_BRAKE_MIN_V = 0.6
_FIT_SLOPE_MIN_V = 0.35
_FIT_BRAKE_SKIP_S = 0.08
_FIT_MIN_SAMPLES = 8
_FIT_MIN_SPAN_S = 0.12
#: MD の制動の境界層 [m/s]（制動の効き = tanh(車速 ÷ これ)。MD のファームの定数で、`vehicle.toml` の
#: `brake_boundary_speed_m_s` と同じ値）
_MD_BRAKE_BOUNDARY_M_S = 0.6


def _slope(samples: list[tuple[float, float]]) -> float | None:
    """`(時刻 [s], 車速 [m/s])` の並びの傾き [m/s²]（最小二乗）。短すぎれば None。"""
    if len(samples) < _FIT_MIN_SAMPLES or samples[-1][0] - samples[0][0] < _FIT_MIN_SPAN_S:
        return None
    n = len(samples)
    mt = sum(t for t, _ in samples) / n
    mv = sum(v for _, v in samples) / n
    den = sum((t - mt) ** 2 for t, _ in samples)
    return sum((t - mt) * (v - mv) for t, v in samples) / den if den > 1e-12 else None


def _brake_slope(samples: list[tuple[float, float]], nm: float) -> float | None:
    """制動中の `(時刻, 車速)` から、制動トルク→減速度の傾き [(m/s²)/(N·m)] を読む。

    減速度 = 傾き×トルク×tanh(車速÷境界層)＋転がり抵抗 を区間で積分した形（落ちた車速 = 傾き×
    トルク×∫tanh dt ＋ 転がり抵抗×時間）で、両端の車速だけから求める（微分しないのでノイズに強い）。"""
    if len(samples) < _FIT_MIN_SAMPLES or samples[-1][0] - samples[0][0] < _FIT_MIN_SPAN_S or nm <= 0:
        return None
    span = samples[-1][0] - samples[0][0]
    eff = sum(0.5 * (math.tanh(v0 / _MD_BRAKE_BOUNDARY_M_S) + math.tanh(v1 / _MD_BRAKE_BOUNDARY_M_S))
              * (t1 - t0) for (t0, v0), (t1, v1) in zip(samples, samples[1:]))
    drop = samples[0][1] - samples[-1][1] - _ROLLING_GUESS_M_S2 * span
    return drop / (nm * eff) if eff > 1e-6 and drop > 0 else None


class SysIdAccel(Planner):
    id = "sysid_accel"
    name = "システム同定: 前後運動"
    description = ("直線で「加速→定速→減速（ブレーキと速度指令0を交互）→後退で戻る」を9回。最初に全開加速・"
                   "最大制動で最大の加速度・減速度を測り、残りの3段をその3〜8割に割り振る（最後にもう2回、全開・最大制動）。"
                   "抑えきれない空転だけすぐ止める。直線2m＋車長が要る。約45秒")
    category = "sysid"
    stats = ()

    #: フェーズ番号。`_PHASE_LABELS`・`plan()`と対応
    _ACCEL, _DECEL, _HOLD, _RETURN = 0, 1, 2, 3
    _PHASE_LABELS = {_ACCEL: "加速", _DECEL: "減速", _HOLD: "停止", _RETURN: "戻り"}

    #: 戻りフェーズの「元の位置」判定の許容誤差 [m]
    _RETURN_TOLERANCE_M = 0.05
    #: 戻りフェーズの安全装置の下限時間 [s]（`OdomRun.RETURN_TIMEOUT_MIN_S` と同じ理由で短くした）
    _RETURN_TIMEOUT_MIN_S = 2.0
    #: 戻りにかける時間の下限 [s] と、そのために落とす戻りの速さの下限 [m/s]。前進が短い回（弱い加速の段・
    #: 速度指令0で止める全開）を `return_speed` のまま戻ると 1.1〜1.3s で着き、ファームの速度 PI が
    #: まだ落ち着かないうちに次の加速が始まる（後退が指令より約5%速いまま）。解析は「加速の頭で制御器は
    #: 定常」と仮定するので、その回だけ真値でも残差が 0.4→2.6cm/s になり、速度による加速度の低下を
    #: 見落とした（ベンチ。戻りが2.1s以上の回は合う）。短い回は遅く戻って時間を取る
    _RETURN_MIN_S = 2.2
    _RETURN_SPEED_MIN_M_S = 0.2
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
        # 段ごとにブレーキ・速度指令0を1回ずつ。定速は hold_s（中立ずれと速度PIの
        # 落ち着き方の材料。解析は指令が0.4s以上変わらない区間の後半を定速とみなす）
        "v_max": 2.0, "run_length_m": 2.0, "cycles_per_stage": 2, "hold_s": 0.8,
        "stop_hold_s": 0.5, "return_speed": 0.6,
    }
    params = ()

    def __init__(self) -> None:
        #: 手順（`SETTINGS`の写し。テスト・ベンチだけが差し替える）
        self.settings = dict(self.SETTINGS)
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
        #: この回の戻りの速さ [m/s]（`_RETURN_MIN_S` 参照）
        self._return_speed = float(self.settings["return_speed"])
        #: 抑えきれない空転の疑いが続いている時刻 [ns] と、それを検出した段（None＝未検出）
        self._spin_since_ns: int | None = None
        self.spin_stage: int | None = None
        #: 全開の段で空転した（その段の残りを飛ばし、下の段へ進む）
        self._spun_at_full = False
        #: 全開の段で読んだ加速度 [m/s²]（1回ごと）と、そこから決めた最大（割り振る段に入るときに決める）
        self._accel_meas: list[float] = []
        self.accel_max: float | None = None
        #: 最大制動で読んだ減速度 [m/s²]
        self.decel_max: float | None = None
        #: 頭打ちしなかった制動から読んだ傾き [(m/s²)/(N·m)] と、ABS が効いた割り振りの段の強さの最小 [N·m]
        self._brake_slopes: list[float] = []
        self._brake_sat_nm: float | None = None
        #: 割り振った段の制動トルク [N·m]（段 → 値。その段のブレーキのサイクルに入るときに決める）
        self._brake_nm: dict[int, float] = {}
        #: 傾きを読むためのサンプル `(時刻 [s], 車速)`・その区間の頭の時刻 [ns]・ABS が効いたか
        self._fit: list[tuple[float, float]] = []
        self._fit_t0_ns: int | None = None
        self._fit_abs = False
        self._guard.reset()

    def reset(self) -> None:
        self._gate.reset()
        self._reset_state()

    def on_vehicle_state(self, vs: VehicleState) -> None:
        """`planning_node`が全サンプルで呼ぶ（ダックタイピング）。前輪と後輪の速度を照合し、
        加速中の空転を見る。"""
        self._guard.update(vs)
        self._collect(vs)
        if self._phase != self._ACCEL or self.spin_stage is not None or len(vs.wheel_speed) < 4:
            self._spin_since_ns = None
            return
        front = vs.speed
        rear = 0.5 * (vs.wheel_speed[2] + vs.wheel_speed[3])
        # 後輪が前進方向に回っているときだけ（加速の段の頭はまだ後退中）
        spinning = rear > 0.1 and rear - front > self._SPIN_TOL_M_S + self._SPIN_TOL_FRAC * abs(front)
        if not spinning:
            self._spin_since_ns = None
        elif self._spin_since_ns is None:
            self._spin_since_ns = vs.t_capture
        elif vs.t_capture - self._spin_since_ns >= self._SPIN_PERSIST_S * 1e9:
            self.spin_stage = self.stage_of(self._cycle_i)
            self._spun_at_full = self._spun_at_full or self.spin_stage == 0

    def _collect(self, vs: VehicleState) -> None:
        """最大の加速度・減速度と制動の傾きを読むためのサンプルを貯める（全サンプルで呼ばれる）。
        加速は全開の段だけ、制動はブレーキのサイクルだけ。"""
        stage = self.stage_of(self._cycle_i)
        if self._phase == self._ACCEL and stage == 0 and self.spin_stage is None:
            if vs.speed >= _FIT_ACCEL_MIN_V:
                self._fit.append((vs.t_capture * 1e-9, vs.speed))
        elif self._phase == self._DECEL and self.uses_brake(self._cycle_i) and self.spin_stage is None:
            if self._fit_t0_ns is None:
                self._fit_t0_ns = vs.t_capture
            self._fit_abs = self._fit_abs or vs.abs_active
            if (vs.t_capture - self._fit_t0_ns >= _FIT_BRAKE_SKIP_S * 1e9
                    and vs.speed >= (_FIT_BRAKE_MIN_V if stage == 0 else _FIT_SLOPE_MIN_V)):
                self._fit.append((vs.t_capture * 1e-9, vs.speed))

    def _close_fit(self, phase: int, cycle_i: int) -> None:
        """加速・減速のフェーズを終えるときに、貯めたサンプルから傾きを読んで記録する。"""
        fit, abs_cut = self._fit, self._fit_abs
        a = _slope(fit)
        self._fit, self._fit_t0_ns, self._fit_abs = [], None, False
        stage = self.stage_of(cycle_i)
        if phase == self._ACCEL and stage == 0:
            if a is not None and a > 0:
                self._accel_meas.append(a)
        elif phase == self._DECEL and self.uses_brake(cycle_i) and self.spin_stage is None:
            if stage == 0:
                if a is not None and a < 0 and self.decel_max is None:
                    self.decel_max = -a
                return
            nm = self._brake_nm.get(stage)
            if nm is None:
                return
            if abs_cut:
                self._brake_sat_nm = nm if self._brake_sat_nm is None else min(self._brake_sat_nm, nm)
            else:
                g = _brake_slope(fit, nm)
                if g is not None:
                    self._brake_slopes.append(g)

    def stage_accel(self, stage: int) -> float:
        """段の加速度 [m/s²]（0 = 全開）。割り振る段は、全開の段で読んだ最大の `ACCEL_FRACTIONS` 倍。"""
        if stage == 0:
            return 0.0
        if self.accel_max is None:
            a = max(self._accel_meas) if self._accel_meas else _ACCEL_MAX_FALLBACK
            self.accel_max = max(_ACCEL_MAX_RANGE[0], min(_ACCEL_MAX_RANGE[1], a))
        return round(ACCEL_FRACTIONS[stage - 1] * self.accel_max, 2)

    def stage_hold_speed(self, stage: int, p: dict[str, float]) -> float:
        return p["v_max"] if stage == 0 else min(HOLD_SPEEDS_M_S[stage - 1], p["v_max"])

    def stage_brake_nm(self, stage: int) -> float:
        """段のブレーキのサイクルの制動トルク [N·m]。最初の段は最大、割り振る段は減速度が最大の
        `BRAKE_FRACTIONS` 倍になる強さ（その段で初めて呼ばれたときに決めて覚える）。"""
        if stage == 0:
            return _BRAKE_MAX_NM
        if stage not in self._brake_nm:
            d_max = self.decel_max if self.decel_max is not None else _DECEL_MAX_FALLBACK
            slope = (sum(self._brake_slopes) / len(self._brake_slopes) if self._brake_slopes
                     else _BRAKE_SLOPE_GUESS)
            nm = (BRAKE_FRACTIONS[stage - 1] * d_max - _ROLLING_GUESS_M_S2) / slope
            if self._brake_sat_nm is not None:
                nm = min(nm, _BRAKE_BACKOFF * self._brake_sat_nm)
            self._brake_nm[stage] = round(max(_BRAKE_MIN_NM, min(_BRAKE_MAX_APPLIED_NM, nm)), 3)
        return self._brake_nm[stage]

    def set_engaged(self, engaged: bool) -> None:
        """`planning_node.py`がplan()の直前に呼ぶ（ダックタイピング）。"""
        self._gate.set_engaged(engaged)

    @staticmethod
    def _front_odom_m(vs: VehicleState) -> float:
        return (vs.odom_dist[0] + vs.odom_dist[1]) / 2.0

    def cycles(self) -> list[tuple[int, bool]]:
        """サイクルの並び `(段, 減速を brake で行うか)`。最初に全開加速・最大制動（段0）、割り振った段は
        ブレーキ・速度指令0を1回ずつ、**最後に全開加速・最大制動をもう2回**。

        全開を速度指令0で止める回は置かない: 2mの直線では止まりきれる速度（約1m/s）までしか出せず、
        速度による加速度の低下（ベンチの真値は1.3m/sから）が見えない。最高速まで出る全開が1〜2回だと
        その低下を見落とした（ベンチの20条件中2〜4条件。固定の段の頃は、限界に近い 2.4m/s² の段が
        材料になっていた）。最大制動も3回になり、頭打ちの再現性が見える。"""
        n = int(self.settings["cycles_per_stage"])
        return ([(0, True)] + [(k, j == 0) for k in range(1, N_STAGES) for j in range(n)]
                + [(0, True), (0, True)])

    def n_cycles(self, p: dict[str, float]) -> int:
        return len(self.cycles())

    def stage_of(self, cycle_i: int) -> int:
        c = self.cycles()
        return c[max(0, min(cycle_i, len(c) - 1))][0]

    def uses_brake(self, cycle_i: int) -> bool:
        """このサイクルの減速を `brake` で行うか（全開の回と、割り振った段の1回目）。"""
        c = self.cycles()
        return c[min(cycle_i, len(c) - 1)][1]

    def _stage_label(self, stage: int) -> str:
        a = self.stage_accel(stage)
        return f"{a:.2f}m/s² の段" if a > 0 else "全開の段"

    def _done(self, st: AutoState) -> AutoState:
        st.target_speed = 0.0
        if self.spin_stage is not None:
            st.reason = f"完了（空転を検出: {self._stage_label(self.spin_stage)}）"
        elif self._spun_at_full:
            st.reason = "完了（全開の段で空転を検出。下の段は走った）"
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
        stage = self.stage_of(self._cycle_i)
        a, v_hold = self.stage_accel(stage), self.stage_hold_speed(stage, p)
        # 制動トルクは、その段のブレーキのサイクルで止まり方を決める時点（加速中）までに決まる
        brake_nm = self.stage_brake_nm(stage) if brake else 0.0
        v = vs.speed
        # 止まりきれなくなる地点の見積もりの減速度（ブレーキの強さに合わせる。速度指令0は最大制動と同じ扱い）
        stop_decel = brake_decel_guess(brake_nm if brake and self.spin_stage is None else 0.0)
        if self.decel_max is not None:
            # 最大制動で読んだ減速度より強くは止まれない（グリップの低い路面で直線をはみ出さない）
            stop_decel = min(stop_decel, 0.9 * self.decel_max)

        if self._phase == self._ACCEL:
            if a > 0:
                # ランプ＋定速。止まりきれなくなる地点が先に来たら切り上げる
                done = self._t >= v_hold / a + p["hold_s"]
                accel_now = a if self._t < v_hold / a else 0.0
            else:
                # 全開: v_max に達したら。1.5m/s²も出ない異常時の保険に時間でも切る
                # （一次の防御は `SensorGuard`）
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
                self._return_speed = max(self._RETURN_SPEED_MIN_M_S,
                                         min(p["return_speed"], abs(traveled) / self._RETURN_MIN_S))
                self._return_timeout_s = max(
                    self._RETURN_TIMEOUT_MIN_S, abs(traveled) / self._return_speed * 1.5 + 1.0)
        else:  # _RETURN
            done = (traveled <= self._RETURN_TOLERANCE_M + OdomRun.RETURN_LEAD_S * self._return_speed
                    or self._t >= self._return_timeout_s)

        if done:
            self._t = 0.0
            self._ramp_t = 0.0
            if self._phase in (self._ACCEL, self._DECEL):
                self._close_fit(self._phase, self._cycle_i)
            self._phase = (self._phase + 1) % 4
            if self._phase == self._ACCEL:
                if self.spin_stage == 0:
                    # 全開の回の空転: 空転するまでに出ていた加速度を最大とみなして、下の段へ進む
                    self.spin_stage = None
                elif self.spin_stage is not None:
                    self._cycle_i = n_cycles
                    return self._done(st)
                self._cycle_i += 1
                # 全開で空転した車は、残りの全開の回（もう一度空転する）を走らない
                while (self._spun_at_full and self._cycle_i < n_cycles
                       and self.stage_of(self._cycle_i) == 0):
                    self._cycle_i += 1
                if self._cycle_i >= n_cycles:
                    return self._done(st)
                brake = self.uses_brake(self._cycle_i)
                stage = self.stage_of(self._cycle_i)
                a, v_hold = self.stage_accel(stage), self.stage_hold_speed(stage, p)
                brake_nm = self.stage_brake_nm(stage) if brake else 0.0

        if self._phase == self._ACCEL:
            if a > 0:
                # 1周先の目標を出す（ファームが `accel_limit` でその間を埋める）。切り替わった周に
                # 目標0を出すと、戻りの後退から0.1sだけ「目標0」が挟まり、その間に動くファームの
                # 目標のランプを解析が区間の頭で知りようがなく、段ごとに約0.15m/s ずれた当てはめに
                # なった（ベンチで車体の利得を22→40と読んだ）
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
            st.target_speed = -self._return_speed
        label = self._PHASE_LABELS[self._phase]
        if self._phase == self._ACCEL:
            label += f"（{a:.2f}m/s²）" if a > 0 else "（全開）"
        elif self._phase == self._DECEL:
            label += (f"（ブレーキ {st.brake_torque:.2f}N·m）" if st.brake_torque > 0 else "（ブレーキ）") \
                if st.brake else "（速度指令0）"
        st.reason = f"{label} {self._cycle_i + 1}/{n_cycles}"
        return st
