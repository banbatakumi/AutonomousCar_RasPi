"""システム同定 — ステア試験（`config/vehicle.toml` `[dynamics]` の実測手順）。

3つの部分からなる（解析は `tools/sysid/fit.py` の `fit_steer()` と `fit_geometry()`）：

1. **走行中・小振幅**: 直線を `roll_speed`（既定1.0m/s）で走りながら ±7.5/15° を
   「+A → −A → −A → +A」の順にステップさせ、止まって後退で戻る。**学習で実際に使う
   速度と舵角で** `tau_steer_s`・`dead_time_s` を測り、舵がほぼ止まっている区間の曲率から
   小さい舵角での舵の効き（`steer_gain`）も読む。ジグザグは舵の遅れで毎回少し向き・
   横位置を残すので、加速・後退の区間でジャイロと速度から積算した自己位置を見て元の
   直線へ弱く（±2°まで）寄せ直す
2. **低速・全域の階段**: 同じ点から「舵角を保って前進 → オドメトリが元の値に戻るまで後退」を、
   舵角を −30°→+30°→−30° の段（`STAIR_LEVELS_DEG`、中立付近は細かく）で変えながら繰り返す。
   舵は後退の終わり（起点の少し手前、まだ転がっている間）に次の段へ切り替える（止まった車で舵を
   切らない。前進区間は丸ごと一定の舵角）。同じ舵角に上りと下りの両方から近づくので、サーボの
   ヒステリシス（`steer_actual`が止まる位置の差）とリンクのガタ（曲率にだけ出る、上りと下りの
   差）・不感帯（中立付近だけ曲率が欠ける）、全域での舵の効き（非線形性）が読める。
   保持した段だけなので、舵が動いている最中の遅れ（サーボ・ヨーの遅れ）が曲率に混ざらない
3. **低速・大振幅のステップ**: 同じ往復を0.2m/sで行い、前進の途中と後退の途中（同じ位置）で
   ±30° をステップ入力する。`steer_rate_limit_rad_s`（サーボ出力側の物理上限）は大振幅でないと
   見えない。前進中のステップは低速でのヨーの過渡（IMUのローパスとタイヤの緩和長の切り分け）の材料

## 階段・大振幅は「同じ点からの往復」（2026-10-13）

前進した道を**同じ舵角で**後退すれば、車は同じ弧を逆にたどって元の位置・向きに戻る。舵を
切り替えた後の道（起点の手前→折り返し→起点）も次の前進が同じ舵角でたどり直すので、
切り替えの位置がどこでも向き・位置は積み重ならない（残るのは舵が動いている間に進む1〜2cm
ぶんだけで、上りと下り・左右の入れ替えで打ち消す）。前進・後退は**時間ではなくオドメトリの
距離**で区切る。2026-10-12 までは各段「1秒前進・1秒後退」で、実機では向きの反転に約0.45sかかって
指令速度に届かず、前進と後退の距離が合わずに階段部だけで約0.45mずれた（速度を0.3↔0.6m/sに
変える段で前進0.38m・後退0.21mなど）。距離で区切れば、反転にかかる時間が実機とシムで違っても
前進区間の長さ（＝定常の観測の量）が変わらない。

`steer_cmd_echo`（STM32がレート制限でランプさせた参照舵角）→`steer_actual`の応答を、
`sim/vehicle.py` の `SteerServo` にそのまま当てはめる。

## なぜ走行中の小振幅と低速の大振幅に分けるか

低速の大振幅だけでは足りない：
- 学習中の車は1m/s前後で5〜15°を切っている。サーボの負荷（タイヤのこじり・
  セルフアライニングトルク）は速度で変わるので、0.2m/sの値がそのまま当てはまる保証は無い
- 前後の向きの切り替えが舵のステップと同じ瞬間に来ると、実質据え切りになる
  （向きの切り替えは保持時間の真ん中に置く）

小振幅なら走りながらでも横に大きく動かない（+A→−A→−A→+Aで向きも横ずれも打ち消す）。
舵の効きを円ではなく**短い弧**で読むので、半径の大きい小舵角も2mの直線で測れる
（旋回グリップ試験は30°の円なので小舵角の効きは測れない）。

## 試験開始/中止を押すまで進まない

`TestGate`（`_sysid_common.py`）参照。開始直後の`SETTLE_S`は静止する。
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from ..core.vehicle import Vehicle
from ..msgs.types import AutoState, Scan, VehicleState
from ._sysid_common import DeadReckon, OdomRun, SensorGuard, TestGate, abort_state, settle_state
from .base import Planner

__all__ = ["SysIdSteer", "ZIGZAG", "LANE_KEEP_MAX_RAD", "STAIR_LEVELS_DEG", "Leg"]

#: 走行中の小振幅ステップの並び（向きと横ずれが打ち消し合う）
ZIGZAG = (1.0, -1.0, -1.0, 1.0)
#: 加速・後退の区間で元の直線へ戻す舵の上限 [rad]。解析はこれより大きい指令
#: （`tools/sysid/fit.py` の `_ZIG_MIN_RAD`）だけをジグザグのステップとして扱う
LANE_KEEP_MAX_RAD = math.radians(2.0)
#: 階段部の段 [deg]（最大舵角で切る）。−30°→+30°の上りと+30°→−30°の下りで、どの段にも両側から
#: 近づく（上りと下りの曲率の差＝ガタ）。不感帯が効く中立付近（0.5〜3°）は細かく、効きの
#: 非線形性を見る大舵角は粗く。6°より内側は段差を3.5°以下に保つ（サーボの止まる位置の幅を小さな段差
#: から読む、`fit._stair_hysteresis`）。2026-10-13 に ±12°・±20° を ±15° にまとめた（25→21段。ベンチの
#: 8条件で復元の誤差は変わらず。±1.5° も抜いた17段は、サーボの止まる位置の幅が約0.07°ずれた）
_STAIR_UP = (-30.0, -15.0, -6.0, -3.0, -1.5, 0.0, 1.5, 3.0, 6.0, 15.0, 30.0)
STAIR_LEVELS_DEG = _STAIR_UP + tuple(reversed(_STAIR_UP[:-1]))


@dataclass(frozen=True)
class Leg:
    """階段部・大振幅部の往復1回: 起点から `length_m` 前進し、起点まで後退する。

    `mid` が None なら舵は `steer` のまま（階段の段）。`mid` があれば、前進の半分で `steer`→`mid` に
    ステップし、後退で同じ位置に来たら `steer` に戻す（大振幅のステップ。前進と後退で同じ位置に
    置くので、同じ道を戻る）。ステップの位置は判断の刻み（10Hz）ぶん前進と後退でずれ、その分だけ
    向きが残るが、往復ごとに左右を入れ替えて打ち消す（`shuttle_legs`）。"""

    part: str               # 表示名（"階段"・"低速"。記録の `reason` の頭）
    steer: float            # [rad]
    mid: float | None       # [rad]
    speed: float            # [m/s]
    length_m: float


class SysIdSteer(Planner):
    id = "sysid_steer"
    name = "システム同定: ステア"
    description = "1.0m/sで直線を走りながら±7.5°・±15°のジグザグ（2往復）→ 同じ点から「舵角を保って前進→元の位置まで後退」を−30°→+30°→−30°の段で繰り返す（中立付近は0.6m/s・それ以外は0.3m/s）→ 同じ往復の途中で±30°のステップ（±30°は最大舵角で頭打ち）。直線2m＋車長・幅0.8mと、開始位置の後ろに0.3mが要る。約1分"
    category = "sysid"
    stats = ()

    _ACCEL, _ZIG, _STOP, _HOLD, _RETURN = range(5)
    _STOPPED_M_S = 0.02

    #: 試験の手順。**GUIからは変えられない**（`_sysid_common.py`「手順を固定した理由」）
    SETTINGS: dict[str, float] = {
        # 走行部: 1.0m/sで ±7.5/15°（最大15°を2段）を +A→−A→−A→+A、各0.3s。1巡（2回の走行）。
        # 1.0m/sは学習中の典型的な速度、15°・1.0m/sの横加速度は約1.2m/s²（限界の3割）。
        # 2026-10-12 までは ±5/10/15° を2巡（6回）。実機の記録から4回分を抜いても、舵の効きは
        # 10°で0.13°・ヨーの遅れの合計は 41→39ms しか変わらず、ベンチでも復元の誤差は同じだった
        "roll_speed": 1.0, "roll_amp_max_deg": 15.0, "roll_levels": 2, "roll_hold_s": 0.3,
        "roll_rounds": 1,
        # 階段部: 0.3m/sで、各段「stair_leg_m 前進 → 起点まで後退」。舵は後退の終わりに次の段へ
        # 切り替える（`_shuttle()`）。段の並びは `STAIR_LEVELS_DEG`（上りと下り）。
        # 中立付近（|舵角|≦stair_fast_deg）の段だけ 0.6m/s: 曲率＝ヨーレート÷速度のノイズは速度に
        # 反比例し、不感帯が効くのはこの範囲だけ。ノイズ3倍のベンチで、0.3m/sのままでは不感帯
        # 0.6°をガタ（0.8→1.6°）に吸われた（掃引の頃も同じ理由で0°付近だけ0.6m/sにしていた）。
        # 距離は、時間で区切っていた頃に実機で1段に進んでいた長さ（0.3m/sで約0.16m。反転に約0.45s
        # かかり、定常で走るのは残りの約0.35s）。中立付近は曲がらないので長くしても横に広がらない。
        # 1段の時間の半分は反転なので、距離を縮めても試験はほとんど短くならない
        "stair_speed": 0.3, "stair_fast_speed": 0.6, "stair_fast_deg": 3.0,
        "stair_leg_m": 0.16, "stair_fast_leg_m": 0.30,
        # 舵を次の段へ切り替えるのは、起点まであと「速度×switch_lead_s」の所（後退中）
        "switch_lead_s": 0.25,
        # 低速部: 0.2m/sで step_leg_m の往復を cycles 回。前進・後退それぞれの半分の位置で ±30° を
        # ステップ（往復ごとに左右を入れ替える）。30°はサーボの物理上限を見るため（上限に頭打ち
        # される区間が長いほど見えやすい）。ステップの後は半分の距離＝0.6s 保つ（解析の窓は0.5s）
        "amplitude_deg": 30.0, "cycles": 2, "creep_speed": 0.2, "step_leg_m": 0.24,
        # 直線の長さ（車体の長さ・余裕は別）と後退で戻る速さ
        "run_length_m": 2.0, "return_speed": 0.6,
    }
    params = ()

    def __init__(self) -> None:
        #: 手順（`SETTINGS`の写し。テスト・ベンチだけが差し替える）
        self.settings = dict(self.SETTINGS)
        self.vehicle = Vehicle.load()
        self._gate = TestGate()
        #: 走行部で置いた直線へ戻るための自己位置（全サンプルで積算。`on_vehicle_state`）
        self._dr = DeadReckon()
        #: 前輪エンコーダの不調で暴走しないための照合（`SensorGuard`）
        self._guard = SensorGuard()
        self._reset_state()

    def on_vehicle_state(self, vs: VehicleState) -> None:
        """`planning_node`が50Hzの全サンプルで呼ぶ（ダックタイピング）。"""
        self._dr.update(vs)
        self._guard.update(vs)

    def _reset_state(self) -> None:
        self._legs: list[Leg] | None = None   # 階段部・大振幅部の往復の並び（始めるときに作る）
        self._leg_i = 0
        self._leg_back = False   # 往復の後退中
        self._leg_t = 0.0        # 前進・後退それぞれの経過時間
        self._leg_origin = 0.0   # 往復の起点の前輪オドメトリ [m]
        self._pass = 0           # 走行部の何回目か
        self._phase = self._ACCEL
        self._t = 0.0
        self._run = OdomRun()
        self._rolling_started = False
        self._guard.reset()

    def reset(self) -> None:
        self._gate.reset()
        self._reset_state()

    def set_engaged(self, engaged: bool) -> None:
        """`planning_node.py`がplan()の直前に呼ぶ（ダックタイピング）。"""
        self._gate.set_engaged(engaged)

    @staticmethod
    def stair_levels(amax: float) -> list[float]:
        """階段部の段 [rad]（最大舵角で切る。切って隣と同じになった段は1つにまとめる）。"""
        out: list[float] = []
        for deg in STAIR_LEVELS_DEG:
            x = max(-amax, min(amax, math.radians(deg)))
            if not out or abs(x - out[-1]) > 1e-9:
                out.append(x)
        return out

    @classmethod
    def shuttle_legs(cls, p: dict[str, float], amax: float) -> list[Leg]:
        """階段部と大振幅部の往復の並び。大振幅部は往復ごとに左右を入れ替える（`Leg` 参照）。"""
        fast = math.radians(p["stair_fast_deg"]) + 1e-9
        legs = [Leg("階段", x, None,
                    p["stair_fast_speed"] if abs(x) <= fast else p["stair_speed"],
                    p["stair_fast_leg_m"] if abs(x) <= fast else p["stair_leg_m"])
                for x in cls.stair_levels(amax)]
        amp = min(math.radians(p["amplitude_deg"]), amax)
        # 階段は −amax で終わるので、大振幅部の1往復目は −amp から始める（境目に余分なステップを作らない）
        legs += [Leg("低速", s * amp, -s * amp, p["creep_speed"], p["step_leg_m"])
                 for s in ([-1.0, 1.0] * int(p["cycles"]))[:int(p["cycles"])]]
        return legs

    def roll_amplitude(self, p: dict[str, float], pass_i: int) -> float:
        levels = int(p["roll_levels"])
        k = pass_i % levels + 1
        return min(math.radians(p["roll_amp_max_deg"]) * k / levels, self.vehicle.max_steer)

    def plan(self, scan: Scan, vs: VehicleState | None,
             p: dict[str, float], dt: float) -> AutoState:
        p = {**self.settings, **p}
        st = AutoState(mode=self.id, planner=self.name)
        st.ready = True
        st.target_speed = 0.0
        st.target_steer = 0.0

        engaged, armed, just_started = self._gate.tick(vs)
        if just_started:
            self._reset_state()
        if not engaged:
            st.reason = "試験開始を押してください"
            return st
        if not armed:
            st.reason = "ARM待ち（Enterを押してください）"
            return st
        assert vs is not None                  # armed なら vs はある（TestGate.tick）
        if self._guard.fault is not None:
            return abort_state(st, self._guard.fault, vs.steer_cmd_echo)
        if self._gate.settling(dt):
            self._dr.stationary = True
            return settle_state(st)

        # ── 1. 走行中・小振幅（先にやる: 低速部の±30°の弧で車の向きが変わる前に、
        #    置いたとおりの向きで直線を使い切る） ──
        n_pass = int(p["roll_levels"]) * int(p["roll_rounds"]) if p["roll_speed"] > 0 else 0
        if self._pass < n_pass:
            return self._rolling(st, vs, p, dt, n_pass)

        # ── 2. 低速・全域の階段、3. 低速・大振幅のステップ ──
        return self._shuttle(st, vs, p, dt)

    #: 前進・後退が距離で終わらないときに次へ進む時間 [s]（「距離÷速度」の倍率と余裕。一次の
    #: 防御は `SensorGuard`）
    _LEG_TIMEOUT_FACTOR = 2.0
    _LEG_TIMEOUT_MARGIN_S = 1.5

    def _shuttle(self, st: AutoState, vs: VehicleState, p: dict[str, float], dt: float) -> AutoState:
        """階段部・大振幅部（`Leg` の並び）。起点からの前輪オドメトリの距離で前進・後退を区切る。"""
        if self._legs is None:
            self._legs = self.shuttle_legs(p, self.vehicle.max_steer)
            self._leg_origin = OdomRun.odom(vs)
        legs = self._legs
        if self._leg_i >= len(legs):
            st.reason = "完了"
            return st
        leg = legs[self._leg_i]
        # 次の判断（dt 後）までの真ん中の位置で比べる（刻みぶんの行き過ぎを前後に均す）
        x = OdomRun.odom(vs) - self._leg_origin + 0.5 * dt * vs.speed
        self._leg_t += dt
        timeout = self._LEG_TIMEOUT_FACTOR * leg.length_m / leg.speed + self._LEG_TIMEOUT_MARGIN_S
        if not self._leg_back:
            if x >= leg.length_m or self._leg_t >= timeout:
                self._leg_back, self._leg_t = True, 0.0
        elif x <= 0.0 or self._leg_t >= timeout:
            self._leg_back, self._leg_t = False, 0.0
            self._leg_i += 1
            if self._leg_i >= len(legs):
                st.reason = "完了"
                return st
            leg = legs[self._leg_i]

        half = 0.5 * leg.length_m
        if not self._leg_back:
            st.target_speed = leg.speed
            st.target_steer = leg.mid if leg.mid is not None and x >= half else leg.steer
        else:
            st.target_speed = -leg.speed
            if x <= leg.speed * p["switch_lead_s"]:
                # 起点の手前で次の段へ（まだ転がっている）。最後の往復は中立へ戻して止まる
                st.target_steer = legs[self._leg_i + 1].steer if self._leg_i + 1 < len(legs) else 0.0
            elif leg.mid is not None and x > half:
                st.target_steer = leg.mid
            else:
                st.target_steer = leg.steer
        n_part = sum(g.part == leg.part for g in legs)
        k_part = sum(g.part == leg.part for g in legs[:self._leg_i]) + 1
        st.reason = f"{leg.part} {k_part}/{n_part}（{math.degrees(st.target_steer):+.1f}°）"
        return st

    def _rolling(self, st: AutoState, vs: VehicleState, p: dict[str, float], dt: float,
                 n_pass: int) -> AutoState:
        if not self._rolling_started:
            self._rolling_started = True
            self._run.start(vs)
            self._dr.reset_origin()            # 置いた位置・向き＝走る直線
        self._t += dt
        v = vs.speed
        ph = self._phase
        zig_len = len(ZIGZAG) * p["roll_hold_s"]
        if ph in (self._ACCEL, self._ZIG) and self._run.must_stop(vs, p["run_length_m"], v):
            ph = self._STOP
        elif ph == self._ACCEL:
            if v >= 0.9 * p["roll_speed"] or self._t >= 2.0:
                ph = self._ZIG
        elif ph == self._ZIG:
            if self._t >= zig_len:
                ph = self._STOP
        elif ph == self._STOP:
            if abs(v) <= self._STOPPED_M_S or self._t >= 4.0:
                ph = self._HOLD
        elif ph == self._HOLD:
            if self._t >= 0.3:
                self._run.arm_return(vs, p["return_speed"])
                ph = self._RETURN
        elif self._run.returned(vs, self._t):
            self._pass += 1
            self._run.start(vs)
            ph = self._ACCEL
        if ph != self._phase:
            self._phase = ph
            self._t = 0.0
        # 止まってから後退を始めるまでの間はジャイロのバイアスを読み直す
        self._dr.stationary = ph == self._HOLD
        if self._pass >= n_pass:
            st.reason = "走行部 完了"
            return st

        amp = self.roll_amplitude(p, self._pass)
        keep = LANE_KEEP_MAX_RAD
        L = self.vehicle.wheelbase
        if ph == self._ACCEL:
            st.target_speed = p["roll_speed"]
            # ジグザグは舵の遅れで毎回少し向き・横位置を残す（減速中に舵が残る分など）。
            # 加速・後退の区間で元の直線へ弱く寄せ直す（これが無いと6回で横に約1mずれた）
            st.target_steer = self._dr.lane_keep_steer(True, keep, L)
            label = "加速"
        elif ph == self._ZIG:
            k = min(int(self._t / p["roll_hold_s"]), len(ZIGZAG) - 1)
            st.target_speed = p["roll_speed"]
            # 走行ごとに符号を反転する（どの振幅も左右1回ずつになり、舵の効きの左右差を
            # 中立ずれと分けて読める）
            sign = 1.0 if self._pass % 2 == 0 else -1.0
            st.target_steer = sign * ZIGZAG[k] * amp
            label = f"{math.degrees(st.target_steer):+.0f}°"
        elif ph in (self._STOP, self._HOLD):
            st.brake = True
            label = "停止"
        else:
            st.target_speed = -p["return_speed"]
            st.target_steer = self._dr.lane_keep_steer(False, keep, L)
            label = "戻り"
        st.reason = f"走行 {self._pass + 1}/{n_pass}（±{math.degrees(amp):.0f}°）{label}"
        return st
