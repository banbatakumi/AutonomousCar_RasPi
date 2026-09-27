"""システム同定 — ステア試験（`config/vehicle.toml` `[dynamics]` の実測手順）。

2つの部分からなる（解析は `tools/sysid/fit.py` の `fit_steer()` と `fit_geometry()`）：

1. **走行中・小振幅**: 直線を `roll_speed`（既定1.0m/s）で走りながら ±5/10/15° を
   「+A → −A → −A → +A」の順にステップさせ、止まって後退で戻る。**学習で実際に使う
   速度と舵角で** `tau_steer_s`・`dead_time_s` を測り、舵がほぼ止まっている区間の曲率から
   小さい舵角での舵の効き（`steer_gain`）も読む。ジグザグは舵の遅れで毎回少し向き・
   横位置を残すので、加速・後退の区間でジャイロと速度から積算した自己位置を見て元の
   直線へ弱く（±2°まで）寄せ直す
2. **低速・全域の階段**（2026-09-27、掃引から置き換え）: 0.3m/sで前後に転がしながら、舵角を
   −30°→+30°→−30° の段（`STAIR_LEVELS_DEG`、中立付近は細かく）で**保持**する。各段は
   「1秒前進→1秒後退」で、舵は後退の途中で次の段へ切り替える（止まった車で舵を切らない。
   前進区間は丸ごと一定の舵角）。同じ舵角に上りと下りの両方から近づくので、サーボの
   ヒステリシス（`steer_actual`が止まる位置の差）とリンクのガタ（曲率にだけ出る、上りと下りの
   差）・不感帯（中立付近だけ曲率が欠ける）、全域での舵の効き（非線形性）が読める。
   以前は三角波の掃引と ±15°・0°付近の小さな往復（速さ・振幅の違う4種類）だったが、バンビの
   「試験はなるべくシンプルな方が正しい値が取れる」を受けて、保持した段だけにした。保持なら
   舵が動いている最中の遅れ（サーボ・ヨーの遅れ）が曲率に混ざらない
3. **低速・大振幅のステップ**: 0.2m/sで前後に転がしながら ±30° をステップ入力する。
   `steer_rate_limit_rad_s`（サーボ出力側の物理上限）は大振幅でないと見えない。
   2・3は±30°の弧で車の向きが変わるので、直線を使う走行部より**後に**やる

`steer_cmd_echo`（STM32がレート制限でランプさせた参照舵角）→`steer_actual`の応答を、
`sim/vehicle.py` の `SteerServo` にそのまま当てはめる。

## なぜ2つに分けたか（2026-09-24）

最初の作り直しでは 0.2m/s で転がしながらの大振幅だけだった。これには2つ問題があった：
- 学習中の車は1m/s前後で5〜15°を切っている。サーボの負荷（タイヤのこじり・
  セルフアライニングトルク）は速度で変わるので、0.2m/sの値がそのまま当てはまる保証は無い
- 前後の向きの切り替えが舵のステップと同じ瞬間に来ていて、半分のステップが実質
  据え切りになっていた（今は向きの切り替えを保持時間の真ん中に置いている）

小振幅なら走りながらでも横に大きく動かない（+A→−A→−A→+Aで向きも横ずれも打ち消す）。
舵の効きを円ではなく**短い弧**で読むので、半径の大きい小舵角も2mの直線で測れる
（旋回グリップ試験は30°の円なので小舵角の効きは測れない）。

## 試験開始/中止を押すまで進まない

`TestGate`（`_sysid_common.py`）参照。開始直後の`SETTLE_S`は静止する。
"""

from __future__ import annotations

import math

from ..core.vehicle import Vehicle
from ..msgs.types import AutoState, Scan, VehicleState
from ._sysid_common import DeadReckon, OdomRun, SensorGuard, TestGate, abort_state, settle_state
from .base import Planner

__all__ = ["SysIdSteer", "ZIGZAG", "LANE_KEEP_MAX_RAD", "STAIR_LEVELS_DEG"]

#: 走行中の小振幅ステップの並び（向きと横ずれが打ち消し合う）
ZIGZAG = (1.0, -1.0, -1.0, 1.0)
#: 加速・後退の区間で元の直線へ戻す舵の上限 [rad]。解析はこれより大きい指令
#: （`tools/sysid/fit.py` の `_ZIG_MIN_RAD`）だけをジグザグのステップとして扱う
LANE_KEEP_MAX_RAD = math.radians(2.0)
#: 階段部の段 [deg]（最大舵角で切る）。−30°→+30°の上りと+30°→−30°の下りで、どの段にも両側から
#: 近づく（上りと下りの曲率の差＝ガタ）。不感帯が効く中立付近（0.5〜3°）は細かく、効きの
#: 非線形性を見る大舵角は粗く
_STAIR_UP = (-30.0, -20.0, -12.0, -6.0, -3.0, -1.5, 0.0, 1.5, 3.0, 6.0, 12.0, 20.0, 30.0)
STAIR_LEVELS_DEG = _STAIR_UP + tuple(reversed(_STAIR_UP[:-1]))


class SysIdSteer(Planner):
    id = "sysid_steer"
    name = "システム同定: ステア"
    description = "1.0m/sで直線を走りながら±5〜15°のジグザグ（6往復）→ 前後に転がしながら舵角を−30°→+30°→−30°の段で保持（各段1秒前進・1秒後退、中立付近は0.6m/s・それ以外は0.3m/s）→ ±30°のステップ。直線2m＋車長・幅1.1mと、開始位置の後ろに0.3mが要る。約1分30秒"
    category = "sysid"
    stats = ()

    _ACCEL, _ZIG, _STOP, _HOLD, _RETURN = range(5)
    _STOPPED_M_S = 0.02

    #: 試験の手順。**GUIからは変えられない**（`_sysid_common.py`「手順を固定した理由」）
    SETTINGS: dict[str, float] = {
        # 走行部: 1.0m/sで ±5/10/15°（最大15°を3段）を +A→−A→−A→+A、各0.3s。これを2巡（6回の走行）。
        # 1.0m/sは学習中の典型的な速度、15°・1.0m/sの横加速度は約1.2m/s²（限界の3割）
        "roll_speed": 1.0, "roll_amp_max_deg": 15.0, "roll_levels": 3, "roll_hold_s": 0.3,
        "roll_rounds": 2,
        # 階段部: 0.3m/sで、各段「stair_period_s の前半で前進・後半で後退」。舵は後退の真ん中で
        # 次の段へ切り替える（`stair_point()`）。段の並びは `STAIR_LEVELS_DEG`（上りと下り）。
        # 中立付近（|舵角|≦stair_fast_deg）の段だけ 0.6m/s: 曲率＝ヨーレート÷速度のノイズは速度に
        # 反比例し、不感帯が効くのはこの範囲だけ。ノイズ3倍のベンチで、0.3m/sのままでは不感帯
        # 0.6°をガタ（0.8→1.6°）に吸われた（掃引の頃も同じ理由で0°付近だけ0.6m/sにしていた）
        "stair_speed": 0.3, "stair_fast_speed": 0.6, "stair_fast_deg": 3.0, "stair_period_s": 2.0,
        # 低速部: 0.2m/sで前後に転がしながら ±30°、各1.0s を3往復。30°はサーボの物理上限を
        # 見るため（上限に頭打ちされる区間が長いほど見えやすい）
        "amplitude_deg": 30.0, "hold_s": 1.0, "cycles": 3, "creep_speed": 0.2,
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
        self._creep_t = 0.0      # 低速部の経過時間
        self._stair_t = 0.0      # 階段部の経過時間
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
    def creep_sign(t: float, hold: float) -> float:
        """低速部の進行方向。舵のステップ（`hold`の整数倍）と重ならないよう、
        保持時間の真ん中（1.5, 3.5, 5.5… × hold）で向きを入れ替える。"""
        return 1.0 if math.floor((t + 0.5 * hold) / (2.0 * hold)) % 2 == 0 else -1.0

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
    def stair_duration(cls, p: dict[str, float], amax: float) -> float:
        return len(cls.stair_levels(amax)) * p["stair_period_s"]

    @classmethod
    def stair_point(cls, p: dict[str, float], amax: float, t: float) -> tuple[float, float]:
        """階段部の `(舵角, 車速の指令)`。各段の周期の前半は前進・後半は後退で、舵は後退の真ん中
        （周期の3/4）で次の段へ切り替える（止まった車で舵を切らない）。最初の段だけは、前の
        部分が停止で終わるので、転がり出すのと同時に切る。"""
        levels = cls.stair_levels(amax)
        period = p["stair_period_s"]
        k, r = divmod(t, period)
        k = int(k)
        if k >= len(levels):
            return 0.0, 0.0
        speed = (p["stair_fast_speed"] if abs(levels[k]) <= math.radians(p["stair_fast_deg"]) + 1e-9
                 else p["stair_speed"])
        v = speed if r < 0.5 * period else -speed
        steer = levels[k + 1] if r >= 0.75 * period and k + 1 < len(levels) else levels[k]
        return steer, v

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

        # ── 2. 低速・全域の階段 ──
        amax = self.vehicle.max_steer
        stair_total = self.stair_duration(p, amax)
        if self._stair_t < stair_total:
            st.target_steer, st.target_speed = self.stair_point(p, amax, self._stair_t)
            st.reason = f"階段 {self._stair_t:.0f}/{stair_total:.0f}s（{math.degrees(st.target_steer):+.1f}°）"
            self._stair_t += dt
            return st

        # ── 3. 低速・大振幅のステップ ──
        hold = p["hold_s"]
        n_creep = int(p["cycles"]) * 2 + 1 if int(p["cycles"]) > 0 else 0
        if self._creep_t < n_creep * hold:
            step_i = int(self._creep_t / hold)
            amp = min(math.radians(p["amplitude_deg"]), self.vehicle.max_steer)
            st.target_steer = 0.0 if step_i == 0 else (amp if step_i % 2 == 1 else -amp)
            st.target_speed = p["creep_speed"] * self.creep_sign(self._creep_t, hold)
            st.reason = f"低速 {step_i}/{n_creep - 1}（{math.degrees(st.target_steer):+.0f}°）"
            self._creep_t += dt
            return st
        st.reason = "完了"
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
